// Copyright (c) 2016-2021 Bluespec, Inc. All Rights Reserved

package CPU_Stage1;

// ================================================================
// This is Stage 1 of the "Flute" CPU.
// It contains the EX functionality.
// EX: "Execute"

// ================================================================
// STEP 11 (50 MHz campaign): elastic stage.  The decode/regfile/ALU
// comb cone (previously fanning out of rg_stage_input straight into
// rl_pipe and measuring ~29 ns to the far consumers) is decoupled by
// a 2-deep input FIFO and a 2-deep output FIFO with a capture rule
// between them.  rl_pipe's interface contract is unchanged (out/deq/
// enq/set_full); the refill side additionally gates on mv_can_enq
// (input FIFO capacity) so rl_pipe's one-cycle deq->enq chaining is
// absorbed by the input FIFO's spare slot and steady-state throughput
// stays 1 instr/cycle.  Drains that the old protocol performed
// implicitly via set_full(False) are now explicit: deq() pops the
// output FIFO at the rl_pipe move, clear() flushes both FIFOs at the
// trap/CSR/FENCE flush sites.  The live epoch check stays in the out
// method so a captured entry is still discardable on a late redirect
// (stock semantics re-checked every cycle until deq).

// ================================================================
// Exports

export
CPU_Stage1_IFC (..),
mkCPU_Stage1;

// ================================================================
// BSV library imports

import FIFOF        :: *;
import GetPut       :: *;
import ClientServer :: *;
import ConfigReg    :: *;

// ----------------
// BSV additional libs

import Cur_Cycle :: *;

// ================================================================
// Project imports

import ISA_Decls        :: *;
import CPU_Globals      :: *;
import Near_Mem_IFC     :: *;
import GPR_RegFile      :: *;
`ifdef ISA_F
import FPR_RegFile      :: *;
`endif
import CSR_RegFile      :: *;
import EX_ALU_functions :: *;

`ifdef ISA_C
// 'C' extension (16b compressed instructions)
import CPU_Decode_C     :: *;
`endif

// ================================================================
// Interface

interface CPU_Stage1_IFC;
   // ---- Reset
   interface Server #(Token, Token) server_reset;

   // ---- Output
   (* always_ready *)
   method Output_Stage1 out;

   (* always_ready *)
   method Action deq;

   // ---- Input
   (* always_ready *)
   method Action enq (Data_StageD_to_Stage1  data);

   (* always_ready *)
   method Action set_full (Bool full);

   // STEP 11: input-side capacity for rl_pipe's stageD->stage1 refill
   // gate (decouples the refill from this stage's output occupancy).
   (* always_ready *)
   method Bool mv_can_enq;

   // STEP 11: flush both FIFOs (trap/CSR/FENCE drain sites that used
   // to rely on set_full(False)).
   (* always_ready *)
   method Action ma_clear;
endinterface

// ================================================================
// Implementation module

module mkCPU_Stage1 #(Bit #(4)         verbosity,
		      GPR_RegFile_IFC  gpr_regfile,
		      Bypass           bypass_from_stage2,
		      Bypass           bypass_from_stage3,
`ifdef ISA_F
		      FPR_RegFile_IFC  fpr_regfile,
		      FBypass          fbypass_from_stage2,
		      FBypass          fbypass_from_stage3,
`endif
		      CSR_RegFile_IFC  csr_regfile,
		      Epoch            cur_epoch,
		      Priv_Mode        cur_priv)
                    (CPU_Stage1_IFC);

   FIFOF #(Token) f_reset_reqs <- mkFIFOF;
   FIFOF #(Token) f_reset_rsps <- mkFIFOF;

   // STEP 11 elastic staging.  Input side: library 2-deep FIFOF (its
   // first/deq are only touched by the capture rule, which may carry
   // implicit conditions).  Output side: hand-rolled 2-deep register
   // FIFO whose 'first' is a pure register mux (NO implicit condition)
   // because the always_ready out method -- and CPU.bsv's
   // no_implicit_conditions rl_show_pipe -- read it.
   FIFOF #(Data_StageD_to_Stage1) f_in  <- mkSizedFIFOF (2);

   Reg #(Bool) out_v0 <- mkReg (False);
   Reg #(Bool) out_v1 <- mkReg (False);
   Reg #(Tuple2 #(Epoch, Output_Stage1)) out_r0 <- mkRegU;
   Reg #(Tuple2 #(Epoch, Output_Stage1)) out_r1 <- mkRegU;

   PulseWire pw_out_deq <- mkPulseWire;

   // STEP 11 clear handshake (toggle register, NOT a PulseWire: a
   // PulseWire reader lands in CanFire and creates a scheduling edge
   // that closes a cycle with the debug rules' GPR writes).  ma_clear
   // toggles; rl_stage1_capture notices the mismatch and clears.  The
   // out method consults the same shadow, so an entry pushed in the
   // cycle ma_clear fires is invisible from the next cycle onward
   // (rl_pipe is halted in every flush state, so nothing consumes it).
   Reg #(Bool) rg_clear_tog  <- mkReg (False);
   Reg #(Bool) rg_clear_seen <- mkReg (False);

   MISA misa   = csr_regfile.read_misa;
   Bit #(2) xl = ((xlen == 32) ? misa_mxl_32 : misa_mxl_64);

   // ----------------------------------------------------------------
   // BEHAVIOR

   rule rl_reset;
      f_reset_reqs.deq;
      f_reset_rsps.enq (?);
   endrule

   // ----------------
   // ALU (combinational; identical to stock, parameterized on the
   // instruction being examined)

   function Output_Stage1 fv_out (Data_StageD_to_Stage1 rg_stage_input);
      let decoded_instr  = rg_stage_input.decoded_instr;
      let funct3         = decoded_instr.funct3;

      // Register rs1 read and bypass
      let rs1 = decoded_instr.rs1;
      let rs1_val = gpr_regfile.read_rs1 (rs1);
      match { .busy1a, .rs1a } = fn_gpr_bypass (bypass_from_stage3, rs1, rs1_val);
      match { .busy1b, .rs1b } = fn_gpr_bypass (bypass_from_stage2, rs1, rs1a);
      Bool rs1_busy = (busy1a || busy1b);
      Word rs1_val_bypassed = ((rs1 == 0) ? 0 : rs1b);

      // Register rs2 read and bypass
      let rs2 = decoded_instr.rs2;
      let rs2_val = gpr_regfile.read_rs2 (rs2);
      match { .busy2a, .rs2a } = fn_gpr_bypass (bypass_from_stage3, rs2, rs2_val);
      match { .busy2b, .rs2b } = fn_gpr_bypass (bypass_from_stage2, rs2, rs2a);
      Bool rs2_busy = (busy2a || busy2b);
      Word rs2_val_bypassed = ((rs2 == 0) ? 0 : rs2b);

`ifdef ISA_F
      // FP Register rs1 read and bypass
      let frs1_val = fpr_regfile.read_rs1 (rs1);
      match { .fbusy1a, .frs1a } = fn_fpr_bypass (fbypass_from_stage3, rs1, frs1_val);
      match { .fbusy1b, .frs1b } = fn_fpr_bypass (fbypass_from_stage2, rs1, frs1a);
      Bool frs1_busy = (fbusy1a || fbusy1b);
      WordFL frs1_val_bypassed = frs1b;

      // FP Register rs2 read and bypass
      let frs2_val = fpr_regfile.read_rs2 (rs2);
      match { .fbusy2a, .frs2a } = fn_fpr_bypass (fbypass_from_stage3, rs2, frs2_val);
      match { .fbusy2b, .frs2b } = fn_fpr_bypass (fbypass_from_stage2, rs2, frs2a);
      Bool frs2_busy = (fbusy2a || fbusy2b);
      WordFL frs2_val_bypassed = frs2b;

      // FP Register rs3 read and bypass
      let rs3 = decoded_instr.rs3;
      let frs3_val = fpr_regfile.read_rs3 (rs3);
      match { .fbusy3a, .frs3a } = fn_fpr_bypass (fbypass_from_stage3, rs3, frs3_val);
      match { .fbusy3b, .frs3b } = fn_fpr_bypass (fbypass_from_stage2, rs3, frs3a);
      Bool frs3_busy = (fbusy3a || fbusy3b);
      WordFL frs3_val_bypassed = frs3b;
`endif

      // ALU function
      let alu_inputs = ALU_Inputs {cur_priv       : cur_priv,
				   pc             : rg_stage_input.pc,
				   is_i32_not_i16 : rg_stage_input.is_i32_not_i16,
				   instr          : rg_stage_input.instr,
`ifdef ISA_C
				   instr_C        : rg_stage_input.instr_C,
`endif
				   decoded_instr  : rg_stage_input.decoded_instr,
				   rs1_val        : rs1_val_bypassed,
				   rs2_val        : rs2_val_bypassed,
`ifdef ISA_F
				   frs1_val       : frs1_val_bypassed,
				   frs2_val       : frs2_val_bypassed,
				   frs3_val       : frs3_val_bypassed,
				   frm            : csr_regfile.read_frm,
`ifdef INCLUDE_TANDEM_VERIF
	                           fflags         : csr_regfile.read_fflags,
`endif
`endif
				   mstatus        : csr_regfile.read_mstatus,
				   misa           : csr_regfile.read_misa };

      let alu_outputs = fv_ALU (alu_inputs);

      let data_to_stage2 = Data_Stage1_to_Stage2 {pc            : rg_stage_input.pc,
						  instr         : rg_stage_input.instr,
						  op_stage2     : alu_outputs.op_stage2,
						  rd            : alu_outputs.rd,
						  addr          : alu_outputs.addr,
						  val1          : alu_outputs.val1,
						  val2          : alu_outputs.val2,
`ifdef ISA_F
						  fval1         : alu_outputs.fval1,
						  fval2         : alu_outputs.fval2,
						  fval3         : alu_outputs.fval3,
						  rd_in_fpr     : alu_outputs.rd_in_fpr,
						  rs_frm_fpr    : alu_outputs.rs_frm_fpr,
						  val1_frm_gpr  : alu_outputs.val1_frm_gpr,
						  rounding_mode : alu_outputs.rm,
`endif
`ifdef INCLUDE_TANDEM_VERIF
						  trace_data    : alu_outputs.trace_data,
`endif
						  priv          : cur_priv };

      // ----------------
      // Combinational output function

      Output_Stage1 output_stage1 = ?;

      // This stage is empty
      if (! f_in.notEmpty) begin
	 output_stage1.ostatus = OSTATUS_EMPTY;
      end

      // Wrong branch-prediction epoch: discard instruction (convert into a NOOP)
      else if (rg_stage_input.epoch != cur_epoch) begin
	 output_stage1.ostatus = OSTATUS_PIPE;
	 output_stage1.control = CONTROL_DISCARD;

	 // For debugging only
	 let data_to_stage2_discard = Data_Stage1_to_Stage2 {pc:        rg_stage_input.pc,
							     instr:     rg_stage_input.instr,
							     op_stage2: OP_Stage2_ALU,
							     rd:        0,
							     addr:      ?,
							     val1:      ?,
							     val2:      ?,
`ifdef ISA_F
							     fval1           : ?,
							     fval2           : ?,
							     fval3           : ?,
							     rd_in_fpr       : ?,
						             rs_frm_fpr      : ?,
						             val1_frm_gpr    : ?,
							     rounding_mode   : ?,
`endif
`ifdef INCLUDE_TANDEM_VERIF
							     trace_data: alu_outputs.trace_data,
`endif
							     priv:      cur_priv
							     };

	 output_stage1.data_to_stage2 = data_to_stage2_discard;
      end

      // Stall if bypass pending for GPR rs1 or rs2
      else if (rs1_busy || rs2_busy) begin
	 output_stage1.ostatus = OSTATUS_BUSY;
      end

`ifdef ISA_F
      // Stall if bypass pending for FPR rs1, rs2 or rs3
      else if (frs1_busy || frs2_busy || frs3_busy) begin
	 output_stage1.ostatus = OSTATUS_BUSY;
      end
`endif

      // Trap on fetch-exception
      else if (rg_stage_input.exc) begin
	 output_stage1.ostatus   = OSTATUS_NONPIPE;
	 output_stage1.control   = CONTROL_TRAP;
	 output_stage1.trap_info = Trap_Info {epc:      rg_stage_input.pc,
					      exc_code: rg_stage_input.exc_code,
					      tval:     rg_stage_input.tval};
	 output_stage1.data_to_stage2 = data_to_stage2;
      end

      // ALU outputs: pipe (straight/branch)
      // and non-pipe (CSRR_W, CSRR_S_or_C, FENCE.I, FENCE, SFENCE_VMA, xRET, WFI, TRAP)
      else begin
	 let ostatus = (  (   (alu_outputs.control == CONTROL_STRAIGHT)
			   || (alu_outputs.control == CONTROL_BRANCH))
			? OSTATUS_PIPE
			: OSTATUS_NONPIPE);

	 // Compute MTVAL in case of traps
	 let tval = 0;
	 if (alu_outputs.exc_code == exc_code_ILLEGAL_INSTRUCTION) begin
	    // The instruction
`ifdef ISA_C
	    tval = (rg_stage_input.is_i32_not_i16
		    ? zeroExtend (rg_stage_input.instr)
		    : zeroExtend (rg_stage_input.instr_C));
`else
	    tval = zeroExtend (rg_stage_input.instr);
`endif
	 end
	 else if (alu_outputs.exc_code == exc_code_INSTR_ADDR_MISALIGNED)
	    tval = alu_outputs.addr;                           // The branch target pc
	 else if (alu_outputs.exc_code == exc_code_BREAKPOINT)
	    tval = rg_stage_input.pc;                          // The faulting virtual address

	 let trap_info = Trap_Info {epc:      rg_stage_input.pc,
				    exc_code: alu_outputs.exc_code,
				    tval:     tval};

	 let fall_through_pc = rg_stage_input.pc + (rg_stage_input.is_i32_not_i16 ? 4 : 2);
	 let next_pc = ((alu_outputs.control == CONTROL_BRANCH)
			? alu_outputs.addr
			: fall_through_pc);
	 let redirect = (next_pc != rg_stage_input.pred_pc);

	 output_stage1.ostatus        = ostatus;
	 output_stage1.control        = alu_outputs.control;
	 output_stage1.trap_info      = trap_info;
	 output_stage1.redirect       = redirect;
	 output_stage1.next_pc        = next_pc;
	 output_stage1.cf_info        = alu_outputs.cf_info;
	 output_stage1.data_to_stage2 = data_to_stage2;
      end

      return output_stage1;
   endfunction: fv_out

   // STEP 11 capture rule: transform input head -> output FIFO.
   // BUSY entries (bypass pending) hold in f_in and retry next cycle.
   // Sole writer of the staging registers; rl_pipe's deq arrives as a
   // PulseWire folded into the next-state computation, and the clear
   // toggle takes priority over everything.
   rule rl_stage1_capture;
      if (rg_clear_tog != rg_clear_seen) begin
	 rg_clear_seen <= rg_clear_tog;
	 f_in.clear;
	 out_v0 <= False;
	 out_v1 <= False;
      end
      else begin
	 let inp = f_in.first;
	 let c   = fv_out (inp);
	 Bool capturable = (c.ostatus != OSTATUS_BUSY);
	 Bool room = pw_out_deq || (! out_v0) || (! out_v1);
	 // post-pop state (deq folds in here)
	 Bool n_v0 = pw_out_deq ? out_v1 : out_v0;
	 Bool n_v1 = pw_out_deq ? False   : out_v1;
	 Tuple2 #(Epoch, Output_Stage1) n_r0 = pw_out_deq ? out_r1 : out_r0;
	 Tuple2 #(Epoch, Output_Stage1) n_r1 = out_r1;
	 if (capturable && room) begin
	    f_in.deq;
	    if (n_v0) begin
	       n_v1 = True;
	       n_r1 = tuple2 (inp.epoch, c);
	    end
	    else begin
	       n_v0 = True;
	       n_r0 = tuple2 (inp.epoch, c);
	    end
	 end
	 out_v0 <= n_v0;
	 out_v1 <= n_v1;
	 out_r0 <= n_r0;
	 out_r1 <= n_r1;
      end
   endrule

   // ================================================================
   // INTERFACE

   // ---- Reset
   interface server_reset = toGPServer (f_reset_reqs, f_reset_rsps);

   // ---- Output
   method Output_Stage1 out;
      // Clear shadow: while a clear is pending, present empty (the
      // stale pushed entry is invisible until the capture rule clears
      // the state one cycle later).
      if (rg_clear_tog != rg_clear_seen) begin
	 Output_Stage1 o = ?;
	 o.ostatus = OSTATUS_EMPTY;
	 return o;
      end
      // Live epoch re-check (stock semantics: discardable until deq).
      else if (out_v0 && (tpl_1 (out_r0) != cur_epoch)) begin
	 let o = tpl_2 (out_r0);
	 o.ostatus = OSTATUS_PIPE;
	 o.control = CONTROL_DISCARD;
	 return o;
      end
      else if (out_v0) begin
	 return tpl_2 (out_r0);
      end
      else if (out_v1 && (tpl_1 (out_r1) != cur_epoch)) begin
	 let o = tpl_2 (out_r1);
	 o.ostatus = OSTATUS_PIPE;
	 o.control = CONTROL_DISCARD;
	 return o;
      end
      else if (out_v1) begin
	 return tpl_2 (out_r1);
      end
      else if (f_in.notEmpty) begin
	 // capture in flight
	 Output_Stage1 o = ?;
	 o.ostatus = OSTATUS_BUSY;
	 return o;
      end
      else begin
	 Output_Stage1 o = ?;
	 o.ostatus = OSTATUS_EMPTY;
	 return o;
      end
   endmethod

   method Action deq ();
      pw_out_deq.send;
   endmethod

   // ---- Input
   method Action enq (Data_StageD_to_Stage1  data);
      if (f_in.notFull) begin
	 f_in.enq (data);
	 if (verbosity > 1)
	    $display ("    CPU_Stage1.enq: 0x%08h", data.pc);
      end
   endmethod

   method Action set_full (Bool full);
      // vestigial under the elastic stage (occupancy is real); the
      // drain/flush protocol uses deq/ma_clear instead.
      noAction;
   endmethod

   method Bool mv_can_enq;
      return f_in.notFull;
   endmethod

   method Action ma_clear;
      rg_clear_tog <= ! rg_clear_tog;
   endmethod
endmodule

// ================================================================

endpackage: CPU_Stage1
