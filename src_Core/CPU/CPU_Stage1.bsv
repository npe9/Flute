// Copyright (c) 2016-2021 Bluespec, Inc. All Rights Reserved

package CPU_Stage1;

// ================================================================
// This is Stage 1 of the "Flute" CPU.
// It contains the EX functionality.
// EX: "Execute"

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

   Reg #(Bool)                  rg_full        <- mkReg (False);
   Reg #(Data_StageD_to_Stage1) rg_stage_input <- mkRegU;

   MISA misa   = csr_regfile.read_misa;
   Bit #(2) xl = ((xlen == 32) ? misa_mxl_32 : misa_mxl_64);

   // ----------------------------------------------------------------
   // BEHAVIOR

   rule rl_reset;
      f_reset_reqs.deq;
      rg_full <= False;
      f_reset_rsps.enq (?);
   endrule

   // ----------------
   // ALU

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

   // E2: control classification (PIPE/NONPIPE/trap, which nonpipe op) is a
   // function of the decoded instruction and CSR state only -- never of the
   // operand VALUES (the only value-dependent control output is
   // BRANCH-vs-STRAIGHT, and both are PIPE).  Evaluate the ALU a second
   // time with the operands tied to zero and take control/exc_code from
   // that copy: synthesis prunes the regfile/bypass/compare cones out of
   // `ostatus`, which is read by rl_pipe's guard and hence by every stage
   // enable.  The real alu_outputs feed only data_to_stage2 (register
   // D-pins).
   let ctl_inputs = alu_inputs;
   ctl_inputs.rs1_val = 0;
   ctl_inputs.rs2_val = 0;
   let ctl_outputs = fv_ALU (ctl_inputs);
   let fall_through_pc = rg_stage_input.pc + (rg_stage_input.is_i32_not_i16 ? 4 : 2);

   // E42: next_pc and redirect without a compare after the operand cone.
   // Only BRANCH (taken), JAL and JALR resolve to CONTROL_BRANCH.  Their
   // targets are pc-relative (operand-independent) except JALR's, so the
   // "!= pred_pc" compares run in parallel with the register read and only
   // a late select by the (value-dependent) control remains.  For JALR,
   // ((rs1 + imm) & ~1) == pred  <=>  pred even and rs1 is pred - imm or
   // pred - imm + 1: equality compares of rs1 against register-only values
   // instead of adder -> compare.  (It was regfile -> bypass -> branch
   // compare -> addr mux -> next_pc mux -> 32-bit compare -> stage2 reg.)
   let opcode   = decoded_instr.opcode;
   Addr pred    = rg_stage_input.pred_pc;
   IntXL off_b  = extend (unpack (decoded_instr.imm13_SB));
   IntXL off_j  = extend (unpack (decoded_instr.imm21_UJ));
   IntXL off_i  = extend (unpack (decoded_instr.imm12_I));
   Addr br_tgt  = pack (unpack (rg_stage_input.pc) + off_b);
   Addr jal_tgt = pack (unpack (rg_stage_input.pc) + off_j);
   Addr jalr_tgt = pack (unpack (rs1_val_bypassed) + off_i);
   jalr_tgt [0] = 1'b0;
   Addr jalr_k0 = pack (unpack ({ pred [xlen-1:1], 1'b0 }) - off_i);
   Bool jalr_hit = (pred [0] == 1'b0)
                   && ((rs1_val_bypassed == jalr_k0) || (rs1_val_bypassed == jalr_k0 + 1));
   Bool is_jal  = (opcode == op_JAL);
   Bool is_jalr = (opcode == op_JALR);
   Addr ctl_tgt = (is_jal ? jal_tgt : (is_jalr ? jalr_tgt : br_tgt));
   Bool tgt_miss = (is_jal ? (jal_tgt != pred) : (is_jalr ? (! jalr_hit) : (br_tgt != pred)));
   Bool ft_miss  = (fall_through_pc != pred);
   Bool resolved_branch  = (alu_outputs.control == CONTROL_BRANCH);
   let  resolved_next_pc = (resolved_branch ? ctl_tgt : fall_through_pc);
   Bool resolved_redirect = (resolved_branch ? tgt_miss : ft_miss);

   let data_to_stage2 = Data_Stage1_to_Stage2 {pc            : rg_stage_input.pc,
					       instr         : rg_stage_input.instr,
					       op_stage2     : alu_outputs.op_stage2,
					       rd            : alu_outputs.rd,
					       addr          : alu_outputs.addr,
					       val1          : alu_outputs.val1,
					       val2          : alu_outputs.val2,
					       redirect      : resolved_redirect,
					       next_pc       : resolved_next_pc,
					       cf_info       : alu_outputs.cf_info,
`ifdef RVFI
					       is_i32_not_i16 : rg_stage_input.is_i32_not_i16,
					       instr_C        : rg_stage_input.instr_C,
					       rs1_val        : rs1_val_bypassed,
					       rs2_val        : rs2_val_bypassed,
`endif
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

   function Output_Stage1 fv_out;
      Output_Stage1 output_stage1 = ?;

      // This stage is empty
      if (! rg_full) begin
	 output_stage1.ostatus = OSTATUS_EMPTY;
      end

      // Wrong branch-prediction epoch: discard instruction (convert into a NOOP)
      else if (rg_stage_input.epoch != cur_epoch) begin
	 output_stage1.ostatus = OSTATUS_PIPE;
	 output_stage1.control = CONTROL_DISCARD;

	 // For debugging only
	 let data_to_stage2 = Data_Stage1_to_Stage2 {pc:        rg_stage_input.pc,
						     instr:     rg_stage_input.instr,
						     op_stage2: OP_Stage2_ALU,
						     rd:        0,
						     addr:      ?,
						     val1:      ?,
						     val2:      ?,
						     redirect:  False,
						     next_pc:   ?,
						     cf_info:   cf_info_none,
`ifdef RVFI
						     is_i32_not_i16: ?,
						     instr_C:   ?,
						     rs1_val:   ?,
						     rs2_val:   ?,
`endif
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

	 output_stage1.data_to_stage2 = data_to_stage2;
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
	 let ostatus = (  (   (ctl_outputs.control == CONTROL_STRAIGHT)
			   || (ctl_outputs.control == CONTROL_BRANCH))
			? OSTATUS_PIPE
			: OSTATUS_NONPIPE);

	 // Compute MTVAL in case of traps
	 let tval = 0;
	 if (ctl_outputs.exc_code == exc_code_ILLEGAL_INSTRUCTION) begin
	    // The instruction
`ifdef ISA_C
	    tval = (rg_stage_input.is_i32_not_i16
		    ? zeroExtend (rg_stage_input.instr)
		    : zeroExtend (rg_stage_input.instr_C));
`else
	    tval = zeroExtend (rg_stage_input.instr);
`endif
	 end
	 else if (ctl_outputs.exc_code == exc_code_INSTR_ADDR_MISALIGNED)
	    tval = ctl_outputs.addr;                           // The branch target pc
	 else if (ctl_outputs.exc_code == exc_code_BREAKPOINT)
	    tval = rg_stage_input.pc;                          // The faulting virtual address

	 let trap_info = Trap_Info {epc:      rg_stage_input.pc,
				    exc_code: ctl_outputs.exc_code,
				    tval:     tval};

	 // E2: redirect/next_pc/cf_info travel to stage2 as data (see
	 // data_to_stage2); the stage1 feedback fields are decode-only.
	 // next_pc here is the fall-through, which is what the nonpipe
	 // rules (CSRRx/FENCE/WFI/xRET restart) need.
	 output_stage1.ostatus        = ostatus;
	 output_stage1.control        = ctl_outputs.control;
	 output_stage1.trap_info      = trap_info;
	 output_stage1.redirect       = False;
	 output_stage1.next_pc        = fall_through_pc;
	 output_stage1.cf_info        = cf_info_none;
	 output_stage1.data_to_stage2 = data_to_stage2;
      end

      return output_stage1;
   endfunction: fv_out

   // ================================================================
   // INTERFACE

   // ---- Reset
   interface server_reset = toGPServer (f_reset_reqs, f_reset_rsps);

   // ---- Output
   method Output_Stage1 out;
      return fv_out;
   endmethod

   method Action deq ();
   endmethod

   // ---- Input
   method Action enq (Data_StageD_to_Stage1  data);
      rg_stage_input <= data;
      if (verbosity > 1)
	 $display ("    CPU_Stage1.enq: 0x%08h", data.pc);
   endmethod

   method Action set_full (Bool full);
      rg_full <= full;
   endmethod
endmodule

// ================================================================

endpackage
