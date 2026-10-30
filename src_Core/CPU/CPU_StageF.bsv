// Copyright (c) 2016-2021 Bluespec, Inc. All Rights Reserved

package CPU_StageF;

// ================================================================
// This is Stage F ("fetch") of the "Flute_V3" CPU.

// ================================================================
// Exports

export
CPU_StageF_IFC (..),
mkCPU_StageF;

// ================================================================
// BSV library imports

import ConfigReg    :: *;
import FIFOF        :: *;
import GetPut       :: *;
import ClientServer :: *;

// ----------------
// BSV additional libs

import Cur_Cycle :: *;

// ================================================================
// Project imports

import ISA_Decls         :: *;
import CPU_Globals       :: *;
import Near_Mem_IFC      :: *;
import Branch_Predictor  :: *;

// ================================================================
// Interface

interface CPU_StageF_IFC;
   // ---- Reset
   interface Server #(Token, Token) server_reset;

   // ---- Output
   (* always_ready *)
   method Output_StageF out;

   (* always_ready *)
   method Action deq;

   // ---- Input
   (* always_ready *)
   method Action enq (Epoch            epoch,
		      WordXL           pc,
		      Priv_Mode        priv,
		      Bit #(1)         sstatus_SUM,
		      Bit #(1)         mstatus_MXR,
		      WordXL           satp);

   // RLD-1': no longer always_ready: enqueue into the internal
   // training FIFO (drained by rl_bp_train_drain in the implementation).
   method Action bp_train (WordXL   pc,
			   Bool     is_i32_not_i16,
			   Instr    instr,
			   CF_Info  cf_info);

   // E14: RAS training from the fetch side (CPU.rl_front)
   (* always_ready *)
   method Action bp_train_ras (WordXL pc, Bool is_i32_not_i16, Instr instr);

   (* always_ready *)
   method Action set_full (Bool full);

   // E29: registered "not BUSY" for the NONPIPE / halting guards in CPU.bsv.
   (* always_ready *)
   method Bool not_busy;
endinterface

// ================================================================
// Implementation module

module mkCPU_StageF #(Bit #(4)  verbosity,
		      IMem_IFC  imem)
                    (CPU_StageF_IFC);

   FIFOF #(Token)  f_reset_reqs <- mkFIFOF;
   FIFOF #(Token)  f_reset_rsps <- mkFIFOF;

   Wire #(Bool)      dw_redirecting <- mkDWire (False);

   // E14: ConfigReg so CPU.rl_pipe (pipe_is_empty reads out.ostatus) can be
   // scheduled after CPU.rl_front (which writes this); hardware unchanged.
   Reg #(Bool)       rg_full  <- mkConfigReg (False);
   Reg #(Epoch)      rg_epoch <- mkReg (0);               // Toggles on redirections
   Reg #(Priv_Mode)  rg_priv  <- mkRegU;

   // E29: rg_fetch_seen = the current imem request has presented (valid) at least
   // once.  ICache_Pipe's valid is monotonic between requests (the queue clears
   // only on a miss -- never while the wanted cword is present -- or on a
   // flush/reset, which only CPU NONPIPE paths start and which re-enq StageF), so
   // (! rg_full) || rg_fetch_seen implies "not BUSY", at most one cycle late.
   // The NONPIPE rules' guards used out.ostatus != BUSY, i.e. the icache's
   // combinational queue match, which then reached rg_state, CSR write enables and
   // near_mem requests (20.8 ns at the 50 MHz constraint).
   Reg #(Bool)       rg_fetch_seen <- mkReg (False);
   PulseWire         pw_fetch_req  <- mkPulseWire;

   (* fire_when_enabled, no_implicit_conditions *)
   rule rl_fetch_seen;
      rg_fetch_seen <= ((! pw_fetch_req) && (rg_fetch_seen || imem.valid));
   endrule

   Branch_Predictor_IFC branch_predictor <- mkBranch_Predictor;

   // RLD-1': stage branch-predictor training through a small FIFO and
   // drain it with a local rule.  This decouples the BTB write from
   // the CPU's rl_pipe comb cone: stock called bp_train directly from
   // rl_pipe, putting stage1's cf_info computation (ALU + regfile
   // read for JALR targets) and the rl_pipe move-guard chain into the
   // same cloud as the BTB RAM write pin (the 33.06 ns binding path
   // on the RLD-0 netlist).  Enq here ends that cone at a FIFO data
   // pin; the drain-side cone (FIFO Q -> BTB) is short.
   // E14: unguarded.  The method's old guard (if f_bp_train.notFull),
   // predicated on "something to train", put the stage2 move decision (dcache
   // hit) into WILL_FIRE_rl_pipe and so into every rl_pipe action, including
   // the fetch request.  The drain rule runs every cycle, so the 2-entry
   // FIFO is never full in practice; if it were, the hint is dropped.
   FIFOF #(Tuple4 #(WordXL, Bool, Instr, CF_Info)) f_bp_train <- mkUGFIFOF;

   // E14: RAS training (fetch side, CPU.rl_front) and BTB training (resolve
   // side, CPU.rl_pipe) come from different rules, so they queue separately
   // and the drain merges them into one bp_train call (pc/instr 0 = no RAS
   // action, cf_info_none = no BTB action).
   FIFOF #(Tuple3 #(WordXL, Bool, Instr)) f_ras_train <- mkUGFIFOF;

   rule rl_bp_train_drain (f_bp_train.notEmpty || f_ras_train.notEmpty);
      WordXL  pc      = 0;
      Bool    is_i32  = True;
      Instr   instr   = 0;
      CF_Info cf_info = cf_info_none;
      if (f_ras_train.notEmpty) begin
	 match { .p, .i, .x } = f_ras_train.first;
	 f_ras_train.deq;
	 pc = p; is_i32 = i; instr = x;
      end
      if (f_bp_train.notEmpty) begin
	 match { .p2, .i2, .x2, .c } = f_bp_train.first;
	 f_bp_train.deq;
	 cf_info = c;
      end
      branch_predictor.bp_train (pc, is_i32, instr, cf_info);
   endrule

   // ----------------------------------------------------------------
   // BEHAVIOR

   rule rl_reset;
      f_reset_reqs.deq;
      rg_full  <= False;
      rg_epoch <= 0;
      f_reset_rsps.enq (?);
   endrule

   // ----------------
   // Combinational output function

   function Output_StageF fv_out;
      let pred_pc = branch_predictor.predict_rsp (imem.is_i32_not_i16, imem.instr);
      let d = Data_StageF_to_StageD {pc:              imem.pc,
				     epoch:           rg_epoch,
				     priv:            rg_priv,
				     is_i32_not_i16:  imem.is_i32_not_i16,
				     exc:             imem.exc,
				     exc_code:        imem.exc_code,
				     tval:            imem.tval,
				     instr:           imem.instr,
				     pred_pc:         pred_pc};

      let ostatus = (  (! rg_full) ? OSTATUS_EMPTY
		     : (  (! imem.valid) ? OSTATUS_BUSY
			: OSTATUS_PIPE));                    // instr or exception

      let output_stageF = Output_StageF {ostatus: ostatus, data_to_stageD: d};

      return output_stageF;
   endfunction: fv_out

   // ================================================================
   // INTERFACE

   // ---- Reset
   interface server_reset = toGPServer (f_reset_reqs, f_reset_rsps);

   // ---- Output
   method Output_StageF out;
      return fv_out;
   endmethod

   method Action deq ();
      noAction;
   endmethod

   // ---- Input
   method Action enq (Epoch            epoch,
		      WordXL           pc,
		      Priv_Mode        priv,
		      Bit #(1)         sstatus_SUM,
		      Bit #(1)         mstatus_MXR,
		      WordXL           satp);
      if (verbosity > 1) begin
	 $write ("    %m.CPU_StageF.ma_enq:  pc:0x%0h  epoch:%0d  priv:%0d", pc, epoch, priv);
	 $write ("  sstatus_SUM:%0d  mstatus_MXR:%0d  satp:0x%0h",
		 sstatus_SUM, mstatus_MXR, satp);
	 $display ("");
      end

      imem.req (f3_LW, pc, priv, sstatus_SUM, mstatus_MXR, satp);
      pw_fetch_req.send;    // E29
      branch_predictor.predict_req (pc);    // TODO: ASID.VA vs PA?

      rg_epoch <= epoch;
      rg_priv  <= priv;
   endmethod

   method Action bp_train (WordXL   pc,
			   Bool     is_i32_not_i16,
			   Instr    instr,
			   CF_Info  cf_info);
      if (f_bp_train.notFull)
	 f_bp_train.enq (tuple4 (pc, is_i32_not_i16, instr, cf_info));
   endmethod

   method Action bp_train_ras (WordXL pc, Bool is_i32_not_i16, Instr instr);
      if (f_ras_train.notFull)
	 f_ras_train.enq (tuple3 (pc, is_i32_not_i16, instr));
   endmethod

   method Action set_full (Bool full);
      rg_full <= full;
   endmethod

   method Bool not_busy = ((! rg_full) || rg_fetch_seen);
endmodule

// ================================================================

endpackage
