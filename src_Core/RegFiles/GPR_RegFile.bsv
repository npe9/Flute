// Copyright (c) 2016-2020 Bluespec, Inc. All Rights Reserved

package GPR_RegFile;

// ================================================================
// GPR (General Purpose Register) Register File

// ================================================================
// Exports

export GPR_RegFile_IFC (..), mkGPR_RegFile;

// ================================================================
// BSV library imports

import ConfigReg    :: *;
import RegFile      :: *;
import FIFOF        :: *;
import GetPut       :: *;
import ClientServer :: *;

// BSV additional libs

import GetPut_Aux :: *;

// ================================================================
// Project imports

import ISA_Decls :: *;

// ================================================================

interface GPR_RegFile_IFC;
   // Reset
   interface Server #(Token, Token) server_reset;

   // GPR read
   (* always_ready *)
   method Word read_rs1 (RegName rs1);
   (* always_ready *)
   method Word read_rs1_port2 (RegName rs1);    // For debugger access only
   (* always_ready *)
   method Word read_rs2 (RegName rs2);

   // GPR write
   (* always_ready *)
   method Action write_rd (RegName rd, Word rd_val);

   // LX38: registered operand reads for Stage1.  ma_prefetch is called every cycle with
   // StageD's rs1/rs2; ma_load when Stage1 takes StageD's instruction.  read_rs1_q /
   // read_rs2_q are the register file contents at the start of the cycle for the
   // registers named by the instruction now in Stage1 (as read_rs1/read_rs2 of its
   // rs1/rs2), but come from flops: writes are forwarded into the captured values.
   (* always_ready *)
   method Action ma_prefetch (RegName rs1, RegName rs2);
   (* always_ready *)
   method Action ma_load;
   (* always_ready *)
   method Word read_rs1_q;
   (* always_ready *)
   method Word read_rs2_q;
endinterface

// ================================================================
// Major states of mkGPR_RegFile module

typedef enum { RF_RESET_START, RF_RESETTING, RF_RUNNING } RF_State
deriving (Eq, Bits, FShow);

// ================================================================

(* synthesize *)
module mkGPR_RegFile (GPR_RegFile_IFC);

   Reg #(RF_State) rg_state      <- mkReg (RF_RESET_START);

   // Reset
   FIFOF #(Token) f_reset_rsps <- mkFIFOF;

   // General Purpose Registers
   // TODO: can we use Reg [0] for some other purpose?
   RegFile #(RegName, Word) regfile <- mkRegFileFull;

   // ----------------------------------------------------------------
   // Reset.
   // This loop initializes all GPRs to 0.
   // The spec does not require this, but it's useful for debugging
   // and tandem verification

// TestRIG (RVFI_DII) also needs x1..x31 = 0 at every reset.
`ifdef INCLUDE_TANDEM_VERIF
`define GPR_RESET_ZERO
`endif
`ifdef RVFI_DII
`define GPR_RESET_ZERO
`endif

`ifdef GPR_RESET_ZERO
   Reg #(RegName) rg_j <- mkRegU;    // reset loop index
`endif

   // ----------------
   // LX38: registered operand reads (see interface).  rg_new_* hold the prefetched
   // registers for an instruction entering Stage1; rg_cur_* those of the instruction
   // held in Stage1; rg_sel_new picks between them.  Every write is forwarded into both.
   RWire #(Tuple2 #(RegName, Word)) rw_wr <- mkRWire;
   Wire #(RegName) dw_pre1 <- mkDWire (0);
   Wire #(RegName) dw_pre2 <- mkDWire (0);
   Wire #(Word)    dw_pre_v1 <- mkDWire (0);    // read in ma_prefetch: RegFile sub < upd
   Wire #(Word)    dw_pre_v2 <- mkDWire (0);
   PulseWire       pw_load <- mkPulseWire;

   Reg #(Bool)    rg_sel_new <- mkReg (False);
   Reg #(RegName) rg_new_a1  <- mkReg (0);
   Reg #(RegName) rg_new_a2  <- mkReg (0);
   Reg #(Word)    rg_new_v1  <- mkReg (0);
   Reg #(Word)    rg_new_v2  <- mkReg (0);
   Reg #(RegName) rg_cur_a1  <- mkReg (0);
   Reg #(RegName) rg_cur_a2  <- mkReg (0);
   Reg #(Word)    rg_cur_v1  <- mkReg (0);
   Reg #(Word)    rg_cur_v2  <- mkReg (0);

   RegName cur_a1 = (rg_sel_new ? rg_new_a1 : rg_cur_a1);
   RegName cur_a2 = (rg_sel_new ? rg_new_a2 : rg_cur_a2);
   Word    cur_v1 = (rg_sel_new ? rg_new_v1 : rg_cur_v1);
   Word    cur_v2 = (rg_sel_new ? rg_new_v2 : rg_cur_v2);

   function Word fv_fwd (RegName r, Word v);
      Word x = v;
      if (rw_wr.wget matches tagged Valid { .rd, .rd_val } &&& (rd == r) &&& (r != 0))
	 x = rd_val;
      return x;
   endfunction

   function Word fv_rd (RegName r) = ((r == 0) ? 0 : regfile.sub (r));

   (* fire_when_enabled, no_implicit_conditions *)
   rule rl_track;
      rg_new_a1  <= dw_pre1;
      rg_new_a2  <= dw_pre2;
      rg_new_v1  <= fv_fwd (dw_pre1, dw_pre_v1);
      rg_new_v2  <= fv_fwd (dw_pre2, dw_pre_v2);
      rg_cur_a1  <= cur_a1;
      rg_cur_a2  <= cur_a2;
      rg_cur_v1  <= fv_fwd (cur_a1, cur_v1);
      rg_cur_v2  <= fv_fwd (cur_a2, cur_v2);
      rg_sel_new <= pw_load;
   endrule

   rule rl_reset_start (rg_state == RF_RESET_START);
      rg_state <= RF_RESETTING;
`ifdef GPR_RESET_ZERO
      rg_j <= 1;
`endif
   endrule

   rule rl_reset_loop (rg_state == RF_RESETTING);
`ifdef GPR_RESET_ZERO
      regfile.upd (rg_j, 0);
      rg_j <= rg_j + 1;
      if (rg_j == 31)
	 rg_state <= RF_RUNNING;
`else
      rg_state <= RF_RUNNING;
`endif
   endrule

   // ----------------------------------------------------------------
   // INTERFACE

   // Reset
   interface Server server_reset;
      interface Put request;
	 method Action put (Token token);
	    rg_state <= RF_RESET_START;

	    // This response is placed here, and not in rl_reset_loop, because
	    // reset_loop can happen on power-up, where no response is expected.
	    f_reset_rsps.enq (?);
	 endmethod
      endinterface
      interface Get response;
	 method ActionValue #(Token) get if (rg_state == RF_RUNNING);
	    let token <- pop (f_reset_rsps);
	    return token;
	 endmethod
      endinterface
   endinterface

   // GPR read
   method Word read_rs1 (RegName rs1);
      return ((rs1 == 0) ? 0 : regfile.sub (rs1));
   endmethod

   // GPR read
   method Word read_rs1_port2 (RegName rs1);        // For debugger access only
      return ((rs1 == 0) ? 0 : regfile.sub (rs1));
   endmethod

   method Word read_rs2 (RegName rs2);
      return ((rs2 == 0) ? 0 : regfile.sub (rs2));
   endmethod

   // GPR write
   method Action write_rd (RegName rd, Word rd_val);
      if (rd != 0) regfile.upd (rd, rd_val);
      rw_wr.wset (tuple2 (rd, rd_val));
   endmethod

   method Action ma_prefetch (RegName rs1, RegName rs2);
      dw_pre1   <= rs1;
      dw_pre2   <= rs2;
      dw_pre_v1 <= fv_rd (rs1);
      dw_pre_v2 <= fv_rd (rs2);
   endmethod

   method Action ma_load;
      pw_load.send;
   endmethod

   method Word read_rs1_q = cur_v1;
   method Word read_rs2_q = cur_v2;

endmodule

// ================================================================

endpackage
