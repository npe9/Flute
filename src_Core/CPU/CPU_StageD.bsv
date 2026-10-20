// Copyright (c) 2016-2021 Bluespec, Inc. All Rights Reserved

package CPU_StageD;

// ================================================================
// This is Stage D ("decode") of the "Flute_V3" CPU.

// ================================================================
// Exports

export
CPU_StageD_IFC (..),
mkCPU_StageD;

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

`ifdef ISA_C
// 'C' extension (16b compressed instructions)
import CPU_Decode_C     :: *;
`endif

// ================================================================
// Interface

interface CPU_StageD_IFC;
   // ---- Reset
   interface Server #(Token, Token) server_reset;

   // ---- Output
   (* always_ready *)
   method Output_StageD out;

   (* always_ready *)
   method Action deq;

   // ---- Input
   (* always_ready *)
   method Action enq (Data_StageF_to_StageD  data_stageF_to_stageD);

   // E14: set_full (False) flushes; set_full (True) is a no-op (occupancy is
   // the FIFO's).  can_enq is registered (the FIFO's notFull).
   (* always_ready *)
   method Action set_full (Bool full);

   (* always_ready *)
   method Bool can_enq;
endinterface

// ================================================================
// Implementation module

module mkCPU_StageD #(Bit #(4)  verbosity, MISA misa)
                    (CPU_StageD_IFC);

   FIFOF #(Token)  f_reset_reqs <- mkFIFOF;
   FIFOF #(Token)  f_reset_rsps <- mkFIFOF;

   // E14: two-entry FIFO instead of a single full/data register pair.  The
   // F->D move (and so the next fetch request) is decided from can_enq, a
   // register, not from whether stage1 drains this cycle -- which rippled
   // the dcache hit (stage2 status) through stage1/D/F into the imem request
   // and BTB address (the 22 ns E13 path).  enq -> first latency is still
   // one cycle (mkFIFOF), and two entries keep full throughput.
   FIFOF #(Data_StageF_to_StageD) f_data <- mkUGFIFOF;
   Bool                           rg_full = f_data.notEmpty;
   Data_StageF_to_StageD          rg_data = f_data.first;

   Bit #(2) xl = ((xlen == 32) ? misa_mxl_32 : misa_mxl_64);

   Instr instr = rg_data.instr;
`ifdef ISA_C
   Instr_C instr_C = instr [15:0];
   if (! rg_data.is_i32_not_i16)
      instr = fv_decode_C (misa, xl, instr_C);
`endif

   // ----------------------------------------------------------------
   // BEHAVIOR

   rule rl_reset;
      f_reset_reqs.deq;
      f_data.clear;
      f_reset_rsps.enq (?);
   endrule

   // ----------------
   // Combinational output function

   function Output_StageD fv_out;
      let decoded_instr = fv_decode (instr);

      Output_StageD  output_stageD = ?;

      // This stage is empty
      if (! rg_full) begin
	 output_stageD.ostatus = OSTATUS_EMPTY;
      end
      else begin
	 output_stageD.ostatus        = OSTATUS_PIPE;
	 output_stageD.data_to_stage1 = Data_StageD_to_Stage1 {pc:             rg_data.pc,
							       priv:           rg_data.priv,
							       epoch:          rg_data.epoch,
							       is_i32_not_i16: rg_data.is_i32_not_i16,
							       exc:            rg_data.exc,
							       exc_code:       rg_data.exc_code,
							       tval:           rg_data.tval,
							       instr:          instr,
`ifdef ISA_C
							       instr_C:        instr_C,
`endif
							       pred_pc:        rg_data.pred_pc,
							       decoded_instr:  decoded_instr};
      end

      return output_stageD;
   endfunction: fv_out

   // ================================================================
   // INTERFACE

   // ---- Reset
   interface server_reset = toGPServer (f_reset_reqs, f_reset_rsps);

   // ---- Output
   method Output_StageD out;
      return fv_out;
   endmethod

   method Action deq ();
      f_data.deq;
   endmethod

   // ---- Input
   method Action enq (Data_StageF_to_StageD  data);
      f_data.enq (data);
      if (verbosity > 1)
	 $display ("    CPU_StageD.enq (Data_StageF_to_StageD)");
   endmethod

   method Action set_full (Bool full);
      if (! full) f_data.clear;
   endmethod

   method Bool can_enq = f_data.notFull;
endmodule

// ================================================================

endpackage
