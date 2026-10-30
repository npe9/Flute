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
import Vector       :: *;

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

   // E28: StageF's output, offered every cycle (CPU.rl_stageD_feed).  The free
   // ring slot loads it unconditionally; enq only advances the pointer/count.
   (* always_ready *)
   method Action stage (Data_StageF_to_StageD  data_stageF_to_stageD);

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
   //
   // E24: the two entries are a ring (write/read pointers + count), not mkUGFIFOF's shift pair.
   // In the shift FIFO, data0's D input muxes on DEQ (shift data1 -> data0), so the stage1
   // advance -- which waits on stage2's dcache hit -- reached 72 data flops (the 21 ns E23 path
   // at the 50 MHz constraint).  Here DEQ only moves rg_rp/rg_cnt; the data flops load only on
   // enq.  All state updates are in rl_update, from wires, so the methods keep the FIFO's
   // read-before-write schedule.
   Vector #(2, Reg #(Data_StageF_to_StageD)) vr_data <- replicateM (mkRegU);
   Reg #(Bit #(1))  rg_wp  <- mkReg (0);
   Reg #(Bit #(1))  rg_rp  <- mkReg (0);
   Reg #(Bit #(2))  rg_cnt <- mkReg (0);
   PulseWire                      pw_enq   <- mkPulseWire;
   // E28: the data flops' enable was WILL_FIRE_rl_front, i.e. the icache's
   // combinational valid (rg_pc vs queue match) -> 73 CEs (19.4 ns at the 50 MHz
   // constraint).  The slot at rg_wp holds nothing live while rg_cnt != 2, so it
   // loads StageF's output every such cycle; the CE is then registers only.
   Wire #(Data_StageF_to_StageD)  dw_stage <- mkDWire (?);
   PulseWire                      pw_deq   <- mkPulseWire;
   PulseWire                      pw_clear <- mkPulseWire;

   Bool                           rg_full = (rg_cnt != 0);
   Data_StageF_to_StageD          rg_data = vr_data [rg_rp];

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
      pw_clear.send;
      f_reset_rsps.enq (?);
   endrule

   (* fire_when_enabled, no_implicit_conditions *)
   rule rl_update;
      if (pw_clear) begin
	 rg_wp  <= 0;
	 rg_rp  <= 0;
	 rg_cnt <= 0;
      end
      else begin
	 Bit #(2) cnt = rg_cnt;
	 if (rg_cnt != 2)
	    vr_data [rg_wp] <= dw_stage;
	 if (pw_enq) begin
	    rg_wp <= rg_wp + 1;
	    cnt = cnt + 1;
	 end
	 if (pw_deq) begin
	    rg_rp <= rg_rp + 1;
	    cnt = cnt - 1;
	 end
	 rg_cnt <= cnt;
      end
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
      pw_deq.send;
   endmethod

   // ---- Input
   // E28: data arrives through stage(); this argument is the same value.
   method Action enq (Data_StageF_to_StageD  data);
      pw_enq.send;
      if (verbosity > 1)
	 $display ("    CPU_StageD.enq (Data_StageF_to_StageD)");
   endmethod

   method Action set_full (Bool full);
      if (! full) pw_clear.send;
   endmethod

   method Action stage (Data_StageF_to_StageD  data);
      dw_stage <= data;
   endmethod

   method Bool can_enq = (rg_cnt != 2);
endmodule

// ================================================================

endpackage
