// Copyright (c) 2026 -- ULX3S Flute Linux program (TestRIG RVFI-DII)

package DII_AXI4_Mem;

// ================================================================
// AXI4 slaves for the RVFI-DII harness (64-bit data, INCR/FIXED bursts).
//
// is_dmem = True: the CPU's data-side master.  RAM = [ram_base, ram_lim),
//    held in C (dii_mem.c, sparse) so it can be zeroed at every TestRIG
//    reset.  Accesses outside RAM get SLVERR.
// is_dmem = False: the CPU's instruction-side master.  Under RVFI-DII the
//    instruction bits come from the TestRIG trace, so the I-side only needs
//    its requests answered: every read returns OKAY with zero data, for any
//    address (Sail's RVFI-DII fetch checks translation and PMP but not the
//    physical memory map).
// ================================================================

import FIFOF :: *;

import Semi_FIFOF  :: *;
import AXI4_Types  :: *;

// ================================================================

import "BDPI" function Action                 dii_mem_clear ();
import "BDPI" function ActionValue #(Bit #(64)) dii_mem_read64 (Bit #(64) addr);
import "BDPI" function Action                 dii_mem_write64 (Bit #(64) addr, Bit #(64) data, Bit #(8) strb);

interface DII_AXI4_Mem_IFC #(numeric type wd_id, numeric type wd_addr, numeric type wd_data, numeric type wd_user);
   interface AXI4_Slave_IFC #(wd_id, wd_addr, wd_data, wd_user) slave;
   method Action clear;     // zero the backing store (D-side only)
endinterface

function Bit #(64) fn_beat_addr (Bit #(64) addr, AXI4_Size size, Bit #(2) burst, Bit #(8) beat);
   Bit #(64) incr = (64'h1 << size);
   return ((burst == axburst_fixed) ? addr : (addr + (zeroExtend (beat) * incr)));
endfunction

module mkDII_AXI4_Mem #(Bool is_dmem, Bit #(64) ram_base, Bit #(64) ram_lim)
                      (DII_AXI4_Mem_IFC #(wd_id, wd_addr, wd_data, wd_user))
   provisos (NumAlias #(wd_addr, 64), NumAlias #(wd_data, 64));

   AXI4_Slave_Xactor_IFC #(wd_id, wd_addr, wd_data, wd_user) xactor <- mkAXI4_Slave_Xactor;

   function Bool fn_in_ram (Bit #(64) a);
      Bit #(64) w = a & (~ 7);
      return ((ram_base <= w) && (w < ram_lim));
   endfunction

   // +dii_memtrace: log every D-side beat
   Reg #(Bool) rg_trace <- mkReg (False);
   Reg #(Bool) rg_trace_init <- mkReg (False);
   rule rl_trace_init (! rg_trace_init);
      let t <- $test$plusargs ("dii_memtrace");
      rg_trace <= t && is_dmem;
      rg_trace_init <= True;
   endrule

   Reg #(Bit #(8)) rg_rd_beat <- mkReg (0);
   Reg #(Bit #(8)) rg_wr_beat <- mkReg (0);
   Reg #(Bool)     rg_wr_err  <- mkReg (False);

   rule rl_rd (rg_trace_init);
      let ar   = xactor.o_rd_addr.first;
      let addr = fn_beat_addr (ar.araddr, ar.arsize, ar.arburst, rg_rd_beat);
      Bit #(64) data = 0;
      Bool ok = True;
      if (is_dmem) begin
	 ok = fn_in_ram (addr);
	 data <- dii_mem_read64 (addr);
      end
      Bool last = (rg_rd_beat == ar.arlen);
      if (rg_trace) $display ("DII_MEM RD addr %0h len %0d sz %0d beat %0d data %016h ok %0d", ar.araddr, ar.arlen, ar.arsize, rg_rd_beat, data, ok);
      xactor.i_rd_data.enq (AXI4_Rd_Data {rid:   ar.arid,
					  rdata: data,
					  rresp: (ok ? axi4_resp_okay : axi4_resp_slverr),
					  rlast: last,
					  ruser: ar.aruser});
      if (last) begin
	 xactor.o_rd_addr.deq;
	 rg_rd_beat <= 0;
      end
      else
	 rg_rd_beat <= rg_rd_beat + 1;
   endrule

   rule rl_wr (rg_trace_init);
      let aw   = xactor.o_wr_addr.first;
      let w    = xactor.o_wr_data.first;
      xactor.o_wr_data.deq;
      let addr = fn_beat_addr (aw.awaddr, aw.awsize, aw.awburst, rg_wr_beat);
      Bool ok  = is_dmem && fn_in_ram (addr);
      if (ok) dii_mem_write64 (addr, w.wdata, w.wstrb);
      if (rg_trace) $display ("DII_MEM WR addr %0h len %0d sz %0d beat %0d data %016h strb %02h ok %0d", aw.awaddr, aw.awlen, aw.awsize, rg_wr_beat, w.wdata, w.wstrb, ok);
      Bool err = rg_wr_err || (! ok);
      if (w.wlast) begin
	 xactor.o_wr_addr.deq;
	 xactor.i_wr_resp.enq (AXI4_Wr_Resp {bid:   aw.awid,
					     bresp: (err ? axi4_resp_slverr : axi4_resp_okay),
					     buser: aw.awuser});
	 rg_wr_beat <= 0;
	 rg_wr_err  <= False;
      end
      else begin
	 rg_wr_beat <= rg_wr_beat + 1;
	 rg_wr_err  <= err;
      end
   endrule

   interface slave = xactor.axi_side;

   method Action clear;
      if (is_dmem) dii_mem_clear;
   endmethod
endmodule

// ================================================================

endpackage
