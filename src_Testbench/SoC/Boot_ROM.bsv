// Copyright (c) 2016-2020 Bluespec, Inc. All Rights Reserved

package Boot_ROM;

// ================================================================
// This package implements a slave IP that is a RISC-V boot ROM of
// 1024 32b locations.
// - Ignores all writes, always responsing OKAY
// - Assumes all reads are 4-byte aligned requests for 4-bytes

// ================================================================

export Boot_ROM_IFC (..), mkBoot_ROM;

// ================================================================
// BSV library imports

import ConfigReg :: *;

// ----------------
// BSV additional libs

import Cur_Cycle  :: *;
import GetPut_Aux :: *;
import Semi_FIFOF :: *;

// ================================================================
// Project imports

import AXI4_Types  :: *;
import Fabric_Defs :: *;

// ================================================================
// Include the auto-generated BSV-include file with the ROM function

`ifdef RV32
`include  "fn_read_ROM_RV32.bsvi"
`endif

`ifdef RV64
`include  "fn_read_ROM_RV64.bsvi"
`endif

// ================================================================
// Interface

interface Boot_ROM_IFC;
   // set_addr_map should be called after this module's reset
   method Action set_addr_map (Fabric_Addr addr_base, Fabric_Addr addr_lim);

   // SoC soft reset (ndmreset): drop transactions in flight with the fabric that is reset with
   // this slave (the fabric routes responses in order; a stale one would go to the wrong request)
   method Action reset;

   // Main Fabric Reqs/Rsps
   interface AXI4_Slave_IFC #(Wd_Id, Wd_Addr, Wd_Data, Wd_User) slave;
endinterface

// ================================================================
// Some local help-functions

function Bool fn_addr_is_aligned (Fabric_Addr addr, AXI4_Size arsize);
   if      (arsize == axsize_1)  return True;
   else if (arsize == axsize_2)  return (addr [0] == 1'b_0);
   else if (arsize == axsize_4)  return (addr [1:0] == 2'b_00);
   else if (arsize == axsize_8)  return (addr [2:0] == 3'b_000);
   else return False;
endfunction

function Bool fn_addr_is_in_range (Fabric_Addr base, Fabric_Addr addr, Fabric_Addr lim);
   return ((base <= addr) && (addr < lim));
endfunction

function Bool fn_addr_is_ok (Fabric_Addr base, Fabric_Addr addr, Fabric_Addr lim, AXI4_Size arsize);
   return (   fn_addr_is_aligned (addr, arsize)
	   && fn_addr_is_in_range (base, addr, lim));
endfunction

// ================================================================

(* synthesize *)
module mkBoot_ROM (Boot_ROM_IFC);

   // Verbosity: 0: quiet; 1: reads/writes
   Integer verbosity = 0;

   Reg #(Bool) rg_module_ready <- mkReg (False);

   Reg #(Fabric_Addr)  rg_addr_base <- mkRegU;
   Reg #(Fabric_Addr)  rg_addr_lim  <- mkRegU;

   // ----------------
   // Connector to fabric

   AXI4_Slave_Xactor_IFC #(Wd_Id, Wd_Addr, Wd_Data, Wd_User) slave_xactor <- mkAXI4_Slave_Xactor;

   // ================================================================
   // BEHAVIOR

   // ----------------------------------------------------------------
   // Handle fabric read requests

   // LX20: the ROM is read into registers (rg_rd_d0 / rg_rd_d4, written only from the ROM
   // functions) and the response goes out next cycle.  A registered ROM output lets yosys
   // map the (PROC_ROM-converted) ROMs to block RAM instead of ~2.4K LUTs of decode.
   // The 4 KiB region needs only a 12-bit offset.  +1 cycle per boot-ROM read.
   Reg #(Bool)       rg_rd_v   <- mkReg (False);
   Reg #(Bit #(32))  rg_rd_d0  <- mkRegU;
   Reg #(Bit #(32))  rg_rd_d4  <- mkRegU;
   Reg #(Bool)       rg_rd_ok  <- mkRegU;
   Reg #(Bool)       rg_rd_hi  <- mkRegU;
   Reg #(AXI4_Rd_Addr #(Wd_Id, Wd_Addr, Wd_User)) rg_rda <- mkRegU;

   rule rl_process_rd_req (rg_module_ready && (! rg_rd_v));
      let rda <- pop_o (slave_xactor.o_rd_addr);
      Bool ok = fn_addr_is_ok (rg_addr_base, rda.araddr, rg_addr_lim, rda.arsize);
      if (! ok) begin
	 $display ("%0d: ERROR: Boot_ROM.rl_process_rd_req: unrecognized or misaligned addr",
		   cur_cycle);
	 $display ("    ", fshow (rda));
      end
      // Byte offset
      let byte_offset = rda.araddr - rg_addr_base;
      Bit #(12) rom_addr_0 = (truncate (byte_offset) & (~ 'b_111));
      Bit #(12) rom_addr_4 = (rom_addr_0 | 'b_100);
      rg_rd_d0 <= fn_read_ROM_0 (rom_addr_0);
      rg_rd_d4 <= fn_read_ROM_4 (rom_addr_4);
      rg_rd_ok <= ok;
      rg_rd_hi <= (byte_offset [2] == 1'b_1);
      rg_rda   <= rda;
      rg_rd_v  <= True;
   endrule

   rule rl_rd_rsp (rg_rd_v);
      let rda = rg_rda;
      AXI4_Resp  rresp  = (rg_rd_ok ? axi4_resp_okay : axi4_resp_slverr);
      Bit #(64)  data64 = 0;
      if (rg_rd_ok) begin
	 if ((valueOf (Wd_Data) == 32) && rg_rd_hi)
	    data64 = { 0, rg_rd_d4 };
	 else
	    data64 = { rg_rd_d4, rg_rd_d0 };
      end

      Bit #(Wd_Data) rdata  = truncate (data64);
      let rdr = AXI4_Rd_Data {rid:   rda.arid,
			      rdata: rdata,
			      rresp: rresp,
			      rlast: True,
			      ruser: rda.aruser};
      slave_xactor.i_rd_data.enq (rdr);
      rg_rd_v <= False;

      if (verbosity > 0) begin
	 $display ("%0d: Boot_ROM.rl_rd_rsp: ", cur_cycle);
	 $display ("        ", fshow (rda));
	 $display ("     => ", fshow (rdr));
      end
   endrule

   // ----------------------------------------------------------------
   // Handle fabric write requests: ignore all of them (this is a ROM)

   rule rl_process_wr_req (rg_module_ready);
      let wra <- pop_o (slave_xactor.o_wr_addr);
      let wrd <- pop_o (slave_xactor.o_wr_data);

      AXI4_Resp  bresp = axi4_resp_okay;
      if (! fn_addr_is_ok (rg_addr_base, wra.awaddr, rg_addr_lim, wra.awsize)) begin
	 bresp = axi4_resp_slverr;
	 $display ("%0d: ERROR: Boot_ROM.rl_process_wr_req: unrecognized or misaligned addr",
		   cur_cycle);
	 $display ("    ", fshow (wra));
      end

      let wrr = AXI4_Wr_Resp {bid:   wra.awid,
			      bresp: bresp,
			      buser: wra.awuser};
      slave_xactor.i_wr_resp.enq (wrr);

      if (verbosity > 0) begin
	 $display ("%0d: Boot_ROM.rl_process_wr_req; ignoring all writes", cur_cycle);
	 $display ("        ", fshow (wra));
	 $display ("        ", fshow (wrd));
	 $display ("     => ", fshow (wrr));
      end
   endrule

   // ================================================================
   // INTERFACE

   // set_addr_map should be called after this module's reset
   method Action  set_addr_map (Fabric_Addr addr_base, Fabric_Addr addr_lim);
      if (valueOf (Wd_Data) == 32) begin
	 if (addr_base [1:0] != 0)
	    $display ("%0d: WARNING: Boot_ROM.set_addr_map: addr_base 0x%0h is not 4-Byte-aligned",
		      cur_cycle, addr_base);

	 if (addr_lim [1:0] != 0)
	    $display ("%0d: WARNING: Boot_ROM.set_addr_map: addr_lim 0x%0h is not 4-Byte-aligned",
		      cur_cycle, addr_lim);
      end
      else if (valueOf (Wd_Data) == 64) begin
	 if (addr_base [2:0] != 0)
	    $display ("%0d: WARNING: Boot_ROM.set_addr_map: addr_base 0x%0h is not 4-Byte-aligned",
		      cur_cycle, addr_base);

	 if (addr_lim [2:0] != 0)
	    $display ("%0d: WARNING: Boot_ROM.set_addr_map: addr_lim 0x%0h is not 4-Byte-aligned",
		      cur_cycle, addr_lim);
      end

      rg_addr_base    <= addr_base;
      rg_addr_lim     <= addr_lim;
      rg_module_ready <= True;
      if (verbosity > 0) begin
	 $display ("%0d: Boot_ROM.set_addr_map: base 0x%0h lim 0x%0h", cur_cycle, addr_base, addr_lim);
      end
   endmethod

   // Main Fabric Reqs/Rsps
   method Action reset;
      slave_xactor.reset;
      rg_rd_v <= False;    // LX20: drop a read held across the reset
   endmethod

   interface  slave = slave_xactor.axi_side;
endmodule

// ================================================================

endpackage
