// Copyright (c) 2016-2020 Bluespec, Inc. All Rights Reserved

package Mem_Controller;

// ================================================================
// This module is a slave on the interconnect Fabric (FABRIC64 only).
//
// On the back side is a streaming "raw" memory port (Sdram_Controller
// on the ULX3S).  A raw request names one 64-byte line and a range of
// its four 16-byte bursts; reads come back as 64-bit chunks in address
// order, writes carry one 32-byte half of a line with byte enables and
// are fire-and-forget.  The raw port executes requests in order.
//
// AXI4 bursts are accepted directly (no Deburster): INCR bursts of
// 64-bit beats, single beats of any size.
//
// Reads stream: an AR becomes one raw read per line it touches, and
// each 64-bit chunk goes out on R as it arrives (an L1 line refill is
// one raw read; its first beat leaves after the first chunk).
//
// Writes are combined in a one-granule (32-byte) write buffer with a
// per-byte dirty mask.  A write to another granule first writes the
// buffer back (its dirty bytes only); a read whose range overlaps a
// dirty byte also writes it back first, so the in-order raw port makes
// every read see every earlier write.  The buffer is written back when
// the controller has been idle for idle_wb_cycles (so the memory, and
// the sim harness's tohost / signature taps on it, catch up), and at the
// latest max_dirty_cycles after it became dirty: a granule written over
// and over (a store spin loop) would otherwise never reach the memory.
//
// Write responses (B) are sent when the write enters the buffer.
// Reads and writes are kept in one queue, in arrival order.
// ================================================================

export

Bits_per_Raw_Mem_Addr,
Raw_Mem_Addr,

Bits_per_Raw_Mem_Word,
Raw_Mem_Word,

Raw_Mem_Line,
Raw_Mem_Req (..),
Raw_Mem_Chunk,

Mem_Controller_IFC (..),
mkMem_Controller,

status_mem_controller_terminated;

// ================================================================
// BSV library imports

import  Vector       :: *;
import  FIFOF        :: *;
import  SpecialFIFOs :: *;
import  GetPut       :: *;
import  ClientServer :: *;
import  ConfigReg    :: *;

// ----------------
// BSV additional libs

import Cur_Cycle  :: *;
import GetPut_Aux :: *;
import Semi_FIFOF :: *;
import ByteLane   :: *;

// ================================================================
// Project imports

import Fabric_Defs :: *;
import SoC_Map     :: *;
import AXI4_Types  :: *;

// ================================================================
// Raw memory port

typedef Bit #(19) Raw_Mem_Line;     // 64-byte line number (32 MB)

typedef struct {
   Bool          write;
   Raw_Mem_Line  line;
   Bit #(2)      b_first;   // read: 16-byte bursts b_first .. b_last of the line
   Bit #(2)      b_last;
   Bit #(256)    wdata;     // write: half b_first[1] of the line
   Bit #(32)     wbe;
   } Raw_Mem_Req
deriving (Bits, FShow);

typedef Bit #(64) Raw_Mem_Chunk;

// The 256-bit word view (= one half line) is kept for the Sdram_Controller
// unit test's adapter.
typedef 256  Bits_per_Raw_Mem_Word;
typedef Bit #(Bits_per_Raw_Mem_Word)  Raw_Mem_Word;

typedef 64  Bits_per_Raw_Mem_Addr;
typedef Bit #(Bits_per_Raw_Mem_Addr)  Raw_Mem_Addr;

// Idle cycles before the write buffer is written back
Integer idle_wb_cycles = 32;
// Cycles a buffer may stay dirty (rg_age saturates here) before it is written back
Integer max_dirty_cycles = 255;

// ================================================================

function Bool fn_addr_is_aligned (Fabric_Addr  addr, AXI4_Size  size);
   Bool is_aligned = (   (size == axsize_1)
		      || ((size == axsize_2)    && (addr [0]   == 1'h0))
		      || ((size == axsize_4)    && (addr [1:0] == 2'h0))
		      || ((size == axsize_8)    && (addr [2:0] == 3'h0)));
   return is_aligned;
endfunction

// ================================================================
// Catch-all status

Integer status_mem_controller_terminated = 1;

// ================================================================
// Interface

interface Mem_Controller_IFC;
   // Reset
   interface Server #(Bit #(0), Bit #(0)) server_reset;

   // set_addr_map should be called after this module's reset
   method Action set_addr_map (Fabric_Addr addr_base, Fabric_Addr addr_lim);

   // Main Fabric Reqs/Rsps
   interface AXI4_Slave_IFC #(Wd_Id, Wd_Addr, Wd_Data, Wd_User) slave;

   // To raw memory (outside the SoC)
   interface Client #(Raw_Mem_Req, Raw_Mem_Chunk)  to_raw_mem;

   // Catch-all status; return-value can identify the origin (0 = none)
   (* always_ready *)
   method Bit #(8) status;

`ifdef WATCH_TOHOST
   // For ISA tests: watch memory writes to <tohost> addr
   method Action set_watch_tohost (Bool watch_tohost, Fabric_Addr tohost_addr);
`endif
endinterface

// ================================================================
// One queue for both channels.  A write burst becomes one Req per beat
// (wr_last on the final one, which carries the B response).

typedef enum { REQ_OP_RD, REQ_OP_WR } Req_Op
deriving (Bits, Eq, FShow);

typedef struct {Req_Op                     req_op;
		Fabric_Id                  id;
		Fabric_Addr                addr;
		AXI4_Len                   len;       // read: beats - 1
		Bit #(Wd_User)             user;
		Bit #(TDiv #(Wd_Data, 8))  wstrb;
		Fabric_Data                data;
		Bool                       wr_last;
		// Computed when the request is merged (registered in f_reqs), not
		// from f_reqs.first in rule guards (E15: 22 ns path)
		Bool                       addr_ok;
		Bit #(25)                  off;       // byte offset from addr_base
   } Req
deriving (Bits, FShow);

// A raw read in flight: which of its chunks go out on R
typedef struct {Fabric_Id       id;
		Bit #(Wd_User)  user;
		Bool            skip_first;  // the burst's first dword precedes the request
		Bit #(4)        n_chunks;
		Bit #(4)        n_beats;
		Bool            is_final;    // last raw read of the AR: rlast on its last beat
   } Rd_Pend
deriving (Bits, FShow);

// ================================================================

(* synthesize *)
module mkMem_Controller (Mem_Controller_IFC);

   // verbosity 0: quiet
   // verbosity 1: reset, initialized
   // verbosity 2: reads, writes
   Reg #(Bit #(4)) cfg_verbosity <- mkConfigReg (0);

   Reg #(Bool)        rg_ready     <- mkReg (False);
   // E15: ConfigRegs: the merge rules read these (set once by set_addr_map at init)
   Reg #(Fabric_Addr) rg_addr_base <- mkConfigRegU;
   Reg #(Fabric_Addr) rg_addr_lim  <- mkConfigRegU;

   FIFOF #(Bit #(0)) f_reset_reqs <- mkFIFOF;
   FIFOF #(Bit #(0)) f_reset_rsps <- mkFIFOF;

   // Communication with fabric
   AXI4_Slave_Xactor_IFC #(Wd_Id, Wd_Addr, Wd_Data, Wd_User) slave_xactor <- mkAXI4_Slave_Xactor;

   // Requests merged from the (WrA, WrD) and RdA channels
   FIFOF #(Req) f_reqs <- mkPipelineFIFOF;

   // Write burst in progress (beats after the first)
   Reg #(AXI4_Len)       rg_wr_left <- mkReg (0);
   Reg #(Fabric_Addr)    rg_wr_addr <- mkRegU;
   Reg #(Fabric_Id)      rg_wr_id   <- mkRegU;
   Reg #(Bit #(Wd_User)) rg_wr_user <- mkRegU;
   Reg #(Bool)           rg_wr_ok   <- mkRegU;

   // Raw memory
   // Bypass: the Sdram side is registered (its f_reqs / f_rsps), so these add no cycle
   FIFOF #(Raw_Mem_Req)   f_raw_reqs <- mkBypassFIFOF;
   FIFOF #(Raw_Mem_Chunk) f_raw_rsps <- mkBypassFIFOF;
   FIFOF #(Rd_Pend)       f_rd_pend  <- mkSizedFIFOF (2);

   // Read at the head of f_reqs, split at a line boundary: rest still to issue
   Reg #(Bool)       rg_seg_on   <- mkReg (False);
   Reg #(Bit #(25))  rg_seg_off  <- mkRegU;
   Reg #(Bit #(9))   rg_seg_left <- mkRegU;     // beats

   // Streaming R: chunk / forwarded-beat counters of f_rd_pend.first
   Reg #(Bit #(4))   rg_ck  <- mkReg (0);
   Reg #(Bit #(4))   rg_fwd <- mkReg (0);

   // Bad-address read: beats answered so far
   Reg #(AXI4_Len)   rg_err_beat <- mkReg (0);

   // Write buffer: one 32-byte granule (offset >> 5), per-byte dirty mask
   Reg #(Bit #(20))  rg_buf_gran  <- mkRegU;
   Reg #(Bit #(256)) rg_buf_data  <- mkRegU;
   Reg #(Bit #(32))  rg_buf_dirty <- mkReg (0);

   Reg #(Bit #(6))   rg_idle <- mkReg (0);
   Reg #(Bit #(8))   rg_age  <- mkConfigReg (0);   // cycles dirty (saturating); ConfigReg: no rule order vs rg_buf_dirty writers

`ifdef WATCH_TOHOST
   // Ad hoc ISA-test simulation support: watch <tohost> and stop on non-zero write.
   Reg #(Bool)        rg_watch_tohost <- mkReg (False);
   Reg #(Fabric_Addr) rg_tohost_addr  <- mkReg ('h_8000_1000);
`endif

   // Catch-all status
   Reg #(Bit #(8)) rg_status <- mkReg (0);

   // ================================================================
   // BEHAVIOR

   // ----------------------------------------------------------------
   // Reset (boot only: raw reads still in flight are not drained)

   function Action fa_reset_actions;
      action
	 slave_xactor.reset;
	 f_reqs.clear;
	 f_raw_reqs.clear;
	 f_raw_rsps.clear;
	 f_rd_pend.clear;
	 rg_wr_left   <= 0;
	 rg_seg_on    <= False;
	 rg_ck        <= 0;
	 rg_fwd       <= 0;
	 rg_err_beat  <= 0;
	 rg_buf_dirty <= 0;
	 rg_status    <= 0;
      endaction
   endfunction

   rule rl_power_on_reset (! rg_ready);
      fa_reset_actions ();
      rg_ready <= True;
   endrule

   rule rl_external_reset (rg_ready);
      f_reset_reqs.deq;
      fa_reset_actions ();
      f_reset_rsps.enq (?);
      if (cfg_verbosity > 1)
	 $display ("%0d: Mem_Controller.rl_external_reset", cur_cycle);
   endrule

   // ----------------------------------------------------------------
   // Merge requests into a single queue, prioritizing reads over writes

   function Bool fn_in_range (Fabric_Addr a);
      return ((rg_addr_base <= a) && (a < rg_addr_lim));
   endfunction

   rule rl_merge_rd_req;
      let rda <- pop_o (slave_xactor.o_rd_addr);
      // A burst must be INCR of 64-bit beats; its last byte must be in range too
      Fabric_Addr last = rda.araddr + ((zeroExtend (rda.arlen) << 3) | 7);
      Bool ok = (   fn_addr_is_aligned (rda.araddr, rda.arsize)
		 && fn_in_range (rda.araddr)
		 && (   (rda.arlen == 0)
		     || ((rda.arsize == axsize_8) && (rda.arburst == axburst_incr) && fn_in_range (last))));
      let req = Req {req_op:   REQ_OP_RD,
		     id:       rda.arid,
		     addr:     rda.araddr,
		     len:      rda.arlen,
		     user:     rda.aruser,
		     wstrb:    ?,
		     data:     ?,
		     wr_last:  ?,
		     addr_ok:  ok,
		     off:      truncate (rda.araddr - rg_addr_base)};
      f_reqs.enq (req);

      if (cfg_verbosity > 2)
	 $display ("%0d: Mem_Controller.rl_merge_rd_req ", cur_cycle, fshow (rda));
   endrule

   (* descending_urgency = "rl_merge_rd_req, rl_merge_wr_req" *)
   rule rl_merge_wr_req (rg_wr_left == 0);
      let wra <- pop_o (slave_xactor.o_wr_addr);
      let wrd <- pop_o (slave_xactor.o_wr_data);
      Fabric_Addr last = wra.awaddr + ((zeroExtend (wra.awlen) << 3) | 7);
      Bool ok = (   fn_addr_is_aligned (wra.awaddr, wra.awsize)
		 && fn_in_range (wra.awaddr)
		 && (   (wra.awlen == 0)
		     || ((wra.awsize == axsize_8) && (wra.awburst == axburst_incr) && fn_in_range (last))));
      let req = Req {req_op:   REQ_OP_WR,
		     id:       wra.awid,
		     addr:     wra.awaddr,
		     len:      0,
		     user:     wra.awuser,
		     wstrb:    wrd.wstrb,
		     data:     wrd.wdata,
		     wr_last:  (wra.awlen == 0),
		     addr_ok:  ok,
		     off:      truncate (wra.awaddr - rg_addr_base)};
      f_reqs.enq (req);
      rg_wr_left <= wra.awlen;
      rg_wr_addr <= wra.awaddr + 8;
      rg_wr_id   <= wra.awid;
      rg_wr_user <= wra.awuser;
      rg_wr_ok   <= ok;

      if (cfg_verbosity > 2)
	 $display ("%0d: Mem_Controller.rl_merge_wr_req ", cur_cycle, fshow (wra), " ", fshow (wrd));
   endrule

   // Remaining beats of a write burst
   (* descending_urgency = "rl_merge_rd_req, rl_merge_wr_beat" *)
   rule rl_merge_wr_beat (rg_wr_left != 0);
      let wrd <- pop_o (slave_xactor.o_wr_data);
      let req = Req {req_op:   REQ_OP_WR,
		     id:       rg_wr_id,
		     addr:     rg_wr_addr,
		     len:      0,
		     user:     rg_wr_user,
		     wstrb:    wrd.wstrb,
		     data:     wrd.wdata,
		     wr_last:  (rg_wr_left == 1),
		     addr_ok:  rg_wr_ok,
		     off:      truncate (rg_wr_addr - rg_addr_base)};
      f_reqs.enq (req);
      rg_wr_left <= rg_wr_left - 1;
      rg_wr_addr <= rg_wr_addr + 8;
   endrule

   // ----------------------------------------------------------------
   // Write buffer

   let head      = f_reqs.first;
   Bit #(20) head_gran = truncate (head.off >> 5);

   function Action fa_writeback;
      action
	 f_raw_reqs.enq (Raw_Mem_Req {write:   True,
				      line:    truncate (rg_buf_gran >> 1),
				      b_first: { rg_buf_gran [0], 1'b0 },
				      b_last:  { rg_buf_gran [0], 1'b1 },
				      wdata:   rg_buf_data,
				      wbe:     rg_buf_dirty});
	 rg_buf_dirty <= 0;
	 if (cfg_verbosity > 2)
	    $display ("%0d: Mem_Controller: writeback granule 0x%0h dirty %h",
		      cur_cycle, rg_buf_gran, rg_buf_dirty);
      endaction
   endfunction

   rule rl_idle_count;
      if (f_reqs.notEmpty) rg_idle <= 0;
      else if (rg_idle != '1) rg_idle <= rg_idle + 1;
   endrule

   rule rl_age_count;
      if (rg_buf_dirty == 0) rg_age <= 0;
      else if (rg_age != '1) rg_age <= rg_age + 1;
   endrule

   Bool age_out = (rg_age >= fromInteger (max_dirty_cycles));

   rule rl_writeback_idle (   rg_ready
			   && (rg_buf_dirty != 0)
			   && (   ((! f_reqs.notEmpty) && (rg_idle >= fromInteger (idle_wb_cycles)))
			       || age_out));
      fa_writeback;
   endrule

   // ----------------------------------------------------------------
   // Writes: into the buffer (taken over without reading memory when
   // nothing in it is dirty); another granule first writes it back.

   Bool wr_head    = rg_ready && f_reqs.notEmpty && (head.req_op == REQ_OP_WR) && head.addr_ok;
   Bool buf_match  = (rg_buf_gran == head_gran);

   rule rl_wr_evict (wr_head && (! buf_match) && (rg_buf_dirty != 0));
      fa_writeback;
   endrule

   // (an aged-out buffer is written back first, rl_writeback_idle)
   rule rl_wr (wr_head && ((buf_match && (! age_out)) || (rg_buf_dirty == 0)));
      Bit #(2) lane = head.off [4:3];
      Vector #(4, Bit #(64)) v = unpack (rg_buf_data);
      Bit #(64) mask = fn_strobe_to_mask (head.wstrb);
      v [lane] = (v [lane] & (~ mask)) | (head.data & mask);
      rg_buf_data <= pack (v);
      Bit #(32) be = zeroExtend (head.wstrb);
      rg_buf_dirty <= (buf_match ? rg_buf_dirty : 0) | (be << { lane, 3'b0 });
      rg_buf_gran  <= head_gran;
      f_reqs.deq;
      if (head.wr_last)
	 slave_xactor.i_wr_resp.enq (AXI4_Wr_Resp {bid:   head.id,
						   bresp: axi4_resp_okay,
						   buser: head.user});
      if (cfg_verbosity > 1)
	 $display ("%0d: Mem_Controller.rl_wr: ", cur_cycle, fshow (head));

`ifdef WATCH_TOHOST
      // For simulation testing of riscv-tests/isa only:
      if (rg_watch_tohost && (head.addr == rg_tohost_addr) && (head.data != 0)) begin
	 let exit_value = (head.data >> 1);
	 if (exit_value == 0)
	    $display ("PASS");
	 else
	    $display ("FAIL %0d", exit_value);
	 rg_status <= fromInteger (status_mem_controller_terminated);
      end
`endif
   endrule

   // ----------------------------------------------------------------
   // Reads: one raw read per line touched; a dirty buffer byte in the
   // range is written back first.

   Bool rd_head = rg_ready && f_reqs.notEmpty && (head.req_op == REQ_OP_RD) && head.addr_ok;
   Bit #(20) rd_gran_hi = truncate ((head.off + (zeroExtend (head.len) << 3)) >> 5);
   Bool rd_overlap = (   (rg_buf_dirty != 0)
		      && (head_gran <= rg_buf_gran) && (rg_buf_gran <= rd_gran_hi));

   rule rl_rd_evict (rd_head && (! rg_seg_on) && rd_overlap);
      fa_writeback;
   endrule

   (* descending_urgency = "rl_writeback_idle, rl_wr_evict, rl_rd_evict, rl_rd_issue" *)
   rule rl_rd_issue (rd_head && (rg_seg_on || (! rd_overlap)));
      Bit #(25) o    = rg_seg_on ? rg_seg_off  : head.off;
      Bit #(9)  left = rg_seg_on ? rg_seg_left : (zeroExtend (head.len) + 1);
      Bit #(3)  d0   = o [5:3];                       // first dword in the line
      Bit #(4)  room = 8 - zeroExtend (d0);
      Bit #(4)  n    = (left < zeroExtend (room)) ? truncate (left) : room;
      Bit #(3)  d1   = truncate (zeroExtend (d0) + n - 1);
      Bit #(2)  b0   = d0 [2:1];
      Bit #(2)  b1   = d1 [2:1];
      Bool      fin  = (zeroExtend (n) == left);
      f_raw_reqs.enq (Raw_Mem_Req {write: False, line: truncate (o >> 6),
				   b_first: b0, b_last: b1, wdata: ?, wbe: ?});
      f_rd_pend.enq (Rd_Pend {id:         head.id,
			      user:       head.user,
			      skip_first: (d0 [0] == 1),
			      n_chunks:   (zeroExtend (b1 - b0) + 1) << 1,
			      n_beats:    n,
			      is_final:   fin});
      if (fin) begin
	 f_reqs.deq;
	 rg_seg_on <= False;
      end
      else begin
	 rg_seg_on   <= True;
	 rg_seg_off  <= o + (zeroExtend (n) << 3);
	 rg_seg_left <= left - zeroExtend (n);
      end
      if (cfg_verbosity > 1)
	 $display ("%0d: Mem_Controller.rl_rd_issue: off 0x%0h beats %0d (line 0x%0h bursts %0d..%0d)",
		   cur_cycle, o, n, o >> 6, b0, b1);
   endrule

   // Raw chunks -> R beats (dwords outside the request are dropped)
   (* descending_urgency = "rl_rd_stream, rl_invalid_rd_address" *)
   rule rl_rd_stream;
      let c = f_raw_rsps.first;
      let p = f_rd_pend.first;
      Bool fwd = (! (p.skip_first && (rg_ck == 0))) && (rg_fwd < p.n_beats);
      if (fwd)
	 slave_xactor.i_rd_data.enq (AXI4_Rd_Data {rid:   p.id,
						   rdata: c,
						   rresp: axi4_resp_okay,
						   rlast: p.is_final && (rg_fwd + 1 == p.n_beats),
						   ruser: p.user});
      f_raw_rsps.deq;
      if (rg_ck + 1 == p.n_chunks) begin
	 f_rd_pend.deq;
	 rg_ck  <= 0;
	 rg_fwd <= 0;
      end
      else begin
	 rg_ck <= rg_ck + 1;
	 if (fwd) rg_fwd <= rg_fwd + 1;
      end
   endrule

   // ================================================================
   // Invalid address: SLVERR (every read beat)

   // Only once earlier reads have streamed out (R stays in order)
   rule rl_invalid_rd_address (   rg_ready && f_reqs.notEmpty
			       && (! head.addr_ok)
			       && (head.req_op == REQ_OP_RD)
			       && (! f_rd_pend.notEmpty));
      Bool last = (rg_err_beat == head.len);
      slave_xactor.i_rd_data.enq (AXI4_Rd_Data {rid:   head.id,
						rdata: zeroExtend (head.addr),   // for debugging only
						rresp: axi4_resp_slverr,
						rlast: last,
						ruser: head.user});
      if (last) begin
	 f_reqs.deq;
	 rg_err_beat <= 0;
	 $display ("%0d: ERROR: Mem_Controller: bad read ", cur_cycle, fshow (head));
	 $display ("        rg_addr_base 0x%0h  rg_addr_lim 0x%0h", rg_addr_base, rg_addr_lim);
      end
      else
	 rg_err_beat <= rg_err_beat + 1;
   endrule

   rule rl_invalid_wr_address (   rg_ready && f_reqs.notEmpty
			       && (! head.addr_ok)
			       && (head.req_op == REQ_OP_WR));
      f_reqs.deq;
      if (head.wr_last) begin
	 slave_xactor.i_wr_resp.enq (AXI4_Wr_Resp {bid:   head.id,
						   bresp: axi4_resp_slverr,
						   buser: head.user});
	 $display ("%0d: ERROR: Mem_Controller: bad write ", cur_cycle, fshow (head));
	 $display ("        rg_addr_base 0x%0h  rg_addr_lim 0x%0h", rg_addr_base, rg_addr_lim);
      end
   endrule

   // ================================================================
   // INTERFACE

   // Reset
   interface  server_reset = toGPServer (f_reset_reqs, f_reset_rsps);

   // set_addr_map should be called after this module's reset
   method Action  set_addr_map (Fabric_Addr addr_base, Fabric_Addr addr_lim) if (rg_ready);
      rg_addr_base <= addr_base;
      rg_addr_lim  <= addr_lim;
      $display ("%0d: Mem_Controller.set_addr_map: addr_base 0x%0h addr_lim 0x%0h",
		cur_cycle, addr_base, addr_lim);
   endmethod

   // Main Fabric Reqs/Rsps
   interface  slave = slave_xactor.axi_side;

   // To raw memory (outside the SoC)
   interface  to_raw_mem = toGPClient (f_raw_reqs, f_raw_rsps);

   // Catch-all status; return-value can identify the origin (0 = none)
   method Bit #(8) status;
      return rg_status;
   endmethod

`ifdef WATCH_TOHOST
   // For ISA tests: watch memory writes to <tohost> addr
   method Action set_watch_tohost (Bool watch_tohost, Fabric_Addr tohost_addr);
      rg_watch_tohost <= watch_tohost;
      rg_tohost_addr  <= tohost_addr;
   endmethod
`endif
endmodule

// ================================================================

endpackage
