// Copyright (c) 2016-2020 Bluespec, Inc. All Rights Reserved

package Mem_Controller;

// ================================================================
// This module is a slave on the interconnect Fabric.
//
// On the back side of the Mem_Controller is a ``raw'' memory
// interface, a simple, wide, R/W interface,
// which is connected to real memory in hardware (BRAM, DRAM, ...)
// and to a model thereof in simulation.
//
// The raw mem interface data width is typically one or two cache lines.
// Note: raw mem write requests are 'fire and forget'; there is no ack

// ----------------
// This slave IP can be attached to fabrics with 32b- or 64b-wide data channels.
//    (NOTE: this is the width of the fabric, which can be chosen
//      independently of the native width of a CPU master on the
//      fabric (such as RV32/RV64 for a RISC-V CPU).
// When attached to 32b-wide fabric, 64-bit locations must be
// read/written in two 32b transaction, once for the lower 32b and
// once for the upper 32b.

// When fabric data is 64b wide, fabric addresses must be 8B-aligned
//    - Reads always return 64b data
//    - Write data should be 64b wide with an 8b byte-strobe indicating
//        which bytes are to be written. Strobes should be for aligned
//        1B, 2B, 4B or 8B chunks.

// When fabric data is 32b wide, fabric addresses must be 4B-aligned
//    - Reads always return 32b data
//    - Write data should be 32b wide with a 4b byte-strobe indicating
//        which bytes are to be written. Strobes should for aligned
//        1B, 2B, or 4B chunks.

// Some of the 'truncate()'s and 'zeroExtend()'s below are no-ops but
// necessary to satisfy type-checking.
// ================================================================

export

Bits_per_Raw_Mem_Addr,
Raw_Mem_Addr,

Bits_per_Raw_Mem_Word,
Raw_Mem_Word,

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
import  Memory       :: *;
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
// Raw mem data width:    256 (bits/ 32 x Byte/ 8 x Word32/ 4 x Word64)
// Raw mem address width: 64  (arbitrarily chosen generously large)

typedef 256  Bits_per_Raw_Mem_Word;
typedef Bit #(Bits_per_Raw_Mem_Word)  Raw_Mem_Word;

typedef 64  Bits_per_Raw_Mem_Addr;
typedef Bit #(Bits_per_Raw_Mem_Addr)  Raw_Mem_Addr;

// ----------------
// Views of raw mem word as bytes, word32s and word64s
// Example values on the right based on 256b raw mem data width
typedef TDiv #(Bits_per_Raw_Mem_Word,  8)              Bytes_per_Raw_Mem_Word;             // 32 bytes
Integer bytes_per_raw_mem_word = valueOf (Bytes_per_Raw_Mem_Word);

// # of addr lsbs to index a byte in a Raw_Mem_Word
typedef TLog #(Bytes_per_Raw_Mem_Word)                 Bits_per_Byte_in_Raw_Mem_Word;      //  5
Integer bits_per_byte_in_raw_mem_word = valueOf (Bits_per_Byte_in_Raw_Mem_Word);
Integer hi_byte_in_raw_mem_word = bits_per_byte_in_raw_mem_word - 1;                       //  4

typedef TDiv #(Bits_per_Raw_Mem_Word, 32)              Word32s_per_Raw_Mem_Word;           //  8 x 32b words
Integer word32s_per_raw_mem_word = valueOf (Word32s_per_Raw_Mem_Word);
// # of addr lsbs to index a Word32 in a Raw_Mem_Word seen as a vector of Word32s
typedef TLog #(Word32s_per_Raw_Mem_Word)               Bits_per_Word32_in_Raw_Mem_Word;    //  3
// Type of index of a Word32 in a Raw_Mem_Word seen as a vector of Word32s
typedef Bit #(Bits_per_Word32_in_Raw_Mem_Word)         Word32_in_Raw_Mem_Word;

typedef TDiv #(Bits_per_Raw_Mem_Word, 64)              Word64s_per_Raw_Mem_Word;           //  4 x 64b words
Integer word64s_per_raw_mem_word = valueOf (Word64s_per_Raw_Mem_Word);
// # of addr lsbs to index a Word64 in a Raw_Mem_Word seen as a vector of Word64s
typedef TLog #(Word64s_per_Raw_Mem_Word)               Bits_per_Word64_in_Raw_Mem_Word;    //  2
// Type of index of a Word64 in a Raw_Mem_Word seen as a vector of Word64s
typedef Bit #(Bits_per_Word64_in_Raw_Mem_Word)         Word64_in_Raw_Mem_Word;

typedef TDiv #(Bytes_per_Raw_Mem_Word, Bytes_per_Fabric_Data)  Fabric_Data_per_Raw_Mem_Word;

// Index of bit that selects a fabric data word in an address
`ifdef FABRIC32
Integer  lo_fabric_data = 2;
`endif

`ifdef FABRIC64
Integer  lo_fabric_data = 3;
`endif

// ================================================================

function Bool fn_addr_is_aligned (Fabric_Addr  addr, AXI4_Size  size);
   Bool is_aligned = (   (size == axsize_1)
		      || ((size == axsize_2)    && (addr [0]   == 1'h0))
		      || ((size == axsize_4)    && (addr [1:0] == 2'h0))
		      || ((size == axsize_8)    && (addr [2:0] == 3'h0))
		      || ((size == axsize_16)   && (addr [3:0] == 4'h0))
		      || ((size == axsize_32)   && (addr [4:0] == 5'h0))
		      || ((size == axsize_64)   && (addr [5:0] == 6'h0))
		      || ((size == axsize_128)  && (addr [6:0] == 7'h0)));
   return is_aligned;
endfunction

function Bool fn_addr_is_in_range (Fabric_Addr addr_base, Fabric_Addr addr, Fabric_Addr addr_lim);
   // Note: 'in_range' is redundant if the fabric only delivers
   // relevant addresses to this module so this is just a bit of
   // defensive programming.

   return ((addr_base <= addr) && (addr < addr_lim));
endfunction

function Bool fn_addr_is_ok (Fabric_Addr addr_base, Fabric_Addr addr, Fabric_Addr addr_lim, AXI4_Size  size);
   return (   fn_addr_is_aligned (addr, size)
	   && fn_addr_is_in_range (addr_base, addr, addr_lim));
endfunction

// Compute raw mem addr that holds a given fabric addr
function Raw_Mem_Addr fn_addr_to_raw_mem_addr (Fabric_Addr addr);
   Fabric_Addr a1 = addr >> log2 (bytes_per_raw_mem_word);
   return extend (a1);
endfunction

// Compute # of raw mem words from base to lim fabric addrs
function Raw_Mem_Addr fn_raw_mem_words_per_mem (Fabric_Addr base, Fabric_Addr lim);
   return fn_addr_to_raw_mem_addr (lim - base);
endfunction

// ================================================================
// Local constants and types

// Module state
typedef enum {STATE_POWER_ON_RESET,
`ifdef INCLUDE_INITIAL_MEMZERO
	      STATE_ZEROING_MEM,           // while zero-ing out memory
`endif
	      STATE_RESET_RELOAD_CACHE,    // on reset, start reload on reset
	      STATE_RELOADING,             // while reloading the raw-mem word cache
	      STATE_READY                  // while handling requests
   } State
deriving (Bits, Eq, FShow);

// ================================================================
// Catch-all status

Integer status_mem_controller_terminated = 1;

// Option B line-half states
Bit #(2) hs_invalid = 0;
Bit #(2) hs_pending = 1;
Bit #(2) hs_valid   = 2;

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
   interface MemoryClient #(Bits_per_Raw_Mem_Addr, Bits_per_Raw_Mem_Word)  to_raw_mem;

   // Catch-all status; return-value can identify the origin (0 = none)
   (* always_ready *)
   method Bit #(8) status;

`ifdef WATCH_TOHOST
   // For ISA tests: watch memory writes to <tohost> addr
   method Action set_watch_tohost (Bool watch_tohost, Fabric_Addr tohost_addr);
`endif
endinterface

// ================================================================
// AXI4 has independent read and write channels and does not specify
// which one should be prioritized if requests are available on both
// channels.  We merge them into a single queue.

typedef enum { REQ_OP_RD, REQ_OP_WR } Req_Op
deriving (Bits, Eq, FShow);

typedef struct {Req_Op                     req_op;

		// AW and AR channel info
		Fabric_Id                  id;
		Fabric_Addr                addr;
		AXI4_Len                   len;
		AXI4_Size                  size;
		AXI4_Burst                 burst;
		AXI4_Lock                  lock;
		AXI4_Cache                 cache;
		AXI4_Prot                  prot;
		AXI4_QoS                   qos;
		AXI4_Region                region;
		Bit #(Wd_User)             user;

		// Write data info
		Bit #(TDiv #(Wd_Data, 8))  wstrb;
		Fabric_Data                data;

		// E15: computed when the request is merged (registered in f_reqs),
		// not from f_reqs.first in every rule guard (addr - base subtract and
		// the range compares were a 22 ns critical path)
		Bool                       addr_ok;
		Raw_Mem_Addr               raw_addr;
   } Req
deriving (Bits, FShow);

// ================================================================

(* synthesize *)
module mkMem_Controller (Mem_Controller_IFC);

   // verbosity 0: quiet
   // verbosity 1: reset, initialized
   // verbosity 2: reads, writes
   // verbosity 3: more detail of local raw_mem interactions
   Reg #(Bit #(4)) cfg_verbosity <- mkConfigReg (0);

   Reg #(State)       rg_state     <- mkReg (STATE_POWER_ON_RESET);
   // E15: ConfigRegs: the merge rules read these (set once by set_addr_map at init)
   Reg #(Fabric_Addr) rg_addr_base <- mkConfigRegU;
   Reg #(Fabric_Addr) rg_addr_lim  <- mkConfigRegU;

   FIFOF #(Bit #(0)) f_reset_reqs <- mkFIFOF;
   FIFOF #(Bit #(0)) f_reset_rsps <- mkFIFOF;

   // Communication with fabric
   AXI4_Slave_Xactor_IFC #(Wd_Id, Wd_Addr, Wd_Data, Wd_User) slave_xactor <- mkAXI4_Slave_Xactor;

   // Requests merged from the (WrA, WrD) and RdA channels
   FIFOF #(Req) f_reqs <- mkPipelineFIFOF;

   // FIFOFs for requests/responses to raw memory
   FIFOF #(MemoryRequest  #(Bits_per_Raw_Mem_Addr, Bits_per_Raw_Mem_Word))
       f_raw_mem_reqs <- mkPipelineFIFOF;
   FIFOF #(MemoryResponse #(Bits_per_Raw_Mem_Word))
       f_raw_mem_rsps <- mkPipelineFIFOF;

   // Option B (mem-optB): a one-LINE cache.  A line is two raw_mem_words
   // (2 x 256 b = the 64-byte L1 line).  On a line switch the half that
   // holds the requested beat is read first and the other half right
   // behind it (both queued in Sdram_Controller), so the second half of
   // an L1 refill no longer pays a full serialized reload.  Each half has
   // its own state and dirty bit; only dirty halves are written back.
   Reg #(Raw_Mem_Addr)        rg_line     <- mkReg (0);     // raw_mem_addr >> 1
   Reg #(Raw_Mem_Word)        rg_half0    <- mkRegU;
   Reg #(Raw_Mem_Word)        rg_half1    <- mkRegU;
   Vector #(2, Reg #(Bit #(2))) rg_hstat  <- replicateM (mkReg (hs_invalid));
   Reg #(Bool)                rg_dirty0   <- mkReg (False);
   Reg #(Bool)                rg_dirty1   <- mkReg (False);
   Reg #(Maybe #(Bit #(1)))   rg_issue_other <- mkReg (tagged Invalid);
   FIFOF #(Bit #(1))          f_rd_order  <- mkSizedFIFOF (2);  // half of each raw read in flight

`ifdef INCLUDE_INITIAL_MEMZERO
   Reg #(Bool)          rg_cached_clean        <- mkRegU;
   Reg #(Raw_Mem_Addr)  rg_cached_raw_mem_addr <- mkRegU;
   Reg #(Raw_Mem_Word)  rg_cached_raw_mem_word <- mkRegU;
`endif

`ifdef WATCH_TOHOST
   // Ad hoc ISA-test simulation support: watch <tohost> and stop on non-zero write.
   // The default tohost_addr here is fragile (may change on recompilation of tests).
   // Proper value can be provided with 'set_watch_tohost' method from symbol table
   Reg #(Bool)        rg_watch_tohost <- mkReg (False);
   Reg #(Fabric_Addr) rg_tohost_addr  <- mkReg ('h_8000_1000);
`endif

   // Catch-all status
   Reg #(Bit #(8)) rg_status <- mkReg (0);

   // ================================================================
   // BEHAVIOR

   // ----------------------------------------------------------------
   // Reset

   function Action fa_reset_actions;
      action
	 slave_xactor.reset;
	 f_raw_mem_reqs.clear;
	 f_raw_mem_rsps.clear;
	 rg_status <= 0;
      endaction
   endfunction

   rule rl_power_on_reset (rg_state == STATE_POWER_ON_RESET);
      if (cfg_verbosity > 1)
	 $display ("%0d: Mem_Controller.rl_power_on_reset", cur_cycle);
      fa_reset_actions ();
      rg_state <= STATE_RESET_RELOAD_CACHE;
   endrule

   rule rl_external_reset (rg_state == STATE_READY);
      if (cfg_verbosity > 1)
	 $display ("%0d: Mem_Controller.rl_external_reset => STATE_RESET_RELOAD_CACHE", cur_cycle);

      f_reset_reqs.deq;
      fa_reset_actions ();
      rg_state <= STATE_RESET_RELOAD_CACHE;
      f_reset_rsps.enq (?);
   endrule

   // On reset the line cache starts empty (both halves invalid).
   rule rl_reset_reload_cache (rg_state == STATE_RESET_RELOAD_CACHE);
      rg_hstat [0]   <= hs_invalid;
      rg_hstat [1]   <= hs_invalid;
      rg_dirty0      <= False;
      rg_dirty1      <= False;
      rg_issue_other <= tagged Invalid;
      f_rd_order.clear;
      rg_state       <= STATE_READY;
   endrule

   // ----------------------------------------------------------------
   // Merge requests into a single queue, prioritizing reads over writes

   rule rl_merge_rd_req;
      let rda <- pop_o (slave_xactor.o_rd_addr);
      let req = Req {req_op:     REQ_OP_RD,
		     id:         rda.arid,
		     addr:       rda.araddr,
		     len:        rda.arlen,
		     size:       rda.arsize,
		     burst:      rda.arburst,
		     lock:       rda.arlock,
		     cache:      rda.arcache,
		     prot:       rda.arprot,
		     qos:        rda.arqos,
		     region:     rda.arregion,
		     user:       rda.aruser,
		     wstrb:      ?,
		     data:       ?,
		     addr_ok:    fn_addr_is_ok (rg_addr_base, rda.araddr, rg_addr_lim, rda.arsize),
		     raw_addr:   fn_addr_to_raw_mem_addr (rda.araddr - rg_addr_base)};
      f_reqs.enq (req);

      if (cfg_verbosity > 2) begin
	 $display ("%0d: Mem_Controller.rl_merge_rd_req", cur_cycle);
	 $display ("        ", fshow (rda));
      end
   endrule

   (* descending_urgency = "rl_merge_rd_req, rl_merge_wr_req" *)
   rule rl_merge_wr_req;
      let wra <- pop_o (slave_xactor.o_wr_addr);
      let wrd <- pop_o (slave_xactor.o_wr_data);
      let req = Req {req_op:     REQ_OP_WR,
		     id:         wra.awid,
		     addr:       wra.awaddr,
		     len:        wra.awlen,
		     size:       wra.awsize,
		     burst:      wra.awburst,
		     lock:       wra.awlock,
		     cache:      wra.awcache,
		     prot:       wra.awprot,
		     qos:        wra.awqos,
		     region:     wra.awregion,
		     user:       wra.awuser,
		     wstrb:      wrd.wstrb,
		     data:       wrd.wdata,
		     addr_ok:    fn_addr_is_ok (rg_addr_base, wra.awaddr, rg_addr_lim, wra.awsize),
		     raw_addr:   fn_addr_to_raw_mem_addr (wra.awaddr - rg_addr_base)};
      f_reqs.enq (req);

      if (cfg_verbosity > 2) begin
	 $display ("%0d: Mem_Controller.rl_merge_wr_req", cur_cycle);
	 $display ("        ", fshow (wra));
	 $display ("        ", fshow (wrd));
      end
   endrule

   // ----------------------------------------------------------------
   // Handle request from fabric

   let req_raw_mem_addr = f_reqs.first.raw_addr;    // E15: precomputed at merge

   Raw_Mem_Addr req_line = req_raw_mem_addr >> 1;
   Bit #(1)     req_half = req_raw_mem_addr [0];
   Bool         same_line = (rg_line == req_line);
   Bool         any_pending = (rg_hstat [0] == hs_pending) || (rg_hstat [1] == hs_pending);
   Bool         req_half_valid = (rg_hstat [req_half] == hs_valid);

   function Action fa_raw_write (Bit #(1) h, Raw_Mem_Word w);
      action
         f_raw_mem_reqs.enq (MemoryRequest {write:   True,
                                            byteen:  '1,
                                            address: { rg_line [valueOf (Bits_per_Raw_Mem_Addr) - 2 : 0], h },
                                            data:    w});
      endaction
   endfunction

   function Action fa_raw_read (Raw_Mem_Addr line, Bit #(1) h);
      action
         f_raw_mem_reqs.enq (MemoryRequest {write:   False,
                                            byteen:  '1,
                                            address: { line [valueOf (Bits_per_Raw_Mem_Addr) - 2 : 0], h },
                                            data:    ?});
         f_rd_order.enq (h);
      endaction
   endfunction

   // ----------------
   // Raw-memory read responses arrive in issue order; f_rd_order says which half.
   (* descending_urgency = "rl_raw_rsp, rl_process_wr_req" *)
   rule rl_raw_rsp;
      let raw_mem_rsp <- pop (f_raw_mem_rsps);
      let h = f_rd_order.first;  f_rd_order.deq;
      if (h == 0) rg_half0 <= raw_mem_rsp.data; else rg_half1 <= raw_mem_rsp.data;
      rg_hstat [h] <= hs_valid;
   endrule

   // Second half of a line switch, issued the cycle after the critical half.
   rule rl_issue_other (rg_state == STATE_READY &&& rg_issue_other matches tagged Valid .h);
      fa_raw_read (rg_line, h);
      rg_issue_other <= tagged Invalid;
   endrule

   // ----------------
   // Write back dirty halves: when idle, or when the next request is for another line.

   rule rl_writeback_dirty (   (rg_state == STATE_READY)
                            && ((! f_reqs.notEmpty) || (f_reqs.first.addr_ok && (! same_line)))
                            && (rg_dirty0 || rg_dirty1));
      if (rg_dirty0) begin
         fa_raw_write (0, rg_half0);
         rg_dirty0 <= False;
      end
      else begin
         fa_raw_write (1, rg_half1);
         rg_dirty1 <= False;
      end
      if (cfg_verbosity > 2)
         $display ("%0d: Mem_Controller.rl_writeback_dirty line 0x%0h", cur_cycle, rg_line);
   endrule

   // ----------------
   // Line switch: clean, nothing in flight.  Critical half first.

   rule rl_switch_line (   (rg_state == STATE_READY)
                        && f_reqs.first.addr_ok
                        && (! same_line)
                        && (! rg_dirty0) && (! rg_dirty1)
                        && (! f_rd_order.notEmpty)
                        && (! isValid (rg_issue_other)));
      fa_raw_read (req_line, req_half);
      rg_line        <= req_line;
      rg_hstat [req_half]     <= hs_pending;
      rg_hstat [1 - req_half] <= hs_pending;
      rg_issue_other <= tagged Valid (1 - req_half);
      if (cfg_verbosity > 2)
         $display ("%0d: Mem_Controller.rl_switch_line: raw addr 0x%0h", cur_cycle, req_raw_mem_addr);
   endrule

   // Same line, half invalid (only after reset): fetch just that half.
   rule rl_fill_half (   (rg_state == STATE_READY)
                      && f_reqs.first.addr_ok
                      && same_line
                      && (rg_hstat [req_half] == hs_invalid)
                      && (! f_rd_order.notEmpty)
                      && (! isValid (rg_issue_other)));
      fa_raw_read (rg_line, req_half);
      rg_hstat [req_half] <= hs_pending;
   endrule

   // ----------------
   // Read hit: returns the full Wd_Data-wide word containing the target byte.

   rule rl_process_rd_req  (   (rg_state == STATE_READY)
                            && f_reqs.first.addr_ok
                            && same_line && req_half_valid
                            && (f_reqs.first.req_op == REQ_OP_RD));
      Raw_Mem_Word w = (req_half == 0) ? rg_half0 : rg_half1;
      Vector #(Fabric_Data_per_Raw_Mem_Word, Bit #(Wd_Data)) raw_mem_word_V_fabric_data = unpack (w);
      Fabric_Addr addr = f_reqs.first.addr;
      Bit #(Bits_per_Byte_in_Raw_Mem_Word) n = addr [hi_byte_in_raw_mem_word : 0];
      n = (n >> lo_fabric_data);
      Bit #(Wd_Data) rdata = raw_mem_word_V_fabric_data [n];

      let rdr = AXI4_Rd_Data {rid:   f_reqs.first.id,
                              rdata: rdata,
                              rresp: axi4_resp_okay,
                              rlast: True,
                              ruser: f_reqs.first.user};
      slave_xactor.i_rd_data.enq (rdr);
      f_reqs.deq;

      if (cfg_verbosity > 1) begin
         $display ("%0d: Mem_Controller.rl_process_rd_req: ", cur_cycle);
         $display ("        ", fshow (f_reqs.first));
         $display ("     => ", fshow (rdr));
      end
   endrule

   // ----------------
   // Write hit (half valid, clean or dirty).

   rule rl_process_wr_req  (   (rg_state == STATE_READY)
                            && f_reqs.first.addr_ok
                            && same_line && req_half_valid
                            && (f_reqs.first.req_op == REQ_OP_WR));
      Raw_Mem_Word w = (req_half == 0) ? rg_half0 : rg_half1;
      Word64_in_Raw_Mem_Word word64_in_raw_mem_word = f_reqs.first.addr [hi_byte_in_raw_mem_word : 3];
      Vector #(Word64s_per_Raw_Mem_Word, Bit #(64)) raw_mem_word_V_Word64 = unpack (w);
      Bit #(64) word64_old = raw_mem_word_V_Word64 [word64_in_raw_mem_word];

      Bit #(64) word64_new = zeroExtend (f_reqs.first.data);
      Bit #(8)  strobe     = zeroExtend (f_reqs.first.wstrb);
      if ((valueOf (Wd_Data) == 32) && (f_reqs.first.addr [2] == 1'b1)) begin
         word64_new = { word64_new [31:0], 0 };
         strobe     = { strobe     [3:0],  0 };
      end
      Bit #(64) mask     = fn_strobe_to_mask (strobe);
      let updated_word64 = ((word64_old & (~ mask)) | (word64_new & mask));
      raw_mem_word_V_Word64 [word64_in_raw_mem_word] = updated_word64;
      if (req_half == 0) begin rg_half0 <= pack (raw_mem_word_V_Word64); rg_dirty0 <= True; end
      else               begin rg_half1 <= pack (raw_mem_word_V_Word64); rg_dirty1 <= True; end

      let wrr = AXI4_Wr_Resp {bid:   f_reqs.first.id,
                              bresp: axi4_resp_okay,
                              buser: f_reqs.first.user};
      slave_xactor.i_wr_resp.enq (wrr);
      f_reqs.deq;

      if (cfg_verbosity > 1) begin
         $display ("%0d: Mem_Controller.rl_process_wr_req: ", cur_cycle);
         $display ("        ", fshow (f_reqs.first));
         $display ("     => ", fshow (wrr));
      end

`ifdef WATCH_TOHOST
      // For simulation testing of riscv-tests/isa only:
      if ((rg_watch_tohost)
	  && (f_reqs.first.addr == rg_tohost_addr)
	  && (word64_new != 0))
	 begin

	    $display ("%0d: Mem_Controller.rl_process_wr_req: addr 0x%0h (<tohost>) data 0x%0h",
		      cur_cycle, f_reqs.first.addr, word64_new);
	    let exit_value = (word64_new >> 1);
	    if (exit_value == 0)
	       $display ("PASS");
	    else
	       $display ("FAIL %0d", exit_value);
	    rg_status <= fromInteger (status_mem_controller_terminated);
	 end
`endif
   endrule

   // ================================================================
   // Zero-memory FSM: zero out memory.
   // If needed, we must provide a way to enable it from the debug_module
   // This rule should be enabled with:
   //     rg_cached_raw_mem_addr <= 0;
   //     rg_state               <= STATE_ZEROING_MEM;

`ifdef INCLUDE_INITIAL_MEMZERO
   rule rl_zero_mem (rg_state == STATE_ZEROING_MEM);
      let raw_mem_req = MemoryRequest {write:   True,
				       byteen:  '1,
				       address: rg_cached_raw_mem_addr,
				       data:    0};
      f_raw_mem_reqs.enq (raw_mem_req);

      // Last write
      let raw_mem_words_per_mem = fn_raw_mem_words_per_mem (rg_addr_base, rg_addr_lim);
      if (rg_cached_raw_mem_addr == (raw_mem_words_per_mem - 1)) begin
	 rg_cached_raw_mem_addr <= rg_cached_raw_mem_addr;
	 rg_cached_raw_mem_word <= unpack (0);
	 rg_cached_clean        <= True;
	 rg_state <= STATE_READY;

	 // if (cfg_verbosity != 0)
	    $display ("%0d: Mem_Controller: zeroed %0d raw-memory locations (%0d-bit words)",
		      cur_cycle, raw_mem_words_per_mem, valueOf (Bits_per_Raw_Mem_Word));
      end
      else
	 rg_cached_raw_mem_addr <= rg_cached_raw_mem_addr + 1;
   endrule
`endif

   // ================================================================
   // Invalid address

   rule rl_invalid_rd_address (   (rg_state == STATE_READY)
			       && (! f_reqs.first.addr_ok)
			       && (f_reqs.first.req_op == REQ_OP_RD));
      Fabric_Data rdata = zeroExtend (f_reqs.first.addr);
      let rdr = AXI4_Rd_Data {rid:   f_reqs.first.id,
			      rdata: rdata,                 // for debugging only
			      rresp: axi4_resp_slverr,
			      rlast: True,
			      ruser: f_reqs.first.user};
      slave_xactor.i_rd_data.enq (rdr);
      f_reqs.deq;

      $write ("%0d: ERROR: Mem_Controller:", cur_cycle);
      if (! fn_addr_is_aligned (f_reqs.first.addr, f_reqs.first.size))
	 $display (" read-addr is misaligned");
      else
	 $display (" read-addr is out of bounds");
      $display ("        rg_addr_base 0x%0h  rg_addr_lim 0x%0h", rg_addr_base, rg_addr_lim);
      $display ("        ", fshow (f_reqs.first));
      $display ("     => ", fshow (rdr));
   endrule

   rule rl_invalid_wr_address (   (rg_state == STATE_READY)
			       && (! f_reqs.first.addr_ok)
			       && (f_reqs.first.req_op == REQ_OP_WR));
      let wrr = AXI4_Wr_Resp {bid:   f_reqs.first.id,
			      bresp: axi4_resp_slverr,
			      buser: f_reqs.first.user};
      slave_xactor.i_wr_resp.enq (wrr);
      f_reqs.deq;

      $write ("%0d: ERROR: Mem_Controller:", cur_cycle);
      if (! fn_addr_is_aligned (f_reqs.first.addr, f_reqs.first.size))
	 $display (" write-addr is misaligned");
      else
	 $display (" write-addr is out of bounds");
      $display ("        rg_addr_base 0x%0h  rg_addr_lim 0x%0h", rg_addr_base, rg_addr_lim);
      $display ("        ", fshow (f_reqs.first));
      $display ("     => ", fshow (wrr));
   endrule

   // ================================================================
   // INTERFACE

   // Reset
   interface  server_reset = toGPServer (f_reset_reqs, f_reset_rsps);

   // set_addr_map should be called after this module's reset
   method Action  set_addr_map (Fabric_Addr addr_base, Fabric_Addr addr_lim) if (rg_state == STATE_READY);
      rg_addr_base <= addr_base;
      rg_addr_lim  <= addr_lim;
      $display ("%0d: Mem_Controller.set_addr_map: addr_base 0x%0h addr_lim 0x%0h",
		cur_cycle, addr_base, addr_lim);

`ifdef INCLUDE_INITIAL_MEMZERO
      rg_cached_raw_mem_addr <= 0;
      rg_state               <= STATE_ZEROING_MEM;
      $display ("%0d: Mem_Controller.set_addr_map: zeroing memory from 0x%0h to 0x%0h",
		cur_cycle, addr_base, addr_lim);
`endif
   endmethod

   // Main Fabric Reqs/Rsps
   interface  slave = slave_xactor.axi_side;

   // To raw memory (outside the SoC)
   interface  to_raw_mem = toGPClient (f_raw_mem_reqs, f_raw_mem_rsps);

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
