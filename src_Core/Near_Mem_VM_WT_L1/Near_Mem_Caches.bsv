// Copyright (c) 2016-2019 Bluespec, Inc. All Rights Reserved.

// Near_Mem_IFC is an abstraction of two alternatives: caches or TCM
// (TCM = Tightly Coupled Memory).  Both are memories that are
// 'near' the CPU (1-cycle access in common case).

// On the CPU side it directly services instruction fetches and DMem
// reads and writes.

// On the Fabric side it has one or two Client sub-interfaces and a
// Server sub-interface.  The Client sub-interfaces are used to
// pass-through, to the fabric, I/O requests, cache-fill/ writeback
// requests, and any other memory requests outside the designated
// address range of Near_Mem.  There are two Client interfaces to
// accommodate IMem and DMem requests concurrently.

// This implementation of Near_Mem contains an ICache and a DCache
//        Fabric-side Server interface is not used (no back door to caches).

package Near_Mem_Caches;

// ================================================================
// BSV lib imports

import ConfigReg    :: *;
import FIFOF        :: *;
import GetPut       :: *;
import ClientServer :: *;
import Connectable  :: *;

// ----------------
// BSV additional libs

import Cur_Cycle  :: *;
import GetPut_Aux :: *;

// ================================================================
// Project imports

import ISA_Decls        :: *;
import Near_Mem_IFC     :: *;
import MMU_Cache_Common :: *;
import MMU_Cache        :: *;
import ICache_Pipe      :: *;
import AXI4_Types       :: *;
import Fabric_Defs      :: *;

`ifdef INCLUDE_DMEM_SLAVE
import MMU_Cache_Arbiter           :: *;
import AXI4_Lite_MMU_Cache_Adapter :: *;
`endif

// System address map and pc_reset value
import SoC_Map :: *;

// ================================================================
// Exports

export mkNear_Mem;
export Imem_Decoup_IFC (..), mkNear_Mem_Imem_Decoup;

// ================================================================
// STEP 5 shim-lean: near-mem-side I-fetch decoupling queue, the
// margin-first redesign of the step-4 shim.  Same functional contract
// (build/timing-exp/step3/DESIGN.md "STEP 4 FULL DESIGN" + "AS-BUILT
// REVISIONS", executable contract test/Imem_Decoup_Unit_Tb.bsv) with the
// CPU-visible response mux and the shim-internal cones REGISTERED:
//   - the word re-shown to the CPU comes from a presentation register
//     (rg_pres_*) loaded by the shim rule at arm time; the CPU-visible
//     cone is a 2:1 mux (pres vs live inner word) + the arrival kill,
//     not a live 4-slot mux (the step-4 s2/s3 critical-path cone).
//   - arrival detection is a single 2-bit compare (wr vs 1-cycle-delayed
//     wr), not a seen-register D-logic chain.
//   - exc responses are never forwarded to the CPU unless the ask stream
//     asked exactly that word (rg_await + wait-selector guards); un-asked
//     faulting spec words are dropped.  (Step 4 forwarded the FSM's raw
//     exc drive.)
//   - the req method is a PURE ENQUEUE (no submodule calls, no reads of
//     rule-written state): a 0-cycle inline launch there is unschedulable
//     (its inner.req call and the rule's launch calls cannot be proven
//     exclusive, and the tb's descending_urgency then starves this rule
//     on every consume cycle).  Launches are register-sourced in the one
//     shim rule: demand = queue-head register, spec = inflight+4.
//     Idle-pipe first fetches pay one cycle (rule-side launch at the
//     ask's arrival cycle); the cost is cycle-locked and mid-stream
//     timing is stock-exact.

interface Imem_Decoup_IFC;
   interface IMem_IFC imem;
   (* always_ready *) method Action ma_clear;
endinterface

typedef struct {
   Bit #(3)   f3;
   WordXL     addr;
   Priv_Mode  priv;
   Bit #(1)   sstatus_SUM;
   Bit #(1)   mstatus_MXR;
   WordXL     satp;
} Imem_Ask
deriving (Bits, FShow);

typedef struct {
   Bool     exc;
   Exc_Code exc_code;
   WordXL   addr;
   Instr    instr;
} Imem_Rsp
deriving (Bits, FShow);

module mkNear_Mem_Imem_Decoup #(MMU_Cache_IFC inner) (Imem_Decoup_IFC);

   // Ask queue: two entries, circular (physical index = counter bit 0).
   // rg_q_wr is written ONLY by the req method; rg_q_rd and rg_q_wr_d
   // ONLY by the shim rule (one writer each; occupancy <= 2 by the
   // wrapper's deposit rate, sim-asserted).
   Reg #(Imem_Ask) rg_q0      <- mkReg (Imem_Ask { f3: 3'b010, addr: 32'h0000_1000,
                                                   priv: m_Priv_Mode,
                                                   sstatus_SUM: 0, mstatus_MXR: 0,
                                                   satp: 0 });
   Reg #(Imem_Ask) rg_q1      <- mkReg (Imem_Ask { f3: 3'b010, addr: 32'h0000_1000,
                                                   priv: m_Priv_Mode,
                                                   sstatus_SUM: 0, mstatus_MXR: 0,
                                                   satp: 0 });
   Reg #(Bit #(2)) rg_q_wr    <- mkReg (0);
   Reg #(Bit #(2)) rg_q_rd    <- mkReg (0);
   Reg #(Bit #(2)) rg_q_wr_d  <- mkReg (0);

   // Outstanding inner read (sole writer: the shim rule).
   Reg #(Bool)     rg_inflight_valid <- mkReg (False);
   Reg #(WordXL)   rg_inflight_addr  <- mkReg (32'h0000_1000);

   // Previous cycle's inner response observation (fresh detection: a new
   // response is valid with the inflight addr after an invalid or
   // different-address cycle -- every launch kills the re-drive for at
   // least one cycle, so responses are always separated by a gap).
   Reg #(Bool)     rg_prev_inner_valid <- mkReg (False);
   Reg #(WordXL)   rg_prev_inner_addr  <- mkReg (32'h0000_1000);

   // VM args of the most recently launched demand ask (spec chain
   // reuses them; I-side is VM-off and every priv/satp change arrives with
   // a redirect that becomes the newest launched demand).
   Reg #(Bit #(3))   rg_cur_f3           <- mkReg (3'b010);
   Reg #(Priv_Mode)  rg_cur_priv         <- mkReg (m_Priv_Mode);
   Reg #(Bit #(1))   rg_cur_sstatus_SUM  <- mkReg (0);
   Reg #(Bit #(1))   rg_cur_mstatus_MXR  <- mkReg (0);
   Reg #(WordXL)     rg_cur_satp         <- mkReg (0);

   // Response hold: four slots (provable bound: 1 armed re-show + 2 queued
   // covered asks + 1 chain word).  Sole writer: the shim rule.
   Reg #(Bool)     rg_s0_valid <- mkReg (False);
   Reg #(WordXL)   rg_s0_addr  <- mkReg (32'h0000_1000);
   Reg #(Instr)    rg_s0_instr <- mkReg (32'h0000_0001);
   Reg #(Bool)     rg_s0_exc   <- mkReg (False);
   Reg #(Exc_Code) rg_s0_code  <- mkReg (0);
   Reg #(Bool)     rg_s1_valid <- mkReg (False);
   Reg #(WordXL)   rg_s1_addr  <- mkReg (32'h0000_1000);
   Reg #(Instr)    rg_s1_instr <- mkReg (32'h0000_0001);
   Reg #(Bool)     rg_s1_exc   <- mkReg (False);
   Reg #(Exc_Code) rg_s1_code  <- mkReg (0);
   Reg #(Bool)     rg_s2_valid <- mkReg (False);
   Reg #(WordXL)   rg_s2_addr  <- mkReg (32'h0000_1000);
   Reg #(Instr)    rg_s2_instr <- mkReg (32'h0000_0001);
   Reg #(Bool)     rg_s2_exc   <- mkReg (False);
   Reg #(Exc_Code) rg_s2_code  <- mkReg (0);
   Reg #(Bool)     rg_s3_valid <- mkReg (False);
   Reg #(WordXL)   rg_s3_addr  <- mkReg (32'h0000_1000);
   Reg #(Instr)    rg_s3_instr <- mkReg (32'h0000_0001);
   Reg #(Bool)     rg_s3_exc   <- mkReg (False);
   Reg #(Exc_Code) rg_s3_code  <- mkReg (0);

   // Registered CPU-visible presentation (the lean response mux): the
   // re-shown word is copied here at arm time, so the CPU-visible data
   // cone is a 2:1 mux instead of the live slot array.
   Reg #(Bool)     rg_shown      <- mkReg (False);
   Reg #(Bool)     rg_pres_exc   <- mkReg (False);
   Reg #(Exc_Code) rg_pres_code  <- mkReg (0);
   Reg #(WordXL)   rg_pres_addr  <- mkReg (32'h0000_1000);
   Reg #(Instr)    rg_pres_instr <- mkReg (32'h0000_0001);

   // Word presented last cycle (slot-freeing key: a deposit this cycle
   // consumed it; free every slot holding it).
   Reg #(Bool)     rg_w_valid <- mkReg (False);
   Reg #(WordXL)   rg_w_addr  <- mkReg (32'h0000_1000);

   // Ask-driven response await: set when a head pops whose word is not yet
   // available -- the wrapper is waiting for a response that has not
   // arrived; gates exc presentation so an un-asked faulting (speculative)
   // word can never trap the CPU.
   Reg #(Bool)     rg_await_valid <- mkReg (False);
   Reg #(WordXL)   rg_await_addr  <- mkReg (32'h0000_1000);

   // Most recently popped ask: with the queue head (when nonempty) it
   // selects which word the CPU-visible port presents (the wrapper only
   // ever waits for its outstanding ask).
   Reg #(Bool)     rg_last_valid <- mkReg (False);
   Reg #(WordXL)   rg_last_addr  <- mkReg (32'h0000_1000);

   // Re-show pending: a re-show arm (an ask covered by an available word)
   // owns the presentation until the armed word is presented-then-
   // consumed; while set, fresh-capture arms must not hijack pres (the
   // run-away spec chain would otherwise shadow the asked word).
   Reg #(Bool)     rg_reshow_pend <- mkReg (False);


   // CPU-visible presentation, STEP 9: registered output stage.  The
   // dw_* DWire discipline is retired -- rl_shim_step writes rg_out_*
   // each cycle from committed state + the live inner response, and the
   // imem interface reads the registers directly.  This cuts the
   // inner-response -> presentation combinational cone (icache.rg_addr
   // -> dw_*, ~9.4 ns of the 35.7 ns fetch loop) at the cost of +1
   // presentation latency.  Nothing in the shim reads rg_out_* same-
   // tick, so no Bluesim cross-rule-visibility deferral (the step-5
   // 7a trap does not apply).
   Reg #(Bool)     rg_out_valid <- mkReg (False);
   Reg #(Bool)     rg_out_exc   <- mkReg (False);
   Reg #(Exc_Code) rg_out_code  <- mkRegU;
   Reg #(WordXL)   rg_out_pc    <- mkRegU;
   Reg #(Instr)    rg_out_instr <- mkRegU;
   Reg #(WordXL)   rg_out_tval  <- mkRegU;

   // CPU-visible presentation wires: comb copies of the registered
   // output stage.  Kept as DWires (not direct register reads in the
   // interface) because CPU rules call imem value-methods and imem.req
   // in the same rule -- the wire discipline is what lets BSC schedule
   // them (the step-5 lesson; the G0004 method-conflict appears if the
   // interface reads registers directly).
   Reg #(Bool)     dw_valid  <- mkDWire (False);
   Reg #(Bool)     dw_exc    <- mkDWire (False);
   Reg #(Exc_Code) dw_code   <- mkDWire (?);
   Reg #(WordXL)   dw_pc     <- mkDWire (?);
   Reg #(Instr)    dw_instr  <- mkDWire (?);
   Reg #(WordXL)   dw_tval   <- mkDWire (?);

   // Flush observation (fence.i / sfence / reset notify).
   Reg #(Bool)     rg_flush_tog  <- mkReg (False);
   Reg #(Bool)     rg_flush_seen <- mkReg (False);

   // ----------------
   // Current-cycle observations (rule CanFire is constant true; see the
   // DESIGN G0033 note: CPU rules already condition on imem.valid while
   // calling imem.req).

   Bool fresh      = inner.valid && rg_inflight_valid
                                  && (inner.addr == rg_inflight_addr)
                                  && (! rg_prev_inner_valid
                                      || (inner.addr != rg_prev_inner_addr));
   Bool arrival    = (rg_q_wr != rg_q_wr_d);
   Bool flush      = (rg_flush_tog != rg_flush_seen);

   Imem_Ask head       = (rg_q_rd [0] == 1'b0) ? rg_q0 : rg_q1;
   Bool     head_valid = (rg_q_wr != rg_q_rd);

   Bool await_match = rg_await_valid && (inner.addr == rg_await_addr);

   Imem_Rsp inner_rsp = Imem_Rsp { exc:      inner.exc,
                                   exc_code: inner.exc_code,
                                   addr:     inner.addr,
                                   instr:    truncate (inner.cword) };

   // Slot lookups for head coverage and capture.
   Bool s0_head = rg_s0_valid && (rg_s0_addr == head.addr);
   Bool s1_head = rg_s1_valid && (rg_s1_addr == head.addr);
   Bool s2_head = rg_s2_valid && (rg_s2_addr == head.addr);
   Bool s3_head = rg_s3_valid && (rg_s3_addr == head.addr);
   Bool slot_head_match = s0_head || s1_head || s2_head || s3_head;

   Bool s0_cap = rg_s0_valid && (rg_s0_addr == inner.addr);
   Bool s1_cap = rg_s1_valid && (rg_s1_addr == inner.addr);
   Bool s2_cap = rg_s2_valid && (rg_s2_addr == inner.addr);
   Bool s3_cap = rg_s3_valid && (rg_s3_addr == inner.addr);
   Bool cap_refresh = s0_cap || s1_cap || s2_cap || s3_cap;

   Bool s0_free = ! rg_s0_valid;
   Bool s1_free = ! rg_s1_valid;
   Bool s2_free = ! rg_s2_valid;
   Bool s3_free = ! rg_s3_valid;
   Bool cap_ok  = cap_refresh || s0_free || s1_free || s2_free || s3_free;
   Bool cap_go  = fresh && cap_ok && (! inner.exc || await_match);

   // Head coverage by the available/inflight word set.
   Bool head_cov_slot  = slot_head_match;
   Bool head_cov_live  = inner.valid && (head.addr == inner.addr);
   Bool head_cov_infl  = rg_inflight_valid && (head.addr == rg_inflight_addr);
   Bool covered        = head_cov_slot || head_cov_live || head_cov_infl;
   Bool head_cov_avail = head_cov_slot || head_cov_live;

   // STEP 5D: SPECULATION OFF.  The spec chain was proven (5c) to be
   // the X-fragility itself -- any spec launch against seeded stock
   // X-state wedges early boot at any run-ahead depth.  Launches are
   // demand-only (queue head on its arrival); the ask queue still
   // decouples the CPU from the FSM round-trip.  No speculative
   // coverage either: with the spec target gone, a head matching
   // nothing available or inflight MUST launch (popping it without a
   // launch would lose the ask -- the reqq-wedge class).  The I3
   // timing contract is re-based to the measured spec-off milestones
   // (documented re-base, see step5/STATUS.md).
   // E8: never launch an inner request while the inner cache has a fabric
   // transaction outstanding (a flush clears rg_inflight_valid, but the
   // icache's AXI burst is still in flight; launching then abandons it and
   // the next refill consumes the stale burst -- the E5 cache-test wedge,
   // see build/timing-exp/50mhz-SURVEY.md "ROOT CAUSE").  Stock StageF had
   // this gate implicitly (BUSY until imem.valid).
   // E8c: a head that cannot launch because the inner cache is busy is HELD,
   // not popped (E8a/E8b popped it -- "every head pops" -- and the ask was
   // lost: fetch hung after the first wrong-path-miss-then-redirect; cache
   // test T-then-hang in sim and on the board).  A held head goes stale when
   // a newer ask arrives behind it (the CPU redirected again): it then pops
   // without launching, so the queue never holds more than the held head
   // plus the arrival (the >= 3 assertion below stays valid).
   Bool stale_head    = arrival && ((rg_q_wr - rg_q_rd) >= 2);
   Bool launch_demand = head_valid && (! covered) && (! inner.mv_busy) && (! stale_head);
   Bool hold_head     = head_valid && (! covered) && inner.mv_busy && (! stale_head);
   Bool any_launch    = launch_demand;

   // ----------------------------------------------------------------
   // Wait-selector: the CPU-visible port presents exactly the word the
   // wrapper is waiting for -- its outstanding ask (queue head if any,
   // else the most recently popped ask).  Over-presenting any other word
   // is invisible to the conds-gated wrapper for plain words, but an
   // un-asked faulting word must never trap the CPU, so the selector also
   // gates the exc presentation (with the rg_await guard below).

   Bool   wait_valid = head_valid || rg_last_valid;
   WordXL wait_addr  = head_valid ? head.addr : rg_last_addr;

   Bool pres_sel  = rg_shown && wait_valid && (rg_pres_addr == wait_addr);
   Bool inner_sel = inner.valid && wait_valid && (inner.addr == wait_addr);

   // Presentation arm: re-show arm (an ask covered by an available word;
   // served at exactly the stock re-fetch cycle), else fresh-capture arm
   // (the new response stays presented -- stock's re-drive -- so a
   // consume at fresh+1 still sees it; blocked while a re-show is pending
   // so the run-away spec chain cannot hijack the asked word's
   // presentation).  Ask-driven exc words arm too; un-asked exc words
   // are dropped (never captured/presented).
   Bool arm_reshow = arrival && head_valid && head_cov_avail;
   Bool arm        = arm_reshow
                     || (fresh && cap_go && (! rg_reshow_pend)
                         && (! inner.exc || await_match));

   Bool show_now = (! arrival) && (pres_sel || inner_sel)
                   && (! inner.exc || await_match);

   // ----------------------------------------------------------------
   // The single shim rule (unconditional; no CanFire edges -> no G0095).

   rule rl_shim_step;
      rg_q_wr_d <= rg_q_wr;

      // Drive the CPU-visible presentation from the registered output
      // stage (STEP 9); committed state + live inner response sampled
      // this tick, visible to the CPU next tick.
      rg_out_valid <= (! flush) && show_now;
      rg_out_pc    <= pres_sel ? rg_pres_addr : inner.addr;
      rg_out_instr <= pres_sel ? rg_pres_instr : truncate (inner.cword);
      rg_out_exc   <= pres_sel ? rg_pres_exc : (inner_sel && inner.exc);
      rg_out_code  <= pres_sel ? rg_pres_code : inner.exc_code;
      rg_out_tval  <= pres_sel ? rg_pres_addr : inner.addr;

      // CPU-visible wires mirror the registered output stage (last
      // cycle's committed word).
      dw_valid <= rg_out_valid;
      dw_exc   <= rg_out_exc;
      dw_code  <= rg_out_code;
      dw_pc    <= rg_out_pc;
      dw_instr <= rg_out_instr;
      dw_tval  <= rg_out_tval;

      if (flush) begin
`ifdef SHIM_DEBUG
         $display ("%0d: SHIM flush", cur_cycle);
`endif
         rg_q_rd           <= rg_q_wr;
         rg_inflight_valid <= False;
         rg_shown          <= False;
         rg_s0_valid <= False;
         rg_s1_valid <= False;
         rg_s2_valid <= False;
         rg_s3_valid <= False;
         rg_w_valid        <= False;
         rg_await_valid    <= False;
         rg_last_valid     <= False;
         rg_prev_inner_valid <= False;
         rg_reshow_pend    <= False;
         rg_flush_seen     <= rg_flush_tog;
      end
      else begin
         // Capture target: refresh the same-addr slot, else the lowest
         // free slot (cap_ok guarantees one exists when cap_go).
         Bit #(2) cap_sel = s0_cap ? 2'd0 : s1_cap ? 2'd1 : s2_cap ? 2'd2 : s3_cap ? 2'd3
                          : s0_free ? 2'd0 : s1_free ? 2'd1 : s2_free ? 2'd2 : 2'd3;
         Bool cap_s0 = cap_go && (cap_sel == 2'd0);
         Bool cap_s1 = cap_go && (cap_sel == 2'd1);
         Bool cap_s2 = cap_go && (cap_sel == 2'd2);
         Bool cap_s3 = cap_go && (cap_sel == 2'd3);

         // Free every slot holding the word presented-then-consumed last
         // cycle (pres is a register copy, so freeing never corrupts an
         // armed re-show).  A slot freed and re-captured the same cycle
         // stays valid with the fresh data.
         Bool free_s0 = arrival && rg_w_valid && rg_s0_valid && (rg_s0_addr == rg_w_addr);
         Bool free_s1 = arrival && rg_w_valid && rg_s1_valid && (rg_s1_addr == rg_w_addr);
         Bool free_s2 = arrival && rg_w_valid && rg_s2_valid && (rg_s2_addr == rg_w_addr);
         Bool free_s3 = arrival && rg_w_valid && rg_s3_valid && (rg_s3_addr == rg_w_addr);

         rg_s0_valid <= (rg_s0_valid && ! free_s0) || cap_s0;
         rg_s1_valid <= (rg_s1_valid && ! free_s1) || cap_s1;
         rg_s2_valid <= (rg_s2_valid && ! free_s2) || cap_s2;
         rg_s3_valid <= (rg_s3_valid && ! free_s3) || cap_s3;
         if (cap_s0) begin
            rg_s0_addr  <= inner.addr;
            rg_s0_instr <= truncate (inner.cword);
            rg_s0_exc   <= inner.exc;
            rg_s0_code  <= inner.exc_code;
         end
         if (cap_s1) begin
            rg_s1_addr  <= inner.addr;
            rg_s1_instr <= truncate (inner.cword);
            rg_s1_exc   <= inner.exc;
            rg_s1_code  <= inner.exc_code;
         end
         if (cap_s2) begin
            rg_s2_addr  <= inner.addr;
            rg_s2_instr <= truncate (inner.cword);
            rg_s2_exc   <= inner.exc;
            rg_s2_code  <= inner.exc_code;
         end
         if (cap_s3) begin
            rg_s3_addr  <= inner.addr;
            rg_s3_instr <= truncate (inner.cword);
            rg_s3_exc   <= inner.exc;
            rg_s3_code  <= inner.exc_code;
         end

         // shown'/pres' (re-show arm wins over the launch death).  The
         // arm copies the COVERING word: the matching slot first, else
         // the live/fresh inner word.  A pending re-show (an asked word
         // not yet consumed) survives launches -- stock's re-drive only
         // dies at the next req, and the next req cannot fire while the
         // word is still being consumed; letting the speculative chain's
         // launches kill it wedges a stalled consumer.
         if (arm) begin
            rg_shown <= True;
            if (s0_head) begin
               rg_pres_exc   <= rg_s0_exc;
               rg_pres_code  <= rg_s0_code;
               rg_pres_addr  <= rg_s0_addr;
               rg_pres_instr <= rg_s0_instr;
            end
            else if (s1_head) begin
               rg_pres_exc   <= rg_s1_exc;
               rg_pres_code  <= rg_s1_code;
               rg_pres_addr  <= rg_s1_addr;
               rg_pres_instr <= rg_s1_instr;
            end
            else if (s2_head) begin
               rg_pres_exc   <= rg_s2_exc;
               rg_pres_code  <= rg_s2_code;
               rg_pres_addr  <= rg_s2_addr;
               rg_pres_instr <= rg_s2_instr;
            end
            else if (s3_head) begin
               rg_pres_exc   <= rg_s3_exc;
               rg_pres_code  <= rg_s3_code;
               rg_pres_addr  <= rg_s3_addr;
               rg_pres_instr <= rg_s3_instr;
            end
            else begin
               rg_pres_exc   <= inner.exc;
               rg_pres_code  <= inner.exc_code;
               rg_pres_addr  <= inner.addr;
               rg_pres_instr <= truncate (inner.cword);
            end
         end
         else if (any_launch && (! rg_reshow_pend)) begin
            rg_shown <= False;
         end

         // Await tracking (exc presentation guard): set whenever a head
         // pops whose word is not yet available (launched or inflight/
         // plus4 covered) -- the wrapper is waiting for that response;
         // cleared when it arrives, when a launch clobbers it, or flush.
         if (fresh || any_launch) begin
            rg_await_valid <= False;
         end
         else if (arrival && head_valid && (! head_cov_avail)) begin
            rg_await_valid <= True;
            rg_await_addr  <= head.addr;
         end

         // Presented-word bookkeeping for next cycle's freeing.
         rg_w_valid <= show_now;
         rg_w_addr  <= pres_sel ? rg_pres_addr : inner.addr;

         // Re-show ownership: set on a re-show arm, cleared when the
         // armed word is presented-then-consumed.
         if (arm_reshow)
            rg_reshow_pend <= True;
         else if (arrival && rg_w_valid && (rg_w_addr == rg_pres_addr))
            rg_reshow_pend <= False;

         // Fresh-detection sample.
         rg_prev_inner_valid <= inner.valid;
         rg_prev_inner_addr  <= inner.addr;
`ifdef SHIM_DEBUG
         $display ("%0d: SHIM obs arrival=%b show=%b pres_sel=%b inner_sel=%b w_v=%b w=%h wait=%h shown=%b pres=%h inner_v=%b inner_a=%h | slots %h %h %h %h v=%b%b%b%b s_head=%b%b%b%b arm_rs=%b cap_go=%b", cur_cycle, arrival, show_now, pres_sel, inner_sel, rg_w_valid, rg_w_addr, wait_addr, rg_shown, rg_pres_addr, inner.valid, inner.addr, rg_s3_addr, rg_s2_addr, rg_s1_addr, rg_s0_addr, rg_s3_valid, rg_s2_valid, rg_s1_valid, rg_s0_valid, s3_head, s2_head, s1_head, s0_head, arm_reshow, cap_go);
`endif

         // Launches and pops.  Every head pops (covered asks are served,
         // uncovered asks launch -- the reqq-wedge class fix); the popped
         // ask becomes the wait-selector address.  The pure-enqueue req
         // method makes idle-pipe first fetches land here as an uncovered
         // head one cycle after the deposit (+1, cycle-locked).
         // A third entry with the queue still two deep means a deposit
         // was not popped -- the wrapper's rate-1 contract is broken.
         if (arrival && ((rg_q_wr - rg_q_rd) >= 3))
            $display ("%0d: ASSERT FAIL Imem_Decoup: ask queue overflow", cur_cycle);


         if (launch_demand) begin
`ifdef SHIM_DEBUG
            $display ("%0d: SHIM demand-launch head=%h fresh=%b cap_ok=%b slots=%b%b%b%b infl_v=%b w_v=%b w=%h arm=%b", cur_cycle, head.addr, fresh, cap_ok, rg_s3_valid, rg_s2_valid, rg_s1_valid, rg_s0_valid, rg_inflight_valid, rg_w_valid, rg_w_addr, arm);
`endif
            inner.req (CACHE_LD, head.f3,
`ifdef ISA_A
                       ?,
`endif
                       head.addr, 0,
                       head.priv, head.sstatus_SUM, head.mstatus_MXR, head.satp);
            rg_inflight_valid <= True;
            rg_inflight_addr  <= head.addr;
            rg_cur_f3          <= head.f3;
            rg_cur_priv        <= head.priv;
            rg_cur_sstatus_SUM <= head.sstatus_SUM;
            rg_cur_mstatus_MXR <= head.mstatus_MXR;
            rg_cur_satp        <= head.satp;
            rg_q_rd            <= rg_q_rd + 1;
            rg_last_valid      <= True;
            rg_last_addr       <= head.addr;
         end
         else begin
            if (head_valid && (! hold_head)) begin    // E8c: hold while busy
               rg_q_rd       <= rg_q_rd + 1;
               rg_last_valid <= True;
               rg_last_addr  <= head.addr;
            end
         end
      end
   endrule

   // ----------------------------------------------------------------
   // INTERFACE

   interface IMem_IFC imem;
      // CPU side: IMem request.  PURE ENQUEUE: no submodule calls, no
      // reads of rule-written state -- a 0-cycle inline launch here is
      // unschedulable (the rule's launch calls and the method's call to
      // inner.req cannot be proven exclusive, and the tb's descending
      // urgency then starves this module's rule on every consume cycle).
      // Idle-pipe first fetches therefore pay one cycle (launched by the
      // rule at the ask's arrival); mid-stream timing is stock-exact.
      method Action req (Bit #(3) f3,
                         WordXL addr,
                         Priv_Mode  priv,
                         Bit #(1)   sstatus_SUM,
                         Bit #(1)   mstatus_MXR,
                         WordXL     satp);
         if (rg_q_wr [0] == 1'b0)
            rg_q0 <= Imem_Ask { f3: f3, addr: addr, priv: priv,
                                sstatus_SUM: sstatus_SUM, mstatus_MXR: mstatus_MXR, satp: satp };
         else
            rg_q1 <= Imem_Ask { f3: f3, addr: addr, priv: priv,
                                sstatus_SUM: sstatus_SUM, mstatus_MXR: mstatus_MXR, satp: satp };
         rg_q_wr <= rg_q_wr + 1;
      endmethod

      // CPU side: IMem response: the dw wires mirror the STEP-9
      // registered output stage (rg_out_*), so the port shows exactly
      // the word the wrapper's outstanding ask waits for, one cycle
      // after the shim commits it.
      method Bool valid = dw_valid;
      method Bool is_i32_not_i16 = True;
      method WordXL pc    = dw_pc;
      method Instr  instr = dw_instr;
      method Bool   exc   = dw_exc;
      method Exc_Code exc_code = dw_code;
      method WordXL tval  = dw_tval;
   endinterface

   method Action ma_clear;
      rg_flush_tog <= ! rg_flush_tog;
   endmethod

endmodule: mkNear_Mem_Imem_Decoup

// ================================================================
// The module

// Module state
typedef enum {STATE_RESET, STATE_RESETTING, STATE_READY } State
deriving (Bits, Eq, FShow);

(* synthesize *)
module mkNear_Mem (Near_Mem_IFC);

   Reg #(Bit #(4)) cfg_verbosity <- mkConfigReg (0);
   Reg #(State)    rg_state      <- mkReg (STATE_READY);

   // ----------------
   // System address map and pc reset value
   SoC_Map_IFC  soc_map  <- mkSoC_Map;

   // Reset response queue
   FIFOF #(Token) f_reset_rsps <- mkFIFOF;

   // E20: pipelined I-side (replaces the I-side MMU_Cache, the Imem_Decoup
   // shim and CPU_Fetch_C; see ICache_Pipe.bsv).
   ICache_Pipe_IFC  icache <- mkICache_Pipe;
   MMU_Cache_IFC  dcache <- mkMMU_Cache (True);
`ifdef INCLUDE_DMEM_SLAVE
   MMU_Cache_Arbiter_IFC #(2) dcache_arbiter <- mkMMU_Cache_Arbiter (dcache);
   dcache = dcache_arbiter.v_from_masters [0];

   AXI4_Lite_MMU_Cache_Adapter_IFC dmem_slave_adapter <- mkAXI4_Lite_MMU_Cache_Adapter(dcache_arbiter.v_from_masters [1]);
`endif


   // ----------------------------------------------------------------
   // BEHAVIOR

   // ----------------
   // Reset
   // This reset state machine operates on external soft-reset request.

   rule rl_reset (rg_state == STATE_RESET);
      icache.server_reset.request.put (?);
      dcache.server_reset.request.put (?);
      rg_state <= STATE_RESETTING;

      if (cfg_verbosity > 1)
	 $display ("%0d: Near_Mem.rl_reset", cur_cycle);
   endrule

   rule rl_reset_complete (rg_state == STATE_RESETTING);
      let _dummy1 <- icache.server_reset.response.get;
      let _dummy2 <- dcache.server_reset.response.get;

      f_reset_rsps.enq (?);
      rg_state <= STATE_READY;

      if (cfg_verbosity > 1)
	 $display ("%0d: Near_Mem.rl_reset_complete", cur_cycle);
   endrule

   // ----------------
   // SFENCE_VMA

`ifdef ISA_PRIV_S
   FIFOF #(Token) f_sfence_vma_reqs <- mkFIFOF;
   FIFOF #(Token) f_sfence_vma_rsps <- mkFIFOF;

   rule rl_sfence_vma;
      let tok <- pop (f_sfence_vma_reqs);
      icache.tlb_flush;
      dcache.tlb_flush;
      f_sfence_vma_rsps.enq (tok);
   endrule
`endif

   // ----------------------------------------------------------------
   // INTERFACE

   // Reset
   interface Server server_reset;
      interface Put request;
	 method Action put (Token t) if (rg_state == STATE_READY);
	    rg_state <= STATE_RESET;
	 endmethod
      endinterface

      interface Get response;
	 method ActionValue #(Token) get ();
	    let rsp <- pop (f_reset_rsps);
	    return rsp;
	 endmethod
      endinterface
   endinterface

   // ----------------
   // IMem

   // CPU side
   interface IMem_IFC imem;
      // CPU side: IMem request
      method Action  req (Bit #(3) f3,
			  WordXL addr,
			  // The following  args for VM
			  Priv_Mode  priv,
			  Bit #(1)   sstatus_SUM,
			  Bit #(1)   mstatus_MXR,
			  WordXL     satp);    // { VM_Mode, ASID, PPN_for_page_table }
	 icache.imem.req (f3, addr, priv, sstatus_SUM, mstatus_MXR, satp);
      endmethod

      // CPU side: IMem response
      method Bool     valid          = icache.imem.valid;
      method Bool     is_i32_not_i16 = icache.imem.is_i32_not_i16;
      method WordXL   pc             = icache.imem.pc;
      method Instr    instr          = icache.imem.instr;
      method Bool     exc            = icache.imem.exc;
      method Exc_Code exc_code       = icache.imem.exc_code;
      method WordXL   tval           = icache.imem.tval;
   endinterface

   // Fabric side
   interface imem_master = icache.mem_master;

   // ----------------
   // DMem

   // CPU side
   interface DMem_IFC dmem;
      // CPU side: DMem request
      method Action  req (CacheOp op,
			  Bit #(3) f3,
`ifdef ISA_A
			  Bit #(7) amo_funct7,
`endif
			  WordXL addr,
			  Bit #(64) store_value,
			  // The following  args for VM
			  Priv_Mode  priv,
			  Bit #(1)   sstatus_SUM,
			  Bit #(1)   mstatus_MXR,
			  WordXL     satp);    // { VM_Mode, ASID, PPN_for_page_table }
	 dcache.req (op, f3,
`ifdef ISA_A
		     amo_funct7,
`endif
		     addr, store_value, priv, sstatus_SUM, mstatus_MXR, satp);
      endmethod

      // CPU side: DMem response
      method Bool       valid      = dcache.valid;
      method Bit #(64)  word64     = dcache.cword;
`ifdef ISA_A
      method Bit #(64)  st_amo_val = dcache.st_amo_val;
`endif
      method Bool       exc        = dcache.exc;
      method Exc_Code   exc_code   = dcache.exc_code;
      method Action     ma_kill_echo = dcache.ma_kill_echo;
      method Bool mv_busy = dcache.mv_busy;    // E8
   endinterface

   // Fabric side
   interface Near_Mem_Fabric_IFC  mem_master = dcache.mem_master;

`ifdef INCLUDE_DMEM_SLAVE
   interface dmem_slave = dmem_slave_adapter.from_master;
`endif

   // ----------------
   // FENCE.I: flush both ICache and DCache

   interface Server server_fence_i;
      interface Put request;
	 method Action put (Token t);
	    icache.server_flush.request.put (?);
	    dcache.server_flush.request.put (?);
	 endmethod
      endinterface
      interface Get response;
	 method ActionValue #(Token) get;
	    let ti <- icache.server_flush.response.get;
	    let td <- dcache.server_flush.response.get;
	    return ?;
	 endmethod
      endinterface
   endinterface

   // ----------------
   // FENCE: flush DCache

   interface Server server_fence;
      interface Put request;
	 method Action put (Fence_Ordering t);
	    dcache.server_flush.request.put (?);
	 endmethod
      endinterface
      interface Get response;
	 method ActionValue #(Token) get;
	    let td <- dcache.server_flush.response.get;
	    return ?;
	 endmethod
      endinterface
   endinterface

   // ----------------
   // SFENCE_VMA: flush TLBs

`ifdef ISA_PRIV_S
   interface Server sfence_vma_server = toGPServer (f_sfence_vma_reqs, f_sfence_vma_rsps);
`endif

   // ----------------------------------------------------------------
   // Interface to 'coherent DMA' port of optional L2 cache
   // Tied off: no L2 cache in WT_L1

   interface AXI4_Slave_IFC dma_server = dummy_AXI4_Slave_ifc;

   // ----------------------------------------------------------------
   // Misc. control and status

   // ----------------
   // For ISA tests: watch memory writes to <tohost> addr

`ifdef WATCH_TOHOST
   method Action set_watch_tohost (Bool watch_tohost, Bit #(64) tohost_addr);
      dcache.set_watch_tohost (watch_tohost, tohost_addr);
   endmethod

   method Bit #(64) mv_tohost_value = dcache.mv_tohost_value;
`endif

   // Inform core that DDR4 has been initialized and is ready to accept requests
   method Action ma_ddr4_ready;
      icache.ma_ddr4_ready;
      dcache.ma_ddr4_ready;
   endmethod

   // Misc. status; 0 = running, no error
   method Bit #(8) mv_status;
      return dcache.mv_status;
   endmethod

   // Performance-monitoring taps (see interface comment)
   method Bool mv_perf_imiss;
      return icache.mv_perf_miss;
   endmethod

   method Bool mv_perf_dmiss;
      return dcache.mv_perf_miss;
   endmethod

endmodule

// ================================================================

endpackage: Near_Mem_Caches
