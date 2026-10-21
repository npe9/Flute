// Copyright (c) 2026 -- ULX3S Flute 50 MHz campaign, E20 (big change 3: pipelined fetch)

package ICache_Pipe;

// ================================================================
// mkICache_Pipe: the I-side of Near_Mem.  Replaces the I-side MMU_Cache,
// the Imem_Decoup shim and CPU_Fetch_C (measured E15: CPI 4.00 on
// straight-line 32-bit code -- every instruction paid the full
// req -> shim -> FSM -> BRAM -> probe -> shim -> CPU round trip, because
// the I-side MMU_Cache is a one-request FSM).
//
// Structure (M/U only, no VM: physical addresses throughout):
//
//   P0  issue      : BRAM B-port read of tag set + both ways' data cword
//                    for the stream address rg_pf (one 64-bit cword per
//                    cycle), or for the CPU's wanted cword on a redirect.
//   S1  BRAM out   : raw BRAM outputs (no logic), captured into S2 regs.
//   S2  hit test   : registered tags/data -> hit -> enqueue the cword.
//                    Miss -> refill (the stream replays from the missed
//                    cword); non-memory -> stream halts (device space is
//                    fetched on demand only, never prefetched).
//   Q   queue      : 4 cwords, contiguous, oldest at q[0].  Issue is
//                    credit-limited: queued + in flight < 4, so run-ahead
//                    is bounded by what the CPU consumes (STEP 5c lesson:
//                    an unbounded speculative chain amplified X-state).
//
// CPU side (IMem_IFC, same contract as CPU_Fetch_C + shim): req(pc)
// records the wanted pc (ConfigReg; no submodule calls -- the STEP 5
// lesson); the response methods present the 16- or 32-bit instruction at
// the wanted pc from q[0]/q[1] (+ the following entry for a 32-bit
// instruction straddling two cwords), combinationally from registers.
// Consumption is inferred lazily: when the wanted pc moves into q[1],
// q[0] is popped; when it is the next cword the stream will deliver, the
// queue is dropped and the request waits; anything else is a redirect
// (queue + pipe dropped, stream restarts at the wanted cword).
//
// Refills and device reads are never abandoned (E5 root cause: an
// abandoned burst filled the NEXT request): a redirect during one marks
// it stale; it completes, and the stream then restarts at the new target.
// All registers are reset-initialized (STEP 5 X-fragility lesson).
//
// Linux program (Sv32 + PMP).  The cache is VIPT: the set index is within
// the page offset, so P0 is unchanged and S1 (raw BRAM out) translates:
//   S1  xlate      : ITLB (8 entries, fully associative, 4 KiB and 4 MiB
//                    leaves) looked up from rg_s1_a, or the identity map in
//                    M-mode / satp Bare; then PMP (execute, on the PA page)
//                    and PA[33:32] == 0.  Registered into S2 as {ppn, kind}.
//   S2             : the tag compare uses the PA (ctag from rg_s2_ppn).  A
//                    page / access fault enqueues a fault entry (QEnt.pf
//                    selects the code) and halts the stream; an ITLB miss
//                    halts it like device space.
//   PTW            : demand only -- when the CPU wants exactly the halted
//                    cword (never for prefetch).  Single-beat 32-bit PTE
//                    reads on this port; each PTE address is PMP-checked as an
//                    S-mode read.  Svade: A = 0 is a page fault.  Leaves are
//                    inserted with their U / X bits; permission is checked at
//                    lookup against the current privilege.
// Context: req toggles rg_ctx_seq when priv or satp change, ma_pmp toggles
// rg_pmp_seq when the PMP table changes, tlb_flush (SFENCE.VMA) toggles
// rg_tlb_seq.  rl_main then drops the queue and the pipe (in-flight walks /
// refills become stale) and, for SFENCE.VMA, clears the ITLB; valid is held
// low while any of them is pending.

// ================================================================

export ICache_Pipe_IFC (..), mkICache_Pipe;

import Vector       :: *;
import BRAMCore     :: *;
import ConfigReg    :: *;
import PMP          :: *;
import FIFOF        :: *;
import GetPut       :: *;
import ClientServer :: *;

import Cur_Cycle     :: *;
import Semi_FIFOF    :: *;

import ISA_Decls        :: *;
import Near_Mem_IFC     :: *;
import MMU_Cache_Common :: *;
import Cache_Decls      :: *;
import SoC_Map          :: *;
import AXI4_Types       :: *;
import Fabric_Defs      :: *;

// ================================================================

interface ICache_Pipe_IFC;
   interface IMem_IFC imem;
   interface Server #(Token, Token) server_reset;
   interface Server #(Token, Token) server_flush;    // FENCE.I
   interface AXI4_Master_IFC #(Wd_Id, Wd_Addr, Wd_Data, Wd_User) mem_master;
   method Action ma_ddr4_ready;
   method Bool mv_perf_miss;
   (* always_ready, always_enabled *)
   method Action ma_pmp (PMP_Table t);
   (* always_ready *)
   method Action tlb_flush;    // SFENCE.VMA
endinterface

// ================================================================

typedef TSub #(XLEN, 3) Bits_per_CW_Addr;
typedef Bit #(Bits_per_CW_Addr) CW_Addr;    // cword (8-byte) address = addr [XLEN-1:3]

typedef enum { CTAG_EMPTY, CTAG_CLEAN } ICTagState
deriving (Bits, Eq, FShow);

typedef struct {
   ICTagState  state;
   CTag        ctag;
   } ITag
deriving (Bits, FShow);

typedef Vector #(Ways_per_CSet, ITag) ITag_CSet;

typedef struct {
   Bool      v;
   CW_Addr   a;
   Bool      exc;
   Bool      pf;      // exc: instruction page fault (else access fault)
   Bit #(64) d;
   } QEnt
deriving (Bits, FShow);

QEnt qent_empty = QEnt { v: False, a: 0, exc: False, pf: False, d: 0 };

typedef enum { ST_RUN, ST_REFILL_REQ, ST_REFILL, ST_IO_REQ, ST_IO_RSP, ST_FLUSH,
               ST_PTW1_REQ, ST_PTW1_RSP, ST_PTW0_REQ, ST_PTW0_RSP } IState
deriving (Bits, Eq, FShow);

// ITLB entry (Sv32 leaf; ASID is read-only 0, so no ASID tag)
typedef struct {
   Bool      v;
   Bool      mega;    // 4 MiB leaf: vpn [9:0] ignored, ppn [9:0] from the VA
   Bit #(20) vpn;
   Bit #(22) ppn;
   Bit #(1)  u;
   Bit #(1)  x;
   } ITLBE
deriving (Bits, FShow);

typedef 8 ITLB_N;

// S1 translation outcome for a cword
typedef enum { XK_OK, XK_MISS, XK_PF, XK_AF } XKind deriving (Bits, Eq, FShow);

typedef struct {
   Bool      valid;
   Bool      is32;
   Instr     instr;
   Bool      exc;
   Bool      pf;
   WordXL    tval;
   Bool      base_hit;    // wanted cword is q[0] or q[1]
   Bool      pop1;        // wanted cword is q[1]: q[0] is consumed
   CW_Addr   need_cw;     // when ! valid: the cword still needed
   } Present
deriving (Bits, FShow);

function WordXL fn_cw_to_addr (CW_Addr a) = { a, 3'b0 };

// PA of a cword from its page number and the VA's page offset
function PA fn_pa (Bit #(22) ppn, CW_Addr a) = { ppn, a [8:0], 3'b0 };

function Bit #(16) fn_parcel (Bit #(64) d, Bit #(2) k);
   Vector #(4, Bit #(16)) p = unpack (d);
   return p [k];
endfunction

// ================================================================

(* synthesize *)
module mkICache_Pipe (ICache_Pipe_IFC);

   SoC_Map_IFC soc_map <- mkSoC_Map;

   // ----------------
   // RAMs.  Port A: refill / flush writes.  Port B: stream reads.
   // Instance names kept from MMU_Cache (ram_state_and_ctag_cset,
   // ram_cword_set = way 0) so the strict_bram harness census still binds.
   Bool config_output_register = False;
   BRAM_DUAL_PORT #(CSet_in_Cache, ITag_CSet) ram_state_and_ctag_cset
      <- mkBRAMCore2 (csets_per_cache, config_output_register);
   BRAM_DUAL_PORT #(CSet_CWord_in_Cache, CWord) ram_cword_set
      <- mkBRAMCore2 (cset_cwords_per_cache, config_output_register);
   BRAM_DUAL_PORT #(CSet_CWord_in_Cache, CWord) ram_cword_set_w1
      <- mkBRAMCore2 (cset_cwords_per_cache, config_output_register);

   function BRAM_DUAL_PORT #(CSet_CWord_in_Cache, CWord) fn_way_ram (Integer w)
      = ((w == 0) ? ram_cword_set : ram_cword_set_w1);

   AXI4_Master_Xactor_IFC #(Wd_Id, Wd_Addr, Wd_Data, Wd_User) master_xactor <- mkAXI4_Master_Xactor;

   // Unguarded internally (rl_main has no implicit conditions); the
   // interface methods and the AXI mover rules carry the guards.
   FIFOF #(Token) f_reset_reqs <- mkUGFIFOF;
   FIFOF #(Token) f_reset_rsps <- mkUGFIFOF;
   FIFOF #(Token) f_flush_reqs <- mkUGFIFOF;
   FIFOF #(Token) f_flush_rsps <- mkUGFIFOF;
   FIFOF #(AXI4_Rd_Addr #(Wd_Id, Wd_Addr, Wd_User)) f_ar <- mkUGFIFOF;
   FIFOF #(AXI4_Rd_Data #(Wd_Id, Wd_Data, Wd_User)) f_rd <- mkUGFIFOF;

   rule rl_ar_move (f_ar.notEmpty);
      master_xactor.i_rd_addr.enq (f_ar.first);
      f_ar.deq;
   endrule

   rule rl_rd_move (f_rd.notFull);
      let x <- pop_o (master_xactor.o_rd_data);
      f_rd.enq (x);
   endrule

   Reg #(Bool) rg_ddr4_ready <- mkReg (False);
   PulseWire   pw_perf_miss  <- mkPulseWire;

   // ----------------
   // CPU-side request (sole writer: the req method; ConfigReg so the main
   // rule, which reads them, may schedule after the CPU rule that calls
   // the response methods and then req).
   Reg #(WordXL) rg_pc   <- mkConfigReg ('h1000);
   Reg #(Bool)   rg_want <- mkConfigReg (False);
   Reg #(Priv_Mode) rg_priv <- mkConfigReg (m_Priv_Mode);    // fetch privilege
   Reg #(WordXL)    rg_satp <- mkConfigReg (0);

   // Context sequence bits (writer: the named method; reader/acknowledger: rl_main)
   Reg #(Bool) rg_ctx_seq  <- mkConfigReg (False);    // req: priv / satp changed
   Reg #(Bool) rg_pmp_seq  <- mkConfigReg (False);    // ma_pmp: table changed
   Reg #(Bool) rg_tlb_seq  <- mkConfigReg (False);    // tlb_flush (SFENCE.VMA)
   Reg #(Bool) rg_ctx_seen <- mkReg (False);
   Reg #(Bool) rg_pmp_seen <- mkReg (False);
   Reg #(Bool) rg_tlb_seen <- mkReg (False);

   Reg #(PMP_Table) rg_pmp <- mkConfigReg (unpack (0));

   // ITLB and PTW (sole writer: rl_main)
   Vector #(ITLB_N, Reg #(ITLBE)) vrg_itlb <- replicateM (mkReg (unpack (0)));
   Reg #(Bit #(3))   rg_itlb_rr  <- mkReg (0);
   Reg #(PA)         rg_pte_pa   <- mkReg (0);
   Reg #(Bool)       rg_halt_walk <- mkReg (False);    // stream at an ITLB-miss cword

   // ----------------
   // Everything below: sole writer rl_main.
   Reg #(IState) rg_st <- mkReg (ST_RUN);

   Vector #(4, Reg #(QEnt)) q <- replicateM (mkReg (qent_empty));

   Reg #(CW_Addr) rg_pf      <- mkReg ('h200);    // next cword the stream reads
   Reg #(Bool)    rg_halt_io  <- mkReg (False);    // stream at a non-memory cword
   Reg #(Bool)    rg_halt_exc <- mkReg (False);    // stream ended with a fault entry

   Reg #(Bool)    rg_s1_v <- mkReg (False);
   Reg #(CW_Addr) rg_s1_a <- mkReg (0);
   Reg #(Bool)    rg_s2_v <- mkReg (False);
   Reg #(CW_Addr) rg_s2_a <- mkReg (0);
   Reg #(Bit #(22)) rg_s2_ppn <- mkReg (0);
   Reg #(XKind)     rg_s2_xk  <- mkReg (XK_OK);
   Reg #(ITag_CSet)                     rg_s2_tags <- mkReg (replicate (ITag { state: CTAG_EMPTY, ctag: 0 }));
   Reg #(Vector #(Ways_per_CSet, CWord)) rg_s2_data <- mkReg (replicate (0));

   // Refill / device read
   Reg #(CW_Addr)        rg_miss_a     <- mkReg (0);
   Reg #(Bit #(22))      rg_miss_ppn   <- mkReg (0);
   Reg #(Bit #(22))      rg_io_ppn     <- mkReg (0);
   Reg #(ITag_CSet)      rg_miss_tags  <- mkReg (replicate (ITag { state: CTAG_EMPTY, ctag: 0 }));
   Reg #(Way_in_CSet)    rg_miss_way   <- mkReg (0);
   Reg #(Way_in_CSet)    rg_victim     <- mkReg (0);
   Reg #(CWord_in_CLine) rg_beat       <- mkReg (0);
   Reg #(Bool)           rg_refill_err <- mkReg (False);
   Reg #(Bool)           rg_stale      <- mkReg (False);

   // Flush / reset loop
   Reg #(CSet_in_Cache)  rg_flush_set   <- mkReg (0);
   Reg #(Bool)           rg_flush_reset <- mkReg (False);

   // ================================================================
   // Presentation (pure function of registers; shared by the response
   // methods and rl_main)

   function Present fv_present ();
      CW_Addr  w = truncateLSB (rg_pc);
      Bit #(2) o = rg_pc [2:1];

      Bool m0 = q [0].v && (q [0].a == w);
      Bool m1 = q [1].v && (q [1].a == w);

      QEnt b = (m0 ? q [0] : q [1]);    // base entry
      QEnt n = (m0 ? q [1] : q [2]);    // following entry (contiguous)

      Bit #(16) p0   = fn_parcel (b.d, o);
      Bool      is32 = (p0 [1:0] == 2'b11);
      Bool      strad = is32 && (o == 2'b11);

      Present r = Present { valid: False, is32: is32, instr: ?, exc: False, pf: False,
                            tval: rg_pc, base_hit: m0 || m1, pop1: (! m0) && m1,
                            need_cw: w };
      if (m0 || m1) begin
         if (b.exc) begin
            r.valid = True;  r.exc = True;  r.pf = b.pf;  r.is32 = False;
         end
         else if (! is32) begin
            r.valid = True;  r.instr = zeroExtend (p0);
         end
         else if (! strad) begin
            r.valid = True;  r.instr = { fn_parcel (b.d, o + 1), p0 };
         end
         else if (n.v) begin
            r.valid = True;  r.instr = { fn_parcel (n.d, 0), p0 };
            r.exc   = n.exc;
            r.pf    = n.pf;
            r.tval  = fn_cw_to_addr (b.a + 1);
         end
         else
            r.need_cw = b.a + 1;
      end
      return r;
   endfunction

   // ================================================================
   // The single engine rule (fires every cycle).

   (* fire_when_enabled, no_implicit_conditions *)
   rule rl_main;
      let pres = fv_present;

      CW_Addr w = truncateLSB (rg_pc);
      Bool run  = (rg_st == ST_RUN);

      // Next cword the in-order stream will enqueue.
      CW_Addr enq_next = (rg_s2_v ? rg_s2_a : (rg_s1_v ? rg_s1_a : rg_pf));

      // ---- Context change (priv / satp / PMP / SFENCE.VMA): drop queue and pipe
      Bool tlb_pend = (rg_tlb_seq != rg_tlb_seen);
      Bool ctx_pend = (rg_ctx_seq != rg_ctx_seen) || (rg_pmp_seq != rg_pmp_seen) || tlb_pend;
      Bool ctx_kill = ctx_pend && (rg_st != ST_FLUSH);
      if (ctx_kill) begin
         rg_ctx_seen <= rg_ctx_seq;
         rg_pmp_seen <= rg_pmp_seq;
         rg_tlb_seen <= rg_tlb_seq;
      end

      // ---- Presentation-driven queue actions
      Bool pending  = (! ctx_kill) && (! pres.base_hit) && (w == enq_next) && (! rg_halt_exc);
      Bool redirect = rg_want && (ctx_kill || ((! pres.base_hit) && (! pending))) && (rg_st != ST_FLUSH);
      Bool q_clear  = (rg_want && (! pres.base_hit) && (rg_st != ST_FLUSH)) || ctx_kill;    // pending or redirect
      Bool q_pop1   = rg_want && pres.pop1;
      Bool kill     = redirect || ctx_kill;    // in-flight pipe / fabric work is for a dead stream

      // ---- S1: translate rg_s1_a (registered into S2)
      WordXL   s1_va   = fn_cw_to_addr (rg_s1_a);
      Bool     xlate   = (rg_priv != m_Priv_Mode) && (rg_satp [31] == 1'b1);
      Bit #(22) s1_ppn = zeroExtend (s1_va [31:12]);
      XKind    s1_xk   = XK_OK;
      if (xlate) begin
         Maybe #(ITLBE) m_e = tagged Invalid;
         for (Integer i = 0; i < valueOf (ITLB_N); i = i + 1) begin
            let e = vrg_itlb [i];
            Bool hit_i = e.v && (e.mega ? (e.vpn [19:10] == s1_va [31:22]) : (e.vpn == s1_va [31:12]));
            if (hit_i) m_e = tagged Valid e;
         end
         if (m_e matches tagged Valid .e) begin
            s1_ppn = (e.mega ? { e.ppn [21:10], s1_va [21:12] } : e.ppn);
            Bool perm = (e.x == 1'b1) && ((rg_priv == u_Priv_Mode) == (e.u == 1'b1));
            if (! perm) s1_xk = XK_PF;
         end
         else
            s1_xk = XK_MISS;
      end
      if ((s1_xk == XK_OK) && (   (s1_ppn [21:20] != 0)
                               || (! fv_pmp_ok (rg_pmp, s1_ppn, rg_priv, PMP_X))))
         s1_xk = XK_AF;

      // ---- Flush / reset start (only from RUN: refills/device reads finish first)
      Bool flush_start = run && (f_reset_reqs.notEmpty || f_flush_reqs.notEmpty);

      // ---- S2: hit test (current stream only)
      PA       s2_pa  = fn_pa (rg_s2_ppn, rg_s2_a);
      CTag     s2_tag = fn_PA_to_CTag (s2_pa);
      Bool     s2_mem = soc_map.m_is_mem_addr (fn_PA_to_Fabric_Addr (s2_pa));
      Bool     s2_ok  = (rg_s2_xk == XK_OK);
      Bool        hit     = False;
      Way_in_CSet hit_way = 0;
      for (Integer wy = 0; wy < ways_per_cset; wy = wy + 1)
         if ((rg_s2_tags [wy].state == CTAG_CLEAN) && (rg_s2_tags [wy].ctag == s2_tag)) begin
            hit = True;  hit_way = fromInteger (wy);
         end
      Bool s2_live  = rg_s2_v && run && (! kill) && (! flush_start);
      Bool s2_enq   = s2_live && s2_ok && s2_mem && hit;
      Bool s2_miss  = s2_live && s2_ok && s2_mem && (! hit);
      Bool s2_io    = s2_live && s2_ok && (! s2_mem);
      Bool s2_walk  = s2_live && (rg_s2_xk == XK_MISS);
      Bool s2_fault = s2_live && ((rg_s2_xk == XK_PF) || (rg_s2_xk == XK_AF));
      Bool p2_kill  = s2_miss || s2_io || s2_walk || s2_fault;

      // ---- Queue next state: clear / pop, then enqueue at the tail
      Vector #(4, QEnt) qn = readVReg (q);
      if (flush_start || q_clear)
         qn = replicate (qent_empty);
      else if (q_pop1)
         qn = shiftInAtN (qn, qent_empty);

      Maybe #(QEnt) m_enq = tagged Invalid;
      if (s2_enq) begin
         CWord d = rg_s2_data [0];
         for (Integer wy = 1; wy < ways_per_cset; wy = wy + 1)
            if (hit_way == fromInteger (wy)) d = rg_s2_data [wy];
         m_enq = tagged Valid (QEnt { v: True, a: rg_s2_a, exc: False, pf: False, d: d });
      end
      if (s2_fault)
         m_enq = tagged Valid (QEnt { v: True, a: rg_s2_a, exc: True, pf: (rg_s2_xk == XK_PF), d: 0 });

      // ---- Device read / refill completion (may also enqueue)
      Bool drop = rg_stale || kill;
      CW_Addr pf_next = rg_pf;
      Bool    halt_io_next   = rg_halt_io;
      Bool    halt_exc_next  = rg_halt_exc;
      Bool    halt_walk_next = rg_halt_walk;
      IState  st_next = rg_st;
      if (s2_fault) begin
         halt_exc_next = True;
         pf_next = rg_s2_a + 1;
      end

      Bool rd_rsp_avail = f_rd.notEmpty;
      let  rd_rsp       = f_rd.first;
      Bool rd_ok        = (rd_rsp.rresp == axi4_resp_okay);

      if ((rg_st == ST_IO_RSP) && rd_rsp_avail) begin
         f_rd.deq;
         if (! drop) begin
            m_enq = tagged Valid (QEnt { v: True, a: rg_pf, exc: ! rd_ok, pf: False, d: zeroExtend (rd_rsp.rdata) });
            pf_next = rg_pf + 1;
            if (! rd_ok) halt_exc_next = True;
            // the next device cword is in another page: re-translate (stream resumes)
            if ((rg_pf + 1) [8:0] == 0) halt_io_next = False;
         end
         st_next = ST_RUN;
      end

      if ((rg_st == ST_REFILL) && rd_rsp_avail) begin
         f_rd.deq;
         Bool err = rg_refill_err || (! rd_ok);
         CSet_in_Cache cset = fn_Addr_to_CSet_in_Cache (fn_cw_to_addr (rg_miss_a));
         CSet_CWord_in_Cache wa = { cset, rg_beat };
         if (rd_ok) begin
            if (rg_miss_way == 0) ram_cword_set.a.put    (True, wa, zeroExtend (rd_rsp.rdata));
            else                  ram_cword_set_w1.a.put (True, wa, zeroExtend (rd_rsp.rdata));
         end
         rg_refill_err <= err;
         rg_beat       <= rg_beat + 1;
         if (rg_beat == fromInteger (cwords_per_cline - 1)) begin
            if (! err) begin
               ITag_CSet t = rg_miss_tags;
               t [rg_miss_way] = ITag { state: CTAG_CLEAN,
                                        ctag:  fn_PA_to_CTag (fn_pa (rg_miss_ppn, rg_miss_a)) };
               ram_state_and_ctag_cset.a.put (True, cset, t);
            end
            else if (! drop) begin
               m_enq = tagged Valid (QEnt { v: True, a: rg_miss_a, exc: True, pf: False, d: 0 });
               halt_exc_next = True;
               pf_next = rg_miss_a + 1;
            end
            st_next = ST_RUN;
         end
      end

      // ---- PTW: PTE responses (single beat; the PTE is the addressed half)
      Bit #(32) pte = (rg_pte_pa [2] == 1'b1) ? rd_rsp.rdata [63:32] : rd_rsp.rdata [31:0];
      Bool pte_v = (pte [0] == 1'b1);
      Bool pte_r = (pte [1] == 1'b1);
      Bool pte_w = (pte [2] == 1'b1);
      Bool pte_x = (pte [3] == 1'b1);
      Bool pte_a = (pte [6] == 1'b1);
      Bool pte_leaf = pte_r || pte_x;
      WordXL walk_va = fn_cw_to_addr (rg_pf);
      Maybe #(Bool) walk_fault = tagged Invalid;    // Valid pf: fault (page fault if True)
      Maybe #(ITLBE) walk_ins  = tagged Invalid;
      Bool walk_done = False;

      if (((rg_st == ST_PTW1_RSP) || (rg_st == ST_PTW0_RSP)) && rd_rsp_avail) begin
         f_rd.deq;
         walk_done = True;
         Bool lvl1 = (rg_st == ST_PTW1_RSP);
         if (! rd_ok)
            walk_fault = tagged Valid False;
         else if ((! pte_v) || ((! pte_r) && pte_w))
            walk_fault = tagged Valid True;
         else if (pte_leaf) begin
            if ((lvl1 && (pte [19:10] != 0)) || (! pte_a))    // misaligned megapage; Svade A
               walk_fault = tagged Valid True;
            else
               walk_ins = tagged Valid (ITLBE { v: True, mega: lvl1, vpn: walk_va [31:12],
                                                ppn: pte [31:10], u: pte [4], x: pte [3] });
         end
         else if (! lvl1)
            walk_fault = tagged Valid True;                   // pointer at level 0
         else if (! drop) begin
            rg_pte_pa <= { pte [31:10], 12'b0 } + zeroExtend ({ walk_va [21:12], 2'b0 });
            walk_done = False;
            st_next = ST_PTW0_REQ;
         end
         if (walk_done) st_next = ST_RUN;
      end

      // PTE read requests (PMP: S-mode read; PA[33:32] must be 0)
      if (((rg_st == ST_PTW1_REQ) || (rg_st == ST_PTW0_REQ)) && rg_ddr4_ready && f_ar.notFull && (! kill)) begin
         PA pte_pa = ((rg_st == ST_PTW1_REQ)
                      ? ({ rg_satp [21:0], 12'b0 } + zeroExtend ({ walk_va [31:22], 2'b0 }))
                      : rg_pte_pa);
         if ((pte_pa [33:32] != 0) || (! fv_pmp_ok (rg_pmp, fv_pa_page (pte_pa), s_Priv_Mode, PMP_R))) begin
            walk_fault = tagged Valid False;
            walk_done  = True;
            st_next    = ST_RUN;
         end
         else begin
            let ar = AXI4_Rd_Addr { arid:     fabric_default_id,
                                    araddr:   fn_PA_to_Fabric_Addr (pte_pa),
                                    arlen:    0,
                                    arsize:   axsize_4,
                                    arburst:  axburst_incr,
                                    arlock:   fabric_default_lock,
                                    arcache:  fabric_default_arcache,
                                    arprot:   fabric_default_prot,
                                    arqos:    fabric_default_qos,
                                    arregion: fabric_default_region,
                                    aruser:   fabric_default_user };
            f_ar.enq (ar);
            rg_pte_pa <= pte_pa;
            st_next = ((rg_st == ST_PTW1_REQ) ? ST_PTW1_RSP : ST_PTW0_RSP);
         end
      end

      if (walk_done && (! drop)) begin
         halt_walk_next = False;
         if (walk_fault matches tagged Valid .is_pf) begin
            m_enq = tagged Valid (QEnt { v: True, a: rg_pf, exc: True, pf: is_pf, d: 0 });
            halt_exc_next = True;
            pf_next = rg_pf + 1;
         end
      end

      // ITLB: SFENCE.VMA clears; a completed walk inserts (round robin)
      if (ctx_kill && tlb_pend)
         for (Integer i = 0; i < valueOf (ITLB_N); i = i + 1)
            vrg_itlb [i] <= unpack (0);
      else if (walk_ins matches tagged Valid .e &&& (! drop)) begin
         vrg_itlb [rg_itlb_rr] <= e;
         rg_itlb_rr <= rg_itlb_rr + 1;
      end

      // Tail enqueue (credit-limited issue guarantees a free slot)
      if (m_enq matches tagged Valid .e) begin
         if (qn [3].v)
            $display ("%0d: ASSERT FAIL ICache_Pipe: enqueue into a full queue", cur_cycle);
         if      (! qn [0].v) qn [0] = e;
         else if (! qn [1].v) qn [1] = e;
         else if (! qn [2].v) qn [2] = e;
         else                 qn [3] = e;
      end
      writeVReg (q, qn);

      // ---- S2 miss / device stop: stream replays from s2's cword
      if (s2_miss) begin
         Way_in_CSet vw = rg_victim;
         for (Integer wy = ways_per_cset - 1; wy >= 0; wy = wy - 1)
            if (rg_s2_tags [wy].state == CTAG_EMPTY) vw = fromInteger (wy);
         rg_miss_a    <= rg_s2_a;
         rg_miss_tags <= rg_s2_tags;
         rg_miss_way  <= vw;
         if (ways_per_cset > 1) rg_victim <= rg_victim + 1;
         pf_next = rg_s2_a;
         st_next = ST_REFILL_REQ;
      end
      if (s2_miss) rg_miss_ppn <= rg_s2_ppn;
      if (s2_io) begin
         pf_next = rg_s2_a;
         halt_io_next = True;
         rg_io_ppn <= rg_s2_ppn;
      end
      if (s2_walk) begin
         pf_next = rg_s2_a;
         halt_walk_next = True;
      end

      // ---- Redirect: restart the stream at the wanted cword
      if (redirect) begin
         pf_next       = w;
         halt_io_next  = False;
         halt_exc_next = False;
         halt_walk_next = False;
      end
      else if (ctx_kill) begin    // no wanted pc: the stream restarts at the next req
         halt_io_next  = False;
         halt_exc_next = False;
         halt_walk_next = False;
      end

      // ---- A redirect before the AR is sent cancels the refill / device
      // read / PTE read outright (nothing outstanding on the fabric).
      if (kill && (   (rg_st == ST_REFILL_REQ) || (rg_st == ST_IO_REQ)
                   || (rg_st == ST_PTW1_REQ)   || (rg_st == ST_PTW0_REQ)))
         st_next = ST_RUN;

      // ---- Stale marking: lives from refill/device-read/walk start to completion
      if ((rg_st == ST_RUN) || (st_next == ST_RUN))
         rg_stale <= False;
      else if (kill)
         rg_stale <= True;

      // ---- P0: issue.  The BRAM address/enable do not depend on the S2
      // result (a kill only drops the S1 valid bit next cycle).
      Bit #(3) n_q  = (q [0].v ? 1 : 0) + (q [1].v ? 1 : 0) + (q [2].v ? 1 : 0) + (q [3].v ? 1 : 0);
      Bit #(3) n_fl = (rg_s1_v ? 1 : 0) + (rg_s2_v ? 1 : 0);
      Bool credit   = ((n_q + n_fl) < 4);
      Bool issue    = run && rg_want && (! flush_start)
                      && (redirect || ((! rg_halt_io) && (! rg_halt_exc) && (! rg_halt_walk) && credit));
      CW_Addr issue_a = (redirect ? w : rg_pf);
      if (issue) begin
         WordXL ia = fn_cw_to_addr (issue_a);
         ram_state_and_ctag_cset.b.put (False, fn_Addr_to_CSet_in_Cache (ia), ?);
         ram_cword_set.b.put    (False, fn_Addr_to_CSet_CWord_in_Cache (ia), ?);
         ram_cword_set_w1.b.put (False, fn_Addr_to_CSet_CWord_in_Cache (ia), ?);
         if (! p2_kill) pf_next = issue_a + 1;
      end

      // ---- Device demand read: the wanted instruction needs exactly the
      // halted stream's cword, nothing in flight.
      Bool need_io = run && rg_halt_io && rg_want && (! redirect) && (! flush_start)
                     && (! pres.valid) && (pres.need_cw == rg_pf)
                     && (! rg_s1_v) && (! rg_s2_v);
      if (need_io) st_next = ST_IO_REQ;

      // ---- Demand page-table walk: the wanted cword is the halted stream's
      Bool need_walk = run && rg_halt_walk && rg_want && (! redirect) && (! flush_start)
                       && (! pres.valid) && (pres.need_cw == rg_pf)
                       && (! rg_s1_v) && (! rg_s2_v);
      if (need_walk) st_next = ST_PTW1_REQ;

      // ---- Pipe advance + BRAM output capture
      rg_s1_v <= issue && (! p2_kill);
      rg_s1_a <= issue_a;
      rg_s2_v <= rg_s1_v && (! p2_kill) && (! kill) && (! flush_start);
      rg_s2_a <= rg_s1_a;
      rg_s2_ppn <= s1_ppn;
      rg_s2_xk  <= s1_xk;
      rg_s2_tags <= ram_state_and_ctag_cset.b.read;
      Vector #(Ways_per_CSet, CWord) dv = newVector;
      for (Integer wy = 0; wy < ways_per_cset; wy = wy + 1)
         dv [wy] = (wy == 0) ? ram_cword_set.b.read : ram_cword_set_w1.b.read;
      rg_s2_data <= dv;

      // ---- Fabric requests
      if ((rg_st == ST_REFILL_REQ) && rg_ddr4_ready && f_ar.notFull) begin
         PA line = fn_align_Addr_to_CLine (fn_pa (rg_miss_ppn, rg_miss_a));
         let ar = AXI4_Rd_Addr { arid:     fabric_default_id,
                                 araddr:   fn_PA_to_Fabric_Addr (line),
                                 arlen:    fromInteger (cwords_per_cline - 1),
                                 arsize:   axsize_8,
                                 arburst:  axburst_incr,
                                 arlock:   fabric_default_lock,
                                 arcache:  fabric_default_arcache,
                                 arprot:   fabric_default_prot,
                                 arqos:    fabric_default_qos,
                                 arregion: fabric_default_region,
                                 aruser:   fabric_default_user };
         f_ar.enq (ar);
         rg_beat       <= 0;
         rg_refill_err <= False;
         st_next = ST_REFILL;
         pw_perf_miss.send;
      end

      if ((rg_st == ST_IO_REQ) && rg_ddr4_ready && f_ar.notFull) begin
         let ar = AXI4_Rd_Addr { arid:     fabric_default_id,
                                 araddr:   fn_PA_to_Fabric_Addr (fn_pa (rg_io_ppn, rg_pf)),
                                 arlen:    0,
                                 arsize:   axsize_8,
                                 arburst:  axburst_incr,
                                 arlock:   fabric_default_lock,
                                 arcache:  fabric_default_arcache,
                                 arprot:   fabric_default_prot,
                                 arqos:    fabric_default_qos,
                                 arregion: fabric_default_region,
                                 aruser:   fabric_default_user };
         f_ar.enq (ar);
         st_next = ST_IO_RSP;
      end

      // ---- Flush / reset: invalidate every tag set
      if (flush_start) begin
         rg_flush_set   <= 0;
         rg_flush_reset <= f_reset_reqs.notEmpty;
         halt_io_next   = False;
         halt_exc_next  = False;
         halt_walk_next = False;
         st_next = ST_FLUSH;
      end
      if (rg_st == ST_FLUSH) begin
         ITag_CSet empty = replicate (ITag { state: CTAG_EMPTY, ctag: 0 });
         ram_state_and_ctag_cset.a.put (True, rg_flush_set, empty);
         if (rg_flush_set != fromInteger (csets_per_cache - 1))
            rg_flush_set <= rg_flush_set + 1;
         else if ((rg_flush_reset && f_reset_rsps.notFull) || ((! rg_flush_reset) && f_flush_rsps.notFull)) begin
            if (rg_flush_reset) begin f_reset_reqs.deq; f_reset_rsps.enq (?); end
            else                begin f_flush_reqs.deq; f_flush_rsps.enq (?); end
            st_next = ST_RUN;
         end
      end

      rg_pf       <= pf_next;
      rg_halt_io  <= halt_io_next;
      rg_halt_exc <= halt_exc_next;
      rg_halt_walk <= halt_walk_next;
      rg_st       <= st_next;
   endrule

   // ================================================================
   // INTERFACE

   interface IMem_IFC imem;
      // PURE register write (no submodule calls, no rule-written reads).
      method Action req (Bit #(3) f3, WordXL addr, Priv_Mode priv,
                         Bit #(1) sstatus_SUM, Bit #(1) mstatus_MXR, WordXL satp);
         rg_pc   <= addr;
         rg_want <= True;
         rg_priv <= priv;
         rg_satp <= satp;
         // reads only req-written registers
         if ((priv != rg_priv) || (satp != rg_satp)) rg_ctx_seq <= ! rg_ctx_seq;
      endmethod

      method Bool     valid          = rg_want && fv_present.valid
                                       && (rg_ctx_seq == rg_ctx_seen) && (rg_pmp_seq == rg_pmp_seen)
                                       && (rg_tlb_seq == rg_tlb_seen);
      method Bool     is_i32_not_i16 = fv_present.is32;
      method WordXL   pc             = rg_pc;
      method Instr    instr          = fv_present.instr;
      method Bool     exc            = fv_present.exc;
      method Exc_Code exc_code       = (fv_present.pf ? exc_code_INSTR_PAGE_FAULT : exc_code_INSTR_ACCESS_FAULT);
      method WordXL   tval           = fv_present.tval;
   endinterface

   interface Server server_reset;
      interface Put request;
         method Action put (Token t) if (f_reset_reqs.notFull);
            f_reset_reqs.enq (t);
         endmethod
      endinterface
      interface Get response;
         method ActionValue #(Token) get () if (f_reset_rsps.notEmpty);
            f_reset_rsps.deq;
            return ?;
         endmethod
      endinterface
   endinterface

   interface Server server_flush;
      interface Put request;
         method Action put (Token t) if (f_flush_reqs.notFull);
            f_flush_reqs.enq (t);
         endmethod
      endinterface
      interface Get response;
         method ActionValue #(Token) get () if (f_flush_rsps.notEmpty);
            f_flush_rsps.deq;
            return ?;
         endmethod
      endinterface
   endinterface

   interface mem_master = master_xactor.axi_side;

   method Action ma_ddr4_ready;
      rg_ddr4_ready <= True;
   endmethod

   method Bool mv_perf_miss = pw_perf_miss;

   method Action ma_pmp (PMP_Table t);
      rg_pmp <= t;
      if (pack (t) != pack (rg_pmp)) rg_pmp_seq <= ! rg_pmp_seq;
   endmethod

   method Action tlb_flush;
      rg_tlb_seq <= ! rg_tlb_seq;
   endmethod
endmodule

endpackage
