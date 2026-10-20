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

// ================================================================

export ICache_Pipe_IFC (..), mkICache_Pipe;

import Vector       :: *;
import BRAMCore     :: *;
import ConfigReg    :: *;
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
   Bit #(64) d;
   } QEnt
deriving (Bits, FShow);

QEnt qent_empty = QEnt { v: False, a: 0, exc: False, d: 0 };

typedef enum { ST_RUN, ST_REFILL_REQ, ST_REFILL, ST_IO_REQ, ST_IO_RSP, ST_FLUSH } IState
deriving (Bits, Eq, FShow);

typedef struct {
   Bool      valid;
   Bool      is32;
   Instr     instr;
   Bool      exc;
   WordXL    tval;
   Bool      base_hit;    // wanted cword is q[0] or q[1]
   Bool      pop1;        // wanted cword is q[1]: q[0] is consumed
   CW_Addr   need_cw;     // when ! valid: the cword still needed
   } Present
deriving (Bits, FShow);

function WordXL fn_cw_to_addr (CW_Addr a) = { a, 3'b0 };

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
   Reg #(ITag_CSet)                     rg_s2_tags <- mkReg (replicate (ITag { state: CTAG_EMPTY, ctag: 0 }));
   Reg #(Vector #(Ways_per_CSet, CWord)) rg_s2_data <- mkReg (replicate (0));

   // Refill / device read
   Reg #(CW_Addr)        rg_miss_a     <- mkReg (0);
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

      Present r = Present { valid: False, is32: is32, instr: ?, exc: False,
                            tval: rg_pc, base_hit: m0 || m1, pop1: (! m0) && m1,
                            need_cw: w };
      if (m0 || m1) begin
         if (b.exc) begin
            r.valid = True;  r.exc = True;  r.is32 = False;
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

      // ---- Presentation-driven queue actions
      Bool pending  = (! pres.base_hit) && (w == enq_next) && (! rg_halt_exc);
      Bool redirect = rg_want && (! pres.base_hit) && (! pending) && (rg_st != ST_FLUSH);
      Bool q_clear  = rg_want && (! pres.base_hit) && (rg_st != ST_FLUSH);    // pending or redirect
      Bool q_pop1   = rg_want && pres.pop1;

      // ---- Flush / reset start (only from RUN: refills/device reads finish first)
      Bool flush_start = run && (f_reset_reqs.notEmpty || f_flush_reqs.notEmpty);

      // ---- S2: hit test (current stream only)
      PA       s2_pa  = fn_WordXL_to_PA (fn_cw_to_addr (rg_s2_a));
      CTag     s2_tag = fn_PA_to_CTag (s2_pa);
      Bool     s2_mem = soc_map.m_is_mem_addr (fn_PA_to_Fabric_Addr (s2_pa));
      Bool        hit     = False;
      Way_in_CSet hit_way = 0;
      for (Integer wy = 0; wy < ways_per_cset; wy = wy + 1)
         if ((rg_s2_tags [wy].state == CTAG_CLEAN) && (rg_s2_tags [wy].ctag == s2_tag)) begin
            hit = True;  hit_way = fromInteger (wy);
         end
      Bool s2_live = rg_s2_v && run && (! redirect) && (! flush_start);
      Bool s2_enq  = s2_live && s2_mem && hit;
      Bool s2_miss = s2_live && s2_mem && (! hit);
      Bool s2_io   = s2_live && (! s2_mem);
      Bool p2_kill = s2_miss || s2_io;

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
         m_enq = tagged Valid (QEnt { v: True, a: rg_s2_a, exc: False, d: d });
      end

      // ---- Device read / refill completion (may also enqueue)
      Bool drop = rg_stale || redirect;
      CW_Addr pf_next = rg_pf;
      Bool    halt_io_next  = rg_halt_io;
      Bool    halt_exc_next = rg_halt_exc;
      IState  st_next = rg_st;

      Bool rd_rsp_avail = f_rd.notEmpty;
      let  rd_rsp       = f_rd.first;
      Bool rd_ok        = (rd_rsp.rresp == axi4_resp_okay);

      if ((rg_st == ST_IO_RSP) && rd_rsp_avail) begin
         f_rd.deq;
         if (! drop) begin
            m_enq = tagged Valid (QEnt { v: True, a: rg_pf, exc: ! rd_ok, d: zeroExtend (rd_rsp.rdata) });
            pf_next = rg_pf + 1;
            if (! rd_ok) halt_exc_next = True;
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
                                        ctag:  fn_PA_to_CTag (fn_WordXL_to_PA (fn_cw_to_addr (rg_miss_a))) };
               ram_state_and_ctag_cset.a.put (True, cset, t);
            end
            else if (! drop) begin
               m_enq = tagged Valid (QEnt { v: True, a: rg_miss_a, exc: True, d: 0 });
               halt_exc_next = True;
               pf_next = rg_miss_a + 1;
            end
            st_next = ST_RUN;
         end
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
      if (s2_io) begin
         pf_next = rg_s2_a;
         halt_io_next = True;
      end

      // ---- Redirect: restart the stream at the wanted cword
      if (redirect) begin
         pf_next       = w;
         halt_io_next  = False;
         halt_exc_next = False;
      end

      // ---- A redirect before the AR is sent cancels the refill / device
      // read outright (nothing outstanding on the fabric).
      if (redirect && ((rg_st == ST_REFILL_REQ) || (rg_st == ST_IO_REQ)))
         st_next = ST_RUN;

      // ---- Stale marking: lives from refill/device-read start to completion
      if ((rg_st == ST_RUN) || (st_next == ST_RUN))
         rg_stale <= False;
      else if (redirect)
         rg_stale <= True;

      // ---- P0: issue.  The BRAM address/enable do not depend on the S2
      // result (a kill only drops the S1 valid bit next cycle).
      Bit #(3) n_q  = (q [0].v ? 1 : 0) + (q [1].v ? 1 : 0) + (q [2].v ? 1 : 0) + (q [3].v ? 1 : 0);
      Bit #(3) n_fl = (rg_s1_v ? 1 : 0) + (rg_s2_v ? 1 : 0);
      Bool credit   = ((n_q + n_fl) < 4);
      Bool issue    = run && rg_want && (! flush_start)
                      && (redirect || ((! rg_halt_io) && (! rg_halt_exc) && credit));
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

      // ---- Pipe advance + BRAM output capture
      rg_s1_v <= issue && (! p2_kill);
      rg_s1_a <= issue_a;
      rg_s2_v <= rg_s1_v && (! p2_kill) && (! redirect) && (! flush_start);
      rg_s2_a <= rg_s1_a;
      rg_s2_tags <= ram_state_and_ctag_cset.b.read;
      Vector #(Ways_per_CSet, CWord) dv = newVector;
      for (Integer wy = 0; wy < ways_per_cset; wy = wy + 1)
         dv [wy] = (wy == 0) ? ram_cword_set.b.read : ram_cword_set_w1.b.read;
      rg_s2_data <= dv;

      // ---- Fabric requests
      if ((rg_st == ST_REFILL_REQ) && rg_ddr4_ready && f_ar.notFull) begin
         WordXL line = fn_align_Addr_to_CLine (fn_cw_to_addr (rg_miss_a));
         let ar = AXI4_Rd_Addr { arid:     fabric_default_id,
                                 araddr:   fn_PA_to_Fabric_Addr (fn_WordXL_to_PA (line)),
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
                                 araddr:   fn_PA_to_Fabric_Addr (fn_WordXL_to_PA (fn_cw_to_addr (rg_pf))),
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
      endmethod

      method Bool     valid          = rg_want && fv_present.valid;
      method Bool     is_i32_not_i16 = fv_present.is32;
      method WordXL   pc             = rg_pc;
      method Instr    instr          = fv_present.instr;
      method Bool     exc            = fv_present.exc;
      method Exc_Code exc_code       = exc_code_INSTR_ACCESS_FAULT;
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
endmodule

endpackage
