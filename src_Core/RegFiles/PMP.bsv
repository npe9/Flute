// Copyright (c) 2026 -- ULX3S Flute Linux program (PMP)

package PMP;

// ================================================================
// Physical Memory Protection: the pmpcfg / pmpaddr CSRs and a normalized,
// registered range table for the fetch and load/store checkers.
//
// Geometry: 16 entries visible (pmpcfg0-3, pmpaddr0-15); entries
// 0 .. PMP_N-1 are implemented, the rest are WARL read-only zero (OFF), a
// legal choice ("all PMP CSR fields are WARL and may be read-only zero").
// pmpcfg4-15 / pmpaddr16-63 also read zero.  Grain G = 10 (4 KiB).
//
// WARL behaviour, transcribed from the Sail model (riscv_pmp_regs.sail):
//   pmpWriteCfg : locked (L) entries ignore writes; bits 6:5 read 0;
//                 R=0 W=1 -> R=W=X=0; NA4 (not selectable with G >= 1) -> OFF.
//   pmpWriteAddr: ignored if the entry is locked, or if the NEXT entry is
//                 locked TOR (its lower bound).
//   pmpReadAddr : NAPOT -> low G-1 bits read 1; OFF / TOR -> low G bits read 0.
// Matching (riscv_pmp_control.sail pmpCheck) uses the read-back pmpaddr
// values: lowest-numbered matching entry wins; M-mode is checked only
// against locked entries and succeeds when nothing matches; S / U fail
// when nothing matches.  With a 4 KiB grain and naturally aligned
// accesses of at most 8 bytes, a partial match cannot occur, so the
// checkers compare page numbers (PA[33:12]) only.
//
// The table is recomputed into a register every cycle (one-cycle lag
// after a CSR write; CSR writes are nonpipe, so no access can observe it).
// ================================================================

import Vector :: *;

import ISA_Decls :: *;

// ================================================================

typedef 8 PMP_N;                       // implemented entries

// One normalized entry: pages [lo, hi) of a 34-bit PA space (23-bit
// bounds so a NAPOT covering all of it has hi = 2^22).
typedef struct {
   Bool      on;       // A != OFF and a non-empty range
   Bit #(23) lo;
   Bit #(23) hi;
   Bit #(1)  l;
   Bit #(1)  x;
   Bit #(1)  w;
   Bit #(1)  r;
   } PMP_Range
deriving (Bits, Eq, FShow);

typedef struct {
   Vector #(PMP_N, PMP_Range) e;
   Bool                       any_locked;
   } PMP_Table
deriving (Bits, Eq, FShow);

// Access kinds for the checker
typedef enum { PMP_R, PMP_W, PMP_RW, PMP_X } PMP_Acc deriving (Bits, Eq, FShow);    // RW: AMO (Sail ReadWrite)

// ----------------------------------------------------------------
// pmpcfg byte fields

function Bit #(1) cfg_l (Bit #(8) c) = c [7];
function Bit #(2) cfg_a (Bit #(8) c) = c [4:3];
function Bit #(1) cfg_x (Bit #(8) c) = c [2];
function Bit #(1) cfg_w (Bit #(8) c) = c [1];
function Bit #(1) cfg_r (Bit #(8) c) = c [0];

Bit #(2) a_OFF   = 2'b00;
Bit #(2) a_TOR   = 2'b01;
Bit #(2) a_NA4   = 2'b10;
Bit #(2) a_NAPOT = 2'b11;

// Sail pmpWriteCfg (for an unlocked entry)
function Bit #(8) fv_legal_cfg (Bit #(8) v);
   Bit #(8) c = v & 8'h9F;
   if ((c [1] == 1'b1) && (c [0] == 1'b0)) c = c & 8'hF8;      // R=0 W=1: RWX <- 0
   if (c [4:3] == a_NA4) c = { c [7:5], a_OFF, c [2:0] };       // G >= 1: NA4 -> OFF
   return c;
endfunction

// Sail pmpReadAddrReg with G = 10
function Bit #(32) fv_read_addr (Bit #(8) c, Bit #(32) a);
   Bit #(32) r = a;
   if (c [4] == 1'b1)  r = a | 32'h0000_01FF;                   // NAPOT: G-1 = 9 ones
   else                r = a & 32'hFFFF_FC00;                   // OFF / TOR: G = 10 zeros
   return r;
endfunction

// pmpAddrRange in 4 KiB pages (pmpaddr is PA[33:2]; page = pmpaddr[31:10])
function PMP_Range fv_range (Bit #(8) c, Bit #(32) a, Bit #(32) a_prev);
   Bit #(32) ra   = fv_read_addr (c, a);
   Bit #(23) lo   = 0;
   Bit #(23) hi   = 0;
   if (cfg_a (c) == a_TOR) begin
      lo = zeroExtend (fv_read_addr (8'h08, a_prev) [31:10]);   // prev entry's read-back value as TOR/OFF base
      hi = zeroExtend (ra [31:10]);
   end
   else if (cfg_a (c) == a_NAPOT) begin
      // mask = ra ^ (ra + 1) in 34 bits: all-ones pmpaddr is a 2^33-unit (2^35-byte) region,
      // whose end does not fit 33 bits.  hi is clamped to the end of the 34-bit PA space
      // (2^22 pages), which every such region covers.
      Bit #(34) ra34 = zeroExtend (ra);
      Bit #(34) mask = ra34 ^ (ra34 + 1);
      Bit #(34) lo34 = ra34 & (~ mask);
      Bit #(34) hi34 = lo34 + mask + 1;
      lo = lo34 [32:10];
      hi = (hi34 [33] == 1'b1) ? 23'h40_0000 : hi34 [32:10];
   end
   Bool on = ((cfg_a (c) != a_OFF) && (lo < hi));
   return PMP_Range {on: on, lo: lo, hi: hi,
                     l: cfg_l (c), x: cfg_x (c), w: cfg_w (c), r: cfg_r (c)};
endfunction

// Page number of a PA, for fv_pmp_ok
function Bit #(22) fv_pa_page (PA pa) = truncate (pa >> 12);

// pmpCheck for a page number (PA[33:12]) at effective privilege priv
function Bool fv_pmp_ok (PMP_Table t, Bit #(22) page, Priv_Mode priv, PMP_Acc acc);
   Bit #(23) p = zeroExtend (page);
   Bool      done = False;
   Bool      ok   = (priv == m_Priv_Mode);     // no match: M succeeds, S/U fail
   for (Integer i = 0; i < valueOf (PMP_N); i = i + 1) begin
      let e = t.e [i];
      if ((! done) && e.on && (e.lo <= p) && (p < e.hi)) begin
         done = True;
         Bool rwx = case (acc)
                       PMP_R: (e.r == 1'b1);
                       PMP_W: (e.w == 1'b1);
                       PMP_RW: ((e.r == 1'b1) && (e.w == 1'b1));
                       default: (e.x == 1'b1);
                    endcase;
         ok = (((priv == m_Priv_Mode) && (e.l == 1'b0)) || rwx);
      end
   end
   return ok;
endfunction

// ================================================================

interface PMP_IFC;
   (* always_ready *)
   method Maybe #(WordXL) read (CSR_Addr a);        // Invalid: not a PMP CSR
   (* always_ready *)
   method ActionValue #(WordXL) write (CSR_Addr a, WordXL v);   // a must be a PMP CSR
   (* always_ready *)
   method PMP_Table pmp_table;
endinterface

function Bool fv_is_pmp_csr (CSR_Addr a) = ((a >= 12'h3A0) && (a <= 12'h3EF));
// 0x3A0-0x3AF pmpcfg0-15, 0x3B0-0x3EF pmpaddr0-63

// Not synthesized: 'read' is called from both CSR read ports.
module mkPMP (PMP_IFC);
   Vector #(PMP_N, Reg #(Bit #(8)))  vrg_cfg  <- replicateM (mkReg (0));   // A = OFF, L = 0 at reset
   Vector #(PMP_N, Reg #(Bit #(32))) vrg_addr <- replicateM (mkReg (0));
   Reg #(PMP_Table)                  rg_table <- mkReg (unpack (0));

   function Bit #(8)  cfg_of  (Integer i) = ((i < valueOf (PMP_N)) ? vrg_cfg  [i] : 0);
   function Bit #(32) addr_of (Integer i) = ((i < valueOf (PMP_N)) ? vrg_addr [i] : 0);

   (* fire_when_enabled, no_implicit_conditions *)
   rule rl_normalize;
      PMP_Table t = unpack (0);
      Bool lk = False;
      for (Integer i = 0; i < valueOf (PMP_N); i = i + 1) begin
         Bit #(32) prev = ((i == 0) ? 0 : vrg_addr [i-1]);
         t.e [i] = fv_range (vrg_cfg [i], vrg_addr [i], prev);
         if (cfg_l (vrg_cfg [i]) == 1'b1) lk = True;
      end
      t.any_locked = lk;
      rg_table <= t;
   endrule

   method Maybe #(WordXL) read (CSR_Addr a);
      Maybe #(WordXL) r = tagged Invalid;
      if (a [11:4] == 8'h3A) begin                       // pmpcfgN: entries 4N .. 4N+3
         Bit #(32) w = 0;
         for (Integer n = 0; n < 4; n = n + 1)
            if (a [3:0] == fromInteger (n))
               w = { cfg_of (4*n+3), cfg_of (4*n+2), cfg_of (4*n+1), cfg_of (4*n) };
         r = tagged Valid w;
      end
      else if ((a [11:6] == 6'b0011_10) || (a [11:6] == 6'b0011_11)) begin   // pmpaddr0-63
         Bit #(32) w = 0;
         for (Integer i = 0; i < valueOf (PMP_N); i = i + 1)
            if (a == (12'h3B0 + fromInteger (i)))
               w = fv_read_addr (vrg_cfg [i], vrg_addr [i]);
         r = tagged Valid w;
      end
      return r;
   endmethod

   method ActionValue #(WordXL) write (CSR_Addr a, WordXL v);
      Bit #(32) nv = 0;
      if (a [11:4] == 8'h3A) begin
         for (Integer n = 0; n < 4; n = n + 1)
            if (a [3:0] == fromInteger (n))
               for (Integer k = 0; k < 4; k = k + 1) begin
                  Integer i = 4*n + k;
                  if (i < valueOf (PMP_N)) begin
                     Bit #(8) c = vrg_cfg [i];
                     Bit #(8) c_new = ((cfg_l (c) == 1'b1) ? c : fv_legal_cfg (v [8*k+7 : 8*k]));
                     vrg_cfg [i] <= c_new;
                     nv [8*k+7 : 8*k] = c_new;
                  end
               end
      end
      else begin
         for (Integer i = 0; i < valueOf (PMP_N); i = i + 1)
            if (a == (12'h3B0 + fromInteger (i))) begin
               Bool locked     = (cfg_l (vrg_cfg [i]) == 1'b1);
               Bool tor_locked = ((i + 1 < valueOf (PMP_N))
                                  && (cfg_l (vrg_cfg [i+1]) == 1'b1)
                                  && (cfg_a (vrg_cfg [i+1]) == a_TOR));
               Bit #(32) a_new = ((locked || tor_locked) ? vrg_addr [i] : v);
               vrg_addr [i] <= a_new;
               nv = fv_read_addr (vrg_cfg [i], a_new);
            end
      end
      return nv;
   endmethod

   method PMP_Table pmp_table = rg_table;
endmodule

endpackage
