// Copyright (c) 2026 -- ULX3S Flute Linux program (Zkr)

package Entropy_Source;

// ================================================================
// Entropy source behind the Zkr seed CSR (0x015).
//
// seed reads {OPST[31:30], reserved[29:24] = 0, custom[23:16] = 0, entropy[15:0]}:
//   OPST 00 BIST  -- self test running, no data
//        01 WAIT  -- no data yet, poll again
//        10 ES16  -- entropy[15:0] holds 16 fresh bits
//        11 DEAD  -- unrecoverable failure
// Every seed access writes the CSR (read-only access is illegal), and the
// CSR write calls consume, so one ES16 sample is never returned twice.
//
// mkEntropy_Source_Wait is the placeholder: it never claims entropy (WAIT,
// data 0), which is legal -- software polls -- and honest. A real source
// (e.g. an ECP5 ring-oscillator sampler with health tests) replaces it
// behind the same interface.
// ================================================================

interface Entropy_Source_IFC;
   (* always_ready *) method Bit #(32) seed;      // current OPST + data
   (* always_ready *) method Action    consume;   // seed CSR access retired
endinterface

Bit #(2) opst_bist = 2'b00;
Bit #(2) opst_wait = 2'b01;
Bit #(2) opst_es16 = 2'b10;
Bit #(2) opst_dead = 2'b11;

module mkEntropy_Source_Wait (Entropy_Source_IFC);
   method Bit #(32) seed = { opst_wait, 30'b0 };
   method Action consume = noAction;
endmodule

// ================================================================
// mkEntropy_Source_RO: ring-oscillator noise (src/RO_TRNG.v) with SP 800-90B health tests.
//
//   raw bit      RO_TRNG.BIT, one per clock (Verilator/formal: an LFSR stand-in).
//   decimation   XOR of 8 consecutive raw bits -> one noise sample (non-cryptographic; Zkr
//                forbids cryptographic conditioning in seed, not decimation).
//   health       assumed min-entropy H = 0.5 per sample, alpha = 2^-20 (SP 800-90B 4.4):
//                  repetition count test: 41 identical samples in a row      -> failure
//                  adaptive proportion test: >= 793 of 1024 equal to the first -> failure
//   OPST         BIST until a 1024-sample startup window passes both tests (samples discarded);
//                then ES16 once 16 samples are collected, WAIT while collecting.  A failure
//                discards the partial word and restarts BIST; 4 failures without an ES16
//                in between -> DEAD (sticky).  consume (every seed access) clears the word.
// ================================================================

interface RO_TRNG_IFC;
   (* always_ready *) method Bit #(1) raw_bit;
endinterface

import "BVI" RO_TRNG =
module mkRO_TRNG (RO_TRNG_IFC);
   default_clock clk (CLK);
   default_reset rst (RST_N);
   method BIT raw_bit;
   schedule (raw_bit) CF (raw_bit);
endmodule

typedef 41  RCT_Cutoff;
typedef 793 APT_Cutoff;
typedef 1024 APT_Window;

module mkEntropy_Source_RO (Entropy_Source_IFC);
   RO_TRNG_IFC ro <- mkRO_TRNG;

   Reg #(Bit #(3))  rg_dec_n   <- mkReg (0);      // raw bits folded into rg_dec so far
   Reg #(Bit #(1))  rg_dec     <- mkReg (0);

   Reg #(Bit #(1))  rg_rct_v   <- mkReg (0);      // RCT: last sample and run length
   Reg #(Bit #(6))  rg_rct_n   <- mkReg (0);
   Reg #(Bit #(1))  rg_apt_v   <- mkReg (0);      // APT: reference sample, window position, matches
   Reg #(Bit #(10)) rg_apt_i   <- mkReg (0);
   Reg #(Bit #(10)) rg_apt_n   <- mkReg (0);

   Reg #(Bit #(2))  rg_opst    <- mkReg (opst_bist);
   Reg #(Bool)      rg_startup_done <- mkReg (False);   // a full APT window passed since (re)start
   Reg #(Bit #(3))  rg_fails   <- mkReg (0);
   Reg #(Bit #(16)) rg_word    <- mkReg (0);
   Reg #(Bit #(5))  rg_word_n  <- mkReg (0);       // samples in rg_word (16 = full)

   PulseWire pw_consume <- mkPulseWire;

   (* fire_when_enabled, no_implicit_conditions *)
   rule rl_step;
      Bit #(1) raw   = ro.raw_bit;
      Bit #(1) samp  = rg_dec ^ raw;
      Bool     fresh = (rg_dec_n == 7);
      rg_dec_n <= rg_dec_n + 1;
      rg_dec   <= fresh ? 0 : samp;

      Bit #(2)  opst     = rg_opst;
      Bit #(16) word     = rg_word;
      Bit #(5)  word_n   = rg_word_n;
      Bool      startup  = rg_startup_done;
      Bit #(3)  fails    = rg_fails;

      if (pw_consume && (opst == opst_es16)) begin
         opst = opst_wait; word = 0; word_n = 0;
      end

      if (fresh && (opst != opst_dead)) begin
         // repetition count test
         Bit #(6) rct_n = ((samp == rg_rct_v) && (rg_rct_n != 0)) ? rg_rct_n + 1 : 1;
         rg_rct_v <= samp;
         Bool rct_fail = (rct_n >= fromInteger (valueOf (RCT_Cutoff)));

         // adaptive proportion test (window of 1024: sample 0 is the reference)
         Bool     first  = (rg_apt_i == 0);
         Bit #(1) ref_v  = first ? samp : rg_apt_v;
         Bit #(10) apt_n = first ? 1 : ((samp == rg_apt_v) ? rg_apt_n + 1 : rg_apt_n);
         Bool apt_fail   = (zeroExtend (apt_n) >= Bit #(11) ' (fromInteger (valueOf (APT_Cutoff))));
         Bool window_end = (rg_apt_i == fromInteger (valueOf (APT_Window) - 1));
         rg_apt_v <= ref_v;

         if (rct_fail || apt_fail) begin
            fails   = fails + 1;
            opst    = (fails == 4) ? opst_dead : opst_bist;
            startup = False; word = 0; word_n = 0;
            rg_apt_i <= 0; rg_apt_n <= 0;
            rg_rct_n <= 0;
         end
         else begin
            rg_rct_n <= rct_n;
            rg_apt_i <= window_end ? 0 : rg_apt_i + 1;
            rg_apt_n <= apt_n;
            if (! startup) begin
               if (window_end) begin startup = True; opst = opst_wait; end
            end
            else if (word_n < 16) begin
               word   = { word [14:0], samp };
               word_n = word_n + 1;
               if (word_n == 16) begin opst = opst_es16; fails = 0; end
            end
         end
      end

      rg_opst <= opst; rg_word <= word; rg_word_n <= word_n;
      rg_startup_done <= startup; rg_fails <= fails;
   endrule

   method Bit #(32) seed;
      return (rg_opst == opst_es16) ? { opst_es16, 14'b0, rg_word } : { rg_opst, 30'b0 };
   endmethod

   method Action consume;
      pw_consume.send;
   endmethod
endmodule

endpackage
