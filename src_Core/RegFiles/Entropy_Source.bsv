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

endpackage
