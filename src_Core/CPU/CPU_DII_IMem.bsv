// Copyright (c) 2026 -- ULX3S Flute Linux program (TestRIG RVFI-DII)

package CPU_DII_IMem;

// ================================================================
// RVFI-DII (TestRIG direct instruction injection) fetch wrapper.
// Only compiled into RVFI_DII builds (CPU.bsv instantiates it under
// `ifdef RVFI_DII); production builds never import this package.
//
// The wrapper sits between StageF and Near_Mem's I-side (ICache_Pipe):
//
//   * Every fetch request still goes to the real I-side, so translation
//     (Sv32 ITLB / PTW), PMP and the fabric access-fault path behave exactly
//     as in the production core.  Only the instruction BITS are replaced:
//     they come from the TestRIG instruction trace, indexed by a sequence
//     number (Dii_Id), not by address.
//   * The sequence number of a fetch is supplied by the CPU with each
//     request (req_seq): sequential fetches use seq(previous) + 1; every
//     redirect (branch resolution, trap, xRET, CSRRx/FENCE restarts) uses the
//     number of instructions retired so far (= RVFI records emitted), since
//     all redirects in this pipeline happen after the redirecting
//     instruction has retired.
//   * The size (16/32-bit) is decided from the injected word, as in Sail's
//     riscv_fetch_rvfi.sail.  Fetch exceptions reported by the real I-side
//     win (Sail checks translation before using the injected word).
//
// Known approximation: for a 32-bit injected instruction at PC = 2 mod 4
// the real I-side sees zeroed memory (a 16-bit encoding) and so does not
// check the PC+2 granule; Sail does.  This matters only when PC+2 falls in
// a different page / PMP region than PC.
// ================================================================

import FIFOF        :: *;
import GetPut       :: *;

import ISA_Decls       :: *;
import Near_Mem_IFC    :: *;
import RVFI_DII_Types  :: *;

// ================================================================

interface CPU_DII_IMem_IFC;
   // The IMem view presented to StageF (replaces near_mem.imem)
   interface IMem_IFC imem;

   // Called in the same action as every imem.req: sequence number of that fetch
   (* always_ready *)
   method Action req_seq (Dii_Id ix);

   // Sequence number of the most recent request (valid with imem.valid)
   (* always_ready *)
   method Dii_Id cur_seq;

   // To the RVFI-DII bridge: which instruction is wanted, and its bits
   interface Get #(Dii_Id)                    dii_seq_req;
   interface Put #(Tuple2 #(Dii_Id, Bit #(32))) dii_instr_rsp;
endinterface

// ================================================================

module mkCPU_DII_IMem #(IMem_IFC  imem_hw) (CPU_DII_IMem_IFC);

   Reg #(Dii_Id) rg_seq <- mkReg (0);

   // Injected word for rg_seq.  Port 0: response from the bridge;
   // port 1: invalidated by a new request (and read by StageF).
   Reg #(Maybe #(Tuple2 #(Dii_Id, Bit #(32)))) crg_instr [2] <- mkCReg (2, tagged Invalid);

   function Bool fn_have_instr;
      Bool ok = False;
      if (crg_instr [1] matches tagged Valid { .s, .x })
	 ok = (s == rg_seq);
      return ok;
   endfunction

   Bit #(32) dii_word = tpl_2 (fromMaybe (?, crg_instr [1]));
   Bool      dii_i32  = (dii_word [1:0] == 2'b11);

   // ----------------------------------------------------------------

   interface IMem_IFC imem;
      method Action  req (Bit #(3) f3,
			  WordXL addr,
			  Priv_Mode  priv,
			  Bit #(1)   sstatus_SUM,
			  Bit #(1)   mstatus_MXR,
			  WordXL     satp);
	 imem_hw.req (f3, addr, priv, sstatus_SUM, mstatus_MXR, satp);
      endmethod

      // An exception from the real I-side needs no injected word.
      method Bool valid = imem_hw.valid && (imem_hw.exc || fn_have_instr);
      method Bool is_i32_not_i16 = (imem_hw.exc ? imem_hw.is_i32_not_i16 : dii_i32);
      method WordXL pc = imem_hw.pc;
      method Instr instr = (dii_i32 ? dii_word : zeroExtend (dii_word [15:0]));
      method Bool exc = imem_hw.exc;
      method Exc_Code exc_code = imem_hw.exc_code;
      method WordXL tval = imem_hw.tval;
   endinterface

   method Action req_seq (Dii_Id ix);
      rg_seq        <= ix;
      crg_instr [1] <= tagged Invalid;
   endmethod

   method Dii_Id cur_seq = rg_seq;

   interface Get dii_seq_req;
      method ActionValue #(Dii_Id) get () if (crg_instr [0] matches tagged Invalid);
	 return rg_seq;
      endmethod
   endinterface

   interface Put dii_instr_rsp;
      method Action put (Tuple2 #(Dii_Id, Bit #(32)) x);
	 crg_instr [0] <= tagged Valid x;
      endmethod
   endinterface
endmodule

// ================================================================

endpackage
