// Copyright (c) 2016-2021 Bluespec, Inc. All Rights Reserved

package CPU_Stage3;

// ================================================================
// This is Stage 3 of the CPU.
// It is the WB ("Write Back") stage:
// - Writes back a GPR register value (if the instr has a GPR Rd)
// - Writes back an FPR register value (if the instr has an FPR Rd)
// - Updates CSR INSTRET
//     Note: this instr cannot be a CSRRx updating INSTRET, since
//           CSRRx is done completely off-pipe.
// = Optionaally: sends a TandemVerification trace packet

// ================================================================
// Exports

export
CPU_Stage3_IFC (..),
`ifdef ISA_F
fn_stage3_sets_fs_dirty,
`endif
mkCPU_Stage3;

// ================================================================
// BSV library imports

import ConfigReg    :: *;
import FIFOF        :: *;
import GetPut       :: *;
import ClientServer :: *;

// ----------------
// BSV additional libs

import Cur_Cycle :: *;

// ================================================================
// Project imports

import ISA_Decls   :: *;
import GPR_RegFile :: *;
`ifdef ISA_F
import FPR_RegFile :: *;
`endif
import CSR_RegFile :: *;
import CPU_Globals :: *;

`ifdef INCLUDE_TANDEM_VERIF
import TV_Trace_Data :: *;
`endif

// ================================================================
// Interface

`ifdef ISA_F
// mstatus.FS becomes Dirty when this op leaves Stage3 (an FPR write, or an FBox op that accrues
// fflags). Shared by fa_deq and the RVFI record (CPU.bsv), so RVFI reports what the hardware does.
function Bool fn_stage3_sets_fs_dirty (Data_Stage2_to_Stage3 x);
`ifdef RVFI_MUTANT_NO_FS_DIRTY
   return False;    // riscv-formal-fp mutant: must FAIL the FP insn checks
`else
   return (x.upd_flags || (x.rd_valid && x.rd_in_fpr));
`endif
endfunction
`endif

interface CPU_Stage3_IFC;
   // ---- Reset
   interface Server #(Token, Token) server_reset;

   // ---- Output
   (* always_ready *)
   method Output_Stage3  out;

   (* always_ready *)
   method Action deq;

   // ---- Input
   (* always_ready *)
   method Action enq (Data_Stage2_to_Stage3 x);

   (* always_ready *)
   method Action set_full (Bool full);

`ifdef RVFI
`ifdef ISA_F
   // RVFI: fflags / FS update of the op in Stage3, applied when it leaves (fa_deq), i.e. in the same
   // cycle as the next retirement; that retirement's fcsr / mstatus read view must include it
   (* always_ready *)
   method Tuple2 #(Bool, Bit #(5)) mv_rvfi_fp_pending;
`endif
`endif

   // ---- Debugging
   method Action show_state;
endinterface

// ================================================================
// Module

module mkCPU_Stage3 #(Bit #(4)         verbosity,
		      GPR_RegFile_IFC  gpr_regfile,
`ifdef ISA_F
		      FPR_RegFile_IFC  fpr_regfile,
`endif
		      CSR_RegFile_IFC  csr_regfile)
                    (CPU_Stage3_IFC);

   FIFOF #(Token) f_reset_reqs <- mkFIFOF;
   FIFOF #(Token) f_reset_rsps <- mkFIFOF;

   // E13: ConfigRegs so rl_writeback (which reads them) can be scheduled
   // after rl_pipe (which reads the GPRs and rewrites these on enq/set_full).
   Reg #(Bool)                  rg_full   <- mkConfigReg (False);
   Reg #(Data_Stage2_to_Stage3) rg_stage3 <- mkConfigRegU;    // From Stage 2

   // ----------------------------------------------------------------
   // BEHAVIOR

   let bypass_base = Bypass {bypass_state: BYPASS_RD_NONE,
			     rd:           rg_stage3.rd,
			     // WordXL        WordXL
			     rd_val:       rg_stage3.rd_val
			     };

`ifdef ISA_F
   let fbypass_base = FBypass {bypass_state: BYPASS_RD_NONE,
			       rd:           rg_stage3.rd,
			       // WordFL        WordFL
			       rd_val:       rg_stage3.frd_val
			       };
`endif

   rule rl_reset;
      f_reset_reqs.deq;
      rg_full <= False;
      f_reset_rsps.enq (?);
   endrule

   // ----------------
   // Combinational output function

   function Output_Stage3 fv_out;
      let bypass = bypass_base;
`ifdef ISA_F
      let fbypass = fbypass_base;
      if (rg_stage3.rd_in_fpr) begin
         bypass.bypass_state = BYPASS_RD_NONE;
         fbypass.bypass_state = (rg_full && rg_stage3.rd_valid) ? BYPASS_RD_RDVAL
                                                                : BYPASS_RD_NONE;
      end
      else begin
         fbypass.bypass_state = BYPASS_RD_NONE;
         bypass.bypass_state = (rg_full && rg_stage3.rd_valid) ? BYPASS_RD_RDVAL
                                                               : BYPASS_RD_NONE;
      end
`else
      bypass.bypass_state = (rg_full && rg_stage3.rd_valid) ? BYPASS_RD_RDVAL
                                                            : BYPASS_RD_NONE;
`endif

`ifdef INCLUDE_TANDEM_VERIF
      let trace_data = rg_stage3.trace_data;
`ifdef ISA_F
      if (rg_stage3.upd_flags) begin
	 let fflags = csr_regfile.mv_update_fcsr_fflags (rg_stage3.fpr_flags);
	 trace_data = fv_trace_update_fcsr_fflags (trace_data, fflags);
      end

      if (rg_stage3.upd_flags || rg_stage3.rd_in_fpr) begin
	 let new_mstatus = csr_regfile.mv_update_mstatus_fs (fs_xs_dirty);
	 trace_data = fv_trace_update_mstatus_fs (trace_data, new_mstatus);
      end
`endif
`endif

      return Output_Stage3 {ostatus: (rg_full ? OSTATUS_PIPE : OSTATUS_EMPTY),
			    bypass : bypass
`ifdef ISA_F
			    , fbypass: fbypass
`endif
`ifdef INCLUDE_TANDEM_VERIF
			    , trace_data: trace_data
`endif
			    };
   endfunction

   // ----------------
   // Actions on 'deq': writeback Rd and update CSR INSTRET

   // E13: the regfile write is driven only by Stage3's own registers, not by
   // rl_pipe firing.  Before, write_rd sat in fa_deq, so the GPR LUTRAM write
   // enables (spread across the die) waited on the whole pipe-advance decision
   // (dcache hit -> stage2 status -> rl_pipe fire): a 22 ns, 77%-routing path.
   // Re-writing the same rd/value on a stalled cycle is idempotent, and a Stage3
   // op is never squashed (every flush path requires Stage3 EMPTY or drained),
   // so writing while full is equivalent to writing at deq.  Readers see the
   // value via the Stage3 bypass in either case.
   (* fire_when_enabled, no_implicit_conditions *)
   rule rl_writeback (rg_full && rg_stage3.rd_valid);
`ifdef ISA_F
         // Write to FPR
         if (rg_stage3.rd_in_fpr)
            fpr_regfile.write_rd (rg_stage3.rd, rg_stage3.frd_val);

         else
            // Write to GPR
            gpr_regfile.write_rd (rg_stage3.rd, rg_stage3.rd_val);
`else
         // Write to GPR in a non-FD system
         gpr_regfile.write_rd (rg_stage3.rd, rg_stage3.rd_val);
`endif

      if (verbosity > 1)
`ifdef ISA_F
            if (rg_stage3.rd_in_fpr)
               $display ("    S3.fa_deq: write FRd 0x%0h, rd_val 0x%0h",
                         rg_stage3.rd, rg_stage3.frd_val);
            else
`endif
               $display ("    S3.fa_deq: write GRd 0x%0h, rd_val 0x%0h",
                         rg_stage3.rd, rg_stage3.rd_val);

   endrule

   // ----------------
   // Actions on 'deq' (writeback moved to rl_writeback, E13)

   function Action fa_deq;
      action
`ifdef ISA_F
	 // fflags / mstatus.FS are updated once, at deq (not in rl_writeback, which
	 // re-fires every cycle Stage3 is full: a repeated fflags OR could land after a
	 // CSR write to fflags, and CSR methods conflict with rl_trap / debug CSR writes)
	 if (rg_full && rg_stage3.upd_flags)
	    csr_regfile.ma_update_fcsr_fflags (rg_stage3.fpr_flags);
	 if (rg_full && fn_stage3_sets_fs_dirty (rg_stage3))
	    csr_regfile.ma_update_mstatus_fs (fs_xs_dirty);
`else
	 noAction;
`endif
      endaction
   endfunction

   // ----------------------------------------------------------------
   // INTERFACE

   // ---- Reset
   interface server_reset = toGPServer (f_reset_reqs, f_reset_rsps);

   // ---- Output

   method Output_Stage3  out;
      return fv_out;
   endmethod

   method Action deq;
      fa_deq;
   endmethod

   // ---- Input
   method Action enq (Data_Stage2_to_Stage3 x);
      rg_stage3 <= x;

      if (verbosity > 1)
	 $display ("    S3.enq: ", fshow (x));
   endmethod

   method Action set_full (Bool full);
      rg_full <= full;
   endmethod

`ifdef RVFI
`ifdef ISA_F
   method Tuple2 #(Bool, Bit #(5)) mv_rvfi_fp_pending;
      Bool dirty = rg_full && fn_stage3_sets_fs_dirty (rg_stage3);
      Bit #(5) flags = ((rg_full && rg_stage3.upd_flags) ? rg_stage3.fpr_flags : 0);
      return tuple2 (dirty, flags);
   endmethod
`endif
`endif

   // ---- Debugging
   method Action show_state;
      if (rg_full)
	 $display ("    CPU_Stage3 state: ", fshow (rg_stage3));
      else
	 $display ("    CPU_Stage3 state: empty");
   endmethod
endmodule

// ================================================================

endpackage
