// Copyright (c) 2016-2023 Bluespec, Inc. All Rights Reserved

package CPU;

// ================================================================
// This is the "Flute_V3" CPU, implementing the RISC-V ISA.
// - RV32/64, ACDFIMSU, 5-stage in order pipeline with branch prediction.
// - Optional Debug Module connection
// - Optional Tandem Verification connection.

`ifdef EXTERNAL_DEBUG_MODULE
`undef INCLUDE_GDB_CONTROL
`endif

// ================================================================
// Exports

export mkCPU;

// ================================================================
// BSV library imports

import FIFOF        :: *;
import SpecialFIFOs :: *;
import GetPut       :: *;
import ClientServer :: *;
import Connectable  :: *;
import ConfigReg    :: *;
import DReg         :: *;

// ----------------
// BSV additional libs

import GetPut_Aux :: *;
import Semi_FIFOF :: *;

// ================================================================
// Project imports

import AXI4_Types :: *;

`ifdef INCLUDE_DMEM_SLAVE
import AXI4_Lite_Types :: *;
`endif

import ISA_Decls :: *;

import PC_Trace      :: *;
import TV_Trace_Data :: *;

import GPR_RegFile :: *;
`ifdef ISA_F
import FPR_RegFile :: *;
`endif
import CSR_RegFile :: *;
import CPU_Globals :: *;
import CPU_IFC     :: *;

`ifdef ISA_C
// 'C' extension (16b compressed instructions)
import CPU_Fetch_C  :: *;
`endif

import CPU_StageF :: *;    // Fetch
import CPU_StageD :: *;    // Decode
import CPU_Stage1 :: *;    // Execute
import CPU_Stage2 :: *;    // Memory and long-latency ops
import CPU_Stage3 :: *;    // Writeback

import Near_Mem_IFC :: *;    // Caches or TCM

`ifdef Near_Mem_Caches
import Near_Mem_Caches :: *;
`endif

`ifdef Near_Mem_TCM
import Near_Mem_TCM :: *;
`endif

`ifdef INCLUDE_GDB_CONTROL
import Debug_Module   :: *;
import DM_CPU_Req_Rsp :: *;
`endif

// System address map and pc_reset value
import SoC_Map :: *;

// ================================================================
// Major States of CPU

typedef enum {CPU_RESET1,
	      CPU_RESET2,

`ifdef INCLUDE_GDB_CONTROL
	      CPU_GDB_PAUSING,      // On GDB breakpoint, while waiting for fence completion
`endif
	      CPU_DEBUG_MODE,       // Stopped (normally for debugger)
	      CPU_RUNNING,          // Normal operation
	      CPU_TRAP,
	      CPU_START_TRAP_HANDLER,
	      CPU_CSRRW_2,
	      CPU_CSRR_S_or_C_2,
	      CPU_CSRRX_RESTART,    // Restart pipe after a CSRRX instruction
	      CPU_FENCE_I,          // While waiting for FENCE.I to complete in Near_Mem
	      CPU_FENCE,            // While waiting for FENCE to complete in Near_Mem
	      CPU_SFENCE_VMA,       // While waiting for FENCE.VMA to complete in Near_Mem

	      CPU_WFI_PAUSED        // On WFI pause
   } CPU_State
deriving (Eq, Bits, FShow);

function Bool fn_is_running (CPU_State  cpu_state);
   return (   (cpu_state != CPU_RESET1)
	   && (cpu_state != CPU_RESET2)
`ifdef INCLUDE_GDB_CONTROL
	   && (cpu_state != CPU_GDB_PAUSING)
	   && (cpu_state != CPU_DEBUG_MODE)
`endif
	   );
endfunction

// ================================================================

(* synthesize *)
module mkCPU (CPU_IFC);

   // ----------------
   // System address map and pc reset value
   SoC_Map_IFC  soc_map  <- mkSoC_Map;

   // ----------------
   // General purpose registers and CSRs
   GPR_RegFile_IFC  gpr_regfile  <- mkGPR_RegFile;
`ifdef ISA_F
   FPR_RegFile_IFC  fpr_regfile  <- mkFPR_RegFile;
`endif

   CSR_RegFile_IFC  csr_regfile  <- mkCSR_RegFile;

   // ----------------
   // Some commonly used CSR values
   let mcycle   = csr_regfile.read_csr_mcycle;
   let mstatus  = csr_regfile.read_mstatus;
   let misa     = csr_regfile.read_misa;
   let minstret = csr_regfile.read_csr_minstret;

   // MSTATUS.MXR and SSTATUS.SUM for Virtual Memory access control
   Bit #(1) mstatus_MXR = mstatus [19];
`ifdef ISA_PRIV_S
   Bit #(1) sstatus_SUM = (csr_regfile.read_sstatus) [18];
`else
   Bit #(1) sstatus_SUM = 0;
`endif

   // ----------------
   // Near mem (caches or TCM, for example)
   Near_Mem_IFC  near_mem <- mkNear_Mem;

   // Take imem as is from near_mem, or use wrapper for 'C' extension
`ifdef ISA_C
   // E20: the I-side (ICache_Pipe) presents 16/32-bit instructions itself.
   IMem_IFC imem = near_mem.imem;
`else
   IMem_IFC imem = near_mem.imem;
`endif

   // ----------------
   // For debugging

   // Verbosity: 0=quiet; 1=instruction trace; 2=more detail
   Reg #(Bit #(4))  cfg_verbosity <- mkConfigReg (0);

   // Verbosity is 0 as long as # of instrs retired is <= cfg_logdelay
   Reg #(Bit #(64))  cfg_logdelay <- mkConfigReg (0);

   // Current verbosity, taking into account log delay
   Bit #(4)  cur_verbosity = ((minstret < cfg_logdelay) ? 0 : cfg_verbosity);

   // ----------------
   // Major CPU states
   Reg #(CPU_State)  rg_state    <- mkReg (CPU_RESET1);
   Reg #(Priv_Mode)  rg_cur_priv <- mkReg (m_Priv_Mode);
   Reg #(Epoch)      rg_epoch    <- mkRegU;

   // These regs save info on a trap in Stage1 or Stage2
   Reg #(Trap_Info)  rg_trap_info       <- mkRegU;
   Reg #(Bool)       rg_trap_interrupt  <- mkRegU;
   Reg #(Instr)      rg_trap_instr      <- mkRegU;
`ifdef INCLUDE_TANDEM_VERIF
   Reg #(Trace_Data) rg_trap_trace_data <- mkRegU;
`endif

   // rg_next_pc is used for redirections (branches, non-pipe
   // instructions, traps, interrupts)
   Reg #(WordXL) rg_next_pc <- mkRegU;

   // Save CSR info in CSRRx istrs to handle in separate rules
   Reg #(WordXL) rg_csr_pc   <- mkRegU;
   Reg #(WordXL) rg_csr_val1 <- mkRegU;

`ifdef RVFI
   // ================================================================
   // RVFI (riscv-formal) retirement record.  One channel; written at every
   // retirement site (the same sites that emit Trace_Data for tandem
   // verification).  rg_rvfi_valid is a DReg: a one-cycle pulse per retired
   // instruction.  Production builds (RVFI undefined) are unaffected.
   Reg #(Bool)      rg_rvfi_valid     <- mkDReg (False);
   Reg #(Bit #(64)) rg_rvfi_order     <- mkReg (0);
   Reg #(Bit #(32)) rg_rvfi_insn      <- mkRegU;
   Reg #(Bool)      rg_rvfi_trap      <- mkRegU;
   Reg #(Bool)      rg_rvfi_intr      <- mkRegU;
   Reg #(Bit #(2))  rg_rvfi_mode      <- mkRegU;
   Reg #(Bit #(5))  rg_rvfi_rs1_addr  <- mkRegU;
   Reg #(Bit #(5))  rg_rvfi_rs2_addr  <- mkRegU;
   Reg #(WordXL)    rg_rvfi_rs1_rdata <- mkRegU;
   Reg #(WordXL)    rg_rvfi_rs2_rdata <- mkRegU;
   Reg #(Bit #(5))  rg_rvfi_rd_addr   <- mkRegU;
   Reg #(WordXL)    rg_rvfi_rd_wdata  <- mkRegU;
   Reg #(WordXL)    rg_rvfi_pc_rdata  <- mkRegU;
   Reg #(WordXL)    rg_rvfi_pc_wdata  <- mkRegU;
   Reg #(WordXL)    rg_rvfi_mem_addr  <- mkRegU;
   Reg #(Bit #(4))  rg_rvfi_mem_rmask <- mkRegU;
   Reg #(Bit #(4))  rg_rvfi_mem_wmask <- mkRegU;
   Reg #(WordXL)    rg_rvfi_mem_rdata <- mkRegU;
   Reg #(WordXL)    rg_rvfi_mem_wdata <- mkRegU;
   // the next retirement is the first instruction of a trap/interrupt handler
   Reg #(Bool)      rg_rvfi_intr_pending <- mkReg (False);
   // size / compressed word of the instruction parked in rg_trap_instr
   Reg #(Bool)      rg_rvfi_trap_is_i32  <- mkRegU;
   Reg #(WordXL) rg_rvfi_trap_rs1_val <- mkRegU;   // rs values of a trapping op (riscv-formal judges misaligned-address traps from rs1)
   Reg #(WordXL) rg_rvfi_trap_rs2_val <- mkRegU;
   Reg #(Instr_C)   rg_rvfi_trap_instr_C <- mkRegU;

   // Which source registers an (expanded) instruction actually reads, so the
   // riscv-formal reg check does not compare immediate bits as rs fields.
   function Tuple2 #(Bit #(5), Bit #(5)) fn_rvfi_rs_addrs (Instr instr);
      let op = instr_opcode (instr);
      Bool uses_rs1 = (   (op == op_JALR) || (op == op_LOAD)  || (op == op_OP_IMM)
		       || (op == op_BRANCH) || (op == op_STORE) || (op == op_OP)
		       || (op == op_AMO)
		       || ((op == op_SYSTEM) && (instr_funct3 (instr) != 0) && (instr [14] == 1'b0)));
      Bool uses_rs2 = ((op == op_BRANCH) || (op == op_STORE) || (op == op_OP) || (op == op_AMO));
      return tuple2 ((uses_rs1 ? instr_rs1 (instr) : 0), (uses_rs2 ? instr_rs2 (instr) : 0));
   endfunction

   function Action fa_rvfi (Bool trap, Bool set_intr_pending,
			    Bool is_i32, Instr instr, Instr_C instr_C,
			    Bit #(5) rs1, Bit #(5) rs2, WordXL rs1_val, WordXL rs2_val,
			    Bit #(5) rd, WordXL rd_val,
			    WordXL pc, WordXL next_pc,
			    WordXL mem_addr, Bit #(4) rmask, Bit #(4) wmask, WordXL mem_rdata, WordXL mem_wdata);
      action
	 rg_rvfi_valid     <= True;
	 rg_rvfi_order     <= rg_rvfi_order + 1;
	 rg_rvfi_insn      <= (is_i32 ? instr : zeroExtend (instr_C));
	 rg_rvfi_trap      <= trap;
	 rg_rvfi_intr      <= rg_rvfi_intr_pending;
	 rg_rvfi_intr_pending <= set_intr_pending;
	 rg_rvfi_mode      <= rg_cur_priv;
	 rg_rvfi_rs1_addr  <= rs1;
	 rg_rvfi_rs2_addr  <= rs2;
	 rg_rvfi_rs1_rdata <= ((rs1 == 0) ? 0 : rs1_val);
	 rg_rvfi_rs2_rdata <= ((rs2 == 0) ? 0 : rs2_val);
	 rg_rvfi_rd_addr   <= rd;
	 rg_rvfi_rd_wdata  <= ((rd == 0) ? 0 : rd_val);
	 rg_rvfi_pc_rdata  <= pc;
	 rg_rvfi_pc_wdata  <= next_pc;
	 rg_rvfi_mem_addr  <= mem_addr;
	 rg_rvfi_mem_rmask <= rmask;
	 rg_rvfi_mem_wmask <= wmask;
	 rg_rvfi_mem_rdata <= mem_rdata;
	 rg_rvfi_mem_wdata <= mem_wdata;
      endaction
   endfunction

   // pipe-op retirement (the 2->3 move in rl_pipe)
   function Action fa_rvfi_pipe (Data_Stage2_to_Stage3 d3);
      action
	 match { .rs1, .rs2 } = fn_rvfi_rs_addrs (d3.instr);
	 fa_rvfi (False, False, d3.rvfi_is_i32, d3.instr, d3.rvfi_instr_C,
		  rs1, rs2, d3.rvfi_rs1_val, d3.rvfi_rs2_val,
		  (d3.rd_valid ? d3.rd : 0), d3.rd_val,
		  d3.pc, d3.rvfi_next_pc,
		  d3.rvfi_mem_addr, d3.rvfi_mem_rmask, d3.rvfi_mem_wmask, d3.rvfi_mem_rdata, d3.rvfi_mem_wdata);
      endaction
   endfunction
`endif

   // Save sstatus_SUM and mstatus_MXR to initiate fetches on an external
   // interrupt
   Reg #(Bit #(1)) rg_sstatus_SUM <- mkRegU;
   Reg #(Bit #(1)) rg_mstatus_MXR <- mkRegU;

   // ----------------
   // Pipeline stages

   CPU_Stage3_IFC stage3 <- mkCPU_Stage3 (cur_verbosity,
					  gpr_regfile,
`ifdef ISA_F
					  fpr_regfile,
`endif
					  csr_regfile);

   CPU_Stage2_IFC stage2 <- mkCPU_Stage2 (cur_verbosity, csr_regfile, near_mem.dmem);

   CPU_Stage1_IFC  stage1 <- mkCPU_Stage1 (cur_verbosity,
					   gpr_regfile,
					   stage2.out.bypass,
					   stage3.out.bypass,
`ifdef ISA_F
					   fpr_regfile,
					   stage2.out.fbypass,
					   stage3.out.fbypass,
`endif
					   csr_regfile,
					   rg_epoch,
					   rg_cur_priv);

   CPU_StageD_IFC  stageD <- mkCPU_StageD (cur_verbosity, misa);

   CPU_StageF_IFC  stageF <- mkCPU_StageF (cur_verbosity, imem);

   // ----------------
   // Interrupt pending based on current priv, mstatus.ie, mie and mip registers

   // E1 (2026-10-16): registered interrupt-pending bridge.
   // csr_regfile.interrupt_pending is a priority encoder over
   // mip/mie/mstatus/misa; read combinationally it sits inside `halting`,
   // i.e. inside rl_pipe's CAN_FIRE and therefore on every stage-register
   // enable.  Register it (and the cause) once per cycle; rl_stage1_interrupt
   // uses the registered cause so a source that clears in the one-cycle
   // window cannot yield exc_code 0.  Cost: +1 cycle interrupt latency.
   // Priv changes (trap / xRET / CSRRx restart) pass through >= 1
   // non-RUNNING cycle before rl_stage1_interrupt can fire again, so a
   // value computed with the old priv is never consumed.
   Reg #(Maybe #(Exc_Code)) rg_int_pending <- mkConfigReg (tagged Invalid);   // ConfigReg: readers (incl. rl_stage1_xRET via halting) must not be ordered against the bridge
   Reg #(Bool)              rg_nmi_pending <- mkConfigReg (False);
   // E3: same bridge for the WFI wake condition ((mip & mie) != 0), which
   // otherwise sits comb in rl_WFI_resume's guard -> rg_state chain.
   Reg #(Bool)              rg_wfi_resume  <- mkConfigReg (False);

   (* no_implicit_conditions, fire_when_enabled *)
   rule rl_int_pending_bridge;
      rg_int_pending <= csr_regfile.interrupt_pending (rg_cur_priv);
      rg_nmi_pending <= csr_regfile.nmi_pending;
      rg_wfi_resume  <= csr_regfile.wfi_resume;
   endrule

   Bool interrupt_pending = (isValid (rg_int_pending) || rg_nmi_pending);

   // ----------------
   // Reset requests and responses

   FIFOF #(Bool)  f_reset_reqs <- mkFIFOF;
   FIFOF #(Bool)  f_reset_rsps <- mkFIFOF;

   // ----------------
   // Performance-monitoring pulses (consumed by Core/SoC_Info counters)
   PulseWire pw_perf_retire   <- mkPulseWire;
   PulseWire pw_perf_redirect <- mkPulseWire;

   // ----------------
   // RLD-1'': registered performance-tap bridges.
   //
   // Stock fed SoC_Info's counters directly from pw_perf_retire /
   // pw_perf_redirect PulseWires.  A PulseWire read carries the
   // sender's WILL_FIRE cone, and rl_pipe's CAN_FIRE contains stage1's
   // output function (via the pipe_is_empty / stage1_halted / halting
   // guards), so the event-to-counter path was a ~30 ns cross-module
   // cone (stage1 reg -> pw -> SoC_Top -> SoC_Info -> counter D) and
   // became the binding path once the bp_train cone was cut (RLD-1').
   //
   // Retire: minstret increments exactly on the mv_23 event, so a
   // delayed-copy delta detects retires with a register compare only,
   // no rl_pipe cone at all.
   //
   // Redirect: no architectural proxy; stage through a 1-cycle tick
   // register inside the CPU so the cone ends at a local FF.
   Reg  #(Bit #(64)) rg_minstret_d1        <- mkReg (0);
   Reg  #(Bool)      rg_perf_redirect_tick <- mkReg (False);

   (* no_implicit_conditions, fire_when_enabled *)
   rule rl_perf_tap_bridge;
      rg_minstret_d1        <= minstret;
      rg_perf_redirect_tick <= pw_perf_redirect;
   endrule

   // ----------------
   // Communication to/from External debug module

`ifdef INCLUDE_GDB_CONTROL

   // Debugger run-control
   FIFOF #(Bool)  f_run_halt_reqs <- mkFIFOF;
   FIFOF #(Bool)  f_run_halt_rsps <- mkFIFOF;

   // Stop-request from debugger (e.g., GDB ^C or Dsharp 'stop')
   Reg #(Bool) rg_stop_req <- mkReg (False);

   // Count instrs after step-request from debugger (via dcsr.step)
   Reg #(Bit #(1))  rg_step_count <- mkReg (0);

   // Debugger GPR read/write request/response
   FIFOF #(DM_CPU_Req #(5,  XLEN)) f_gpr_reqs <- mkFIFOF;
   FIFOF #(DM_CPU_Rsp #(XLEN))     f_gpr_rsps <- mkFIFOF;

`ifdef ISA_F
   // Debugger FPR read/write request/response
   FIFOF #(DM_CPU_Req #(5,  FLEN)) f_fpr_reqs <- mkFIFOF;
   FIFOF #(DM_CPU_Rsp #(FLEN))     f_fpr_rsps <- mkFIFOF;
`endif

   // Debugger CSR read/write request/response
   FIFOF #(DM_CPU_Req #(12, XLEN)) f_csr_reqs <- mkFIFOF;
   FIFOF #(DM_CPU_Rsp #(XLEN))     f_csr_rsps <- mkFIFOF;

`endif

   // ----------------
   // PC trace output

`ifdef INCLUDE_PC_TRACE
   FIFOF #(PC_Trace) f_pc_trace  <- mkFIFOF;
`endif

   // ----------------
   // Tandem Verification

`ifdef INCLUDE_TANDEM_VERIF
   FIFOF #(Trace_Data) f_trace_data  <- mkFIFOF;

   // State for deciding if a MIP update needs to be sent into the trace file
   Reg #(WordXL) rg_prev_mip <- mkReg (0);
`endif

   function Bool mip_cmd_needed ();
`ifdef INCLUDE_TANDEM_VERIF
      // If the MTIP, MSIP, or xEIP bits of MIP have changed, then send a MIP update
      WordXL new_mip = csr_regfile.csr_mip_read;
      Bool mip_has_changed = (new_mip != rg_prev_mip);
      return mip_has_changed;
`else
      return False;
`endif
   endfunction: mip_cmd_needed

   // ================================================================
   // Debugging: print instruction trace info

   Reg #(Bit #(64)) rg_instret_reported <- mkReg ('1);
   Reg #(WordXL)    rg_pc_reported      <- mkReg ('1);

   function Action fa_emit_instr_trace (Bit #(64) instret, WordXL pc, Instr instr, Priv_Mode priv);
      action
	 // Display info if verbosity
	 // or: every 0x10_0000 instrs
	 //     (but not if in fetch-trap loop, where instret/pc do not change)

	 if ((cur_verbosity >= 1)
	     || (   ((instret & 'h_F_FFFF) == 0)
		 && (   (instret != rg_instret_reported)
		     || (pc      != rg_pc_reported))))
	    $display ("instret:%0d  PC:0x%0h  instr:0x%0h  priv:%0d",
		      instret, pc, instr, priv);

	 rg_instret_reported <= instret;
	 rg_pc_reported      <= pc;

`ifdef INCLUDE_PC_TRACE
	 let pc_trace = PC_Trace {cycle:   mcycle,
				  instret: instret,
				  pc:      zeroExtend (pc)};
	 f_pc_trace.enq (pc_trace);
`endif
      endaction
   endfunction

   // ================================================================
   // CPI measurement in each 'run' (from Debug Mode pause to Debug Mode pause)

   Reg #(Bit #(64))  rg_start_CPI_cycles <- mkRegU;
   Reg #(Bit #(64))  rg_start_CPI_instrs <- mkRegU;

   function Action fa_report_CPI;
      action
	 Bit #(64) delta_CPI_cycles = mcycle - rg_start_CPI_cycles;
	 Bit #(64) delta_CPI_instrs = minstret - rg_start_CPI_instrs;

	 // Make delta_CPI_instrs at least 1, to avoid divide-by-zero
	 if (delta_CPI_instrs == 0)
	    delta_CPI_instrs = delta_CPI_instrs + 1;

	 // Report CPI to 1 decimal place.
	 let x = (delta_CPI_cycles * 10) / delta_CPI_instrs;
	 let cpi     = x / 10;
	 let cpifrac = x % 10;
	 $display ("    CPI: %0d.%0d = (%0d/%0d) since last 'continue'",
		   cpi, cpifrac, delta_CPI_cycles, delta_CPI_instrs);
      endaction
   endfunction

   // ================================================================
   // Actions to restart from Debug Mode (e.g., GDB 'continue' after a breakpoint)
   // We re-initialize CPI_instrs and CPI_cycles.

   function Action fa_stageF_redirect (Addr new_pc);
      action
	 // Update epoch
	 let new_epoch = rg_epoch + 1;
	 rg_epoch     <= new_epoch;

	 stageF.enq (new_epoch,
		     new_pc,
		     rg_cur_priv,
		     sstatus_SUM,
		     mstatus_MXR,
		     csr_regfile.read_satp);
	 stageF.set_full (True);
	 rg_state <= CPU_RUNNING;

	 if (cur_verbosity > 1)
	    $display ("    fa_stageF_redirect: minstret:%0d  new_pc:%0x  cur_priv:%0d, epoch %0d->%0d",
		      minstret, new_pc, rg_cur_priv, rg_epoch, new_epoch);
      endaction
   endfunction

   function Action fa_restart_from_halt (Addr resume_pc);
      action
	 stage3.set_full (False);
	 stage2.set_full (False);
	 stage1.set_full (False);
	 stageD.set_full (False);

	 fa_stageF_redirect (resume_pc);

	 rg_start_CPI_cycles <= mcycle;
	 rg_start_CPI_instrs <= minstret;
      endaction
   endfunction

   // ================================================================
   // Debug tracing: show pipe state

   (* no_implicit_conditions, fire_when_enabled *)
   rule rl_show_pipe (   (cur_verbosity > 1)
		      && fn_is_running (rg_state)
		      && (rg_state != CPU_WFI_PAUSED));
      $display ("================================================================");
      $display ("%0d: Pipeline State:  minstret:%0d  cur_priv:%0d  mstatus:%0x  epoch:%0d",
		mcycle, minstret, rg_cur_priv, mstatus, rg_epoch);
      $display ("    ", fshow_mstatus (misa, mstatus));

      $display ("    Stage3: ", fshow (stage3.out));
      $display ("        Bypass  to Stage1: ", fshow (stage3.out.bypass));
`ifdef ISA_F
      $display ("        FBypass to Stage1: ", fshow (stage3.out.fbypass));
`endif
      $display ("    Stage2: pc 0x%08h instr 0x%08h priv %0d",
		stage2.out.data_to_stage3.pc,
		stage2.out.data_to_stage3.instr,
		stage2.out.data_to_stage3.priv);
      $display ("        ", fshow (stage2.out));
      $display ("        Bypass  to Stage1: ", fshow (stage2.out.bypass));
`ifdef ISA_F
      $display ("        FBypass to Stage1: ", fshow (stage2.out.fbypass));
`endif

      $display ("    Stage1: pc 0x%08h instr 0x%08h priv %0d",
		stage1.out.data_to_stage2.pc,
		stage1.out.data_to_stage2.instr,
		stage1.out.data_to_stage2.priv);
      $display ("        ", fshow (stage1.out));

      $display ("    StageD: pc 0x%08h instr 0x%08h priv %0d epoch %0d",
		stageD.out.data_to_stage1.pc,
		stageD.out.data_to_stage1.instr,
		stageD.out.data_to_stage1.priv,
		stageD.out.data_to_stage1.epoch);
      $display ("        ", fshow (stageD.out));

      $display ("    StageF: pc 0x%08h instr 0x%08h priv %0d epoch %0d",
		stageF.out.data_to_stageD.pc,
		stageF.out.data_to_stageD.instr,
		stageF.out.data_to_stageD.priv,
		stageF.out.data_to_stageD.epoch);
      $display ("        ", fshow (stageF.out));
      $display ("----------------");
   endrule

   // ================================================================
   // Reset

   Reg #(Bool) rg_run_on_reset <- mkReg (False);

   rule rl_reset_start (rg_state == CPU_RESET1);
      let run_on_reset <- pop (f_reset_reqs);
      rg_run_on_reset <= run_on_reset;

      $display ("================================================================");
      $write   ("CPU: Bluespec  RISC-V  Flute  v3.0");
      if (rv_version == RV32)
	 $display (" (RV32)");
      else
	 $display (" (RV64)");
      $display ("Copyright (c) 2016-2023 Bluespec, Inc. All Rights Reserved.");
      $display ("================================================================");

      gpr_regfile.server_reset.request.put (?);
`ifdef ISA_F
      fpr_regfile.server_reset.request.put (?);
`endif
      csr_regfile.server_reset.request.put (?);
      near_mem.server_reset.request.put (?);

      stageF.server_reset.request.put (?);
      stageD.server_reset.request.put (?);
      stage1.server_reset.request.put (?);
      stage2.server_reset.request.put (?);
      stage3.server_reset.request.put (?);

      rg_cur_priv <= m_Priv_Mode;
      rg_state    <= CPU_RESET2;
      rg_epoch    <= 0;

`ifdef INCLUDE_GDB_CONTROL
      rg_stop_req   <= False;
      rg_step_count <= 0;
`endif

`ifdef INCLUDE_TANDEM_VERIF
      let trace_data = mkTrace_RESET;
      f_trace_data.enq (trace_data);

      rg_prev_mip <= 0;
`endif
   endrule: rl_reset_start

   // ----------------

`ifdef ISA_C
   // TODO: analyze this carefully; added to resolve a blockage.
   // imem_rl_fetch_next_32b is in CPU_Fetch_C.bsv, and calls imem32.req (near_mem.imem_req).
   // fa_stageF_redirect calls stageF.enq which also calls imem.req which calls imem32.req.
   // But cond_i32_odd_fetch_next should make these rules mutually exclusive; why doesn't bsc realize this?
`endif

   rule rl_reset_complete (rg_state == CPU_RESET2);
      let ack_gpr <- gpr_regfile.server_reset.response.get;
`ifdef ISA_F
      let ack_fpr <- fpr_regfile.server_reset.response.get;
`endif
      let ack_csr <- csr_regfile.server_reset.response.get;
      let ack_nm  <- near_mem.server_reset.response.get;

      let ackF <- stageF.server_reset.response.get;
      let ackD <- stageD.server_reset.response.get;
      let ack1 <- stage1.server_reset.response.get;
      let ack2 <- stage2.server_reset.response.get;
      let ack3 <- stage3.server_reset.response.get;

      WordXL dpc = truncate (soc_map.m_pc_reset_value);

      f_reset_rsps.enq (rg_run_on_reset);

      if (rg_run_on_reset) begin
	 if (cur_verbosity != 0)
	    $display ("%0d: %m.rl_reset_complete", mcycle);
	 $display ("CPU: Restart at PC = 0x%0h", dpc);
	 fa_restart_from_halt (dpc);
      end
      else begin
	 rg_state <= CPU_DEBUG_MODE;
`ifdef INCLUDE_GDB_CONTROL
	 csr_regfile.write_dcsr_cause_priv (DCSR_CAUSE_HALTREQ, m_Priv_Mode);
	 csr_regfile.write_dpc (dpc);
`endif
	 if (cur_verbosity != 0)
	    $display ("%0d: %m.rl_reset_complete", mcycle);
	 $display ("CPU: Entering DEBUG_MODE", mcycle);
      end
   endrule: rl_reset_complete

   // ================================================================
   // Various conditions on the pipe

   Bool pipe_is_empty = (   (stage3.out.ostatus == OSTATUS_EMPTY)
			 && (stage2.out.ostatus == OSTATUS_EMPTY)
			 && (stage1.out.ostatus == OSTATUS_EMPTY)
			 && (stageD.out.ostatus == OSTATUS_EMPTY)
			 && (stageF.out.ostatus == OSTATUS_EMPTY));

   // The pipe is ready to execute a non-pipe if any stage has NONPIPE
   // and all stages downstream of that stage are EMPTY
   Bool pipe_has_nonpipe = (   (stage3.out.ostatus == OSTATUS_NONPIPE)
			    || (   (stage3.out.ostatus == OSTATUS_EMPTY)
				&& (stage2.out.ostatus == OSTATUS_NONPIPE))
			    || (   (stage3.out.ostatus == OSTATUS_EMPTY)
				&& (stage2.out.ostatus == OSTATUS_EMPTY)
				&& (stage1.out.ostatus == OSTATUS_NONPIPE)));

   // Stage 1 contains an architectural (not mis-predicted) instruction
   Bool stage1_has_arch_instr = (   (   (stage1.out.ostatus == OSTATUS_PIPE)
				     && (stage1.out.control != CONTROL_DISCARD))
				 || (stage1.out.ostatus == OSTATUS_NONPIPE));

`ifdef INCLUDE_GDB_CONTROL
   Bool stop_step_req = (   rg_stop_req
			 || rg_step_count == 1);
`else
   Bool stop_step_req = False;
`endif

   // Debugger stop and step should only happen on architectural instructions
   Bool stop_step_halt = stage1_has_arch_instr && stop_step_req;

   // Halting conditions
   Bool halting = (stop_step_halt || mip_cmd_needed || (interrupt_pending && stage1_has_arch_instr));
   // Stage1 can halt only when actually contains an instruction, downstream is
   // empty and, if a branch misprediction, StageF is able to be redirected.
   Bool stage1_halted = (   halting
			 && (   (stage1.out.ostatus == OSTATUS_PIPE)
			     || (stage1.out.ostatus == OSTATUS_NONPIPE))
			 && (   (! stage1.out.redirect)
			     || (stageF.out.ostatus != OSTATUS_BUSY))
			 && (stage2.out.ostatus == OSTATUS_EMPTY)
			 && (stage3.out.ostatus == OSTATUS_EMPTY));

   // Stage1 halt reasons, in decreasing priority order
   Bool stage1_send_mip_cmd   = stage1_halted && mip_cmd_needed;
   Bool stage1_take_interrupt = stage1_halted && (! mip_cmd_needed) && interrupt_pending && stage1_has_arch_instr;
   Bool stage1_stop           = stage1_halted && (! mip_cmd_needed) && (! (interrupt_pending && stage1_has_arch_instr));

   // ================================================================
   // Every time an instruction finishes stage 1
   //    (i.e., stage1.set_full () is invoked, Stage 1 has an architectural
   //    instruction and, if a branch misprediction, Stage F is able to be
   //    redirected)
   // this function checks if this is a 'stepped' instruction
   //    (i.e., dcsr.step is set and rg_step_count == 0)
   // If so, set rg_step_count <= 1 so the stage will halt on the next
   // architectural instruction.

   function Action fa_step_check;
      action
`ifdef INCLUDE_GDB_CONTROL
	 if (   stage1_has_arch_instr
	    && (   (! stage1.out.redirect)
		|| (stageF.out.ostatus != OSTATUS_BUSY))
	    && csr_regfile.read_dcsr_step
	    && (rg_step_count == 0)) begin

	    rg_step_count <= 1;
	 end
`endif
      endaction
   endfunction

   // ================================================================

`ifdef INCLUDE_TANDEM_VERIF
   rule rl_stage1_mip_cmd (   (rg_state == CPU_RUNNING)
			   && stage1_send_mip_cmd);
      WordXL new_mip = csr_regfile.csr_mip_read;
      rg_prev_mip <= new_mip;

      let trace_data = mkTrace_CSR_WRITE (csr_addr_mip, new_mip);
      f_trace_data.enq (trace_data);

      if (cur_verbosity > 1)
	 $display ("%0d: %m.rl_stage1_mip_cmd: MIP new 0x%0h, old 0x%0h", mcycle, new_mip, rg_prev_mip);
   endrule
`endif

   // ================================================================
   // PIPELINE BEHAVIOR (excluding nonpipe special instructions and exceptions)

   // We do not attempt to manage CSR values in the pipeline like GPRs
   // (read reg, writeback, bypassing) because of complexity: too many
   // CSRs can change simultaneously.  A CSRRx instruction in stage1
   // is stalled until downstream stages are empty. Then, we delay for
   // a cycle before restarting the pipe by re-fetching the next
   // instr, since the fetch may need the just-written CSR value.

`ifdef ISA_C
   // TODO: analyze this carefully; added to resolve a blockage
   // imem_rl_fetch_next_32b is in CPU_Fetch_C.bsv, and calls imem32.req (near_mem.imem_req).
   // fa_stageF_redirect calls stageF.enq which also calls imem.req which calls imem32.req.
   // But cond_i32_odd_fetch_next should make these rules mutually exclusive; why doesn't bsc realize this?
`endif

   rule rl_pipe (   (rg_state == CPU_RUNNING)
		 && (! pipe_is_empty)
		 && (! pipe_has_nonpipe)
		 && (! stage1_halted));

      if (cur_verbosity > 1) $display ("%0d: %m.rl_pipe", mcycle);

      Bool stage3_full = (stage3.out.ostatus != OSTATUS_EMPTY);
      Bool stage2_full = (stage2.out.ostatus != OSTATUS_EMPTY);
      Bool stage1_full = (stage1.out.ostatus != OSTATUS_EMPTY);
      Bool stageD_full = (stageD.out.ostatus != OSTATUS_EMPTY);
      Bool stageF_full = (stageF.out.ostatus != OSTATUS_EMPTY);
      Bool redirect    = False;
      // E5: BTB training is decoupled from the fetch-feed block.  cf_train is
      // the resolved cf_info of the op leaving stage2 THIS cycle (one shot);
      // train_ras marks that the feed block ran (RAS uses the fetched instr).
      CF_Info cf_train  = cf_info_none;
      Bool    train_ras = False;

      // ----------------
      // Stage3 sink (does regfile writebacks)

      if (stage3.out.ostatus == OSTATUS_PIPE) begin
	 stage3.deq; stage3_full = False;

`ifdef INCLUDE_TANDEM_VERIF
	 // To Verifier
	 let trace_data = stage3.out.trace_data;
	 f_trace_data.enq (trace_data);
`endif
      end

      // ----------------
      // Move instruction from Stage2 to Stage3

      if ((! stage3_full) && (stage2.out.ostatus == OSTATUS_PIPE)) begin
	 stage3.enq (stage2.out.data_to_stage3);  stage3_full = True;
	 stage2.deq;                              stage2_full = False;

	 // Increment csr_INSTRET.
	 // Note: this instr cannot be a CSRRx updating INSTRET, since
	 // CSRRx is done off-pipe
	 csr_regfile.csr_minstret_incr;
	 pw_perf_retire.send;
`ifdef RVFI
	 fa_rvfi_pipe (stage2.out.data_to_stage3);
`endif

	 // E2: branch resolution is applied here, when the op leaves
	 // stage2, from REGISTERED fields (was: from stage1's combinational
	 // output at the 1->2 move).  +1 cycle mispredict penalty; the
	 // wrong-path instruction in stage1 is held out of stage2 below and
	 // discarded by the epoch bump in rl_trap_fetch.
	 if (stage2.out.redirect) begin
	    rg_next_pc <= stage2.out.next_pc;
	    redirect    = True;
	    pw_perf_redirect.send;
	    // Drop the wrong-path instruction in stage1 now (it would be
	    // discarded by the epoch bump anyway).  Holding it instead keeps
	    // the D/F chain full, which skips the feed block in exactly this
	    // cycle and starves bp_train of the mispredicted branch's cf_info
	    // (predictor never learns first-seen taken branches).
	    stage1_full = False;
	 end
	 // E5: train the BTB from the op leaving stage2, whether or not the
	 // feed block runs this cycle.  (E4 consumed stage2.out.cf_info only
	 // inside the feed block; with a 2-cycle fetch the feed block runs
	 // every other cycle, so a fixed loop's branch was dropped every
	 // iteration and the BTB never learned it: 2.0M redirects vs 395 on
	 // rld4 in the J1 run, loader repair window 40 vs 26 cycles/iter.)
	 cf_train = stage2.out.cf_info;
	 fa_emit_instr_trace (minstret, stage2.out.data_to_stage3.pc, stage2.out.data_to_stage3.instr, rg_cur_priv);
      end

      // ----------------
      // Move instruction from Stage1 to Stage2, except:
      //  - Discard if in branch mispredict region
      //  - If a branch, proceed only if can redirect stage1

      if (   (! halting)
	  && (! stage2_full)
	  && (stage1.out.ostatus == OSTATUS_PIPE))
	 begin
	    if (stage1.out.control == CONTROL_DISCARD) begin
	       stage2_full = False;
	       stage1_full = False;
	       if (cur_verbosity > 1)
		  $display ("    rl_pipe: Discarding stage1 due to redirection");
	    end
	    // E2: never let the instruction behind a redirecting op enter
	    // stage2 (it is wrong-path by definition; stage2 has side effects).
	    else if (! stage2.out.redirect) begin
	       stage2.enq (stage1.out.data_to_stage2);  stage2_full = True;
	       stage1.deq;                              stage1_full = False;
	    end
	 end
	  
      // ----------------
      // Move instruction from StageD to Stage1
      if (   (! stage1_full)
	  && (stageD.out.ostatus == OSTATUS_PIPE))
	 begin
	    stage1.enq (stageD.out.data_to_stage1);  stage1_full = True;
	    stageD.deq;                              stageD_full = False;
	 end

      // E14: the StageF->StageD move, the fetch feed and RAS training are in
      // rl_front (below); only BTB training (cf_train) stays here.
      if (cf_train.cf_op != CF_None)
	 stageF.bp_train (0, True, 0, cf_train);

      // E2: the redirect state change must not depend on the feed block
      // running this cycle (the held-back stage1 instruction can keep the
      // D/F chain full).
      if (redirect)
	 rg_state <= CPU_START_TRAP_HANDLER;

      stage3.set_full (stage3_full);
      stage2.set_full (stage2_full);
      stage1.set_full (stage1_full);    fa_step_check;
   endrule: rl_pipe

   // ================================================================
   // E14: front end in its own rule.  Everything in rl_pipe is gated by
   // WILL_FIRE_rl_pipe, whose guard (pipe_has_nonpipe, stage statuses) and
   // move chain depend on the dcache response; with the fetch request inside
   // rl_pipe that put dcache-hit -> imem request / BTB address on the critical
   // path (E13 seeds 1 and 3, ~22 ns, 80% routing).  rl_front decides from
   // registers only: rg_state, StageD.can_enq (2-entry FIFO), StageF/imem
   // status and stage2.out.redirect (rg_full && rg_stage2.redirect).
   // Suppressing the feed while a redirecting op sits in stage2 matches the
   // old in-rl_pipe "! redirect" (a redirecting op in stage2 is never BUSY, so
   // it leaves stage2, and the redirect refills StageF via rl_trap_fetch).
   // Fetching while a stage1 NONPIPE op (CSRRx, xRET, FENCE*, WFI, trap) is
   // being handled is safe: each of those flushes StageD and redirects StageF.

   // Urgency: the stage1 NONPIPE rules (stop, trap, BREAK, FENCE*, xRET, WFI,
   // interrupt) run only with stage1 stalled, so StageD fills within two
   // cycles and rl_front stops (no starvation); giving them priority would put
   // stage1/stage2 status back into WILL_FIRE_rl_front.  The attributes sit
   // on those rules (bsc wants them after both definitions).
   rule rl_front (   (rg_state == CPU_RUNNING)
		  && stageD.can_enq
		  && (stageF.out.ostatus == OSTATUS_PIPE));
      let d = stageF.out.data_to_stageD;
      stageD.enq (d);
      stageF.deq;

      // Feed Stage F (RAS trained from the instruction leaving StageF)
      if (! stage2.out.redirect)
	 stageF.enq (d.epoch,
		     d.pred_pc,
		     rg_cur_priv,
		     sstatus_SUM,
		     mstatus_MXR,
		     csr_regfile.read_satp);
      else
	 stageF.set_full (False);
      stageF.bp_train_ras (d.pc, d.is_i32_not_i16, d.instr);
   endrule: rl_front

   // ================================================================
   // Stage2: nonpipe special: all stage2 nonpipe behaviors are traps

   rule rl_stage2_nonpipe (   (rg_state == CPU_RUNNING)
			   && (stage3.out.ostatus == OSTATUS_EMPTY)
			   && (stage2.out.ostatus == OSTATUS_NONPIPE));
      if (cur_verbosity > 1)
	 $display ("%0d: %m.rl_stage2_nonpipe -> CPU_TRAP", mcycle);

      // Just save relevant info and handle in next clock
      rg_trap_info       <= stage2.out.trap_info;
      rg_trap_interrupt  <= False;
      rg_trap_instr      <= stage2.out.data_to_stage3.instr;
`ifdef RVFI
      rg_rvfi_trap_is_i32  <= stage2.out.data_to_stage3.rvfi_is_i32;
      rg_rvfi_trap_rs1_val <= stage2.out.data_to_stage3.rvfi_rs1_val;
      rg_rvfi_trap_rs2_val <= stage2.out.data_to_stage3.rvfi_rs2_val;
      rg_rvfi_trap_instr_C <= stage2.out.data_to_stage3.rvfi_instr_C;
`endif
`ifdef INCLUDE_TANDEM_VERIF
      rg_trap_trace_data <= stage2.out.data_to_stage3.trace_data;
`endif

      rg_state           <= CPU_TRAP;
   endrule: rl_stage2_nonpipe

   // ================================================================
   // Stage1: nonpipe traps (except BREAKs that enter Debug Mode)

`ifdef INCLUDE_GDB_CONTROL
   Bool break_into_Debug_Mode = (   (stage1.out.trap_info.exc_code == exc_code_BREAKPOINT)
				 && csr_regfile.dcsr_break_enters_debug (rg_cur_priv));
`else
   Bool break_into_Debug_Mode = False;
`endif

   (* descending_urgency = "rl_front, rl_stage1_trap" *)
   rule rl_stage1_trap (   (rg_state == CPU_RUNNING)
			&& (! halting)
			&& (stage3.out.ostatus == OSTATUS_EMPTY)
			&& (stage2.out.ostatus == OSTATUS_EMPTY)
			&& (stage1.out.ostatus == OSTATUS_NONPIPE)
			&& (stage1.out.control == CONTROL_TRAP)
			&& (! break_into_Debug_Mode)
			&& (stageF.out.ostatus != OSTATUS_BUSY));
      if (cur_verbosity > 1) $display ("%0d: %m.rl_stage1_trap", mcycle);

      // Just save relevant info and handle in next clock
      rg_trap_info       <= stage1.out.trap_info;
      rg_trap_interrupt  <= False;
      rg_trap_instr      <= stage1.out.data_to_stage2.instr;
`ifdef RVFI
      rg_rvfi_trap_is_i32  <= stage1.out.data_to_stage2.is_i32_not_i16;
      rg_rvfi_trap_rs1_val <= stage1.out.data_to_stage2.rs1_val;
      rg_rvfi_trap_rs2_val <= stage1.out.data_to_stage2.rs2_val;
      rg_rvfi_trap_instr_C <= stage1.out.data_to_stage2.instr_C;
`endif
`ifdef INCLUDE_TANDEM_VERIF
      rg_trap_trace_data <= stage1.out.data_to_stage2.trace_data;
`endif

      rg_state           <= CPU_TRAP;
   endrule: rl_stage1_trap

   // ================================================================
   // Traps

   rule rl_trap ((rg_state == CPU_TRAP)
		 && (stageF.out.ostatus != OSTATUS_BUSY));
      let epc          = rg_trap_info.epc;
      let exc_code     = rg_trap_info.exc_code;
      let tval         = rg_trap_info.tval;
      let instr        = rg_trap_instr;
      let is_interrupt = rg_trap_interrupt;

      // Take trap, save trap information for next phase
      let trap_info <- csr_regfile.csr_trap_actions (rg_cur_priv,    // from priv
						     epc,
						     (is_interrupt && csr_regfile.nmi_pending),
						     (is_interrupt && (! csr_regfile.nmi_pending)),
						     exc_code,
						     tval);

      let next_pc    = trap_info.pc;
      let new_mstatus= trap_info.mstatus;
      let mcause     = trap_info.mcause;
      let new_priv   = trap_info.priv;

      // Save new privilege and pc for ifetch
      rg_cur_priv <= new_priv;
      rg_next_pc  <= next_pc;

      // Note old MSTATUS.MXR and SSTATUS.SUM for initiating FETCH in next phase
      rg_mstatus_MXR <= mstatus_MXR;
      rg_sstatus_SUM <= sstatus_SUM;

      rg_state <= CPU_START_TRAP_HANDLER;

      stageD.set_full (False);
      stage1.set_full (False);    fa_step_check;
      stage2.set_full (False);
`ifdef RVFI
      if (is_interrupt)
	 rg_rvfi_intr_pending <= True;
      else
	 begin
	    match { .t_rs1, .t_rs2 } = fn_rvfi_rs_addrs (instr);
	    fa_rvfi (True, True, rg_rvfi_trap_is_i32, instr, rg_rvfi_trap_instr_C,
		     t_rs1, t_rs2, rg_rvfi_trap_rs1_val, rg_rvfi_trap_rs2_val, 0, 0, epc, next_pc, 0, 0, 0, 0, 0);
	 end
`endif

      // Accounting    TODO: should traps be counted as retired insrs?
      // csr_regfile.csr_minstret_incr;

      // Tandem Verification and Debug related actions
`ifdef INCLUDE_TANDEM_VERIF
      // Trace data
      Trace_Data trace_data;
      if (is_interrupt)
	 trace_data = mkTrace_INTR (next_pc, new_priv, new_mstatus, mcause, epc, 0);
      else begin
	 trace_data = rg_trap_trace_data;
	 trace_data.op = TRACE_TRAP;
	 trace_data.pc = next_pc;
	 // trace_data.instr_sz    should already be set
	 // trace_data.instr       should already be set
	 trace_data.rd    = zeroExtend (new_priv);
	 trace_data.word1 = new_mstatus;
	 trace_data.word2 = mcause;
	 trace_data.word3 = zeroExtend (epc);
	 trace_data.word4 = tval;
      end
      f_trace_data.enq (trace_data);
`endif

      // Simulation heuristic: finish if trap back to this instr
`ifndef INCLUDE_GDB_CONTROL
      if (epc == next_pc) begin
	 $display ("%0d: %m.rl_stage1_trap: Tight infinite trap loop: pc 0x%0x instr 0x%08x", mcycle,
		   next_pc, instr);
	 fa_report_CPI;
	 $finish (0);
      end
`endif

      fa_emit_instr_trace (minstret, epc, instr, rg_cur_priv);

      // Debug
      if (cur_verbosity != 0)
	 $display ("    mcause:0x%0h  epc 0x%0h  tval:0x%0h  next_pc 0x%0h, new_priv %0d new_mstatus 0x%0h",
		   mcause, epc, tval, next_pc, new_priv, new_mstatus);
   endrule: rl_trap

   // ================================================================
   // Stage1: nonpipe special: CSRRW and CSRRWI

   rule rl_stage1_CSRR_W (   (rg_state == CPU_RUNNING)
			  && (! halting)
			  && (stage3.out.ostatus == OSTATUS_EMPTY)
			  && (stage2.out.ostatus == OSTATUS_EMPTY)
			  && (stage1.out.ostatus == OSTATUS_NONPIPE)
			  && (stage1.out.control == CONTROL_CSRR_W));

      if (cur_verbosity > 1) $display ("%0d: %m.rl_stage1_CSRR_W", mcycle);

      // Register required info and handle in next clock
      rg_csr_pc  <= stage1.out.data_to_stage2.pc;
      rg_next_pc <= stage1.out.next_pc;

`ifdef ISA_F
      // With FP, the val is always Bit #(64)
      // TODO: is this ifdef necessary? Can't we always use 'truncate'?
      rg_csr_val1 <= truncate (stage1.out.data_to_stage2.val1);
`else
      rg_csr_val1 <= stage1.out.data_to_stage2.val1;
`endif

      // In case of trap (illegal CSRRW)
      rg_trap_info      <= Trap_Info {epc:      stage1.out.data_to_stage2.pc,
                                      exc_code: exc_code_ILLEGAL_INSTRUCTION,
                                      tval:     stage1.out.trap_info.tval};
      rg_trap_interrupt <= False;
      rg_trap_instr     <= stage1.out.data_to_stage2.instr;    // Also used in successful CSSRW
`ifdef RVFI
      rg_rvfi_trap_is_i32  <= stage1.out.data_to_stage2.is_i32_not_i16;
      rg_rvfi_trap_rs1_val <= stage1.out.data_to_stage2.rs1_val;
      rg_rvfi_trap_rs2_val <= stage1.out.data_to_stage2.rs2_val;
      rg_rvfi_trap_instr_C <= stage1.out.data_to_stage2.instr_C;
`endif
`ifdef INCLUDE_TANDEM_VERIF
      rg_trap_trace_data <= stage1.out.data_to_stage2.trace_data;
`endif

      rg_state <= CPU_CSRRW_2;
   endrule: rl_stage1_CSRR_W

   // ----------------

   // E13: Stage3 writeback (only fires with Stage3 full; this rule runs with it empty)
   (* descending_urgency = "stage3_rl_writeback, rl_stage1_CSRR_W_2" *)
   rule rl_stage1_CSRR_W_2 (rg_state == CPU_CSRRW_2);
      if (cur_verbosity > 1) $display ("%0d: %m.rl_stage1_CSRR_W_2", mcycle);

      let instr    = rg_trap_instr;
      let csr_addr = instr_csr    (instr);
      let rs1      = instr_rs1    (instr);
      let funct3   = instr_funct3 (instr);
      let rd       = instr_rd     (instr);

      let rs1_val  = (  (funct3 == f3_CSRRW)
		      ? rg_csr_val1                       // CSRRW
		      : extend (rs1));                    // CSRRWI

      Bool read_not_write = False;    // CSRRW always writes the CSR
      Bool permitted = csr_regfile.access_permitted_1 (rg_cur_priv, csr_addr, read_not_write);

      if (! permitted) begin
	 rg_state <= CPU_TRAP;

	 // Debug
	 if (cur_verbosity > 1) begin
	    $display ("    Trap on CSR permissions: Rs1 %0d Rs1_val 0x%0h csr 0x%0h Rd %0d",
		      rs1, rs1_val, csr_addr, rd);
	 end
      end
      else begin
	 // Read the CSR only if Rd is not 0
	 WordXL csr_val = ?;
	 if (rd != 0) begin
	    begin
	       // Note: csr_regfile.read should become ActionValue if it acquires side effects
	       let m_csr_val = csr_regfile.read_csr (csr_addr);
	       csr_val   = fromMaybe (?, m_csr_val);
	    end
	 end

	 // Writeback to GPR file
	 let new_rd_val = csr_val;
	 gpr_regfile.write_rd (rd, new_rd_val);

	 // Writeback to CSR file
	 let csr_write_result <- csr_regfile.mav_csr_write (csr_addr, rs1_val);
	 let new_csr_val       = csr_write_result.new_csr_value;
	 let m_new_mstatus     = csr_write_result.m_new_csr_value2;

	 // Accounting
	 csr_regfile.csr_minstret_incr;

	 // Restart the pipe
	 rg_state   <= CPU_CSRRX_RESTART;
`ifdef RVFI
	 Bool rvfi_rs1_used = (funct3 [2] == 1'b0);    // register (not immediate) forms
	 fa_rvfi (False, False, rg_rvfi_trap_is_i32, instr, rg_rvfi_trap_instr_C,
		  (rvfi_rs1_used ? rs1 : 0), 0, (rvfi_rs1_used ? rg_csr_val1 : 0), 0,
		  rd, new_rd_val,
		  rg_csr_pc, rg_csr_pc + (rg_rvfi_trap_is_i32 ? 4 : 2), 0, 0, 0, 0, 0);
`endif

`ifdef INCLUDE_TANDEM_VERIF
	 // Trace data
	 let trace_data = mkTrace_CSRRX (rg_trap_trace_data.pc,
					 rg_trap_trace_data.instr_sz,
					 rg_trap_trace_data.instr,
					 rd,
					 new_rd_val,
					 True,    // updated-CSR info is valid
					 csr_addr,
					 new_csr_val,
					 isValid (m_new_mstatus),
					 fromMaybe (?, m_new_mstatus));
	 f_trace_data.enq (trace_data);
`endif

	 // Debug
	 fa_emit_instr_trace (minstret, rg_csr_pc, instr, rg_cur_priv);
	 if (cur_verbosity > 1) begin
	    $display ("    S1: write CSRRW/CSRRWI Rs1 %0d Rs1_val 0x%0h csr 0x%0h csr_val 0x%0h Rd %0d",
		      rs1, rs1_val, csr_addr, csr_val, rd);
	 end
      end
   endrule: rl_stage1_CSRR_W_2

   // ================================================================
   // Stage1: nonpipe special: CSRRS, CSRRSI, CSRRC, CSRRCI

   rule rl_stage1_CSRR_S_or_C (   (rg_state == CPU_RUNNING)
			       && (! halting)
			       && (stage3.out.ostatus == OSTATUS_EMPTY)
			       && (stage2.out.ostatus == OSTATUS_EMPTY)
			       && (stage1.out.ostatus == OSTATUS_NONPIPE)
			       && (stage1.out.control == CONTROL_CSRR_S_or_C));

      if (cur_verbosity > 1) $display ("%0d: %m.rl_stage1_CSRR_S_or_C", mcycle);

      // Register required info and handle in next clock
      rg_csr_pc  <= stage1.out.data_to_stage2.pc;
      rg_next_pc <= stage1.out.next_pc;

`ifdef ISA_F
      // With FP, the val is always Bit #(64)
      // TODO: is this ifdef necessary? Can't we always use 'truncate'?
      rg_csr_val1 <= truncate (stage1.out.data_to_stage2.val1);
`else
      rg_csr_val1 <= stage1.out.data_to_stage2.val1;
`endif

      // In case of trap (illegal CSRRW)
      rg_trap_info      <= Trap_Info {epc:      stage1.out.data_to_stage2.pc,
                                      exc_code: exc_code_ILLEGAL_INSTRUCTION,
                                      tval:     stage1.out.trap_info.tval};
      rg_trap_interrupt <= False;
      rg_trap_instr     <= stage1.out.data_to_stage2.instr;    // TODO: this is also used for successful CSRRW
`ifdef RVFI
      rg_rvfi_trap_is_i32  <= stage1.out.data_to_stage2.is_i32_not_i16;
      rg_rvfi_trap_rs1_val <= stage1.out.data_to_stage2.rs1_val;
      rg_rvfi_trap_rs2_val <= stage1.out.data_to_stage2.rs2_val;
      rg_rvfi_trap_instr_C <= stage1.out.data_to_stage2.instr_C;
`endif
`ifdef INCLUDE_TANDEM_VERIF
      rg_trap_trace_data <= stage1.out.data_to_stage2.trace_data;    // TODO: this is also used for successful CSRRW
`endif

      rg_state <= CPU_CSRR_S_or_C_2;
   endrule: rl_stage1_CSRR_S_or_C

   // ----------------

   // E13: Stage3 writeback (only fires with Stage3 full; this rule runs with it empty)
   (* descending_urgency = "stage3_rl_writeback, rl_stage1_CSRR_S_or_C_2" *)
   rule rl_stage1_CSRR_S_or_C_2 (rg_state == CPU_CSRR_S_or_C_2);
      if (cur_verbosity > 1) $display ("%0d: %m.rl_stage1_CSRR_S_or_C_2", mcycle);

      let instr    = rg_trap_instr;
      let csr_addr = instr_csr    (instr);
      let rs1      = instr_rs1    (instr);
      let funct3   = instr_funct3 (instr);
      let rd       = instr_rd     (instr);

      let rs1_val  = (  ((funct3 == f3_CSRRS) || (funct3 == f3_CSRRC))
		      ? rg_csr_val1                      // CSRRS,  CSRRC
		      : extend (rs1));                   // CSRRSI, CSRRCI

      Bool read_not_write = (rs1_val == 0);    // CSRR_S_or_C only reads, does not write CSR, if rs1_val == 0
      Bool permitted = csr_regfile.access_permitted_2 (rg_cur_priv, csr_addr, read_not_write);

      if (! permitted) begin
	 rg_state <= CPU_TRAP;

	 // Debug
	 if (cur_verbosity > 1) begin
	    $display ("    Trap on CSR permissions: Rs1 %0d Rs1_val 0x%0h csr 0x%0h Rd %0d",
		      rs1, rs1_val, csr_addr, rd);
	 end
      end
      else begin
	 // Read the CSR
	 WordXL csr_val = ?;
	 begin
	    // Note: csr_regfile.read should become ActionValue if it acquires side effects
	    let m_csr_val  = csr_regfile.read_csr (csr_addr);
	    csr_val = fromMaybe (?, m_csr_val);
	 end

	 // Writeback to GPR file
	 let new_rd_val = csr_val;
	 gpr_regfile.write_rd (rd, new_rd_val);

	 // Writeback to CSR file, but only if rs1 != 0
	 WordXL          new_csr_val = ?;
	 Maybe #(WordXL) m_new_mstatus = tagged Invalid;
	 if (rs1 != 0) begin
	    let x = (  ((funct3 == f3_CSRRS) || (funct3 == f3_CSRRSI))
		     ? (csr_val | rs1_val)                // CSRRS, CSRRSI
		     : csr_val & (~ rs1_val));            // CSRRC, CSRRCI

	    let csr_write_result <- csr_regfile.mav_csr_write (csr_addr, x);
	    new_csr_val           = csr_write_result.new_csr_value;
	    m_new_mstatus         = csr_write_result.m_new_csr_value2;
	 end

	 // Accounting
	 csr_regfile.csr_minstret_incr;

	 // Restart the pipe
	 rg_state   <= CPU_CSRRX_RESTART;
`ifdef RVFI
	 Bool rvfi_rs1_used = (funct3 [2] == 1'b0);    // register (not immediate) forms
	 fa_rvfi (False, False, rg_rvfi_trap_is_i32, instr, rg_rvfi_trap_instr_C,
		  (rvfi_rs1_used ? rs1 : 0), 0, (rvfi_rs1_used ? rg_csr_val1 : 0), 0,
		  rd, new_rd_val,
		  rg_csr_pc, rg_csr_pc + (rg_rvfi_trap_is_i32 ? 4 : 2), 0, 0, 0, 0, 0);
`endif

`ifdef INCLUDE_TANDEM_VERIF
	 // Trace data
	 let trace_data = mkTrace_CSRRX (rg_trap_trace_data.pc,
					 rg_trap_trace_data.instr_sz,
					 rg_trap_trace_data.instr,
					 rd,
					 new_rd_val,
					 (rs1 != 0),    // whether we've written csr or not
					 csr_addr,
					 new_csr_val,
					 isValid (m_new_mstatus),
					 fromMaybe (?, m_new_mstatus));
	 f_trace_data.enq (trace_data);
`endif

	 // Debug
	 fa_emit_instr_trace (minstret, rg_csr_pc, instr, rg_cur_priv);
	 if (cur_verbosity > 1) begin
	    $display ("    S1: write CSRR_S_or_C: Rs1 %0d Rs1_val 0x%0h csr 0x%0h csr_val 0x%0h Rd %0d",
		      rs1, rs1_val, csr_addr, csr_val, rd);
	 end
      end
   endrule: rl_stage1_CSRR_S_or_C_2

   // ================================================================
   // Restart the pipe after a CSRRX stall
   // waiting for stageF to be non-busy (servicing a previous request)

   rule rl_stage1_restart_after_csrrx (   (rg_state == CPU_CSRRX_RESTART)
				       && (stageF.out.ostatus != OSTATUS_BUSY));
      if (cur_verbosity > 1)
	 $display ("%0d: %m.rl_stage1_restart_after_csrrx", mcycle);

      stageD.set_full (False);
      stage1.set_full (False);    fa_step_check;
      fa_stageF_redirect (rg_next_pc);
   endrule

   // ================================================================
   // Stage1: nonpipe special: MRET/SRET/URET

   // E14: the front end may run alongside; these flush StageD (rl_front only fires with StageD not full)
   (* descending_urgency = "rl_front, rl_stage1_xRET" *)
   rule rl_stage1_xRET (   (rg_state == CPU_RUNNING)
			&& (! halting)
			&& (stage3.out.ostatus == OSTATUS_EMPTY)
			&& (stage2.out.ostatus == OSTATUS_EMPTY)
			&& (stage1.out.ostatus == OSTATUS_NONPIPE)
			&& (   (stage1.out.control == CONTROL_MRET)
			    || (stage1.out.control == CONTROL_SRET)
			    || (stage1.out.control == CONTROL_URET))
			&& (stageF.out.ostatus != OSTATUS_BUSY));
      if (cur_verbosity > 1) $display ("%0d: %m.rl_stage1_xRET", mcycle);

      // Return-from-exception actions on CSRs
      Priv_Mode from_priv = ((stage1.out.control == CONTROL_MRET) ?
			     m_Priv_Mode : ((stage1.out.control == CONTROL_SRET) ?
					    s_Priv_Mode : u_Priv_Mode));
      match { .next_pc, .new_priv, .new_mstatus } <- csr_regfile.csr_ret_actions (from_priv);
      // Save new privilege and pc for ifetch
      rg_cur_priv <= new_priv;
      rg_next_pc  <= next_pc;

      // Note old MSTATUS.MXR and SSTATUS.SUM for initiating FETCH in next phase
      rg_mstatus_MXR <= mstatus_MXR;
      rg_sstatus_SUM <= sstatus_SUM;

      rg_state <= CPU_START_TRAP_HANDLER;    // TODO: bad naming; this is not starting the trap handler

      stageD.set_full (False);
      stage1.set_full (False);    fa_step_check;

      // Accounting
      csr_regfile.csr_minstret_incr;
`ifdef RVFI
      let rvfi_d2 = stage1.out.data_to_stage2;
      fa_rvfi (False, False, rvfi_d2.is_i32_not_i16, rvfi_d2.instr, rvfi_d2.instr_C, 0, 0, 0, 0, 0, 0,
	       rvfi_d2.pc, next_pc, 0, 0, 0, 0, 0);
`endif

`ifdef INCLUDE_TANDEM_VERIF
      // Trace data
      let td  = stage1.out.data_to_stage2.trace_data;
      let td1 = mkTrace_RET (next_pc, td.instr_sz, td.instr, new_priv, new_mstatus);
      f_trace_data.enq (td1);
`endif

      // Debug
      fa_emit_instr_trace (minstret, stage1.out.data_to_stage2.pc, stage1.out.data_to_stage2.instr, rg_cur_priv);
      if (cur_verbosity != 0)
	 $display ("    xRET: next_pc:0x%0h  new mstatus:0x%0h  new priv:%0d", next_pc, new_mstatus, new_priv);
   endrule: rl_stage1_xRET

   // ================================================================
   // Stage1: nonpipe special: FENCE.I

   (* descending_urgency = "rl_front, rl_stage1_FENCE_I" *)
   rule rl_stage1_FENCE_I (   (rg_state== CPU_RUNNING)
			   && (! halting)
			   && (stage3.out.ostatus == OSTATUS_EMPTY)
			   && (stage2.out.ostatus == OSTATUS_EMPTY)
			   && (stage1.out.ostatus == OSTATUS_NONPIPE)
			   && (stage1.out.control == CONTROL_FENCE_I)
			   && (stageF.out.ostatus != OSTATUS_BUSY));
      if (cur_verbosity > 1) $display ("%0d: %m.rl_stage1_FENCE_I", mcycle);

      // Save stage1.out.next_pc since it can be destroyed by FENCE.I op
      rg_next_pc <= stage1.out.next_pc;
      near_mem.server_fence_i.request.put (?);
      rg_state <= CPU_FENCE_I;
   endrule

   // ----------------
   // Finish FENCE.I

   rule rl_finish_FENCE_I (rg_state == CPU_FENCE_I);
      if (cur_verbosity > 1) $display ("%0d: %m.rl_finish_FENCE_I", mcycle);

      // Await mem system FENCE.I completion
      let dummy <- near_mem.server_fence_i.response.get;
`ifdef RVFI
      let rvfi_d2 = stage1.out.data_to_stage2;
      fa_rvfi (False, False, rvfi_d2.is_i32_not_i16, rvfi_d2.instr, rvfi_d2.instr_C, 0, 0, 0, 0, 0, 0,
	       rvfi_d2.pc, rg_next_pc, 0, 0, 0, 0, 0);
`endif

      // Accounting
      csr_regfile.csr_minstret_incr;
      // Debug
      fa_emit_instr_trace (minstret,
			   stage1.out.data_to_stage2.pc,
			   stage1.out.data_to_stage2.instr,
			   rg_cur_priv);
`ifdef INCLUDE_TANDEM_VERIF
      let trace_data = stage1.out.data_to_stage2.trace_data;
      f_trace_data.enq (trace_data);
`endif
      // Resume pipe
      stageD.set_full (False);
      stage1.set_full (False);    fa_step_check;
      fa_stageF_redirect (rg_next_pc);
   endrule: rl_finish_FENCE_I

   // ================================================================
   // Stage1: nonpipe special: FENCE

   (* descending_urgency = "rl_front, rl_stage1_FENCE" *)
   rule rl_stage1_FENCE (   (rg_state== CPU_RUNNING)
			 && (! halting)
			 && (stage3.out.ostatus == OSTATUS_EMPTY)
			 && (stage2.out.ostatus == OSTATUS_EMPTY)
			 && (stage1.out.ostatus == OSTATUS_NONPIPE)
			 && (stage1.out.control == CONTROL_FENCE)
			 && (stageF.out.ostatus != OSTATUS_BUSY));
      if (cur_verbosity > 1) $display ("%0d: %m.rl_stage1_FENCE", mcycle);

      // Save stage1.out.next_pc since it can be destroyed by FENCE op
      rg_next_pc <= stage1.out.next_pc;
      near_mem.server_fence.request.put (?);
      rg_state <= CPU_FENCE;
   endrule: rl_stage1_FENCE

   // ----------------

   rule rl_finish_FENCE (rg_state == CPU_FENCE);
      if (cur_verbosity > 1) $display ("%0d: %m.rl_finish_FENCE", mcycle);

      // Await mem system FENCE completion
      let dummy <- near_mem.server_fence.response.get;
`ifdef RVFI
      let rvfi_d2 = stage1.out.data_to_stage2;
      fa_rvfi (False, False, rvfi_d2.is_i32_not_i16, rvfi_d2.instr, rvfi_d2.instr_C, 0, 0, 0, 0, 0, 0,
	       rvfi_d2.pc, rg_next_pc, 0, 0, 0, 0, 0);
`endif

      // Accounting
      csr_regfile.csr_minstret_incr;
      // Debug
      fa_emit_instr_trace (minstret, stage1.out.data_to_stage2.pc,
			   stage1.out.data_to_stage2.instr,
			   rg_cur_priv);
`ifdef INCLUDE_TANDEM_VERIF
      // Trace data
      let trace_data = stage1.out.data_to_stage2.trace_data;
      f_trace_data.enq (trace_data);
`endif

      // Resume pipe
      stageD.set_full (False);
      stage1.set_full (False);    fa_step_check;
      fa_stageF_redirect (rg_next_pc);
   endrule: rl_finish_FENCE

   // ================================================================
   // Stage1: nonpipe special: SFENCE.VMA

`ifdef ISA_PRIV_S
`ifdef ISA_C
   // TODO: analyze this carefully; added to resolve a blockage
   // imem_rl_fetch_next_32b is in CPU_Fetch_C.bsv, and calls imem32.req (near_mem.imem_req).
   // fa_stageF_redirect calls stageF.enq which also calls imem.req which calls imem32.req.
   // But cond_i32_odd_fetch_next should make these rules mutually exclusive; why doesn't bsc realize this?
   (* descending_urgency = "imem_rl_fetch_next_32b, rl_stage1_SFENCE_VMA" *)
`endif

   rule rl_stage1_SFENCE_VMA (   (rg_state== CPU_RUNNING)
			      && (! halting)
			      && (stage3.out.ostatus == OSTATUS_EMPTY)
			      && (stage2.out.ostatus == OSTATUS_EMPTY)
			      && (stage1.out.ostatus == OSTATUS_NONPIPE)
			      && (stage1.out.control == CONTROL_SFENCE_VMA)
			      && (stageF.out.ostatus != OSTATUS_BUSY));
      if (cur_verbosity > 1) $display ("%0d: %m.rl_stage1_SFENCE_VMA", mcycle);

      // Save stage1.out.next_pc since it can be destroyed by FENCE op
      rg_next_pc <= stage1.out.next_pc;
      near_mem.sfence_vma_server.request.put (?);
      rg_state <= CPU_SFENCE_VMA;
   endrule: rl_stage1_SFENCE_VMA

   // ----------------

   rule rl_finish_SFENCE_VMA (rg_state == CPU_SFENCE_VMA);
      if (cur_verbosity > 1) $display ("%0d: %m.rl_finish_SFENCE_VMA", mcycle);

      // Await SFENCE.VMA completion
      let dummy <- near_mem.sfence_vma_server.response.get;
`ifdef RVFI
      let rvfi_d2 = stage1.out.data_to_stage2;
      fa_rvfi (False, False, rvfi_d2.is_i32_not_i16, rvfi_d2.instr, rvfi_d2.instr_C, 0, 0, 0, 0, 0, 0,
	       rvfi_d2.pc, rg_next_pc, 0, 0, 0, 0, 0);
`endif

      // Accounting
      csr_regfile.csr_minstret_incr;
      // Debug
      fa_emit_instr_trace (minstret,
			   stage1.out.data_to_stage2.pc,
			   stage1.out.data_to_stage2.instr,
			   rg_cur_priv);
`ifdef INCLUDE_TANDEM_VERIF
      // Trace data
      let trace_data = stage1.out.data_to_stage2.trace_data;
      f_trace_data.enq (trace_data);
`endif
      // Resume pipe
      stageD.set_full (False);
      stage1.set_full (False);    fa_step_check;
      fa_stageF_redirect (rg_next_pc);
   endrule: rl_finish_SFENCE_VMA
`endif

   // ================================================================
   // Stage1: nonpipe special: WFI

   Reg #(Bit #(32)) rg_wfi_counter <- mkReg ('h_FFFF_FFFF);

   // E14: the front end may run alongside; these flush StageD (rl_front only fires with StageD not full)
   (* descending_urgency = "rl_front, rl_stage1_WFI" *)
   rule rl_stage1_WFI (   (rg_state== CPU_RUNNING)
		       && (! halting)
		       && (stage3.out.ostatus == OSTATUS_EMPTY)
		       && (stage2.out.ostatus == OSTATUS_EMPTY)
		       && (stage1.out.ostatus == OSTATUS_NONPIPE)
		       && (stage1.out.control == CONTROL_WFI)
		       && (stageF.out.ostatus != OSTATUS_BUSY));

      rg_next_pc     <= stage1.out.next_pc;
      rg_state       <= CPU_WFI_PAUSED;
      rg_wfi_counter <= 1;
`ifdef RVFI
      let rvfi_d2 = stage1.out.data_to_stage2;
      fa_rvfi (False, False, rvfi_d2.is_i32_not_i16, rvfi_d2.instr, rvfi_d2.instr_C, 0, 0, 0, 0, 0, 0,
	       rvfi_d2.pc, stage1.out.next_pc, 0, 0, 0, 0, 0);
`endif

      stageD.set_full (False);
      stage1.set_full (False);    fa_step_check;

      // Accounting
      csr_regfile.csr_minstret_incr;

`ifdef INCLUDE_TANDEM_VERIF
      // Trace data
      let trace_data = stage1.out.data_to_stage2.trace_data;
      f_trace_data.enq (trace_data);
`endif

      // Debug
      fa_emit_instr_trace (minstret, stage1.out.data_to_stage2.pc, stage1.out.data_to_stage2.instr, rg_cur_priv);
      if (cur_verbosity > 1)
	 $display ("%0d: CPU.rl_stage1_WFI: stage.out.next_pc 0x%0h",
		   mcycle,
		   stage1.out.data_to_stage2.priv,
		   stage1.out.data_to_stage2.pc,
		   stage1.out.data_to_stage2.instr,
		   stage1.out.next_pc);
   endrule: rl_stage1_WFI

   // ----------------

   rule rl_WFI_count (rg_state == CPU_WFI_PAUSED);
      if ((rg_wfi_counter & 'hF_FFFF) == 0) begin
	 $display ("%0d: CPU: rl_WFI_count: waited so far for %0d clocks",
		   mcycle, rg_wfi_counter);
	 $display ("    csr_regfile.wfi_resume = ", fshow (csr_regfile.wfi_resume));
	 $display ("    stageF.out.ostatus     = ", fshow (stageF.out.ostatus));
      end
      rg_wfi_counter <= rg_wfi_counter + 1;
   endrule

   // ----------------

   (* descending_urgency = "rl_WFI_resume, rl_WFI_count" *)
   rule rl_WFI_resume (   (rg_state == CPU_WFI_PAUSED)
		       && (   rg_wfi_resume
			   || stop_step_req)
		       && (stageF.out.ostatus != OSTATUS_BUSY));
      if (cur_verbosity > 1)
	 $display ("%0d: CPU: rl_WFI_resume after waiting %0d cycles", mcycle, rg_wfi_counter);

      // Resume pipe (it will handle the interrupt, if one is pending)
      stageD.set_full (False);
      fa_stageF_redirect (rg_next_pc);
   endrule: rl_WFI_resume

   // ----------------
   rule rl_reset_from_WFI (   (rg_state == CPU_WFI_PAUSED)
			   && f_reset_reqs.notEmpty);
      if (cur_verbosity > 1) $display ("%0d: %m.rl_reset_from_WFI", mcycle);

      rg_state <= CPU_RESET1;
   endrule: rl_reset_from_WFI

   // ================================================================
   // Initiate instruction fetch from new_pc.
   // These actions were formerly part of the stage1 and stage2 trap,
   // external interrupt and RET rules. Separated to break long timing
   // paths from stage2 and stage3 status to IFetch

   rule rl_trap_fetch ((rg_state == CPU_START_TRAP_HANDLER)
		       && (stageF.out.ostatus != OSTATUS_BUSY));
      stageD.set_full (False);
      fa_stageF_redirect (rg_next_pc);
   endrule: rl_trap_fetch

   // ================================================================
   // Stage1: nonpipe trap: BREAK into Debug Mode when dcsr.ebreakm/s/u is set
   // Not setting tval, as we are breaking to the debugger.
   // TODO: Does the spec say anything about this?

`ifdef INCLUDE_GDB_CONTROL
   (* descending_urgency = "rl_front, rl_trap_BREAK_to_Debug_Mode" *)
   rule rl_trap_BREAK_to_Debug_Mode (   (rg_state == CPU_RUNNING)
				     && (! halting)
				     && (stage3.out.ostatus == OSTATUS_EMPTY)
				     && (stage2.out.ostatus == OSTATUS_EMPTY)
				     && (stage1.out.ostatus == OSTATUS_NONPIPE)
				     && (stage1.out.control == CONTROL_TRAP)
				     && (stageF.out.ostatus != OSTATUS_BUSY)
				     && break_into_Debug_Mode);
      if (cur_verbosity > 1) $display ("%0d: %m.rl_trap_BREAK_to_Debug_Mode", mcycle);

      let pc    = stage1.out.data_to_stage2.pc;
      let instr = stage1.out.data_to_stage2.instr;

      $display ("CPU: BREAK to Debug Mode: PC 0x%08h instr 0x%08h", pc, instr);
      if (cur_verbosity > 0) begin
	 $display (    "%m");
	 $display (    "%0d: rule rl_trap_BREAK_to_Debug_Mode", mcycle);
      end
      if (cur_verbosity > 1)
	 $display ("    Flushing caches");

      csr_regfile.write_dcsr_cause_priv (DCSR_CAUSE_EBREAK, rg_cur_priv);
      csr_regfile.write_dpc (pc);    // Where we'll resume on 'continue'
      rg_state <= CPU_GDB_PAUSING;

      // Flush both caches -- using the same interface as that used by FENCE_I
      near_mem.server_fence_i.request.put (?);
   endrule: rl_trap_BREAK_to_Debug_Mode

   // ----------------
   // Handle the flush responses from the caches when the flush was initiated
   // on entering CPU_GDB_PAUSING state

   rule rl_BREAK_cache_flush_finish (rg_state == CPU_GDB_PAUSING);
      let ack <- near_mem.server_fence_i.response.get;
      rg_state <= CPU_DEBUG_MODE;

      // Notify debugger that we've halted
      f_run_halt_rsps.enq (False);

      $display ("CPU halted");
      if (cur_verbosity > 1) begin
	 $display (    "%m");
	 $display (    "%0d: rule rl_BREAK_cache_flush_finish", mcycle);
      end
   endrule

   // ----------------
   // Reset from Debug Module

   rule rl_reset_from_Debug_Module (f_reset_reqs.notEmpty && (rg_state != CPU_RESET1));
      $display ("%0d: CPU reset from Debug Module", mcycle);
      $display ("    At: %m.rl_reset_from_Debug_Module");
      rg_state <= CPU_RESET1;
   endrule
`endif

   // ================================================================
   // EXTERNAL and GDB INTERRUPTS while running
   // We take an interrupt when Stage1 is frozen
   // and Stage2 and Stage3 have drained,
   // encapsulated in condition 'stage1_take_interrupt'

   // E14: rl_front first; during halting StageD fills within two cycles and rl_front stops
   (* descending_urgency = "rl_front, rl_stage1_interrupt" *)
   rule rl_stage1_interrupt (interrupt_pending
			     && (rg_state == CPU_RUNNING)
			     && stage1_take_interrupt
			     && (stageF.out.ostatus != OSTATUS_BUSY));
      if (cur_verbosity > 1) $display ("%0d: %m.rl_stage1_interrupt", mcycle);

      // Just save relevant info and handle in next clock
      Exc_Code exc_code = 0;    // "Unknown cause" for NMI
      if (rg_int_pending matches tagged Valid .ec
	  &&& (! rg_nmi_pending))
	 exc_code = ec;

      rg_trap_info       <= Trap_Info {epc:      stage1.out.data_to_stage2.pc,
				       exc_code: exc_code,
				       tval:     0};
      rg_trap_interrupt  <= True;
      rg_trap_instr      <= stage1.out.data_to_stage2.instr;
`ifdef RVFI
      rg_rvfi_trap_is_i32  <= stage1.out.data_to_stage2.is_i32_not_i16;
      rg_rvfi_trap_rs1_val <= stage1.out.data_to_stage2.rs1_val;
      rg_rvfi_trap_rs2_val <= stage1.out.data_to_stage2.rs2_val;
      rg_rvfi_trap_instr_C <= stage1.out.data_to_stage2.instr_C;
`endif

`ifdef INCLUDE_TANDEM_VERIF
      // rg_trap_trace_data <= ?;    // Will be filled in in rl_trap
`endif

      rg_state           <= CPU_TRAP;
   endrule: rl_stage1_interrupt

   // ================================================================
   // Stage1: Handle debugger stop-request and dcsr.step step-request while running
   // and no interrupt pending.  Stage1 has an architectural instruction,
   // and stage2 and stage3 are empty, and stageF is not BUSY.

`ifdef INCLUDE_GDB_CONTROL
   (* descending_urgency = "rl_front, rl_stage1_stop" *)
   rule rl_stage1_stop (   (rg_state== CPU_RUNNING)
			&& stage1_stop
			&& (stageF.out.ostatus != OSTATUS_BUSY));
      if (cur_verbosity > 1) $display ("%0d: %m.rl_stage1_stop", mcycle);

      let pc    = stage1.out.data_to_stage2.pc;    // We'll retry this instruction on 'continue'
      let instr = stage1.out.data_to_stage2.instr;

      // Report CPI only stop-req, but not on step-req (where it's not very useful)
      if (rg_stop_req) begin
	 $display ("CPU stop for debugger: minstret %0d priv %0d PC 0x%0h instr 0x%0h",
		   minstret, rg_cur_priv, pc, instr);
	 fa_report_CPI;
      end
      else
	 $display ("CPU: stop after single-step. PC = 0x%08h", pc);
      $display ("    %0d: %m", mcycle);
      $display ("    Rule rl_stage1_stop");

      DCSR_Cause cause= (rg_stop_req ? DCSR_CAUSE_HALTREQ : DCSR_CAUSE_STEP);
      csr_regfile.write_dcsr_cause_priv (cause, rg_cur_priv);
      csr_regfile.write_dpc (pc);    // We'll retry this instruction on 'continue'
      rg_state      <= CPU_GDB_PAUSING;
      rg_stop_req   <= False;
      rg_step_count <= 0;

      // Flush both caches -- using the same interface as that used by FENCE_I
      near_mem.server_fence_i.request.put (?);

      // Accounting: none (instruction is abandoned)
   endrule: rl_stage1_stop
`endif

   // ================================================================
   // ================================================================
   // ================================================================
   // DEBUGGER ACCESS

   // ----------------
   // Debug Module Run/Halt control

`ifdef INCLUDE_GDB_CONTROL
   rule rl_debug_run ((f_run_halt_reqs.first == True) && (rg_state == CPU_DEBUG_MODE));
      f_run_halt_reqs.deq;

      // Debugger 'resume' request (e.g., GDB 'continue' command)
      let dpc = csr_regfile.read_dpc;
      fa_restart_from_halt (dpc);

      // Notify debugger that we've started running
      f_run_halt_rsps.enq (True);

      $display ("CPU: Debugger run request: running: DPC = 0x%0x", dpc);
      if (cur_verbosity > 1) begin
	 $display ("    %m");
	 $display ("    %0d: Rule rl_debug_run", mcycle);
      end
   endrule

   (* descending_urgency = "rl_debug_run_redundant, rl_pipe" *)
   rule rl_debug_run_redundant (   (f_run_halt_reqs.first == True)
				&& fn_is_running (rg_state));
      f_run_halt_reqs.deq;

      // Notify debugger that we're running
      f_run_halt_rsps.enq (True);

      $display ("CPU: Redundant debugger run request: already running.");
      if (cur_verbosity > 1) begin
	 $display ("    %m");
	 $display ("    %0d: Rule rl_debug_run_redundant", mcycle);
      end
   endrule

   // E19: the debug-reset and debug-halt rules write rg_state / rg_stop_req, so they conflict
   // with most pipeline-control rules; bsc's arbitrary urgency put them in the MIDDLE (below
   // stage2_nonpipe / CSRR_W / trap..., above stage1_trap / xRET / stop / CSRR_W_2...), which
   // chained dcache / stage status through !WILL_FIRE_rl_reset_from_Debug_Module into the
   // CSR write enables (mav_csr_write) and the stage1 trap rules.  Most urgent instead: the
   // others see only their CAN_FIRE (f_reset_reqs / f_run_halt_reqs flags, rg_state).
   (* descending_urgency = "rl_reset_from_Debug_Module, rl_BREAK_cache_flush_finish" *)
   (* descending_urgency = "rl_reset_from_Debug_Module, rl_finish_FENCE_I" *)
   (* descending_urgency = "rl_reset_from_Debug_Module, rl_finish_FENCE" *)
   (* descending_urgency = "rl_reset_from_Debug_Module, rl_debug_run" *)
   (* descending_urgency = "rl_reset_from_Debug_Module, rl_pipe" *)
   (* descending_urgency = "rl_reset_from_Debug_Module, rl_reset_complete" *)
   (* descending_urgency = "rl_reset_from_Debug_Module, rl_stage1_CSRR_S_or_C_2" *)
   (* descending_urgency = "rl_reset_from_Debug_Module, rl_stage1_CSRR_W_2" *)
   (* descending_urgency = "rl_reset_from_Debug_Module, rl_stage1_FENCE_I" *)
   (* descending_urgency = "rl_reset_from_Debug_Module, rl_stage1_FENCE" *)
   (* descending_urgency = "rl_reset_from_Debug_Module, rl_stage1_interrupt" *)
   (* descending_urgency = "rl_reset_from_Debug_Module, rl_stage1_stop" *)
   (* descending_urgency = "rl_reset_from_Debug_Module, rl_stage1_trap" *)
   (* descending_urgency = "rl_reset_from_Debug_Module, rl_stage1_WFI" *)
   (* descending_urgency = "rl_reset_from_Debug_Module, rl_stage1_xRET" *)
   (* descending_urgency = "rl_reset_from_Debug_Module, rl_trap_BREAK_to_Debug_Mode" *)
   (* descending_urgency = "rl_reset_from_Debug_Module, rl_reset_from_WFI" *)
   (* descending_urgency = "rl_reset_from_Debug_Module, rl_stage1_CSRR_S_or_C" *)
   (* descending_urgency = "rl_reset_from_Debug_Module, rl_stage1_CSRR_W" *)
   (* descending_urgency = "rl_reset_from_Debug_Module, rl_stage1_restart_after_csrrx" *)
   (* descending_urgency = "rl_reset_from_Debug_Module, rl_stage2_nonpipe" *)
   (* descending_urgency = "rl_reset_from_Debug_Module, rl_trap_fetch" *)
   (* descending_urgency = "rl_reset_from_Debug_Module, rl_trap" *)
   (* descending_urgency = "rl_debug_halt, rl_stage1_FENCE_I" *)
   (* descending_urgency = "rl_debug_halt, rl_stage1_FENCE" *)
   (* descending_urgency = "rl_debug_halt, rl_stage1_stop" *)
   (* descending_urgency = "rl_debug_halt, rl_stage1_trap" *)
   (* descending_urgency = "rl_debug_halt, rl_stage1_WFI" *)
   (* descending_urgency = "rl_debug_halt, rl_stage1_xRET" *)
   (* descending_urgency = "rl_debug_halt, rl_trap_BREAK_to_Debug_Mode" *)
   (* descending_urgency = "rl_debug_halt, rl_stage1_CSRR_S_or_C" *)
   (* descending_urgency = "rl_debug_halt, rl_stage1_CSRR_W" *)
   (* descending_urgency = "rl_debug_halt, rl_WFI_resume" *)
   (* descending_urgency = "rl_debug_halt, rl_pipe" *)
   rule rl_debug_halt ((f_run_halt_reqs.first == False) && fn_is_running (rg_state));
      f_run_halt_reqs.deq;

      // Debugger 'halt' request (e.g., GDB '^C' command)
      rg_stop_req <= True;
      $display ("CPU: Debugger halt request: requested.");
      if (cur_verbosity > 1) begin
	 $display ("    %m");
	 $display ("    %d: Rule rl_debug_halt", mcycle);
      end
   endrule

   rule rl_debug_halt_redundant (   (f_run_halt_reqs.first == False)
				 && (! fn_is_running (rg_state)));
      f_run_halt_reqs.deq;

      // Notify debugger that we've 'halted'
      $display ("CPU: Debugger halt request: but CPU is not currently running.");
      f_run_halt_rsps.enq (False);

      $display ("CPU: Redundant debugger halt request: state = ", fshow (rg_state));
      if (cur_verbosity > 1) begin
	 $display ("    %m");
	 $display ("    %0d: Rule rl_debug_halt_redundant", mcycle);
      end
   endrule

   // ----------------
   // Debug Module GPR read/write

   rule rl_debug_read_gpr ((rg_state == CPU_DEBUG_MODE) && (! f_gpr_reqs.first.write));
      let req <- pop (f_gpr_reqs);
      Bit #(5) regname = req.address;
      let data = gpr_regfile.read_rs1_port2 (regname);
      let rsp = DM_CPU_Rsp {ok: True, data: data};
      f_gpr_rsps.enq (rsp);

      $display ("CPU: Debugger read GPR %0d => 0x%0h", regname, data);
      if (cur_verbosity > 1) begin
	 $display ("    %m");
	 $display ("    %0d: Rule rl_debug_read_gpr", mcycle);
      end
   endrule

   // E13: Stage3 writeback (only fires with Stage3 full; this rule runs with it empty)
   (* descending_urgency = "stage3_rl_writeback, rl_debug_write_gpr" *)
   rule rl_debug_write_gpr ((rg_state == CPU_DEBUG_MODE) && f_gpr_reqs.first.write);
      let req <- pop (f_gpr_reqs);
      Bit #(5) regname = req.address;
      let data = req.data;
      gpr_regfile.write_rd (regname, data);

      let rsp = DM_CPU_Rsp {ok: True, data: ?};
      f_gpr_rsps.enq (rsp);

      $display ("CPU: Debugger write GPR %0d <= 0x%0h", regname, data);
      if (cur_verbosity > 1) begin
	 $display ("    %m");
	 $display ("    %0d: Rule rl_debug_write_gpr", mcycle);
      end
   endrule

   rule rl_debug_gpr_access_busy (rg_state != CPU_DEBUG_MODE);
      let req <- pop (f_gpr_reqs);
      let rsp = DM_CPU_Rsp {ok: False, data: ?};
      f_gpr_rsps.enq (rsp);

      $display ("CPU: Debugger GPR access busy: state (not CPU_DEBUG_MODE) = ", fshow (rg_state));
      if (cur_verbosity > 1) begin
	 $display ("    %m");
	 $display ("    %0d: Rule rl_debug_gpr_access_busy", mcycle);
      end
   endrule

   // ----------------
   // Debug Module FPR read/write

`ifdef ISA_F
   rule rl_debug_read_fpr ((rg_state == CPU_DEBUG_MODE) && (! f_fpr_reqs.first.write));
      let req <- pop (f_fpr_reqs);
      Bit #(5) regname = req.address;
      let data = fpr_regfile.read_rs1_port2 (regname);
      let rsp = DM_CPU_Rsp {ok: True, data: data};
      f_fpr_rsps.enq (rsp);

      $display ("CPU: Debugger read FPR %0d => 0x%0h", regname, data);
      if (cur_verbosity > 1) begin
	 $display ("    %m");
	 $display ("    %0d: Rule rl_debug_read_fpr", mcycle);
      end
   endrule

   rule rl_debug_write_fpr ((rg_state == CPU_DEBUG_MODE) && f_fpr_reqs.first.write);
      let req <- pop (f_fpr_reqs);
      Bit #(5) regname = req.address;
      let data = req.data;
      fpr_regfile.write_rd (regname, data);

      let rsp = DM_CPU_Rsp {ok: True, data: ?};
      f_fpr_rsps.enq (rsp);

      $display ("CPU: Debugger write FPR %0d => 0x%0h", regname, data);
      if (cur_verbosity > 1) begin
	 $display ("    %m");
	 $display ("    %0d: Rule rl_debug_write_fpr", mcycle);
      end
   endrule

   rule rl_debug_fpr_access_busy (rg_state != CPU_DEBUG_MODE);
      let req <- pop (f_fpr_reqs);
      let rsp = DM_CPU_Rsp {ok: False, data: ?};
      f_fpr_rsps.enq (rsp);

      $display ("CPU: Debugger FPR access busy: state (not CPU_DEBUG_MODE) = ", fshow (rg_state));
      if (cur_verbosity > 1) begin
	 $display ("    %m");
	 $display ("    %0d: Rule rl_debug_fpr_access_busy", mcycle);
      end
   endrule
`endif

   // ----------------
   // Debug Module CSR read/write

   rule rl_debug_read_csr ((rg_state == CPU_DEBUG_MODE) && (! f_csr_reqs.first.write));
      let req <- pop (f_csr_reqs);
      Bit #(12) csr_addr = req.address;
      let m_data = csr_regfile.read_csr_port2 (csr_addr);
      let data = fromMaybe (?, m_data);
      let rsp = DM_CPU_Rsp {ok: True, data: data};
      f_csr_rsps.enq (rsp);

      $display ("CPU: Debugger read CSR 0x%0x => 0x%0h", csr_addr, data);
      if (cur_verbosity > 1) begin
	 $display ("    %m");
	 $display ("    %0d: Rule rl_debug_read_csr", mcycle);
      end
   endrule

   rule rl_debug_write_csr ((rg_state == CPU_DEBUG_MODE) && f_csr_reqs.first.write);
      let req <- pop (f_csr_reqs);
      Bit #(12) csr_addr = req.address;
      let data = req.data;
      let csr_write_result <- csr_regfile.mav_csr_write (csr_addr, data);

      let rsp = DM_CPU_Rsp {ok: True, data: ?};
      f_csr_rsps.enq (rsp);

      $display ("CPU: Debugger write CSR 0x%0h <= 0x%0h.  New csr contents: 0x%0h",
		csr_addr, data, csr_write_result.new_csr_value);
      if (cur_verbosity > 1) begin
	 $display ("    %m");
	 $display ("    %0d: Rule rl_debug_write_csr", mcycle);
      end
   endrule

   rule rl_debug_csr_access_busy (rg_state != CPU_DEBUG_MODE);
      let req <- pop (f_csr_reqs);
      let rsp = DM_CPU_Rsp {ok: False, data: ?};
      f_csr_rsps.enq (rsp);

      $display ("CPU: Debugger CSR access busy: state (not CPU_DEBUG_MODE) = ", fshow (rg_state));
      if (cur_verbosity > 1) begin
	 $display ("    %m");
	 $display ("    %0d: Rule rl_debug_csr_access_busy", mcycle);
      end
   endrule
`endif

   // ================================================================
   // ================================================================
   // ================================================================
   // INTERFACE

   // Reset
   interface Server  hart0_server_reset = toGPServer (f_reset_reqs, f_reset_rsps);

   // ----------------
   // SoC fabric connections

   // IMem to fabric master interface
   interface  imem_master = near_mem.imem_master;

   // DMem to fabric master interface
   interface Near_Mem_Fabric_IFC  mem_master = near_mem.mem_master;

   // ----------------------------------------------------------------
   // Optional AXI4-Lite D-cache slave interface

`ifdef INCLUDE_DMEM_SLAVE
   interface  dmem_slave = near_mem.dmem_slave;
`endif

   // ----------------
   // Interface to 'coherent DMA' port of optional L2 cache

   interface AXI4_Slave_IFC dma_server = near_mem.dma_server;

   // ----------------------------------------------------------------
   // External interrupts

   method Action  m_external_interrupt_req (x) = csr_regfile.m_external_interrupt_req (x);
   method Action  s_external_interrupt_req (x) = csr_regfile.s_external_interrupt_req (x);

   // ----------------
   // Set CSR TIME (shadow-copy of MTIME)

   method Action ma_set_csr_time (Bit #(64) t) = csr_regfile.ma_set_csr_time (t);

   // ----------------
   // Software and timer interrupts (from Near_Mem_IO/CLINT)

   method Action  software_interrupt_req (x) = csr_regfile.software_interrupt_req (x);
   method Action  timer_interrupt_req    (x) = csr_regfile.timer_interrupt_req    (x);

   // ----------------
   // Non-maskable interrupt

   method Action  nmi_req (x);
      csr_regfile.nmi_req (x);
   endmethod

   // ----------------
   // Optional PC trace interface
   // Outputs a stream of 2-tuples: (cycle_count, pc)

`ifdef INCLUDE_PC_TRACE
   interface Get  g_pc_trace = toGet (f_pc_trace);
`endif

   // ----------------
   // Optional interface to Tandem Verifier

`ifdef INCLUDE_TANDEM_VERIF
   interface Get  trace_data_out = toGet (f_trace_data);
`endif

   // ----------------
   // Optional interface to Debug Module

`ifdef INCLUDE_GDB_CONTROL
   // run-control, other
   interface Server  hart0_server_run_halt = toGPServer (f_run_halt_reqs, f_run_halt_rsps);

   interface Put  hart0_put_other_req;
      method Action  put (Bit #(4) req);
	 cfg_verbosity <= req;
      endmethod
   endinterface

   // GPR access
   interface Server  hart0_gpr_mem_server = toGPServer (f_gpr_reqs, f_gpr_rsps);

`ifdef ISA_F
   // FPR access
   interface Server  hart0_fpr_mem_server = toGPServer (f_fpr_reqs, f_fpr_rsps);
`endif

   // CSR access
   interface Server  hart0_csr_mem_server = toGPServer (f_csr_reqs, f_csr_rsps);
`endif

   // ----------------------------------------------------------------
   // Misc. control and status

   // ----------------
   // Debugging: set core's verbosity

   method Action  set_verbosity (Bit #(4)  verbosity, Bit #(64)  logdelay);
      cfg_verbosity <= verbosity;
      cfg_logdelay  <= logdelay;
      if (verbosity != cfg_verbosity) begin
	 $display ("%0d: CPU verbosity change to %0d; logdelay %0d", mcycle, verbosity, logdelay);
	 $display ("    %m");
      end
   endmethod

   // ----------------
   // For ISA tests: watch memory writes to <tohost> addr

`ifdef WATCH_TOHOST
   method Action set_watch_tohost (Bool watch_tohost, Bit #(64) tohost_addr);
      near_mem.set_watch_tohost (watch_tohost, tohost_addr);
   endmethod

   method Bit #(64) mv_tohost_value = near_mem.mv_tohost_value;
`endif

   // Inform core that DDR4 has been initialized and is ready to accept requests
   method Action ma_ddr4_ready;
      near_mem.ma_ddr4_ready;
   endmethod

   // Misc. status; 0 = running, no error
   method Bit #(8) mv_status;
      return near_mem.mv_status;
   endmethod

   // Performance-monitoring taps (see interface comment)
`ifdef RVFI
   method Bool        mv_rvfi_valid     = rg_rvfi_valid;
   method Bit #(64)   mv_rvfi_order     = rg_rvfi_order;
   method Bit #(32)   mv_rvfi_insn      = rg_rvfi_insn;
   method Bool        mv_rvfi_trap      = rg_rvfi_trap;
   method Bool        mv_rvfi_halt      = False;
   method Bool        mv_rvfi_intr      = rg_rvfi_intr;
   method Bit #(2)    mv_rvfi_mode      = rg_rvfi_mode;
   method Bit #(2)    mv_rvfi_ixl       = 1;
   method Bit #(5)    mv_rvfi_rs1_addr  = rg_rvfi_rs1_addr;
   method Bit #(5)    mv_rvfi_rs2_addr  = rg_rvfi_rs2_addr;
   method WordXL      mv_rvfi_rs1_rdata = rg_rvfi_rs1_rdata;
   method WordXL      mv_rvfi_rs2_rdata = rg_rvfi_rs2_rdata;
   method Bit #(5)    mv_rvfi_rd_addr   = rg_rvfi_rd_addr;
   method WordXL      mv_rvfi_rd_wdata  = rg_rvfi_rd_wdata;
   method WordXL      mv_rvfi_pc_rdata  = rg_rvfi_pc_rdata;
   method WordXL      mv_rvfi_pc_wdata  = rg_rvfi_pc_wdata;
   method WordXL      mv_rvfi_mem_addr  = rg_rvfi_mem_addr;
   method Bit #(4)    mv_rvfi_mem_rmask = rg_rvfi_mem_rmask;
   method Bit #(4)    mv_rvfi_mem_wmask = rg_rvfi_mem_wmask;
   method WordXL      mv_rvfi_mem_rdata = rg_rvfi_mem_rdata;
   method WordXL      mv_rvfi_mem_wdata = rg_rvfi_mem_wdata;
`endif
   method Bool mv_perf_retire;
      return (minstret != rg_minstret_d1);
   endmethod

   method Bool mv_perf_redirect;
      return rg_perf_redirect_tick;
   endmethod

   method Bool mv_perf_imiss;
      return near_mem.mv_perf_imiss;
   endmethod

   method Bool mv_perf_dmiss;
      return near_mem.mv_perf_dmiss;
   endmethod

endmodule: mkCPU

// ================================================================

endpackage
