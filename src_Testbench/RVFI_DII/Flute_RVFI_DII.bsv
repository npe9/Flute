// Copyright (c) 2026 -- ULX3S Flute Linux program (TestRIG RVFI-DII)

package Flute_RVFI_DII;

// ================================================================
// TestRIG RVFI-DII top level (Bluesim): mkFlute_RVFI_DII.
//
//   TestRIG VEngine  <--TCP (port $RVFI_DII_PORT, default 5001)-->  RVFI_DII_Bridge
//                                                                     |  instructions by index
//                                                                     v
//                     mkFlute_DII_DUT (reset_by bridge.new_rst): mkCPU (built with -D RVFI
//                       -D RVFI_DII) + D-side RAM (8 MiB @ 0x8000_0000) + I-side zero responder
//
// The bridge buffers a whole instruction trace, then lets the CPU fetch
// instruction words by index; each retirement's RVFI record (riscv-formal
// convention inside the CPU) is converted to the TestRIG / Sail convention
// and sent back.  After the last record the bridge sends a halt packet and
// pulses new_rst, which resets the CPU, caches, and (via dii_mem_clear) RAM.
//
// Adapted from the CTSRD-CHERI Flute/Piccolo RVFI-DII integration
// (src_Testbench/Top/Top_HW_Side.bsv, src_Verifier/Flute_RVFI_DII_Bridge.bsv).
// ================================================================

import FIFOF        :: *;
import GetPut       :: *;
import ClientServer :: *;
import Connectable  :: *;

import ISA_Decls    :: *;
import AXI4_Types   :: *;
import Fabric_Defs  :: *;
import AXI_Widths   :: *;    // Wd_*_Dma
import CPU_IFC      :: *;
import CPU          :: *;

import RVFI_DII     :: *;
import DII_AXI4_Mem :: *;

// ================================================================
// RVFI record convention: the CPU reports riscv-formal "aligned" memory
// fields (word address, lane-shifted mask/data); Sail / TestRIG report the
// byte address of the access with LSB-aligned mask/data.

typedef RVFI_DII_Execution #(XLEN, RVFI_MEMLEN) Flute_RVFI;

// RVFI_MemMask is 4 bits, or 8 with ISA_D (FLD/FSD: RVFI_MEMLEN = 64)
function Bit #(TLog #(SizeOf #(RVFI_MemMask))) fn_lowest_lane (RVFI_MemMask m);
   Bit #(TLog #(SizeOf #(RVFI_MemMask))) lane = 0;
   for (Integer j = valueOf (SizeOf #(RVFI_MemMask)) - 1; j >= 0; j = j - 1)
      if (m [j] == 1) lane = fromInteger (j);
   return lane;
endfunction

// RVC HINTs that Flute executes as a write of rd's unchanged value
// (c.addi rd,0 / c.slli rd,0 / c.srli rd',0 / c.srai rd',0) but Sail retires
// as no-ops (riscv_insts_hints.sail): report "no rd write" for them.
// Architecturally identical; only the RVFI rd fields differ.
function Bool fn_rvc_rd_hint (Bit #(32) insn);
   Bit #(16) c = insn [15:0];
   Bool shamt0 = (c [12] == 0) && (c [6:2] == 0);
   Bool c_addi = (c [1:0] == 2'b01) && (c [15:13] == 3'b000) && shamt0 && (c [11:7] != 0);
   Bool c_slli = (c [1:0] == 2'b10) && (c [15:13] == 3'b000) && shamt0;
   Bool c_srxi = (c [1:0] == 2'b01) && (c [15:13] == 3'b100) && (c [11] == 0) && shamt0;
   return (insn [1:0] != 2'b11) && (c_addi || c_slli || c_srxi);
endfunction

interface Flute_DII_DUT_IFC;
   interface Get #(Dii_Id)                      seq_req;
   interface Put #(Tuple2 #(Dii_Id, Bit #(32))) instr_rsp;
   interface Get #(Flute_RVFI)                  report;
endinterface

module mkFlute_DII_DUT (Flute_DII_DUT_IFC);
   CPU_IFC cpu <- mkCPU;

`ifdef RVFI_DII_RAM_8M
   // Sail's default RVFI-DII platform: 8 MiB of RAM at 0x8000_0000, the rest vacant
   Bit #(64) ram_base = 'h_8000_0000;
   Bit #(64) ram_lim  = 'h_8080_0000;
`else
   // Default: RAM is [0, 0xFFFF_FFC0), everything but the top 64-byte line
   // (Sail run with SAIL_RVFI_RAM_BASE=0 SAIL_RVFI_RAM_SIZE=0xffffffc0,
   // testrig/sail-rvfi-ram.patch)
   Bit #(64) ram_base = 0;
   Bit #(64) ram_lim  = 'h_FFFF_FFC0;    // see SoC_Map fn_is_mem_addr
`endif
   DII_AXI4_Mem_IFC #(Wd_Id, Wd_Addr, Wd_Data, Wd_User) dmem <- mkDII_AXI4_Mem (True,  ram_base, ram_lim);
   DII_AXI4_Mem_IFC #(Wd_Id, Wd_Addr, Wd_Data, Wd_User) imem <- mkDII_AXI4_Mem (False, ram_base, ram_lim);

   mkConnection (cpu.mem_master,  dmem.slave);
   mkConnection (cpu.imem_master, imem.slave);
   AXI4_Master_IFC #(Wd_Id_Dma, Wd_Addr_Dma, Wd_Data_Dma, Wd_User_Dma) no_dma = dummy_AXI4_Master_ifc;
   mkConnection (no_dma, cpu.dma_server);

   // 0: clear RAM + start the hart; 1: wait for reset completion; 2: running
   Reg #(Bit #(2)) rg_phase <- mkReg (0);

   FIFOF #(Flute_RVFI) f_reports <- mkSizedFIFOF (64);

   // No interrupt sources (Sail's RVFI-DII platform has no CLINT / PLIC)
   (* fire_when_enabled, no_implicit_conditions *)
   rule rl_tie_meip;  cpu.m_external_interrupt_req (False);  endrule
   (* fire_when_enabled, no_implicit_conditions *)
   rule rl_tie_seip;  cpu.s_external_interrupt_req (False);  endrule
   (* fire_when_enabled, no_implicit_conditions *)
   rule rl_tie_msip;  cpu.software_interrupt_req (False);    endrule
   (* fire_when_enabled, no_implicit_conditions *)
   rule rl_tie_mtip;  cpu.timer_interrupt_req (False);       endrule
   (* fire_when_enabled, no_implicit_conditions *)
   rule rl_tie_nmi;   cpu.nmi_req (False);                   endrule
   (* fire_when_enabled, no_implicit_conditions *)
   rule rl_tie_time;  cpu.ma_set_csr_time (0);               endrule

   rule rl_start (rg_phase == 0);
      dmem.clear;
      cpu.hart0_server_reset.request.put (True);    // run on reset
      rg_phase <= 1;
   endrule

   rule rl_started (rg_phase == 1);
      let ok <- cpu.hart0_server_reset.response.get;
      cpu.ma_ddr4_ready;
      rg_phase <= 2;
   endrule

   (* fire_when_enabled *)
   rule rl_rvfi (cpu.mv_rvfi_valid);
      RVFI_MemMask m  = cpu.mv_rvfi_mem_rmask | cpu.mv_rvfi_mem_wmask;
      let     sh = fn_lowest_lane (m);
      Bool    hint = (! cpu.mv_rvfi_trap) && fn_rvc_rd_hint (cpu.mv_rvfi_insn);
      Flute_RVFI r = RVFI_DII_Execution {
	 rvfi_order:     cpu.mv_rvfi_order,
	 rvfi_trap:      cpu.mv_rvfi_trap,
	 rvfi_halt:      False,
	 rvfi_intr:      cpu.mv_rvfi_intr,
	 rvfi_insn:      cpu.mv_rvfi_insn,
	 rvfi_rs1_addr:  cpu.mv_rvfi_rs1_addr,
	 rvfi_rs2_addr:  cpu.mv_rvfi_rs2_addr,
	 rvfi_rs1_data:  cpu.mv_rvfi_rs1_rdata,
	 rvfi_rs2_data:  cpu.mv_rvfi_rs2_rdata,
	 rvfi_pc_rdata:  cpu.mv_rvfi_pc_rdata,
	 rvfi_pc_wdata:  cpu.mv_rvfi_pc_wdata,
	 rvfi_mem_wdata: cpu.mv_rvfi_mem_wdata >> {sh, 3'b000},
	 rvfi_rd_addr:   (hint ? 0 : cpu.mv_rvfi_rd_addr),
	 rvfi_rd_wdata:  (hint ? 0 : cpu.mv_rvfi_rd_wdata),
	 rvfi_mem_addr:  ((m == 0) ? cpu.mv_rvfi_mem_addr : (cpu.mv_rvfi_mem_addr + zeroExtend (sh))),
	 rvfi_mem_rmask: cpu.mv_rvfi_mem_rmask >> sh,
	 rvfi_mem_wmask: cpu.mv_rvfi_mem_wmask >> sh,
	 rvfi_mem_rdata: cpu.mv_rvfi_mem_rdata >> {sh, 3'b000}
	 };
      if (f_reports.notFull)
	 f_reports.enq (r);
      else
	 $display ("ERROR: mkFlute_DII_DUT: RVFI report FIFO overflow; record lost");
   endrule

   interface seq_req   = cpu.dii_seq_req;
   interface instr_rsp = cpu.dii_instr_rsp;
   interface report    = toGet (f_reports);
endmodule

// ================================================================

(* synthesize *)
module mkFlute_RVFI_DII (Empty);
   RVFI_DII_Bridge_Scalar #(XLEN, RVFI_MEMLEN) bridge <- mkRVFI_DII_Bridge_Scalar ("RVFI_DII", 5001);
   Flute_DII_DUT_IFC dut <- mkFlute_DII_DUT (reset_by bridge.new_rst);

   // Instruction words by trace index; past the end of the trace, a NOP
   // (only reachable on a wrong path, or after the last record, when the
   // bridge is about to halt and reset anyway).
   rule rl_instr;
      let ix  <- dut.seq_req.get;
      let m   <- bridge.client.getInst (ix);
      dut.instr_rsp.put (tuple2 (ix, fromMaybe (32'h_0000_0013, m)));
   endrule

   mkConnection (dut.report, bridge.client.report);
endmodule

// ================================================================

endpackage
