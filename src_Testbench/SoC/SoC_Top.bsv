// Copyright (c) 2016-2022 Bluespec, Inc. All Rights Reserved.

package SoC_Top;

// ================================================================
// This package is the SoC "top-level".

// (Note: there will be further layer(s) above this for
//    simulation top-level, FPGA top-level, etc.)

// ================================================================
// Exports

export SoC_Top_IFC (..), mkSoC_Top;

// ================================================================
// BSV library imports

import FIFOF         :: *;
import GetPut        :: *;
import ClientServer  :: *;
import Connectable   :: *;
import Memory        :: *;

// ----------------
// BSV additional libs

import Cur_Cycle   :: *;
import GetPut_Aux  :: *;

// ================================================================
// Project imports

// Main fabric
import AXI4_Types     :: *;
import AXI4_Fabric    :: *;
import AXI4_Deburster :: *;
import AXI_Widths     :: *;

`ifdef INCLUDE_DMEM_SLAVE
import AXI4_Lite_Types :: *;
`endif

import Fabric_Defs :: *;
import SoC_Map     :: *;
import SoC_Fabric  :: *;

// SoC components (CPU, mem, and IPs)

import Near_Mem_IFC :: *;    // For Wd_{Id,Addr,Data,User}_Dma
import Core_IFC     :: *;
import Core         :: *;
import PLIC         :: *;    // For interface to PLIC interrupt sources, in Core_IFC

import Boot_ROM       :: *;
import Mem_Controller :: *;
import UART_Model     :: *;

`ifdef INCLUDE_CAMERA_MODEL
import Camera_Model   :: *;
`endif

`ifdef INCLUDE_ACCEL0
import AXI4_GPIO      :: *;
`ifdef INCLUDE_SPI0
import AXI4_SPI       :: *;
`endif
`ifdef INCLUDE_VIDEO
import AXI4_RegBus    :: *;
`endif
import SoC_Info       :: *;
`endif

`ifdef INCLUDE_TANDEM_VERIF
import TV_Info :: *;
`endif

`ifdef INCLUDE_GDB_CONTROL
import External_Control :: *;    // Control requests/responses from HSFE
import Debug_Module     :: *;
`endif

// ================================================================
// The outermost interface of the SoC

interface SoC_Top_IFC;
`ifdef INCLUDE_GDB_CONTROL
   // To external controller (E.g., GDB)
   interface Server #(Control_Req, Control_Rsp) server_external_control;
`endif

`ifdef INCLUDE_TANDEM_VERIF
   // To tandem verifier
   interface Get #(TV_Info) tv_verifier_info_get;
`endif

   // External real memory
   interface Client #(Raw_Mem_Req, Raw_Mem_Chunk)  to_raw_mem;

   // UART0 to external console
   interface Get #(Bit #(8)) get_to_console;
   interface Put #(Bit #(8)) put_from_console;

   // Catch-all status; return-value can identify the origin (0 = none)
   (* always_ready *)
   method Bit #(8) status;

`ifdef INCLUDE_ACCEL0
   // GPIO (Accel0 slot repurposed for ULX3S LEDs/buttons)
   (* always_ready *) method Bit #(8) gpio_leds;
   (* always_enabled *) method Action gpio_btn1 (Bit #(1) v);
`endif
`ifdef INCLUDE_SPI0
   // microSD slot, SPI mode (src/AXI4_SPI.bsv, chip select 0)
   (* always_ready *)   method Bit #(1) sd_sclk;
   (* always_ready *)   method Bit #(1) sd_mosi;
   (* always_ready *)   method Bit #(1) sd_cs_n;
   (* always_enabled *) method Action   sd_miso (Bit #(1) v);
   // configuration flash on chip select 1 of the same host (its own pins; the clock leaves
   // through USRMCLK in src/Top.v)
   (* always_ready *)   method Bit #(1) flash_sclk;
   (* always_ready *)   method Bit #(1) flash_mosi;
   (* always_ready *)   method Bit #(1) flash_cs_n;
   (* always_enabled *) method Action   flash_miso (Bit #(1) v);
`endif
`ifdef INCLUDE_SPI2
   // on-board ADC (second SPI host, chip select 0)
   (* always_ready *)   method Bit #(1) adc_sclk;
   (* always_ready *)   method Bit #(1) adc_mosi;
   (* always_ready *)   method Bit #(1) adc_cs_n;
   (* always_enabled *) method Action   adc_miso (Bit #(1) v);
`endif
`ifdef INCLUDE_VIDEO
   // register bus (src/AXI4_RegBus.bsv) to the Verilog devices in src/Top.v (the video block)
   (* always_ready *)   method Bit #(1)  rb_valid;
   (* always_ready *)   method Bit #(1)  rb_we;
   (* always_ready *)   method Bit #(32) rb_addr;
   (* always_ready *)   method Bit #(32) rb_wdata;
   (* always_ready *)   method Bit #(4)  rb_wstrb;
   (* always_enabled *) method Action    rb_ack (Bit #(1) ack, Bit #(32) rdata);
`endif

   // ----------------------------------------------------------------
   // Misc. control and status

   // ----------------
   // Debugging: set core's verbosity
   method Action  set_verbosity (Bit #(4)  verbosity, Bit #(64)  logdelay);

   // ----------------
   // For ISA tests: watch memory writes to <tohost> addr
`ifdef WATCH_TOHOST
   method Action set_watch_tohost (Bool  watch_tohost, Fabric_Addr  tohost_addr);
   method Bit #(64) mv_tohost_value;
`endif

   // ----------------
   // Inform core that DDR4 has been initialized and is ready to accept requests
   method Action ma_ddr4_ready;

   // ----------------
   // Misc. status; 0 = running, no error
   (* always_ready *)
   method Bit #(8) mv_status;

endinterface

// ================================================================
// Local types and constants

typedef enum {SOC_START,
	      SOC_RESETTING,
`ifdef INCLUDE_GDB_CONTROL
	      SOC_RESETTING_NDM,
	      SOC_NDM_SETTLE,
	      SOC_RESETTING_NDM2,
`endif
	      SOC_IDLE} SoC_State
deriving (Bits, Eq, FShow);

// ================================================================
// The module

(* synthesize *)
module mkSoC_Top #(int clk_hz) (SoC_Top_IFC);
   Integer verbosity = 0;    // Normally 0; non-zero for debugging

   Reg #(SoC_State) rg_state <- mkReg (SOC_START);
   Reg #(Bool)      rg_intr_prev <- mkReg (False);    // perf: irq edge detect

   // SoC address map specifying base and limit for memories, IPs, etc.
   SoC_Map_IFC soc_map <- mkSoC_Map;

   // Core: CPU + Near_Mem_IO (CLINT) + PLIC + Debug module (optional) + TV (optional)
   Reset dm_power_on_reset <- exposeCurrentReset;    // TODO: plumb this in from outside
   Core_IFC #(N_External_Interrupt_Sources)  core <- mkCore (dm_power_on_reset);

   // SoC Fabric, and the peripheral bridge behind it (low-speed slaves)
   Fabric_AXI4_IFC         fabric <- mkFabric_AXI4;
   Periph_Fabric_AXI4_IFC  periph <- mkPeriph_Fabric_AXI4;

   // SoC Boot ROM
   Boot_ROM_IFC  boot_rom <- mkBoot_ROM;
   // AXI4 Deburster in front of Boot_ROM
   AXI4_Deburster_IFC #(Wd_Id,
			Wd_Addr,
			Wd_Data,
			Wd_User)  boot_rom_axi4_deburster <- mkAXI4_Deburster_A;

   // SoC Memory (takes AXI4 bursts directly: line refills stream)
   Mem_Controller_IFC  mem0_controller <- mkMem_Controller;

   // SoC IPs
   UART_IFC   uart0  <- mkUART;

`ifdef INCLUDE_ACCEL0
   // Accel0 master to fabric
   AXI4_GPIO_IFC  accel0 <- mkAXI4_GPIO;

   // SoC self-description block (device manifest + id registers)
   SoC_Info_IFC  soc_info <- mkSoC_Info (clk_hz);
`endif

`ifdef INCLUDE_SPI0
   // SPI host: microSD slot (CS 0) and configuration flash (CS 1)
   AXI4_SPI_IFC  spi0 <- mkAXI4_SPI;
`endif
`ifdef INCLUDE_SPI2
   // SPI host: the on-board ADC (CS 0; CS 1 has no pin and its MISO is tied high)
   AXI4_SPI_IFC  spi2 <- mkAXI4_SPI;
`endif
`ifdef INCLUDE_VIDEO
   // register bus for the Verilog-side devices (video)
   AXI4_RegBus_IFC  regbus <- mkAXI4_RegBus;
`endif

   // ----------------
   // SoC fabric master connections
   // Note: see 'SoC_Map' for 'master_num' definitions

   // CPU IMem master to fabric
   mkConnection (core.cpu_imem_master,  fabric.v_from_masters [imem_master_num]);

   // CPU DMem master to fabric
   mkConnection (core.core_mem_master,  fabric.v_from_masters [dmem_master_num]);

   // Tie-off unused 'coherent DMA port' into optional L2 cache (LLC, Last Level Cache)
   AXI4_Master_IFC #(Wd_Id_Dma, Wd_Addr_Dma, Wd_Data_Dma, Wd_User_Dma)
                   dummy_master = dummy_AXI4_Master_ifc;
   mkConnection (dummy_master, core.dma_server);

`ifdef INCLUDE_ACCEL0
   // accel_aes0 to fabric
   mkConnection (accel0.master,  fabric.v_from_masters [accel0_master_num]);
`endif

   // ----------------
   // SoC fabric slave connections
   // Note: see 'SoC_Map' for 'slave_num' definitions

   // Fabric to Boot ROM
   mkConnection (fabric.v_to_slaves [boot_rom_slave_num], boot_rom_axi4_deburster.from_master);
   mkConnection (boot_rom_axi4_deburster.to_slave,        boot_rom.slave);

   // Fabric to Mem Controller
   mkConnection (fabric.v_to_slaves [mem0_controller_slave_num], mem0_controller.slave);

   // Fabric to the peripheral bridge
   mkConnection (fabric.v_to_slaves [periph_slave_num], periph.v_from_masters [0]);

   // Bridge to UART0
   mkConnection (periph.v_to_slaves [uart0_periph_num],  uart0.slave);

`ifdef INCLUDE_ACCEL0
   // Fabric to accel0
   mkConnection (fabric.v_to_slaves [accel0_slave_num], accel0.slave);

   // Bridge to soc_info
   mkConnection (periph.v_to_slaves [soc_info_periph_num], soc_info.slave);

`ifdef INCLUDE_SPI0
   // Bridge to spi0
   mkConnection (periph.v_to_slaves [spi0_periph_num], spi0.slave);
`endif
`ifdef INCLUDE_SPI2
   mkConnection (periph.v_to_slaves [spi2_periph_num], spi2.slave);
   // CS 1 of spi2 has no pin: its MISO idles high
   (* fire_when_enabled, no_implicit_conditions *)
   rule rl_spi2_miso1_tie;
      spi2.miso1 (1);
   endrule
`endif
`ifdef INCLUDE_VIDEO
   mkConnection (periph.v_to_slaves [regbus_periph_num], regbus.slave);
`endif

   // Bus-error monitor: drain fabric decode-error events into SoC_Info
   rule rl_bus_err_capture;
      let e <- fabric.err_events.get;
      soc_info.note_err (e);
   endrule

   // Performance-monitoring taps: core retire/redirect/cache-miss pulses,
   // plus interrupt edges observable at this level.  Each fires its
   // one-cycle pulse into SoC_Info's programmable counters.
   rule rl_perf_taps_core;
      if (core.mv_perf_retire)   soc_info.note_retire;
      if (core.mv_perf_redirect) soc_info.note_redirect;
      if (core.mv_perf_imiss)    soc_info.note_imiss;
      if (core.mv_perf_dmiss)    soc_info.note_dmiss;
   endrule

   rule rl_perf_taps_irq;
      let intr = uart0.intr;
      if (intr && ! rg_intr_prev) begin
	 soc_info.note_uart_irq;
	 soc_info.note_ext_irq;
      end
      rg_intr_prev <= intr;
   endrule
`endif

`ifdef HTIF_MEMORY
   AXI4_Slave_IFC#(Wd_Id, Wd_Addr, Wd_Data, Wd_User) htif <- mkAxi4LRegFile(bytes_per_htif);

   mkConnection (fabric.v_to_slaves [htif_slave_num], htif);
`endif

   // ----------------------------------------------------------------
   // Optional AXI4-Lite D-cache slave interface tie-off (not used)

`ifdef INCLUDE_DMEM_SLAVE
   rule rl_always_dmem_slave (True);
      core.cpu_dmem_slave.m_arvalid (False, ?, ?, ?);
      core.cpu_dmem_slave.m_rready (False);
      core.cpu_dmem_slave.m_awvalid (False, ?, ?, ?);
      core.cpu_dmem_slave.m_wvalid (False, ?, ?);
      core.cpu_dmem_slave.m_bready (False);
   endrule
`endif

   // ----------------
   // Connect interrupt sources for CPU external interrupt request inputs.

   // Reg #(Bool) rg_intr_prev <- mkReg (False);    // For debugging only

   (* fire_when_enabled, no_implicit_conditions *)
   rule rl_connect_external_interrupt_requests;
      Bool intr = uart0.intr;

      // UART
      core.core_external_interrupt_sources [irq_num_uart0].m_interrupt_req (intr);
      Integer last_irq_num = irq_num_uart0;

`ifdef INCLUDE_ACCEL0
      Bool intr_accel0 = accel0.interrupt_req;
      core.core_external_interrupt_sources [irq_num_accel0].m_interrupt_req (intr_accel0);
      last_irq_num = irq_num_accel0;
`endif
`ifdef INCLUDE_SPI0
      core.core_external_interrupt_sources [irq_num_spi0].m_interrupt_req (spi0.interrupt_req);
      last_irq_num = irq_num_spi0;
`endif
`ifdef INCLUDE_SPI2
      core.core_external_interrupt_sources [irq_num_spi2].m_interrupt_req (spi2.interrupt_req);
      last_irq_num = irq_num_spi2;
`endif

      // Tie off remaining interrupt request lines (2..N)
      for (Integer j = last_irq_num + 1; j < valueOf (N_External_Interrupt_Sources); j = j + 1)
	 core.core_external_interrupt_sources [j].m_interrupt_req (False);

      // Non-maskable interrupt request. [Tie-off; TODO: connect to genuine sources]
      core.nmi_req (False);

      /* For debugging only
      if ((! rg_intr_prev) && intr)
	 $display ("SoC_Top: intr posedge");
      else if (rg_intr_prev && (! intr))
	 $display ("SoC_Top: intr negedge");

      rg_intr_prev <= intr;
      */
   endrule

   // ================================================================
   // SOFT RESET

   Reg #(Bool) rg_fabric_reset <- mkReg (False);
   Reg #(Bool) rg_accel0_reset <- mkReg (False);
   Reg #(Bool) rg_periph_reset <- mkReg (False);
   Bool resets_pending = rg_fabric_reset || rg_accel0_reset || rg_periph_reset;

   function Action fa_reset_start_actions (Bool running);
      action
	 core.cpu_reset_server.request.put (running);
	 mem0_controller.server_reset.request.put (?);
	 uart0.server_reset.request.put (?);
	 rg_fabric_reset <= True;
`ifdef INCLUDE_ACCEL0
	 rg_accel0_reset <= True;
`endif
	 rg_periph_reset <= True;
      endaction
   endfunction

   // The fabric and the slaves without a reset handshake (which drop their in-flight
   // transactions with the fabric's) reset one cycle later, from registers, one rule per
   // corner of the die.  Folding their reset methods' ready signals into the guards of the
   // reset-start rules put cross-die routes in one sys_clk cycle (39-41 MHz with video).
   rule rl_fabric_reset (rg_fabric_reset);
      fabric.reset;
      boot_rom.reset;
      boot_rom_axi4_deburster.reset;
      rg_fabric_reset <= False;
   endrule

`ifdef INCLUDE_ACCEL0
   rule rl_accel0_reset (rg_accel0_reset);
      accel0.reset;
      rg_accel0_reset <= False;
   endrule
`endif

   // Likewise the peripheral sub-fabric and its handshake-less slaves (the video block pulls
   // the peripheral side to the far corner).  The reset-complete rules wait for all of these
   // passes, so they still land inside the reset window.
   rule rl_periph_reset (rg_periph_reset);
      periph.reset;
      soc_info.reset;
`ifdef INCLUDE_SPI0
      spi0.reset;
`endif
`ifdef INCLUDE_SPI2
      spi2.reset;
`endif
`ifdef INCLUDE_VIDEO
      regbus.reset;
`endif
      rg_periph_reset <= False;
   endrule

   function Action fa_reset_complete_actions ();
      action
	 let cpu_rsp             <- core.cpu_reset_server.response.get;
	 let mem0_controller_rsp <- mem0_controller.server_reset.response.get;
	 let uart0_rsp           <- uart0.server_reset.response.get;

	 // Initialize address maps of slave IPs
	 boot_rom.set_addr_map (soc_map.m_boot_rom_addr_base,
				soc_map.m_boot_rom_addr_lim);

	 mem0_controller.set_addr_map (soc_map.m_mem0_controller_addr_base,
				       soc_map.m_mem0_controller_addr_lim);

	 uart0.set_addr_map (soc_map.m_uart0_addr_base, soc_map.m_uart0_addr_lim);

`ifdef INCLUDE_ACCEL0
	 accel0.init (fabric_default_id,
		      soc_map.m_accel0_addr_base,
		      soc_map.m_accel0_addr_lim);
`endif
	 soc_info.init (fabric_default_id,
			soc_map.m_soc_info_addr_base,
			soc_map.m_soc_info_addr_lim);
`ifdef INCLUDE_SPI0
	 spi0.init (fabric_default_id, soc_map.m_spi0_addr_base, soc_map.m_spi0_addr_lim);
`endif
`ifdef INCLUDE_SPI2
	 spi2.init (fabric_default_id, soc_map.m_spi2_addr_base, soc_map.m_spi2_addr_lim);
`endif

	 if (verbosity != 0) begin
	    $display ("  SoC address map:");
	    $display ("  Boot ROM:        0x%0h .. 0x%0h",
		      soc_map.m_boot_rom_addr_base,
		      soc_map.m_boot_rom_addr_lim);
	    $display ("  Mem0 Controller: 0x%0h .. 0x%0h",
		      soc_map.m_mem0_controller_addr_base,
		      soc_map.m_mem0_controller_addr_lim);
	    $display ("  UART0:           0x%0h .. 0x%0h",
		      soc_map.m_uart0_addr_base,
		      soc_map.m_uart0_addr_lim);
	 end
      endaction
   endfunction

   // ----------------
   // Initial reset; CPU comes up running.

   rule rl_reset_start_initial (rg_state == SOC_START);
      Bool running = True;
      fa_reset_start_actions (running);
      rg_state <= SOC_RESETTING;

      $display ("%0d:%m.rl_reset_start_initial ...", cur_cycle);
   endrule

   rule rl_reset_complete_initial ((rg_state == SOC_RESETTING) && (! resets_pending));
      fa_reset_complete_actions;
      rg_state <= SOC_IDLE;

      $display ("%0d:%m.rl_reset_complete_initial", cur_cycle);
   endrule

   // ----------------
   // NDM (non-debug-module) reset (requested from Debug Module)
   // Request argument indicates if CPU comes up running or halted

`ifdef INCLUDE_GDB_CONTROL
   Reg #(Bool) rg_running <- mkRegU;

   // Two passes.  The masters (I$, D$, debug-module bus) reset a few cycles after the fabrics
   // and slaves, so a reset with traffic in flight can leave behind a write address whose data
   // was dropped (or data whose address was), or a response the fabric's in-order routing will
   // hand to the wrong request.  The first pass brings the hart up halted, so once it completes
   // no master issues anything; after a short settle the second pass resets everything again
   // with nothing in flight and brings the hart up as requested.  "Nothing" is not quite true:
   // the halted hart's I$ may still be refilling a line (8 beats from the boot ROM, ~20 cycles),
   // and a second pass landing inside that burst leaves its tail beats to the next fetch, which
   // then hangs.  The settle covers a whole refill.
   Reg #(Bit #(6)) rg_ndm_settle <- mkReg (0);

   rule rl_ndm_reset_start (rg_state == SOC_IDLE);
      let running <- core.ndm_reset_client.request.get;
      rg_running <= running;

      fa_reset_start_actions (False);
      rg_state <= SOC_RESETTING_NDM;

      $display ("%0d:%m.rl_ndm_reset_start (non-debug-module) running = ",
		cur_cycle, fshow (running));
   endrule

   rule rl_ndm_reset_quiet ((rg_state == SOC_RESETTING_NDM) && (! resets_pending));
      fa_reset_complete_actions;
      rg_ndm_settle <= 0;
      rg_state <= SOC_NDM_SETTLE;
   endrule

   rule rl_ndm_reset_settle (rg_state == SOC_NDM_SETTLE);
      if (rg_ndm_settle == '1) begin
	 fa_reset_start_actions (rg_running);
	 rg_state <= SOC_RESETTING_NDM2;
      end
      rg_ndm_settle <= rg_ndm_settle + 1;
   endrule

   rule rl_ndm_reset_complete ((rg_state == SOC_RESETTING_NDM2) && (! resets_pending));
      fa_reset_complete_actions;
      rg_state <= SOC_IDLE;

      core.ndm_reset_client.response.put (rg_running);

      $display ("%0d:%m.rl_ndm_reset_complete (non-debug-module) running = ",
		cur_cycle, fshow (rg_running));
   endrule
`endif

   // ================================================================
   // BEHAVIOR WITH DEBUG MODULE

`ifdef INCLUDE_GDB_CONTROL
   // ----------------------------------------------------------------
   // External debug requests and responses (e.g., GDB)

   FIFOF #(Control_Req) f_external_control_reqs <- mkFIFOF;
   FIFOF #(Control_Rsp) f_external_control_rsps <- mkFIFOF;

   Control_Req req = f_external_control_reqs.first;

   rule rl_handle_external_req_read_request (req.op == external_control_req_op_read_control_fabric);
      f_external_control_reqs.deq;
      core.dm_dmi.read_addr (truncate (req.arg1));
      if (verbosity != 0) begin
	 $display ("%0d:%m.rl_handle_external_req_read_request", cur_cycle);
         $display ("    ", fshow (req));
      end
   endrule

   rule rl_handle_external_req_read_response;
      let x <- core.dm_dmi.read_data;
      let rsp = Control_Rsp {status: external_control_rsp_status_ok, result: signExtend (x)};
      f_external_control_rsps.enq (rsp);
      if (verbosity != 0) begin
	 $display ("%0d:%m.rl_handle_external_req_read_response", cur_cycle);
         $display ("    ", fshow (rsp));
      end
   endrule

   rule rl_handle_external_req_write (req.op == external_control_req_op_write_control_fabric);
      f_external_control_reqs.deq;
      core.dm_dmi.write (truncate (req.arg1), truncate (req.arg2));
      let rsp = Control_Rsp {status: external_control_rsp_status_ok, result: 0};
      f_external_control_rsps.enq (rsp);
      if (verbosity != 0) begin
         $display ("%0d:%m.rl_handle_external_req_write", cur_cycle);
         $display ("    ", fshow (req));
      end
   endrule

   rule rl_handle_external_req_err (   (req.op != external_control_req_op_read_control_fabric)
				    && (req.op != external_control_req_op_write_control_fabric));
      f_external_control_reqs.deq;
      let rsp = Control_Rsp {status: external_control_rsp_status_err, result: 0};
      f_external_control_rsps.enq (rsp);

      $display ("%0d:%m.rl_handle_external_req_err: unknown req.op", cur_cycle);
      $display ("    ", fshow (req));
   endrule
`endif

   // ================================================================
   // INTERFACE

   // To external controller (E.g., GDB)
`ifdef INCLUDE_GDB_CONTROL
   interface server_external_control = toGPServer (f_external_control_reqs, f_external_control_rsps);
`endif

`ifdef INCLUDE_TANDEM_VERIF
   // To tandem verifier
   interface tv_verifier_info_get = core.tv_verifier_info_get;
`endif

   // External real memory
   interface to_raw_mem = mem0_controller.to_raw_mem;

   // UART to external console
   interface get_to_console   = uart0.get_to_console;
   interface put_from_console = uart0.put_from_console;

   // Catch-all status; return-value can identify the origin (0 = none)
   method Bit #(8) status = 0;

`ifdef INCLUDE_ACCEL0
   // GPIO (Accel0 slot repurposed for ULX3S LEDs/buttons)
   method Bit #(8) gpio_leds = accel0.leds;
   method Action gpio_btn1 (Bit #(1) v);
      accel0.btn1 (v);
   endmethod
`endif
`ifdef INCLUDE_SPI0
   method Bit #(1) sd_sclk = spi0.sclk;
   method Bit #(1) sd_mosi = spi0.mosi;
   method Bit #(1) sd_cs_n = spi0.cs_n [0];
   method Action sd_miso (Bit #(1) v);
      spi0.miso0 (v);
   endmethod
   method Bit #(1) flash_sclk = spi0.sclk;
   method Bit #(1) flash_mosi = spi0.mosi;
   method Bit #(1) flash_cs_n = spi0.cs_n [1];
   method Action flash_miso (Bit #(1) v);
      spi0.miso1 (v);
   endmethod
`endif
`ifdef INCLUDE_SPI2
   method Bit #(1) adc_sclk = spi2.sclk;
   method Bit #(1) adc_mosi = spi2.mosi;
   method Bit #(1) adc_cs_n = spi2.cs_n [0];
   method Action adc_miso (Bit #(1) v);
      spi2.miso0 (v);
   endmethod
`endif
`ifdef INCLUDE_VIDEO
   method Bit #(1)  rb_valid = regbus.rb_valid;
   method Bit #(1)  rb_we    = regbus.rb_we;
   method Bit #(32) rb_addr  = regbus.rb_addr;
   method Bit #(32) rb_wdata = regbus.rb_wdata;
   method Bit #(4)  rb_wstrb = regbus.rb_wstrb;
   method Action rb_ack (Bit #(1) ack, Bit #(32) rdata);
      regbus.rb_ack (ack, rdata);
   endmethod
`endif

   // ----------------------------------------------------------------
   // Misc. control and status

   method Action  set_verbosity (Bit #(4)  verbosity1, Bit #(64)  logdelay);
      core.set_verbosity (verbosity1, logdelay);
   endmethod

`ifdef WATCH_TOHOST
   // For ISA tests: watch memory writes to <tohost> addr
   method Action set_watch_tohost (Bool  watch_tohost, Fabric_Addr  tohost_addr);
      core.set_watch_tohost (watch_tohost, tohost_addr);
   endmethod

   method Bit #(64) mv_tohost_value;
      Bit #(64) tohost_value = 0;
      tohost_value = core.mv_tohost_value;
      return tohost_value;
   endmethod
`endif

   method Action ma_ddr4_ready;
      core.ma_ddr4_ready;
   endmethod

   method Bit #(8) mv_status;
      return core.mv_status;    // 0 = running, no error
   endmethod
endmodule: mkSoC_Top

// ================================================================
// Specialization of parameterized AXI4 Deburster for this SoC.

(* synthesize *)
module mkAXI4_Deburster_A (AXI4_Deburster_IFC #(Wd_Id,
						Wd_Addr,
						Wd_Data,
						Wd_User));
   let m <- mkAXI4_Deburster;
   return m;
endmodule

// ================================================================

endpackage
