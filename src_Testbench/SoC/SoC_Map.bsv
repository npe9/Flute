// Copyright (c) 2013-2020 Bluespec, Inc. All Rights Reserved

package SoC_Map;

// ================================================================
// This module defines the overall 'address map' of the SoC, showing
// the addresses serviced by each slave IP, and which addresses are
// memory vs. I/O.

// ***** WARNING! WARNING! WARNING! *****

// During system integration, this address map should be identical to
// the system interconnect settings (e.g., routing of requests between
// masters and slaves).  This map is also needed by software so that
// it knows how to address various IPs.

// This module contains no state; it just has constants, and so can be
// freely instantiated at multiple places in the SoC module hierarchy
// at no hardware cost.  It allows this map to be defined in one
// place and shared across the SoC.

// ================================================================
// Exports

export  SoC_Map_IFC (..), mkSoC_Map;
export  DeviceId, MemRegion (..), Slave_Num, fn_soc_memmap, fn_dev_slave_num;
export  mem0_controller_addr_size;

// export  fn_addr_in_range;

export  Num_Masters;
export  imem_master_num;
export  dmem_master_num;
export  accel0_master_num;

export  Num_Slaves;
export  boot_rom_slave_num;
export  mem0_controller_slave_num;
export  uart0_slave_num;
export  accel0_slave_num;
export  soc_info_slave_num;

export  N_External_Interrupt_Sources;
export  n_external_interrupt_sources;
export  irq_num_uart0;
export  irq_num_accel0;

// ================================================================
// Bluespec library imports

import List   :: *;
import Vector :: *;

// ================================================================
// Project imports

import Fabric_Defs :: *;    // Only for type Fabric_Addr


// ================================================================
// Declarative memory map -- ONE table drives the AXI fabric decoder
// (SoC_Fabric.bsv) AND the elaboration-time software manifest
// (messageM below; captured from the bsc output by the Makefile and
// turned into build/soc-memory-map.json by scripts/soc_manifest.py).
//
// 'bus' classifies how a region is reached:
//   "axi"      decoded by the AXI fabric (fabric_decoded = True)
//   "near_mem" implemented on the core's near-memory port (CLINT);
//              not an AXI slave by architecture
//   "reserved" declared in the map but with no fabric slave and no
//              device (the PLIC range on this SoC)
//
// Order matters for the fabric decoder: first match wins.  The
// constants below are the single source of truth; the module methods
// and the table both reference them.

typedef Bit #(TLog #(Num_Slaves))  Slave_Num;

typedef enum {Dev_BootROM, Dev_SDRAM, Dev_CLINT, Dev_UART0, Dev_GPIO, Dev_PLIC, Dev_SoCInfo}
        DeviceId deriving (Bits, Eq);

typedef struct {
   String     name;
   DeviceId   device;
   Bit #(32)  base;
   Bit #(32)  size;
   String     bus;
   Bool       fabric_decoded;
   Bit #(32)  irq;       // external-interrupt source, or 0xFF if none
   Bit #(32)  flags;     // bit0: decoded by AXI fabric; bits[3:1]: bus
                         //       (0=axi, 1=near_mem, 2=reserved)
} MemRegion;

// ---- region constants (package level: shared by methods + table)

Fabric_Addr near_mem_io_addr_base = 'h_0200_0000;
Fabric_Addr near_mem_io_addr_size = 'h_0000_C000;    // 48K
Fabric_Addr near_mem_io_addr_lim  = near_mem_io_addr_base + near_mem_io_addr_size;

Fabric_Addr plic_addr_base = 'h_0C00_0000;
Fabric_Addr plic_addr_size = 'h_0040_0000;    // 4M
Fabric_Addr plic_addr_lim  = plic_addr_base + plic_addr_size;

Fabric_Addr uart0_addr_base = 'hC000_0000;
Fabric_Addr uart0_addr_size = 'h0000_0080;    // 128
Fabric_Addr uart0_addr_lim  = uart0_addr_base + uart0_addr_size;

`ifdef INCLUDE_ACCEL0
Fabric_Addr accel0_addr_base = 'hC000_2000;
Fabric_Addr accel0_addr_size = 'h0000_1000;    // 4K
Fabric_Addr accel0_addr_lim  = accel0_addr_base + accel0_addr_size;

function Bool fn_is_accel0_addr (Fabric_Addr addr);
   return ((accel0_addr_base <= addr) && (addr < accel0_addr_lim));
endfunction
`endif

Fabric_Addr boot_rom_addr_base = 'h_0000_1000;
Fabric_Addr boot_rom_addr_size = 'h_0000_1000;    // 4K
Fabric_Addr boot_rom_addr_lim  = boot_rom_addr_base + boot_rom_addr_size;

Fabric_Addr mem0_controller_addr_base = 'h_8000_0000;
Fabric_Addr mem0_controller_addr_size = 'h_0200_0000;    // 32 MB (ULX3S W9825G6KH)
Fabric_Addr mem0_controller_addr_lim  = mem0_controller_addr_base + mem0_controller_addr_size;

Fabric_Addr soc_info_addr_base = 'hC000_3000;
Fabric_Addr soc_info_addr_size = 'h0000_1000;    // 4K
Fabric_Addr soc_info_addr_lim  = soc_info_addr_base + soc_info_addr_size;

// ---- address predicates (package level; used by the module methods)

function Bool fn_is_near_mem_io_addr (Fabric_Addr addr);
   return ((near_mem_io_addr_base <= addr) && (addr < near_mem_io_addr_lim));
endfunction

function Bool fn_is_plic_addr (Fabric_Addr addr);
   return ((plic_addr_base <= addr) && (addr < plic_addr_lim));
endfunction

function Bool fn_is_uart0_addr (Fabric_Addr addr);
   return ((uart0_addr_base <= addr) && (addr < uart0_addr_lim));
endfunction

function Bool fn_is_boot_rom_addr (Fabric_Addr addr);
   return ((boot_rom_addr_base <= addr) && (addr < boot_rom_addr_lim));
endfunction

function Bool fn_is_mem0_controller_addr (Fabric_Addr addr);
   return ((mem0_controller_addr_base <= addr) && (addr < mem0_controller_addr_lim));
endfunction

// ---- the table itself

function List #(MemRegion) fn_soc_memmap ();
   List #(MemRegion) rs = List::nil;
   rs = List::cons (MemRegion {name: "sdram",    device: Dev_SDRAM,  base: truncate (mem0_controller_addr_base), size: truncate (mem0_controller_addr_size), bus: "axi",      fabric_decoded: True, irq: 32'h0000_00FF, flags: 1}, rs);
   rs = List::cons (MemRegion {name: "boot_rom", device: Dev_BootROM, base: truncate (boot_rom_addr_base),       size: truncate (boot_rom_addr_size),       bus: "axi",      fabric_decoded: True, irq: 32'h0000_00FF, flags: 1}, rs);
   rs = List::cons (MemRegion {name: "uart0",    device: Dev_UART0,  base: truncate (uart0_addr_base),          size: truncate (uart0_addr_size),          bus: "axi",      fabric_decoded: True, irq: fromInteger (irq_num_uart0),  flags: 1}, rs);
`ifdef INCLUDE_ACCEL0
   rs = List::cons (MemRegion {name: "gpio",     device: Dev_GPIO,   base: truncate (accel0_addr_base),         size: truncate (accel0_addr_size),         bus: "axi",      fabric_decoded: True, irq: fromInteger (irq_num_accel0), flags: 1}, rs);
`endif
   rs = List::cons (MemRegion {name: "clint",    device: Dev_CLINT,  base: truncate (near_mem_io_addr_base),    size: truncate (near_mem_io_addr_size),    bus: "near_mem", fabric_decoded: False, irq: 32'h0000_00FF, flags: 2}, rs);
   rs = List::cons (MemRegion {name: "plic",     device: Dev_PLIC,   base: truncate (plic_addr_base),           size: truncate (plic_addr_size),           bus: "reserved", fabric_decoded: False, irq: 32'h0000_00FF, flags: 4}, rs);
   rs = List::cons (MemRegion {name: "soc_info", device: Dev_SoCInfo, base: truncate (soc_info_addr_base),       size: truncate (soc_info_addr_size),       bus: "axi",      fabric_decoded: True, irq: 32'h0000_00FF, flags: 1}, rs);
   return rs;
endfunction

// Map a table device to its AXI fabric slave number (package-level
// Integers defined below).  Regions not decoded by the fabric never
// call this.
function Slave_Num fn_dev_slave_num (DeviceId d);
   case (d)
      Dev_SDRAM:  return fromInteger (mem0_controller_slave_num);
      Dev_BootROM: return fromInteger (boot_rom_slave_num);
      Dev_UART0:  return fromInteger (uart0_slave_num);
      Dev_GPIO:   return fromInteger (accel0_slave_num);
      Dev_SoCInfo: return fromInteger (soc_info_slave_num);
      default:    return fromInteger (0);
   endcase
endfunction

// ================================================================
// Interface and module for the address map


interface SoC_Map_IFC;
   (* always_ready *)   method  Fabric_Addr  m_near_mem_io_addr_base;
   (* always_ready *)   method  Fabric_Addr  m_near_mem_io_addr_size;
   (* always_ready *)   method  Fabric_Addr  m_near_mem_io_addr_lim;

   (* always_ready *)   method  Fabric_Addr  m_plic_addr_base;
   (* always_ready *)   method  Fabric_Addr  m_plic_addr_size;
   (* always_ready *)   method  Fabric_Addr  m_plic_addr_lim;

   (* always_ready *)   method  Fabric_Addr  m_uart0_addr_base;
   (* always_ready *)   method  Fabric_Addr  m_uart0_addr_size;
   (* always_ready *)   method  Fabric_Addr  m_uart0_addr_lim;

`ifdef INCLUDE_ACCEL0
   (* always_ready *)   method  Fabric_Addr  m_accel0_addr_base;
   (* always_ready *)   method  Fabric_Addr  m_accel0_addr_size;
   (* always_ready *)   method  Fabric_Addr  m_accel0_addr_lim;
`endif

   (* always_ready *)   method  Fabric_Addr  m_boot_rom_addr_base;
   (* always_ready *)   method  Fabric_Addr  m_boot_rom_addr_size;
   (* always_ready *)   method  Fabric_Addr  m_boot_rom_addr_lim;

   (* always_ready *)   method  Fabric_Addr  m_mem0_controller_addr_base;
   (* always_ready *)   method  Fabric_Addr  m_mem0_controller_addr_size;
   (* always_ready *)   method  Fabric_Addr  m_mem0_controller_addr_lim;

   (* always_ready *)   method  Fabric_Addr  m_soc_info_addr_base;
   (* always_ready *)   method  Fabric_Addr  m_soc_info_addr_size;
   (* always_ready *)   method  Fabric_Addr  m_soc_info_addr_lim;

   (* always_ready *)   method  Fabric_Addr  m_tcm_addr_base;
   (* always_ready *)   method  Fabric_Addr  m_tcm_addr_size;
   (* always_ready *)   method  Fabric_Addr  m_tcm_addr_lim;

   (* always_ready *)
   method  Bool  m_is_mem_addr (Fabric_Addr addr);

   (* always_ready *)
   method  Bool  m_is_IO_addr (Fabric_Addr addr);

   (* always_ready *)
   method  Bool  m_is_near_mem_IO_addr (Fabric_Addr addr);

   (* always_ready *)   method  Bit #(64)  m_pc_reset_value;
   (* always_ready *)   method  Bit #(64)  m_mtvec_reset_value;

   // Non-maskable interrupt vector
   (* always_ready *)   method  Bit #(64)  m_nmivec_reset_value;
endinterface

// ================================================================

module mkSoC_Map (SoC_Map_IFC);

   // Elaboration-time manifest for software: emitted from the SAME
   // declarative table the AXI fabric decodes (fn_soc_memmap), so the
   // JSON/HolyC map derived from this log always matches the hardware
   // actually elaborated (same config flags, same constants).  The
   // Makefile captures the bsc output; scripts/soc_manifest.py turns
   // these lines into build/soc-memory-map.json.
   function Action fn_emit_region (MemRegion r);
      return messageM ("MEMMAP " + r.name + " "
                       + bitToString (r.base) + " "
                       + bitToString (r.size) + " "
                       + r.bus + " "
                       + (r.fabric_decoded ? "decoded" : "no-slave"));
   endfunction

   function Action fn_emit_all (List #(MemRegion) rs);
      if (List::isNull (rs))
         return noAction;
      else
         return
            action
               fn_emit_region (List::head (rs));
               fn_emit_all (List::tail (rs));
            endaction;
   endfunction

   // The rule body contains only messageM calls, which execute at
   // ELABORATION time (the rule itself is removed: no runtime actions).
   rule rl_emit_memmap_manifest;
      fn_emit_all (fn_soc_memmap ());
`ifdef Near_Mem_TCM
      messageM ("MEMMAP WARNING Near_Mem_TCM is set but fn_soc_memmap has no TCM entry; the table-driven fabric will not decode it");
`endif
`ifdef HTIF_MEMORY
      messageM ("MEMMAP WARNING HTIF_MEMORY is set but fn_soc_memmap has no HTIF entry; the table-driven fabric will not decode it");
`endif
      messageM ("MEMMAPIRQ uart0 " + integerToString (irq_num_uart0) + " core_external");
`ifdef INCLUDE_ACCEL0
      messageM ("MEMMAPIRQ gpio " + integerToString (irq_num_accel0) + " core_external");
`endif
   endrule

   // ----------------------------------------------------------------
   // Tightly-coupled memory ('TCM'; optional)

`ifdef Near_Mem_TCM
// Integer kB_per_TCM = 'h4;         // 4KB
// Integer kB_per_TCM = 'h40;     // 64KB
// Integer kB_per_TCM = 'h80;     // 128KB
// Integer kB_per_TCM = 'h400;    // 1 MB
   Integer kB_per_TCM = 'h4000;    // 16 MB
`else
   Integer kB_per_TCM = 0;
`endif
   Integer bytes_per_TCM = kB_per_TCM * 'h400;

   Fabric_Addr tcm_addr_base = 'h_0000_0000;
   Fabric_Addr tcm_addr_size = fromInteger (bytes_per_TCM);
   Fabric_Addr tcm_addr_lim  = tcm_addr_base + tcm_addr_size;

   function Bool fn_is_tcm_addr (Fabric_Addr addr);
      return ((tcm_addr_base <= addr) && (addr < tcm_addr_lim));
   endfunction

   // ----------------------------------------------------------------
   // Memory address predicate
   // Identifies memory addresses in the Fabric.
   // (Caches need this information to cache these addresses.)

   function Bool fn_is_mem_addr (Fabric_Addr addr);
`ifdef RVFI
      // riscv-formal build: every address is cacheable memory (the formal AXI
      // slave answers everything), so symbolic addresses never fault.
      return True;
`else
      return (   fn_is_boot_rom_addr (addr)
	      || fn_is_mem0_controller_addr (addr)
	      || fn_is_tcm_addr (addr)
	      );
`endif
   endfunction

   // ----------------------------------------------------------------
   // I/O address predicate
   // Identifies I/O addresses in the Fabric.
   // (Caches need this information to avoid cacheing these addresses.)

   function Bool fn_is_IO_addr (Fabric_Addr addr);
`ifdef RVFI
      return False;
`else
      return (   fn_is_near_mem_io_addr (addr)
	      || fn_is_plic_addr (addr)
	      || fn_is_uart0_addr  (addr)
	      || ((soc_info_addr_base <= addr) && (addr < soc_info_addr_lim))
`ifdef INCLUDE_ACCEL0
	      || fn_is_accel0_addr  (addr)
`endif
	      );
`endif
   endfunction

   // ----------------------------------------------------------------
   // PC, MTVEC and NMIVEC reset values

   Bit #(64) pc_reset_value     = boot_rom_addr_base;
   Bit #(64) mtvec_reset_value  = 'h1000;    // TODO

   // Non-maskable interrupt vector
   Bit #(64) nmivec_reset_value = ?;         // TODO

   // ================================================================
   // INTERFACE

   method  Fabric_Addr  m_near_mem_io_addr_base = near_mem_io_addr_base;
   method  Fabric_Addr  m_near_mem_io_addr_size = near_mem_io_addr_size;
   method  Fabric_Addr  m_near_mem_io_addr_lim  = near_mem_io_addr_lim;

   method  Fabric_Addr  m_plic_addr_base = plic_addr_base;
   method  Fabric_Addr  m_plic_addr_size = plic_addr_size;
   method  Fabric_Addr  m_plic_addr_lim  = plic_addr_lim;

   method  Fabric_Addr  m_uart0_addr_base = uart0_addr_base;
   method  Fabric_Addr  m_uart0_addr_size = uart0_addr_size;
   method  Fabric_Addr  m_uart0_addr_lim  = uart0_addr_lim;

`ifdef INCLUDE_ACCEL0
   method  Fabric_Addr  m_accel0_addr_base = accel0_addr_base;
   method  Fabric_Addr  m_accel0_addr_size = accel0_addr_size;
   method  Fabric_Addr  m_accel0_addr_lim  = accel0_addr_lim;
`endif

   method  Fabric_Addr  m_boot_rom_addr_base = boot_rom_addr_base;
   method  Fabric_Addr  m_boot_rom_addr_size = boot_rom_addr_size;
   method  Fabric_Addr  m_boot_rom_addr_lim  = boot_rom_addr_lim;

   method  Fabric_Addr  m_mem0_controller_addr_base = mem0_controller_addr_base;
   method  Fabric_Addr  m_soc_info_addr_base = soc_info_addr_base;
   method  Fabric_Addr  m_soc_info_addr_size = soc_info_addr_size;
   method  Fabric_Addr  m_soc_info_addr_lim  = soc_info_addr_lim;
   method  Fabric_Addr  m_mem0_controller_addr_size = mem0_controller_addr_size;
   method  Fabric_Addr  m_mem0_controller_addr_lim  = mem0_controller_addr_lim;

   method  Fabric_Addr  m_tcm_addr_base = tcm_addr_base;
   method  Fabric_Addr  m_tcm_addr_size = tcm_addr_size;
   method  Fabric_Addr  m_tcm_addr_lim  = tcm_addr_lim;

   method  Bool  m_is_mem_addr (Fabric_Addr addr) = fn_is_mem_addr (addr);

   method  Bool  m_is_IO_addr (Fabric_Addr addr) = fn_is_IO_addr (addr);

   method  Bool  m_is_near_mem_IO_addr (Fabric_Addr addr) = fn_is_near_mem_io_addr (addr);

   method  Bit #(64)  m_pc_reset_value     = pc_reset_value;
   method  Bit #(64)  m_mtvec_reset_value  = mtvec_reset_value;

   // Non-maskable interrupt vector
   method  Bit #(64)  m_nmivec_reset_value = nmivec_reset_value;
endmodule

// ================================================================
// Count and master-numbers of masters in the fabric.

Integer imem_master_num   = 0;
Integer dmem_master_num   = 1;
Integer accel0_master_num = 2;

`ifdef INCLUDE_ACCEL0

typedef 3 Num_Masters;

`else

typedef 2 Num_Masters;

`endif

// ================================================================
// Count and slave-numbers of slaves in the fabric.

`ifdef INCLUDE_ACCEL0

typedef 5 Num_Slaves;

`else

typedef 4 Num_Slaves;

`endif


Integer boot_rom_slave_num        = 0;
Integer mem0_controller_slave_num = 1;
Integer uart0_slave_num           = 2;
Integer accel0_slave_num          = 3;
Integer soc_info_slave_num        = 4;

// ================================================================
// Interrupt request numbers (== index in to vector of
// interrupt-request lines in Core)

typedef  16  N_External_Interrupt_Sources;
Integer  n_external_interrupt_sources = valueOf (N_External_Interrupt_Sources);

Integer irq_num_uart0  = 0;
Integer irq_num_accel0 = 1;

// ================================================================

endpackage
