// Copyright (c) 2016-2022 Bluespec, Inc. All Rights Reserved.

package MMU_Cache;

// ================================================================
// A combined MMU and L1 Cache for RISC-V.
// The MMU is capable of handling pages, superpages and gigapages.
// The cache is simple, in-order, blocking, and has a "write-around" policy:
//    All writes (hits and misses) write back to fabric.
//    On write-hit, also update line in cache.
//    On write-miss: don't refill line.
// Thus, cache lines are always clean, never written back.

// Handles LD, ST, AMO_LR, AMO_SC, and remaining AMO_ops.
// Does VA-to-PA addr translation, but bypasses caches, for IO addresses.

// This MMU_Cache is parameterized for data-width on both the front
// side interface (facing CPU) and the back side interface (facing
// fabric).

// CPU-facing interface: can be used for both RV32 and RV64 CPUs.
// RV32 vs. RV64 only affects width of some CPU-side interface
// ports:
//    - inputs req 'addr' and 'satp'    (type WordXL)
//    - output response 'addr' (copy of requesting addr)    (type WordXL)
//    - output response load-value and input request store-value are
//        always 64b because of double-precision floating point LD/ST
//        in RV32
// For RV32, a cache line is 8 x 32b words.
// For RV64, a cache line is 8 x 64b words.

// Fabric-facing interface: AXI4, with data width 32b or 64b (type Wd_Data).

// Internally, the data RAM width is fixed at 64b.

// After MMU translation, there is a 2-way triage based on physical addr:
//  - Memory addrs: request goes to the cache logic
//                      (back end of cache logic talks to fabric interface)
//  - IO:           request does directly to fabric interface (no cacheing)

// ================================================================
// BSV lib imports

import Vector       :: *;
import BRAMCore     :: *;
import ConfigReg    :: *;
import FIFOF        :: *;
import SpecialFIFOs :: *;
import GetPut       :: *;
import ClientServer :: *;

// ----------------
// BSV additional libs

import Cur_Cycle     :: *;
import GetPut_Aux    :: *;
import Semi_FIFOF    :: *;
import CreditCounter :: *;

// ================================================================
// Project imports

import ISA_Decls        :: *;
import Near_Mem_IFC     :: *;
import MMU_Cache_Common :: *;
import Cache_Decls      :: *;

`ifdef ISA_PRIV_S
import TLB :: *;
`endif

import SoC_Map      :: *;
import AXI4_Types   :: *;
import Fabric_Defs  :: *;

// ================================================================

export  MMU_Cache_IFC (..),  mkMMU_Cache;

// ================================================================
// MMU_Cache interface

interface MMU_Cache_IFC;
   method Action set_verbosity (Bit #(4) verbosity);

   // Reset request/response
   interface Server #(Token, Token) server_reset;

   // CPU interface: request
   (* always_ready *)
   method Action  req (CacheOp op,
		       Bit #(3) f3,
`ifdef ISA_A
		       Bit #(7) amo_funct7,
`endif
		       WordXL addr,
		       CWord st_value,
		       // The following  args for VM
		       Priv_Mode  priv,
		       Bit #(1)   sstatus_SUM,
		       Bit #(1)   mstatus_MXR,
		       WordXL     satp);    // { VM_Mode, ASID, PPN_for_page_table }

   // CPU interface: response
   (* always_ready *)  method Bool       valid;
   (* always_ready *)  method WordXL     addr;        // req addr for which this is a response
   (* always_ready *)  method CWord  cword;      // rd_val data for LD, LR, AMO, SC success/fail result)
   (* always_ready *)  method CWord  st_amo_val;  // Final stored value for ST, SC, AMO
   (* always_ready *)  method Bool       exc;
   (* always_ready *)  method Exc_Code   exc_code;

   // CPU interface: response-echo kill (RLD-4).  The CPU (CPU_Stage2.fa_enq)
   // sets this on every Stage2 enqueue; the next dmem req (this module's
   // 'req' method) clears it.  While set -- i.e. for exactly the one cycle
   // between an op entering Stage2 and its deferred request launch -- the
   // response drivers (rl_probe_and_immed_rsp and the three lingering-valid
   // states) are silenced and the linger states exit to MODULE_READY.
   //
   // Why: stock near-mem safety rests on the next dmem req arriving at
   // Stage2-enqueue time, clobbering any stale response echo (probe
   // re-drive or linger state) no later than the cycle the next op becomes
   // visible in Stage2.  The RLD-4 deferred launch makes the clobber one
   // cycle late; without this kill, a newly fed op would sample the
   // previous op's response.  Register-write + register-read only: no
   // same-cycle observation channel, so no Enable->ReturnValue cycle
   // (G0033 class) can form.
   //
   // Safe on every enqueue (incl. non-dmem ops): a live response's
   // consumer is by construction the op in Stage2, and Stage2 is empty
   // at enqueue; any echo still present is stale.
   (* always_ready *)
   method Action  ma_kill_echo;

   // Cache flush request/response
   interface Server #(Token, Token) server_flush;

   // TLB flush
   method Action tlb_flush;

   // Fabric master interface
   interface AXI4_Master_IFC #(Wd_Id, Wd_Addr, Wd_Data, Wd_User) mem_master;

   // ----------------------------------------------------------------
   // Misc. control and status

   // ----------------
   // For ISA tests: watch memory writes to <tohost> addr

`ifdef WATCH_TOHOST
   method Action set_watch_tohost (Bool watch_tohost, Bit #(64) tohost_addr);
   method Bit #(64) mv_tohost_value;
`endif

   // Inform core that DDR4 has been initialized and is ready to accept requests
   method Action ma_ddr4_ready;

   // Misc. status; 0 = running, no error
   (* always_ready *)
   method Bit #(8) mv_status;

   // Performance-monitoring tap: pulses for one cycle when a cache-line
   // refill starts (i.e., a cache miss commits to refilling).
   (* always_ready *)
   // E8 (2026-10-17): True while a fabric transaction may be outstanding
   // (refill, IO, PTW, ST/AMO response, reset).  'req' has no flow control
   // (see its NOTE): a caller that issues req during a refill abandons the
   // AXI read burst in flight, and the NEXT refill then consumes that burst
   // as its own data (the E5 cache-test wedge).  Callers must gate on this.
   method Bool       mv_busy;

   method Bool mv_perf_miss;

endinterface

// ****************************************************************
// ****************************************************************
// ****************************************************************
// Internal types and constants

typedef enum { CTAG_EMPTY, CTAG_CLEAN } CTagState
deriving (Bits, Eq, FShow);

typedef struct {
   CTagState  state;
   CTag       ctag;
   } State_and_CTag
deriving (Bits, FShow);

typedef Vector #(Ways_per_CSet, State_and_CTag) State_and_CTag_CSet;
typedef Vector #(Ways_per_CSet, CWord)          CWord_Set;

typedef enum {MODULE_PRERESET,              // After power on reset, before soft reset
              MODULE_RESETTING,             // Clearing all tags to EMPTY state
              MODULE_READY,                 // Reset done, ready for first request
              MODULE_RUNNING,               // Normal operation, during hits
              MODULE_EXCEPTION_RSP,         // On misaligned and access exceptions

              PTW_START,                    // On TLB miss, initiate refill of PTE into TLB
`ifdef RV64
	      PTW_LEVEL_2,                  // Page Table Walk, Request Level 2
`endif
	      PTW_LEVEL_1,                  // Page Table Walk, Request Level 1
	      PTW_LEVEL_0,                  // Page Table Walk, Request Level 0

              CACHE_START_REFILL,           // On cache miss, initiate refill of cache line in cache
              CACHE_SNAPSHOT,               // Snapshot victim line's other-way words (BRAM read-only)
              CACHE_REFILL,                 // Refill
              CACHE_REREQ,                  // After refill, redo request that missed
              CACHE_AMO_EXEC,               // E12: AMO hit captured at probe; ALU + write here
              CACHE_ST_AMO_RSP,             // Provide ST/SC/AMO response

              IO_REQ,                       // For memory-mapped I/O requests
              IO_AWAITING_READ_RSP,         // No caching
              IO_READ_RSP,                  // Provide IO-read response

	      IO_AWAITING_AMO_READ_RSP
   } Module_State
deriving (Bits, Eq, FShow);

Bool bram_cmd_read  = False;
Bool bram_cmd_write = True;

// The reset-loop is run based on requests for reset and requests for flush
typedef enum {REQUESTOR_RESET_IFC, REQUESTOR_FLUSH_IFC} Requestor
deriving (Bits, Eq, FShow);

Bit #(Wd_User) dummy_user = ?;    // For AXI4 'user' field (here unused)

`ifndef ISA_PRIV_S

// VM Xlate related definitions which are only for the case where there is no
// VM, effectively making the following definitions, dummy ones. If VM, these
// definitions are taken from the TLB package, and include fields like the PTE

typedef enum { VM_XLATE_OK, VM_XLATE_TLB_MISS, VM_XLATE_EXCEPTION } VM_Xlate_Outcome
deriving (Bits, Eq, FShow);

typedef struct {
   VM_Xlate_Outcome   outcome;
   PA                 pa;            // phys addr, if VM_XLATE_OK
   Exc_Code           exc_code;      // if VM_XLATE_EXC
   } VM_Xlate_Result
deriving (Bits, FShow);

`endif

// ================================================================
// Convert RISC-V funct3 code into AXI4_Size code (number of bytes in a beat)

function AXI4_Size fn_funct3_to_AXI4_Size (Bit #(3) funct3);
   Bit #(2)   x = funct3 [1:0];
   AXI4_Size  result;
   if      (x == f3_SIZE_B)        result = axsize_1;
   else if (x == f3_SIZE_H)        result = axsize_2;
   else if (x == f3_SIZE_W)        result = axsize_4;
   else /* if (x == f3_SIZE_D) */  result = axsize_8;
   return result;
endfunction

// ================================================================
// Compute address, data and strobe (byte-enables) for writes to fabric

function Tuple4 #(Fabric_Addr,    // addr is 32b- or 64b-aligned
		  Fabric_Data,    // data is lane-aligned
		  Fabric_Strb,    // strobe
		  AXI4_Size)      // 8 for 8-byte writes, else 4

   fn_to_fabric_write_fields (Bit #(3)  f3,      // RISC-V size code: B/H/W/D
			      Bit #(n)  addr,    // actual byte addr
			      Bit #(64) word64)  // data is in lsbs
   provisos (Add #(_, n, 64));

   // First compute addr, data and strobe for a 64b-wide fabric
   Bit #(8)   strobe64    = 0;
   Bit #(3)   shift_bytes = addr [2:0];
   Bit #(6)   shift_bits  = { shift_bytes, 3'b0 };
   Bit #(64)  addr64      = zeroExtend (addr);
   AXI4_Size  axsize      = axsize_128;    // Will be updated in 'case' below

   case (f3 [1:0])
      f3_SIZE_B: begin
		    word64   = (word64 << shift_bits);
		    strobe64 = ('b_1   << shift_bytes);
		    axsize   = axsize_1;
		 end
      f3_SIZE_H: begin
		    word64   = (word64 << shift_bits);
		    strobe64 = ('b_11  << shift_bytes);
		    axsize   = axsize_2;
		 end
      f3_SIZE_W: begin
		    word64   = (word64  << shift_bits);
		    strobe64 = ('b_1111 << shift_bytes);
		    axsize   = axsize_4;
		 end
      f3_SIZE_D: begin
		    strobe64 = 'b_1111_1111;
		    axsize   = axsize_8;
		 end
   endcase

   // Adjust for 32b fabrics
   if ((valueOf (Wd_Data) == 32) && (addr [2] == 1'b1)) begin
      word64   = { 32'h0, word64 [63:32] };
      strobe64 = { 4'h0, strobe64 [7:4] };
   end

   // Finally, create fabric addr/data/strobe
   Fabric_Addr  fabric_addr   = truncate (addr64);
   Fabric_Data  fabric_data   = truncate (word64);
   Fabric_Strb  fabric_strobe = truncate (strobe64);

   return tuple4 (fabric_addr, fabric_data, fabric_strobe, axsize);
endfunction: fn_to_fabric_write_fields

// ================================================================
// Update a byte, halfword, word or doubleword in a CWord at Way in a CWord_Set

function CWord_Set fn_update_cword_set (CWord_Set   old_cword_set,
					Way_in_CSet way,
					Bit #(n)    addr,
					Bit #(3)    f3,
					CWord       cword)
   provisos (Add#(_, 64, SizeOf #(CWord)));
   let old_cword     = old_cword_set [way];
   let old_B0        = old_cword [7:0];
   let old_B1        = old_cword [15:8];
   let old_B2        = old_cword [23:16];
   let old_B3        = old_cword [31:24];
   let old_B4        = old_cword [39:32];
   let old_B5        = old_cword [47:40];
   let old_B6        = old_cword [55:48];
   let old_B7        = old_cword [63:56];

   let new_cword_set = old_cword_set;
   let new_cword     = old_cword;
   Bit #(3) addr_lsbs = addr [2:0];

   // Replace relevant bytes in new_cword
   case (f3)
      f3_SB:  case (addr_lsbs)
		 'h0 : new_cword [ 7:0 ] = cword [7:0];
		 'h1 : new_cword [15:8 ] = cword [7:0];
		 'h2 : new_cword [23:16] = cword [7:0];
		 'h3 : new_cword [31:24] = cword [7:0];
		 'h4 : new_cword [39:32] = cword [7:0];
		 'h5 : new_cword [47:40] = cword [7:0];
		 'h6 : new_cword [55:48] = cword [7:0];
		 'h7 : new_cword [63:56] = cword [7:0];
	      endcase
      f3_SH:  case (addr_lsbs)
		 'h0 : new_cword [15:0 ] = cword [15:0];
		 'h2 : new_cword [31:16] = cword [15:0];
		 'h4 : new_cword [47:32] = cword [15:0];
		 'h6 : new_cword [63:48] = cword [15:0];
	      endcase
      f3_SW:  case (addr_lsbs)
		 'h0 : new_cword [31:0]  = cword [31:0];
		 'h4 : new_cword [63:32] = cword [31:0];
	      endcase
      f3_SD:  new_cword = cword;
   endcase
   new_cword_set [way] = new_cword;
   return new_cword_set;
endfunction: fn_update_cword_set

// ================================================================
// ALU for AMO ops.
// Args: ld_val (64b from mem) and st_val (64b from CPU reg Rs2)
// Result: (final_ld_val, final_st_val)

function Tuple2 #(Bit #(64),
		  Bit #(64)) fv_amo_op (Bit #(3)   funct3,    // encodes data size (.W or .D)
					Bit #(7)   funct7,    // encodes the AMO op
					WordXL     addr,      // lsbs indicate which 32b W in 64b D (.W)
					Bit #(64)  ld_val,    // 64b value loaded from mem
					Bit #(64)  st_val);   // 64b value from CPU reg Rs2
   return fv_amo_op_x (funct3, funct7, fn_extract_and_extend_bytes (funct3, addr, ld_val), st_val);
endfunction: fv_amo_op

// E14: the AMO ALU on an already-extracted operand (w1 = the addressed
// W/D of the loaded word, extended), so the cache path can register the
// extraction at probe time and keep it out of the rl_amo_exec cone.
function Tuple2 #(Bit #(64),
		  Bit #(64)) fv_amo_op_x (Bit #(3)   funct3,
					  Bit #(7)   funct7,
					  Bit #(64)  w1_extracted,
					  Bit #(64)  st_val);
   Bit #(64) w1     = w1_extracted;
   Bit #(64) w2     = st_val;
   Int #(64) i1     = unpack (w1);    // Signed, for signed ops
   Int #(64) i2     = unpack (w2);    // Signed, for signed ops
   if (funct3 == f3_AMO_W) begin
      w1 = zeroExtend (w1 [31:0]);
      w2 = zeroExtend (w2 [31:0]);
      i1 = unpack (signExtend (w1 [31:0]));
      i2 = unpack (signExtend (w2 [31:0]));
   end
   Bit #(5)  f5     = funct7 [6:2];
   // new_st_val is new value to be stored back to mem (w1 op w2)
   Bit #(64) new_st_val = ?;
   case (f5)
      f5_AMO_SWAP: new_st_val = w2;
      f5_AMO_ADD:  new_st_val = pack (i1 + i2);
      f5_AMO_XOR:  new_st_val = w1 ^ w2;
      f5_AMO_AND:  new_st_val = w1 & w2;
      f5_AMO_OR:   new_st_val = w1 | w2;
      f5_AMO_MINU: new_st_val = ((w1 < w2) ? w1 : w2);
      f5_AMO_MAXU: new_st_val = ((w1 > w2) ? w1 : w2);
      f5_AMO_MIN:  new_st_val = ((i1 < i2) ? w1 : w2);
      f5_AMO_MAX:  new_st_val = ((i1 > i2) ? w1 : w2);
   endcase

   if (funct3 == f3_AMO_W)
      new_st_val = zeroExtend (new_st_val [31:0]);

   return tuple2 (truncate (pack (i1)), new_st_val);
endfunction: fv_amo_op_x

// ================================================================
// Displays, for debugging

function Action fa_display_state_and_ctag_cset (CSet_in_Cache        cset_in_cache,
						State_and_CTag_CSet  state_and_ctag_cset);
   action
      $write ("        CSet 0x%0x: (state, tag):", cset_in_cache);
      for (Integer j = 0; j < ways_per_cset; j = j + 1) begin
	 $write (" (", fshow (state_and_ctag_cset [j].state));
	 if (state_and_ctag_cset [j].state == CTAG_EMPTY)
	    $write (", --");
	 else
	    $write (", 0x%0x", state_and_ctag_cset [j].ctag);
	 $write (")");
      end
      $write ("\n");
   endaction
endfunction

function Action fa_display_cword_set (CSet_in_Cache  cset_in_cache,
				      CWord_in_CLine cword_in_cline,
				      CWord_Set      cword_set);
   action
      $write ("        CSet 0x%0x, CWord 0x%0x: ", cset_in_cache, cword_in_cline);
      for (Integer j = 0; j < ways_per_cset; j = j + 1) begin
	 $write (" 0x%0x", cword_set [j]);
      end
      $write ("\n");
   endaction
endfunction

function Reg #(t) fn_genNullRegIfc (t x) provisos (Literal#(t));
   return (
      interface Reg;
         method _read = x;
         method _write (y) = noAction;
      endinterface
   );
endfunction

// ****************************************************************
// ****************************************************************
// ****************************************************************
// The module implementation
                
(* synthesize *)
module mkMMU_Cache  #(parameter Bool dmem_not_imem)  (MMU_Cache_IFC);

   String d_or_i = (dmem_not_imem ? "D_MMU_Cache" : "I_MMU_Cache");

   // Verbosity: 0: quiet; 1 reset info; 2: + detail; 3: cache refill loop detail
   Integer verbosity = (dmem_not_imem ? 0 : 0);
   Reg #(Bit #(4)) cfg_verbosity <- mkConfigReg (fromInteger (verbosity));

   // Overall state of this module
   Reg #(Module_State)  rg_state      <- mkReg (MODULE_PRERESET);
   Reg #(Bool)          rg_ddr4_ready <- mkReg (False);
   Reg #(Bool)          rg_wr_rsp_err <- mkReg (False);

   // SoC_Map is needed for method 'm_is_mem_addr' to distinguish mem
   // (cached) and other (non-cached) addrs
   SoC_Map_IFC soc_map <- mkSoC_Map;

   // Reset request/response: REQUESTOR_RESET_IFC, REQUESTOR_FLUSH_IFC
   FIFOF #(Requestor) f_reset_reqs <- mkFIFOF;
   FIFOF #(Requestor) f_reset_rsps <- mkFIFOF;

   // Fabric request/response
   AXI4_Master_Xactor_IFC #(Wd_Id, Wd_Addr, Wd_Data, Wd_User) master_xactor <- mkAXI4_Master_Xactor;

`ifdef ISA_PRIV_S
   // The TLB
   TLB_IFC  tlb <- mkTLB (dmem_not_imem);
`endif

   // For discarding write-responses
   CreditCounter_IFC #(4) ctr_wr_rsps_pending <- mkCreditCounter; // Max 15 writes outstanding

   // Cache RAMs
   // BRAM Port A is only used for writing
   // BRAM Port B is only used for reading
   Bool config_output_register = False;    // i.e., no output register
   // Tag RAM
   BRAM_DUAL_PORT #(CSet_in_Cache,
		    State_and_CTag_CSet)  ram_state_and_ctag_cset <- mkBRAMCore2 (csets_per_cache,
										  config_output_register);

   // Data RAM
   BRAM_DUAL_PORT #(CSet_CWord_in_Cache, CWord_Set) ram_cword_set <- mkBRAMCore2 (cset_cwords_per_cache,
										  config_output_register);

   // Registers holding incoming request args
   Reg #(CacheOp)    rg_op          <- mkRegU;    // CACHE_LD, CACHE_ST, CACHE_AMO
   Reg #(Bit #(3))   rg_f3          <- mkRegU;    // rg_f3[1:0] specifies B/H/W/D access size
`ifdef ISA_A
   Reg #(Bit #(7))   rg_amo_funct7  <- mkRegU;    // specifies which kind of AMO op
   // E12: an AMO hit is captured at probe and the AMO ALU, cache write and
   // writeback run one cycle later in rl_amo_exec (the probe-cycle cone
   // tag-compare -> AMO add/compare -> cword merge was the deepest in the SoC).
   Reg #(CWord_Set)   rg_amo_cword_set <- mkRegU;
   Reg #(Way_in_CSet) rg_amo_way       <- mkRegU;
   Reg #(Bit #(64))   rg_amo_w1        <- mkRegU;    // E14: extracted AMO operand
`endif
   Reg #(WordXL)     rg_addr        <- mkRegU;    // VA or PA
   Reg #(Bit #(64))  rg_st_amo_val  <- mkRegU;    // Store-value for ST, SC, AMO

   // The following are needed for VM
`ifdef ISA_PRIV_S
   Reg #(Priv_Mode)  rg_priv        <- mkRegU;    // Privilege level for this request
   Reg #(Bit #(1))   rg_sstatus_SUM <- mkRegU;    // SUM bit in SSTATUS CSR
   Reg #(Bit #(1))   rg_mstatus_MXR <- mkRegU;    // MXR bit in MSTATUS CSR

   Reg #(WordXL)     rg_satp        <- mkRegU;    // Copy of value in SATP CSR { VM_Mode, ASID, PPN }
`else
   // Dummy registers in non-VM mode
   Priv_Mode x = m_Priv_Mode;
   Reg #(Priv_Mode)  rg_priv        = fn_genNullRegIfc (x);

   Bit #(1) y = ?;
   Reg #(Bit #(1))   rg_sstatus_SUM = fn_genNullRegIfc (y);
   Reg #(Bit #(1))   rg_mstatus_MXR = fn_genNullRegIfc (y);

   WordXL z = ?;
   Reg #(WordXL)     rg_satp        = fn_genNullRegIfc (z);
`endif

   // Phys addr (initially taken from rg_addr; VM xlation may replace it)
   Reg #(PA)  rg_pa <- mkRegU;

`ifdef ISA_PRIV_S
   // Derivations from rg_addr (virtual addr)
   VA      va     = fn_WordXL_to_VA (rg_addr);
   VPN     vpn    = fn_Addr_to_VPN (va);
`ifdef RV64
   VPN_J   vpn_2  = fn_Addr_to_VPN_2 (va);
`endif
   VPN_J   vpn_1  = fn_Addr_to_VPN_1 (va);
   VPN_J   vpn_0  = fn_Addr_to_VPN_0 (va);
   Offset  offset = fn_Addr_to_Offset (rg_addr);
`endif

   CSet_in_Cache                 cset_in_cache       = fn_Addr_to_CSet_in_Cache  (rg_addr);
   CSet_CWord_in_Cache           cset_cword_in_cache = fn_Addr_to_CSet_CWord_in_Cache (rg_addr);
   CWord_in_CLine                cword_in_cline      = fn_Addr_to_CWord_in_CLine (rg_addr);
   Bit #(Bits_per_Byte_in_CWord) byte_in_cword = fn_Addr_to_Byte_in_CWord  (rg_addr);

`ifdef ISA_PRIV_S
   // Derivations from rg_satp
   VM_Mode  vm_mode  = fn_satp_to_VM_Mode (rg_satp);
   ASID     asid     = fn_satp_to_ASID    (rg_satp);
   PPN      satp_ppn = fn_satp_to_PPN     (rg_satp);
   PA       satp_pa  = fn_PPN_and_Offset_to_PA (satp_ppn, 12'b0);

   // We continuously probe the TLB with (asid, vpn)
   TLB_Lookup_Result  tlb_result = tlb.lookup (asid, vpn);
`endif

   // Outputs
   Reg #(Bool)      dw_valid             <- mkDWire (False);
   Reg #(Bool)      dw_exc               <- mkDWire (False);
   Reg #(Exc_Code)  rg_exc_code          <- mkRegU;
   Reg #(Exc_Code)  dw_exc_code          <- mkDWire (?);
   Reg #(Bit #(64)) rg_ld_val            <- mkRegU;         // Load-value for LOAD/LR/AMO, success/fail for SC
   Reg #(Bit #(64)) dw_output_ld_val     <- mkDWire (?);
   Reg #(Bit #(64)) dw_output_st_amo_val <- mkDWire (?);    // stored value for ST, SC, AMO (for verification only)

   // RLD-4: response-echo kill (see interface comment).  Set by the CPU on
   // every Stage2 enqueue (ma_kill_echo); cleared by the next req.  One
   // cycle of kill covers exactly the window between an op entering
   // Stage2 and its deferred request launch.
   Reg #(Bool)      rg_kill_echo         <- mkReg (False);

   // E8: one-cycle-stale mirror of "a fabric transaction may be outstanding",
   // read by mv_busy.  A ConfigReg so that external rules can read it in the
   // same rule as req/valid/exc without a scheduling conflict (a direct read
   // of rg_state is G0004 against req).  The one-cycle lag is safe: a req
   // arriving in the lag cycle conflicts with rl_start_cache_refill on
   // rg_state and the method wins, so the AR is never sent.
   Reg #(Bool) rg_busy_mirror <- mkConfigReg (False);
   (* fire_when_enabled, no_implicit_conditions *)
   rule rl_busy_mirror;
      rg_busy_mirror <= (   (rg_state != MODULE_READY)
			 && (rg_state != MODULE_RUNNING)
			 && (rg_state != MODULE_EXCEPTION_RSP)
			 // E8b: the two hold-the-response states park until the
			 // CPU's NEXT req (see rl_ST_AMO_response /
			 // rl_maintain_io_read_rsp); nothing is outstanding on
			 // the fabric there.  Counting them as busy deadlocks the
			 // pipeline after the first store (E8a: 'T' then hang).
			 && (rg_state != CACHE_ST_AMO_RSP)
			 && (rg_state != IO_READ_RSP));
   endrule

   // This reg is used during PTWs
   Reg #(PA) rg_pte_pa <- mkRegU;

`ifdef ISA_A
   // Reservation regs for AMO LR/SC (Load-Reserved/Store-Conditional)
   Reg #(Bool)     rg_lrsc_valid  <- mkReg (False);
   Reg #(PA)       rg_lrsc_pa     <- mkRegU;    // Phys. address for an active LR
`endif

   // This reg is used in the reset-loop when resetting all states
   Reg #(CSet_in_Cache)  rg_cset_in_cache   <- mkReg (0);

   // These regs are used in the cache refill loop for ram_State_and_CTag_CSet
   // and ram_cword_set
   Reg #(CSet_CWord_in_Cache) rg_cset_cword_in_cache <- mkRegU;
   Reg #(Bool)                rg_error_during_refill <- mkRegU;
   // Refill snapshot: the victim line's OTHER-way word64s, captured from the
   // data RAM before the refill loop starts writing (see rl_cache_snapshot).
   // The refill loop then never needs to read the data RAM while writing it,
   // which removes every read/write scheduling constraint on the BRAM ports
   // (a same-port read while a write is pending loses its data on real
   // DP16KD hardware even though Bluesim's BRAM model returns it).
   Reg #(Vector #(CWords_per_CLine, CWord)) rg_other_way_line <- mkRegU;
   Reg #(Bit #(8))                          rg_snap_cnt        <- mkReg (0);
   // In 32b fabrics, these hold the lower word32 while we're fetching the upper word32 of a word64
   Reg #(Bool)      rg_lower_word32_full <- mkReg (False);
   Reg #(Bit #(32)) rg_lower_word32      <- mkRegU;

   // Performance-monitoring: pulses when a cache miss commits to a refill.
   PulseWire pw_perf_miss <- mkPulseWire;

   // When a CSet is full and we need to replace a cache line due to a refill,
   // the victim is picked 'randomly' according to this register
   Reg #(Way_in_CSet)  rg_victim_way <- mkRegU;

`ifdef WATCH_TOHOST
   // See NOTE: "tohost" above.
   // "tohost" addr on which to monitor writes, for standard ISA tests.
   // These are set by the 'set_watch_tohost' method but are otherwise read-only.
   Reg #(Bool)      rg_watch_tohost <- mkReg (False);
   Reg #(Bit #(64)) rg_tohost_addr  <- mkReg ('h_8000_1000);
   Reg #(Bit #(64)) rg_tohost_value <- mkReg (0);
`endif

`ifdef BRAM_HAZARD_ASSERT
   // Simulation-only BRAM access-discipline assertions (see AGENTS.md,
   // "Bluesim vs hardware").  Bluesim's BRAM model serializes same-port
   // accesses and never loses a pending read; real ECP5 DP16KD hardware
   // DOES lose it (confirmed in the pipe2/pipe3 bring-up).  These guards
   // enforce the discipline that keeps the design out of that corner:
   //   port A = write-only, port B = read-only (both RAMs).
   // Sim builds define BRAM_HAZARD_ASSERT; synthesis never defines it, so
   // hardware cost is zero.
   //
   // Only the historically fatal class is guarded here: a read put on
   // port A (the pipe2/pipe3 bug).  Both helpers are PURE literal checks
   // at each put site -- no wires, no shared state -- because any
   // same-cycle observation channel from inside 'req' makes BSC compute
   // an Enable(imem_req) -> ReturnValue(imem_instr) edge and every CPU
   // rule self-dependent (G0033).  Cross-port same-cycle access is
   // unreachable in this FSM (A-writing rules live in reset/store/refill
   // states; B-read issuers live in READY/REREQ/SNAPSHOT states) and is
   // covered by the F1 strict Verilator BRAM model instead.

   function Action fa_haz_A_write (Bool is_tag, Bool write, Bit #(32) addr);
      action
	 if (write != bram_cmd_write) begin
	    $display ("%0d: %s ASSERT FAIL: %s port A READ put issued (port A is write-only), addr 0x%0h",
		      cur_cycle, d_or_i, is_tag ? "tag RAM" : "data RAM", addr);
	    $finish (1);
	 end
      endaction
   endfunction

   function Action fa_haz_B_read (Bool is_tag, Bool write, Bit #(32) addr);
      action
	 if (write != bram_cmd_read) begin
	    $display ("%0d: %s ASSERT FAIL: %s port B WRITE put issued (port B is read-only), addr 0x%0h",
		      cur_cycle, d_or_i, is_tag ? "tag RAM" : "data RAM", addr);
	    $finish (1);
	 end
      endaction
   endfunction
`endif

   // ----------------------------------------------------------------
   // This function initiates a read request on the 'B' ports of the rams
   // Invoked from original cache request method, and internally after refills

   function Action fa_req_ram_B (Addr addr);
      action
	 // Request tag RAM
	 let cset_in_cache = fn_Addr_to_CSet_in_Cache (addr);
	 ram_state_and_ctag_cset.b.put (bram_cmd_read, cset_in_cache,    ?);
`ifdef BRAM_HAZARD_ASSERT
	 fa_haz_B_read (True, bram_cmd_read, extend (cset_in_cache));
`endif

	 // Request data RAM
	 let cset_cword_in_cache = fn_Addr_to_CSet_CWord_in_Cache (addr);
	 ram_cword_set.b.put          (bram_cmd_read, cset_cword_in_cache, ?);
`ifdef BRAM_HAZARD_ASSERT
	 fa_haz_B_read (False, bram_cmd_read, extend (cset_cword_in_cache));
`endif

	 if (cfg_verbosity > 1)
	    $display ("    fa_req_ram_B tagCSet [0x%0x] cword_set [0x%0d]",
		      cset_in_cache, cset_cword_in_cache);
      endaction
   endfunction

   // ----------------------------------------------------------------
   // Outputs of RAM read-ports (B ports)
   //
   // The BRAM read responses are registered (rg_tag_ram_q / rg_data_ram_q)
   // via a free-running capture rule: a read launched by 'req' in cycle C0
   // has raw B-port output valid in C1, captured at the C1->C2 edge, and
   // is consumed by the probe in C2 (MODULE_PROBE).  This is +1 cycle on
   // hit latency, and buys a large slice of timing margin (previously:
   // BRAM DO -> hit test -> FSM -> fabric FIFO within one cycle).

   Reg #(State_and_CTag_CSet) rg_tag_ram_q  <- mkRegU;
   Reg #(CWord_Set)           rg_data_ram_q <- mkRegU;

   rule rl_capture_ram_outputs;
      rg_tag_ram_q    <= ram_state_and_ctag_cset.b.read;
      rg_data_ram_q   <= ram_cword_set.b.read;
   endrule

   let state_and_ctag_cset = rg_tag_ram_q;
   let cword_set           = rg_data_ram_q;

   // One wait cycle after a RAM B-port read is launched.  The launch
   // One wait cycle after a RAM B-port read is launched.  'req' (and
   // rl_rereq) toggle rg_req_seq; rl_age_ram_rsp copies the new level into
   // rg_age_seen one cycle later, when the capture registers above have
   // sampled the raw BRAM outputs.  The probe rule fires only when the two
   // levels match, so the hit test and all RAM-data consumers see the
   // registered (captured) values.  Two disjoint registers (no write
   // conflict with the req method, which must remain callable while the
   // cache is in MODULE_RUNNING).
   Reg #(Bool) rg_req_seq   <- mkReg (False);
   Reg #(Bool) rg_age_seen  <- mkReg (False);

   rule rl_age_ram_rsp ((rg_state == MODULE_RUNNING) && (rg_age_seen != rg_req_seq));
      rg_age_seen <= rg_req_seq;
   endrule

   // Test cache hit or miss; if hit, return which 'way', and the word64 data
   // ---- This pure function is an ActionValue only for the $display inside
   function ActionValue #(Tuple3 #(Bool, Way_in_CSet, Bit #(64))) fn_test_cache_hit_or_miss (CTag  pa_ctag);
      actionvalue
	 Bool        hit     = False;
	 Way_in_CSet way_hit = 0;
	 CWord       cword   = 0;

	 for (Integer way = 0; way < ways_per_cset; way = way + 1) begin
	    let hit_at_way  = (   (state_and_ctag_cset [way].state != CTAG_EMPTY)
			       && (state_and_ctag_cset [way].ctag  == pa_ctag));
	    let cword_at_way = cword_set [way];

	    // Assertion: cannot have > 1 hit in a set
	    if (hit && hit_at_way)
	       $display ("        ASSERTION ERROR: fn_test_cache_hit_or_miss: multiple hits in set at [%0d] and [%0d]",
			 way, way_hit);

	    hit   = hit || hit_at_way;
	    if (hit_at_way) way_hit = fromInteger (way);
	    cword = (cword | (cword_at_way & pack (replicate (hit_at_way))));
	 end

	 return tuple3 (hit, way_hit, cword);
      endactionvalue
   endfunction

   // Abbreviations testing for LR and SC (avoids ifdef clutter later)
`ifdef ISA_A
   Bool is_AMO    = (rg_op == CACHE_AMO);
   Bool is_AMO_LR = ((rg_op == CACHE_AMO) && (rg_amo_funct7 [6:2] == f5_AMO_LR));
   Bool is_AMO_SC = ((rg_op == CACHE_AMO) && (rg_amo_funct7 [6:2] == f5_AMO_SC));
`else
   Bool is_AMO    = False;
   Bool is_AMO_LR = False;
   Bool is_AMO_SC = False;
`endif

   Exc_Code access_exc_code     = fn_access_exc_code     (dmem_not_imem, ((rg_op == CACHE_LD) || is_AMO_LR));

`ifdef ISA_PRIV_S
   Exc_Code page_fault_exc_code = fn_page_fault_exc_code (dmem_not_imem, ((rg_op == CACHE_LD) || is_AMO_LR));
`endif

   // ----------------------------------------------------------------
   // Functions to drive read-responses (outputs)

   // Memory-read responses
   function Action fa_drive_mem_rsp (Bit #(3) f3, Addr addr, Bit #(64) ld_val, Bit #(64) st_amo_val);
      action
	 dw_valid             <= True;
	 // Value loaded into rd (LOAD, LR, AMO, SC success/fail result)
	 dw_output_ld_val     <= (is_AMO_SC ? ld_val : fn_extract_and_extend_bytes (f3, addr, ld_val));
	 // Value stored into mem (STORE, SC, AMO final value stored)
	 dw_output_st_amo_val <= st_amo_val;
	 if (cfg_verbosity > 1)
	    $display ("%0d: %s.drive_mem_rsp: addr 0x%0h ld_val 0x%0h st_amo_val 0x%0h",
		      cur_cycle, d_or_i, addr, ld_val, st_amo_val);
      endaction
   endfunction

   // IO-read responses
   function Action fa_drive_IO_read_rsp (Bit #(3) f3, Addr addr, Bit #(64) ld_val);
      action
	 dw_valid         <= True;
	 // Value loaded into rd (LOAD, LR, AMO, SC success/fail result)
	 dw_output_ld_val <= ld_val;
	 if (cfg_verbosity > 1)
	    $display ("%0d: %s.drive_IO_read_rsp: addr 0x%0h ld_val 0x%0h", cur_cycle, d_or_i, addr, ld_val);
      endaction
   endfunction

   // Send a read-request into the fabric
   function Action fa_fabric_send_read_req (Fabric_Addr  addr, AXI4_Size  size);
      action
	 let mem_req_rd_addr = AXI4_Rd_Addr {arid:     fabric_default_id,
					     araddr:   addr,
					     arlen:    0,           // burst len = arlen+1
					     arsize:   size,
					     arburst:  fabric_default_burst,
					     arlock:   fabric_default_lock,
					     arcache:  fabric_default_arcache,
					     arprot:   fabric_default_prot,
					     arqos:    fabric_default_qos,
					     arregion: fabric_default_region,
					     aruser:   fabric_default_user};

	 master_xactor.i_rd_addr.enq (mem_req_rd_addr);

	 // Debugging
	 if (cfg_verbosity > 1) begin
	    $display ("            To fabric: ", fshow (mem_req_rd_addr));
	 end
      endaction
   endfunction

   // Send a read-burst request into the fabric to get a cache line.
   // 'addr' is already aligned to a cache-line.
   function Action fa_fabric_send_read_burst_req (Fabric_Addr  addr);
      action
	 AXI4_Size size = ((bytes_per_fabric_data == 4) ? axsize_4 : axsize_8);
	 // Note: AXI4 codes a burst length of 'n' as 'n-1'
	 AXI4_Len  len  = fromInteger ((bytes_per_cline / bytes_per_fabric_data) - 1);

	 let mem_req_rd_addr = AXI4_Rd_Addr {arid:     fabric_default_id,
					     araddr:   addr,
					     arlen:    len,
					     arsize:   size,
					     arburst:  axburst_incr,
					     arlock:   fabric_default_lock,
					     arcache:  fabric_default_arcache,
					     arprot:   fabric_default_prot,
					     arqos:    fabric_default_qos,
					     arregion: fabric_default_region,
					     aruser:   fabric_default_user};

	 master_xactor.i_rd_addr.enq (mem_req_rd_addr);

	 // Debugging
	 if (cfg_verbosity > 1) begin
	    $display ("    To fabric: ", fshow (mem_req_rd_addr));
	 end
      endaction
   endfunction

   FIFOF #(Tuple3 #(Bit #(3), PA, Bit #(64))) f_fabric_write_reqs <- mkFIFOF;

   // Send a write-request into the fabric
   function Action fa_fabric_send_write_req (Bit #(3)  f3, PA  pa, Bit #(64)  st_val);
      action
	 f_fabric_write_reqs.enq (tuple3 (f3, pa, st_val));
      endaction
   endfunction

   rule rl_fabric_send_write_req (rg_ddr4_ready);
      match { .f3, .pa, .st_val } <- pop (f_fabric_write_reqs);

      match {.fabric_addr,
	     .fabric_data,
	     .fabric_strb,
	     .fabric_size} = fn_to_fabric_write_fields (f3, pa, st_val);

      let mem_req_wr_addr = AXI4_Wr_Addr {awid:     fabric_default_id,
					  awaddr:   fabric_addr,
					  awlen:    0,           // burst len = awlen+1
					  awsize:   fabric_size,
					  awburst:  fabric_default_burst,
					  awlock:   fabric_default_lock,
					  awcache:  fabric_default_awcache,
					  awprot:   fabric_default_prot,
					  awqos:    fabric_default_qos,
					  awregion: fabric_default_region,
					  awuser:   fabric_default_user};

      let mem_req_wr_data = AXI4_Wr_Data {wdata:  fabric_data,
					  wstrb:  fabric_strb,
					  wlast:  True,
					  wuser:  fabric_default_user};

      master_xactor.i_wr_addr.enq (mem_req_wr_addr);
      master_xactor.i_wr_data.enq (mem_req_wr_data);

      // Expect a fabric response
      ctr_wr_rsps_pending.incr;

      // Debugging
      if (cfg_verbosity > 1) begin
	 $display ("            To fabric: ", fshow (mem_req_wr_addr));
	 $display ("                       ", fshow (mem_req_wr_data));
      end
   endrule

   // ================================================================
   // When PTE.A or PTE.D is updated, this function records it in the TLB
   // and enqueues a writeback to memory.

`ifdef ISA_PRIV_S
   FIFOF #(Tuple2 #(PA, PTE)) f_pte_writebacks <- mkFIFOF;

   function Action fa_record_pte_A_D_updates (TLB_Lookup_Result  tlb_result1,  VM_Xlate_Result  vm_xlate_result);
      action
	 if (vm_xlate_result.pte_modified) begin
	    // Update the TLB
	    tlb.insert (asid, vpn, vm_xlate_result.pte, tlb_result1.pte_level, tlb_result1.pte_pa);
	    // Enqueue it to be written back to memory
	    f_pte_writebacks.enq (tuple2 (tlb_result1.pte_pa, vm_xlate_result.pte));
	    if (cfg_verbosity >= 2) begin
	       $display ("    fa_record_pte_A_D_updates:");
	       $display ("      ", fshow (tlb_result1));
	       $display ("      ", fshow (vm_xlate_result));
	    end
	 end
      endaction
   endfunction

   rule rl_writeback_updated_PTE;
      match { .pa, .pte } <- pop (f_pte_writebacks);
      let f3 = ((xlen == 32) ? f3_SW : f3_SD);
      fa_fabric_send_write_req (f3, pa, zeroExtend (pte));
   endrule
`endif

   // ================================================================
   // BEHAVIOR

   // ----------------------------------------------------------------
   // Reset

   // Reset epoch, sent as the low ARID/AWID bits and bumped with every reset of the master
   // transactor.  A reset abandons the transactions in flight, but the SoC resets the fabric
   // a few cycles before this cache, so responses can still arrive afterwards: requests issued
   // in that gap (or still held by a slave the SoC does not reset).  The fabric routes R and B
   // in order, so taking one of them as the answer to a new request would shift every later
   // response.  mem_master drops R/B beats whose ID carries an older epoch.  Every slave on
   // the D-side path echoes ARID/AWID.
   Reg #(Bit #(2)) rg_epoch <- mkReg (0);

   rule rl_start_reset ((f_reset_reqs.notEmpty) && (rg_state != MODULE_RESETTING));
      rg_state             <= MODULE_RESETTING;
      rg_cset_in_cache     <= 0;
      rg_lower_word32_full <= False;

      // Flush the TLB
`ifdef ISA_PRIV_S
      tlb.flush;
`endif

`ifdef ISA_A
      rg_lrsc_valid  <= False;
`endif

      if (f_reset_reqs.first == REQUESTOR_RESET_IFC) begin
	 master_xactor.reset;
	 ctr_wr_rsps_pending.clear;
	 rg_epoch <= rg_epoch + 1;
      end

      $display ("%0d: %s: cache size %0d KB, associativity %0d, line size %0d bytes (= %0d XLEN words)",
		cur_cycle, d_or_i, kb_per_cache, ways_per_cset,
		(cwords_per_cline * 8),
`ifdef RV32
		(cwords_per_cline * 2)
`else
		(cwords_per_cline * 1)
`endif
		);
   endrule

   // This rule loops over csets, setting state of each cline in the set to EMPTY
   rule rl_reset (rg_state == MODULE_RESETTING);
      let state_and_ctag = State_and_CTag { state: CTAG_EMPTY, ctag: ? };
      ram_state_and_ctag_cset.a.put (bram_cmd_write, rg_cset_in_cache, replicate (state_and_ctag));
`ifdef BRAM_HAZARD_ASSERT
      fa_haz_A_write (True, bram_cmd_write, extend (rg_cset_in_cache));
`endif

      if (rg_cset_in_cache == fromInteger (csets_per_cache - 1)) begin
	 // This is the last cset; exit the loop
	 let requestor <- pop (f_reset_reqs);
	 f_reset_rsps.enq (requestor);
	 rg_state <= MODULE_READY;

	 if ((cfg_verbosity != 0) && (requestor == REQUESTOR_RESET_IFC))
	    $display ("%0d: %s.rl_reset: %0d sets x %0d ways: all tag states reset to CTAG_EMPTY",
		      cur_cycle, d_or_i, csets_per_cache, ways_per_cset);
	 if ((cfg_verbosity > 1) && (requestor == REQUESTOR_FLUSH_IFC))
	    $display ("%0d: %s.rl_reset: Flushed", cur_cycle, d_or_i);
      end
      rg_cset_in_cache <= rg_cset_in_cache + 1;
   endrule

   // ----------------------------------------------------------------
   // This rule probes the MMU and provides an immediate response for
   // memory (non-IO) requests, if possible, i.e., if
   //     VM off, LD or AMO_LR, cache hit
   //     VM on,  LD or AMO_LR, TLB hit and cache hit
   // Otherwise, moves to other states that handle TLB misses, cache
   // misses, 1-cycle delayed responses for ST and AMO, I/O requests, etc.

`ifdef ISA_PRIV_S
   (* descending_urgency = "rl_probe_and_immed_rsp, rl_writeback_updated_PTE" *)
`endif

   rule rl_probe_and_immed_rsp (rg_ddr4_ready && (rg_state == MODULE_RUNNING)
				&& (rg_age_seen == rg_req_seq)
				&& (! rg_kill_echo));    // RLD-4: see interface comment

      // Print some initial information for debugging
      if (cfg_verbosity > 1) begin
	 $display ("%0d: %s: rl_probe_and_immed_rsp; eaddr %0h", cur_cycle, d_or_i, rg_addr);

`ifdef ISA_PRIV_S
`ifdef RV32
	 if (vm_mode != satp_mode_RV32_bare)
	    $display ("        Priv:%0d  SATP:{mode %0d asid %0h pa %0h}  VA:%0h.%0h.%0h",
		      rg_priv, vm_mode, asid, satp_pa, vpn_1, vpn_0, offset);
`elsif SV39
	 if (vm_mode != satp_mode_RV64_bare)
	    $display ("        Priv:%0d  SATP:{mode %0d asid %0h pa %0h}  VA:%0h.%0h.%0h",
		      rg_priv, vm_mode, asid, satp_pa, vpn_1, vpn_0, offset);
`endif
`endif
	 $display ("        eaddr = {CTag 0x%0h  CSet 0x%0h  Word64 0x%0h  Byte 0x%0h}",
		   fn_PA_to_CTag (fn_WordXL_to_PA (rg_addr)),
		   cset_in_cache,
		   cword_in_cline,
		   byte_in_cword);
	 fa_display_state_and_ctag_cset (cset_in_cache, state_and_ctag_cset);
	 fa_display_cword_set (cset_in_cache, cword_in_cline, cword_set);
      end

      // ----------------
      // Virtual Memory translation

`ifdef ISA_PRIV_S
      VM_Xlate_Result vm_xlate_result <- fav_vm_xlate (rg_addr,
						       rg_satp,
						       tlb_result,
						       dmem_not_imem,
						       ((rg_op == CACHE_LD) || is_AMO_LR),
						       rg_priv,
						       rg_sstatus_SUM,
						       rg_mstatus_MXR);
`else
      // In non-VM, PA is always WordXL
      VM_Xlate_Result vm_xlate_result = VM_Xlate_Result {outcome:      VM_XLATE_OK,
							 pa:           rg_addr,
							 exc_code:     ?};
`endif

      if (cfg_verbosity > 1)
	 $display ("    TLB result: ", fshow (vm_xlate_result));

      // ---- TLB miss
      if (vm_xlate_result.outcome == VM_XLATE_TLB_MISS) begin
	 rg_state <= PTW_START;
      end

      // ---- TLB translation exception
      else if (vm_xlate_result.outcome == VM_XLATE_EXCEPTION) begin
	 rg_state <= MODULE_EXCEPTION_RSP;
	 rg_exc_code <= vm_xlate_result.exc_code;
      end

      // ---- vm_xlate_result.outcome == VM_XLATE_OK
      else begin
`ifdef ISA_PRIV_S
	 fa_record_pte_A_D_updates (tlb_result, vm_xlate_result);
`endif

	 rg_pa <= vm_xlate_result.pa;
	 let is_mem_addr = soc_map.m_is_mem_addr (fn_PA_to_Fabric_Addr (vm_xlate_result.pa));

	 // Access to non-memory
	 if (dmem_not_imem && (! is_mem_addr)) begin
	    // IO requests
	    rg_state <= IO_REQ;

	    if (cfg_verbosity > 1)
	       $display ("    => IO_REQ");
	 end

	 // Memory requests. Note: it's ok that this can go to non-memory space.
	 else begin
	    // Compute cache hit/miss. If hit, also compute Way_in_CSet and Word64
	    let pa_ctag = fn_PA_to_CTag (vm_xlate_result.pa);
	    match { .hit, .way_hit, .word64 } <- fn_test_cache_hit_or_miss (pa_ctag);

	    // ----------------
	    // Memory LD and AMO_LR
	    if ((rg_op == CACHE_LD) || is_AMO_LR) begin
	       if (hit) begin
		  // Cache hit; drive response
		  fa_drive_mem_rsp (rg_f3, rg_addr, word64, 0);

`ifdef ISA_A
		  if (is_AMO_LR) begin
		     rg_lrsc_valid <= True;
		     rg_lrsc_pa    <= vm_xlate_result.pa;
		     if (cfg_verbosity > 1)
			$display ("        AMO LR: reserving PA 0x%0h", vm_xlate_result.pa);
		  end
`endif
		  if (cfg_verbosity > 1) begin
		     $display ("        Read-hit: addr 0x%0h word64 0x%0h", rg_addr, word64);
		  end
	       end
	       else begin
		  // Cache miss; start cache-line refill
		  rg_state <= CACHE_START_REFILL;
		  if (cfg_verbosity > 1)
		     $display ("        Read Miss: -> CACHE_START_REFILL.");
`ifdef ISA_A
		  // TODO: this is pessimistic; unnecessary in a single-hart system?
		  if (is_AMO_LR && (vm_xlate_result.pa == rg_lrsc_pa)) begin
		     rg_lrsc_valid <= False;
		     if (cfg_verbosity > 1)
			$display ("        AMO LR: cache refill: cancelling LR/SC reservation for PA 0x%0h", rg_lrsc_pa);
		  end
`endif
	       end
	    end

	    // ----------------
	    // Memory ST and AMO SC
	    else if ((rg_op == CACHE_ST) || is_AMO_SC) begin
	       Bool do_write = True;    // Always True for ST; success/fail for AMO_SC
`ifdef ISA_A
	       // ST: if to an LR/SC reserved address, invalidate the reservation
	       if ((rg_op == CACHE_ST) && (vm_xlate_result.pa == rg_lrsc_pa)) begin
		  rg_lrsc_valid <= False;
		  if (cfg_verbosity > 1)
		     $display ("        ST: cancelling LR/SC reservation for PA", vm_xlate_result.pa);
	       end

	       // AMO_SC
	       else if (is_AMO_SC) begin
		  // Fail if reservation is not valid, or if not to the reserved addr
		  if (! rg_lrsc_valid) begin
		     do_write = False;
		     if (cfg_verbosity > 1)
			$display ("        AMO SC: fail due to invalid LR/SC reservation");
		  end
		  else if (rg_lrsc_pa != vm_xlate_result.pa) begin
		     do_write = False;
		     if (cfg_verbosity > 1)
			$display ("        AMO SC: fail: reserved addr 0x%0h, this address 0x%0h",
				  rg_lrsc_pa, vm_xlate_result.pa);
		  end

		  // SC result=0 on success, =1 on failure
		  Bit #(1) lrsc_result = (do_write ? 1'b0 : 1'b1);

		  rg_ld_val     <= zeroExtend (lrsc_result);
		  rg_lrsc_valid <= False;
		  if (cfg_verbosity > 1)
		     $display ("        AMO SC result = %0d", lrsc_result);
	       end
`endif
	       if (do_write) begin
		  // ST, or successful SC
		  if (hit) begin
		     // Update cache line in cache
		     let new_cword_set = fn_update_cword_set (cword_set, way_hit, vm_xlate_result.pa, rg_f3, rg_st_amo_val);
		     ram_cword_set.a.put (bram_cmd_write, cset_cword_in_cache, new_cword_set);
`ifdef BRAM_HAZARD_ASSERT
		     fa_haz_A_write (False, bram_cmd_write, extend (cset_cword_in_cache));
`endif

		     if (cfg_verbosity > 1) begin
			$display ("        Write-Cache-Hit: pa 0x%0h word64 0x%0h", vm_xlate_result.pa, rg_st_amo_val);
			$write   ("        New Word64_Set:");
			fa_display_cword_set (cset_in_cache, cword_in_cline, new_cword_set);
		     end
		  end
		  else begin
		     if (cfg_verbosity > 1)
			$display ("        Write-Cache-Miss: pa 0x%0h word64 0x%0h", vm_xlate_result.pa, rg_st_amo_val);
		  end

		  if (cfg_verbosity > 1)
		     $display ("        Write-Cache-Hit/Miss: eaddr 0x%0h word64 0x%0h", rg_addr, rg_st_amo_val);

		  // For write-hits and write-misses, writeback data to memory (so cache remains clean)
		  fa_fabric_send_write_req (rg_f3, vm_xlate_result.pa, rg_st_amo_val);

		  // Provide write-response after 1-cycle delay (thus locking the cset for 1 cycle),
		  // in case the next incoming request tries to read from the same SRAM address.
		  rg_state <= CACHE_ST_AMO_RSP;

		  if (cfg_verbosity > 1)
		     $display ("        => rl_write_response");
	       end
	       else begin // do_write == False
		  // SC fail
		  fa_drive_mem_rsp (rg_f3, rg_addr, 1, 0);
		  if (cfg_verbosity > 1)
		     $display ("        AMO SC: Fail response for addr 0x%0h", rg_addr);
	       end
	    end

`ifdef ISA_A
	    // ----------------
	    // Remaining AMOs
	    else begin
	       if (! hit) begin
		  // Cache miss; AMOs are only done in the cache, so first refill the cache-line
		  rg_state <= CACHE_START_REFILL;
		  if (cfg_verbosity > 1)
		     $display ("        AMO Miss: -> CACHE_START_REFILL.");
	       end
	       else begin
		  if (cfg_verbosity > 1) begin
		     $display ("        AMO: addr 0x%0h amo_f7 0x%0h f3 %0d rs2_val 0x%0h",
			       rg_addr, rg_amo_funct7, rg_f3, rg_st_amo_val);
		     $display ("          PA 0x%0h ", vm_xlate_result.pa);
		     $display ("          Cache word64 0x%0h, load-result 0x%0h", word64, word64);
		  end

		  // E12: capture the hit; the AMO op, the cache write and the
		  // writeback run in rl_amo_exec next cycle.  Stage2 never drops
		  // an outstanding dmem op (every trap/interrupt/halt path needs
		  // Stage2 EMPTY), so no req can pre-empt CACHE_AMO_EXEC.
		  rg_amo_cword_set <= cword_set;
		  rg_amo_way       <= way_hit;
		  rg_amo_w1        <= fn_extract_and_extend_bytes (rg_f3, rg_addr, word64);

		  // If this is to the LR/SC reserved address, invalidate the reservation
		  // TODO: should we invalidate even if to a different
		  // addr, since LR/SC pairs are not supposed to have
		  // other mem ops between them?
		  if (vm_xlate_result.pa == rg_lrsc_pa) begin
		     rg_lrsc_valid <= False;
		     if (cfg_verbosity > 1)
			$display ("        AMO_op: cancelling LR/SC reservation for PA", vm_xlate_result.pa);
		  end

		  rg_state <= CACHE_AMO_EXEC;
	       end
	    end
`endif
	 end
      end
   endrule: rl_probe_and_immed_rsp

`ifdef ISA_PRIV_S
   // ****************************************************************
   // TLB REFILLS (Page Table Walks)
   // ****************************************************************

   // TODO: should this rule be merged into rl_probe_and_immed_rsp, to avoid losing a cycle?
   //       or does that worsen critical path?

   rule rl_start_tlb_refill ((rg_state == PTW_START) && (ctr_wr_rsps_pending.value == 0));

`ifdef RV32

      // RV32.Sv32: Page Table top is at Level 1

      if (cfg_verbosity > 1)
	 $display ("%0d: %s.rl_start_tlb_refill for eaddr 0x%0h; req for level 1 PTE",
		   cur_cycle, d_or_i, rg_addr);

      PA           vpn_1_pa            = (zeroExtend (vpn_1) << bits_per_byte_in_wordxl);
      PA           lev_1_pte_pa        = satp_pa + vpn_1_pa;
      PA           lev_1_pte_pa_w64    = { lev_1_pte_pa [pa_sz - 1 : 3], 3'b0 };    // 64b-aligned addr
      Fabric_Addr  lev_1_pte_pa_w64_fa = fn_PA_to_Fabric_Addr (lev_1_pte_pa_w64);
      fa_fabric_send_read_req (lev_1_pte_pa_w64_fa, axsize_4);

      rg_pte_pa <= lev_1_pte_pa;
      rg_state  <= PTW_LEVEL_1;
`elsif SV39    // ifdef RV32

      // RV64.Sv39: Page Table top is at Level 2

      if (cfg_verbosity > 1)
	 $display ("%0d: %s.rl_start_tlb_refill for eaddr 0x%0h; req for level 2 PTE",
		   cur_cycle, d_or_i, rg_addr);

      PA           vpn_2_pa            = (zeroExtend (vpn_2) << bits_per_byte_in_wordxl);
      PA           lev_2_pte_pa        = satp_pa + vpn_2_pa;
      PA           lev_2_pte_pa_w64    = { lev_2_pte_pa [pa_sz - 1 : 3], 3'b0 };    // 64b-aligned addr
      Fabric_Addr  lev_2_pte_pa_w64_fa = fn_PA_to_Fabric_Addr (lev_2_pte_pa_w64);
      fa_fabric_send_read_req (lev_2_pte_pa_w64_fa, axsize_8);

      rg_pte_pa <= lev_2_pte_pa;
      rg_state  <= PTW_LEVEL_2;
`endif         // elsif SV39

   endrule

   // ----------------
   // Receive Level 2 PTE and process it (Sv39 or Sv48 only)

`ifdef SV39
   rule rl_ptw_level_2 (rg_state == PTW_LEVEL_2);
      // Memory read-response is a level 1 PTE
      let  mem_rsp <- pop_o (master_xactor.o_rd_data);

      Bit #(64) x64 = zeroExtend (mem_rsp.rdata);
      WordXL pte;

      // PTE is 64b response (RV32 does not have Level 2 PTEs)
      // TODO: this is ok only when Wd_Data == 64
      // When Wd_Data == 32, have to do two transactions to get a PTE
      pte = mem_rsp.rdata;

      // Bus error
      if (mem_rsp.rresp != axi4_resp_okay) begin
	 rg_exc_code <= access_exc_code;
	 rg_state    <= MODULE_EXCEPTION_RSP;
	 if (cfg_verbosity > 1)
	    $display ("%0d: %s.rl_ptw_level_2: for eaddr 0x%0h: pte_pa 0x%0h: FABRIC_RSP_ERR: access exception %0d",
		      cur_cycle, d_or_i, rg_addr, rg_pte_pa, access_exc_code);
      end

      // Invalid PTE
      else if (is_invalid_pte (pte)) begin
	 rg_exc_code <= page_fault_exc_code;
	 rg_state    <= MODULE_EXCEPTION_RSP;

	 if (cfg_verbosity > 1)
	    $display ("%0d: %s.rl_ptw_level_2: for eaddr 0x%0h: pte 0x%0h @ 0x%0h: Invalid PTE; page fault %0d",
		      cur_cycle, d_or_i, rg_addr, pte, rg_pte_pa, page_fault_exc_code);
      end

      // Pointer to next-level PTE
      else if ((fn_PTE_to_X (pte) == 0) && (fn_PTE_to_R (pte) == 0)) begin
	 if (cfg_verbosity > 1) begin
	    $display ("%0d: %s.rl_rl_ptw_level_2: for eaddr 0x%0h: pte 0x%0h @ 0x%0h: continue to level 1",
		      cur_cycle, d_or_i, rg_addr, pte, rg_pte_pa);
	    $display ("    Req for level 1 PTE");
	 end

	 PPN          ppn                 = fn_PTE_to_PPN (pte);
	 PA           lev_1_PTN_pa        = fn_PPN_and_Offset_to_PA (ppn, 12'b0);
	 PA           vpn_1_pa            = (zeroExtend (vpn_1) << bits_per_byte_in_wordxl);
	 PA           lev_1_pte_pa        = lev_1_PTN_pa + vpn_1_pa;
	 PA           lev_1_pte_pa_w64    = { lev_1_pte_pa [pa_sz - 1 : 3], 3'b0 };    // 64b-aligned addr
	 Fabric_Addr  lev_1_pte_pa_w64_fa = fn_PA_to_Fabric_Addr (lev_1_pte_pa_w64);
	 fa_fabric_send_read_req (lev_1_pte_pa_w64_fa, axsize_8);

	 rg_pte_pa <= lev_1_pte_pa;
	 rg_state  <= PTW_LEVEL_1;
      end

      // Leaf PTE pointing at address-space gigapage
      else begin
	 // Fault if PPN [1] or PPN [0] are not 0
	 PPN_1 ppn_1 = fn_PTE_to_PPN_1 (pte);
	 PPN_0 ppn_0 = fn_PTE_to_PPN_0 (pte);
	 if ((ppn_1 != 0) || (ppn_0 != 0)) begin
	    rg_exc_code <= page_fault_exc_code;
	    rg_state    <= MODULE_EXCEPTION_RSP;

	    if (cfg_verbosity > 1)
	       $display ("%0d: %s.rl_ptw_level_2: for eaddr 0x%0h: gigapage pte 0x%0h @ 0x%0h",
			 cur_cycle, d_or_i, rg_addr, pte, rg_pte_pa);
	       $display ("    Invalid PTE: PPN[1] or PPN[0] is not zero; page fault %0d",
			 page_fault_exc_code);
	 end

	 // Insert gigapage PTE in TLB (permissions will be checked on subsequent TLB hit)
	 else begin
	    tlb.insert (asid, vpn, pte, /* level */ 2, rg_pte_pa);
	    rg_state <= CACHE_REREQ;

	    if (cfg_verbosity > 1) begin
	       PPN  ppn                = fn_PTE_to_PPN (pte);
	       PA   addr_space_page_pa = fn_PPN_and_Offset_to_PA (ppn, 12'b0);
	       $display ("%0d: %s.rl_ptw_level_2: for eaddr 0x%0h: pte 0x%0h @ 0x%0h: leaf PTE for gigapage",
			 cur_cycle, d_or_i, rg_addr, pte, rg_pte_pa);
	       $display ("    Addr Space megapage pa: 0x%0h", addr_space_page_pa);
	    end
	 end
      end
   endrule: rl_ptw_level_2
`endif      // ifdef SV39

   // ----------------
   // Receive Level 1 PTE and process it (Sv32, Sv39 or Sv48)

   rule rl_ptw_level_1 (rg_state == PTW_LEVEL_1);
      // Memory read-response is a level 1 PTE
      let  mem_rsp <- pop_o (master_xactor.o_rd_data);

      Bit #(64) x64 = zeroExtend (mem_rsp.rdata);
      WordXL pte;
`ifdef RV32
      // PTE is lower or upper 32b word of 64b mem response
      pte = x64 [31:0];
      if ((valueOf (Wd_Data) == 64) && (rg_pte_pa [2] == 1'b1))
	 pte = x64 [63:32];
`else       // ifdef RV32
      // PTE is 64b response
      // TODO: this is ok only when Wd_Data == 64
      // When Wd_Data == 32, have to do two transactions to get a PTE
      pte = mem_rsp.rdata;
`endif      // ifndef RV32

      // Bus error
      if (mem_rsp.rresp != axi4_resp_okay) begin
	 rg_exc_code <= access_exc_code;
	 rg_state    <= MODULE_EXCEPTION_RSP;
	 if (cfg_verbosity > 1)
	    $display ("%0d: %s.rl_ptw_level_1: for eaddr 0x%0h: pte_pa 0x%0h: FABRIC_RSP_ERR: access exception %0d",
		      cur_cycle, d_or_i, rg_addr, rg_pte_pa, access_exc_code);
      end

      // Invalid PTE
      else if (is_invalid_pte (pte)) begin
	 rg_exc_code <= page_fault_exc_code;
	 rg_state    <= MODULE_EXCEPTION_RSP;

	 if (cfg_verbosity > 1)
	    $display ("%0d: %s.rl_ptw_level_1: for eaddr 0x%0h: pte 0x%0h @ 0x%0h: Invalid PTE; page fault %0d",
		      cur_cycle, d_or_i, rg_addr, pte, rg_pte_pa, page_fault_exc_code);
      end

      // Pointer to next-level PTE
      else if ((fn_PTE_to_X (pte) == 0) && (fn_PTE_to_R (pte) == 0)) begin
	 if (cfg_verbosity > 1) begin
	    $display ("%0d: %s.rl_rl_ptw_level_1: for eaddr 0x%0h: pte 0x%0h @ 0x%0h: continue to level 0",
		      cur_cycle, d_or_i, rg_addr, pte, rg_pte_pa);
	    $display ("    Req for level 0 PTE");
	 end

	 PPN          ppn                 = fn_PTE_to_PPN (pte);
	 PA           lev_0_PTN_pa        = fn_PPN_and_Offset_to_PA (ppn, 12'b0);
	 PA           vpn_0_pa            = (zeroExtend (vpn_0) << bits_per_byte_in_wordxl);
	 PA           lev_0_pte_pa        = lev_0_PTN_pa + vpn_0_pa;
	 PA           lev_0_pte_pa_w64    = { lev_0_pte_pa [pa_sz - 1 : 3], 3'b0 };    // 64b-aligned addr
	 Fabric_Addr  lev_0_pte_pa_w64_fa = fn_PA_to_Fabric_Addr (lev_0_pte_pa_w64);
`ifdef SV32
	 AXI4_Size    axi4_size           = axsize_4;
`else
	 AXI4_Size    axi4_size           = axsize_8;
`endif
	 fa_fabric_send_read_req (lev_0_pte_pa_w64_fa, axi4_size);

	 rg_pte_pa <= lev_0_pte_pa;
	 rg_state  <= PTW_LEVEL_0;

      end

      // Leaf PTE pointing at address-space megapage
      // (permissions will be checked on subsequent TLB hit)
      else begin
	 // Fault if PPN [0] is not 0
	 PPN_0 ppn_0 = fn_PTE_to_PPN_0 (pte);
	 if (ppn_0 != 0) begin
	    rg_exc_code <= page_fault_exc_code;
	    rg_state    <= MODULE_EXCEPTION_RSP;

	    if (cfg_verbosity > 1)
	       $display ("%0d: %s.rl_ptw_level_1: for eaddr 0x%0h: megapage pte 0x%0h @ 0x%0h",
			 cur_cycle, d_or_i, rg_addr, pte, rg_pte_pa);
	       $display ("    Invalid PTE: PPN [0] is not zero; page fault %0d",
			 page_fault_exc_code);
	 end

	 // Insert gigapage PTE in TLB (permissions will be checked on subsequent TLB hit)
	 else begin
	    tlb.insert (asid, vpn, pte, /* level */ 1, rg_pte_pa);
	    rg_state <= CACHE_REREQ;

	    if (cfg_verbosity > 1) begin
	       PPN ppn                = fn_PTE_to_PPN (pte);
	       PA  addr_space_page_pa = fn_PPN_and_Offset_to_PA (ppn, 12'b0);
	       $display ("%0d: %s.rl_ptw_level_1: for eaddr 0x%0h: pte 0x%0h @ 0x%0h: leaf PTE for megapage",
			 cur_cycle, d_or_i, rg_addr, pte, rg_pte_pa);
	       $display ("    Addr Space megapage pa: 0x%0h", addr_space_page_pa);
	    end
	 end
      end
   endrule: rl_ptw_level_1

   // ----------------
   // Receive Level 0 PTE and process it

   rule rl_ptw_level_0 (rg_state == PTW_LEVEL_0);
      // Memory read-response is a level 0 PTE
      let mem_rsp <- pop_o (master_xactor.o_rd_data);

      Bit #(64) x64 = zeroExtend (mem_rsp.rdata);
      WordXL pte;
`ifdef RV32
      // PTE is lower or upper 32b word of 64b mem response
      pte = x64 [31:0];
      if ((valueOf (Wd_Data) == 64) && (rg_pte_pa [2] == 1'b1))
	 pte = x64 [63:32];
`else       // ifdef RV32
      // PTE is 64b response
      // TODO: this is ok only when Wd_Data == 64
      // When Wd_Data == 32, have to do two transactions to get a PTE
      pte = mem_rsp.rdata;
`endif      // ifndef RV32

      // Bus error
      if (mem_rsp.rresp != axi4_resp_okay) begin
	 rg_exc_code <= access_exc_code;
	 rg_state    <= MODULE_EXCEPTION_RSP;
	 if (cfg_verbosity > 1)
	    $display ("%0d: %s.rl_ptw_level_0: for eaddr 0x%0h: pte_pa 0x%0h: FABRIC_RSP_ERR: access exception %0d",
		      cur_cycle, d_or_i, rg_addr, rg_pte_pa, access_exc_code);
      end

      // Invalid PTE
      else if (is_invalid_pte (pte)) begin
	 rg_exc_code <= page_fault_exc_code;
	 rg_state    <= MODULE_EXCEPTION_RSP;

	 if (cfg_verbosity > 1)
	    $display ("%0d: %s.rl_ptw_level_0: for eaddr 0x%0h: pte 0x%0h @ 0x%0h: Invalid PTE; page fault %0d",
		      cur_cycle, d_or_i, rg_addr, pte, rg_pte_pa, page_fault_exc_code);
      end

      // Pointer to next-level PTE: invalid at level 0
      else if ((fn_PTE_to_X (pte) == 0) && (fn_PTE_to_R (pte) == 0)) begin
	 rg_exc_code <= page_fault_exc_code;
	 rg_state    <= MODULE_EXCEPTION_RSP;

	 if (cfg_verbosity > 1)
	    $display ("%0d: %s.rl_ptw_level_0: for eaddr 0x%0h: pte 0x%0h @ 0x50h: Not a leaf PTE; page fault %0d",
		      cur_cycle, d_or_i, rg_addr, pte, rg_pte_pa, page_fault_exc_code);
      end

      // Leaf PTE pointing at address-space page; insert in TLB
      // (permissions will be checked on next TLB hit)
      else begin
	 tlb.insert (asid, vpn, pte, /* level */ 0, rg_pte_pa);
	 rg_state <= CACHE_REREQ;

	 if (cfg_verbosity > 1) begin
	    PPN ppn                = fn_PTE_to_PPN (pte);
	    PA  addr_space_page_pa = fn_PPN_and_Offset_to_PA (ppn, 12'b0);
	    $display ("%0d: %s.rl_ptw_level_0: for eaddr 0x%0h: pte 0x%0h @ 0x%0h: leaf PTE",
		      cur_cycle, d_or_i, rg_addr, pte, rg_pte_pa);
	    $display ("    Addr Space page pa: 0x%0h", addr_space_page_pa);
	 end
      end
   endrule
`endif      // ifdef ISA_PRIV_S

   // ****************************************************************
   // CACHE REFILLS
   // ****************************************************************

   // Start cache-line refill loop when no more write-responses are outstanding
   // Send request into fabric for first fabric-word of cache line.
   // Pick victim way, update ctag.
   // Initiate read of cword_set in cache for read-modify-write of word64

   rule rl_start_cache_refill (   (rg_state == CACHE_START_REFILL)
			       && (ctr_wr_rsps_pending.value == 0));
      if (cfg_verbosity > 1)
	 $display ("%0d: %s.rl_start_cache_refill: ", cur_cycle, d_or_i);

      // Send burst request into fabric for full cache line
      PA             cline_addr        = fn_align_Addr_to_CLine (rg_pa);
      Fabric_Addr    cline_fabric_addr = fn_PA_to_Fabric_Addr (cline_addr);
      fa_fabric_send_read_burst_req (cline_fabric_addr);

      // Pick a victim 'way'
      // TODO: prioritize picking an EMPTY slot over a CLEAN slot
      // Currently just uses rg_victim_way and increments it
      // The following extend/truncate trickery is because
      // Bits_per_Way_in_CSet may be 0 (direct-mapped),
      // for which the '1' in '+1' is not a valid literal
      Bit #(TAdd #(1, Bits_per_Way_in_CSet)) tmp = extend (rg_victim_way);
      tmp = tmp + 1;
      Way_in_CSet new_victim_way = truncate (tmp);
      rg_victim_way <= new_victim_way;

      // State_and_CTag_CSet are updated in rl_cache_refill_rsps_loop
      // only after observing the first read-response, to assure that
      // the read is successfule and not an access error.

      // The refill loop preserves the victim line's OTHER-way word64s via a
      // pre-refill snapshot (see rl_cache_snapshot), so no data-RAM read is
      // needed here.  Do NOT serve any refill read from port A: port A is
      // write-only during refills, and a same-port read while a write is
      // pending loses its data on real DP16KD hardware (READ_FIRST shows
      // the write address's old data) even though Bluesim's BRAM model
      // returns the pending read data -- a sim/hardware divergence that
      // corrupts the other way's word64 on multi-way configs.
      rg_cset_cword_in_cache  <= { cset_in_cache, 0 };
      rg_snap_cnt             <= 0;

      // Enter snapshot phase (captures other-way words, then CACHE_REFILL)
      rg_lower_word32_full   <= False;
      rg_error_during_refill <= False;
      rg_state               <= CACHE_SNAPSHOT;
      pw_perf_miss.send;

      if (cfg_verbosity > 1)
	 $display ("    Victim way %0d; => CACHE_SNAPSHOT", new_victim_way);
   endrule: rl_start_cache_refill

   // ----------------------------------------------------------------
   // Snapshot the victim line's OTHER-way word64s before the refill loop
   // starts writing.  One data-RAM read per cycle, purely on port B, with
   // port A idle: zero read/write interleave, so the capture registers
   // always see clean read data (on both Bluesim's BRAM model and real
   // DP16KD).  Costs cwords_per_cline+2 cycles per refill -- negligible
   // against a ~30-cycle SDRAM miss.
   //
   // Note the tag RAM's B port is untouched here, so the tag capture
   // (rg_tag_ram_q) from the original request stays valid for the
   // State_and_CTag update in the refill loop's first response.

   rule rl_cache_snapshot (rg_state == CACHE_SNAPSHOT);
      // Issue this cycle's read (cnt = word index being read)
      Bit #(8) cnt = rg_snap_cnt;
      if (cnt <= fromInteger (cwords_per_cline - 1)) begin
	 CSet_CWord_in_Cache snap_addr = { cset_in_cache, truncate (cnt) };
	 ram_cword_set.b.put (bram_cmd_read, snap_addr, ?);
`ifdef BRAM_HAZARD_ASSERT
	 fa_haz_B_read (False, bram_cmd_read, extend (snap_addr));
`endif
      end

      // Capture the read issued two cycles ago (cnt-2); its data is in the
      // capture registers now.  Extract the non-victim way's word64.
      if (cnt >= 2) begin
	 let captured = cword_set;    // rg_data_ram_q
	 Way_in_CSet other_way = (ways_per_cset == 1) ? 0 : (rg_victim_way ^ 1);
	 CWord_in_CLine widx = truncate (cnt - 2);
	 let v = rg_other_way_line;
	 v [widx] = captured [other_way];
	 rg_other_way_line <= v;
      end

      if (cnt == fromInteger (cwords_per_cline + 1)) begin
	 rg_state <= CACHE_REFILL;
	 if (cfg_verbosity > 1)
	    $display ("%0d: %s.rl_cache_snapshot: snapshot done; => CACHE_REFILL",
		      cur_cycle, d_or_i);
      end
      rg_snap_cnt <= cnt + 1;
   endrule: rl_cache_snapshot

   // ----------------------------------------------------------------
   // TODO (possibly): we complete a cache refill (in rl_cache_refill_loop) and
   // then, in rl_rereq, redo the missing request, just in case the
   // last word64 of the refill is exactly the word64 we need in which
   // case we'd have a race on ram port A (refill write) and port B
   // (request).
   // An alternative would be to buffer the target word64 during the
   // refill and drive it as a result, but that would cost more state
   // and/or muxes.
   // An alternative would be to do a "wrapping refill", in which case
   // the last word64 of the refill will never conflict with the
   // requested word.

   // ----------------------------------------------------------------
   // Loop that receives responses from the fabric with fabric-words of the cline (from mem).
   // For 32b fabrics:
   //     If this is the lower Word32, just register it.
   //     else concat with lower Word32 and update word64 in cword_set
   // For 64b fabrics:
   //     update word64 in cword_set.
   // Update word64 in cword_set:
   //     write back to cword_set ram, and
   //     initiate read of next cword_set from ram
   //         (for set read-modify-write; not relevant for direct-mapped)

   rule rl_cache_refill_rsps_loop (rg_state == CACHE_REFILL);
      let mem_rsp <- pop_o (master_xactor.o_rd_data);
      if (cfg_verbosity > 2) begin
	 $display ("%0d: %s.rl_cache_refill_rsps_loop:", cur_cycle, d_or_i);
	 $display ("        ", fshow (mem_rsp));
      end

      // Bus errors; remember it, and raise exception after all the refill responses
      Bool err_rsp = (mem_rsp.rresp != axi4_resp_okay);
      if (err_rsp) begin
	 rg_error_during_refill <= True;
	 rg_exc_code            <= access_exc_code;
	 if (cfg_verbosity > 1)
	    $display ("%0d: %s.rl_cache_refill_rsps_loop: FABRIC_RSP_ERR: raising access exception %0d",
		      cur_cycle, d_or_i, access_exc_code);
      end

      // For 32b fabrics, if this is lower Word32, just register it to hold until upper Word32 arrives
      if ((valueOf (Wd_Data) == 32) && (! rg_lower_word32_full)) begin
	 rg_lower_word32      <= truncate (mem_rsp.rdata);
	 rg_lower_word32_full <= True;
	 if (cfg_verbosity > 2)
	    $display ("        Recording rdata in rg_lower_word32");
      end

      // Refill 64b of cache line
      else begin
	 Bit #(64) new_word64 = zeroExtend (mem_rsp.rdata);
	 if (valueOf (Wd_Data) == 32) begin
	    // Assert: rg_lower_32_full == True
	    new_word64 = { new_word64 [31:0], rg_lower_word32 };
	    rg_lower_word32_full <= False;
	    if (cfg_verbosity > 2)
	       $display ("        32b fabric: concat with rg_lower_word32: new_word64 0x%0x", new_word64);
	 end

	 CWord_in_CLine cword_in_cline = truncate (rg_cset_cword_in_cache);

	 // Update the State_and_CTag_CSet (BRAM port A) (if this is the first
	 // response and not an error)
	 if ((cword_in_cline == 0) && (! err_rsp)) begin
	    let new_state_and_ctag_cset = state_and_ctag_cset;
	    new_state_and_ctag_cset [rg_victim_way] = State_and_CTag {state: CTAG_CLEAN,
								      ctag : fn_PA_to_CTag (rg_pa)};
	    ram_state_and_ctag_cset.a.put (bram_cmd_write, cset_in_cache, new_state_and_ctag_cset);
`ifdef BRAM_HAZARD_ASSERT
	    fa_haz_A_write (True, bram_cmd_write, extend (cset_in_cache));
`endif
	 end

	 // Update the Word64_Set (BRAM port A) (if this response was not an
	 // error).  The new word64 goes into the victim way; the other way's
	 // word64 comes from the pre-refill snapshot (rl_cache_snapshot), never
	 // from a RAM read racing the refill writes.
	 CWord_in_CLine cword_idx = truncate (rg_cset_cword_in_cache);
	 let other_word64 = rg_other_way_line [cword_idx];
	 Vector #(Ways_per_CSet, CWord) new_cword_set = newVector;
	 for (Integer w = 0; w < ways_per_cset; w = w + 1)
	    new_cword_set [w] = (fromInteger (w) == rg_victim_way)
				? new_word64
				: other_word64;
	 if (! err_rsp) begin
	    ram_cword_set.a.put (bram_cmd_write, rg_cset_cword_in_cache, new_cword_set);
`ifdef BRAM_HAZARD_ASSERT
	    fa_haz_A_write (False, bram_cmd_write, extend (rg_cset_cword_in_cache));
`endif
	 end

	 // If more cword_sets in cacheline: advance the write index.  No RAM
	 // reads here -- the snapshot already holds the other way's old words.
	 if (cword_in_cline != fromInteger (cwords_per_cline - 1)) begin
	    rg_cset_cword_in_cache <= rg_cset_cword_in_cache + 1;
	 end

	 // else final Word64 of CLine; raise exception if pending,
	 // or redo original missing request on port B.
	 // The word64 we just wrote in port A may be the word64 we request on port B,
	 // so we do it a cycle later, in rl_rereq.
	 else if (err_rsp || rg_error_during_refill) begin
	    rg_state    <= MODULE_EXCEPTION_RSP;
	    if (cfg_verbosity > 1)
	       $display ("    => MODULE_EXCEPTION_RSP");
	 end

	 else begin
	    rg_state <= CACHE_REREQ;
	    if (cfg_verbosity > 1)
	       $display ("    => CACHE_REREQ");
	 end

	 if (cfg_verbosity > 2) begin
	    $display ("        Updating Cache cword_set 0x%0h, cword_in_cline %0d) old => new",
		      rg_cset_cword_in_cache, cword_in_cline);

	    fa_display_cword_set (cset_in_cache, cword_in_cline, cword_set);
	    fa_display_cword_set (cset_in_cache, cword_in_cline, new_cword_set);
	 end
      end
   endrule: rl_cache_refill_rsps_loop

   // ----------------------------------------------------------------
   // After tlb and cache refills, redo the missing request,
   // i.e., probe the TLB and cache (BRAM port B) again

   rule rl_rereq (rg_state == CACHE_REREQ);
      rg_state <= MODULE_RUNNING;
      rg_req_seq <= ! rg_req_seq;
      fa_req_ram_B (rg_addr);
   endrule

`ifdef ISA_A
   // ----------------------------------------------------------------
   // E12: AMO execute (hit captured by rl_probe_and_immed_rsp).
   // rg_addr/rg_pa/rg_f3/rg_amo_funct7/rg_st_amo_val are unchanged since the
   // probe (no req can arrive while the AMO is outstanding).

   rule rl_amo_exec (rg_state == CACHE_AMO_EXEC);
      // Do the AMO op on the loaded value and the store value
      match {.new_ld_val,
	     .new_st_val} = fv_amo_op_x (rg_f3, rg_amo_funct7, rg_amo_w1, rg_st_amo_val);

      // Update cache line in cache
      let new_cword_set = fn_update_cword_set (rg_amo_cword_set, rg_amo_way, rg_pa, rg_f3, new_st_val);
      ram_cword_set.a.put (bram_cmd_write, cset_cword_in_cache, new_cword_set);
`ifdef BRAM_HAZARD_ASSERT
      fa_haz_A_write (False, bram_cmd_write, extend (cset_cword_in_cache));
`endif

      if (cfg_verbosity > 1) begin
	 $display ("%0d: %s.rl_amo_exec: 0x%0h  op  0x%0h -> 0x%0h",
		   cur_cycle, d_or_i, rg_amo_w1, rg_st_amo_val, new_st_val);
	 $write   ("          New Word64_Set:");
	 fa_display_cword_set (cset_in_cache, cword_in_cline, new_cword_set);
      end

      // Writeback data to memory (so cache remains clean)
      fa_fabric_send_write_req (rg_f3, rg_pa, new_st_val);

      // Provide amo response after 1-cycle delay (thus locking the cset for 1 cycle),
      // in case the next incoming request tries to read from the same address.
      rg_ld_val     <= new_ld_val;
      rg_st_amo_val <= new_st_val;
      rg_state      <= CACHE_ST_AMO_RSP;
   endrule
`endif

   // ----------------------------------------------------------------
   // Provide write-response (ST op)
   // Stays in this state until CPU's next request puts it back into RUNNING state

   rule rl_ST_AMO_response (rg_state == CACHE_ST_AMO_RSP);
      if (rg_kill_echo) begin
	 // RLD-4: a new op just entered Stage2 (its deferred req launches
	 // this same cycle and supersedes this stale response); the req
	 // method's write of MODULE_RUNNING wins this cycle's write-mux if
	 // both fire.
	 rg_state <= MODULE_READY;
      end
      else begin
	 dw_valid             <= True;
	 dw_output_ld_val     <= zeroExtend (rg_ld_val);        // Irrelevant for ST; relevant for SC, AMO
	 dw_output_st_amo_val <= zeroExtend (rg_st_amo_val);
      end

`ifdef WATCH_TOHOST
      // ----------------
      // "tohost" addr on which to monitor writes, for standard ISA tests.
      // See NOTE: "tohost" above.
      if (rg_watch_tohost
	  && (zeroExtend (rg_pa) == rg_tohost_addr)
	  && (rg_st_amo_val != 0))
	 begin					      
	    rg_tohost_value <= rg_st_amo_val;

	    if (verbosity >= 1) begin
	       let test_num = (rg_st_amo_val >> 1);
	       if (test_num == 0) $display ("PASS:");
	       else               $display ("FAIL <test_%0d>:", test_num);
	       $display ("  (<tohost>  addr %0h  data %0h)", rg_tohost_addr, rg_st_amo_val);
	    end
	 end
`endif
   endrule

   // ----------------------------------------------------------------
   // Memory-mapped I/O read requests (LD and AMO_LR)
   // LRs are treated just like LDs, but we do not place any reservation on the address
   // (so a subsequent SC is guaranteed to fail).
   // TODO: Move this into rl_probe_and_immed_rsp, post MMU translation?
   // No caching, send request directly to fabric

   rule rl_io_read_req (   (rg_state == IO_REQ)
			&& ((rg_op == CACHE_LD) || is_AMO_LR)
			&& (ctr_wr_rsps_pending.value == 0));

      if (cfg_verbosity > 1)
	 $display ("%0d: %s.rl_io_read_req; f3 0x%0h vaddr %0h  paddr %0h",
		   cur_cycle, d_or_i, rg_f3, rg_addr, rg_pa);

      Fabric_Addr fabric_addr = fn_PA_to_Fabric_Addr (rg_pa);
      fa_fabric_send_read_req (fabric_addr, fn_funct3_to_AXI4_Size (rg_f3));

`ifdef ISA_A
      // Invalidate LR/SC reservation if AMO_LR
      if (is_AMO_LR) rg_lrsc_valid <= False;
`endif
      rg_state <= IO_AWAITING_READ_RSP;
   endrule

   // ----------------------------------------------------------------
   // Receive I/O read response from fabric

   rule rl_io_read_rsp ((rg_state == IO_AWAITING_READ_RSP));

      let rd_data <- pop_o (master_xactor.o_rd_data);
      if (cfg_verbosity > 1) begin
	 $display ("%0d: %s.rl_io_read_rsp: vaddr 0x%0h  paddr 0x%0h", cur_cycle, d_or_i, rg_addr, rg_pa);
	 $display ("    ", fshow (rd_data));
      end

      let ld_val = fn_extract_and_extend_bytes(rg_f3, rg_addr, zeroExtend (rd_data.rdata));
      rg_ld_val <= ld_val;

      // Successful read
      if (rd_data.rresp == axi4_resp_okay) begin
	 fa_drive_IO_read_rsp (rg_f3, rg_addr, ld_val);
	 rg_state <= IO_READ_RSP;
      end

      // Bus error
      else begin
	 rg_state    <= MODULE_EXCEPTION_RSP;
	 rg_exc_code <= exc_code_LOAD_ACCESS_FAULT;
	 if (cfg_verbosity > 1)
	    $display ("%0d: %s.rl_io_read_rsp: FABRIC_RSP_ERR: raising trap LOAD_ACCESS_FAULT",
		      cur_cycle, d_or_i);
      end
   endrule

   // ----------------
   // Maintain I/O-read response
   // Stays in this state until CPU's next request puts it back into RUNNING state

   rule rl_maintain_io_read_rsp (rg_state == IO_READ_RSP);
      if (rg_kill_echo) begin
	 // RLD-4: stale response; exit (see rl_ST_AMO_response).
	 rg_state <= MODULE_READY;
      end
      else
	 fa_drive_IO_read_rsp (rg_f3, rg_addr, rg_ld_val);
   endrule

   // ----------------------------------------------------------------
   // Memory-mapped I/O write requests (ST)
   // No caching, send request directly to fabric.
   // TODO: Move this into rl_probe_and_immed_rsp, post MMU translation

`ifdef ISA_PRIV_S
   (* descending_urgency = "rl_io_write_req, rl_writeback_updated_PTE" *)
`endif

   rule rl_io_write_req ((rg_state == IO_REQ) && (rg_op == CACHE_ST));
      if (cfg_verbosity > 1)
	 $display ("%0d: %s: rl_io_write_req; f3 0x%0h  vaddr %0h  paddr %0h  word64 0x%0h",
		   cur_cycle, d_or_i, rg_f3, rg_addr, rg_pa, rg_st_amo_val);

      fa_fabric_send_write_req (rg_f3, rg_pa, rg_st_amo_val);

      rg_state <= CACHE_ST_AMO_RSP;

      if (cfg_verbosity > 1)
	 $display ("    => rl_ST_AMO_response");
   endrule

   // ----------------------------------------------------------------
   // Memory-mapped I/O AMO_SC requests. Always fail.

`ifdef ISA_A
   rule rl_io_AMO_SC_req ((rg_state == IO_REQ) && is_AMO_SC);

      rg_ld_val <= 1;    // 1 is LR/SC failure value
      rg_state  <= CACHE_ST_AMO_RSP;

      if (cfg_verbosity > 1) begin
	 $display ("%0d: %s: rl_io_AMO_SC_req; f3 0x%0h  vaddr %0h  paddr %0h  word64 0x%0h",
		   cur_cycle, d_or_i, rg_f3, rg_addr, rg_pa, rg_st_amo_val);
	 $display ("    FAIL due to I/O address.");
	 $display ("    => rl_ST_AMO_response");
      end
   endrule
`endif

   // ----------------------------------------------------------------
   // Memory-mapped I/O AMO requests other than LR/SC
   // Fail with STORE/AMO Access fault exception
   // TODO: Extend fabric to do these ops at the I/O device?

`ifdef ISA_A
   rule rl_io_AMO_op_req ((rg_state == IO_REQ) && is_AMO && (! is_AMO_LR) && (! is_AMO_SC));
      if (cfg_verbosity > 1)
	 $display ("%0d: %s.rl_io_AMO_op_req; f3 0x%0h vaddr %0h  paddr %0h",
		   cur_cycle, d_or_i, rg_f3, rg_addr, rg_pa);

      Fabric_Addr fabric_addr = fn_PA_to_Fabric_Addr (rg_pa);
      fa_fabric_send_read_req (fabric_addr, fn_funct3_to_AXI4_Size (rg_f3));

      rg_state <= IO_AWAITING_AMO_READ_RSP;

   endrule
`endif

   // ----------------
   // Receive I/O AMO read response from fabric,
   // Do the AMO op, and send store to fabric

`ifdef ISA_A
`ifdef ISA_PRIV_S
   (* descending_urgency = "rl_io_AMO_read_rsp, rl_writeback_updated_PTE" *)
`endif

   rule rl_io_AMO_read_rsp (rg_state == IO_AWAITING_AMO_READ_RSP);
      let rd_data <- pop_o (master_xactor.o_rd_data);
      if (cfg_verbosity > 1) begin
	 $display ("%0d: %s.rl_io_AMO_read_rsp: vaddr 0x%0h  paddr 0x%0h", cur_cycle, d_or_i, rg_addr, rg_pa);
	 $display ("    ", fshow (rd_data));
      end

      let ld_val = fn_extract_and_extend_bytes(rg_f3, rg_addr, zeroExtend (rd_data.rdata));

      // Bus error for AMO read
      if (rd_data.rresp != axi4_resp_okay) begin
	 rg_state    <= MODULE_EXCEPTION_RSP;
	 rg_exc_code <= exc_code_STORE_AMO_ACCESS_FAULT;
	 if (cfg_verbosity > 1)
	    $display ("%0d: %s.rl_io_AMO_read_rsp: FABRIC_RSP_ERR: raising trap STORE_AMO_ACCESS_FAULT",
		      cur_cycle, d_or_i);
      end
      // Successful AMO read
      else begin
	 if (cfg_verbosity > 1)
	    $display ("%0d: %s: rl_io_AMO_read_rsp; f3 0x%0h  vaddr %0h  paddr %0h  word64 0x%0h",
		      cur_cycle, d_or_i, rg_f3, rg_addr, rg_pa, rg_st_amo_val);

	 // Do the AMO op on the loaded value and the store value
	 match {.new_ld_val,
		.new_st_val} = fv_amo_op (rg_f3, rg_amo_funct7, rg_addr, ld_val, rg_st_amo_val);

	 // Write back new st_val to fabric
	 fa_fabric_send_write_req (rg_f3, rg_pa, new_st_val);

	 fa_drive_IO_read_rsp (rg_f3, rg_addr, new_ld_val);
	 rg_ld_val <= new_ld_val;
	 rg_state  <= IO_READ_RSP;

	 if (cfg_verbosity > 1)
	    $display ("    => rl_ST_AMO_response");
      end
   endrule
`endif

   // ----------------------------------------------------------------
   // Discard write-responses from the fabric
   // NOTE: assuming in-order responses from fabric

   rule rl_discard_write_rsp;
      let wr_resp <- pop_o (master_xactor.o_wr_resp);

      if (ctr_wr_rsps_pending.value == 0) begin
	 $display ("%0d: ERROR: %s.rl_discard_write_rsp: unexpected W response (ctr_wr_rsps_pending.value == 0)",
		   cur_cycle, d_or_i);
	 $display ("    ", fshow (wr_resp));
	 $finish (1);    // Assertion failure
      end

      ctr_wr_rsps_pending.decr;

      if (wr_resp.bresp != axi4_resp_okay) begin
	 rg_wr_rsp_err <= True;
	 $display ("%0d: %s.rl_discard_write_rsp: fabric response error: exit", cur_cycle, d_or_i);
	 $display ("    ", fshow (wr_resp));
	 // TODO: need to raise a non-maskable interrupt (NMI) here?
      end
      else if (cfg_verbosity > 1) begin
	 $display ("%0d: %s.rl_discard_write_rsp: pending %0d ",
		   cur_cycle, d_or_i, ctr_wr_rsps_pending.value, fshow (wr_resp));
      end
   endrule

   // ----------------------------------------------------------------
   // This rule drives an exception response until the cache is put
   // into MODULE_RUNNING state by the next request.

   rule rl_drive_exception_rsp (rg_state == MODULE_EXCEPTION_RSP);
      if (rg_kill_echo) begin
	 // RLD-4: stale response; exit (see rl_ST_AMO_response).
	 rg_state <= MODULE_READY;
      end
      else begin
	 dw_valid    <= True;
	 dw_exc      <= True;
	 dw_exc_code <= rg_exc_code;
      end
   endrule

   // ================================================================
   // INTERFACE

   method Action set_verbosity (Bit #(4) v);
      cfg_verbosity <= v;
   endmethod

   interface Server server_reset;
      interface Put request;
	 method Action put (Token t);
	    f_reset_reqs.enq (REQUESTOR_RESET_IFC);
	 endmethod
      endinterface
      interface Get response;
	 method ActionValue #(Token) get () if (f_reset_rsps.first == REQUESTOR_RESET_IFC);
	    f_reset_rsps.deq;
	    return ?;
	 endmethod
      endinterface
   endinterface

   // CPU interface: request
   // NOTE: this has no flow control: CPU should only invoke it when consuming prev output.
   // As soon as this method is called, the module starts working on this new request.
   method Action  req (CacheOp op,
		       Bit #(3) f3,
`ifdef ISA_A
		       Bit #(7) amo_funct7,
`endif
		       Addr addr,
		       Bit #(64) st_value,
		       // The following  args for VM
		       Priv_Mode  priv,
		       Bit #(1)   sstatus_SUM,
		       Bit #(1)   mstatus_MXR,
		       WordXL     satp);    // { VM_Mode, ASID, PPN_for_page_table }

      if (cfg_verbosity > 1) begin
	 $display ("%0d: %m.req: op:", cur_cycle, fshow (op),
		   " f3:%0d addr:0x%0h st_value:0x%0h", f3, addr, st_value);
	 $display ("    priv:", fshow_Priv_Mode (priv),
		   " sstatus_SUM:%0d mstatus_MXR:%0d satp:0x%0h",
		   sstatus_SUM,    mstatus_MXR,    satp);
`ifdef ISA_A
	 $display ("    amo_funct7 = 0x%0h", amo_funct7);
`endif
      end

      rg_op          <= op;
      rg_f3          <= f3;
`ifdef ISA_A
      rg_amo_funct7  <= amo_funct7;
`endif
      rg_addr        <= addr;
      rg_st_amo_val  <= st_value;
      rg_req_seq     <= ! rg_req_seq;
      rg_kill_echo   <= False;    // RLD-4: this req supersedes any stale echo

      rg_priv        <= priv;
      rg_sstatus_SUM <= sstatus_SUM;
      rg_mstatus_MXR <= mstatus_MXR;
      rg_satp        <= satp;

      // Initial default PA assumes no VM translation
      rg_pa <= fn_WordXL_to_PA (addr);

      if (! fn_is_aligned (f3, addr)) begin
	 // We detect misaligned accesses and trap on them
	 rg_state    <= MODULE_EXCEPTION_RSP;
	 // LR is a load: misaligned LR raises LOAD_ADDR_MISALIGNED (Sail, Spike), the same
	 // load/store split the access- and page-fault codes already use (is_AMO_LR).
	 Bool is_lr = ((op == CACHE_AMO) && (amo_funct7 [6:2] == f5_AMO_LR));
	 rg_exc_code <= (((op == CACHE_LD) || is_lr) ? exc_code_LOAD_ADDR_MISALIGNED : exc_code_STORE_AMO_ADDR_MISALIGNED);
      end
      else begin
	 rg_state <= MODULE_RUNNING;
	 fa_req_ram_B (addr);
      end
   endmethod

   method Action  ma_kill_echo;
      rg_kill_echo <= True;
   endmethod

   method Bool  valid;
      return dw_valid;
   endmethod

   method WordXL  addr;    // req addr for which this is a response
      return rg_addr;
   endmethod

   method Bit #(64)  cword;
      return dw_output_ld_val;
   endmethod

   method Bit #(64)  st_amo_val;
      return dw_output_st_amo_val;
   endmethod

   method Bool  exc;
      return dw_exc;
   endmethod

   method Exc_Code  exc_code;
      return dw_exc_code;
   endmethod

   // Flush request/response
   interface Server  server_flush;
      interface Put  request;
	 method Action  put (Token t);
	    f_reset_reqs.enq (REQUESTOR_FLUSH_IFC);
	 endmethod
      endinterface
      interface Get  response;
	 method ActionValue #(Token)  get () if (f_reset_rsps.first == REQUESTOR_FLUSH_IFC);
	    f_reset_rsps.deq;
	    return ?;
	 endmethod
      endinterface
   endinterface

   // TLB flush
   method Action tlb_flush;
`ifdef ISA_PRIV_S
      tlb.flush;
      rg_state <= MODULE_READY;
      if (cfg_verbosity > 1)
	 $display ("%0d: %s.tlb_flush", cur_cycle, d_or_i);
`else
      noAction;
`endif
   endmethod

   // Fabric master interface: master_xactor's, with the reset epoch in the IDs (see rg_epoch)
   interface AXI4_Master_IFC mem_master;
      let m = master_xactor.axi_side;
      method Bool           m_awvalid  = m.m_awvalid;
      method Bit #(Wd_Id)   m_awid     = zeroExtend (rg_epoch);
      method Bit #(Wd_Addr) m_awaddr   = m.m_awaddr;
      method Bit #(8)       m_awlen    = m.m_awlen;
      method AXI4_Size      m_awsize   = m.m_awsize;
      method Bit #(2)       m_awburst  = m.m_awburst;
      method Bit #(1)       m_awlock   = m.m_awlock;
      method Bit #(4)       m_awcache  = m.m_awcache;
      method Bit #(3)       m_awprot   = m.m_awprot;
      method Bit #(4)       m_awqos    = m.m_awqos;
      method Bit #(4)       m_awregion = m.m_awregion;
      method Bit #(Wd_User) m_awuser   = m.m_awuser;
      method Action m_awready (Bool awready) = m.m_awready (awready);

      method Bool                      m_wvalid = m.m_wvalid;
      method Bit #(Wd_Data)            m_wdata  = m.m_wdata;
      method Bit #(TDiv #(Wd_Data, 8)) m_wstrb  = m.m_wstrb;
      method Bool                      m_wlast  = m.m_wlast;
      method Bit #(Wd_User)            m_wuser  = m.m_wuser;
      method Action m_wready (Bool wready) = m.m_wready (wready);

      method Action m_bvalid (Bool bvalid, Bit #(Wd_Id) bid, Bit #(2) bresp, Bit #(Wd_User) buser);
         m.m_bvalid (bvalid && (bid [1:0] == rg_epoch), bid, bresp, buser);    // stale B: dropped
      endmethod
      method Bool m_bready = m.m_bready;

      method Bool           m_arvalid  = m.m_arvalid;
      method Bit #(Wd_Id)   m_arid     = zeroExtend (rg_epoch);
      method Bit #(Wd_Addr) m_araddr   = m.m_araddr;
      method Bit #(8)       m_arlen    = m.m_arlen;
      method AXI4_Size      m_arsize   = m.m_arsize;
      method Bit #(2)       m_arburst  = m.m_arburst;
      method Bit #(1)       m_arlock   = m.m_arlock;
      method Bit #(4)       m_arcache  = m.m_arcache;
      method Bit #(3)       m_arprot   = m.m_arprot;
      method Bit #(4)       m_arqos    = m.m_arqos;
      method Bit #(4)       m_arregion = m.m_arregion;
      method Bit #(Wd_User) m_aruser   = m.m_aruser;
      method Action m_arready (Bool arready) = m.m_arready (arready);

      method Action m_rvalid (Bool rvalid, Bit #(Wd_Id) rid, Bit #(Wd_Data) rdata, Bit #(2) rresp, Bool rlast, Bit #(Wd_User) ruser);
         m.m_rvalid (rvalid && (rid [1:0] == rg_epoch), rid, rdata, rresp, rlast, ruser);    // stale R: dropped
      endmethod
      method Bool m_rready = m.m_rready;
   endinterface

   // ----------------------------------------------------------------
   // Misc. control and status

   // ----------------
   // For ISA tests: watch memory writes to <tohost> addr

`ifdef WATCH_TOHOST
   method Action set_watch_tohost (Bool watch_tohost, Bit #(64) tohost_addr);
      rg_watch_tohost <= watch_tohost;
      rg_tohost_addr  <= tohost_addr;
      $display ("%0d: %m.set_watch_tohost: watch %0d, addr %0h",
		cur_cycle, watch_tohost, tohost_addr);   endmethod

   method Bit #(64) mv_tohost_value;
      return rg_tohost_value;
   endmethod
`endif

   // Inform core that DDR4 has been initialized and is ready to accept requests
   method Action ma_ddr4_ready;
      rg_ddr4_ready <= True;
`ifdef TB_PROBES
      $display ("%0d: %m.ma_ddr4_ready: Enabling MMU_Cache", cur_cycle);
`endif
   endmethod

   // Misc. status; 0 = running, no error
   method Bit #(8) mv_status;
      return (rg_wr_rsp_err ? 1 : 0);
   endmethod

   // Performance-monitoring tap (see interface comment)
   method Bool mv_busy = rg_busy_mirror;

   method Bool mv_perf_miss;
      return pw_perf_miss;
   endmethod

endmodule: mkMMU_Cache

// ================================================================

endpackage: MMU_Cache
