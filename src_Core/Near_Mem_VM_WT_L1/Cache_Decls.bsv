// Copyright (c) 2016-2020 Bluespec, Inc. All Rights Reserved.

package Cache_Decls;

// ================================================================
// This is just a convenience package, encapsulating the selection of
// a specific set of cache declarations corresponding to a particular
// cache config.

// ================================================================

// RVFI_DII (TestRIG) builds also define RVFI but keep the production geometry.
`ifdef RVFI
`ifndef RVFI_DII
`define RVFI_TINY_CACHES
`endif
`endif

`ifdef RV32
`ifdef RVFI_TINY_CACHES
// riscv-formal build: tiny geometry so the cache reset loops fit in a BMC bound
`ifdef SV32
import Cache_Decls_RV32_Sv32_1KB_1way :: *;   // 34-bit PA (Gen_BSV_Cache_Decls.py)
export Cache_Decls_RV32_Sv32_1KB_1way :: *;
`else
import Cache_Decls_RV32_1KB_1way :: *;
export Cache_Decls_RV32_1KB_1way :: *;
`endif
`elsif SV32
import Cache_Decls_RV32_Sv32_8KB_2way :: *;
export Cache_Decls_RV32_Sv32_8KB_2way :: *;
`else
import Cache_Decls_RV32_8KB_2way :: *;
export Cache_Decls_RV32_8KB_2way :: *;
`endif
`endif

`ifdef RV64
`ifdef SV39
import Cache_Decls_RV64_Sv39_8KB_2way :: *;
export Cache_Decls_RV64_Sv39_8KB_2way :: *;
`else
import Cache_Decls_RV64_8KB_2way :: *;
export Cache_Decls_RV64_8KB_2way :: *;
`endif
`endif

endpackage
