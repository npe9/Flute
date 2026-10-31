// Copyright (c) 2026 -- ULX3S Flute port.
// RISCV_XBox: bitmanip (Zba/Zbb/Zbc/Zbs, -D ISA_B) and scalar crypto Zkn (Zbkb/Zbkc/Zbkx/
// Zkne/Zknd/Zknh, -D ISA_ZKN) execution unit for RV32, in Stage2 like the MBox.
//
// Every op has the same fixed latency (req, one compute cycle, valid), independent of the
// operand values: Zkt (data-independent execution latency) for the Zkn ops, and it keeps
// all B/K logic off Stage1 (the ALU there is on the critical path).
// Semantics: the Sail RISC-V model (model/riscv_insts_{zba,zbb,zbc,zbs,zbkb,zbkx,zkn}.sail,
// riscv_types_kext.sail); the same transcription is in riscv-formal (branch bk-ext,
// insns/generate.py), checked against the Sail emulator, and the riscv-formal insn checks
// run these models against this unit.

package RISCV_XBox;

import ConfigReg :: *;
import ISA_Decls :: *;

export XOp (..), fv_xbox_decode, RISCV_XBox_IFC (..), mkRISCV_XBox;

// ================================================================

typedef enum {
   // Zba
   X_SH1ADD, X_SH2ADD, X_SH3ADD,
   // Zbb (ANDN ORN XNOR ROL ROR RORI REV8 also Zbkb)
   X_ANDN, X_ORN, X_XNOR, X_MAX, X_MAXU, X_MIN, X_MINU, X_ROL, X_ROR, X_RORI,
   X_CLZ, X_CTZ, X_CPOP, X_SEXTB, X_SEXTH, X_ZEXTH, X_ORCB, X_REV8,
   // Zbc (CLMUL CLMULH also Zbkc)
   X_CLMUL, X_CLMULH, X_CLMULR,
   // Zbs
   X_BCLR, X_BEXT, X_BINV, X_BSET, X_BCLRI, X_BEXTI, X_BINVI, X_BSETI,
   // Zbkb, Zbkx
   X_PACK, X_PACKH, X_BREV8, X_ZIP, X_UNZIP, X_XPERM8, X_XPERM4,
   // Zkne, Zknd
   X_AES32ESI, X_AES32ESMI, X_AES32DSI, X_AES32DSMI,
   // Zknh
   X_SHA256SUM0, X_SHA256SUM1, X_SHA256SIG0, X_SHA256SIG1,
   X_SHA512SUM0R, X_SHA512SUM1R, X_SHA512SIG0L, X_SHA512SIG0H, X_SHA512SIG1L, X_SHA512SIG1H
   } XOp
deriving (Bits, Eq, FShow);

// ================================================================
// Decode: Valid only for encodings of an enabled extension (anything else stays illegal).
// Encodings from the Sail encdec clauses (RV32).

function Maybe #(XOp) fv_xbox_decode (Instr instr);
   Bit #(7)  opc = instr [6:0];
   Bit #(3)  f3  = instr [14:12];
   Bit #(7)  f7  = instr [31:25];
   Bit #(5)  rs2 = instr [24:20];
   Bit #(12) i12 = instr [31:20];
   Bool b = False;  Bool k = False;
`ifdef ISA_B
   b = True;
`endif
`ifdef ISA_ZKN
   k = True;
`endif
   Bool bk = b || k;
   Maybe #(XOp) m = tagged Invalid;
   function Maybe #(XOp) on (Bool en, XOp op) = (en ? tagged Valid op : tagged Invalid);

   if (opc == 7'b0110011) begin        // OP
      case ({f7, f3})
	 {7'b0010000, 3'b010}: m = on (b,  X_SH1ADD);
	 {7'b0010000, 3'b100}: m = on (b,  X_SH2ADD);
	 {7'b0010000, 3'b110}: m = on (b,  X_SH3ADD);
	 {7'b0100000, 3'b111}: m = on (bk, X_ANDN);
	 {7'b0100000, 3'b110}: m = on (bk, X_ORN);
	 {7'b0100000, 3'b100}: m = on (bk, X_XNOR);
	 {7'b0000101, 3'b110}: m = on (b,  X_MAX);
	 {7'b0000101, 3'b111}: m = on (b,  X_MAXU);
	 {7'b0000101, 3'b100}: m = on (b,  X_MIN);
	 {7'b0000101, 3'b101}: m = on (b,  X_MINU);
	 {7'b0110000, 3'b001}: m = on (bk, X_ROL);
	 {7'b0110000, 3'b101}: m = on (bk, X_ROR);
	 {7'b0000101, 3'b001}: m = on (bk, X_CLMUL);
	 {7'b0000101, 3'b011}: m = on (bk, X_CLMULH);
	 {7'b0000101, 3'b010}: m = on (b,  X_CLMULR);
	 {7'b0100100, 3'b001}: m = on (b,  X_BCLR);
	 {7'b0100100, 3'b101}: m = on (b,  X_BEXT);
	 {7'b0110100, 3'b001}: m = on (b,  X_BINV);
	 {7'b0010100, 3'b001}: m = on (b,  X_BSET);
	 // zext.h (Zbb) is pack rd, rs1, x0 (Zbkb): same encoding, same result
	 {7'b0000100, 3'b100}: m = (k ? tagged Valid X_PACK : on (b && (rs2 == 0), X_ZEXTH));
	 {7'b0000100, 3'b111}: m = on (k,  X_PACKH);
	 {7'b0010100, 3'b100}: m = on (k,  X_XPERM8);
	 {7'b0010100, 3'b010}: m = on (k,  X_XPERM4);
	 {7'b0101000, 3'b000}: m = on (k,  X_SHA512SUM0R);
	 {7'b0101001, 3'b000}: m = on (k,  X_SHA512SUM1R);
	 {7'b0101010, 3'b000}: m = on (k,  X_SHA512SIG0L);
	 {7'b0101110, 3'b000}: m = on (k,  X_SHA512SIG0H);
	 {7'b0101011, 3'b000}: m = on (k,  X_SHA512SIG1L);
	 {7'b0101111, 3'b000}: m = on (k,  X_SHA512SIG1H);
      endcase
      // AES32: bs = instr [31:30], funct5 = instr [29:25], funct3 = 000
      if (f3 == 3'b000)
	 case (instr [29:25])
	    5'b10001: m = on (k, X_AES32ESI);
	    5'b10011: m = on (k, X_AES32ESMI);
	    5'b10101: m = on (k, X_AES32DSI);
	    5'b10111: m = on (k, X_AES32DSMI);
	 endcase
   end
   else if (opc == 7'b0010011) begin   // OP-IMM
      if (f3 == 3'b001)
	 case (i12)
	    12'b011000000000: m = on (b, X_CLZ);
	    12'b011000000001: m = on (b, X_CTZ);
	    12'b011000000010: m = on (b, X_CPOP);
	    12'b011000000100: m = on (b, X_SEXTB);
	    12'b011000000101: m = on (b, X_SEXTH);
	    12'b000100000000: m = on (k, X_SHA256SUM0);
	    12'b000100000001: m = on (k, X_SHA256SUM1);
	    12'b000100000010: m = on (k, X_SHA256SIG0);
	    12'b000100000011: m = on (k, X_SHA256SIG1);
	    12'b000010001111: m = on (k, X_ZIP);
	    default:
	       case (f7)    // shamt forms (RV32: shamt [5] = instr [25] = 0)
		  7'b0100100: m = on (b, X_BCLRI);
		  7'b0110100: m = on (b, X_BINVI);
		  7'b0010100: m = on (b, X_BSETI);
	       endcase
	 endcase
      else if (f3 == 3'b101)
	 case (i12)
	    12'b001010000111: m = on (b,  X_ORCB);
	    12'b011010011000: m = on (bk, X_REV8);
	    12'b011010000111: m = on (k,  X_BREV8);
	    12'b000010001111: m = on (k,  X_UNZIP);
	    default:
	       case (f7)
		  7'b0110000: m = on (bk, X_RORI);
		  7'b0100100: m = on (b,  X_BEXTI);
	       endcase
	 endcase
   end
   return m;
endfunction

// ================================================================
// Helpers (Sail riscv_types_kext.sail: xt2, gfmul, aes_mixcolumn_byte_fwd/inv)

function Bit #(32) fn_ror32 (Bit #(32) x, Integer n) = ((x >> n) | (x << (32 - n)));

function Bit #(32) fn_rol_var (Bit #(32) x, Bit #(5) s);
   Bit #(6) s6 = zeroExtend (s);
   return ((x << s6) | (x >> (6'd32 - s6)));
endfunction

function Bit #(32) fn_ror_var (Bit #(32) x, Bit #(5) s);
   Bit #(6) s6 = zeroExtend (s);
   return ((x >> s6) | (x << (6'd32 - s6)));
endfunction

function Bit #(8) fn_xt2 (Bit #(8) x) = ({x [6:0], 1'b0} ^ ((x [7] == 1'b1) ? 8'h1b : 8'h00));

function Bit #(8) fn_gfmul (Bit #(8) x, Bit #(4) y);
   Bit #(8) x2 = fn_xt2 (x);
   Bit #(8) x4 = fn_xt2 (x2);
   Bit #(8) x8 = fn_xt2 (x4);
   return (  ((y [0] == 1'b1) ? x  : 0) ^ ((y [1] == 1'b1) ? x2 : 0)
	   ^ ((y [2] == 1'b1) ? x4 : 0) ^ ((y [3] == 1'b1) ? x8 : 0));
endfunction

// AES forward S-box: generated from sail-riscv model/riscv_types_kext.sail (aes_sbox_fwd_table)
function Bit #(8) fn_aes_sbox_fwd (Bit #(8) x);
   Bit #(8) y = 0;
   case (x)
      8'h00: y = 8'h63; 8'h01: y = 8'h7c; 8'h02: y = 8'h77; 8'h03: y = 8'h7b;
      8'h04: y = 8'hf2; 8'h05: y = 8'h6b; 8'h06: y = 8'h6f; 8'h07: y = 8'hc5;
      8'h08: y = 8'h30; 8'h09: y = 8'h01; 8'h0a: y = 8'h67; 8'h0b: y = 8'h2b;
      8'h0c: y = 8'hfe; 8'h0d: y = 8'hd7; 8'h0e: y = 8'hab; 8'h0f: y = 8'h76;
      8'h10: y = 8'hca; 8'h11: y = 8'h82; 8'h12: y = 8'hc9; 8'h13: y = 8'h7d;
      8'h14: y = 8'hfa; 8'h15: y = 8'h59; 8'h16: y = 8'h47; 8'h17: y = 8'hf0;
      8'h18: y = 8'had; 8'h19: y = 8'hd4; 8'h1a: y = 8'ha2; 8'h1b: y = 8'haf;
      8'h1c: y = 8'h9c; 8'h1d: y = 8'ha4; 8'h1e: y = 8'h72; 8'h1f: y = 8'hc0;
      8'h20: y = 8'hb7; 8'h21: y = 8'hfd; 8'h22: y = 8'h93; 8'h23: y = 8'h26;
      8'h24: y = 8'h36; 8'h25: y = 8'h3f; 8'h26: y = 8'hf7; 8'h27: y = 8'hcc;
      8'h28: y = 8'h34; 8'h29: y = 8'ha5; 8'h2a: y = 8'he5; 8'h2b: y = 8'hf1;
      8'h2c: y = 8'h71; 8'h2d: y = 8'hd8; 8'h2e: y = 8'h31; 8'h2f: y = 8'h15;
      8'h30: y = 8'h04; 8'h31: y = 8'hc7; 8'h32: y = 8'h23; 8'h33: y = 8'hc3;
      8'h34: y = 8'h18; 8'h35: y = 8'h96; 8'h36: y = 8'h05; 8'h37: y = 8'h9a;
      8'h38: y = 8'h07; 8'h39: y = 8'h12; 8'h3a: y = 8'h80; 8'h3b: y = 8'he2;
      8'h3c: y = 8'heb; 8'h3d: y = 8'h27; 8'h3e: y = 8'hb2; 8'h3f: y = 8'h75;
      8'h40: y = 8'h09; 8'h41: y = 8'h83; 8'h42: y = 8'h2c; 8'h43: y = 8'h1a;
      8'h44: y = 8'h1b; 8'h45: y = 8'h6e; 8'h46: y = 8'h5a; 8'h47: y = 8'ha0;
      8'h48: y = 8'h52; 8'h49: y = 8'h3b; 8'h4a: y = 8'hd6; 8'h4b: y = 8'hb3;
      8'h4c: y = 8'h29; 8'h4d: y = 8'he3; 8'h4e: y = 8'h2f; 8'h4f: y = 8'h84;
      8'h50: y = 8'h53; 8'h51: y = 8'hd1; 8'h52: y = 8'h00; 8'h53: y = 8'hed;
      8'h54: y = 8'h20; 8'h55: y = 8'hfc; 8'h56: y = 8'hb1; 8'h57: y = 8'h5b;
      8'h58: y = 8'h6a; 8'h59: y = 8'hcb; 8'h5a: y = 8'hbe; 8'h5b: y = 8'h39;
      8'h5c: y = 8'h4a; 8'h5d: y = 8'h4c; 8'h5e: y = 8'h58; 8'h5f: y = 8'hcf;
      8'h60: y = 8'hd0; 8'h61: y = 8'hef; 8'h62: y = 8'haa; 8'h63: y = 8'hfb;
      8'h64: y = 8'h43; 8'h65: y = 8'h4d; 8'h66: y = 8'h33; 8'h67: y = 8'h85;
      8'h68: y = 8'h45; 8'h69: y = 8'hf9; 8'h6a: y = 8'h02; 8'h6b: y = 8'h7f;
      8'h6c: y = 8'h50; 8'h6d: y = 8'h3c; 8'h6e: y = 8'h9f; 8'h6f: y = 8'ha8;
      8'h70: y = 8'h51; 8'h71: y = 8'ha3; 8'h72: y = 8'h40; 8'h73: y = 8'h8f;
      8'h74: y = 8'h92; 8'h75: y = 8'h9d; 8'h76: y = 8'h38; 8'h77: y = 8'hf5;
      8'h78: y = 8'hbc; 8'h79: y = 8'hb6; 8'h7a: y = 8'hda; 8'h7b: y = 8'h21;
      8'h7c: y = 8'h10; 8'h7d: y = 8'hff; 8'h7e: y = 8'hf3; 8'h7f: y = 8'hd2;
      8'h80: y = 8'hcd; 8'h81: y = 8'h0c; 8'h82: y = 8'h13; 8'h83: y = 8'hec;
      8'h84: y = 8'h5f; 8'h85: y = 8'h97; 8'h86: y = 8'h44; 8'h87: y = 8'h17;
      8'h88: y = 8'hc4; 8'h89: y = 8'ha7; 8'h8a: y = 8'h7e; 8'h8b: y = 8'h3d;
      8'h8c: y = 8'h64; 8'h8d: y = 8'h5d; 8'h8e: y = 8'h19; 8'h8f: y = 8'h73;
      8'h90: y = 8'h60; 8'h91: y = 8'h81; 8'h92: y = 8'h4f; 8'h93: y = 8'hdc;
      8'h94: y = 8'h22; 8'h95: y = 8'h2a; 8'h96: y = 8'h90; 8'h97: y = 8'h88;
      8'h98: y = 8'h46; 8'h99: y = 8'hee; 8'h9a: y = 8'hb8; 8'h9b: y = 8'h14;
      8'h9c: y = 8'hde; 8'h9d: y = 8'h5e; 8'h9e: y = 8'h0b; 8'h9f: y = 8'hdb;
      8'ha0: y = 8'he0; 8'ha1: y = 8'h32; 8'ha2: y = 8'h3a; 8'ha3: y = 8'h0a;
      8'ha4: y = 8'h49; 8'ha5: y = 8'h06; 8'ha6: y = 8'h24; 8'ha7: y = 8'h5c;
      8'ha8: y = 8'hc2; 8'ha9: y = 8'hd3; 8'haa: y = 8'hac; 8'hab: y = 8'h62;
      8'hac: y = 8'h91; 8'had: y = 8'h95; 8'hae: y = 8'he4; 8'haf: y = 8'h79;
      8'hb0: y = 8'he7; 8'hb1: y = 8'hc8; 8'hb2: y = 8'h37; 8'hb3: y = 8'h6d;
      8'hb4: y = 8'h8d; 8'hb5: y = 8'hd5; 8'hb6: y = 8'h4e; 8'hb7: y = 8'ha9;
      8'hb8: y = 8'h6c; 8'hb9: y = 8'h56; 8'hba: y = 8'hf4; 8'hbb: y = 8'hea;
      8'hbc: y = 8'h65; 8'hbd: y = 8'h7a; 8'hbe: y = 8'hae; 8'hbf: y = 8'h08;
      8'hc0: y = 8'hba; 8'hc1: y = 8'h78; 8'hc2: y = 8'h25; 8'hc3: y = 8'h2e;
      8'hc4: y = 8'h1c; 8'hc5: y = 8'ha6; 8'hc6: y = 8'hb4; 8'hc7: y = 8'hc6;
      8'hc8: y = 8'he8; 8'hc9: y = 8'hdd; 8'hca: y = 8'h74; 8'hcb: y = 8'h1f;
      8'hcc: y = 8'h4b; 8'hcd: y = 8'hbd; 8'hce: y = 8'h8b; 8'hcf: y = 8'h8a;
      8'hd0: y = 8'h70; 8'hd1: y = 8'h3e; 8'hd2: y = 8'hb5; 8'hd3: y = 8'h66;
      8'hd4: y = 8'h48; 8'hd5: y = 8'h03; 8'hd6: y = 8'hf6; 8'hd7: y = 8'h0e;
      8'hd8: y = 8'h61; 8'hd9: y = 8'h35; 8'hda: y = 8'h57; 8'hdb: y = 8'hb9;
      8'hdc: y = 8'h86; 8'hdd: y = 8'hc1; 8'hde: y = 8'h1d; 8'hdf: y = 8'h9e;
      8'he0: y = 8'he1; 8'he1: y = 8'hf8; 8'he2: y = 8'h98; 8'he3: y = 8'h11;
      8'he4: y = 8'h69; 8'he5: y = 8'hd9; 8'he6: y = 8'h8e; 8'he7: y = 8'h94;
      8'he8: y = 8'h9b; 8'he9: y = 8'h1e; 8'hea: y = 8'h87; 8'heb: y = 8'he9;
      8'hec: y = 8'hce; 8'hed: y = 8'h55; 8'hee: y = 8'h28; 8'hef: y = 8'hdf;
      8'hf0: y = 8'h8c; 8'hf1: y = 8'ha1; 8'hf2: y = 8'h89; 8'hf3: y = 8'h0d;
      8'hf4: y = 8'hbf; 8'hf5: y = 8'he6; 8'hf6: y = 8'h42; 8'hf7: y = 8'h68;
      8'hf8: y = 8'h41; 8'hf9: y = 8'h99; 8'hfa: y = 8'h2d; 8'hfb: y = 8'h0f;
      8'hfc: y = 8'hb0; 8'hfd: y = 8'h54; 8'hfe: y = 8'hbb; 8'hff: y = 8'h16;
   endcase
   return y;
endfunction

// AES inverse S-box: generated from sail-riscv model/riscv_types_kext.sail (aes_sbox_inv_table)
function Bit #(8) fn_aes_sbox_inv (Bit #(8) x);
   Bit #(8) y = 0;
   case (x)
      8'h00: y = 8'h52; 8'h01: y = 8'h09; 8'h02: y = 8'h6a; 8'h03: y = 8'hd5;
      8'h04: y = 8'h30; 8'h05: y = 8'h36; 8'h06: y = 8'ha5; 8'h07: y = 8'h38;
      8'h08: y = 8'hbf; 8'h09: y = 8'h40; 8'h0a: y = 8'ha3; 8'h0b: y = 8'h9e;
      8'h0c: y = 8'h81; 8'h0d: y = 8'hf3; 8'h0e: y = 8'hd7; 8'h0f: y = 8'hfb;
      8'h10: y = 8'h7c; 8'h11: y = 8'he3; 8'h12: y = 8'h39; 8'h13: y = 8'h82;
      8'h14: y = 8'h9b; 8'h15: y = 8'h2f; 8'h16: y = 8'hff; 8'h17: y = 8'h87;
      8'h18: y = 8'h34; 8'h19: y = 8'h8e; 8'h1a: y = 8'h43; 8'h1b: y = 8'h44;
      8'h1c: y = 8'hc4; 8'h1d: y = 8'hde; 8'h1e: y = 8'he9; 8'h1f: y = 8'hcb;
      8'h20: y = 8'h54; 8'h21: y = 8'h7b; 8'h22: y = 8'h94; 8'h23: y = 8'h32;
      8'h24: y = 8'ha6; 8'h25: y = 8'hc2; 8'h26: y = 8'h23; 8'h27: y = 8'h3d;
      8'h28: y = 8'hee; 8'h29: y = 8'h4c; 8'h2a: y = 8'h95; 8'h2b: y = 8'h0b;
      8'h2c: y = 8'h42; 8'h2d: y = 8'hfa; 8'h2e: y = 8'hc3; 8'h2f: y = 8'h4e;
      8'h30: y = 8'h08; 8'h31: y = 8'h2e; 8'h32: y = 8'ha1; 8'h33: y = 8'h66;
      8'h34: y = 8'h28; 8'h35: y = 8'hd9; 8'h36: y = 8'h24; 8'h37: y = 8'hb2;
      8'h38: y = 8'h76; 8'h39: y = 8'h5b; 8'h3a: y = 8'ha2; 8'h3b: y = 8'h49;
      8'h3c: y = 8'h6d; 8'h3d: y = 8'h8b; 8'h3e: y = 8'hd1; 8'h3f: y = 8'h25;
      8'h40: y = 8'h72; 8'h41: y = 8'hf8; 8'h42: y = 8'hf6; 8'h43: y = 8'h64;
      8'h44: y = 8'h86; 8'h45: y = 8'h68; 8'h46: y = 8'h98; 8'h47: y = 8'h16;
      8'h48: y = 8'hd4; 8'h49: y = 8'ha4; 8'h4a: y = 8'h5c; 8'h4b: y = 8'hcc;
      8'h4c: y = 8'h5d; 8'h4d: y = 8'h65; 8'h4e: y = 8'hb6; 8'h4f: y = 8'h92;
      8'h50: y = 8'h6c; 8'h51: y = 8'h70; 8'h52: y = 8'h48; 8'h53: y = 8'h50;
      8'h54: y = 8'hfd; 8'h55: y = 8'hed; 8'h56: y = 8'hb9; 8'h57: y = 8'hda;
      8'h58: y = 8'h5e; 8'h59: y = 8'h15; 8'h5a: y = 8'h46; 8'h5b: y = 8'h57;
      8'h5c: y = 8'ha7; 8'h5d: y = 8'h8d; 8'h5e: y = 8'h9d; 8'h5f: y = 8'h84;
      8'h60: y = 8'h90; 8'h61: y = 8'hd8; 8'h62: y = 8'hab; 8'h63: y = 8'h00;
      8'h64: y = 8'h8c; 8'h65: y = 8'hbc; 8'h66: y = 8'hd3; 8'h67: y = 8'h0a;
      8'h68: y = 8'hf7; 8'h69: y = 8'he4; 8'h6a: y = 8'h58; 8'h6b: y = 8'h05;
      8'h6c: y = 8'hb8; 8'h6d: y = 8'hb3; 8'h6e: y = 8'h45; 8'h6f: y = 8'h06;
      8'h70: y = 8'hd0; 8'h71: y = 8'h2c; 8'h72: y = 8'h1e; 8'h73: y = 8'h8f;
      8'h74: y = 8'hca; 8'h75: y = 8'h3f; 8'h76: y = 8'h0f; 8'h77: y = 8'h02;
      8'h78: y = 8'hc1; 8'h79: y = 8'haf; 8'h7a: y = 8'hbd; 8'h7b: y = 8'h03;
      8'h7c: y = 8'h01; 8'h7d: y = 8'h13; 8'h7e: y = 8'h8a; 8'h7f: y = 8'h6b;
      8'h80: y = 8'h3a; 8'h81: y = 8'h91; 8'h82: y = 8'h11; 8'h83: y = 8'h41;
      8'h84: y = 8'h4f; 8'h85: y = 8'h67; 8'h86: y = 8'hdc; 8'h87: y = 8'hea;
      8'h88: y = 8'h97; 8'h89: y = 8'hf2; 8'h8a: y = 8'hcf; 8'h8b: y = 8'hce;
      8'h8c: y = 8'hf0; 8'h8d: y = 8'hb4; 8'h8e: y = 8'he6; 8'h8f: y = 8'h73;
      8'h90: y = 8'h96; 8'h91: y = 8'hac; 8'h92: y = 8'h74; 8'h93: y = 8'h22;
      8'h94: y = 8'he7; 8'h95: y = 8'had; 8'h96: y = 8'h35; 8'h97: y = 8'h85;
      8'h98: y = 8'he2; 8'h99: y = 8'hf9; 8'h9a: y = 8'h37; 8'h9b: y = 8'he8;
      8'h9c: y = 8'h1c; 8'h9d: y = 8'h75; 8'h9e: y = 8'hdf; 8'h9f: y = 8'h6e;
      8'ha0: y = 8'h47; 8'ha1: y = 8'hf1; 8'ha2: y = 8'h1a; 8'ha3: y = 8'h71;
      8'ha4: y = 8'h1d; 8'ha5: y = 8'h29; 8'ha6: y = 8'hc5; 8'ha7: y = 8'h89;
      8'ha8: y = 8'h6f; 8'ha9: y = 8'hb7; 8'haa: y = 8'h62; 8'hab: y = 8'h0e;
      8'hac: y = 8'haa; 8'had: y = 8'h18; 8'hae: y = 8'hbe; 8'haf: y = 8'h1b;
      8'hb0: y = 8'hfc; 8'hb1: y = 8'h56; 8'hb2: y = 8'h3e; 8'hb3: y = 8'h4b;
      8'hb4: y = 8'hc6; 8'hb5: y = 8'hd2; 8'hb6: y = 8'h79; 8'hb7: y = 8'h20;
      8'hb8: y = 8'h9a; 8'hb9: y = 8'hdb; 8'hba: y = 8'hc0; 8'hbb: y = 8'hfe;
      8'hbc: y = 8'h78; 8'hbd: y = 8'hcd; 8'hbe: y = 8'h5a; 8'hbf: y = 8'hf4;
      8'hc0: y = 8'h1f; 8'hc1: y = 8'hdd; 8'hc2: y = 8'ha8; 8'hc3: y = 8'h33;
      8'hc4: y = 8'h88; 8'hc5: y = 8'h07; 8'hc6: y = 8'hc7; 8'hc7: y = 8'h31;
      8'hc8: y = 8'hb1; 8'hc9: y = 8'h12; 8'hca: y = 8'h10; 8'hcb: y = 8'h59;
      8'hcc: y = 8'h27; 8'hcd: y = 8'h80; 8'hce: y = 8'hec; 8'hcf: y = 8'h5f;
      8'hd0: y = 8'h60; 8'hd1: y = 8'h51; 8'hd2: y = 8'h7f; 8'hd3: y = 8'ha9;
      8'hd4: y = 8'h19; 8'hd5: y = 8'hb5; 8'hd6: y = 8'h4a; 8'hd7: y = 8'h0d;
      8'hd8: y = 8'h2d; 8'hd9: y = 8'he5; 8'hda: y = 8'h7a; 8'hdb: y = 8'h9f;
      8'hdc: y = 8'h93; 8'hdd: y = 8'hc9; 8'hde: y = 8'h9c; 8'hdf: y = 8'hef;
      8'he0: y = 8'ha0; 8'he1: y = 8'he0; 8'he2: y = 8'h3b; 8'he3: y = 8'h4d;
      8'he4: y = 8'hae; 8'he5: y = 8'h2a; 8'he6: y = 8'hf5; 8'he7: y = 8'hb0;
      8'he8: y = 8'hc8; 8'he9: y = 8'heb; 8'hea: y = 8'hbb; 8'heb: y = 8'h3c;
      8'hec: y = 8'h83; 8'hed: y = 8'h53; 8'hee: y = 8'h99; 8'hef: y = 8'h61;
      8'hf0: y = 8'h17; 8'hf1: y = 8'h2b; 8'hf2: y = 8'h04; 8'hf3: y = 8'h7e;
      8'hf4: y = 8'hba; 8'hf5: y = 8'h77; 8'hf6: y = 8'hd6; 8'hf7: y = 8'h26;
      8'hf8: y = 8'he1; 8'hf9: y = 8'h69; 8'hfa: y = 8'h14; 8'hfb: y = 8'h63;
      8'hfc: y = 8'h55; 8'hfd: y = 8'h21; 8'hfe: y = 8'h0c; 8'hff: y = 8'h7d;
   endcase
   return y;
endfunction

// LX29: both S-boxes as one 512-entry table ({inverse, byte}) feeding rg_so directly, so
// yosys maps it (registered read) into a block RAM instead of ~1K LUTs of two muxed ROMs.
function Bit #(8) fn_aes_sbox_both (Bit #(9) x);
   Bit #(8) y = 0;
   case (x)
      9'h000: y = 8'h63; 9'h001: y = 8'h7c; 9'h002: y = 8'h77; 9'h003: y = 8'h7b;
      9'h004: y = 8'hf2; 9'h005: y = 8'h6b; 9'h006: y = 8'h6f; 9'h007: y = 8'hc5;
      9'h008: y = 8'h30; 9'h009: y = 8'h01; 9'h00a: y = 8'h67; 9'h00b: y = 8'h2b;
      9'h00c: y = 8'hfe; 9'h00d: y = 8'hd7; 9'h00e: y = 8'hab; 9'h00f: y = 8'h76;
      9'h010: y = 8'hca; 9'h011: y = 8'h82; 9'h012: y = 8'hc9; 9'h013: y = 8'h7d;
      9'h014: y = 8'hfa; 9'h015: y = 8'h59; 9'h016: y = 8'h47; 9'h017: y = 8'hf0;
      9'h018: y = 8'had; 9'h019: y = 8'hd4; 9'h01a: y = 8'ha2; 9'h01b: y = 8'haf;
      9'h01c: y = 8'h9c; 9'h01d: y = 8'ha4; 9'h01e: y = 8'h72; 9'h01f: y = 8'hc0;
      9'h020: y = 8'hb7; 9'h021: y = 8'hfd; 9'h022: y = 8'h93; 9'h023: y = 8'h26;
      9'h024: y = 8'h36; 9'h025: y = 8'h3f; 9'h026: y = 8'hf7; 9'h027: y = 8'hcc;
      9'h028: y = 8'h34; 9'h029: y = 8'ha5; 9'h02a: y = 8'he5; 9'h02b: y = 8'hf1;
      9'h02c: y = 8'h71; 9'h02d: y = 8'hd8; 9'h02e: y = 8'h31; 9'h02f: y = 8'h15;
      9'h030: y = 8'h04; 9'h031: y = 8'hc7; 9'h032: y = 8'h23; 9'h033: y = 8'hc3;
      9'h034: y = 8'h18; 9'h035: y = 8'h96; 9'h036: y = 8'h05; 9'h037: y = 8'h9a;
      9'h038: y = 8'h07; 9'h039: y = 8'h12; 9'h03a: y = 8'h80; 9'h03b: y = 8'he2;
      9'h03c: y = 8'heb; 9'h03d: y = 8'h27; 9'h03e: y = 8'hb2; 9'h03f: y = 8'h75;
      9'h040: y = 8'h09; 9'h041: y = 8'h83; 9'h042: y = 8'h2c; 9'h043: y = 8'h1a;
      9'h044: y = 8'h1b; 9'h045: y = 8'h6e; 9'h046: y = 8'h5a; 9'h047: y = 8'ha0;
      9'h048: y = 8'h52; 9'h049: y = 8'h3b; 9'h04a: y = 8'hd6; 9'h04b: y = 8'hb3;
      9'h04c: y = 8'h29; 9'h04d: y = 8'he3; 9'h04e: y = 8'h2f; 9'h04f: y = 8'h84;
      9'h050: y = 8'h53; 9'h051: y = 8'hd1; 9'h052: y = 8'h00; 9'h053: y = 8'hed;
      9'h054: y = 8'h20; 9'h055: y = 8'hfc; 9'h056: y = 8'hb1; 9'h057: y = 8'h5b;
      9'h058: y = 8'h6a; 9'h059: y = 8'hcb; 9'h05a: y = 8'hbe; 9'h05b: y = 8'h39;
      9'h05c: y = 8'h4a; 9'h05d: y = 8'h4c; 9'h05e: y = 8'h58; 9'h05f: y = 8'hcf;
      9'h060: y = 8'hd0; 9'h061: y = 8'hef; 9'h062: y = 8'haa; 9'h063: y = 8'hfb;
      9'h064: y = 8'h43; 9'h065: y = 8'h4d; 9'h066: y = 8'h33; 9'h067: y = 8'h85;
      9'h068: y = 8'h45; 9'h069: y = 8'hf9; 9'h06a: y = 8'h02; 9'h06b: y = 8'h7f;
      9'h06c: y = 8'h50; 9'h06d: y = 8'h3c; 9'h06e: y = 8'h9f; 9'h06f: y = 8'ha8;
      9'h070: y = 8'h51; 9'h071: y = 8'ha3; 9'h072: y = 8'h40; 9'h073: y = 8'h8f;
      9'h074: y = 8'h92; 9'h075: y = 8'h9d; 9'h076: y = 8'h38; 9'h077: y = 8'hf5;
      9'h078: y = 8'hbc; 9'h079: y = 8'hb6; 9'h07a: y = 8'hda; 9'h07b: y = 8'h21;
      9'h07c: y = 8'h10; 9'h07d: y = 8'hff; 9'h07e: y = 8'hf3; 9'h07f: y = 8'hd2;
      9'h080: y = 8'hcd; 9'h081: y = 8'h0c; 9'h082: y = 8'h13; 9'h083: y = 8'hec;
      9'h084: y = 8'h5f; 9'h085: y = 8'h97; 9'h086: y = 8'h44; 9'h087: y = 8'h17;
      9'h088: y = 8'hc4; 9'h089: y = 8'ha7; 9'h08a: y = 8'h7e; 9'h08b: y = 8'h3d;
      9'h08c: y = 8'h64; 9'h08d: y = 8'h5d; 9'h08e: y = 8'h19; 9'h08f: y = 8'h73;
      9'h090: y = 8'h60; 9'h091: y = 8'h81; 9'h092: y = 8'h4f; 9'h093: y = 8'hdc;
      9'h094: y = 8'h22; 9'h095: y = 8'h2a; 9'h096: y = 8'h90; 9'h097: y = 8'h88;
      9'h098: y = 8'h46; 9'h099: y = 8'hee; 9'h09a: y = 8'hb8; 9'h09b: y = 8'h14;
      9'h09c: y = 8'hde; 9'h09d: y = 8'h5e; 9'h09e: y = 8'h0b; 9'h09f: y = 8'hdb;
      9'h0a0: y = 8'he0; 9'h0a1: y = 8'h32; 9'h0a2: y = 8'h3a; 9'h0a3: y = 8'h0a;
      9'h0a4: y = 8'h49; 9'h0a5: y = 8'h06; 9'h0a6: y = 8'h24; 9'h0a7: y = 8'h5c;
      9'h0a8: y = 8'hc2; 9'h0a9: y = 8'hd3; 9'h0aa: y = 8'hac; 9'h0ab: y = 8'h62;
      9'h0ac: y = 8'h91; 9'h0ad: y = 8'h95; 9'h0ae: y = 8'he4; 9'h0af: y = 8'h79;
      9'h0b0: y = 8'he7; 9'h0b1: y = 8'hc8; 9'h0b2: y = 8'h37; 9'h0b3: y = 8'h6d;
      9'h0b4: y = 8'h8d; 9'h0b5: y = 8'hd5; 9'h0b6: y = 8'h4e; 9'h0b7: y = 8'ha9;
      9'h0b8: y = 8'h6c; 9'h0b9: y = 8'h56; 9'h0ba: y = 8'hf4; 9'h0bb: y = 8'hea;
      9'h0bc: y = 8'h65; 9'h0bd: y = 8'h7a; 9'h0be: y = 8'hae; 9'h0bf: y = 8'h08;
      9'h0c0: y = 8'hba; 9'h0c1: y = 8'h78; 9'h0c2: y = 8'h25; 9'h0c3: y = 8'h2e;
      9'h0c4: y = 8'h1c; 9'h0c5: y = 8'ha6; 9'h0c6: y = 8'hb4; 9'h0c7: y = 8'hc6;
      9'h0c8: y = 8'he8; 9'h0c9: y = 8'hdd; 9'h0ca: y = 8'h74; 9'h0cb: y = 8'h1f;
      9'h0cc: y = 8'h4b; 9'h0cd: y = 8'hbd; 9'h0ce: y = 8'h8b; 9'h0cf: y = 8'h8a;
      9'h0d0: y = 8'h70; 9'h0d1: y = 8'h3e; 9'h0d2: y = 8'hb5; 9'h0d3: y = 8'h66;
      9'h0d4: y = 8'h48; 9'h0d5: y = 8'h03; 9'h0d6: y = 8'hf6; 9'h0d7: y = 8'h0e;
      9'h0d8: y = 8'h61; 9'h0d9: y = 8'h35; 9'h0da: y = 8'h57; 9'h0db: y = 8'hb9;
      9'h0dc: y = 8'h86; 9'h0dd: y = 8'hc1; 9'h0de: y = 8'h1d; 9'h0df: y = 8'h9e;
      9'h0e0: y = 8'he1; 9'h0e1: y = 8'hf8; 9'h0e2: y = 8'h98; 9'h0e3: y = 8'h11;
      9'h0e4: y = 8'h69; 9'h0e5: y = 8'hd9; 9'h0e6: y = 8'h8e; 9'h0e7: y = 8'h94;
      9'h0e8: y = 8'h9b; 9'h0e9: y = 8'h1e; 9'h0ea: y = 8'h87; 9'h0eb: y = 8'he9;
      9'h0ec: y = 8'hce; 9'h0ed: y = 8'h55; 9'h0ee: y = 8'h28; 9'h0ef: y = 8'hdf;
      9'h0f0: y = 8'h8c; 9'h0f1: y = 8'ha1; 9'h0f2: y = 8'h89; 9'h0f3: y = 8'h0d;
      9'h0f4: y = 8'hbf; 9'h0f5: y = 8'he6; 9'h0f6: y = 8'h42; 9'h0f7: y = 8'h68;
      9'h0f8: y = 8'h41; 9'h0f9: y = 8'h99; 9'h0fa: y = 8'h2d; 9'h0fb: y = 8'h0f;
      9'h0fc: y = 8'hb0; 9'h0fd: y = 8'h54; 9'h0fe: y = 8'hbb; 9'h0ff: y = 8'h16;
      9'h100: y = 8'h52; 9'h101: y = 8'h09; 9'h102: y = 8'h6a; 9'h103: y = 8'hd5;
      9'h104: y = 8'h30; 9'h105: y = 8'h36; 9'h106: y = 8'ha5; 9'h107: y = 8'h38;
      9'h108: y = 8'hbf; 9'h109: y = 8'h40; 9'h10a: y = 8'ha3; 9'h10b: y = 8'h9e;
      9'h10c: y = 8'h81; 9'h10d: y = 8'hf3; 9'h10e: y = 8'hd7; 9'h10f: y = 8'hfb;
      9'h110: y = 8'h7c; 9'h111: y = 8'he3; 9'h112: y = 8'h39; 9'h113: y = 8'h82;
      9'h114: y = 8'h9b; 9'h115: y = 8'h2f; 9'h116: y = 8'hff; 9'h117: y = 8'h87;
      9'h118: y = 8'h34; 9'h119: y = 8'h8e; 9'h11a: y = 8'h43; 9'h11b: y = 8'h44;
      9'h11c: y = 8'hc4; 9'h11d: y = 8'hde; 9'h11e: y = 8'he9; 9'h11f: y = 8'hcb;
      9'h120: y = 8'h54; 9'h121: y = 8'h7b; 9'h122: y = 8'h94; 9'h123: y = 8'h32;
      9'h124: y = 8'ha6; 9'h125: y = 8'hc2; 9'h126: y = 8'h23; 9'h127: y = 8'h3d;
      9'h128: y = 8'hee; 9'h129: y = 8'h4c; 9'h12a: y = 8'h95; 9'h12b: y = 8'h0b;
      9'h12c: y = 8'h42; 9'h12d: y = 8'hfa; 9'h12e: y = 8'hc3; 9'h12f: y = 8'h4e;
      9'h130: y = 8'h08; 9'h131: y = 8'h2e; 9'h132: y = 8'ha1; 9'h133: y = 8'h66;
      9'h134: y = 8'h28; 9'h135: y = 8'hd9; 9'h136: y = 8'h24; 9'h137: y = 8'hb2;
      9'h138: y = 8'h76; 9'h139: y = 8'h5b; 9'h13a: y = 8'ha2; 9'h13b: y = 8'h49;
      9'h13c: y = 8'h6d; 9'h13d: y = 8'h8b; 9'h13e: y = 8'hd1; 9'h13f: y = 8'h25;
      9'h140: y = 8'h72; 9'h141: y = 8'hf8; 9'h142: y = 8'hf6; 9'h143: y = 8'h64;
      9'h144: y = 8'h86; 9'h145: y = 8'h68; 9'h146: y = 8'h98; 9'h147: y = 8'h16;
      9'h148: y = 8'hd4; 9'h149: y = 8'ha4; 9'h14a: y = 8'h5c; 9'h14b: y = 8'hcc;
      9'h14c: y = 8'h5d; 9'h14d: y = 8'h65; 9'h14e: y = 8'hb6; 9'h14f: y = 8'h92;
      9'h150: y = 8'h6c; 9'h151: y = 8'h70; 9'h152: y = 8'h48; 9'h153: y = 8'h50;
      9'h154: y = 8'hfd; 9'h155: y = 8'hed; 9'h156: y = 8'hb9; 9'h157: y = 8'hda;
      9'h158: y = 8'h5e; 9'h159: y = 8'h15; 9'h15a: y = 8'h46; 9'h15b: y = 8'h57;
      9'h15c: y = 8'ha7; 9'h15d: y = 8'h8d; 9'h15e: y = 8'h9d; 9'h15f: y = 8'h84;
      9'h160: y = 8'h90; 9'h161: y = 8'hd8; 9'h162: y = 8'hab; 9'h163: y = 8'h00;
      9'h164: y = 8'h8c; 9'h165: y = 8'hbc; 9'h166: y = 8'hd3; 9'h167: y = 8'h0a;
      9'h168: y = 8'hf7; 9'h169: y = 8'he4; 9'h16a: y = 8'h58; 9'h16b: y = 8'h05;
      9'h16c: y = 8'hb8; 9'h16d: y = 8'hb3; 9'h16e: y = 8'h45; 9'h16f: y = 8'h06;
      9'h170: y = 8'hd0; 9'h171: y = 8'h2c; 9'h172: y = 8'h1e; 9'h173: y = 8'h8f;
      9'h174: y = 8'hca; 9'h175: y = 8'h3f; 9'h176: y = 8'h0f; 9'h177: y = 8'h02;
      9'h178: y = 8'hc1; 9'h179: y = 8'haf; 9'h17a: y = 8'hbd; 9'h17b: y = 8'h03;
      9'h17c: y = 8'h01; 9'h17d: y = 8'h13; 9'h17e: y = 8'h8a; 9'h17f: y = 8'h6b;
      9'h180: y = 8'h3a; 9'h181: y = 8'h91; 9'h182: y = 8'h11; 9'h183: y = 8'h41;
      9'h184: y = 8'h4f; 9'h185: y = 8'h67; 9'h186: y = 8'hdc; 9'h187: y = 8'hea;
      9'h188: y = 8'h97; 9'h189: y = 8'hf2; 9'h18a: y = 8'hcf; 9'h18b: y = 8'hce;
      9'h18c: y = 8'hf0; 9'h18d: y = 8'hb4; 9'h18e: y = 8'he6; 9'h18f: y = 8'h73;
      9'h190: y = 8'h96; 9'h191: y = 8'hac; 9'h192: y = 8'h74; 9'h193: y = 8'h22;
      9'h194: y = 8'he7; 9'h195: y = 8'had; 9'h196: y = 8'h35; 9'h197: y = 8'h85;
      9'h198: y = 8'he2; 9'h199: y = 8'hf9; 9'h19a: y = 8'h37; 9'h19b: y = 8'he8;
      9'h19c: y = 8'h1c; 9'h19d: y = 8'h75; 9'h19e: y = 8'hdf; 9'h19f: y = 8'h6e;
      9'h1a0: y = 8'h47; 9'h1a1: y = 8'hf1; 9'h1a2: y = 8'h1a; 9'h1a3: y = 8'h71;
      9'h1a4: y = 8'h1d; 9'h1a5: y = 8'h29; 9'h1a6: y = 8'hc5; 9'h1a7: y = 8'h89;
      9'h1a8: y = 8'h6f; 9'h1a9: y = 8'hb7; 9'h1aa: y = 8'h62; 9'h1ab: y = 8'h0e;
      9'h1ac: y = 8'haa; 9'h1ad: y = 8'h18; 9'h1ae: y = 8'hbe; 9'h1af: y = 8'h1b;
      9'h1b0: y = 8'hfc; 9'h1b1: y = 8'h56; 9'h1b2: y = 8'h3e; 9'h1b3: y = 8'h4b;
      9'h1b4: y = 8'hc6; 9'h1b5: y = 8'hd2; 9'h1b6: y = 8'h79; 9'h1b7: y = 8'h20;
      9'h1b8: y = 8'h9a; 9'h1b9: y = 8'hdb; 9'h1ba: y = 8'hc0; 9'h1bb: y = 8'hfe;
      9'h1bc: y = 8'h78; 9'h1bd: y = 8'hcd; 9'h1be: y = 8'h5a; 9'h1bf: y = 8'hf4;
      9'h1c0: y = 8'h1f; 9'h1c1: y = 8'hdd; 9'h1c2: y = 8'ha8; 9'h1c3: y = 8'h33;
      9'h1c4: y = 8'h88; 9'h1c5: y = 8'h07; 9'h1c6: y = 8'hc7; 9'h1c7: y = 8'h31;
      9'h1c8: y = 8'hb1; 9'h1c9: y = 8'h12; 9'h1ca: y = 8'h10; 9'h1cb: y = 8'h59;
      9'h1cc: y = 8'h27; 9'h1cd: y = 8'h80; 9'h1ce: y = 8'hec; 9'h1cf: y = 8'h5f;
      9'h1d0: y = 8'h60; 9'h1d1: y = 8'h51; 9'h1d2: y = 8'h7f; 9'h1d3: y = 8'ha9;
      9'h1d4: y = 8'h19; 9'h1d5: y = 8'hb5; 9'h1d6: y = 8'h4a; 9'h1d7: y = 8'h0d;
      9'h1d8: y = 8'h2d; 9'h1d9: y = 8'he5; 9'h1da: y = 8'h7a; 9'h1db: y = 8'h9f;
      9'h1dc: y = 8'h93; 9'h1dd: y = 8'hc9; 9'h1de: y = 8'h9c; 9'h1df: y = 8'hef;
      9'h1e0: y = 8'ha0; 9'h1e1: y = 8'he0; 9'h1e2: y = 8'h3b; 9'h1e3: y = 8'h4d;
      9'h1e4: y = 8'hae; 9'h1e5: y = 8'h2a; 9'h1e6: y = 8'hf5; 9'h1e7: y = 8'hb0;
      9'h1e8: y = 8'hc8; 9'h1e9: y = 8'heb; 9'h1ea: y = 8'hbb; 9'h1eb: y = 8'h3c;
      9'h1ec: y = 8'h83; 9'h1ed: y = 8'h53; 9'h1ee: y = 8'h99; 9'h1ef: y = 8'h61;
      9'h1f0: y = 8'h17; 9'h1f1: y = 8'h2b; 9'h1f2: y = 8'h04; 9'h1f3: y = 8'h7e;
      9'h1f4: y = 8'hba; 9'h1f5: y = 8'h77; 9'h1f6: y = 8'hd6; 9'h1f7: y = 8'h26;
      9'h1f8: y = 8'he1; 9'h1f9: y = 8'h69; 9'h1fa: y = 8'h14; 9'h1fb: y = 8'h63;
      9'h1fc: y = 8'h55; 9'h1fd: y = 8'h21; 9'h1fe: y = 8'h0c; 9'h1ff: y = 8'h7d;
   endcase
   return y;
endfunction

// ================================================================
// Compute (one cycle, from registered operands)

function WordXL fv_xbox_compute (XOp op, Instr instr, WordXL a, WordXL b);
   // Single-cycle reference (unused in the core): stage A then stage B.
   return fv_xbox_finish (op, instr, a, fv_xbox_compute_main (op, instr, a, b), fv_xbox_clp (a, b), fv_xbox_aes_so (op, instr, b), fv_xbox_cnt (op, a));
endfunction

// LX3: two-stage compute.  Stage A registers the main case (every op but
// clmul / AES / clz / ctz / cpop, which yield 0 there) together with the
// carry-less product, the AES S-box byte and the bit count; stage B picks
// the family and finishes AES (MixColumn column, rotate, xor).  The single-
// cycle form was 17 LUT levels deep (clmul xor tree / S-box -> GF mul ->
// rotate -> 40-way op mux).  +1 cycle per B / Zk op.

function Bit #(64) fv_xbox_clp (WordXL a, WordXL b);
   Bit #(64) clp = 0;
   for (Integer i = 0; i < 32; i = i + 1)
      if (b [i] == 1'b1) clp = clp ^ (zeroExtend (a) << i);
   return clp;
endfunction

function Bit #(8) fv_xbox_aes_so (XOp op, Instr instr, WordXL b);
   Bit #(5)  aes_sh  = {instr [31:30], 3'b000};
   Bit #(8)  si      = truncate (b >> aes_sh);
   Bool      aes_fwd = ((op == X_AES32ESI) || (op == X_AES32ESMI));
   return fn_aes_sbox_both ({ pack (! aes_fwd), si });    // LX29
endfunction

function Bit #(6) fv_xbox_cnt (XOp op, WordXL a);
   return case (op)
	     X_CLZ:   pack (countZerosMSB (a));
	     X_CTZ:   pack (countZerosLSB (a));
	     default: pack (countOnes (a));
	  endcase;
endfunction

function WordXL fv_xbox_finish (XOp op, Instr instr, WordXL a, WordXL part, Bit #(64) clp, Bit #(8) so, Bit #(6) cnt);
   Bit #(5)  aes_sh  = {instr [31:30], 3'b000};
   Bit #(32) mixed   = case (op)
			  X_AES32ESMI: {fn_gfmul (so, 4'h3), so, so, fn_gfmul (so, 4'h2)};
			  X_AES32DSMI: {fn_gfmul (so, 4'hb), fn_gfmul (so, 4'hd), fn_gfmul (so, 4'h9), fn_gfmul (so, 4'he)};
			  default:     zeroExtend (so);
		       endcase;
   return case (op)
	     X_CLMUL:  clp [31:0];
	     X_CLMULH: clp [63:32];
	     X_CLMULR: clp [62:31];
	     X_CLZ, X_CTZ, X_CPOP: zeroExtend (cnt);
	     X_AES32ESI, X_AES32ESMI, X_AES32DSI, X_AES32DSMI: (a ^ fn_rol_var (mixed, aes_sh));
	     default:  part;
	  endcase;
endfunction

function WordXL fv_xbox_compute_main (XOp op, Instr instr, WordXL a, WordXL b);
   Bit #(5) shamt_i = instr [24:20];
   Bit #(5) shamt_r = b [4:0];
   Bit #(5) bidx    = ((op == X_BCLRI) || (op == X_BEXTI) || (op == X_BINVI) || (op == X_BSETI)) ? shamt_i : shamt_r;
   Bit #(32) bmask  = (32'h1 << bidx);

   Bit #(32) r = 0;
   case (op)
      X_SH1ADD:  r = (a << 1) + b;
      X_SH2ADD:  r = (a << 2) + b;
      X_SH3ADD:  r = (a << 3) + b;
      X_ANDN:    r = a & (~ b);
      X_ORN:     r = a | (~ b);
      X_XNOR:    r = ~ (a ^ b);
      X_MAX:     r = (signedLT (a, b) ? b : a);
      X_MAXU:    r = ((a < b) ? b : a);
      X_MIN:     r = (signedLT (a, b) ? a : b);
      X_MINU:    r = ((a < b) ? a : b);
      X_ROL:     r = fn_rol_var (a, shamt_r);
      X_ROR:     r = fn_ror_var (a, shamt_r);
      X_RORI:    r = fn_ror_var (a, shamt_i);
      X_SEXTB:   r = signExtend (a [7:0]);
      X_SEXTH:   r = signExtend (a [15:0]);
      X_ZEXTH:   r = zeroExtend (a [15:0]);
      X_ORCB:    for (Integer i = 0; i < 32; i = i + 8) begin
		    Bit #(8) byte_i = a [i+7:i];
		    r [i+7:i] = ((byte_i == 0) ? 8'h00 : 8'hff);
		 end
      X_REV8:    for (Integer i = 0; i < 32; i = i + 8) begin
		    Bit #(8) byte_i = a [31-i:24-i];
		    r [i+7:i] = byte_i;
		 end
      X_BCLR, X_BCLRI: r = a & (~ bmask);
      X_BEXT, X_BEXTI: r = zeroExtend (pack ((a & bmask) != 0));
      X_BINV, X_BINVI: r = a ^ bmask;
      X_BSET, X_BSETI: r = a | bmask;
      X_PACK:    r = {b [15:0], a [15:0]};
      X_PACKH:   r = {16'b0, b [7:0], a [7:0]};
      X_BREV8:   for (Integer i = 0; i < 32; i = i + 1) r [i] = a [(i - (i % 8)) + (7 - (i % 8))];
      X_ZIP:     for (Integer i = 0; i < 16; i = i + 1) begin r [2*i] = a [i]; r [2*i+1] = a [i+16]; end
      X_UNZIP:   for (Integer i = 0; i < 16; i = i + 1) begin r [i] = a [2*i]; r [i+16] = a [2*i+1]; end
      X_XPERM8:  for (Integer i = 0; i < 32; i = i + 8) begin
		    Bit #(8) idx = b [i+7:i];
		    Bit #(32) sel = a >> {idx [1:0], 3'b000};
		    r [i+7:i] = ((idx < 4) ? sel [7:0] : 0);
		 end
      X_XPERM4:  for (Integer i = 0; i < 32; i = i + 4) begin
		    Bit #(4) idx = b [i+3:i];
		    Bit #(32) sel = a >> {idx [2:0], 2'b00};
		    r [i+3:i] = ((idx < 8) ? sel [3:0] : 0);
		 end
      X_SHA256SUM0:  r = fn_ror32 (a, 2) ^ fn_ror32 (a, 13) ^ fn_ror32 (a, 22);
      X_SHA256SUM1:  r = fn_ror32 (a, 6) ^ fn_ror32 (a, 11) ^ fn_ror32 (a, 25);
      X_SHA256SIG0:  r = fn_ror32 (a, 7) ^ fn_ror32 (a, 18) ^ (a >> 3);
      X_SHA256SIG1:  r = fn_ror32 (a, 17) ^ fn_ror32 (a, 19) ^ (a >> 10);
      X_SHA512SUM0R: r = (a << 25) ^ (a << 30) ^ (a >> 28) ^ (b >> 7) ^ (b >> 2) ^ (b << 4);
      X_SHA512SUM1R: r = (a << 23) ^ (a >> 14) ^ (a >> 18) ^ (b >> 9) ^ (b << 18) ^ (b << 14);
      X_SHA512SIG0L: r = (a >> 1) ^ (a >> 7) ^ (a >> 8) ^ (b << 31) ^ (b << 25) ^ (b << 24);
      X_SHA512SIG0H: r = (a >> 1) ^ (a >> 7) ^ (a >> 8) ^ (b << 31) ^ (b << 24);
      X_SHA512SIG1L: r = (a << 3) ^ (a >> 6) ^ (a >> 19) ^ (b >> 29) ^ (b << 26) ^ (b << 13);
      X_SHA512SIG1H: r = (a << 3) ^ (a >> 6) ^ (a >> 19) ^ (b >> 29) ^ (b << 13);
   endcase
   return r;
endfunction

// ================================================================

interface RISCV_XBox_IFC;
   // Stage2 entry: the instruction (already known to decode, fv_xbox_decode) and rs1 / rs2
   (* always_ready *)
   method Action req (Instr instr, WordXL v1, WordXL v2);

   (* always_ready *)   method Bool    valid;
   (* always_ready *)   method WordXL  word;
endinterface

(* synthesize *)
module mkRISCV_XBox (RISCV_XBox_IFC);
   // 0: idle, 1: operands registered (stage A), 2: stage B, 3: result valid
   // ConfigRegs: valid/word (read by stage2 output) and req (stage2 enq) are called from the
   // same CPU rule; plain regs would order valid/word < rl_compute < req (SBR, G0004).
   Reg #(Bit #(2))  rg_state  <- mkConfigReg (0);
   Reg #(XOp)       rg_op     <- mkRegU;
   Reg #(Instr)     rg_instr  <- mkRegU;
   Reg #(WordXL)    rg_v1     <- mkRegU;
   Reg #(WordXL)    rg_v2     <- mkRegU;
   Reg #(WordXL)    rg_result <- mkConfigRegU;

   Reg #(WordXL)    rg_part   <- mkRegU;
   Reg #(Bit #(64)) rg_clp    <- mkRegU;
   Reg #(Bit #(8))  rg_so     <- mkRegU;
   Reg #(Bit #(6))  rg_cnt    <- mkRegU;

   rule rl_compute (rg_state == 1);
      rg_part  <= fv_xbox_compute_main (rg_op, rg_instr, rg_v1, rg_v2);
      rg_clp   <= fv_xbox_clp (rg_v1, rg_v2);
      rg_so    <= fv_xbox_aes_so (rg_op, rg_instr, rg_v2);
      rg_cnt   <= fv_xbox_cnt (rg_op, rg_v1);
      rg_state <= 2;
   endrule

   rule rl_finish (rg_state == 2);
      rg_result <= fv_xbox_finish (rg_op, rg_instr, rg_v1, rg_part, rg_clp, rg_so, rg_cnt);
      rg_state  <= 3;
   endrule

   method Action req (Instr instr, WordXL v1, WordXL v2);
      rg_op    <= fromMaybe (?, fv_xbox_decode (instr));
      rg_instr <= instr;
      rg_v1    <= v1;
      rg_v2    <= v2;
      rg_state <= 1;
   endmethod

   method Bool   valid = (rg_state == 3);
   method WordXL word  = rg_result;
endmodule

endpackage
