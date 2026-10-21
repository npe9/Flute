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

// ================================================================
// Compute (one cycle, from registered operands)

function WordXL fv_xbox_compute (XOp op, Instr instr, WordXL a, WordXL b);
   Bit #(5) shamt_i = instr [24:20];
   Bit #(5) shamt_r = b [4:0];
   Bit #(5) bidx    = ((op == X_BCLRI) || (op == X_BEXTI) || (op == X_BINVI) || (op == X_BSETI)) ? shamt_i : shamt_r;
   Bit #(32) bmask  = (32'h1 << bidx);

   // clmul family (Sail RISCV_CLMUL / CLMULH / CLMULR): xor of shifted copies of a
   Bit #(64) clp = 0;
   for (Integer i = 0; i < 32; i = i + 1)
      if (b [i] == 1'b1) clp = clp ^ (zeroExtend (a) << i);

   // AES32 (Sail AES32ESI/ESMI/DSI/DSMI)
   Bit #(5)  aes_sh  = {instr [31:30], 3'b000};
   Bit #(8)  si      = truncate (b >> aes_sh);
   Bool      aes_fwd = ((op == X_AES32ESI) || (op == X_AES32ESMI));
   Bit #(8)  so      = (aes_fwd ? fn_aes_sbox_fwd (si) : fn_aes_sbox_inv (si));
   Bit #(32) mixed   = case (op)
			  X_AES32ESMI: {fn_gfmul (so, 4'h3), so, so, fn_gfmul (so, 4'h2)};
			  X_AES32DSMI: {fn_gfmul (so, 4'hb), fn_gfmul (so, 4'hd), fn_gfmul (so, 4'h9), fn_gfmul (so, 4'he)};
			  default:     zeroExtend (so);
		       endcase;

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
      X_CLZ:     r = zeroExtend (pack (countZerosMSB (a)));
      X_CTZ:     r = zeroExtend (pack (countZerosLSB (a)));
      X_CPOP:    r = zeroExtend (pack (countOnes (a)));
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
      X_CLMUL:   r = clp [31:0];
      X_CLMULH:  r = clp [63:32];
      X_CLMULR:  r = clp [62:31];
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
      X_AES32ESI, X_AES32ESMI, X_AES32DSI, X_AES32DSMI:
		 r = a ^ fn_rol_var (mixed, aes_sh);
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
   // 0: idle, 1: operands registered (compute this cycle), 2: result valid
   // ConfigRegs: valid/word (read by stage2 output) and req (stage2 enq) are called from the
   // same CPU rule; plain regs would order valid/word < rl_compute < req (SBR, G0004).
   Reg #(Bit #(2))  rg_state  <- mkConfigReg (0);
   Reg #(XOp)       rg_op     <- mkRegU;
   Reg #(Instr)     rg_instr  <- mkRegU;
   Reg #(WordXL)    rg_v1     <- mkRegU;
   Reg #(WordXL)    rg_v2     <- mkRegU;
   Reg #(WordXL)    rg_result <- mkConfigRegU;

   rule rl_compute (rg_state == 1);
      rg_result <= fv_xbox_compute (rg_op, rg_instr, rg_v1, rg_v2);
      rg_state  <= 2;
   endrule

   method Action req (Instr instr, WordXL v1, WordXL v2);
      rg_op    <= fromMaybe (?, fv_xbox_decode (instr));
      rg_instr <= instr;
      rg_v1    <= v1;
      rg_v2    <= v2;
      rg_state <= 1;
   endmethod

   method Bool   valid = (rg_state == 2);
   method WordXL word  = rg_result;
endmodule

endpackage
