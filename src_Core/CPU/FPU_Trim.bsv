// Copyright (c) 2026 Noah Evans.  Area-trimmed F + D floating point unit for Flute RV32.

package FPU_Trim;

// ================================================================
// Drop-in replacement for mkFBox_Top (same FBox_Top_IFC), selected in CPU_Stage2 by FPU_TRIM.
//
// One multi-cycle datapath shared by every arithmetic op and both precisions:
//
//   S, X     108-bit significand registers.  A value is S * 2^(eS - 107): bit 107 has weight 2^eS.
//            Operands are unpacked to a 53-bit significand at S[107:55] (f32: 24 bits at S[107:84]).
//   shifter  ONE right shifter with sticky (jam into bit 0), on S or X.  Left shifts (normalize)
//            go through the same shifter with the operand and result bit-reversed.
//   LZC      on the shifter operand; normalize = NORM1 (count, registered) + NORM2 (shift).
//   adder    ONE 109-bit add/subtract S +- X, result >> 1 with jam (eS + 1).
//   mul      54 x 18 bits per cycle on 3 DSPs; the 106-bit product accumulates in S (f64: 3 chunks,
//            f32: 2 chunks), one pipeline register between the DSPs and the accumulator.
//   div/sqrt shared radix-2 restoring recurrence (R, D, Q registers), 55 (f64) / 26 (f32) cycles.
//   round    RND0: tininess (after rounding) + denormalize shift; RND1: increment at 24 or 53 bits;
//            PACK: exponent, overflow, flags.  FCVT to integer rounds at the 53-bit position after
//            shifting the units bit there (F2I state) and packs in IPACK.
//
// Sign injection, min/max, compares, class and moves are combinational from the request registers
// (LOAD state).  FMA is fused: the exact product is normalized and added to the addend in the
// 108-bit window (larger operand unshifted, smaller one shifted with jam), then rounded once.
//
// Conventions (CPU_Stage2): f32 operands NaN-boxed (an improperly boxed f32 is the canonical NaN),
// f32 results boxed, integer sources in [31:0], rm already resolved (DYN) by Stage1.
// Results are held in registers (valid / word are plain register reads) until the next req.
// ================================================================

import FIFOF        :: *;
import GetPut       :: *;
import ClientServer :: *;

import Vector       :: *;

import GetPut_Aux :: *;

import ISA_Decls :: *;
import FBox_Top  :: *;    // FBox_Top_IFC

// ================================================================

typedef Bit #(108) FT_Sig;
typedef Int #(14)  FT_Exp;

typedef enum {
   FT_IDLE, FT_LOAD, FT_MUL, FT_NORM1, FT_NORM2, FT_ALIGN0, FT_ALIGN, FT_ADD,
   FT_DIVINIT, FT_DIVITER, FT_DIVEND, FT_RND0, FT_DENORM, FT_RND1, FT_PACK, FT_F2I, FT_IPACK
} FT_State deriving (Bits, Eq, FShow);

typedef struct {
   Bit #(1)  sign;
   FT_Exp    e;       // unbiased; subnormals: emin (significand not normalized)
   Bit #(53) sig;     // hidden bit at [52]; f32: 24 bits at [52:29]
   Bool      zero;
   Bool      sub;
   Bool      inf;
   Bool      nan;
   Bool      snan;
   Bit #(64) raw;     // f64 value, or f32 value in [31:0] (canonical NaN if not NaN-boxed)
} FT_Unp deriving (Bits);

function FT_Unp fn_unpack (Bool dbl, Bit #(64) v);
   Bit #(32) s32  = ((v [63:32] == '1) ? v [31:0] : 32'h7fc00000);
   Bit #(11) ef   = (dbl ? v [62:52] : zeroExtend (s32 [30:23]));
   Bit #(52) fr   = (dbl ? v [51:0]  : { s32 [22:0], 29'b0 });
   Bool      ez   = (ef == 0);
   Bool      emx  = (dbl ? (v [62:52] == '1) : (s32 [30:23] == '1));
   Bool      fz   = (fr == 0);
   FT_Exp    eu   = unpack (zeroExtend (ef));
   FT_Exp    bias = (dbl ? 1023 : 127);
   return FT_Unp {sign: (dbl ? v [63] : s32 [31]),
                  e:    (ez ? (1 - bias) : (eu - bias)),
                  sig:  { pack (! ez), fr },
                  zero: ez && fz,
                  sub:  ez && (! fz),
                  inf:  emx && fz,
                  nan:  emx && (! fz),
                  snan: emx && (! fz) && (fr [51] == 0),
                  raw:  (dbl ? v : zeroExtend (s32))};
endfunction

function Bit #(64) fn_box (Bool dbl, Bit #(64) x) = (dbl ? x : { 32'hffffffff, x [31:0] });

// x >> amt, with every bit shifted out ORed into bit 0 (jam)
function FT_Sig fn_shr_jam (FT_Sig x, Bit #(7) amt);
   Bit #(1) st = 0;
   for (Integer k = 0; k < 7; k = k + 1) begin
      Integer n = 2 ** k;
      FT_Sig mask = fromInteger (2 ** n - 1);
      if (amt [k] == 1) begin
         if ((x & mask) != 0) st = 1;
         x = x >> n;
      end
   end
   return { x [107:1], x [0] | st };
endfunction

// leading-zero count, log-depth tree (merge = concatenation, no adders); 0 -> 108
function Bit #(7) fn_lzc (FT_Sig s);
   Bit #(128) x = { s, 20'h80000 };
   Vector #(128, Bool)     z = replicate (False);
   Vector #(128, Bit #(7)) c = replicate (0);
   for (Integer i = 0; i < 128; i = i + 1) z [i] = (x [127 - i] == 0);
   for (Integer lvl = 0; lvl < 7; lvl = lvl + 1) begin
      Integer n = 64 / (2 ** lvl);
      for (Integer i = 0; i < n; i = i + 1) begin
         Bool zl = z [2 * i];
         c [i] = (zl ? (fromInteger (2 ** lvl) | c [2 * i + 1]) : c [2 * i]);
         z [i] = zl && z [2 * i + 1];
      end
   end
   return c [0];
endfunction

// shift amount clamped to 127 (>= 108 shifts everything into the jam bit); x >= 0
function Bit #(7) fn_clamp (FT_Exp x) = ((x > 127) ? 7'h7f : truncate (pack (x)));

// round-increment decision (rm: 0 RNE, 1 RTZ, 2 RDN, 3 RUP, 4 RMM)
function Bool fn_rinc (Bit #(3) rm, Bit #(1) sign, Bit #(1) lsb, Bit #(1) g, Bool st);
   Bool gb  = (g == 1);
   Bool any = gb || st;
   case (rm)
      3'b000:  return gb && (st || (lsb == 1));
      3'b010:  return (sign == 1) && any;
      3'b011:  return (sign == 0) && any;
      3'b100:  return gb;
      default: return False;
   endcase
endfunction

// ================================================================

(* synthesize *)
module mkFPU_Trim #(Bit #(4) verbosity) (FBox_Top_IFC);

   FIFOF #(Token) f_reset_reqs <- mkFIFOF;
   FIFOF #(Token) f_reset_rsps <- mkFIFOF;

   RWire #(Tuple7 #(Opcode, Bit #(7), Bit #(3), RegName, Bit #(64), Bit #(64), Bit #(64))) rw_req <- mkRWire;

   // ---- request, held for the whole operation
   Reg #(Opcode)    rg_opc <- mkRegU;
   Reg #(Bit #(7))  rg_f7  <- mkRegU;
   Reg #(Bit #(3))  rg_rm  <- mkRegU;
   Reg #(RegName)   rg_rs2 <- mkRegU;
   Reg #(Bit #(64)) rg_v1  <- mkRegU;
   Reg #(Bit #(64)) rg_v2  <- mkRegU;
   Reg #(Bit #(64)) rg_v3  <- mkRegU;

   // ---- response
   Reg #(FT_State)  rg_state <- mkReg (FT_IDLE);
   Reg #(Bool)      rg_valid <- mkReg (False);
   Reg #(Bit #(64)) rg_res   <- mkRegU;
   Reg #(Bit #(5))  rg_fl    <- mkRegU;

   // ---- control
   Reg #(Bool)      rg_dbl   <- mkRegU;    // FP result is f64
   Reg #(Bool)      rg_p53   <- mkRegU;    // round at the 53-bit position (f64 result or FCVT to int)
   Reg #(Bit #(1))  rg_sign  <- mkRegU;    // result sign (product sign for add)
   Reg #(Bit #(1))  rg_signc <- mkRegU;    // addend sign
   Reg #(Bool)      rg_sub   <- mkRegU;    // effective subtraction
   Reg #(Bool)      rg_xbig  <- mkRegU;    // add: X is the larger operand
   Reg #(Bool)      rg_pns   <- mkRegU;    // pending: normalize S
   Reg #(Bool)      rg_pnx   <- mkRegU;    // pending: normalize X
   Reg #(Bool)      rg_pmul  <- mkRegU;    // pending: multiply
   Reg #(Bool)      rg_padd  <- mkRegU;    // pending: align + add
   Reg #(Bool)      rg_pdiv  <- mkRegU;    // pending: div / sqrt
   Reg #(Bool)      rg_sqrt  <- mkRegU;
   Reg #(Bool)      rg_f2i   <- mkRegU;    // FCVT to integer
   Reg #(Bool)      rg_uns   <- mkRegU;    // FCVT.WU
   Reg #(Bool)      rg_ppv   <- mkRegU;    // rg_pp holds a partial product
   Reg #(Bit #(6))  rg_cnt   <- mkRegU;
   Reg #(Bool)      rg_tiny  <- mkRegU;
   Reg #(Bool)      rg_nx    <- mkRegU;

   // ---- datapath
   Reg #(FT_Sig)    rg_s    <- mkRegU;
   Reg #(FT_Sig)    rg_x    <- mkRegU;
   Reg #(FT_Exp)    rg_es   <- mkRegU;
   Reg #(FT_Exp)    rg_ex   <- mkRegU;
   Reg #(Bit #(7))  rg_amt  <- mkRegU;    // shifter amount (always registered)
   Reg #(Bool)      rg_xneg <- mkRegU;    // ALIGN: X has the larger exponent, shift S
   Reg #(Bit #(72)) rg_pp   <- mkRegU;
   Reg #(Bit #(57)) rg_r    <- mkRegU;
   Reg #(Bit #(54)) rg_d    <- mkRegU;
   Reg #(Bit #(55)) rg_q    <- mkRegU;
   Reg #(Bit #(54)) rg_rsig <- mkRegU;

   // =============================================================
   // Decode (request registers)

   Bit #(5) f5     = rg_f7 [6:2];
   Bool op_fp      = (rg_opc == op_FP);
   Bool is_fma     = (rg_opc [6:4] == 3'b100);
   Bool is_add     = op_fp && ((f5 == 5'b00000) || (f5 == 5'b00001));
   Bool is_subop   = (f5 == 5'b00001);
   Bool is_mul     = op_fp && (f5 == 5'b00010);
   Bool is_div     = op_fp && (f5 == 5'b00011);
   Bool is_sqrt    = op_fp && (f5 == 5'b01011);
   Bool is_sgnj    = op_fp && (f5 == 5'b00100);
   Bool is_minmax  = op_fp && (f5 == 5'b00101);
   Bool is_cvtff   = op_fp && (f5 == 5'b01000);
   Bool is_cmp     = op_fp && (f5 == 5'b10100);
   Bool is_f2i     = op_fp && (f5 == 5'b11000);
   Bool is_i2f     = op_fp && (f5 == 5'b11010);
   Bool is_mvx     = op_fp && (f5 == 5'b11100);    // FMV.X.W (rm 0) / FCLASS (rm 1)
   Bool is_mvw     = op_fp && (f5 == 5'b11110);    // FMV.W.X

   Bool in_dbl     = (is_cvtff ? (rg_rs2 [0] == 1) : (rg_f7 [0] == 1));
   Bool out_dbl    = (rg_f7 [0] == 1);

   FT_Unp ua = fn_unpack (in_dbl, rg_v1);
   FT_Unp ub = fn_unpack (in_dbl, rg_v2);
   FT_Unp ux = fn_unpack (in_dbl, (is_fma ? rg_v3 : rg_v2));   // addend (FMA) or rs2

   FT_Exp emin = (rg_dbl ? -1022 : -126);

   // =============================================================
   // Shared datapath

   // ---- shifter (right with jam; left via bit reversal)
   // all shifter controls are registers (amount set in the previous state)
   FT_Exp ediff  = rg_es - rg_ex;
   FT_Exp ediffn = rg_ex - rg_es;

   Bool sh_selx = ((((rg_state == FT_NORM1) || (rg_state == FT_NORM2)) && (! rg_pns))
                   || ((rg_state == FT_ALIGN) && (! rg_xneg)));
   Bool sh_left = (rg_state == FT_NORM2);
   Bit #(7) sh_amt = rg_amt;
   FT_Sig sh_src = (sh_selx ? rg_x : rg_s);
   FT_Sig sh_in  = (sh_left ? reverseBits (sh_src) : sh_src);
   FT_Sig sh_o   = fn_shr_jam (sh_in, sh_amt);
   FT_Sig sh_out = (sh_left ? reverseBits (sh_o) : sh_o);

   Bit #(7) lzc = fn_lzc (sh_src);

   // ---- adder: S + X, S - X or X - S, result >> 1 with jam
   FT_Sig    ad_a = ((rg_sub && rg_xbig)    ? (~ rg_s) : rg_s);
   FT_Sig    ad_b = ((rg_sub && (! rg_xbig)) ? (~ rg_x) : rg_x);
   Bit #(109) ad_t = zeroExtend (ad_a) + zeroExtend (ad_b) + (rg_sub ? 1 : 0);
   Bool      ad_neg = rg_sub && (ad_t [108] == 0);
   Bit #(109) ad_r = (rg_sub ? { 1'b0, ad_t [107:0] } : ad_t);
   FT_Sig    ad_out = { ad_r [108:2], ad_r [1] | ad_r [0] };

   // ---- multiplier: 54 x 18 per cycle (3 DSPs), product accumulated in S
   Bit #(54) m_b    = zeroExtend (ub.sig);
   Bit #(18) m_chunk = case (rg_cnt [1:0])
                          2'd0:    m_b [17:0];
                          2'd1:    m_b [35:18];
                          default: m_b [53:36];
                       endcase;
   UInt #(54) m_au  = unpack (zeroExtend (ua.sig));
   UInt #(18) m_bu  = unpack (m_chunk);
   Bit #(72) m_pp   = pack (unsignedMul (m_au, m_bu));
   Bit #(72) m_acc  = zeroExtend (rg_s [107:54]) + rg_pp;
   FT_Sig    mul_out = { m_acc, rg_s [53:18] };

   // ---- div / sqrt recurrence
   Bit #(59) dv_rn = (rg_sqrt ? { rg_r, rg_d [53:52] } : { 1'b0, rg_r, 1'b0 });
   Bit #(59) dv_y  = (rg_sqrt ? zeroExtend ({ rg_q, 2'b01 }) : zeroExtend ({ rg_d, 1'b0 }));
   Bit #(60) dv_t  = zeroExtend (dv_rn) - zeroExtend (dv_y);
   Bool      dv_ge = (dv_t [59] == 0);

   // ---- rounding (on S): 24 or 53 bits
   Bit #(1) rn_lsb = (rg_p53 ? rg_s [55] : rg_s [84]);
   Bit #(1) rn_g   = (rg_p53 ? rg_s [54] : rg_s [83]);
   Bool     st53   = (rg_s [53:0] != 0);
   Bool     rn_st  = (rg_p53 ? st53 : (st53 || (rg_s [82:54] != 0)));
   Bool     rn_inc = fn_rinc (rg_rm, rg_sign, rn_lsb, rn_g, rn_st);
   Bool     rn_ones = (rg_p53 ? (rg_s [107:55] == '1) : (rg_s [107:84] == '1));
   Bit #(54) rn_sum = zeroExtend (rg_s [107:55]) + (rn_inc ? (rg_p53 ? 54'h1 : 54'h20000000) : 0);

   // ---- next step after a phase
   function FT_State fn_next (Bool pns, Bool pnx, Bool pmul, Bool padd, Bool pdiv, Bool f2i);
      if (pns || pnx) return FT_NORM1;
      else if (pmul)  return FT_MUL;
      else if (padd)  return FT_ALIGN0;
      else if (pdiv)  return FT_DIVINIT;
      else if (f2i)   return FT_F2I;
      else            return FT_RND0;
   endfunction

   function Action fa_done (Bit #(64) res, Bit #(5) fl);
      action
         rg_res   <= res;
         rg_fl    <= fl;
         rg_valid <= True;
         rg_state <= FT_IDLE;
      endaction
   endfunction

   // =============================================================
   // LOAD: quick ops, special operands, datapath setup

   Bit #(64) cnan = (out_dbl ? 64'h7ff8000000000000 : 64'hffffffff7fc00000);
   function Bit #(64) fn_inf (Bit #(1) s)  = (out_dbl ? { s, 11'h7ff, 52'b0 } : { 32'hffffffff, s, 8'hff, 23'b0 });
   function Bit #(64) fn_zero (Bit #(1) s) = (out_dbl ? { s, 63'b0 } : { 32'hffffffff, s, 31'b0 });
   function Bit #(64) fn_sgn (Bit #(1) s, Bit #(64) raw)
      = (out_dbl ? { s, raw [62:0] } : { 32'hffffffff, s, raw [30:0] });

   // compares
   Bit #(63) mag_a = (in_dbl ? ua.raw [62:0] : zeroExtend (ua.raw [30:0]));
   Bit #(63) mag_b = (in_dbl ? ub.raw [62:0] : zeroExtend (ub.raw [30:0]));
   Bool m_lt   = (mag_a < mag_b);
   Bool m_eq   = (mag_a == mag_b);
   Bool both_z = ua.zero && ub.zero;
   Bool lt_tot = ((ua.sign != ub.sign) ? (ua.sign == 1)
                                       : ((ua.sign == 1) ? ((! m_lt) && (! m_eq)) : m_lt));
   Bool eq_ieee = (m_eq && (ua.sign == ub.sign)) || both_z;
   Bool lt_ieee = lt_tot && (! both_z);
   Bool any_nan2  = ua.nan || ub.nan;
   Bool any_snan2 = ua.snan || ub.snan;

   function Bit #(10) fn_class (FT_Unp u);
      Bool normal = ! (u.zero || u.sub || u.inf || u.nan);
      Bool neg = (u.sign == 1);
      Bit #(10) c = 0;
      if (u.nan)            c = (u.snan ? 10'h100 : 10'h200);
      else if (u.inf)       c = (neg ? 10'h001 : 10'h080);
      else if (u.zero)      c = (neg ? 10'h008 : 10'h010);
      else if (u.sub)       c = (neg ? 10'h004 : 10'h020);
      else                  c = (neg ? 10'h002 : 10'h040);
      return c;
   endfunction

   // product / addend view for FMA, FMUL, FADD
   Bit #(1) neg_p = ((is_fma && (rg_opc [3] == 1)) ? 1 : 0);
   Bit #(1) neg_c = (((is_fma && (rg_opc [2] == 1)) || (is_add && is_subop)) ? 1 : 0);
   Bit #(1) sp    = (is_add ? ua.sign : (ua.sign ^ ub.sign ^ neg_p));
   Bit #(1) sc    = ux.sign ^ neg_c;
   Bool p_nan  = ua.nan  || ((! is_add) && ub.nan);
   Bool p_inf  = ua.inf  || ((! is_add) && ub.inf);
   Bool p_zero = ua.zero || ((! is_add) && ub.zero);
   Bool p_inv  = (! is_add) && ((ua.inf && ub.zero) || (ua.zero && ub.inf));
   Bool has_c  = is_fma || is_add;
   Bool fma_snan = ua.snan || ((! is_add) && ub.snan) || (has_c && ux.snan);

   Bit #(32) i_val = rg_v1 [31:0];
   Bool      i_neg = (rg_rs2 [0] == 0) && (i_val [31] == 1);
   Bit #(32) i_mag = (i_neg ? (- i_val) : i_val);

   Bit #(64) f2i_sat_pos = ((rg_rs2 [0] == 1) ? 64'hffffffffffffffff : 64'h7fffffff);
   Bit #(64) f2i_sat_neg = ((rg_rs2 [0] == 1) ? 0 : 64'hffffffff80000000);

   // =============================================================
   // RVFI_ALTOPS (riscv-formal-fp, formal builds only): arithmetic and FCVT ops return the stand-in
   // functions of riscv-formal-fp insns/generate.py (same table and constants as FBox_Core).
`ifdef RVFI_ALTOPS
   function Bit #(32) fn_rotl32 (Bit #(32) x, Integer n) = ((x << n) | (x >> (32 - n)));
   function Bit #(64) fn_rotl64 (Bit #(64) x, Integer n) = ((x << n) | (x >> (64 - n)));
   function Bit #(32) fn_unbox32 (Bit #(64) v) = ((v [63:32] == '1) ? v [31:0] : 32'h7fc00000);
   Bit #(32) alt_a = fn_unbox32 (rg_v1);
   Bit #(32) alt_b = fn_unbox32 (rg_v2);
   Bit #(32) alt_c = fn_unbox32 (rg_v3);
   Bit #(64) alt_da = rg_v1;
   Bit #(64) alt_db = rg_v2;
   Bit #(64) alt_dc = rg_v3;
   Bit #(32) alt_x  = rg_v1 [31:0];
   Bit #(32) alt_rm32 = zeroExtend (rg_rm);
   Bit #(64) alt_rm64 = zeroExtend (rg_rm);
`ifdef RVFI_MUTANT_FSUB_SWAP
   Bit #(32) alt_fsub_s_e = alt_b - fn_rotl32 (alt_a, 1);
`else
   Bit #(32) alt_fsub_s_e = alt_a - fn_rotl32 (alt_b, 1);
`endif
   Bit #(32) alt_ab_s = alt_a + fn_rotl32 (alt_b, 1);
   Bit #(64) alt_ab_d = alt_da + fn_rotl64 (alt_db, 1);
   Bit #(32) alt_s = ?;
   Bit #(64) alt_d = ?;
   Bool      alt_hit = True;
   Bool      alt_dres = out_dbl;       // result is an f64 (else f32 boxed, or integer)
   Bool      alt_ires = False;         // integer result (sign-extended 32 bits)
   Bit #(2)  fmav = rg_opc [3:2];      // FMADD FMSUB FNMSUB FNMADD
   if (is_fma && (! out_dbl))
      alt_s = case (fmav)
                 2'b00: ((alt_ab_s + fn_rotl32 (alt_c, 2)) ^ 32'hd6b3f97 ^ alt_rm32);
                 2'b01: ((alt_ab_s - fn_rotl32 (alt_c, 2)) ^ 32'h48c27a1d ^ alt_rm32);
                 2'b10: (((~ alt_ab_s) + fn_rotl32 (alt_c, 2)) ^ 32'h31f9e063 ^ alt_rm32);
                 2'b11: (((~ alt_ab_s) - fn_rotl32 (alt_c, 2)) ^ 32'h6a84d2fb ^ alt_rm32);
              endcase;
   else if (is_fma)
      alt_d = case (fmav)
                 2'b00: ((alt_ab_d + fn_rotl64 (alt_dc, 2)) ^ 64'hd6b3f9752ac8e04 ^ alt_rm64);
                 2'b01: ((alt_ab_d - fn_rotl64 (alt_dc, 2)) ^ 64'h48c27a1db93f6052 ^ alt_rm64);
                 2'b10: (((~ alt_ab_d) + fn_rotl64 (alt_dc, 2)) ^ 64'h31f9e0634d8a1bc7 ^ alt_rm64);
                 2'b11: (((~ alt_ab_d) - fn_rotl64 (alt_dc, 2)) ^ 64'h6a84d2fb0c57e913 ^ alt_rm64);
              endcase;
   else if (is_add && (! is_subop))
      begin alt_s = alt_ab_s ^ 32'h2c1e5a07 ^ alt_rm32;
            alt_d = alt_ab_d ^ 64'h2c1e5a07d4c3b2a1 ^ alt_rm64; end
   else if (is_add)
      begin alt_s = alt_fsub_s_e ^ 32'h6f3d1b42 ^ alt_rm32;
            alt_d = (alt_da - fn_rotl64 (alt_db, 1)) ^ 64'h6f3d1b4291e8f705 ^ alt_rm64; end
   else if (is_mul)
      begin alt_s = (alt_a ^ fn_rotl32 (alt_b, 3)) ^ 32'h13a5c6e9 ^ alt_rm32;
            alt_d = (alt_da ^ fn_rotl64 (alt_db, 3)) ^ 64'h13a5c6e97b2d4e18 ^ alt_rm64; end
   else if (is_div)
      begin alt_s = (alt_a - fn_rotl32 (alt_b, 5)) ^ 32'h5b0f8d31 ^ alt_rm32;
            alt_d = (alt_da - fn_rotl64 (alt_db, 5)) ^ 64'h5b0f8d3106f1a9c3 ^ alt_rm64; end
   else if (is_sqrt)
      begin alt_s = fn_rotl32 (alt_a, 7) ^ 32'h7e29b4c5 ^ alt_rm32;
            alt_d = fn_rotl64 (alt_da, 7) ^ 64'h7e29b4c5e8173d6b ^ alt_rm64; end
   else if (is_f2i) begin
      alt_ires = True;
      Bit #(32) dx = alt_da [31:0] ^ alt_da [63:32];
      alt_s = (out_dbl ? ((rg_rs2 [0] == 0) ? (dx ^ 32'h3d2f75e1) : (dx ^ 32'h60a4c83f))
                       : ((rg_rs2 [0] == 0) ? (alt_a ^ 32'h1b7e3c55) : (alt_a ^ 32'h26d1a0c9))) ^ alt_rm32;
   end
   else if (is_i2f)
      begin alt_s = ((rg_rs2 [0] == 0) ? (alt_x ^ 32'h4f5a9e13) : (alt_x ^ 32'h573c08ad)) ^ alt_rm32;
            alt_d = ((rg_rs2 [0] == 0) ? ({ alt_x, alt_x } ^ 64'h4f5a9e13a8c6205d)
                                       : ({ alt_x, alt_x } ^ 64'h573c08ad1f94b6e2)) ^ alt_rm64; end
   else if (is_cvtff)
      begin alt_s = (alt_da [31:0] ^ alt_da [63:32]) ^ 32'he9b6271 ^ alt_rm32;
            alt_d = { alt_a, alt_a } ^ 64'he9b627195da43f8 ^ alt_rm64; end
   else
      alt_hit = False;
   Bit #(64) alt_value = (alt_ires ? signExtend (alt_s)
                                   : (alt_dres ? alt_d : { 32'hffffffff, alt_s }));
   Bit #(5)  alt_flags = (alt_ires ? alt_s [9:5] : (alt_dres ? alt_d [9:5] : alt_s [9:5]));
`endif

   // =============================================================
   // BEHAVIOR

   rule rl_reset;
      f_reset_reqs.deq;
      rg_state <= FT_IDLE;
      rg_valid <= False;
      f_reset_rsps.enq (?);
   endrule

   rule rl_step;
      if (rw_req.wget matches tagged Valid {.opc, .f7, .rm, .rs2, .v1, .v2, .v3}) begin
         rg_opc <= opc; rg_f7 <= f7; rg_rm <= rm; rg_rs2 <= rs2;
         rg_v1 <= v1; rg_v2 <= v2; rg_v3 <= v3;
         rg_valid <= False;
         rg_state <= FT_LOAD;
      end
      else case (rg_state)

      // ----------------------------------------------------------------
      FT_LOAD: begin
         // quick results (done) or datapath setup (go)
         Bool      done = True;
         Bit #(64) res  = 0;
         Bit #(5)  fl   = 0;
         FT_Sig    s0   = { ua.sig, 55'b0 };
         FT_Exp    es0  = ua.e;
         Bit #(1)  sg   = ua.sign;
         Bool pns = False; Bool pnx = False; Bool pmul = False; Bool padd = False;
         Bool pdiv = False; Bool f2i = False;

         if (is_sgnj) begin
            Bit #(1) sa = ua.raw [in_dbl ? 63 : 31];
            Bit #(1) sb = ub.raw [in_dbl ? 63 : 31];
            Bit #(1) s  = case (rg_rm)
                             3'b000:  sb;
                             3'b001:  ~ sb;
                             default: sa ^ sb;
                          endcase;
            res = fn_sgn (s, ua.raw);
         end
         else if (is_minmax) begin
            Bool take_a = ((rg_rm [0] == 0) ? lt_tot : (! lt_tot));
            res = ((ua.nan && ub.nan) ? cnan
                   : (ua.nan ? fn_box (out_dbl, ub.raw)
                      : (ub.nan ? fn_box (out_dbl, ua.raw)
                         : fn_box (out_dbl, (take_a ? ua.raw : ub.raw)))));
            fl = (any_snan2 ? 5'h10 : 0);
         end
         else if (is_cmp) begin
            Bool r  = case (rg_rm)
                         3'b000:  (lt_ieee || eq_ieee);
                         3'b001:  lt_ieee;
                         default: eq_ieee;
                      endcase;
            Bool nv = ((rg_rm == 3'b010) ? any_snan2 : any_nan2);
            res = (((! any_nan2) && r) ? 1 : 0);
            fl  = (nv ? 5'h10 : 0);
         end
         else if (is_mvx)
            res = ((rg_rm [0] == 1) ? zeroExtend (fn_class (ua)) : signExtend (rg_v1 [31:0]));
         else if (is_mvw)
            res = { 32'hffffffff, rg_v1 [31:0] };

         // ---- FMA / FMUL / FADD / FSUB
         else if (is_fma || is_mul || is_add) begin
            if (p_nan || (has_c && ux.nan) || p_inv) begin
               res = cnan;
               fl  = ((fma_snan || p_inv) ? 5'h10 : 0);
            end
            else if (p_inf) begin
               if (has_c && ux.inf && (sp != sc)) begin res = cnan; fl = 5'h10; end
               else res = fn_inf (sp);
            end
            else if (has_c && ux.inf)
               res = fn_inf (sc);
            else if (p_zero) begin
               if ((! has_c) || ux.zero)
                  res = fn_zero (((! has_c) || (sp == sc)) ? sp : ((rg_rm == 3'b010) ? 1 : 0));
               else
                  res = fn_sgn (sc, ux.raw);
            end
            else begin
               done = False;
               sg   = sp;
               padd = has_c && (! ux.zero);
               pnx  = padd && ux.sub;
               if (is_add) pns = ua.sub;
               else begin
                  pmul = True;
                  s0   = 0;
                  es0  = ua.e + ub.e + 3;
               end
            end
         end

         // ---- FDIV
         else if (is_div) begin
            Bit #(1) s = ua.sign ^ ub.sign;
            if (any_nan2 || (ua.inf && ub.inf) || both_z) begin
               res = cnan;
               fl  = ((any_snan2 || (! any_nan2)) ? 5'h10 : 0);
            end
            else if (ua.inf)            res = fn_inf (s);
            else if (ub.zero) begin     res = fn_inf (s); fl = 5'h08; end
            else if (ua.zero || ub.inf) res = fn_zero (s);
            else begin
               done = False;
               sg   = s;
               pns  = ua.sub;
               pnx  = ub.sub;
               pdiv = True;
            end
         end

         // ---- FSQRT
         else if (is_sqrt) begin
            if (ua.nan)            begin res = cnan; fl = (ua.snan ? 5'h10 : 0); end
            else if (ua.zero)      res = fn_zero (ua.sign);
            else if (ua.sign == 1) begin res = cnan; fl = 5'h10; end
            else if (ua.inf)       res = fn_inf (0);
            else begin
               done = False;
               pns  = ua.sub;
               pdiv = True;
            end
         end

         // ---- FCVT.S.D / FCVT.D.S
         else if (is_cvtff) begin
            if (ua.nan)       begin res = cnan; fl = (ua.snan ? 5'h10 : 0); end
            else if (ua.inf)  res = fn_inf (ua.sign);
            else if (ua.zero) res = fn_zero (ua.sign);
            else begin
               done = False;
               pns  = ua.sub;
            end
         end

         // ---- FCVT.W[U].S/D
         else if (is_f2i) begin
            if (ua.nan) begin
               res = f2i_sat_pos; fl = 5'h10;
            end
            else if (ua.inf || (ua.e > 32)) begin
               res = ((ua.sign == 1) ? f2i_sat_neg : f2i_sat_pos); fl = 5'h10;
            end
            else if (! ua.zero) begin
               done = False;
               f2i  = True;
            end
         end

         // ---- FCVT.S/D.W[U]
         else if (is_i2f) begin
            if (i_val == 0) res = fn_zero (0);
            else begin
               done = False;
               s0   = { i_mag, 76'b0 };
               es0  = 31;
               sg   = pack (i_neg);
               pns  = True;
            end
         end
         // anything else is not an F/D op (Stage1 traps those): result 0

`ifdef RVFI_ALTOPS
         if (alt_hit) begin
            done = True;
            res  = alt_value;
            fl   = alt_flags;
         end
`endif

         rg_dbl   <= out_dbl;
         rg_p53   <= out_dbl || is_f2i;
         rg_sqrt  <= is_sqrt;
         rg_uns   <= (rg_rs2 [0] == 1);
         rg_ppv   <= False;
         rg_xbig  <= False;
         rg_tiny  <= False;
         rg_cnt   <= (in_dbl ? 0 : 1);       // first multiplier chunk (f32: chunk 0 of rs2 is zero)
         rg_s     <= s0;
         rg_es    <= es0;
         rg_x     <= { ux.sig, 55'b0 };
         rg_ex    <= ux.e;
         rg_sign  <= sg;
         rg_signc <= sc;
         rg_sub   <= (sp != sc);
         rg_pns   <= pns;
         rg_pnx   <= pnx;
         rg_pmul  <= pmul;
         rg_padd  <= padd;
         rg_pdiv  <= pdiv;
         rg_f2i   <= f2i;
         rg_amt   <= fn_clamp (52 - ua.e);     // FT_F2I: units bit to S[55]
         if (done) fa_done (res, fl);
         else      rg_state <= fn_next (pns, pnx, pmul, padd, pdiv, f2i);
      end

      // ----------------------------------------------------------------
      // multiply: chunk k of rs2 into the DSPs, previous partial product into S
      FT_MUL: begin
         if (rg_cnt [1:0] != 3) rg_pp <= m_pp;
         rg_ppv <= (rg_cnt [1:0] != 3);
         if (rg_ppv) rg_s <= mul_out;
         rg_cnt <= rg_cnt + 1;
         if (rg_cnt [1:0] == 3) begin
            rg_pmul  <= False;
            rg_pns   <= True;
            rg_state <= FT_NORM1;
         end
      end

      // ----------------------------------------------------------------
      // normalize S (pending first) or X
      FT_NORM1: begin
         if (rg_pns && (rg_s == 0))
            // exact zero from cancellation: +0, or -0 under RDN
            fa_done (fn_zero ((rg_rm == 3'b010) ? 1 : 0), 0);
         else begin
            rg_amt   <= lzc;
            rg_state <= FT_NORM2;
         end
      end

      FT_NORM2: begin
         FT_Exp lz = unpack (zeroExtend (rg_amt));
         Bool pns = rg_pns;
         Bool pnx = rg_pnx;
         if (rg_pns) begin
            rg_s  <= sh_out;
            rg_es <= rg_es - lz;
            pns = False;
         end
         else begin
            rg_x  <= sh_out;
            rg_ex <= rg_ex - lz;
            pnx = False;
         end
         rg_pns   <= pns;
         rg_pnx   <= pnx;
         rg_state <= fn_next (pns, pnx, rg_pmul, rg_padd, rg_pdiv, rg_f2i);
      end

      // ----------------------------------------------------------------
      // align: shift the operand with the smaller exponent
      FT_ALIGN0: begin
         Bool xneg = (ediff < 0);
         rg_xneg  <= xneg;
         rg_amt   <= (xneg ? fn_clamp (ediffn) : fn_clamp (ediff));
         rg_state <= FT_ALIGN;
      end

      FT_ALIGN: begin
         if (rg_xneg) begin
            rg_s    <= sh_out;
            rg_es   <= rg_ex;
            rg_xbig <= True;
         end
         else
            rg_x <= sh_out;
         rg_state <= FT_ADD;
      end

      FT_ADD: begin
         if (ad_neg)
            rg_xbig <= True;          // equal exponents and |X| > |S|: redo as X - S
         else begin
            rg_s     <= ad_out;
            rg_es    <= rg_es + 1;
            if (rg_sub && rg_xbig) rg_sign <= rg_signc;
            rg_padd  <= False;
            rg_pns   <= True;
            rg_state <= FT_NORM1;
         end
      end

      // ----------------------------------------------------------------
      // div / sqrt (operands normalized in S and X)
      FT_DIVINIT: begin
         rg_q <= 0;
         if (rg_sqrt) begin
            rg_r  <= 0;
            rg_d  <= ((pack (rg_es) [0] == 1) ? { rg_s [107:55], 1'b0 } : { 1'b0, rg_s [107:55] });
            rg_es <= rg_es >> 1;
         end
         else begin
            rg_r  <= zeroExtend (rg_s [107:55]);
            rg_d  <= zeroExtend (rg_x [107:55]);
            rg_es <= rg_es - rg_ex;
         end
         rg_cnt   <= (rg_dbl ? 55 : 26);
         rg_state <= FT_DIVITER;
      end

      FT_DIVITER: begin
         rg_r <= (dv_ge ? dv_t [56:0] : dv_rn [56:0]);
         rg_q <= { rg_q [53:0], pack (dv_ge) };
         if (rg_sqrt) rg_d <= rg_d << 2;
         rg_cnt <= rg_cnt - 1;
         if (rg_cnt == 1) rg_state <= FT_DIVEND;
      end

      FT_DIVEND: begin
         Bit #(55) q = (rg_dbl ? rg_q : { rg_q [25:0], 29'b0 });
         rg_s     <= { q, 52'b0, pack (rg_r != 0) };
         rg_pdiv  <= False;
         rg_pns   <= True;
         rg_state <= FT_NORM1;
      end

      // ----------------------------------------------------------------
      // FCVT to integer: units bit to S[55]
      FT_F2I: begin
         rg_s     <= sh_out;
         rg_state <= FT_RND1;
      end

      // ----------------------------------------------------------------
      // round: tininess after rounding, denormalize
      FT_RND0: begin
         rg_amt <= fn_clamp (emin - rg_es);
         if (rg_es < emin) begin
            rg_tiny  <= ! ((rg_es == emin - 1) && rn_ones && rn_inc);
            rg_state <= FT_DENORM;
         end
         else
            rg_state <= FT_RND1;
      end

      FT_DENORM: begin
         rg_s     <= sh_out;
         rg_es    <= emin;
         rg_state <= FT_RND1;
      end

      FT_RND1: begin
         rg_rsig  <= rn_sum;
         rg_nx    <= (rn_g == 1) || rn_st;
         rg_state <= (rg_f2i ? FT_IPACK : FT_PACK);
      end

      FT_PACK: begin
         Bit #(1) carry = rg_rsig [53];
         Bool     nz    = (carry == 1) || (rg_rsig [52] == 1);
         FT_Exp   eb    = rg_es + (rg_dbl ? 1023 : 127) + ((carry == 1) ? 1 : 0);
         Bit #(11) ef   = (nz ? truncate (pack (eb)) : 0);
         Bool     ovf   = nz && (eb >= (rg_dbl ? 2047 : 255));
         Bool     to_inf = case (rg_rm)
                              3'b001:  False;
                              3'b010:  (rg_sign == 1);
                              3'b011:  (rg_sign == 0);
                              default: True;
                           endcase;
         Bit #(64) r;
         if (ovf)
            r = (rg_dbl ? (to_inf ? { rg_sign, 11'h7ff, 52'b0 } : { rg_sign, 11'h7fe, '1 })
                        : { 32'hffffffff, rg_sign, (to_inf ? { 8'hff, 23'b0 } : { 8'hfe, 23'h7fffff }) });
         else
            r = (rg_dbl ? { rg_sign, ef, rg_rsig [51:0] }
                        : { 32'hffffffff, rg_sign, ef [7:0], rg_rsig [51:29] });
         Bool nx = rg_nx || ovf;
         fa_done (r, { 2'b0, pack (ovf), pack (rg_tiny && rg_nx), pack (nx) });
      end

      FT_IPACK: begin
         Bit #(54) m   = rg_rsig;
         Bool      neg = (rg_sign == 1);
         Bool      ok  = (rg_uns ? (neg ? (m == 0) : (m < 54'h100000000))
                                 : (neg ? (m <= 54'h80000000) : (m < 54'h80000000)));
         Bit #(32) r32 = (neg ? (- m [31:0]) : m [31:0]);
         if (ok) fa_done (signExtend (r32), (rg_nx ? 5'h01 : 0));
         else    fa_done ((neg ? f2i_sat_neg : f2i_sat_pos), 5'h10);
      end

      default: noAction;
      endcase
   endrule

   // =============================================================
   // INTERFACE

   interface server_reset = toGPServer (f_reset_reqs, f_reset_rsps);

   method Action req (Opcode opcode, Bit #(7) f7, Bit #(3) rm, RegName rs2,
                      Bit #(64) v1, Bit #(64) v2, Bit #(64) v3);
      rw_req.wset (tuple7 (opcode, f7, rm, rs2, v1, v2, v3));
   endmethod

   method Bool valid = rg_valid;
   method Tuple2 #(Bit #(64), Bit #(5)) word = tuple2 (rg_res, rg_fl);

endmodule

// ================================================================

endpackage
