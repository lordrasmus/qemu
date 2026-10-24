/*
 * Blackfin 32-bit DSP instructions: ALU, MAC/multiply, shifter
 *
 * Semantics after the Blackfin Processor Programming Reference, Rev. 2.2,
 * chapters 9 and 13 to 19; field meanings after appendix C (tables C-23).
 * The instruction word is passed whole; the translator has checked that
 * it is one the manual lists.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "cpu.h"
#include "exec/helper-proto.h"

/* ---------------------------------------------------- register access */

/*
 * In a multi-issue bundle all three instructions read their operands
 * before any of them writes; the D-register results of the DSP slot are
 * held back until the two 16-bit slots have run (helper_dsp_commit).
 */
static void set_dreg(CPUBfinState *env, int r, uint32_t v)
{
    if (env->dsp_defer) {
        env->dsp_wb_mask |= 1u << r;
        env->dsp_wb[r] = v;
    } else {
        env->reg[BFIN_R0 + r] = v;
    }
}

static uint32_t get_dreg(CPUBfinState *env, int r)
{
    return env->reg[BFIN_R0 + r];
}

/* a half register as written so far by this instruction */
static uint32_t cur_dreg(CPUBfinState *env, int r)
{
    if (env->dsp_defer && (env->dsp_wb_mask & (1u << r))) {
        return env->dsp_wb[r];
    }
    return env->reg[BFIN_R0 + r];
}

static uint16_t get_half(CPUBfinState *env, int r, int hi)
{
    return get_dreg(env, r) >> (hi ? 16 : 0);
}

static void set_half(CPUBfinState *env, int r, int hi, uint16_t v)
{
    set_dreg(env, r, deposit32(cur_dreg(env, r), hi ? 16 : 0, 16, v));
}

void helper_dsp_commit(CPUBfinState *env)
{
    for (int r = 0; r < 8; r++) {
        if (env->dsp_wb_mask & (1u << r)) {
            env->reg[BFIN_R0 + r] = env->dsp_wb[r];
        }
    }
    env->dsp_wb_mask = 0;
    env->dsp_defer = 0;
}

void helper_dsp_defer(CPUBfinState *env)
{
    env->dsp_defer = 1;
    env->dsp_wb_mask = 0;
}

/* 40-bit accumulators, kept sign extended in an int64_t */
static int64_t get_acc(CPUBfinState *env, int n)
{
    uint64_t v = ((uint64_t)(env->reg[BFIN_A0X + 2 * n] & 0xff) << 32) |
                 env->reg[BFIN_A0W + 2 * n];

    return sextract64(v, 0, 40);
}

static void set_acc(CPUBfinState *env, int n, int64_t v)
{
    env->reg[BFIN_A0X + 2 * n] = (v >> 32) & 0xff;
    env->reg[BFIN_A0W + 2 * n] = (uint32_t)v;
}

#define ACC_MAX  ((int64_t)0x7fffffffffll)
#define ACC_MIN  (-(int64_t)0x8000000000ll)

static void set_flag(CPUBfinState *env, int bit, bool v)
{
    env->astat[bit] = v;
    if (bit == ASTAT_AC0) {
        env->astat[ASTAT_AC0_COPY] = v;
    } else if (bit == ASTAT_V) {
        env->astat[ASTAT_V_COPY] = v;
    }
}

static void set_v(CPUBfinState *env, bool v)
{
    set_flag(env, ASTAT_V, v);
    if (v) {
        env->astat[ASTAT_VS] = 1;
    }
}

static void set_av(CPUBfinState *env, int n, bool v)
{
    env->astat[n ? ASTAT_AV1 : ASTAT_AV0] = v;
    if (v) {
        env->astat[n ? ASTAT_AV1S : ASTAT_AV0S] = 1;
    }
}

static int32_t sat32(int64_t v, bool *ov)
{
    if (v > INT32_MAX) {
        *ov = true;
        return INT32_MAX;
    }
    if (v < INT32_MIN) {
        *ov = true;
        return INT32_MIN;
    }
    return v;
}

static int16_t sat16(int64_t v, bool *ov)
{
    if (v > INT16_MAX) {
        *ov = true;
        return INT16_MAX;
    }
    if (v < INT16_MIN) {
        *ov = true;
        return INT16_MIN;
    }
    return v;
}

static uint16_t usat16(int64_t v, bool *ov)
{
    if (v > UINT16_MAX) {
        *ov = true;
        return UINT16_MAX;
    }
    if (v < 0) {
        *ov = true;
        return 0;
    }
    return v;
}

static uint32_t usat32(int64_t v, bool *ov)
{
    if (v > UINT32_MAX) {
        *ov = true;
        return UINT32_MAX;
    }
    if (v < 0) {
        *ov = true;
        return 0;
    }
    return v;
}

static int64_t sat40(int64_t v, bool *ov)
{
    if (v > ACC_MAX) {
        *ov = true;
        return ACC_MAX;
    }
    if (v < ACC_MIN) {
        *ov = true;
        return ACC_MIN;
    }
    return v;
}

/* --------------------------------------------------------- shifter */

/*
 * Shift a value of @bits width by @n (positive left, negative right).
 * Arithmetic right shifts fill with the sign; left shifts are logical
 * unless @sat, which saturates when significant bits would be lost.
 * *ov reports lost significant bits of a left shift.
 */
static int64_t shift_val(int64_t v, int bits, int n, bool arith, bool sat,
                         bool *ov)
{
    uint64_t mask = MAKE_64BIT_MASK(0, bits);

    if (n >= 0) {
        int64_t r;

        if (n >= bits) {
            r = 0;
        } else {
            r = sextract64((uint64_t)v << n, 0, bits);
        }
        /* bits shifted out, or the sign changed */
        if (arith && n > 0 && (n >= bits || (r >> n) != sextract64(v, 0, bits))) {
            *ov = true;
            if (sat) {
                return v < 0 ? -(int64_t)(1ull << (bits - 1))
                             : (int64_t)((1ull << (bits - 1)) - 1);
            }
        }
        return arith ? r : (int64_t)((uint64_t)r & mask);
    }
    n = -n;
    if (arith) {
        if (n >= bits) {
            return v < 0 ? -1 : 0;
        }
        return sextract64(v, 0, bits) >> n;
    }
    if (n >= bits) {
        return 0;
    }
    return ((uint64_t)v & mask) >> n;
}

/* AZ and AN of a @bits wide result */
static void set_az_an(CPUBfinState *env, int64_t r, int bits)
{
    set_flag(env, ASTAT_AZ, extract64(r, 0, bits) == 0);
    set_flag(env, ASTAT_AN, extract64(r, bits - 1, 1));
}

/*
 * The shift ops shared by dsp32shift (magnitude from a register) and
 * dsp32shiftimm (magnitude from the instruction).  kind 0 = ASHIFT,
 * 1 = ASHIFT (S), 2 = LSHIFT.
 */
static void shift_half(CPUBfinState *env, int dst, int dsth, int src, int srch,
                       int n, int kind)
{
    bool ov = false;
    int64_t v = (int16_t)get_half(env, src, srch);
    int64_t r = shift_val(kind == 2 ? (uint16_t)v : v, 16, n, kind != 2,
                          kind == 1, &ov);

    set_half(env, dst, dsth, r);
    set_az_an(env, r, 16);
    set_v(env, kind != 2 && ov);
}

static void shift_vec(CPUBfinState *env, int dst, int src, int n, int kind)
{
    bool ov = false, az = false, an = false;
    uint32_t s = get_dreg(env, src), out = 0;

    for (int h = 0; h < 2; h++) {
        int64_t v = (int16_t)(s >> (16 * h));
        int64_t r = shift_val(kind == 2 ? (uint16_t)v : v, 16, n, kind != 2,
                              kind == 1, &ov);

        out |= (uint32_t)(r & 0xffff) << (16 * h);
        az |= (r & 0xffff) == 0;
        an |= (r >> 15) & 1;
    }
    set_dreg(env, dst, out);
    set_flag(env, ASTAT_AZ, az);
    set_flag(env, ASTAT_AN, an);
    set_v(env, kind != 2 && ov);
}

static void shift_word(CPUBfinState *env, int dst, int src, int n, int kind)
{
    bool ov = false;
    int64_t v = (int32_t)get_dreg(env, src);
    int64_t r = shift_val(kind == 2 ? (uint32_t)v : v, 32, n, kind != 2,
                          kind == 1, &ov);

    set_dreg(env, dst, r);
    set_az_an(env, r, 32);
    set_v(env, kind != 2 && ov);
}

/*
 * Accumulator shifts do not saturate.  AVn is cleared, except that the
 * arithmetic ones set it for a zero result: the PRM's "AV0 is set if
 * result is zero" reads like a slip, but ADI's random_0011/0014 vectors
 * (overflowing left shifts, AV0 left clear) agree with it
 */
static void shift_acc(CPUBfinState *env, int a, int n, int kind)
{
    bool ov = false;
    int64_t v = get_acc(env, a);
    int64_t r = shift_val(kind == 2 ? (v & MAKE_64BIT_MASK(0, 40)) : v, 40, n,
                          kind != 2, false, &ov);

    r = sextract64(r, 0, 40);
    set_acc(env, a, r);
    set_az_an(env, r, 40);
    set_av(env, a, kind != 2 && r == 0);
}

/* rotate @bits bits plus CC by @n (positive left) */
static uint64_t rot(CPUBfinState *env, uint64_t v, int bits, int n)
{
    int w = bits + 1;
    uint64_t x = (v & MAKE_64BIT_MASK(0, bits)) |
                 ((uint64_t)env->astat[ASTAT_CC] << bits);

    n %= w;
    if (n < 0) {
        n += w;
    }
    if (n) {
        x = ((x << n) | (x >> (w - n))) & MAKE_64BIT_MASK(0, w);
    }
    env->astat[ASTAT_CC] = (x >> bits) & 1;
    return x & MAKE_64BIT_MASK(0, bits);
}

static int signbits32(uint32_t v)
{
    return (v >> 31 ? clz32(~v) : clz32(v)) - 1;
}

static int signbits16(uint16_t v)
{
    return signbits32((int32_t)(int16_t)v) - 16;
}

static int signbits40(int64_t v)
{
    uint64_t x = (uint64_t)v << 24;

    return (v < 0 ? clz64(~x) : clz64(x)) - 9;
}

/*
 * dsp32shiftimm: sopcde 0 half, 1 vector, 2 word, 3 accumulator.
 * The 6-bit magnitude is signed; arithmetic right shifts are encoded
 * as negative values.
 */
void helper_dsp32shiftimm(CPUBfinState *env, uint32_t insn)
{
    int sopcde = extract32(insn, 16, 5);
    int sop = extract32(insn, 14, 2);
    int hls = extract32(insn, 12, 2);
    int dst = extract32(insn, 9, 3);
    int imm = sextract32(insn, 3, 6);
    int src = extract32(insn, 0, 3);
    int kind = sop == 0 ? 0 : sop == 1 ? 1 : 2;

    /*
     * The 16-bit forms use the low five bits of the magnitude: ADI's
     * random_0018 vectors turn R1.H >>> 29 (immag -29) into a left shift
     * by 3.  Every count the PRM allows reads the same either way
     */
    int imm5 = sextract32(insn, 3, 5);

    switch (sopcde) {
    case 0:
        shift_half(env, dst, hls >> 1, src, hls & 1, imm5, kind);
        break;
    case 1:
        shift_vec(env, dst, src, imm5, kind);
        break;
    case 2:
        if (sop == 3) {
            set_dreg(env, dst, rot(env, get_dreg(env, src), 32, imm));
        } else {
            shift_word(env, dst, src, imm, kind);
        }
        break;
    case 3:
        if (sop == 2) {
            set_acc(env, hls & 1, sextract64(rot(env, get_acc(env, hls & 1),
                                                   40, imm), 0, 40));
        } else {
            shift_acc(env, hls & 1, imm, sop == 0 ? 0 : 2);
        }
        break;
    }
}

/*
 * dsp32shift: the magnitude is the sign extended low six bits of
 * src0.L; the value is src1.  Also PACK, SIGNBITS, ONES, EXTRACT,
 * DEPOSIT and ALIGN.
 */
void helper_dsp32shift(CPUBfinState *env, uint32_t insn)
{
    int sopcde = extract32(insn, 16, 5);
    int sop = extract32(insn, 14, 2);
    int hls = extract32(insn, 12, 2);
    int dst = extract32(insn, 9, 3);
    int src0 = extract32(insn, 3, 3);
    int src1 = extract32(insn, 0, 3);
    int n = sextract32(get_dreg(env, src0), 0, 6);
    uint32_t a = get_dreg(env, src0), b = get_dreg(env, src1);

    switch (sopcde) {
    case 0:
        shift_half(env, dst, hls >> 1, src1, hls & 1, n,
                   sop == 0 ? 0 : sop == 1 ? 1 : 2);
        break;
    case 1:
        shift_vec(env, dst, src1, n, sop == 0 ? 0 : sop == 1 ? 1 : 2);
        break;
    case 2:
        if (sop == 3) {
            set_dreg(env, dst, rot(env, b, 32, n));
        } else {
            shift_word(env, dst, src1, n, sop == 0 ? 0 : sop == 1 ? 1 : 2);
        }
        break;
    case 3:
        if (sop == 2) {
            set_acc(env, hls & 1, sextract64(rot(env, get_acc(env, hls & 1),
                                                   40, n), 0, 40));
        } else {
            shift_acc(env, hls & 1, n, sop == 0 ? 0 : 2);
        }
        break;
    case 4:
        /*
         * PACK (h0, h1): h0 is the src1 field, h1 the src0 field, as
         * __muldi3's R0 = PACK (R0.L, R3.L) (0xc6040018) relies on
         */
        set_dreg(env, dst, ((uint32_t)(uint16_t)(b >> (sop & 2 ? 16 : 0)) << 16) |
                           (uint16_t)(a >> (sop & 1 ? 16 : 0)));
        break;
    case 5:                                         /* SIGNBITS Dreg[_lo/_hi] */
        set_half(env, dst, 0, sop == 0 ? signbits32(b) :
                              signbits16(b >> (sop == 2 ? 16 : 0)));
        break;
    case 6:
        if (sop == 3) {                             /* ONES */
            set_half(env, dst, 0, ctpop32(b));
        } else {                                    /* SIGNBITS A0/A1 */
            set_half(env, dst, 0, signbits40(get_acc(env, sop)));
        }
        break;
    case 7:
        /*
         * Dreg_lo = EXPADJ (sample, Dreg_lo): the smaller of the sample's
         * exponent (redundant sign bits) and the reference.  The sample
         * is the src1 field: sop 0 word, 1 both halves (V), 2 low, 3 high
         */
        /*
         * The hardware compares only the exponent bits, 5 for a word and
         * 4 for a half, and returns the winner whole: ADI's
         * c_dsp32shift_expadj_l vectors give EXPADJ (0x1000, 0x1001) =
         * 0x1001, not the 2 a plain minimum would
         */
        {
            uint16_t e = a;
            unsigned m = sop == 0 ? 0x1f : 0xf;
            uint16_t x[2];
            int cnt = 1;

            if (sop == 0) {
                x[0] = signbits32(b);
            } else if (sop == 1) {
                x[0] = signbits16(b >> 16);
                x[1] = signbits16(b);
                cnt = 2;
            } else {
                x[0] = signbits16(b >> (sop == 3 ? 16 : 0));
            }
            for (int i = 0; i < cnt; i++) {
                if ((x[i] & m) < (e & m)) {
                    e = x[i];
                }
            }
            set_half(env, dst, 0, e);
        }
        break;
    case 8:
        /*
         * BITMUX (source_1, source_0, A0): source_1 is the src0 field,
         * source_0 the src1 field (PRM tables 13-3..13-5).  (ASR) shifts
         * the LSBs into A0 bits 39 (source_0) and 38 (source_1), (ASL) the
         * MSBs into bits 1 (source_0) and 0 (source_1)
         */
        {
            uint64_t acc = get_acc(env, 0) & MAKE_64BIT_MASK(0, 40);

            if (sop == 0) {
                acc = (acc >> 2) | ((uint64_t)(a & 1) << 38) |
                      ((uint64_t)(b & 1) << 39);
                set_dreg(env, src0, a >> 1);
                set_dreg(env, src1, b >> 1);
            } else {
                acc = (acc << 2) | (a >> 31) | ((b >> 31) << 1);
                set_dreg(env, src0, a << 1);
                set_dreg(env, src1, b << 1);
            }
            set_acc(env, 0, sextract64(acc, 0, 40));
        }
        break;
    case 9:
        /*
         * VIT_MAX: the larger of each register's halves; A0 shifts by one
         * (single) or two (dual) places and records which half won, 1 for
         * the high one.  ASL appends at bit 0, ASR at bit 31; the first
         * operand (src1 field) fills dest.H and the LSB.  The hardware
         * takes the sign of the 16-bit difference, not a signed compare:
         * ADI's c_dsp32shift_vmax keeps 0xc001 out of 0x4300c001
         */
        {
            uint64_t acc = get_acc(env, 0) & MAKE_64BIT_MASK(0, 40);
            bool asr = sop & 1;
            int16_t yh = b >> 16, yl = b, zh = a >> 16, zl = a;
            bool y = !((uint16_t)(yh - yl) & 0x8000);
            bool z = !((uint16_t)(zh - zl) & 0x8000);

            if (sop < 2) {
                acc = asr ? acc >> 1 : acc << 1;
                if (y) {
                    acc |= asr ? 0x80000000u : 1;
                }
                set_half(env, dst, 0, y ? yh : yl);
            } else {
                acc = asr ? acc >> 2 : acc << 2;
                if (y) {
                    acc |= asr ? 0x80000000u : 1;
                }
                if (z) {
                    acc |= asr ? 0x40000000u : 2;
                }
                set_dreg(env, dst, ((uint32_t)(uint16_t)(y ? yh : yl) << 16) |
                                   (uint16_t)(z ? zh : zl));
            }
            set_acc(env, 0, sextract64(acc, 0, 40));
        }
        break;
    case 11:
    case 12:
        /*
         * BXORSHIFT, BXOR: XOR reduction of A0 masked by Dreg (11, the
         * src0 field) or A1 (12).  11/0 shifts A0 left first and keeps
         * it; 12 XORs in CC, 12/0 shifts the result into A0 and leaves
         * CC, 12/1 writes CC
         */
        {
            uint64_t acc = get_acc(env, 0) & MAKE_64BIT_MASK(0, 40);
            uint64_t mask = sopcde == 11 ? a
                          : get_acc(env, 1) & MAKE_64BIT_MASK(0, 40);
            bool cc;

            if (sopcde == 11 && sop == 0) {
                acc = (acc << 1) & MAKE_64BIT_MASK(0, 40);
                set_acc(env, 0, sextract64(acc, 0, 40));
            }
            cc = ctpop64(acc & mask) & 1;
            if (sopcde == 12) {
                cc ^= env->astat[ASTAT_CC];
                if (sop == 0) {
                    acc = ((acc << 1) | cc) & MAKE_64BIT_MASK(0, 40);
                    set_acc(env, 0, sextract64(acc, 0, 40));
                    break;
                }
            }
            env->astat[ASTAT_CC] = cc;
            set_half(env, dst, 0, cc);
        }
        break;
    case 10:                                        /* EXTRACT, DEPOSIT */
        {
            int p = extract32(a, 8, 5), l = extract32(a, 0, 5);
            uint32_t r;

            if (sop < 2) {
                uint64_t f = p + l > 32 ? (uint64_t)b >> p : extract64(b, p, l);

                r = l == 0 ? 0 : (sop ? sextract64(f, 0, l) : f & MAKE_64BIT_MASK(0, l));
            } else {
                uint32_t fg = a >> 16;

                if (l > 16) {
                    l = 16;
                }
                r = b;
                if (l) {
                    uint32_t m = (uint32_t)(MAKE_64BIT_MASK(0, l) << p);

                    r = (b & ~m) | (((uint64_t)fg << p) & m);
                }
                if (sop == 3) {
                    int top = p + l;

                    if (top > 0 && top < 32) {
                        r = sextract32(r, 0, top);
                    } else if (top == 0) {
                        r = 0;
                    }
                }
                b = r;
                a = 0;
            }
            set_dreg(env, dst, r);
            set_az_an(env, (int32_t)r, 32);
            set_flag(env, ASTAT_AC0, 0);
            set_v(env, 0);
        }
        break;
    case 13:
        /*
         * ALIGN8/16/24 (src_reg_1, src_reg_0): src_reg_1 is the upper
         * word and, as for PACK, the src1 field
         */
        {
            int s = 8 * (sop + 1);

            set_dreg(env, dst, (uint32_t)(((uint64_t)b << 32 | a) >> s));
        }
        break;
    }
}

/* ------------------------------------------------------------- ALU */

/* 16-bit add/sub with optional saturation; AC is the carry out */
static uint16_t alu16s(CPUBfinState *env, uint16_t x, uint16_t y, bool sub,
                       bool sat, int scale, bool *v, bool *ac)
{
    int32_t r = sub ? (int16_t)x - (int16_t)y : (int16_t)x + (int16_t)y;
    uint32_t u = sub ? (uint32_t)x + (uint16_t)~y + 1 : (uint32_t)x + y;

    *ac = u >> 16;
    /* (ASR)/(ASL) scale the exact result, before saturation (PRM) */
    if (scale) {
        r = scale > 0 ? r * 2 : r >> 1;
    }
    if (r > INT16_MAX || r < INT16_MIN) {
        *v = true;
        if (sat) {
            return r < 0 ? INT16_MIN : INT16_MAX;
        }
    }
    return r;
}

static uint16_t alu16(CPUBfinState *env, uint16_t x, uint16_t y, bool sub,
                      bool sat, bool *v, bool *ac)
{
    return alu16s(env, x, y, sub, sat, 0, v, ac);
}

static uint32_t alu32(CPUBfinState *env, uint32_t x, uint32_t y, bool sub,
                      bool sat, bool *v, bool *ac)
{
    int64_t r = sub ? (int64_t)(int32_t)x - (int32_t)y
                    : (int64_t)(int32_t)x + (int32_t)y;

    *ac = sub ? x >= y : (uint64_t)x + y > UINT32_MAX;
    if (r > INT32_MAX || r < INT32_MIN) {
        *v = true;
        if (sat) {
            return r < 0 ? INT32_MIN : INT32_MAX;
        }
    }
    return r;
}

/* the "Status Bits Affected" of the dual 16-bit add/subtract family */
static void vec_flags(CPUBfinState *env, uint32_t r, bool v, bool ac0, bool ac1)
{
    set_flag(env, ASTAT_AZ, (r & 0xffff) == 0 || (r >> 16) == 0);
    set_flag(env, ASTAT_AN, ((r >> 15) & 1) || (r >> 31));
    set_v(env, v);
    set_flag(env, ASTAT_AC0, ac0);
    set_flag(env, ASTAT_AC1, ac1);
}

/*
 * src0 +|+ src1 and friends: aop bit 1 selects - for the high half,
 * bit 0 for the low half
 */
static uint32_t vec16(CPUBfinState *env, uint32_t a, uint32_t b, int aop,
                      bool sat, int scale, bool *v, bool *ac0, bool *ac1)
{
    uint16_t hi = alu16s(env, a >> 16, b >> 16, aop & 2, sat, scale, v, ac1);
    uint16_t lo = alu16s(env, a, b, aop & 1, sat, scale, v, ac0);

    return ((uint32_t)hi << 16) | lo;
}

static bool any_half_zero(uint32_t r)
{
    return (r & 0xffff) == 0 || (r >> 16) == 0;
}

static bool any_half_neg(uint32_t r)
{
    return ((r >> 15) & 1) || (r >> 31);
}

/*
 * The video pixel instructions read src0 and src1 as register pairs
 * R1:0 or R3:2 and take an unaligned word out of each: the low two bits
 * of I0 (pair src0) and I1 (pair src1) give the byte offset; (R) swaps
 * the registers within the pair (PRM chapter 18)
 */
static uint32_t byte_align(CPUBfinState *env, int pair, int ireg, bool rev)
{
    uint32_t lo = get_dreg(env, pair), hi = get_dreg(env, pair + 1);
    int n = env->reg[BFIN_I0 + ireg] & 3;

    if (rev) {
        uint32_t t = lo;

        lo = hi;
        hi = t;
    }
    return n ? (lo >> (8 * n)) | (hi << (32 - 8 * n)) : lo;
}

static unsigned byte_of(uint32_t w, int n)
{
    return (w >> (8 * n)) & 0xff;
}

/* accumulator results: AZ, AN from the 40-bit value, AV saturation */
static void acc_result(CPUBfinState *env, int n, int64_t v, bool ov)
{
    set_acc(env, n, v);
    set_flag(env, ASTAT_AZ, v == 0);
    set_flag(env, ASTAT_AN, v < 0);
    set_av(env, n, ov);
}

/* round the 40-bit accumulator at bit 16 (RND_MOD: biased) */
static int64_t round16(CPUBfinState *env, int64_t v)
{
    if (env->astat[ASTAT_RND_MOD]) {
        return v + 0x8000;
    }
    /* unbiased: ties to even */
    if ((v & 0xffff) == 0x8000) {
        return v + ((v & 0x10000) ? 0x8000 : 0);
    }
    return v + 0x8000;
}

void helper_dsp32alu(CPUBfinState *env, uint32_t insn)
{
    int hl = extract32(insn, 21, 1);
    int aopcde = extract32(insn, 16, 5);
    int aop = extract32(insn, 14, 2);
    int s = extract32(insn, 13, 1);
    int x = extract32(insn, 12, 1);
    int dst0 = extract32(insn, 9, 3);
    int dst1 = extract32(insn, 6, 3);
    int src0 = extract32(insn, 3, 3);
    int src1 = extract32(insn, 0, 3);
    uint32_t a = get_dreg(env, src0), b = get_dreg(env, src1);
    bool v = false, ac = false;

    switch (aopcde) {
    case 0:                                         /* +|+ +|- -|+ -|- */
        {
            bool ac0, ac1;
            uint32_t r = vec16(env, a, b, aop, s, 0, &v, &ac0, &ac1);

            /* (CO) crosses the result halves */
            set_dreg(env, dst0, x ? ror32(r, 16) : r);
            vec_flags(env, r, v, ac0, ac1);
        }
        break;
    case 1:
        /*
         * quad: hl=0 dst1 = +|+, dst0 = -|-; hl=1 dst1 = +|-, dst0 = -|+.
         * AC0/AC1 stay (PRM "unaffected if a quad operation"); (CO)
         * crosses the second result only, as ADI's c_dsp32alu_rrppmm
         * hardware vectors have it
         */
        {
            int scale = aop == 2 ? -1 : aop == 3 ? 1 : 0;
            uint32_t r1 = vec16(env, a, b, hl, s, scale, &v, &ac, &ac);
            uint32_t r0 = vec16(env, a, b, 3 - hl, s, scale, &v, &ac, &ac);

            set_dreg(env, dst1, r1);
            set_dreg(env, dst0, x ? ror32(r0, 16) : r0);
            set_flag(env, ASTAT_AZ, any_half_zero(r0) || any_half_zero(r1));
            set_flag(env, ASTAT_AN, any_half_neg(r0) || any_half_neg(r1));
            set_v(env, v);
        }
        break;
    case 2:                                         /* half add */
    case 3:                                         /* half subtract */
        {
            uint16_t r = alu16(env, a >> (aop & 2 ? 16 : 0),
                               b >> (aop & 1 ? 16 : 0), aopcde == 3, s,
                               &v, &ac);

            set_half(env, dst0, hl, r);
            set_az_an(env, (int16_t)r, 16);
            set_v(env, v);
            set_flag(env, ASTAT_AC0, ac);
        }
        break;
    case 4:
        if (aop < 2) {                              /* Dreg = Dreg +/- Dreg */
            uint32_t r = alu32(env, a, b, aop, s, &v, &ac);

            set_dreg(env, dst0, r);
            set_az_an(env, (int32_t)r, 32);
            set_v(env, v);
            set_flag(env, ASTAT_AC0, ac);
        } else {                                    /* dst1 = +, dst0 = - */
            bool ac1;
            uint32_t r1 = alu32(env, a, b, 0, s, &v, &ac);
            uint32_t r0 = alu32(env, a, b, 1, s, &v, &ac1);

            set_dreg(env, dst1, r1);
            set_dreg(env, dst0, r0);
            set_flag(env, ASTAT_AZ, r0 == 0 || r1 == 0);
            set_flag(env, ASTAT_AN, (r0 | r1) >> 31);
            set_v(env, v);
            set_flag(env, ASTAT_AC0, ac);
            set_flag(env, ASTAT_AC1, ac1);
        }
        break;
    case 5:
        /*
         * aop 0/1: Dreg_lo_hi = Dreg +/- Dreg (RND12), operands scaled
         * up by 4, rounded and saturated; aop 2/3: (RND20), scaled down
         * by 4, rounded, V cleared.  Both round biased, whatever RND_MOD
         */
        {
            int64_t x0 = (int32_t)a, x1 = (int32_t)b, r;
            uint16_t h;

            /*
             * (RND20) subtracts by adding the shifted negation: ADI's
             * c_dsp32alu_rh_rnd20_m vectors round 0x8911 - 0x5678891b to
             * 0xfa98, not the 0xfa99 that shifting both first gives
             */
            if (aop & 1) {
                x1 = -x1;
            }
            if (aop < 2) {
                x0 <<= 4;
                x1 <<= 4;
            } else {
                x0 >>= 4;
                x1 >>= 4;
            }
            r = (x0 + x1 + 0x8000) >> 16;
            if (aop < 2) {
                h = sat16(r, &v);
                set_v(env, v);
            } else {
                h = r;
                set_flag(env, ASTAT_V, 0);
            }
            set_half(env, dst0, hl, h);
            set_az_an(env, (int16_t)h, 16);
        }
        break;
    case 6:                                         /* vector MAX/MIN/ABS */
        {
            uint32_t r = 0;

            for (int h = 0; h < 2; h++) {
                int16_t x0 = a >> (16 * h), x1 = b >> (16 * h);
                int32_t y = aop == 0 ? MAX(x0, x1) : aop == 1 ? MIN(x0, x1)
                          : (x0 < 0 ? -(int32_t)x0 : x0);

                if (aop == 2) {
                    y = sat16(y, &v);
                }
                r |= (uint32_t)(uint16_t)y << (16 * h);
            }
            set_dreg(env, dst0, r);
            set_flag(env, ASTAT_AZ, (r & 0xffff) == 0 || (r >> 16) == 0);
            set_flag(env, ASTAT_AN, ((r >> 15) & 1) || (r >> 31));
            set_v(env, v);
        }
        break;
    case 7:
        {
            int32_t x0 = a, x1 = b;
            int64_t r;

            switch (aop) {
            case 0:
                r = MAX(x0, x1);
                break;
            case 1:
                r = MIN(x0, x1);
                break;
            case 2:                                 /* ABS, saturating */
                r = sat32(x0 < 0 ? -(int64_t)x0 : x0, &v);
                break;
            default:                                /* NEG (NS)/(S) */
                r = -(int64_t)x0;
                if (r > INT32_MAX) {
                    v = true;
                    r = s ? INT32_MAX : INT32_MIN;
                }
                set_flag(env, ASTAT_AC0, x0 == 0);
                break;
            }
            set_dreg(env, dst0, r);
            set_az_an(env, r, 32);
            set_v(env, v);
        }
        break;
    case 8:                                         /* A = 0, A = A (S) */
        if (aop == 3) {                             /* A0 = A1, A1 = A0 */
            set_acc(env, s, get_acc(env, !s));
            break;
        }
        for (int n = 0; n < 2; n++) {
            if (aop != 2 && aop != n) {
                continue;
            }
            if (s) {
                bool ov = false;
                int64_t r = sat32(get_acc(env, n), &ov);

                acc_result(env, n, r, ov);
            } else {
                set_acc(env, n, 0);
            }
        }
        break;
    case 9:                                         /* A0.L/H/X, A1... = Dreg */
        {
            int n = aop >> 1;
            uint16_t h = a >> (hl ? 16 : 0);

            if (s) {                                /* A0 = Dreg, A1 = Dreg */
                set_acc(env, n, (int32_t)a);
            } else if (aop & 1) {
                env->reg[BFIN_A0X + 2 * n] = h & 0xff;
            } else {
                env->reg[BFIN_A0W + 2 * n] =
                    deposit32(env->reg[BFIN_A0W + 2 * n], hl ? 16 : 0, 16, h);
            }
        }
        break;
    case 10:                                        /* Dreg_lo = A0.X/A1.X */
        set_half(env, dst0, 0, (int8_t)env->reg[BFIN_A0X + 2 * aop]);
        break;
    case 11:
        {
            bool ov = false;
            int64_t r;

            if (aop == 3) {
                r = get_acc(env, 0) - get_acc(env, 1);
            } else {
                r = get_acc(env, 0) + get_acc(env, 1);
            }
            r = s ? sat32(r, &ov) : sat40(r, &ov);
            acc_result(env, 0, r, ov);
            if (aop == 0) {                         /* Dreg = (A0 += A1) */
                bool v32 = false;

                set_dreg(env, dst0, sat32(r, &v32));
                set_v(env, v32);
            } else if (aop == 1) {                  /* Dreg_lo/hi = (...) */
                bool v32 = false;
                int32_t w = sat32(round16(env, r), &v32);

                set_half(env, dst0, hl, w >> 16);
                set_v(env, v32);
            }
        }
        break;
    case 12:
        if (aop == 3) {                             /* Dreg_lo/hi = Dreg (RND) */
            int32_t w = sat32((int64_t)(int32_t)a + 0x8000, &v);

            set_half(env, dst0, hl, w >> 16);
            set_az_an(env, (int16_t)(w >> 16), 16);
            set_v(env, v);
        } else if (aop == 0) {
            /*
             * Dreg_hi = Dreg_lo = SIGN (src0_hi) * src1_hi + SIGN (src0_lo)
             * * src1_lo: unsaturated, no status bits
             */
            int16_t h = (int16_t)(a >> 16) < 0 ? -(int16_t)(b >> 16) : b >> 16;
            int16_t l = (int16_t)a < 0 ? -(int16_t)b : b;
            uint16_t r = h + l;

            set_dreg(env, dst0, ((uint32_t)r << 16) | r);
        } else if (aop == 1) {
            /* Dreg = A1.L + A1.H, Dreg = A0.L + A0.H, no status bits */
            uint32_t w1 = env->reg[BFIN_A1W], w0 = env->reg[BFIN_A0W];

            set_dreg(env, dst1, (int16_t)w1 + (int16_t)(w1 >> 16));
            set_dreg(env, dst0, (int16_t)w0 + (int16_t)(w0 >> 16));
        }
        break;
    case 17:
        /*
         * dst1 = A1 + A0, dst0 = A1 - A0 (aop 1: A0 - A1), 32-bit results,
         * (S) saturates.  AC1 is the carry of the add, AC0 of the subtract,
         * both on the 40-bit Accumulators
         */
        {
            int64_t x0 = get_acc(env, 0), x1 = get_acc(env, 1);
            uint64_t u0 = x0 & MAKE_64BIT_MASK(0, 40);
            uint64_t u1 = x1 & MAKE_64BIT_MASK(0, 40);
            int64_t r1 = x1 + x0, r0 = aop ? x0 - x1 : x1 - x0;
            int64_t s1 = sat32(r1, &v), s0 = sat32(r0, &v);
            uint32_t d1 = s ? s1 : r1, d0 = s ? s0 : r0;

            set_dreg(env, dst1, d1);
            set_dreg(env, dst0, d0);
            set_v(env, v);
            set_flag(env, ASTAT_AN, (d0 | d1) >> 31);
            set_flag(env, ASTAT_AZ, d0 == 0 || d1 == 0);
            set_flag(env, ASTAT_AC1, u0 + u1 > MAKE_64BIT_MASK(0, 40));
            set_flag(env, ASTAT_AC0, aop ? u1 <= u0 : u0 <= u1);
        }
        break;
    case 13:
        /*
         * (dst1, dst0) = SEARCH src0 (GT, GE, LT, LE): each half against
         * the low half of A1 (high) or A0 (low); a winner replaces it and
         * P0 goes into the destination.  The surviving half is written
         * back sign extended either way: ADI's random_0007 vectors turn
         * an unchanged A0 of 0x0142357aea into 0x7aea
         */
        {
            uint32_t p0 = env->reg[BFIN_P0];

            for (int n = 0; n < 2; n++) {
                int16_t hv = a >> (16 * n);
                int16_t acc = env->reg[BFIN_A0W + 2 * n];
                bool up = aop == 0 ? hv > acc : aop == 1 ? hv >= acc
                        : aop == 2 ? hv < acc : hv <= acc;

                set_acc(env, n, up ? hv : acc);
                if (up) {
                    set_dreg(env, n ? dst1 : dst0, p0);
                }
            }
        }
        break;
    case 18:
        if (aop == 0) {
            /*
             * SAA: |y - z| of each byte added to A0.L, A0.H, A1.L, A1.H,
             * each saturating at 16 bits unsigned; the extensions clear
             */
            uint32_t y = byte_align(env, src0, 0, s);
            uint32_t z = byte_align(env, src1, 1, s);

            for (int n = 0; n < 2; n++) {
                uint32_t w = env->reg[BFIN_A0W + 2 * n], r = 0;

                for (int h = 0; h < 2; h++) {
                    int i = 2 * n + h;
                    unsigned d = abs((int)byte_of(y, i) - (int)byte_of(z, i));

                    r |= MIN((w >> (16 * h) & 0xffff) + d, 0xffffu) << (16 * h);
                }
                env->reg[BFIN_A0W + 2 * n] = r;
                env->reg[BFIN_A0X + 2 * n] = 0;
            }
        }
        /* aop 3: DISALGNEXCPT, misaligned loads do not trap here anyway */
        break;
    case 20:
        /* BYTEOP1P: byte-wise mean, rounded up unless (T) (aop 1) */
        {
            uint32_t y = byte_align(env, src0, 0, s);
            uint32_t z = byte_align(env, src1, 1, s), r = 0;

            for (int i = 0; i < 4; i++) {
                r |= ((byte_of(y, i) + byte_of(z, i) + !aop) >> 1) << (8 * i);
            }
            set_dreg(env, dst0, r);
        }
        break;
    case 21:
        /*
         * BYTEOP16P (aop 0) / BYTEOP16M (aop 1): byte-wise sum or
         * difference as half-words, bytes 0-1 into dst0, 2-3 into dst1
         */
        {
            uint32_t y = byte_align(env, src0, 0, s);
            uint32_t z = byte_align(env, src1, 1, s);
            uint16_t h[4];

            for (int i = 0; i < 4; i++) {
                h[i] = aop ? byte_of(y, i) - byte_of(z, i)
                           : byte_of(y, i) + byte_of(z, i);
            }
            set_dreg(env, dst0, h[0] | (uint32_t)h[1] << 16);
            set_dreg(env, dst1, h[2] | (uint32_t)h[3] << 16);
        }
        break;
    case 22:
        /*
         * BYTEOP2P: mean of the two byte pairs of each half, both pairs
         * aligned by I0 only; aop 0 rounds, 1 truncates; HL puts the
         * results in the high bytes of the half-words
         */
        {
            uint32_t y = byte_align(env, src0, 0, s);
            uint32_t z = byte_align(env, src1, 0, s), r = 0;

            for (int n = 0; n < 2; n++) {
                unsigned sum = byte_of(y, 2 * n) + byte_of(y, 2 * n + 1) +
                               byte_of(z, 2 * n) + byte_of(z, 2 * n + 1) +
                               (aop ? 0 : 2);

                r |= ((sum >> 2) & 0xff) << (16 * n + 8 * hl);
            }
            set_dreg(env, dst0, r);
        }
        break;
    case 23:
        /*
         * BYTEOP3P: a signed half of the src0 pair plus a byte of the
         * src1 pair, clipped to 0..255, into the low (LO) or high (HI)
         * byte of each half-word.  The bytes are 1 and 3 for (LO), 0 and
         * 2 for (HI); the PRM tables 18-3/4 are images only, ADI's
         * c_dsp32alu_byteop3p vectors confirm the choice
         */
        {
            uint32_t y = byte_align(env, src0, 0, s);
            uint32_t z = byte_align(env, src1, 1, s), r = 0;

            for (int n = 0; n < 2; n++) {
                int sum = (int16_t)(y >> (16 * n)) +
                          (int)byte_of(z, 2 * n + !hl);

                r |= (uint32_t)MIN(MAX(sum, 0), 255) << (16 * n + 8 * hl);
            }
            set_dreg(env, dst0, r);
        }
        break;
    case 24:
        if (aop == 0) {
            /* BYTEPACK: the low bytes of each half of src0 and src1 */
            set_dreg(env, dst0, byte_of(a, 0) | byte_of(a, 2) << 8 |
                                byte_of(b, 0) << 16 | byte_of(b, 2) << 24);
        } else {
            /*
             * (dst1, dst0) = BYTEUNPACK src0 pair: four bytes from the
             * offset I0 gives, zero-extended into the half-words
             */
            uint32_t y = byte_align(env, src0, 0, s);

            set_dreg(env, dst0, byte_of(y, 0) | byte_of(y, 1) << 16);
            set_dreg(env, dst1, byte_of(y, 2) | byte_of(y, 3) << 16);
        }
        break;
    case 14:                                        /* A = -A */
    case 16:                                        /* A = ABS A */
        {
            int src = aop & 1, dsts = aop == 3 ? 3 : 1 << hl;

            for (int n = 0; n < 2; n++) {
                bool ov = false;
                int64_t val = get_acc(env, aop == 3 ? n : src);

                if (!(dsts & (1 << n))) {
                    continue;
                }
                /*
                 * negate sets the carry of the destination Accumulator
                 * when the source is zero: the PRM names AC0 only, ADI's
                 * random_0008 vectors show A1 = -A1 writing AC1
                 */
                if (aopcde == 14 && aop != 3) {
                    set_flag(env, n ? ASTAT_AC1 : ASTAT_AC0, val == 0);
                }
                val = aopcde == 14 || val < 0 ? -val : val;
                acc_result(env, n, sat40(val, &ov), ov);
            }
        }
        break;
    case 15:                                        /* Dreg = -Dreg (V) */
        {
            uint32_t r = 0;
            bool ac0, ac1;

            r = alu16(env, 0, a >> 16, true, true, &v, &ac1) << 16;
            r |= alu16(env, 0, a, true, true, &v, &ac0);
            set_dreg(env, dst0, r);
            vec_flags(env, r, v, ac0, ac1);
        }
        break;
    }
}

/* ------------------------------------------------- multiply and MAC */

/* the mmod values of appendix C */
enum {
    MM_DEF = 0, MM_S2RND = 1, MM_T = 2, MM_W32 = 3, MM_FU = 4, MM_TFU = 6,
    MM_IS = 8, MM_ISS2 = 9, MM_IH = 11, MM_IU = 12,
};

static bool mm_unsigned(int mmod)
{
    return mmod == MM_FU || mmod == MM_TFU || mmod == MM_IU;
}

static bool mm_integer(int mmod)
{
    return mmod == MM_IS || mmod == MM_ISS2 || mmod == MM_IH || mmod == MM_IU;
}

/*
 * MAC1 in mixed mode (M) produces a signed result, so the unsigned
 * modes saturate and extract it as their signed counterparts (ADI's
 * issue119 vectors: (M, FU) leaves 0xc0000000 alone)
 */
static int mm_mixed(int mmod)
{
    switch (mmod) {
    case MM_FU:
        return MM_DEF;
    case MM_TFU:
        return MM_T;
    case MM_IU:
        return MM_IS;
    }
    return mmod;
}

/*
 * The unsigned modes read the Accumulator without its sign extension:
 * ADI's issue103 vectors extract A1 = 0xfff12ebeaa (FU) as 0xffffffff
 */
static int64_t mac_acc(CPUBfinState *env, int n, int mode)
{
    int64_t acc = get_acc(env, n);

    return mm_unsigned(mode) ? acc & MAKE_64BIT_MASK(0, 40) : acc;
}

/*
 * The 16 x 16 product of one MAC unit.  Fraction modes shift the signed
 * product left one bit; 0x8000 * 0x8000 saturates to 0x7fffffff.
 * mixed (M, MAC1 only): src0 signed, src1 unsigned, no shift.
 */
static int64_t mac_product(uint16_t x, uint16_t y, int mmod, bool mixed,
                           bool *ov)
{
    if (mixed) {
        return (int64_t)(int16_t)x * (uint16_t)y;
    }
    if (mm_unsigned(mmod)) {
        return (uint64_t)x * y;
    }
    if (mm_integer(mmod)) {
        return (int64_t)(int16_t)x * (int16_t)y;
    }
    if (x == 0x8000 && y == 0x8000) {
        *ov = true;
        return INT32_MAX;
    }
    return ((int64_t)(int16_t)x * (int16_t)y) << 1;
}

/* saturate an accumulator value for the mode: W32 at 32 bits */
static int64_t acc_sat(int64_t v, int mmod, bool *ov)
{
    /* IH keeps the Accumulator at 32.0 precision too (PRM table 15-4) */
    if (mmod == MM_W32 || mmod == MM_IH) {
        return sat32(v, ov);
    }
    if (mm_unsigned(mmod)) {
        if (v < 0) {
            *ov = true;
            return 0;
        }
        if (v > (int64_t)MAKE_64BIT_MASK(0, 40)) {
            *ov = true;
            return MAKE_64BIT_MASK(0, 40);
        }
        return v;
    }
    return sat40(v, ov);
}

/* extract a 16-bit result from an accumulator value (chapter 9, 15) */
static uint16_t extract_half(CPUBfinState *env, int64_t v, int mmod, bool *ov)
{
    switch (mmod) {
    case MM_IS:
        return sat16(v, ov);
    case MM_IU:
        return usat16(v, ov);
    case MM_ISS2:
        return sat16(v * 2, ov);
    case MM_IH:
        return sat32(round16(env, v), ov) >> 16;
    case MM_T:
        return sat32(v, ov) >> 16;
    case MM_TFU:
        return usat32(v, ov) >> 16;
    case MM_FU:
        return usat32(round16(env, v), ov) >> 16;
    case MM_S2RND:
        return sat32(round16(env, v * 2), ov) >> 16;
    default:
        return sat32(round16(env, v), ov) >> 16;
    }
}

static uint32_t extract_word(int64_t v, int mmod, bool *ov)
{
    switch (mmod) {
    case MM_FU:
    case MM_IU:
        return usat32(v, ov);
    case MM_S2RND:
    case MM_ISS2:
        return sat32(v * 2, ov);
    default:
        return sat32(v, ov);
    }
}

/*
 * dsp32mac (mult = 0) and dsp32mult (mult = 1).  MAC0 uses h00/h10,
 * op0 and w0 and writes dst.L (or dst when P); MAC1 uses h01/h11, op1,
 * w1 and writes dst.H (or dst + 1 when P).
 */
static void mac(CPUBfinState *env, uint32_t insn, bool mult)
{
    int mmod = extract32(insn, 21, 4);
    int mm = extract32(insn, 20, 1);
    int p = extract32(insn, 19, 1);
    int w1 = extract32(insn, 18, 1);
    int op1 = extract32(insn, 16, 2);
    int h01 = extract32(insn, 15, 1);
    int h11 = extract32(insn, 14, 1);
    int w0 = extract32(insn, 13, 1);
    int op0 = extract32(insn, 11, 2);
    int h00 = extract32(insn, 10, 1);
    int h10 = extract32(insn, 9, 1);
    int dst = extract32(insn, 6, 3);
    int src0 = extract32(insn, 3, 3);
    int src1 = extract32(insn, 0, 3);
    /* both MACs read the sources before either writes its result */
    uint32_t a = get_dreg(env, src0), b = get_dreg(env, src1);
    bool v = false;
    bool any_v = false;

    for (int n = 0; n < 2; n++) {
        int op = n ? op1 : op0;
        int w = n ? w1 : w0;
        int mode;
        bool ov = false;
        int64_t val;

        if (mult ? !w : (op == 3 && !w)) {
            continue;
        }
        mode = n && mm ? mm_mixed(mmod) : mmod;
        if (mult || op != 3) {
            uint16_t x = a >> ((n ? h01 : h00) * 16);
            uint16_t y = b >> ((n ? h11 : h10) * 16);
            int64_t prod = mac_product(x, y, mmod, n && mm, &v);

            if (mult) {
                val = prod;
            } else {
                int64_t acc = mac_acc(env, n, mode);

                val = op == 0 ? prod : op == 1 ? acc + prod : acc - prod;
                val = acc_sat(val, mode, &ov);
                set_acc(env, n, val);
                set_av(env, n, ov);
            }
        } else {
            val = mac_acc(env, n, mode);
        }
        if (!w) {
            continue;
        }
        ov = false;
        if (p) {
            set_dreg(env, dst + n, extract_word(val, mode, &ov));
        } else {
            set_half(env, dst, n, extract_half(env, val, mode, &ov));
        }
        any_v |= ov;
    }
    if (w0 || w1) {
        set_v(env, v || any_v);
    }
}

void helper_dsp32mac(CPUBfinState *env, uint32_t insn)
{
    mac(env, insn, false);
}

void helper_dsp32mult(CPUBfinState *env, uint32_t insn)
{
    mac(env, insn, true);
}
