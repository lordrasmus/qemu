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
}

static void set_v(CPUBfinState *env, bool v)
{
    env->astat[ASTAT_V] = v;
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

static void shift_acc(CPUBfinState *env, int a, int n, int kind)
{
    bool ov = false;
    int64_t v = get_acc(env, a);
    int64_t r = shift_val(kind == 2 ? (v & MAKE_64BIT_MASK(0, 40)) : v, 40, n,
                          kind != 2, false, &ov);

    set_acc(env, a, sextract64(r, 0, 40));
    set_az_an(env, r, 40);
    set_av(env, a, kind != 2 && ov);
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

    switch (sopcde) {
    case 0:
        shift_half(env, dst, hls >> 1, src, hls & 1, imm, kind);
        break;
    case 1:
        shift_vec(env, dst, src, imm, kind);
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
    case 13:                                        /* ALIGN8/16/24 */
        {
            int s = 8 * (sop + 1);

            set_dreg(env, dst, (uint32_t)(((uint64_t)a << 32 | b) >> s));
        }
        break;
    }
}

/* ------------------------------------------------------------- ALU */

/* 16-bit add/sub with optional saturation; AC is the carry out */
static uint16_t alu16(CPUBfinState *env, uint16_t x, uint16_t y, bool sub,
                      bool sat, bool *v, bool *ac)
{
    int32_t r = sub ? (int16_t)x - (int16_t)y : (int16_t)x + (int16_t)y;
    uint32_t u = sub ? (uint32_t)x + (uint16_t)~y + 1 : (uint32_t)x + y;

    *ac = u >> 16;
    if (r > INT16_MAX || r < INT16_MIN) {
        *v = true;
        if (sat) {
            return r < 0 ? INT16_MIN : INT16_MAX;
        }
    }
    return r;
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

static uint16_t vec_scale(int32_t r16, int aop)
{
    return r16;
}

/*
 * Dreg = src0 +|+ src1 and friends: aop bit 1 selects - for the high
 * half, bit 0 for the low half; x (CO) crosses the result halves.
 */
static void alu_vec(CPUBfinState *env, int dst, uint32_t a, uint32_t b,
                    int aop, bool sat, bool co, int scale)
{
    bool v = false, ac0, ac1;
    uint16_t hi = alu16(env, a >> 16, b >> 16, aop & 2, sat, &v, &ac1);
    uint16_t lo = alu16(env, a, b, aop & 1, sat, &v, &ac0);
    uint32_t r;

    if (scale) {
        /* (ASR) halves, (ASL) doubles the 16-bit results */
        int32_t h = scale > 0 ? (int16_t)hi << 1 : (int16_t)hi >> 1;
        int32_t l = scale > 0 ? (int16_t)lo << 1 : (int16_t)lo >> 1;

        hi = sat ? sat16(h, &v) : h;
        lo = sat ? sat16(l, &v) : l;
    }
    r = co ? ((uint32_t)lo << 16) | hi : ((uint32_t)hi << 16) | lo;
    set_dreg(env, dst, r);
    vec_flags(env, r, v, ac0, ac1);
    (void)vec_scale;
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
        alu_vec(env, dst0, a, b, aop, s, x, 0);
        break;
    case 1:                                         /* dual vector */
        {
            bool sat = s, co = x;
            int scale = aop == 2 ? -1 : aop == 3 ? 1 : 0;
            int first = hl ? 1 : 0;

            /* hl=0: dst1 = +|+, dst0 = -|-; hl=1: dst1 = +|-, dst0 = -|+ */
            alu_vec(env, dst1, a, b, first, sat, co, scale);
            alu_vec(env, dst0, a, b, 3 - first, sat, co, scale);
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

            if (aop & 1) {
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
    case 18:                                        /* DISALGNEXCPT */
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
    if (mmod == MM_W32) {
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
    bool v = false;
    bool any_v = false;

    for (int n = 0; n < 2; n++) {
        int op = n ? op1 : op0;
        int w = n ? w1 : w0;
        bool ov = false;
        int64_t val;

        if (mult ? !w : (op == 3 && !w)) {
            continue;
        }
        if (mult || op != 3) {
            uint16_t x = get_half(env, src0, n ? h01 : h00);
            uint16_t y = get_half(env, src1, n ? h11 : h10);
            int64_t prod = mac_product(x, y, mmod, n && mm, &v);

            if (mult) {
                val = prod;
            } else {
                int64_t acc = get_acc(env, n);

                val = op == 0 ? prod : op == 1 ? acc + prod : acc - prod;
                val = acc_sat(val, mmod, &ov);
                set_acc(env, n, val);
                set_av(env, n, ov);
            }
        } else {
            val = get_acc(env, n);
        }
        if (!w) {
            continue;
        }
        ov = false;
        if (p) {
            set_dreg(env, dst + n, extract_word(val, mmod, &ov));
        } else {
            set_half(env, dst, n, extract_half(env, val, mmod, &ov));
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
