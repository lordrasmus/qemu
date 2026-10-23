/*
 * QEMU Kalray kvx CPU: floating point and other compound-operation helpers
 *
 * Copyright (c) 2019-2020 GreenSocs SAS
 *
 * The semantics follow Kalray's kv3 behaviour model (LAO, kvx/kv3-Behavior.c
 * and CDT/BSL/Flip*.c, GPLv2 or LGPLv2.1).  IEEE arithmetic is done with
 * QEMU's softfloat; the reciprocal and square root seeds and the integer
 * operations are carried over from that model.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "cpu.h"
#include "exec/helper-proto.h"
#include "fpu/softfloat.h"

/*
 * The generated translator calls an operation helper and then fetches each
 * IEEE flag with a helper of its own (finexact() and friends), without env
 * on either side.  The status therefore lives here, per thread; kvx does
 * not run with MTTCG.
 */
static __thread float_status kvx_fp;
static __thread bool kvx_fp_ready;

static float_status *fp_status(void)
{
    if (!kvx_fp_ready) {
        set_float_detect_tininess(float_tininess_after_rounding, &kvx_fp);
        set_default_nan_mode(true, &kvx_fp);
        set_float_default_nan_pattern(0b01000000, &kvx_fp);
        set_float_2nan_prop_rule(float_2nan_prop_s_ab, &kvx_fp);
        set_float_3nan_prop_rule(float_3nan_prop_s_abc, &kvx_fp);
        set_float_infzeronan_rule(float_infzeronan_dnan_always, &kvx_fp);
        kvx_fp_ready = true;
    }
    return &kvx_fp;
}

/* kv3 rounding modifier: RN RU RD RZ RNA RNZ RO, 7 = none */
static void fp_round(uint64_t rm)
{
    static const FloatRoundMode modes[8] = {
        float_round_nearest_even, float_round_up, float_round_down,
        float_round_to_zero, float_round_ties_away, float_round_nearest_even,
        float_round_to_odd, float_round_nearest_even,
    };

    set_float_rounding_mode(modes[rm & 7], fp_status());
}

static float_status *fp_reset(void)
{
    float_status *s = fp_status();

    set_float_exception_flags(0, s);
    return s;
}

static float_status *fp_begin(uint64_t rm)
{
    fp_round(rm);
    return fp_reset();
}

static uint64_t fp_flag(int flag)
{
    return (get_float_exception_flags(fp_status()) & flag) != 0;
}

uint64_t HELPER(fdivbyzero)(void)
{
    return fp_flag(float_flag_divbyzero);
}

uint64_t HELPER(finexact)(void)
{
    return fp_flag(float_flag_inexact);
}

uint64_t HELPER(finvalid)(void)
{
    return fp_flag(float_flag_invalid);
}

uint64_t HELPER(foverflow)(void)
{
    return fp_flag(float_flag_overflow);
}

uint64_t HELPER(funderflow)(void)
{
    return fp_flag(float_flag_underflow);
}

/*
 * Element-wise arithmetic on a 64-bit register.
 */

enum { OP_ADD, OP_SBF, OP_MUL };

static uint64_t arith(int esize, int op, uint64_t x, uint64_t y,
                      float_status *s)
{
    switch (esize) {
    case 16:
        switch (op) {
        case OP_ADD:
            return float16_add(x, y, s);
        case OP_SBF:
            return float16_sub(y, x, s);
        default:
            return float16_mul(x, y, s);
        }
    case 32:
        switch (op) {
        case OP_ADD:
            return float32_add(x, y, s);
        case OP_SBF:
            return float32_sub(y, x, s);
        default:
            return float32_mul(x, y, s);
        }
    default:
        switch (op) {
        case OP_ADD:
            return float64_add(x, y, s);
        case OP_SBF:
            return float64_sub(y, x, s);
        default:
            return float64_mul(x, y, s);
        }
    }
}

static uint64_t vec_arith(int esize, int op, uint64_t x, uint64_t y,
                          float_status *s)
{
    uint64_t r = 0;

    for (int i = 0; i < 64; i += esize) {
        r = deposit64(r, i, esize, arith(esize, op, extract64(x, i, esize),
                                         extract64(y, i, esize), s));
    }
    return r;
}

/* c * b + a, the operand order of the kv3 FFMA* instructions */
static uint64_t fma_elt(int esize, uint64_t a, uint64_t b, uint64_t c,
                        bool sub, float_status *s)
{
    int flags = sub ? float_muladd_negate_product : 0;

    switch (esize) {
    case 16:
        return float16_muladd(c, b, a, flags, s);
    case 32:
        return float32_muladd(c, b, a, flags, s);
    default:
        return float64_muladd(c, b, a, flags, s);
    }
}

static uint64_t vec_fma(int esize, uint64_t a, uint64_t b, uint64_t c,
                        bool sub, float_status *s)
{
    uint64_t r = 0;

    for (int i = 0; i < 64; i += esize) {
        r = deposit64(r, i, esize,
                      fma_elt(esize, extract64(a, i, esize),
                              extract64(b, i, esize), extract64(c, i, esize),
                              sub, s));
    }
    return r;
}

/* widening c * b + a: half to single, or single to double */
static uint64_t fma_wide(int esize, uint64_t a, uint64_t b, uint64_t c,
                         bool sub, float_status *s)
{
    int flags = sub ? float_muladd_negate_product : 0;

    if (esize == 16) {
        return float32_muladd(float16_to_float32(c, true, s),
                              float16_to_float32(b, true, s), a, flags, s);
    }
    return float64_muladd(float32_to_float64(c, s), float32_to_float64(b, s),
                          a, flags, s);
}

static uint64_t mul_wide(int esize, uint64_t a, uint64_t b, float_status *s)
{
    if (esize == 16) {
        return float32_mul(float16_to_float32(a, true, s),
                           float16_to_float32(b, true, s), s);
    }
    return float64_mul(float32_to_float64(a, s), float32_to_float64(b, s), s);
}

/*
 * a + b rounded to odd, so that the conversion to the destination format
 * rounds only once.  An exact zero takes its sign from the real mode.
 */
static float128 add_ro(float128 a, float128 b, float_status *s)
{
    FloatRoundMode rm = get_float_rounding_mode(s);
    float128 r;

    set_float_rounding_mode(float_round_to_odd, s);
    r = float128_add(a, b, s);
    set_float_rounding_mode(rm, s);
    if (float128_is_zero(r)) {
        r = float128_add(a, b, s);
    }
    return r;
}

/* a * b + c * d (or a * b - c * d) on singles, rounded once */
static uint64_t fdma(uint32_t a, uint32_t b, uint32_t c, uint32_t d,
                     bool sub, bool wide, float_status *s)
{
    float128 p, q, r;

    if (float32_is_any_nan(a) || float32_is_any_nan(b) ||
        float32_is_any_nan(c) || float32_is_any_nan(d)) {
        if (float32_is_signaling_nan(a, s) || float32_is_signaling_nan(b, s) ||
            float32_is_signaling_nan(c, s) || float32_is_signaling_nan(d, s)) {
            float_raise(float_flag_invalid, s);
        }
        return wide ? float64_default_nan(s) : float32_default_nan(s);
    }
    p = float128_mul(float32_to_float128(a, s), float32_to_float128(b, s), s);
    q = float128_mul(float32_to_float128(c, s), float32_to_float128(d, s), s);
    if (sub) {
        q = float128_chs(q);
    }
    r = add_ro(p, q, s);
    return wide ? float128_to_float64(r, s) : float128_to_float32(r, s);
}

/* complex multiply of (r, i) pairs; conj negates the imaginary part of b */
static void cmul(uint64_t a, uint64_t b, bool conj, bool wide,
                 uint64_t *re, uint64_t *im, float_status *s)
{
    uint32_t ar = a, ai = a >> 32, br = b, bi = b >> 32;

    *re = fdma(ar, br, ai, bi, !conj, wide, s);
    *im = fdma(ar, bi, ai, br, conj, wide, s);
}

/*
 * Sum of four exact half-precision products plus a single accumulator,
 * rounded once.  The products are multiples of 2^-48 below 2^33, so their
 * sum is exact in float128.
 */
static uint32_t fdot4add(uint64_t a, uint64_t b, uint32_t acc, float_status *s)
{
    int saved = get_float_exception_flags(s);
    bool nan = float32_is_any_nan(acc);
    bool snan = float32_is_signaling_nan(acc, s);
    float128 sum = float128_zero;
    float128 r;

    for (int i = 0; i < 4; i++) {
        uint16_t x = extract64(a, 16 * i, 16), y = extract64(b, 16 * i, 16);
        float128 p;

        nan |= float16_is_any_nan(x) || float16_is_any_nan(y);
        snan |= float16_is_signaling_nan(x, s) || float16_is_signaling_nan(y, s);
        p = float128_mul(float32_to_float128(float16_to_float32(x, true, s), s),
                         float32_to_float128(float16_to_float32(y, true, s), s),
                         s);
        sum = i ? float128_add(sum, p, s) : p;
    }
    r = add_ro(sum, float32_to_float128(acc, s), s);
    if (nan && !snan && !(saved & float_flag_invalid)) {
        /* a quiet NaN input hides an invalid operation */
        set_float_exception_flags(get_float_exception_flags(s) &
                                  ~float_flag_invalid, s);
    }
    return float128_to_float32(r, s);
}

/*
 * Min, max and compare work on the encoding and never raise a flag.
 * A signalling NaN operand is returned as is.
 */

enum { CMP_UNORDERED, CMP_LT, CMP_EQ, CMP_GT };

static int frac_bits(int w)
{
    return w == 16 ? 10 : w == 32 ? 23 : 52;
}

static bool raw_is_nan(uint64_t a, int w)
{
    uint64_t abs = a & MAKE_64BIT_MASK(0, w - 1);

    return abs > (MAKE_64BIT_MASK(frac_bits(w), w - 1 - frac_bits(w)));
}

static bool raw_is_snan(uint64_t a, int w)
{
    return raw_is_nan(a, w) && !extract64(a, frac_bits(w) - 1, 1);
}

static bool raw_is_zero(uint64_t a, int w)
{
    return !(a & MAKE_64BIT_MASK(0, w - 1));
}

static int raw_compare(uint64_t a, uint64_t b, int w)
{
    bool sa = extract64(a, w - 1, 1), sb = extract64(b, w - 1, 1);

    if (raw_is_nan(a, w) || raw_is_nan(b, w)) {
        return CMP_UNORDERED;
    }
    if (sa != sb) {
        if (raw_is_zero(a, w) && raw_is_zero(b, w)) {
            return CMP_EQ;
        }
        return sa ? CMP_LT : CMP_GT;
    }
    if (a == b) {
        return CMP_EQ;
    }
    return (a > b) != sa ? CMP_GT : CMP_LT;
}

static uint64_t raw_minmax(uint64_t a, uint64_t b, int w, bool max)
{
    if (raw_is_nan(a, w)) {
        if (raw_is_snan(a, w)) {
            return a;
        }
        if (raw_is_snan(b, w)) {
            return b;
        }
        return raw_is_nan(b, w) ? a : b;
    }
    if (raw_is_nan(b, w)) {
        return raw_is_snan(b, w) ? b : a;
    }
    if (raw_is_zero(a, w) && raw_is_zero(b, w)) {
        /* min gives -0, max +0 */
        return extract64(a, w - 1, 1) != max ? a : b;
    }
    if (max) {
        return raw_compare(a, b, w) == CMP_GT ? a : b;
    }
    return raw_compare(a, b, w) == CMP_LT ? a : b;
}

static uint64_t vec_minmax(int esize, uint64_t x, uint64_t y, bool max)
{
    uint64_t r = 0;

    fp_reset();
    for (int i = 0; i < 64; i += esize) {
        r = deposit64(r, i, esize, raw_minmax(extract64(x, i, esize),
                                              extract64(y, i, esize),
                                              esize, max));
    }
    return r;
}

/* kv3 floatcomp modifier: ONE UEQ OEQ UNE OLT UGE OGE ULT */
static uint64_t floatcomp(uint64_t cond, uint64_t a, uint64_t b, int w)
{
    int c = raw_compare(extract64(a, 0, w), extract64(b, 0, w), w);

    fp_reset();
    switch (cond) {
    case 0:
        return c != CMP_UNORDERED && c != CMP_EQ;
    case 1:
        return c == CMP_UNORDERED || c == CMP_EQ;
    case 2:
        return c == CMP_EQ;
    case 3:
        return c != CMP_EQ;
    case 4:
        return c == CMP_LT;
    case 5:
        return c == CMP_UNORDERED || c == CMP_GT || c == CMP_EQ;
    case 6:
        return c == CMP_GT || c == CMP_EQ;
    case 7:
        return c == CMP_UNORDERED || c == CMP_LT;
    default:
        g_assert_not_reached();
    }
}

uint64_t HELPER(floatcomp_16)(uint64_t cond, uint64_t a, uint64_t b)
{
    return floatcomp(cond, a, b, 16);
}

uint64_t HELPER(floatcomp_32)(uint64_t cond, uint64_t a, uint64_t b)
{
    return floatcomp(cond, a, b, 32);
}

uint64_t HELPER(floatcomp_64)(uint64_t cond, uint64_t a, uint64_t b)
{
    return floatcomp(cond, a, b, 64);
}

uint64_t HELPER(fmin_64)(uint64_t a, uint64_t b)
{
    return vec_minmax(64, a, b, false);
}

uint64_t HELPER(fmax_64)(uint64_t a, uint64_t b)
{
    return vec_minmax(64, a, b, true);
}

uint64_t HELPER(fmin_32)(uint64_t a, uint64_t b)
{
    return vec_minmax(32, (uint32_t)a, (uint32_t)b, false) & UINT32_MAX;
}

uint64_t HELPER(fmax_32)(uint64_t a, uint64_t b)
{
    return vec_minmax(32, (uint32_t)a, (uint32_t)b, true) & UINT32_MAX;
}

uint64_t HELPER(fmin_32_x2)(uint64_t a, uint64_t b)
{
    return vec_minmax(32, a, b, false);
}

uint64_t HELPER(fmax_32_x2)(uint64_t a, uint64_t b)
{
    return vec_minmax(32, a, b, true);
}

uint64_t HELPER(fmin_16_x4)(uint64_t a, uint64_t b)
{
    return vec_minmax(16, a, b, false);
}

uint64_t HELPER(fmax_16_x4)(uint64_t a, uint64_t b)
{
    return vec_minmax(16, a, b, true);
}

void HELPER(frelu_32_32_x8)(CPUKVXState *env, uint64_t a0, uint64_t a1,
                            uint64_t a2, uint64_t a3)
{
    uint64_t in[4] = { a0, a1, a2, a3 };

    for (int i = 0; i < 4; i++) {
        uint64_t r = 0;

        for (int j = 0; j < 64; j += 32) {
            uint64_t x = extract64(in[i], j, 32);
            uint64_t y = raw_minmax(x, 0x80000000, 32, true);

            if (raw_is_nan(x, 32) || raw_is_nan(y, 32)) {
                y = float32_default_nan(fp_status());
            }
            r = deposit64(r, j, 32, y);
        }
        env->scratch[i] = r;
    }
}

/*
 * Arithmetic
 */

uint64_t HELPER(fadd_64_64)(uint64_t rm, uint64_t a, uint64_t b)
{
    return vec_arith(64, OP_ADD, a, b, fp_begin(rm));
}

uint64_t HELPER(fadd_32_32)(uint64_t rm, uint64_t a, uint64_t b)
{
    return float32_add(a, b, fp_begin(rm));
}

uint64_t HELPER(fadd_32_32_x2)(uint64_t rm, uint64_t a, uint64_t b)
{
    return vec_arith(32, OP_ADD, a, b, fp_begin(rm));
}

uint64_t HELPER(fadd_16_16_x4)(uint64_t rm, uint64_t a, uint64_t b)
{
    return vec_arith(16, OP_ADD, a, b, fp_begin(rm));
}

uint64_t HELPER(fsbf_64_64)(uint64_t rm, uint64_t a, uint64_t b)
{
    return vec_arith(64, OP_SBF, a, b, fp_begin(rm));
}

uint64_t HELPER(fsbf_32_32)(uint64_t rm, uint64_t a, uint64_t b)
{
    return float32_sub(b, a, fp_begin(rm));
}

uint64_t HELPER(fsbf_32_32_x2)(uint64_t rm, uint64_t a, uint64_t b)
{
    return vec_arith(32, OP_SBF, a, b, fp_begin(rm));
}

uint64_t HELPER(fsbf_16_16_x4)(uint64_t rm, uint64_t a, uint64_t b)
{
    return vec_arith(16, OP_SBF, a, b, fp_begin(rm));
}

uint64_t HELPER(fmul_64_64)(uint64_t rm, uint64_t a, uint64_t b)
{
    return vec_arith(64, OP_MUL, a, b, fp_begin(rm));
}

uint64_t HELPER(fmul_32_32)(uint64_t rm, uint64_t a, uint64_t b)
{
    return float32_mul(a, b, fp_begin(rm));
}

uint64_t HELPER(fmul_32_32_x2)(uint64_t rm, uint64_t a, uint64_t b)
{
    return vec_arith(32, OP_MUL, a, b, fp_begin(rm));
}

uint64_t HELPER(fmul_16_16_x4)(uint64_t rm, uint64_t a, uint64_t b)
{
    return vec_arith(16, OP_MUL, a, b, fp_begin(rm));
}

/* 128-bit forms: both halves under one rounding mode and flag set */
#define ARITH128(name, esize, op)                                           \
void HELPER(name)(CPUKVXState *env, uint64_t rm, uint64_t a_l,              \
                  uint64_t a_h, uint64_t b_l, uint64_t b_h)                 \
{                                                                           \
    float_status *s = fp_begin(rm);                                         \
                                                                            \
    env->scratch[0] = vec_arith(esize, op, a_l, b_l, s);                    \
    env->scratch[1] = vec_arith(esize, op, a_h, b_h, s);                    \
}

ARITH128(fadd_16_16_x8, 16, OP_ADD)
ARITH128(fadd_32_32_x4, 32, OP_ADD)
ARITH128(fadd_64_64_x2, 64, OP_ADD)
ARITH128(fsbf_16_16_x8, 16, OP_SBF)
ARITH128(fsbf_32_32_x4, 32, OP_SBF)
ARITH128(fsbf_64_64_x2, 64, OP_SBF)
ARITH128(fmul_16_16_x8, 16, OP_MUL)
ARITH128(fmul_32_32_x4, 32, OP_MUL)

/* complex add/sub: (r, i) pairs, the imaginary part of the first conjugated */
static uint64_t caddsub32(uint64_t a, uint64_t b, bool sbf, float_status *s)
{
    uint32_t ar = a, ai = a >> 32, br = b, bi = b >> 32;
    uint32_t r, i;

    if (sbf) {
        r = float32_sub(br, ar, s);
        i = float32_add(ai, bi, s);
    } else {
        r = float32_add(ar, br, s);
        i = float32_sub(bi, ai, s);
    }
    return deposit64(r, 32, 32, i);
}

uint64_t HELPER(faddcc_32_32)(uint64_t rm, uint64_t a, uint64_t b)
{
    return caddsub32(a, b, false, fp_begin(rm));
}

uint64_t HELPER(fsbfcc_32_32)(uint64_t rm, uint64_t a, uint64_t b)
{
    return caddsub32(a, b, true, fp_begin(rm));
}

void HELPER(faddcc_32_32_x2)(CPUKVXState *env, uint64_t rm, uint64_t a_l,
                             uint64_t a_h, uint64_t b_l, uint64_t b_h)
{
    float_status *s = fp_begin(rm);

    env->scratch[0] = caddsub32(a_l, b_l, false, s);
    env->scratch[1] = caddsub32(a_h, b_h, false, s);
}

void HELPER(fsbfcc_32_32_x2)(CPUKVXState *env, uint64_t rm, uint64_t a_l,
                             uint64_t a_h, uint64_t b_l, uint64_t b_h)
{
    float_status *s = fp_begin(rm);

    env->scratch[0] = caddsub32(a_l, b_l, true, s);
    env->scratch[1] = caddsub32(a_h, b_h, true, s);
}

void HELPER(faddcc_64_64)(CPUKVXState *env, uint64_t rm, uint64_t a_r,
                          uint64_t a_i, uint64_t b_r, uint64_t b_i)
{
    float_status *s = fp_begin(rm);

    env->scratch[0] = float64_add(a_r, b_r, s);
    env->scratch[1] = float64_sub(b_i, a_i, s);
}

void HELPER(fsbfcc_64_64)(CPUKVXState *env, uint64_t rm, uint64_t a_r,
                          uint64_t a_i, uint64_t b_r, uint64_t b_i)
{
    float_status *s = fp_begin(rm);

    env->scratch[0] = float64_sub(b_r, a_r, s);
    env->scratch[1] = float64_add(a_i, b_i, s);
}

/* widening multiplies */
uint64_t HELPER(fmul_16_32)(uint64_t rm, uint64_t a, uint64_t b)
{
    return mul_wide(16, (uint16_t)a, (uint16_t)b, fp_begin(rm));
}

uint64_t HELPER(fmul_32_64)(uint64_t rm, uint64_t a, uint64_t b)
{
    return mul_wide(32, (uint32_t)a, (uint32_t)b, fp_begin(rm));
}

void HELPER(fmul_16_32_x4)(CPUKVXState *env, uint64_t rm, uint64_t a,
                           uint64_t b)
{
    float_status *s = fp_begin(rm);

    for (int i = 0; i < 2; i++) {
        uint64_t r = 0;

        for (int j = 0; j < 2; j++) {
            int k = 32 * i + 16 * j;

            r = deposit64(r, 32 * j, 32, mul_wide(16, extract64(a, k, 16),
                                                  extract64(b, k, 16), s));
        }
        env->scratch[i] = r;
    }
}

void HELPER(fmul_32_64_x2)(CPUKVXState *env, uint64_t rm, uint64_t a,
                           uint64_t b)
{
    float_status *s = fp_begin(rm);

    env->scratch[0] = mul_wide(32, (uint32_t)a, (uint32_t)b, s);
    env->scratch[1] = mul_wide(32, a >> 32, b >> 32, s);
}

/* outer product of two single pairs */
void HELPER(fmm2wq)(CPUKVXState *env, uint64_t rm, uint64_t a, uint64_t b)
{
    float_status *s = fp_begin(rm);
    uint32_t ax = a, ay = a >> 32, bx = b, by = b >> 32;

    env->scratch[0] = deposit64(float32_mul(ax, bx, s), 32, 32,
                                float32_mul(ax, by, s));
    env->scratch[1] = deposit64(float32_mul(ay, bx, s), 32, 32,
                                float32_mul(ay, by, s));
}

static void fmm2(CPUKVXState *env, uint64_t rm, uint64_t acc_l, uint64_t acc_h,
                 uint64_t b, uint64_t c, bool sub)
{
    float_status *s = fp_begin(rm);
    uint32_t bx = b, by = b >> 32, cx = c, cy = c >> 32;

    env->scratch[0] = deposit64(fma_elt(32, (uint32_t)acc_l, bx, cx, sub, s),
                                32, 32,
                                fma_elt(32, acc_l >> 32, bx, cy, sub, s));
    env->scratch[1] = deposit64(fma_elt(32, (uint32_t)acc_h, by, cx, sub, s),
                                32, 32,
                                fma_elt(32, acc_h >> 32, by, cy, sub, s));
}

void HELPER(fmm2awq)(CPUKVXState *env, uint64_t rm, uint64_t acc_l,
                     uint64_t acc_h, uint64_t b, uint64_t c)
{
    fmm2(env, rm, acc_l, acc_h, b, c, false);
}

void HELPER(fmm2swq)(CPUKVXState *env, uint64_t rm, uint64_t acc_l,
                     uint64_t acc_h, uint64_t b, uint64_t c)
{
    fmm2(env, rm, acc_l, acc_h, b, c, true);
}

/* complex multiplies and dot products, each element rounded once */
uint64_t HELPER(fmulc_32_32)(uint64_t rm, uint64_t a, uint64_t b)
{
    uint64_t re, im;

    cmul(a, b, false, false, &re, &im, fp_begin(rm));
    return deposit64(re, 32, 32, im);
}

uint64_t HELPER(fmulcc_32_32)(uint64_t rm, uint64_t a, uint64_t b)
{
    uint64_t re, im;

    cmul(a, b, true, false, &re, &im, fp_begin(rm));
    return deposit64(re, 32, 32, im);
}

void HELPER(fmulc_32_64)(CPUKVXState *env, uint64_t rm, uint64_t a,
                         uint64_t b)
{
    cmul(a, b, false, true, &env->scratch[0], &env->scratch[1], fp_begin(rm));
}

void HELPER(fmulcc_32_64)(CPUKVXState *env, uint64_t rm, uint64_t a,
                          uint64_t b)
{
    cmul(a, b, true, true, &env->scratch[0], &env->scratch[1], fp_begin(rm));
}

uint64_t HELPER(fdot2_32_32)(uint64_t rm, uint64_t a, uint64_t b)
{
    return fdma(a, b, a >> 32, b >> 32, false, false, fp_begin(rm));
}

uint64_t HELPER(fdot2_32_64)(uint64_t rm, uint64_t a, uint64_t b)
{
    return fdma(a, b, a >> 32, b >> 32, false, true, fp_begin(rm));
}

void HELPER(fdot2_32_32_x2)(CPUKVXState *env, uint64_t rm, uint64_t a_l,
                            uint64_t a_h, uint64_t b_l, uint64_t b_h)
{
    float_status *s = fp_begin(rm);

    env->scratch[0] = fdma(a_l, b_l, a_l >> 32, b_l >> 32, false, false, s);
    env->scratch[1] = fdma(a_h, b_h, a_h >> 32, b_h >> 32, false, false, s);
}

void HELPER(fdot2_32_64_x2)(CPUKVXState *env, uint64_t rm, uint64_t a_l,
                            uint64_t a_h, uint64_t b_l, uint64_t b_h)
{
    float_status *s = fp_begin(rm);

    env->scratch[0] = fdma(a_l, b_l, a_l >> 32, b_l >> 32, false, true, s);
    env->scratch[1] = fdma(a_h, b_h, a_h >> 32, b_h >> 32, false, true, s);
}

uint64_t HELPER(fdot4add_16_32_reset)(uint64_t rm, uint64_t a, uint64_t b,
                                      uint64_t acc)
{
    return fdot4add(a, b, acc, fp_begin(rm));
}

/* second half of a split operation: keeps the flags of the first */
uint64_t HELPER(fdot4add_16_32_noreset)(uint64_t rm, uint64_t a, uint64_t b,
                                        uint64_t acc)
{
    fp_round(rm);
    return fdot4add(a, b, acc, fp_status());
}

/*
 * Fused multiply-add: a + c * b, FFMS a - c * b
 */

uint64_t HELPER(ffma_64_64)(uint64_t rm, uint64_t a, uint64_t b, uint64_t c)
{
    return vec_fma(64, a, b, c, false, fp_begin(rm));
}

uint64_t HELPER(ffma_32_32)(uint64_t rm, uint64_t a, uint64_t b, uint64_t c)
{
    return fma_elt(32, (uint32_t)a, (uint32_t)b, (uint32_t)c, false,
                   fp_begin(rm));
}

uint64_t HELPER(ffma_32_32_x2)(uint64_t rm, uint64_t a, uint64_t b,
                               uint64_t c)
{
    return vec_fma(32, a, b, c, false, fp_begin(rm));
}

uint64_t HELPER(ffma_16_16_x4)(uint64_t rm, uint64_t a, uint64_t b,
                               uint64_t c)
{
    return vec_fma(16, a, b, c, false, fp_begin(rm));
}

uint64_t HELPER(ffms_64_64)(uint64_t rm, uint64_t a, uint64_t b, uint64_t c)
{
    return vec_fma(64, a, b, c, true, fp_begin(rm));
}

uint64_t HELPER(ffms_32_32)(uint64_t rm, uint64_t a, uint64_t b, uint64_t c)
{
    return fma_elt(32, (uint32_t)a, (uint32_t)b, (uint32_t)c, true,
                   fp_begin(rm));
}

uint64_t HELPER(ffms_32_32_x2)(uint64_t rm, uint64_t a, uint64_t b,
                               uint64_t c)
{
    return vec_fma(32, a, b, c, true, fp_begin(rm));
}

uint64_t HELPER(ffms_16_16_x4)(uint64_t rm, uint64_t a, uint64_t b,
                               uint64_t c)
{
    return vec_fma(16, a, b, c, true, fp_begin(rm));
}

uint64_t HELPER(ffma_16_32)(uint64_t rm, uint64_t a, uint64_t b, uint64_t c)
{
    return fma_wide(16, (uint32_t)a, (uint16_t)b, (uint16_t)c, false,
                    fp_begin(rm));
}

uint64_t HELPER(ffms_16_32)(uint64_t rm, uint64_t a, uint64_t b, uint64_t c)
{
    return fma_wide(16, (uint32_t)a, (uint16_t)b, (uint16_t)c, true,
                    fp_begin(rm));
}

uint64_t HELPER(ffma_32_64)(uint64_t rm, uint64_t a, uint64_t b, uint64_t c)
{
    return fma_wide(32, a, (uint32_t)b, (uint32_t)c, false, fp_begin(rm));
}

uint64_t HELPER(ffms_32_64)(uint64_t rm, uint64_t a, uint64_t b, uint64_t c)
{
    return fma_wide(32, a, (uint32_t)b, (uint32_t)c, true, fp_begin(rm));
}

/* 128-bit accumulator, two 64-bit factors of narrower elements */
static void fma_wide128(CPUKVXState *env, int esize, uint64_t rm,
                        uint64_t a_l, uint64_t a_h, uint64_t b, uint64_t c,
                        bool sub)
{
    float_status *s = fp_begin(rm);
    uint64_t acc[2] = { a_l, a_h };
    int wsize = 2 * esize;

    for (int i = 0; i < 2; i++) {
        uint64_t r = 0;

        for (int j = 0; j < 64; j += wsize) {
            int k = 32 * i + j / 2;

            r = deposit64(r, j, wsize,
                          fma_wide(esize, extract64(acc[i], j, wsize),
                                   extract64(b, k, esize),
                                   extract64(c, k, esize), sub, s));
        }
        env->scratch[i] = r;
    }
}

void HELPER(ffma_16_32_x4)(CPUKVXState *env, uint64_t rm, uint64_t a_l,
                           uint64_t a_h, uint64_t b, uint64_t c)
{
    fma_wide128(env, 16, rm, a_l, a_h, b, c, false);
}

void HELPER(ffms_16_32_x4)(CPUKVXState *env, uint64_t rm, uint64_t a_l,
                           uint64_t a_h, uint64_t b, uint64_t c)
{
    fma_wide128(env, 16, rm, a_l, a_h, b, c, true);
}

void HELPER(ffma_32_64_x2)(CPUKVXState *env, uint64_t rm, uint64_t a_l,
                           uint64_t a_h, uint64_t b, uint64_t c)
{
    fma_wide128(env, 32, rm, a_l, a_h, b, c, false);
}

void HELPER(ffms_32_64_x2)(CPUKVXState *env, uint64_t rm, uint64_t a_l,
                           uint64_t a_h, uint64_t b, uint64_t c)
{
    fma_wide128(env, 32, rm, a_l, a_h, b, c, true);
}

/* all three operands 128 bits wide; the third comes in through scratch */
static void fma128(CPUKVXState *env, int esize, uint64_t rm, uint64_t a_l,
                   uint64_t a_h, uint64_t b_l, uint64_t b_h, bool sub)
{
    float_status *s = fp_begin(rm);
    uint64_t c_l = env->scratch[0], c_h = env->scratch[1];

    env->scratch[0] = vec_fma(esize, a_l, b_l, c_l, sub, s);
    env->scratch[1] = vec_fma(esize, a_h, b_h, c_h, sub, s);
}

void HELPER(ffma_16_16_x8)(CPUKVXState *env, uint64_t rm, uint64_t a_l,
                           uint64_t a_h, uint64_t b_l, uint64_t b_h)
{
    fma128(env, 16, rm, a_l, a_h, b_l, b_h, false);
}

void HELPER(ffma_32_32_x4)(CPUKVXState *env, uint64_t rm, uint64_t a_l,
                           uint64_t a_h, uint64_t b_l, uint64_t b_h)
{
    fma128(env, 32, rm, a_l, a_h, b_l, b_h, false);
}

void HELPER(ffms_16_16_x8)(CPUKVXState *env, uint64_t rm, uint64_t a_l,
                           uint64_t a_h, uint64_t b_l, uint64_t b_h)
{
    fma128(env, 16, rm, a_l, a_h, b_l, b_h, true);
}

void HELPER(ffms_32_32_x4)(CPUKVXState *env, uint64_t rm, uint64_t a_l,
                           uint64_t a_h, uint64_t b_l, uint64_t b_h)
{
    fma128(env, 32, rm, a_l, a_h, b_l, b_h, true);
}

/*
 * Reciprocal and reciprocal square root
 */

uint64_t HELPER(frec_32)(uint64_t rm, uint64_t a)
{
    return float32_div(float32_one, a, fp_begin(rm));
}

uint64_t HELPER(frsq_32)(uint64_t rm, uint64_t a)
{
    float_status *s = fp_reset();
    float64 r;

    /* the intermediate steps round to nearest, only the result to rm */
    set_float_rounding_mode(float_round_nearest_even, s);
    r = float64_div(float64_one, float64_sqrt(float32_to_float64(a, s), s), s);
    fp_round(rm);
    return float64_to_float32(r, s);
}

/*
 * FSREC, FSDIV, FCDIV, FSRSR: seeds and special-case bits for the software
 * division and square root sequences, defined by the hardware tables.
 * behaviour 1 is FSDIV/FSREC, 2 is FCDIV.
 */

static const uint16_t recip_table[128] = {
    508, 500, 492, 485, 477, 470, 463, 455, 448, 441, 434, 428, 421, 414, 408,
    401, 395, 389, 383, 377, 371, 365, 359, 353, 347, 342, 336, 331, 326, 320,
    315, 310, 305, 300, 295, 290, 285, 280, 275, 271, 266, 261, 257, 252, 248,
    243, 239, 235, 231, 226, 222, 218, 214, 210, 206, 202, 198, 195, 191, 187,
    183, 180, 176, 172, 169, 165, 162, 158, 155, 152, 148, 145, 142, 138, 135,
    132, 129, 126, 123, 120, 117, 114, 111, 108, 105, 102, 99, 96, 93, 91, 88,
    85, 82, 80, 77, 74, 72, 69, 67, 64, 62, 59, 57, 54, 52, 49, 47, 45, 42, 40,
    38, 35, 33, 31, 29, 26, 24, 22, 20, 18, 15, 13, 11, 9, 7, 5, 3, 0,
};

static const uint16_t rsqrt_table[128] = {
    209, 204, 198, 193, 188, 183, 178, 173, 168, 164, 159, 155, 150, 146, 142,
    138, 134, 130, 126, 122, 118, 114, 111, 107, 104, 100, 97, 94, 90, 87, 84,
    81, 78, 75, 72, 69, 66, 63, 60, 57, 55, 52, 49, 47, 44, 42, 39, 37, 34, 32,
    29, 27, 25, 22, 20, 18, 16, 14, 11, 9, 7, 5, 3, 1, 508, 500, 493, 485, 478,
    471, 464, 457, 450, 444, 437, 431, 425, 419, 413, 407, 401, 395, 390, 384,
    379, 374, 369, 364, 359, 354, 349, 344, 340, 335, 331, 326, 322, 318, 313,
    309, 305, 301, 297, 293, 289, 286, 282, 278, 274, 271, 267, 264, 260, 257,
    254, 250, 247, 244, 241, 237, 234, 231, 228, 225, 222, 219, 216, 213,
};

/* w is 32 or 64; ebits/fbits the field widths of that format */
static uint64_t seed_inv(uint64_t A, uint64_t B, int behavior, int w)
{
    float_status *s = fp_status();
    int fbits = w == 32 ? 23 : 52, ebits = w - 1 - fbits;
    uint64_t emax = MAKE_64BIT_MASK(0, ebits);
    uint64_t sign = 1ULL << (w - 1), qbit = 1ULL << (fbits - 1);
    uint64_t mask = ~(uint64_t)0x201 & MAKE_64BIT_MASK(0, w);
    uint64_t sgnA = A & sign, sgnB = B & sign;
    uint64_t sigA = extract64(A, 0, fbits), sigB = extract64(B, 0, fbits);
    int expA = extract64(A, fbits, ebits), expB = extract64(B, fbits, ebits);
    bool AisInfOrNaN = expA == emax, BisInfOrNaN = expB == emax;
    bool AisZero = !(A & (sign - 1)), BisZero = !(B & (sign - 1));
    bool AisNaN = AisInfOrNaN && sigA, BisNaN = BisInfOrNaN && sigB;
    bool BisInf = BisInfOrNaN && !sigB;
    /* the thresholds of the single format, scaled to the double one */
    int lim6 = w == 32 ? 26 : 55, lim7 = w == 32 ? -124 : -1020;
    int lim8 = w == 32 ? 127 : 1023, lim11 = w == 32 ? 251 : 2043;
    int ovf = w == 32 ? 128 : 1024;
    uint64_t rexp, bits;
    bool b0, b1, b2, b3, b4, b5, b6, b7, b8, b11, b12;

    rexp = extract64(emax - 2 - expB, 0, ebits);
    b1 = expB == 0;
    b2 = expA == 0;
    b3 = BisInfOrNaN;
    b4 = expB == emax - 2 || expB == emax - 1;
    b5 = AisInfOrNaN;
    b6 = expA != 0 && expA < lim6;
    b7 = expA - expB <= lim7;
    b8 = expA - expB >= lim8;
    b11 = behavior == 2 && expB >= lim11;
    b12 = behavior == 2 && (b1 || b3 || b4);
    b0 = !(b1 || b2 || b3 || b4 || b5 || b6 || b7 || b8 || b11 || b12);
    bits = b0 | b1 << 1 | b2 << 2 | b3 << 3 | b4 << 4 | b5 << 5 | b6 << 6 |
           b7 << 7 | b8 << 8 | (uint64_t)!b0 << 9 | (uint64_t)b11 << 11 |
           (uint64_t)b12 << 12;

    if (AisNaN) {
        if (!(sigA & qbit) || (BisNaN && !(sigB & qbit))) {
            float_raise(float_flag_invalid, s);
        }
        return (A | qbit) & mask;
    }
    if (BisNaN) {
        if (!(sigB & qbit)) {
            float_raise(float_flag_invalid, s);
        }
        return (B | qbit) & mask;
    }
    if (AisInfOrNaN) {
        if (BisZero) {
            return B ^ A;
        }
        if (BisInf) {
            float_raise(float_flag_invalid, s);
            return mask;
        }
        return sgnB ^ A;
    }
    if (BisInfOrNaN) {
        return sgnA ^ sgnB;
    }
    if (BisZero && !AisZero) {
        float_raise(float_flag_divbyzero, s);
        return (sgnA ^ sgnB) | emax << fbits;
    }
    if (expA - expB > ovf) {
        float_raise(float_flag_overflow | float_flag_inexact, s);
        return (sgnA ^ sgnB) | 0x600;
    }
    if (BisZero) {
        float_raise(float_flag_invalid, s);
        return mask;
    }
    if (AisZero) {
        return (sgnA ^ sgnB) & mask;
    }
    return sgnB | rexp << fbits |
           (uint64_t)recip_table[sigB >> (fbits - 7)] << (fbits - 9) | bits;
}

static uint64_t seed_rsqrt(uint64_t X, int w)
{
    int fbits = w == 32 ? 23 : 52, ebits = w - 1 - fbits;
    uint64_t emax = MAKE_64BIT_MASK(0, ebits), bias = emax >> 1;
    uint64_t sign = X & (1ULL << (w - 1));
    uint64_t exp = extract64(X, fbits, ebits), sig = extract64(X, 0, fbits);
    bool normal = (X & ((1ULL << (w - 1)) - 1)) > MAKE_64BIT_MASK(0, fbits);
    bool zero = !normal && !sig;
    bool nan = exp == emax && sig, inf = exp == emax && !sig;
    uint64_t rexp = ((exp & 1 ? 3 * bias - 2 : 3 * bias - 1) - exp) >> 1;
    bool b1 = zero, b2 = inf && !sign, b3 = nan || (sign && !zero);
    bool b4 = exp <= (w == 32 ? 26 : 55);
    bool b0 = !(b1 || b2 || b3 || b4 || !normal);

    return sign | rexp << fbits |
           (uint64_t)rsqrt_table[extract64(X, fbits - 6, 7)] << (fbits - 9) |
           b4 << 4 | b3 << 3 | b2 << 2 | b1 << 1 | b0;
}

uint64_t HELPER(fsrec_64)(uint64_t a)
{
    fp_reset();
    return seed_inv(float64_one, a, 1, 64);
}

uint64_t HELPER(fsrec_32)(uint64_t a)
{
    fp_reset();
    return seed_inv(float32_one, (uint32_t)a, 1, 32);
}

uint64_t HELPER(fsrec_32_x2)(uint64_t a)
{
    fp_reset();
    return deposit64(seed_inv(float32_one, (uint32_t)a, 1, 32), 32, 32,
                     seed_inv(float32_one, a >> 32, 1, 32));
}

uint64_t HELPER(fsdiv_64)(uint64_t a, uint64_t b)
{
    fp_reset();
    return seed_inv(a, b, 1, 64);
}

uint64_t HELPER(fcdiv_64)(uint64_t a, uint64_t b)
{
    fp_reset();
    return seed_inv(a, b, 2, 64);
}

uint64_t HELPER(fsdiv_32)(uint64_t a, uint64_t b)
{
    fp_reset();
    return seed_inv((uint32_t)a, (uint32_t)b, 1, 32);
}

uint64_t HELPER(fcdiv_32)(uint64_t a, uint64_t b)
{
    fp_reset();
    return seed_inv((uint32_t)a, (uint32_t)b, 2, 32);
}

uint64_t HELPER(fsdiv_32_x2)(uint64_t a, uint64_t b)
{
    fp_reset();
    return deposit64(seed_inv((uint32_t)a, (uint32_t)b, 1, 32), 32, 32,
                     seed_inv(a >> 32, b >> 32, 1, 32));
}

uint64_t HELPER(fcdiv_32_x2)(uint64_t a, uint64_t b)
{
    fp_reset();
    return deposit64(seed_inv((uint32_t)a, (uint32_t)b, 2, 32), 32, 32,
                     seed_inv(a >> 32, b >> 32, 2, 32));
}

uint64_t HELPER(fsrsr_64)(uint64_t a)
{
    fp_reset();
    return seed_rsqrt(a, 64);
}

uint64_t HELPER(fsrsr_32)(uint64_t a)
{
    fp_reset();
    return seed_rsqrt((uint32_t)a, 32);
}

uint64_t HELPER(fsrsr_32_x2)(uint64_t a)
{
    fp_reset();
    return deposit64(seed_rsqrt((uint32_t)a, 32), 32, 32,
                     seed_rsqrt(a >> 32, 32));
}

/*
 * Conversions between formats
 */

uint64_t HELPER(fwiden_16_32)(uint64_t a)
{
    return float16_to_float32(a, true, fp_reset());
}

uint64_t HELPER(fwiden_16_32_x2)(uint64_t a)
{
    float_status *s = fp_reset();

    return deposit64(float16_to_float32(a, true, s), 32, 32,
                     float16_to_float32(a >> 16, true, s));
}

uint64_t HELPER(fwiden_32_64)(uint64_t a)
{
    return float32_to_float64(a, fp_reset());
}

uint64_t HELPER(fnarrow_32_16)(uint64_t rm, uint64_t a)
{
    return float32_to_float16(a, true, fp_begin(rm));
}

uint64_t HELPER(fnarrow_32_16_x4)(uint64_t rm, uint64_t a_l, uint64_t a_h)
{
    float_status *s = fp_begin(rm);
    uint64_t in[2] = { a_l, a_h }, r = 0;

    for (int i = 0; i < 4; i++) {
        r = deposit64(r, 16 * i, 16,
                      float32_to_float16(extract64(in[i / 2], 32 * (i & 1), 32),
                                         true, s));
    }
    return r;
}

static void fnarrow_x8(CPUKVXState *env, uint64_t a0, uint64_t a1,
                       uint64_t a2, uint64_t a3)
{
    float_status *s = fp_status();
    uint64_t in[4] = { a0, a1, a2, a3 }, r[2] = { 0, 0 };

    for (int i = 0; i < 8; i++) {
        r[i / 4] = deposit64(r[i / 4], 16 * (i & 3), 16,
                             float32_to_float16(extract64(in[i / 2],
                                                          32 * (i & 1), 32),
                                                true, s));
    }
    env->scratch[0] = r[0];
    env->scratch[1] = r[1];
    env->scratch[2] = 0;
    env->scratch[3] = 0;
}

void HELPER(fnarrow_32_16_x8_reset)(CPUKVXState *env, uint64_t rm,
                                    uint64_t a0, uint64_t a1, uint64_t a2,
                                    uint64_t a3)
{
    fp_begin(rm);
    fnarrow_x8(env, a0, a1, a2, a3);
}

void HELPER(fnarrow_32_16_x8_noreset)(CPUKVXState *env, uint64_t rm,
                                      uint64_t a0, uint64_t a1, uint64_t a2,
                                      uint64_t a3)
{
    fp_round(rm);
    fnarrow_x8(env, a0, a1, a2, a3);
}

uint64_t HELPER(fnarrow_64_32)(uint64_t rm, uint64_t a)
{
    return float64_to_float32(a, fp_begin(rm));
}

uint64_t HELPER(fnarrow_64_32_x2)(uint64_t rm, uint64_t a_l, uint64_t a_h)
{
    float_status *s = fp_begin(rm);

    return deposit64(float64_to_float32(a_l, s), 32, 32,
                     float64_to_float32(a_h, s));
}

/* each element multiplied by 2^k, k wrapping in the 8-bit exponent field */
void HELPER(fpow2scale_32_32_x8)(CPUKVXState *env, uint64_t rm, uint64_t k,
                                 uint64_t a0, uint64_t a1, uint64_t a2,
                                 uint64_t a3)
{
    float_status *s = fp_begin(rm);
    uint64_t in[4] = { a0, a1, a2, a3 };
    float32 scale = extract32(127 + (uint32_t)k, 0, 8) << 23;

    for (int i = 0; i < 4; i++) {
        env->scratch[i] =
            deposit64(float32_mul(in[i], scale, s), 32, 32,
                      float32_mul(in[i] >> 32, scale, s));
    }
}

/*
 * Conversions between floating and fixed point: FIXED converts x * 2^scale
 * to an integer, FLOAT converts n / 2^scale to floating point.
 * A NaN, and for the unsigned forms any negative non-zero value, gives 0
 * and raises invalid.
 */

static uint64_t fixed32(uint32_t x, uint32_t scale, bool uns, float_status *s)
{
    FloatRoundMode m = get_float_rounding_mode(s);

    if (float32_is_any_nan(x) ||
        (uns && float32_is_neg(x) && !float32_is_zero(x))) {
        float_raise(float_flag_invalid, s);
        return 0;
    }
    if (uns) {
        return float32_to_uint32_scalbn(x, m, scale, s);
    }
    return (uint32_t)float32_to_int32_scalbn(x, m, scale, s);
}

static uint64_t fixed64(uint64_t x, uint32_t scale, bool uns, float_status *s)
{
    FloatRoundMode m = get_float_rounding_mode(s);

    if (float64_is_any_nan(x) ||
        (uns && float64_is_neg(x) && !float64_is_zero(x))) {
        float_raise(float_flag_invalid, s);
        return 0;
    }
    if (uns) {
        return float64_to_uint64_scalbn(x, m, scale, s);
    }
    return float64_to_int64_scalbn(x, m, scale, s);
}

uint64_t HELPER(fixed_64)(uint64_t rm, uint64_t a, uint64_t scale)
{
    return fixed64(a, scale, false, fp_begin(rm));
}

uint64_t HELPER(fixedu_64)(uint64_t rm, uint64_t a, uint64_t scale)
{
    return fixed64(a, scale, true, fp_begin(rm));
}

uint64_t HELPER(fixed_32)(uint64_t rm, uint64_t a, uint64_t scale)
{
    return fixed32(a, scale, false, fp_begin(rm));
}

uint64_t HELPER(fixedu_32)(uint64_t rm, uint64_t a, uint64_t scale)
{
    return fixed32(a, scale, true, fp_begin(rm));
}

uint64_t HELPER(fixed_32_x2)(uint64_t rm, uint64_t a, uint64_t scale)
{
    float_status *s = fp_begin(rm);

    return deposit64(fixed32(a, scale, false, s), 32, 32,
                     fixed32(a >> 32, scale, false, s));
}

uint64_t HELPER(fixedu_32_x2)(uint64_t rm, uint64_t a, uint64_t scale)
{
    float_status *s = fp_begin(rm);

    return deposit64(fixed32(a, scale, true, s), 32, 32,
                     fixed32(a >> 32, scale, true, s));
}

uint64_t HELPER(float_64)(uint64_t rm, uint64_t a, uint64_t scale)
{
    return int64_to_float64_scalbn(a, -(int)scale, fp_begin(rm));
}

uint64_t HELPER(floatu_64)(uint64_t rm, uint64_t a, uint64_t scale)
{
    return uint64_to_float64_scalbn(a, -(int)scale, fp_begin(rm));
}

uint64_t HELPER(float_32)(uint64_t rm, uint64_t a, uint64_t scale)
{
    return int32_to_float32_scalbn(a, -(int)scale, fp_begin(rm));
}

uint64_t HELPER(floatu_32)(uint64_t rm, uint64_t a, uint64_t scale)
{
    return uint32_to_float32_scalbn(a, -(int)scale, fp_begin(rm));
}

uint64_t HELPER(float_32_x2)(uint64_t rm, uint64_t a, uint64_t scale)
{
    float_status *s = fp_begin(rm);

    return deposit64(int32_to_float32_scalbn(a, -(int)scale, s), 32, 32,
                     int32_to_float32_scalbn(a >> 32, -(int)scale, s));
}

uint64_t HELPER(floatu_32_x2)(uint64_t rm, uint64_t a, uint64_t scale)
{
    float_status *s = fp_begin(rm);

    return deposit64(uint32_to_float32_scalbn(a, -(int)scale, s), 32, 32,
                     uint32_to_float32_scalbn(a >> 32, -(int)scale, s));
}

/*
 * Integer operations
 */

/* 8x8 bit matrix multiply and transpose */
uint64_t HELPER(bmm_8)(uint64_t a, uint64_t b)
{
    uint64_t c = 0;

    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++) {
            if (a & 1) {
                c ^= ((b >> (j * 8)) & 0xff) << (i * 8);
            }
            a >>= 1;
        }
    }
    return c;
}

uint64_t HELPER(bmt_8)(uint64_t a)
{
    uint64_t b = 0;

    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++) {
            if (a & 1) {
                b ^= 1ULL << (j * 8 + i);
            }
            a >>= 1;
        }
    }
    return b;
}

static uint64_t rshift_unbound(uint64_t x, unsigned s)
{
    return s >= 64 ? 0 : x >> s;
}

/* carry-less multiply */
static void clmul(uint64_t a, uint64_t b, uint64_t *lo, uint64_t *hi)
{
    *lo = *hi = 0;
    for (int i = 0; i < 64; i++) {
        if ((a >> i) & 1) {
            *lo ^= b << i;
            *hi ^= rshift_unbound(b, 64 - i);
        }
    }
}

void HELPER(clm_64_128)(CPUKVXState *env, uint64_t a, uint64_t b)
{
    clmul(a, b, &env->scratch[0], &env->scratch[1]);
}

/* GCM: bit-reflected multiply in GF(2^128) modulo x^128 + x^7 + x^2 + x + 1 */
#define GCM_MOD_LO ((1 << 7) ^ (1 << 2) ^ (1 << 1) ^ (1 << 0))

static void gcm_reduce(uint64_t from, uint64_t *lo, uint64_t *hi)
{
    for (int i = 63; i >= 0; i--) {
        if ((from >> i) & 1) {
            *lo ^= (uint64_t)GCM_MOD_LO << i;
            *hi ^= rshift_unbound(GCM_MOD_LO, 64 - i);
        }
    }
}

static void gcm(CPUKVXState *env, uint64_t a, uint64_t b, int kind)
{
    uint64_t mul_lo, mul_hi, acc_lo = 0, acc_hi = 0;

    clmul(revbit64(a), revbit64(b), &mul_lo, &mul_hi);
    acc_hi = mul_lo;
    switch (kind) {
    case 0: /* BB */
        acc_hi ^= mul_hi;
        acc_lo = mul_lo;
        gcm_reduce(mul_hi, &acc_lo, &acc_hi);
        break;
    case 1: /* BT */
        gcm_reduce(mul_hi, &acc_lo, &acc_hi);
        break;
    default: { /* TT */
        uint64_t extra;

        acc_hi ^= mul_hi;
        acc_lo = mul_lo;
        gcm_reduce(mul_hi, &acc_lo, &acc_hi);
        extra = acc_hi;
        acc_hi = acc_lo;
        acc_lo = 0;
        gcm_reduce(extra, &acc_lo, &acc_hi);
        break;
    }
    }
    env->scratch[0] = revbit64(acc_hi);
    env->scratch[1] = revbit64(acc_lo);
}

void HELPER(gcm_bb_64_128)(CPUKVXState *env, uint64_t a, uint64_t b)
{
    gcm(env, a, b, 0);
}

void HELPER(gcm_bt_64_128)(CPUKVXState *env, uint64_t a, uint64_t b)
{
    gcm(env, a, b, 1);
}

void HELPER(gcm_tt_64_128)(CPUKVXState *env, uint64_t a, uint64_t b)
{
    gcm(env, a, b, 2);
}

uint64_t HELPER(crc32_be_u32)(uint64_t data, uint64_t poly)
{
    uint32_t crc = data;

    for (int i = 0; i < 32; i++) {
        crc = (crc << 1) ^ ((crc & 0x80000000u) ? (uint32_t)poly : 0);
    }
    return crc;
}

uint64_t HELPER(reflect_32)(uint64_t a)
{
    return revbit32(a);
}

uint64_t HELPER(dot8_8_32)(uint64_t a, uint64_t b)
{
    uint32_t r = 0;

    for (int i = 0; i < 64; i += 8) {
        r += (int8_t)extract64(a, i, 8) * (int8_t)extract64(b, i, 8);
    }
    return r;
}

uint64_t HELPER(dot8u_8_32)(uint64_t a, uint64_t b)
{
    uint32_t r = 0;

    for (int i = 0; i < 64; i += 8) {
        r += (uint8_t)extract64(a, i, 8) * (uint8_t)extract64(b, i, 8);
    }
    return r;
}

uint64_t HELPER(dot8su_8_32)(uint64_t a, uint64_t b)
{
    uint32_t r = 0;

    for (int i = 0; i < 64; i += 8) {
        r += (int8_t)extract64(a, i, 8) * (uint8_t)extract64(b, i, 8);
    }
    return r;
}

uint64_t HELPER(dot8us_8_32)(uint64_t a, uint64_t b)
{
    uint32_t r = 0;

    for (int i = 0; i < 64; i += 8) {
        r += (uint8_t)extract64(a, i, 8) * (int8_t)extract64(b, i, 8);
    }
    return r;
}

uint64_t HELPER(dot4_16_64)(uint64_t a, uint64_t b)
{
    uint64_t r = 0;

    for (int i = 0; i < 64; i += 16) {
        r += (int64_t)(int16_t)extract64(a, i, 16) *
             (int16_t)extract64(b, i, 16);
    }
    return r;
}

uint64_t HELPER(dot4u_16_64)(uint64_t a, uint64_t b)
{
    uint64_t r = 0;

    for (int i = 0; i < 64; i += 16) {
        r += (uint64_t)extract64(a, i, 16) * extract64(b, i, 16);
    }
    return r;
}

uint64_t HELPER(dot4su_16_64)(uint64_t a, uint64_t b)
{
    uint64_t r = 0;

    for (int i = 0; i < 64; i += 16) {
        r += (int64_t)(int16_t)extract64(a, i, 16) * extract64(b, i, 16);
    }
    return r;
}

uint64_t HELPER(dot4us_16_64)(uint64_t a, uint64_t b)
{
    uint64_t r = 0;

    for (int i = 0; i < 64; i += 16) {
        r += extract64(a, i, 16) * (int64_t)(int16_t)extract64(b, i, 16);
    }
    return r;
}

uint64_t HELPER(sxb_x4)(uint64_t a)
{
    uint64_t r = 0;

    for (int i = 0; i < 4; i++) {
        r = deposit64(r, 16 * i, 16, (int8_t)extract64(a, 8 * i, 8));
    }
    return r;
}

uint64_t HELPER(zxb_x4)(uint64_t a)
{
    uint64_t r = 0;

    for (int i = 0; i < 4; i++) {
        r = deposit64(r, 16 * i, 16, extract64(a, 8 * i, 8));
    }
    return r;
}

uint64_t HELPER(sat_32_8_x8)(uint64_t a0, uint64_t a1, uint64_t a2,
                             uint64_t a3)
{
    uint64_t in[4] = { a0, a1, a2, a3 }, r = 0;

    for (int i = 0; i < 8; i++) {
        int32_t v = extract64(in[i / 2], 32 * (i & 1), 32);

        r = deposit64(r, 8 * i, 8, MIN(MAX(v, -0x80), 0x7f));
    }
    return r;
}

uint64_t HELPER(satu_32_8_x8)(uint64_t a0, uint64_t a1, uint64_t a2,
                              uint64_t a3)
{
    uint64_t in[4] = { a0, a1, a2, a3 }, r = 0;

    for (int i = 0; i < 8; i++) {
        int32_t v = extract64(in[i / 2], 32 * (i & 1), 32);

        r = deposit64(r, 8 * i, 8, MIN(MAX(v, 0), 0xff));
    }
    return r;
}

/* kv3 roundint modifier: RN RU RD RZ RHU */
enum { RI_RN, RI_RU, RI_RD, RI_RZ, RI_RHU };

#define STICKY(v, shift, w) ((shift) > 1 ? (v) << ((w) + 1 - (shift)) : 0)
#define FRAC(v, shift, w)   ((shift) ? (v) << ((w) - (shift)) : 0)

static int32_t round32(uint32_t mode, int32_t v, unsigned shift)
{
    int32_t offset = shift ? 1 << (shift - 1) : 0;

    switch (mode) {
    case RI_RHU:
        return ((v & ~offset) >> shift) + ((v & offset) != 0);
    case RI_RZ:
        if (v >= 0) {
            return (v & ~offset) >> shift;
        }
        return (v >> shift) + (FRAC(v, shift, 32) != 0);
    case RI_RD:
        return (v & ~offset) >> shift;
    case RI_RU:
        return (v >> shift) + (FRAC(v, shift, 32) != 0);
    case RI_RN:
        return ((v & ~offset) >> shift) +
               ((v & offset) != 0 &&
                (STICKY(v, shift, 32) != 0 || ((v >> shift) & 1) != 0));
    default:
        return -1;
    }
}

static int64_t round64(uint32_t mode, int64_t v, unsigned shift)
{
    int64_t offset = shift ? 1LL << (shift - 1) : 0;

    switch (mode) {
    case RI_RHU:
        return ((v & ~offset) >> shift) + ((v & offset) != 0);
    case RI_RZ:
        if (v > 0) {
            return v >> shift;
        }
        return (v >> shift) + (FRAC(v, shift, 64) != 0);
    case RI_RD:
        return v >> shift;
    case RI_RU:
        return ((v & ~offset) >> shift) + (FRAC(v, shift, 64) != 0);
    case RI_RN:
        return (v >> shift) +
               ((v & offset) != 0 &&
                (STICKY(v, shift, 64) != 0 || ((v >> shift) & 1) != 0));
    default:
        return -1;
    }
}

void HELPER(round_32_x8)(CPUKVXState *env, uint64_t mode, uint64_t drop,
                         uint64_t a0, uint64_t a1, uint64_t a2, uint64_t a3)
{
    uint64_t in[4] = { a0, a1, a2, a3 };
    unsigned shift = drop & 0x1f;

    for (int i = 0; i < 4; i++) {
        env->scratch[i] =
            deposit64((uint32_t)round32(mode, in[i], shift), 32, 32,
                      (uint32_t)round32(mode, in[i] >> 32, shift));
    }
}

void HELPER(round_64_x4)(CPUKVXState *env, uint64_t mode, uint64_t drop,
                         uint64_t a0, uint64_t a1, uint64_t a2, uint64_t a3)
{
    uint64_t in[4] = { a0, a1, a2, a3 };
    unsigned shift = drop & 0x3f;

    for (int i = 0; i < 4; i++) {
        env->scratch[i] = round64(mode, in[i], shift);
    }
}
