/*
 * TILE-Gx floating point helper instructions
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 *
 * TILE-Gx has no FPU.  It has a handful of instructions that do the
 * expensive parts of IEEE single and double arithmetic, and gcc combines
 * them into fixed sequences (config/tilegx/tilegx.md: addsf3, subsf3,
 * mulsf3, adddf3, subdf3, muldf3, float(uns)si{s,d}f2, and the compares
 * in tilegx.c).  The ISA manual names what each one computes but not the
 * intermediate formats, so they are chosen here to fit those sequences:
 *
 * Double: an unpacked mantissa has the hidden bit at bit 59, which leaves
 * seven bits below the fraction for rounding.  The flags word carries a
 * signed exponent V in bits 7..19, the result sign in bit 20, and the
 * compare bits of table 7-2 in bits 25..31.  fdouble_pack2 takes a 128-bit
 * mantissa (SrcA:SrcB) worth M * 2^(V - 1146), which is what all three
 * producers hand it:
 *   add/sub:   max mantissa in SrcA, V = exponent of the larger operand;
 *   mul:       the 128-bit product of two unpacked mantissas,
 *              V = ea + eb - 1018;
 *   int->dbl:  gcc's own flags 0x21b00 (V = 0x436) with the integer << 4.
 *
 * Single: the whole result fits next to the flags, so fsingle_add1,
 * fsingle_sub1 and fsingle_mul1 compute it and put it in bits 32..63 with
 * a marker in bit 11; the other single steps pass it along.  gcc's
 * float(uns)sisf2 builds its own word: exponent 0x9e in bits 0..7, sign
 * in bit 10, the magnitude in bits 32..63.
 */

#include "qemu/osdep.h"
#include "qemu/host-utils.h"
#include "cpu.h"
#include "exec/helper-proto.h"
#include "fpu/softfloat.h"

/* Compare flags, table 7-2 */
#define FP_UNORDERED    (1ULL << 25)
#define FP_LT           (1ULL << 26)
#define FP_LE           (1ULL << 27)
#define FP_GT           (1ULL << 28)
#define FP_GE           (1ULL << 29)
#define FP_EQ           (1ULL << 30)
#define FP_NE           (1ULL << 31)

/* Our own bits in the flags words */
#define DF_SIGN         (1ULL << 20)
#define DF_EFF_SUB      (1ULL << 21)
#define DF_NAN          (1ULL << 22)
#define DF_INF          (1ULL << 23)
#define SF_DONE         (1ULL << 11)

#define DF_HIDDEN       59

static void fp_status_init(float_status *s)
{
    memset(s, 0, sizeof(*s));
    set_float_rounding_mode(float_round_nearest_even, s);
    set_float_2nan_prop_rule(float_2nan_prop_s_ab, s);
    set_float_default_nan_pattern(0b01000000, s);
}

static uint64_t compare_flags(FloatRelation r)
{
    switch (r) {
    case float_relation_less:
        return FP_LT | FP_LE | FP_NE;
    case float_relation_equal:
        return FP_LE | FP_GE | FP_EQ;
    case float_relation_greater:
        return FP_GT | FP_GE | FP_NE;
    default:
        return FP_UNORDERED | FP_NE;
    }
}

/* ------------------------------------------------------------------ */
/* Double                                                              */

static bool df_is_nan(uint64_t x)
{
    return (x & 0x7fffffffffffffffULL) > 0x7ff0000000000000ULL;
}

static bool df_is_inf(uint64_t x)
{
    return (x & 0x7fffffffffffffffULL) == 0x7ff0000000000000ULL;
}

static int df_exp(uint64_t x)
{
    int e = (x >> 52) & 0x7ff;

    return e ? e : 1;
}

static uint64_t df_mant(uint64_t x)
{
    uint64_t m = x & ((1ULL << 52) - 1);

    if ((x >> 52) & 0x7ff) {
        m |= 1ULL << 52;
    }
    return m << (DF_HIDDEN - 52);
}

static uint64_t df_mag(uint64_t x)
{
    return x & 0x7fffffffffffffffULL;
}

uint64_t helper_fdouble_unpack_max(uint64_t a, uint64_t b)
{
    return df_mant(df_mag(a) >= df_mag(b) ? a : b);
}

/* The smaller mantissa, aligned to the larger, with a sticky bit 0. */
uint64_t helper_fdouble_unpack_min(uint64_t a, uint64_t b)
{
    uint64_t max = df_mag(a) >= df_mag(b) ? a : b;
    uint64_t min = df_mag(a) >= df_mag(b) ? b : a;
    uint64_t m = df_mant(min);
    int d = df_exp(max) - df_exp(min);

    if (d == 0) {
        return m;
    }
    if (d >= 64) {
        return m != 0;
    }
    return (m >> d) | ((m & ((1ULL << d) - 1)) != 0);
}

static uint64_t df_addsub_flags(uint64_t a, uint64_t b, bool sub)
{
    float_status st;
    bool sa = a >> 63, sb = (b >> 63) ^ sub;
    bool a_is_max = df_mag(a) >= df_mag(b);
    bool eff_sub = sa != sb;
    uint64_t flags;
    bool sign;

    fp_status_init(&st);
    flags = compare_flags(float64_compare_quiet(make_float64(a),
                                                make_float64(b), &st));

    if (df_is_nan(a) || df_is_nan(b)) {
        return flags | DF_NAN;
    }
    if (df_is_inf(a) || df_is_inf(b)) {
        if (df_is_inf(a) && df_is_inf(b) && eff_sub) {
            return flags | DF_NAN;
        }
        sign = df_is_inf(a) ? sa : sb;
        return flags | DF_INF | (sign ? DF_SIGN : 0);
    }

    sign = a_is_max ? sa : sb;
    if (eff_sub && df_mag(a) == df_mag(b)) {
        sign = false;                   /* x - x = +0 */
    }
    flags |= (uint64_t)(df_exp(a_is_max ? a : b) & 0x1fff) << 7;
    flags |= sign ? DF_SIGN : 0;
    flags |= eff_sub ? DF_EFF_SUB : 0;
    return flags;
}

uint64_t helper_fdouble_add_flags(uint64_t a, uint64_t b)
{
    return df_addsub_flags(a, b, false);
}

uint64_t helper_fdouble_sub_flags(uint64_t a, uint64_t b)
{
    return df_addsub_flags(a, b, true);
}

uint64_t helper_fdouble_addsub(uint64_t max, uint64_t min, uint64_t flags)
{
    return flags & DF_EFF_SUB ? max - min : max + min;
}

uint64_t helper_fdouble_mul_flags(uint64_t a, uint64_t b)
{
    bool sign = (a ^ b) >> 63;
    bool a_zero = df_mag(a) == 0, b_zero = df_mag(b) == 0;
    uint64_t flags = sign ? DF_SIGN : 0;

    if (df_is_nan(a) || df_is_nan(b) ||
        (df_is_inf(a) && b_zero) || (df_is_inf(b) && a_zero)) {
        return DF_NAN;
    }
    if (df_is_inf(a) || df_is_inf(b)) {
        return flags | DF_INF;
    }
    return flags |
           ((uint64_t)((df_exp(a) + df_exp(b) - 1018) & 0x1fff) << 7);
}

uint64_t helper_fdouble_pack1(uint64_t mant, uint64_t flags)
{
    return flags;
}

/* Normalize and round M * 2^(V - 1146), M = hi:lo, to the nearest even. */
uint64_t helper_fdouble_pack2(uint64_t flags, uint64_t hi, uint64_t lo)
{
    uint64_t sign = flags & DF_SIGN ? 1ULL << 63 : 0;
    int v = sextract64(flags, 7, 13);
    Int128 m = int128_make128(lo, hi);
    uint64_t mant, half, rest;
    int p, e, sh;

    if (flags & DF_NAN) {
        return 0x7ff8000000000000ULL;
    }
    if (flags & DF_INF) {
        return sign | 0x7ff0000000000000ULL;
    }
    if (!hi && !lo) {
        return sign;
    }

    p = hi ? 127 - clz64(hi) : 63 - clz64(lo);
    e = p + v - 123;                    /* biased exponent */
    sh = p - 52;
    if (e < 1) {
        sh += 1 - e;                    /* subnormal: fewer bits */
        e = 0;
    }
    if (sh <= 0) {
        return sign | (((uint64_t)e << 52) +
                       (int128_getlo(int128_lshift(m, -sh)) -
                        (e ? 1ULL << 52 : 0)));
    }
    if (sh >= 128) {
        return sign;                    /* rounds to zero */
    }

    mant = int128_getlo(int128_urshift(m, sh));
    /* rounding bits: half is the first one shifted out */
    half = int128_getlo(int128_urshift(m, sh - 1)) & 1;
    rest = int128_nz(int128_and(m, int128_sub(int128_lshift(int128_one(),
                                                            sh - 1),
                                              int128_one())));
    if (half && (rest || (mant & 1))) {
        mant++;
    }
    if (e == 0) {
        /* a subnormal that rounds up to 2^52 is the smallest normal */
        return sign | mant;
    }
    if (mant >> 53) {
        mant >>= 1;
        e++;
    }
    if (e >= 0x7ff) {
        return sign | 0x7ff0000000000000ULL;
    }
    return sign | ((uint64_t)e << 52) | (mant & ((1ULL << 52) - 1));
}

/* ------------------------------------------------------------------ */
/* Single                                                              */

static uint64_t sf_done(float32 r, uint64_t flags)
{
    return ((uint64_t)float32_val(r) << 32) | SF_DONE | flags;
}

static uint64_t sf_addsub1(uint64_t a, uint64_t b, bool sub)
{
    float_status st;
    float32 fa = make_float32(a), fb = make_float32(b), r;
    uint64_t flags;

    fp_status_init(&st);
    flags = compare_flags(float32_compare_quiet(fa, fb, &st));
    r = sub ? float32_sub(fa, fb, &st) : float32_add(fa, fb, &st);
    return sf_done(r, flags);
}

uint64_t helper_fsingle_add1(uint64_t a, uint64_t b)
{
    return sf_addsub1(a, b, false);
}

uint64_t helper_fsingle_sub1(uint64_t a, uint64_t b)
{
    return sf_addsub1(a, b, true);
}

uint64_t helper_fsingle_addsub2(uint64_t d, uint64_t a, uint64_t b)
{
    return d;
}

uint64_t helper_fsingle_mul1(uint64_t a, uint64_t b)
{
    float_status st;

    fp_status_init(&st);
    return sf_done(float32_mul(make_float32(a), make_float32(b), &st), 0);
}

uint64_t helper_fsingle_mul2(uint64_t a, uint64_t b)
{
    return a;
}

uint64_t helper_fsingle_pack1(uint64_t a)
{
    return a;
}

uint64_t helper_fsingle_pack2(uint64_t a, uint64_t flags)
{
    float_status st;
    uint32_t mag = a >> 32;
    float32 r;

    if (a & SF_DONE) {
        return a >> 32;
    }
    /* gcc's float(uns)sisf2: magnitude * 2^(exp - 158) */
    fp_status_init(&st);
    r = uint32_to_float32_scalbn(mag, (int)(a & 0xff) - 158, &st);
    if (a & (1 << 10)) {
        r = float32_chs(r);
    }
    return float32_val(r);
}
