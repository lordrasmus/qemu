/*
 * IA-64 floating point
 *
 * Floating-point registers hold a 64-bit significand with an explicit
 * integer bit and a 17-bit exponent.  Arithmetic is done exactly on a
 * 256-bit significand and rounded once, to the precision and exponent
 * range selected by the completer and the status field (pc, wre), which
 * is what fused multiply-add and the frcpa/frsqrta based division and
 * square root sequences rely on.  Parallel (paired single) operations use
 * softfloat.
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include "qemu/osdep.h"
#include "qemu/host-utils.h"
#include "cpu.h"
#include "exec/helper-proto.h"
#include "accel/tcg/cpu-ldst.h"
#include "fpu/softfloat.h"
#include "accel/tcg/probe.h"
#include "exec/tlb-flags.h"

#include "fp-tables.h.inc"

/* Status field flags in the order of the FPSR (v d z o u i). */
#define FL_V    (1u << 0)
#define FL_D    (1u << 1)
#define FL_Z    (1u << 2)
#define FL_O    (1u << 3)
#define FL_U    (1u << 4)
#define FL_I    (1u << 5)

enum { FPC_ZERO, FPC_FINITE, FPC_INF, FPC_NAN, FPC_NATVAL };

typedef struct FPV {
    int cls;
    bool sign;
    bool snan;
    bool unnormal;      /* input was not normalized: D flag */
    int32_t exp;        /* unbiased exponent of bit 63 */
    uint64_t mant;      /* normalized for FPC_FINITE */
} FPV;

typedef struct U256 {
    uint64_t w[4];      /* w[3] most significant */
} U256;

static const IA64FReg natval = { 0, FP_EXP_NATVAL };

/* Rounding environment */
typedef struct FPEnv {
    int prec;
    int emin, emax;
    int rc;
    bool ftz;
    uint32_t flags;
} FPEnv;

static uint32_t sf_bits(CPUIA64State *env, int sf)
{
    return extract64(env->ar[AR_FPSR], 6 + 13 * sf, 13);
}

/* completer: 0 none (pc), 1 .s, 2 .d, 3 register (integer/extended) */
static void fp_env(CPUIA64State *env, int sf, int completer, FPEnv *fe)
{
    uint32_t b = sf_bits(env, sf);
    bool wre = b & SF_WRE;
    int pc = (b >> SF_PC_SHIFT) & 3;

    fe->rc = (b >> SF_RC_SHIFT) & 3;
    fe->ftz = b & SF_FTZ;
    fe->flags = 0;
    if (completer == 1 || (completer == 0 && pc == 0)) {
        fe->prec = 24;
        fe->emin = -126;
        fe->emax = 127;
    } else if (completer == 2 || (completer == 0 && pc == 2)) {
        fe->prec = 53;
        fe->emin = -1022;
        fe->emax = 1023;
    } else {
        fe->prec = 64;
        fe->emin = -16382;
        fe->emax = 16383;
    }
    if (wre || completer == 3) {
        fe->emin = -65534;
        fe->emax = 65535;
    }
}

static void fp_set_flags(CPUIA64State *env, int sf, uint32_t flags)
{
    if (flags) {
        env->ar[AR_FPSR] |= (uint64_t)flags << (6 + 13 * sf + 7);
    }
}

/*
 * Pack / unpack
 */

static FPV fp_unpack(IA64FReg r)
{
    FPV v = { 0 };
    uint32_t e = r.se & 0x1ffff;

    v.sign = (r.se >> 17) & 1;
    if (e == FP_EXP_NATVAL && r.mant == 0 && !v.sign) {
        v.cls = FPC_NATVAL;
    } else if (e == FP_EXP_MAX) {
        if ((r.mant << 1) == 0 && (r.mant >> 63)) {
            v.cls = FPC_INF;
        } else {
            v.cls = FPC_NAN;
            v.snan = !((r.mant >> 62) & 1);
            v.mant = r.mant;
        }
    } else if (r.mant == 0) {
        v.cls = FPC_ZERO;
    } else {
        int sh = clz64(r.mant);

        v.cls = FPC_FINITE;
        v.unnormal = sh != 0;
        v.mant = r.mant << sh;
        v.exp = (e ? (int32_t)e : 1) - FP_EXP_BIAS - sh;
    }
    return v;
}

static IA64FReg fp_pack_special(int cls, bool sign, uint64_t nanmant)
{
    IA64FReg r;

    switch (cls) {
    case FPC_ZERO:
        r.mant = 0;
        r.se = 0;
        break;
    case FPC_INF:
        r.mant = 1ull << 63;
        r.se = FP_EXP_MAX;
        break;
    case FPC_NATVAL:
        return natval;
    default:            /* quiet NaN */
        r.mant = nanmant | (3ull << 62);
        r.se = FP_EXP_MAX;
        break;
    }
    r.se |= (uint64_t)sign << 17;
    return r;
}

/* Default (real indefinite) quiet NaN */
static IA64FReg fp_default_nan(void)
{
    return fp_pack_special(FPC_NAN, true, 0);
}

/*
 * 256-bit significand arithmetic
 */

static bool u256_zero(const U256 *a)
{
    return !(a->w[0] | a->w[1] | a->w[2] | a->w[3]);
}

static int u256_clz(const U256 *a)
{
    for (int i = 3; i >= 0; i--) {
        if (a->w[i]) {
            return (3 - i) * 64 + clz64(a->w[i]);
        }
    }
    return 256;
}

static void u256_shl(U256 *a, int n)
{
    while (n >= 64) {
        a->w[3] = a->w[2];
        a->w[2] = a->w[1];
        a->w[1] = a->w[0];
        a->w[0] = 0;
        n -= 64;
    }
    if (n) {
        for (int i = 3; i > 0; i--) {
            a->w[i] = (a->w[i] << n) | (a->w[i - 1] >> (64 - n));
        }
        a->w[0] <<= n;
    }
}

/* Shift right, OR-ing the lost bits into bit 0. */
static void u256_shr_jam(U256 *a, int n)
{
    bool sticky = false;

    if (n >= 256) {
        sticky = !u256_zero(a);
        memset(a, 0, sizeof(*a));
        a->w[0] = sticky;
        return;
    }
    while (n >= 64) {
        sticky |= a->w[0] != 0;
        a->w[0] = a->w[1];
        a->w[1] = a->w[2];
        a->w[2] = a->w[3];
        a->w[3] = 0;
        n -= 64;
    }
    if (n) {
        sticky |= (a->w[0] << (64 - n)) != 0;
        for (int i = 0; i < 3; i++) {
            a->w[i] = (a->w[i] >> n) | (a->w[i + 1] << (64 - n));
        }
        a->w[3] >>= n;
    }
    a->w[0] |= sticky;
}

static void u256_add(U256 *r, const U256 *a, const U256 *b)
{
    uint64_t c = 0;

    for (int i = 0; i < 4; i++) {
        uint64_t s = a->w[i] + b->w[i];
        uint64_t c1 = s < a->w[i];

        r->w[i] = s + c;
        c = c1 | (r->w[i] < s);
    }
}

/* r = a - b, a >= b */
static void u256_sub(U256 *r, const U256 *a, const U256 *b)
{
    uint64_t bw = 0;

    for (int i = 0; i < 4; i++) {
        uint64_t d = a->w[i] - b->w[i];
        uint64_t b1 = a->w[i] < b->w[i];

        r->w[i] = d - bw;
        bw = b1 | (d < bw);
    }
}

static int u256_cmp(const U256 *a, const U256 *b)
{
    for (int i = 3; i >= 0; i--) {
        if (a->w[i] != b->w[i]) {
            return a->w[i] < b->w[i] ? -1 : 1;
        }
    }
    return 0;
}


static int u256_bit(const U256 *a, int pos)
{
    return (a->w[pos >> 6] >> (pos & 63)) & 1;
}

/* Any bit below pos set? */
static bool u256_any_below(const U256 *a, int pos)
{
    int w = pos >> 6;

    if (pos <= 0) {
        return false;
    }
    if (a->w[w] & ((1ull << (pos & 63)) - 1)) {
        return true;
    }
    for (int i = 0; i < w; i++) {
        if (a->w[i]) {
            return true;
        }
    }
    return false;
}

/*
 * Round sign * m * 2^(e - 255) (m nonzero) once into the environment.
 * Results tiny in the target range are delivered denormalized, with the
 * minimum exponent and integer bit 0, as the hardware does; the memory
 * format conversion of stfs/stfd relies on that.
 */
static IA64FReg fp_round(FPEnv *fe, bool sign, int32_t e, U256 m)
{
    int sh = u256_clz(&m);
    int32_t elsb, er;
    int drop, msb;
    uint64_t kept;
    bool guard, sticky, up, tiny;
    IA64FReg r;

    u256_shl(&m, sh);
    e -= sh;                        /* exponent of bit 255 */

    tiny = e < fe->emin;
    if (tiny && fe->ftz) {
        fe->flags |= FL_U | FL_I;
        r.mant = 0;
        r.se = (uint64_t)sign << 17;
        return r;
    }
    elsb = (tiny ? fe->emin : e) - fe->prec + 1;
    drop = elsb - (e - 255);        /* bits of m below the result LSB */

    if (drop > 256) {
        kept = 0;
        guard = false;
        sticky = true;
    } else if (drop == 256) {
        kept = 0;
        guard = u256_bit(&m, 255);
        sticky = u256_any_below(&m, 255);
    } else {
        U256 t = m;

        /* drop >= 256 - 64: the kept part fits in one word */
        u256_shr_jam(&t, drop);
        kept = t.w[0];
        if (u256_any_below(&m, drop)) {
            /* undo the jam bit */
            kept = (kept & ~1ull) | u256_bit(&m, drop);
        }
        guard = u256_bit(&m, drop - 1);
        sticky = u256_any_below(&m, drop - 1);
    }

    switch (fe->rc) {
    case 0:
        up = guard && (sticky || (kept & 1));
        break;
    case 1:             /* toward -inf */
        up = sign && (guard || sticky);
        break;
    case 2:             /* toward +inf */
        up = !sign && (guard || sticky);
        break;
    default:
        up = false;
        break;
    }
    if (guard || sticky) {
        fe->flags |= FL_I;
        if (tiny) {
            fe->flags |= FL_U;
        }
    }
    if (up && kept == ~0ull) {
        /* carry out of a 64-bit significand */
        kept = 1ull << 63;
        elsb++;
    } else {
        kept += up;
    }
    if (kept == 0) {
        r.mant = 0;
        r.se = (uint64_t)sign << 17;
        return r;
    }
    msb = 63 - clz64(kept);
    if (msb >= fe->prec) {
        /* carry out of the precision */
        kept >>= 1;
        elsb++;
        msb--;
    }
    er = elsb + msb;
    if (er > fe->emax) {
        fe->flags |= FL_O | FL_I;
        if (fe->rc == 0 || (fe->rc == 1 && sign) || (fe->rc == 2 && !sign)) {
            return fp_pack_special(FPC_INF, sign, 0);
        }
        r.mant = ~0ull << (64 - fe->prec);
        r.se = ((uint64_t)sign << 17) | (fe->emax + FP_EXP_BIAS);
        return r;
    }
    if (er < fe->emin) {
        /* denormal in the target range */
        r.mant = kept << (64 - fe->prec);
        r.se = ((uint64_t)sign << 17) | (fe->emin + FP_EXP_BIAS);
        return r;
    }
    r.mant = kept << (63 - msb);
    r.se = ((uint64_t)sign << 17) | (er + FP_EXP_BIAS);
    return r;
}

static IA64FReg fp_round64(FPEnv *fe, bool sign, int32_t e63, uint64_t mant)
{
    U256 m = { { 0, 0, 0, mant } };

    /* mant has its bit 63 at exponent e63, i.e. bit 255 of m */
    return fp_round(fe, sign, e63, m);
}

/* NaN result of an operation with NaN operands: first NaN, quietened. */
static IA64FReg fp_nan_result(FPEnv *fe, const FPV *a, const FPV *b,
                              const FPV *c)
{
    const FPV *ops[3] = { a, b, c };

    for (int i = 0; i < 3; i++) {
        if (ops[i] && ops[i]->cls == FPC_NAN && ops[i]->snan) {
            fe->flags |= FL_V;
        }
    }
    for (int i = 0; i < 3; i++) {
        if (ops[i] && ops[i]->cls == FPC_NAN) {
            return fp_pack_special(FPC_NAN, ops[i]->sign, ops[i]->mant);
        }
    }
    g_assert_not_reached();
}

/*
 * a * b + c, with the signs of the product and the addend flipped as
 * requested.
 */
static IA64FReg fp_fma(FPEnv *fe, FPV a, FPV b, FPV c, bool negp, bool negc)
{
    bool ps = a.sign ^ b.sign ^ negp;
    bool cs = c.sign ^ negc;

    if (a.cls == FPC_NATVAL || b.cls == FPC_NATVAL || c.cls == FPC_NATVAL) {
        return natval;
    }
    if (a.unnormal || b.unnormal || c.unnormal) {
        fe->flags |= FL_D;
    }
    if (a.cls == FPC_NAN || b.cls == FPC_NAN || c.cls == FPC_NAN) {
        return fp_nan_result(fe, &a, &b, &c);
    }
    if ((a.cls == FPC_INF && b.cls == FPC_ZERO) ||
        (a.cls == FPC_ZERO && b.cls == FPC_INF)) {
        fe->flags |= FL_V;
        return fp_default_nan();
    }
    if (a.cls == FPC_INF || b.cls == FPC_INF) {
        if (c.cls == FPC_INF && cs != ps) {
            fe->flags |= FL_V;
            return fp_default_nan();
        }
        return fp_pack_special(FPC_INF, ps, 0);
    }
    if (c.cls == FPC_INF) {
        return fp_pack_special(FPC_INF, cs, 0);
    }

    if (a.cls == FPC_ZERO || b.cls == FPC_ZERO) {
        if (c.cls == FPC_ZERO) {
            /* exact zero: sign rules */
            bool s = (ps == cs) ? ps : (fe->rc == 1);
            return fp_pack_special(FPC_ZERO, s, 0);
        }
        return fp_round64(fe, cs, c.exp, c.mant);
    }

    {
        uint64_t hi, lo;
        U256 p = { { 0 } }, q = { { 0 } }, r;
        int32_t pe, qe;     /* exponent of bit 255 */
        bool rs;
        int cmp;

        mulu64(&lo, &hi, a.mant, b.mant);
        p.w[3] = hi;
        p.w[2] = lo;
        /* a.mant * b.mant has its bit 127 (= bit 255 of p) at a.exp+b.exp+1 */
        pe = a.exp + b.exp + 1;

        if (c.cls == FPC_ZERO) {
            return fp_round(fe, ps, pe, p);
        }
        q.w[3] = c.mant;
        qe = c.exp;

        /* normalize p (its top bit may be clear) */
        {
            int sh = u256_clz(&p);

            u256_shl(&p, sh);
            pe -= sh;
        }
        if (pe < qe || (pe == qe && u256_cmp(&p, &q) < 0)) {
            U256 t = p;
            int32_t te = pe;
            bool ts = ps;

            p = q;
            pe = qe;
            ps = cs;
            q = t;
            qe = te;
            cs = ts;
        }
        /* p is the larger magnitude; make room for a carry */
        u256_shr_jam(&p, 1);
        u256_shr_jam(&q, 1);
        pe++;
        qe++;
        u256_shr_jam(&q, pe - qe);
        rs = ps;
        if (ps == cs) {
            u256_add(&r, &p, &q);
        } else {
            cmp = u256_cmp(&p, &q);
            if (cmp == 0) {
                return fp_pack_special(FPC_ZERO, fe->rc == 1, 0);
            }
            u256_sub(&r, &p, &q);
        }
        return fp_round(fe, rs, pe, r);
    }
}

static IA64FReg fr_read(CPUIA64State *env, int f)
{
    return env->fr[f];
}

static void fr_write(CPUIA64State *env, int f, IA64FReg v)
{
    env->fr[f] = v;
    env->psr |= f < 32 ? PSR_MFL : PSR_MFH;
}

#define RF1(r)  ((r) & 0x7f)
#define RF2(r)  (((r) >> 8) & 0x7f)
#define RF3(r)  (((r) >> 16) & 0x7f)
#define RF4(r)  (((r) >> 24) & 0x7f)

/*
 * fma, fms, fnma: f1 = (+/-)f3 * f4 (+/-) f2
 * ctl: kind (0 fma, 1 fms, 2 fnma) | completer << 4 | sf << 8
 */
void HELPER(fma)(CPUIA64State *env, uint32_t regs, uint32_t ctl)
{
    int kind = ctl & 3;
    int sf = (ctl >> 8) & 3;
    FPEnv fe;
    IA64FReg r;
    FPV a, b, c;

    fp_env(env, sf, (ctl >> 4) & 3, &fe);
    a = fp_unpack(fr_read(env, RF3(regs)));
    b = fp_unpack(fr_read(env, RF4(regs)));
    c = fp_unpack(fr_read(env, RF2(regs)));
    if (RF2(regs) == 0) {
        /* f2 = f0: an IEEE multiply, a zero product keeps its sign */
        c.sign = a.sign ^ b.sign ^ (kind == 2) ^ (kind == 1);
    }
    r = fp_fma(&fe, a, b, c, kind == 2, kind == 1);
    fp_set_flags(env, sf, fe.flags);
    fr_write(env, RF1(regs), r);
}

/* xma.l, xma.h, xma.hu: integer multiply-add on significands */
void HELPER(xma)(CPUIA64State *env, uint32_t regs, uint32_t kind)
{
    IA64FReg a = fr_read(env, RF3(regs));
    IA64FReg b = fr_read(env, RF4(regs));
    IA64FReg c = fr_read(env, RF2(regs));
    IA64FReg r;
    uint64_t hi, lo;

    if (fp_unpack(a).cls == FPC_NATVAL || fp_unpack(b).cls == FPC_NATVAL ||
        fp_unpack(c).cls == FPC_NATVAL) {
        fr_write(env, RF1(regs), natval);
        return;
    }
    if (kind == 1) {
        int64_t shi;

        muls64(&lo, (uint64_t *)&shi, a.mant, b.mant);
        hi = shi;
        /* add sign-extended c */
        lo += c.mant;
        hi += (lo < c.mant) + ((int64_t)c.mant < 0 ? -1 : 0);
    } else {
        mulu64(&lo, &hi, a.mant, b.mant);
        lo += c.mant;
        hi += lo < c.mant;
    }
    r.mant = kind == 0 ? lo : hi;
    r.se = FP_EXP_INTEGER;
    fr_write(env, RF1(regs), r);
}

/*
 * Comparison.  rel: 0 eq, 1 lt, 2 le, 3 unord.
 * Returns bit 0 = relation, bit 1 = NaTVal source.
 */
static int fp_compare(FPEnv *fe, FPV a, FPV b, int rel)
{
    int c;

    if (a.cls == FPC_NAN || b.cls == FPC_NAN) {
        if (rel == 1 || rel == 2 || (a.cls == FPC_NAN && a.snan) ||
            (b.cls == FPC_NAN && b.snan)) {
            fe->flags |= FL_V;
        }
        return rel == 3;
    }
    if (rel == 3) {
        return 0;
    }
    if (a.unnormal || b.unnormal) {
        fe->flags |= FL_D;
    }
    /* order: -inf < finite < +inf, zeros equal */
    if (a.cls == FPC_ZERO && b.cls == FPC_ZERO) {
        c = 0;
    } else {
        int sa = a.cls == FPC_ZERO ? 0 : (a.sign ? -1 : 1);
        int sb = b.cls == FPC_ZERO ? 0 : (b.sign ? -1 : 1);

        if (sa != sb) {
            c = sa < sb ? -1 : 1;
        } else {
            /* same sign, both nonzero */
            int mag;

            if (a.cls == FPC_INF || b.cls == FPC_INF) {
                mag = (a.cls == FPC_INF) - (b.cls == FPC_INF);
            } else if (a.exp != b.exp) {
                mag = a.exp < b.exp ? -1 : 1;
            } else {
                mag = a.mant < b.mant ? -1 : (a.mant > b.mant);
            }
            c = sa < 0 ? -mag : mag;
        }
    }
    switch (rel) {
    case 0: return c == 0;
    case 1: return c < 0;
    default: return c <= 0;
    }
}

uint32_t HELPER(fcmp)(CPUIA64State *env, uint32_t regs, uint32_t ctl)
{
    int sf = (ctl >> 8) & 3;
    FPV a = fp_unpack(fr_read(env, RF2(regs)));
    FPV b = fp_unpack(fr_read(env, RF3(regs)));
    FPEnv fe;
    int res;

    if (a.cls == FPC_NATVAL || b.cls == FPC_NATVAL) {
        return 2;
    }
    fp_env(env, sf, 3, &fe);
    res = fp_compare(&fe, a, b, ctl & 3);
    fp_set_flags(env, sf, fe.flags);
    return res;
}

/* fclass.m: bit 0 = membership, bit 1 never (NaTVal is a class) */
uint32_t HELPER(fclass)(CPUIA64State *env, uint32_t f2, uint32_t fclass9)
{
    IA64FReg r = fr_read(env, f2);
    FPV v = fp_unpack(r);
    bool sign_ok, type;

    if (v.cls == FPC_NATVAL) {
        return !!(fclass9 & 0x100);
    }
    if (v.cls == FPC_NAN) {
        return v.snan ? !!(fclass9 & 0x040) : !!(fclass9 & 0x080);
    }
    sign_ok = v.sign ? (fclass9 & 0x002) : (fclass9 & 0x001);
    switch (v.cls) {
    case FPC_ZERO:
        type = fclass9 & 0x004;
        break;
    case FPC_INF:
        type = fclass9 & 0x020;
        break;
    default:
        type = (r.mant >> 63) ? (fclass9 & 0x010) : (fclass9 & 0x008);
        break;
    }
    return sign_ok && type;
}

/*
 * Division and square root sequences can fail near the ends of the
 * register exponent range; there the hardware asks for software assist.
 * We compute the IEEE result ourselves instead, as the OS handler would.
 */
static bool frcpa_needs_assist(const FPV *n, const FPV *d)
{
    int32_t ed = d->exp, en = n->cls == FPC_FINITE ? n->exp : 0;

    return ed >= 65535 - 2 || ed <= -65534 + 1 ||
           en - ed >= 65535 - 2 || en - ed <= -65534 + 64 ||
           en <= -65534 + 63;
}

static IA64FReg fp_divide(FPEnv *fe, FPV n, FPV d)
{
    /* long division to 128 bits plus sticky */
    U256 q = { { 0 } };
    uint64_t rhi = 0, rlo = n.mant;
    uint64_t dm = d.mant;
    bool sign = n.sign ^ d.sign;
    unsigned __int128 rem = (unsigned __int128)n.mant;
    unsigned __int128 qq = 0;

    for (int i = 0; i < 128; i++) {
        qq <<= 1;
        if (rem >= dm) {
            rem -= dm;
            qq |= 1;
        }
        rem <<= 1;
    }
    (void)rhi;
    (void)rlo;
    q.w[3] = qq >> 64;
    q.w[2] = (uint64_t)qq;
    q.w[1] = rem != 0;
    /* bit 255 of q weighs 2^(n.exp - d.exp) */
    return fp_round(fe, sign, n.exp - d.exp, q);
}

uint32_t HELPER(frcpa)(CPUIA64State *env, uint32_t regs, uint32_t sf)
{
    FPV n = fp_unpack(fr_read(env, RF2(regs)));
    FPV d = fp_unpack(fr_read(env, RF3(regs)));
    FPEnv fe;
    IA64FReg r;
    int p2 = 0;

    if (n.cls == FPC_NATVAL || d.cls == FPC_NATVAL) {
        fr_write(env, RF1(regs), natval);
        return 0;
    }
    fp_env(env, sf, 0, &fe);
    if (n.unnormal || d.unnormal) {
        fe.flags |= FL_D;
    }
    if (n.cls == FPC_NAN || d.cls == FPC_NAN) {
        r = fp_nan_result(&fe, &n, &d, NULL);
    } else if ((n.cls == FPC_INF && d.cls == FPC_INF) ||
               (n.cls == FPC_ZERO && d.cls == FPC_ZERO)) {
        fe.flags |= FL_V;
        r = fp_default_nan();
    } else if (n.cls == FPC_INF) {
        r = fp_pack_special(FPC_INF, n.sign ^ d.sign, 0);
    } else if (d.cls == FPC_INF || n.cls == FPC_ZERO) {
        r = fp_pack_special(FPC_ZERO, n.sign ^ d.sign, 0);
    } else if (d.cls == FPC_ZERO) {
        fe.flags |= FL_Z;
        r = fp_pack_special(FPC_INF, n.sign ^ d.sign, 0);
    } else if (frcpa_needs_assist(&n, &d)) {
        r = fp_divide(&fe, n, d);
    } else {
        int idx = (d.mant >> 55) & 0xff;

        r.mant = (1ull << 63) | ((uint64_t)recip_table[idx] << 53);
        r.se = ((uint64_t)d.sign << 17) |
               (FP_EXP_MAX - 2 - (d.exp + FP_EXP_BIAS));
        p2 = 1;
    }
    fp_set_flags(env, sf, fe.flags);
    fr_write(env, RF1(regs), r);
    return p2;
}

static IA64FReg fp_sqrt(FPEnv *fe, FPV v)
{
    /* exact integer square root of the significand scaled to 256 bits */
    U256 m = { { 0 } };
    int32_t e = v.exp;
    unsigned __int128 x, rt = 0, rem = 0;
    uint64_t hi, lo;

    /* value = mant * 2^(e - 63); make the exponent even */
    if ((e - 63) & 1) {
        /* x * 2^(e - 126), written as x * 2^((e + 1) - 127) */
        x = (unsigned __int128)v.mant << 63;
        e += 1;
    } else {
        x = (unsigned __int128)v.mant << 64;
    }
    /* x * 2^(e - 63 - 64) with x < 2^128: sqrt over 64 result bits */
    hi = x >> 64;
    lo = (uint64_t)x;
    for (int i = 63; i >= 0; i--) {
        unsigned __int128 trial;

        rem = (rem << 2) | (((i >= 32 ? hi >> ((i - 32) * 2) : lo >> (i * 2)))
                            & 3);
        trial = (rt << 2) | 1;
        rt <<= 1;
        if (rem >= trial) {
            rem -= trial;
            rt |= 1;
        }
    }
    m.w[3] = (uint64_t)rt;
    m.w[2] = rem != 0;
    /* rt ~ sqrt(x), x weighed 2^(e - 127): bit 63 of rt at (e - 127)/2 + 63 */
    return fp_round(fe, false, (e - 127) / 2 + 63, m);
}

uint32_t HELPER(frsqrta)(CPUIA64State *env, uint32_t regs, uint32_t sf)
{
    FPV v = fp_unpack(fr_read(env, RF3(regs)));
    FPEnv fe;
    IA64FReg r;
    int p2 = 0;

    if (v.cls == FPC_NATVAL) {
        fr_write(env, RF1(regs), natval);
        return 0;
    }
    fp_env(env, sf, 0, &fe);
    if (v.unnormal) {
        fe.flags |= FL_D;
    }
    if (v.cls == FPC_NAN) {
        r = fp_nan_result(&fe, &v, NULL, NULL);
    } else if (v.cls == FPC_ZERO) {
        r = fp_pack_special(FPC_ZERO, v.sign, 0);
    } else if (v.sign) {
        fe.flags |= FL_V;
        r = fp_default_nan();
    } else if (v.cls == FPC_INF) {
        r = fp_pack_special(FPC_INF, false, 0);
    } else if (v.exp >= 65535 - 1 || v.exp <= -65534 + 64) {
        r = fp_sqrt(&fe, v);
    } else {
        int32_t E = v.exp + FP_EXP_BIAS;
        int idx = ((E & 1) << 7) | ((v.mant >> 56) & 0x7f);

        r.mant = (1ull << 63) | ((uint64_t)recip_sqrt_table[idx] << 53);
        r.se = 0xfffe - (v.exp >> 1);
        p2 = 1;
    }
    fp_set_flags(env, sf, fe.flags);
    fr_write(env, RF1(regs), r);
    return p2;
}

/*
 * fmin/fmax/famin/famax.  op: 0 min, 1 max, 2 amin, 3 amax.
 * f1 = (f2 rel f3) ? f2 : f3, f3 when unordered.
 */
void HELPER(fminmax)(CPUIA64State *env, uint32_t regs, uint32_t ctl)
{
    int op = ctl & 3;
    int sf = (ctl >> 8) & 3;
    IA64FReg ra = fr_read(env, RF2(regs));
    IA64FReg rb = fr_read(env, RF3(regs));
    FPV a = fp_unpack(ra), b = fp_unpack(rb);
    FPEnv fe;
    bool lt;

    if (a.cls == FPC_NATVAL || b.cls == FPC_NATVAL) {
        fr_write(env, RF1(regs), natval);
        return;
    }
    fp_env(env, sf, 3, &fe);
    if (op >= 2) {
        a.sign = b.sign = false;
    }
    if (op == 0 || op == 2) {
        lt = fp_compare(&fe, a, b, 1);
    } else {
        lt = fp_compare(&fe, b, a, 1);
    }
    fp_set_flags(env, sf, fe.flags);
    fr_write(env, RF1(regs), lt ? ra : rb);
}

/* fcvt.fx[u][.trunc]: ctl = unsigned | trunc << 1 | sf << 8 */
static uint64_t fp_to_int(FPEnv *fe, FPV v, bool uns, bool trunc,
                          bool *invalid)
{
    U256 m = { { 0, 0, 0, 0 } };
    int rc = fe->rc;
    uint64_t res;
    int32_t sh;
    bool guard, sticky, up;

    *invalid = false;
    if (v.cls == FPC_NAN || v.cls == FPC_INF) {
        *invalid = true;
        return 1ull << 63;
    }
    if (v.cls == FPC_ZERO) {
        return 0;
    }
    if (v.unnormal) {
        fe->flags |= FL_D;
    }
    if (v.exp > 63) {
        *invalid = true;
        return 1ull << 63;
    }
    m.w[3] = v.mant;
    /* integer part: shift so that the LSB of the integer lands at bit 0
     * of word 2 */
    sh = 63 - v.exp;            /* bits of mant below the binary point */
    if (sh > 200) {
        sh = 200;
    }
    u256_shr_jam(&m, sh);
    res = m.w[3];
    guard = m.w[2] >> 63;
    sticky = (m.w[2] << 1) || m.w[1] || m.w[0];
    if (trunc) {
        rc = 3;
    }
    switch (rc) {
    case 0: up = guard && (sticky || (res & 1)); break;
    case 1: up = v.sign && (guard || sticky); break;
    case 2: up = !v.sign && (guard || sticky); break;
    default: up = false; break;
    }
    if (guard || sticky) {
        fe->flags |= FL_I;
    }
    res += up;
    if (up && res == 0) {
        *invalid = true;
        return 1ull << 63;
    }
    if (uns) {
        if (v.sign && res != 0) {
            *invalid = true;
            return 1ull << 63;
        }
        return res;
    }
    if (v.sign) {
        if (res > (1ull << 63)) {
            *invalid = true;
            return 1ull << 63;
        }
        return -res;
    }
    if (res >= (1ull << 63)) {
        *invalid = true;
        return 1ull << 63;
    }
    return res;
}

void HELPER(fcvt_fx)(CPUIA64State *env, uint32_t regs, uint32_t ctl)
{
    int sf = (ctl >> 8) & 3;
    FPV v = fp_unpack(fr_read(env, RF2(regs)));
    FPEnv fe;
    IA64FReg r;
    bool inv;

    if (v.cls == FPC_NATVAL) {
        fr_write(env, RF1(regs), natval);
        return;
    }
    fp_env(env, sf, 3, &fe);
    r.mant = fp_to_int(&fe, v, ctl & 1, ctl & 2, &inv);
    r.se = FP_EXP_INTEGER;
    if (inv) {
        fe.flags = FL_V;
    }
    fp_set_flags(env, sf, fe.flags);
    fr_write(env, RF1(regs), r);
}

/* fcvt.xf: signed 64-bit integer significand to floating point, exact */
void HELPER(fcvt_xf)(CPUIA64State *env, uint32_t regs)
{
    IA64FReg s = fr_read(env, RF2(regs));
    IA64FReg r;
    int64_t i = s.mant;
    uint64_t m;
    int sh;

    if (fp_unpack(s).cls == FPC_NATVAL) {
        fr_write(env, RF1(regs), natval);
        return;
    }
    if (i == 0) {
        r.mant = 0;
        r.se = 0;
    } else {
        m = i < 0 ? -(uint64_t)i : (uint64_t)i;
        sh = clz64(m);
        r.mant = m << sh;
        r.se = ((uint64_t)(i < 0) << 17) | (FP_EXP_INTEGER - sh);
    }
    fr_write(env, RF1(regs), r);
}

/*
 * Parallel (paired single) operations through softfloat
 */

static void sf_status(CPUIA64State *env, int sf, float_status *st)
{
    uint32_t b = sf_bits(env, sf);
    static const FloatRoundMode rm[4] = {
        float_round_nearest_even, float_round_down, float_round_up,
        float_round_to_zero,
    };

    memset(st, 0, sizeof(*st));
    set_float_rounding_mode(rm[(b >> SF_RC_SHIFT) & 3], st);
    set_flush_to_zero(b & SF_FTZ, st);
    set_float_2nan_prop_rule(float_2nan_prop_ab, st);
    set_float_3nan_prop_rule(float_3nan_prop_abc, st);
    set_float_infzeronan_rule(float_infzeronan_dnan_never, st);
    set_float_default_nan_pattern(0b11000000, st);
}

static void sf_flags(CPUIA64State *env, int sf, float_status *st)
{
    int x = get_float_exception_flags(st);
    uint32_t f = 0;

    if (x & float_flag_invalid) {
        f |= FL_V;
    }
    if (x & float_flag_divbyzero) {
        f |= FL_Z;
    }
    if (x & float_flag_overflow) {
        f |= FL_O;
    }
    if (x & float_flag_underflow) {
        f |= FL_U;
    }
    if (x & float_flag_inexact) {
        f |= FL_I;
    }
    if (x & float_flag_input_denormal_used) {
        f |= FL_D;
    }
    fp_set_flags(env, sf, f);
}

static IA64FReg pair(uint32_t hi, uint32_t lo)
{
    IA64FReg r = { ((uint64_t)hi << 32) | lo, FP_EXP_INTEGER };

    return r;
}

/* fpma, fpms, fpnma: kind as for fma */
void HELPER(fpma)(CPUIA64State *env, uint32_t regs, uint32_t ctl)
{
    int kind = ctl & 3;
    int sf = (ctl >> 8) & 3;
    IA64FReg a = fr_read(env, RF3(regs));
    IA64FReg b = fr_read(env, RF4(regs));
    IA64FReg c = fr_read(env, RF2(regs));
    float_status st;
    uint32_t res[2];

    sf_status(env, sf, &st);
    for (int i = 0; i < 2; i++) {
        float32 x = a.mant >> (32 * i);
        float32 y = b.mant >> (32 * i);
        float32 z = c.mant >> (32 * i);
        int flags = 0;

        if (kind == 1) {
            flags = float_muladd_negate_c;
        } else if (kind == 2) {
            flags = float_muladd_negate_product;
        }
        res[i] = float32_muladd(x, y, z, flags, &st);
    }
    sf_flags(env, sf, &st);
    fr_write(env, RF1(regs), pair(res[1], res[0]));
}

/*
 * Parallel min/max (op as fminmax) and compares
 * (rel: 0 eq, 1 lt, 2 le, 3 unord, 4 neq, 5 nlt, 6 nle, 7 ord).
 */
void HELPER(fpminmax)(CPUIA64State *env, uint32_t regs, uint32_t ctl)
{
    int op = ctl & 3;
    int sf = (ctl >> 8) & 3;
    IA64FReg a = fr_read(env, RF2(regs));
    IA64FReg b = fr_read(env, RF3(regs));
    float_status st;
    uint32_t res[2];

    sf_status(env, sf, &st);
    for (int i = 0; i < 2; i++) {
        float32 x = a.mant >> (32 * i);
        float32 y = b.mant >> (32 * i);
        float32 xx = x, yy = y;
        bool lt;

        if (op >= 2) {
            xx = float32_abs(x);
            yy = float32_abs(y);
        }
        if (op == 0 || op == 2) {
            lt = float32_lt(xx, yy, &st);
        } else {
            lt = float32_lt(yy, xx, &st);
        }
        res[i] = lt ? x : y;
    }
    sf_flags(env, sf, &st);
    fr_write(env, RF1(regs), pair(res[1], res[0]));
}

void HELPER(fpcmp)(CPUIA64State *env, uint32_t regs, uint32_t ctl)
{
    int rel = ctl & 7;
    int sf = (ctl >> 8) & 3;
    IA64FReg a = fr_read(env, RF2(regs));
    IA64FReg b = fr_read(env, RF3(regs));
    float_status st;
    uint32_t res[2];

    sf_status(env, sf, &st);
    for (int i = 0; i < 2; i++) {
        float32 x = a.mant >> (32 * i);
        float32 y = b.mant >> (32 * i);
        FloatRelation c = (rel & 3) == 0 || (rel & 3) == 3 ?
                          float32_compare_quiet(x, y, &st) :
                          float32_compare(x, y, &st);
        bool t;

        switch (rel & 3) {
        case 0: t = c == float_relation_equal; break;
        case 1: t = c == float_relation_less; break;
        case 2: t = c == float_relation_less || c == float_relation_equal;
            break;
        default: t = c == float_relation_unordered; break;
        }
        if (rel & 4) {
            t = !t;
        }
        res[i] = t ? 0xffffffff : 0;
    }
    sf_flags(env, sf, &st);
    fr_write(env, RF1(regs), pair(res[1], res[0]));
}

/* fpcvt.fx[u][.trunc] */
void HELPER(fpcvt)(CPUIA64State *env, uint32_t regs, uint32_t ctl)
{
    int sf = (ctl >> 8) & 3;
    IA64FReg a = fr_read(env, RF2(regs));
    float_status st;
    uint32_t res[2];

    sf_status(env, sf, &st);
    if (ctl & 2) {
        set_float_rounding_mode(float_round_to_zero, &st);
    }
    for (int i = 0; i < 2; i++) {
        float32 x = a.mant >> (32 * i);

        res[i] = (ctl & 1) ? float32_to_uint32(x, &st)
                           : (uint32_t)float32_to_int32(x, &st);
    }
    sf_flags(env, sf, &st);
    fr_write(env, RF1(regs), pair(res[1], res[0]));
}

/* fprcpa / fprsqrta: the full IEEE result, predicate cleared */
uint32_t HELPER(fprcpa)(CPUIA64State *env, uint32_t regs, uint32_t ctl)
{
    int sf = (ctl >> 8) & 3;
    bool rsqrt = ctl & 1;
    IA64FReg a = fr_read(env, RF2(regs));
    IA64FReg b = fr_read(env, RF3(regs));
    float_status st;
    uint32_t res[2];

    sf_status(env, sf, &st);
    for (int i = 0; i < 2; i++) {
        float32 x = a.mant >> (32 * i);
        float32 y = b.mant >> (32 * i);

        res[i] = rsqrt ? float32_sqrt(y, &st) : float32_div(x, y, &st);
    }
    sf_flags(env, sf, &st);
    fr_write(env, RF1(regs), pair(res[1], res[0]));
    return 0;
}

/*
 * Non-arithmetic F-unit operations, keyed by a small code from the
 * translator.
 */
enum {
    FM_MERGE_S, FM_MERGE_NS, FM_MERGE_SE,
    FM_MIX_L, FM_MIX_R, FM_MIX_LR,
    FM_SXT_L, FM_SXT_R,
    FM_PACK, FM_SWAP, FM_SWAP_NL, FM_SWAP_NR,
    FM_AND, FM_ANDCM, FM_OR, FM_XOR,
    FM_PMERGE_S, FM_PMERGE_NS, FM_PMERGE_SE,
    FM_SELECT,
};

void HELPER(fmisc)(CPUIA64State *env, uint32_t regs, uint32_t op)
{
    IA64FReg a = fr_read(env, RF2(regs));
    IA64FReg b = fr_read(env, RF3(regs));
    IA64FReg r = { 0, FP_EXP_INTEGER };
    uint32_t ah = a.mant >> 32, al = a.mant;
    uint32_t bh = b.mant >> 32, bl = b.mant;

    if ((fp_unpack(a).cls == FPC_NATVAL && op != FM_SELECT) ||
        fp_unpack(b).cls == FPC_NATVAL) {
        fr_write(env, RF1(regs), natval);
        return;
    }
    switch (op) {
    case FM_MERGE_S:
        r.mant = b.mant;
        r.se = (b.se & 0x1ffff) | (a.se & (1 << 17));
        break;
    case FM_MERGE_NS:
        r.mant = b.mant;
        r.se = (b.se & 0x1ffff) | (~a.se & (1 << 17));
        break;
    case FM_MERGE_SE:
        r.mant = b.mant;
        r.se = a.se;
        break;
    case FM_MIX_L:
        r.mant = ((uint64_t)ah << 32) | bh;
        break;
    case FM_MIX_R:
        r.mant = ((uint64_t)al << 32) | bl;
        break;
    case FM_MIX_LR:
        r.mant = ((uint64_t)ah << 32) | bl;
        break;
    case FM_SXT_L:
        r.mant = ((uint64_t)((int32_t)ah >> 31) << 32) | bh;
        break;
    case FM_SXT_R:
        r.mant = ((uint64_t)((int32_t)al >> 31) << 32) | bl;
        break;
    case FM_SWAP:
        r.mant = ((uint64_t)al << 32) | bh;
        break;
    case FM_SWAP_NL:
        r.mant = ((uint64_t)(al ^ 0x80000000) << 32) | bh;
        break;
    case FM_SWAP_NR:
        r.mant = ((uint64_t)al << 32) | (bh ^ 0x80000000);
        break;
    case FM_AND:
        r.mant = a.mant & b.mant;
        break;
    case FM_ANDCM:
        r.mant = a.mant & ~b.mant;
        break;
    case FM_OR:
        r.mant = a.mant | b.mant;
        break;
    case FM_XOR:
        r.mant = a.mant ^ b.mant;
        break;
    case FM_PMERGE_S:
        r.mant = (a.mant & 0x8000000080000000ull) |
                 (b.mant & ~0x8000000080000000ull);
        break;
    case FM_PMERGE_NS:
        r.mant = (~a.mant & 0x8000000080000000ull) |
                 (b.mant & ~0x8000000080000000ull);
        break;
    case FM_PMERGE_SE:
        r.mant = (a.mant & 0xff800000ff800000ull) |
                 (b.mant & ~0xff800000ff800000ull);
        break;
    case FM_PACK: {
        /* both register values converted to single (no rounding) */
        uint32_t s2, s3;

        s2 = ((a.se >> 17) << 31) |
             (((((a.se >> 16) & 1) << 7) | (a.se & 0x7f)) <<
              ((a.mant >> 63) ? 23 : 32)) | ((a.mant << 1) >> 41);
        s3 = ((b.se >> 17) << 31) |
             (((((b.se >> 16) & 1) << 7) | (b.se & 0x7f)) <<
              ((b.mant >> 63) ? 23 : 32)) | ((b.mant << 1) >> 41);
        r.mant = ((uint64_t)s2 << 32) | s3;
        break;
    }
    default:
        g_assert_not_reached();
    }
    fr_write(env, RF1(regs), r);
}

/* fselect f1 = f3, f4, f2 */
void HELPER(fselect)(CPUIA64State *env, uint32_t regs)
{
    IA64FReg c = fr_read(env, RF2(regs));
    IA64FReg a = fr_read(env, RF3(regs));
    IA64FReg m = fr_read(env, RF4(regs));
    IA64FReg r = { 0, FP_EXP_INTEGER };

    if (fp_unpack(a).cls == FPC_NATVAL || fp_unpack(m).cls == FPC_NATVAL ||
        fp_unpack(c).cls == FPC_NATVAL) {
        fr_write(env, RF1(regs), natval);
        return;
    }
    r.mant = (a.mant & m.mant) | (c.mant & ~m.mant);
    fr_write(env, RF1(regs), r);
}

/* fsetc.sf amask7, omask7 */
void HELPER(fsetc)(CPUIA64State *env, uint32_t sf, uint32_t masks)
{
    uint32_t amask = masks & 0x7f, omask = (masks >> 8) & 0x7f;
    uint32_t ctl0 = sf_bits(env, 0) & 0x7f;
    uint32_t v = (ctl0 & amask) | omask;
    int pos = 6 + 13 * sf;

    env->ar[AR_FPSR] = deposit64(env->ar[AR_FPSR], pos, 7, v);
}

void HELPER(fclrf)(CPUIA64State *env, uint32_t sf)
{
    env->ar[AR_FPSR] = deposit64(env->ar[AR_FPSR], 6 + 13 * sf + 7, 6, 0);
}

/* fchkf: branch if sf flags are not a subset of sf0's or would trap */
uint32_t HELPER(fchkf)(CPUIA64State *env, uint32_t sf)
{
    uint32_t f = sf_bits(env, sf) >> 7;
    uint32_t f0 = sf_bits(env, 0) >> 7;
    uint32_t enabled = ~env->ar[AR_FPSR] & 0x3f;

    return (f & ~f0) || (f & enabled);
}

/*
 * Memory formats
 */

static IA64FReg single_to_reg(uint32_t v)
{
    IA64FReg r;
    uint32_t e = (v >> 23) & 0xff, f = v & 0x7fffff;
    uint64_t s = (uint64_t)(v >> 31) << 17;

    if (e == 0xff) {
        r.mant = (1ull << 63) | ((uint64_t)f << 40);
        r.se = s | FP_EXP_MAX;
    } else if (e == 0) {
        r.mant = (uint64_t)f << 40;
        r.se = s | (f ? 0xff81 : 0);
    } else {
        r.mant = (1ull << 63) | ((uint64_t)f << 40);
        r.se = s | (e - 127 + FP_EXP_BIAS);
    }
    return r;
}

static IA64FReg double_to_reg(uint64_t v)
{
    IA64FReg r;
    uint32_t e = (v >> 52) & 0x7ff;
    uint64_t f = v & ((1ull << 52) - 1);
    uint64_t s = (v >> 63) << 17;

    if (e == 0x7ff) {
        r.mant = (1ull << 63) | (f << 11);
        r.se = s | FP_EXP_MAX;
    } else if (e == 0) {
        r.mant = f << 11;
        r.se = s | (f ? 0xfc01 : 0);
    } else {
        r.mant = (1ull << 63) | (f << 11);
        r.se = s | (e - 1023 + FP_EXP_BIAS);
    }
    return r;
}

static IA64FReg ext_to_reg(uint64_t mant, uint16_t se)
{
    IA64FReg r;
    uint32_t e = se & 0x7fff;
    uint64_t s = (uint64_t)(se >> 15) << 17;

    r.mant = mant;
    if (e == 0x7fff) {
        r.se = s | FP_EXP_MAX;
    } else if (e == 0) {
        r.se = s | (mant ? 0xc001 : 0);
    } else {
        r.se = s | (e - 16383 + FP_EXP_BIAS);
    }
    return r;
}

static uint32_t reg_to_single(IA64FReg r)
{
    uint32_t e = r.se & 0x1ffff;
    uint32_t me = ((e >> 16) << 7) | (e & 0x7f);

    if (!(r.mant >> 63)) {
        me = 0;
    }
    return ((uint32_t)(r.se >> 17) << 31) | (me << 23) |
           (uint32_t)((r.mant << 1) >> 41);
}

static uint64_t reg_to_double(IA64FReg r)
{
    uint32_t e = r.se & 0x1ffff;
    uint64_t me = ((e >> 16) << 10) | (e & 0x3ff);

    if (!(r.mant >> 63)) {
        me = 0;
    }
    return ((r.se >> 17) << 63) | (me << 52) | ((r.mant << 1) >> 12);
}

static uint16_t reg_to_ext_se(IA64FReg r)
{
    uint32_t e = r.se & 0x1ffff;
    uint16_t me = ((e >> 16) << 14) | (e & 0x3fff);

    if (e == 0) {
        me = 0;
    }
    return ((r.se >> 17) << 15) | me;
}

/* kinds of ldf/stf/setf/getf */
enum { FK_S, FK_D, FK_E, FK_8, FK_FILL, FK_EXP, FK_SIG };

static bool is_natval(IA64FReg r)
{
    return (r.se & 0x3ffff) == FP_EXP_NATVAL && r.mant == 0;
}

/*
 * ldf: kind | spec << 4.  Speculative loads defer every fault: the
 * register becomes NaTVal.
 */
void HELPER(ldf)(CPUIA64State *env, uint32_t f1, uint64_t addr, uint32_t ctl,
                 uint32_t mmu_idx)
{
    int kind = ctl & 15;
    bool spec = ctl & 16;
    uintptr_t ra = GETPC();
    MemOpIdx oi;
    IA64FReg r;

    if (spec) {
        int size = (kind == FK_S) ? 4 : (kind == FK_E || kind == FK_FILL)
                                        ? 16 : 8;
        void *host;

        if (probe_access_flags(env, addr, size, MMU_DATA_LOAD, mmu_idx,
                               true, &host, ra) & TLB_INVALID_MASK ||
            (addr & ((kind == FK_E || kind == FK_FILL) ? 15 : size - 1))) {
            fr_write(env, f1, natval);
            return;
        }
    }
    switch (kind) {
    case FK_S:
        oi = make_memop_idx(MO_LEUL | MO_ALIGN, mmu_idx);
        r = single_to_reg(cpu_ldl_mmu(env, addr, oi, ra));
        break;
    case FK_D:
        oi = make_memop_idx(MO_LEUQ | MO_ALIGN, mmu_idx);
        r = double_to_reg(cpu_ldq_mmu(env, addr, oi, ra));
        break;
    case FK_8:
        oi = make_memop_idx(MO_LEUQ | MO_ALIGN, mmu_idx);
        r.mant = cpu_ldq_mmu(env, addr, oi, ra);
        r.se = FP_EXP_INTEGER;
        break;
    case FK_E: {
        uint64_t m;
        uint16_t se;

        if (addr & 15) {
            /* ldfe needs 16-byte alignment */
            oi = make_memop_idx(MO_LEUQ | MO_ALIGN_16, mmu_idx);
        } else {
            oi = make_memop_idx(MO_LEUQ, mmu_idx);
        }
        m = cpu_ldq_mmu(env, addr, oi, ra);
        se = cpu_ldw_mmu(env, addr + 8, make_memop_idx(MO_LEUW, mmu_idx), ra);
        r = ext_to_reg(m, se);
        break;
    }
    default: {          /* fill */
        oi = make_memop_idx(MO_LEUQ | MO_ALIGN_16, mmu_idx);
        r.mant = cpu_ldq_mmu(env, addr, oi, ra);
        r.se = cpu_ldq_mmu(env, addr + 8, make_memop_idx(MO_LEUQ, mmu_idx),
                           ra) & 0x3ffff;
        break;
    }
    }
    fr_write(env, f1, r);
}

/* ldfps/ldfpd/ldfp8: f1, f2 from consecutive elements */
void HELPER(ldfp)(CPUIA64State *env, uint32_t regs, uint64_t addr,
                  uint32_t ctl, uint32_t mmu_idx)
{
    int kind = ctl & 15;
    bool spec = ctl & 16;
    uintptr_t ra = GETPC();
    int size = kind == FK_S ? 4 : 8;
    IA64FReg r1, r2;

    if (spec) {
        void *host;

        if (probe_access_flags(env, addr, size * 2, MMU_DATA_LOAD, mmu_idx,
                               true, &host, ra) & TLB_INVALID_MASK ||
            (addr & (size * 2 - 1))) {
            fr_write(env, RF1(regs), natval);
            fr_write(env, RF2(regs), natval);
            return;
        }
    }
    if (kind == FK_S) {
        MemOpIdx oi = make_memop_idx(MO_LEUQ | MO_ALIGN, mmu_idx);
        uint64_t v = cpu_ldq_mmu(env, addr, oi, ra);

        r1 = single_to_reg(v);
        r2 = single_to_reg(v >> 32);
    } else {
        MemOpIdx oi = make_memop_idx(MO_LEUQ | MO_ALIGN_16, mmu_idx);
        uint64_t v1 = cpu_ldq_mmu(env, addr, oi, ra);
        uint64_t v2 = cpu_ldq_mmu(env, addr + 8,
                                  make_memop_idx(MO_LEUQ, mmu_idx), ra);

        if (kind == FK_D) {
            r1 = double_to_reg(v1);
            r2 = double_to_reg(v2);
        } else {
            r1.mant = v1;
            r1.se = FP_EXP_INTEGER;
            r2.mant = v2;
            r2.se = FP_EXP_INTEGER;
        }
    }
    fr_write(env, RF1(regs), r1);
    fr_write(env, RF2(regs), r2);
}

void HELPER(stf)(CPUIA64State *env, uint32_t f2, uint64_t addr, uint32_t kind,
                 uint32_t mmu_idx)
{
    IA64FReg r = fr_read(env, f2);
    uintptr_t ra = GETPC();

    if (kind != FK_FILL && is_natval(r)) {
        ia64_raise_fault(env, IVT_NAT_CONSUMPTION, ISR_W, 0, 0, ra);
    }
    switch (kind) {
    case FK_S:
        cpu_stl_mmu(env, addr, reg_to_single(r),
                    make_memop_idx(MO_LEUL | MO_ALIGN, mmu_idx), ra);
        break;
    case FK_D:
        cpu_stq_mmu(env, addr, reg_to_double(r),
                    make_memop_idx(MO_LEUQ | MO_ALIGN, mmu_idx), ra);
        break;
    case FK_8:
        cpu_stq_mmu(env, addr, r.mant,
                    make_memop_idx(MO_LEUQ | MO_ALIGN, mmu_idx), ra);
        break;
    case FK_E:
        cpu_stq_mmu(env, addr, r.mant,
                    make_memop_idx(MO_LEUQ | MO_ALIGN_16, mmu_idx), ra);
        cpu_stw_mmu(env, addr + 8, reg_to_ext_se(r),
                    make_memop_idx(MO_LEUW, mmu_idx), ra);
        break;
    default:            /* spill: 16 bytes */
        cpu_stq_mmu(env, addr, r.mant,
                    make_memop_idx(MO_LEUQ | MO_ALIGN_16, mmu_idx), ra);
        cpu_stq_mmu(env, addr + 8, r.se & 0x3ffff,
                    make_memop_idx(MO_LEUQ, mmu_idx), ra);
        break;
    }
}

/* setf: kind | nat << 4 */
void HELPER(setf)(CPUIA64State *env, uint32_t f1, uint64_t val, uint32_t ctl)
{
    IA64FReg r;

    if (ctl & 16) {
        fr_write(env, f1, natval);
        return;
    }
    switch (ctl & 15) {
    case FK_S:
        r = single_to_reg(val);
        break;
    case FK_D:
        r = double_to_reg(val);
        break;
    case FK_EXP:
        r.mant = 1ull << 63;
        r.se = val & 0x3ffff;
        break;
    default:            /* sig */
        r.mant = val;
        r.se = FP_EXP_INTEGER;
        break;
    }
    fr_write(env, f1, r);
}

/* getf; the caller handles NaTVal */
uint64_t HELPER(getf)(CPUIA64State *env, uint32_t f2, uint32_t kind)
{
    IA64FReg r = fr_read(env, f2);

    switch (kind) {
    case FK_S:
        return reg_to_single(r);
    case FK_D:
        return reg_to_double(r);
    case FK_EXP:
        return r.se & 0x3ffff;
    default:
        return r.mant;
    }
}

uint32_t HELPER(fr_is_natval)(CPUIA64State *env, uint32_t f)
{
    return is_natval(env->fr[f]);
}
