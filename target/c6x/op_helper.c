/*
 * TI C64x+ / C674x arithmetic helpers
 *
 * Semantics follow the "Execution" sections of SPRU732J (C64x+) and
 * SPRUFE8B (C674x floating point).
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/helper-proto.h"
#include "fpu/softfloat.h"
#include "qemu/host-utils.h"

/* Bit fields: csta/cstb as in EXT/EXTU (shift amounts), SET/CLR (bits) */

uint32_t HELPER(extu)(uint32_t v, uint32_t csta, uint32_t cstb)
{
    return (v << (csta & 31)) >> (cstb & 31);
}

uint32_t HELPER(ext)(uint32_t v, uint32_t csta, uint32_t cstb)
{
    return (int32_t)(v << (csta & 31)) >> (cstb & 31);
}

static uint32_t field_mask(uint32_t csta, uint32_t cstb)
{
    csta &= 31;
    cstb &= 31;
    if (cstb < csta) {
        return 0;
    }
    return (uint32_t)(((2ull << cstb) - 1) & ~((1ull << csta) - 1));
}

uint32_t HELPER(set)(uint32_t v, uint32_t csta, uint32_t cstb)
{
    return v | field_mask(csta, cstb);
}

uint32_t HELPER(clr)(uint32_t v, uint32_t csta, uint32_t cstb)
{
    return v & ~field_mask(csta, cstb);
}

/* Packed data */

static inline int32_t h(uint32_t v, int i)
{
    return (int16_t)(v >> (16 * i));
}

static inline uint32_t uh(uint32_t v, int i)
{
    return (uint16_t)(v >> (16 * i));
}

static inline int32_t sb(uint32_t v, int i)
{
    return (int8_t)(v >> (8 * i));
}

static inline uint32_t ub(uint32_t v, int i)
{
    return (uint8_t)(v >> (8 * i));
}

static inline uint32_t mkh(uint32_t lo, uint32_t hi)
{
    return (lo & 0xffff) | (hi << 16);
}

static inline uint32_t mkb(uint32_t b0, uint32_t b1, uint32_t b2, uint32_t b3)
{
    return (b0 & 0xff) | (b1 & 0xff) << 8 | (b2 & 0xff) << 16 | b3 << 24;
}

uint32_t HELPER(add2)(uint32_t a, uint32_t b)
{
    return mkh(a + b, (a >> 16) + (b >> 16));
}

uint32_t HELPER(sub2)(uint32_t a, uint32_t b)
{
    return mkh(a - b, (a >> 16) - (b >> 16));
}

uint32_t HELPER(add4)(uint32_t a, uint32_t b)
{
    return mkb(ub(a, 0) + ub(b, 0), ub(a, 1) + ub(b, 1),
               ub(a, 2) + ub(b, 2), (ub(a, 3) + ub(b, 3)) & 0xff);
}

uint32_t HELPER(sub4)(uint32_t a, uint32_t b)
{
    return mkb(ub(a, 0) - ub(b, 0), ub(a, 1) - ub(b, 1),
               ub(a, 2) - ub(b, 2), (ub(a, 3) - ub(b, 3)) & 0xff);
}

static int32_t sat16(int32_t v)
{
    return MIN(MAX(v, -32768), 32767);
}

uint32_t HELPER(abs2)(uint32_t a)
{
    return mkh(sat16(abs(h(a, 0))), sat16(abs(h(a, 1))));
}

uint32_t HELPER(swap4)(uint32_t a)
{
    return mkb(ub(a, 1), ub(a, 0), ub(a, 3), ub(a, 2));
}

uint32_t HELPER(unpklu4)(uint32_t a)
{
    return ub(a, 0) | ub(a, 1) << 16;
}

uint32_t HELPER(unpkhu4)(uint32_t a)
{
    return ub(a, 2) | ub(a, 3) << 16;
}

uint32_t HELPER(pack2)(uint32_t a, uint32_t b)
{
    return mkh(b, a);
}

uint32_t HELPER(packlh2)(uint32_t a, uint32_t b)
{
    return mkh(b >> 16, a);
}

uint32_t HELPER(packhl2)(uint32_t a, uint32_t b)
{
    return mkh(b, a >> 16);
}

uint32_t HELPER(packh2)(uint32_t a, uint32_t b)
{
    return mkh(b >> 16, a >> 16);
}

uint32_t HELPER(packl4)(uint32_t a, uint32_t b)
{
    return mkb(ub(b, 0), ub(b, 2), ub(a, 0), ub(a, 2));
}

uint32_t HELPER(packh4)(uint32_t a, uint32_t b)
{
    return mkb(ub(b, 1), ub(b, 3), ub(a, 1), ub(a, 3));
}

uint32_t HELPER(min2)(uint32_t a, uint32_t b)
{
    return mkh(MIN(h(a, 0), h(b, 0)), MIN(h(a, 1), h(b, 1)));
}

uint32_t HELPER(max2)(uint32_t a, uint32_t b)
{
    return mkh(MAX(h(a, 0), h(b, 0)), MAX(h(a, 1), h(b, 1)));
}

uint32_t HELPER(minu4)(uint32_t a, uint32_t b)
{
    return mkb(MIN(ub(a, 0), ub(b, 0)), MIN(ub(a, 1), ub(b, 1)),
               MIN(ub(a, 2), ub(b, 2)), MIN(ub(a, 3), ub(b, 3)));
}

uint32_t HELPER(maxu4)(uint32_t a, uint32_t b)
{
    return mkb(MAX(ub(a, 0), ub(b, 0)), MAX(ub(a, 1), ub(b, 1)),
               MAX(ub(a, 2), ub(b, 2)), MAX(ub(a, 3), ub(b, 3)));
}

uint32_t HELPER(subabs4)(uint32_t a, uint32_t b)
{
    uint32_t r[4];

    for (int i = 0; i < 4; i++) {
        r[i] = abs((int)ub(a, i) - (int)ub(b, i));
    }
    return mkb(r[0], r[1], r[2], r[3]);
}

/* SUBC: conditional subtract and shift, for division */
uint32_t HELPER(subc)(uint32_t a, uint32_t b)
{
    return a >= b ? ((a - b) << 1) + 1 : a << 1;
}

/* LMBD: leftmost 1 or 0 bit, depending on bit 0 of src1 */
uint32_t HELPER(lmbd)(uint32_t a, uint32_t b)
{
    return clz32((a & 1) ? b : ~b);
}

uint32_t HELPER(shlmb)(uint32_t a, uint32_t b)
{
    return (b << 8) | ub(a, 3);
}

uint32_t HELPER(shrmb)(uint32_t a, uint32_t b)
{
    return (b >> 8) | (a << 24);
}

uint32_t HELPER(cmpeq2)(uint32_t a, uint32_t b)
{
    return (h(a, 0) == h(b, 0)) | (h(a, 1) == h(b, 1)) << 1;
}

uint32_t HELPER(cmpeq4)(uint32_t a, uint32_t b)
{
    uint32_t r = 0;

    for (int i = 0; i < 4; i++) {
        r |= (ub(a, i) == ub(b, i)) << i;
    }
    return r;
}

uint32_t HELPER(cmpgt2)(uint32_t a, uint32_t b)
{
    return (h(a, 0) > h(b, 0)) | (h(a, 1) > h(b, 1)) << 1;
}

uint32_t HELPER(cmpgtu4)(uint32_t a, uint32_t b)
{
    uint32_t r = 0;

    for (int i = 0; i < 4; i++) {
        r |= (ub(a, i) > ub(b, i)) << i;
    }
    return r;
}

/* SHR2/SHRU2 (amount, value) */
uint32_t HELPER(shr2)(uint32_t n, uint32_t v)
{
    n &= 31;
    n = MIN(n, 31);
    return mkh(h(v, 0) >> MIN(n, 15), h(v, 1) >> MIN(n, 15));
}

uint32_t HELPER(shru2)(uint32_t n, uint32_t v)
{
    n &= 31;
    if (n > 15) {
        return 0;
    }
    return mkh(uh(v, 0) >> n, uh(v, 1) >> n);
}

uint32_t HELPER(avg2)(uint32_t a, uint32_t b)
{
    return mkh((h(a, 0) + h(b, 0) + 1) >> 1, (h(a, 1) + h(b, 1) + 1) >> 1);
}

uint32_t HELPER(avgu4)(uint32_t a, uint32_t b)
{
    uint32_t r[4];

    for (int i = 0; i < 4; i++) {
        r[i] = (ub(a, i) + ub(b, i) + 1) >> 1;
    }
    return mkb(r[0], r[1], r[2], r[3]);
}

uint32_t HELPER(bitc4)(uint32_t a)
{
    return mkb(ctpop8(a), ctpop8(a >> 8), ctpop8(a >> 16), ctpop8(a >> 24));
}

uint32_t HELPER(bitr)(uint32_t a)
{
    return revbit32(a);
}

/* DEAL: odd bits to the upper, even bits to the lower half */
uint32_t HELPER(deal)(uint32_t a)
{
    uint32_t r = 0;

    for (int i = 0; i < 16; i++) {
        r |= ((a >> (2 * i)) & 1) << i;
        r |= ((a >> (2 * i + 1)) & 1) << (16 + i);
    }
    return r;
}

/* SHFL: interleave the halves, upper half to the odd bits */
uint32_t HELPER(shfl)(uint32_t a)
{
    uint32_t r = 0;

    for (int i = 0; i < 16; i++) {
        r |= ((a >> i) & 1) << (2 * i);
        r |= ((a >> (16 + i)) & 1) << (2 * i + 1);
    }
    return r;
}

uint32_t HELPER(xpnd2)(uint32_t a)
{
    return ((a & 1) ? 0xffff : 0) | ((a & 2) ? 0xffff0000 : 0);
}

uint32_t HELPER(xpnd4)(uint32_t a)
{
    uint32_t r = 0;

    for (int i = 0; i < 4; i++) {
        if (a & (1 << i)) {
            r |= 0xffu << (8 * i);
        }
    }
    return r;
}

static uint64_t shfl3(uint32_t a, uint32_t b)
{
    uint64_t r = 0;
    uint32_t in0 = b & 0xffff, in1 = a & 0xffff, in2 = a >> 16;

    for (int i = 0; i < 16; i++) {
        r |= (uint64_t)((in0 >> i) & 1) << (i * 3);
        r |= (uint64_t)((in1 >> i) & 1) << (i * 3 + 1);
        r |= (uint64_t)((in2 >> i) & 1) << (i * 3 + 2);
    }
    return r;
}

uint32_t HELPER(shfl3_lo)(uint32_t a, uint32_t b)
{
    return shfl3(a, b);
}

uint32_t HELPER(shfl3_hi)(uint32_t a, uint32_t b)
{
    return shfl3(a, b) >> 32;
}

/* Multiplies */

uint32_t HELPER(dotpsu4)(uint32_t a, uint32_t b)
{
    int32_t r = 0;

    for (int i = 0; i < 4; i++) {
        r += sb(a, i) * (int32_t)ub(b, i);
    }
    return r;
}

uint32_t HELPER(dotpu4)(uint32_t a, uint32_t b)
{
    uint32_t r = 0;

    for (int i = 0; i < 4; i++) {
        r += ub(a, i) * ub(b, i);
    }
    return r;
}

uint32_t HELPER(dotpnrsu2)(uint32_t a, uint32_t b)
{
    int64_t r = (int64_t)h(a, 1) * uh(b, 1) - (int64_t)h(a, 0) * uh(b, 0)
                + 0x8000;

    return r >> 16;
}

uint32_t HELPER(dotprsu2)(uint32_t a, uint32_t b)
{
    int64_t r = (int64_t)h(a, 1) * uh(b, 1) + (int64_t)h(a, 0) * uh(b, 0)
                + 0x8000;

    return r >> 16;
}

uint32_t HELPER(dotpn2)(uint32_t a, uint32_t b)
{
    return h(a, 1) * h(b, 1) - h(a, 0) * h(b, 0);
}

uint32_t HELPER(dotp2)(uint32_t a, uint32_t b)
{
    return (uint32_t)(h(a, 0) * h(b, 0)) + (uint32_t)(h(a, 1) * h(b, 1));
}

uint64_t HELPER(dotp2_64)(uint32_t a, uint32_t b)
{
    return (int64_t)h(a, 0) * h(b, 0) + (int64_t)h(a, 1) * h(b, 1);
}

uint32_t HELPER(mpylir)(uint32_t a, uint32_t b)
{
    return ((int64_t)h(a, 0) * (int32_t)b + 0x4000) >> 15;
}

uint32_t HELPER(mpyhir)(uint32_t a, uint32_t b)
{
    return ((int64_t)h(a, 1) * (int32_t)b + 0x4000) >> 15;
}

uint64_t HELPER(mpyhi)(uint32_t a, uint32_t b)
{
    return (int64_t)h(a, 1) * (int32_t)b;
}

uint64_t HELPER(mpyli)(uint32_t a, uint32_t b)
{
    return (int64_t)h(a, 0) * (int32_t)b;
}

uint64_t HELPER(mpy2)(uint32_t a, uint32_t b)
{
    return (uint32_t)(h(a, 0) * h(b, 0)) |
           (uint64_t)(uint32_t)(h(a, 1) * h(b, 1)) << 32;
}

uint64_t HELPER(mpyu4)(uint32_t a, uint32_t b)
{
    uint64_t r = 0;

    for (int i = 0; i < 4; i++) {
        r |= (uint64_t)(ub(a, i) * ub(b, i)) << (16 * i);
    }
    return r;
}

uint64_t HELPER(mpysu4)(uint32_t a, uint32_t b)
{
    uint64_t r = 0;

    for (int i = 0; i < 4; i++) {
        r |= (uint64_t)(uint16_t)(sb(a, i) * (int32_t)ub(b, i)) << (16 * i);
    }
    return r;
}

uint64_t HELPER(ddotp4)(uint32_t a, uint32_t b)
{
    uint32_t e = h(a, 1) * sb(b, 1) + h(a, 0) * sb(b, 0);
    uint32_t o = h(a, 1) * sb(b, 3) + h(a, 0) * sb(b, 2);

    return e | (uint64_t)o << 32;
}

static int32_t sat32(int64_t v, bool *sat)
{
    if (v > INT32_MAX) {
        *sat = true;
        return INT32_MAX;
    }
    if (v < INT32_MIN) {
        *sat = true;
        return INT32_MIN;
    }
    return v;
}

uint64_t HELPER(ddotpl2)(uint64_t a, uint32_t b)
{
    uint32_t ae = a, ao = a >> 32;
    bool sat = false;
    uint32_t e = sat32((int64_t)h(ae, 1) * h(b, 1) +
                       (int64_t)h(ae, 0) * h(b, 0), &sat);
    uint32_t o = sat32((int64_t)h(ao, 0) * h(b, 1) +
                       (int64_t)h(ae, 1) * h(b, 0), &sat);

    return e | (uint64_t)o << 32;
}

uint64_t HELPER(ddotph2)(uint64_t a, uint32_t b)
{
    uint32_t ae = a, ao = a >> 32;
    bool sat = false;
    uint32_t o = sat32((int64_t)h(ao, 1) * h(b, 1) +
                       (int64_t)h(ao, 0) * h(b, 0), &sat);
    uint32_t e = sat32((int64_t)h(ao, 0) * h(b, 1) +
                       (int64_t)h(ae, 1) * h(b, 0), &sat);

    return e | (uint64_t)o << 32;
}

/* Galois field multiply in GF(2^(size+1)) */
static uint32_t gf_mul(uint32_t a, uint32_t b, uint32_t poly, int m)
{
    uint32_t r = 0;

    for (int i = m - 1; i >= 0; i--) {
        r <<= 1;
        if (r & (1u << m)) {
            r ^= (1u << m) | poly;
        }
        if (b & (1u << i)) {
            r ^= a;
        }
    }
    return r & ((1u << m) - 1);
}

uint32_t HELPER(gmpy4)(uint32_t a, uint32_t b)
{
    /* default GFPGFR: size 7, polynomial 1Dh */
    uint32_t r[4];

    for (int i = 0; i < 4; i++) {
        r[i] = gf_mul(ub(a, i), ub(b, i), 0x1d, 8);
    }
    return mkb(r[0], r[1], r[2], r[3]);
}

static uint32_t gmpy(uint32_t src1, uint32_t src2, uint32_t poly)
{
    uint32_t pp = 0, mask = 0x100;

    for (int i = 0; i < 8; i++) {
        uint32_t tpp;

        if (src2 & mask) {
            pp ^= src1;
        }
        mask >>= 1;
        tpp = pp << 1;
        pp = (pp & 0x80000000) ? poly ^ tpp : tpp;
    }
    if (src2 & 1) {
        pp ^= src1;
    }
    return pp;
}

uint32_t HELPER(gmpy)(uint32_t a, uint32_t b, uint32_t poly)
{
    return gmpy(a, b, poly);
}

uint32_t HELPER(xormpy)(uint32_t a, uint32_t b)
{
    return gmpy(a, b, 0);
}

/* Saturating operations set CSR.SAT */

static void set_sat(CPUC6xState *env, bool sat)
{
    if (sat) {
        env->csr |= R_CSR_SAT_MASK;
    }
}

uint32_t HELPER(sadd)(CPUC6xState *env, uint32_t a, uint32_t b)
{
    bool sat = false;
    uint32_t r = sat32((int64_t)(int32_t)a + (int32_t)b, &sat);

    set_sat(env, sat);
    return r;
}

uint32_t HELPER(ssub)(CPUC6xState *env, uint32_t a, uint32_t b)
{
    bool sat = false;
    uint32_t r = sat32((int64_t)(int32_t)a - (int32_t)b, &sat);

    set_sat(env, sat);
    return r;
}

/* SSHL (value, amount); amounts over 31 saturate any nonzero value */
uint32_t HELPER(sshl)(CPUC6xState *env, uint32_t v, uint32_t n)
{
    int32_t s = v;
    int64_t r;
    bool sat = false;

    n &= 63;
    if (n > 31) {
        if (s == 0) {
            return 0;
        }
        set_sat(env, true);
        return s > 0 ? INT32_MAX : INT32_MIN;
    }
    r = (int64_t)s << n;
    v = sat32(r, &sat);
    set_sat(env, sat);
    return v;
}

static int32_t sat_n(int32_t v, int32_t lo, int32_t hi, bool *sat)
{
    if (v < lo) {
        *sat = true;
        return lo;
    }
    if (v > hi) {
        *sat = true;
        return hi;
    }
    return v;
}

uint32_t HELPER(sadd2)(CPUC6xState *env, uint32_t a, uint32_t b)
{
    bool sat = false;
    uint32_t r = mkh(sat_n(h(a, 0) + h(b, 0), -32768, 32767, &sat),
                     sat_n(h(a, 1) + h(b, 1), -32768, 32767, &sat));

    set_sat(env, sat);
    return r;
}

uint32_t HELPER(ssub2)(CPUC6xState *env, uint32_t a, uint32_t b)
{
    bool sat = false;
    uint32_t r = mkh(sat_n(h(a, 0) - h(b, 0), -32768, 32767, &sat),
                     sat_n(h(a, 1) - h(b, 1), -32768, 32767, &sat));

    set_sat(env, sat);
    return r;
}

uint32_t HELPER(saddus2)(CPUC6xState *env, uint32_t a, uint32_t b)
{
    bool sat = false;
    uint32_t r = mkh(sat_n(uh(a, 0) + h(b, 0), 0, 65535, &sat),
                     sat_n(uh(a, 1) + h(b, 1), 0, 65535, &sat));

    set_sat(env, sat);
    return r;
}

uint32_t HELPER(saddu4)(CPUC6xState *env, uint32_t a, uint32_t b)
{
    bool sat = false;
    uint32_t r[4];

    for (int i = 0; i < 4; i++) {
        r[i] = sat_n(ub(a, i) + ub(b, i), 0, 255, &sat);
    }
    set_sat(env, sat);
    return mkb(r[0], r[1], r[2], r[3]);
}

uint32_t HELPER(spack2)(CPUC6xState *env, uint32_t a, uint32_t b)
{
    bool sat = false;
    uint32_t r = mkh(sat_n(b, -32768, 32767, &sat),
                     sat_n(a, -32768, 32767, &sat));

    set_sat(env, sat);
    return r;
}

uint32_t HELPER(spacku4)(CPUC6xState *env, uint32_t a, uint32_t b)
{
    bool sat = false;
    uint32_t r = mkb(sat_n(h(b, 0), 0, 255, &sat), sat_n(h(b, 1), 0, 255, &sat),
                     sat_n(h(a, 0), 0, 255, &sat), sat_n(h(a, 1), 0, 255, &sat));

    set_sat(env, sat);
    return r;
}

uint32_t HELPER(rpack2)(CPUC6xState *env, uint32_t a, uint32_t b)
{
    bool sat = false;
    uint32_t sa = sat32((int64_t)(int32_t)a << 1, &sat);
    uint32_t sbv = sat32((int64_t)(int32_t)b << 1, &sat);

    set_sat(env, sat);
    return (sa & 0xffff0000) | (sbv >> 16);
}

static uint32_t smpy16(CPUC6xState *env, int32_t x, int32_t y)
{
    uint32_t r = (uint32_t)(x * y) << 1;

    if (r == 0x80000000) {
        set_sat(env, true);
        return 0x7fffffff;
    }
    return r;
}

uint32_t HELPER(smpy)(CPUC6xState *env, uint32_t a, uint32_t b)
{
    return smpy16(env, h(a, 0), h(b, 0));
}

uint32_t HELPER(smpyh)(CPUC6xState *env, uint32_t a, uint32_t b)
{
    return smpy16(env, h(a, 1), h(b, 1));
}

uint32_t HELPER(smpyhl)(CPUC6xState *env, uint32_t a, uint32_t b)
{
    return smpy16(env, h(a, 1), h(b, 0));
}

uint32_t HELPER(smpylh)(CPUC6xState *env, uint32_t a, uint32_t b)
{
    return smpy16(env, h(a, 0), h(b, 1));
}

uint32_t HELPER(smpy32)(CPUC6xState *env, uint32_t a, uint32_t b)
{
    int64_t p = (int64_t)(int32_t)a * (int32_t)b;

    if (p == (int64_t)1 << 62) {
        set_sat(env, true);
        return 0x7fffffff;
    }
    return (uint64_t)(p << 1) >> 32;
}

/* SSHVL/SSHVR (value, signed amount) */
uint32_t HELPER(sshvl)(CPUC6xState *env, uint32_t v, uint32_t n)
{
    int32_t s = n;

    if (s < 0) {
        return (int32_t)v >> MIN(-s, 31);
    }
    return helper_sshl(env, v, MIN(s, 31));
}

uint32_t HELPER(sshvr)(CPUC6xState *env, uint32_t v, uint32_t n)
{
    int32_t s = n;

    if (s >= 0) {
        return (int32_t)v >> MIN(s, 31);
    }
    return helper_sshl(env, v, MIN(-s, 31));
}

uint64_t HELPER(smpy2)(CPUC6xState *env, uint32_t a, uint32_t b)
{
    uint32_t lo = smpy16(env, h(a, 0), h(b, 0));
    uint32_t hi = smpy16(env, h(a, 1), h(b, 1));

    return lo | (uint64_t)hi << 32;
}

uint64_t HELPER(cmpy)(CPUC6xState *env, uint32_t a, uint32_t b)
{
    bool sat = false;
    uint32_t e = sat32((int64_t)h(a, 0) * h(b, 1) +
                       (int64_t)h(a, 1) * h(b, 0), &sat);
    uint32_t o = h(a, 1) * h(b, 1) - h(a, 0) * h(b, 0);

    set_sat(env, sat);
    return e | (uint64_t)o << 32;
}

uint32_t HELPER(cmpyr)(CPUC6xState *env, uint32_t a, uint32_t b)
{
    bool sat = false;
    int32_t e = sat32((int64_t)h(a, 0) * h(b, 1) +
                      (int64_t)h(a, 1) * h(b, 0), &sat);
    int32_t o = sat32((int64_t)h(a, 1) * h(b, 1) -
                      (int64_t)h(a, 0) * h(b, 0), &sat);

    e = sat32((int64_t)e + 0x8000, &sat);
    o = sat32((int64_t)o + 0x8000, &sat);
    set_sat(env, sat);
    return ((uint32_t)e >> 16) | ((uint32_t)o & 0xffff0000);
}

uint32_t HELPER(cmpyr1)(CPUC6xState *env, uint32_t a, uint32_t b)
{
    bool sat = false;
    int32_t e = sat32((int64_t)h(a, 0) * h(b, 1) +
                      (int64_t)h(a, 1) * h(b, 0), &sat);
    int32_t o = sat32((int64_t)h(a, 1) * h(b, 1) -
                      (int64_t)h(a, 0) * h(b, 0), &sat);

    e = sat32(((int64_t)e + 0x4000) << 1, &sat);
    o = sat32(((int64_t)o + 0x4000) << 1, &sat);
    set_sat(env, sat);
    return ((uint32_t)e >> 16) | ((uint32_t)o & 0xffff0000);
}

static uint32_t mpyir(CPUC6xState *env, int32_t x, int32_t y)
{
    if (x == -32768 && y == INT32_MIN) {
        set_sat(env, true);
        return 0x7fffffff;
    }
    return ((int64_t)x * y + 0x4000) >> 15;
}

uint64_t HELPER(mpy2ir)(CPUC6xState *env, uint32_t a, uint32_t b)
{
    uint32_t e = mpyir(env, h(a, 0), b);
    uint32_t o = mpyir(env, h(a, 1), b);

    return e | (uint64_t)o << 32;
}

uint32_t HELPER(ddotpl2r)(CPUC6xState *env, uint64_t a, uint32_t b)
{
    uint32_t ae = a, ao = a >> 32;
    bool sat = false;
    int32_t lo = sat32((int64_t)h(ae, 1) * h(b, 1) +
                       (int64_t)h(ae, 0) * h(b, 0) + 0x8000, &sat);
    int32_t hi = sat32((int64_t)h(ao, 0) * h(b, 1) +
                       (int64_t)h(ae, 1) * h(b, 0) + 0x8000, &sat);

    set_sat(env, sat);
    return ((uint32_t)lo >> 16) | ((uint32_t)hi & 0xffff0000);
}

uint32_t HELPER(ddotph2r)(CPUC6xState *env, uint64_t a, uint32_t b)
{
    uint32_t ae = a, ao = a >> 32;
    bool sat = false;
    int32_t hi = sat32((int64_t)h(ao, 1) * h(b, 1) +
                       (int64_t)h(ao, 0) * h(b, 0) + 0x8000, &sat);
    int32_t lo = sat32((int64_t)h(ao, 0) * h(b, 1) +
                       (int64_t)h(ae, 1) * h(b, 0) + 0x8000, &sat);

    set_sat(env, sat);
    return ((uint32_t)lo >> 16) | ((uint32_t)hi & 0xffff0000);
}

/* 40-bit longs, sign extended into int64_t */

static int64_t sat40(int64_t v, bool *sat)
{
    const int64_t max = ((int64_t)1 << 39) - 1;

    if (v > max) {
        *sat = true;
        return max;
    }
    if (v < -max - 1) {
        *sat = true;
        return -max - 1;
    }
    return v;
}

uint64_t HELPER(sadd40)(CPUC6xState *env, uint64_t a, uint64_t b)
{
    bool sat = false;
    int64_t r = sat40((int64_t)a + (int64_t)b, &sat);

    set_sat(env, sat);
    return r;
}

uint64_t HELPER(ssub40)(CPUC6xState *env, uint64_t a, uint64_t b)
{
    bool sat = false;
    int64_t r = sat40((int64_t)a - (int64_t)b, &sat);

    set_sat(env, sat);
    return r;
}

uint64_t HELPER(abs40)(CPUC6xState *env, uint64_t a)
{
    bool sat = false;
    int64_t r = sat40(llabs((int64_t)a), &sat);

    set_sat(env, sat);
    return r;
}

uint32_t HELPER(sat40)(CPUC6xState *env, uint64_t a)
{
    bool sat = false;
    uint32_t r = sat32((int64_t)a, &sat);

    set_sat(env, sat);
    return r;
}

/*
 * Floating point.  The C67x+ units treat denormal inputs as zero, flush
 * denormal results to zero and return an all-ones NaN.  Flags and the
 * rounding modes of FADCR/FAUCR/FMCR are not modelled; everything rounds
 * to nearest.
 */

#define NAN_SP  0x7fffffffu
#define NAN_DP  0x7fffffffffffffffull

static float_status *fps(CPUC6xState *env)
{
    return &env->fp_status;
}

static uint32_t sp_out(float32 r)
{
    return float32_is_any_nan(r) ? NAN_SP : float32_val(r);
}

static uint64_t dp_out(float64 r)
{
    return float64_is_any_nan(r) ? NAN_DP : float64_val(r);
}

/* Products give a signed NaN_out: the sign is that of the operands' XOR */
static uint32_t mpy_sp_out(float32 r, uint32_t sign)
{
    return float32_is_any_nan(r) ? NAN_SP | sign : float32_val(r);
}

static uint64_t mpy_dp_out(float64 r, uint64_t sign)
{
    return float64_is_any_nan(r) ? NAN_DP | sign : float64_val(r);
}

uint32_t HELPER(addsp)(CPUC6xState *env, uint32_t a, uint32_t b)
{
    return sp_out(float32_add(make_float32(a), make_float32(b), fps(env)));
}

uint32_t HELPER(subsp)(CPUC6xState *env, uint32_t a, uint32_t b)
{
    return sp_out(float32_sub(make_float32(a), make_float32(b), fps(env)));
}

uint32_t HELPER(mpysp)(CPUC6xState *env, uint32_t a, uint32_t b)
{
    return mpy_sp_out(float32_mul(make_float32(a), make_float32(b), fps(env)),
                      (a ^ b) & 0x80000000u);
}

uint64_t HELPER(adddp)(CPUC6xState *env, uint64_t a, uint64_t b)
{
    return dp_out(float64_add(make_float64(a), make_float64(b), fps(env)));
}

uint64_t HELPER(subdp)(CPUC6xState *env, uint64_t a, uint64_t b)
{
    return dp_out(float64_sub(make_float64(a), make_float64(b), fps(env)));
}

uint64_t HELPER(mpydp)(CPUC6xState *env, uint64_t a, uint64_t b)
{
    return mpy_dp_out(float64_mul(make_float64(a), make_float64(b), fps(env)),
                      (a ^ b) & (1ull << 63));
}

uint64_t HELPER(mpyspdp)(CPUC6xState *env, uint32_t a, uint64_t b)
{
    float64 x = float32_to_float64(make_float32(a), fps(env));

    return mpy_dp_out(float64_mul(x, make_float64(b), fps(env)),
                      ((uint64_t)a << 32 ^ b) & (1ull << 63));
}

uint64_t HELPER(mpysp2dp)(CPUC6xState *env, uint32_t a, uint32_t b)
{
    float64 x = float32_to_float64(make_float32(a), fps(env));
    float64 y = float32_to_float64(make_float32(b), fps(env));

    return mpy_dp_out(float64_mul(x, y, fps(env)),
                      (uint64_t)((a ^ b) & 0x80000000u) << 32);
}

/* op: 0 EQ, 1 GT, 2 LT */
uint32_t HELPER(cmpsp)(CPUC6xState *env, uint32_t a, uint32_t b, uint32_t op)
{
    FloatRelation r = float32_compare_quiet(make_float32(a), make_float32(b),
                                            fps(env));

    switch (op) {
    case 0:
        return r == float_relation_equal;
    case 1:
        return r == float_relation_greater;
    default:
        return r == float_relation_less;
    }
}

uint32_t HELPER(cmpdp)(CPUC6xState *env, uint64_t a, uint64_t b, uint32_t op)
{
    FloatRelation r = float64_compare_quiet(make_float64(a), make_float64(b),
                                            fps(env));

    switch (op) {
    case 0:
        return r == float_relation_equal;
    case 1:
        return r == float_relation_greater;
    default:
        return r == float_relation_less;
    }
}

uint32_t HELPER(abssp)(CPUC6xState *env, uint32_t a)
{
    float32 x = float32_squash_input_denormal(make_float32(a), fps(env));

    if (float32_is_any_nan(x)) {
        return NAN_SP;
    }
    return float32_val(float32_abs(x));
}

uint64_t HELPER(absdp)(CPUC6xState *env, uint64_t a)
{
    float64 x = float64_squash_input_denormal(make_float64(a), fps(env));

    if (float64_is_any_nan(x)) {
        return NAN_DP;
    }
    return float64_val(float64_abs(x));
}

/* Conversions to integer saturate; NaN gives 7FFF FFFFh */
uint32_t HELPER(spint)(CPUC6xState *env, uint32_t a)
{
    if (float32_is_any_nan(make_float32(a))) {
        return 0x7fffffff;
    }
    return float32_to_int32(make_float32(a), fps(env));
}

uint32_t HELPER(sptrunc)(CPUC6xState *env, uint32_t a)
{
    if (float32_is_any_nan(make_float32(a))) {
        return 0x7fffffff;
    }
    return float32_to_int32_round_to_zero(make_float32(a), fps(env));
}

uint32_t HELPER(dpint)(CPUC6xState *env, uint64_t a)
{
    if (float64_is_any_nan(make_float64(a))) {
        return 0x7fffffff;
    }
    return float64_to_int32(make_float64(a), fps(env));
}

uint32_t HELPER(dptrunc)(CPUC6xState *env, uint64_t a)
{
    if (float64_is_any_nan(make_float64(a))) {
        return 0x7fffffff;
    }
    return float64_to_int32_round_to_zero(make_float64(a), fps(env));
}

uint32_t HELPER(intsp)(CPUC6xState *env, uint32_t a)
{
    return float32_val(int32_to_float32(a, fps(env)));
}

uint32_t HELPER(intspu)(CPUC6xState *env, uint32_t a)
{
    return float32_val(uint32_to_float32(a, fps(env)));
}

uint64_t HELPER(intdp)(CPUC6xState *env, uint32_t a)
{
    return float64_val(int32_to_float64(a, fps(env)));
}

uint64_t HELPER(intdpu)(CPUC6xState *env, uint32_t a)
{
    return float64_val(uint32_to_float64(a, fps(env)));
}

uint64_t HELPER(spdp)(CPUC6xState *env, uint32_t a)
{
    return dp_out(float32_to_float64(make_float32(a), fps(env)));
}

uint32_t HELPER(dpsp)(CPUC6xState *env, uint64_t a)
{
    return sp_out(float64_to_float32(make_float64(a), fps(env)));
}

/*
 * The reciprocal and reciprocal square root approximations are exact
 * to 8 mantissa bits.  Truncating the correctly rounded result to that
 * precision is close enough for the Newton-Raphson steps that follow.
 */
uint32_t HELPER(rcpsp)(CPUC6xState *env, uint32_t a)
{
    float32 r = float32_div(float32_one, make_float32(a), fps(env));

    if (float32_is_any_nan(r)) {
        return NAN_SP;
    }
    if (float32_is_infinity(r) || float32_is_zero(r)) {
        return float32_val(r);
    }
    return float32_val(r) & ~0x7fffu;
}

uint32_t HELPER(rsqrsp)(CPUC6xState *env, uint32_t a)
{
    float32 s = float32_sqrt(make_float32(a), fps(env));
    float32 r = float32_div(float32_one, s, fps(env));

    if (float32_is_any_nan(r)) {
        return NAN_SP;
    }
    if (float32_is_infinity(r) || float32_is_zero(r)) {
        return float32_val(r);
    }
    return float32_val(r) & ~0x7fffu;
}

uint64_t HELPER(rcpdp)(CPUC6xState *env, uint64_t a)
{
    float64 r = float64_div(float64_one, make_float64(a), fps(env));

    if (float64_is_any_nan(r)) {
        return NAN_DP;
    }
    if (float64_is_infinity(r) || float64_is_zero(r)) {
        return float64_val(r);
    }
    return float64_val(r) & ~((1ull << 44) - 1);
}

uint64_t HELPER(rsqrdp)(CPUC6xState *env, uint64_t a)
{
    float64 s = float64_sqrt(make_float64(a), fps(env));
    float64 r = float64_div(float64_one, s, fps(env));

    if (float64_is_any_nan(r)) {
        return NAN_DP;
    }
    if (float64_is_infinity(r) || float64_is_zero(r)) {
        return float64_val(r);
    }
    return float64_val(r) & ~((1ull << 44) - 1);
}
