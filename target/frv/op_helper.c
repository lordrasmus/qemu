/*
 * FR-V instruction helpers
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "cpu.h"
#include "exec/helper-proto.h"
#include "accel/tcg/cpu-loop.h"
#include "accel/tcg/cpu-ldst.h"

G_NORETURN void frv_raise(CPUFRVState *env, int tt, uintptr_t ra)
{
    CPUState *cs = env_cpu(env);

    cs->exception_index = tt;
    cpu_loop_exit_restore(cs, ra);
}

void helper_raise(CPUFRVState *env, uint32_t tt)
{
    frv_raise(env, tt, GETPC());
}

/* tira and friends: trap 0x80 + the low seven bits of the sum */
void helper_trap(CPUFRVState *env, uint32_t val)
{
    frv_raise(env, TT_TRAP + (val & 0x7f), GETPC());
}

static G_NORETURN void division_exception(CPUFRVState *env, uint32_t dtt,
                                          uintptr_t ra)
{
    env->excp_dtt = dtt;
    frv_raise(env, TT_DIVISION, ra);
}

uint32_t helper_sdiv(CPUFRVState *env, uint32_t a, uint32_t b)
{
    if (a == 0x80000000 && b == 0xffffffff) {
        /* with ISR.EDE the overflow is only recorded */
        if (env->spr[SPR_ISR] & ISR_EDE) {
            env->spr[SPR_ISR] |= ISR_AEXC;
            return 0x7fffffff;
        }
        division_exception(env, ISR_DTT_OVERFLOW, GETPC());
    }
    if (b == 0) {
        division_exception(env, ISR_DTT_DIVBYZERO, GETPC());
    }
    return (int32_t)a / (int32_t)b;
}

uint32_t helper_udiv(CPUFRVState *env, uint32_t a, uint32_t b)
{
    if (b == 0) {
        division_exception(env, ISR_DTT_DIVBYZERO, GETPC());
    }
    return a / b;
}

uint32_t helper_movsg(CPUFRVState *env, uint32_t spr)
{
    return frv_spr_read(env, spr);
}

void helper_movgs(CPUFRVState *env, uint32_t spr, uint32_t val)
{
    /* a pending interrupt is rechecked when the TB ends after this */
    frv_spr_write(env, spr, val);
}

/*
 * rett: back from a trap (debug 0) or from the break handler (debug 1).
 * Returns the new PC; anything else than these two cases is illegal.
 */
uint32_t helper_rett(CPUFRVState *env, uint32_t debug)
{
    uint32_t psr = env->spr[SPR_PSR];

    if (!FIELD_EX32(psr, PSR, S)) {
        frv_raise(env, TT_PRIV_INSN, GETPC());
    }
    if (FIELD_EX32(psr, PSR, ET)) {
        frv_raise(env, TT_ILLEGAL_INSN, GETPC());
    }
    if (!env->debug_mode && !debug) {
        psr = FIELD_DP32(psr, PSR, ET, 1);
        psr = FIELD_DP32(psr, PSR, S, FIELD_EX32(psr, PSR, PS));
        env->spr[SPR_PSR] = psr;
        return env->spr[SPR_PCSR];
    }
    if (env->debug_mode && debug) {
        uint32_t bpsr = env->spr[SPR_BPSR];

        psr = FIELD_DP32(psr, PSR, ET, !!(bpsr & BPSR_BET));
        psr = FIELD_DP32(psr, PSR, S, !!(bpsr & BPSR_BS));
        env->spr[SPR_PSR] = psr;
        env->debug_mode = false;
        return env->spr[SPR_BPCSR];
    }
    frv_raise(env, TT_ILLEGAL_INSN, GETPC());
}

/*
 * andcr, orcr, ...: each CC holds 0/1 (undefined), 2 (false) or 3
 * (true).  The result for every pair of inputs, rows CRi, columns CRj.
 */
#define U 0
#define F 2
#define T 3
static const uint8_t cr_logic[9][4][4] = {
    { {U, U, U, U}, {U, U, U, U}, {U, U, U, U}, {U, U, F, T} }, /* andcr */
    { {U, U, F, T}, {U, U, F, T}, {F, F, F, T}, {T, T, T, T} }, /* orcr */
    { {U, U, U, U}, {U, U, U, U}, {U, U, F, T}, {T, T, T, F} }, /* xorcr */
    { {U, U, U, U}, {U, U, U, U}, {U, U, U, U}, {U, U, T, F} }, /* nandcr */
    { {U, U, T, F}, {U, U, T, F}, {T, T, T, F}, {F, F, F, F} }, /* norcr */
    { {U, U, U, U}, {U, U, U, U}, {U, U, F, T}, {U, U, U, U} }, /* andncr */
    { {U, U, F, T}, {U, U, F, T}, {T, T, T, T}, {F, F, F, T} }, /* orncr */
    { {U, U, U, U}, {U, U, U, U}, {U, U, T, F}, {U, U, U, U} }, /* nandncr */
    { {U, U, T, F}, {U, U, T, F}, {F, F, F, F}, {T, T, T, F} }, /* norncr */
};
#undef U
#undef F
#undef T

uint32_t helper_crlogic(uint32_t op, uint32_t a, uint32_t b)
{
    return op < 9 ? cr_logic[op][a & 3][b & 3] : 0;
}

static uint32_t sat_or(uint32_t res, uint32_t a, bool ovf)
{
    if (!ovf) {
        return res;
    }
    return (int32_t)a > 0 ? 0x7fffffff : (int32_t)a < 0 ? 0x80000000 : 0;
}

uint32_t helper_addss(uint32_t a, uint32_t b)
{
    uint32_t r = a + b;
    return sat_or(r, a, ((a ^ r) & (b ^ r)) >> 31);
}

uint32_t helper_subss(uint32_t a, uint32_t b)
{
    uint32_t r = a - b;
    return sat_or(r, a, ((a ^ b) & (a ^ r)) >> 31);
}

uint32_t helper_slass(uint32_t a, uint32_t b)
{
    int32_t x = a, n = b;
    bool neg = x < 0;

    if (n <= 0 || x == 0) {
        return x;
    }
    if (n >= 31) {
        return x > 0 ? 0x7fffffff : 0x80000000;
    }
    while (n-- > 0) {
        x = (uint32_t)x << 1;
        if (neg && x >= 0) {
            return 0x80000000;
        } else if (!neg && x < 0) {
            return 0x7fffffff;
        }
    }
    return x;
}

static int64_t iacc_get(CPUFRVState *env)
{
    return (int64_t)((uint64_t)env->spr[SPR_IACC0H] << 32 |
                     env->spr[SPR_IACC0L]);
}

static void iacc_set(CPUFRVState *env, int64_t v)
{
    env->spr[SPR_IACC0H] = (uint64_t)v >> 32;
    env->spr[SPR_IACC0L] = v;
}

/* smu (op 0), smass (op 1), smsss (op 2): IACC0 = / += / -= product */
void helper_iacc_mul(CPUFRVState *env, uint32_t op, uint32_t a, uint32_t b)
{
    int64_t p = (int64_t)(int32_t)a * (int32_t)b;
    int64_t acc = iacc_get(env), r;

    switch (op) {
    case 0:
        r = p;
        break;
    case 1:
        if (__builtin_add_overflow(acc, p, &r)) {
            r = p > 0 ? INT64_MAX : INT64_MIN;
        }
        break;
    default:
        if (__builtin_sub_overflow(acc, p, &r)) {
            r = p < 0 ? INT64_MAX : INT64_MIN;
        }
        break;
    }
    iacc_set(env, r);
}

/* scutss: bits of IACC0 at the cut point GRj, rounded and saturated */
uint32_t helper_scutss(CPUFRVState *env, uint32_t j)
{
    int64_t acc = iacc_get(env), lower, upper;
    int cut = (int32_t)(j << 25) >> 25;

    if (cut < 0) {
        acc = cut == -64 ? acc >> 63 : acc >> -cut;
        cut = 0;
    }
    if (cut < 32) {
        lower = acc >> (32 - cut);
        if (lower < 0x7fffffff && (acc & (1ull << (31 - cut)))) {
            lower++;
        }
        upper = lower >> 31;
    } else {
        lower = (uint64_t)acc << (cut - 32);
        upper = acc >> (63 - cut);
    }
    if (upper < -1) {
        return 0x80000000;
    } else if (upper > 0) {
        return 0x7fffffff;
    }
    return lower;
}
