/*
 * IA-64 integer helpers: multimedia instructions, czx, speculative loads
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/helper-proto.h"
#include "accel/tcg/cpu-ldst.h"
#include "accel/tcg/probe.h"
#include "exec/tlb-flags.h"
#include "ia64-disasm-int.h"
#include "ia64-disasm.h"

/* czx1/czx2: index of the first zero element from the left or right */
uint64_t HELPER(czx)(uint64_t v, uint32_t half, uint32_t left)
{
    int n = half ? 4 : 8;
    int w = half ? 16 : 8;
    uint64_t m = half ? 0xffff : 0xff;

    for (int i = 0; i < n; i++) {
        int e = left ? n - 1 - i : i;

        if (((v >> (e * w)) & m) == 0) {
            return i;
        }
    }
    return n;
}

static int64_t sat(int64_t v, int64_t lo, int64_t hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

static uint64_t elem(uint64_t v, int i, int w)
{
    return (v >> (i * w)) & ((w == 64) ? ~0ull : (1ull << w) - 1);
}

static int64_t selem(uint64_t v, int i, int w)
{
    return sextract64(v, i * w, w);
}

static uint64_t put(uint64_t r, int i, int w, uint64_t x)
{
    return deposit64(r, i * w, w, x);
}

/* add/sub with the saturation kinds of padd/psub */
enum { SAT_NONE, SAT_SSS, SAT_UUS, SAT_UUU };

static uint64_t padd(uint64_t a, uint64_t b, int w, int satk, bool sub)
{
    uint64_t r = 0;
    int n = 64 / w;
    int64_t smax = (1ll << (w - 1)) - 1, smin = -(1ll << (w - 1));
    int64_t umax = (1ll << w) - 1;

    for (int i = 0; i < n; i++) {
        int64_t x, y, v;

        switch (satk) {
        case SAT_SSS:
            x = selem(a, i, w);
            y = selem(b, i, w);
            v = sat(sub ? x - y : x + y, smin, smax);
            break;
        case SAT_UUS:
            x = elem(a, i, w);
            y = selem(b, i, w);
            v = sat(sub ? x - y : x + y, 0, umax);
            break;
        case SAT_UUU:
            x = elem(a, i, w);
            y = elem(b, i, w);
            v = sat(sub ? x - y : x + y, 0, umax);
            break;
        default:
            x = elem(a, i, w);
            y = elem(b, i, w);
            v = sub ? x - y : x + y;
            break;
        }
        r = put(r, i, w, v);
    }
    return r;
}

static uint64_t mux1(uint64_t v, int mbt)
{
    uint8_t x[8], y[8];
    static const uint8_t mix[8] = { 0, 4, 2, 6, 1, 5, 3, 7 };
    static const uint8_t shuf[8] = { 0, 4, 1, 5, 2, 6, 3, 7 };
    static const uint8_t alt[8] = { 0, 2, 4, 6, 1, 3, 5, 7 };
    uint64_t r = 0;

    for (int i = 0; i < 8; i++) {
        x[i] = v >> (i * 8);
    }
    for (int i = 0; i < 8; i++) {
        switch (mbt) {
        case 0x0: y[i] = x[0]; break;          /* @brcst */
        case 0x8: y[i] = x[mix[i]]; break;
        case 0x9: y[i] = x[shuf[i]]; break;
        case 0xa: y[i] = x[alt[i]]; break;
        case 0xb: y[i] = x[7 - i]; break;      /* @rev */
        default: y[i] = 0; break;
        }
        r |= (uint64_t)y[i] << (i * 8);
    }
    return r;
}

/*
 * Multimedia operations.  a/b are r2/r3 (or r3/count for the shifts by
 * count, see translate-int.c.inc), imm is a format specific immediate.
 */
uint64_t HELPER(mm)(CPUIA64State *env, uint32_t op, uint64_t a, uint64_t b,
                    uint32_t imm)
{
    uint64_t r = 0;

    switch (op) {
    case ASM_OP_PADD1_: return padd(a, b, 8, SAT_NONE, false);
    case ASM_OP_PADD1_SSS: return padd(a, b, 8, SAT_SSS, false);
    case ASM_OP_PADD1_UUS: return padd(a, b, 8, SAT_UUS, false);
    case ASM_OP_PADD1_UUU: return padd(a, b, 8, SAT_UUU, false);
    case ASM_OP_PADD2_: return padd(a, b, 16, SAT_NONE, false);
    case ASM_OP_PADD2_SSS: return padd(a, b, 16, SAT_SSS, false);
    case ASM_OP_PADD2_UUS: return padd(a, b, 16, SAT_UUS, false);
    case ASM_OP_PADD2_UUU: return padd(a, b, 16, SAT_UUU, false);
    case ASM_OP_PADD4: return padd(a, b, 32, SAT_NONE, false);
    case ASM_OP_PSUB1_: return padd(a, b, 8, SAT_NONE, true);
    case ASM_OP_PSUB1_SSS: return padd(a, b, 8, SAT_SSS, true);
    case ASM_OP_PSUB1_UUS: return padd(a, b, 8, SAT_UUS, true);
    case ASM_OP_PSUB1_UUU: return padd(a, b, 8, SAT_UUU, true);
    case ASM_OP_PSUB2_: return padd(a, b, 16, SAT_NONE, true);
    case ASM_OP_PSUB2_SSS: return padd(a, b, 16, SAT_SSS, true);
    case ASM_OP_PSUB2_UUS: return padd(a, b, 16, SAT_UUS, true);
    case ASM_OP_PSUB2_UUU: return padd(a, b, 16, SAT_UUU, true);
    case ASM_OP_PSUB4: return padd(a, b, 32, SAT_NONE, true);

    case ASM_OP_PAVG1_: case ASM_OP_PAVG1_RAZ:
    case ASM_OP_PAVG2_: case ASM_OP_PAVG2_RAZ: {
        int w = (op == ASM_OP_PAVG1_ || op == ASM_OP_PAVG1_RAZ) ? 8 : 16;
        bool raz = op == ASM_OP_PAVG1_RAZ || op == ASM_OP_PAVG2_RAZ;

        for (int i = 0; i < 64 / w; i++) {
            uint64_t t = elem(a, i, w) + elem(b, i, w);

            r = put(r, i, w, raz ? (t + 1) >> 1 : (t >> 1) | (t & 1));
        }
        return r;
    }
    case ASM_OP_PAVGSUB1: case ASM_OP_PAVGSUB2: {
        int w = op == ASM_OP_PAVGSUB1 ? 8 : 16;

        for (int i = 0; i < 64 / w; i++) {
            uint64_t t = (elem(a, i, w) - elem(b, i, w)) &
                         ((1ull << (w + 1)) - 1);

            r = put(r, i, w, (t >> 1) | (t & 1));
        }
        return r;
    }
    case ASM_OP_PCMP1_EQ: case ASM_OP_PCMP2_EQ: case ASM_OP_PCMP4_EQ:
    case ASM_OP_PCMP1_GT: case ASM_OP_PCMP2_GT: case ASM_OP_PCMP4_GT: {
        int w = (op == ASM_OP_PCMP1_EQ || op == ASM_OP_PCMP1_GT) ? 8 :
                (op == ASM_OP_PCMP2_EQ || op == ASM_OP_PCMP2_GT) ? 16 : 32;
        bool gt = op == ASM_OP_PCMP1_GT || op == ASM_OP_PCMP2_GT ||
                  op == ASM_OP_PCMP4_GT;

        for (int i = 0; i < 64 / w; i++) {
            bool t = gt ? selem(a, i, w) > selem(b, i, w)
                        : elem(a, i, w) == elem(b, i, w);

            r = put(r, i, w, t ? ~0ull : 0);
        }
        return r;
    }
    case ASM_OP_PSHLADD2: case ASM_OP_PSHRADD2:
        for (int i = 0; i < 4; i++) {
            int64_t x = selem(a, i, 16);
            int64_t t = op == ASM_OP_PSHLADD2 ?
                        sat(x << imm, -32768, 32767) : x >> imm;

            r = put(r, i, 16, sat(t + selem(b, i, 16), -32768, 32767));
        }
        return r;

    case ASM_OP_PMPYSHR2_: case ASM_OP_PMPYSHR2_U:
        for (int i = 0; i < 4; i++) {
            int64_t p = op == ASM_OP_PMPYSHR2_ ?
                        selem(a, i, 16) * selem(b, i, 16) :
                        (int64_t)(elem(a, i, 16) * elem(b, i, 16));

            r = put(r, i, 16, p >> imm);
        }
        return r;
    case ASM_OP_PMPY2_R:
    case ASM_OP_PMPY2_L: {
        int o = op == ASM_OP_PMPY2_L;

        r = put(r, 0, 32, selem(a, o, 16) * selem(b, o, 16));
        r = put(r, 1, 32, selem(a, 2 + o, 16) * selem(b, 2 + o, 16));
        return r;
    }
    case ASM_OP_PSAD1:
        for (int i = 0; i < 8; i++) {
            int64_t d = (int64_t)elem(a, i, 8) - (int64_t)elem(b, i, 8);

            r += d < 0 ? -d : d;
        }
        return r;
    case ASM_OP_PMIN1_U: case ASM_OP_PMAX1_U:
        for (int i = 0; i < 8; i++) {
            uint64_t x = elem(a, i, 8), y = elem(b, i, 8);
            bool mx = op == ASM_OP_PMAX1_U;

            r = put(r, i, 8, (mx ? x > y : x < y) ? x : y);
        }
        return r;
    case ASM_OP_PMIN2: case ASM_OP_PMAX2:
        for (int i = 0; i < 4; i++) {
            int64_t x = selem(a, i, 16), y = selem(b, i, 16);
            bool mx = op == ASM_OP_PMAX2;

            r = put(r, i, 16, (mx ? x > y : x < y) ? x : y);
        }
        return r;

    case ASM_OP_MIX1_L: case ASM_OP_MIX1_R:
    case ASM_OP_MIX2_L: case ASM_OP_MIX2_R:
    case ASM_OP_MIX4_L: case ASM_OP_MIX4_R: {
        int w = (op == ASM_OP_MIX1_L || op == ASM_OP_MIX1_R) ? 8 :
                (op == ASM_OP_MIX2_L || op == ASM_OP_MIX2_R) ? 16 : 32;
        bool left = op == ASM_OP_MIX1_L || op == ASM_OP_MIX2_L ||
                    op == ASM_OP_MIX4_L;

        for (int i = 0; i < 64 / w; i += 2) {
            r = put(r, i, w, elem(b, i + left, w));
            r = put(r, i + 1, w, elem(a, i + left, w));
        }
        return r;
    }
    case ASM_OP_UNPACK1_H: case ASM_OP_UNPACK1_L:
    case ASM_OP_UNPACK2_H: case ASM_OP_UNPACK2_L:
    case ASM_OP_UNPACK4_H: case ASM_OP_UNPACK4_L: {
        int w = (op == ASM_OP_UNPACK1_H || op == ASM_OP_UNPACK1_L) ? 8 :
                (op == ASM_OP_UNPACK2_H || op == ASM_OP_UNPACK2_L) ? 16 : 32;
        bool high = op == ASM_OP_UNPACK1_H || op == ASM_OP_UNPACK2_H ||
                    op == ASM_OP_UNPACK4_H;
        int n = 64 / w, base = high ? n / 2 : 0;

        for (int i = 0; i < n / 2; i++) {
            r = put(r, 2 * i, w, elem(b, base + i, w));
            r = put(r, 2 * i + 1, w, elem(a, base + i, w));
        }
        return r;
    }
    case ASM_OP_PACK2_SSS: case ASM_OP_PACK2_USS:
        for (int i = 0; i < 8; i++) {
            int64_t v = selem(i < 4 ? a : b, i & 3, 16);

            v = op == ASM_OP_PACK2_SSS ? sat(v, -128, 127) : sat(v, 0, 255);
            r = put(r, i, 8, v);
        }
        return r;
    case ASM_OP_PACK4_SSS:
        for (int i = 0; i < 4; i++) {
            int64_t v = selem(i < 2 ? a : b, i & 1, 32);

            r = put(r, i, 16, sat(v, -32768, 32767));
        }
        return r;

    case ASM_OP_MUX1:
        return mux1(a, imm);
    case ASM_OP_MUX2:
        for (int i = 0; i < 4; i++) {
            r = put(r, i, 16, elem(a, (imm >> (2 * i)) & 3, 16));
        }
        return r;

    /* shifts: a = value, b = count */
    case ASM_OP_PSHL2: case ASM_OP_PSHL4: {
        int w = op == ASM_OP_PSHL2 ? 16 : 32;

        for (int i = 0; i < 64 / w; i++) {
            r = put(r, i, w, b >= (uint64_t)w ? 0 : elem(a, i, w) << b);
        }
        return r;
    }
    case ASM_OP_PSHR2_: case ASM_OP_PSHR4_: {
        int w = op == ASM_OP_PSHR2_ ? 16 : 32;
        int c = b >= (uint64_t)w ? w - 1 : b;

        for (int i = 0; i < 64 / w; i++) {
            r = put(r, i, w, selem(a, i, w) >> c);
        }
        return r;
    }
    case ASM_OP_PSHR2_U: case ASM_OP_PSHR4_U: {
        int w = op == ASM_OP_PSHR2_U ? 16 : 32;

        for (int i = 0; i < 64 / w; i++) {
            r = put(r, i, w, b >= (uint64_t)w ? 0 : elem(a, i, w) >> b);
        }
        return r;
    }
    }
    g_assert_not_reached();
}

/*
 * ld.s / ld.sa: every fault is deferred, the target becomes NaT.
 * The architecture allows an implementation to defer any speculative load.
 */
void HELPER(ld_spec)(CPUIA64State *env, uint32_t r1, uint64_t addr,
                     uint32_t addr_nat, uint32_t size, uint32_t mmu_idx)
{
    uintptr_t ra = GETPC();
    void *host;
    uint64_t v = 0;
    MemOp mop;

    if (addr_nat || (addr & (size - 1)) ||
        (probe_access_flags(env, addr, size, MMU_DATA_LOAD, mmu_idx, true,
                            &host, ra) & TLB_INVALID_MASK)) {
        env->gr[r1] = 0;
        env->nat[r1] = 1;
        return;
    }
    switch (size) {
    case 1: mop = MO_UB; break;
    case 2: mop = MO_LEUW; break;
    case 4: mop = MO_LEUL; break;
    default: mop = MO_LEUQ; break;
    }
    switch (size) {
    case 1: v = cpu_ldb_mmu(env, addr, make_memop_idx(mop, mmu_idx), ra);
        break;
    case 2: v = cpu_ldw_mmu(env, addr, make_memop_idx(mop, mmu_idx), ra);
        break;
    case 4: v = cpu_ldl_mmu(env, addr, make_memop_idx(mop, mmu_idx), ra);
        break;
    default: v = cpu_ldq_mmu(env, addr, make_memop_idx(mop, mmu_idx), ra);
        break;
    }
    env->gr[r1] = v;
    env->nat[r1] = 0;
}

/*
 * cmp8xchg16: compare 8 bytes at addr with ar.ccv; if equal, store r2 and
 * ar.csd as the 16-byte aligned pair (r2 at addr).
 */
uint64_t HELPER(cmp8xchg16)(CPUIA64State *env, uint64_t addr, uint64_t r2,
                            uint32_t mmu_idx)
{
    uintptr_t ra = GETPC();
    MemOpIdx oi = make_memop_idx(MO_LEUQ | MO_ALIGN, mmu_idx);
    uint64_t old = cpu_ldq_mmu(env, addr, oi, ra);

    if (old == env->ar[AR_CCV]) {
        cpu_stq_mmu(env, addr, r2, oi, ra);
        cpu_stq_mmu(env, addr ^ 8, env->ar[AR_CSD], oi, ra);
    }
    return old;
}

/* mov pr = r2, mask: predicates in physical order */
void HELPER(mov_to_pr)(CPUIA64State *env, uint64_t val, uint64_t mask)
{
    for (int i = 1; i < 64; i++) {
        if (mask & (1ull << i)) {
            env->pr[i] = (val >> i) & 1;
        }
    }
}

uint64_t HELPER(mov_from_pr)(CPUIA64State *env)
{
    uint64_t v = 1;

    for (int i = 1; i < 64; i++) {
        v |= (uint64_t)(env->pr[i] & 1) << i;
    }
    return v;
}
