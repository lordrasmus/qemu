/*
 * MC68030 / MC68851 paged memory management unit
 *
 * The 68020 has no MMU of its own; Linux runs it with a 68851, whose
 * programming model for TC, CRP, SRP and the table walk is the one the
 * 68030 later took over. Both are handled here. The 68030 transparent
 * translation registers are accepted on the 68020 as well.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/cputlb.h"
#include "exec/page-protection.h"
#include "exec/target_page.h"
#include "exec/helper-proto.h"
#include "accel/tcg/cpu-ldst.h"
#include "accel/tcg/cpu-loop.h"
#include "system/memory.h"

#define TC_E            (1u << 31)
#define TC_SRE          (1u << 25)
#define TC_FCL          (1u << 24)

#define TT_E            (1u << 15)
#define TT_RW           (1u << 9)
#define TT_RWM          (1u << 8)

#define DESC_DT         3
#define DESC_WP         (1u << 2)
#define DESC_U          (1u << 3)
#define DESC_M          (1u << 4)
#define DESC_S          (1u << 8)           /* long format only */

#define DT_INVALID      0
#define DT_PAGE         1
#define DT_SHORT        2
#define DT_LONG         3

typedef struct Walk030 {
    hwaddr pa;
    int prot;
    uint16_t mmusr;
    uint32_t desc;              /* address of the last descriptor */
} Walk030;

static bool tt030_match(uint32_t tt, uint32_t addr, int fc, bool write)
{
    uint32_t base = tt >> 24, mask = (tt >> 16) & 0xff;
    int fcbase = (tt >> 4) & 7, fcmask = tt & 7;

    if (!(tt & TT_E)) {
        return false;
    }
    if (((addr >> 24) ^ base) & ~mask & 0xff) {
        return false;
    }
    if ((fc ^ fcbase) & ~fcmask & 7) {
        return false;
    }
    if (!(tt & TT_RWM) && !!(tt & TT_RW) == write) {
        return false;
    }
    return true;
}

static uint32_t ld_desc(CPUM68KState *env, hwaddr pa, bool *err)
{
    MemTxResult res;
    uint32_t val = address_space_ldl_be(env_cpu(env)->as, pa,
                                        MEMTXATTRS_UNSPECIFIED, &res);

    *err |= res != MEMTX_OK;
    return val;
}

static void st_desc(CPUM68KState *env, hwaddr pa, uint32_t val)
{
    address_space_stl_be(env_cpu(env)->as, pa, val,
                         MEMTXATTRS_UNSPECIFIED, NULL);
}

/*
 * Walk the translation tables for @addr. @update sets the U and M bits as
 * the hardware does on a real access; PTEST and debug lookups leave the
 * tables alone. @maxlevel limits the walk like PTEST's level field.
 * Returns false if the access is not allowed; mmusr says why.
 */
static bool walk030(CPUM68KState *env, uint32_t addr, int fc, bool write,
                    bool update, int maxlevel, Walk030 *w)
{
    uint32_t tc = env->mmu.tc030;
    int is = (tc >> 16) & 15;
    int width[5], nlevels = 0, consumed = is, levels = 0;
    uint64_t root = (tc & TC_SRE) && (fc & 4) ? env->mmu.srp030
                                               : env->mmu.crp030;
    uint32_t hi = root >> 32;
    int dt = hi & DESC_DT;
    uint32_t table = (uint32_t)root & ~0xfu;
    bool has_limit = true, lower = hi & (1u << 31);
    uint32_t limit = (hi >> 16) & 0x7fff;
    bool wp = false, s = false, err = false, user = !(fc & 4);
    uint32_t pdesc = DT_PAGE, pdesc_addr = 0, page = table & ~0xffu;
    bool long_page = false;

    memset(w, 0, sizeof(*w));

    if (tc & TC_FCL) {
        width[nlevels++] = -3;              /* function code lookup */
    }
    for (int i = 0; i < 4; i++) {
        int ti = (tc >> (12 - 4 * i)) & 15;

        if (!ti) {
            break;
        }
        width[nlevels++] = ti;
    }

    for (int i = 0; i < nlevels && dt != DT_PAGE; i++) {
        uint32_t idx, d0, d1 = 0, daddr;
        int size;

        if (dt == DT_INVALID) {
            w->mmusr |= M68K_MMU_I_030;
            goto out;
        }
        if (levels >= maxlevel) {
            goto out;
        }
        if (width[i] < 0) {
            idx = fc;
        } else {
            idx = (addr << consumed) >> (32 - width[i]);
            consumed += width[i];
        }
        if (has_limit && (lower ? idx < limit : idx > limit)) {
            w->mmusr |= M68K_MMU_L_030 | M68K_MMU_I_030;
            goto out;
        }
        size = dt == DT_SHORT ? 4 : 8;
        daddr = table + idx * size;
        d0 = ld_desc(env, daddr, &err);
        if (size == 8) {
            d1 = ld_desc(env, daddr + 4, &err);
        }
        levels++;
        if (err) {
            w->mmusr |= M68K_MMU_B_030 | M68K_MMU_I_030;
            goto out;
        }
        w->desc = daddr;
        dt = d0 & DESC_DT;
        if (dt == DT_INVALID) {
            w->mmusr |= M68K_MMU_I_030;
            goto out;
        }
        wp |= d0 & DESC_WP;
        if (size == 8) {
            s |= d0 & DESC_S;
            lower = d0 & (1u << 31);
            limit = (d0 >> 16) & 0x7fff;
            has_limit = true;
        } else {
            has_limit = false;
        }
        if (dt == DT_PAGE) {
            pdesc = d0;
            pdesc_addr = daddr;
            long_page = size == 8;
            page = size == 8 ? d1 & ~0xffu : d0 & ~0xffu;
        } else {
            if (update && !(d0 & DESC_U)) {
                st_desc(env, daddr, d0 | DESC_U);
            }
            table = (size == 8 ? d1 : d0) & ~0xfu;
        }
    }

    if (dt == DT_INVALID) {
        w->mmusr |= M68K_MMU_I_030;
        goto out;
    }
    if (dt != DT_PAGE) {
        /* table descriptor after the last level: indirect page descriptor */
        if (levels >= maxlevel) {
            goto out;
        }
        pdesc_addr = table;
        long_page = dt == DT_LONG;
        pdesc = ld_desc(env, table, &err);
        page = long_page ? ld_desc(env, table + 4, &err) : pdesc;
        page &= ~0xffu;
        levels++;
        if (err) {
            w->mmusr |= M68K_MMU_B_030 | M68K_MMU_I_030;
            goto out;
        }
        w->desc = table;
        if ((pdesc & DESC_DT) != DT_PAGE) {
            w->mmusr |= M68K_MMU_I_030;
            goto out;
        }
    }

    wp |= pdesc & DESC_WP;
    if (long_page) {
        s |= pdesc & DESC_S;
    }
    if (wp) {
        w->mmusr |= M68K_MMU_W_030;
    }
    if (s && user) {
        w->mmusr |= M68K_MMU_S_030;
        goto out;
    }
    if (write && wp) {
        goto out;
    }
    if (update && pdesc_addr) {
        uint32_t nd = pdesc | DESC_U | (write ? DESC_M : 0);

        if (nd != pdesc) {
            st_desc(env, pdesc_addr, nd);
            pdesc = nd;
        }
    }
    if (pdesc & DESC_M) {
        w->mmusr |= M68K_MMU_M_030;
    }

    w->pa = page + (addr & (0xffffffffu >> consumed));
    w->prot = PAGE_READ | PAGE_EXEC;
    /* keep a clean page read-only so the first write sets M */
    if (!wp && (pdesc & DESC_M)) {
        w->prot |= PAGE_WRITE;
    }
    w->mmusr |= levels;
    return true;

out:
    w->mmusr |= levels;
    return false;
}

static int fc030(int mmu_idx, MMUAccessType access)
{
    return (mmu_idx == MMU_USER_IDX ? 0 : 4) |
           (access == MMU_INST_FETCH ? 2 : 1);
}

static bool tt030(CPUM68KState *env, uint32_t addr, int fc, bool write)
{
    return tt030_match(env->mmu.tt030[0], addr, fc, write) ||
           tt030_match(env->mmu.tt030[1], addr, fc, write);
}

bool m68k_030_tlb_fill(CPUState *cs, vaddr address, int size,
                       MMUAccessType access, int mmu_idx,
                       bool probe, uintptr_t retaddr)
{
    CPUM68KState *env = cpu_env(cs);
    bool write = access == MMU_DATA_STORE;
    int fc = fc030(mmu_idx, access);
    Walk030 w;

    if (!(env->mmu.tc030 & TC_E) || tt030(env, address, fc, write)) {
        tlb_set_page(cs, address & TARGET_PAGE_MASK,
                     address & TARGET_PAGE_MASK,
                     PAGE_READ | PAGE_WRITE | PAGE_EXEC,
                     mmu_idx, TARGET_PAGE_SIZE);
        return true;
    }
    if (walk030(env, address, fc, write, !probe, 7, &w)) {
        tlb_set_page(cs, address & TARGET_PAGE_MASK, w.pa & TARGET_PAGE_MASK,
                     w.prot, mmu_idx, TARGET_PAGE_SIZE);
        return true;
    }
    if (probe) {
        return false;
    }

    /* bus error, stacked as a short bus cycle fault frame (format $A) */
    if (access == MMU_INST_FETCH) {
        env->mmu.ssw = M68K_SSW_FB_030 | fc;
        env->mmu.insn_fault030 = true;
    } else {
        env->mmu.ssw = M68K_SSW_DF_030 | fc;
        if (!write) {
            env->mmu.ssw |= M68K_SSW_RW_030;
        }
        switch (size) {
        case 1:
            env->mmu.ssw |= 1 << 4;
            break;
        case 2:
            env->mmu.ssw |= 2 << 4;
            break;
        }
        env->mmu.insn_fault030 = false;
    }
    env->mmu.ar = address;
    cs->exception_index = EXCP_ACCESS;
    cpu_loop_exit_restore(cs, retaddr);
}

hwaddr m68k_030_get_phys_addr_debug(CPUM68KState *env, vaddr addr)
{
    int fc = (env->sr & SR_S) ? 5 : 1;
    Walk030 w;

    if (!(env->mmu.tc030 & TC_E) || tt030(env, addr, fc, false)) {
        return addr;
    }
    if (!walk030(env, addr, fc, false, false, 7, &w)) {
        return -1;
    }
    return w.pa;
}

/* The function code field of PTEST, PLOAD and PFLUSH */
static int ea_fc(CPUM68KState *env, uint32_t ext)
{
    int f = ext & 0x1f;

    if (f & 0x10) {
        return f & 7;
    }
    if (f & 0x08) {
        return env->dregs[f & 7] & 7;
    }
    return f ? env->dfc & 7 : env->sfc & 7;
}

/*
 * PMOVE between memory at @addr and an MMU register. Returns true if the
 * ATC has to be flushed.
 */
void HELPER(pmove030)(CPUM68KState *env, uint32_t addr, uint32_t ext)
{
    CPUState *cs = env_cpu(env);
    bool to_mem = ext & 0x200;
    bool flush = !(ext & 0x100);
    int preg = (ext >> 10) & 7;
    int idx = cpu_mmu_index(cs, false);
    uintptr_t ra = GETPC();
    uint64_t *rp = NULL;
    uint32_t *r32 = NULL;

    switch (ext >> 13) {
    case 0:                             /* TT0, TT1 */
        if (preg != 2 && preg != 3) {
            return;
        }
        r32 = &env->mmu.tt030[preg - 2];
        break;
    case 2:                             /* TC, SRP, CRP */
        if (preg == 0) {
            r32 = &env->mmu.tc030;
        } else if (preg == 2) {
            rp = &env->mmu.srp030;
        } else if (preg == 3) {
            rp = &env->mmu.crp030;
        } else {
            return;
        }
        break;
    case 3:                             /* MMUSR / PSR */
        if (to_mem) {
            cpu_stw_be_mmuidx_ra(env, addr, env->mmu.mmusr030, idx, ra);
        } else {
            env->mmu.mmusr030 = cpu_lduw_be_mmuidx_ra(env, addr, idx, ra);
        }
        return;
    default:
        return;
    }

    if (r32) {
        if (to_mem) {
            cpu_stl_be_mmuidx_ra(env, addr, *r32, idx, ra);
            return;
        }
        *r32 = cpu_ldl_be_mmuidx_ra(env, addr, idx, ra);
    } else {
        if (to_mem) {
            cpu_stl_be_mmuidx_ra(env, addr, *rp >> 32, idx, ra);
            cpu_stl_be_mmuidx_ra(env, addr + 4, *rp, idx, ra);
            return;
        }
        *rp = (uint64_t)cpu_ldl_be_mmuidx_ra(env, addr, idx, ra) << 32 |
              cpu_ldl_be_mmuidx_ra(env, addr + 4, idx, ra);
    }
    if (flush) {
        tlb_flush(cs);
    }
}

uint32_t HELPER(ptest030)(CPUM68KState *env, uint32_t addr, uint32_t ext)
{
    int level = (ext >> 10) & 7;
    bool write = !(ext & 0x200);
    int fc = ea_fc(env, ext);
    Walk030 w;

    if (tt030(env, addr, fc, write)) {
        env->mmu.mmusr030 = M68K_MMU_T_030;
        return 0;
    }
    if (level == 0 || !(env->mmu.tc030 & TC_E)) {
        /* ATC lookup only; QEMU keeps no ATC of its own */
        env->mmu.mmusr030 = 0;
        return 0;
    }
    walk030(env, addr, fc, write, false, level, &w);
    env->mmu.mmusr030 = w.mmusr;
    return w.desc;
}

void HELPER(pflush030)(CPUM68KState *env)
{
    tlb_flush(env_cpu(env));
}
