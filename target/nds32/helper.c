/*
 * NDS32 exceptions, system registers and MMU
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "accel/tcg/cpu-loop.h"
#include "cpu.h"
#include "exec/helper-proto.h"
#include "exec/log.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "hw/core/cpu.h"
#include "qemu/main-loop.h"

/* Exceptions */

void nds32_raise(CPUNDS32State *env, int vec, uint32_t itype,
                 uint32_t eva, bool eva_valid, uintptr_t retaddr)
{
    CPUState *cs = env_cpu(env);

    env->pending_itype = itype;
    env->pending_eva = eva;
    env->pending_eva_valid = eva_valid;
    cs->exception_index = EXCP_NDS32(vec);
    cpu_loop_exit_restore(cs, retaddr);
}

/* The PC of the faulting instruction is stored before the call. */
void helper_raise(CPUNDS32State *env, uint32_t vec, uint32_t itype,
                  uint32_t eva)
{
    CPUState *cs = env_cpu(env);

    env->pending_itype = itype;
    env->pending_eva = eva;
    env->pending_eva_valid = false;
    cs->exception_index = EXCP_NDS32(vec);
    cpu_loop_exit(cs);
}

void nds32_cpu_do_interrupt(CPUState *cs)
{
    CPUNDS32State *env = cpu_env(cs);
    uint32_t psw = env->sr[SR_PSW];
    unsigned intl = FIELD_EX32(psw, PSW, INTL);
    unsigned vec = cs->exception_index;
    uint32_t itype;
    unsigned esz;

    if (cs->exception_index == EXCP_IRQ) {
        uint32_t pend = env->irq_lines & env->sr[SR_INT_MASK2];

        vec = VEC_INTERRUPT + ctz32(pend);
        itype = 0;
    } else {
        itype = env->pending_itype;
    }
    itype = FIELD_DP32(itype, ITYPE, VECTOR, vec);

    qemu_log_mask(CPU_LOG_INT, "nds32: vector %u itype %08x pc %08x "
                  "eva %08x intl %u\n", vec, itype, env->pc,
                  env->pending_eva, intl);

    if (intl >= 1) {
        env->sr[SR_P_IPSW] = env->sr[SR_IPSW];
        env->sr[SR_P_IPC] = env->sr[SR_IPC];
        env->sr[SR_P_ITYPE] = env->sr[SR_ITYPE];
        env->sr[SR_P_EVA] = env->sr[SR_EVA];
        env->sr[SR_P_P0] = env->gpr[R_P0];
        env->sr[SR_P_P1] = env->gpr[R_P1];
    }
    env->sr[SR_IPSW] = psw;
    env->sr[SR_IPC] = env->pc;
    env->sr[SR_ITYPE] = itype;
    if (cs->exception_index != EXCP_IRQ && env->pending_eva_valid) {
        env->sr[SR_EVA] = env->pending_eva;
    }

    if (FIELD_EX32(psw, PSW, POM) == 0 &&
        (env->sr[SR_MISC_CTL] & MISC_CTL_SP_SHADOW_EN)) {
        env->sr[SR_SP_USR] = env->gpr[R_SP];
        env->gpr[R_SP] = env->sr[SR_SP_PRIV];
    }

    psw = FIELD_DP32(psw, PSW, GIE, 0);
    psw = FIELD_DP32(psw, PSW, INTL, MIN(intl + 1, 3));
    psw = FIELD_DP32(psw, PSW, POM, 1);
    psw = FIELD_DP32(psw, PSW, IT, 0);
    psw = FIELD_DP32(psw, PSW, DT, 0);
    env->sr[SR_PSW] = psw;

    env->lock_addr = -1;

    esz = 4 << (2 * ((env->sr[SR_IVB] >> 14) & 3));
    env->pc = (env->sr[SR_IVB] & 0xffff0000) + vec * esz;
    cs->exception_index = -1;
}

void helper_iret(CPUNDS32State *env)
{
    uint32_t psw = env->sr[SR_PSW];
    uint32_t npsw = env->sr[SR_IPSW];

    env->pc = env->sr[SR_IPC];
    if (FIELD_EX32(psw, PSW, INTL) >= 2) {
        env->sr[SR_IPSW] = env->sr[SR_P_IPSW];
        env->sr[SR_IPC] = env->sr[SR_P_IPC];
        env->sr[SR_ITYPE] = env->sr[SR_P_ITYPE];
        env->sr[SR_EVA] = env->sr[SR_P_EVA];
        env->gpr[R_P0] = env->sr[SR_P_P0];
        env->gpr[R_P1] = env->sr[SR_P_P1];
    }
    if (FIELD_EX32(psw, PSW, POM) != 0 && FIELD_EX32(npsw, PSW, POM) == 0 &&
        (env->sr[SR_MISC_CTL] & MISC_CTL_SP_SHADOW_EN)) {
        env->sr[SR_SP_PRIV] = env->gpr[R_SP];
        env->gpr[R_SP] = env->sr[SR_SP_USR];
    }
    env->sr[SR_PSW] = npsw;
    env->lock_addr = -1;
    nds32_update_irq(env);
}

void nds32_update_irq(CPUNDS32State *env)
{
    CPUState *cs = env_cpu(env);

    BQL_LOCK_GUARD();
    if (env->irq_lines & env->sr[SR_INT_MASK2]) {
        cpu_interrupt(cs, CPU_INTERRUPT_HARD);
    } else {
        cpu_reset_interrupt(cs, CPU_INTERRUPT_HARD);
    }
}

void helper_standby(CPUNDS32State *env)
{
    CPUState *cs = env_cpu(env);

    cs->halted = 1;
    cs->exception_index = EXCP_HLT;
    cpu_loop_exit(cs);
}

/* Division: the result is quotient | remainder << 32. */

static void div_by_zero(CPUNDS32State *env, uintptr_t ra)
{
    if (env->sr[SR_INT_MASK] & INT_MASK_IDIVZE) {
        nds32_raise(env, VEC_GENERAL, ETYPE_ARITH, 0, false, ra);
    }
}

uint64_t helper_divsr(CPUNDS32State *env, int32_t a, int32_t b)
{
    if (b == 0) {
        div_by_zero(env, GETPC());
        return 0;
    }
    if (a == INT32_MIN && b == -1) {
        return (uint32_t)INT32_MIN;
    }
    return (uint32_t)(a / b) | ((uint64_t)(uint32_t)(a % b) << 32);
}

uint64_t helper_divr(CPUNDS32State *env, uint32_t a, uint32_t b)
{
    if (b == 0) {
        div_by_zero(env, GETPC());
        return 0;
    }
    return (a / b) | ((uint64_t)(a % b) << 32);
}

/* System registers */

uint32_t helper_mfsr(CPUNDS32State *env, uint32_t idx)
{
    switch (idx) {
    case SR_INT_PEND2:
        return env->irq_lines;
    default:
        return env->sr[idx & (NUM_SR - 1)];
    }
}

void helper_mtsr(CPUNDS32State *env, uint32_t idx, uint32_t val)
{
    CPUState *cs = env_cpu(env);

    idx &= NUM_SR - 1;
    switch (idx) {
    case SR_CPU_VER:
    case SR_CORE_ID:
    case SR_ICM_CFG:
    case SR_DCM_CFG:
    case SR_MMU_CFG:
    case SR_MSC_CFG:
    case SR_MSC_CFG2:
    case SR_FUCOP_EXIST:
    case SR_INT_PEND2:
        return;
    case SR_IVB:
        env->sr[idx] = (val & 0xffffe000) | (env->sr[idx] & 0x1ffe);
        return;
    case SR_PSW:
        env->sr[idx] = val;
        nds32_update_irq(env);
        return;
    case SR_INT_MASK2:
        env->sr[idx] = val;
        nds32_update_irq(env);
        return;
    case SR_TLB_MISC:
        if (FIELD_EX32(val ^ env->sr[idx], TLB_MISC, CID)) {
            tlb_flush(cs);
        }
        env->sr[idx] = val;
        return;
    case SR_L1_PPTB:
    case SR_MMU_CTL:
        if (val != env->sr[idx]) {
            tlb_flush(cs);
        }
        env->sr[idx] = val;
        return;
    default:
        env->sr[idx] = val;
        return;
    }
}

/* TLB */

static inline unsigned tlb_set_of(uint32_t vpn)
{
    return vpn & (TLB_SETS - 1);
}

static inline uint16_t cur_cid(CPUNDS32State *env)
{
    return FIELD_EX32(env->sr[SR_TLB_MISC], TLB_MISC, CID);
}

static NDS32TLBEntry *tlb_lookup(CPUNDS32State *env, uint32_t vpn)
{
    NDS32TLBEntry *set = env->tlb[tlb_set_of(vpn)];
    uint16_t cid = cur_cid(env);

    for (int w = 0; w < TLB_WAYS; w++) {
        NDS32TLBEntry *e = &set[w];
        if (e->valid && e->vpn == vpn &&
            ((e->data & PTE_G) || e->cid == cid)) {
            return e;
        }
    }
    return NULL;
}

static void tlb_write(CPUNDS32State *env, NDS32TLBEntry *e, uint32_t data,
                      bool lock)
{
    CPUState *cs = env_cpu(env);

    if (e->valid) {
        tlb_flush_page(cs, e->vpn << TARGET_PAGE_BITS);
    }
    e->vpn = env->sr[SR_TLB_VPN] >> TARGET_PAGE_BITS;
    e->data = data;
    e->cid = cur_cid(env);
    e->valid = true;
    e->locked = lock;
    tlb_flush_page(cs, e->vpn << TARGET_PAGE_BITS);
}

static void tlb_random_write(CPUNDS32State *env, uint32_t data, bool lock)
{
    uint32_t vpn = env->sr[SR_TLB_VPN] >> TARGET_PAGE_BITS;
    NDS32TLBEntry *set = env->tlb[tlb_set_of(vpn)];
    NDS32TLBEntry *e = tlb_lookup(env, vpn);

    if (!e) {
        for (int w = 0; w < TLB_WAYS; w++) {
            if (!set[w].valid) {
                e = &set[w];
                break;
            }
        }
    }
    if (!e) {
        for (int i = 0; i < TLB_WAYS; i++) {
            NDS32TLBEntry *c = &set[(env->tlb_rr + i) % TLB_WAYS];
            if (!c->locked) {
                e = c;
                break;
            }
        }
        env->tlb_rr++;
    }
    if (!e) {
        /* All ways locked: nothing to replace. */
        return;
    }
    tlb_write(env, e, data, lock);
}

void nds32_tlb_flush_all(CPUNDS32State *env)
{
    for (int s = 0; s < TLB_SETS; s++) {
        for (int w = 0; w < TLB_WAYS; w++) {
            if (!env->tlb[s][w].locked) {
                env->tlb[s][w].valid = false;
            }
        }
    }
    tlb_flush(env_cpu(env));
}

void helper_tlbop(CPUNDS32State *env, uint32_t ra, uint32_t op)
{
    CPUState *cs = env_cpu(env);
    uint32_t vpn = ra >> TARGET_PAGE_BITS;
    NDS32TLBEntry *e;

    switch (op) {
    case 0: /* TargetRead: Ra = way << 9 | set */
        e = &env->tlb[ra & (TLB_SETS - 1)][(ra >> 9) & (TLB_WAYS - 1)];
        env->sr[SR_TLB_VPN] = e->vpn << TARGET_PAGE_BITS;
        env->sr[SR_TLB_DATA] = e->valid ? e->data : 0;
        env->sr[SR_TLB_MISC] = FIELD_DP32(env->sr[SR_TLB_MISC], TLB_MISC,
                                          CID, e->cid);
        break;
    case 1: /* TargetWrite */
        e = &env->tlb[ra & (TLB_SETS - 1)][(ra >> 9) & (TLB_WAYS - 1)];
        tlb_write(env, e, env->sr[SR_TLB_DATA], false);
        break;
    case 2: /* RWrite: Ra holds the PTE, TLB_VPN the address */
        tlb_random_write(env, ra, false);
        break;
    case 3: /* RWriteLock */
        tlb_random_write(env, ra, true);
        break;
    case 4: /* Unlock */
        for (int w = 0; w < TLB_WAYS; w++) {
            e = &env->tlb[tlb_set_of(vpn)][w];
            if (e->valid && e->vpn == vpn) {
                e->locked = false;
            }
        }
        break;
    case 6: /* Invalidate */
        for (int w = 0; w < TLB_WAYS; w++) {
            e = &env->tlb[tlb_set_of(vpn)][w];
            if (e->valid && e->vpn == vpn) {
                e->valid = false;
                e->locked = false;
            }
        }
        tlb_flush_page(cs, vpn << TARGET_PAGE_BITS);
        break;
    case 7: /* FlushAll */
        nds32_tlb_flush_all(env);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "nds32: tlbop %u\n", op);
        break;
    }
}

uint32_t helper_tlbop_probe(CPUNDS32State *env, uint32_t ra)
{
    uint32_t vpn = ra >> TARGET_PAGE_BITS;
    NDS32TLBEntry *set = env->tlb[tlb_set_of(vpn)];
    NDS32TLBEntry *e = tlb_lookup(env, vpn);

    if (!e) {
        return 0;
    }
    return (1u << 31) | ((e - set) << 9) | tlb_set_of(vpn);
}

/* Address translation */

static hwaddr ntm_map(CPUNDS32State *env, uint32_t va)
{
    unsigned part = va >> 30;
    unsigned to = (env->sr[SR_MMU_CTL] >> (11 + 2 * part)) & 3;

    return ((hwaddr)to << 30) | (va & 0x3fffffff);
}

enum {
    XL_OK,
    XL_NONLEAF_NP,      /* PTE not present, ETYPE 0 */
    XL_LEAF_NP,         /* PTE not present, ETYPE 1 */
};

static int get_pte(CPUNDS32State *env, uint32_t va, uint32_t *pte)
{
    CPUState *cs = env_cpu(env);
    NDS32TLBEntry *e = tlb_lookup(env, va >> TARGET_PAGE_BITS);
    uint32_t l1, l1a, l2a;

    if (e) {
        *pte = e->data;
        return (e->data & PTE_V) ? XL_OK : XL_LEAF_NP;
    }

    l1a = (env->sr[SR_L1_PPTB] & 0xfffff000) | ((va >> 22) << 2);
    l1 = address_space_ldl_le(cs->as, l1a, MEMTXATTRS_UNSPECIFIED, NULL);
    if (l1 & 1) {
        return XL_NONLEAF_NP;
    }
    l2a = (l1 & 0xfffff000) | (((va >> 12) & 0x3ff) << 2);
    *pte = address_space_ldl_le(cs->as, l2a, MEMTXATTRS_UNSPECIFIED, NULL);
    return (*pte & PTE_V) ? XL_OK : XL_LEAF_NP;
}

static int pte_prot(uint32_t pte, bool user)
{
    unsigned m = (pte & PTE_M_MASK) >> PTE_M_SHIFT;
    bool r, w, x;

    if (user) {
        r = m >= 1 && m <= 3;
        w = m == 3;
        x = (pte & PTE_X) && m <= 3;
    } else {
        r = m != 4 && m != 6;
        w = m == 0 || m == 2 || m == 3 || m == 7;
        x = pte & PTE_X;
    }
    return (r ? PAGE_READ : 0) | (w ? PAGE_WRITE : 0) | (x ? PAGE_EXEC : 0);
}

hwaddr nds32_cpu_get_phys_page_debug(CPUState *cs, vaddr addr)
{
    CPUNDS32State *env = cpu_env(cs);
    uint32_t pte;

    if (!FIELD_EX32(env->sr[SR_PSW], PSW, DT)) {
        return ntm_map(env, addr);
    }
    if (get_pte(env, addr, &pte) != XL_OK) {
        return -1;
    }
    return (pte & TARGET_PAGE_MASK) | (addr & ~TARGET_PAGE_MASK);
}

bool nds32_cpu_tlb_fill(CPUState *cs, vaddr address, int size,
                        MMUAccessType access_type, int mmu_idx,
                        bool probe, uintptr_t retaddr)
{
    CPUNDS32State *env = cpu_env(cs);
    uint32_t va = address;
    uint32_t itype = access_type == MMU_INST_FETCH ? R_ITYPE_INST_MASK : 0;
    uint32_t pte;
    int prot, r;

    if (!(mmu_idx & MMU_IDX_XLATE)) {
        tlb_set_page(cs, va & TARGET_PAGE_MASK,
                     ntm_map(env, va) & TARGET_PAGE_MASK,
                     PAGE_RWX, mmu_idx, TARGET_PAGE_SIZE);
        return true;
    }

    r = get_pte(env, va, &pte);
    if (r != XL_OK) {
        if (probe) {
            return false;
        }
        nds32_raise(env, VEC_PTE_NOT_PRESENT,
                    itype | (r == XL_LEAF_NP ? 1 : 0), va, true, retaddr);
    }

    prot = pte_prot(pte, mmu_idx & MMU_IDX_USER);
    switch (access_type) {
    case MMU_DATA_LOAD:
        if (!(prot & PAGE_READ)) {
            itype |= TLBMISC_RD_PROT;
            goto fault;
        }
        break;
    case MMU_DATA_STORE:
        if (!(prot & PAGE_WRITE)) {
            itype |= TLBMISC_WRT_PROT;
            goto fault;
        }
        if (!(pte & PTE_D)) {
            itype |= TLBMISC_PAGE_MODIFY;
            goto fault;
        }
        break;
    case MMU_INST_FETCH:
        if (!(prot & PAGE_EXEC)) {
            itype |= TLBMISC_NOEXEC;
            goto fault;
        }
        break;
    }
    if (!(pte & PTE_D)) {
        prot &= ~PAGE_WRITE;
    }
    tlb_set_page(cs, va & TARGET_PAGE_MASK, pte & TARGET_PAGE_MASK,
                 prot, mmu_idx, TARGET_PAGE_SIZE);
    return true;

fault:
    if (probe) {
        return false;
    }
    nds32_raise(env, VEC_TLB_MISC, itype, va, true, retaddr);
}

void nds32_cpu_do_unaligned_access(CPUState *cs, vaddr addr,
                                   MMUAccessType access_type,
                                   int mmu_idx, uintptr_t retaddr)
{
    CPUNDS32State *env = cpu_env(cs);
    uint32_t itype = ETYPE_ALIGN;

    if (access_type == MMU_INST_FETCH) {
        itype |= R_ITYPE_INST_MASK;
    }
    nds32_raise(env, VEC_GENERAL, itype, addr, true, retaddr);
}
