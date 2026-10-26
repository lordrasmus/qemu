/*
 * IA-64 system emulation: interruptions, local SAPIC, interval timer,
 * address translation, register stack engine and the system instructions
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "accel/tcg/cpu-loop.h"
#include "exec/tlb-flags.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h"
#include "cpu.h"
#include "exec/helper-proto.h"
#include "exec/cputlb.h"
#include "exec/page-protection.h"
#include "accel/tcg/cpu-ldst.h"
#include "accel/tcg/probe.h"

/*
 * Interruptions
 */

G_NORETURN void ia64_raise_fault(CPUIA64State *env, int vec, uint64_t isr,
                                 uint32_t flags, uint64_t ifa, uintptr_t ra)
{
    CPUState *cs = env_cpu(env);

    if (ra) {
        cpu_restore_state(cs, ra);
    }
    env->pend_isr = isr;
    env->pend_flags = flags;
    env->pend_ifa = ifa;
    cs->exception_index = EXCP_IA64(vec);
    cpu_loop_exit(cs);
}

G_NORETURN void ia64_raise(CPUIA64State *env, int vec, uint64_t isr,
                           uintptr_t ra)
{
    ia64_raise_fault(env, vec, isr, 0, 0, ra);
}

void HELPER(fault)(CPUIA64State *env, uint32_t vec, uint64_t isr)
{
    ia64_raise(env, vec, isr, 0);
}

/* Immediates of break that call into the board firmware. */
#define FW_BREAK_FIRST  0x1c0000
#define FW_BREAK_LAST   0x1c00ff

void HELPER(break)(CPUIA64State *env, uint32_t imm)
{
    if (imm >= FW_BREAK_FIRST && imm <= FW_BREAK_LAST &&
        env->firmware_call && ia64_cpl(env) == 0) {
        /* continue behind the break unless the firmware branches */
        if (env->ri < 2) {
            env->ri++;
        } else {
            env->ri = 0;
            env->ip += 16;
        }
        env->firmware_call(env, imm - FW_BREAK_FIRST);
        return;
    }
    env->pend_iim = imm;
    ia64_raise_fault(env, IVT_BREAK, 0, PEND_IIM, 0, 0);
}

void ia64_switch_bank(CPUIA64State *env, bool bn)
{
    if (!!(env->psr & PSR_BN) == bn) {
        return;
    }
    for (int i = 0; i < 16; i++) {
        uint64_t t = env->gr[16 + i];
        uint32_t n = env->nat[16 + i];

        env->gr[16 + i] = env->bank[i];
        env->nat[16 + i] = env->bank_nat[i];
        env->bank[i] = t;
        env->bank_nat[i] = n;
    }
    env->psr = bn ? env->psr | PSR_BN : env->psr & ~PSR_BN;
}

/* All psr writes go through here: the bank and the TB key follow. */
void ia64_set_psr(CPUIA64State *env, uint64_t psr)
{
    psr &= PSR_DEFINED_MASK & ~PSR_RI;
    ia64_switch_bank(env, psr & PSR_BN);
    env->psr = psr;
    ia64_update_irq(env);
}

void ia64_cpu_do_interrupt(CPUState *cs)
{
    CPUIA64State *env = cpu_env(cs);
    int vec = cs->exception_index;
    uint64_t psr = env->psr;
    uint64_t isr = env->pend_isr;
    uint64_t newpsr;
    bool ic = psr & PSR_IC;

    if (vec < 0 || vec >= 0x8000) {
        return;
    }
    if (qemu_loglevel_mask(CPU_LOG_INT)) {
        qemu_log("ia64: interruption %04x ip=%016" PRIx64 " ri=%d isr=%016"
                 PRIx64 " ifa=%016" PRIx64 " psr=%016" PRIx64 "\n",
                 vec, env->ip, env->ri, isr, env->pend_ifa, psr);
    }

    /* A data TLB miss with collection off is a nested TLB fault. */
    if (!ic && (vec == IVT_DTLB || vec == IVT_ALT_DTLB)) {
        vec = IVT_NESTED_DTLB;
    } else {
        if (vec != IVT_EXTINT) {
            isr = deposit64(isr, ISR_EI_SHIFT, 2,
                            (env->pend_flags & PEND_TRAP) ? 0 : env->ri);
        }
        if (!ic) {
            isr |= ISR_NI;
        }
        env->cr[CR_ISR] = isr;
        if (ic) {
            env->cr[CR_IPSR] = psr | ((uint64_t)env->ri << PSR_RI_SHIFT);
            env->cr[CR_IIP] = env->ip;
            env->cr[CR_IIPA] = env->ip;
            env->cr[CR_IFS] &= ~IFS_V;
            if (env->pend_flags & PEND_IFA) {
                env->cr[CR_IFA] = env->pend_ifa;
            }
            if (env->pend_flags & PEND_ITIR) {
                env->cr[CR_ITIR] = env->pend_itir;
            }
            if (env->pend_flags & PEND_IHA) {
                env->cr[CR_IHA] = env->pend_iha;
            }
            if (env->pend_flags & PEND_IIM) {
                env->cr[CR_IIM] = env->pend_iim;
            }
        }
    }

    newpsr = psr & (PSR_UP | PSR_MFL | PSR_MFH | PSR_PK | PSR_DT | PSR_RT |
                    PSR_MC | PSR_IT);
    if (FIELD_EX64(env->cr[CR_DCR], DCR, BE)) {
        newpsr |= PSR_BE;
    }
    if (FIELD_EX64(env->cr[CR_DCR], DCR, PP)) {
        newpsr |= PSR_PP;
    }
    ia64_switch_bank(env, false);
    env->psr = newpsr;
    env->ip = env->cr[CR_IVA] + vec;
    env->ri = 0;
    env->pend_flags = 0;
    cs->exception_index = -1;
}

/*
 * Local SAPIC
 */

static int lsapic_highest(const uint64_t v[4])
{
    for (int i = 3; i >= 0; i--) {
        if (v[i]) {
            return i * 64 + 63 - clz64(v[i]);
        }
    }
    return -1;
}

/* Highest pending vector that the TPR and the in-service vectors allow. */
static int lsapic_pending(CPUIA64State *env)
{
    int v = lsapic_highest(env->irr);
    int s = lsapic_highest(env->insvc);
    uint64_t tpr = env->cr[CR_TPR];

    if (v < 16) {
        return -1;
    }
    if (tpr & (1 << 16)) {
        return -1;
    }
    if ((v >> 4) <= ((tpr >> 4) & 0xf)) {
        return -1;
    }
    if (s >= 0 && (v >> 4) <= (s >> 4)) {
        return -1;
    }
    return v;
}

void ia64_update_irq(CPUIA64State *env)
{
    CPUState *cs = env_cpu(env);

    BQL_LOCK_GUARD();

    if (lsapic_pending(env) >= 0) {
        cpu_interrupt(cs, CPU_INTERRUPT_HARD);
    } else {
        cpu_reset_interrupt(cs, CPU_INTERRUPT_HARD);
    }
}

void ia64_set_irq_vector(CPUIA64State *env, int vector, bool level)
{
    if (vector < 0 || vector > 255) {
        return;
    }
    if (level) {
        env->irr[vector >> 6] |= 1ull << (vector & 63);
    } else {
        env->irr[vector >> 6] &= ~(1ull << (vector & 63));
    }
    ia64_update_irq(env);
}

void ia64_cpu_deliver_vector(ArchCPU *cpu, int vector)
{
    BQL_LOCK_GUARD();
    ia64_set_irq_vector(&cpu->env, vector, true);
}

static uint64_t lsapic_read_ivr(CPUIA64State *env)
{
    int v = lsapic_pending(env);

    if (v < 0) {
        return 15;              /* spurious */
    }
    env->irr[v >> 6] &= ~(1ull << (v & 63));
    env->insvc[v >> 6] |= 1ull << (v & 63);
    ia64_update_irq(env);
    return v;
}

static void lsapic_eoi(CPUIA64State *env)
{
    int s = lsapic_highest(env->insvc);

    if (s >= 0) {
        env->insvc[s >> 6] &= ~(1ull << (s & 63));
    }
    ia64_update_irq(env);
}

/*
 * Interval timer
 */

uint64_t ia64_read_itc(CPUIA64State *env)
{
    int64_t ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    return muldiv64(ns, env->itc_freq, NANOSECONDS_PER_SECOND) +
           env->itc_offset;
}

void ia64_write_itc(CPUIA64State *env, uint64_t val)
{
    int64_t ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    env->itc_offset = val - muldiv64(ns, env->itc_freq,
                                     NANOSECONDS_PER_SECOND);
    ia64_rearm_itm(env);
}

void ia64_rearm_itm(CPUIA64State *env)
{
    int64_t delta = env->cr[CR_ITM] - ia64_read_itc(env);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (!env->itm_timer) {
        return;
    }
    if (delta <= 0) {
        /* already passed: the match happens only after a wrap */
        timer_del(env->itm_timer);
        return;
    }
    timer_mod(env->itm_timer,
              now + muldiv64(delta, NANOSECONDS_PER_SECOND, env->itc_freq));
}

/*
 * Address translation
 */

static uint32_t rr_rid(CPUIA64State *env, uint64_t va)
{
    return FIELD_EX64(env->rr[va >> 61], RR, RID);
}

static bool tlb_match(const IA64TLBEntry *e, uint64_t va, uint32_t rid)
{
    return e->valid && e->rid == rid &&
           ((va ^ e->vpn) & ~((1ull << e->ps) - 1)) == 0;
}

static IA64TLBEntry *tlb_lookup(CPUIA64State *env, uint64_t va, bool insn)
{
    IA64TLBEntry *tr = insn ? env->itr : env->dtr;
    IA64TLBEntry *tc = insn ? env->itc : env->dtc;
    uint32_t rid = rr_rid(env, va);

    for (int i = 0; i < IA64_NUM_TR; i++) {
        if (tlb_match(&tr[i], va, rid)) {
            return &tr[i];
        }
    }
    for (int i = 0; i < IA64_NUM_TC; i++) {
        if (tlb_match(&tc[i], va, rid)) {
            return &tc[i];
        }
    }
    return NULL;
}

/* Drop both translation caches; the translation registers stay. */
void ia64_tlb_flush_all(CPUIA64State *env)
{
    for (int i = 0; i < IA64_NUM_TC; i++) {
        env->itc[i].valid = false;
        env->dtc[i].valid = false;
    }
    tlb_flush(env_cpu(env));
}

static void tlb_flush_entry(CPUIA64State *env, const IA64TLBEntry *e)
{
    uint64_t size = 1ull << e->ps;

    if (size > (1ull << 30)) {
        tlb_flush(env_cpu(env));
    } else {
        tlb_flush_range_by_mmuidx(env_cpu(env), e->vpn, size,
                                  (1 << NB_MMU_MODES_USED) - 1, 64);
    }
}

uint64_t ia64_thash(CPUIA64State *env, uint64_t va)
{
    uint64_t pta = env->cr[CR_PTA];
    int ps = FIELD_EX64(env->rr[va >> 61], RR, PS);
    int size = FIELD_EX64(pta, PTA, SIZE);
    uint64_t mask = (size >= 64 ? ~0ull : (1ull << size) - 1);
    uint64_t off, base;

    if (FIELD_EX64(pta, PTA, VF)) {
        /* long format: not implemented, return the tag-less hash */
        off = ((va & ((1ull << 61) - 1)) >> ps) << 5;
    } else {
        off = ((va & ((1ull << 61) - 1)) >> ps) << 3;
    }
    off &= mask;
    base = pta & ~((1ull << 15) - 1) & ((1ull << 61) - 1);
    base &= ~mask;
    return (va & (7ull << 61)) | base | off;
}

uint64_t ia64_ttag(CPUIA64State *env, uint64_t va)
{
    int ps = FIELD_EX64(env->rr[va >> 61], RR, PS);

    return ((va & ((1ull << 61) - 1)) >> ps) ^
           ((uint64_t)rr_rid(env, va) << 39);
}

/* Access rights per Table 4-4; returns PAGE_READ/WRITE/EXEC. */
static int page_rights(int ar, int pl, int cpl)
{
    if (ar == 7) {
        return cpl == 0 ? PAGE_READ | PAGE_EXEC : PAGE_EXEC;
    }
    if (cpl > pl) {
        return 0;
    }
    switch (ar) {
    case 0: return PAGE_READ;
    case 1: return PAGE_READ | PAGE_EXEC;
    case 2: return PAGE_READ | PAGE_WRITE;
    case 3: return PAGE_READ | PAGE_WRITE | PAGE_EXEC;
    case 4:
        return (cpl < pl || cpl == 0) ? PAGE_READ | PAGE_WRITE : PAGE_READ;
    case 5:
        return cpl == 0 ? PAGE_READ | PAGE_WRITE | PAGE_EXEC
                        : PAGE_READ | PAGE_EXEC;
    default:
        return (cpl == pl && pl != 0) ? PAGE_READ | PAGE_WRITE | PAGE_EXEC
                                      : PAGE_READ | PAGE_WRITE;
    }
}

static uint64_t access_isr(MMUAccessType type)
{
    switch (type) {
    case MMU_DATA_STORE: return ISR_W;
    case MMU_INST_FETCH: return ISR_X;
    default: return ISR_R;
    }
}

/*
 * Translate va for an access at privilege pl.  On failure fills the
 * interruption resources and returns the vector, else 0.
 */
static int ia64_translate(CPUIA64State *env, uint64_t va, MMUAccessType type,
                          int pl, bool virt, hwaddr *pa, int *prot,
                          uint64_t *psize, bool for_probe)
{
    bool insn = type == MMU_INST_FETCH;
    IA64TLBEntry *e;
    uint64_t rr = env->rr[va >> 61];
    uint64_t isr = access_isr(type);
    int rights, need;

    if (!virt) {
        *pa = va & ((1ull << (IA64_IMPL_PA_MSB + 1)) - 1);
        *prot = PAGE_READ | PAGE_WRITE | PAGE_EXEC;
        *psize = TARGET_PAGE_SIZE;
        return 0;
    }

    env->pend_isr = isr;
    env->pend_ifa = va;
    env->pend_itir = rr & (R_RR_PS_MASK | R_RR_RID_MASK);
    env->pend_iha = ia64_thash(env, va);
    env->pend_flags = PEND_IFA | PEND_ITIR | PEND_IHA;

    e = tlb_lookup(env, va, insn);
    if (!e) {
        bool vhpt = FIELD_EX64(env->cr[CR_PTA], PTA, VE) &&
                    FIELD_EX64(rr, RR, VE);

        if (insn) {
            return vhpt ? IVT_ITLB : IVT_ALT_ITLB;
        }
        return vhpt ? IVT_DTLB : IVT_ALT_DTLB;
    }
    env->pend_itir = e->itir;
    if (!(e->pte & PTE_P)) {
        return IVT_PAGE_NOT_PRESENT;
    }
    rights = page_rights(FIELD_EX64(e->pte, PTE, AR),
                         FIELD_EX64(e->pte, PTE, PL), pl);
    need = insn ? PAGE_EXEC : (type == MMU_DATA_STORE ? PAGE_WRITE
                                                      : PAGE_READ);
    if (!(rights & need)) {
        return insn ? IVT_IACCESS_RIGHTS : IVT_DACCESS_RIGHTS;
    }
    if (!(e->pte & PTE_A) && !(insn ? (env->psr & PSR_IA)
                                    : (env->psr & PSR_DA))) {
        return insn ? IVT_IACCESS_BIT : IVT_DACCESS_BIT;
    }
    if (type == MMU_DATA_STORE && !(e->pte & PTE_D) &&
        !(env->psr & PSR_DD)) {
        return IVT_DIRTY;
    }
    if (!(e->pte & PTE_D) && !(env->psr & PSR_DD)) {
        rights &= ~PAGE_WRITE;
    }
    *psize = 1ull << e->ps;
    *pa = ((e->pte & (R_PTE_PPN_MASK)) & ~(*psize - 1)) | (va & (*psize - 1));
    *prot = rights;
    env->pend_flags = 0;
    return 0;
}

bool ia64_cpu_tlb_fill(CPUState *cs, vaddr address, int size,
                       MMUAccessType access_type, int mmu_idx,
                       bool probe, uintptr_t retaddr)
{
    CPUIA64State *env = cpu_env(cs);
    hwaddr pa;
    int prot, vec;
    uint64_t psize;

    vec = ia64_translate(env, address, access_type, mmu_idx & 3,
                         mmu_idx & MMU_IDX_VIRT, &pa, &prot, &psize, probe);
    if (vec == 0) {
        if (psize < TARGET_PAGE_SIZE) {
            psize = TARGET_PAGE_SIZE;
        }
        /* the page of the access; psize only marks it as large */
        tlb_set_page(cs, address & TARGET_PAGE_MASK, pa & TARGET_PAGE_MASK,
                     prot, mmu_idx, psize);
        return true;
    }
    if (probe) {
        env->pend_flags = 0;
        return false;
    }
    ia64_raise_fault(env, vec, env->pend_isr, env->pend_flags, address,
                     retaddr);
}

G_NORETURN void ia64_cpu_do_unaligned_access(CPUState *cs, vaddr addr,
                                             MMUAccessType access_type,
                                             int mmu_idx, uintptr_t retaddr)
{
    CPUIA64State *env = cpu_env(cs);

    ia64_raise_fault(env, IVT_UNALIGNED, access_isr(access_type), PEND_IFA,
                     addr, retaddr);
}

hwaddr ia64_cpu_get_phys_page_debug(CPUState *cs, vaddr addr)
{
    CPUIA64State *env = cpu_env(cs);
    hwaddr pa;
    int prot;
    uint64_t psize;
    uint64_t save[6] = { env->pend_isr, env->pend_ifa, env->pend_itir,
                         env->pend_iha, env->pend_flags };

    if (ia64_translate(env, addr, MMU_DATA_LOAD, 0, env->psr & PSR_DT,
                       &pa, &prot, &psize, true)) {
        pa = -1;
    }
    env->pend_isr = save[0];
    env->pend_ifa = save[1];
    env->pend_itir = save[2];
    env->pend_iha = save[3];
    env->pend_flags = save[4];
    return pa;
}

/* Insertion and purges */

static IA64TLBEntry make_entry(CPUIA64State *env, uint64_t va,
                               uint64_t pte, uint64_t itir)
{
    IA64TLBEntry e;

    e.ps = FIELD_EX64(itir, ITIR, PS);
    if (e.ps < 12) {
        e.ps = 12;
    }
    e.vpn = va & ~((1ull << e.ps) - 1);
    e.pte = pte;
    e.itir = itir;
    e.rid = rr_rid(env, va);
    e.valid = true;
    return e;
}

static bool overlap(const IA64TLBEntry *e, uint64_t va, int ps, uint32_t rid)
{
    uint64_t size = 1ull << MAX(ps, e->ps);

    return e->valid && e->rid == rid && ((e->vpn ^ va) & ~(size - 1)) == 0;
}

/* Purge the translation cache entries overlapping va/ps in one TLB. */
static void tc_purge_one(CPUIA64State *env, IA64TLBEntry *tc, uint64_t va,
                         int ps, uint32_t rid)
{
    for (int i = 0; i < IA64_NUM_TC; i++) {
        if (overlap(&tc[i], va, ps, rid)) {
            tlb_flush_entry(env, &tc[i]);
            tc[i].valid = false;
        }
    }
}

/* ptc purges both translation caches. */
static void tc_purge(CPUIA64State *env, uint64_t va, int ps, uint32_t rid)
{
    tc_purge_one(env, env->itc, va, ps, rid);
    tc_purge_one(env, env->dtc, va, ps, rid);
}

void HELPER(itc)(CPUIA64State *env, uint32_t insn, uint64_t pte)
{
    IA64TLBEntry e = make_entry(env, env->cr[CR_IFA], pte, env->cr[CR_ITIR]);
    IA64TLBEntry *tc = insn ? env->itc : env->dtc;
    uint32_t *next = insn ? &env->itc_next : &env->dtc_next;

    /* an insertion replaces overlapping entries of its own TLB only */
    tc_purge_one(env, tc, e.vpn, e.ps, e.rid);
    if (tc[*next].valid) {
        tlb_flush_entry(env, &tc[*next]);
    }
    tc[*next] = e;
    *next = (*next + 1) % IA64_NUM_TC;
}

static void itr_insert(CPUIA64State *env, bool insn, int slot, uint64_t pte)
{
    IA64TLBEntry e = make_entry(env, env->cr[CR_IFA], pte, env->cr[CR_ITIR]);
    IA64TLBEntry *tr = insn ? env->itr : env->dtr;

    slot &= IA64_NUM_TR - 1;
    tc_purge(env, e.vpn, e.ps, e.rid);
    if (tr[slot].valid) {
        tlb_flush_entry(env, &tr[slot]);
    }
    tr[slot] = e;
    tlb_flush_entry(env, &e);
}

void HELPER(ptc)(CPUIA64State *env, uint64_t va, uint64_t r2)
{
    int ps = extract64(r2, 2, 6);

    tc_purge(env, va & ~((1ull << ps) - 1), ps, rr_rid(env, va));
}

void HELPER(ptr)(CPUIA64State *env, uint32_t insn, uint64_t va, uint64_t r2)
{
    int ps = extract64(r2, 2, 6);
    IA64TLBEntry *tr = insn ? env->itr : env->dtr;
    uint32_t rid = rr_rid(env, va);

    va &= ~((1ull << ps) - 1);
    for (int i = 0; i < IA64_NUM_TR; i++) {
        if (overlap(&tr[i], va, ps, rid)) {
            tlb_flush_entry(env, &tr[i]);
            tr[i].valid = false;
        }
    }
    tc_purge(env, va, ps, rid);
}

void HELPER(ptc_e)(CPUIA64State *env)
{
    ia64_tlb_flush_all(env);
}

uint64_t HELPER(thash)(CPUIA64State *env, uint64_t va)
{
    return ia64_thash(env, va);
}

uint64_t HELPER(ttag)(CPUIA64State *env, uint64_t va)
{
    return ia64_ttag(env, va);
}

/* tpa: translation of va; faults like a non-access data reference */
uint64_t HELPER(tpa)(CPUIA64State *env, uint64_t va)
{
    hwaddr pa;
    int prot, vec;
    uint64_t psize;
    IA64TLBEntry *e;

    if (!(env->psr & PSR_DT)) {
        return va & ((1ull << (IA64_IMPL_PA_MSB + 1)) - 1);
    }
    e = tlb_lookup(env, va, false);
    if (!e || !(e->pte & PTE_P)) {
        vec = ia64_translate(env, va, MMU_DATA_LOAD, 0, true, &pa, &prot,
                             &psize, false);
        ia64_raise_fault(env, vec, ISR_NA | ISR_R, env->pend_flags, va,
                         GETPC());
    }
    psize = 1ull << e->ps;
    return ((e->pte & R_PTE_PPN_MASK) & ~(psize - 1)) | (va & (psize - 1));
}

uint64_t HELPER(tak)(CPUIA64State *env, uint64_t va)
{
    IA64TLBEntry *e = tlb_lookup(env, va, false);

    if (!(env->psr & PSR_DT) || !e || !(e->pte & PTE_P)) {
        return 1;
    }
    return FIELD_EX64(e->itir, ITIR, KEY);
}

/*
 * probe: kind 0 r, 1 w, 2 r.fault, 3 w.fault, 4 rw.fault.
 * The regular form answers whether the access would be allowed.
 */
uint64_t HELPER(probe)(CPUIA64State *env, uint32_t kind, uint64_t va,
                       uint64_t pl)
{
    hwaddr pa;
    int prot;
    uint64_t psize;
    int vec;
    int cpl = ia64_cpl(env);
    MMUAccessType t = (kind == 1 || kind == 3) ? MMU_DATA_STORE
                                               : MMU_DATA_LOAD;

    pl &= 3;
    if (pl < cpl) {
        pl = cpl;
    }
    vec = ia64_translate(env, va, t, pl, env->psr & PSR_DT, &pa, &prot,
                         &psize, true);
    if (kind == 4 && !vec) {
        vec = ia64_translate(env, va, MMU_DATA_STORE, pl, env->psr & PSR_DT,
                             &pa, &prot, &psize, true);
    }
    if (kind < 2) {
        if (vec == IVT_DTLB || vec == IVT_ALT_DTLB) {
            /* a TLB miss faults for the regular form too */
            ia64_raise_fault(env, vec, ISR_NA | access_isr(t),
                             env->pend_flags, va, GETPC());
        }
        env->pend_flags = 0;
        return vec == 0;
    }
    if (vec) {
        ia64_raise_fault(env, vec, ISR_NA | access_isr(t), env->pend_flags,
                         va, GETPC());
    }
    return 0;
}

/*
 * Register stack engine.
 *
 * gr[32..32+sof) hold the current frame.  rse[0..ndirty) hold the dirty
 * registers of the parent frames, rse[0] being the next one to go to
 * ar.bspstore.  The clean partition is not modelled: registers written to
 * the backing store are reloaded from memory when needed.
 */

static int rse_mmu_idx(CPUIA64State *env)
{
    int pl = FIELD_EX64(env->ar[AR_RSC], RSC, PL);

    return pl | ((env->psr & PSR_RT) ? MMU_IDX_VIRT : 0);
}

static uint64_t rse_skip_regs(uint64_t addr, int64_t num)
{
    int64_t delta = ((addr >> 3) & 0x3f) + num;

    if (num < 0) {
        delta -= 0x3e;
    }
    return addr + ((num + delta / 0x3f) << 3);
}

uint64_t ia64_bsp(CPUIA64State *env)
{
    return rse_skip_regs(env->ar[AR_BSPSTORE], env->rse_ndirty);
}

static void rse_spill(CPUIA64State *env, int n, uintptr_t ra)
{
    int idx = rse_mmu_idx(env);
    MemOpIdx oi = make_memop_idx(MO_LEUQ | MO_ALIGN, idx);

    for (int i = 0; i < n; i++) {
        uint64_t addr = env->ar[AR_BSPSTORE];

        if (ia64_is_rnat_slot(addr)) {
            cpu_stq_mmu(env, addr, env->ar[AR_RNAT], oi, ra);
            env->ar[AR_RNAT] = 0;
            addr += 8;
            env->ar[AR_BSPSTORE] = addr;
        }
        cpu_stq_mmu(env, addr, env->rse[0], oi, ra);
        env->ar[AR_RNAT] = deposit64(env->ar[AR_RNAT], (addr >> 3) & 0x3f, 1,
                                     env->rse_nat[0]);
        env->ar[AR_BSPSTORE] = addr + 8;
        memmove(env->rse, env->rse + 1, (env->rse_ndirty - 1) *
                sizeof(env->rse[0]));
        memmove(env->rse_nat, env->rse_nat + 1, (env->rse_ndirty - 1) *
                sizeof(env->rse_nat[0]));
        env->rse_ndirty--;
    }
}

/* Load one register from below ar.bspstore into the bottom of the stack. */
static void rse_fill_one(CPUIA64State *env, uintptr_t ra)
{
    int idx = rse_mmu_idx(env);
    MemOpIdx oi = make_memop_idx(MO_LEUQ | MO_ALIGN, idx);
    uint64_t addr = env->ar[AR_BSPSTORE] - 8;
    uint64_t val;

    if (ia64_is_rnat_slot(addr)) {
        env->ar[AR_RNAT] = cpu_ldq_mmu(env, addr, oi, ra);
        env->ar[AR_BSPSTORE] = addr;
        addr -= 8;
    }
    val = cpu_ldq_mmu(env, addr, oi, ra);
    g_assert(env->rse_ndirty < IA64_NUM_STACKED);
    memmove(env->rse + 1, env->rse, env->rse_ndirty * sizeof(env->rse[0]));
    memmove(env->rse_nat + 1, env->rse_nat,
            env->rse_ndirty * sizeof(env->rse_nat[0]));
    env->rse[0] = val;
    env->rse_nat[0] = (env->ar[AR_RNAT] >> ((addr >> 3) & 0x3f)) & 1;
    env->rse_ndirty++;
    env->ar[AR_BSPSTORE] = addr;
}

/* Make room so that ndirty + frame fits the physical register file. */
static void rse_make_room(CPUIA64State *env, int frame, uintptr_t ra)
{
    int over = env->rse_ndirty + frame - IA64_NUM_STACKED;

    if (over > 0) {
        rse_spill(env, over, ra);
    }
}

/* Push the first n registers of the current frame onto the dirty stack. */
static void rse_push(CPUIA64State *env, int n)
{
    g_assert(env->rse_ndirty + n <= IA64_NUM_STACKED);
    memcpy(env->rse + env->rse_ndirty, env->gr + 32, n * sizeof(uint64_t));
    memcpy(env->rse_nat + env->rse_ndirty, env->nat + 32,
           n * sizeof(uint32_t));
    env->rse_ndirty += n;
}

/* Pop n registers from the dirty stack into gr[32..32+n). */
static void rse_pop(CPUIA64State *env, int n, uintptr_t ra)
{
    while (env->rse_ndirty < n) {
        rse_fill_one(env, ra);
    }
    env->rse_ndirty -= n;
    memcpy(env->gr + 32, env->rse + env->rse_ndirty, n * sizeof(uint64_t));
    memcpy(env->nat + 32, env->rse_nat + env->rse_ndirty,
           n * sizeof(uint32_t));
}

void ia64_rse_frame_load(CPUIA64State *env, int n, uintptr_t ra)
{
    rse_pop(env, n, ra);
}

void HELPER(br_call)(CPUIA64State *env)
{
    uint64_t cfm = env->cfm;
    int sof = FIELD_EX64(cfm, CFM, SOF);
    int sol = FIELD_EX64(cfm, CFM, SOL);

    env->ar[AR_PFS] = (cfm & CFM_MASK) |
                      ((env->ar[AR_EC] & 0x3f) << R_PFS_PEC_SHIFT) |
                      ((uint64_t)ia64_cpl(env) << R_PFS_PPL_SHIFT);
    rse_push(env, sol);
    memmove(env->gr + 32, env->gr + 32 + sol, (sof - sol) * sizeof(uint64_t));
    memmove(env->nat + 32, env->nat + 32 + sol,
            (sof - sol) * sizeof(uint32_t));
    env->cfm = sof - sol;
}

void HELPER(br_ret)(CPUIA64State *env)
{
    uintptr_t ra = GETPC();
    uint64_t pfs = env->ar[AR_PFS];
    uint64_t pfm = FIELD_EX64(pfs, PFS, PFM);
    int sof = FIELD_EX64(pfm, CFM, SOF);
    int sol = FIELD_EX64(pfm, CFM, SOL);
    int outs = sof - sol;
    int ppl = FIELD_EX64(pfs, PFS, PPL);

    if (sol > sof || sof > 96) {
        /* the frame is lost; see "RSE Operation" */
        pfm = 0;
        sof = sol = outs = 0;
    }
    /* fill first, so that a fault leaves the frame untouched */
    while (env->rse_ndirty < sol) {
        rse_fill_one(env, ra);
    }
    memmove(env->gr + 32 + sol, env->gr + 32, outs * sizeof(uint64_t));
    memmove(env->nat + 32 + sol, env->nat + 32, outs * sizeof(uint32_t));
    rse_pop(env, sol, ra);
    env->cfm = pfm;
    env->ar[AR_EC] = FIELD_EX64(pfs, PFS, PEC);
    if (ia64_cpl(env) < ppl) {
        env->psr = deposit64(env->psr, PSR_CPL_SHIFT, 2, ppl);
    }
    rse_make_room(env, 0, ra);
}

/* kind: 0 ctop, 1 cexit, 2 wtop, 3 wexit; qp is the physical predicate */
uint32_t HELPER(br_loop)(CPUIA64State *env, uint32_t kind, uint32_t qp)
{
    uint64_t cfm = env->cfm;
    int sor8 = FIELD_EX64(cfm, CFM, SOR) * 8;
    int rrb_pr = FIELD_EX64(cfm, CFM, RRB_PR);
    int p63 = 16 + (63 - 16 + rrb_pr) % 48;
    bool taken, rotate;
    uint64_t lc = env->ar[AR_LC], ec = env->ar[AR_EC] & 0x3f;

    if (kind < 2) {
        taken = lc != 0 || ec > 1;
        if (lc != 0) {
            env->ar[AR_LC] = lc - 1;
            env->pr[p63] = 1;
            rotate = true;
        } else if (ec != 0) {
            env->ar[AR_EC] = ec - 1;
            env->pr[p63] = 0;
            rotate = true;
        } else {
            env->pr[p63] = 0;
            rotate = false;
        }
    } else {
        bool q = qp == 0 ? true : env->pr[qp];

        taken = q || ec > 1;
        if (q) {
            env->pr[p63] = 0;
            rotate = true;
        } else if (ec != 0) {
            env->ar[AR_EC] = ec - 1;
            env->pr[p63] = 0;
            rotate = true;
        } else {
            env->pr[p63] = 0;
            rotate = false;
        }
    }
    if (kind == 1 || kind == 3) {
        taken = !taken;
    }
    if (rotate) {
        int gr = FIELD_EX64(cfm, CFM, RRB_GR);
        int fr = FIELD_EX64(cfm, CFM, RRB_FR);

        if (sor8) {
            gr = (gr + sor8 - 1) % sor8;
        }
        fr = (fr + 95) % 96;
        rrb_pr = (rrb_pr + 47) % 48;
        cfm = FIELD_DP64(cfm, CFM, RRB_GR, gr);
        cfm = FIELD_DP64(cfm, CFM, RRB_FR, fr);
        cfm = FIELD_DP64(cfm, CFM, RRB_PR, rrb_pr);
        env->cfm = cfm;
    }
    return taken;
}

void HELPER(alloc)(CPUIA64State *env, uint32_t sof, uint32_t sol,
                   uint32_t sor)
{
    uint64_t cfm = env->cfm;
    bool rrb = FIELD_EX64(cfm, CFM, RRB_GR) || FIELD_EX64(cfm, CFM, RRB_FR) ||
               FIELD_EX64(cfm, CFM, RRB_PR);

    if (rrb && sor != FIELD_EX64(cfm, CFM, SOR)) {
        ia64_raise(env, IVT_GENERAL, GE_ILLEGAL_OP, 0);
    }
    rse_make_room(env, sof, GETPC());
    for (uint32_t i = FIELD_EX64(cfm, CFM, SOF); i < sof; i++) {
        env->nat[32 + i] = 0;
    }
    cfm = FIELD_DP64(cfm, CFM, SOF, sof);
    cfm = FIELD_DP64(cfm, CFM, SOL, sol);
    cfm = FIELD_DP64(cfm, CFM, SOR, sor);
    env->cfm = cfm;
}

void HELPER(cover)(CPUIA64State *env)
{
    int sof = FIELD_EX64(env->cfm, CFM, SOF);

    if (!(env->psr & PSR_IC)) {
        if (ia64_cpl(env) != 0) {
            ia64_raise(env, IVT_GENERAL, GE_PRIV_OP, 0);
        }
        env->cr[CR_IFS] = env->cfm | IFS_V;
    }
    rse_push(env, sof);
    env->cfm = 0;
}

void HELPER(flushrs)(CPUIA64State *env)
{
    rse_spill(env, env->rse_ndirty, GETPC());
}

void HELPER(loadrs)(CPUIA64State *env)
{
    uintptr_t ra = GETPC();
    /* RSC.loadrs is a byte count (low three bits ignored) */
    uint64_t bytes = FIELD_EX64(env->ar[AR_RSC], RSC, LOADRS) & ~7ull;
    uint64_t bsp = ia64_bsp(env);
    uint64_t target = bsp - bytes;

    if (FIELD_EX64(env->cfm, CFM, SOF) != 0 ||
        FIELD_EX64(env->ar[AR_RSC], RSC, MODE) != 0) {
        ia64_raise(env, IVT_GENERAL, GE_ILLEGAL_OP, 0);
    }
    if (target >= env->ar[AR_BSPSTORE]) {
        /* discard the dirty registers below target */
        while (env->ar[AR_BSPSTORE] < target && env->rse_ndirty) {
            uint64_t addr = env->ar[AR_BSPSTORE];

            if (ia64_is_rnat_slot(addr)) {
                addr += 8;
            } else {
                memmove(env->rse, env->rse + 1,
                        (env->rse_ndirty - 1) * sizeof(env->rse[0]));
                memmove(env->rse_nat, env->rse_nat + 1,
                        (env->rse_ndirty - 1) * sizeof(env->rse_nat[0]));
                env->rse_ndirty--;
                addr += 8;
            }
            env->ar[AR_BSPSTORE] = addr;
        }
        env->ar[AR_BSPSTORE] = target;
    } else {
        while (env->ar[AR_BSPSTORE] > target) {
            uint64_t addr = env->ar[AR_BSPSTORE] - 8;

            if (ia64_is_rnat_slot(addr)) {
                MemOpIdx oi = make_memop_idx(MO_LEUQ | MO_ALIGN,
                                             rse_mmu_idx(env));

                env->ar[AR_RNAT] = cpu_ldq_mmu(env, addr, oi, ra);
                env->ar[AR_BSPSTORE] = addr;
                continue;
            }
            rse_fill_one(env, ra);
        }
    }
}

void HELPER(rfi)(CPUIA64State *env)
{
    uint64_t ipsr = env->cr[CR_IPSR];
    uint64_t ifs = env->cr[CR_IFS];


    if (ifs & IFS_V) {
        uint64_t ifm = ifs & CFM_MASK;
        int sof = FIELD_EX64(ifm, CFM, SOF);

        rse_pop(env, sof, GETPC());
        env->cfm = ifm;
    }
    env->ip = env->cr[CR_IIP] & ~0xfull;
    env->ri = (ipsr >> PSR_RI_SHIFT) & 3;
    if (env->ri == 3) {
        env->ri = 0;
    }
    ia64_set_psr(env, ipsr);
}

void HELPER(bsw)(CPUIA64State *env, uint32_t bn)
{
    ia64_switch_bank(env, bn);
}

/* epc: promote on an execute-only page */
void HELPER(epc)(CPUIA64State *env)
{
    IA64TLBEntry *e;
    int pl;

    if (!(env->psr & PSR_IT)) {
        return;
    }
    e = tlb_lookup(env, env->ip, true);
    if (!e || FIELD_EX64(e->pte, PTE, AR) != 7) {
        return;
    }
    pl = FIELD_EX64(e->pte, PTE, PL);
    if (FIELD_EX64(env->ar[AR_PFS], PFS, PPL) < ia64_cpl(env)) {
        ia64_raise(env, IVT_GENERAL, GE_ILLEGAL_OP, 0);
    }
    if (pl < ia64_cpl(env)) {
        env->psr = deposit64(env->psr, PSR_CPL_SHIFT, 2, pl);
    }
}

/*
 * Application and control registers
 */

void HELPER(mov_to_ar)(CPUIA64State *env, uint32_t ar, uint64_t val)
{
    int cpl = ia64_cpl(env);

    switch (ar) {
    case AR_K0 ... AR_K7:
        if (cpl) {
            ia64_raise(env, IVT_GENERAL, GE_PRIV_REG, 0);
        }
        env->ar[ar] = val;
        break;
    case AR_RSC: {
        int pl = FIELD_EX64(val, RSC, PL);

        if (pl < cpl) {
            val = FIELD_DP64(val, RSC, PL, cpl);
        }
        env->ar[ar] = val & 0x3fff001full;
        break;
    }
    case AR_BSP:
        ia64_raise(env, IVT_GENERAL, GE_ILLEGAL_OP, 0);
    case AR_BSPSTORE:
        env->ar[ar] = val & ~7ull;
        break;
    case AR_ITC:
        if (cpl) {
            ia64_raise(env, IVT_GENERAL, GE_PRIV_REG, 0);
        }
        ia64_write_itc(env, val);
        break;
    case AR_RUC:
        if (cpl) {
            ia64_raise(env, IVT_GENERAL, GE_PRIV_REG, 0);
        }
        env->ar[ar] = val;
        break;
    case AR_FPSR:
        env->ar[ar] = val & ((1ull << 58) - 1);
        break;
    case AR_EC:
        env->ar[ar] = val & 0x3f;
        break;
    default:
        env->ar[ar] = val;
        break;
    }
}

uint64_t HELPER(mov_from_ar)(CPUIA64State *env, uint32_t ar)
{
    switch (ar) {
    case AR_BSP:
        return ia64_bsp(env);
    case AR_ITC:
        if (ia64_cpl(env) && (env->psr & PSR_SI)) {
            ia64_raise(env, IVT_GENERAL, GE_PRIV_REG, 0);
        }
        return ia64_read_itc(env);
    case AR_RUC:
        return ia64_read_itc(env);
    default:
        return env->ar[ar];
    }
}

void HELPER(mov_to_cr)(CPUIA64State *env, uint32_t cr, uint64_t val)
{
    switch (cr) {
    case CR_IVR:
    case CR_IRR0 ... CR_IRR3:
        ia64_raise(env, IVT_GENERAL, GE_ILLEGAL_OP, 0);
    case CR_EOI:
        lsapic_eoi(env);
        break;
    case CR_TPR:
        env->cr[cr] = val;
        ia64_update_irq(env);
        break;
    case CR_ITM:
        env->cr[cr] = val;
        ia64_rearm_itm(env);
        break;
    case CR_ITV:
        env->cr[cr] = val;
        break;
    case CR_IVA:
        env->cr[cr] = val & ~0x7fffull;
        break;
    default:
        env->cr[cr] = val;
        break;
    }
}

uint64_t HELPER(mov_from_cr)(CPUIA64State *env, uint32_t cr)
{
    switch (cr) {
    case CR_IVR:
        return lsapic_read_ivr(env);
    case CR_IRR0 ... CR_IRR3:
        return env->irr[cr - CR_IRR0];
    default:
        return env->cr[cr];
    }
}

/* which: 0 rr, 1 pkr, 2 ibr, 3 dbr, 4 pmc, 5 pmd, 6 cpuid, 7 msr,
 * 8 itr.i, 9 itr.d */
void HELPER(mov_to_ind)(CPUIA64State *env, uint32_t which, uint64_t idx,
                        uint64_t val)
{
    switch (which) {
    case 0:
        env->rr[idx >> 61] = val & (R_RR_VE_MASK | R_RR_PS_MASK |
                                    R_RR_RID_MASK);
        tlb_flush(env_cpu(env));
        break;
    case 1:
        env->pkr[idx & 15] = val;
        break;
    case 2:
        env->ibr[idx & 15] = val;
        break;
    case 3:
        env->dbr[idx & 15] = val;
        break;
    case 8:
    case 9:
        itr_insert(env, which == 8, idx, val);
        break;
    default:
        break;
    }
}

uint64_t HELPER(mov_from_ind)(CPUIA64State *env, uint32_t which, uint64_t idx)
{
    switch (which) {
    case 0:
        return env->rr[idx >> 61];
    case 1:
        return env->pkr[idx & 15];
    case 2:
        return env->ibr[idx & 15];
    case 3:
        return env->dbr[idx & 15];
    case 6:
        switch (idx & 0xff) {
        case 0: return 0x49656e69756e6547ull;   /* "GenuineI" */
        case 1: return 0x000000006c65746eull;   /* "ntel" */
        case 2: return 0;
        case 3:
            /* number 4, revision 0, model 0, family 0x1f (Itanium 2) */
            return (4ull) | (0x1full << 24);
        case 4: return 0;
        }
        return 0;
    default:
        return 0;
    }
}

/* mov psr.l = r2 (l) or mov psr.um = r2 */
void HELPER(mov_to_psr)(CPUIA64State *env, uint64_t val, uint32_t l)
{
    uint64_t psr = env->psr;

    if (l) {
        psr = (psr & ~0xffffffffull) | (val & 0xffffffffull);
    } else {
        psr = (psr & ~PSR_UM_MASK) | (val & PSR_UM_MASK);
    }
    ia64_set_psr(env, psr);
}

void HELPER(ssm_rsm)(CPUIA64State *env, uint64_t imm, uint32_t set)
{
    uint64_t psr = env->psr;

    imm &= 0xffffff;
    psr = set ? psr | imm : psr & ~imm;
    ia64_set_psr(env, psr);
}
