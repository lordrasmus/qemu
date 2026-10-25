/*
 * TI C64x+ interrupts, exceptions, control registers and the delay queue
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "cpu.h"
#include "exec/helper-proto.h"
#include "accel/tcg/cpu-loop.h"
#include "accel/tcg/cpu-ldst.h"
#include "exec/target_page.h"
#include "system/cpus.h"
#include "qemu/main-loop.h"

/* The delay queue */

static void dq_write(CPUC6xState *env, C6xDelayed *d)
{
    switch (d->dest) {
    case DQ_DEST_PC:
        env->pc = d->val;
        break;
    case DQ_DEST_ILC:
        env->ilc = d->val;
        break;
    case DQ_DEST_RILC:
        env->rilc = d->val;
        break;
    default:
        env->gpr[d->dest] = d->val;
        break;
    }
}

static void dq_update_live(CPUC6xState *env)
{
    int live = 0;

    for (int i = 0; i < DQ_SLOTS; i++) {
        if (env->dq_n[i]) {
            live = i + 1;
        }
    }
    env->dq_live = live;
}

void HELPER(dq_push)(CPUC6xState *env, uint32_t slot, uint32_t dest,
                     uint32_t val, uint32_t cond)
{
    C6xDelayed *d;

    if (!cond) {
        return;
    }
    assert(slot < DQ_SLOTS && env->dq_n[slot] < DQ_ENTRIES);
    d = &env->dq[slot][env->dq_n[slot]++];
    d->dest = dest;
    d->val = val;
    if (slot >= env->dq_live) {
        env->dq_live = slot + 1;
    }
}

/* End of a cycle: retire slot 0 */
void HELPER(dq_tick)(CPUC6xState *env)
{
    for (int i = 0; i < env->dq_n[0]; i++) {
        dq_write(env, &env->dq[0][i]);
    }
    memmove(&env->dq_n[0], &env->dq_n[1], DQ_SLOTS - 1);
    memmove(&env->dq[0], &env->dq[1], sizeof(env->dq[0]) * (DQ_SLOTS - 1));
    env->dq_n[DQ_SLOTS - 1] = 0;
    dq_update_live(env);
}

/*
 * Interrupts and exceptions let everything in flight complete before the
 * handler starts.
 */
void c6x_dq_drain(CPUC6xState *env)
{
    uint32_t pc = env->pc;

    while (env->dq_live) {
        helper_dq_tick(env);
    }
    /* a branch still in flight is annulled */
    env->pc = pc;
}

/* Interrupts */

static int pending_irq(CPUC6xState *env)
{
    uint32_t p = env->ifr & env->ier & INT_MASKABLE;

    if (!p || !(env->ier & IER_NMIE) || !FIELD_EX32(env->tsr, TSR, GIE)) {
        return -1;
    }
    return ctz32(p);
}

void c6x_update_irq(CPUC6xState *env)
{
    CPUState *cs = env_cpu(env);

    BQL_LOCK_GUARD();

    if (pending_irq(env) >= 0) {
        cpu_interrupt(cs, CPU_INTERRUPT_HARD);
    } else {
        cpu_reset_interrupt(cs, CPU_INTERRUPT_HARD);
    }
}

bool c6x_cpu_exec_interrupt(CPUState *cs, int interrupt_request)
{
    CPUC6xState *env = cpu_env(cs);

    if ((interrupt_request & CPU_INTERRUPT_HARD) && env->clean_eps >= 5 &&
        pending_irq(env) >= 0) {
        cs->exception_index = EXCP_IRQ;
        c6x_cpu_do_interrupt(cs);
        return true;
    }
    return false;
}

void c6x_cpu_do_interrupt(CPUState *cs)
{
    CPUC6xState *env = cpu_env(cs);
    uint32_t tsr = env->tsr;
    uint32_t istb = env->istp & ~0x3ffu;

    c6x_dq_drain(env);

    switch (cs->exception_index) {
    case EXCP_IRQ: {
        int n = pending_irq(env);

        if (n < 0) {
            break;
        }
        qemu_log_mask(CPU_LOG_INT, "c6x: interrupt %d at %08x\n", n,
                      env->pc);
        env->irp = env->pc;
        env->itsr = tsr & 0xffff;
        tsr = FIELD_DP32(tsr, TSR, GIE, 0);
        tsr = FIELD_DP32(tsr, TSR, SGIE, 0);
        tsr = FIELD_DP32(tsr, TSR, XEN, 0);
        tsr = FIELD_DP32(tsr, TSR, CXM, 0);
        tsr = FIELD_DP32(tsr, TSR, INT, 1);
        tsr = FIELD_DP32(tsr, TSR, EXC, 0);
        tsr = FIELD_DP32(tsr, TSR, SPLX, 0);
        tsr = FIELD_DP32(tsr, TSR, IB, 0);
        env->tsr = tsr;
        env->ifr &= ~(1u << n);
        env->pc = istb + n * 32;
        break;
    }
    case EXCP_SWE:
    case EXCP_SWENR:
    case EXCP_INTERNAL:
        if (cs->exception_index == EXCP_INTERNAL) {
            env->efr |= EFR_IXF;
            env->ierr |= env->excp_ierr;
            qemu_log_mask(CPU_LOG_INT, "c6x: internal exception %x at %08x\n",
                          env->excp_ierr, env->pc);
        } else {
            env->efr |= EFR_SXF;
        }
        if (cs->exception_index != EXCP_SWENR) {
            env->nrp = env->pc;
        }
        env->ntsr = (tsr & 0xffff) |
                    (cs->exception_index == EXCP_INTERNAL ? R_TSR_HWE_MASK : 0);
        tsr = FIELD_DP32(tsr, TSR, GIE, 0);
        tsr = FIELD_DP32(tsr, TSR, SGIE, 0);
        tsr = FIELD_DP32(tsr, TSR, XEN, 0);
        tsr = FIELD_DP32(tsr, TSR, CXM, 0);
        tsr = FIELD_DP32(tsr, TSR, INT, 0);
        tsr = FIELD_DP32(tsr, TSR, EXC, 1);
        tsr = FIELD_DP32(tsr, TSR, SPLX, 0);
        tsr = FIELD_DP32(tsr, TSR, IB, 0);
        env->tsr = tsr;
        env->pc = cs->exception_index == EXCP_SWENR ? env->rep : istb + 0x20;
        break;
    }
    env->clean_eps = 0;
    cs->exception_index = -1;
    c6x_update_irq(env);
}

G_NORETURN void HELPER(raise)(CPUC6xState *env, uint32_t excp, uint32_t ierr)
{
    CPUState *cs = env_cpu(env);

    cs->exception_index = excp;
    env->excp_ierr = ierr;
    cpu_loop_exit(cs);
}

G_NORETURN void HELPER(idle)(CPUC6xState *env)
{
    CPUState *cs = env_cpu(env);

    env->clean_eps = 5;
    cs->halted = 1;
    cs->exception_index = EXCP_HLT;
    cpu_loop_exit(cs);
}

/* Control registers */

uint64_t c6x_tsc(CPUC6xState *env)
{
    int64_t ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (!env->tsc_running) {
        return 0;
    }
    return muldiv64(ns, env->cpu_freq, NANOSECONDS_PER_SECOND) -
           env->tsc_base;
}

uint32_t c6x_csr_read(CPUC6xState *env)
{
    return (env->csr & ~(R_CSR_GIE_MASK | R_CSR_PGIE_MASK)) |
           FIELD_EX32(env->tsr, TSR, GIE) |
           FIELD_EX32(env->itsr, TSR, GIE) << 1;
}

static bool cr_restricted(int cr)
{
    switch (cr) {
    case CR_ECR_EFR:
    case CR_ICR:
    case CR_IER:
    case CR_IERR:
    case CR_ISR_IFR:
    case CR_ISTP:
    case CR_ITSR:
    case CR_NTSR:
    case CR_REP:
    case CR_DIER:
        return true;
    }
    return false;
}

static G_NORETURN void cr_fault(CPUC6xState *env, uintptr_t ra)
{
    CPUState *cs = env_cpu(env);

    cs->exception_index = EXCP_INTERNAL;
    env->excp_ierr = IERR_RAX | IERR_PRX;
    cpu_loop_exit_restore(cs, ra);
}

void HELPER(mvc_to)(CPUC6xState *env, uint32_t cr, uint32_t val)
{
    bool user = c6x_user_mode(env);

    if (user && cr_restricted(cr)) {
        cr_fault(env, GETPC());
    }

    switch (cr) {
    case CR_AMR:
        env->amr = val & 0x03ffffff;
        break;
    case CR_CSR:
        env->tsr = FIELD_DP32(env->tsr, TSR, GIE, val & 1);
        if (!(val & R_CSR_SAT_MASK)) {
            env->csr &= ~R_CSR_SAT_MASK;
        }
        if (!user) {
            env->itsr = FIELD_DP32(env->itsr, TSR, GIE, (val >> 1) & 1);
            env->csr = (env->csr & ~R_CSR_PWRD_MASK) | (val & R_CSR_PWRD_MASK);
        }
        break;
    case CR_ISR_IFR:
        env->ifr |= val & INT_MASKABLE;
        break;
    case CR_ICR:
        env->ifr &= ~(val & INT_MASKABLE);
        break;
    case CR_IER:
        env->ier = (env->ier & IER_NMIE) | (val & (INT_MASKABLE | IER_NMIE)) |
                   1;
        break;
    case CR_ISTP:
        env->istp = val & ~0x3ffu;
        break;
    case CR_IRP:
        env->irp = val;
        break;
    case CR_NRP:
        env->nrp = val;
        break;
    case CR_TSCL:
        if (!env->tsc_running) {
            env->tsc_running = true;
            env->tsc_base = muldiv64(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL),
                                     env->cpu_freq, NANOSECONDS_PER_SECOND);
        }
        break;
    case CR_REP:
        env->rep = val;
        break;
    case CR_SSR:
        env->ssr = val & 0x3f;
        break;
    case CR_GPLYA:
        env->gplya = val;
        break;
    case CR_GPLYB:
        env->gplyb = val;
        break;
    case CR_GFPGFR:
        env->gfpgfr = val & 0x070000ff;
        break;
    case CR_DIER:
        env->dier = val;
        break;
    case CR_FADCR:
        env->fadcr = val;
        break;
    case CR_FAUCR:
        env->faucr = val;
        break;
    case CR_FMCR:
        env->fmcr = val;
        break;
    case CR_TSR: {
        uint32_t m = R_TSR_GIE_MASK | R_TSR_SGIE_MASK;

        if (!user) {
            m |= R_TSR_XEN_MASK | R_TSR_DBGM_MASK;
            if (val & R_TSR_GEE_MASK) {
                env->tsr |= R_TSR_GEE_MASK;
            }
            if (!(val & R_TSR_EXC_MASK)) {
                env->tsr &= ~R_TSR_EXC_MASK;
            }
        }
        env->tsr = (env->tsr & ~m) | (val & m);
        break;
    }
    case CR_ITSR:
        env->itsr = val & 0xc6df;
        break;
    case CR_NTSR:
        env->ntsr = val & 0x1c6df;
        break;
    case CR_ECR_EFR:
        env->efr &= ~val;
        break;
    case CR_IERR:
        env->ierr = val & 0x1ff;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "c6x: MVC to control register %u\n",
                      cr);
        break;
    }
    c6x_update_irq(env);
}

uint32_t HELPER(mvc_from)(CPUC6xState *env, uint32_t cr)
{
    if (c6x_user_mode(env) && cr_restricted(cr)) {
        cr_fault(env, GETPC());
    }

    switch (cr) {
    case CR_AMR:
        return env->amr;
    case CR_CSR:
        return c6x_csr_read(env);
    case CR_ISR_IFR:
        return env->ifr;
    case CR_IER:
        return env->ier;
    case CR_ISTP: {
        uint32_t p = env->ifr & env->ier & INT_MASKABLE;

        return env->istp | (p ? ctz32(p) << 5 : 0);
    }
    case CR_IRP:
        return env->irp;
    case CR_NRP:
        return env->nrp;
    case CR_TSCL: {
        uint64_t t = c6x_tsc(env);

        env->tsch = t >> 32;
        return t;
    }
    case CR_TSCH:
        return env->tsch;
    case CR_ILC:
        return env->ilc;
    case CR_RILC:
        return env->rilc;
    case CR_REP:
        return env->rep;
    case CR_DNUM:
        return env->dnum;
    case CR_SSR:
        return env->ssr;
    case CR_GPLYA:
        return env->gplya;
    case CR_GPLYB:
        return env->gplyb;
    case CR_GFPGFR:
        return env->gfpgfr;
    case CR_DIER:
        return env->dier;
    case CR_FADCR:
        return env->fadcr;
    case CR_FAUCR:
        return env->faucr;
    case CR_FMCR:
        return env->fmcr;
    case CR_TSR:
        return env->tsr & 0xffff;
    case CR_ITSR:
        return env->itsr;
    case CR_NTSR:
        return env->ntsr;
    case CR_ECR_EFR:
        return env->efr;
    case CR_IERR:
        return env->ierr;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "c6x: MVC from control register %u\n", cr);
    return 0;
}

/* B IRP: TSR from ITSR.  B NRP: TSR from NTSR, NMIE set. */
uint32_t HELPER(b_irp)(CPUC6xState *env)
{
    env->tsr = (env->tsr & ~0xc6dfu) | (env->itsr & 0xc6df) |
               (env->tsr & R_TSR_GEE_MASK);
    c6x_update_irq(env);
    return env->irp & ~3u;
}

uint32_t HELPER(b_nrp)(CPUC6xState *env)
{
    env->tsr = (env->tsr & ~0xc6dfu) | (env->ntsr & 0xc6df) |
               (env->tsr & R_TSR_GEE_MASK);
    env->ier |= IER_NMIE;
    c6x_update_irq(env);
    return env->nrp & ~3u;
}

void HELPER(dint)(CPUC6xState *env)
{
    env->tsr = FIELD_DP32(env->tsr, TSR, SGIE, FIELD_EX32(env->tsr, TSR, GIE));
    env->tsr = FIELD_DP32(env->tsr, TSR, GIE, 0);
    c6x_update_irq(env);
}

void HELPER(rint)(CPUC6xState *env)
{
    env->tsr = FIELD_DP32(env->tsr, TSR, GIE, FIELD_EX32(env->tsr, TSR, SGIE));
    env->tsr = FIELD_DP32(env->tsr, TSR, SGIE, 0);
    c6x_update_irq(env);
}

/* Address arithmetic with AMR circular buffers for A4-A7 and B4-B7 */
uint32_t HELPER(adda)(CPUC6xState *env, uint32_t base, uint32_t off,
                      uint32_t reg)
{
    int idx = (reg & 31) - 4 + (reg >= 32 ? 4 : 0);
    int mode = extract32(env->amr, idx * 2, 2);
    uint32_t size;

    if (mode == 0 || mode == 3) {
        return base + off;
    }
    size = 2u << extract32(env->amr, mode == 1 ? 16 : 21, 5);
    if (size == 0) {
        return base + off;
    }
    return (base & ~(size - 1)) | ((base + off) & (size - 1));
}

/* LL, SL, CMTL on a single CPU */
uint32_t HELPER(ll)(CPUC6xState *env, uint32_t addr)
{
    addr &= ~3u;
    env->ll_addr = addr;
    env->ll_valid = 1;
    return cpu_ldl_le_data_ra(env, addr, GETPC());
}

void HELPER(sl)(CPUC6xState *env, uint32_t addr, uint32_t val)
{
    addr &= ~3u;
    if (addr != env->ll_addr) {
        env->ll_valid = 0;
    }
    env->sl_val = val;
    env->sl_valid = 1;
}

uint32_t HELPER(cmtl)(CPUC6xState *env, uint32_t addr)
{
    addr &= ~3u;
    if (env->ll_valid && env->sl_valid && addr == env->ll_addr) {
        cpu_stl_le_data_ra(env, addr, env->sl_val, GETPC());
        env->ll_valid = 0;
        env->sl_valid = 0;
        return 1;
    }
    env->ll_valid = 0;
    env->sl_valid = 0;
    return 0;
}

/* SPLOOP, see translate-sploop.c.inc */

void HELPER(sploop_start)(CPUC6xState *env)
{
    env->spl_active = 0;
    env->spl_m = -1;
    env->spl_t = 0;
    env->spl_P = UINT32_MAX;
    env->spl_term = 0;
    env->spl_drain = 0;
}

/*
 * At the start of each period: does another iteration start?  kind is
 * the SPLOOP opfield, 0xc SPLOOP or 0xd SPLOOPD.
 */
void HELPER(sploop_boundary)(CPUC6xState *env, uint32_t kind, uint32_t ii,
                             uint32_t stages, uint32_t dynlen, uint32_t delay)
{
    int m = ++env->spl_m;
    uint32_t t = m * ii;
    uint32_t start = 0;

    if (!env->spl_term) {
        if (kind == 0xd && t <= 3) {
            start = 1;
        } else if (env->ilc) {
            env->ilc--;
            start = 1;
        } else {
            uint32_t epi = dynlen - ii;

            env->spl_term = 1;
            if (m >= stages) {
                env->spl_P = t + MIN(delay, epi);
                env->spl_drain = delay < epi;
            } else {
                uint32_t e = m ? (m - 1) * ii + dynlen : 0;

                env->spl_P = MAX(e, stages * ii);
            }
        }
    }
    env->spl_active = ((env->spl_active << 1) | start) &
                      (uint32_t)MAKE_64BIT_MASK(0, stages);
}

/* No MMU: virtual is physical */

hwaddr c6x_cpu_get_phys_page_debug(CPUState *cs, vaddr addr)
{
    return addr & TARGET_PAGE_MASK;
}

bool c6x_cpu_tlb_fill(CPUState *cs, vaddr address, int size,
                      MMUAccessType access_type, int mmu_idx,
                      bool probe, uintptr_t retaddr)
{
    address &= TARGET_PAGE_MASK;
    tlb_set_page(cs, address, address, PAGE_READ | PAGE_WRITE | PAGE_EXEC,
                 mmu_idx, TARGET_PAGE_SIZE);
    return true;
}
