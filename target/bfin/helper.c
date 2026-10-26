/*
 * Blackfin helpers: ASTAT, events
 *
 * Event handling after the Blackfin Processor Programming Reference,
 * Rev. 2.2, chapters 3 and 4.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h"
#include "cpu.h"
#include "exec/cpu-interrupt.h"
#include "exec/target_page.h"
#include "exec/cputlb.h"
#include "exec/page-protection.h"
#include "exec/helper-proto.h"
#include "accel/tcg/cpu-loop.h"

uint32_t bfin_get_astat(CPUBfinState *env)
{
    uint32_t val = 0;

    for (int i = 0; i < 32; i++) {
        val |= (env->astat[i] & 1) << i;
    }
    return val;
}

void bfin_set_astat(CPUBfinState *env, uint32_t val)
{
    for (int i = 0; i < 32; i++) {
        env->astat[i] = (val >> i) & 1;
    }
}

/*
 * SP is the user or the supervisor stack pointer depending on the mode;
 * the other one lives in USP (user SP while in supervisor mode) or ksp.
 */
static void bfin_switch_mode(CPUBfinState *env, bool was_super)
{
    bool super = bfin_supervisor(env);

    if (super == was_super) {
        return;
    }
    if (super) {
        env->reg[BFIN_USP] = env->reg[BFIN_SP];
        env->reg[BFIN_SP] = env->ksp;
    } else {
        env->ksp = env->reg[BFIN_SP];
        env->reg[BFIN_SP] = env->reg[BFIN_USP];
    }
}

/*
 * Take event @ivg: save the return address, mark it pending, enter
 * supervisor mode and jump to its vector.
 */
static void bfin_take_event(CPUBfinState *env, int ivg, uint32_t ret)
{
    bool was_super = bfin_supervisor(env);

    switch (ivg) {
    case 0:
        env->reg[BFIN_RETE] = ret;
        break;
    case 2:
        env->reg[BFIN_RETN] = ret;
        break;
    case 3:
        env->reg[BFIN_RETX] = ret;
        break;
    default:
        env->reg[BFIN_RETI] = ret;
        /* interrupts stay off until the handler saves RETI */
        env->ipend |= 1u << 4;
        break;
    }
    env->ipend |= 1u << ivg;
    env->ilat &= ~(1u << ivg);
    bfin_switch_mode(env, was_super);
    env->pc = env->evt[ivg];
}

G_NORETURN void helper_excpt(CPUBfinState *env, uint32_t excause)
{
    CPUState *cs = env_cpu(env);

    env->reg[BFIN_SEQSTAT] = deposit32(env->reg[BFIN_SEQSTAT], 0, 6, excause);
    cs->exception_index = EXCP_BFIN_EXCEPTION;
    cpu_loop_exit(cs);
}

void bfin_cpu_do_interrupt(CPUState *cs)
{
    CPUBfinState *env = cpu_env(cs);

    qemu_log_mask(CPU_LOG_INT, "bfin: %s %d at pc=%08x seqstat=%08x "
                  "ipend=%04x\n",
                  cs->exception_index == EXCP_BFIN_EXCEPTION ? "exception"
                                                             : "interrupt",
                  cs->exception_index == EXCP_BFIN_EXCEPTION
                  ? (int)(env->reg[BFIN_SEQSTAT] & 0x3f) : env->irq_ivg,
                  env->pc, env->reg[BFIN_SEQSTAT], env->ipend);

    switch (cs->exception_index) {
    case EXCP_BFIN_EXCEPTION:
        bfin_take_event(env, 3, env->pc);
        break;
    case EXCP_BFIN_IRQ:
        bfin_take_event(env, env->irq_ivg, env->pc);
        break;
    default:
        cpu_abort(cs, "bfin: unhandled exception %d\n", cs->exception_index);
    }
    cs->exception_index = -1;
}

/* highest priority latched and unmasked interrupt that may preempt */
static int bfin_pending_ivg(CPUBfinState *env)
{
    /*
     * The SIC re-requests an interrupt as long as its source is asserted,
     * so a line held high counts as latched.
     */
    uint32_t req = (env->ilat | env->irq_level) & (env->imask | 0x1f);

    if (env->ipend & (1u << 4)) {
        req &= 0x7;                     /* only EMU, RST, NMI */
    }
    /*
     * An active event, the exception (EVX, IPEND[3]) included, holds off
     * every event of lower priority; IPEND[4] is the global disable
     */
    for (int ivg = 0; ivg < 16; ivg++) {
        if (ivg == 4) {
            continue;
        }
        if (env->ipend & (1u << ivg)) {
            return -1;                  /* a higher or equal one is active */
        }
        if (req & (1u << ivg)) {
            return ivg;
        }
    }
    return -1;
}

bool bfin_cpu_exec_interrupt(CPUState *cs, int interrupt_request)
{
    CPUBfinState *env = cpu_env(cs);
    int ivg;

    if (!(interrupt_request & CPU_INTERRUPT_HARD)) {
        return false;
    }
    ivg = bfin_pending_ivg(env);
    if (ivg < 0) {
        return false;
    }
    env->irq_ivg = ivg;
    cs->exception_index = EXCP_BFIN_IRQ;
    bfin_cpu_do_interrupt(cs);
    return true;
}

void bfin_update_irq(CPUBfinState *env)
{
    CPUState *cs = env_cpu(env);
    /* called from helpers too, which run without the BQL */
    BQL_LOCK_GUARD();

    if (bfin_pending_ivg(env) >= 0) {
        cpu_interrupt(cs, CPU_INTERRUPT_HARD);
    } else {
        cpu_reset_interrupt(cs, CPU_INTERRUPT_HARD);
    }
}

/* return from event @ivg: clear it, leave supervisor mode if it was last */
void helper_rtx(CPUBfinState *env, uint32_t ivg)
{
    bool was_super = bfin_supervisor(env);

    if (ivg == 5) {
        /*
         * RTI clears the highest priority bit set in IPEND (PGR chapter
         * 4); that includes the reset event, which code leaves with RTI
         * (listing 3-1).  IPEND[4] is the global disable, not an event.
         */
        for (ivg = 1; ivg < 16; ivg++) {
            if (ivg != 4 && (env->ipend & (1u << ivg))) {
                break;
            }
        }
        env->ipend &= ~(1u << 4);
    }
    if (ivg < 16) {
        env->ipend &= ~(1u << ivg);
    }
    qemu_log_mask(CPU_LOG_INT, "bfin: return from %u to pc=%08x ipend=%04x\n",
                  ivg, env->pc, env->ipend);
    bfin_switch_mode(env, was_super);
    bfin_update_irq(env);
}

uint32_t helper_get_astat(CPUBfinState *env)
{
    return bfin_get_astat(env);
}

void helper_set_astat(CPUBfinState *env, uint32_t val)
{
    bfin_set_astat(env, val);
}

/* CYCLES counts core clocks; CYCLES2 is its upper half */
static uint64_t bfin_cycles(CPUBfinState *env)
{
    int64_t ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - env->cycles_base;

    return muldiv64(ns, env->clock_freq, NANOSECONDS_PER_SECOND);
}

/*
 * A read of CYCLES latches CYCLES2 into a shadow, which reads of CYCLES2
 * return, so the two halves read in that order belong together (PRM
 * "CYCLES and CYCLES2 Registers")
 */
uint32_t helper_get_cycles(CPUBfinState *env, uint32_t high)
{
    uint64_t c;

    if (high) {
        return env->cycles2_shadow;
    }
    c = bfin_cycles(env);
    env->cycles2_shadow = c >> 32;
    return c;
}

void helper_set_cycles(CPUBfinState *env, uint32_t high, uint32_t val)
{
    uint64_t c = bfin_cycles(env);

    c = high ? deposit64(c, 32, 32, val) : deposit64(c, 0, 32, val);
    env->cycles_base = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) -
                       muldiv64(c, NANOSECONDS_PER_SECOND, env->clock_freq);
}

/*
 * DIVS / DIVQ, the steps of a nonrestoring division (chapter 15): the
 * divisor is the low half of the second register, the dividend is
 * shifted left one bit per step and collects the quotient bits.
 */
uint32_t helper_div(CPUBfinState *env, uint32_t dividend, uint32_t divisor,
                    uint32_t divs)
{
    uint32_t aq;

    divisor &= 0xffff;
    if (!divs) {
        uint16_t hi = dividend >> 16;

        hi = env->astat[ASTAT_AQ] ? hi + divisor : hi - divisor;
        dividend = deposit32(dividend, 16, 16, hi);
    }
    aq = (dividend >> 31) ^ (divisor >> 15);
    env->astat[ASTAT_AQ] = aq;
    return (dividend << 1) | (divs ? aq : !aq);
}

/* 40-bit accumulator as a signed value */
static int64_t bfin_acc(CPUBfinState *env, int n)
{
    uint64_t v = ((uint64_t)(env->reg[BFIN_A0X + 2 * n] & 0xff) << 32) |
                 env->reg[BFIN_A0W + 2 * n];

    return sextract64(v, 0, 40);
}

/* CC = A0 == A1, A0 < A1, A0 <= A1; AZ, AN, AC0 from A0 - A1 */
void helper_cc_acc(CPUBfinState *env, uint32_t op)
{
    int64_t a0 = bfin_acc(env, 0), a1 = bfin_acc(env, 1);
    uint64_t ua0 = a0 & MAKE_64BIT_MASK(0, 40);
    uint64_t ua1 = a1 & MAKE_64BIT_MASK(0, 40);

    env->astat[ASTAT_AZ] = a0 == a1;
    env->astat[ASTAT_AN] = a0 < a1;
    env->astat[ASTAT_AC0] = env->astat[ASTAT_AC0_COPY] = ua0 >= ua1;
    env->astat[ASTAT_CC] = op == 0 ? a0 == a1 : op == 1 ? a0 < a1 : a0 <= a1;
}

/* Ireg + M with circular buffer wraparound, chapter 5 */
uint32_t helper_dag_add(uint32_t i, uint32_t m, uint32_t b, uint32_t l)
{
    /*
     * the PRM's comparisons are on the mathematical sum: 0 - 10 lies
     * below a base of 0 (ADI's cir1 test), so do them without wrapping
     */
    int64_t n = (int64_t)i + (int32_t)m;

    if (l == 0) {
        return n;
    }
    if ((int32_t)m >= 0) {
        if (n >= (int64_t)b + l) {
            n -= l;
        }
    } else if (n < (int64_t)b) {
        n += l;
    }
    return n;
}

/* bit reversed carry adder: the carry runs from bit 31 towards bit 0 */
uint32_t helper_dag_brev(uint32_t i, uint32_t m)
{
    return revbit32(revbit32(i) + revbit32(m));
}

/* pushing RETI enables the interrupt system, popping it disables it */
void helper_reti_saved(CPUBfinState *env, uint32_t saved)
{
    if (saved) {
        env->ipend &= ~(1u << 4);
    } else {
        env->ipend |= 1u << 4;
    }
    bfin_update_irq(env);
}

G_NORETURN void helper_idle(CPUBfinState *env)
{
    CPUState *cs = env_cpu(env);

    cs->halted = 1;
    cs->exception_index = EXCP_HLT;
    cpu_loop_exit(cs);
}

/* IMASK bits 0..4 (EMU, RST, NMI, EVX, global disable) cannot be masked */
#define IMASK_FIXED 0x1fu

uint32_t helper_cli(CPUBfinState *env)
{
    uint32_t old = env->imask;

    env->imask = IMASK_FIXED;
    return old;
}

void helper_sti(CPUBfinState *env, uint32_t val)
{
    env->imask = (val & 0xffff) | IMASK_FIXED;
    bfin_update_irq(env);
}

void helper_raise(CPUBfinState *env, uint32_t ivg)
{
    env->ilat |= 1u << ivg;
    bfin_update_irq(env);
}

/*
 * No MMU: addresses are physical.  CPLB protection is not modelled; the
 * MMR space is supervisor only.
 */
bool bfin_cpu_tlb_fill(CPUState *cs, vaddr address, int size,
                       MMUAccessType access_type, int mmu_idx,
                       bool probe, uintptr_t retaddr)
{
    vaddr page = address & TARGET_PAGE_MASK;

    tlb_set_page(cs, page, page, PAGE_READ | PAGE_WRITE | PAGE_EXEC,
                 mmu_idx, TARGET_PAGE_SIZE);
    return true;
}

/*
 * A load or store that is not aligned to its size raises the data
 * misaligned exception (EXCAUSE 0x24), a fetch from an odd address the
 * instruction one (0x2a); the address goes to the fault register Linux
 * reports as si_addr, and RETX points at the faulting instruction.
 */
G_NORETURN void bfin_cpu_do_unaligned_access(CPUState *cs, vaddr addr,
                                             MMUAccessType access_type,
                                             int mmu_idx, uintptr_t retaddr)
{
    CPUBfinState *env = cpu_env(cs);
    uint32_t excause;

    cpu_restore_state(cs, retaddr);
    if (access_type == MMU_INST_FETCH) {
        env->icplb_fault_addr = addr;
        excause = 0x2a;
    } else {
        env->dcplb_fault_addr = addr;
        excause = 0x24;
    }
    env->reg[BFIN_SEQSTAT] = deposit32(env->reg[BFIN_SEQSTAT], 0, 6, excause);
    cs->exception_index = EXCP_BFIN_EXCEPTION;
    cpu_loop_exit(cs);
}

hwaddr bfin_cpu_get_phys_page_debug(CPUState *cs, vaddr addr)
{
    return addr;
}
