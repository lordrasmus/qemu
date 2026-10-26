/*
 * FR-V interrupts, exceptions and special purpose registers
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "cpu.h"
#include "exec/cputlb.h"
#include "exec/target_page.h"
#include "accel/tcg/cpu-loop.h"
#include "accel/tcg/cpu-ldst.h"

void frv_update_irq(CPUFRVState *env)
{
    CPUState *cs = env_cpu(env);

    if (env->irq_level) {
        cpu_interrupt(cs, CPU_INTERRUPT_HARD);
    } else {
        cpu_reset_interrupt(cs, CPU_INTERRUPT_HARD);
    }
}

static bool irq_pending(CPUFRVState *env)
{
    uint32_t psr = env->spr[SPR_PSR];
    int level = env->irq_level;

    if (!level || !FIELD_EX32(psr, PSR, ET) || env->debug_mode) {
        return false;
    }
    return level == 15 || level > FIELD_EX32(psr, PSR, PIL);
}

bool frv_cpu_exec_interrupt(CPUState *cs, int interrupt_request)
{
    CPUFRVState *env = cpu_env(cs);

    if ((interrupt_request & CPU_INTERRUPT_HARD) && irq_pending(env)) {
        cs->exception_index = EXCP_IRQ;
        frv_cpu_do_interrupt(cs);
        return true;
    }
    return false;
}

/* Enter the handler TBR points to for trap type tt, returning to ret */
static void enter_trap(CPUFRVState *env, int tt, uint32_t ret)
{
    uint32_t psr = env->spr[SPR_PSR];

    if (!FIELD_EX32(psr, PSR, ET) && tt < TT_INTERRUPT) {
        qemu_log_mask(LOG_GUEST_ERROR, "frv: trap 0x%02x at %08x with "
                      "PSR.ET clear, the CPU would stop here\n", tt, env->pc);
    }
    psr = FIELD_DP32(psr, PSR, PS, FIELD_EX32(psr, PSR, S));
    psr = FIELD_DP32(psr, PSR, ET, 0);
    psr = FIELD_DP32(psr, PSR, S, 1);
    env->spr[SPR_PSR] = psr;
    env->spr[SPR_PCSR] = ret;
    env->spr[SPR_TBR] = FIELD_DP32(env->spr[SPR_TBR], TBR, TT, tt);
    env->pc = env->spr[SPR_TBR];
}

static void clear_exception_status(CPUFRVState *env)
{
    env->spr[SPR_ESFR0] = 0;
    env->spr[SPR_ESFR1] = 0;
    for (int i = 0; i < 16; i++) {
        env->spr[SPR_ESR0 + i] &= ~ESR_VALID;
    }
}

/* ESR.EC values as Linux names them */
static uint32_t esr_ec(int tt)
{
    switch (tt) {
    case TT_PRIV_INSN:
        return 0x08;
    case TT_ILLEGAL_INSN:
        return 0x0a;
    case TT_DATA_ACCESS_ERROR:
        return 0x20;
    case TT_DIVISION:
        return 0x26;
    default:
        return 0;
    }
}

void frv_cpu_do_interrupt(CPUState *cs)
{
    CPUFRVState *env = cpu_env(cs);
    int tt = cs->exception_index;

    if (tt == EXCP_IRQ) {
        int level = env->irq_level;

        qemu_log_mask(CPU_LOG_INT, "frv: interrupt %d at %08x\n", level,
                      env->pc);
        enter_trap(env, TT_INTERRUPT + level, env->pc);
        cs->exception_index = -1;
        return;
    }

    if (tt >= TT_TRAP && tt < TT_BREAK) {
        qemu_log_mask(CPU_LOG_INT, "frv: trap 0x%02x at %08x, gr7 %d "
                      "gr8-12 %08x %08x %08x %08x %08x\n", tt, env->pc,
                      env->gr[7], env->gr[8], env->gr[9], env->gr[10],
                      env->gr[11], env->gr[12]);
    } else {
        qemu_log_mask(CPU_LOG_INT, "frv: trap 0x%02x at %08x, next %08x, "
                      "addr %08x\n", tt, env->pc, env->excp_next,
                      env->excp_addr);
    }

    switch (tt) {
    case TT_BREAK: {
        uint32_t psr = env->spr[SPR_PSR];
        uint32_t bpsr = 0;

        if (FIELD_EX32(psr, PSR, S)) {
            bpsr |= BPSR_BS;
        }
        if (FIELD_EX32(psr, PSR, ET)) {
            bpsr |= BPSR_BET;
        }
        env->spr[SPR_BPSR] = bpsr;
        env->spr[SPR_BPCSR] = env->excp_next;
        env->spr[SPR_BRR] |= BRR_SB;
        psr = FIELD_DP32(psr, PSR, S, 1);
        psr = FIELD_DP32(psr, PSR, ET, 0);
        env->spr[SPR_PSR] = psr;
        env->spr[SPR_TBR] = FIELD_DP32(env->spr[SPR_TBR], TBR, TT, TT_BREAK);
        env->pc = env->spr[SPR_TBR];
        env->debug_mode = true;
        break;
    }
    case TT_PRIV_INSN:
    case TT_ILLEGAL_INSN:
    case TT_FP_DISABLED:
    case TT_MP_DISABLED:
        /* precise: PCSR is the packet that faulted */
        clear_exception_status(env);
        env->spr[SPR_ESR0] = ESR_VALID | esr_ec(tt);
        env->spr[SPR_EPCR0] = env->pc | EPCR_V;
        env->spr[SPR_ESFR1] |= ESFR1_ESR0;
        enter_trap(env, tt, env->pc);
        break;
    case TT_DATA_ACCESS_ERROR:
        clear_exception_status(env);
        env->spr[SPR_ESR15] = ESR_VALID | ESR_EAV | esr_ec(tt);
        env->spr[SPR_EAR15] = env->excp_addr;
        env->spr[SPR_ESFR1] |= ESFR1_ESR15;
        enter_trap(env, tt, env->excp_next);
        break;
    case TT_DIVISION:
        clear_exception_status(env);
        env->spr[SPR_ISR] |= env->excp_dtt;
        env->spr[SPR_ESR0] = ESR_VALID | esr_ec(tt);
        env->spr[SPR_EPCR0] = env->pc | EPCR_V;
        env->spr[SPR_ESFR1] |= ESFR1_ESR0;
        enter_trap(env, tt, env->excp_next);
        break;
    default:
        /* software traps, TT_TRAP + n */
        enter_trap(env, tt, env->excp_next);
        break;
    }
    cs->exception_index = -1;
}

uint32_t frv_spr_read(CPUFRVState *env, int spr)
{
    uint32_t v = 0;

    switch (spr) {
    case SPR_CCR:
        for (int i = 0; i < 4; i++) {
            v |= (env->icc[i] & 0xf) << (16 + 4 * i);
            v |= (env->fcc[i] & 0xf) << (4 * i);
        }
        return v;
    case SPR_CCCR:
        for (int i = 0; i < 8; i++) {
            v |= (env->cc[i] & 3) << (2 * i);
        }
        return v;
    default:
        return env->spr[spr];
    }
}

void frv_spr_write(CPUFRVState *env, int spr, uint32_t val)
{
    switch (spr) {
    case SPR_PSR:
        /* IMPLE and VER are read only */
        env->spr[SPR_PSR] = (env->spr[SPR_PSR] & 0xff000000) |
                            (val & 0x00ffffff);
        break;
    case SPR_TBR:
        env->spr[SPR_TBR] = FIELD_DP32(env->spr[SPR_TBR], TBR, TBA,
                                       FIELD_EX32(val, TBR, TBA));
        break;
    case SPR_CCR:
        for (int i = 0; i < 4; i++) {
            env->icc[i] = (val >> (16 + 4 * i)) & 0xf;
            env->fcc[i] = (val >> (4 * i)) & 0xf;
        }
        break;
    case SPR_CCCR:
        for (int i = 0; i < 8; i++) {
            env->cc[i] = (val >> (2 * i)) & 3;
        }
        break;
    default:
        if (spr >= SPR_ACCG0 && spr < SPR_ACCG0 + 64) {
            val &= 0xff;
        }
        env->spr[spr] = val;
        break;
    }
}

hwaddr frv_cpu_get_phys_page_debug(CPUState *cs, vaddr addr)
{
    return addr;
}

/* No MMU: virtual and physical addresses are the same */
bool frv_cpu_tlb_fill(CPUState *cs, vaddr address, int size,
                      MMUAccessType access_type, int mmu_idx,
                      bool probe, uintptr_t retaddr)
{
    address &= TARGET_PAGE_MASK;
    tlb_set_page(cs, address, address, PAGE_READ | PAGE_WRITE | PAGE_EXEC,
                 mmu_idx, TARGET_PAGE_SIZE);
    return true;
}

/* The FR400 reports a misaligned access as data_access_error */
G_NORETURN void frv_cpu_do_unaligned_access(CPUState *cs, vaddr addr,
                                 MMUAccessType access_type, int mmu_idx,
                                 uintptr_t retaddr)
{
    CPUFRVState *env = cpu_env(cs);

    env->excp_addr = addr;
    frv_raise(env, TT_DATA_ACCESS_ERROR, retaddr);
}
