/*
 * NEC V850E1 interrupts and exceptions
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/cputlb.h"
#include "exec/page-protection.h"
#include "exec/log.h"
#include "qemu/log.h"

void v850_update_irq(CPUV850State *env)
{
    CPUState *cs = env_cpu(env);

    if (env->irq_pending) {
        cpu_interrupt(cs, CPU_INTERRUPT_HARD);
    } else {
        cpu_reset_interrupt(cs, CPU_INTERRUPT_HARD);
    }
}

bool v850_cpu_exec_interrupt(CPUState *cs, int interrupt_request)
{
    CPUV850State *env = cpu_env(cs);

    if ((interrupt_request & CPU_INTERRUPT_HARD) && env->irq_pending &&
        !(env->psw & (PSW_ID | PSW_NP))) {
        cs->exception_index = EXCP_IRQ;
        v850_cpu_do_interrupt(cs);
        return true;
    }
    return false;
}

void v850_cpu_do_interrupt(CPUState *cs)
{
    CPUV850State *env = cpu_env(cs);
    uint32_t psw = v850_psw_read(env);
    uint32_t code;
    int irq;

    switch (cs->exception_index) {
    case EXCP_IRQ:
        /* the lowest number has the highest priority */
        irq = ctz32(env->irq_pending);
        env->irq_pending &= ~(1u << irq);
        v850_update_irq(env);
        code = 0x80 + irq * 0x10;
        env->sr[SR_EIPC] = env->pc;
        env->sr[SR_EIPSW] = psw;
        env->sr[SR_ECR] = (env->sr[SR_ECR] & 0xffff0000) | code;
        env->psw = (env->psw | PSW_ID) & ~PSW_EP;
        env->pc = code;
        break;
    case EXCP_TRAP:
        /* pc is already past the TRAP */
        code = env->excp_code;
        env->sr[SR_EIPC] = env->pc;
        env->sr[SR_EIPSW] = psw;
        env->sr[SR_ECR] = (env->sr[SR_ECR] & 0xffff0000) | code;
        env->psw |= PSW_EP | PSW_ID;
        env->pc = code & 0xf0;
        break;
    case EXCP_ILGOP:
    case EXCP_DBTRAP:
        env->sr[SR_DBPC] = env->pc;
        env->sr[SR_DBPSW] = psw;
        env->psw |= PSW_NP | PSW_EP | PSW_ID;
        env->pc = 0x60;
        break;
    default:
        cpu_abort(cs, "unhandled exception %d\n", cs->exception_index);
    }
    qemu_log_mask(CPU_LOG_INT, "v850: exception %d -> pc %08x\n",
                  cs->exception_index, env->pc);
    cs->exception_index = -1;
}

/* No MMU: the debug address is the physical one, page offset included */
hwaddr v850_cpu_get_phys_page_debug(CPUState *cs, vaddr addr)
{
    return addr;
}

/* No MMU, no protection */
bool v850_cpu_tlb_fill(CPUState *cs, vaddr address, int size,
                       MMUAccessType access_type, int mmu_idx,
                       bool probe, uintptr_t retaddr)
{
    address &= TARGET_PAGE_MASK;
    tlb_set_page(cs, address, address, PAGE_RWX, mmu_idx, TARGET_PAGE_SIZE);
    return true;
}
