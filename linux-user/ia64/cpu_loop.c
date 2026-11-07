/*
 * qemu user cpu loop
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu.h"
#include "user-internals.h"
#include "user/cpu_loop.h"
#include "signal-common.h"
#include "user-mmap.h"
#include "target/ia64/cpu.h"

#define BREAK_SYSCALL   0x100000

/* The address of the current instruction as the program sees it. */
static abi_ulong ia64_user_ip(CPUIA64State *env)
{
    return ia64_user_unfold(env->ip) + env->ri;
}

/* arch/ia64/kernel/traps.c, ia64_bad_break() */
static void ia64_bad_break(CPUIA64State *env, uint32_t imm)
{
    int sig, code;

    switch (imm) {
    case 0:
        sig = TARGET_SIGILL;
        code = TARGET_ILL_ILLOPC;
        break;
    case 1:
        sig = TARGET_SIGFPE;
        code = TARGET_FPE_INTDIV;
        break;
    case 2:
        sig = TARGET_SIGFPE;
        code = TARGET_FPE_INTOVF;
        break;
    case 3:
        sig = TARGET_SIGFPE;
        code = TARGET_FPE_FLTSUB;
        break;
    case 4:
        sig = TARGET_SIGSEGV;
        code = TARGET_SEGV_MAPERR;
        break;
    case 5:
        sig = TARGET_SIGSEGV;
        code = TARGET_BUS_ADRALN;
        break;
    default:
        if (imm < 0x80000) {
            sig = TARGET_SIGILL;
            code = TARGET_ILL_ILLTRP;
        } else {
            sig = TARGET_SIGTRAP;
            code = TARGET_TRAP_BRKPT;
        }
        break;
    }
    force_sig_fault(sig, code, ia64_user_ip(env));
}

static void ia64_syscall(CPUIA64State *env)
{
    uint64_t ip = env->ip;
    uint32_t ri = env->ri;
    abi_long ret;

    /* the kernel returns behind the break */
    if (env->ri < 2) {
        env->ri++;
    } else {
        env->ri = 0;
        env->ip += 16;
    }
    ret = do_syscall(env, env->gr[15],
                     *ia64_out_reg(env, 0), *ia64_out_reg(env, 1),
                     *ia64_out_reg(env, 2), *ia64_out_reg(env, 3),
                     *ia64_out_reg(env, 4), *ia64_out_reg(env, 5),
                     *ia64_out_reg(env, 6), *ia64_out_reg(env, 7));
    if (ret == -QEMU_ERESTARTSYS) {
        env->ip = ip;
        env->ri = ri;
    } else if (ret != -QEMU_ESIGRETURN && ret != -QEMU_ESETPC) {
        /* r10 flags an error, r8 then holds the positive errno */
        if ((abi_ulong)ret >= (abi_ulong)-4095) {
            env->gr[8] = -ret;
            env->gr[10] = -1;
        } else {
            env->gr[8] = ret;
            env->gr[10] = 0;
        }
        env->nat[8] = 0;
        env->nat[10] = 0;
    }
}

void cpu_loop(CPUIA64State *env)
{
    CPUState *cs = env_cpu(env);
    int trapnr, code;

    while (1) {
        cpu_exec_start(cs);
        trapnr = cpu_exec(cs);
        cpu_exec_end(cs);
        qemu_process_cpu_events(cs);

        switch (trapnr) {
        case EXCP_INTERRUPT:
            /* just indicate that signals should be handled asap */
            break;
        case EXCP_ATOMIC:
            cpu_exec_step_atomic(cs);
            break;
        case EXCP_DEBUG:
            force_sig_fault(TARGET_SIGTRAP, TARGET_TRAP_BRKPT,
                            ia64_user_ip(env));
            break;
        case EXCP_IA64(IVT_BREAK):
            if (env->pend_iim == BREAK_SYSCALL) {
                ia64_syscall(env);
            } else {
                ia64_bad_break(env, env->pend_iim);
            }
            break;
        case EXCP_IA64(IVT_GENERAL):
            code = FIELD_EX64(env->pend_isr, ISR, CODE) & 0xf0;
            force_sig_fault(TARGET_SIGILL,
                            code == GE_PRIV_OP ? TARGET_ILL_PRVOPC :
                            code == GE_PRIV_REG ? TARGET_ILL_PRVREG :
                            code == GE_RESERVED_REG ? TARGET_ILL_ILLOPN :
                            TARGET_ILL_ILLOPC, ia64_user_ip(env));
            break;
        case EXCP_IA64(IVT_NAT_CONSUMPTION):
            if ((FIELD_EX64(env->pend_isr, ISR, CODE) & 0xf0) == 0x20) {
                force_sig_fault(TARGET_SIGSEGV, TARGET_SEGV_ACCERR,
                                env->pend_ifa);
            } else {
                force_sig_fault(TARGET_SIGILL, TARGET_ILL_ILLOPN,
                                ia64_user_ip(env));
            }
            break;
        case EXCP_IA64(IVT_UNALIGNED):
            force_sig_fault(TARGET_SIGBUS, TARGET_BUS_ADRALN, env->pend_ifa);
            break;
        case EXCP_IA64(IVT_FP_FAULT):
        case EXCP_IA64(IVT_FP_TRAP):
            force_sig_fault(TARGET_SIGFPE, TARGET_FPE_FLTINV,
                            ia64_user_ip(env));
            break;
        case EXCP_IA64(IVT_UNSUPPORTED_DATA):
            force_sig_fault(TARGET_SIGILL, TARGET_ILL_ILLOPN,
                            ia64_user_ip(env));
            break;
        default:
            fprintf(stderr, "Unhandled trap: 0x%x\n", trapnr);
            cpu_dump_state(cs, stderr, 0);
            exit(EXIT_FAILURE);
        }
        process_pending_signals(env);
    }
}

void init_main_thread(CPUState *cs, struct image_info *info)
{
    CPUArchState *env = cpu_env(cs);
    abi_ulong rbs_size = 16 * 1024 * 1024;
    abi_long rbs;

    /* arch/ia64/include/asm/processor.h, start_thread() */
    env->psr = PSR_IC | PSR_I | PSR_DT | PSR_IT | PSR_RT | PSR_BN |
               (3ull << PSR_CPL_SHIFT);
    env->ip = info->entry & ~0xfull;
    env->ri = 0;
    env->gr[12] = info->start_stack - 16;
    env->gr[8] = 0;
    env->cfm = 0;
    env->ar[AR_RSC] = 0xf;

    /* the register backing store grows up, away from the memory stack */
    rbs = target_mmap(0, rbs_size, PROT_READ | PROT_WRITE,
                      MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (rbs == -1) {
        fprintf(stderr, "qemu: cannot allocate the register stack\n");
        exit(EXIT_FAILURE);
    }
    env->ar[AR_BSPSTORE] = rbs;
    env->ar[AR_RNAT] = 0;
    env->rse_ndirty = 0;
}
