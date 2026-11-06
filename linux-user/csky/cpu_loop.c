/*
 *  qemu user cpu loop
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu.h"
#include "user-internals.h"
#include "user/cpu_loop.h"
#include "signal-common.h"

/*
 * trap 2: the kernel's csky_cmpxchg for CPUs without ldex/stex.
 * a0 = old, a1 = new, a2 = address; a0 returns 0 on success, a3 the
 * value found.  Use a host atomic: the futex word is also changed by
 * the host kernel (FUTEX_WAKE_OP), which start_exclusive cannot stop.
 */
static void csky_cmpxchg(CPUCSKYState *env)
{
    CPUState *cs = env_cpu(env);
    abi_ulong addr = env->regs[2];
    uint32_t old = env->regs[0], val;

    if ((addr & 3) || !access_ok(cs, VERIFY_WRITE, addr, 4)) {
        force_sig_fault(TARGET_SIGSEGV, TARGET_SEGV_MAPERR, addr);
        return;
    }
    val = tswap32(qatomic_cmpxchg((uint32_t *)g2h(cs, addr), tswap32(old),
                                  tswap32(env->regs[1])));
    env->regs[0] = val != old;
    env->regs[3] = val;
    env->pc += 4;
}

void cpu_loop(CPUCSKYState *env)
{
    CPUState *cs = env_cpu(env);
    int trapnr;
    abi_long ret;

    for (;;) {
        cpu_exec_start(cs);
        trapnr = cpu_exec(cs);
        cpu_exec_end(cs);
        qemu_process_cpu_events(cs);

        switch (trapnr) {
        case EXCP_CSKY_TRAP0:
            env->pc += 4;
            ret = do_syscall(env, env->regs[7],
                             env->regs[0], env->regs[1], env->regs[2],
                             env->regs[3], env->regs[4], env->regs[5],
                             0, 0);
            if (ret == -QEMU_ERESTARTSYS) {
                env->pc -= 4;
            } else if (ret != -QEMU_ESIGRETURN && ret != -QEMU_ESETPC) {
                env->regs[0] = ret;
            }
            break;
        case EXCP_CSKY_TRAP2:
            csky_cmpxchg(env);
            break;
        case EXCP_CSKY_TRAP3:
            /* the kernel's csky_get_tls */
            env->regs[0] = env->tls_value;
            env->pc += 4;
            break;
        case EXCP_CSKY_TRAP1:
        case EXCP_CSKY_BKPT:
        case EXCP_DEBUG:
            force_sig_fault(TARGET_SIGTRAP, TARGET_TRAP_BRKPT, env->pc);
            break;
        case EXCP_CSKY_TRACE:
            force_sig_fault(TARGET_SIGTRAP, TARGET_TRAP_TRACE, env->pc);
            break;
        case EXCP_CSKY_ALIGN:
            force_sig_fault(TARGET_SIGBUS, TARGET_BUS_ADRALN, env->pc);
            break;
        case EXCP_CSKY_DIV:
            force_sig_fault(TARGET_SIGFPE, TARGET_FPE_INTDIV, env->pc);
            break;
        case EXCP_CSKY_FLOAT:
        case EXCP_CSKY_FP:
            force_sig_fault(TARGET_SIGFPE, 0, env->pc);
            break;
        case EXCP_CSKY_PRIVILEGE:
            force_sig_fault(TARGET_SIGILL, TARGET_ILL_PRVOPC, env->pc);
            break;
        case EXCP_CSKY_UDEF:
        case EXCP_CSKY_SEMIHOST:
            force_sig_fault(TARGET_SIGILL, TARGET_ILL_ILLOPC, env->pc);
            break;
        case EXCP_INTERRUPT:
            /* We processed the pending cpu work above.  */
            break;
        case EXCP_ATOMIC:
            cpu_exec_step_atomic(cs);
            break;
        default:
            fprintf(stderr, "qemu: unhandled CPU exception 0x%x - aborting\n",
                    trapnr);
            cpu_dump_state(cs, stderr, 0);
            abort();
        }
        process_pending_signals(env);
    }
}

void init_main_thread(CPUState *cs, struct image_info *info)
{
    CPUArchState *env = cpu_env(cs);

    env->pc = info->entry;
    env->regs[14] = info->start_stack;
}
