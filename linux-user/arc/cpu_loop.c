/*
 *  qemu user cpu loop
 *
 *  Copyright (c) 2003-2008 Fabrice Bellard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qemu.h"
#include "user-internals.h"
#include "user/cpu_loop.h"
#include "signal-common.h"

static void arc_set_pc(CPUARCState *env, target_ulong pc)
{
    env->pc = pc;
    CPU_PCL(env) = pc & ~(target_ulong)3;
}

/* arch/arc/kernel/process.c: only for cores without LLOCK/SCOND. */
static abi_long do_arc_usr_cmpxchg(CPUARCState *env, abi_ulong addr,
                                   uint32_t expected, uint32_t new)
{
    uint32_t val;

    env->stat.Zf = 0;
    start_exclusive();
    if (get_user_u32(val, addr)) {
        end_exclusive();
        force_sig_fault(TARGET_SIGSEGV, TARGET_SEGV_MAPERR, addr);
        return -TARGET_EFAULT;
    }
    if (val == expected) {
        if (put_user_u32(new, addr)) {
            end_exclusive();
            force_sig_fault(TARGET_SIGSEGV, TARGET_SEGV_ACCERR, addr);
            return -TARGET_EFAULT;
        }
        env->stat.Zf = 1;
    }
    end_exclusive();
    return val;
}

static void do_trap(CPUARCState *env)
{
    target_ulong trap_pc = env->pc;
    target_ulong next_pc = env->eret;
    abi_long ret;

    /* trap_s 1..: breakpoints and debugger hooks, not system calls */
    if (env->param != 0) {
        force_sig_fault(TARGET_SIGTRAP, TARGET_TRAP_BRKPT, trap_pc);
        return;
    }

    /* A trap in a delay slot returns to the branch target (do_interrupt). */
    if (env->stat.pstate & STATUS32_DE) {
        next_pc = env->bta;
        env->stat.pstate &= ~STATUS32_DE;
    }
    arc_set_pc(env, next_pc);

    switch (env->r[8]) {
    case TARGET_NR_arc_settls:
        env->tls_backup = env->r[0];
        ret = 0;
        break;
    case TARGET_NR_arc_gettls:
        ret = env->tls_backup;
        break;
    case TARGET_NR_cacheflush:
        ret = 0;
        break;
    case TARGET_NR_arc_usr_cmpxchg:
        ret = do_arc_usr_cmpxchg(env, env->r[0], env->r[1], env->r[2]);
        break;
    default:
        ret = do_syscall(env, env->r[8],
                         env->r[0], env->r[1], env->r[2], env->r[3],
                         env->r[4], env->r[5], 0, 0);
        break;
    }

    if (ret == -QEMU_ERESTARTSYS) {
        arc_set_pc(env, trap_pc);
    } else if (ret != -QEMU_ESIGRETURN && ret != -QEMU_ESETPC) {
        env->r[0] = ret;
    }
}

void cpu_loop(CPUARCState *env)
{
    CPUState *cs = env_cpu(env);
    int trapnr;

    for (;;) {
        cpu_exec_start(cs);
        trapnr = cpu_exec(cs);
        cpu_exec_end(cs);
        qemu_process_cpu_events(cs);

        switch (trapnr) {
        case EXCP_INTERRUPT:
            /* just indicate that signals should be handled asap */
            break;
        case EXCP_TRAP:
            do_trap(env);
            break;
        case EXCP_LPEND_REACHED:
            arc_set_pc(env, env->param);
            break;
        case EXCP_SWI:
            force_sig_fault(TARGET_SIGTRAP, TARGET_TRAP_BRKPT, env->pc);
            break;
        case EXCP_INST_ERROR:
        case EXCP_EXTENSION:
            force_sig_fault(TARGET_SIGILL, TARGET_ILL_ILLOPC, env->pc);
            break;
        case EXCP_PRIVILEGEV:
            force_sig_fault(TARGET_SIGILL, TARGET_ILL_PRVOPC, env->pc);
            break;
        case EXCP_DIVZERO:
            force_sig_fault(TARGET_SIGFPE, TARGET_FPE_INTDIV, env->pc);
            break;
        case EXCP_MISALIGNED:
            force_sig_fault(TARGET_SIGBUS, TARGET_BUS_ADRALN, env->efa);
            break;
        case EXCP_TLB_MISS_I:
        case EXCP_TLB_MISS_D:
        case EXCP_PROTV:
        case EXCP_MEMORY_ERROR:
            force_sig_fault(TARGET_SIGSEGV, TARGET_SEGV_MAPERR, env->efa);
            break;
        case EXCP_DEBUG:
            force_sig_fault(TARGET_SIGTRAP, TARGET_TRAP_BRKPT, env->pc);
            break;
        case EXCP_ATOMIC:
            cpu_exec_step_atomic(cs);
            break;
        default:
            fprintf(stderr, "qemu: unhandled CPU exception 0x%x - aborting\n",
                    trapnr);
            cpu_dump_state(cs, stderr, 0);
            exit(EXIT_FAILURE);
        }
        process_pending_signals(env);
    }
}

void init_main_thread(CPUState *cs, struct image_info *info)
{
    CPUArchState *env = cpu_env(cs);

    arc_set_pc(env, info->entry);
    CPU_SP(env) = info->start_stack;
    SET_STATUS_BIT(env->stat, Uf, 1);
    /* ELF_PLAT_INIT */
    env->r[0] = 0;
}
