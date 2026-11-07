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

/* TRAPA #n raises vector 8 + n. */
#define EXCP_TRAPA0     8
#define EXCP_TRAPA3     11

/*
 * arch/h8300/kernel/process.c: clone takes a pointer to
 * { flags, newsp, parent_tidptr, child_tidptr }; there is no TLS.
 */
static abi_long do_h8300_clone(CPUH8300State *env, abi_ulong args)
{
    abi_ulong flags, newsp, ptid, ctid;

    if (get_user_ual(flags, args) ||
        get_user_ual(newsp, args + 4) ||
        get_user_ual(ptid, args + 8) ||
        get_user_ual(ctid, args + 12)) {
        return -TARGET_EFAULT;
    }
    return do_syscall(env, TARGET_NR_clone, flags, newsp, ptid, ctid, 0,
                      0, 0, 0);
}

void cpu_loop(CPUH8300State *env)
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
        case EXCP_TRAPA0:
            /* Number in er0, arguments in er1..er6, result in er0. */
            if (env->regs[0] == TARGET_NR_clone) {
                ret = do_h8300_clone(env, env->regs[1]);
            } else {
                ret = do_syscall(env, env->regs[0],
                                 env->regs[1], env->regs[2], env->regs[3],
                                 env->regs[4], env->regs[5], env->regs[6],
                                 0, 0);
            }
            if (ret == -QEMU_ERESTARTSYS) {
                env->pc -= 2;
            } else if (ret != -QEMU_ESIGRETURN && ret != -QEMU_ESETPC) {
                env->regs[0] = ret;
            }
            break;
        case EXCP_TRAPA3:
            force_sig_fault(TARGET_SIGTRAP, TARGET_TRAP_BRKPT, env->pc);
            break;
        case EXCP_TRAPA0 + 1:
        case EXCP_TRAPA0 + 2:
            /* The kernel leaves these vectors empty. */
            force_sig_fault(TARGET_SIGILL, TARGET_ILL_ILLTRP, env->pc);
            break;
        case EXCP_INTERRUPT:
            /* We processed the pending cpu work above.  */
            break;
        case EXCP_DEBUG:
            force_sig_fault(TARGET_SIGTRAP, TARGET_TRAP_BRKPT, env->pc);
            break;
        case EXCP_ATOMIC:
            cpu_exec_step_atomic(cs);
            break;
        default:
            EXCP_DUMP(env, "\nqemu: unhandled CPU exception %#x - aborting\n",
                      trapnr);
            abort();
        }
        process_pending_signals(env);
    }
}

void init_main_thread(CPUState *cs, struct image_info *info)
{
    CPUArchState *env = cpu_env(cs);

    /* start_thread: all CCR flags clear, er5 = GOT base. */
    h8300_cpu_unpack_ccr(env, 0);
    env->regs[5] = info->start_data;
    env->pc = info->entry;
    env->regs[7] = info->start_stack;
}
