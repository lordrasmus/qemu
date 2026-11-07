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

void cpu_loop(CPULM32State *env)
{
    CPUState *cs = env_cpu(env);
    int trapnr;
    abi_long ret;

    while (1) {
        cpu_exec_start(cs);
        trapnr = cpu_exec(cs);
        cpu_exec_end(cs);
        qemu_process_cpu_events(cs);

        switch (trapnr) {
        case EXCP_INTERRUPT:
            /* just indicate that signals should be handled asap */
            break;
        case EXCP_SYSTEMCALL:
            /* scall leaves pc on itself, the kernel returns to ea + 4 */
            env->pc += 4;
            if (env->regs[R_R8] == TARGET_NR_clone) {
                /* sys_clone(fn, newsp, flags, arg) */
                ret = do_syscall(env, TARGET_NR_clone,
                                 env->regs[R_R3], env->regs[R_R2],
                                 0, 0, 0, 0, 0, 0);
            } else {
                ret = do_syscall(env,
                                 env->regs[R_R8],
                                 env->regs[R_R1],
                                 env->regs[R_R2],
                                 env->regs[R_R3],
                                 env->regs[R_R4],
                                 env->regs[R_R5],
                                 env->regs[R_R6],
                                 0, 0);
            }
            if (ret == -QEMU_ERESTARTSYS) {
                env->pc -= 4;
            } else if (ret != -QEMU_ESIGRETURN && ret != -QEMU_ESETPC) {
                env->regs[R_R1] = ret;
            }
            break;
        case EXCP_DIVIDE_BY_ZERO:
            force_sig_fault(TARGET_SIGFPE, TARGET_FPE_INTDIV, env->pc);
            break;
        case EXCP_INSN_BUS_ERROR:
            /* raised for undefined opcodes by helper_ill */
            force_sig_fault(TARGET_SIGILL, TARGET_ILL_ILLOPC, env->pc);
            break;
        case EXCP_BREAKPOINT:
        case EXCP_DEBUG:
            force_sig_fault(TARGET_SIGTRAP, TARGET_TRAP_BRKPT, env->pc);
            break;
        case EXCP_HLT:
            /* "and r0, r0, r0" halts the core; pc is already past it */
            cs->halted = 0;
            break;
        case EXCP_ATOMIC:
            cpu_exec_step_atomic(cs);
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

    env->pc = info->entry;
    env->regs[R_SP] = info->start_stack;
    /* start_thread() hands the data segment start over in fp */
    env->regs[R_FP] = info->start_data;
}
