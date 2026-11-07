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

void cpu_loop(CPUNDS32State *env)
{
    CPUState *cs = env_cpu(env);
    int trapnr;
    uint32_t etype;
    abi_long ret;

    for (;;) {
        cpu_exec_start(cs);
        trapnr = cpu_exec(cs);
        cpu_exec_end(cs);
        qemu_process_cpu_events(cs);

        switch (trapnr) {
        case VEC_SYSCALL:
            /* Number in $ta (r15), arguments in r0..r5, result in r0. */
            env->pc += 4;
            ret = do_syscall(env, env->gpr[R_TA],
                             env->gpr[0], env->gpr[1], env->gpr[2],
                             env->gpr[3], env->gpr[4], env->gpr[5],
                             0, 0);
            if (ret == -QEMU_ERESTARTSYS) {
                env->pc -= 4;
            } else if (ret != -QEMU_ESIGRETURN && ret != -QEMU_ESETPC) {
                env->gpr[0] = ret;
            }
            break;
        case VEC_GENERAL:
            etype = FIELD_EX32(env->pending_itype, ITYPE, ETYPE);
            switch (etype) {
            case ETYPE_ALIGN:
                force_sig_fault(TARGET_SIGBUS, TARGET_BUS_ADRALN,
                                env->pending_eva);
                break;
            case ETYPE_TRAP:
                force_sig_fault(TARGET_SIGTRAP, TARGET_TRAP_BRKPT, env->pc);
                break;
            case ETYPE_ARITH:
                force_sig_fault(TARGET_SIGFPE, TARGET_FPE_INTDIV, env->pc);
                break;
            default:
                force_sig_fault(TARGET_SIGILL, TARGET_ILL_ILLOPC, env->pc);
                break;
            }
            break;
        case VEC_DEBUG:
            force_sig_fault(TARGET_SIGTRAP, TARGET_TRAP_BRKPT, env->pc);
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

    /* User mode, translation on, as the kernel's start_thread leaves it. */
    env->sr[SR_PSW] = R_PSW_IT_MASK | R_PSW_DT_MASK;
    /* Linux enables the divide-by-zero trap (INT_MASK_INITAIAL_VAL). */
    env->sr[SR_INT_MASK] |= INT_MASK_IDIVZE;
    env->pc = info->entry;
    env->gpr[R_SP] = info->start_stack;
}
