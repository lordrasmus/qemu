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

void cpu_loop(CPUV850State *env)
{
    CPUState *cs = env_cpu(env);
    uint32_t *r = env->gpr;
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
        case EXCP_TRAP:
            /*
             * pc is already past the trap.  trap 0: number in r12,
             * arguments in r6..r9; trap 1 adds r13 and r14.
             */
            switch (env->excp_code - 0x40) {
            case 0:
            case 1:
                if (r[12] == TARGET_NR_clone) {
                    /* the kernel only looks at flags and the child sp */
                    ret = do_syscall(env, TARGET_NR_clone, r[6], r[7],
                                     0, 0, 0, 0, 0, 0);
                } else if (env->excp_code == 0x40) {
                    ret = do_syscall(env, r[12], r[6], r[7], r[8], r[9],
                                     0, 0, 0, 0);
                } else {
                    ret = do_syscall(env, r[12], r[6], r[7], r[8], r[9],
                                     r[13], r[14], 0, 0);
                }
                break;
            default:
                /* bad_trap() */
                ret = -TARGET_ENOSYS;
                break;
            }
            if (ret == -QEMU_ERESTARTSYS) {
                env->pc -= 4;
            } else if (ret != -QEMU_ESIGRETURN && ret != -QEMU_ESETPC) {
                r[10] = ret;
            }
            break;
        case EXCP_ILGOP:
            force_sig_fault(TARGET_SIGILL, TARGET_ILL_ILLOPC, env->pc);
            break;
        case EXCP_DBTRAP:
        case EXCP_DEBUG:
            force_sig_fault(TARGET_SIGTRAP, TARGET_TRAP_BRKPT, env->pc);
            break;
        case EXCP_HLT:
            /* HALT: nothing to wait for, carry on */
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
    env->gpr[R_SP] = info->start_stack;
}
