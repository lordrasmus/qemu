/*
 *  qemu user cpu loop
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

void cpu_loop(CPUAVR32State *env)
{
    CPUState *cs = env_cpu(env);
    int trapnr;
    abi_long ret;
    uint32_t pc;

    while (1) {
        cpu_exec_start(cs);
        trapnr = cpu_exec(cs);
        cpu_exec_end(cs);
        qemu_process_cpu_events(cs);

        pc = env->r[AVR32_PC_REG];
        switch (trapnr) {
        case EXCP_INTERRUPT:
            /* just indicate that signals should be handled asap */
            break;
        case EXCP_AVR32_SCALL:
            /*
             * Number in r8, arguments in r12, r11, r10, r9, r5, r3,
             * result in r12 (arch/avr32/kernel/entry-avr32b.S).
             * The PC already points past the scall.
             */
            ret = do_syscall(env, env->r[8],
                             env->r[12], env->r[11], env->r[10],
                             env->r[9], env->r[5], env->r[3],
                             0, 0);
            if (ret == -QEMU_ERESTARTSYS) {
                env->r[AVR32_PC_REG] -= 2;
            } else if (ret != -QEMU_ESIGRETURN && ret != -QEMU_ESETPC) {
                env->r[12] = ret;
            }
            break;
        case EXCP_AVR32_ILLEGAL:
        case EXCP_AVR32_UNIMPLEMENTED:
            force_sig_fault(TARGET_SIGILL, TARGET_ILL_ILLOPC, pc);
            break;
        case EXCP_AVR32_PRIVILEGE:
            force_sig_fault(TARGET_SIGILL, TARGET_ILL_PRVOPC, pc);
            break;
        case EXCP_AVR32_COPROC:
            force_sig_fault(TARGET_SIGILL, TARGET_ILL_COPROC, pc);
            break;
        case EXCP_AVR32_FPE:
            force_sig_fault(TARGET_SIGFPE, TARGET_FPE_FLTINV, pc);
            break;
        case EXCP_AVR32_BREAKPOINT:
        case EXCP_DEBUG:
            force_sig_fault(TARGET_SIGTRAP, TARGET_TRAP_BRKPT, pc);
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

    /* Application mode, everything unmasked, all registers zero */
    avr32_set_sr(env, 0);
    for (int i = 0; i < AVR32_NREGS; i++) {
        env->r[i] = 0;
    }
    env->r[AVR32_SP_REG] = info->start_stack;
    env->r[AVR32_PC_REG] = info->entry & ~1;
}
