/*
 *  qemu user cpu loop, TI C6x
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
#include "exec/translation-block.h"
#include <linux/futex.h>

#define TARGET_NR_cache_sync    244

/*
 * Like the hardware, take signals only with GIE set and no branch in
 * flight; then let what is in flight complete.
 */
static bool c6x_signal_point(CPUC6xState *env)
{
    if (!FIELD_EX32(env->tsr, TSR, GIE)) {
        return false;
    }
    for (int i = 0; i < env->dq_live; i++) {
        for (int j = 0; j < env->dq_n[i]; j++) {
            if (env->dq[i][j].dest == DQ_DEST_PC) {
                return false;
            }
        }
    }
    return true;
}

/*
 * Futex operations that change the futex word in the kernel.  On one CPU
 * they are atomic against the DINT sections of user code; here they must
 * take the DINT lock, or a compare-and-store that read the word before
 * may overwrite them.  Only those that do not block.
 */
static bool c6x_futex_writes(CPUC6xState *env)
{
    abi_ulong nr = env->gpr[REG_B(0)];

    if (nr != TARGET_NR_futex && nr != TARGET_NR_futex_time64) {
        return false;
    }
    switch (env->gpr[REG_B(4)] & FUTEX_CMD_MASK) {
    case FUTEX_WAKE_OP:
    case FUTEX_UNLOCK_PI:
    case FUTEX_TRYLOCK_PI:
        return true;
    }
    return false;
}

static void c6x_signals(CPUC6xState *env)
{
    CPUState *cs = env_cpu(env);
    TaskState *ts = get_task_state(cs);

    if (!qatomic_read(&ts->signal_pending)) {
        return;
    }
    if (!ts->sync_signal.pending && !c6x_signal_point(env)) {
        /* run one execute packet and come back */
        cs->cflags_next_tb = cs->tcg_cflags | CF_NOIRQ | 1;
        qatomic_set(&cs->exit_request, true);
        return;
    }
    c6x_dq_drain(env);
    process_pending_signals(env);
}

void cpu_loop(CPUC6xState *env)
{
    CPUState *cs = env_cpu(env);
    int trapnr, si_code;
    abi_long ret;
    bool locked;

    for (;;) {
        cpu_exec_start(cs);
        trapnr = cpu_exec(cs);
        cpu_exec_end(cs);
        qemu_process_cpu_events(cs);

        switch (trapnr) {
        case EXCP_SWE:
            /* pc is the next execute packet; a branch in flight is lost */
            c6x_dq_drain(env);
            if (env->gpr[REG_B(0)] == TARGET_NR_cache_sync) {
                env->gpr[REG_A(4)] = 0;
                break;
            }
            locked = c6x_futex_writes(env);
            if (locked) {
                c6x_user_lock(env, false);
            }
            ret = do_syscall(env, env->gpr[REG_B(0)],
                             env->gpr[REG_A(4)], env->gpr[REG_B(4)],
                             env->gpr[REG_A(6)], env->gpr[REG_B(6)],
                             env->gpr[REG_A(8)], env->gpr[REG_B(8)],
                             0, 0);
            if (locked) {
                c6x_user_unlock(env);
            }
            if (ret == -QEMU_ERESTARTSYS) {
                env->pc -= 4;
            } else if (ret != -QEMU_ESIGRETURN && ret != -QEMU_ESETPC) {
                env->gpr[REG_A(4)] = ret;
            }
            break;
        case EXCP_INTERNAL:
            c6x_dq_drain(env);
            si_code = env->excp_ierr & IERR_PRX ? TARGET_ILL_PRVOPC
                                                 : TARGET_ILL_ILLOPC;
            force_sig_fault(TARGET_SIGILL, si_code, env->pc);
            break;
        case EXCP_INTERRUPT:
            /* signals are handled below */
            break;
        case EXCP_DEBUG:
            force_sig_fault(TARGET_SIGTRAP, TARGET_TRAP_BRKPT, env->pc);
            break;
        case EXCP_ATOMIC:
            cpu_exec_step_atomic(cs);
            break;
        default:
            fprintf(stderr, "Unhandled trap: 0x%x\n", trapnr);
            cpu_dump_state(cs, stderr, 0);
            exit(EXIT_FAILURE);
        }
        c6x_signals(env);
    }
}

void init_main_thread(CPUState *cs, struct image_info *info)
{
    CPUArchState *env = cpu_env(cs);

    /* user mode, interrupts on (kernel-c6x-user-gie.patch), TSC running */
    env->tsr = FIELD_DP32(env->tsr, TSR, CXM, 1);
    env->tsr = FIELD_DP32(env->tsr, TSR, GIE, 1);
    env->tsc_running = true;
    env->tsc_base = 0;

    /* start_thread: the C6x stack is "empty", argc sits at 8(sp) */
    env->gpr[REG_B(15)] = info->start_stack - 8;
    env->pc = info->entry;

    /* ELF_FDPIC_PLAT_INIT */
    env->gpr[REG_B(4)] = info->loadmap_addr;
    env->gpr[REG_A(6)] = info->interpreter_loadmap_addr;
    env->gpr[REG_B(6)] = info->interpreter_loadmap_addr ?
                         info->interpreter_pt_dynamic_addr :
                         info->pt_dynamic_addr;
}
