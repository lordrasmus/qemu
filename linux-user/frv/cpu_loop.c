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

/* tira gr0,#120..126: the kernel's user space atomics (arch/frv/kernel/traps.c) */
enum {
    TRAP_CMPXCHG32 = 120,
    TRAP_XCHG32,
    TRAP_XOR,
    TRAP_OR,
    TRAP_AND,
    TRAP_SUB,
    TRAP_ADD,
};

static void do_atomic_trap(CPUState *cs, CPUFRVState *env, int op)
{
    abi_ulong addr = env->gr[8];
    uint32_t x = env->gr[9], y, z, old;
    uint32_t *p;

    if ((addr & 3) || !access_ok(cs, VERIFY_WRITE, addr, 4)) {
        force_sig_fault(TARGET_SIGSEGV, TARGET_SEGV_MAPERR, addr);
        return;
    }
    p = g2h(cs, addr);

    switch (op) {
    case TRAP_CMPXCHG32:
        y = env->gr[10];
        z = tswap32(qatomic_cmpxchg(p, tswap32(x), tswap32(y)));
        break;
    case TRAP_XCHG32:
        y = x;
        z = tswap32(qatomic_xchg(p, tswap32(y)));
        break;
    default:
        z = tswap32(qatomic_read(p));
        for (;;) {
            switch (op) {
            case TRAP_XOR:
                y = z ^ x;
                break;
            case TRAP_OR:
                y = z | x;
                break;
            case TRAP_AND:
                y = z & x;
                break;
            case TRAP_SUB:
                y = z - x;
                break;
            default:
                y = z + x;
                break;
            }
            old = tswap32(qatomic_cmpxchg(p, tswap32(z), tswap32(y)));
            if (old == z) {
                break;
            }
            z = old;
        }
        break;
    }
    env->gr[5] = z;
    env->gr[9] = y;
}

void cpu_loop(CPUFRVState *env)
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
        case TT_TRAP:
            /* tira gr0,#0: number in gr7, arguments in gr8-gr13 */
            env->pc = env->excp_next;
            ret = do_syscall(env, env->gr[7],
                             env->gr[8], env->gr[9], env->gr[10],
                             env->gr[11], env->gr[12], env->gr[13],
                             0, 0);
            if (ret == -QEMU_ERESTARTSYS) {
                env->pc -= 4;
            } else if (ret != -QEMU_ESIGRETURN && ret != -QEMU_ESETPC) {
                env->gr[8] = ret;
            }
            break;
        case TT_TRAP + TRAP_CMPXCHG32 ... TT_TRAP + TRAP_ADD:
            env->pc = env->excp_next;
            do_atomic_trap(cs, env, trapnr - TT_TRAP);
            break;
        case TT_TRAP + 1:
        case TT_BREAK:
            /* gdb uses tira gr0,#1 as its breakpoint */
            force_sig_fault(TARGET_SIGTRAP, TARGET_TRAP_BRKPT, env->pc);
            break;
        case TT_TRAP + 2 ... TT_TRAP + TRAP_CMPXCHG32 - 1:
            force_sig_fault(TARGET_SIGILL, TARGET_ILL_ILLTRP, env->pc);
            break;
        case TT_ILLEGAL_INSN:
        case TT_FP_DISABLED:
        case TT_MP_DISABLED:
            force_sig_fault(TARGET_SIGILL, TARGET_ILL_ILLOPC, env->pc);
            break;
        case TT_PRIV_INSN:
            force_sig_fault(TARGET_SIGILL, TARGET_ILL_PRVOPC, env->pc);
            break;
        case TT_DATA_ACCESS_ERROR:
            force_sig_fault(TARGET_SIGSEGV, TARGET_SEGV_ACCERR,
                            env->excp_addr);
            break;
        case TT_DIVISION:
            /* the kernel returns past the packet that divided */
            env->pc = env->excp_next;
            force_sig_fault(TARGET_SIGFPE, TARGET_FPE_INTDIV, env->pc);
            break;
        case EXCP_INTERRUPT:
            /* just indicate that signals should be handled asap */
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
        process_pending_signals(env);
    }
}

void init_main_thread(CPUState *cs, struct image_info *info)
{
    CPUArchState *env = cpu_env(cs);

    /* user mode, traps enabled */
    env->spr[SPR_PSR] &= ~R_PSR_S_MASK;
    env->spr[SPR_PSR] |= R_PSR_ET_MASK;
    env->gr[1] = info->start_stack;
    env->pc = info->entry;

    /* ELF_FDPIC_PLAT_INIT: load maps in gr16/gr17, dynamic section in gr18 */
    if (info_is_fdpic(info)) {
        env->gr[16] = info->loadmap_addr;
        env->gr[17] = info->interpreter_loadmap_addr;
        env->gr[18] = info->interpreter_pt_dynamic_addr
                      ? info->interpreter_pt_dynamic_addr
                      : info->pt_dynamic_addr;
    }
}
