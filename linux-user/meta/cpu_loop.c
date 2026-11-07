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
#include "user/page-protection.h"
#include "exec/page-protection.h"
#include "exec/mmap-lock.h"
#include "signal-common.h"
#include "target_elf.h"

/* SWITCH #__METAG_SW_SYS, arch/metag/include/asm/switch.h */
#define META_SWITCH_SYSCALL     0xaf440001u

#define D0(i)   dregs[0][i]
#define D1(i)   dregs[1][i]

/*
 * __kuser_cmpxchg: D1Ar1 = oldval, D0Ar2 = newval, D1Ar3 = ptr;
 * D0Re0 = 0 on success.  Only D0Re0 is clobbered.
 */
static void meta_gateway_cmpxchg(CPUMETAState *env)
{
    uint32_t addr = env->D1(2);
    uint32_t oldval = tswap32(env->D1(3));
    uint32_t newval = tswap32(env->D0(3));
    int need = PAGE_READ | PAGE_WRITE_ORG | PAGE_VALID;
    int flags;
    uint32_t *host;

    if (addr & 3) {
        force_sig_fault(TARGET_SIGBUS, TARGET_BUS_ADRALN, addr);
        return;
    }
    mmap_lock();
    flags = page_get_flags(addr);
    if ((flags & need) != need) {
        mmap_unlock();
        force_sig_fault(TARGET_SIGSEGV, flags & PAGE_VALID ?
                        TARGET_SEGV_ACCERR : TARGET_SEGV_MAPERR, addr);
        return;
    }
    host = g2h(env_cpu(env), addr);
    env->D0(0) = qatomic_cmpxchg__nocheck(host, oldval, newval) != oldval;
    mmap_unlock();
}

void cpu_loop(CPUMETAState *env)
{
    CPUState *cs = env_cpu(env);
    int trapnr, signum;
    uint32_t pc;
    abi_long ret;

    for (;;) {
        cpu_exec_start(cs);
        trapnr = cpu_exec(cs);
        cpu_exec_end(cs);
        qemu_process_cpu_events(cs);

        pc = env->pc[META_PC];
        switch (trapnr) {
        case EXCP_FAULT_BASE ... EXCP_FAULT_END:
            signum = trapnr - EXCP_FAULT_BASE;
            switch (signum) {
            case _META_SIGNUM(NOERROR, SWITCH):
                if (pc == META_GATEWAY_GET_TLS) {
                    env->D0(0) = env->tls_value;
                    env->pc[META_PC] = env->D1(4);      /* D1RtP */
                } else if (pc == META_GATEWAY_CMPXCHG) {
                    meta_gateway_cmpxchg(env);
                    env->pc[META_PC] = env->D1(4);
                } else if (env->switch_code == META_SWITCH_SYSCALL) {
                    env->pc[META_PC] = env->switch_nextpc;
                    ret = do_syscall(env,
                                     env->D1(0),    /* D1Re0: number */
                                     env->D1(3),    /* D1Ar1 */
                                     env->D0(3),    /* D0Ar2 */
                                     env->D1(2),    /* D1Ar3 */
                                     env->D0(2),    /* D0Ar4 */
                                     env->D1(1),    /* D1Ar5 */
                                     env->D0(1),    /* D0Ar6 */
                                     0, 0);
                    if (ret == -QEMU_ERESTARTSYS) {
                        env->pc[META_PC] = pc;
                    } else if (ret != -QEMU_ESIGRETURN &&
                               ret != -QEMU_ESETPC) {
                        env->D0(0) = ret;           /* D0Re0 */
                    }
                } else {
                    /* breakpoints and unknown SWITCHes, as switch1_handler */
                    force_sig(TARGET_SIGTRAP);
                }
                break;
            case META_SIGNUM_IIF:
                force_sig_fault(TARGET_SIGILL, TARGET_ILL_ILLOPC, pc);
                break;
            case META_SIGNUM_PFG:
                force_sig_fault(TARGET_SIGSEGV, TARGET_SEGV_ACCERR, pc);
                break;
            case META_SIGNUM_DHF:
                force_sig_fault(TARGET_SIGBUS, TARGET_BUS_ADRALN, pc);
                break;
            default:
                force_sig_fault(TARGET_SIGSEGV, TARGET_SEGV_MAPERR, pc);
                break;
            }
            break;
        case EXCP_FPU:
            {
                int fpe = env->cregs[META_TXDEFR] >> META_TXDEFR_TRIGSTAT_SHIFT;
                int code = 0;

                if (fpe & META_DEFR_FPU_INVALID_MASK) {
                    code = TARGET_FPE_FLTINV;
                } else if (fpe & META_DEFR_FPU_DIVZERO_MASK) {
                    code = TARGET_FPE_FLTDIV;
                } else if (fpe & META_DEFR_FPU_OVERFLOW_MASK) {
                    code = TARGET_FPE_FLTOVF;
                } else if (fpe & META_DEFR_FPU_UNDERFLOW_MASK) {
                    code = TARGET_FPE_FLTUND;
                } else if (fpe & META_DEFR_FPU_INEXACT_MASK) {
                    code = TARGET_FPE_FLTRES;
                }
                force_sig_fault(TARGET_SIGFPE, code, pc);
            }
            break;
        case EXCP_INTERRUPT:
            /* just indicate that signals should be handled asap */
            break;
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

/* arch/metag/include/asm/processor.h:start_thread(), ELF_PLAT_INIT */
void init_main_thread(CPUState *cs, struct image_info *info)
{
    CPUArchState *env = cpu_env(cs);

    env->cregs[META_TXENABLE] = META_TXENABLE_ENABLED;
    env->block = META_BLOCK_NONE;
    /* TXPRIVEXT as arch/metag/kernel/setup.c sets it (PRIV_BITS) */
    env->cregs[META_TXPRIVEXT] = 0xff0f7f10;

    env->pc[META_PC] = info->entry;
    env->aregs[0][0] = info->start_stack;   /* A0StP */
    env->D1(3) = info->argc;                /* D1Ar1 */
    env->D0(3) = info->argv;                /* D0Ar2 */
    env->D1(2) = info->envp;                /* D1Ar3 */
    env->D0(2) = 0;                         /* D0Ar4: rtld_fini */
}
