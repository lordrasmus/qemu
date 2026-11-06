/*
 *  Emulation of Linux signals
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
#include "signal-common.h"
#include "linux-user/trace.h"
#include "target_ptrace.h"

/* arch/arc/include/uapi/asm/sigcontext.h */
struct target_sigcontext {
    struct target_user_regs_struct regs;
    struct target_user_regs_arcv2 v2abi;
};

struct target_ucontext {
    abi_ulong                uc_flags;
    abi_ulong                uc_link;
    target_stack_t           uc_stack;
    struct target_sigcontext uc_mcontext;
    target_sigset_t          uc_sigmask;
};

/* arch/arc/kernel/signal.c */
struct target_rt_sigframe {
    struct target_siginfo info;
    struct target_ucontext uc;
#define MAGIC_SIGALTSTK    0x07302004
    uint32_t sigret_magic;
};

static bool arc_is_arcv2(CPUARCState *env)
{
    return !(env_archcpu(env)->family & ARC_OPCODE_ARCV1);
}

static void setup_sigcontext(struct target_sigcontext *sc, CPUARCState *env)
{
    __put_user(env->bta, &sc->regs.scratch.bta);
    __put_user(env->lps, &sc->regs.scratch.lp_start);
    __put_user(env->lpe, &sc->regs.scratch.lp_end);
    __put_user(CPU_LP(env), &sc->regs.scratch.lp_count);
    __put_user(pack_status32(&env->stat), &sc->regs.scratch.status32);
    __put_user(env->pc, &sc->regs.scratch.ret);
    __put_user(CPU_BLINK(env), &sc->regs.scratch.blink);
    __put_user(CPU_FP(env), &sc->regs.scratch.fp);
    __put_user(CPU_GP(env), &sc->regs.scratch.gp);
    __put_user(env->r[12], &sc->regs.scratch.r12);
    __put_user(env->r[11], &sc->regs.scratch.r11);
    __put_user(env->r[10], &sc->regs.scratch.r10);
    __put_user(env->r[9], &sc->regs.scratch.r9);
    __put_user(env->r[8], &sc->regs.scratch.r8);
    __put_user(env->r[7], &sc->regs.scratch.r7);
    __put_user(env->r[6], &sc->regs.scratch.r6);
    __put_user(env->r[5], &sc->regs.scratch.r5);
    __put_user(env->r[4], &sc->regs.scratch.r4);
    __put_user(env->r[3], &sc->regs.scratch.r3);
    __put_user(env->r[2], &sc->regs.scratch.r2);
    __put_user(env->r[1], &sc->regs.scratch.r1);
    __put_user(env->r[0], &sc->regs.scratch.r0);
    __put_user(CPU_SP(env), &sc->regs.scratch.sp);

    if (arc_is_arcv2(env)) {
        __put_user(env->r[30], &sc->v2abi.r30);
        __put_user(env->r[58], &sc->v2abi.r58);
        __put_user(env->r[59], &sc->v2abi.r59);
    }
}

static void restore_sigcontext(CPUARCState *env, struct target_sigcontext *sc)
{
    abi_ulong v;

    __get_user(env->bta, &sc->regs.scratch.bta);
    __get_user(env->lps, &sc->regs.scratch.lp_start);
    __get_user(env->lpe, &sc->regs.scratch.lp_end);
    __get_user(CPU_LP(env), &sc->regs.scratch.lp_count);
    __get_user(v, &sc->regs.scratch.status32);
    unpack_status32(&env->stat, v);
    /* sigreturn always returns to user mode */
    SET_STATUS_BIT(env->stat, Uf, 1);
    __get_user(v, &sc->regs.scratch.ret);
    env->pc = v;
    CPU_PCL(env) = v & ~(target_ulong)3;
    __get_user(CPU_BLINK(env), &sc->regs.scratch.blink);
    __get_user(CPU_FP(env), &sc->regs.scratch.fp);
    __get_user(CPU_GP(env), &sc->regs.scratch.gp);
    __get_user(env->r[12], &sc->regs.scratch.r12);
    __get_user(env->r[11], &sc->regs.scratch.r11);
    __get_user(env->r[10], &sc->regs.scratch.r10);
    __get_user(env->r[9], &sc->regs.scratch.r9);
    __get_user(env->r[8], &sc->regs.scratch.r8);
    __get_user(env->r[7], &sc->regs.scratch.r7);
    __get_user(env->r[6], &sc->regs.scratch.r6);
    __get_user(env->r[5], &sc->regs.scratch.r5);
    __get_user(env->r[4], &sc->regs.scratch.r4);
    __get_user(env->r[3], &sc->regs.scratch.r3);
    __get_user(env->r[2], &sc->regs.scratch.r2);
    __get_user(env->r[1], &sc->regs.scratch.r1);
    __get_user(env->r[0], &sc->regs.scratch.r0);
    __get_user(CPU_SP(env), &sc->regs.scratch.sp);

    if (arc_is_arcv2(env)) {
        __get_user(env->r[30], &sc->v2abi.r30);
        __get_user(env->r[58], &sc->v2abi.r58);
        __get_user(env->r[59], &sc->v2abi.r59);
    }
}

void setup_rt_frame(int sig, struct target_sigaction *ka,
                    target_siginfo_t *info,
                    target_sigset_t *set, CPUARCState *env)
{
    struct target_rt_sigframe *sf;
    abi_ulong sf_addr;
    uint32_t magic = 0;
    int i;

    /* ATPCS B01 mandates 8-byte alignment */
    sf_addr = (target_sigsp(CPU_SP(env), ka) - sizeof(*sf)) & ~7;
    trace_user_setup_rt_frame(env, sf_addr);
    if (!lock_user_struct(VERIFY_WRITE, sf, sf_addr, 0)) {
        goto badframe;
    }

    setup_sigcontext(&sf->uc.uc_mcontext, env);
    for (i = 0; i < TARGET_NSIG_WORDS; i++) {
        __put_user(set->sig[i], &sf->uc.uc_sigmask.sig[i]);
    }

    if (ka->sa_flags & TARGET_SA_SIGINFO) {
        sf->info = *info;
        __put_user(0, &sf->uc.uc_flags);
        __put_user(0, &sf->uc.uc_link);
        target_save_altstack(&sf->uc.uc_stack, env);
        env->r[1] = sf_addr + offsetof(struct target_rt_sigframe, info);
        env->r[2] = sf_addr + offsetof(struct target_rt_sigframe, uc);
        magic = MAGIC_SIGALTSTK;
    }
    __put_user(magic, &sf->sigret_magic);
    unlock_user_struct(sf, sf_addr, 1);

    /* The kernel kills the task if libc gave no restorer. */
    if (!(ka->sa_flags & TARGET_SA_RESTORER)) {
        force_sigsegv(sig);
        return;
    }

    env->r[0] = sig;
    env->pc = ka->_sa_handler;
    CPU_PCL(env) = env->pc & ~(target_ulong)3;
    CPU_BLINK(env) = ka->sa_restorer;
    CPU_SP(env) = sf_addr;
    env->stat.pstate &= ~(STATUS32_DE | BRANCH_DELAY);
    SET_STATUS_BIT(env->stat, Lf, 1);
    return;

badframe:
    force_sigsegv(sig);
}

long do_rt_sigreturn(CPUARCState *env)
{
    struct target_rt_sigframe *sf;
    abi_ulong sf_addr = CPU_SP(env);
    target_sigset_t target_set;
    sigset_t set;
    uint32_t magic;
    int i;

    trace_user_do_rt_sigreturn(env, sf_addr);
    if (sf_addr & 3) {
        goto badframe;
    }
    if (!lock_user_struct(VERIFY_READ, sf, sf_addr, 1)) {
        goto badframe;
    }

    for (i = 0; i < TARGET_NSIG_WORDS; i++) {
        __get_user(target_set.sig[i], &sf->uc.uc_sigmask.sig[i]);
    }
    target_to_host_sigset_internal(&set, &target_set);
    set_sigmask(&set);

    __get_user(magic, &sf->sigret_magic);
    if (magic == MAGIC_SIGALTSTK) {
        target_restore_altstack(&sf->uc.uc_stack, env);
    }
    restore_sigcontext(env, &sf->uc.uc_mcontext);

    unlock_user_struct(sf, sf_addr, 0);
    return -QEMU_ESIGRETURN;

badframe:
    force_sig(TARGET_SIGSEGV);
    return -QEMU_ESIGRETURN;
}
