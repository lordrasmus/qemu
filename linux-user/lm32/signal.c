/*
 *  Emulation of Linux signals
 *
 *  Copyright (c) 2003 Fabrice Bellard
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

/* arch/lm32/kernel/signal.c, asm-generic ucontext */

struct target_sigcontext {
    struct target_pt_regs regs;
    abi_ulong oldmask;
};

struct target_ucontext {
    abi_ulong tuc_flags;
    abi_ulong tuc_link;
    target_stack_t tuc_stack;
    struct target_sigcontext tuc_mcontext;
    target_sigset_t tuc_sigmask;
};

struct target_rt_sigframe {
    struct target_siginfo info;
    struct target_ucontext uc;
    uint32_t tramp[2];
};

static void setup_sigcontext(struct target_sigcontext *sc,
                             CPULM32State *env, abi_ulong mask)
{
    struct target_pt_regs *regs = &sc->regs;
    int i;

    for (i = 0; i < 26; i++) {
        __put_user(env->regs[i], &regs->r[i]);
    }
    __put_user(env->regs[R_GP], &regs->gp);
    __put_user(env->regs[R_FP], &regs->fp);
    __put_user(env->regs[R_SP], &regs->sp);
    __put_user(env->regs[R_RA], &regs->ra);
    /* the kernel keeps ea on the scall and adds 4 on the way out */
    __put_user(env->pc - 4, &regs->ea);
    __put_user(env->regs[R_BA], &regs->ba);
    __put_user(0, &regs->pt_mode);
    __put_user(mask, &sc->oldmask);
}

static void restore_sigcontext(CPULM32State *env,
                               struct target_sigcontext *sc)
{
    struct target_pt_regs *regs = &sc->regs;
    abi_ulong ea;
    int i;

    for (i = 1; i < 26; i++) {
        __get_user(env->regs[i], &regs->r[i]);
    }
    __get_user(env->regs[R_GP], &regs->gp);
    __get_user(env->regs[R_FP], &regs->fp);
    __get_user(env->regs[R_SP], &regs->sp);
    __get_user(env->regs[R_RA], &regs->ra);
    __get_user(ea, &regs->ea);
    __get_user(env->regs[R_BA], &regs->ba);
    env->pc = ea + 4;
}

static abi_ulong get_sigframe(struct target_sigaction *ka,
                              CPULM32State *env, size_t frame_size)
{
    abi_ulong sp = target_sigsp(get_sp_from_cpustate(env), ka);

    return (sp - frame_size) & -8ul;
}

void setup_rt_frame(int sig, struct target_sigaction *ka,
                    target_siginfo_t *info,
                    target_sigset_t *set, CPULM32State *env)
{
    struct target_rt_sigframe *frame;
    abi_ulong frame_addr;
    int i;

    frame_addr = get_sigframe(ka, env, sizeof(*frame));
    trace_user_setup_rt_frame(env, frame_addr);
    if (!lock_user_struct(VERIFY_WRITE, frame, frame_addr, 0)) {
        force_sigsegv(sig);
        return;
    }

    frame->info = *info;

    __put_user(0, &frame->uc.tuc_flags);
    __put_user(0, &frame->uc.tuc_link);
    target_save_altstack(&frame->uc.tuc_stack, env);
    setup_sigcontext(&frame->uc.tuc_mcontext, env, set->sig[0]);
    for (i = 0; i < TARGET_NSIG_WORDS; i++) {
        __put_user(set->sig[i], &frame->uc.tuc_sigmask.sig[i]);
    }
    /*
     * mvi r8, __NR_rt_sigreturn; scall -- on the stack like the kernel,
     * bFLT programs get no sigtramp page.
     */
    __put_user(0x34080000 | TARGET_NR_rt_sigreturn, &frame->tramp[0]);
    __put_user(0xac000007, &frame->tramp[1]);

    env->regs[R_RA] = frame_addr + offsetof(struct target_rt_sigframe, tramp);
    env->regs[R_SP] = frame_addr - 4;
    env->regs[R_R1] = sig;
    env->regs[R_R2] = frame_addr + offsetof(struct target_rt_sigframe, info);
    env->regs[R_R3] = frame_addr + offsetof(struct target_rt_sigframe, uc);
    env->pc = ka->_sa_handler;

    unlock_user_struct(frame, frame_addr, 1);
}

long do_rt_sigreturn(CPULM32State *env)
{
    /* the handler returns with sp back at frame - 4 */
    abi_ulong frame_addr = env->regs[R_SP] + 4;
    struct target_rt_sigframe *frame;
    sigset_t set;

    trace_user_do_rt_sigreturn(env, frame_addr);
    if (!lock_user_struct(VERIFY_READ, frame, frame_addr, 1)) {
        goto badframe;
    }

    target_to_host_sigset(&set, &frame->uc.tuc_sigmask);
    set_sigmask(&set);

    restore_sigcontext(env, &frame->uc.tuc_mcontext);
    target_restore_altstack(&frame->uc.tuc_stack, env);

    unlock_user_struct(frame, frame_addr, 0);
    return -QEMU_ESIGRETURN;

badframe:
    unlock_user_struct(frame, frame_addr, 0);
    force_sig(TARGET_SIGSEGV);
    return -QEMU_ESIGRETURN;
}
