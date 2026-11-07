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

/* arch/v850/kernel/signal.c of Linux 2.6.26 */

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

struct target_sigframe {
    struct target_sigcontext sc;
    abi_ulong extramask[TARGET_NSIG_WORDS - 1];
    uint32_t tramp[2];
};

struct target_rt_sigframe {
    struct target_siginfo info;
    struct target_ucontext uc;
    uint32_t tramp[2];
};

static void setup_sigcontext(struct target_sigcontext *sc,
                             CPUV850State *env, abi_ulong mask)
{
    struct target_pt_regs *regs = &sc->regs;
    int i;

    /* gpr[0] is where the kernel keeps the system call number, 0 here */
    __put_user(0, &regs->gpr[0]);
    for (i = 1; i < 32; i++) {
        __put_user(env->gpr[i], &regs->gpr[i]);
    }
    __put_user(env->pc, &regs->pc);
    __put_user(v850_psw_read(env), &regs->psw);
    __put_user(env->sr[SR_CTPC], &regs->ctpc);
    __put_user(env->sr[SR_CTPSW], &regs->ctpsw);
    __put_user(env->sr[SR_CTBP], &regs->ctbp);
    __put_user(0, &regs->kernel_mode);
    __put_user(mask, &sc->oldmask);
}

static void restore_sigcontext(CPUV850State *env,
                               struct target_sigcontext *sc)
{
    struct target_pt_regs *regs = &sc->regs;
    abi_ulong val;
    int i;

    for (i = 1; i < 32; i++) {
        __get_user(env->gpr[i], &regs->gpr[i]);
    }
    __get_user(val, &regs->pc);
    env->pc = val & ~1;
    __get_user(val, &regs->psw);
    v850_psw_write(env, val);
    __get_user(env->sr[SR_CTPC], &regs->ctpc);
    __get_user(env->sr[SR_CTPSW], &regs->ctpsw);
    __get_user(env->sr[SR_CTBP], &regs->ctbp);
}

static abi_ulong get_sigframe(struct target_sigaction *ka,
                              CPUV850State *env, size_t frame_size)
{
    abi_ulong sp = target_sigsp(get_sp_from_cpustate(env), ka);

    return (sp - frame_size) & -8ul;
}

/*
 * Return through sa_restorer if there is one, else through
 * "addi nr, r0, r12; trap 0" in the frame.
 */
static void setup_return(CPUV850State *env, struct target_sigaction *ka,
                         uint32_t *tramp, abi_ulong tramp_addr, int nr)
{
    if (ka->sa_flags & TARGET_SA_RESTORER) {
        env->gpr[31] = ka->sa_restorer;
    } else {
        __put_user(0x6600 | (nr << 16), &tramp[0]);
        __put_user(0x010007e0, &tramp[1]);
        env->gpr[31] = tramp_addr;
    }
}

void setup_frame(int sig, struct target_sigaction *ka,
                 target_sigset_t *set, CPUV850State *env)
{
    struct target_sigframe *frame;
    abi_ulong frame_addr;
    int i;

    frame_addr = get_sigframe(ka, env, sizeof(*frame));
    trace_user_setup_frame(env, frame_addr);
    if (!lock_user_struct(VERIFY_WRITE, frame, frame_addr, 0)) {
        force_sigsegv(sig);
        return;
    }

    setup_sigcontext(&frame->sc, env, set->sig[0]);
    for (i = 1; i < TARGET_NSIG_WORDS; i++) {
        __put_user(set->sig[i], &frame->extramask[i - 1]);
    }
    setup_return(env, ka, frame->tramp,
                 frame_addr + offsetof(struct target_sigframe, tramp),
                 TARGET_NR_sigreturn);

    env->pc = ka->_sa_handler;
    env->gpr[R_SP] = frame_addr;
    env->gpr[6] = sig;
    env->gpr[7] = frame_addr + offsetof(struct target_sigframe, sc);

    unlock_user_struct(frame, frame_addr, 1);
}

void setup_rt_frame(int sig, struct target_sigaction *ka,
                    target_siginfo_t *info,
                    target_sigset_t *set, CPUV850State *env)
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
    /* the kernel's own trampoline wrongly uses __NR_sigreturn here */
    setup_return(env, ka, frame->tramp,
                 frame_addr + offsetof(struct target_rt_sigframe, tramp),
                 TARGET_NR_rt_sigreturn);

    env->pc = ka->_sa_handler;
    env->gpr[R_SP] = frame_addr;
    env->gpr[6] = sig;
    env->gpr[7] = frame_addr + offsetof(struct target_rt_sigframe, info);
    env->gpr[8] = frame_addr + offsetof(struct target_rt_sigframe, uc);

    unlock_user_struct(frame, frame_addr, 1);
}

long do_sigreturn(CPUV850State *env)
{
    abi_ulong frame_addr = env->gpr[R_SP];
    struct target_sigframe *frame;
    target_sigset_t target_set;
    sigset_t set;
    int i;

    trace_user_do_sigreturn(env, frame_addr);
    if (!lock_user_struct(VERIFY_READ, frame, frame_addr, 1)) {
        goto badframe;
    }

    __get_user(target_set.sig[0], &frame->sc.oldmask);
    for (i = 1; i < TARGET_NSIG_WORDS; i++) {
        __get_user(target_set.sig[i], &frame->extramask[i - 1]);
    }
    target_to_host_sigset_internal(&set, &target_set);
    set_sigmask(&set);

    restore_sigcontext(env, &frame->sc);

    unlock_user_struct(frame, frame_addr, 0);
    return -QEMU_ESIGRETURN;

badframe:
    force_sig(TARGET_SIGSEGV);
    return -QEMU_ESIGRETURN;
}

long do_rt_sigreturn(CPUV850State *env)
{
    abi_ulong frame_addr = env->gpr[R_SP];
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
    force_sig(TARGET_SIGSEGV);
    return -QEMU_ESIGRETURN;
}
