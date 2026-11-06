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

/* arch/tile/include/uapi/asm/sigcontext.h: a copy of pt_regs */
struct target_sigcontext {
    abi_ulong gregs[56];
    abi_ulong pc;
    abi_ulong ics;          /* EX_CONTEXT_0_1: PL and ICS */
    abi_ulong faultnum;
    abi_ulong orig_r0;
    abi_ulong flags;
    abi_ulong cmpexch;
    abi_ulong pad[2];
};

struct target_ucontext {
    abi_ulong tuc_flags;
    abi_ulong tuc_link;
    target_stack_t tuc_stack;
    struct target_sigcontext tuc_mcontext;
    target_sigset_t tuc_sigmask;   /* mask last for extensibility */
};

struct target_rt_sigframe {
    unsigned char save_area[16];   /* caller save area */
    struct target_siginfo info;
    struct target_ucontext uc;
};

QEMU_BUILD_BUG_ON(sizeof(struct target_sigcontext) !=
                  sizeof(struct target_pt_regs));

static void setup_sigcontext(struct target_sigcontext *sc, CPUTLGState *env)
{
    int i;

    for (i = 0; i < TILEGX_R_COUNT; ++i) {
        __put_user(env->regs[i], &sc->gregs[i]);
    }
    __put_user(env->pc, &sc->pc);
    __put_user(env->ics << 2, &sc->ics);
    __put_user(0, &sc->faultnum);
    __put_user(0, &sc->orig_r0);
    __put_user(0, &sc->flags);
    __put_user(env->cmpexch_value, &sc->cmpexch);
}

static void restore_sigcontext(CPUTLGState *env, struct target_sigcontext *sc)
{
    abi_ulong ex1;
    int i;

    for (i = 0; i < TILEGX_R_COUNT; ++i) {
        __get_user(env->regs[i], &sc->gregs[i]);
    }
    __get_user(env->pc, &sc->pc);
    __get_user(ex1, &sc->ics);
    env->ics = (ex1 >> 2) & 1;
    __get_user(env->cmpexch_value, &sc->cmpexch);
}

static abi_ulong get_sigframe(struct target_sigaction *ka, CPUTLGState *env,
                              size_t frame_size)
{
    abi_ulong sp = get_sp_from_cpustate(env);

    if (on_sig_stack(sp) && !likely(on_sig_stack(sp - frame_size))) {
        return -1UL;
    }

    sp = target_sigsp(sp, ka) - frame_size;
    sp &= -16UL;
    return sp;
}

void setup_rt_frame(int sig, struct target_sigaction *ka,
                    target_siginfo_t *info,
                    target_sigset_t *set, CPUTLGState *env)
{
    abi_ulong frame_addr;
    struct target_rt_sigframe *frame;
    abi_ulong restorer;
    int i;

    frame_addr = get_sigframe(ka, env, sizeof(*frame));
    trace_user_setup_rt_frame(env, frame_addr);
    if (!lock_user_struct(VERIFY_WRITE, frame, frame_addr, 0)) {
        goto give_sigsegv;
    }

    /* Always write at least the signal number for the stack backtracer. */
    if (ka->sa_flags & TARGET_SA_SIGINFO) {
        frame->info = *info;
    } else {
        __put_user(info->si_signo, &frame->info.si_signo);
    }

    memset(frame->save_area, 0, sizeof(frame->save_area));
    __put_user(0, &frame->uc.tuc_flags);
    __put_user(0, &frame->uc.tuc_link);
    target_save_altstack(&frame->uc.tuc_stack, env);
    setup_sigcontext(&frame->uc.tuc_mcontext, env);
    for (i = 0; i < TARGET_NSIG_WORDS; ++i) {
        __put_user(set->sig[i], &frame->uc.tuc_sigmask.sig[i]);
    }

    if (ka->sa_flags & TARGET_SA_RESTORER) {
        restorer = ka->sa_restorer;
    } else {
        restorer = default_rt_sigreturn;
    }

    /* The kernel enters the handler with ICS set. */
    env->pc = ka->_sa_handler;
    env->ics = 1;
    env->regs[TILEGX_R_SP] = frame_addr;
    env->regs[TILEGX_R_LR] = restorer;
    env->regs[0] = sig;
    env->regs[1] = frame_addr + offsetof(struct target_rt_sigframe, info);
    env->regs[2] = frame_addr + offsetof(struct target_rt_sigframe, uc);

    unlock_user_struct(frame, frame_addr, 1);
    return;

give_sigsegv:
    force_sigsegv(sig);
}

long do_rt_sigreturn(CPUTLGState *env)
{
    abi_ulong frame_addr = env->regs[TILEGX_R_SP];
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

void setup_sigtramp(abi_ulong sigtramp_page)
{
    uint64_t *tramp = lock_user(VERIFY_WRITE, sigtramp_page, 16, 0);
    assert(tramp != NULL);

    /* { moveli r10, __NR_rt_sigreturn } { swint1 } */
    __put_user(0x00045fe551483000ULL, tramp + 0);
    __put_user(0x286b180051485000ULL, tramp + 1);
    QEMU_BUILD_BUG_ON(TARGET_NR_rt_sigreturn != 139);

    default_rt_sigreturn = sigtramp_page;
    unlock_user(tramp, sigtramp_page, 16);
}
