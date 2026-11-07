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

/* arch/avr32/include/uapi/asm/sigcontext.h */
struct target_sigcontext {
    abi_ulong oldmask;
    abi_ulong sr;
    abi_ulong pc;
    abi_ulong lr;
    abi_ulong sp;
    abi_ulong r12, r11, r10, r9, r8, r7, r6, r5, r4, r3, r2, r1, r0;
};

struct target_ucontext {
    abi_ulong tuc_flags;
    abi_ulong tuc_link;
    target_stack_t tuc_stack;
    struct target_sigcontext tuc_mcontext;
    target_sigset_t tuc_sigmask;
};

/* arch/avr32/kernel/signal.c */
struct target_rt_sigframe {
    struct target_siginfo info;
    struct target_ucontext uc;
    abi_ulong retcode;
};

/* The user-visible part of SR: the flags, without the mode bits */
#define AVR32_SR_USER_MASK  0x0000ffffu

static void setup_sigcontext(struct target_sigcontext *sc,
                             CPUAVR32State *env)
{
    /* No stcond across a signal handler */
    uint32_t sr = avr32_get_sr(env) & ~(1u << SR_L);

    __put_user(sr, &sc->sr);
    __put_user(env->r[AVR32_PC_REG], &sc->pc);
    __put_user(env->r[AVR32_LR_REG], &sc->lr);
    __put_user(env->r[AVR32_SP_REG], &sc->sp);
    __put_user(env->r[12], &sc->r12);
    __put_user(env->r[11], &sc->r11);
    __put_user(env->r[10], &sc->r10);
    __put_user(env->r[9], &sc->r9);
    __put_user(env->r[8], &sc->r8);
    __put_user(env->r[7], &sc->r7);
    __put_user(env->r[6], &sc->r6);
    __put_user(env->r[5], &sc->r5);
    __put_user(env->r[4], &sc->r4);
    __put_user(env->r[3], &sc->r3);
    __put_user(env->r[2], &sc->r2);
    __put_user(env->r[1], &sc->r1);
    __put_user(env->r[0], &sc->r0);
}

static void restore_sigcontext(CPUAVR32State *env,
                               struct target_sigcontext *sc)
{
    abi_ulong sr;

    __get_user(sr, &sc->sr);
    avr32_set_sr(env, sr & AVR32_SR_USER_MASK);
    __get_user(env->r[AVR32_PC_REG], &sc->pc);
    __get_user(env->r[AVR32_LR_REG], &sc->lr);
    __get_user(env->r[AVR32_SP_REG], &sc->sp);
    __get_user(env->r[12], &sc->r12);
    __get_user(env->r[11], &sc->r11);
    __get_user(env->r[10], &sc->r10);
    __get_user(env->r[9], &sc->r9);
    __get_user(env->r[8], &sc->r8);
    __get_user(env->r[7], &sc->r7);
    __get_user(env->r[6], &sc->r6);
    __get_user(env->r[5], &sc->r5);
    __get_user(env->r[4], &sc->r4);
    __get_user(env->r[3], &sc->r3);
    __get_user(env->r[2], &sc->r2);
    __get_user(env->r[1], &sc->r1);
    __get_user(env->r[0], &sc->r0);
}

static abi_ulong get_sigframe(struct target_sigaction *ka,
                              CPUAVR32State *env, size_t frame_size)
{
    abi_ulong sp = target_sigsp(get_sp_from_cpustate(env), ka);

    return (sp - frame_size) & ~3;
}

void setup_rt_frame(int sig, struct target_sigaction *ka,
                    target_siginfo_t *info,
                    target_sigset_t *set, CPUAVR32State *env)
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

    /* mov r8, __NR_rt_sigreturn; scall */
    __put_user(0x3008d733 | (TARGET_NR_rt_sigreturn << 20), &frame->retcode);

    frame->info = *info;
    __put_user(0, &frame->uc.tuc_flags);
    __put_user(0, &frame->uc.tuc_link);
    target_save_altstack(&frame->uc.tuc_stack, env);
    setup_sigcontext(&frame->uc.tuc_mcontext, env);
    for (i = 0; i < TARGET_NSIG_WORDS; i++) {
        __put_user(set->sig[i], &frame->uc.tuc_sigmask.sig[i]);
    }
    unlock_user_struct(frame, frame_addr, 1);

    env->r[12] = sig;
    env->r[11] = frame_addr + offsetof(struct target_rt_sigframe, info);
    env->r[10] = frame_addr + offsetof(struct target_rt_sigframe, uc);
    env->r[AVR32_SP_REG] = frame_addr;
    if (ka->sa_flags & TARGET_SA_RESTORER) {
        env->r[AVR32_LR_REG] = ka->sa_restorer;
    } else {
        env->r[AVR32_LR_REG] = default_rt_sigreturn;
    }
    env->r[AVR32_PC_REG] = ka->_sa_handler & ~1;
    env->sflags[SR_L] = 0;
}

long do_rt_sigreturn(CPUAVR32State *env)
{
    abi_ulong frame_addr = env->r[AVR32_SP_REG];
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
    uint32_t *tramp = lock_user(VERIFY_WRITE, sigtramp_page, 4, 0);
    assert(tramp != NULL);

    /* mov r8, __NR_rt_sigreturn; scall */
    __put_user(0x3008d733 | (TARGET_NR_rt_sigreturn << 20), tramp);

    default_rt_sigreturn = sigtramp_page;
    unlock_user(tramp, sigtramp_page, 4);
}
