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
#include "target_cpu.h"

/* arch/blackfin/include/uapi/asm/sigcontext.h */
struct target_sigcontext {
    abi_ulong sc_r[8];
    abi_ulong sc_p[6];
    abi_ulong sc_usp;
    abi_ulong sc_a0w;
    abi_ulong sc_a1w;
    abi_ulong sc_a0x;
    abi_ulong sc_a1x;
    abi_ulong sc_astat;
    abi_ulong sc_rets;
    abi_ulong sc_pc;
    abi_ulong sc_retx;
    abi_ulong sc_fp;
    abi_ulong sc_i[4];
    abi_ulong sc_m[4];
    abi_ulong sc_l[4];
    abi_ulong sc_b[4];
    abi_ulong sc_lc0;
    abi_ulong sc_lc1;
    abi_ulong sc_lt0;
    abi_ulong sc_lt1;
    abi_ulong sc_lb0;
    abi_ulong sc_lb1;
    abi_ulong sc_seqstat;
};

struct target_ucontext {
    abi_ulong tuc_flags;
    abi_ulong tuc_link;
    target_stack_t tuc_stack;
    struct target_sigcontext tuc_mcontext;
    target_sigset_t tuc_sigmask;
};

/* arch/blackfin/kernel/signal.c */
struct target_rt_sigframe {
    abi_int sig;
    abi_ulong pinfo;
    abi_ulong puc;
    char retcode[8];
    target_siginfo_t info;
    struct target_ucontext uc;
};

static void setup_sigcontext(struct target_sigcontext *sc, CPUBfinState *env)
{
    for (int i = 0; i < 8; i++) {
        __put_user(env->reg[BFIN_R0 + i], &sc->sc_r[i]);
    }
    for (int i = 0; i < 6; i++) {
        __put_user(env->reg[BFIN_P0 + i], &sc->sc_p[i]);
    }
    __put_user(env->reg[BFIN_SP], &sc->sc_usp);
    __put_user(env->reg[BFIN_A0W], &sc->sc_a0w);
    __put_user(env->reg[BFIN_A1W], &sc->sc_a1w);
    __put_user(env->reg[BFIN_A0X], &sc->sc_a0x);
    __put_user(env->reg[BFIN_A1X], &sc->sc_a1x);
    __put_user(bfin_get_astat(env), &sc->sc_astat);
    __put_user(env->reg[BFIN_RETS], &sc->sc_rets);
    __put_user(env->pc, &sc->sc_pc);
    __put_user(env->reg[BFIN_RETX], &sc->sc_retx);
    __put_user(env->reg[BFIN_FP], &sc->sc_fp);
    for (int i = 0; i < 4; i++) {
        __put_user(env->reg[BFIN_I0 + i], &sc->sc_i[i]);
        __put_user(env->reg[BFIN_M0 + i], &sc->sc_m[i]);
        __put_user(env->reg[BFIN_L0 + i], &sc->sc_l[i]);
        __put_user(env->reg[BFIN_B0 + i], &sc->sc_b[i]);
    }
    __put_user(env->reg[BFIN_LC0], &sc->sc_lc0);
    __put_user(env->reg[BFIN_LC1], &sc->sc_lc1);
    __put_user(env->reg[BFIN_LT0], &sc->sc_lt0);
    __put_user(env->reg[BFIN_LT1], &sc->sc_lt1);
    __put_user(env->reg[BFIN_LB0], &sc->sc_lb0);
    __put_user(env->reg[BFIN_LB1], &sc->sc_lb1);
    __put_user(env->reg[BFIN_SEQSTAT], &sc->sc_seqstat);
}

static void restore_sigcontext(struct target_sigcontext *sc,
                               CPUBfinState *env)
{
    uint32_t astat;

    for (int i = 0; i < 8; i++) {
        __get_user(env->reg[BFIN_R0 + i], &sc->sc_r[i]);
    }
    for (int i = 0; i < 6; i++) {
        __get_user(env->reg[BFIN_P0 + i], &sc->sc_p[i]);
    }
    __get_user(env->reg[BFIN_SP], &sc->sc_usp);
    __get_user(env->reg[BFIN_A0W], &sc->sc_a0w);
    __get_user(env->reg[BFIN_A1W], &sc->sc_a1w);
    __get_user(env->reg[BFIN_A0X], &sc->sc_a0x);
    __get_user(env->reg[BFIN_A1X], &sc->sc_a1x);
    env->reg[BFIN_A0X] &= 0xff;
    env->reg[BFIN_A1X] &= 0xff;
    __get_user(astat, &sc->sc_astat);
    bfin_set_astat(env, astat);
    __get_user(env->reg[BFIN_RETS], &sc->sc_rets);
    __get_user(env->pc, &sc->sc_pc);
    __get_user(env->reg[BFIN_RETX], &sc->sc_retx);
    __get_user(env->reg[BFIN_FP], &sc->sc_fp);
    for (int i = 0; i < 4; i++) {
        __get_user(env->reg[BFIN_I0 + i], &sc->sc_i[i]);
        __get_user(env->reg[BFIN_M0 + i], &sc->sc_m[i]);
        __get_user(env->reg[BFIN_L0 + i], &sc->sc_l[i]);
        __get_user(env->reg[BFIN_B0 + i], &sc->sc_b[i]);
    }
    __get_user(env->reg[BFIN_LC0], &sc->sc_lc0);
    __get_user(env->reg[BFIN_LC1], &sc->sc_lc1);
    __get_user(env->reg[BFIN_LT0], &sc->sc_lt0);
    __get_user(env->reg[BFIN_LT1], &sc->sc_lt1);
    __get_user(env->reg[BFIN_LB0], &sc->sc_lb0);
    __get_user(env->reg[BFIN_LB1], &sc->sc_lb1);
    __get_user(env->reg[BFIN_SEQSTAT], &sc->sc_seqstat);
}

static abi_ulong get_sigframe(struct target_sigaction *ka,
                              CPUBfinState *env, int frame_size)
{
    abi_ulong sp = target_sigsp(get_sp_from_cpustate(env), ka);

    return (sp - frame_size) & -8UL;
}

void setup_rt_frame(int sig, struct target_sigaction *ka,
                    target_siginfo_t *info,
                    target_sigset_t *set, CPUBfinState *env)
{
    struct target_rt_sigframe *frame;
    abi_ulong frame_addr;

    frame_addr = get_sigframe(ka, env, sizeof(*frame));
    trace_user_setup_rt_frame(env, frame_addr);

    if (!lock_user_struct(VERIFY_WRITE, frame, frame_addr, 0)) {
        force_sigsegv(sig);
        return;
    }

    __put_user(sig, &frame->sig);
    __put_user(frame_addr + offsetof(struct target_rt_sigframe, info),
               &frame->pinfo);
    __put_user(frame_addr + offsetof(struct target_rt_sigframe, uc),
               &frame->puc);
    frame->info = *info;

    __put_user(0, &frame->uc.tuc_flags);
    __put_user(0, &frame->uc.tuc_link);
    target_save_altstack(&frame->uc.tuc_stack, env);
    setup_sigcontext(&frame->uc.tuc_mcontext, env);
    for (int i = 0; i < TARGET_NSIG_WORDS; i++) {
        __put_user(set->sig[i], &frame->uc.tuc_sigmask.sig[i]);
    }

    /* the kernel ignores SA_RESTORER and returns through its stub */
    env->pc = ka->_sa_handler;
    env->reg[BFIN_SP] = frame_addr;
    env->reg[BFIN_RETS] = BFIN_SIGRETURN_STUB;
    env->reg[BFIN_R0] = sig;
    env->reg[BFIN_R0 + 1] = frame_addr +
                            offsetof(struct target_rt_sigframe, info);
    env->reg[BFIN_R0 + 2] = frame_addr +
                            offsetof(struct target_rt_sigframe, uc);

    unlock_user_struct(frame, frame_addr, 1);
}

long do_sigreturn(CPUBfinState *env)
{
    return -TARGET_ENOSYS;
}

long do_rt_sigreturn(CPUBfinState *env)
{
    struct target_rt_sigframe *frame;
    abi_ulong frame_addr = env->reg[BFIN_SP];
    sigset_t set;

    trace_user_do_rt_sigreturn(env, frame_addr);

    if (!lock_user_struct(VERIFY_READ, frame, frame_addr, 1)) {
        goto badframe;
    }

    target_to_host_sigset(&set, &frame->uc.tuc_sigmask);
    set_sigmask(&set);

    restore_sigcontext(&frame->uc.tuc_mcontext, env);
    target_restore_altstack(&frame->uc.tuc_stack, env);

    unlock_user_struct(frame, frame_addr, 0);
    return -QEMU_ESIGRETURN;

 badframe:
    force_sig(TARGET_SIGSEGV);
    return -QEMU_ESIGRETURN;
}
