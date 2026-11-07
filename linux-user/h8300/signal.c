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

/* arch/h8300/include/uapi/asm/sigcontext.h */
struct target_sigcontext {
    abi_ulong sc_mask;
    abi_ulong sc_usp;
    abi_ulong sc_er0;
    abi_ulong sc_er1;
    abi_ulong sc_er2;
    abi_ulong sc_er3;
    abi_ulong sc_er4;
    abi_ulong sc_er5;
    abi_ulong sc_er6;
    uint16_t sc_ccr;
    abi_ulong sc_pc;
};

struct target_ucontext {
    abi_ulong tuc_flags;
    abi_ulong tuc_link;
    target_stack_t tuc_stack;
    struct target_sigcontext tuc_mcontext;
    target_sigset_t tuc_sigmask;
};

/*
 * arch/h8300/kernel/signal.c, from pretcode on: the dummy er0, vector,
 * exr and pc words below it are popped by the kernel's return to user
 * space, the handler starts with sp at pretcode.
 */
struct target_rt_sigframe {
    abi_ulong pretcode;
    abi_ulong pinfo;
    abi_ulong puc;
    uint8_t retcode[8];
    struct target_siginfo info;
    struct target_ucontext uc;
};

/*
 * The kernel's user sp (_sw_usp) sits below the er0, vector, exr and
 * ccr/pc words of the trap frame: 14 bytes on the H8S.
 */
#define H8300_USP_BIAS 14

/* sub.l er0,er0; mov.b #__NR_rt_sigreturn,r0l; trapa #0 */
static const uint8_t h8300_rt_sigreturn_code[6] = {
    0x1a, 0x80, 0xf8, TARGET_NR_rt_sigreturn, 0x57, 0x00,
};

static void setup_sigcontext(struct target_sigcontext *sc,
                             CPUH8300State *env, abi_ulong mask)
{
    __put_user(mask, &sc->sc_mask);
    __put_user(env->regs[7] - H8300_USP_BIAS, &sc->sc_usp);
    __put_user(env->regs[0], &sc->sc_er0);
    __put_user(env->regs[1], &sc->sc_er1);
    __put_user(env->regs[2], &sc->sc_er2);
    __put_user(env->regs[3], &sc->sc_er3);
    __put_user(env->regs[4], &sc->sc_er4);
    __put_user(env->regs[5], &sc->sc_er5);
    __put_user(env->regs[6], &sc->sc_er6);
    __put_user(h8300_cpu_pack_ccr(env), &sc->sc_ccr);
    __put_user(env->pc, &sc->sc_pc);
}

static void restore_sigcontext(CPUH8300State *env,
                               struct target_sigcontext *sc)
{
    abi_ulong usp;
    uint16_t ccr;

    __get_user(env->regs[0], &sc->sc_er0);
    __get_user(env->regs[1], &sc->sc_er1);
    __get_user(env->regs[2], &sc->sc_er2);
    __get_user(env->regs[3], &sc->sc_er3);
    __get_user(env->regs[4], &sc->sc_er4);
    __get_user(env->regs[5], &sc->sc_er5);
    __get_user(env->regs[6], &sc->sc_er6);
    __get_user(usp, &sc->sc_usp);
    env->regs[7] = usp + H8300_USP_BIAS;
    __get_user(ccr, &sc->sc_ccr);
    h8300_cpu_unpack_ccr(env, ccr & 0xff);
    __get_user(env->pc, &sc->sc_pc);
}

void setup_rt_frame(int sig, struct target_sigaction *ka,
                    target_siginfo_t *info,
                    target_sigset_t *set, CPUH8300State *env)
{
    TaskState *ts = get_task_state(env_cpu(env));
    struct target_rt_sigframe *frame;
    abi_ulong frame_addr;
    int i;

    frame_addr = target_sigsp(get_sp_from_cpustate(env), ka)
                 - H8300_USP_BIAS - sizeof(*frame);
    frame_addr &= ~7;
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
    memcpy(frame->retcode, h8300_rt_sigreturn_code,
           sizeof(h8300_rt_sigreturn_code));
    __put_user(frame_addr + offsetof(struct target_rt_sigframe, info),
               &frame->pinfo);
    __put_user(frame_addr + offsetof(struct target_rt_sigframe, uc),
               &frame->puc);
    if (ka->sa_flags & TARGET_SA_RESTORER) {
        __put_user(ka->sa_restorer, &frame->pretcode);
    } else {
        __put_user(default_rt_sigreturn, &frame->pretcode);
    }
    unlock_user_struct(frame, frame_addr, 1);

    env->regs[0] = sig;
    env->regs[1] = frame_addr + offsetof(struct target_rt_sigframe, info);
    env->regs[2] = frame_addr + offsetof(struct target_rt_sigframe, uc);
    env->regs[5] = ts->info->start_data;     /* GOT base */
    env->regs[7] = frame_addr;
    env->pc = ka->_sa_handler & 0x00ffffff;
}

long do_rt_sigreturn(CPUH8300State *env)
{
    /* The handler's rts popped pretcode. */
    abi_ulong frame_addr = env->regs[7] - 4;
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
    uint8_t *tramp = lock_user(VERIFY_WRITE, sigtramp_page,
                               sizeof(h8300_rt_sigreturn_code), 0);

    assert(tramp != NULL);
    QEMU_BUILD_BUG_ON(TARGET_NR_rt_sigreturn > 0xff);
    memcpy(tramp, h8300_rt_sigreturn_code, sizeof(h8300_rt_sigreturn_code));
    default_rt_sigreturn = sigtramp_page;
    unlock_user(tramp, sigtramp_page, sizeof(h8300_rt_sigreturn_code));
}
