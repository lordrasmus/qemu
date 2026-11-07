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

/* arch/metag/include/uapi/asm/sigcontext.h */
struct target_sigcontext {
    struct target_user_gp_regs regs;
    struct target_user_cb_regs cb;
    struct target_user_rp_state rp;
    abi_ulong oldmask;
};

struct target_ucontext {
    abi_ulong tuc_flags;
    abi_ulong tuc_link;
    target_stack_t tuc_stack;
    struct target_sigcontext tuc_mcontext;
    target_sigset_t tuc_sigmask;
};

/* arch/metag/kernel/signal.c */
struct target_rt_sigframe {
    struct target_siginfo info;
    struct target_ucontext uc;
    abi_ulong retcode[2];
};

/* TXSTATUS bits the kernel keeps in the context (user_gp_regs.status) */
#define META_STATUS_CF_MASK     0x1f        /* C V N Z, SCC */
#define META_STATUS_LSM_SHIFT   8
#define META_STATUS_LSM_MASK    (7 << META_STATUS_LSM_SHIFT)

static void setup_sigcontext(struct target_sigcontext *sc,
                             CPUMETAState *env, abi_ulong mask)
{
    uint32_t status = 0;
    int i;

    for (i = 0; i < 8; i++) {
        __put_user(env->dregs[0][i], &sc->regs.dx[i][0]);
        __put_user(env->dregs[1][i], &sc->regs.dx[i][1]);
    }
    for (i = 0; i < 4; i++) {
        __put_user(env->aregs[0][i], &sc->regs.ax[i][0]);
        __put_user(env->aregs[1][i], &sc->regs.ax[i][1]);
    }
    __put_user(env->pc[META_PC], &sc->regs.pc);
    for (i = 0; i < 5; i++) {
        status |= (env->cf[i] & 1) << i;
    }
    status |= env->lsmstep << META_STATUS_LSM_SHIFT;
    __put_user(status, &sc->regs.status);
    __put_user(env->cregs[META_TXRPT], &sc->regs.rpt);
    __put_user(env->cregs[META_TXBPOBITS], &sc->regs.bpobits);
    __put_user(env->cregs[META_TXMODE], &sc->regs.mode);
    __put_user(0, &sc->regs._pad1);

    /* no catch buffer or read pipeline state to hand out */
    memset(&sc->cb, 0, sizeof(sc->cb));
    memset(&sc->rp, 0, sizeof(sc->rp));

    __put_user(mask, &sc->oldmask);

    /* the handler starts outside any MSET/MGET */
    env->lsmstep = 0;
}

static void restore_sigcontext(CPUMETAState *env,
                               struct target_sigcontext *sc)
{
    uint32_t status;
    int i;

    for (i = 0; i < 8; i++) {
        __get_user(env->dregs[0][i], &sc->regs.dx[i][0]);
        __get_user(env->dregs[1][i], &sc->regs.dx[i][1]);
    }
    for (i = 0; i < 4; i++) {
        __get_user(env->aregs[0][i], &sc->regs.ax[i][0]);
        __get_user(env->aregs[1][i], &sc->regs.ax[i][1]);
    }
    __get_user(env->pc[META_PC], &sc->regs.pc);
    __get_user(status, &sc->regs.status);
    for (i = 0; i < 5; i++) {
        env->cf[i] = (status >> i) & 1;
    }
    env->lsmstep = (status & META_STATUS_LSM_MASK) >> META_STATUS_LSM_SHIFT;
    __get_user(env->cregs[META_TXRPT], &sc->regs.rpt);
    __get_user(env->cregs[META_TXBPOBITS], &sc->regs.bpobits);
    __get_user(env->cregs[META_TXMODE], &sc->regs.mode);
}

/* The stack grows upwards: the frame goes at the (aligned) stack pointer. */
static abi_ulong get_sigframe(struct target_sigaction *ka, CPUMETAState *env)
{
    abi_ulong sp = get_sp_from_cpustate(env);

    if ((ka->sa_flags & TARGET_SA_ONSTACK) && !sas_ss_flags(sp)) {
        sp = get_task_state(thread_cpu)->sigaltstack_used.ss_sp;
    }
    return QEMU_ALIGN_UP(sp, 8);
}

void setup_rt_frame(int sig, struct target_sigaction *ka,
                    target_siginfo_t *info,
                    target_sigset_t *set, CPUMETAState *env)
{
    struct target_rt_sigframe *frame;
    abi_ulong frame_addr;
    int i;

    frame_addr = get_sigframe(ka, env);
    trace_user_setup_rt_frame(env, frame_addr);
    if (!lock_user_struct(VERIFY_WRITE, frame, frame_addr, 0)) {
        goto give_sigsegv;
    }

    frame->info = *info;

    __put_user(0, &frame->uc.tuc_flags);
    __put_user(0, &frame->uc.tuc_link);
    target_save_altstack(&frame->uc.tuc_stack, env);
    setup_sigcontext(&frame->uc.tuc_mcontext, env, set->sig[0]);
    for (i = 0; i < TARGET_NSIG_WORDS; i++) {
        __put_user(set->sig[i], &frame->uc.tuc_sigmask.sig[i]);
    }

    /* the kernel's on-stack return code, for unwinders; we use the page */
    __put_user(0x03000004 | (TARGET_NR_rt_sigreturn << 3), &frame->retcode[0]);
    __put_user(0xaf440001, &frame->retcode[1]);

    unlock_user_struct(frame, frame_addr, 1);

    env->dregs[1][4] = default_rt_sigreturn;                    /* D1RtP */
    env->aregs[0][0] = frame_addr + sizeof(*frame);             /* A0StP */
    env->dregs[1][3] = sig;                                     /* D1Ar1 */
    env->dregs[0][3] = frame_addr
                       + offsetof(struct target_rt_sigframe, info); /* D0Ar2 */
    env->dregs[1][2] = frame_addr
                       + offsetof(struct target_rt_sigframe, uc);   /* D1Ar3 */
    env->pc[META_PC] = ka->_sa_handler;
    return;

give_sigsegv:
    force_sigsegv(sig);
}

long do_rt_sigreturn(CPUMETAState *env)
{
    struct target_rt_sigframe *frame;
    abi_ulong frame_addr;
    sigset_t set;

    frame_addr = get_sp_from_cpustate(env) - sizeof(*frame);
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
    uint32_t *tramp = lock_user(VERIFY_WRITE, sigtramp_page, 8, 0);
    assert(tramp != NULL);

    /* MOV D1Re0, #__NR_rt_sigreturn; SWITCH #0x440001 */
    __put_user(0x03000004 | (TARGET_NR_rt_sigreturn << 3), tramp + 0);
    __put_user(0xaf440001, tramp + 1);

    default_rt_sigreturn = sigtramp_page;
    unlock_user(tramp, sigtramp_page, 8);
}
