/*
 *  Emulation of Linux signals, FR-V
 *
 *  Layout and register use from arch/frv/kernel/signal.c (Linux 4.9).
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

/* struct user_context, arch/frv/include/uapi/asm/registers.h */
struct target_user_context {
    /* struct user_int_regs */
    abi_ulong psr;
    abi_ulong isr;
    abi_ulong ccr;
    abi_ulong cccr;
    abi_ulong lr;
    abi_ulong lcr;
    abi_ulong pc;
    abi_ulong status;
    abi_ulong syscallno;
    abi_ulong orig_gr8;
    abi_ulong gner[2];
    abi_ulong iacc[2];          /* unsigned long long iacc[1] */
    abi_ulong gr[64];           /* gr[0] holds TBR */
    /* struct user_fpmedia_regs */
    abi_ulong fr[64];
    abi_ulong fner[2];
    abi_ulong msr[2];
    abi_ulong acc[8];
    uint8_t accg[8];
    abi_ulong fsr[1];
    abi_ulong extension;
} QEMU_ALIGNED(8);

struct target_sigcontext {
    struct target_user_context sc_context;
    abi_ulong sc_oldmask;
} QEMU_ALIGNED(8);

struct target_ucontext {
    abi_ulong tuc_flags;
    abi_ulong tuc_link;
    target_stack_t tuc_stack;
    struct target_sigcontext tuc_mcontext;
    target_sigset_t tuc_sigmask;
};

struct target_sigframe {
    abi_ulong pretcode;
    abi_int sig;
    struct target_sigcontext sc;
    abi_ulong extramask[TARGET_NSIG_WORDS - 1];
    uint32_t retcode[2];
};

struct target_rt_sigframe {
    abi_ulong pretcode;
    abi_int sig;
    abi_ulong pinfo;
    abi_ulong puc;
    target_siginfo_t info;
    struct target_ucontext uc;
    uint32_t retcode[2];
};

QEMU_BUILD_BUG_ON(sizeof(struct target_user_context) != 632);
QEMU_BUILD_BUG_ON(offsetof(struct target_sigframe, sc) != 8);
QEMU_BUILD_BUG_ON(offsetof(struct target_ucontext, tuc_mcontext) != 24);
QEMU_BUILD_BUG_ON(offsetof(struct target_rt_sigframe, uc) != 144);

static void setup_sigcontext(struct target_sigcontext *sc,
                             CPUFRVState *env, abi_ulong mask)
{
    struct target_user_context *uc = &sc->sc_context;
    int i;

    __put_user(env->spr[SPR_PSR], &uc->psr);
    __put_user(env->spr[SPR_ISR], &uc->isr);
    __put_user(frv_spr_read(env, SPR_CCR), &uc->ccr);
    __put_user(frv_spr_read(env, SPR_CCCR), &uc->cccr);
    __put_user(env->spr[SPR_LR], &uc->lr);
    __put_user(env->spr[SPR_LCR], &uc->lcr);
    __put_user(env->pc, &uc->pc);
    __put_user(0, &uc->status);
    __put_user(-1, &uc->syscallno);
    __put_user(env->gr[8], &uc->orig_gr8);
    __put_user(env->spr[SPR_IACC0H], &uc->iacc[0]);
    __put_user(env->spr[SPR_IACC0L], &uc->iacc[1]);
    __put_user(env->spr[SPR_TBR], &uc->gr[0]);
    for (i = 1; i < 64; i++) {
        __put_user(env->gr[i], &uc->gr[i]);
    }
    for (i = 0; i < 64; i++) {
        __put_user(env->fr[i], &uc->fr[i]);
    }
    __put_user(env->spr[SPR_MSR0], &uc->msr[0]);
    __put_user(env->spr[SPR_MSR1], &uc->msr[1]);
    for (i = 0; i < 8; i++) {
        __put_user(env->spr[SPR_ACC0 + i], &uc->acc[i]);
        uc->accg[i] = env->spr[SPR_ACCG0 + i];
    }
    __put_user(0, &uc->extension);
    __put_user(mask, &sc->sc_oldmask);
}

/* Like the kernel, everything but PSR and TBR comes back */
static void restore_sigcontext(CPUFRVState *env,
                               struct target_sigcontext *sc)
{
    struct target_user_context *uc = &sc->sc_context;
    abi_ulong v;
    int i;

    __get_user(env->spr[SPR_ISR], &uc->isr);
    __get_user(v, &uc->ccr);
    frv_spr_write(env, SPR_CCR, v);
    __get_user(v, &uc->cccr);
    frv_spr_write(env, SPR_CCCR, v);
    __get_user(env->spr[SPR_LR], &uc->lr);
    __get_user(env->spr[SPR_LCR], &uc->lcr);
    __get_user(env->pc, &uc->pc);
    __get_user(env->spr[SPR_IACC0H], &uc->iacc[0]);
    __get_user(env->spr[SPR_IACC0L], &uc->iacc[1]);
    for (i = 1; i < 64; i++) {
        __get_user(env->gr[i], &uc->gr[i]);
    }
    for (i = 0; i < 64; i++) {
        __get_user(env->fr[i], &uc->fr[i]);
    }
    __get_user(env->spr[SPR_MSR0], &uc->msr[0]);
    __get_user(env->spr[SPR_MSR1], &uc->msr[1]);
    for (i = 0; i < 8; i++) {
        __get_user(env->spr[SPR_ACC0 + i], &uc->acc[i]);
        env->spr[SPR_ACCG0 + i] = uc->accg[i];
    }
}

static abi_ulong get_sigframe(struct target_sigaction *ka,
                              CPUFRVState *env, size_t frame_size)
{
    abi_ulong sp = target_sigsp(get_sp_from_cpustate(env), ka);

    return (sp - frame_size) & ~7;
}

/* setlos #nr,gr7; tira gr0,#0 */
static void setup_sigreturn(uint32_t *retcode, int nr)
{
    __put_user(0x8efc0000 | nr, retcode + 0);
    __put_user(0xc0700000, retcode + 1);
}

/* Enter the handler; FDPIC handlers are function descriptors */
static bool setup_handler(CPUFRVState *env, struct target_sigaction *ka,
                          abi_ulong frame_addr, abi_ulong retcode, int sig)
{
    abi_ulong handler = ka->_sa_handler;

    if (info_is_fdpic(get_task_state(thread_cpu)->info)) {
        abi_ulong text, got;

        if (get_user_ual(text, handler) || get_user_ual(got, handler + 4)) {
            return false;
        }
        env->pc = text;
        env->gr[15] = got;
    } else {
        env->pc = handler;
        env->gr[15] = 0;
    }
    env->gr[1] = frame_addr;
    env->spr[SPR_LR] = retcode;
    env->gr[8] = sig;
    return true;
}

void setup_frame(int sig, struct target_sigaction *ka,
                 target_sigset_t *set, CPUFRVState *env)
{
    struct target_sigframe *frame;
    abi_ulong frame_addr;
    int i;

    frame_addr = get_sigframe(ka, env, sizeof(*frame));
    trace_user_setup_frame(env, frame_addr);
    if (!lock_user_struct(VERIFY_WRITE, frame, frame_addr, 0)) {
        goto give_sigsegv;
    }

    __put_user(sig, &frame->sig);
    setup_sigcontext(&frame->sc, env, set->sig[0]);
    for (i = 1; i < TARGET_NSIG_WORDS; i++) {
        __put_user(set->sig[i], &frame->extramask[i - 1]);
    }
    /* The kernel's stack trampoline, kept for unwinders; we return via the page */
    setup_sigreturn(frame->retcode, TARGET_NR_sigreturn);
    __put_user(default_sigreturn, &frame->pretcode);
    unlock_user_struct(frame, frame_addr, 1);

    if (!setup_handler(env, ka, frame_addr, default_sigreturn, sig)) {
        force_sigsegv(sig);
    }
    return;

give_sigsegv:
    force_sigsegv(sig);
}

void setup_rt_frame(int sig, struct target_sigaction *ka,
                    target_siginfo_t *info,
                    target_sigset_t *set, CPUFRVState *env)
{
    struct target_rt_sigframe *frame;
    abi_ulong frame_addr;
    int i;

    frame_addr = get_sigframe(ka, env, sizeof(*frame));
    trace_user_setup_rt_frame(env, frame_addr);
    if (!lock_user_struct(VERIFY_WRITE, frame, frame_addr, 0)) {
        goto give_sigsegv;
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
    setup_sigcontext(&frame->uc.tuc_mcontext, env, set->sig[0]);
    for (i = 0; i < TARGET_NSIG_WORDS; i++) {
        __put_user(set->sig[i], &frame->uc.tuc_sigmask.sig[i]);
    }
    setup_sigreturn(frame->retcode, TARGET_NR_rt_sigreturn);
    __put_user(default_rt_sigreturn, &frame->pretcode);
    unlock_user_struct(frame, frame_addr, 1);

    if (!setup_handler(env, ka, frame_addr, default_rt_sigreturn, sig)) {
        force_sigsegv(sig);
        return;
    }
    env->gr[9] = frame_addr + offsetof(struct target_rt_sigframe, info);
    /* kernel-frv-sigframe-ucontext.patch: the third argument */
    env->gr[10] = frame_addr + offsetof(struct target_rt_sigframe, uc);
    return;

give_sigsegv:
    force_sigsegv(sig);
}

long do_sigreturn(CPUFRVState *env)
{
    struct target_sigframe *frame;
    abi_ulong frame_addr = get_sp_from_cpustate(env);
    target_sigset_t target_set;
    sigset_t set;
    int i;

    trace_user_do_sigreturn(env, frame_addr);
    if (!lock_user_struct(VERIFY_READ, frame, frame_addr, 1)) {
        goto badframe;
    }

    __get_user(target_set.sig[0], &frame->sc.sc_oldmask);
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

long do_rt_sigreturn(CPUFRVState *env)
{
    struct target_rt_sigframe *frame;
    abi_ulong frame_addr = get_sp_from_cpustate(env);
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

void setup_sigtramp(abi_ulong sigtramp_page)
{
    uint32_t *tramp = lock_user(VERIFY_WRITE, sigtramp_page, 16, 0);
    assert(tramp != NULL);

    default_sigreturn = sigtramp_page;
    setup_sigreturn(tramp, TARGET_NR_sigreturn);
    default_rt_sigreturn = sigtramp_page + 8;
    setup_sigreturn(tramp + 2, TARGET_NR_rt_sigreturn);

    unlock_user(tramp, sigtramp_page, 16);
}
