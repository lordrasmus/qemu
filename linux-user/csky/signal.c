/*
 *  Emulation of Linux signals
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu.h"
#include "user-internals.h"
#include "signal-common.h"
#include "linux-user/trace.h"
#include "target_ptrace.h"

/* See arch/csky/include/uapi/asm/sigcontext.h and kernel/signal.c. */
struct target_sigcontext {
    struct target_pt_regs sc_pt_regs;
    struct target_user_fp sc_user_fp;
};

struct target_ucontext {
    abi_ulong tuc_flags;
    abi_ulong tuc_link;
    target_stack_t tuc_stack;
    struct target_sigcontext tuc_mcontext;
    target_sigset_t tuc_sigmask;
};

struct target_rt_sigframe {
    abi_int pad[3];             /* libgcc's linux-unwind.h expects it */
    struct target_siginfo info;
    struct target_ucontext uc;
};

static void setup_sigcontext(struct target_sigcontext *sc, CPUCSKYState *env)
{
    struct target_pt_regs *r = &sc->sc_pt_regs;
    int i;

    __put_user(env->regs[31], &r->tls);
    __put_user(env->regs[15], &r->lr);
    __put_user(env->pc, &r->pc);
    __put_user((env->cp0.psr & ~1) | env->psr_c, &r->sr);
    __put_user(env->regs[14], &r->usp);
    __put_user(env->regs[0], &r->orig_a0);
    __put_user(env->regs[0], &r->a0);
    __put_user(env->regs[1], &r->a1);
    __put_user(env->regs[2], &r->a2);
    __put_user(env->regs[3], &r->a3);
    for (i = 0; i < 10; i++) {
        __put_user(env->regs[4 + i], &r->regs[i]);
    }
    for (i = 0; i < 15; i++) {
        __put_user(env->regs[16 + i], &r->exregs[i]);
    }
    __put_user(env->hi, &r->rhi);
    __put_user(env->lo, &r->rlo);
    __put_user(env->dcsr_v, &r->dcsr);

    /* FPUv2: fstmu.64 vr0-vr31, two words per register */
    for (i = 0; i < 32; i++) {
        __put_user((uint32_t)env->vfp.reg[i].udspl[0], &sc->sc_user_fp.vr[2 * i]);
        __put_user((uint32_t)(env->vfp.reg[i].udspl[0] >> 32),
                   &sc->sc_user_fp.vr[2 * i + 1]);
    }
    __put_user(env->vfp.fcr, &sc->sc_user_fp.fcr);
    __put_user(env->vfp.fesr, &sc->sc_user_fp.fesr);
    __put_user(env->vfp.fid, &sc->sc_user_fp.fid);
}

static void restore_sigcontext(CPUCSKYState *env,
                               struct target_sigcontext *sc)
{
    struct target_pt_regs *r = &sc->sc_pt_regs;
    abi_ulong v;
    uint32_t lo, hi;
    int i;

    __get_user(env->regs[31], &r->tls);
    __get_user(env->regs[15], &r->lr);
    __get_user(env->pc, &r->pc);
    /* only the condition bit comes from user space */
    __get_user(v, &r->sr);
    env->psr_c = v & 1;
    __get_user(env->regs[14], &r->usp);
    __get_user(env->regs[0], &r->a0);
    __get_user(env->regs[1], &r->a1);
    __get_user(env->regs[2], &r->a2);
    __get_user(env->regs[3], &r->a3);
    for (i = 0; i < 10; i++) {
        __get_user(env->regs[4 + i], &r->regs[i]);
    }
    for (i = 0; i < 15; i++) {
        __get_user(env->regs[16 + i], &r->exregs[i]);
    }
    __get_user(env->hi, &r->rhi);
    __get_user(env->lo, &r->rlo);
    __get_user(env->dcsr_v, &r->dcsr);

    for (i = 0; i < 32; i++) {
        __get_user(lo, &sc->sc_user_fp.vr[2 * i]);
        __get_user(hi, &sc->sc_user_fp.vr[2 * i + 1]);
        env->vfp.reg[i].udspl[0] = deposit64(lo, 32, 32, hi);
    }
    __get_user(env->vfp.fcr, &sc->sc_user_fp.fcr);
    __get_user(env->vfp.fesr, &sc->sc_user_fp.fesr);
}

static abi_ulong get_sigframe(struct target_sigaction *ka,
                              CPUCSKYState *env, size_t frame_size)
{
    abi_ulong sp = target_sigsp(get_sp_from_cpustate(env), ka);

    return (sp - frame_size) & ~7ul;
}

void setup_rt_frame(int sig, struct target_sigaction *ka,
                    target_siginfo_t *info,
                    target_sigset_t *set, CPUCSKYState *env)
{
    abi_ulong frame_addr;
    struct target_rt_sigframe *frame;
    int i;

    frame_addr = get_sigframe(ka, env, sizeof(*frame));
    trace_user_setup_rt_frame(env, frame_addr);
    if (!lock_user_struct(VERIFY_WRITE, frame, frame_addr, 0)) {
        goto give_sigsegv;
    }

    for (i = 0; i < 3; i++) {
        __put_user(0, &frame->pad[i]);
    }
    frame->info = *info;
    __put_user(0, &frame->uc.tuc_flags);
    __put_user(0, &frame->uc.tuc_link);
    target_save_altstack(&frame->uc.tuc_stack, env);
    setup_sigcontext(&frame->uc.tuc_mcontext, env);
    for (i = 0; i < TARGET_NSIG_WORDS; ++i) {
        __put_user(set->sig[i], &frame->uc.tuc_sigmask.sig[i]);
    }
    unlock_user_struct(frame, frame_addr, 1);

    env->regs[15] = default_rt_sigreturn;
    env->pc = ka->_sa_handler;
    env->regs[14] = frame_addr;
    env->regs[0] = sig;
    env->regs[1] = frame_addr + offsetof(struct target_rt_sigframe, info);
    env->regs[2] = frame_addr + offsetof(struct target_rt_sigframe, uc);
    return;

give_sigsegv:
    force_sigsegv(sig);
}

long do_rt_sigreturn(CPUCSKYState *env)
{
    abi_ulong frame_addr = env->regs[14];
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

void setup_sigtramp(abi_ulong sigtramp_page)
{
    uint16_t *tramp = lock_user(VERIFY_WRITE, sigtramp_page, 8, 0);
    assert(tramp != NULL);

    /* movi r7, __NR_rt_sigreturn; trap 0 (arch/csky/abiv2/inc/abi/vdso.h) */
    __put_user(0xea07, tramp + 0);
    __put_user(TARGET_NR_rt_sigreturn, tramp + 1);
    __put_user(0xc000, tramp + 2);
    __put_user(0x2020, tramp + 3);

    default_rt_sigreturn = sigtramp_page;
    unlock_user(tramp, sigtramp_page, 8);
}
