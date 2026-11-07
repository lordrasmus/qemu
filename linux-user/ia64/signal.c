/*
 * Emulation of Linux signals, ia64
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu.h"
#include "user-internals.h"
#include "signal-common.h"
#include "linux-user/trace.h"

/* arch/ia64/include/uapi/asm/sigcontext.h */
#define TARGET_SC_FLAG_ONSTACK  1

#define FPSR_DEFAULT            0x0009804c0270033full

struct target_sigcontext {
    abi_ulong sc_flags;
    abi_ulong sc_nat;
    target_stack_t sc_stack;
    abi_ulong sc_ip;
    abi_ulong sc_cfm;
    abi_ulong sc_um;
    abi_ulong sc_ar_rsc;
    abi_ulong sc_ar_bsp;
    abi_ulong sc_ar_rnat;
    abi_ulong sc_ar_ccv;
    abi_ulong sc_ar_unat;
    abi_ulong sc_ar_fpsr;
    abi_ulong sc_ar_pfs;
    abi_ulong sc_ar_lc;
    abi_ulong sc_pr;
    abi_ulong sc_br[8];
    abi_ulong sc_gr[32];
    abi_ulong sc_fr[128][2] QEMU_ALIGNED(16);   /* spill format */
    abi_ulong sc_rbs_base;
    abi_ulong sc_loadrs;
    abi_ulong sc_ar25;
    abi_ulong sc_ar26;
    abi_ulong sc_rsvd[12];
    target_sigset_t sc_mask;
};

/* arch/ia64/kernel/sigframe.h */
struct target_sigframe {
    abi_ulong arg0;
    abi_ulong arg1;
    abi_ulong arg2;
    abi_ulong handler;
    target_siginfo_t info;
    struct target_sigcontext sc QEMU_ALIGNED(16);
};

QEMU_BUILD_BUG_ON(offsetof(struct target_sigcontext, sc_fr) != 464);
QEMU_BUILD_BUG_ON(offsetof(struct target_sigcontext, sc_mask) != 2640);
QEMU_BUILD_BUG_ON(offsetof(struct target_sigframe, sc) != 160);

static void setup_sigcontext(struct target_sigcontext *sc,
                             CPUIA64State *env)
{
    uint64_t nat = 0, pr = 0;
    int i;

    for (i = 1; i < 32; i++) {
        __put_user(env->gr[i], &sc->sc_gr[i]);
        nat |= (uint64_t)(env->nat[i] & 1) << i;
    }
    __put_user(0, &sc->sc_gr[0]);
    for (i = 0; i < 64; i++) {
        pr |= (uint64_t)(env->pr[i] & 1) << i;
    }
    for (i = 0; i < 8; i++) {
        __put_user(env->br[i], &sc->sc_br[i]);
    }
    for (i = 0; i < 128; i++) {
        __put_user(env->fr[i].mant, &sc->sc_fr[i][0]);
        __put_user(env->fr[i].se & 0x3ffff, &sc->sc_fr[i][1]);
    }
    __put_user(nat, &sc->sc_nat);
    __put_user(pr | 1, &sc->sc_pr);
    __put_user(ia64_user_unfold(env->ip) + env->ri, &sc->sc_ip);
    __put_user(env->cfm, &sc->sc_cfm);
    __put_user(env->psr & PSR_UM_MASK, &sc->sc_um);
    __put_user(env->ar[AR_RSC], &sc->sc_ar_rsc);
    __put_user(env->ar[AR_CCV], &sc->sc_ar_ccv);
    __put_user(env->ar[AR_UNAT], &sc->sc_ar_unat);
    __put_user(env->ar[AR_FPSR], &sc->sc_ar_fpsr);
    __put_user(env->ar[AR_PFS], &sc->sc_ar_pfs);
    __put_user(env->ar[AR_LC], &sc->sc_ar_lc);
    __put_user(env->ar[AR_CSD], &sc->sc_ar25);
    __put_user(env->ar[AR_SSD], &sc->sc_ar26);
    __put_user(0, &sc->sc_loadrs);
}

static void restore_sigcontext(CPUIA64State *env,
                               struct target_sigcontext *sc)
{
    uint64_t v, nat, pr, um;
    int i;

    __get_user(nat, &sc->sc_nat);
    for (i = 1; i < 32; i++) {
        __get_user(env->gr[i], &sc->sc_gr[i]);
        env->nat[i] = (nat >> i) & 1;
    }
    __get_user(pr, &sc->sc_pr);
    for (i = 1; i < 64; i++) {
        env->pr[i] = (pr >> i) & 1;
    }
    for (i = 0; i < 8; i++) {
        __get_user(env->br[i], &sc->sc_br[i]);
    }
    for (i = 2; i < 128; i++) {
        __get_user(env->fr[i].mant, &sc->sc_fr[i][0]);
        __get_user(v, &sc->sc_fr[i][1]);
        env->fr[i].se = v & 0x3ffff;
    }
    __get_user(v, &sc->sc_ip);
    env->ip = ia64_user_fold(v & ~0xfull);
    env->ri = v & 3;
    if (env->ri == 3) {
        env->ri = 0;
    }
    __get_user(um, &sc->sc_um);
    env->psr = (env->psr & ~PSR_UM_MASK) | (um & PSR_UM_MASK);
    __get_user(v, &sc->sc_ar_rsc);
    env->ar[AR_RSC] = v | (3 << 2);
    __get_user(env->ar[AR_CCV], &sc->sc_ar_ccv);
    __get_user(env->ar[AR_UNAT], &sc->sc_ar_unat);
    __get_user(env->ar[AR_FPSR], &sc->sc_ar_fpsr);
    __get_user(env->ar[AR_PFS], &sc->sc_ar_pfs);
    __get_user(env->ar[AR_LC], &sc->sc_ar_lc);
    __get_user(env->ar[AR_CSD], &sc->sc_ar25);
    __get_user(env->ar[AR_SSD], &sc->sc_ar26);
}

void setup_rt_frame(int sig, struct target_sigaction *ka,
                    target_siginfo_t *info,
                    target_sigset_t *set, CPUIA64State *env)
{
    TaskState *ts = get_task_state(thread_cpu);
    abi_ulong frame_addr, sp, new_rbs = 0, handler_ip, handler_gp;
    struct target_sigframe *frame;
    uint64_t cfm = env->cfm;
    int i;

    /*
     * The kernel's sigtramp covers the interrupted frame and flushes the
     * register stack; sc_ar_bsp is then ar.bsp.
     */
    ia64_rse_flush(env);

    sp = target_sigsp(env->gr[12], ka);
    if (sp != env->gr[12] &&
        env->ar[AR_BSPSTORE] - ts->sigaltstack_used.ss_sp >=
        ts->sigaltstack_used.ss_size) {
        /* the register stack moves to the bottom of the signal stack */
        new_rbs = ROUND_UP(ts->sigaltstack_used.ss_sp, 8);
    }
    frame_addr = (sp - sizeof(*frame)) & -16ull;
    trace_user_setup_rt_frame(env, frame_addr);
    if (!lock_user_struct(VERIFY_WRITE, frame, frame_addr, 0)) {
        goto give_sigsegv;
    }

    __put_user(sig, &frame->arg0);
    __put_user(frame_addr + offsetof(struct target_sigframe, info),
               &frame->arg1);
    __put_user(frame_addr + offsetof(struct target_sigframe, sc),
               &frame->arg2);
    __put_user(ka->_sa_handler, &frame->handler);
    frame->info = *info;

    setup_sigcontext(&frame->sc, env);
    __put_user(on_sig_stack(frame_addr) ? TARGET_SC_FLAG_ONSTACK : 0,
               &frame->sc.sc_flags);
    __put_user(cfm, &frame->sc.sc_cfm);
    __put_user(env->ar[AR_BSPSTORE], &frame->sc.sc_ar_bsp);
    __put_user(env->ar[AR_RNAT], &frame->sc.sc_ar_rnat);
    __put_user(new_rbs, &frame->sc.sc_rbs_base);
    target_save_altstack(&frame->sc.sc_stack, env);
    for (i = 0; i < TARGET_NSIG_WORDS; i++) {
        __put_user(set->sig[i], &frame->sc.sc_mask.sig[i]);
    }
    unlock_user_struct(frame, frame_addr, 1);

    /* sa_handler points at a function descriptor */
    if (get_user_u64(handler_ip, ka->_sa_handler) ||
        get_user_u64(handler_gp, ka->_sa_handler + 8)) {
        goto give_sigsegv;
    }

    if (new_rbs) {
        env->ar[AR_BSPSTORE] = new_rbs;
        env->ar[AR_RNAT] = 0;
    }
    /* call the handler, returning to the sigtramp with ar.ec intact */
    env->ar[AR_PFS] = ((env->ar[AR_EC] & 0x3f) << R_PFS_PEC_SHIFT) |
                      (3ull << R_PFS_PPL_SHIFT);
    env->br[0] = default_rt_sigreturn;
    env->gr[1] = handler_gp;
    env->nat[1] = 0;
    env->gr[12] = frame_addr - 16;
    env->nat[12] = 0;
    env->cfm = FIELD_DP64(0, CFM, SOF, 3);
    env->gr[32] = sig;
    env->gr[33] = frame_addr + offsetof(struct target_sigframe, info);
    env->gr[34] = frame_addr + offsetof(struct target_sigframe, sc);
    env->nat[32] = env->nat[33] = env->nat[34] = 0;
    env->ar[AR_FPSR] = FPSR_DEFAULT;
    env->psr &= ~PSR_BE;
    env->ip = ia64_user_fold(handler_ip & ~0xfull);
    env->ri = 0;
    return;

give_sigsegv:
    force_sigsegv(sig);
}

long do_rt_sigreturn(CPUIA64State *env)
{
    abi_ulong frame_addr = env->gr[12] + 16;
    struct target_sigframe *frame;
    abi_ulong bsp, rnat, cfm;
    sigset_t set;

    trace_user_do_rt_sigreturn(env, frame_addr);
    if (!lock_user_struct(VERIFY_READ, frame, frame_addr, 1)) {
        goto badframe;
    }
    target_to_host_sigset(&set, &frame->sc.sc_mask);
    set_sigmask(&set);

    restore_sigcontext(env, &frame->sc);
    __get_user(bsp, &frame->sc.sc_ar_bsp);
    __get_user(rnat, &frame->sc.sc_ar_rnat);
    __get_user(cfm, &frame->sc.sc_cfm);
    target_restore_altstack(&frame->sc.sc_stack, env);
    unlock_user_struct(frame, frame_addr, 0);

    /* the interrupted frame comes back from the backing store */
    env->rse_ndirty = 0;
    env->rse_missing = 0;
    env->ar[AR_BSPSTORE] = bsp & ~7ull;
    env->ar[AR_RNAT] = rnat;
    env->cfm = cfm & CFM_MASK;
    ia64_rse_frame_load(env, FIELD_EX64(env->cfm, CFM, SOF), 0);
    return -QEMU_ESIGRETURN;

badframe:
    unlock_user_struct(frame, frame_addr, 0);
    force_sig(TARGET_SIGSEGV);
    return -QEMU_ESIGRETURN;
}

void setup_sigtramp(abi_ulong sigtramp_page)
{
    uint64_t *tramp = lock_user(VERIFY_WRITE, sigtramp_page, 16, 0);

    assert(tramp != NULL);
    /* { .mii nop.m 0; mov r15 = __NR_rt_sigreturn; break.i 0x100000;; } */
    QEMU_BUILD_BUG_ON(TARGET_NR_rt_sigreturn != 1181);
    __put_user(0xe8f0000100000001ull, tramp + 0);
    __put_user(0x0800000000481200ull, tramp + 1);
    default_rt_sigreturn = sigtramp_page;
    unlock_user(tramp, sigtramp_page, 16);
}
