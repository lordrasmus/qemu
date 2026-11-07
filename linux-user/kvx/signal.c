/*
 * Emulation of Linux signals for Kalray kvx
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu.h"
#include "user-internals.h"
#include "signal-common.h"
#include "linux-user/trace.h"
#include "target_ptrace.h"

/* arch/kvx/include/uapi/asm/sigcontext.h, asm-generic/ucontext.h */
typedef struct target_sigcontext {
    struct target_user_pt_regs sc_regs;
} target_sigcontext;

typedef struct target_ucontext {
    abi_ulong tuc_flags;
    abi_ulong tuc_link;
    target_stack_t tuc_stack;
    target_sigcontext tuc_mcontext;
    target_sigset_t tuc_sigmask;
} target_ucontext;

typedef struct target_rt_sigframe {
    struct target_siginfo info;
    target_ucontext uc;
} target_rt_sigframe;

static void setup_sigcontext(target_sigcontext *sc, CPUKVXState *env)
{
    struct target_user_pt_regs *r = &sc->sc_regs;
    int i;

    for (i = 0; i < 64; i++) {
        __put_user(*kvx_gpr(env, i), &r->r[i]);
    }
    __put_user(kvx_register_read_u64(env, REG_kv3_LC), &r->lc);
    __put_user(kvx_register_read_u64(env, REG_kv3_LE), &r->le);
    __put_user(kvx_register_read_u64(env, REG_kv3_LS), &r->ls);
    __put_user(kvx_register_read_u64(env, REG_kv3_RA), &r->ra);
    __put_user(kvx_register_read_u64(env, REG_kv3_CS), &r->cs);
    __put_user(kvx_register_read_u64(env, REG_kv3_PC), &r->spc);
}

static void restore_sigcontext(CPUKVXState *env, target_sigcontext *sc)
{
    struct target_user_pt_regs *r = &sc->sc_regs;
    uint64_t v;
    int i;

    for (i = 0; i < 64; i++) {
        __get_user(*kvx_gpr(env, i), &r->r[i]);
    }
    __get_user(v, &r->ls);
    kvx_register_write_u64(env, REG_kv3_LS, v);
    __get_user(v, &r->ra);
    kvx_register_write_u64(env, REG_kv3_RA, v);
    __get_user(v, &r->cs);
    kvx_register_write_u64(env, REG_kv3_CS, v);
    __get_user(v, &r->spc);
    kvx_register_write_u64(env, REG_kv3_PC, v);
    /* LE last, the hardware loop end is cached in translated code */
    __get_user(v, &r->lc);
    kvx_register_write_u64(env, REG_kv3_LC, v);
    __get_user(v, &r->le);
    kvx_reg_write_lc_le(env_archcpu(env), REG_kv3_LE, v);
}

static abi_ulong get_sigframe(struct target_sigaction *ka,
                              CPUKVXState *env, size_t frame_size)
{
    abi_ulong sp = target_sigsp(get_sp_from_cpustate(env), ka);

    return QEMU_ALIGN_DOWN(sp - frame_size, 16);
}

void setup_rt_frame(int sig, struct target_sigaction *ka,
                    target_siginfo_t *info,
                    target_sigset_t *set, CPUKVXState *env)
{
    abi_ulong frame_addr;
    target_rt_sigframe *frame;
    int i;

    frame_addr = get_sigframe(ka, env, sizeof(*frame));
    trace_user_setup_rt_frame(env, frame_addr);
    if (!lock_user_struct(VERIFY_WRITE, frame, frame_addr, 0)) {
        goto give_sigsegv;
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

    kvx_register_write_u64(env, REG_kv3_RA, default_rt_sigreturn);
    kvx_register_write_u64(env, REG_kv3_PC, ka->_sa_handler);
    *kvx_gpr(env, 12) = frame_addr;
    *kvx_gpr(env, 0) = sig;
    *kvx_gpr(env, 1) = frame_addr + offsetof(target_rt_sigframe, info);
    *kvx_gpr(env, 2) = frame_addr + offsetof(target_rt_sigframe, uc);
    return;

give_sigsegv:
    force_sigsegv(sig);
}

long do_rt_sigreturn(CPUKVXState *env)
{
    abi_ulong frame_addr = get_sp_from_cpustate(env);
    target_rt_sigframe *frame;
    sigset_t set;

    trace_user_do_rt_sigreturn(env, frame_addr);
    if (frame_addr & 15) {
        goto badframe;
    }
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

    /* make $r6 = __NR_rt_sigreturn ;; scall $r6 ;; */
    QEMU_BUILD_BUG_ON(TARGET_NR_rt_sigreturn != 139);
    __put_user(0x6018008b, tramp + 0);
    __put_user(0x0fe40006, tramp + 1);

    default_rt_sigreturn = sigtramp_page;
    unlock_user(tramp, sigtramp_page, 8);
}
