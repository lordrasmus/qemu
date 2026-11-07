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

/* arch/nds32/include/uapi/asm/sigcontext.h */
struct target_fpu_struct {
    abi_ullong fd_regs[32];
    abi_ulong fpcsr;
    abi_ulong UDF_IEX_trap;
};

struct target_zol_struct {
    abi_ulong nds32_lc;
    abi_ulong nds32_le;
    abi_ulong nds32_lb;
};

struct target_sigcontext {
    abi_ulong trap_no;
    abi_ulong error_code;
    abi_ulong oldmask;
    abi_ulong regs[26];     /* r0 .. r25 */
    abi_ulong fp;
    abi_ulong gp;
    abi_ulong lp;
    abi_ulong sp;
    abi_ulong ipc;
    abi_ulong fault_address;
    abi_ulong used_math_flag;
    struct target_fpu_struct fpu;
    struct target_zol_struct zol;
};

struct target_ucontext {
    abi_ulong tuc_flags;
    abi_ulong tuc_link;
    target_stack_t tuc_stack;
    struct target_sigcontext tuc_mcontext;
    target_sigset_t tuc_sigmask;
};

struct target_rt_sigframe {
    struct target_siginfo info;
    struct target_ucontext uc;
};

static void setup_sigcontext(struct target_sigcontext *sc,
                             CPUNDS32State *env, abi_ulong mask)
{
    int i;

    __put_user(0, &sc->trap_no);
    __put_user(0, &sc->error_code);
    __put_user(mask, &sc->oldmask);
    for (i = 0; i < 26; i++) {
        __put_user(env->gpr[i], &sc->regs[i]);
    }
    __put_user(env->gpr[R_FP], &sc->fp);
    __put_user(env->gpr[R_GP], &sc->gp);
    __put_user(env->gpr[R_LP], &sc->lp);
    __put_user(env->gpr[R_SP], &sc->sp);
    __put_user(env->pc, &sc->ipc);
    __put_user(0, &sc->fault_address);
    /* No FPU and no ZOL: the kernel config leaves both out. */
    __put_user(0, &sc->used_math_flag);
}

static void restore_sigcontext(CPUNDS32State *env,
                               struct target_sigcontext *sc)
{
    int i;

    for (i = 0; i < 26; i++) {
        __get_user(env->gpr[i], &sc->regs[i]);
    }
    __get_user(env->gpr[R_FP], &sc->fp);
    __get_user(env->gpr[R_GP], &sc->gp);
    __get_user(env->gpr[R_LP], &sc->lp);
    __get_user(env->gpr[R_SP], &sc->sp);
    __get_user(env->pc, &sc->ipc);
}

static abi_ulong get_sigframe(struct target_sigaction *ka,
                              CPUNDS32State *env, size_t frame_size)
{
    abi_ulong sp = target_sigsp(get_sp_from_cpustate(env), ka);

    return (sp - frame_size) & ~7;
}

void setup_rt_frame(int sig, struct target_sigaction *ka,
                    target_siginfo_t *info,
                    target_sigset_t *set, CPUNDS32State *env)
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

    __put_user(0, &frame->uc.tuc_flags);
    __put_user(0, &frame->uc.tuc_link);
    target_save_altstack(&frame->uc.tuc_stack, env);
    setup_sigcontext(&frame->uc.tuc_mcontext, env, set->sig[0]);
    for (i = 0; i < TARGET_NSIG_WORDS; i++) {
        __put_user(set->sig[i], &frame->uc.tuc_sigmask.sig[i]);
    }
    if (ka->sa_flags & TARGET_SA_SIGINFO) {
        frame->info = *info;
        env->gpr[1] = frame_addr + offsetof(struct target_rt_sigframe, info);
        env->gpr[2] = frame_addr + offsetof(struct target_rt_sigframe, uc);
    }
    unlock_user_struct(frame, frame_addr, 1);

    env->gpr[0] = sig;
    env->gpr[R_SP] = frame_addr;
    env->gpr[R_LP] = default_rt_sigreturn;
    env->pc = ka->_sa_handler;
}

long do_rt_sigreturn(CPUNDS32State *env)
{
    abi_ulong frame_addr = env->gpr[R_SP];
    struct target_rt_sigframe *frame;
    sigset_t set;

    trace_user_do_rt_sigreturn(env, frame_addr);
    if (frame_addr & 7) {
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
    /*
     * The vdso's __kernel_rt_sigreturn: movi $r15, nr; syscall nr.
     * gcc's unwinder matches these words, which are stored big-endian.
     */
    static const uint8_t insn[8] = {
        0x44, 0xf0, 0x00, TARGET_NR_rt_sigreturn,
        0x64, 0x00, (TARGET_NR_rt_sigreturn >> 3) & 0xff,
        ((TARGET_NR_rt_sigreturn & 7) << 5) | 0x0b,
    };
    uint8_t *tramp = lock_user(VERIFY_WRITE, sigtramp_page, 8, 0);

    assert(tramp != NULL);
    QEMU_BUILD_BUG_ON(TARGET_NR_rt_sigreturn > 0xff);
    memcpy(tramp, insn, sizeof(insn));
    default_rt_sigreturn = sigtramp_page;
    unlock_user(tramp, sigtramp_page, 8);
}
