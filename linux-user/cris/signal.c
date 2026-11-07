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

/*
 * struct sigcontext is struct pt_regs followed by oldmask and usp, and
 * pt_regs differs between v10 (23 words) and v32 (24 words), see
 * arch/cris/include/uapi/asm/ptrace_v{10,32}.h.  Keep it as words.
 */
#define SC_MAX_REGS     24

struct target_sigcontext {
    abi_ulong w[SC_MAX_REGS + 2];
};

struct target_ucontext {
    abi_ulong tuc_flags;
    abi_ulong tuc_link;
    target_stack_t tuc_stack;
    struct target_sigcontext tuc_mcontext;
    target_sigset_t tuc_sigmask;
};

struct target_rt_sigframe {
    abi_ulong pinfo;
    abi_ulong puc;
    target_siginfo_t info;
    struct target_ucontext uc;
    uint16_t retcode[4];
};

struct target_signal_frame {
    struct target_sigcontext sc;
    abi_ulong extramask[TARGET_NSIG_WORDS - 1];
    uint16_t retcode[4];
};

void helper_top_evaluate_flags(CPUCRISState *env);

static bool is_v32(CPUCRISState *env)
{
    return env->pregs[PR_VR] >= 32;
}

static int sc_nregs(CPUCRISState *env)
{
    return is_v32(env) ? 24 : 23;
}

/* Word index of rN (0..13) in pt_regs */
static int sc_gpr(CPUCRISState *env, int n)
{
    return is_v32(env) ? 1 + n : 15 - n;
}

#define SC_MOF(env)   (is_v32(env) ? 17 : 16)
#define SC_CCS(env)   (is_v32(env) ? 19 : 17)
#define SC_SRP(env)   (is_v32(env) ? 20 : 18)
#define SC_PC(env)    (is_v32(env) ? 21 : 19)
#define SC_ACR        15
#define SC_OLDMASK(env) (sc_nregs(env))
#define SC_USP(env)     (sc_nregs(env) + 1)

/* Offset of oldmask, i.e. the size of the frame up to extramask */
static abi_ulong sc_size(CPUCRISState *env)
{
    return (sc_nregs(env) + 2) * 4;
}

static void setup_sigcontext(abi_ulong *w, CPUCRISState *env, abi_ulong mask)
{
    helper_top_evaluate_flags(env);
    env->cc_op = CC_OP_FLAGS;
    /*
     * Interrupted in a delay slot: restart at the branch, as the CPU
     * does for an exception (ERP -= dslot).
     */
    env->pc -= env->dslot;
    env->dslot = 0;

    for (int i = 0; i < sc_nregs(env) + 2; i++) {
        __put_user(0, &w[i]);
    }
    for (int i = 0; i < 14; i++) {
        __put_user(env->regs[i], &w[sc_gpr(env, i)]);
    }
    if (is_v32(env)) {
        __put_user(env->regs[15], &w[SC_ACR]);
    }
    __put_user(env->pregs[PR_MOF], &w[SC_MOF(env)]);
    __put_user(env->pregs[PR_CCS], &w[SC_CCS(env)]);
    __put_user(env->pregs[PR_SRP], &w[SC_SRP(env)]);
    __put_user(env->pc, &w[SC_PC(env)]);
    __put_user(mask, &w[SC_OLDMASK(env)]);
    __put_user(env->regs[R_SP], &w[SC_USP(env)]);
}

static void restore_sigcontext(abi_ulong *w, CPUCRISState *env)
{
    abi_ulong ccs;

    for (int i = 0; i < 14; i++) {
        __get_user(env->regs[i], &w[sc_gpr(env, i)]);
    }
    if (is_v32(env)) {
        __get_user(env->regs[15], &w[SC_ACR]);
    }
    __get_user(env->pregs[PR_MOF], &w[SC_MOF(env)]);
    __get_user(env->pregs[PR_SRP], &w[SC_SRP(env)]);
    __get_user(env->pc, &w[SC_PC(env)]);
    __get_user(env->regs[R_SP], &w[SC_USP(env)]);
    /* Only the condition codes come back, as in the kernel */
    __get_user(ccs, &w[SC_CCS(env)]);
    env->pregs[PR_CCS] = (env->pregs[PR_CCS] & ~0xff) | (ccs & 0xff);
    env->cc_op = CC_OP_FLAGS;
}

static abi_ulong get_sigframe(struct target_sigaction *ka,
                              CPUCRISState *env, int framesize)
{
    abi_ulong sp = target_sigsp(get_sp_from_cpustate(env), ka);

    return (sp & ~3) - framesize;
}

/* movu.w nr, r9; break 13 */
static void setup_sigreturn(uint16_t *retcode, int nr)
{
    __put_user(0x9c5f, retcode + 0);
    __put_user(nr, retcode + 1);
    __put_user(0xe93d, retcode + 2);
}

void setup_frame(int sig, struct target_sigaction *ka,
                 target_sigset_t *set, CPUCRISState *env)
{
    abi_ulong frame_addr, size = sc_size(env) + sizeof(abi_ulong) *
                                 (TARGET_NSIG_WORDS - 1) + 8;
    abi_ulong *frame;
    uint16_t *retcode;

    frame_addr = get_sigframe(ka, env, size);
    trace_user_setup_frame(env, frame_addr);
    frame = lock_user(VERIFY_WRITE, frame_addr, size, 0);
    if (!frame) {
        force_sigsegv(sig);
        return;
    }

    setup_sigcontext(frame, env, set->sig[0]);
    for (int i = 1; i < TARGET_NSIG_WORDS; i++) {
        __put_user(set->sig[i], &frame[sc_nregs(env) + 2 + i - 1]);
    }
    retcode = (uint16_t *)&frame[sc_nregs(env) + 2 + TARGET_NSIG_WORDS - 1];
    setup_sigreturn(retcode, TARGET_NR_sigreturn);

    env->regs[R_SP] = frame_addr;
    env->regs[10] = sig;
    env->pc = ka->_sa_handler;
    if (ka->sa_flags & TARGET_SA_RESTORER) {
        env->pregs[PR_SRP] = ka->sa_restorer;
    } else {
        env->pregs[PR_SRP] = default_sigreturn;
    }
    unlock_user(frame, frame_addr, size);
}

/*
 * The ucontext ends with uc_sigmask after the variable-size mcontext, so
 * build the rt frame by offsets.
 */
void setup_rt_frame(int sig, struct target_sigaction *ka,
                    target_siginfo_t *info,
                    target_sigset_t *set, CPUCRISState *env)
{
    abi_ulong info_off = 8;
    abi_ulong uc_off = info_off + sizeof(target_siginfo_t);
    abi_ulong mc_off = uc_off + 8 + sizeof(target_stack_t);
    abi_ulong mask_off = mc_off + sc_size(env);
    abi_ulong ret_off = mask_off + sizeof(target_sigset_t);
    abi_ulong size = ret_off + 8;
    abi_ulong frame_addr;
    uint8_t *frame;

    frame_addr = get_sigframe(ka, env, size);
    trace_user_setup_rt_frame(env, frame_addr);
    frame = lock_user(VERIFY_WRITE, frame_addr, size, 0);
    if (!frame) {
        force_sigsegv(sig);
        return;
    }

    __put_user(frame_addr + info_off, (abi_ulong *)frame);
    __put_user(frame_addr + uc_off, (abi_ulong *)(frame + 4));
    memcpy(frame + info_off, info, sizeof(*info));

    __put_user(0, (abi_ulong *)(frame + uc_off));
    __put_user(0, (abi_ulong *)(frame + uc_off + 4));
    target_save_altstack((target_stack_t *)(frame + uc_off + 8), env);
    setup_sigcontext((abi_ulong *)(frame + mc_off), env, set->sig[0]);
    for (int i = 0; i < TARGET_NSIG_WORDS; i++) {
        __put_user(set->sig[i], (abi_ulong *)(frame + mask_off) + i);
    }
    setup_sigreturn((uint16_t *)(frame + ret_off), TARGET_NR_rt_sigreturn);

    env->regs[R_SP] = frame_addr;
    env->regs[10] = sig;
    env->regs[11] = frame_addr + info_off;
    env->regs[12] = frame_addr + uc_off;
    env->pc = ka->_sa_handler;
    if (ka->sa_flags & TARGET_SA_RESTORER) {
        env->pregs[PR_SRP] = ka->sa_restorer;
    } else {
        env->pregs[PR_SRP] = default_rt_sigreturn;
    }
    unlock_user(frame, frame_addr, size);
}

long do_sigreturn(CPUCRISState *env)
{
    abi_ulong size = sc_size(env) + sizeof(abi_ulong) * (TARGET_NSIG_WORDS - 1);
    abi_ulong frame_addr = env->regs[R_SP];
    target_sigset_t target_set;
    sigset_t set;
    abi_ulong *frame;

    trace_user_do_sigreturn(env, frame_addr);
    frame = lock_user(VERIFY_READ, frame_addr, size, 1);
    if (!frame) {
        force_sig(TARGET_SIGSEGV);
        return -QEMU_ESIGRETURN;
    }
    __get_user(target_set.sig[0], &frame[SC_OLDMASK(env)]);
    for (int i = 1; i < TARGET_NSIG_WORDS; i++) {
        __get_user(target_set.sig[i], &frame[sc_nregs(env) + 2 + i - 1]);
    }
    target_to_host_sigset_internal(&set, &target_set);
    set_sigmask(&set);
    restore_sigcontext(frame, env);
    unlock_user(frame, frame_addr, 0);
    return -QEMU_ESIGRETURN;
}

long do_rt_sigreturn(CPUCRISState *env)
{
    abi_ulong frame_addr = env->regs[R_SP];
    abi_ulong uc_off = 8 + sizeof(target_siginfo_t);
    abi_ulong mc_off = uc_off + 8 + sizeof(target_stack_t);
    abi_ulong mask_off = mc_off + sc_size(env);
    abi_ulong size = mask_off + sizeof(target_sigset_t);
    target_sigset_t target_set;
    sigset_t set;
    uint8_t *frame;

    trace_user_do_rt_sigreturn(env, frame_addr);
    frame = lock_user(VERIFY_READ, frame_addr, size, 1);
    if (!frame) {
        force_sig(TARGET_SIGSEGV);
        return -QEMU_ESIGRETURN;
    }
    for (int i = 0; i < TARGET_NSIG_WORDS; i++) {
        __get_user(target_set.sig[i], (abi_ulong *)(frame + mask_off) + i);
    }
    target_to_host_sigset(&set, &target_set);
    set_sigmask(&set);
    restore_sigcontext((abi_ulong *)(frame + mc_off), env);
    target_restore_altstack((target_stack_t *)(frame + uc_off + 8), env);
    unlock_user(frame, frame_addr, 0);
    return -QEMU_ESIGRETURN;
}

void setup_sigtramp(abi_ulong sigtramp_page)
{
    uint16_t *tramp = lock_user(VERIFY_WRITE, sigtramp_page, 12, 0);
    assert(tramp != NULL);

    default_sigreturn = sigtramp_page;
    setup_sigreturn(tramp, TARGET_NR_sigreturn);
    default_rt_sigreturn = sigtramp_page + 6;
    setup_sigreturn(tramp + 3, TARGET_NR_rt_sigreturn);

    unlock_user(tramp, sigtramp_page, 12);
}
