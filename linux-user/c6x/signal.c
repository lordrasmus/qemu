/*
 *  Emulation of Linux signals, TI C6x
 *
 *  Follows arch/c6x/kernel/signal.c of Linux 5.10.
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

/* arch/c6x/include/uapi/asm/sigcontext.h */
struct target_sigcontext {
    abi_ulong sc_mask;
    abi_ulong sc_sp;
    abi_ulong sc_a4, sc_b4, sc_a6, sc_b6, sc_a8, sc_b8;
    abi_ulong sc_a0, sc_a1, sc_a2, sc_a3, sc_a5, sc_a7, sc_a9;
    abi_ulong sc_b0, sc_b1, sc_b2, sc_b3, sc_b5, sc_b7, sc_b9;
    abi_ulong sc_a16_31[16];
    abi_ulong sc_b16_31[16];
    abi_ulong sc_csr;
    abi_ulong sc_pc;
};

struct target_ucontext {
    abi_ulong tuc_flags;
    abi_ulong tuc_link;
    target_stack_t tuc_stack;
    struct target_sigcontext tuc_mcontext;
    target_sigset_t tuc_sigmask;
};

#define RETCODE_WORDS 9

struct target_rt_sigframe {
    abi_ulong pinfo;
    abi_ulong puc;
    struct target_siginfo info;
    struct target_ucontext uc;
    abi_ulong retcode[RETCODE_WORDS];
};

/* The registers in sigcontext order; the others are callee-saved. */
static const uint8_t sc_regs[] = {
    REG_A(4), REG_B(4), REG_A(6), REG_B(6), REG_A(8), REG_B(8),
    REG_A(0), REG_A(1), REG_A(2), REG_A(3), REG_A(5), REG_A(7), REG_A(9),
    REG_B(0), REG_B(1), REG_B(2), REG_B(3), REG_B(5), REG_B(7), REG_B(9),
};

static void setup_sigcontext(struct target_sigcontext *sc, CPUC6xState *env,
                             abi_ulong mask)
{
    abi_ulong *r = &sc->sc_a4;

    __put_user(mask, &sc->sc_mask);
    __put_user(env->gpr[REG_B(15)], &sc->sc_sp);
    for (int i = 0; i < ARRAY_SIZE(sc_regs); i++) {
        __put_user(env->gpr[sc_regs[i]], &r[i]);
    }
    for (int i = 0; i < 16; i++) {
        __put_user(env->gpr[REG_A(16 + i)], &sc->sc_a16_31[i]);
        __put_user(env->gpr[REG_B(16 + i)], &sc->sc_b16_31[i]);
    }
    __put_user(c6x_csr_read(env), &sc->sc_csr);
    __put_user(env->pc, &sc->sc_pc);
}

static void restore_sigcontext(CPUC6xState *env,
                               struct target_sigcontext *sc)
{
    abi_ulong *r = &sc->sc_a4;
    abi_ulong csr;

    __get_user(env->gpr[REG_B(15)], &sc->sc_sp);
    for (int i = 0; i < ARRAY_SIZE(sc_regs); i++) {
        __get_user(env->gpr[sc_regs[i]], &r[i]);
    }
    for (int i = 0; i < 16; i++) {
        __get_user(env->gpr[REG_A(16 + i)], &sc->sc_a16_31[i]);
        __get_user(env->gpr[REG_B(16 + i)], &sc->sc_b16_31[i]);
    }
    /* only SAT is the program's; GIE stays set, see cpu_loop.c */
    __get_user(csr, &sc->sc_csr);
    env->csr = (env->csr & ~R_CSR_SAT_MASK) | (csr & R_CSR_SAT_MASK);
    __get_user(env->pc, &sc->sc_pc);
}

static void write_retcode(abi_ulong *rc)
{
    /* MVK __NR_rt_sigreturn,B0; SWE; NOP 4 (x7) */
    __put_user(0x0000002a | (TARGET_NR_rt_sigreturn << 7), rc + 0);
    __put_user(0x10000000, rc + 1);
    for (int i = 2; i < RETCODE_WORDS; i++) {
        __put_user(0x00006000, rc + i);
    }
}

void setup_rt_frame(int sig, struct target_sigaction *ka,
                    target_siginfo_t *info,
                    target_sigset_t *set, CPUC6xState *env)
{
    struct target_rt_sigframe *frame;
    abi_ulong frame_addr, sp;

    sp = target_sigsp(get_sp_from_cpustate(env), ka);
    frame_addr = (sp - sizeof(*frame)) & ~7;
    trace_user_setup_rt_frame(env, frame_addr);
    if (!lock_user_struct(VERIFY_WRITE, frame, frame_addr, 0)) {
        force_sigsegv(sig);
        return;
    }

    __put_user(frame_addr + offsetof(struct target_rt_sigframe, info),
               &frame->pinfo);
    __put_user(frame_addr + offsetof(struct target_rt_sigframe, uc),
               &frame->puc);
    frame->info = *info;

    /* the kernel clears everything before uc_mcontext, uc_stack too */
    memset(&frame->uc, 0, offsetof(struct target_ucontext, tuc_mcontext));
    setup_sigcontext(&frame->uc.tuc_mcontext, env, set->sig[0]);
    for (int i = 0; i < TARGET_NSIG_WORDS; i++) {
        __put_user(set->sig[i], &frame->uc.tuc_sigmask.sig[i]);
    }
    write_retcode(frame->retcode);
    unlock_user_struct(frame, frame_addr, 1);

    /* a synchronous signal may hit a DINT section */
    if (!FIELD_EX32(env->tsr, TSR, GIE)) {
        env->tsr = FIELD_DP32(env->tsr, TSR, GIE, 1);
        c6x_user_unlock(env);
    }

    env->gpr[REG_B(15)] = frame_addr - 8;
    env->gpr[REG_B(3)] = default_rt_sigreturn;
    env->gpr[REG_A(4)] = sig;
    env->gpr[REG_B(4)] = frame_addr + offsetof(struct target_rt_sigframe, info);
    env->gpr[REG_A(6)] = frame_addr + offsetof(struct target_rt_sigframe, uc);
    env->pc = ka->_sa_handler;
}

long do_rt_sigreturn(CPUC6xState *env)
{
    abi_ulong sp = get_sp_from_cpustate(env);
    abi_ulong frame_addr = sp + 8;
    struct target_rt_sigframe *frame;
    sigset_t set;

    trace_user_do_rt_sigreturn(env, frame_addr);
    if (sp & 7) {
        goto badframe;
    }
    if (!lock_user_struct(VERIFY_READ, frame, frame_addr, 1)) {
        goto badframe;
    }

    target_to_host_sigset(&set, &frame->uc.tuc_sigmask);
    set_sigmask(&set);
    restore_sigcontext(env, &frame->uc.tuc_mcontext);

    unlock_user_struct(frame, frame_addr, 0);
    return -QEMU_ESIGRETURN;

badframe:
    force_sig(TARGET_SIGSEGV);
    return -QEMU_ESIGRETURN;
}

void setup_sigtramp(abi_ulong sigtramp_page)
{
    abi_ulong *tramp = lock_user(VERIFY_WRITE, sigtramp_page,
                                 RETCODE_WORDS * 4, 0);
    assert(tramp != NULL);

    write_retcode(tramp);
    default_rt_sigreturn = sigtramp_page;
    unlock_user(tramp, sigtramp_page, RETCODE_WORDS * 4);
}
