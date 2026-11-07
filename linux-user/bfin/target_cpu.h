/*
 * Blackfin specific CPU ABI and functions for linux-user
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef BFIN_TARGET_CPU_H
#define BFIN_TARGET_CPU_H

/* arch/blackfin/include/uapi/asm/fixed_code.h */
#define BFIN_SIGRETURN_STUB     0x400
#define BFIN_ATOMIC_SEQS_START  0x410
#define BFIN_ATOMIC_SEQS_END    0x480

static inline void cpu_clone_regs_child(CPUBfinState *env, target_ulong newsp,
                                        unsigned flags)
{
    /* bfin_clone() starts the child 12 bytes below the new stack */
    if (newsp) {
        env->reg[BFIN_SP] = newsp - 12;
    }
    env->reg[BFIN_R0] = 0;
}

static inline void cpu_clone_regs_parent(CPUBfinState *env, unsigned flags)
{
}

/* B3 holds the thread pointer (kernel-bfin-tls.patch in uclibc-ng-qemu-imgs) */
static inline void cpu_set_tls(CPUBfinState *env, target_ulong newtls)
{
    env->reg[BFIN_B0 + 3] = newtls;
}

static inline target_ulong cpu_get_tls(CPUBfinState *env)
{
    return env->reg[BFIN_B0 + 3];
}

static inline abi_ulong get_sp_from_cpustate(CPUBfinState *state)
{
    return state->reg[BFIN_SP];
}
#endif
