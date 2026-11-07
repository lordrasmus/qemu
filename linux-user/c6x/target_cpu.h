/*
 * TI C6x specific CPU ABI and functions for linux-user
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */
#ifndef C6X_TARGET_CPU_H
#define C6X_TARGET_CPU_H

static inline void cpu_clone_regs_child(CPUC6xState *env, target_ulong newsp,
                                        unsigned flags)
{
    if (newsp) {
        env->gpr[REG_B(15)] = newsp;
    }
    env->gpr[REG_A(4)] = 0;
}

static inline void cpu_clone_regs_parent(CPUC6xState *env, unsigned flags)
{
}

/*
 * The thread pointer lives in GPLYB, which the uclibc-ng kernel patch
 * kernel-c6x-tls.patch switches per thread and sets from CLONE_SETTLS.
 */
static inline void cpu_set_tls(CPUC6xState *env, target_ulong newtls)
{
    env->gplyb = newtls;
}

static inline abi_ulong get_sp_from_cpustate(CPUC6xState *state)
{
    return state->gpr[REG_B(15)];
}
#endif
