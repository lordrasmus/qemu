/*
 * LatticeMico32 specific CPU ABI and functions for linux-user
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */
#ifndef LM32_TARGET_CPU_H
#define LM32_TARGET_CPU_H

static inline void cpu_clone_regs_child(CPULM32State *env, target_ulong newsp,
                                        unsigned flags)
{
    if (newsp) {
        env->regs[R_SP] = newsp;
    }
    env->regs[R_R1] = 0;
}

static inline void cpu_clone_regs_parent(CPULM32State *env, unsigned flags)
{
}

/* The lm32 kernel has no TLS support. */
static inline void cpu_set_tls(CPULM32State *env, target_ulong newtls)
{
}

static inline abi_ulong get_sp_from_cpustate(CPULM32State *state)
{
    return state->regs[R_SP];
}
#endif
