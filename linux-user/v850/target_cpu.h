/*
 * NEC V850 specific CPU ABI and functions for linux-user
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */
#ifndef V850_TARGET_CPU_H
#define V850_TARGET_CPU_H

static inline void cpu_clone_regs_child(CPUV850State *env, target_ulong newsp,
                                        unsigned flags)
{
    if (newsp) {
        env->gpr[R_SP] = newsp;
    }
    env->gpr[10] = 0;
}

static inline void cpu_clone_regs_parent(CPUV850State *env, unsigned flags)
{
}

/* Linux/v850 has no TLS. */
static inline void cpu_set_tls(CPUV850State *env, target_ulong newtls)
{
}

static inline abi_ulong get_sp_from_cpustate(CPUV850State *state)
{
    return state->gpr[R_SP];
}
#endif
