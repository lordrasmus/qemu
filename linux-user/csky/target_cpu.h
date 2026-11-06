/*
 * C-SKY specific CPU ABI and functions for linux-user
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef CSKY_TARGET_CPU_H
#define CSKY_TARGET_CPU_H

static inline void cpu_clone_regs_child(CPUCSKYState *env,
                                        target_ulong newsp,
                                        unsigned flags)
{
    if (newsp) {
        env->regs[14] = newsp;
    }
    env->regs[0] = 0;
}

static inline void cpu_clone_regs_parent(CPUCSKYState *env, unsigned flags)
{
}

/* The thread pointer lives in r31; trap 3 reads the kernel's copy. */
static inline void cpu_set_tls(CPUCSKYState *env, target_ulong newtls)
{
    env->regs[31] = newtls;
    env->tls_value = newtls;
}

static inline abi_ulong get_sp_from_cpustate(CPUCSKYState *state)
{
    return state->regs[14];
}
#endif
