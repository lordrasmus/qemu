/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * H8/300 specific CPU ABI and functions for linux-user
 */

#ifndef H8300_TARGET_CPU_H
#define H8300_TARGET_CPU_H

static inline void cpu_clone_regs_child(CPUH8300State *env, target_ulong newsp,
                                        unsigned flags)
{
    if (newsp) {
        env->regs[7] = newsp;
    }
    env->regs[0] = 0;
}

static inline void cpu_clone_regs_parent(CPUH8300State *env, unsigned flags)
{
}

/* No thread pointer: Linux/h8300 has no TLS. */
static inline void cpu_set_tls(CPUH8300State *env, target_ulong newtls)
{
}

static inline abi_ulong get_sp_from_cpustate(CPUH8300State *state)
{
    return state->regs[7];
}
#endif
