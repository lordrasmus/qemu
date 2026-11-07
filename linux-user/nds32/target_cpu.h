/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * NDS32 specific CPU ABI and functions for linux-user
 */

#ifndef NDS32_TARGET_CPU_H
#define NDS32_TARGET_CPU_H

static inline void cpu_clone_regs_child(CPUNDS32State *env, target_ulong newsp,
                                        unsigned flags)
{
    if (newsp) {
        env->gpr[R_SP] = newsp;
    }
    env->gpr[0] = 0;
}

static inline void cpu_clone_regs_parent(CPUNDS32State *env, unsigned flags)
{
}

/* The thread pointer is $r25 (arch/nds32/kernel/process.c). */
static inline void cpu_set_tls(CPUNDS32State *env, target_ulong newtls)
{
    env->gpr[25] = newtls;
}

static inline abi_ulong get_sp_from_cpustate(CPUNDS32State *state)
{
    return state->gpr[R_SP];
}
#endif
