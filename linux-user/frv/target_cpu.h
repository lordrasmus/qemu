/*
 * FR-V specific CPU ABI and functions for linux-user
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */
#ifndef FRV_TARGET_CPU_H
#define FRV_TARGET_CPU_H

static inline void cpu_clone_regs_child(CPUFRVState *env, target_ulong newsp,
                                        unsigned flags)
{
    if (newsp) {
        env->gr[1] = newsp;
    }
    env->gr[8] = 0;
}

static inline void cpu_clone_regs_parent(CPUFRVState *env, unsigned flags)
{
}

/* CLONE_SETTLS loads gr29, the NPTL thread pointer (arch/frv/kernel/process.c) */
static inline void cpu_set_tls(CPUFRVState *env, target_ulong newtls)
{
    env->gr[29] = newtls;
}

static inline target_ulong cpu_get_tls(CPUFRVState *env)
{
    return env->gr[29];
}

static inline abi_ulong get_sp_from_cpustate(CPUFRVState *state)
{
    return state->gr[1];
}
#endif
