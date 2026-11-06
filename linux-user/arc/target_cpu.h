/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef ARC_TARGET_CPU_H
#define ARC_TARGET_CPU_H

static inline void cpu_clone_regs_child(CPUARCState *env, target_ulong newsp,
                                        unsigned flags)
{
    if (newsp) {
        CPU_SP(env) = newsp;
    }
    env->r[0] = 0;
}

static inline void cpu_clone_regs_parent(CPUARCState *env, unsigned flags)
{
}

/* Kernel keeps the pointer for arc_gettls and hands it to the child in r25. */
static inline void cpu_set_tls(CPUARCState *env, target_ulong newtls)
{
    env->tls_backup = newtls;
    env->r[25] = newtls;
}

static inline abi_ulong get_sp_from_cpustate(CPUARCState *env)
{
    return CPU_SP(env);
}
#endif
