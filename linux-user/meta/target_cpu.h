/*
 * META specific CPU ABI and functions for linux-user
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef META_TARGET_CPU_H
#define META_TARGET_CPU_H

/* arch/metag/kernel/process.c:copy_thread() */
static inline void cpu_clone_regs_child(CPUMETAState *env, target_ulong newsp,
                                        unsigned flags)
{
    if (newsp) {
        env->aregs[0][0] = QEMU_ALIGN_UP(newsp, 8);   /* A0StP */
    }
    env->dregs[0][0] = 0;                               /* D0Re0 */
}

static inline void cpu_clone_regs_parent(CPUMETAState *env, unsigned flags)
{
}

/* The kernel keeps it per thread and serves it from the gateway page. */
static inline void cpu_set_tls(CPUMETAState *env, target_ulong newtls)
{
    env->tls_value = newtls;
}

static inline target_ulong cpu_get_tls(CPUMETAState *env)
{
    return env->tls_value;
}

static inline abi_ulong get_sp_from_cpustate(CPUMETAState *state)
{
    return state->aregs[0][0];
}
#endif
