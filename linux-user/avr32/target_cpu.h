/*
 * AVR32 specific CPU ABI and functions for linux-user
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */
#ifndef AVR32_TARGET_CPU_H
#define AVR32_TARGET_CPU_H

static inline void cpu_clone_regs_child(CPUAVR32State *env, target_ulong newsp,
                                        unsigned flags)
{
    if (newsp) {
        env->r[AVR32_SP_REG] = newsp;
    }
    env->r[12] = 0;
}

static inline void cpu_clone_regs_parent(CPUAVR32State *env, unsigned flags)
{
}

/* The avr32 kernel has no TLS (copy_thread ignores CLONE_SETTLS) */
static inline void cpu_set_tls(CPUAVR32State *env, target_ulong newtls)
{
}

static inline abi_ulong get_sp_from_cpustate(CPUAVR32State *state)
{
    return state->r[AVR32_SP_REG];
}
#endif
