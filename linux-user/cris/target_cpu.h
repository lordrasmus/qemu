/*
 * CRIS specific CPU ABI and functions for linux-user
 *
 * Copyright (c) 2007 AXIS Communications AB
 * Written by Edgar E. Iglesias
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */
#ifndef CRIS_TARGET_CPU_H
#define CRIS_TARGET_CPU_H

static inline void cpu_clone_regs_child(CPUCRISState *env, target_ulong newsp,
                                        unsigned flags)
{
    if (newsp) {
        env->regs[14] = newsp;
    }
    env->regs[10] = 0;
}

static inline void cpu_clone_regs_parent(CPUCRISState *env, unsigned flags)
{
}

/*
 * v32 keeps the thread pointer in the upper 24 bits of $pid, v10 has no
 * register for it and reserves r6 (kernel-cris-tls.patch in uclibc-ng-qemu-imgs).
 */
static inline void cpu_set_tls(CPUCRISState *env, target_ulong newtls)
{
    if (env->pregs[PR_VR] >= 32) {
        env->pregs[PR_PID] = (env->pregs[PR_PID] & 0xff) | newtls;
    } else {
        env->regs[6] = newtls;
    }
}

static inline target_ulong cpu_get_tls(CPUCRISState *env)
{
    if (env->pregs[PR_VR] >= 32) {
        return env->pregs[PR_PID] & ~0xff;
    }
    return env->regs[6];
}

static inline abi_ulong get_sp_from_cpustate(CPUCRISState *state)
{
    return state->regs[14];
}
#endif
