/*
 * Kalray kvx specific CPU ABI and functions for linux-user
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef KVX_TARGET_CPU_H
#define KVX_TARGET_CPU_H

/*
 * Addresses are sign-extended from 41 bits and the kernel's TASK_SIZE is
 * 1 << 39 (arch/kvx/include/asm/processor.h), so keep the guest there.
 */
#define MAX_RESERVED_VA(CPU)            ((1ul << 39) - 1)
#define TARGET_RESERVE_VA_BY_DEFAULT    1

static inline uint64_t *kvx_gpr(CPUKVXState *env, int n)
{
    return kvx_register_ptr_u64(env, REG_kv3_R0 + n);
}

static inline void cpu_clone_regs_child(CPUKVXState *env, target_ulong newsp,
                                        unsigned flags)
{
    if (newsp) {
        *kvx_gpr(env, 12) = newsp;
    }
    *kvx_gpr(env, 0) = 0;
}

static inline void cpu_clone_regs_parent(CPUKVXState *env, unsigned flags)
{
}

/* The thread pointer is $r13 (copy_thread_tls). */
static inline void cpu_set_tls(CPUKVXState *env, target_ulong newtls)
{
    *kvx_gpr(env, 13) = newtls;
}

static inline abi_ulong get_sp_from_cpustate(CPUKVXState *state)
{
    return *kvx_gpr(state, 12);
}
#endif
