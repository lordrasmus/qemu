/*
 * ia64 specific CPU ABI and functions for linux-user
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */
#ifndef IA64_TARGET_CPU_H
#define IA64_TARGET_CPU_H

/* Output register n of the current frame (a syscall argument). */
static inline uint64_t *ia64_out_reg(CPUIA64State *env, int n)
{
    int i = FIELD_EX64(env->cfm, CFM, SOL) + n;
    int rot = FIELD_EX64(env->cfm, CFM, SOR) * 8;

    if (i < rot) {
        i = (i + FIELD_EX64(env->cfm, CFM, RRB_GR)) % rot;
    }
    return &env->gr[32 + i];
}

/*
 * arch/ia64/kernel/process.c, copy_thread(): a new stack gets the register
 * backing store at its base and sp 16 below its top; the parent frames are
 * left behind, the current one is kept.  clone passes the stack top only
 * and the kernel takes 16 for the size.
 */
static inline void cpu_clone_regs_child(CPUIA64State *env, target_ulong newsp,
                                        unsigned flags)
{
    if (newsp) {
        uint64_t size = 16;

        if (env->gr[15] == TARGET_NR_clone2) {
            size = *ia64_out_reg(env, 2);
        }
        env->gr[12] = newsp + size - 16;
        env->ar[AR_BSPSTORE] = newsp;
        env->ar[AR_RNAT] = 0;
        env->rse_ndirty = 0;
    }
    env->gr[8] = 0;
    env->gr[10] = 0;
}

static inline void cpu_clone_regs_parent(CPUIA64State *env, unsigned flags)
{
}

static inline void cpu_set_tls(CPUIA64State *env, target_ulong newtls)
{
    env->gr[13] = newtls;
}

static inline target_ulong cpu_get_tls(CPUIA64State *env)
{
    return env->gr[13];
}

static inline abi_ulong get_sp_from_cpustate(CPUIA64State *state)
{
    return state->gr[12];
}
#endif
