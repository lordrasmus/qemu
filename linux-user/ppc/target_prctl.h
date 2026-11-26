/*
 * PowerPC specific prctl functions for linux-user
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef PPC_TARGET_PRCTL_H
#define PPC_TARGET_PRCTL_H

/*
 * PR_GET_FPEXC/PR_SET_FPEXC for SPE (e500): the C library keeps the
 * exception mode there (fesetenv, feenableexcept).  Only the mode is
 * kept; with no kernel emulation behind it no SIGFPE is sent.
 */
#define PPC_PR_FP_ALL_EXCEPT (PR_FP_EXC_DIV | PR_FP_EXC_OVF | PR_FP_EXC_UND | \
                              PR_FP_EXC_RES | PR_FP_EXC_INV)

static abi_long do_prctl_get_fpexc(CPUArchState *env, abi_long arg2)
{
    if (!(env->insns_flags & PPC_SPE)) {
        return -TARGET_EINVAL;
    }
    if (put_user_u32(env->spe_fpexc_mode, arg2)) {
        return -TARGET_EFAULT;
    }
    return 0;
}
#define do_prctl_get_fpexc do_prctl_get_fpexc

static abi_long do_prctl_set_fpexc(CPUArchState *env, abi_long arg2)
{
    if (!(env->insns_flags & PPC_SPE)) {
        return -TARGET_EINVAL;
    }
    if (arg2 & PR_FP_EXC_SW_ENABLE) {
        env->spe_fpexc_mode = arg2 & (PR_FP_EXC_SW_ENABLE |
                                      PPC_PR_FP_ALL_EXCEPT);
    } else if (arg2 > PR_FP_EXC_PRECISE) {
        return -TARGET_EINVAL;
    } else {
        env->spe_fpexc_mode = arg2;
    }
    return 0;
}
#define do_prctl_set_fpexc do_prctl_set_fpexc

#endif
