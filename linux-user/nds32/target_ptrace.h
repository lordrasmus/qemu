/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef NDS32_TARGET_PTRACE_H
#define NDS32_TARGET_PTRACE_H

/* arch/nds32/include/uapi/asm/ptrace.h */
struct target_user_regs_struct {
    abi_long uregs[26];
    abi_long fp;
    abi_long gp;
    abi_long lp;
    abi_long sp;
    abi_long ipc;
    abi_long lb;
    abi_long le;
    abi_long lc;
    abi_long syscallno;
};

#endif
