/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef V850_TARGET_PTRACE_H
#define V850_TARGET_PTRACE_H

/* include/asm-v850/ptrace.h, also the sigcontext frame */
struct target_pt_regs {
    abi_ulong gpr[32];  /* gpr[0] holds the system call number in the kernel */
    abi_ulong pc;
    abi_ulong psw;
    abi_ulong ctpc;
    abi_ulong ctpsw;
    abi_ulong ctbp;
    abi_ulong kernel_mode;  /* a char, padded */
};

#endif
