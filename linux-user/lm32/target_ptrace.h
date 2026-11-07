/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef LM32_TARGET_PTRACE_H
#define LM32_TARGET_PTRACE_H

/* arch/lm32/include/asm/ptrace.h, also the sigcontext frame */
struct target_pt_regs {
    abi_ulong r[26];    /* r0 .. r25 */
    abi_ulong gp;
    abi_ulong fp;
    abi_ulong sp;
    abi_ulong ra;
    abi_ulong ea;       /* address of the scall, the kernel returns to ea + 4 */
    abi_ulong ba;
    abi_uint pt_mode;
};

#endif
