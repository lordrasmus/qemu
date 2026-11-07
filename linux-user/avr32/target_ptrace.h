/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef AVR32_TARGET_PTRACE_H
#define AVR32_TARGET_PTRACE_H

/* arch/avr32/include/uapi/asm/ptrace.h */
struct target_pt_regs {
    abi_ulong sr;
    abi_ulong pc;
    abi_ulong lr;
    abi_ulong sp;
    abi_ulong r12, r11, r10, r9, r8, r7, r6, r5, r4, r3, r2, r1, r0;
    abi_ulong r12_orig;
};

#endif
