/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef ARC_TARGET_PTRACE_H
#define ARC_TARGET_PTRACE_H

/* arch/arc/include/uapi/asm/ptrace.h */
struct target_user_regs_struct {
    abi_ulong pad;
    struct {
        abi_ulong bta, lp_start, lp_end, lp_count;
        abi_ulong status32, ret, blink, fp, gp;
        abi_ulong r12, r11, r10, r9, r8, r7, r6, r5, r4, r3, r2, r1, r0;
        abi_ulong sp;
    } scratch;
    abi_ulong pad2;
    struct {
        abi_ulong r25, r24, r23, r22, r21, r20;
        abi_ulong r19, r18, r17, r16, r15, r14, r13;
    } callee;
    abi_ulong efa;
    abi_ulong stop_pc;
};

struct target_user_regs_arcv2 {
    abi_ulong r30, r58, r59;
};

#endif
