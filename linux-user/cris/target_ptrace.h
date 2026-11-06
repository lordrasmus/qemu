/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef CRIS_TARGET_PTRACE_H
#define CRIS_TARGET_PTRACE_H

/* arch/cris/include/uapi/arch-v10/asm/ptrace.h, also the sigcontext frame */
struct target_pt_regs {
    abi_ulong orig_r10;
    abi_ulong r0, r1, r2, r3, r4, r5, r6, r7;
    abi_ulong r8, r9, r10, r11, r12, r13;
    abi_ulong acr;
    abi_ulong srs;
    abi_ulong mof;
    abi_ulong spc;
    abi_ulong ccs;
    abi_ulong srp;
    abi_ulong erp;      /* the PC */
    abi_ulong exs;
    abi_ulong eda;
};

#endif
