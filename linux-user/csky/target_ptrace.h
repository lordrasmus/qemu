/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef CSKY_TARGET_PTRACE_H
#define CSKY_TARGET_PTRACE_H

/* See arch/csky/include/uapi/asm/ptrace.h, ABIV2 layout. */
struct target_pt_regs {
    abi_ulong tls;
    abi_ulong lr;
    abi_ulong pc;
    abi_ulong sr;
    abi_ulong usp;
    abi_ulong orig_a0;
    abi_ulong a0;
    abi_ulong a1;
    abi_ulong a2;
    abi_ulong a3;
    abi_ulong regs[10];         /* r4 - r13 */
    abi_ulong exregs[15];       /* r16 - r30 */
    abi_ulong rhi;
    abi_ulong rlo;
    abi_ulong dcsr;
};

struct target_user_fp {
    abi_ulong vr[96];
    abi_ulong fcr;
    abi_ulong fesr;
    abi_ulong fid;
    abi_ulong reserved;
};

#endif /* CSKY_TARGET_PTRACE_H */
