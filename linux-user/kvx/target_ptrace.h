/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef KVX_TARGET_PTRACE_H
#define KVX_TARGET_PTRACE_H

/* See arch/kvx/include/uapi/asm/ptrace.h. */
struct target_user_pt_regs {
    abi_ulong r[64];
    abi_ulong lc;
    abi_ulong le;
    abi_ulong ls;
    abi_ulong ra;
    abi_ulong cs;
    abi_ulong spc;
};

#endif
