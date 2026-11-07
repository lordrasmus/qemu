/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef H8300_TARGET_PTRACE_H
#define H8300_TARGET_PTRACE_H

/* arch/h8300/include/uapi/asm/ptrace.h, H8/300H */
struct target_pt_regs {
    abi_long retpc;
    abi_long er4;
    abi_long er5;
    abi_long er6;
    abi_long er3;
    abi_long er2;
    abi_long er1;
    abi_long orig_er0;
    abi_long sp;
    uint16_t ccr;
    abi_long er0;
    abi_long vector;
    abi_ulong pc;
} QEMU_PACKED;

#endif
