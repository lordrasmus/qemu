/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef TILEGX_TARGET_PTRACE_H
#define TILEGX_TARGET_PTRACE_H

/* arch/tile/include/uapi/asm/ptrace.h, also the sigcontext layout */
struct target_pt_regs {
    abi_ulong regs[56];         /* r53 tp, r54 sp, r55 lr */
    abi_ulong pc;
    abi_ulong ex1;              /* sigcontext: ics */
    abi_ulong faultnum;
    abi_ulong orig_r0;
    abi_ulong flags;
    abi_ulong cmpexch;
    abi_ulong pad[2];
};

#endif
