/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef META_TARGET_PTRACE_H
#define META_TARGET_PTRACE_H

/* arch/metag/include/uapi/asm/ptrace.h */
struct target_user_gp_regs {
    abi_ulong dx[8][2];     /* D0.i, D1.i */
    abi_ulong ax[4][2];     /* A0.i, A1.i */
    abi_ulong pc;
    abi_ulong status;       /* TXSTATUS flags, LSM_STEP, CATCH bit */
    abi_ulong rpt;
    abi_ulong bpobits;
    abi_ulong mode;
    abi_ulong _pad1;
};

struct target_user_cb_regs {
    abi_ulong flags;
    abi_ulong addr;
    uint64_t data;
};

struct target_user_rp_state {
    uint64_t entries[6];
    abi_ulong mask;
};

#endif
