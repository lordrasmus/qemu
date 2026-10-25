/*
 * TILE-Gx cpu parameters for qemu.
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#ifndef TILEGX_CPU_PARAM_H
#define TILEGX_CPU_PARAM_H

/* The smallest page the hypervisor hands out is 4 KiB (HV_CTX_PG_SM_4K). */
#define TARGET_PAGE_BITS 12
/* 42-bit VAs, sign-extended: the kernel lives at the top of the space. */
#define TARGET_VIRT_ADDR_SPACE_BITS 64

#endif
