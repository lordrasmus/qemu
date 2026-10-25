/*
 * META cpu parameters for qemu.
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#ifndef META_CPU_PARAM_H
#define META_CPU_PARAM_H

#define TARGET_LONG_BITS 32
/* META1 only uses 4k pages, on META2 they vary but are often 4k */
#define TARGET_PAGE_BITS 12
#define TARGET_PHYS_ADDR_SPACE_BITS 32
#define TARGET_VIRT_ADDR_SPACE_BITS 32

#endif
