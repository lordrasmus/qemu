/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation, or (at your option) any
 * later version. See the COPYING file in the top-level directory.
 */

#ifndef IA64_TARGET_ELF_H
#define IA64_TARGET_ELF_H

#define ELF_CLASS               ELFCLASS64
#define ELF_MACHINE             EM_IA_64

/* The regions of the image are folded into QEMU's address space. */
#define ELF_VADDR(va)           ia64_user_fold(va)

#endif
