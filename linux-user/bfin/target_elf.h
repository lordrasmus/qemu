/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation, or (at your option) any
 * later version. See the COPYING file in the top-level directory.
 */

#ifndef BFIN_TARGET_ELF_H
#define BFIN_TARGET_ELF_H

#define ELF_CLASS               ELFCLASS32
#define ELF_MACHINE             EM_BLACKFIN

/*
 * The kernel's fixed code (sigreturn stub, atomic sequences) lives at
 * 0x400 in page zero, see elfload.c.
 */
#define COMMPAGE                0

#endif
