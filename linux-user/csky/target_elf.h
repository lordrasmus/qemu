/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation, or (at your option) any
 * later version. See the COPYING file in the top-level directory.
 */

#ifndef CSKY_TARGET_ELF_H
#define CSKY_TARGET_ELF_H

#include "target_ptrace.h"

#define ELF_MACHINE             EM_CSKY
#define ELF_CLASS               ELFCLASS32

#define HAVE_ELF_CORE_DUMP      1

/* arch/csky/include/asm/elf.h: ELF_NGREG covers struct pt_regs. */
typedef struct target_elf_gregset_t {
    struct target_pt_regs pt;
} target_elf_gregset_t;

#endif
