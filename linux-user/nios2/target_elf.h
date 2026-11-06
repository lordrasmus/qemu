/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation, or (at your option) any
 * later version. See the COPYING file in the top-level directory.
 */

#ifndef NIOS2_TARGET_ELF_H
#define NIOS2_TARGET_ELF_H

#define ELF_CLASS               ELFCLASS32
#define ELF_MACHINE             EM_ALTERA_NIOS2

#define HAVE_ELF_CORE_DUMP      1

/* arch/nios2/include/uapi/asm/elf.h: ELF_NGREG 49 */
typedef struct target_elf_gregset_t {
    abi_ulong regs[49];
} target_elf_gregset_t;

/* The kuser helper page, KUSER_BASE in arch/nios2/include/asm/processor.h */
#define COMMPAGE                0x1000

#endif
