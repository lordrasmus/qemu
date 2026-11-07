/*
 * arch/nds32/include/asm/memory.h, with the default PAGE_OFFSET 0xc0000000:
 * TASK_SIZE            PAGE_OFFSET - SZ_32M = 0xbe000000
 * TASK_UNMAPPED_BASE   ALIGN(TASK_SIZE / 3, SZ_32M)
 */
#define TASK_UNMAPPED_BASE      0x40000000

/* arch/nds32/include/asm/elf.h: 2 * TASK_SIZE / 3 */
#define ELF_ET_DYN_BASE         0x7eaaa000

#include "../generic/target_mman.h"
