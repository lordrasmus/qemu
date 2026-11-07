/*
 * arch/metag/include/asm/processor.h, page.h (CONFIG_PAGE_OFFSET 0x40000000):
 * TASK_UNMAPPED_BASE      META_MEMORY_BASE = LINLOCAL_BASE
 */
#define TASK_UNMAPPED_BASE      0x08000000

/* arch/metag/include/asm/elf.h */
#define ELF_ET_DYN_BASE         0x08000000

#include "../generic/target_mman.h"
