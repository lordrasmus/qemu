/*
 * arch/arc/include/asm/processor.h:
 * TASK_SIZE               0x60000000
 * TASK_UNMAPPED_BASE      (TASK_SIZE / 3)
 */
#define TASK_UNMAPPED_BASE      0x20000000

/* arch/arc/include/asm/elf.h: 2 * TASK_SIZE / 3 */
#define ELF_ET_DYN_BASE         0x40000000

#include "../generic/target_mman.h"
