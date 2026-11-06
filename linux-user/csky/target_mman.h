/*
 * arch/csky/include/asm/processor.h:
 * TASK_SIZE               0x7fff8000UL
 * TASK_UNMAPPED_BASE      (TASK_SIZE / 3)
 */
#define TASK_UNMAPPED_BASE      0x2aaa8000

/* the kernel leaves ET_DYN placement to mmap (ELF_ET_DYN_BASE 0) */
#define ELF_ET_DYN_BASE         0x2aaa8000

#include "../generic/target_mman.h"
