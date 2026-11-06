/*
 * arch/tile/include/asm/processor.h:
 * TASK_SIZE               (1UL << 40), half of the 42-bit VA space
 * TASK_UNMAPPED_BASE      (PAGE_ALIGN(TASK_SIZE / 3))
 */
#define TASK_UNMAPPED_BASE      TARGET_PAGE_ALIGN((1ULL << 40) / 3)

/* arch/tile/include/asm/elf.h */
#define ELF_ET_DYN_BASE         (TASK_UNMAPPED_BASE * 2)

#include "../generic/target_mman.h"
