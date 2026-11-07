/*
 * arch/avr32/include/asm/processor.h:
 * TASK_SIZE               0x80000000
 * TASK_UNMAPPED_BASE      (PAGE_ALIGN(TASK_SIZE / 3))
 *
 * arch/avr32/include/asm/elf.h:
 * ELF_ET_DYN_BASE         (TASK_SIZE / 3 * 2)
 */
#define TASK_UNMAPPED_BASE TARGET_PAGE_ALIGN(0x80000000 / 3)
#define ELF_ET_DYN_BASE    (0x80000000 / 3 * 2)

#include "../generic/target_mman.h"
