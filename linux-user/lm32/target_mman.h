/*
 * lm32 has no MMU; these only place the guest mappings.
 * arch/lm32/include/asm/processor.h: TASK_SIZE 0xf0000000
 */
#define TASK_UNMAPPED_BASE      0x40000000
#define ELF_ET_DYN_BASE         0x08000000

#include "../generic/target_mman.h"
