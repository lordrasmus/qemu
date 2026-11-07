/*
 * v850 has no MMU; these only place the guest mappings.
 * include/asm-v850/processor.h: TASK_SIZE 0xffffffff
 */
#define TASK_UNMAPPED_BASE      0x40000000
#define ELF_ET_DYN_BASE         0x08000000

#include "../generic/target_mman.h"
