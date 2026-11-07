/*
 * C6X Linux has no MMU and places mappings anywhere; these only need to
 * keep mmap and PIE images apart in the 32-bit address space.
 */
#define TASK_UNMAPPED_BASE      0x40000000
#define ELF_ET_DYN_BASE         0x10000000

/* no MMU, so loaded text is writable, see target_mmap() */
#define TARGET_NOMMU_PRIVATE_WRITABLE 1

#include "../generic/target_mman.h"
