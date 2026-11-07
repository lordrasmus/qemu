/*
 * Blackfin has no MMU; mappings go wherever the allocator puts them.
 * These only steer the qemu allocator.
 */
#define TASK_UNMAPPED_BASE      0x40000000
#define ELF_ET_DYN_BASE         0x10000000

#include "../generic/target_mman.h"
