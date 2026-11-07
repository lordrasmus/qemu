/*
 * No MMU: the kernel hands out memory from its page allocator.  Keep
 * mappings away from page 0, inside the 16 MB the CPU can address.
 */
#define TASK_UNMAPPED_BASE      0x00100000
#define ELF_ET_DYN_BASE         0x00200000

#include "../generic/target_mman.h"
