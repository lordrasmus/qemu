/*
 * Linux maps at 0x2000000000000000 (region 1).  The addresses QEMU picks
 * stay below 1 << 44 instead, where the guest and QEMU see the same
 * address (see ia64_user_fold).
 */
#define TASK_UNMAPPED_BASE      0x0000080000000000ull
#define ELF_ET_DYN_BASE         0x00000c0000000000ull

#include "../generic/target_mman.h"
