/* arch/kvx/include/asm/processor.h: TASK_SIZE is 1 << 39 */
#define TASK_UNMAPPED_BASE      TARGET_PAGE_ALIGN(1ull << 38)

/* arch/kvx/include/asm/elf.h */
#define ELF_ET_DYN_BASE         (((1ull << 39) / 3) * 2)

#include "../generic/target_mman.h"
