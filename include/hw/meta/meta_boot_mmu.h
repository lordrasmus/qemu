#ifndef _META_BOOT_MMU_H
#define _META_BOOT_MMU_H


#include "qemu/bitops.h"

/* mmu_seg::flags */
#define MMUF_JOIN_BIG   BIT(0)  /* starts on same bigpage as previous ends */
#define MMUF_SINGLE_BIG BIT(1)  /* fits on a single bigpage */

struct meta_mmu_seg {
    /* Input */
    target_ulong vstart;
    target_ulong vend;
    hwaddr pstart;
    unsigned int seg_id;

    /* Internal use only */
    unsigned int flags;
    unsigned int lg_start_pg;
    unsigned int lg_mid_pg;
    unsigned int lg_end_pg;
};

static inline void meta_mmu_init_seg(struct meta_mmu_seg *seg,
                                     target_ulong vstart, target_ulong vend,
                                     hwaddr pstart, unsigned int seg_id)
{
    seg->vstart = (uint32_t)vstart;
    seg->vend = (uint32_t)vend;
    seg->pstart = (uint32_t)pstart;
    seg->seg_id = seg_id;
}

static inline void meta_mmu_null_seg(struct meta_mmu_seg *seg)
{
    seg->vstart = 0;
}

/* Segments must be sorted by virtual address and non-overlapping */
void meta_mmu_setup(CPUArchState *env, bool htp, hwaddr eram,
                    struct meta_mmu_seg *l_segs, struct meta_mmu_seg *g_segs);

#endif /* _META_BOOT_MMU_H */
