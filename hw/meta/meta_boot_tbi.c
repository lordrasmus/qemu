#include "qemu/osdep.h"
#include "cpu.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "exec/cputlb.h"
#include "hw/meta/meta_boot_mmu.h"
#include "hw/meta/meta_boot_tbi.h"

#define DEBUG_LEVEL 0

#if DEBUG_LEVEL >= 1
#  define DBGLOG(...) qemu_log(__VA_ARGS__)
#else
#  define DBGLOG(...) do { } while (0)
#endif

#define TBI_SEG_SIZE    (8*4)
#define TBI_SIZE        ((4 + 32)*4)

target_ulong meta_tbi_setup(CPUArchState *env, target_ulong *brk, hwaddr v2p)
{
    target_ulong tbi;
    hwaddr tbi_p;
    unsigned int i;

    tbi = *brk;
    tbi_p = tbi + v2p;
    *brk += TBI_SIZE;

    DBGLOG("TBI @%08" HWADDR_PRIx " ->%08x\n",
           tbi_p, tbi);

    /* all fields can be zero initialised */
    for (i = 0; i < TBI_SIZE; i += 4) {
        stl_le_phys(&address_space_memory, tbi_p + i, 0);
    }
    return tbi;
}

static void meta_tbi_init_seg(hwaddr seg, target_ulong next,
                              unsigned int id, target_ulong l_addr,
                              target_ulong size)
{
    DBGLOG("TBI seg @%08" HWADDR_PRIx " ->%08x id=%08x addr=%08x sz=%08x\n",
           seg, next, id, l_addr, size);
    /* seg->link */
    stl_le_phys(&address_space_memory, seg, next);
    /* seg->id */
    stl_le_phys(&address_space_memory, seg += 4, id);
    /* seg->lock */
    stl_le_phys(&address_space_memory, seg += 4, 0);
    /* seg->bytes */
    stl_le_phys(&address_space_memory, seg += 4, size);
    /* seg->g_addr */
    stl_le_phys(&address_space_memory, seg += 4, 0);
    /* seg->l_addr */
    stl_le_phys(&address_space_memory, seg += 4, l_addr);
    /* seg->data */
    stl_le_phys(&address_space_memory, seg += 4, 0);
    stl_le_phys(&address_space_memory, seg += 4, 0);
}

target_ulong meta_tbi_seg_setup(CPUArchState *env, target_ulong *brk,
                                hwaddr v2p, struct meta_mmu_seg *l_segs,
                                struct meta_mmu_seg *g_segs)
{
    target_ulong segs, seg;
    struct meta_mmu_seg *mmu_seg;

    segs = *brk;

    seg = segs;
    if (l_segs) {
        for (mmu_seg = l_segs; mmu_seg->vstart;
             ++mmu_seg, seg += TBI_SEG_SIZE) {
            meta_tbi_init_seg(seg + v2p, seg + TBI_SEG_SIZE, mmu_seg->seg_id,
                              mmu_seg->vstart, mmu_seg->vend - mmu_seg->vstart);
        }
    }
    if (g_segs) {
        for (mmu_seg = g_segs; mmu_seg->vstart;
             ++mmu_seg, seg += TBI_SEG_SIZE) {
            meta_tbi_init_seg(seg + v2p, seg + TBI_SEG_SIZE, mmu_seg->seg_id,
                              mmu_seg->vstart, mmu_seg->vend - mmu_seg->vstart);
        }
    }
    /* seg[-1].link = NULL */
    stl_le_phys(&address_space_memory, seg + v2p - TBI_SEG_SIZE, 0);

    *brk = seg;
    return segs;
}
