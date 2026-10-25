#include "qemu/osdep.h"
#include "cpu.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "exec/cputlb.h"
#include "qemu/bitops.h"
#include "core.h"
#include "qemu/host-utils.h"
#include "hw/meta/meta_boot_mmu.h"
#include "qemu/log.h"

#define DEBUG_LEVEL 0

#if DEBUG_LEVEL >= 1
#  define DBGLOG(...) qemu_log(__VA_ARGS__)
#else
#  define DBGLOG(...) do { } while (0)
#endif

#define MASK(x) (BIT(x) - 1)

#define PTE_DEFAULT     0x0cf   /* Valid, priv, write, combine, cached */

/* Allocate memory from the top of physical RAM for MMU table data */
static hwaddr meta_mmu_malloc(hwaddr *brk, hwaddr sz, hwaddr alignment)
{
    *brk -= sz;
    *brk &= -alignment;
    return *brk;
}

static hwaddr mmu_setup_pgd(CPUArchState *env, bool htp, hwaddr *brk,
                            hwaddr pgd, unsigned int threadid,
                            unsigned int global, struct meta_mmu_seg *segs)
{
    /* Constants */
    const hwaddr pte_align = BIT(htp ? 6 : 12);

    /* Other threads */
    MetaCore *core = META_THREAD2CORE(env);
    uint32_t phys0;
    int i;

    /* Range of segments */
    struct meta_mmu_seg *seg;
    target_ulong vstart, vend;

    /* Size of address space */
    int lg_sz;
    target_ulong pgd_size;

    /* Iteration */
    target_ulong vaddr;
    hwaddr pgd_ptr;
    uint32_t pte_val;

    /* Calculate full range of segments (and pre-calculate page sizes) */
    vstart = segs->vstart;
    for (seg = segs; ; ++seg) {
        if (!seg->vstart) {
            /* reached end of segment */
            vend = seg[-1].vend;
            break;
        }
        if (htp) {
            target_ulong misalignment, temp;

            seg->flags = 0;
            /* Overlap same big page as previous? */
            if (seg != segs &&
                (((seg[-1].vend - 1) ^ seg->vstart) & MASK(22)) == 0) {
                seg->flags |= MMUF_JOIN_BIG;
            }
            /* Fit in a single big page? */
            if (((seg->vstart ^ (seg->vend - 1)) & MASK(22)) == 0) {
                seg->flags |= MMUF_SINGLE_BIG;
            }

            /* Find physical-virtual misalignment */
            misalignment = seg->vstart ^ seg->pstart;
            misalignment |= BIT(22);

            temp = misalignment;
            temp |= seg->vstart;
            /* Propagate page size if sharing big page with previous segment */
            if (seg->flags & MMUF_JOIN_BIG) {
                temp |= BIT(seg[-1].lg_end_pg);
            }
            seg->lg_start_pg = ffs(temp) - 1;
            if (seg->lg_start_pg < 12) {
                seg->lg_start_pg = 12;
            }

            seg->lg_mid_pg = ffs(misalignment) - 1;
            if (seg->lg_mid_pg < 12) {
                seg->lg_mid_pg = 12;
            }

            temp = misalignment;
            temp |= seg->vend;
            /* Propagate page size if contained within a single big page */
            if (seg->flags & MMUF_SINGLE_BIG) {
                temp |= BIT(seg->lg_start_pg);
            }
            seg->lg_end_pg = ffs(temp) - 1;
            if (seg->lg_end_pg < 12) {
                seg->lg_end_pg = 12;
            }
        } else {
            seg->lg_start_pg = 12;
            seg->lg_mid_pg   = 12;
            seg->lg_end_pg   = 12;
        }
    }
    /* work back propagating page sizes so segments in each big page agree */
    if (htp) {
        for (--seg; seg >= segs; --seg) {
            if (seg->flags & MMUF_SINGLE_BIG) {
                seg->lg_start_pg = seg->lg_end_pg;
            }
            if (seg->flags & MMUF_JOIN_BIG) {
                seg[-1].lg_end_pg = seg->lg_start_pg;
            }
        }
    }

    /* Calculate number of big (4MiB) pages required to cover that range */
    if (htp) {
        /*
         * Linux reaches the table through the MMPT window as pgd[pgd_index()],
         * which only holds if the local table starts at linear address 0.
         */
        vaddr = global ? (vstart & -BIT(22)) : 0;
        lg_sz = 10 - clz32(vaddr ^ (vend - 1));
        if (lg_sz < 0) {
            lg_sz = 0;
        }
    } else {
        vaddr = global << 31;
        lg_sz = 9; /* 2GiB */
    }
    pgd_size = BIT(lg_sz + 2);

    /* Allocate the PGD */
    if (!pgd) {
        pgd = meta_mmu_malloc(brk, pgd_size, 4);
    }

    /* Set up HTP MMU table registers */
    if (htp) {
        phys0 = vaddr | PTE_DEFAULT | lg_sz << META_MMCUTBLPHYS0_RANGE_SHIFT;
        if (!global) {
            env->mmu_tblphys[0] = phys0;
            env->mmu_tblphys[1] = pgd;
        } else {
            for (i = 0; i < core->num_threads; ++i) {
                core->threads[i].env.mmu_tblphys[2] = phys0;
                core->threads[i].env.mmu_tblphys[3] = pgd;
            }
        }
    }

    /* Set up PGD */
    seg = segs;
    vstart = seg->vstart;
    vend = seg->vend;
    pte_val = seg->pstart & ~MASK(12);
    pte_val |= PTE_DEFAULT;
    for (pgd_ptr = pgd; pgd_ptr < pgd + pgd_size; pgd_ptr += 4) {
        unsigned int lg_pg, pt_size;
        hwaddr pte, pte_ptr;
        target_ulong pg_size;
        uint32_t pgd_val;

        /* Not reached start of next segment yet? */
        if (!vstart || vaddr + BIT(22) <= vstart) {
            DBGLOG("PGD @%08x (%08" HWADDR_PRIx ") = 0\n",
                   vaddr, pgd_ptr);
            stl_le_phys(&address_space_memory, pgd_ptr, 0);
            vaddr += BIT(22);
            continue;
        }

        /* Calculate page size depending on start/mid/end */
        lg_pg = seg->lg_mid_pg;
        if (vstart > vaddr && lg_pg > seg->lg_start_pg) {
            lg_pg = seg->lg_start_pg;
        }
        if (vend < vaddr + BIT(22) && lg_pg > seg->lg_end_pg) {
            lg_pg = seg->lg_end_pg;
        }
        pg_size = BIT(lg_pg);
        pt_size = BIT(24 - lg_pg);

        /* Allocate PTE table and store in PGD entry */
        pte = meta_mmu_malloc(brk, pt_size, pte_align);
        pgd_val = pte | (lg_pg - 12) << 1 | 1;
        stl_le_phys(&address_space_memory, pgd_ptr, pgd_val);
        DBGLOG("PGD @%08x (%08" HWADDR_PRIx ") = %08x %dKiB pages\n",
               vaddr, pgd_ptr, pgd_val, pg_size >> 10);

        vstart &= -pg_size;
        pte_val &= -pg_size | MASK(12);
        for (pte_ptr = pte;
             pte_ptr < pte + pt_size;
             pte_ptr += 4, vaddr += pg_size) {
            /* No more segments left */
            if (!vstart) {
                stl_le_phys(&address_space_memory, pte_ptr, 0);
                DBGLOG("  PTE @%08x (%08" HWADDR_PRIx ") = 0\n",
                       vaddr, pte_ptr);
                continue;
            }
            /* Segment finished, onto the next one */
            if (vaddr >= vend) {
                ++seg;
                vstart = seg->vstart & -pg_size;
                vend = seg->vend;
                pte_val = seg->pstart & -pg_size;
                if (vstart) {
                    pte_val |= PTE_DEFAULT;
                }
            }
            /* Not reached start of segment */
            if (!vstart || vaddr < vstart) {
                stl_le_phys(&address_space_memory, pte_ptr, 0);
                DBGLOG("  PTE @%08x (%08" HWADDR_PRIx ") = 0\n",
                       vaddr, pte_ptr);
                continue;
            }
            stl_le_phys(&address_space_memory, pte_ptr, pte_val);
            DBGLOG("  PTE @%08x (%08" HWADDR_PRIx ") = %08x\n",
                   vaddr, pte_ptr, pte_val);
            pte_val += pg_size;
        }
    }

    return pgd;
}

static void meta_mmu_enable(CPUArchState *env)
{
    unsigned int hwthread = env->thread_num;
    MetaCore *core = META_THREAD2CORE(env);
    unsigned int t;

    /* Flush TLBs */
    for (t = 0; t < core->num_threads; ++t) {
        tlb_flush(CPU(&core->threads[t]));
    }

    /* Enable full MMU mode */
    core->global.mmu = 1;
    core->global.mmu_flags |= META_MMU_EB_DCACHE | META_MMU_EB_ICACHE;

    /* Set up cache partitioning (all cache to local) */
    core->dcparts[hwthread] = 0x8000000f;
    core->icparts[hwthread] = 0x0000000f;;
}

void meta_mmu_setup(CPUArchState *env, bool htp, hwaddr eram,
                    struct meta_mmu_seg *l_segs, struct meta_mmu_seg *g_segs)
{
    unsigned int hwthread = env->thread_num;
    MetaCore *core = META_THREAD2CORE(env);
    hwaddr brk, pgds;
    hwaddr g_pgd = 0, l_pgd = 0;

    brk = eram;

    if (core->global.core_rev >= META_COREREV1(2)) {
        /* Mark HTP style MMU table */
        core->global.mmu_flags |= META_MMU_HTP;
    } else {
        /* Allocate all pgds up front */
        pgds = meta_mmu_malloc(&brk, 5 * 4 * 512, 0x1000);
        l_pgd = pgds + 512*hwthread;
        g_pgd = pgds + 512*4;
        /* Set MMU table pointer */
        core->global.mmu_ptroot = pgds;
    }

    if (g_segs && g_segs->vstart) {
        mmu_setup_pgd(env, htp, &brk, g_pgd, hwthread, 1, g_segs);
    }
    if (l_segs && l_segs->vstart) {
        mmu_setup_pgd(env, htp, &brk, l_pgd, hwthread, 0, l_segs);
    }

    DBGLOG("Used %#" HWADDR_PRIx " bytes for page tables\n",
           eram - brk);

    meta_mmu_enable(env);
}
