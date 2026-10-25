#include "qemu/osdep.h"
#include "cpu.h"
#include "qemu/log.h"
#include "accel/tcg/cpu-loop.h"
#include "core.h"
#include "system/address-spaces.h"
#include "qemu/host-utils.h"
#include "exec/helper-proto.h"
#include "system/memory.h"
#include "hw/meta/meta_switch.h"

#define DEBUG_THREADS 0
#define MMU_DEBUG_LEVEL 0

#if DEBUG_THREADS >= 2
#  define THREADLOG(...) fprintf(stderr, ## __VA_ARGS__)
#elif DEBUG_THREADS >= 1
#  define THREADLOG(...) qemu_log(__VA_ARGS__)
#else
#  define THREADLOG(...) do { } while (0)
#endif

#if MMU_DEBUG_LEVEL >= 1
#  define MMULOG(...) fprintf(stderr, ## __VA_ARGS__)
#else
#  define MMULOG(...) do { } while (0)
#endif

/* find first non-sign bit */
int32_t HELPER(ffb)(int32_t arg)
{
    /* flip all bits if negative */
    arg ^= arg >> 31;
    /* use count leading zeros */
    return 31 - clz32(arg);
}

/* find first non-sign bit */
int32_t HELPER(ffb16)(int32_t arg)
{
    /* flip all bits if negative */
    arg ^= arg >> 31;
    /* use count leading zeros */
    return 31 - clz32(arg & 0xffff);
}

/* find left shifts to normalise */
int32_t HELPER(norm)(int32_t arg)
{
    /* flip all bits if negative */
    arg ^= arg >> 31;
    /* use count leading zeros */
    return clz32(arg) - 1;
}

/* find left shifts to normalise */
int32_t HELPER(norm16)(int32_t arg)
{
    /* flip all bits if negative */
    arg ^= arg >> 31;
    /* use count leading zeros */
    return clz32(arg & 0xffff) - 17;
}


/* Switch into interrupt mode */
static void take_interrupt(CPUArchState *env)
{
    uint32_t pc;

    if (env->cregs[META_TXSTATUS] & META_TXSTATUS_ISTAT_MASK) {
        cpu_abort(env_cpu(env), "take_interrupt() was called with ISTAT set");
    }

    /* stop waiting for LOCK */
    if (env->global->lock & META_LOCKW_THR(env->thread_num)) {
        env->global->lock &= ~META_LOCKW_THR(env->thread_num);
        do_unblock(env);
    }

    /* swap PC and PCX */
    pc = env->pc[META_PCX];
    env->pc[META_PCX] = env->pc[META_PC];
    env->pc[META_PC] = pc;

    /* enter interrupt mode */
    env->cregs[META_TXSTATUS] |= META_TXSTATUS_ISTAT_MASK;

    /* optionally toggle privilege mode */
    if (env->cregs[META_TXPRIVEXT] & META_TXPRIVEXT_PTOGGLE_MASK) {
        env->cregs[META_TXSTATUS] |= META_TXSTATUS_PSTAT_MASK;
    }

    /* clear lnkaddr so that next lnkset will fail */
    env->lnkaddr = 0;

    /* set RPDirty bit if the pipeline contains data */
    if (env->readport_count) {
        env->cregs[META_TXDIVTIME] |= META_TXDIVTIME_RPDIRTY_MASK;
    }
}

/*
 * Try to signal a CPU interrupt. This fails if it's already in interrupt mode
 * (returning 1), but otherwise signals the interrupt which will get
 * do_interrupt() called at a safe time.
 */
static int try_cpu_interrupt(CPUArchState *env)
{
    /* already in interrupt mode */
    if (env->cregs[META_TXSTATUS] & META_TXSTATUS_ISTAT_MASK) {
        return 1;
    }

    /* signal the interrupt */
    meta_interrupt(env, CPU_INTERRUPT_HARD);
    return 0;
}

/* Unhalt thread after being blocked on something */
void do_unblock(CPUArchState *env)
{
    env->block = META_BLOCK_NONE;
    env_cpu(env)->halted = 0;
    meta_reset_interrupt(env, CPU_INTERRUPT_HALT);
    /* kick the thread */
    meta_interrupt(env, CPU_INTERRUPT_WAKE);
}

/*
 * Handle a background trigger (e.g. TXTIMER or general)
 * Return 1 if handled by TXSTAT
 */
int do_bgtrigger(CPUArchState *env, uint32_t trigger_mask)
{
    int ret = 0;
    /* don't trigger twice */
    if ((env->tregs[META_TXSTAT] & trigger_mask) == trigger_mask) {
        return 0;
    }
    env->tregs[META_TXSTAT] |= trigger_mask;
    /* blocked on TXSTAT with this trigger */
    if (env_cpu(env)->halted && env->block == META_BLOCK_TXSTAT) {
        if (env->tregs[META_TXMASK] & trigger_mask) {
            do_unblock(env);
            ret |= 1;
        }
    }
    return ret;
}

/*
 * Triggers from outside the vCPU (timers, device IRQs) must not touch the
 * trigger registers while the vCPU runs: generated code updates TXSTAT(I)
 * and TXMASK(I) without the BQL.  Queue them to run on the vCPU instead.
 */
static void meta_trigger_work(CPUState *cs, run_on_cpu_data data)
{
    CPUArchState *env = cpu_env(cs);
    uint32_t mask = data.host_int >> 2;

    switch (data.host_int & 3) {
    case META_TRIGGER_BG:
        do_bgtrigger(env, mask);
        break;
    case META_TRIGGER_INT:
        do_itrigger(env, mask);
        break;
    default:
        do_trigger(env, mask);
        break;
    }
}

void meta_trigger_async(CPUArchState *env, uint32_t mask, int kind)
{
    async_run_on_cpu(env_cpu(env), meta_trigger_work,
                     RUN_ON_CPU_HOST_INT(mask << 2 | kind));
}

/* Update the highest priority interrupt number */
void HELPER(update_irqenc)(CPUArchState *env)
{
    uint32_t txstati, divtime;
    int irqenc;

    /* priority highest 15-4 1 0 3 2 lowest */
    txstati = env->tregs[META_TXMASKI] & env->tregs[META_TXSTATI] & 0xffff;

    /* swap triggers 1-0 with 3-2 so bits are in priority order for clz */
    txstati = (txstati & ~0xf)
            | ((txstati << 2) & 0xc)
            | ((txstati >> 2) & 0x3);

    /* use count leading zeros (clz) to find highest priority trigger */
    irqenc = 31 - clz32(txstati);

    /* fixup irqenc if one of the rearranged triggers */
    if (irqenc < 4) {
        irqenc ^= 0x2;
    }

    /* put IRQEnc into TXDIVTIME register */
    divtime = env->cregs[META_TXDIVTIME];
    divtime &= ~META_TXDIVTIME_IRQENC_MASK;
    divtime |= irqenc << META_TXDIVTIME_IRQENV_SHIFT;
    env->cregs[META_TXDIVTIME] = divtime;
}

/*
 * Handle an interrupt trigger (e.g. TXTIMERI or general)
 * Return 1 if handled by TXSTATI, 2 if handled by interrupt
 */
int do_itrigger(CPUArchState *env, uint32_t trigger_mask)
{
    int ret = 0;
    /* don't trigger twice */
    if ((env->tregs[META_TXSTATI] & trigger_mask) == trigger_mask) {
        return 0;
    }
    env->tregs[META_TXSTATI] |= trigger_mask;
    /* if an interrupt trigger, interrupt now */
    if ((env->cregs[META_TXENABLE] == META_TXENABLE_ENABLED) &&
            (env->tregs[META_TXMASKI] & trigger_mask)) {
        if (!try_cpu_interrupt(env)) {
            ret |= 2;
        }
    }
    /* blocked on TXSTAT or TXSTATI with this trigger */
    if (env_cpu(env)->halted && env->block == META_BLOCK_TXSTATI) {
        if (env->tregs[META_TXMASKI] & trigger_mask) {
            do_unblock(env);
            ret |= 1;
        }
    }
    return ret;
}

/* return 1 if caller should exit tb */
uint32_t HELPER(unmask_itrigger)(CPUArchState *env)
{
    if (env->tregs[META_TXMASKI] & env->tregs[META_TXSTATI]) {
        return !try_cpu_interrupt(env);
    }
    return 0;
}

/*
 * Handle a general (both background and interrupt) trigger
 * Return 1 if handled by TXSTATI, 2 if handled by interrupt
 */
int do_trigger(CPUArchState *env, uint32_t trigger_mask)
{
    int ret = 0;
    /* ordering important */
    ret |= do_itrigger(env, trigger_mask);
    ret |= do_bgtrigger(env, trigger_mask);
    return ret;
}

/*
 * Called when a hard interrupt has occurred (we must be ready to switch to
 * interrupt mode).
 */
void meta_cpu_do_interrupt(CPUState *cs)
{
    take_interrupt(cpu_env(cs));
}

bool meta_cpu_exec_interrupt(CPUState *cs, int interrupt_request)
{
    if (interrupt_request & CPU_INTERRUPT_HARD) {
        take_interrupt(cpu_env(cs));
        cpu_reset_interrupt(cs, CPU_INTERRUPT_HARD);
        return true;
    }
    return false;
}

void HELPER(block)(CPUArchState *env, uint32_t block)
{
    /* set the reason for the halt */
    env->block = block;
    /* and stop execution */
    meta_reset_interrupt(env, CPU_INTERRUPT_WAKE);
    meta_interrupt(env, CPU_INTERRUPT_HALT);
}

void HELPER(thread_enable)(CPUArchState *env)
{
    uint32_t txstatus;
    int replay = 0;

    THREADLOG("T%d Enable\n", env->thread_num);

    /* replay catch buffers if the appropriate CBMARKER bit is set */
    txstatus = env->cregs[META_TXSTATUS];
    if ((txstatus & (META_TXSTATUS_ISTAT_MASK | META_TXSTATUS_CBIMARKER_MASK))
                == (META_TXSTATUS_ISTAT_MASK | META_TXSTATUS_CBIMARKER_MASK)) {
        replay = 1;
    }
    if ((txstatus & (META_TXSTATUS_ISTAT_MASK | META_TXSTATUS_CBMARKER_MASK))
                == META_TXSTATUS_CBMARKER_MASK) {
        replay = 1;
    }

    if (replay) {
        env->catch_replay = 1;
    } else {
        HELPER(unmask_itrigger)(env);
    }
    env->cregs[META_TXENABLE] = META_TXENABLE_ENABLED;
    do_unblock(env);
}

static void update_thread_hwstatmeta(CPUArchState *env)
{
    MetaCore *core = META_THREAD2CORE(env);
    uint32_t hstat = 0;
    uint32_t diff;
    MetaHReason hreason;
    int shift;

    /* which bits should be set? */
    if (env->cregs[META_TXENABLE] != META_TXENABLE_ENABLED) {
        if (env->cregs[META_TXSTATUS] & META_TXSTATUS_ISTAT_MASK) {
            hstat = 2;
        } else {
            hstat = 1;
        }
        if (env->cregs[META_TXENABLE] == META_TXENABLE_OFF) {
            hreason = (env->cregs[META_TXSTATUS] & META_TXSTATUS_HREASON_MASK)
                            >> META_TXSTATUS_HREASON_SHIFT;
            if (hreason == META_HREASON_FAULT
                    || hreason == META_HREASON_UNKNOWN) {
                hstat |= 4;
            }
        }
    }

    /* Update HWSTATMETA */
    shift = env->thread_num * 4;
    hstat <<= shift;
    diff = core->triggers.stat_meta ^ hstat;
    diff &= 0xf << shift;
    core->triggers.stat_meta ^= diff;

    /* FIXME look for raised bits */
#if 0
    for (; diff; ++shift) {
        if (diff & hstat & (1 << shift)) {
            /* look for the vector */
            diff &= ~(1 << shift);
        }
    }
#endif
}

/* put the thread in the off state (but not stopped) */
void HELPER(thread_disable)(CPUArchState *env)
{
    THREADLOG("T%d Disable\n", env->thread_num);
    /*
     * We shouldn't be able to get here while waiting for LOCK, so there's no
     * need to clear the LOCKW bit.
     */
    env->cregs[META_TXENABLE] = META_TXENABLE_OFF;
    update_thread_hwstatmeta(env);
    HELPER(block)(env, META_BLOCK_OFF);
}

/* TXENABLE.ThreadEnable = 0 to stop the thread */
void HELPER(thread_stop)(CPUArchState *env)
{
    THREADLOG("T%d Stop\n", env->thread_num);
    /* stop waiting for LOCK */
    env->global->lock &= ~META_LOCKW_THR(env->thread_num);
    env->cregs[META_TXENABLE] = META_TXENABLE_STOPPED;
    update_thread_hwstatmeta(env);
    HELPER(block)(env, META_BLOCK_OFF);
}

/* Trigger a halt from target code. */
void HELPER(halt)(CPUArchState *env)
{
    MetaHReason hreason;
    MetaFReason freason;

    hreason = (env->cregs[META_TXSTATUS] & META_TXSTATUS_HREASON_MASK)
                                    >> META_TXSTATUS_HREASON_SHIFT;
    freason = (env->cregs[META_TXSTATUS] & META_TXSTATUS_FREASON_MASK)
                                    >> META_TXSTATUS_FREASON_SHIFT;

#if DEBUG_THREADS >= 1
    THREADLOG("T%d Halt:%s, Fault:%s, at %08x\n",
              env->thread_num,
              meta_hreason_names[hreason],
              meta_freason_names[freason],
              env->pc[META_PC]);
#endif
    /*
     * If the halt trigger cannot be handled by an interrupt, don't call
     * do_itrigger even though it will return 0 to indicate failure, because
     * it will also set the bit in TXSTATI which we don't want unless the
     * interrupt can be handled.
     */
    if (!(env->tregs[META_TXMASKI] & META_TRIGGER_HALT_MASK) ||
            (env->cregs[META_TXSTATUS] & META_TXSTATUS_ISTAT_MASK) ||
            !do_itrigger(env, META_TRIGGER_HALT_MASK)) {
        /* attempt to handle a SWITCH rather than disabling the thread */
        if ((hreason != META_HREASON_SWITCH) ||
            (freason != META_FREASON_NOERROR) ||
            meta_switch_handle(env, env->switch_code, env->aregs[0][0])) {
            /* not a SWITCH, or an unhandled SWITCH */
            HELPER(thread_disable)(env);
        } else {
            /* handled SWITCH, step over it */
            env->pc[META_PC] = env->switch_nextpc;
        }
    }
}

/* Trigger a halt for a specific signal. */
void HELPER(halt_signum)(CPUArchState *env, uint32_t signum)
{
    /* put reason codes into TXSTATUS */
    uint32_t txstatus = env->cregs[META_TXSTATUS];
    txstatus &= ~(META_TXSTATUS_FREASON_MASK | META_TXSTATUS_HREASON_MASK);
    txstatus |= (signum << META_TXSTATUS_HREASON_SHIFT);
    env->cregs[META_TXSTATUS] = txstatus;

    /* and trigger the halt */
    HELPER(halt)(env);

    /* abandon the current block */
    cpu_loop_exit(env_cpu(env));
}

uint32_t meta_get_pte1(MetaCore *core, int t, uint32_t segment)
{
    int global = segment & (1 << 9);
    if (global) {
        segment &= ~(1 << 9);
    }
    if (core->global.mmu_flags & META_MMU_HTP) {
        uint32_t range;
        /* check against the range of addresses */
        range = (core->threads[t].env.mmu_tblphys[global >> 8]
                    & (META_MMCUTBLPHYS0_BASE_MASK & ~(1<<31)))
                >> 22;
        if (unlikely(segment < range)) {
            return 0;
        }
        segment -= range;
        range = 1 << ((core->threads[t].env.mmu_tblphys[global >> 8]
                        & META_MMCUTBLPHYS0_RANGE_MASK)
                        >> META_MMCUTBLPHYS0_RANGE_SHIFT);
        if (unlikely(segment >= range)) {
            return 0;
        }
        /* HTP: per-thread global page tables */
        MMULOG("T%d PTE1[%08x] @%08x = %08x\n",
               t, segment << 22, core->threads[t].env.mmu_tblphys[(global >> 8) | 1] + (segment << 2),
                        ldl_le_phys(&address_space_memory, core->threads[t].env.mmu_tblphys[(global >> 8) | 1] + (segment << 2)));
        return ldl_le_phys(&address_space_memory, core->threads[t].env.mmu_tblphys[(global >> 8) | 1] +
                        (segment << 2));
    } else {
        /* ATP: single global page table after local page tables */
        if (global) {
            t = META_MAX_THREADS;
        }
        MMULOG("T%d PTE1[%08x] @%08x = %08x\n",
               t, segment << 22, core->global.mmu_ptroot + (t << 11) + (segment << 2),
                        ldl_le_phys(&address_space_memory, core->global.mmu_ptroot + (t << 11) + (segment << 2)));
        return ldl_le_phys(&address_space_memory, core->global.mmu_ptroot +
                        (t << 11) +
                        (segment << 2));
    }
}

void meta_set_pte1(MetaCore *core, int t, uint32_t segment, uint32_t pte)
{
    int global = segment & (1 << 9);
    if (global) {
        segment &= ~(1 << 9);
    }
    if (core->global.mmu_flags & META_MMU_HTP) {
        /* FIXME should offset by the base and bounds check with the range */
        /* HTP: per-thread global page tables */
        stl_le_phys(&address_space_memory, core->threads[t].env.mmu_tblphys[(global >> 8) | 1] +
                 (segment << 2), pte);
    } else {
        /* ATP: single global page table after local page tables */
        if (global) {
            t = META_MAX_THREADS;
        }
        stl_le_phys(&address_space_memory, core->global.mmu_ptroot +
                 (t << 11) +
                 (segment << 2),
                 pte);
    }
}

/*
 * Return page size shift on top of minimum 12.
 * e.g. for ATP compatibility mode this always returns 0.
 */
static int meta_get_pte1_sizeshift(MetaCore *core, uint32_t pte1)
{
    if (core->global.mmu_flags & META_MMU_HTP) {
        return (pte1 >> 1) & 0xf;
    } else {
        return 0;
    }
}

/* Return physical address of second level page table */
static uint32_t meta_get_pte1_ptr(MetaCore *core, uint32_t pte1)
{
    if (!(pte1 & 1)) {
        return 0;
    }

    if (core->global.mmu_flags & META_MMU_HTP) {
        return pte1 & ~0x3f;
    } else {
        return pte1 & ~0xfff;
    }
}

/*
 * Return physical address of second level page table entry
 * page always in 4KB pages, gets shifted if larger
 */
static uint32_t meta_get_pte2p_from_pte1(MetaCore *core, int t, uint32_t pte1,
                                         uint32_t page)
{
    uint32_t ptei2, ptep2;
    int size_shift;

    /* MiniM compressed 2nd level page table */
    if (core->global.mmu_flags & META_MMU_HTP && pte1 & (1 << 5)) {
        /* Only the 2nd 0.5MB is valid in each 2MB region */
        if ((page & (0x3 << 7)) != (0x1 << 7)) {
            return 1;
        }
        /*
         * Only the 2 valid 0.5MB regions in the 4MB segment have PTEs, so
         * transform bits [9:7] of page number (2MB-0.5MB) like so:
         *  X** -> 00X
         */
        page = (page & ~(0x7 << 7)) |
               ((page >> 2) & (0x1 << 7));
    }

    size_shift = meta_get_pte1_sizeshift(core, pte1);
    ptei2 = (page >> size_shift) & (0x3ff >> size_shift);
    ptep2 = meta_get_pte1_ptr(core, pte1);

    return ptep2 + (ptei2 << 2);
}

/* page always in 4KB pages, gets shifted if larger */
static uint32_t meta_get_pte2_from_pte1(MetaCore *core, int t, uint32_t pte1,
                                        uint32_t page)
{
    uint32_t pte2p;

    /* Check 1st level page table entry is valid */
    if (!(pte1 & 1)) {
        return 0;
    }

    pte2p = meta_get_pte2p_from_pte1(core, t, pte1, page);
    if (unlikely(pte2p & 0x3)) {
        return 0;
    }
    return ldl_le_phys(&address_space_memory, pte2p);
}

uint32_t meta_get_pte2(MetaCore *core, int t, uint32_t page)
{
    uint32_t pte1 = meta_get_pte1(core, t, page >> 10);
    return meta_get_pte2_from_pte1(core, t, pte1, page);
}

#if MMU_DEBUG_LEVEL < 2
static inline void dump_page_table(MetaCore *core, int t)
{
}
#else
static void dump_page_table(MetaCore *core, int t)
{
    int i, j;
    for (i = 0; i < 1024; ++i) {
        uint32_t pte = meta_get_pte1(core, t, i);
        if (pte & 1) {
            int size_shift = meta_get_pte1_sizeshift(core, pte);
            MMULOG("T%d PTE1[%08x]=%08x (%dKB)\n",
                   t, i << 22, pte, 4 << size_shift);
            for (j = 0; j < 1024; j += (1 << size_shift)) {
                uint32_t pte2 = meta_get_pte2_from_pte1(core, t, pte, j);
                if (pte2 & 1) {
                    MMULOG("       [%08x]=%08x\n",
                           (i << 22) | (j << 12),
                           pte2);
                }
            }
        }
    }
}
#endif

static inline int get_phys_addr(CPUArchState *env, uint32_t address,
                                int access_type, int mmu_idx,
                                uint32_t *phys_ptr, int *prot,
                                target_ulong *page_size,
                                uint32_t *ppgd, uint32_t *ppte, uint32_t *pptep)
{
    MetaCore *core = META_THREAD2CORE(env);
    MetaFReason freason;
    int mmu_off = (mmu_idx == MMU_NOMMU_IDX);
    int unmappable = !(address & 0x78000000);
    MemoryRegionSection section;

    *phys_ptr = 0;
    *prot = 0;
    *page_size = 0;
    if (unmappable || mmu_off) {
        freason = META_FREASON_GENERAL;

        if (ppgd) {
            *ppgd = 0;
        }
        if (ppte) {
            *ppte = 0;
        }
        if (pptep) {
            *pptep = 0;
        }

        /* Direct maps */
        if ((address & 0xfe000000) == 0x06000000) {
            int dmap = (address >> 23) & 0x3;
            address &= (1 << 23) - 1;
            address |= core->directmap_bases[dmap];
        }
        /* On HTP the memory mapped page table regions are like direct maps */
        else if (core->global.core_rev >= META_COREREV1(2)
                && (address & 0xff000000) == 0x05000000) {
            int t = (address >> 21) & (core->num_threads - 1);
            int global = (address >> 22) & 2;
            uint32_t phys0 = core->threads[t].env.mmu_tblphys[global];
            address &= 0x1fffff;
            /* check it's valid */
            if (!(phys0 & META_MMCUTBLPHYS0_VALID_MASK)) {
                goto fail;
            }
            /* check priv */
            if (!(mmu_idx & MMU_IDX_PSTAT)
                        && (phys0 & META_MMCUTBLPHYS0_PRIV_MASK)) {
                goto fail;
            }
            /* check write */
            *prot = PAGE_READ;
            if (phys0 & META_MMCUTBLPHYS0_WRITE_MASK) {
                *prot |= PAGE_WRITE;
            }

            *page_size = 1 << 19; /* 512KB */
            address = (core->threads[t].env.mmu_tblphys[global | 1]
                            & META_MMCUTBLPHYS1_MMPTMASK) + address;
        }

        /* Unmappable or MMU Bypassed */
        section = memory_region_find(get_system_memory(), address, 1);
        if (!section.size) {
            /*
             * FIXME With MMU bypassed, this should be read as 0xdeadbeef and
             * not halt (but may still trigger if TXSTATI & 4).
             */
            goto fail;
        }

        *phys_ptr = address;
        /* for memory mapped page tables (see above), keep existing values */
        if (!*prot) {
            *prot = PAGE_READ | PAGE_WRITE | PAGE_EXEC;
            *page_size = TARGET_PAGE_SIZE;
            /* Core code memories aren't readable on META1 */
            if ((address & 0xfe000000) == 0x80000000) {
                /* FIXME is this possible without making reads fault? */
                *prot &= ~PAGE_READ;
            }
            /* Core data memories aren't executable */
            if ((address & 0xfe000000) == 0x82000000) {
                *prot &= ~PAGE_EXEC;
            }
        }
    } else {
        int t, size_shift;
        uint32_t ptei, pte;
        freason = META_FREASON_PAGE;
        t = mmu_idx & MMU_IDX_TX;

        if (ppte) {
            *ppte = 0;
        }
        if (pptep) {
            *pptep = 0;
        }

        ptei = (address >> 22) & 0x3ff;
        pte = meta_get_pte1(core, t, ptei);
        if (ppgd) {
            *ppgd = pte;
        }
        /* valid 1st level pte? */
        if (!(pte & 1)) {
            MMULOG("T%d PTE1[%08x]=%08x not valid\n",
                   t, address, pte);
            dump_page_table(core, t);
            goto fail;
        }
        size_shift = 12 + meta_get_pte1_sizeshift(core, pte);
        *page_size = 1 << size_shift;
        if (pptep) {
            *pptep = meta_get_pte2p_from_pte1(core, t, pte,
                                              (address >> 12) & 0x3ff);
        }
        pte = meta_get_pte2_from_pte1(core, t, pte, (address >> 12) & 0x3ff);
        if (ppte) {
            *ppte = pte;
        }
        /* valid 2nd level pte? */
        if (!(pte & (1 << 0))) {
            MMULOG("T%d PTE2[%08x]=%08x not valid\n",
                   t, address, pte);
            dump_page_table(core, t);
            goto fail;
        }
        /* priv without previlege? */
        if (!(mmu_idx & MMU_IDX_PSTAT) && (pte & (1 << 2))) {
            MMULOG("T%d PTE2[%08x]=%08x priv'd\n",
                   t, address, pte);
            dump_page_table(core, t);
            freason = META_FREASON_GENERAL;
            goto fail;
        }

        *phys_ptr = (pte & 0xfffff000) + (address & (*page_size-1));
        *prot = PAGE_READ | PAGE_EXEC;
        if (pte & (1 << 1)) {
            *prot |= PAGE_WRITE;
        }
    }

    *prot |= PAGE_VALID;

fail:
    if (access_type & ~*prot) {
        /* trying to execute? */
        if (access_type & PAGE_EXEC) {
            return META_SIGNUM(freason, META_HREASON_UNKNOWN);
        }
        /* don't have permission to read, whether reading or writing? */
        if (!(*prot & PAGE_READ)) {
            return META_SIGNUM(freason, META_HREASON_FAULT);
        }
        /* don't have permission to write? */
        if (access_type & PAGE_WRITE) {
            return META_SIGNUM_DWF;
        }
        if (ppgd) {
            *ppgd = 0;
        }
        if (ppte) {
            *ppte = 0;
        }
        if (pptep) {
            *pptep = 0;
        }
    }
    return 0;
}

bool meta_cpu_tlb_fill(CPUState *cs, vaddr address, int size,
                       MMUAccessType access_type, int mmu_idx,
                       bool probe, uintptr_t retaddr)
{
    CPUArchState *env = cpu_env(cs);
    uint32_t phys_addr;
    target_ulong page_size;
    int prot;
    int ret;

    /* MMU_DATA_LOAD/STORE/INST_FETCH give PAGE_READ/WRITE/EXEC */
    ret = get_phys_addr(env, address, 1 << access_type, mmu_idx, &phys_addr,
                        &prot, &page_size, NULL, NULL, NULL);
    if (ret == 0) {
        /* Map a single page.  */
        phys_addr &= ~(uint32_t)0xfff;
        address &= ~(uint32_t)0xfff;
        tlb_set_page(cs, address, phys_addr, prot, mmu_idx, page_size);
        return true;
    }
    if (probe) {
        return false;
    }

    qemu_log_mask(CPU_LOG_MMU, "T%d %s fault at 0x%08" PRIx64
                  " (mmu_idx %d): signum 0x%x\n", env->thread_num,
                  access_type == MMU_INST_FETCH ? "fetch" :
                  access_type == MMU_DATA_STORE ? "store" : "load",
                  (uint64_t)address, mmu_idx, ret);

    if (retaddr && cpu_restore_state(cs, retaddr)) {
        meta_restore_fault_state(env);
    } else {
        if (env->catch_replay == 2) {
            /* we're already replaying a catch, just update the fault reason */
            meta_update_txcatch_signum(ret, &env->cregs[META_TXCATCH0]);
            env->catch_replay = 0;
        }
    }
    HELPER(halt_signum)(env, ret);
}

hwaddr meta_cpu_get_phys_page_debug(CPUState *cs, vaddr addr)
{
    CPUArchState *env = cpu_env(cs);
    uint32_t phys;
    int prot;
    target_ulong page_size;
    int mmuid;

    if (env->global->mmu) {
        mmuid = MMU_MMUTXP_IDX(env->thread_num);
    } else {
        mmuid = MMU_NOMMU_IDX;
    }
    if (!get_phys_addr(env, addr, PAGE_VALID, mmuid,
                       &phys, &prot, &page_size, NULL, NULL, NULL)) {
        return phys;
    } else {
        return -1;
    }
}

/* Do CACHERL on non-mmuable memory or with mmu off */
static uint64_t nonmmu_cacherl(CPUArchState *env, uint32_t addr)
{
    uint32_t phys;
    int prot;
    target_ulong page_size;
    int err;
    err = get_phys_addr(env, addr, 0, MMU_NOMMU_IDX,
                        &phys, &prot, &page_size, NULL, NULL, NULL);
    /* if nothing is there, return all zeros */
    if (err) {
        return 0;
    }
    /* non-mmuable memory is effectively a direct mapping to physical */
    phys = 1 | (addr & TARGET_PAGE_MASK);
    if (prot & PAGE_WRITE) {
        phys |= 2;
    }
    /* FIXME implement priv when mmu is off */
    return ((uint64_t)phys << 32) | phys;
}

/* CACHERL - read mmu translation information */
uint64_t HELPER(cacherl)(CPUArchState *env, uint32_t addr)
{
    MetaCore *core = META_THREAD2CORE(env);
    int t = env->thread_num;
    uint32_t pte1;
    uint32_t pte2;
    uint32_t size_mask;
    uint32_t pte2_bits;

    /* bit 3 of address indicates icache instead of dcache */

    /* Handle nommu and non-mmuable memory */
    if (unlikely(!env->global->mmu || !(addr & 0x78000000))) {
        return nonmmu_cacherl(env, addr);
    }

    pte1 = meta_get_pte1(core, t, addr >> 22);
    /* first level entry invalid */
    if (!(pte1 & 1)) {
        return 4;
    }
    pte2 = meta_get_pte2_from_pte1(core, t, pte1, addr >> 12);
    /* second level entry invalid */
    if (!(pte2 & 1)) {
        return 0;
    }

    size_mask = (-1 << meta_get_pte1_sizeshift(core, pte1)) & 0xfff;

    /*
     * The physical address is to 4k resolution so we cannot just take the 2nd
     * level PTE directly. Only the bits set in pte2_bits must come from the
     * pte2, the rest from the virtual address.
     */
    pte2_bits = (size_mask << 12) | 0xff000fff;
    pte2 = (pte2 & pte2_bits) | (addr & ~pte2_bits);

    return ((uint64_t)size_mask << 36) | ((uint64_t)1 << 32) | pte2;
}

