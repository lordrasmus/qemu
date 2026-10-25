/*
 * META Core Registers.
 *
 * Copyright (C) 2011 Imagination Technologies Ltd.
 */

#include "qemu/osdep.h"
#include "exec/tb-flush.h"
#include "core_registers.h"
#include "core.h"

#include "qemu/log.h"
#include "system/address-spaces.h"
#include "qemu/host-utils.h"
#include "system/memory.h"
#include "hw/core/irq.h"

#define DEBUG_LEVEL 1

#if DEBUG_LEVEL >= 2
#  define DBGLOG(...) fprintf(stderr, ## __VA_ARGS__)
#elif DEBUG_LEVEL >= 1
#  define DBGLOG(...) qemu_log_mask(LOG_UNIMP, __VA_ARGS__)
#else
#  define DBGLOG(...) do { } while (0)
#endif

#define META_EXPANSION_BASE     0x03000000
#define META_EXPANSION_SIZE      0x1000000
#define META_CACHEFLUSH_BASE    0x04400000
#define META_CACHEFLUSH_SIZE      0x200000
#define META_TLBFLUSH_BASE      0x04700000
#define META_TLBFLUSH_SIZE        0x100000
#define META_COREREGS_BASE      0x04800000
#define META_COREREGS_SIZE         0x10000
#define META_PRIVREGS_BASE      0x04810000
#define META_PRIVREGS_SIZE         0x10000
#define META_TRIGREGS_BASE      0x04820000
#define META_TRIGREGS_SIZE         0x10000
#define META_CTRLREGS_BASE      0x04830000
#define META_CTRLREGS_SIZE         0x10000
#define META_SECUREREGS_BASE    0x04840000
#define META_SECUREREGS_SIZE       0x10000

/* Expansion registers */
typedef enum {
    META_EXPAND_TIMER_DIV   = 0x40,
} MetaExpandRegs;

#define META_EXPAND_TIMER_DIV_MASK  0xfff

/* Core registers */
#define META_COREREGS_TSTRIDE   0x01000 /* hardware thread stride */
#define META_COREREGS_TSHIFT    12      /* hardware thread stride shift */

/* Per-thread core registers */
typedef enum {
    /* kicks */
    META_TXKICK             = 0x800,
    META_TXKICKI            = 0x808,
    /* AMA */
    META_TXAMAREG4          = 0x810,
    META_TXAMAREG5          = 0x818,
    META_TXAMAREG6          = 0x820,
    /* priv */
    META_TXPRIVCORE         = 0x828,
} MetaThreadCoreRegs;

/* Global core registers */
typedef enum {
    /* code breakpoints */
    META_CODEB0ADDR         = 0xff00,
    META_CODEB0CTRL         = 0xff08,
    META_CODEB1ADDR         = 0xff10,
    META_CODEB1CTRL         = 0xff18,
    META_CODEB2ADDR         = 0xff20,
    META_CODEB2CTRL         = 0xff28,
    META_CODEB3ADDR         = 0xff30,
    META_CODEB3CTRL         = 0xff38,
    META_CODEB_STRIDE       = 0x10,
    /* performance counters */
    META_PERFCOUNT0         = 0xffe0,
    META_PERFCOUNT1         = 0xffe8,
    /* internal register port */
    META_TXUXXRXDT          = 0xfff0,
    META_TXUXXRXRQ          = 0xfff8,
} MetaGlobalCoreRegs;

#define META_TXUXXRXRQ_DREADY_MASK  0x80000000
#define META_TXUXXRXRQ_DSPEXT_MASK  0x00020000
#define META_TXUXXRXRQ_RNW_MASK     0x00010000
#define META_TXUXXRXRQ_T_MASK       0x00003000
#define META_TXUXXRXRQ_T_SHIFT      12
#define META_TXUXXRXRQ_R_MASK       0x000001f0
#define META_TXUXXRXRQ_R_SHIFT      4
#define META_TXUXXRXRQ_U_MASK       0x0000000f
#define META_TXUXXRXRQ_U_SHIFT      0
#define META_TXUXXRXRQ_MASK         0x000331ff

/* Control registers */
typedef enum {
    META_METAID             = 0x0000,
    META_MMCUTBLPHYS        = 0x0010,
    META_SYSCCACHEMMUCONFIG = 0x0028,
    META_SYSCDCACHEFLUSH    = 0x0038,
    META_SYSCICACHEFLUSH    = 0x0040,

    META_MMCUDIRECTMAP0ADDR = 0x0080,
    META_MMCUDIRECTMAP1ADDR = 0x0090,
    META_MMCUDIRECTMAP2ADDR = 0x00a0,
    META_MMCUDIRECTMAP3ADDR = 0x00b0,

    META_SYSCDCPART0        = 0x0200,
    META_SYSCDCPART1        = 0x0208,
    META_SYSCDCPART2        = 0x0210,
    META_SYSCDCPART3        = 0x0218,
    META_SYSCICPART0        = 0x0220,
    META_SYSCICPART1        = 0x0228,
    META_SYSCICPART2        = 0x0230,
    META_SYSCICPART3        = 0x0238,

    META_MCMDATAX           = 0x0300,
    META_MCMDATAT           = 0x0308,
    META_MCMGCTRL           = 0x0310,
    META_MCMSTATUS          = 0x0318,

    META_MMCUTBLPHYS_MIN    = 0x0700,
    META_MMCUTBLPHYS_MAX    = 0x07ff,
    META_MMCULOCALTBLPHYS0  = 0x00,
    META_MMCULOCALTBLPHYS1  = 0x08,
    META_MMCUGLOBALTBLPHYS0 = 0x10,
    META_MMCUGLOBALTBLPHYS1 = 0x18,
    META_MMCUTBLPHYS_SHIFT  = 5,
    META_MMCUTBLPHYS_STRIDE = 0x20,

    META_CORE_ID            = 0x1000,
    META_CORE_REV           = 0x1008,
    META_DESIGNER1          = 0x1010,
    META_DESIGNER2          = 0x1018,
    META_CORE_CONFIG2       = 0x1020,
} MetaControlRegs;

#define META_MMCUTBLSPHYS_ADDR_ATP      0xfffffff8
#define META_MMCUTBLSPHYS_ADDR_HTP      0xfffffffc
#define META_MMCUTBLSPHYS_MODE          0x00000001

#define META_SYSCCACHEMMUCONFIG_EN        0x1
#define META_SYSCCACHEMMUCONFIG_EB_DCACHE 0x2
#define META_SYSCCACHEMMUCONFIG_EB_ICACHE 0x4

#define META_MCMGCTRL_TRIG_MASK         0x80000000
#define META_MCMGCTRL_SEL_MASK          0x0ff00000
#define META_MCMGCTRL_SEL_SHIFT         20
#define META_MCMGCTRL_ADDR_MASK         0x000ffffc
#define META_MCMGCTRL_ADDR_SHIFT        2
#define META_MCMGCTRL_INC_MASK          0x00000002
#define META_MCMGCTRL_INC_BITS          0x07fffffc  /* The bits that are incremented */
#define META_MCMGCTRL_READ_MASK         0x00000001
#define META_MCMGCTRL_MASK              0x8ffffffe  /* LSB is not emulated */


/* Memory mapped page table */
#define META_MMUTBL_BASE        0x05000000
#define META_MMUTBL_SIZE        0x01000000

/* META 1 specific memory mapped page table */
#define META1_MMUROOT_OFFSET    0x00014050
#define META1_MMUROOT_SIZE      0xc
#define META1_MMU1ST_OFFSET     0x00014080
#define META1_MMU1ST_TSTRIDE    0x00000800
#define META1_MMU1ST_TSHIFT     11
#define META1_MMU2ND_OFFSET     0x00020000
#define META1_MMU2ND_TSIZE      0x001e0000
#define META1_MMU2ND_TSTRIDE    0x00200000
#define META1_MMU2ND_TSHIFT     21

/* Secure registers */
typedef enum {
    META_GP_SECURE0         = 0x0000,
    META_GP_SECURE1         = 0x0008,
    META_GP_SECURE2         = 0x0010,
    META_GP_SECURE3         = 0x0018,
    META_GP_SECURE4         = 0x0020,
    META_GP_SECURE5         = 0x0028,
    META_GP_SECURE6         = 0x0030,
    META_GP_SECURE7         = 0x0038,
} MetaSecureRegs;

typedef enum {
    META_CORE_CODE_MEM_0    = 0x10,
    META_CORE_CODE_MEM_MAX  = 0x17,
    META_CORE_DATA_MEM_0    = 0x18,
    META_CORE_DATA_MEM_MAX  = 0x1f,
    /* core->core_mems array is only 32 long for now */
} MetaCoreMemSel;

/*
 * Initialise a core memory.
 * size must be a power of 2.
 */
void cpu_meta_core_mem_init(MetaCore *core, MetaCoreMemType type,
                            hwaddr phys, hwaddr size)
{
    int mini = 1, maxi = 0;
    const char *name = NULL;
    int i;
    if (type & META_CORE_MEM_CODE) {
        mini = META_CORE_CODE_MEM_0;
        maxi = META_CORE_CODE_MEM_MAX;
        name = "code";
    } else if (type & META_CORE_MEM_DATA) {
        mini = META_CORE_DATA_MEM_0;
        maxi = META_CORE_DATA_MEM_MAX;
        name = "data";
    }
    for (i = mini; i <= maxi; ++i) {
        if (core->core_mems[i].type == META_CORE_MEM_UNUSED) {
            core->core_mems[i].type = type;
            core->core_mems[i].phys = phys;
            core->core_mems[i].size = size;
            qemu_log_mask(CPU_LOG_RESET, "Registered core %s %s 0x%x @ address 0x%08" HWADDR_PRIx
                     ", size=%" HWADDR_PRId " Kbytes\n",
                     name,
                     type & META_CORE_MEM_ROM ? "rom" : "ram",
                     i, phys, size >> 10);
            return;
        }
    }
    assert(!"cpu_meta_core_mem_init could not init core memory");
}

static uint32_t meta_debugport_do_read(MetaCore *core, uint8_t select,
                                       uint32_t addr)
{
    if (unlikely(select >= ARRAY_SIZE(core->core_mems))) {
        goto fail;
    }
    if (unlikely(core->core_mems[select].type == META_CORE_MEM_UNUSED)) {
        goto fail;
    }

    if (core->core_mems[select].phys) {
        addr = (addr << 2) & (core->core_mems[select].size - 1);
        return ldl_le_phys(&address_space_memory, core->core_mems[select].phys + addr);
    }

fail:
    DBGLOG("Unhandled META debug port read(0x%02x, 0x%x)\n", select, addr);
    return 0;
}

static void meta_debugport_do_write(MetaCore *core, uint8_t select,
                                    uint32_t addr, uint32_t val)
{
    if (unlikely(select >= ARRAY_SIZE(core->core_mems))) {
        goto fail;
    }
    if (unlikely(core->core_mems[select].type == META_CORE_MEM_UNUSED)) {
        goto fail;
    }

    /* cannot write to read only memory */
    /* FIXME DA-Sim doesn't understand read only memory */
#if 0
    if (unlikely(core->core_mems[select].type & META_CORE_MEM_ROM)) {
        goto fail;
    }
#endif

    if (core->core_mems[select].phys) {
        addr = (addr << 2) & (core->core_mems[select].size - 1);
        stl_le_phys(&address_space_memory, core->core_mems[select].phys + addr, val);
        return;
    }

fail:
    DBGLOG("Unhandled META debug port write(0x%02x, 0x%x, 0x%08x)\n", select, addr, val);
    return;
}

static void meta_debugport_read(MetaCore *core)
{
    /* FIXME locking between threads */
    uint8_t select = (core->mcm_ctrl & META_MCMGCTRL_SEL_MASK)
                                >> META_MCMGCTRL_SEL_SHIFT;
    uint32_t addr = (core->mcm_ctrl & META_MCMGCTRL_ADDR_MASK)
                                >> META_MCMGCTRL_ADDR_SHIFT;
    core->mcm_data = meta_debugport_do_read(core, select, addr);
    if (core->mcm_ctrl & META_MCMGCTRL_INC_MASK) {
        uint32_t incbits = (core->mcm_ctrl + (1 << META_MCMGCTRL_ADDR_SHIFT))
                                & META_MCMGCTRL_INC_BITS;
        core->mcm_ctrl = (core->mcm_ctrl & ~META_MCMGCTRL_INC_BITS) | incbits;
    }
}

static void meta_debugport_write(MetaCore *core)
{
    /* FIXME locking between threads */
    uint8_t select = (core->mcm_ctrl & META_MCMGCTRL_SEL_MASK)
                                >> META_MCMGCTRL_SEL_SHIFT;
    uint32_t addr = (core->mcm_ctrl & META_MCMGCTRL_ADDR_MASK)
                                >> META_MCMGCTRL_ADDR_SHIFT;
    meta_debugport_do_write(core, select, addr, core->mcm_data);
    if (core->mcm_ctrl & META_MCMGCTRL_INC_MASK) {
        uint32_t incbits = (core->mcm_ctrl + (1 << META_MCMGCTRL_ADDR_SHIFT))
                                & META_MCMGCTRL_INC_BITS;
        core->mcm_ctrl = (core->mcm_ctrl & ~META_MCMGCTRL_INC_BITS) | incbits;
    }
}

static uint64_t meta_expansion_io_read(void *opaque, hwaddr addr,
                                       unsigned size)
{
    MetaCore *core = opaque;
    uint32_t ret = 0;

    if (addr & 0x4) {
        return 0;
    }
    addr &= 0x7c;

    switch (addr) {
    case META_EXPAND_TIMER_DIV:
        ret = core->expand.timer_div;
        break;
    default:
        DBGLOG("Illegal META expansion area read(0x%08" HWADDR_PRIx ")\n",
               META_EXPANSION_BASE + addr);
        break;
    }

    return ret;
}

static void meta_expansion_io_write(void *opaque, hwaddr addr,
                                    uint64_t val, unsigned size)
{
    MetaCore *core = opaque;

    addr &= -8;
    if (addr & -0x80) {
        return;
    }

    switch (addr) {
    case META_EXPAND_TIMER_DIV:
        core->expand.timer_div = val & META_EXPAND_TIMER_DIV_MASK;
        break;
    default:
        DBGLOG("unhandled META expansion area write(0x%08" HWADDR_PRIx
               ", 0x%08" PRIx64 ")\n", META_EXPANSION_BASE + addr, val);
        break;
    }
}

static const MemoryRegionOps expansion_io_ops = {
    .read = meta_expansion_io_read,
    .write = meta_expansion_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static uint64_t meta_cache_io_read(void *opaque, hwaddr addr,
                                   unsigned size)
{
    /* FIXME fail with GPE */
    DBGLOG("Illegal META cache flush read(0x%08" HWADDR_PRIx ")\n",
           META_CACHEFLUSH_BASE + addr);
    return 0;
}

static void meta_cache_io_write(void *opaque, hwaddr addr,
                                uint64_t val, unsigned size)
{
    /* there are no caches to flush; code writes are tracked by qemu */
}

static const MemoryRegionOps cacheflush_io_ops = {
    .read = meta_cache_io_read,
    .write = meta_cache_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static uint64_t meta_tlb_io_read(void *opaque, hwaddr addr,
                                 unsigned size)
{
    /* FIXME fail with GPE */
    DBGLOG("Illegal META tlb flush read(0x%08" HWADDR_PRIx ")\n",
           META_TLBFLUSH_BASE + addr);
    return 0;
}

static void meta_tlb_io_write(void *opaque, hwaddr addr,
                              uint64_t val, unsigned size)
{
    MetaCore *core = opaque;
    int t;
    switch (addr) {
    case 0x0:   /* All threads TLB invalidate */
        for (t = 0; t < core->num_threads; ++t) {
            tlb_flush(CPU(&core->threads[t]));
        }
        break;

    case 0x20:  /* Thread 0 TLB invalidate */
    case 0x28:  /* Thread 1 TLB invalidate */
    case 0x30:  /* Thread 2 TLB invalidate */
    case 0x38:  /* Thread 3 TLB invalidate */
        t = (addr - 0x20) >> 3;
        if (t < core->num_threads) {
            tlb_flush(CPU(&core->threads[t]));
        }
        break;

    default:
        DBGLOG("unhandled META tlb flush write(0x%08" HWADDR_PRIx ", 0x%08"
               PRIx64 ")\n", META_TLBFLUSH_BASE + addr, val);
    }
}

static const MemoryRegionOps tlbflush_io_ops = {
    .read = meta_tlb_io_read,
    .write = meta_tlb_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static uint64_t meta_coreregs_io_read(void *opaque, hwaddr addr,
                                      unsigned size)
{
    MetaCore *core = opaque;
    unsigned int offset;
    unsigned int reg;
    uint32_t ret = 0;
    unsigned int t;

    /* invalid sized access */
    if (size != 4) {
        DBGLOG("Bad sized META read%d(0x%08" HWADDR_PRIx ")\n", size*8, addr);
        return 0;
    }

    /* 8 byte stride between 32bit registers */
    if (addr & 0x7) {
        return 0;
    }

    t = (addr >> META_COREREGS_TSHIFT);
    if (t < core->num_threads) {
        /* per-thread core registers */
        offset = addr & (META_COREREGS_TSTRIDE-1);
        reg = offset >> 3;
        if (reg < 32) {
            /* memory mapped CT registers */
            ret = meta_core_intreg_read(&core->threads[t].env, META_UNIT_CT, reg);
        } else {
            switch (offset) {
            case META_TXKICK:
            case META_TXKICKI:
                ret = 0;
                break;
            default:
                DBGLOG("unhandled ");
            }
        }
    } else if (t >= META_MAX_THREADS) {
        /* global core registers */
        switch (addr) {
        /* code breakpoints */
        case META_CODEB0ADDR:
        case META_CODEB1ADDR:
        case META_CODEB2ADDR:
        case META_CODEB3ADDR:
            t = (addr - META_CODEB0ADDR) / META_CODEB_STRIDE;
            ret = core->codebaddr[t];
            break;
        case META_CODEB0CTRL:
        case META_CODEB1CTRL:
        case META_CODEB2CTRL:
        case META_CODEB3CTRL:
            t = (addr - META_CODEB0CTRL) / META_CODEB_STRIDE;
            ret = core->codebctrl[t];
            ret |= core->global.codebcount[t] << META_CODEBXCTRL_COUNT_SHIFT;
            break;
        /* performance counters */
        case META_PERFCOUNT0:
            ret = core->perf_count[0];
            break;
        case META_PERFCOUNT1:
            ret = core->perf_count[1];
            break;
        /* internal register port */
        case META_TXUXXRXDT:
            ret = core->txuxxrx_dt;
            break;
        case META_TXUXXRXRQ:
            ret = core->txuxxrx_rq;
            break;
        default:
            DBGLOG("unhandled ");
        }
    }
    DBGLOG("META core reg read(0x%08" HWADDR_PRIx ") = 0x%08x\n",
           META_COREREGS_BASE + addr, ret);
    return ret;
}

/* update a h/w breakpoint, invalidating any tb's affected by it */
static void meta_codeb_update(MetaCore *core, int codeb)
{
    /* the translator matches the CODEB registers into the code it emits */
    queue_tb_flush(CPU(&core->threads[0]));
}

static void meta_coreregs_io_write(void *opaque, hwaddr addr,
                                   uint64_t val, unsigned size)
{
    MetaCore *core = opaque;
    unsigned int offset;
    unsigned int reg;
    unsigned int t;

    /* invalid sized access */
    if (size != 4) {
        DBGLOG("Bad sized META write%d(0x%08" HWADDR_PRIx ", 0x%" PRIx64
               ")\n", size*8, addr, val);
        return;
    }

    /* 8 byte stride between 32bit registers */
    if (addr & 0x7) {
        return;
    }

    t = (addr >> META_COREREGS_TSHIFT);
    if (t < core->num_threads) {
        /* per-thread core registers */
        offset = addr & (META_COREREGS_TSTRIDE-1);
        reg = offset >> 3;
        if (reg < 32) {
            /* memory mapped CT registers */
            meta_core_intreg_write(&core->threads[t].env, META_UNIT_CT, reg, val);
        } else {
            switch (offset) {
            case META_TXKICK:
                core->threads[t].env.kicks += (val & 0xffff);
                do_bgtrigger(&core->threads[t].env, META_TRIGGER_KICK_MASK);
                break;
            case META_TXKICKI:
                core->threads[t].env.ikicks += (val & 0xffff);

                do_itrigger(&core->threads[t].env, META_TRIGGER_KICK_MASK);
                break;
            default:
                DBGLOG("unhandled ");
            }
        }
    } else if (t >= META_MAX_THREADS) {
        /* global core registers */
        switch (addr) {
        /* code breakpoints */
        case META_CODEB0ADDR:
        case META_CODEB1ADDR:
        case META_CODEB2ADDR:
        case META_CODEB3ADDR:
            t = (addr - META_CODEB0ADDR) / META_CODEB_STRIDE;
            val &= META_CODEBXADDR_MASK;
            if (val != core->codebaddr[t]) {
                core->codebaddr[t] = val;
                meta_codeb_update(core, t);
            }
            break;
        case META_CODEB0CTRL:
        case META_CODEB1CTRL:
        case META_CODEB2CTRL:
        case META_CODEB3CTRL:
            t = (addr - META_CODEB0CTRL) / META_CODEB_STRIDE;
            /* update count */
            core->global.codebcount[t] = (val & META_CODEBXCTRL_COUNT_MASK)
                                        >> META_CODEBXCTRL_COUNT_SHIFT;
            /* and the rest of it if it has changed */
            val &= META_CODEBXCTRL_MASK;
            if (val != core->codebctrl[t]) {
                core->codebctrl[t] = val;
                meta_codeb_update(core, t);
            }
            break;
        /* performance counters */
        case META_PERFCOUNT0:
            core->perf_count[0] = val;
            break;
        case META_PERFCOUNT1:
            core->perf_count[1] = val;
            break;
        /* internal register port */
        case META_TXUXXRXDT:
            core->txuxxrx_dt = val;
            break;
        case META_TXUXXRXRQ:
            {
                /*
                 * Request a read or write of a core internal register.
                 * Value read or written to TXUXXRXDT.
                 */
                uint32_t dsp, rnw;
                uint8_t u, r;
                val &= META_TXUXXRXRQ_MASK;
                dsp = val & META_TXUXXRXRQ_DSPEXT_MASK;
                rnw = val & META_TXUXXRXRQ_RNW_MASK;
                t = (val & META_TXUXXRXRQ_T_MASK) >> META_TXUXXRXRQ_T_SHIFT;
                u = (val & META_TXUXXRXRQ_U_MASK) >> META_TXUXXRXRQ_U_SHIFT;
                r = (val & META_TXUXXRXRQ_R_MASK) >> META_TXUXXRXRQ_R_SHIFT;
                if (u == META_UNIT_CT) {
                    if (rnw) { /* read */
                        core->txuxxrx_dt = 0;
                    }
                    core->txuxxrx_rq = META_TXUXXRXRQ_DREADY_MASK | val;
                    break;
                }
                if (dsp && (u == META_UNIT_D0 || u == META_UNIT_D1)) {
                    int du = u - META_UNIT_D0;
                    if (r & 0x10) {
                        /* accumulator */
                        uint64_t *pacc;
                        bool top = r & 0x8;
                        if (r & 0x3) {
                            /* global */
                            pacc = &core->threads[t].env.global->accregs[du][r & 0x3];
                        } else {
                            /* local */
                            pacc = &core->threads[t].env.accregs[du][r & 0x3];
                        }
                        if (rnw) {
                            /* read */
                            core->txuxxrx_dt = (uint32_t)(*pacc >> (top ? 32 : 0));
                            if (top) {
                                core->txuxxrx_dt &= 0xff;
                            }
                        } else {
                            /* write */
                            if (top) {
                                *pacc = (*pacc & 0xffffffff) |
                                        ((uint64_t)core->txuxxrx_dt << 32);
                            } else {
                                *pacc = (*pacc & 0xff00000000ull) |
                                        core->txuxxrx_dt;
                            }
                        }
                    } else {
                        /* DSP RAM */
                        bool rami = r & 0x8;
                        bool ramb = r & 0x4;
                        bool ramw = r & 0x2;
                        int idx = r & 0x1;
                        uint32_t *pdsp;
                        if (rami) {
                            /* increment */
                            if (ramw) {
                                pdsp = &core->threads[t].env.dspram_wpi[du][ramb][idx];
                            } else {
                                pdsp = &core->threads[t].env.dspram_rpi[du][ramb][idx];
                            }
                        } else {
                            /* pointer */
                            if (ramw) {
                                pdsp = &core->threads[t].env.dspram_wp[du][ramb][idx];
                            } else {
                                pdsp = &core->threads[t].env.dspram_rp[du][ramb][idx];
                            }
                        }
                        if (rnw) {
                            /* read */
                            core->txuxxrx_dt = *pdsp;
                        } else {
                            *pdsp = core->txuxxrx_dt;
                            if (!rami && !ramw) {
                                /* set a read pointer, perform prefetch */
                                DBGLOG("unimplemented TXUXXRXRQ DSPRAM prefetch");
                            }
                        }
                    }
                    core->txuxxrx_rq = META_TXUXXRXRQ_DREADY_MASK | val;
                    break;
                }
                if (rnw) { /* read the value */
                    core->txuxxrx_dt = meta_core_intreg_read(&core->threads[t].env, u, r);
                } else { /* write the value */
                    meta_core_intreg_write(&core->threads[t].env, u, r, core->txuxxrx_dt);
                }
                core->txuxxrx_rq = META_TXUXXRXRQ_DREADY_MASK | val;
            }
            break;
        default:
            DBGLOG("unhandled ");
        }
    }
    DBGLOG("META core reg write(0x%08" HWADDR_PRIx ", 0x%08" PRIx64 ")\n",
           META_COREREGS_BASE + addr, val);
}

static const MemoryRegionOps coreregs_io_ops = {
    .read = meta_coreregs_io_read,
    .write = meta_coreregs_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static uint64_t meta_privregs_io_read(void *opaque, hwaddr addr,
                                      unsigned size)
{
    uint32_t ret = 0;

    /* invalid sized access */
    if (size != 4) {
        DBGLOG("Bad sized META read%d(0x%08" HWADDR_PRIx ")\n", size*8, addr);
        return 0;
    }

    DBGLOG("META priv read(0x%08" HWADDR_PRIx ") = 0x%08x\n",
            META_PRIVREGS_BASE + addr, ret);
    return ret;
}

static void meta_privregs_io_write(void *opaque, hwaddr addr,
                                   uint64_t val, unsigned size)
{
    /* invalid sized access */
    if (size != 4) {
        DBGLOG("Bad sized META write%d(0x%08" HWADDR_PRIx ", 0x%" PRIx64 ")\n",
               size*8, addr, val);
        return;
    }

    DBGLOG("META priv write(0x%08" HWADDR_PRIx ", 0x%08" PRIx64 ")\n",
            META_CTRLREGS_BASE + addr, val);
}

static const MemoryRegionOps privregs_io_ops = {
    .read = meta_privregs_io_read,
    .write = meta_privregs_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static uint64_t meta_control_io_read(void *opaque, hwaddr addr,
                                     unsigned size)
{
    MetaCore *core = opaque;
    uint32_t ret = 0;
    int t;

    /* invalid sized access */
    if (size != 4) {
        DBGLOG("Bad sized META read%d(0x%08" HWADDR_PRIx ")\n", size*8, addr);
        return 0;
    }

    /* registers are 8 bytes apart */
    if (addr & 4) {
        return 0;
    }

    switch (addr) {
    /* META control registers */
    case META_METAID:
        ret = core->global.core_rev;
        break;

    case META_MMCUTBLPHYS:
        ret = core->global.mmu_ptroot;
        if (core->global.core_rev >= META_COREREV1(2)) {
            if (core->global.mmu_flags & META_MMU_HTP) {
                ret |= META_MMCUTBLSPHYS_MODE;
            }
        }
        break;

    case META_SYSCCACHEMMUCONFIG:
        if (core->global.mmu) {
            ret = 7;
        } else {
            ret = 0;
        }
        break;
    case META_SYSCDCACHEFLUSH:
    case META_SYSCICACHEFLUSH:
        /* always report cache flush as complete */
        ret = 1;
        break;

    /* Direct maps */
    case META_MMCUDIRECTMAP0ADDR:
    case META_MMCUDIRECTMAP1ADDR:
    case META_MMCUDIRECTMAP2ADDR:
    case META_MMCUDIRECTMAP3ADDR:
        {
            int dmap = (addr - META_MMCUDIRECTMAP0ADDR) >> 4;
            ret = core->directmap_bases[dmap];
        }
        break;

    case META_SYSCDCPART0:
    case META_SYSCDCPART1:
    case META_SYSCDCPART2:
    case META_SYSCDCPART3:
        {
            t = (addr - META_SYSCDCPART0) >> 3;
            ret = core->dcparts[t];
        }
        break;

    case META_SYSCICPART0:
    case META_SYSCICPART1:
    case META_SYSCICPART2:
    case META_SYSCICPART3:
        {
            t = (addr - META_SYSCICPART0) >> 3;
            ret = core->icparts[t];
        }
        break;

    case META_MCMDATAX:
        ret = core->mcm_data;
        break;
    case META_MCMDATAT:
        ret = core->mcm_data;
        meta_debugport_read(core);
        break;
    case META_MCMGCTRL:
        ret = core->mcm_ctrl;
        break;
    case META_MCMSTATUS:
        ret = 1;
        break;

    /* HTP per thread local/global page tables */
    case META_MMCUTBLPHYS_MIN ... META_MMCUTBLPHYS_MAX:
        t = (addr >> META_MMCUTBLPHYS_SHIFT) & (core->num_threads - 1);
        addr &= META_MMCUTBLPHYS_STRIDE - 1;
        ret = core->threads[t].env.mmu_tblphys[addr >> 3];
        break;

    case META_CORE_ID:
        ret = core->global.core_id;
        break;
    case META_CORE_REV:
        ret = ((core->global.core_rev & 0xffff0000) >> 8)
            | (core->global.core_rev & 0xff);
        break;
    case META_CORE_CONFIG2:
        ret = core->global.core_config2;
        break;
    default:
        DBGLOG("unhandled ");
    }

    return ret;
}

static void meta_control_io_write(void *opaque, hwaddr addr,
                                  uint64_t val, unsigned size)
{
    MetaCore *core = opaque;
    int t;

    /* invalid sized access */
    if (size != 4) {
        DBGLOG("Bad sized META write%d(0x%08" HWADDR_PRIx ", 0x%" PRIx64 ")\n",
               size*8, addr, val);
        return;
    }

    /* registers are 8 bytes apart */
    addr &= -8;

    switch (addr) {
    /* META control registers */
    case META_MMCUTBLPHYS:
        /* FIXME priv TxPIOREG */
        if (core->global.core_rev >= META_COREREV1(2)) {
            core->global.mmu_ptroot = val & META_MMCUTBLSPHYS_ADDR_HTP;
            if (val & META_MMCUTBLSPHYS_MODE) {
                core->global.mmu_flags |= META_MMU_HTP;
            } else {
                core->global.mmu_flags &= ~META_MMU_HTP;
            }
        } else {
            core->global.mmu_ptroot = val & META_MMCUTBLSPHYS_ADDR_ATP;
        }
        break;

    case META_SYSCCACHEMMUCONFIG:
        if (META_CORE_COREMAJOR(core) >= 2) {
            if (val & META_SYSCCACHEMMUCONFIG_EB_DCACHE) {
                core->global.mmu_flags |= META_MMU_EB_DCACHE;
                val &= ~META_SYSCCACHEMMUCONFIG_EB_DCACHE;
            } else
                core->global.mmu_flags &= ~META_MMU_EB_DCACHE;

            if (val & META_SYSCCACHEMMUCONFIG_EB_ICACHE) {
                core->global.mmu_flags |= META_MMU_EB_ICACHE;
                val &= ~META_SYSCCACHEMMUCONFIG_EB_ICACHE;
            } else
                core->global.mmu_flags &= ~META_MMU_EB_ICACHE;

            core->global.mmu = val & META_SYSCCACHEMMUCONFIG_EN;
            val &= ~META_SYSCCACHEMMUCONFIG_EN;

            if (val) {
                cpu_abort(current_cpu, "unknown mmu option %08" PRIx64, val);
            }
        } else {
            if (val == 7) {
                core->global.mmu = 1;
            } else if (val == 0) {
                core->global.mmu = 0;
            } else {
                cpu_abort(current_cpu, "unknown mmu option %08" PRIx64, val);
            }
        }
        break;
    case META_SYSCDCACHEFLUSH:
    case META_SYSCICACHEFLUSH:
        break;

    /* Direct maps */
    case META_MMCUDIRECTMAP0ADDR:
    case META_MMCUDIRECTMAP1ADDR:
    case META_MMCUDIRECTMAP2ADDR:
    case META_MMCUDIRECTMAP3ADDR:
        {
            int dmap = (addr - META_MMCUDIRECTMAP0ADDR) >> 4;
            hwaddr ptr, end;

            core->directmap_bases[dmap] = val & (-1 << 23);

            /*
             * Invalidate TLB pages in relevant direct map area in all
             * threads as direct maps don't go through the META TLB.
             */
            ptr = 0x06000000 + (dmap << 23);
            end = ptr + (1 << 23);
            for (t = 0; t < core->num_threads; ++t) {
                for (; ptr < end; ptr += (1 << TARGET_PAGE_BITS)) {
                    tlb_flush_page(CPU(&core->threads[t]), ptr);
                }
            }
        }
        break;

    case META_SYSCDCPART0:
    case META_SYSCDCPART1:
    case META_SYSCDCPART2:
    case META_SYSCDCPART3:
        {
            t = (addr - META_SYSCDCPART0) >> 3;
            core->dcparts[t] = val;
        }
        break;

    case META_SYSCICPART0:
    case META_SYSCICPART1:
    case META_SYSCICPART2:
    case META_SYSCICPART3:
        {
            t = (addr - META_SYSCICPART0) >> 3;
            core->icparts[t] = val;
        }
        break;

    case META_MCMDATAX:
        core->mcm_data = val;
        break;
    case META_MCMDATAT:
        core->mcm_data = val;
        meta_debugport_write(core);
        break;
    case META_MCMGCTRL:
        core->mcm_ctrl = val & META_MCMGCTRL_MASK;
        if (val & META_MCMGCTRL_READ_MASK) {
            meta_debugport_read(core);
        }
        break;
    case META_MCMSTATUS:
        /* read only */
        break;

    /* HTP per thread local/global page tables */
    case META_MMCUTBLPHYS_MIN ... META_MMCUTBLPHYS_MAX:
        if (core->global.core_rev < META_COREREV1(2)) {
            break;
        }
        t = (addr >> META_MMCUTBLPHYS_SHIFT) & (META_MAX_THREADS - 1);
        if (t >= core->num_threads) {
            break;
        }
        addr &= META_MMCUTBLPHYS_STRIDE - 1;
        if (addr & 8) {
            val &= META_MMCUTBLPHYS1_MASK;
        } else {
            val &= META_MMCUTBLPHYS0_MASK;
        }
        core->threads[t].env.mmu_tblphys[addr >> 3] = val;
        break;

    default:
        DBGLOG("unhandled ");
    }

}

static const MemoryRegionOps ctrlregs_io_ops = {
    .read = meta_control_io_read,
    .write = meta_control_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static uint64_t meta_secure_io_read(void *opaque, hwaddr addr,
                                    unsigned size)
{
    MetaCore *core = opaque;

    /* invalid sized access */
    if (size != 4) {
        DBGLOG("Bad sized META read%d(0x%08" HWADDR_PRIx ")\n", size*8, addr);
        return 0;
    }

    /* registers are 8 bytes apart */
    if (addr & 4) {
        return 0;
    }

    /*
     * These registers were introduced in HTP cores, but we always allow them to
     * be read, since they will normally be zero, and meta_boot uses them to
     * pass information to the Linux bootloader.
     */

    if (addr >= META_MAX_GP_SECURE*8) {
        return 0;
    }

    return core->gp_secure[addr >> 3];
}

static void meta_secure_io_write(void *opaque, hwaddr addr,
                                 uint64_t val, unsigned size)
{
    MetaCore *core = opaque;

    /* invalid sized access */
    if (size != 4) {
        DBGLOG("Bad sized META write%d(0x%08" HWADDR_PRIx
               ", 0x%" PRIx64 ")\n", size*8, addr, val);
        return;
    }

    /* registers are 8 bytes apart */
    addr &= -8;

    /* HTP and onwards */
    if (core->global.core_rev < META_COREREV1(2)) {
        return;
    }

    if (addr >= META_MAX_GP_SECURE*8) {
        return;
    }

    core->gp_secure[addr >> 3] = val;
}

static const MemoryRegionOps secureregs_io_ops = {
    .read = meta_secure_io_read,
    .write = meta_secure_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

void meta_reset_core_registers(MetaCore *core)
{
    core->perf_count[0] = 0x0f000000;
    core->perf_count[1] = 0x1f000000;
    core->txuxxrx_dt = 0x00000000;
    core->txuxxrx_rq = 0x80000000;
    memset(core->gp_secure, 0, sizeof(core->gp_secure));
    core->mcm_ctrl = 0xff000000;

    core->expand.timer_div = 50; /* pretend to be 50MHz */
}

/* Memory mapped page tables */

/* META 1 specific memory mapped page table */

static uint64_t meta1_mmpt_io_read(void *opaque, hwaddr addr,
                                   unsigned size)
{
    MetaCore *core = opaque;
    int t = addr >> META1_MMU2ND_TSHIFT;
    int seg = (addr & (META1_MMU1ST_TSTRIDE-1)) >> 2;
    int page = (addr & (META1_MMU2ND_TSTRIDE-1)) >> 2;

    if (size != 4) {
        DBGLOG("bad length META MMPT read(0x%08" HWADDR_PRIx ")\n",
                META_MMUTBL_BASE + addr);
        return 0;
    }

    if (addr >= META1_MMU2ND_OFFSET) {
        /* second level page table */
        return meta_get_pte2(core, t, page);
    } else if (addr >= META1_MMU1ST_OFFSET) {
        /* first level page table */
        return meta_get_pte1(core, t, seg);
    } else if (addr >= META1_MMUROOT_OFFSET) {
        /* root page table */
    }

    DBGLOG("Unhandled META1 MMPT read(0x%08" HWADDR_PRIx ")\n",
            META_MMUTBL_BASE + addr);
    return 0;
}
static void meta1_mmpt_io_write(void *opaque, hwaddr addr,
                                uint64_t val, unsigned size)
{
    MetaCore *core = opaque;
    int t = addr >> META1_MMU2ND_TSHIFT;
    int seg = (addr & (META1_MMU1ST_TSTRIDE-1)) >> 2;

    if (size != 4) {
        DBGLOG("bad length META MMPT write(0x%08" HWADDR_PRIx
                ", 0x%08" PRIx64 ")\n", META_MMUTBL_BASE + addr, val);
        return;
    }

    /*int page = (addr & (META1_MMU2ND_TSTRIDE-1)) >> 2;*/
    if (addr >= META1_MMU2ND_OFFSET) {
        /* second level page table */
        /*meta_set_pte2(core, t, page, val);*/
    } else if (addr >= META1_MMU1ST_OFFSET) {
        /* first level page table */
        meta_set_pte1(core, t, seg, val);
    } else if (addr >= META1_MMUROOT_OFFSET) {
        /* root page table */
    }

    DBGLOG("unhandled META1 MMPT write(0x%08" HWADDR_PRIx
            ", 0x%08" PRIx64 ")\n", META_MMUTBL_BASE + addr, val);
}

static const MemoryRegionOps meta1_mmpt_io_ops = {
    .read = meta1_mmpt_io_read,
    .write = meta1_mmpt_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

void meta_init_core_registers(MetaCore *core, unsigned int extirqs)
{
    MemoryRegion *memory = get_system_memory();

    memory_region_init_io(&core->expansion_iomem, NULL, &expansion_io_ops, core,
                          "meta-expansion", META_EXPANSION_SIZE - 1);
    memory_region_add_subregion(memory, META_EXPANSION_BASE, &core->expansion_iomem);

    memory_region_init_io(&core->cacheflush_iomem, NULL, &cacheflush_io_ops, core,
                          "meta-cacheflush", META_CACHEFLUSH_SIZE - 1);
    memory_region_add_subregion(memory, META_CACHEFLUSH_BASE, &core->cacheflush_iomem);

    memory_region_init_io(&core->tlbflush_iomem, NULL, &tlbflush_io_ops, core,
                          "meta-tlbflush", META_TLBFLUSH_SIZE - 1);
    memory_region_add_subregion(memory, META_TLBFLUSH_BASE, &core->tlbflush_iomem);

    memory_region_init_io(&core->coreregs_iomem, NULL, &coreregs_io_ops, core,
                          "meta-coreregs", META_COREREGS_SIZE - 1);
    memory_region_add_subregion(memory, META_COREREGS_BASE, &core->coreregs_iomem);

    memory_region_init_io(&core->privregs_iomem, NULL, &privregs_io_ops, core,
                          "meta-privregs", META_PRIVREGS_SIZE - 1);
    memory_region_add_subregion(memory, META_PRIVREGS_BASE, &core->privregs_iomem);

    memory_region_init_io(&core->ctrlregs_iomem, NULL, &ctrlregs_io_ops, core,
                          "meta-ctrlregs", META_CTRLREGS_SIZE - 1);
    memory_region_add_subregion(memory, META_CTRLREGS_BASE, &core->ctrlregs_iomem);

    memory_region_init_io(&core->secureregs_iomem, NULL, &secureregs_io_ops, core,
                          "meta-secureregs", META_SECUREREGS_SIZE - 1);
    memory_region_add_subregion(memory, META_SECUREREGS_BASE, &core->secureregs_iomem);

    meta_triggers_init(core, extirqs, &core->trigregs_iomem);
    memory_region_add_subregion(memory, META_TRIGREGS_BASE, &core->trigregs_iomem);

    meta_reset_core_registers(core);

    /*
     * Memory mapped page tables (META 2 MMPTs are like direct maps so no need
     * to register them).
     */
    if (core->global.core_rev < META_COREREV1(2)) {
        memory_region_init_io(&core->mmutbl_iomem, NULL, &meta1_mmpt_io_ops, core,
                              "meta-mmutbl", META_MMUTBL_SIZE - 1);
        memory_region_add_subregion(memory, META_MMUTBL_BASE, &core->mmutbl_iomem);
    }
#if 0
    cpu_register_physical_memory(META_MMUTBL_BASE, META_MMUTBL_SIZE-1,
                                 iomemtype);
#endif
}
