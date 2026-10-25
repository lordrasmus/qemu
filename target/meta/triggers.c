/*
 *  META triggers and interrupt block
 *
 *  Copyright (c) 2011 Imagination Technologies
 *  Written by James Hogan
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "triggers.h"
#include "core.h"

#include "qemu/log.h"
#include "qemu/host-utils.h"
#include "hw/core/irq.h"

#define DEBUG_LEVEL 0

#if DEBUG_LEVEL >= 2
#  define DBGLOG(...) fprintf(stderr, ## __VA_ARGS__)
#elif DEBUG_LEVEL >= 1
#  define DBGLOG(...) qemu_log(__VA_ARGS__)
#else
#  define DBGLOG(...) do { } while (0)
#endif

#define META_TRIGREGS_SIZE         0x10000

/* Trigger registers */
typedef enum {
    META_HWSTATMETA         = 0x0000,
    META_HWSTATEXT          = 0x0010,
    META_HWSTATEXT2         = 0x0018,
    META_HWSTATEXT4         = 0x0020,
    META_HWSTATEXT6         = 0x0028,
    META_HWLEVELEXT         = 0x0030,
    META_HWLEVELEXT2        = 0x0038,
    META_HWLEVELEXT4        = 0x0040,
    META_HWLEVELEXT6        = 0x0048,
    /* introduced for Comet PS1 (HTP) */
    META_HWMASKEXT          = 0x0050,
    META_HWMASKEXT2         = 0x0058,
    META_HWMASKEXT4         = 0x0060,
    META_HWMASKEXT6         = 0x0068,

    META_T0VECINT_BHALT     = 0x0500,
    META_T0VECINT_IHALT     = 0x0508,
    META_T0VECINT_PHALT     = 0x0510,
    META_T0VECINT_RHALT     = 0x0518,
    META_T1VECINT_BHALT     = 0x0520,
    META_T1VECINT_IHALT     = 0x0528,
    META_T1VECINT_PHALT     = 0x0530,
    META_T1VECINT_RHALT     = 0x0538,
    META_T2VECINT_BHALT     = 0x0540,
    META_T2VECINT_IHALT     = 0x0548,
    META_T2VECINT_PHALT     = 0x0550,
    META_T2VECINT_RHALT     = 0x0558,
    META_T3VECINT_BHALT     = 0x0560,
    META_T3VECINT_IHALT     = 0x0568,
    META_T3VECINT_PHALT     = 0x0570,
    META_T3VECINT_RHALT     = 0x0578,
    META_TXVECINT_STRIDE    = 0x20,

    META_HWVEC0EXT          = 0x0700,
    META_HWVEC20EXT         = 0x1700,
    META_HWVEC40EXT         = 0x2700,
    META_HWVEC60EXT         = 0x3700,
    META_HWVEC20_STRIDE     = 0x1000,
} MetaTriggerRegs;

/* get an interrupt vector */
static int get_meta_irq_vector(MetaCore *core, int n)
{
    uint32_t vec = core->triggers.vec[n >> 3];
    vec = (vec >> ((n & 0x7) << 2)) & 0xf;
    return vec;
}

static void meta_core_trigger(MetaCore *core, uint32_t mask)
{
    int i;
    if (mask) {
        for (i = 0; i < core->num_threads; ++i) {
            meta_trigger_async(&core->threads[i].env, mask,
                               META_TRIGGER_BOTH);
        }
    }
}

/* set an internal interrupt vector */
static void set_meta_int_irq_vector(MetaCore *core, int n, uint8_t trigger)
{
    uint16_t vec = core->triggers.int_vec[n >> 2];
    int shift = 4 * (n & 3);

    /* set up new trigger */
    vec &= ~(0xf << shift);
    vec |= trigger << shift;
    core->triggers.int_vec[n >> 2] = vec;
}

/* set an interrupt vector */
static void set_meta_irq_vector(MetaCore *core, int n, uint8_t trigger)
{
    uint32_t vec = core->triggers.vec[n >> 3];
    int shift = (n & 0x7) << 2;

    /* set up new trigger */
    vec &= ~(0xf << shift);
    vec |= trigger << shift;
    core->triggers.vec[n >> 3] = vec;
}

/* get the trigger vectored on an interrupt, or -1 */
static int get_meta_irq_trigger(MetaCore *core, int n)
{
    uint32_t vec = get_meta_irq_vector(core, n);
    if (vec < 4) {
        return -1;
    }
    return vec;
}

/* vector a chunk of 32 irqs onto trigger numbers */
static uint32_t get_meta_irqs_triggers(MetaCore *core, int start, uint32_t irqs)
{
    uint32_t mask = 0;

    while (irqs) {
        int irq = 31 - clz32(irqs);
        int trigger = get_meta_irq_trigger(core, start + irq);
        if (trigger >= 0) {
            mask |= (1 << trigger);
        }
        irqs &= ~(1 << irq);
    }

    return mask;
}

/* do the core uncount/core without checking level/mask/high */
static void _meta_core_irq_count(MetaCore *core, int n, int count)
{
    /* it must also be vectored to a trigger */
    int vec = get_meta_irq_trigger(core, n);
    if (vec < 0) {
        return;
    }
    /* count or uncount */
    if (count) {
        ++core->triggers.level_counts[vec];
        vec = 1 << vec;
        core->triggers.level_nonzero |= vec;
        meta_core_trigger(core, vec);
    } else if (!--core->triggers.level_counts[vec]) {
        core->triggers.level_nonzero &= ~(1 << vec);
    }
}

/* uncount (count=0) or count (count=1) an interrupt's trigger */
static void meta_core_irq_count(MetaCore *core, int n, int count)
{
    int block = n >> 5;
    int irq = n & 0x1f;
    /* an interrupt must be level, unmasked, high to be counted */
    if (!(core->triggers.level[block]
            & core->triggers.mask[block]
            & core->triggers.level_stat[block]
            & (1 << irq))) {
        return;
    }
    _meta_core_irq_count(core, n, count);
}

/* uncount (count=0) or count (count=1) multiple interrupt */
static void meta_core_irqs_count(MetaCore *core, int block, uint32_t irq_mask,
                                 int count)
{
    /* an interrupt must be level, unmasked, high to be counted */
    irq_mask &= core->triggers.level[block];
    irq_mask &= core->triggers.mask[block];
    irq_mask &= core->triggers.level_stat[block];
    /* go through each interrupt to uncount/count if vectored */
    while (irq_mask) {
        int irq = 31 - clz32(irq_mask);
        _meta_core_irq_count(core, (block << 5) + irq, count);
        irq_mask &= ~(1 << irq);
    }
}

static void meta_core_irq_handler(void *opaque, int n, int level)
{
    MetaCore *core = opaque;
    int block = n >> 5;
    int irq = n & 0x1f;
    uint32_t mask = 1 << irq;
    uint32_t new_level = !!level << irq;
    uint32_t old_level = core->triggers.level_stat[block] & mask;
    uint32_t change_level = new_level ^ old_level;
    int vec = get_meta_irq_trigger(core, n);

    /* unlatched (level sensitive) interrupt */
    meta_core_irq_count(core, n, 0);
    core->triggers.level_stat[block] ^= change_level;
    meta_core_irq_count(core, n, 1);

    if (!(core->triggers.level[block] & mask) && level) {
        /* latched (edge triggered) interrupt */
        if (vec >= 0 && change_level && level) {
            /* raised */
            core->triggers.edge_stat[block] |= mask;
            if (core->triggers.mask[block] & mask) {
                vec = 1 << vec;
                meta_core_trigger(core, vec);
            }
        }
    }
}

static uint64_t meta_trigregs_io_read(void *opaque, hwaddr addr,
                                      unsigned size)
{
    MetaCore *core = opaque;
    int i;
    uint32_t ret = 0;

    switch (addr) {
    case META_HWSTATMETA:
        ret = core->triggers.stat_meta;
        break;

    case META_HWSTATEXT:
    case META_HWSTATEXT2:
    case META_HWSTATEXT4:
    case META_HWSTATEXT6:
        i = (addr - META_HWSTATEXT) >> 3;
        if (i >= META_MAX_EXT_IRQ_BLOCKS) {
            break;
        }
        ret = core->triggers.level[i] & core->triggers.level_stat[i];
        ret |= ~core->triggers.level[i] & core->triggers.edge_stat[i];
        ret &= core->triggers.mask[i];
        break;

    case META_HWLEVELEXT:
    case META_HWLEVELEXT2:
    case META_HWLEVELEXT4:
    case META_HWLEVELEXT6:
        i = (addr - META_HWLEVELEXT) >> 3;
        if (i >= META_MAX_EXT_IRQ_BLOCKS) {
            break;
        }
        ret = core->triggers.level[i];
        break;

    case META_HWMASKEXT:
    case META_HWMASKEXT2:
    case META_HWMASKEXT4:
    case META_HWMASKEXT6:
        if (unlikely(META_CORE_COREMAJOR(core) < 2)) {
            break;
        }
        i = (addr - META_HWMASKEXT) >> 3;
        if (i >= META_MAX_EXT_IRQ_BLOCKS) {
            ret = 0xffffffff;
            break;
        }
        ret = core->triggers.mask[i];
        break;

    case META_T0VECINT_BHALT ... META_T3VECINT_RHALT:
        i = (addr - META_T0VECINT_BHALT) >> 3;
        ret = 0xf & (core->triggers.int_vec[i >> 2] >> (4 * (i & 3)));
        break;

    case META_HWVEC0EXT  ... META_HWVEC0EXT  + (0x18 << 3):
    case META_HWVEC20EXT ... META_HWVEC20EXT + (0x18 << 3):
    case META_HWVEC40EXT ... META_HWVEC40EXT + (0x18 << 3):
    case META_HWVEC60EXT ... META_HWVEC60EXT + (0x18 << 3):
        if (likely(!(addr & 0x7))) {
            /* which block of 32 interrupts? */
            addr -= META_HWVEC0EXT;
            i = addr / META_HWVEC20_STRIDE;
            if (i >= META_MAX_EXT_IRQ_BLOCKS) {
                break;
            }
            /* which interrupt number? */
            addr -= i * META_HWVEC20_STRIDE;
            i = i * 0x20 + (addr >> 3);
            ret = get_meta_irq_vector(core, i);
            break;
        }

        /* fall through */
    default:
        DBGLOG("Unhandled META trigger read(0x%08x)\n",
                META_TRIGREGS_BASE + addr);
    };
    return ret;
}

static void meta_trigregs_io_write(void *opaque, hwaddr addr,
                                   uint64_t val, unsigned size)
{
    MetaCore *core = opaque;
    int i;
    uint32_t changed, trigger;

    switch (addr) {
    case META_HWSTATMETA:
        core->triggers.stat_meta ^= val;
        break;

    case META_HWSTATEXT:
    case META_HWSTATEXT2:
    case META_HWSTATEXT4:
    case META_HWSTATEXT6:
        i = (addr - META_HWSTATEXT) >> 3;
        if (i >= META_MAX_EXT_IRQ_BLOCKS) {
            break;
        }
        changed = core->triggers.edge_stat[i] ^ val;
        /* edge triggered bits, toggle all 1's written */
        core->triggers.edge_stat[i] ^= ~core->triggers.level[i] & val;
        /*
         * level sensitive bits: trigger all 1's written
         * edge sensitive bits; trigger on low to high
         */
        trigger = core->triggers.mask[i] & val &
                  ((changed & ~core->triggers.level[i]) |
                   core->triggers.level[i]);
        trigger = get_meta_irqs_triggers(core, i * 0x20, trigger);
        meta_core_trigger(core, trigger);
        break;

    case META_HWLEVELEXT:
    case META_HWLEVELEXT2:
    case META_HWLEVELEXT4:
    case META_HWLEVELEXT6:
        i = (addr - META_HWLEVELEXT) >> 3;
        if (i >= META_MAX_EXT_IRQ_BLOCKS) {
            break;
        }
        changed = core->triggers.level[i] ^ val;
        if (!changed) {
            break;
        }
        meta_core_irqs_count(core, i, changed, 0);
        core->triggers.level[i] = val;
        meta_core_irqs_count(core, i, changed, 1);
        break;

    case META_HWMASKEXT:
    case META_HWMASKEXT2:
    case META_HWMASKEXT4:
    case META_HWMASKEXT6:
        if (unlikely(META_CORE_COREMAJOR(core) < 2)) {
            break;
        }
        i = (addr - META_HWMASKEXT) >> 3;
        if (i >= META_MAX_EXT_IRQ_BLOCKS) {
            break;
        }
        changed = val ^ core->triggers.mask[i];
        if (!changed) {
            break;
        }
        meta_core_irqs_count(core, i, changed, 0);
        core->triggers.mask[i] = val;
        meta_core_irqs_count(core, i, changed, 1);
        break;

    case META_T0VECINT_BHALT ... META_T3VECINT_RHALT:
        i = (addr - META_T0VECINT_BHALT) >> 3;
        set_meta_int_irq_vector(core, i, val & 0xf);
        break;

    case META_HWVEC0EXT  ... META_HWVEC0EXT  + (0x18 << 3):
    case META_HWVEC20EXT ... META_HWVEC20EXT + (0x18 << 3):
    case META_HWVEC40EXT ... META_HWVEC40EXT + (0x18 << 3):
    case META_HWVEC60EXT ... META_HWVEC60EXT + (0x18 << 3):
        if (likely(!(addr & 0x7))) {
            /* which block of 32 interrupts? */
            addr -= META_HWVEC0EXT;
            i = addr / META_HWVEC20_STRIDE;
            if (i >= META_MAX_EXT_IRQ_BLOCKS) {
                break;
            }
            /* which interrupt number? */
            addr -= i * META_HWVEC20_STRIDE;
            i = i * 0x20 + (addr >> 3);

            meta_core_irq_count(core, i, 0);
            set_meta_irq_vector(core, i, val & 0xf);
            meta_core_irq_count(core, i, 1);
            break;
        }

        /* fall through */
    default:
        DBGLOG("META trigger write(0x%08x, 0x%08" PRIx64 ")\n",
                META_TRIGREGS_BASE + addr, val);
    }
}

static const MemoryRegionOps trigregs_io_ops = {
    .read = meta_trigregs_io_read,
    .write = meta_trigregs_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

void meta_triggers_reset(MetaCore *core)
{
    core->triggers.stat_meta = 0;
    memset(core->triggers.edge_stat, 0, sizeof(core->triggers.edge_stat));
    /* don't clear level_stat, it just caches unlatched interrupt status */
    memset(core->triggers.level, 0, sizeof(core->triggers.level));
    memset(core->triggers.mask, 0xff, sizeof(core->triggers.mask));
    memset(core->triggers.vec, 0, sizeof(core->triggers.vec));
    memset(core->triggers.int_vec, 0, sizeof(core->triggers.int_vec));
}

void meta_triggers_init(MetaCore *core, unsigned int extirqs, MemoryRegion *iomem)
{
    memory_region_init_io(iomem, NULL, &trigregs_io_ops, core,
                          "meta-triggers", META_TRIGREGS_SIZE - 1);

    core->triggers.level_nonzero = 0;
    memset(core->triggers.level_counts, 0, sizeof(core->triggers.level_counts));
    memset(core->triggers.level_stat, 0, sizeof(core->triggers.level_stat));
    /* limit to META_MAX_EXT_IRQS external irqs */
    if (extirqs > META_MAX_EXT_IRQS) {
        warn_report("%s: Number of external irqs %u exceeded maximum of %u",
                    __func__, extirqs, META_MAX_EXT_IRQS);
        extirqs = META_MAX_EXT_IRQS;
    }
    core->triggers.num_ext_irqs = extirqs;
    core->triggers.irqs = qemu_allocate_irqs(meta_core_irq_handler, core,
                                             extirqs);

    meta_triggers_reset(core);
}
