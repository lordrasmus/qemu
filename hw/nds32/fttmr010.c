/*
 * Faraday FTTMR010 timer (three 32-bit timers)
 *
 * Only what the Andes Linux 3.4 BSP (arch/nds32/platforms/timer.c) uses:
 * free running up/down counters on the bus clock, reload on underflow
 * resp. overflow, the overflow interrupt and the write-one-to-clear
 * status register.  Match registers are stored but never fire.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "qom/object.h"

#define TYPE_FTTMR010 "fttmr010"
OBJECT_DECLARE_SIMPLE_TYPE(FTTMR010State, FTTMR010)

#define NTIMERS     3

#define R_TMCR      0x30
#define R_STATE     0x34
#define R_MASK      0x38

/* TMCR, per timer n: enable bit 3n, clock 3n+1, overflow enable 3n+2 */
#define TMCR_EN(n)      (1u << (3 * (n)))
#define TMCR_OF(n)      (1u << (3 * (n) + 2))
#define TMCR_UP(n)      (1u << (9 + (n)))

/* status / mask, per timer n: match1 3n, match2 3n+1, overflow 3n+2 */
#define ST_ALL(n)       (7u << (3 * (n)))
#define ST_OVF(n)       (1u << (3 * (n) + 2))

typedef struct FTTMR010Timer {
    uint32_t count;     /* value at 'stamp' (or frozen value if disabled) */
    int64_t stamp;      /* QEMU_CLOCK_VIRTUAL ns when 'count' was valid */
    uint32_t load;
    uint32_t match[2];
    QEMUTimer *timer;
} FTTMR010Timer;

struct FTTMR010State {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq[NTIMERS];
    FTTMR010Timer t[NTIMERS];
    uint32_t tmcr;
    uint32_t state;
    uint32_t mask;
    uint32_t freq;
};

static int64_t ticks_to_ns(FTTMR010State *s, uint64_t ticks)
{
    return muldiv64(ticks, NANOSECONDS_PER_SECOND, s->freq);
}

static uint64_t ns_to_ticks(FTTMR010State *s, int64_t ns)
{
    return muldiv64(ns, s->freq, NANOSECONDS_PER_SECOND);
}

static void fttmr010_update_irq(FTTMR010State *s)
{
    for (int n = 0; n < NTIMERS; n++) {
        qemu_set_irq(s->irq[n], !!(s->state & ~s->mask & ST_ALL(n)));
    }
}

/*
 * Bring timer n up to 'now': count the reloads that happened since
 * 'stamp' and latch the overflow status if any did.
 */
static void fttmr010_sync(FTTMR010State *s, int n, int64_t now)
{
    FTTMR010Timer *t = &s->t[n];
    uint64_t ticks, first, period;
    bool up = s->tmcr & TMCR_UP(n);

    if (!(s->tmcr & TMCR_EN(n))) {
        t->stamp = now;
        return;
    }
    ticks = ns_to_ticks(s, now - t->stamp);
    if (ticks == 0) {
        return;
    }
    /* Ticks until the first reload, and between reloads. */
    first = up ? (uint64_t)(0xffffffffu - t->count) + 1 : (uint64_t)t->count + 1;
    period = up ? (uint64_t)(0xffffffffu - t->load) + 1 : (uint64_t)t->load + 1;

    if (ticks < first) {
        t->count = up ? t->count + ticks : t->count - ticks;
    } else {
        uint64_t rest = (ticks - first) % period;

        t->count = up ? t->load + rest : t->load - rest;
        if (s->tmcr & TMCR_OF(n)) {
            s->state |= ST_OVF(n);
        }
    }
    t->stamp += ticks_to_ns(s, ticks);
}

static void fttmr010_schedule(FTTMR010State *s, int n)
{
    FTTMR010Timer *t = &s->t[n];
    uint64_t first;

    if (!(s->tmcr & TMCR_EN(n)) || !(s->tmcr & TMCR_OF(n))) {
        timer_del(t->timer);
        return;
    }
    first = (s->tmcr & TMCR_UP(n)) ? (uint64_t)(0xffffffffu - t->count) + 1
                                   : (uint64_t)t->count + 1;
    timer_mod(t->timer, t->stamp + ticks_to_ns(s, first));
}

static void fttmr010_sync_all(FTTMR010State *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    for (int n = 0; n < NTIMERS; n++) {
        fttmr010_sync(s, n, now);
    }
}

static void fttmr010_schedule_all(FTTMR010State *s)
{
    for (int n = 0; n < NTIMERS; n++) {
        fttmr010_schedule(s, n);
    }
    fttmr010_update_irq(s);
}

static void fttmr010_tick(void *opaque)
{
    FTTMR010State *s = opaque;

    fttmr010_sync_all(s);
    fttmr010_schedule_all(s);
}

static uint64_t fttmr010_read(void *opaque, hwaddr addr, unsigned size)
{
    FTTMR010State *s = opaque;
    int n = addr >> 4;

    fttmr010_sync_all(s);
    if (addr < R_TMCR) {
        switch (addr & 0xc) {
        case 0x0:
            return s->t[n].count;
        case 0x4:
            return s->t[n].load;
        default:
            return s->t[n].match[((addr & 0xc) >> 2) - 2];
        }
    }
    switch (addr) {
    case R_TMCR:
        return s->tmcr;
    case R_STATE:
        return s->state;
    case R_MASK:
        return s->mask;
    case 0x3c:
        return 0x00010003;  /* revision */
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "fttmr010: read 0x%" HWADDR_PRIx "\n",
                      addr);
        return 0;
    }
}

static void fttmr010_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    FTTMR010State *s = opaque;
    int n = addr >> 4;

    fttmr010_sync_all(s);
    if (addr < R_TMCR) {
        switch (addr & 0xc) {
        case 0x0:
            s->t[n].count = val;
            break;
        case 0x4:
            s->t[n].load = val;
            break;
        default:
            s->t[n].match[((addr & 0xc) >> 2) - 2] = val;
            break;
        }
    } else {
        switch (addr) {
        case R_TMCR:
            s->tmcr = val & 0xfff;
            break;
        case R_STATE:
            s->state &= ~val;
            break;
        case R_MASK:
            s->mask = val & 0x1ff;
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                          "fttmr010: write 0x%" HWADDR_PRIx "\n", addr);
            break;
        }
    }
    fttmr010_schedule_all(s);
}

static const MemoryRegionOps fttmr010_ops = {
    .read = fttmr010_read,
    .write = fttmr010_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void fttmr010_reset(DeviceState *dev)
{
    FTTMR010State *s = FTTMR010(dev);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    for (int n = 0; n < NTIMERS; n++) {
        timer_del(s->t[n].timer);
        s->t[n].count = 0;
        s->t[n].load = 0;
        s->t[n].match[0] = s->t[n].match[1] = 0;
        s->t[n].stamp = now;
    }
    s->tmcr = 0;
    s->state = 0;
    s->mask = 0;
    fttmr010_update_irq(s);
}

static void fttmr010_init(Object *obj)
{
    FTTMR010State *s = FTTMR010(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &fttmr010_ops, s, TYPE_FTTMR010,
                          0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    for (int n = 0; n < NTIMERS; n++) {
        sysbus_init_irq(sbd, &s->irq[n]);
        s->t[n].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, fttmr010_tick, s);
    }
}

static const Property fttmr010_properties[] = {
    DEFINE_PROP_UINT32("clock-frequency", FTTMR010State, freq, 20000000),
};

static void fttmr010_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, fttmr010_reset);
    device_class_set_props(dc, fttmr010_properties);
}

static const TypeInfo fttmr010_info = {
    .name = TYPE_FTTMR010,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(FTTMR010State),
    .instance_init = fttmr010_init,
    .class_init = fttmr010_class_init,
};

static void fttmr010_register_types(void)
{
    type_register_static(&fttmr010_info);
}

type_init(fttmr010_register_types)
