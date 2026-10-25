/*
 * TI C64x+ 64-bit timer, as far as Linux uses it: the low half in dual
 * 32-bit unchained mode, one-shot or continuous.
 *
 * Register layout from the Linux arch/c6x timer64 driver.  The input
 * clock is the CPU clock divided by EMUMGT[19:16].
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/log.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/ptimer.h"
#include "hw/core/qdev-properties.h"
#include "qom/object.h"

#define TYPE_C64X_TIMER64 "c64x-timer64"
OBJECT_DECLARE_SIMPLE_TYPE(C64xTimer64State, C64X_TIMER64)

#define R_EMUMGT    0x04
#define R_CNTLO     0x10
#define R_CNTHI     0x14
#define R_PRDLO     0x18
#define R_PRDHI     0x1c
#define R_TCR       0x20
#define R_TGCR      0x24
#define R_WDTCR     0x28

#define TCR_ENAMODELO_MASK  0xc0
#define TCR_ENAMODELO_ONCE  0x40
#define TCR_ENAMODELO_CONT  0x80
#define TGCR_TIMLORS        0x1

struct C64xTimer64State {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    qemu_irq irq;
    ptimer_state *ptimer;
    uint32_t cpu_freq;

    uint32_t emumgt;
    uint32_t cnthi;
    uint32_t prdlo;
    uint32_t prdhi;
    uint32_t tcr;
    uint32_t tgcr;
    uint32_t wdtcr;
    uint32_t cntlo;     /* count while stopped */
};

static bool timer_running(C64xTimer64State *s)
{
    return (s->tcr & TCR_ENAMODELO_MASK) && (s->tgcr & TGCR_TIMLORS);
}

static uint32_t timer_count(C64xTimer64State *s)
{
    if (!timer_running(s)) {
        return s->cntlo;
    }
    return s->prdlo + 1 - ptimer_get_count(s->ptimer);
}

static void timer_hit(void *opaque)
{
    C64xTimer64State *s = opaque;

    qemu_irq_pulse(s->irq);
    if ((s->tcr & TCR_ENAMODELO_MASK) == TCR_ENAMODELO_ONCE) {
        s->tcr &= ~TCR_ENAMODELO_MASK;
        s->cntlo = s->prdlo;
    }
}

/* Called inside a ptimer transaction */
static void timer_apply(C64xTimer64State *s)
{
    uint32_t div = extract32(s->emumgt, 16, 4);

    ptimer_set_freq(s->ptimer, s->cpu_freq / (div ? div : 1));
    if (timer_running(s)) {
        ptimer_set_limit(s->ptimer, (uint64_t)s->prdlo + 1, 0);
        ptimer_set_count(s->ptimer, (uint64_t)s->prdlo + 1 - s->cntlo);
        ptimer_run(s->ptimer,
                   (s->tcr & TCR_ENAMODELO_MASK) == TCR_ENAMODELO_ONCE);
    } else {
        ptimer_stop(s->ptimer);
    }
}

static uint64_t timer_read(void *opaque, hwaddr addr, unsigned size)
{
    C64xTimer64State *s = opaque;

    switch (addr) {
    case R_EMUMGT:
        return s->emumgt;
    case R_CNTLO:
        return timer_count(s);
    case R_CNTHI:
        return s->cnthi;
    case R_PRDLO:
        return s->prdlo;
    case R_PRDHI:
        return s->prdhi;
    case R_TCR:
        return s->tcr;
    case R_TGCR:
        return s->tgcr;
    case R_WDTCR:
        return s->wdtcr;
    }
    return 0;
}

static void timer_write(void *opaque, hwaddr addr, uint64_t val,
                        unsigned size)
{
    C64xTimer64State *s = opaque;

    ptimer_transaction_begin(s->ptimer);
    s->cntlo = timer_count(s);
    switch (addr) {
    case R_EMUMGT:
        s->emumgt = (s->emumgt & 0xf0000) | (val & 0x3);
        break;
    case R_CNTLO:
        s->cntlo = val;
        break;
    case R_CNTHI:
        s->cnthi = val;
        break;
    case R_PRDLO:
        s->prdlo = val;
        break;
    case R_PRDHI:
        s->prdhi = val;
        break;
    case R_TCR:
        s->tcr = val;
        break;
    case R_TGCR:
        s->tgcr = val;
        break;
    case R_WDTCR:
        s->wdtcr = val;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "c64x-timer64: write at 0x%"
                      HWADDR_PRIx "\n", addr);
        break;
    }
    timer_apply(s);
    ptimer_transaction_commit(s->ptimer);
}

static const MemoryRegionOps timer_ops = {
    .read = timer_read,
    .write = timer_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void timer_reset(DeviceState *dev)
{
    C64xTimer64State *s = C64X_TIMER64(dev);

    /* the timers run at CPU/6 on the C6455 */
    s->emumgt = 6 << 16;
    s->cntlo = s->cnthi = s->prdlo = s->prdhi = 0;
    s->tcr = s->tgcr = s->wdtcr = 0;
    ptimer_transaction_begin(s->ptimer);
    timer_apply(s);
    ptimer_transaction_commit(s->ptimer);
}

static void timer_realize(DeviceState *dev, Error **errp)
{
    C64xTimer64State *s = C64X_TIMER64(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    s->ptimer = ptimer_init(timer_hit, s, PTIMER_POLICY_LEGACY);
    memory_region_init_io(&s->mmio, OBJECT(s), &timer_ops, s,
                          TYPE_C64X_TIMER64, 0x40);
    sysbus_init_mmio(sbd, &s->mmio);
    sysbus_init_irq(sbd, &s->irq);
}

static const Property timer_properties[] = {
    DEFINE_PROP_UINT32("cpu-frequency", C64xTimer64State, cpu_freq,
                       1000000000),
};

static void timer_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = timer_realize;
    device_class_set_legacy_reset(dc, timer_reset);
    device_class_set_props(dc, timer_properties);
}

static const TypeInfo timer_info = {
    .name          = TYPE_C64X_TIMER64,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(C64xTimer64State),
    .class_init    = timer_class_init,
};

static void timer_register(void)
{
    type_register_static(&timer_info);
}

type_init(timer_register)
