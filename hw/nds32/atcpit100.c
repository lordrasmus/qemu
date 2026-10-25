/*
 * Andes ATCPIT100 programmable interval timer (32-bit mode only)
 *
 * Register layout from the Linux timer-atcpit100 driver.
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

#define TYPE_ATCPIT100 "atcpit100"
OBJECT_DECLARE_SIMPLE_TYPE(ATCPIT100State, ATCPIT100)

#define NCH         4

#define R_ID_REV    0x00
#define R_CFG       0x10
#define R_INT_EN    0x14
#define R_INT_STA   0x18
#define R_CH_EN     0x1c
#define R_CH_BASE   0x20    /* + 0x10 * ch: CTL, REL, CNT */

typedef struct ATCPIT100Channel {
    struct ATCPIT100State *s;
    int idx;
    ptimer_state *ptimer;
    uint32_t ctl;
    uint32_t reload;
} ATCPIT100Channel;

struct ATCPIT100State {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    qemu_irq irq;
    uint32_t freq_hz;

    uint32_t int_en;
    uint32_t int_sta;
    uint32_t ch_en;
    ATCPIT100Channel ch[NCH];
};

static void atcpit100_update_irq(ATCPIT100State *s)
{
    qemu_set_irq(s->irq, !!(s->int_en & s->int_sta));
}

static void atcpit100_hit(void *opaque)
{
    ATCPIT100Channel *c = opaque;

    c->s->int_sta |= 1u << (4 * c->idx);
    atcpit100_update_irq(c->s);
}

/* Called inside a ptimer transaction. */
static void atcpit100_ch_apply(ATCPIT100State *s, int i)
{
    ATCPIT100Channel *c = &s->ch[i];
    bool run = (s->ch_en >> (4 * i)) & 1;

    if (run && c->reload) {
        ptimer_set_limit(c->ptimer, c->reload, 0);
        ptimer_run(c->ptimer, 0);
    } else {
        ptimer_stop(c->ptimer);
    }
}

static uint64_t atcpit100_read(void *opaque, hwaddr addr, unsigned size)
{
    ATCPIT100State *s = opaque;

    switch (addr) {
    case R_ID_REV:
        return 0x03031000;
    case R_CFG:
        return NCH;
    case R_INT_EN:
        return s->int_en;
    case R_INT_STA:
        return s->int_sta;
    case R_CH_EN:
        return s->ch_en;
    }
    if (addr >= R_CH_BASE && addr < R_CH_BASE + 0x10 * NCH) {
        ATCPIT100Channel *c = &s->ch[(addr - R_CH_BASE) >> 4];

        switch (addr & 0xf) {
        case 0x0:
            return c->ctl;
        case 0x4:
            return c->reload;
        case 0x8:
            return ptimer_get_count(c->ptimer);
        }
    }
    qemu_log_mask(LOG_GUEST_ERROR, "atcpit100: read at 0x%" HWADDR_PRIx "\n",
                  addr);
    return 0;
}

static void atcpit100_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned size)
{
    ATCPIT100State *s = opaque;

    switch (addr) {
    case R_INT_EN:
        s->int_en = val;
        atcpit100_update_irq(s);
        return;
    case R_INT_STA:
        s->int_sta &= ~val;
        atcpit100_update_irq(s);
        return;
    case R_CH_EN:
        s->ch_en = val;
        for (int i = 0; i < NCH; i++) {
            ptimer_transaction_begin(s->ch[i].ptimer);
            atcpit100_ch_apply(s, i);
            ptimer_transaction_commit(s->ch[i].ptimer);
        }
        return;
    }
    if (addr >= R_CH_BASE && addr < R_CH_BASE + 0x10 * NCH) {
        int i = (addr - R_CH_BASE) >> 4;
        ATCPIT100Channel *c = &s->ch[i];

        ptimer_transaction_begin(c->ptimer);
        switch (addr & 0xf) {
        case 0x0:
            c->ctl = val;
            break;
        case 0x4:
            c->reload = val;
            if (c->reload) {
                ptimer_set_limit(c->ptimer, c->reload, 0);
            }
            break;
        case 0x8:
            ptimer_set_count(c->ptimer, val);
            break;
        }
        ptimer_transaction_commit(c->ptimer);
        return;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "atcpit100: write at 0x%" HWADDR_PRIx "\n",
                  addr);
}

static const MemoryRegionOps atcpit100_ops = {
    .read = atcpit100_read,
    .write = atcpit100_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void atcpit100_realize(DeviceState *dev, Error **errp)
{
    ATCPIT100State *s = ATCPIT100(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    for (int i = 0; i < NCH; i++) {
        ATCPIT100Channel *c = &s->ch[i];

        c->s = s;
        c->idx = i;
        c->ptimer = ptimer_init(atcpit100_hit, c, PTIMER_POLICY_LEGACY);
        ptimer_transaction_begin(c->ptimer);
        ptimer_set_freq(c->ptimer, s->freq_hz);
        ptimer_transaction_commit(c->ptimer);
    }
    memory_region_init_io(&s->mmio, OBJECT(s), &atcpit100_ops, s,
                          TYPE_ATCPIT100, 0x1000);
    sysbus_init_mmio(sbd, &s->mmio);
    sysbus_init_irq(sbd, &s->irq);
}

static void atcpit100_reset(DeviceState *dev)
{
    ATCPIT100State *s = ATCPIT100(dev);

    s->int_en = 0;
    s->int_sta = 0;
    s->ch_en = 0;
    for (int i = 0; i < NCH; i++) {
        s->ch[i].ctl = 0;
        s->ch[i].reload = 0;
        ptimer_transaction_begin(s->ch[i].ptimer);
        ptimer_stop(s->ch[i].ptimer);
        ptimer_transaction_commit(s->ch[i].ptimer);
    }
}

static const Property atcpit100_properties[] = {
    DEFINE_PROP_UINT32("clock-frequency", ATCPIT100State, freq_hz, 30000000),
};

static void atcpit100_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = atcpit100_realize;
    device_class_set_legacy_reset(dc, atcpit100_reset);
    device_class_set_props(dc, atcpit100_properties);
}

static const TypeInfo atcpit100_info = {
    .name          = TYPE_ATCPIT100,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ATCPIT100State),
    .class_init    = atcpit100_class_init,
};

static void atcpit100_register(void)
{
    type_register_static(&atcpit100_info);
}

type_init(atcpit100_register)
