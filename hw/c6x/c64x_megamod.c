/*
 * TI C64x+ megamodule interrupt controller
 *
 * 128 events, four event combiners and the mux to CPU interrupts 4-15,
 * with the register layout the Linux arch/c6x megamod-pic driver uses.
 * Event flags follow the input level: clearing a flag whose input is
 * still high sets it again.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/log.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "qom/object.h"

#define TYPE_C64X_MEGAMOD "c64x-megamod"
OBJECT_DECLARE_SIMPLE_TYPE(C64xMegamodState, C64X_MEGAMOD)

#define NEVT    128
#define NOUT    12

struct C64xMegamodState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    qemu_irq out[NOUT];

    uint32_t level[4];
    uint32_t evtflag[4];
    uint32_t evtmask[4];
    uint32_t expmask[4];
    uint32_t intmux[3];
};

static void megamod_update(C64xMegamodState *s)
{
    for (int o = 0; o < NOUT; o++) {
        int src = extract32(s->intmux[o / 4], (o % 4) * 8, 7);
        int lvl;

        if (src < 4) {
            lvl = (s->evtflag[src] & ~s->evtmask[src]) != 0;
        } else {
            lvl = extract32(s->level[src / 32], src % 32, 1);
        }
        qemu_set_irq(s->out[o], lvl);
    }
}

static void megamod_event(void *opaque, int n, int level)
{
    C64xMegamodState *s = opaque;
    uint32_t bit = 1u << (n % 32);

    if (level) {
        s->level[n / 32] |= bit;
        s->evtflag[n / 32] |= bit;
    } else {
        s->level[n / 32] &= ~bit;
    }
    megamod_update(s);
}

static uint64_t megamod_read(void *opaque, hwaddr addr, unsigned size)
{
    C64xMegamodState *s = opaque;
    int i = (addr >> 2) & 7;

    switch (addr & ~0x1f) {
    case 0x00:
        return i < 4 ? s->evtflag[i] : 0;
    case 0x80:
        return i < 4 ? s->evtmask[i] : 0;
    case 0xa0:
        return i < 4 ? s->evtflag[i] & ~s->evtmask[i] : 0;
    case 0xc0:
        return i < 4 ? s->expmask[i] : 0;
    case 0xe0:
        return i < 4 ? s->evtflag[i] & ~s->expmask[i] : 0;
    case 0x100:
        return (i >= 1 && i <= 3) ? s->intmux[i - 1] : 0;
    }
    return 0;
}

static void megamod_write(void *opaque, hwaddr addr, uint64_t val,
                          unsigned size)
{
    C64xMegamodState *s = opaque;
    int i = (addr >> 2) & 7;

    switch (addr & ~0x1f) {
    case 0x20:      /* EVTSET */
        if (i < 4) {
            s->evtflag[i] |= val;
        }
        break;
    case 0x40:      /* EVTCLR */
        if (i < 4) {
            s->evtflag[i] &= ~val | s->level[i];
        }
        break;
    case 0x80:
        if (i < 4) {
            s->evtmask[i] = val;
        }
        break;
    case 0xc0:
        if (i < 4) {
            s->expmask[i] = val;
        }
        break;
    case 0x100:
        if (i >= 1 && i <= 3) {
            s->intmux[i - 1] = val;
        }
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "c64x-megamod: write at 0x%" HWADDR_PRIx
                      "\n", addr);
        return;
    }
    megamod_update(s);
}

static const MemoryRegionOps megamod_ops = {
    .read = megamod_read,
    .write = megamod_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void megamod_reset(DeviceState *dev)
{
    C64xMegamodState *s = C64X_MEGAMOD(dev);

    memset(s->evtflag, 0, sizeof(s->evtflag));
    memset(s->evtmask, 0, sizeof(s->evtmask));
    memset(s->expmask, 0xff, sizeof(s->expmask));
    /* events 4-15 to CPU interrupts 4-15 */
    s->intmux[0] = 0x07060504;
    s->intmux[1] = 0x0b0a0908;
    s->intmux[2] = 0x0f0e0d0c;
    for (int i = 0; i < 4; i++) {
        s->evtflag[i] = s->level[i];
    }
}

static void megamod_realize(DeviceState *dev, Error **errp)
{
    C64xMegamodState *s = C64X_MEGAMOD(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->mmio, OBJECT(s), &megamod_ops, s,
                          TYPE_C64X_MEGAMOD, 0x1000);
    sysbus_init_mmio(sbd, &s->mmio);
    qdev_init_gpio_in(dev, megamod_event, NEVT);
    qdev_init_gpio_out(dev, s->out, NOUT);
}

static void megamod_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = megamod_realize;
    device_class_set_legacy_reset(dc, megamod_reset);
}

static const TypeInfo megamod_info = {
    .name          = TYPE_C64X_MEGAMOD,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(C64xMegamodState),
    .class_init    = megamod_class_init,
};

static void megamod_register(void)
{
    type_register_static(&megamod_info);
}

type_init(megamod_register)
