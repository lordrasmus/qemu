/*
 * Intel I/O SAPIC, as programmed by Linux arch/ia64/kernel/iosapic.c
 *
 * Indirect register window (select at 0x00, window at 0x10), EOI at
 * 0x40.  Every redirection entry is delivered to the boot CPU.
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/ia64/iosapic.h"
#include "cpu.h"

#define RTE_VECTOR      0xff
#define RTE_LEVEL       (1u << 15)
#define RTE_MASK        (1u << 16)

static void iosapic_deliver(IOSAPICState *s, int n)
{
    uint32_t lo = s->rte[n] & 0xffffffff;

    if (lo & RTE_MASK) {
        return;
    }
    if (lo & RTE_LEVEL) {
        if (s->remote_irr & (1u << n)) {
            return;
        }
        s->remote_irr |= 1u << n;
    }
    ia64_cpu_deliver_vector(s->cpu, lo & RTE_VECTOR);
}

static void iosapic_set_irq(void *opaque, int n, int level)
{
    IOSAPICState *s = opaque;
    bool old = s->level & (1u << n);

    if (level) {
        s->level |= 1u << n;
    } else {
        s->level &= ~(1u << n);
    }
    if (level && (!old || (s->rte[n] & RTE_LEVEL))) {
        iosapic_deliver(s, n);
    }
}

static uint64_t iosapic_read(void *opaque, hwaddr addr, unsigned size)
{
    IOSAPICState *s = opaque;
    uint32_t sel = s->select;

    if (addr != 0x10) {
        return addr == 0 ? s->select : 0;
    }
    if (sel == 0x00) {
        return 0;                               /* id */
    } else if (sel == 0x01) {
        return 0x21 | ((IOSAPIC_PINS - 1) << 16);   /* version, max rte */
    } else if (sel >= 0x10 && sel < 0x10 + 2 * IOSAPIC_PINS) {
        int n = (sel - 0x10) / 2;

        return (sel & 1) ? s->rte[n] >> 32 : (uint32_t)s->rte[n];
    }
    return 0;
}

static void iosapic_write(void *opaque, hwaddr addr, uint64_t val,
                          unsigned size)
{
    IOSAPICState *s = opaque;
    uint32_t sel = s->select;

    switch (addr) {
    case 0x00:
        s->select = val & 0xff;
        break;
    case 0x10:
        if (sel >= 0x10 && sel < 0x10 + 2 * IOSAPIC_PINS) {
            int n = (sel - 0x10) / 2;

            if (sel & 1) {
                s->rte[n] = deposit64(s->rte[n], 32, 32, val);
            } else {
                s->rte[n] = deposit64(s->rte[n], 0, 32, val);
                /* unmasked with the line up: deliver now */
                if ((s->level & (1u << n)) && (s->rte[n] & RTE_LEVEL)) {
                    iosapic_deliver(s, n);
                }
            }
        }
        break;
    case 0x40:      /* EOI: vector */
        for (int n = 0; n < IOSAPIC_PINS; n++) {
            if ((s->rte[n] & RTE_VECTOR) == (val & 0xff)) {
                s->remote_irr &= ~(1u << n);
                if ((s->level & (1u << n)) && (s->rte[n] & RTE_LEVEL)) {
                    iosapic_deliver(s, n);
                }
            }
        }
        break;
    default:
        break;
    }
}

static const MemoryRegionOps iosapic_ops = {
    .read = iosapic_read,
    .write = iosapic_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void iosapic_reset(DeviceState *dev)
{
    IOSAPICState *s = IOSAPIC(dev);

    for (int n = 0; n < IOSAPIC_PINS; n++) {
        s->rte[n] = RTE_MASK;
    }
    s->select = 0;
    s->remote_irr = 0;
}

static void iosapic_init(Object *obj)
{
    IOSAPICState *s = IOSAPIC(obj);

    memory_region_init_io(&s->mmio, obj, &iosapic_ops, s, "iosapic", 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    qdev_init_gpio_in(DEVICE(obj), iosapic_set_irq, IOSAPIC_PINS);
}

static void iosapic_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, iosapic_reset);
}

static const TypeInfo iosapic_info = {
    .name = TYPE_IOSAPIC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IOSAPICState),
    .instance_init = iosapic_init,
    .class_init = iosapic_class_init,
};

static void iosapic_register_types(void)
{
    type_register_static(&iosapic_info);
}

type_init(iosapic_register_types)
