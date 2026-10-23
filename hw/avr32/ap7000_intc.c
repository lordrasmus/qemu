/*
 * AT32AP7000 interrupt controller (INTC)
 *
 * 64 groups of up to 32 request lines. Each group has a priority register
 * with the interrupt level and the autovector offset; for every level the
 * controller reports the highest numbered requesting group to the CPU.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "hw/avr32/ap7000.h"
#include "target/avr32/cpu.h"

#define INTC_IPR(n)     (4 * (n))
#define INTC_IRR(n)     (0x100 + 4 * (n))
#define INTC_ICR0       0x20c           /* ICR3 at 0x200 */

static int intc_group_level(AP7000IntcState *s, int g)
{
    return s->ipr[g] >> 30;
}

static void intc_update(AP7000IntcState *s)
{
    int group[4] = { -1, -1, -1, -1 };

    for (int g = 0; g < AP7000_INTC_GROUPS; g++) {
        if (s->irr[g]) {
            group[intc_group_level(s, g)] = g;
        }
    }
    for (int level = 0; level < 4; level++) {
        if (group[level] >= 0) {
            s->icr[level] = group[level];
        }
        avr32_cpu_set_irq(AVR32_CPU(s->cpu), level, group[level] >= 0,
                          group[level] >= 0 ?
                          s->ipr[group[level]] & 0xffffff : 0);
    }
}

static void intc_set_irq(void *opaque, int n, int level)
{
    AP7000IntcState *s = opaque;
    int g = n / 32, line = n % 32;

    if (level) {
        s->irr[g] |= 1u << line;
    } else {
        s->irr[g] &= ~(1u << line);
    }
    intc_update(s);
}

static uint64_t intc_read(void *opaque, hwaddr offset, unsigned size)
{
    AP7000IntcState *s = opaque;

    if (offset < INTC_IRR(0)) {
        return s->ipr[offset / 4];
    }
    if (offset < INTC_IRR(AP7000_INTC_GROUPS)) {
        return s->irr[(offset - INTC_IRR(0)) / 4];
    }
    if (offset >= 0x200 && offset <= INTC_ICR0) {
        return s->icr[(INTC_ICR0 - offset) / 4];
    }
    qemu_log_mask(LOG_GUEST_ERROR, "ap7000_intc: read at 0x%" HWADDR_PRIx
                  "\n", offset);
    return 0;
}

static void intc_write(void *opaque, hwaddr offset, uint64_t value,
                       unsigned size)
{
    AP7000IntcState *s = opaque;

    if (offset < INTC_IRR(0)) {
        /* INTLEVEL and the autovector offset */
        s->ipr[offset / 4] = value & 0xc0ffffff;
        intc_update(s);
        return;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "ap7000_intc: write at 0x%" HWADDR_PRIx
                  "\n", offset);
}

static const MemoryRegionOps intc_ops = {
    .read = intc_read,
    .write = intc_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void intc_reset(DeviceState *dev)
{
    AP7000IntcState *s = AP7000_INTC(dev);

    memset(s->ipr, 0, sizeof(s->ipr));
    memset(s->icr, 0, sizeof(s->icr));
}

static void intc_realize(DeviceState *dev, Error **errp)
{
    AP7000IntcState *s = AP7000_INTC(dev);

    if (!s->cpu) {
        error_setg(errp, "ap7000_intc: 'cpu' link not set");
        return;
    }
    memory_region_init_io(&s->mmio, OBJECT(s), &intc_ops, s, "ap7000-intc",
                          0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->mmio);
    qdev_init_gpio_in(dev, intc_set_irq, AP7000_INTC_GROUPS * 32);
}

static const VMStateDescription vmstate_intc = {
    .name = "ap7000-intc",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(ipr, AP7000IntcState, AP7000_INTC_GROUPS),
        VMSTATE_UINT32_ARRAY(irr, AP7000IntcState, AP7000_INTC_GROUPS),
        VMSTATE_UINT32_ARRAY(icr, AP7000IntcState, 4),
        VMSTATE_END_OF_LIST()
    }
};

static const Property intc_properties[] = {
    DEFINE_PROP_LINK("cpu", AP7000IntcState, cpu, TYPE_CPU, CPUState *),
};

static void intc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = intc_realize;
    device_class_set_legacy_reset(dc, intc_reset);
    dc->vmsd = &vmstate_intc;
    device_class_set_props(dc, intc_properties);
}

static const TypeInfo intc_info = {
    .name = TYPE_AP7000_INTC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AP7000IntcState),
    .class_init = intc_class_init,
};

static void intc_register_types(void)
{
    type_register_static(&intc_info);
}

type_init(intc_register_types)
