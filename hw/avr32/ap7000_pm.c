/*
 * AT32AP7000 power manager (PM)
 *
 * The clocks are left as a boot loader sets them up on the NGW100:
 * PLL0 = 20 MHz osc0 * 7 = 140 MHz as the main clock, CPU 140 MHz,
 * HSB 70 MHz, PBA 35 MHz, PBB 70 MHz. The PLLs report lock at once and
 * clock changes take effect immediately.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "hw/avr32/ap7000.h"

#define PM_MCCTRL   0x00
#define PM_CKSEL    0x04
#define PM_CPU_MASK 0x08
#define PM_HSB_MASK 0x0c
#define PM_PBA_MASK 0x10
#define PM_PBB_MASK 0x14
#define PM_PLL0     0x20
#define PM_PLL1     0x24
#define PM_IER      0x40
#define PM_IDR      0x44
#define PM_IMR      0x48
#define PM_ISR      0x4c
#define PM_ICR      0x50
#define PM_RCAUSE   0xc0

/* LOCK0, LOCK1, CKRDY, MSKRDY */
#define PM_ISR_READY    0x63

static uint64_t pm_read(void *opaque, hwaddr offset, unsigned size)
{
    AP7000PMState *s = opaque;

    switch (offset) {
    case PM_ISR:
        return PM_ISR_READY;
    case PM_RCAUSE:
        return 1;                       /* power-on reset */
    default:
        return s->regs[offset / 4];
    }
}

static void pm_write(void *opaque, hwaddr offset, uint64_t value,
                     unsigned size)
{
    AP7000PMState *s = opaque;

    switch (offset) {
    case PM_IER:
        s->regs[PM_IMR / 4] |= value;
        break;
    case PM_IDR:
        s->regs[PM_IMR / 4] &= ~value;
        break;
    case PM_ISR:
    case PM_ICR:
    case PM_RCAUSE:
        break;
    default:
        s->regs[offset / 4] = value;
        break;
    }
}

static const MemoryRegionOps pm_ops = {
    .read = pm_read,
    .write = pm_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void pm_reset(DeviceState *dev)
{
    AP7000PMState *s = AP7000_PM(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[PM_MCCTRL / 4] = 1 << 1;                    /* PLLSEL */
    /* CPU = main, HSB = PBB = main / 2, PBA = main / 4 */
    s->regs[PM_CKSEL / 4] = 0x80818000;
    s->regs[PM_CPU_MASK / 4] = 0xffffffff;
    s->regs[PM_HSB_MASK / 4] = 0xffffffff;
    s->regs[PM_PBA_MASK / 4] = 0xffffffff;
    s->regs[PM_PBB_MASK / 4] = 0xffffffff;
    /* PLLCOUNT 16, PLLMUL 6 (x7), PLLDIV 0, osc0, enabled */
    s->regs[PM_PLL0 / 4] = 0x10060001;
}

static void pm_init(Object *obj)
{
    AP7000PMState *s = AP7000_PM(obj);

    memory_region_init_io(&s->mmio, obj, &pm_ops, s, "ap7000-pm", 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
}

static const VMStateDescription vmstate_pm = {
    .name = "ap7000-pm",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, AP7000PMState, 0x100 / 4),
        VMSTATE_END_OF_LIST()
    }
};

static void pm_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, pm_reset);
    dc->vmsd = &vmstate_pm;
}

static const TypeInfo pm_info = {
    .name = TYPE_AP7000_PM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AP7000PMState),
    .instance_init = pm_init,
    .class_init = pm_class_init,
};

static void pm_register_types(void)
{
    type_register_static(&pm_info);
}

type_init(pm_register_types)
