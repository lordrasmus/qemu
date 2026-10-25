#ifndef QEMU_HW_MILKYMIST_HW_H
#define QEMU_HW_MILKYMIST_HW_H

#include "hw/core/qdev.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/core/sysbus.h"
#include "qapi/error.h"

static inline DeviceState *milkymist_uart_create(hwaddr base,
                                                 qemu_irq irq,
                                                 Chardev *chr)
{
    DeviceState *dev;

    dev = qdev_new("milkymist-uart");
    qdev_prop_set_chr(dev, "chardev", chr);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, irq);

    return dev;
}

static inline DeviceState *milkymist_hpdmc_create(hwaddr base)
{
    DeviceState *dev;

    dev = qdev_new("milkymist-hpdmc");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);

    return dev;
}

static inline DeviceState *milkymist_sysctl_create(hwaddr base,
        qemu_irq gpio_irq, qemu_irq timer0_irq, qemu_irq timer1_irq,
        uint32_t freq_hz, uint32_t system_id, uint32_t capabilities,
        uint32_t gpio_strappings)
{
    DeviceState *dev;

    dev = qdev_new("milkymist-sysctl");
    qdev_prop_set_uint32(dev, "frequency", freq_hz);
    qdev_prop_set_uint32(dev, "systemid", system_id);
    qdev_prop_set_uint32(dev, "capabilities", capabilities);
    qdev_prop_set_uint32(dev, "gpio_strappings", gpio_strappings);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, gpio_irq);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 1, timer0_irq);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 2, timer1_irq);

    return dev;
}

#endif /* QEMU_HW_MILKYMIST_HW_H */
