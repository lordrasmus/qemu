#ifndef HW_LM32_H
#define HW_LM32_H

#include "hw/char/lm32_juart.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/core/sysbus.h"
#include "qapi/error.h"

static inline DeviceState *lm32_pic_init(qemu_irq cpu_irq)
{
    DeviceState *dev;
    SysBusDevice *d;

    dev = qdev_new("lm32-pic");
    d = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(d, &error_fatal);
    sysbus_connect_irq(d, 0, cpu_irq);

    return dev;
}

static inline DeviceState *lm32_juart_init(Chardev *chr)
{
    DeviceState *dev;

    dev = qdev_new(TYPE_LM32_JUART);
    qdev_prop_set_chr(dev, "chardev", chr);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);

    return dev;
}

#endif
