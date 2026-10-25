/*
 * ImgTec Debug Adapter (DA)
 *
 * "-device da" puts the DA's SWITCH handlers on the core's switch bus: the
 * misc ops (exit, the CHANBIOS console channels) and the UnExpXXX unwind.
 * The original also had the host file system (dafs) and LogF (dalogf); the
 * first gives the guest the host's files, the second hands guest format
 * strings to the host printf, so neither is carried over.
 *
 * Copyright (C) 2013 Imagination Technologies Ltd.
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "cpu.h"
#include "hw/core/qdev-properties.h"
#include "hw/meta/meta_switch.h"
#include "hw/meta/meta_damisc.h"

#define TYPE_META_DA "da"
OBJECT_DECLARE_SIMPLE_TYPE(MetaDAState, META_DA)

struct MetaDAState {
    DeviceState parent_obj;
    int32_t exit_threads;
};

static void meta_da_realize(DeviceState *qdev, Error **errp)
{
    MetaDAState *s = META_DA(qdev);
    BusState *bus = BUS(meta_switch_bus_get());
    DeviceState *dev;

    if (!bus) {
        error_setg(errp, "the machine has no META SWITCH bus");
        return;
    }

    /* UnExpXXX unwind */
    dev = qdev_new("daunexp");
    if (!qdev_realize_and_unref(dev, bus, errp)) {
        return;
    }

    /* misc */
    dev = qdev_new(TYPE_META_DAMISC);
    if (s->exit_threads >= 0) {
        qdev_prop_set_int32(dev, "exit_threads", s->exit_threads);
    }
    qdev_realize_and_unref(dev, bus, errp);
}

static const Property meta_da_properties[] = {
    DEFINE_PROP_INT32("exit_threads", MetaDAState, exit_threads, -1),
};

static void meta_da_class_initfn(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = meta_da_realize;
    dc->desc = "META Debug Adapter (console channels, exit)";
    device_class_set_props(dc, meta_da_properties);
}

static const TypeInfo meta_da_info = {
    .name          = TYPE_META_DA,
    .parent        = TYPE_DEVICE,
    .instance_size = sizeof(MetaDAState),
    .class_init    = meta_da_class_initfn,
};

static void meta_da_register(void)
{
    type_register_static(&meta_da_info);
}

type_init(meta_da_register)
