/*
 * META SWITCH instruction handling
 *
 * A SWITCH with a recognised code reaches a debug adapter device on the
 * switch bus, selected by its subgroup (and thread for per-thread devices).
 *
 * Copyright (c) 2012 Imagination Technologies
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#ifndef HW_META_META_SWITCH_H
#define HW_META_META_SWITCH_H

#include "hw/core/qdev.h"
#include "exec/vaddr.h"

#define META_SWITCH_MAX_THREADS 4

typedef enum {
    META_SWITCH_USER     = 0x0,
    META_SWITCH_INT      = 0x1,
    META_SWITCH_THREAD   = 0x2,
    META_SWITCH_EXTERNAL = 0x3,
} MetaSwitchScope;

typedef enum {
    META_SWITCH_FILE     = 0x00,
    META_SWITCH_LOGF     = 0x01,
    META_SWITCH_UNEXP    = 0x02,
    META_SWITCH_MISC     = 0x03,
    META_SWITCH_LINUX    = 0x04,

    META_SWITCH_BREAK    = 0x3f,
    META_SWITCH_SUBGROUP_MAX,
} MetaSwitchSubgroup;

#define TYPE_META_SWITCH_BUS "META_SWITCH"
OBJECT_DECLARE_SIMPLE_TYPE(MetaSwitchBus, META_SWITCH_BUS)

#define TYPE_META_SWITCH_DEVICE "meta-switch-device"
OBJECT_DECLARE_TYPE(MetaSwitchDevice, MetaSwitchDeviceClass, META_SWITCH_DEVICE)

struct MetaSwitchBus {
    BusState qbus;
    MetaSwitchDevice *devices[META_SWITCH_SUBGROUP_MAX][META_SWITCH_MAX_THREADS];
};

struct MetaSwitchDevice {
    DeviceState qdev;
    uint8_t thread_mask;
    int (*handler)(MetaSwitchDevice *dev, CPUArchState *env,
                   const uint32_t *args, int nargs,
                   uint32_t *ret, int nret);
};

struct MetaSwitchDeviceClass {
    DeviceClass parent_class;
    uint32_t subgroup;
    bool per_thread;
    int (*init)(MetaSwitchDevice *dev);
};

MetaSwitchBus *meta_switch_bus_new(DeviceState *dev);
/* the bus the core created, or NULL */
MetaSwitchBus *meta_switch_bus_get(void);
bool meta_switch_bus_has_device(MetaSwitchBus *bus);
MetaSwitchDevice *meta_switch_bus_lookup_device(MetaSwitchBus *bus,
                                                CPUArchState *env,
                                                uint8_t subgroup);

int meta_switch_handle(CPUArchState *env, uint32_t swtch, vaddr stp);
char *meta_switch_arg_string(CPUArchState *env, vaddr addr, uint32_t max_len);

static inline MetaSwitchBus *meta_switch_bus_from_device(MetaSwitchDevice *dev)
{
    return META_SWITCH_BUS(qdev_get_parent_bus(DEVICE(dev)));
}

static inline MetaSwitchDevice *meta_switch_sibling(MetaSwitchDevice *dev,
                                                    CPUArchState *env,
                                                    uint8_t subgroup)
{
    return meta_switch_bus_lookup_device(meta_switch_bus_from_device(dev),
                                         env, subgroup);
}

#endif
