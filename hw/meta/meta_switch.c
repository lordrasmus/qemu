/*
 * ImgTec Meta SWITCH bus
 *
 * Copyright (C) 2013 Imagination Technologies Ltd.
 *
 * Authors:
 *  Paul Burton <paul.burton@imgtec.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "cpu.h"
#include "hw/meta/meta_switch.h"
#include "hw/core/sysbus.h"
#include "accel/tcg/cpu-ldst.h"

static MetaSwitchBus *switch_bus;

static void device_realize(DeviceState *qdev, Error **errp)
{
    MetaSwitchDevice *dev = META_SWITCH_DEVICE(qdev);
    MetaSwitchDeviceClass *klass = META_SWITCH_DEVICE_GET_CLASS(dev);
    uint8_t thread_mask;
    int i;

    if (klass->init && klass->init(dev)) {
        error_setg(errp, "SWITCH device subgroup %d failed to start",
                   klass->subgroup);
        return;
    }

    if (klass->per_thread) {
        thread_mask = dev->thread_mask;
    } else {
        thread_mask = (1 << META_SWITCH_MAX_THREADS) - 1;
    }

    for (i = 0; i < META_SWITCH_MAX_THREADS; i++) {
        if (!(thread_mask & (1 << i))) {
            continue;
        }
        if (switch_bus->devices[klass->subgroup][i]) {
            warn_report("Multiple SWITCH devices for thread %d subgroup %d",
                        i, klass->subgroup);
        }
        switch_bus->devices[klass->subgroup][i] = dev;
    }
}

static void device_unrealize(DeviceState *qdev)
{
    MetaSwitchDevice *dev = META_SWITCH_DEVICE(qdev);
    MetaSwitchDeviceClass *klass = META_SWITCH_DEVICE_GET_CLASS(dev);
    int i;

    for (i = 0; i < META_SWITCH_MAX_THREADS; i++) {
        if (switch_bus->devices[klass->subgroup][i] == dev) {
            switch_bus->devices[klass->subgroup][i] = NULL;
        }
    }
}

static void device_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *k = DEVICE_CLASS(klass);

    k->realize = device_realize;
    k->unrealize = device_unrealize;
    k->bus_type = TYPE_META_SWITCH_BUS;
}

#define TYPE_META_SWITCH_BRIDGE "meta_switch_bus-bridge"

static void bridge_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->fw_name = "meta_switch";
    dc->user_creatable = false;
}

static char *bus_get_fw_dev_path(DeviceState *dev)
{
    MetaSwitchDeviceClass *klass = META_SWITCH_DEVICE_GET_CLASS(dev);

    return g_strdup_printf("%s@%02x", qdev_fw_name(dev), klass->subgroup);
}

static void bus_class_init(ObjectClass *klass, const void *data)
{
    BusClass *k = BUS_CLASS(klass);

    k->get_fw_dev_path = bus_get_fw_dev_path;
}

static const TypeInfo meta_switch_types[] = {
    {
        .name = TYPE_META_SWITCH_BUS,
        .parent = TYPE_BUS,
        .instance_size = sizeof(MetaSwitchBus),
        .class_init = bus_class_init,
    }, {
        .name          = TYPE_META_SWITCH_BRIDGE,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(SysBusDevice),
        .class_init    = bridge_class_init,
    }, {
        .name = TYPE_META_SWITCH_DEVICE,
        .parent = TYPE_DEVICE,
        .instance_size = sizeof(MetaSwitchDevice),
        .abstract = true,
        .class_size = sizeof(MetaSwitchDeviceClass),
        .class_init = device_class_init,
    },
};

DEFINE_TYPES(meta_switch_types)

MetaSwitchBus *meta_switch_bus_new(DeviceState *dev)
{
    if (switch_bus) {
        error_report("Can't create a second meta switch bus");
        return NULL;
    }
    if (!dev) {
        dev = qdev_new(TYPE_META_SWITCH_BRIDGE);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    }

    switch_bus = META_SWITCH_BUS(qbus_new(TYPE_META_SWITCH_BUS, dev, NULL));
    return switch_bus;
}

MetaSwitchBus *meta_switch_bus_get(void)
{
    return switch_bus;
}

/*
 * Returns true if any device is attached to the switch bus.
 */
bool meta_switch_bus_has_device(MetaSwitchBus *bus)
{
    int i, t;

    for (i = 0; i < META_SWITCH_SUBGROUP_MAX; i++) {
        for (t = 0; t < META_SWITCH_MAX_THREADS; t++) {
            if (bus->devices[i][t]) {
                return true;
            }
        }
    }

    return false;
}

/*
 * Find the device handling a subgroup for the thread executing env.
 */
MetaSwitchDevice *meta_switch_bus_lookup_device(MetaSwitchBus *bus,
                                                CPUArchState *env,
                                                uint8_t subgroup)
{
    return switch_bus->devices[subgroup][env->thread_num];
}

/*
 * Handle an external SWITCH: the arguments are on the stack below stp, the
 * results go back to the same place.  Returns the device's result code, or
 * a negative value if no device took the SWITCH.
 */
int meta_switch_handle(CPUArchState *env, uint32_t swtch, vaddr stp)
{
    MetaSwitchDevice *dev;
    uint32_t scope, subgroup, res_window, stack_window;
    uint32_t args_onstack[6], *args = args_onstack;
    uint32_t ret_onstack[2], *ret = ret_onstack;
    int nargs, nret, i, retcode, mmu_idx = meta_mmu_index(env);

    if (meta_current_isa(env) == MINIM) {
        scope = (swtch >> 6) & 0x3;
        subgroup = (swtch >> 4) & 0x3;
        res_window = ((swtch >> 3) & 0x1) << 1;
        stack_window = (swtch & 0x7) << 1;
    } else {
        scope = (swtch >> 22) & 0x3;
        subgroup = (swtch >> 16) & 0x3f;
        res_window = (swtch >> 8) & 0xff;
        stack_window = swtch & 0xff;
    }

    nargs = stack_window - res_window;
    nret = res_window;

    if (scope != META_SWITCH_EXTERNAL) {
        return -1;
    }
    if (!switch_bus) {
        return -2;
    }

    /* retrieve the appropriate switch device */
    dev = meta_switch_bus_lookup_device(switch_bus, env, subgroup);
    if (!dev) {
        qemu_log_mask(LOG_UNIMP, "Unhandled external SWITCH subgroup %d\n",
                      subgroup);
        return -3;
    }

    /* allocate enough memory for args & return */
    if (nargs > ARRAY_SIZE(args_onstack)) {
        args = g_malloc0(sizeof(args[0]) * nargs);
    }
    if (nret > ARRAY_SIZE(ret_onstack)) {
        ret = g_malloc0(sizeof(ret[0]) * nret);
    }

    /* retrieve arg values */
    for (i = 0; i < nargs; i++) {
        uint32_t addr = stp - ((res_window + i + 1) << 2);
        args[i] = cpu_ldl_le_mmuidx_ra(env, addr, mmu_idx, 0);
    }

    /* handle the switch */
    retcode = dev->handler(dev, env, args, nargs, ret, nret);

    /* store result */
    for (i = 0; i < nret; i++) {
        uint32_t addr = stp - ((i + 1) << 2);
        cpu_stl_le_mmuidx_ra(env, addr, ret[i], mmu_idx, 0);
    }

    /* cleanup */
    if (args != args_onstack) {
        g_free(args);
    }
    if (ret != ret_onstack) {
        g_free(ret);
    }

    return retcode;
}

char *meta_switch_arg_string(CPUArchState *env, vaddr addr, uint32_t max_len)
{
    char buf[65], *ret;
    int nlen, len = 0;

    buf[sizeof(buf) - 1] = 0;

    /* find the null terminator to detemine string length */
    while (len < max_len) {
        if (cpu_memory_rw_debug(env_cpu(env), addr + len, buf,
                                sizeof(buf) - 1, false) < 0) {
            break;
        }
        nlen = strlen(buf);
        len += nlen;
        if (nlen < (sizeof(buf) - 1)) {
            break;
        }
    }

    /* limit length */
    if (len > max_len) {
        len = max_len;
    }

    /* allocate null-terminated string */
    ret = g_malloc(len + 1);
    ret[len] = 0;

    /* if we only required one buffer read to find length, reuse it */
    if (len < sizeof(buf)) {
        memcpy(ret, buf, len);
        return ret;
    }

    /* re-copy the memory */
    if (cpu_memory_rw_debug(env_cpu(env), addr, ret, len + 1, false) < 0) {
        len = 0;
    }
    ret[len] = 0;
    return ret;
}
