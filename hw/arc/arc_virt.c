/*
 * QEMU ARC CPU
 *
 * Copyright (c) 2023 Synopsys Inc.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see
 * http://www.gnu.org/licenses/lgpl-2.1.html
 */

#include "qemu/osdep.h"
#include "hw/core/boards.h"
#include "hw/arc/virt.h"
#include "cpu.h"

/*
 * A machine is not a device, so its options are QOM properties rather than
 * qdev ones.  "ram-start" defaults to 0x8000_0000 for compatibility reasons.
 */
static void arc_virt_machine_instance_init(Object *obj)
{
    ARCVirtMachineState *vms = ARC_VIRT_MACHINE(obj);

    vms->ram_start = 0x80000000;
    object_property_add_uint64_ptr(obj, "ram-start", &vms->ram_start,
                                   OBJ_PROP_FLAG_READWRITE);
}

/* Definition of parent ARC virtual machine */
static void arc_virt_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

#ifdef TARGET_ARC64
    mc->default_cpu_type = TYPE_ARC_CPU_HS6X;
#else
    mc->default_cpu_type = TYPE_ARC_CPU_ARCHS;
#endif
}

static const TypeInfo arc_virt_machine_info = {
    .name          = TYPE_ARC_VIRT_MACHINE,
    .parent        = TYPE_MACHINE,
    .abstract      = true,
    .instance_size = sizeof(ARCVirtMachineState),
    .instance_init = arc_virt_machine_instance_init,
    .class_size    = sizeof(ARCVirtMachineClass),
    .class_init    = arc_virt_machine_class_init,
};

static void machvirt_machine_init(void)
{
    type_register_static(&arc_virt_machine_info);
}
type_init(machvirt_machine_init);
