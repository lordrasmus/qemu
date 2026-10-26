/*
 * Intel I/O SAPIC
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#ifndef HW_IA64_IOSAPIC_H
#define HW_IA64_IOSAPIC_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_IOSAPIC "ia64-iosapic"
OBJECT_DECLARE_SIMPLE_TYPE(IOSAPICState, IOSAPIC)

#define IOSAPIC_PINS    24

struct IOSAPICState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    ArchCPU *cpu;
    uint32_t select;
    uint64_t rte[IOSAPIC_PINS];
    uint32_t level;
    uint32_t remote_irr;
};

#endif
