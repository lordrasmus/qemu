/*
 * QEMU FR-V CPU QOM header (target agnostic)
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#ifndef QEMU_FRV_CPU_QOM_H
#define QEMU_FRV_CPU_QOM_H

#include "hw/core/cpu.h"

#define TYPE_FRV_CPU "frv-cpu"

OBJECT_DECLARE_CPU_TYPE(FRVCPU, FRVCPUClass, FRV_CPU)

#endif
