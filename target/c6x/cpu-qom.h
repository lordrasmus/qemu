/*
 * QEMU C6x CPU QOM header (target agnostic)
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#ifndef QEMU_C6X_CPU_QOM_H
#define QEMU_C6X_CPU_QOM_H

#include "hw/core/cpu.h"

#define TYPE_C6X_CPU "c6x-cpu"

OBJECT_DECLARE_CPU_TYPE(C6xCPU, C6xCPUClass, C6X_CPU)

#endif
