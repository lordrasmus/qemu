/*
 * QEMU V850 CPU QOM header (target agnostic)
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#ifndef QEMU_V850_CPU_QOM_H
#define QEMU_V850_CPU_QOM_H

#include "hw/core/cpu.h"

#define TYPE_V850_CPU "v850-cpu"

OBJECT_DECLARE_CPU_TYPE(V850CPU, V850CPUClass, V850_CPU)

#endif
