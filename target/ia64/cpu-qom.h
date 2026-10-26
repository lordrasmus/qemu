/*
 * QEMU IA-64 CPU QOM header (target agnostic)
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#ifndef QEMU_IA64_CPU_QOM_H
#define QEMU_IA64_CPU_QOM_H

#include "hw/core/cpu.h"

#define TYPE_IA64_CPU "ia64-cpu"

OBJECT_DECLARE_CPU_TYPE(IA64CPU, IA64CPUClass, IA64_CPU)

#endif
