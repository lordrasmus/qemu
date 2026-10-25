/*
 * QEMU NDS32 CPU QOM header (target agnostic)
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#ifndef QEMU_NDS32_CPU_QOM_H
#define QEMU_NDS32_CPU_QOM_H

#include "hw/core/cpu.h"

#define TYPE_NDS32_CPU "nds32-cpu"

OBJECT_DECLARE_CPU_TYPE(NDS32CPU, NDS32CPUClass, NDS32_CPU)

#endif
