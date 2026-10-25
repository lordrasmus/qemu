/*
 * QEMU TILE-Gx CPU QOM header (target agnostic)
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#ifndef QEMU_TILEGX_CPU_QOM_H
#define QEMU_TILEGX_CPU_QOM_H

#include "hw/core/cpu.h"

#define TYPE_TILEGX_CPU "tilegx-cpu"

OBJECT_DECLARE_CPU_TYPE(TileGXCPU, TileGXCPUClass, TILEGX_CPU)

#endif
