/*
 * Blackfin CPU QOM header
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef BFIN_CPU_QOM_H
#define BFIN_CPU_QOM_H

#include "hw/core/cpu.h"

#define TYPE_BFIN_CPU "bfin-cpu"

OBJECT_DECLARE_CPU_TYPE(BfinCPU, BfinCPUClass, BFIN_CPU)

#endif
