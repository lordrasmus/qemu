/*
 * QEMU META CPU QOM header (target agnostic)
 *
 * Copyright (c) 2012 Imagination Technologies
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#ifndef QEMU_META_CPU_QOM_H
#define QEMU_META_CPU_QOM_H

#include "hw/core/cpu.h"

#define TYPE_META_CPU "meta-cpu"

OBJECT_DECLARE_CPU_TYPE(METACPU, METACPUClass, META_CPU)

#endif
