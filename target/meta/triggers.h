/*
 *  META triggers and interrupt block
 *
 *  Copyright (c) 2011 Imagination Technologies
 *  Written by James Hogan
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#ifndef META_TRIGGERS_H
#define META_TRIGGERS_H

#ifndef CONFIG_USER_ONLY

#include "cpu.h"
#include "system/memory.h"
#include "hw/core/irq.h"

#define META_MAX_EXT_IRQS           128
#define META_MAX_EXT_IRQ_BLOCKS     (META_MAX_EXT_IRQS >> 5)
#define META_MAX_EXT_IRQ_VECBLOCKS  (META_MAX_EXT_IRQS >> 3)

typedef struct MetaTriggerBlock {
    /* internal trigger status (edge triggered) */
    uint32_t stat_meta;

    /* logic one, level triggered interrupts on each trigger.
     * When non-zero, writing TXSTAT(I)/HWSTATMETA does not clear bits.
     */
    uint32_t level_nonzero;
    uint8_t  level_counts[32];

    /* number of external irqs (multiple of 32) */
    unsigned int num_ext_irqs;

    /* irq edge triggered status */
    uint32_t edge_stat[META_MAX_EXT_IRQ_BLOCKS];

    /* irq level triggered status */
    uint32_t level_stat[META_MAX_EXT_IRQ_BLOCKS];

    /* irq edge/level model (1=level) */
    uint32_t level[META_MAX_EXT_IRQ_BLOCKS];

    /* irq mask (1=enabled) */
    uint32_t mask[META_MAX_EXT_IRQ_BLOCKS];

    /* 4 bit little endian irq trigger vectors */
    uint32_t vec[META_MAX_EXT_IRQ_VECBLOCKS];

    /* 4 bit little endian trigger matrixing vectors */
    uint16_t int_vec[META_MAX_EXT_IRQ_BLOCKS];

    /* external irqs */
    qemu_irq *irqs;
} MetaTriggerBlock;

void meta_triggers_reset(MetaCore *core);
/* returns iomem type for register region */
void meta_triggers_init(MetaCore *core, unsigned int extirqs, MemoryRegion *iomem);

#endif /* CONFIG_USER_ONLY */
#endif
