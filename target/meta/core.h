/*
 *  META virtual CPU header
 *
 *  Copyright (c) 2010 Imagination Technologies
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
#ifndef CORE_META_H
#define CORE_META_H

#include "cpu.h"
#include "system/memory.h"
#include "triggers.h"
#include "hw/meta/meta_switch.h"

#define META_CORE_COREMAJOR(CORE) ((CORE)->global.core_rev >> 24)

#define META_MAX_GP_SECURE 8

/* core memory types */
typedef enum {
    META_CORE_MEM_UNUSED    = 0,
    META_CORE_MEM_CODE      = 1,
    META_CORE_MEM_DATA      = 2,
    META_CORE_MEM_ROM       = 4,

    META_CORE_MEM_CODE_ROM  = META_CORE_MEM_CODE | META_CORE_MEM_ROM,
} MetaCoreMemType;

struct MetaCore {
    /* Global state (MUST be at beginning of MetaCore) */
    MetaGlobalState global;

    /* Hardware thread states */
    METACPU threads[META_MAX_THREADS];
    uint8_t num_threads;

#if !defined(CONFIG_USER_ONLY)
    /* Trigger block */
    MetaTriggerBlock triggers;
#endif

    /* Global timer (before per-thread divide) */
    uint32_t timer_period; /* ns */

    /* Code breakpoints */
    uint32_t codebaddr[4];
    uint32_t codebctrl[4]; /* count is stored in global */

    /* Perf counters */
    /* FIXME implement some perf counters (there's a lot in HTP) */
    uint32_t perf_count[2];

#if !defined(CONFIG_USER_ONLY)
    /* TXUXXRX{DT,RQ} core register port */
    uint32_t txuxxrx_dt;
    uint32_t txuxxrx_rq;

    /* Direct maps */
    hwaddr directmap_bases[4];

    /* FIXME locking when multiple threads can access */

    /* Cache partitioning */
    uint32_t dcparts[META_MAX_THREADS];
    uint32_t icparts[META_MAX_THREADS];

    /* Secure registers */
    uint32_t gp_secure[META_MAX_GP_SECURE];

    /* Debug port state */
    uint32_t mcm_data;
    uint32_t mcm_ctrl;

    struct {
        MetaCoreMemType type;
        hwaddr phys;
        hwaddr size;
    } core_mems[32];

    MemoryRegion expansion_iomem;
    MemoryRegion cacheflush_iomem;
    MemoryRegion tlbflush_iomem;
    MemoryRegion coreregs_iomem;
    MemoryRegion privregs_iomem;
    MemoryRegion ctrlregs_iomem;
    MemoryRegion secureregs_iomem;
    MemoryRegion trigregs_iomem;
    MemoryRegion mmutbl_iomem;

    MetaSwitchBus *switch_bus;
#endif

    /* expansion area */
    struct {
        uint16_t timer_div;
    } expand;
};

#define META_CODEBXADDR_MASK        0xfffffffc
#define META_CODEBXCTRL_EN_MASK     (1 << 31)
#define META_CODEBXCTRL_TONLY_MASK  (1 << 28)
#define META_CODEBXCTRL_COUNT_MASK  0x00ff0000
#define META_CODEBXCTRL_COUNT_SHIFT 16
#define META_CODEBXCTRL_MASK_MASK   0x0000fffc
#define META_CODEBXCTRL_MASK_SHIFT  2
#define META_CODEBXCTRL_THREAD_MASK 0x00000003
#define META_CODEBXCTRL_MASK        0x9000ffff  /* exclude count */

MetaCore *cpu_meta_core_init(const char *cpu_model, unsigned int extirqs);
void cpu_meta_core_reset(MetaCore *core);
void cpu_meta_core_set_timer_freq(MetaCore *core, uint32_t timer_freq);
#if !defined(CONFIG_USER_ONLY)
void cpu_meta_core_mem_init(MetaCore *core, MetaCoreMemType type,
                            hwaddr phys, hwaddr size);
#endif

#endif
