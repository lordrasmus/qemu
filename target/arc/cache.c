/*
 * QEMU ARC CPU
 *
 * Copyright (c) 2020 Synppsys Inc.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see
 * http://www.gnu.org/licenses/lgpl-2.1.html
 */

#include "qemu/osdep.h"
#include "hw/core/hw-error.h"
#ifndef CONFIG_USER_ONLY
#endif
#include "cpu.h"
#include "target/arc/regs.h"
#include "target/arc/cache.h"

/* TODO: Revisit this code to optimize which addresses get invalidated. */
static void
arc_invalidate_cache(CPUARCState *env)
{
    CPUState *cs = env_cpu(env);
    tlb_flush(cs);
}

void arc_cache_aux_set(const struct arc_aux_reg_detail *aux_reg_detail,
                       target_ulong val, void *data)
{

    CPUARCState *env = (CPUARCState *) data;
    struct arc_cache *cache = &env->cache;

    switch (aux_reg_detail->id) {
    case AUX_ID_ic_ivic:
    case AUX_ID_ivic:          /* ARCompact name for IC_IVIC */
    case AUX_ID_ic_ivil:
    case AUX_ID_unlockline:    /* ARCompact name for IC_IVIL */
    case AUX_ID_dc_ivdc:
    case AUX_ID_dc_ivdl:
    case AUX_ID_dc_flsh:
    case AUX_ID_dc_fldl:
    case AUX_ID_dc_startr:
	arc_invalidate_cache(env);
	break;

    case AUX_ID_ic_ctrl:
    case AUX_ID_che_mode:  /* ARCompact name for IC_CTRL */
	arc_invalidate_cache(env);
        cache->ic_disabled = val & 1;
        break;

    case AUX_ID_ic_ivir:
	arc_invalidate_cache(env);
        cache->ic_ivir = val & 0xffffff00;
        break;

    case AUX_ID_ic_endr:
	arc_invalidate_cache(env);
        cache->ic_endr = val & 0xffffff00;
        break;

    case AUX_ID_ic_ptag:
        cache->ic_ptag = val;
        break;

    case AUX_ID_ic_ptag_hi:
        cache->ic_ptag_hi = val & 0xff;
        break;

/*
 * Description of the register content in order:
 *   DC - Disable Cache: Enables/Disables the cache: 0 - Enabled, 1 - Disabled
 *   IM - Invalidate Mode: Selects the invalidate type
 */
    case AUX_ID_dc_ctrl:
        cache->dc_disabled = val & 1; /* DC */
        cache->dc_inv_mode = (val >> 6) & 1; /* IM */
        break;

    case AUX_ID_dc_endr:
        cache->dc_endr = val & 0xffffff00;
        break;

    /*
     * ARCompact's aliasing D-cache needs the physical tag next to the
     * virtual address when invalidating a line: __sync_icache_dcache()
     * writes DC_PTAG and then DC_IVDL.  Nothing here caches anything, so
     * storing the value is enough -- but the register has to exist, or the
     * flush loop dies on an illegal instruction.
     */
    case AUX_ID_dc_ptag:
        cache->dc_ptag = val;
        break;

    case AUX_ID_dc_ptag_hi:
        cache->dc_ptag_hi = val & 0xff;
        break;

    default:
#ifndef CONFIG_USER_ONLY
        hw_error("%s@%d: Attempt to write read-only register 0x%02x!\n",
                 __func__, __LINE__, (unsigned int)aux_reg_detail->id);
#endif
        break;
    }

    return;
}

target_ulong arc_cache_aux_get(const struct arc_aux_reg_detail *aux_reg_detail,
                               void *data)
{
    CPUARCState *env = (CPUARCState *) data;
    ARCCPU *cpu = env_archcpu(env);
    struct arc_cache *cache = &env->cache;
    uint32_t reg = 0;

    switch (aux_reg_detail->id) {
/*
 * Description of the register content in order.
 * Layout:  -------- -DFFBBBB CCCCAAAA VVVVVVVV
 *   D - indicates that IC is disabled on reset
 *   FL - Feature level: 10b - line lock, invalidate, advanced debug features
 *   BSize - indicates the cache block size in bytes: 0011b - 64 bytes
 *   Cache capacity: 0111b - 64 Kbytes
 *   Cache Associativiy: 0010b - Four-way set associative
 *   Version number: 4 - ARCv2
 */
    case AUX_ID_ic_build:
        if (cpu->family & ARC_OPCODE_ARCV1) {
            /*
             * ARCompact reports version 3, and the kernel then insists on
             * the fixed geometry that goes with it (see read_decode_cache_bcr
             * in the kernel's arch/arc/mm/cache.c): config must be 3, which
             * means 2-way, and the I-cache must not alias, i.e.
             * size / ways / page size <= 1.  16K/2-way with 8K pages fits,
             * and a 64 byte line matches the kernel's L1_CACHE_BYTES.
             */
            reg = (3 << 16)   /* line size: 8 << 3 = 64 bytes */
                | (5 << 12)   /* capacity:  1 << (5 - 1) = 16 KB */
                | (3 << 8)    /* config: fixed 2-way on version <= 3 */
                | (3 << 0);   /* version: ARCompact */
            break;
        }
        reg = (0 << 22) | /* D */
              (2 << 20) | /* FL */
              (3 << 16) | /* BBSixe*/
              (7 << 12) | /* Cache capacity */
              (2 << 8)  | /* Cache Associativiy */
              (4 << 0);   /* Version Number */
        break;

    case AUX_ID_ic_ctrl:
    case AUX_ID_che_mode:  /* ARCompact name for IC_CTRL */
        reg = cache->ic_disabled & 1;
        break;

    case AUX_ID_ic_ivir:
        reg = cache->ic_ivir;
        break;

    case AUX_ID_ic_endr:
        reg = cache->ic_endr;
        break;

    case AUX_ID_dc_ptag:
        reg = cache->dc_ptag;
        break;

    case AUX_ID_ic_ptag:
        reg = cache->ic_ptag;
        break;

    case AUX_ID_ic_ptag_hi:
        reg = cache->ic_ptag_hi;
        break;

/*
 * Description of the register content in order:
 *   FL - Feature level: 10b - line lock, invalidate, advanced debug features
 *   BSize - indicates the cache block size in bytes: 0010b - 64 bytes
 *   Cache capacity: 0111b - 64 Kbytes
 *   Cache Associativiy: 0001b - Two-way set associative
 *   Version number: 4 - ARCv2 with fixed number of cycles
 */
    case AUX_ID_dc_build:
        if (cpu->family & ARC_OPCODE_ARCV1) {
            /* Same as above; version <= 3 means a fixed 4-way D-cache. */
            reg = (2 << 16)   /* line size: 16 << 2 = 64 bytes */
                | (6 << 12)   /* capacity:  1 << (6 - 1) = 32 KB */
                | (2 << 8)    /* config: fixed 4-way on version <= 3 */
                | (3 << 0);   /* version: ARCompact */
            break;
        }
        reg = (2 << 20) | /* FL */
              (2 << 16) | /* BSize */
              (7 << 12) | /* Cache capacity */
              (1 << 8)  | /* Cache Associativiy */
              (4 << 0);   /* Version number */

	if(IS_ARCV3(cpu)) {
	  reg |= (1 << 30); /* Hardware prefetcher */
	}
        break;

/*
 * Description of the register content in order:
 *   DC - Disable Cache: Enables/Disables the cache: 0 - Enabled, 1 - Disabled
 *   SB - Success Bit: of last cache operation: 1 - succeded (immediately)
 *   IM - Invalidate Mode: Selects the invalidate type
 */
    case AUX_ID_dc_ctrl:
       reg = (cache->dc_disabled & 1) << 0 |  /* DC */
             (1 << 2) |                       /* SB */
             (cache->dc_inv_mode & 1) << 6;   /* IM */
        break;

    case AUX_ID_dc_endr:
        reg = cache->dc_endr;
        break;

    case AUX_ID_dc_ptag_hi:
        reg = cache->dc_ptag_hi;
        break;

    default:
#ifndef CONFIG_USER_ONLY
        hw_error("%s@%d: Attempt to read write-only register 0x%02x!\n",
                 __func__, __LINE__, (unsigned int)aux_reg_detail->id);
#endif
        break;
    }

    return reg;
}
