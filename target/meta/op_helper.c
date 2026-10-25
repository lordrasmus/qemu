/*
 *  META helper routines
 *
 *  Copyright (c) 2009 Alexander Graf
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/helper-proto.h"
#include "exec/tb-flush.h"
#include "fpu/softfloat.h"
#include "qemu/log.h"
#include "accel/tcg/cpu-loop.h"
#include "accel/tcg/cpu-ldst.h"
#include "qemu/host-utils.h"
#include "core.h"
#include "bits.h"
#include "dsp_sincos_table.h"

#include <assert.h>
#include <math.h>

/* change define to 1 to enable */
#define DEBUG_ADDRESS    0
#define DEBUG_MODULO     0
#define DEBUG_UNALIGNED  0

/*
 * An unaligned access faults when TXPRIVEXT.UNALIGNFAULT is set (META2 only);
 * the translator then asks for aligned accesses, see META_TBFLAG_ALIGNFAULT.
 * With the fault off the hardware ignores the low address bits and rotates
 * the data; qemu does the natural unaligned access instead.
 */
void meta_cpu_do_unaligned_access(CPUState *cs, vaddr addr,
                                  MMUAccessType access_type,
                                  int mmu_idx, uintptr_t retaddr)
{
    CPUArchState *env = cpu_env(cs);
    uint32_t txstatus;

    /* put reason codes into TXSTATUS */
    txstatus = env->cregs[META_TXSTATUS];
    txstatus &= ~(META_TXSTATUS_FREASON_MASK | META_TXSTATUS_HREASON_MASK);
    txstatus |= (META_SIGNUM_DHF << META_TXSTATUS_HREASON_SHIFT);
    env->cregs[META_TXSTATUS] = txstatus;

    /* restore virtual CPU state */
    if (cpu_restore_state(cs, retaddr)) {
        meta_restore_fault_state(env);
    }

    /* trigger the halt and abandon the current block */
    HELPER(halt)(env);
    cpu_loop_exit(cs);
}

/*
 * Replay memory access in TXCATCH registers.
 * See replay_catchbuffer in arch/metag/kernel/traps.c of Linux kernel.
 */
void HELPER(replay_catch)(CPUArchState *env)
{
    uint32_t catch0 = env->cregs[META_TXCATCH0];
    uint32_t catch1 = env->cregs[META_TXCATCH1];
    uint32_t catch2;
    uint32_t catch3;
    uint32_t txdefr = 0;
    int reg;
    int unit;
    int raxx;
    int base_addr;
    int mask;
    int mmu_idx = meta_mmu_index(env);
    int nogcr;
    int sz;

    env->catch_replay = 2;

    /********* READS or LOADS *************/
    if (catch0 & META_TXCATCH0_READ_MASK) {
        reg = (catch0 & META_TXCATCH0_LDRXX_MASK) >> META_TXCATCH0_LDRXX_SHIFT;
        unit = catch0 & META_TXCATCH0_LDDST_MASK;

        /* check we know how to handle the specified registers */
        if (!unit) {
            raxx = (catch0 & META_TXCATCH0_RAXX_MASK)
                        >> META_TXCATCH0_RAXX_SHIFT;

            switch (raxx) {
            case META_RA:
                sz = 3;
                break;

            case META_RADZ:
            case META_RADX:
            case META_RAM8X:
            case META_RAM16X:
                sz = 2;
                break;

            case META_RAWZ:
            case META_RAWX:
            case META_RAM8X32:
                sz = 1;
                break;

            case META_RABZ:
            case META_RABX:
                sz = 0;
                break;

            default:
                cpu_abort(env_cpu(env), "Catch replay (PC=%08x, TXCATCH={%08x,%08x}): "
                          "RD READ port unimplemented, raxx=%#x\n",
                          env->pc[META_PC], catch0, catch1, raxx);
                goto error;
            }
        } else {
            if (unit & ~(META_TXCATCH0_LDDST_D_MASK | META_TXCATCH0_LDDST_A_MASK |
                         META_TXCATCH0_LDDST_CT_MASK)) {
                cpu_abort(env_cpu(env), "Catch replay (PC=%08x, TXCATCH={%08x,%08x}): "
                          "Unimplemented load units %#x\n",
                          env->pc[META_PC], catch0, catch1, unit);
                goto error;
            }

            raxx = 0;
            sz = (catch0 & META_TXCATCH0_LDL2L1_MASK)
                     >> META_TXCATCH0_LDL2L1_SHIFT;
        }

        /* check global common register priv */
        nogcr = !(env->cregs[META_TXSTATUS] & META_TXSTATUS_PSTAT_MASK)
              && (env->cregs[META_TXPRIVEXT] & META_TXPRIVEXT_GCR_MASK);
        if (nogcr) {
            if (unit & META_TXCATCH0_LDDST_D_MASK) {
                if (reg >= 16) {
                    goto priv_error;
                }
            }
            if (unit & META_TXCATCH0_LDDST_A_MASK) {
                if (reg >= 8 && reg < 16) {
                    goto priv_error;
                }
            }
        }

        /* load the right size of data */
        switch (sz) {
        case 0: /* 8 bit */
            catch2 = cpu_ldub_mmuidx_ra(env, catch1, mmu_idx, 0);
            catch3 = 0;
            break;

        case 1: /* 16 bit */
            catch2 = cpu_lduw_le_mmuidx_ra(env, catch1, mmu_idx, 0);
            catch3 = 0;
            break;

        case 2: /* 32 bit */
            catch2 = cpu_ldl_le_mmuidx_ra(env, catch1, mmu_idx, 0);
            catch3 = 0;
            break;

        default:
        case 3: { /* 64 bit */
                uint64_t data = cpu_ldq_le_mmuidx_ra(env, catch1, mmu_idx, 0);
                catch2 = data;
                catch3 = data >> 32;
                break;
            }
        }
        /* swap halves if LDM16 bit is set */
        if (catch0 & META_TXCATCH0_LDM16_MASK) {
            uint32_t tmp = catch2;
            catch2 = catch3;
            catch3 = tmp;
        }

        /* set read pipeline data */
        if (!unit) {
            switch (raxx) {
            case META_RABZ:
                helper_read_append_rabz(env, catch2);
                break;

            case META_RAWZ:
                helper_read_append_rawz(env, catch2);
                break;

            case META_RADZ:
                helper_read_append_radz(env, catch2);
                break;

            case META_RABX:
                helper_read_append_rabx(env, catch2);
                break;

            case META_RAWX:
                helper_read_append_rawx(env, catch2);
                break;

            case META_RADX:
                helper_read_append_radx(env, catch2);
                break;

            case META_RAM16X:
                helper_read_append_ram16x(env, catch2);
                break;

            case META_RAM8X32:
                helper_read_append_ram8x32(env, catch2);
                break;

            case META_RAM8X:
                helper_read_append_ram8x(env, catch2);
                break;

            default:
                helper_read_append_ra(env, ((uint64_t)catch3 << 32) | catch2);
            }
        }

        /* set the appropriate registers */
        if (unit & META_TXCATCH0_LDDST_CT_MASK) {
            if (reg < 31) {
                env->cregs[reg] = catch2;
            } else {
                env->global->cregs[reg - 31] = catch2;
            }
        }
        if (unit & META_TXCATCH0_LDDST_D0_MASK) {
            if (reg < 16) {
                env->dregs[0][reg] = catch2;
            } else {
                env->global->dregs[0][reg - 16] = catch2;
            }
        }
        if (unit & META_TXCATCH0_LDDST_D1_MASK) {
            if (reg < 16) {
                env->dregs[1][reg] = catch3;
            } else {
                env->global->dregs[1][reg - 16] = catch3;
            }
        }
        if (unit & META_TXCATCH0_LDDST_A0_MASK) {
            if (reg < 8) {
                env->aregs[0][reg] = catch2;
            } else if (reg < 16) {
                env->global->aregs[0][reg - 8] = catch2;
            }
        }
        if (unit & META_TXCATCH0_LDDST_A1_MASK) {
            if (reg < 8) {
                env->aregs[1][reg] = catch3;
            } else if (reg < 16) {
                env->global->aregs[1][reg - 8] = catch3;
            }
        }

        /* lnkget sets link status */
        if (catch0 & META_TXCATCH0_LNKGET_MASK) {
            env->lnkaddr = catch1;
        }
    } else { /********************* WRITES **************/
        catch2 = env->cregs[META_TXCATCH2];
        catch3 = env->cregs[META_TXCATCH3];

        base_addr = catch1 & ~0x7;
        mask = (catch0 & META_TXCATCH0_WMASK_MASK) >> META_TXCATCH0_WMASK_SHIFT;

        /* The mask is an active low byte lane mask. */
        if (!(mask & 0x0f)) {               /* 0000ffff 1st 32bits */
            cpu_stl_le_mmuidx_ra(env, base_addr, catch2, mmu_idx, 0);
        } else {
            if (!(mask & 0x03)) {           /* 000000ff 1st 16bits */
                cpu_stw_le_mmuidx_ra(env, base_addr, catch2, mmu_idx, 0);
            } else if (!(mask & 0x01)) {    /* 0000000f 1st byte */
                cpu_stb_mmuidx_ra(env, base_addr, catch2, mmu_idx, 0);
            } else if (!(mask & 0x02)) {    /* 000000f0 2nd byte */
                cpu_stb_mmuidx_ra(env, base_addr + 1, catch2 >> 8, mmu_idx, 0);
            }
            if (!(mask & 0xc)) {            /* 0000ff00 2nd 16bits */
                cpu_stw_le_mmuidx_ra(env, base_addr + 2, catch2 >> 16, mmu_idx, 0);
            } else if (!(mask & 0x4)) {     /* 00000f00 3rd byte */
                cpu_stb_mmuidx_ra(env, base_addr + 2, catch2 >> 16, mmu_idx, 0);
            } else if (!(mask & 0x8)) {     /* 0000f000 4th byte */
                cpu_stb_mmuidx_ra(env, base_addr + 3, catch2 >> 24, mmu_idx, 0);
            }
        }
        if (!(mask & 0xf0)) {               /* ffff0000 2nd 32bits */
            cpu_stl_le_mmuidx_ra(env, base_addr + 4, catch3, mmu_idx, 0);
        } else {
            if (!(mask & 0x30)) {           /* 00ff0000 3rd 16bits */
                cpu_stw_le_mmuidx_ra(env, base_addr + 4, catch3, mmu_idx, 0);
            } else if (!(mask & 0x10)) {    /* 000f0000 5th byte */
                cpu_stb_mmuidx_ra(env, base_addr + 4, catch3, mmu_idx, 0);
            } else if (!(mask & 0x20)) {    /* 00f00000 6th byte */
                cpu_stb_mmuidx_ra(env, base_addr + 5, catch3 >> 8, mmu_idx, 0);
            }
            if (!(mask & 0xc0)) {           /* ff000000 4th 16bits */
                cpu_stw_le_mmuidx_ra(env, base_addr + 6, catch3 >> 16, mmu_idx, 0);
            } else if (!(mask & 0x40)) {    /* 0f000000 7th byte */
                cpu_stb_mmuidx_ra(env, base_addr + 6, catch3 >> 16, mmu_idx, 0);
            } else if (!(mask & 0x80)) {    /* f0000000 8th byte */
                cpu_stb_mmuidx_ra(env, base_addr + 7, catch3 >> 24, mmu_idx, 0);
            }
        }

        /* lnkset is encoded as a blank byte lane mask (0xff) */
        if (mask == 0xff) {
            /* update defr to indicate a failed lnkset */
            txdefr = env->cregs[META_TXDEFR];
            txdefr &= ~META_TXDEFR_BUSERR_ALL_MASK;
            txdefr |= META_TXDEFR_LNKSET_FAIL;
            env->cregs[META_TXDEFR] = txdefr;
            /* trigger the deferred trigger in a moment */
        }
    }

    env->cregs[META_TXSTATUS] &= ~(META_TXSTATUS_CBIMARKER_MASK
                                    | META_TXSTATUS_CBMARKER_MASK);
    env->catch_replay = 0;

    qemu_log_mask(CPU_LOG_INT, "Catch replay (PC=%08x, TXCATCH={%08x,%08x,%08x,%08x})\n",
              env->pc[META_PC], catch0, catch1, catch2, catch3);

    /* trigger the LNKSET deferred exception */
    if (txdefr) {
        HELPER(defr_trigger)(env, META_DEFR_BUSERR);
    }

    return;

priv_error:
    /* reading/writing registers without appropriate priv */
    HELPER(halt_signum)(env, META_SIGNUM_PFG);
    return;

error:
    /* it's doing something we haven't implemented */
    HELPER(thread_stop)(env);
    cpu_loop_exit(env_cpu(env));
    return;
}

/*
 * Called during unlock.
 * If no thread has the lock and another thread is waiting, give it LOCK1.
 */
uint32_t HELPER(unlock)(CPUArchState *env, uint32_t lock_state, uint32_t lockw_oth)
{
    uint32_t tmp, thr;

    /* not if a thread already has LOCK1, or no threads are waiting */
    if ((lock_state & META_LOCK1_ALL) || !(lock_state & lockw_oth)) {
        return lock_state;
    }

    /*
     * repeat waiting threads to the right, and mask to get next
     * thread with lower id than ours
     */
    tmp = lock_state & META_LOCKW_ALL;
    tmp |= tmp >> META_MAX_THREADS;
    tmp &= (META_LOCKW_ALL >> META_MAX_THREADS) << env->thread_num;
    /* count leading zeros, and separate the largest bit */
    thr = clz32(tmp);
    tmp = 0x80000000u >> thr;
    /* repeat the bits downwards again and mask LOCK1 area */
    tmp |= tmp >> META_MAX_THREADS;
    tmp &= META_LOCK1_ALL;

    if (tmp) {
        /* give that thread the voluntary lock */
        lock_state |= tmp;
        /* and allow it to resume by clearing it's LOCKW */
        lock_state &= ~(tmp << META_MAX_THREADS);
        /* and giving it a kick */
        thr = (31 - thr) & (META_MAX_THREADS - 1);
        do_unblock(&META_THREAD2CORE(env)->threads[thr].env);
    }
    return lock_state;
}

/* get the deferred trigger bits for background level */
static inline uint32_t _txpoll_deferred(uint32_t txdefr)
{
    /* find which deferred bits are active in background mode */
    uint32_t temp = txdefr ^ META_TXDEFR_TRIGICTRL_MASK;
    return txdefr & (temp << 16);
}

/* get the deferred trigger bits for interrupt level */
static inline uint32_t _txpolli_deferred(uint32_t txdefr)
{
    /* find which deferred bits are active in interrupt mode */
    return txdefr & (txdefr << 16);
}

/* get the deferred trigger bits to or with a trigger register */
static inline uint32_t _txpollx_defr_bits(uint32_t txdefr, uint32_t txpollx_defr)
{
    if (txpollx_defr & (META_DEFR_BUSERR_MASK << META_TXDEFR_TRIGSTAT_SHIFT)) {
        /* buserr is active so or in buserr state */
        return txdefr & (META_TXDEFR_BUSERR_ALL_MASK |
                (META_DEFR_BUSERR_MASK << META_TXDEFR_TRIGSTAT_SHIFT));
    } else {
        /* other defr bits may be active */
        return txpollx_defr;
    }
}

static inline uint32_t _defr_ack_mask(uint32_t mask)
{
    if (mask & (META_DEFR_BUSERR_MASK << META_TXDEFR_TRIGSTAT_SHIFT)) {
        mask |= META_TXDEFR_BUSERR_ALL_MASK;
    }
    return mask;
}


/* raw read of TXPOLL */
static inline uint32_t _get_txpoll(CPUArchState *env)
{
    uint32_t result, txdefr;

    result = env->tregs[META_TXSTAT] & env->tregs[META_TXMASK];

    /* upper half is kick count if DEFR isn't highest priority */
    if (likely((result & META_TRIGGER_GE_DEFR_MASK)
                    != META_TRIGGER_DEFR_MASK)) {
        return result | (env->kicks << 16);
    }

    txdefr = env->cregs[META_TXDEFR];
    result |= _txpollx_defr_bits(txdefr, _txpoll_deferred(txdefr));
    return result;
}

/* basic read of TXPOLLI */
static inline uint32_t _get_txpolli(CPUArchState *env)
{
    uint32_t result, txdefr;

    result = env->tregs[META_TXSTATI] & env->tregs[META_TXMASKI];

    /* upper half is kick count if DEFR isn't highest priority */
    if (likely((result & META_TRIGGER_GE_DEFR_MASK)
                    != META_TRIGGER_DEFR_MASK)) {
        return result | (env->ikicks << 16);
    }

    txdefr = env->cregs[META_TXDEFR];
    result |= _txpollx_defr_bits(txdefr, _txpolli_deferred(txdefr));
    return result;
}

/* basic read of TXPOLL */
uint32_t HELPER(get_txpoll)(CPUArchState *env)
{
    return _get_txpoll(env);
}

/* basic read of TXPOLLI */
uint32_t HELPER(get_txpolli)(CPUArchState *env)
{
    return _get_txpolli(env);
}

/* external read of TXPOLL */
uint32_t HELPER(txpoll_read)(CPUArchState *env)
{
    return _get_txpoll(env);
}

/* external read of TXPOLLI */
uint32_t HELPER(txpolli_read)(CPUArchState *env)
{
    return _get_txpolli(env);
}

/* raw DEFR read of TXPOLL */
static inline uint32_t _defr_txpoll(CPUArchState *env)
{
    uint32_t result, txdefr;

    result = env->tregs[META_TXSTAT] & META_TRIGGER_DEFR_MASK;
    txdefr = env->cregs[META_TXDEFR];
    result |= _txpollx_defr_bits(txdefr, _txpoll_deferred(txdefr));
    return result;
}

/* raw DEFR read of TXPOLLI */
static inline uint32_t _defr_txpolli(CPUArchState *env)
{
    uint32_t result, txdefr;

    result = env->tregs[META_TXSTATI] & META_TRIGGER_DEFR_MASK;
    txdefr = env->cregs[META_TXDEFR];
    result |= _txpollx_defr_bits(txdefr, _txpolli_deferred(txdefr));
    return result;
}

/* DEFR read of TXPOLL */
uint32_t HELPER(defr_txpoll)(CPUArchState *env)
{
    return _defr_txpoll(env);
}

/* DEFR read of TXPOLLI */
uint32_t HELPER(defr_txpolli)(CPUArchState *env)
{
    return _defr_txpolli(env);
}

/* ACK deferred background triggers */
static inline void _defr_ack_txstat(CPUArchState *env, uint32_t result)
{
    uint32_t txdefr = env->cregs[META_TXDEFR];
    txdefr &= ~(result & 0xffff0000);
    env->cregs[META_TXDEFR] = txdefr;

    /* ack the trigger if there are no more deferred triggers */
    if (!_txpoll_deferred(txdefr)) {
        env->tregs[META_TXSTAT] &= ~META_TRIGGER_DEFR_MASK;
    }
}

/* ACK deferred interrupt triggers */
static inline void _defr_ack_txstati(CPUArchState *env, uint32_t result)
{
    uint32_t txdefr = env->cregs[META_TXDEFR];
    txdefr &= ~(result & 0xffff0000);
    env->cregs[META_TXDEFR] = txdefr;

    /* ack the trigger if there are no more deferred triggers */
    if (!_txpolli_deferred(txdefr)) {
        env->tregs[META_TXSTATI] &= ~META_TRIGGER_DEFR_MASK;
    }
}

/* DEFR read of TXSTAT (acks) */
uint32_t HELPER(defr_txstat)(CPUArchState *env)
{
    uint32_t result = _defr_txpoll(env);
    /* ACK the deferred triggers, and only the high priority ones in result */
    _defr_ack_txstat(env, result);
    return result;
}

/* DEFR read of TXSTATI (Acks) */
uint32_t HELPER(defr_txstati)(CPUArchState *env)
{
    uint32_t result = _defr_txpolli(env);
    /* ACK the deferred triggers, and only the high priority ones in result */
    _defr_ack_txstati(env, result);
    return result;
}

static inline void _ack_defr_txstat(CPUArchState *env, uint32_t mask)
{
    uint32_t txdefr, result;
    txdefr = env->cregs[META_TXDEFR];
    result = _defr_ack_mask(mask & _txpoll_deferred(txdefr));
    _defr_ack_txstat(env, result);
}

static inline void _ack_defr_txstati(CPUArchState *env, uint32_t mask)
{
    uint32_t txdefr, result;
    txdefr = env->cregs[META_TXDEFR];
    result = _defr_ack_mask(mask & _txpolli_deferred(txdefr));
    _defr_ack_txstati(env, result);
}

/* ack highest priority deferred background trigger bits */
void HELPER(ack_defr_txstat)(CPUArchState *env, uint32_t mask)
{
    _ack_defr_txstat(env, mask);
}

/* ack highest priority deferred interrupt trigger bits */
void HELPER(ack_defr_txstati)(CPUArchState *env, uint32_t mask)
{
    _ack_defr_txstati(env, mask);
}

/* external ack highest priority deferred background trigger bits */
void HELPER(txstat_ack_defr)(CPUArchState *env, uint32_t mask)
{
    _ack_defr_txstat(env, mask);
}

/* external ack highest priority deferred interrupt trigger bits */
void HELPER(txstati_ack_defr)(CPUArchState *env, uint32_t mask)
{
    _ack_defr_txstati(env, mask);
}

/* generate a deferred trigger */
void HELPER(defr_trigger)(CPUArchState *env, uint32_t defr)
{
    uint32_t mask = 1 << defr;
    env->cregs[META_TXDEFR] |= mask << META_TXDEFR_TRIGSTAT_SHIFT;
    if (env->cregs[META_TXDEFR] & mask) {
        do_itrigger(env, META_TRIGGER_DEFR_MASK);
    } else {
        do_bgtrigger(env, META_TRIGGER_DEFR_MASK);
    }
}

void helper_raise_exception(CPUArchState *env, uint32_t index)
{
    env_cpu(env)->exception_index = index;
    cpu_loop_exit(env_cpu(env));
}

/* The trace unit (TT) and memory tracing output are not emulated. */
void HELPER(ttevent0)(uint32_t actcyc, uint32_t header)
{
}

void HELPER(ttevent1)(uint32_t actcyc, uint32_t header, uint32_t d1)
{
}

void HELPER(ttevent2)(uint32_t actcyc, uint32_t header, uint32_t d1,
                      uint32_t d2)
{
}

void HELPER(ttevent3)(uint32_t actcyc, uint32_t header, uint32_t d1,
                      uint32_t d2, uint32_t d3)
{
}

void HELPER(ttevent4)(uint32_t actcyc, uint32_t header, uint32_t d1,
                      uint32_t d2, uint32_t d3, uint32_t d4)
{
}

void HELPER(ifetch)(CPUArchState *env, uint32_t pc, uint32_t actcyc)
{
}

void HELPER(dload)(CPUArchState *env, uint32_t addr, uint32_t l, uint32_t pc,
                   uint32_t actcyc)
{
}

void HELPER(dstore)(CPUArchState *env, uint32_t addr, uint32_t l, uint32_t pc,
                    uint32_t actcyc)
{
}

void HELPER(dprefetch)(CPUArchState *env, uint32_t addr, uint32_t pc,
                       uint32_t actcyc)
{
}

void HELPER(cachew)(CPUArchState *env, uint32_t addr, uint32_t val,
                    uint32_t actcyc)
{
    if (val & 2) {
        tlb_flush_page(env_cpu(env), addr);
    }
    if (val & 0xfc) {
        qemu_log_mask(LOG_UNIMP, "Unimplemented 0xfc bits in CACHEW [%08x], %#x\n",
                      addr, val);
    }
}

float32 HELPER(fx_add32)(CPUArchState *env, float32 a, float32 b)
{
    return float32_add(a, b, &env->fx.status);
}

float64 HELPER(fx_add64)(CPUArchState *env, float64 a, float64 b)
{
    return float64_add(a, b, &env->fx.status);
}

float32 HELPER(fx_mul32)(CPUArchState *env, float32 a, float32 b)
{
    return float32_mul(a, b, &env->fx.status);
}

float64 HELPER(fx_mul64)(CPUArchState *env, float64 a, float64 b)
{
    return float64_mul(a, b, &env->fx.status);
}

float32 HELPER(fx_sub32)(CPUArchState *env, float32 a, float32 b)
{
    return float32_sub(a, b, &env->fx.status);
}

float64 HELPER(fx_sub64)(CPUArchState *env, float64 a, float64 b)
{
    return float64_sub(a, b, &env->fx.status);
}

float32 HELPER(fx_dtof)(CPUArchState *env, float64 d)
{
    return float64_to_float32(d, &env->fx.status);
}

float64 HELPER(fx_ftod)(CPUArchState *env, float32 f)
{
    return float32_to_float64(f, &env->fx.status);
}

uint32_t HELPER(fx_ftoh)(CPUArchState *env, float32 f)
{
    uint32_t ret = float16_val(float32_to_float16(f, 1, &env->fx.status));
    uint32_t exflags = get_float_exception_flags(&env->fx.status);
    if (exflags & float_flag_underflow) {
        /* float32_to_float16 sets underflow when rounding, but the FPU TRM
           states that we should raise an inexact result exception. */
        exflags &= ~float_flag_underflow;
        exflags |= float_flag_inexact;
        set_float_exception_flags(exflags, &env->fx.status);
    }
    return ret;
}

uint32_t HELPER(fx_dtoh)(CPUArchState *env, float64 d)
{
    uint32_t ret = float16_val(float32_to_float16(float64_to_float32(d, &env->fx.status), 1, &env->fx.status));
    uint32_t exflags = get_float_exception_flags(&env->fx.status);
    if (exflags & float_flag_underflow) {
        /* float32_to_float16 sets underflow when rounding, but the FPU TRM
           states that we should raise an inexact result exception. */
        exflags &= ~float_flag_underflow;
        exflags |= float_flag_inexact;
        set_float_exception_flags(exflags, &env->fx.status);
    }
    return ret;
}

float32 HELPER(fx_htof)(CPUArchState *env, uint32_t f)
{
    return float16_to_float32(make_float16(f), 1, &env->fx.status);
}

float64 HELPER(fx_htod)(CPUArchState *env, uint32_t f)
{
    return float32_to_float64(float16_to_float32(make_float16(f), 1, &env->fx.status), &env->fx.status);
}

int32_t HELPER(fx_ftoi)(CPUArchState *env, float32 f, int z_override)
{
    if (z_override)
        return float32_to_int32_round_to_zero(f, &env->fx.status);
    return float32_to_int32(f, &env->fx.status);
}

int32_t HELPER(fx_dtoi)(CPUArchState *env, float64 d, int z_override)
{
    if (z_override)
        return float64_to_int32_round_to_zero(d, &env->fx.status);
    return float64_to_int32(d, &env->fx.status);
}

float32 HELPER(fx_itof)(CPUArchState *env, int32_t i)
{
    return int32_to_float32(i, &env->fx.status);
}

float64 HELPER(fx_itod)(CPUArchState *env, int32_t i)
{
    return int32_to_float64(i, &env->fx.status);
}

int64_t HELPER(fx_dtol)(CPUArchState *env, float64 d, int z_override)
{
    if (z_override)
        return float64_to_int64_round_to_zero(d, &env->fx.status);
    return float64_to_int64(d, &env->fx.status);
}

float64 HELPER(fx_ltod)(CPUArchState *env, int64_t l)
{
    return int64_to_float64(l, &env->fx.status);
}

int32_t HELPER(fx_ftox)(CPUArchState *env, float32 src, int fracbits)
{
    float32 tmp;
    if (float32_is_signaling_nan(src, &env->fx.status) || float32_is_infinity(src)) {
        float_raise(float_flag_invalid, &env->fx.status);
        return 0;
    }
    tmp = float32_scalbn(src, fracbits, &env->fx.status);
    return float32_to_int32_round_to_zero(tmp, &env->fx.status);
}

int32_t HELPER(fx_dtox)(CPUArchState *env, float64 src, int fracbits)
{
    float64 tmp;
    if (float64_is_signaling_nan(src, &env->fx.status) || float64_is_infinity(src)) {
        float_raise(float_flag_invalid, &env->fx.status);
        return 0;
    }
    tmp = float64_scalbn(src, fracbits, &env->fx.status);
    return float64_to_int32_round_to_zero(tmp, &env->fx.status);
}

float32 HELPER(fx_xtof)(CPUArchState *env, int32_t src, int fracbits)
{
    float32 tmp = int32_to_float32(src, &env->fx.status);
    return float32_scalbn(tmp, -fracbits, &env->fx.status);
}

float64 HELPER(fx_xtod)(CPUArchState *env, int32_t src, int fracbits)
{
    float64 tmp = int32_to_float64(src, &env->fx.status);
    return float64_scalbn(tmp, -fracbits, &env->fx.status);
}

int64_t HELPER(fx_dtoxl)(CPUArchState *env, float64 src, int fracbits)
{
    float64 tmp;
    if (float64_is_signaling_nan(src, &env->fx.status) || float64_is_infinity(src)) {
        float_raise(float_flag_invalid, &env->fx.status);
        return 0;
    }
    tmp = float64_scalbn(src, fracbits, &env->fx.status);
    return float64_to_int64_round_to_zero(tmp, &env->fx.status);
}

float64 HELPER(fx_xltod)(CPUArchState *env, int64_t src, int fracbits)
{
    float64 tmp = int64_to_float64(src, &env->fx.status);
    return float64_scalbn(tmp, -fracbits, &env->fx.status);
}

float32 HELPER(fx_cmp32)(CPUArchState *env, float32 src1, float32 src2, int flags)
{
    float32 cmp1, cmp2;
    int ret;

    cmp1 = src1;
    cmp2 = src2;

    if (flags & META_FXINST_A) {
        cmp1 = float32_abs(src1);
        cmp2 = float32_abs(src2);
    }

    if (flags & META_FXINST_Q)
        ret = float32_compare_quiet(cmp1, cmp2, &env->fx.status);
    else
        ret = float32_compare(cmp1, cmp2, &env->fx.status);

    if (flags & META_FXINST_P) {
        if (flags & META_FXINST_HIGH) {
            env->cf[META_SFF_HZ] = (ret == float_relation_equal);
            env->cf[META_SFF_HN] = (ret < 0);
        } else {
            env->cf[META_SFF_LZ] = (ret == float_relation_equal);
            env->cf[META_SFF_LN] = (ret < 0);
        }
        env->cf[META_CF_SCC] = true;
    } else {
        env->cf[META_FF_Z] = (ret == float_relation_equal);
        env->cf[META_FF_N] = (ret < 0);
        env->cf[META_FF_V] = (ret == float_relation_unordered);
        env->cf[META_FF_C] = env->cf[META_FF_N];
        env->cf[META_CF_SCC] = false;
    }

    if (flags & META_FXINST_MAX)
        return (ret < 0) ? src2 : src1;
    return (ret < 0) ? src1 : src2;
}

float64 HELPER(fx_cmp64)(CPUArchState *env, float64 src1, float64 src2, int flags)
{
    float64 cmp1, cmp2;
    int ret;

    cmp1 = src1;
    cmp2 = src2;

    if (flags & META_FXINST_A) {
        cmp1 = float64_abs(src1);
        cmp2 = float64_abs(src2);
    }

    if (flags & META_FXINST_Q)
        ret = float64_compare_quiet(cmp1, cmp2, &env->fx.status);
    else
        ret = float64_compare(cmp1, cmp2, &env->fx.status);

    env->cf[META_FF_Z] = (ret == float_relation_equal);
    env->cf[META_FF_N] = (ret < 0);
    env->cf[META_FF_V] = (ret == float_relation_unordered);
    env->cf[META_FF_C] = env->cf[META_FF_N];
    env->cf[META_CF_SCC] = false;

    if (flags & META_FXINST_MAX)
        return (ret < 0) ? src2 : src1;
    return (ret < 0) ? src1 : src2;
}

static inline float32 meta_maybe_silence_nan32(float32 a, float_status *st)
{
    return float32_is_signaling_nan(a, st) ? float32_silence_nan(a, st) : a;
}

static inline float64 meta_maybe_silence_nan64(float64 a, float_status *st)
{
    return float64_is_signaling_nan(a, st) ? float64_silence_nan(a, st) : a;
}

#define HELPER_MUZ(n) \
float ## n HELPER(fx_muz ## n)(CPUArchState *env, float ## n src1, \
        float ## n src2, float ## n src3, int flags) \
{ \
    float ## n val; \
    val = float ## n ## _mul(src1, src2, &env->fx.status); \
    if (flags & META_FXINST_Q) \
        val = meta_maybe_silence_nan ## n(val, &env->fx.status); \
    if (flags & META_FXINST_PM) \
        val = float ## n ## _sub(val, src3, &env->fx.status); \
    else \
        val = float ## n ## _add(val, src3, &env->fx.status); \
    if (flags & META_FXINST_Q) \
        val = meta_maybe_silence_nan ## n(val, &env->fx.status); \
    return val; \
}

HELPER_MUZ(32)
HELPER_MUZ(64)

#undef HELPER_MUZ

#define HELPER_RCP(n) \
float ## n HELPER(fx_rcp ## n)(CPUArchState *env, float ## n src, int flags) \
{ \
    float ## n ret; \
    ret = float ## n ## _div(float ## n ## _one, src, &env->fx.status); \
    if ((flags & META_FXINST_Z) && float ## n ## _is_zero_or_denormal(ret)) \
        ret = float ## n ## _zero; \
    return ret; \
}

HELPER_RCP(32)
HELPER_RCP(64)

#undef HELPER_RCP

#define HELPER_RSQ(n) \
float ## n HELPER(fx_rsq ## n)(CPUArchState *env, float ## n src, int flags) \
{ \
    float ## n ret; \
    ret = float ## n ## _sqrt(src, &env->fx.status); \
    ret = float ## n ## _div(float ## n ## _one, ret, &env->fx.status); \
    if ((flags & META_FXINST_Z) && float ## n ## _is_zero_or_denormal(ret)) \
        ret = float ## n ## _zero; \
    return ret; \
}

HELPER_RSQ(32)
HELPER_RSQ(64)

#undef HELPER_RSQ

void HELPER(fx_handle_exception)(CPUArchState *env, uint32_t op, uint32_t args)
{
    MetaFxInstInfo info;
    int exflags, exmask, excp;

    info.op.raw = op;
    info.args.raw = args;

    exflags = get_float_exception_flags(&env->fx.status);
    exmask = 0;
    excp = -1;

    if (exflags & float_flag_inexact) {
        excp = META_DEFR_FPU_INEXACT;
        exmask |= 1 << excp;
    }
    if (exflags & float_flag_underflow) {
        excp = META_DEFR_FPU_UNDERFLOW;
        exmask |= 1 << excp;
    }
    if (exflags & float_flag_overflow) {
        excp = META_DEFR_FPU_OVERFLOW;
        exmask |= 1 << excp;
    }
    if (exflags & float_flag_divbyzero) {
        excp = META_DEFR_FPU_DIVZERO;
        exmask |= 1 << excp;
    }
    if (exflags & float_flag_invalid) {
        excp = META_DEFR_FPU_INVALID;
        exmask |= 1 << excp;
    }

    if (exmask) {
        env->cregs[META_TXDEFR] |= exmask << META_TXDEFR_TRIGSTAT_SHIFT;

        env->cregs[META_TXSTATUS] |= META_TXSTATUS_FPACTIVE_MASK;

        env->cregs[META_TXCATCH0] &= ~(META_TXCATCH0_FPURDREG_MASK << META_TXCATCH0_FPURDREG_SHIFT);
        env->cregs[META_TXCATCH0] |= info.args.rd << META_TXCATCH0_FPURDREG_SHIFT;

        env->cregs[META_TXCATCH0] &= ~(META_TXCATCH0_FPUSELROP1_MASK << META_TXCATCH0_FPUSELROP1_SHIFT);
        env->cregs[META_TXCATCH0] |= info.args.rs1 << META_TXCATCH0_FPUSELROP1_SHIFT;

        env->cregs[META_TXCATCH0] |= (META_TXCATCH0_FPUSPECIAL_MASK << META_TXCATCH0_FPUSPECIAL_SHIFT);

        env->cregs[META_TXCATCH0] &= ~(META_TXCATCH0_FPUFLAGS_MASK << META_TXCATCH0_FPUFLAGS_SHIFT);
        env->cregs[META_TXCATCH0] |= (info.op.flags & META_TXCATCH0_FPUFLAGS_MASK) << META_TXCATCH0_FPUFLAGS_SHIFT;

        env->cregs[META_TXCATCH0] &= ~(META_TXCATCH0_FPUWIDTH_MASK << META_TXCATCH0_FPUWIDTH_SHIFT);
        env->cregs[META_TXCATCH0] |= ((info.op.flags >> 5) & 0x3) << META_TXCATCH0_FPUWIDTH_SHIFT;

        env->cregs[META_TXCATCH0] &= ~(META_TXCATCH0_FPUOPENC_MASK << META_TXCATCH0_FPUOPENC_SHIFT);
        env->cregs[META_TXCATCH0] |= info.op.enc << META_TXCATCH0_FPUOPENC_SHIFT;

        env->cregs[META_TXCATCH1] &= ~(META_TXCATCH1_FPUSELROP2_MASK << META_TXCATCH1_FPUSELROP2_SHIFT);
        env->cregs[META_TXCATCH1] |= info.args.rs2 << META_TXCATCH1_FPUSELROP2_SHIFT;

        env->cregs[META_TXCATCH1] &= ~(META_TXCATCH1_FPUSELROP3_MASK << META_TXCATCH1_FPUSELROP3_SHIFT);
        env->cregs[META_TXCATCH1] |= info.args.rs3 << META_TXCATCH1_FPUSELROP3_SHIFT;

        env->cregs[META_TXCATCH1] &= ~(META_TXCATCH1_FPUIMM_MASK << META_TXCATCH1_FPUIMM_SHIFT);
        env->cregs[META_TXCATCH1] |= info.args.imm << META_TXCATCH1_FPUIMM_SHIFT;

#ifdef CONFIG_USER_ONLY
        if (env->cregs[META_TXDEFR] & exmask) {
            HELPER(raise_exception)(env, EXCP_FPU);
        }
#else
        HELPER(defr_trigger)(env, excp);
#endif
    }
}

uint64_t HELPER(dspram_radix)(uint32_t pointer, uint32_t inc_reg, uint32_t radix_scale, uint32_t addr_mask)
{
    uint32_t term_a, term_b, increment, n_inc_reg;
    uint32_t radix_end, radix_step, radix_org, radix_state;
    uint32_t n_radix_step, n_radix_state;

    /* extract increment register values */
    radix_end = (inc_reg >> 28) & 0x3;
    radix_step = (inc_reg >> 24) & 0x3;
    radix_org = (inc_reg >> 22) & 0x3;
    radix_state = (inc_reg >> 16) & 0x1f;
    increment = inc_reg & 0xffff;

    if (!radix_step && !radix_state && !radix_org && !radix_end) {
        /* not active */
        pointer = (pointer + increment) & addr_mask;
        n_inc_reg = inc_reg;
        goto done;
    }

    /* calculate RAM offset, straight from DSP TRM */
    term_a = increment >> (4 - radix_step);
    if (radix_state)
        term_b = radix_state << (radix_scale + 4);
    else
        term_b = term_a;
    pointer = (term_a + term_b) & ((8 << (radix_scale + 4)) - 1);

    /* calculate new radix setup */
    n_radix_step = radix_step;

    if (!radix_state) {
        /* 0 special case  */
        n_radix_state = 8 * (4 - radix_step);
    } else if (radix_state == 12 || radix_state == 22 || radix_state == 31) {
        /* special cases */
        if (radix_step == radix_end) {
            n_radix_state = 0;
            n_radix_step = radix_org;
        } else {
            switch (radix_state) {
            case 12: n_radix_state = 16; break;
            case 22: n_radix_state = 24; break;
            default: n_radix_state = 0; break;
            }
            n_radix_step--;
        }
    } else {
        /* default */
        n_radix_state = radix_state + ((1 << radix_step) >> 1);
    }

    /* pack it back into increment register format */
    n_inc_reg =
        (radix_end << 28) |
        (n_radix_step << 24) |
        (radix_org << 22) |
        (n_radix_state << 16) |
        increment;

done:
    /* return high=increg, low=pointer */
    return ((uint64_t)n_inc_reg << 32) | pointer;
}

static uint32_t dspram_radix(uint32_t addr, uint32_t *inc_reg,
                             uint32_t radix_scale)
{
    uint32_t term_a, term_b, increment, n_inc_reg;
    uint32_t radix_end, radix_step, radix_org, radix_state;
    uint32_t n_radix_step, n_radix_state;

    /* extract increment register values */
    radix_end = (*inc_reg >> 28) & 0x3;
    radix_step = (*inc_reg >> 24) & 0x3;
    radix_org = (*inc_reg >> 22) & 0x3;
    radix_state = (*inc_reg >> 16) & 0x1f;
    increment = *inc_reg & 0xffff;

    /* calculate RAM offset, straight from DSP TRM */
    term_a = increment >> (4 - radix_step);
    if (radix_state) {
        term_b = radix_state << (radix_scale + 4);
    } else {
        term_b = term_a;
    }
    addr = (term_a + term_b) & ((8 << (radix_scale + 4)) - 1);

    /* calculate new radix setup */
    n_radix_step = radix_step;

    if (!radix_state) {
        /* 0 special case  */
        n_radix_state = 8 * (4 - radix_step);
    } else if (radix_state == 12 || radix_state == 22 || radix_state == 31) {
        /* special cases */
        if (radix_step == radix_end) {
            n_radix_state = 0;
            n_radix_step = radix_org;
        } else {
            switch (radix_state) {
            case 12:
                n_radix_state = 16;
                break;

            case 22:
                n_radix_state = 24;
                break;

            default:
                n_radix_state = 0;
            }
            n_radix_step--;
        }
    } else {
        /* default */
        n_radix_state = radix_state + ((1 << radix_step) >> 1);
    }

    /* pack it back into increment register format */
    n_inc_reg =
        (radix_end << 28) |
        (n_radix_step << 24) |
        (radix_org << 22) |
        (n_radix_state << 16) |
        increment;

    /* update increment */
    *inc_reg = n_inc_reg;
    return addr;
}

static uint32_t dspram_modulo(uint32_t old, uint32_t new,
                              uint32_t mod_size, int addr_bits)
{
    int32_t old_se = ((int32_t)old << (32 - addr_bits)) >> (32 - addr_bits);
    int32_t new_se = ((int32_t)new << (32 - addr_bits)) >> (32 - addr_bits);
    uint32_t ret = new;

    if ((new >= mod_size) ||                /* address outside region */
        ((old_se >= 0) && (new_se < 0))) {  /* or overflow +ve to -ve */
        /* perform adjustment */
        if (new_se < 0) {
            ret += mod_size;
        } else {
            ret -= mod_size;
        }
    }

    /* mask to available bits */
    ret &= (1 << addr_bits) - 1;

    return ret;
}

void HELPER(dspram_ptr_w_set)(CPUArchState *env,
                              int du, int bank, int ptr,
                              uint32_t val, void *inc_ptr)
{
    uint32_t addr = val;
    uint32_t *inc = inc_ptr;
    int mod_size, mod_shift, addr_bits;

    if (bank) {
        mod_shift = META_TXDRSIZE_RAMBMODSIZE_SHIFT;
    } else {
        mod_shift = META_TXDRSIZE_RAMAMODSIZE_SHIFT;
    }
    mod_size = (env->cregs[META_TXDRSIZE] >> mod_shift) & 0xfff;

    /* decide how many address bits are available */
    addr_bits = env->global->dspram_addr_bits;

    /* perform increment */
    if (inc) {
        addr += ((int32_t)*inc << 16) >> 16;

        /* mask to available bits */
        addr &= (1 << addr_bits) - 1;
    }

    /* apply modulo addressing if applicable */
    if (mod_size) {
        addr = dspram_modulo(val, addr, mod_size, addr_bits);
    }

    /* set the pointer */
    env->dspram_wp[du][bank][ptr] = addr;
}

uint32_t HELPER(dspram_ptr_r_set)(CPUArchState *env,
                                  int du, int bank, int ptr,
                                  uint32_t val, void *inc_ptr)
{
    uint32_t mode = env->cregs[META_TXMODE];
    uint32_t addr = val;
    uint32_t data, off;
    uint32_t *inc = inc_ptr;
    uint32_t off_and = env->dspram_off_and[du];
    uint32_t off_or = env->dspram_off_or[du];
    bool radix = mode & META_TXMODE_DSPRRADIX_MASK;
    int mod_size, mod_shift, addr_bits;

    if (bank) {
        mod_shift = META_TXDRSIZE_RAMBMODSIZE_SHIFT;
    } else {
        mod_shift = META_TXDRSIZE_RAMAMODSIZE_SHIFT;
    }
    mod_size = (env->cregs[META_TXDRSIZE] >> mod_shift) & 0xfff;

    /* how many address bits are available */
    addr_bits = env->global->dspram_addr_bits;

    /* radix increments bypass modulo addressing */
    if (radix) {
        mod_size = 0;
    }

    /* perform increment */
    if (inc) {
        if (radix && (*inc >> 16)) {
            int scale_shift;
            uint32_t scale;

            if (bank) {
                scale_shift = META_TXDRSIZE_RAMBRADIXSCALE_SHIFT;
            } else {
                scale_shift = META_TXDRSIZE_RAMARADIXSCALE_SHIFT;
            }
            scale = (env->cregs[META_TXDRSIZE] >> scale_shift) & 0x7;

            addr = dspram_radix(addr, inc, scale);
        } else {
            addr += ((int32_t)*inc << 16) >> 16;

            /* mask to available bits */
            addr &= (1 << addr_bits) - 1;
        }
    }

    /* apply modulo addressing if applicable */
    if (mod_size) {
        addr = dspram_modulo(val, addr, mod_size, addr_bits);
    }

    /* set the pointer */
    env->dspram_rp[du][bank][ptr] = addr;

    /* pre-fetch the data */
    off = (addr & off_and) | off_or;
    data = env->global->dspram[du][bank][off];

    env->dspram_fetched[du][bank][ptr] = data;
    return data;
}

uint32_t HELPER(dspram_read)(CPUArchState *env, int du, uint32_t spec)
{
    uint32_t data;
    int bank = (spec >> 3) & 0x1;
    int ptr_idx = (spec >> 2) & 0x1;

    data = env->dspram_fetched[du][bank][ptr_idx];

    if (spec & 0x3) {
        /* post-increment */
        uint32_t inc, *inc_ptr;

        if (spec & 0x2) {
            /* by a register */
            inc_ptr = &env->dspram_rpi[du][bank][spec & 0x1];
        } else {
            /* by 1 */
            inc = 1;
            inc_ptr = &inc;
        }

        /* perform the post-increment */
        HELPER(dspram_ptr_r_set)(env, du, bank, ptr_idx,
                                 env->dspram_rp[du][bank][ptr_idx],
                                 inc_ptr);
    }

    return data;
}

void HELPER(dspram_write)(CPUArchState *env, int du, uint32_t spec,
                          uint32_t val)
{
    int bank = (spec >> 3) & 0x1;
    int ptr_idx = (spec >> 2) & 0x1;
    uint32_t addr = env->dspram_wp[du][bank][ptr_idx];
    uint32_t off_and = env->dspram_off_and[du];
    uint32_t off_or = env->dspram_off_or[du];
    uint32_t off = (addr & off_and) | off_or;

    /* write the data */
    env->global->dspram[du][bank][off] = val;

    if (spec & 0x3) {
        /* post-increment */
        uint32_t inc;

        if (spec & 0x2) {
            /* by a register */
            inc = env->dspram_wpi[du][bank][spec & 0x1];
        } else {
            /* by 1 */
            inc = 1;
        }

        /* perform the post-increment */
        HELPER(dspram_ptr_w_set)(env, du, bank, ptr_idx,
                                 env->dspram_wp[du][bank][ptr_idx],
                                 &inc);
    }
}

uint32_t HELPER(au_add)(CPUArchState *env, int au,
                        uint32_t src1, uint32_t src2)
{
    MetaAUAddrMode mode = meta_auaddrmode(env, au);
    uint32_t add1 = src1;
    uint32_t add2 = src2;
    uint32_t result;
    uint32_t mod_size = env->cregs[META_TXMRSIZE] & 0xffff;
    bool bitrev, modulo;

    if (env->cregs[META_TXSTATUS] & META_TXSTATUS_ISTAT_MASK) {
        /* always linear with IStat set */
        mode = META_AUADDR_LINEAR;
    }

    bitrev = mode & 0x4;
    modulo = (mode == META_AUADDR_MODULO) && (mod_size >= 2);

    if (bitrev) {
        add1 = bitreverse_i32(add1);
        add2 = bitreverse_i32(add2);
    }

    result = add1 + add2;

    if (bitrev) {
        result = bitreverse_i32(result);

        /* clear lower bits as appropriate */
        result &= ~((1 << (mode - META_AUADDR_BITREV8)) - 1);
    }

    if (modulo &&
        (((result & env->aumod_mask) != (src1 & env->aumod_mask)) ||
         (result & ~env->aumod_mask) >= mod_size)) {
        /* perform adjustment */
        int32_t adj = ((int32_t)src2 < 0) ? mod_size : -mod_size;
#if DEBUG_MODULO
        qemu_log("Modulo adjustment: 0x%08x + 0x%x = 0x%08x",
                 src1, src2, result);
#endif
        result += adj;
#if DEBUG_MODULO
        qemu_log(" -> 0x%08x (%c0x%x)\n", result,
                 (adj < 0) ? '-' : '+', mod_size);
#endif
    }

#if DEBUG_ADDRESS
    qemu_log("A%d: 0x%08x + 0x%x = 0x%08x\n",
             au, src1, src2, result);
#endif

    return result;
}

/* A redefined DSP template: drop the TBs that inlined the old one. */
void HELPER(daoppame_invalidate_template)(CPUArchState *env, uint32_t tidx)
{
    queue_tb_flush(env_cpu(env));
}

static inline bool read_append_full(CPUArchState *env)
{
    if (env->readport_count < META_READPORT_DEPTH) {
        return false;
    }

#if !defined(CONFIG_USER_ONLY)
    HELPER(halt_signum)(env, META_SIGNUM_IIF);
#else
    HELPER(raise_exception)(env, EXCP_FAULT_BASE + META_SIGNUM_IIF);
#endif
    return true;
}

static inline void read_append_data(CPUArchState *env, uint64_t data)
{
    env->readport_data[env->readport_idx_w++] = data;
    env->readport_idx_w %= META_READPORT_DEPTH;
    env->readport_count++;
}

void HELPER(read_append_ra)(CPUArchState *env, uint64_t data)
{
    if (!read_append_full(env)) {
        read_append_data(env, data);
    }
}

void HELPER(read_append_rabz)(CPUArchState *env, uint64_t data)
{
    if (!read_append_full(env)) {
        read_append_data(env, data & 0xff);
    }
}

void HELPER(read_append_rawz)(CPUArchState *env, uint64_t data)
{
    if (!read_append_full(env)) {
        read_append_data(env, data & 0xffff);
    }
}

void HELPER(read_append_radz)(CPUArchState *env, uint64_t data)
{
    if (!read_append_full(env)) {
        read_append_data(env, data);
    }
}

void HELPER(read_append_rabx)(CPUArchState *env, uint64_t data)
{
    if (!read_append_full(env)) {
        read_append_data(env, (int64_t)((int8_t)data));
    }
}

void HELPER(read_append_rawx)(CPUArchState *env, uint64_t data)
{
    if (!read_append_full(env)) {
        read_append_data(env, (int64_t)((int16_t)data));
    }
}

void HELPER(read_append_radx)(CPUArchState *env, uint64_t data)
{
    if (!read_append_full(env)) {
        read_append_data(env, (int64_t)((int32_t)data));
    }
}

void HELPER(read_append_ram8x)(CPUArchState *env, uint64_t data)
{
    if (!read_append_full(env)) {
        data =
            (((data >> 24) & 0xff) << 48) |
            (((data >> 16) & 0xff) << 32) |
            (((data >> 8) & 0xff) << 16) |
            (((data >> 0) & 0xff) << 0);
        read_append_data(env, data);
    }
}

void HELPER(read_append_ram8x32)(CPUArchState *env, uint64_t data)
{
    if (!read_append_full(env)) {
        data =
            (((data >> 8) & 0xff) << 32) |
            (((data >> 0) & 0xff) << 0);
        read_append_data(env, data);
    }
}

void HELPER(read_append_ram16x)(CPUArchState *env, uint64_t data)
{
    if (!read_append_full(env)) {
        data =
            (((int64_t)(int16_t)(data >> 16)) << 32) |
            (uint32_t)(int32_t)(int16_t)data;
        read_append_data(env, data);
    }
}

void HELPER(sim_start)(CPUArchState *env)
{
    qemu_log("Meta sim start event\n");
    env->sim_active = true;
}

void HELPER(sim_stop)(CPUArchState *env)
{
    qemu_log("Meta sim stop event\n");
    env->sim_active = false;
}

void HELPER(sim_dump)(CPUArchState *env, uint32_t delay)
{
    qemu_log("Meta sim dump event, delay=%d\n", delay);
}

uint64_t HELPER(shift_rspp_sats9)(uint64_t src1, uint32_t src2, uint64_t val)
{
    int first_bit, sign_bit;

    /* find highest non-sign bit */
    sign_bit = (src1 >> 31) & 0x1;
    for (first_bit = 30; first_bit >= 0; first_bit--) {
        if (((src1 >> first_bit) & 0x1) != sign_bit) {
            break;
        }
    }

    if ((first_bit - (int)src2) >= 8) {
        if (val & 0x8000000000llu) {
            /* negative value */
            if ((val & 0xffffff00) != 0xffffff00) {
                /* saturate to -256 */
                return 0xffffff00;
            }
        } else {
            /* positive value */
            if (val > 0xff) {
                /* saturate to 255 */
                return 0xff;
            }
        }
    }

    /* not saturated */
    return val;
}

uint64_t HELPER(shift_rspp_satu8)(uint64_t src1, uint32_t src2, uint64_t val)
{
    int first_bit, sign_bit;

    if (val & 0x8000000000llu) {
        /* negative value, saturate to 0 */
        return 0;
    }

    /* find highest non-sign bit */
    sign_bit = (src1 >> 31) & 0x1;
    for (first_bit = 30; first_bit >= 0; first_bit--) {
        if (((src1 >> first_bit) & 0x1) != sign_bit) {
            break;
        }
    }

    if ((first_bit - (int)src2) >= 8) {
        /* >8b positive value, saturate to 255 */
        return 0xff;
    }

    /* not saturated */
    return val;
}

uint64_t HELPER(saturate_asl_i40)(uint64_t val, uint64_t src1, uint32_t src2)
{
    uint32_t left_overs;

    /* bits that don't fit in the guard bits */
    left_overs = (uint32_t)((int64_t)val >> 40);

    /* bits lost from 64-bit temporary */
    if (src2 > 24) {
        left_overs &= 0x00FFFFFFL;
        left_overs |= (((src1 >> 32) << (src2 - 8L)) & 0xFF000000L);
    }

    /* saturate if have bits outside the guard */
    if (((left_overs != 0xFFFFFFFFL) || !(val & (1llu << 39))) &&
        ((left_overs != 0x00000000L) || (val & (1llu << 39)))) {
        /* saturate to a sign extended 64-bit result */
        val = (src1 & (1llu << 39)) ? 0xffffff8000000000llu : 0x7fffffffffllu;
    }

    return val;
}

uint32_t HELPER(mul_ps_sat16)(uint32_t val)
{
    if (((val << 1) ^ val) >> 31) {
        return 0x7fffffff ^ ((int32_t)val >> 31);
    }

    return val << 1;
}

uint32_t HELPER(add16_overflow)(uint32_t val, uint32_t src1, uint32_t src2)
{
    return (((val ^ src1) & ~(src1 ^ src2)) >> 15) & 0x1;
}

uint32_t HELPER(sub16_overflow)(uint32_t val, uint32_t src1, uint32_t src2)
{
    return (((val ^ src1) & (src1 ^ src2)) >> 15) & 0x1;
}

uint32_t HELPER(add16_sat)(uint32_t val, uint32_t src1, uint32_t src2)
{
    if (HELPER(add16_overflow)(val, src1, src2)) {
        return ((src1 >> 15) & 0x1) ? 0x8000 : 0x7fff;
    }
    return val & 0xffff;
}

uint32_t HELPER(sub16_sat)(uint32_t val, uint32_t src1, uint32_t src2)
{
    if (HELPER(sub16_overflow)(val, src1, src2)) {
        return ((src1 >> 15) & 0x1) ? 0x8000 : 0x7fff;
    }
    return val & 0xffff;
}

uint32_t HELPER(addsub16_pshiftse)(uint32_t val)
{
    return val | ((val & 0x8000) ? 0x10000 : 0);
}
