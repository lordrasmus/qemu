/*
 *  PowerPC CPU routines for qemu.
 *
 * Copyright (c) 2017 Nikunj A Dadhania, IBM Corporation.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
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
#include "cpu-models.h"
#include "cpu-qom.h"
#include "exec/log.h"
#include "exec/watchpoint.h"
#include "fpu/softfloat-helpers.h"
#include "mmu-hash64.h"
#include "helper_regs.h"
#include "system/tcg.h"

target_ulong cpu_read_xer(const CPUPPCState *env)
{
    if (is_isa300(env)) {
        return env->xer | (env->so << XER_SO) |
            (env->ov << XER_OV) | (env->ca << XER_CA) |
            (env->ov32 << XER_OV32) | (env->ca32 << XER_CA32);
    }

    return env->xer | (env->so << XER_SO) | (env->ov << XER_OV) |
        (env->ca << XER_CA);
}

void cpu_write_xer(CPUPPCState *env, target_ulong xer)
{
    env->so = (xer >> XER_SO) & 1;
    env->ov = (xer >> XER_OV) & 1;
    env->ca = (xer >> XER_CA) & 1;
    /* write all the flags, while reading back check of isa300 */
    env->ov32 = (xer >> XER_OV32) & 1;
    env->ca32 = (xer >> XER_CA32) & 1;
    env->xer = xer & ~((1ul << XER_SO) |
                       (1ul << XER_OV) | (1ul << XER_CA) |
                       (1ul << XER_OV32) | (1ul << XER_CA32));
}

void ppc_store_vscr(CPUPPCState *env, uint32_t vscr)
{
    env->vscr = vscr & ~(1u << VSCR_SAT);
    /* Which bit we set is completely arbitrary, but clear the rest.  */
    env->vscr_sat.u64[0] = vscr & (1u << VSCR_SAT);
    env->vscr_sat.u64[1] = 0;
    set_flush_to_zero((vscr >> VSCR_NJ) & 1, &env->vec_status);
    set_flush_inputs_to_zero((vscr >> VSCR_NJ) & 1, &env->vec_status);
}

/*
 * SPEFSCR (e500).  The sticky exception bits are kept as softfloat
 * flags in spe_status while instructions run and are merged into
 * spe_fscr whenever the register is read.
 */
#define SPEFSCR_FINXS   0x00200000
#define SPEFSCR_FINVS   0x00100000
#define SPEFSCR_FDBZS   0x00080000
#define SPEFSCR_FUNFS   0x00040000
#define SPEFSCR_FOVFS   0x00020000
#define SPEFSCR_FRMC    0x00000003

uint32_t ppc_get_spefscr(CPUPPCState *env)
{
    int flags = get_float_exception_flags(&env->spe_status);

    if (flags & float_flag_inexact) {
        env->spe_fscr |= SPEFSCR_FINXS;
    }
    if (flags & float_flag_invalid) {
        env->spe_fscr |= SPEFSCR_FINVS;
    }
    if (flags & float_flag_divbyzero) {
        env->spe_fscr |= SPEFSCR_FDBZS;
    }
    if (flags & float_flag_underflow) {
        env->spe_fscr |= SPEFSCR_FUNFS;
    }
    if (flags & float_flag_overflow) {
        env->spe_fscr |= SPEFSCR_FOVFS;
    }
    set_float_exception_flags(0, &env->spe_status);
    return env->spe_fscr;
}

void ppc_store_spefscr(CPUPPCState *env, uint32_t val)
{
    /*
     * The e500 truncates toward +/-inf and the kernel corrects the last
     * bit from the round interrupt (see spe_end()); in user mode there
     * is no kernel, so round directly.
     */
    static const FloatRoundMode rm[4] = {
        float_round_nearest_even, float_round_to_zero,
#if defined(CONFIG_USER_ONLY)
        float_round_up, float_round_down,
#else
        float_round_to_zero, float_round_to_zero,
#endif
    };

    env->spe_fscr = val;
    set_float_exception_flags(0, &env->spe_status);
    set_float_rounding_mode(rm[val & SPEFSCR_FRMC], &env->spe_status);
}

uint32_t ppc_get_vscr(CPUPPCState *env)
{
    uint32_t sat = (env->vscr_sat.u64[0] | env->vscr_sat.u64[1]) != 0;
    return env->vscr | (sat << VSCR_SAT);
}

void ppc_set_cr(CPUPPCState *env, uint64_t cr)
{
    for (int i = 7; i >= 0; i--) {
        env->crf[i] = cr & 0xf;
        cr >>= 4;
    }
}

uint64_t ppc_get_cr(const CPUPPCState *env)
{
    uint64_t cr = 0;
    for (int i = 0; i < 8; i++) {
        cr |= (env->crf[i] & 0xf) << (4 * (7 - i));
    }
    return cr;
}

/* GDBstub can read and write MSR... */
void ppc_store_msr(CPUPPCState *env, target_ulong value)
{
    hreg_store_msr(env, value, 0);
}

#if !defined(CONFIG_USER_ONLY)
void ppc_store_lpcr(PowerPCCPU *cpu, target_ulong val)
{
    PowerPCCPUClass *pcc = POWERPC_CPU_GET_CLASS(cpu);
    CPUPPCState *env = &cpu->env;

    env->spr[SPR_LPCR] = val & pcc->lpcr_mask;
    /* The gtse bit affects hflags */
    hreg_compute_hflags(env);

    ppc_maybe_interrupt(env);
}

#if defined(TARGET_PPC64)
void ppc_update_ciabr(CPUPPCState *env)
{
    CPUState *cs = env_cpu(env);
    target_ulong ciabr = env->spr[SPR_CIABR];
    target_ulong ciea, priv;

    ciea = ciabr & PPC_BITMASK(0, 61);
    priv = ciabr & PPC_BITMASK(62, 63);

    if (env->ciabr_breakpoint) {
        cpu_breakpoint_remove_by_ref(cs, env->ciabr_breakpoint);
        env->ciabr_breakpoint = NULL;
    }

    if (priv) {
        cpu_breakpoint_insert(cs, ciea, BP_CPU, &env->ciabr_breakpoint);
    }
}

void ppc_store_ciabr(CPUPPCState *env, target_ulong val)
{
    env->spr[SPR_CIABR] = val;
    ppc_update_ciabr(env);
}

void ppc_update_daw(CPUPPCState *env, int rid)
{
    CPUState *cs = env_cpu(env);
    int spr_dawr = rid ? SPR_DAWR1 : SPR_DAWR0;
    int spr_dawrx = rid ? SPR_DAWRX1 : SPR_DAWRX0;
    target_ulong deaw = env->spr[spr_dawr] & PPC_BITMASK(0, 60);
    uint32_t dawrx = env->spr[spr_dawrx];
    int mrd = extract32(dawrx, PPC_BIT_NR(48), 54 - 48);
    bool dw = extract32(dawrx, PPC_BIT_NR(57), 1);
    bool dr = extract32(dawrx, PPC_BIT_NR(58), 1);
    bool hv = extract32(dawrx, PPC_BIT_NR(61), 1);
    bool sv = extract32(dawrx, PPC_BIT_NR(62), 1);
    bool pr = extract32(dawrx, PPC_BIT_NR(62), 1);
    vaddr len;
    int flags;

    assert(tcg_enabled());

    if (env->dawr_watchpoint[rid]) {
        cpu_watchpoint_remove_by_ref(cs, env->dawr_watchpoint[rid]);
        env->dawr_watchpoint[rid] = NULL;
    }

    if (!dr && !dw) {
        return;
    }

    if (!hv && !sv && !pr) {
        return;
    }

    len = (mrd + 1) * 8;
    flags = BP_CPU | BP_STOP_BEFORE_ACCESS;
    if (dr) {
        flags |= BP_MEM_READ;
    }
    if (dw) {
        flags |= BP_MEM_WRITE;
    }

    cpu_watchpoint_insert(cs, deaw, len, flags, &env->dawr_watchpoint[rid]);
}

void ppc_store_dawr0(CPUPPCState *env, target_ulong val)
{
    env->spr[SPR_DAWR0] = val;
    ppc_update_daw(env, 0);
}

static void ppc_store_dawrx(CPUPPCState *env, uint32_t val, int rid)
{
    int hrammc = extract32(val, PPC_BIT_NR(56), 1);

    if (hrammc) {
        /* This might be done with a second watchpoint at the xor of DEAW[0] */
        qemu_log_mask(LOG_UNIMP, "%s: DAWRX%d[HRAMMC] is unimplemented\n",
                      __func__, rid);
    }

    env->spr[rid ? SPR_DAWRX1 : SPR_DAWRX0] = val;
    ppc_update_daw(env, rid);
}

void ppc_store_dawrx0(CPUPPCState *env, uint32_t val)
{
    ppc_store_dawrx(env, val, 0);
}

void ppc_store_dawr1(CPUPPCState *env, target_ulong val)
{
    env->spr[SPR_DAWR1] = val;
    ppc_update_daw(env, 1);
}

void ppc_store_dawrx1(CPUPPCState *env, uint32_t val)
{
    ppc_store_dawrx(env, val, 1);
}

#endif
#endif

static inline void fpscr_set_rounding_mode(CPUPPCState *env)
{
    int rnd_type;

    /* Set rounding mode */
    switch (env->fpscr & FP_RN) {
    case 0:
        /* Best approximation (round to nearest) */
        rnd_type = float_round_nearest_even;
        break;
    case 1:
        /* Smaller magnitude (round toward zero) */
        rnd_type = float_round_to_zero;
        break;
    case 2:
        /* Round toward +infinite */
        rnd_type = float_round_up;
        break;
    default:
    case 3:
        /* Round toward -infinite */
        rnd_type = float_round_down;
        break;
    }
    set_float_rounding_mode(rnd_type, &env->fp_status);
}

void ppc_store_fpscr(CPUPPCState *env, target_ulong val)
{
    val &= FPSCR_MTFS_MASK;
    if (val & FPSCR_IX) {
        val |= FP_VX;
    }
    if ((val >> FPSCR_XX) & (val >> FPSCR_XE) & 0x1f) {
        val |= FP_FEX;
    }
    env->fpscr = val;
    set_float_rebias_overflow(FP_OE & env->fpscr, &env->fp_status);
    set_float_rebias_underflow(FP_UE & env->fpscr, &env->fp_status);
    if (tcg_enabled()) {
        fpscr_set_rounding_mode(env);
    }
}
