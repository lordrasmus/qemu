/*
 * CSKY translate header
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

#ifndef CSKY_TRANSLATE_H
#define CSKY_TRANSLATE_H
#include "qemu/log.h"
#include "exec/translator.h"
#include "accel/tcg/cpu-ldst.h"

/* Code fetches outside the translator (exception vectors, semihosting).  */
static inline uint16_t csky_lduw_code(CPUCSKYState *env, target_ulong addr)
{
    MemOpIdx oi = make_memop_idx(MO_TEUW, cpu_mmu_index(env_cpu(env), true));

    return cpu_ldw_code_mmu(env, addr, oi, 0);
}

static inline uint32_t csky_ldl_code(CPUCSKYState *env, target_ulong addr)
{
    MemOpIdx oi = make_memop_idx(MO_TEUL, cpu_mmu_index(env_cpu(env), true));

    return cpu_ldl_code_mmu(env, addr, oi, 0);
}

#if !defined(CONFIG_USER_ONLY)
typedef enum TraceMode {
    NORMAL_MODE = 0,
    INST_TRACE_MODE = 1,
    BRAN_TRACE_MODE = 3,
} TraceMode;
#endif

typedef struct DisasContext {
    DisasContextBase base;

    target_ulong pc;
    target_ulong next_page_start;
    uint32_t insn;

    /* Routine used to access memory */
    int mem_idx;   /*selects user or supervisor access*/
    /*
     * MO_ALIGN unless the CPU allows unaligned accesses.  qemu 3.0 built
     * this target with ALIGNED_ONLY, which checked every access and let
     * do_unaligned_access() return for UNALIGNED_ACCESS CPUs; today the
     * hook may not return, so the check is only requested when it raises.
     */
    MemOp align;
    int bctm;
    int condexec_cond;
    int idly4_counter;
    bool trace_match;
    TCGLabel *condlabel;

    uint64_t features;

#if !defined(CONFIG_USER_ONLY)
    int super;
    int trust;
    int current_cp;

    /* trace mode support */
    TraceMode trace_mode;
    int cannot_be_traced;
    int maybe_change_flow;
#endif
} DisasContext;

#if !defined(CONFIG_USER_ONLY)
typedef struct csky_tlb_t csky_tlb_t;
struct csky_tlb_t {
    uint32_t VPN;
    uint32_t PageMask; /* [24:13] */
    uint8_t ASID;
    uint16_t G:1;
    uint16_t C0:3;
    uint16_t C1:3;
    uint16_t V0:1;
    uint16_t V1:1;
    uint16_t D0:1;
    uint16_t D1:1;
    uint32_t PFN[2]; /* [31:12] */
};

#define CSKY_TLB_MAX            128
typedef struct CPUCSKYTLBContext {
    int (*get_physical_address)(CPUCSKYState *env, hwaddr *physical, int *prot,
                                target_ulong address, int rw);

    void (*helper_tlbwi) (CPUCSKYState *env);
    void (*helper_tlbwr) (CPUCSKYState *env);
    void (*helper_tlbp) (CPUCSKYState *env);
    void (*helper_tlbr) (CPUCSKYState *env);

    csky_tlb_t *tlb;
    uint8_t *round_robin;
    csky_tlb_t nt_tlb[CSKY_TLB_MAX];
    csky_tlb_t t_tlb[CSKY_TLB_MAX];
    uint8_t nt_round_robin[CSKY_TLB_MAX / 2];
    uint8_t t_round_robin[CSKY_TLB_MAX / 2];
} CPUCSKYTLBContext;

int mmu_get_physical_address(CPUCSKYState *env, hwaddr *physical,
                             int *prot, target_ulong address, int rw);
int thin_mmu_get_physical_address(CPUCSKYState *env, hwaddr *physical,
                                  int *prot, target_ulong address, int rw);
void csky_tlbwi(CPUCSKYState *env);
void csky_tlbwr(CPUCSKYState *env);
void csky_tlbp(CPUCSKYState *env);
void csky_tlbr(CPUCSKYState *env);
void helper_ttlbinv_all(CPUCSKYState *env);
void helper_tlbinv_idx(CPUCSKYState *env);
void helper_tlbinv_all(CPUCSKYState *env);
void helper_tlbinv_all_s(CPUCSKYState *env);
void helper_tlbinv_asid(CPUCSKYState *env, uint32_t rx);
void helper_tlbinv_asid_s(CPUCSKYState *env, uint32_t rx);
void helper_tlbinv_vaa(CPUCSKYState *env, uint32_t rx);
void helper_tlbinv_vaa_s(CPUCSKYState *env, uint32_t rx);
void helper_tlbinv_va(CPUCSKYState *env, uint32_t rx);
void helper_tlbinv_va_s(CPUCSKYState *env, uint32_t rx);
void helper_tlbinv(CPUCSKYState *env);
int nommu_get_physical_address(CPUCSKYState *env, hwaddr *physical,
                               int *prot, target_ulong address, int rw);
int mgu_get_physical_address(CPUCSKYState *env, hwaddr *physical,
                             int *prot, target_ulong address, int rw);
#endif

/* FPUv3 Macro begin */
/* FPUv3 ISA secondary/third OP MASK and SHIFT */
#define CSKY_FPUV3_SIGN_SHI         4
#define CSKY_FPUV3_SIGN_MASK        0x1
#define CSKY_FPUV3_WIDTH_BIT0       20
#define CSKY_FPUV3_WIDTH_BIT1       25

#define   OP_LDBI_B                 0x20
#define   OP_LDBI_H                 0x21
#define   OP_LDBI_W                 0x22
#define   OP_PLDBI_D                0x23
#define   OP_LDBI_BS                0x25
#define   OP_LDBI_HS                0x24
#define   OP_LDBIR_B                0x28
#define   OP_LDBIR_H                0x29
#define   OP_LDBIR_W                0x2a
#define   OP_PLDBIR_D               0x2b
#define   OP_LDBIR_BS               0x2c
#define   OP_LDBIR_HS               0x2d
#define   OP_STBI_B                 0x20
#define   OP_STBI_H                 0x21
#define   OP_STBI_W                 0x22
#define   OP_STBIR_B                0x28
#define   OP_STBIR_H                0x29
#define   OP_STBIR_W                0x2a

#define CSKY_FPUV3_REG_MASK         0x1f
#define CSKY_FPUV3_REG_SHI_RX       16
#define CSKY_FPUV3_REG_SHI_RY       21
#define CSKY_FPUV3_REG_SHI_RZ       0
#define CSKY_FPUV3_VREG_MASK        0x1f
#define CSKY_FPUV3_VREG_SHI_VRX     16
#define CSKY_FPUV3_VREG_SHI_VRY     21
#define CSKY_FPUV3_VREG_SHI_VRZ     0
#define CSKY_FPUV3_INDEX_SHI        21
#define CSKY_FPUV3_INDEX_MASK       0xf
#define CSKY_FPUV3_IMM_SHI          0xf
#define CSKY_FPUV3_IMM_MASK         0x1f
#define CSKY_FPUV3_N_SHI            0x6
#define CSKY_FPUV3_N_MASK           0x3
#define CSKY_FPUV3_IMM2_SHI         0x7
#define CSKY_FPUV3_IMM2_MASK        0x3
#define CSKY_FPUV3_IMM3_SHI         0x6
#define CSKY_FPUV3_IMM3_MASK        0x7
#define CSKY_FPUV3_IMM4_MASK        0xf
#define CSKY_FPUV3_IMM5_MASK        0x1f

#define FPUV3_HELPER(name) HELPER(glue(fpu3_, name))

/* FPUv3 Macro end. */


static inline void helper_update_psr(CPUCSKYState *env)
{
    env->cp0.psr &= ~0xc000c401;
    env->cp0.psr |= env->psr_s << 31;
    env->cp0.psr |= env->psr_t << 30;
    env->cp0.psr |= env->psr_bm << 10;
    env->cp0.psr |= env->psr_c;
    env->cp0.psr |= env->psr_tm << 14;
}

static inline void helper_record_psr_bits(CPUCSKYState *env)
{
    env->psr_s = PSR_S(env->cp0.psr);
    env->psr_t = PSR_T(env->cp0.psr);
    env->psr_bm = PSR_BM(env->cp0.psr);
    env->psr_c = PSR_C(env->cp0.psr);
    env->psr_tm = PSR_TM(env->cp0.psr);
}

static inline void helper_switch_regs(CPUCSKYState *env)
{
    uint32_t temps[16];
    if (env->features & (CPU_610 | CPU_C807 | CPU_C810)) {
        memcpy(temps, env->regs, 16 * 4);
        memcpy(env->regs, env->banked_regs, 16 * 4);
        memcpy(env->banked_regs, temps, 16 * 4);
    }
}

#ifdef TARGET_CSKYV2
static inline void helper_save_sp(CPUCSKYState *env)
{
    if (env->psr_t && (env->features & ABIV2_TEE)) {
        if ((env->cp0.psr & 0x2)
            && (env->features & (CPU_C807 | CPU_C810))) {
            env->stackpoint.t_asp = env->regs[14];
        } else if (env->psr_s) {
            if (((env->cp0.psr & (0xff << 16)) >= 32)
                && (env->features & CPU_E802)
                && (env->cp0.chr & (1 << 14))) {
                /* last state is in interrupt */
                env->stackpoint.t_int_sp = env->regs[14];
            } else {
                env->stackpoint.t_ssp = env->regs[14];
            }
        } else {
            env->stackpoint.t_usp = env->regs[14];
        }
    } else {
        if ((env->cp0.psr & 0x2)
            && (env->features & (CPU_C807 | CPU_C810))) {
            env->stackpoint.nt_asp = env->regs[14];
        } else if (env->psr_s) {
            if (((env->cp0.psr & (0xff << 16)) >= 32)
                && (env->features & CPU_E802)
                && (env->cp0.chr & (1 << 14))) {
                /* last state is in interrupt */
                env->stackpoint.nt_int_sp = env->regs[14];
            } else {
                env->stackpoint.nt_ssp = env->regs[14];
            }
        } else {
            env->stackpoint.nt_usp = env->regs[14];
        }
    }
}

static inline void helper_choose_sp(CPUCSKYState *env)
{
    if (env->psr_t && (env->features & ABIV2_TEE)) {
        if ((env->cp0.psr & 0x2)
            && (env->features & (CPU_C807 | CPU_C810))) {
            env->regs[14] = env->stackpoint.t_asp;
        } else if (env->psr_s) {
            if (((env->cp0.psr & (0xff << 16)) >= 32)
                && (env->features & CPU_E802)
                && (env->cp0.chr & (1 << 14))) {
                /* next state is in interrupt */
                env->regs[14] = env->stackpoint.t_int_sp;
            } else {
                env->regs[14] = env->stackpoint.t_ssp;
            }
        } else {
            env->regs[14] = env->stackpoint.t_usp;
        }
    } else {
        if ((env->cp0.psr & 0x2)
            && (env->features & (CPU_C807 | CPU_C810))) {
            env->regs[14] = env->stackpoint.nt_asp;
        } else if (env->psr_s) {
            if (((env->cp0.psr & (0xff << 16)) >= 32)
                && (env->features & CPU_E802)
                && (env->cp0.chr & (1 << 14))) {
                /* next state is in interrupt */
                env->regs[14] = env->stackpoint.nt_int_sp;
            } else {
                env->regs[14] = env->stackpoint.nt_ssp;
            }
        } else {
            env->regs[14] = env->stackpoint.nt_usp;
        }
    }
}

static inline void helper_tee_save_cr(CPUCSKYState *env)
{
    if (env->psr_t) {
        env->tee.t_vbr = env->cp0.vbr;
        env->tee.t_epsr = env->cp0.epsr;
        env->tee.t_epc = env->cp0.epc;
        env->t_mmu = env->mmu;
    } else {
        env->tee.nt_vbr = env->cp0.vbr;
        env->tee.nt_epsr = env->cp0.epsr;
        env->tee.nt_epc = env->cp0.epc;
        env->nt_mmu = env->mmu;
    }
}

static inline void helper_tee_choose_cr(CPUCSKYState *env)
{
    if (env->psr_t) {
        env->cp0.vbr = env->tee.t_vbr;
        env->cp0.epsr = env->tee.t_epsr;
        env->cp0.epc = env->tee.t_epc;
        env->mmu = env->t_mmu;
#if !defined(CONFIG_USER_ONLY)
        env->tlb_context->tlb = env->tlb_context->t_tlb;
        env->tlb_context->round_robin = env->tlb_context->t_round_robin;
#endif
    } else {
        env->cp0.vbr = env->tee.nt_vbr;
        env->cp0.epsr = env->tee.nt_epsr;
        env->cp0.epc = env->tee.nt_epc;
        env->mmu = env->nt_mmu;
#if !defined(CONFIG_USER_ONLY)
        env->tlb_context->tlb = env->tlb_context->nt_tlb;
        env->tlb_context->round_robin = env->tlb_context->nt_round_robin;
#endif
    }
}

/* For ck_tee_lite, when change from Trust to Non-Trust world by NT-interrupt,
 * need to push the GPRs to trust-supervised stack, and clear them. */
static inline void helper_tee_save_gpr(CPUCSKYState *env)
{
    int32_t i;
    if (env->features & CPU_E801) {
        for (i = 0; i <= 8; i++) {
            env->stackpoint.t_ssp -= 4;
            cpu_stl_data(env, env->stackpoint.t_ssp, env->regs[i]);
            env->regs[i] = 0;
        }
        cpu_stl_data(env, env->stackpoint.t_ssp - 4, env->regs[13]);
        env->regs[13] = 0;
        cpu_stl_data(env, env->stackpoint.t_ssp - 8, env->regs[15]);
        env->regs[15] = 0;
        env->stackpoint.t_ssp -= 8;
    } else if (env->features & CPU_E802) {
        for (i = 0; i <= 13; i++) {
            env->stackpoint.t_ssp -= 4;
            cpu_stl_data(env, env->stackpoint.t_ssp, env->regs[i]);
            env->regs[i] = 0;
        }
        cpu_stl_data(env, env->stackpoint.t_ssp - 4, env->regs[15]);
        env->regs[15] = 0;
        env->stackpoint.t_ssp -= 4;
    } else if (env->features & CPU_E803) {
        for (i = 0; i <= 13; i++) {
            env->stackpoint.t_ssp -= 4;
            cpu_stl_data(env, env->stackpoint.t_ssp, env->regs[i]);
            env->regs[i] = 0;
        }
        cpu_stl_data(env, env->stackpoint.t_ssp - 4, env->regs[15]);
        env->regs[15] = 0;
        cpu_stl_data(env, env->stackpoint.t_ssp - 8, env->regs[28]);
        env->regs[28] = 0;
        env->stackpoint.t_ssp -= 8;
    }
}

/* For ck_tee_lite, when return from NT-interrupt which change the world from
 * Trust to Non-Trust world before, need to pop the GPRs from
 * trust-supervised stack. */
static inline void helper_tee_restore_gpr(CPUCSKYState *env)
{
    int32_t i;
    if (env->features & CPU_E801) {
        env->regs[15] = cpu_ldl_data(env, env->stackpoint.t_ssp);
        env->regs[13] = cpu_ldl_data(env, env->stackpoint.t_ssp + 4);
        env->stackpoint.t_ssp += 8;
        for (i = 8; i >= 0; i--) {
            env->regs[i] = cpu_ldl_data(env, env->stackpoint.t_ssp);
            env->stackpoint.t_ssp += 4;
        }
    } else if (env->features & CPU_E802) {
        env->regs[15] = cpu_ldl_data(env, env->stackpoint.t_ssp);
        env->stackpoint.t_ssp += 4;
        for (i = 13; i >= 0; i--) {
            env->regs[i] = cpu_ldl_data(env, env->stackpoint.t_ssp);
            env->stackpoint.t_ssp += 4;
        }
    } else if (env->features & CPU_E803) {
        env->regs[28] = cpu_ldl_data(env, env->stackpoint.t_ssp);
        env->regs[15] = cpu_ldl_data(env, env->stackpoint.t_ssp + 4);
        env->stackpoint.t_ssp += 8;
        for (i = 13; i >= 0; i--) {
            env->regs[i] = cpu_ldl_data(env, env->stackpoint.t_ssp);
            env->stackpoint.t_ssp += 4;
        }
    }
}
#endif

static inline int has_insn(DisasContext *ctx, uint64_t flags)
{
    if (ctx->features & flags) {
        return 1;
    } else {
        return 0;
    }
}

static inline void print_exception(DisasContext *ctx, int excp)
{
    switch (excp) {
    case EXCP_CSKY_RESET:
    case EXCP_CSKY_ALIGN:
    case EXCP_CSKY_DATA_ABORT:
    case EXCP_CSKY_DIV:
    case EXCP_CSKY_UDEF:
    case EXCP_CSKY_PRIVILEGE:
    case EXCP_CSKY_TRACE:
    case EXCP_CSKY_BKPT:
    case EXCP_CSKY_URESTORE:
    case EXCP_CSKY_IDLY4:
    case EXCP_CSKY_HAI:
        qemu_log_mask(LOG_GUEST_ERROR, "##exception No = 0x%x\n", excp);
        qemu_log_mask(LOG_GUEST_ERROR, "##exception pc = 0x%x\n", ctx->pc);
        break;
    default:
        break;
    }
}
#endif
