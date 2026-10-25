/*
 *  META translation
 *
 *  Copyright (c) 2010 Imagination Technologies
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
#include "qapi/error.h"
#include "hw/core/boards.h"
#include "qemu/main-loop.h"
#include "qemu/qemu-print.h"
#include "exec/log.h"
#include "cpu.h"
#include "system/cpus.h"
#include "core.h"
#include "disas/disas.h"
#include "tcg/tcg-op.h"
#include "qemu/log.h"
#include "core_registers.h"
#include "minim.h"
#include "hw/core/ptimer.h"
#include "fpu/softfloat.h"
#include "qemu/host-utils.h"
#include "dsp_sincos_table.h"
#include "bits.h"
#include "instruction.h"
#include "log.h"
#include "hw/meta/meta_switch.h"

#include "exec/helper-proto.h"
#include "exec/helper-gen.h"
#include "exec/translator.h"
#include "exec/translation-block.h"
#include "accel/tcg/cpu-ldst.h"

#define HELPER_H "helper.h"
#include "exec/helper-info.c.inc"
#undef  HELPER_H

/*
 * Another thread's registers may be -MAX_THREADS+1..MAX_THREADS-1.
 * These can be accessed relative to the current env.
 */
#define META_T2T_MID            (META_MAX_THREADS - 1)
#define META_T2T_NR             (META_MAX_THREADS*2 - 1)
#define META_T2T(OTHER, THIS)   (META_T2T_MID + (OTHER) - (THIS))

/* global register arrays are indexed first by thread accessing it */
static TCGv cpu_cregs[31];
static TCGv cpu_gcregs[META_MAX_THREADS][1];
static TCGv cpu_dregs[2][16];
static TCGv cpu_gdregs[META_MAX_THREADS][2][16];
static TCGv cpu_aregs[2][8];
static TCGv cpu_garegs[META_MAX_THREADS][2][8];
static TCGv cpu_pc[META_PC_MAX];
static TCGv cpu_tregs[META_TR_MAX];
static TCGv cpu_ttregs[4];
static TCGv cpu_gttregs[META_MAX_THREADS][1];
static TCGv cpu_fxregs[16];
static TCGv cpu_cf[5];
static TCGv cpu_lsmstep;
static TCGv cpu_lnkaddr[META_T2T_NR];
static TCGv_i64 cpu_accregs[2][1];
static TCGv_i64 cpu_gaccregs[META_MAX_THREADS][2][3];
static TCGv cpu_dspram_rp[2][2][2];
static TCGv cpu_dspram_rpi[2][2][2];
static TCGv cpu_dspram_wp[2][2][2];
static TCGv cpu_dspram_wpi[2][2][2];
static TCGv cpu_dspram_fetched[2][2][2];
static TCGv cpu_dspram_off_and[2];
static TCGv cpu_dspram_off_or[2];
static TCGv cpu_aumod_mask;
static TCGv cpu_readport_idx_w;
static TCGv cpu_readport_idx_r;
static TCGv cpu_readport_count;
#if !defined(CONFIG_USER_ONLY)
static TCGv cpu_glock[META_MAX_THREADS];
#endif
static TCGv cpu_repcyc;
static TCGv cpu_kicks;
static TCGv cpu_ikicks;
static TCGv cpu_switch_code;
static TCGv cpu_switch_nextpc;
static TCGv cpu_codebcount[META_MAX_THREADS][4];


#define DISAS_META
#define DEBUG_LEVEL 0

#ifdef DISAS_META
#  define LOG_DIS(...) qemu_log_mask(CPU_LOG_TB_IN_ASM, ## __VA_ARGS__)
#else
#  define LOG_DIS(...) do { } while (0)
#endif

#if DEBUG_LEVEL >= 2
#  define DBGLOG(...) fprintf(stderr, ## __VA_ARGS__)
#elif DEBUG_LEVEL >= 1
#  define DBGLOG(...) qemu_log(__VA_ARGS__)
#else
#  define DBGLOG(...) do { } while (0)
#endif

/* A writable temp holding a constant (tcg_constant_* must not be written). */
static inline TCGv_i32 meta_const_i32(int32_t val)
{
    TCGv_i32 t = tcg_temp_new_i32();
    tcg_gen_movi_i32(t, val);
    return t;
}

static inline TCGv_i64 meta_const_i64(int64_t val)
{
    TCGv_i64 t = tcg_temp_new_i64();
    tcg_gen_movi_i64(t, val);
    return t;
}

#define meta_const_tl meta_const_i32

/* see http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSet64 */
#define NBITSSET(v) \
    (((v) * 0x200040008001ULL & 0x111111111111111ULL) % 0xf)


typedef struct DisasContext {
    DisasContextBase base;
    CPUArchState *env;
    MetaCore *core;
    target_ulong last_pc;   /* so we don't re-disassemble LSMs */
    target_ulong pc;
    target_ulong next_pc;
    target_ulong next_pc_nohwloop;
    /* instructions */
    target_ulong instr_count;
    /* Last PC fetched */
    target_ulong last_fetch;

    /* Decoder.  */
    void (*decoder)(struct DisasContext *dc);

    /* flags the TB was translated with (both halves) */
    uint64_t tb_flags;
    int is_jmp;

    /* instruction info */
    target_ulong instr;

#define JMP_NOJMP    0
#define JMP_DIRECT   1
#define JMP_INDIRECT 2
    int jmp;

    /* current tb flags (modified from tb->flags) */
    uint64_t tbflags;
    /* delayed tb flags to change (xor after this instruction) */
    uint64_t delayed_tbflags;

    /* register access is using DEFR instruction */
    char reg_defr;
    /* register access is using KICK instruction */
    char reg_kick;
    /* indicates an exception to the normal register priv rules */
    char reg_priv_exception;

    /* Catch state, captured when restoring state after a fault */
    unsigned char catching, caught;
    /* Translation time TXCATCH0, for restoring TXCATCH0.
     * Differs from TXCATCH0 for writes:
     * - need to know the source registers to get the value.
     * - the write mask is unknown until the address is known.
     * These load fields are therefore set for writes too so that TXCATCH0 can
     * be restored in the event of a fault.
     */
    uint32_t catch0;
    /* Translation time catch address, for restoring TXCATCH0 and TXCATCH1.
     * Differs completely from TXCATCH1 as the address isn't known at
     * translation time so must be extracted from registers (see #defines below
     * for the fields).
     */
    uint32_t catch1;
#define CATCH0_UB_MASK      0x0000000f  /* base unit */
#define CATCH0_UB_SHIFT     0
#define CATCH0_RB_MASK      0x000001f0  /* base register */
#define CATCH0_RB_SHIFT     4
#define CATCH0_UA_MASK      0x00000200  /* update base address */
#define CATCH0_UA_SHIFT     9
#define CATCH0_PP_MASK      0x00000400  /* post increment base address */
#define CATCH0_PP_SHIFT     10
#define CATCH0_OIMM_MASK    0x00000800  /* offset from base is immediate */
#define CATCH0_OIMM_SHIFT   11
/* when offset is immediate */
#define CATCH0_IMM_MASK     0xfffff000  /* signed immediate offset from base in
                                           bytes (assumed at msb) */
#define CATCH0_IMM_SHIFT    12
/* when offset isn't immediate */
#define CATCH0_OREG_MASK    0x00001000  /* offset from base is in register */
#define CATCH0_OREG_SHIFT   12
#define CATCH0_UO_MASK      0x0001e000  /* offset unit */
#define CATCH0_UO_SHIFT     13
#define CATCH0_RO_MASK      0x001e0000  /* offset register */
#define CATCH0_RO_SHIFT     17
#define CATCH0_LNK_MASK     0x00200000  /* lnkget or lnkset */
#define CATCH0_LNK_SHIFT    21

    /* LSM index, increment from base register for multiple issues */
    uint16_t lsm_idx;

    /* true if hardware loop decrement has been handled by the _gen function */
    bool hwloop_handled;
} DisasContext;

#if defined(CONFIG_USER_ONLY)
#define IS_USER(dc) MMU_USER_IDX
#else
#define IS_USER(dc) tb_mmu_index(dc->tbflags)
#define COUNT_ACTCYC
#endif

struct {
    MetaOpWidth width;
    bool heed_l1;
} readport_info[META_RA_MAX] = {
    [0 ... META_RA_MAX-1] = { WIDTH_32B, true },
    [META_RA]             = { WIDTH_32B, true },
    [META_RABZ]           = { WIDTH_8B,  false },
    [META_RAWZ]           = { WIDTH_16B, false },
    [META_RADZ]           = { WIDTH_32B, false },
    [META_RABX]           = { WIDTH_8B,  false },
    [META_RAWX]           = { WIDTH_16B, false },
    [META_RADX]           = { WIDTH_32B, false },
    [META_RAM8X]          = { WIDTH_16B, true },
    [META_RAM8X32]        = { WIDTH_8B,  true },
    [META_RAM16X]         = { WIDTH_16B, true },
};

/* dc->is_jmp values, the META translator's own */
#define DISAS_JUMP    DISAS_TARGET_0 /* only pc was modified dynamically */
#define DISAS_UPDATE  DISAS_TARGET_1 /* cpu state was modified dynamically */
#define DISAS_TB_JUMP DISAS_TARGET_2 /* only pc was modified statically */

static int handle_breakpoint(DisasContext *dc);

static uint32_t *meta_raw_get_reg(CPUMETAState *env, MetaUnit unit, int reg);

static void meta_minim_init_table(void);
static uint32_t meta_minim_decode_insn(uint16_t **minim_ptr, size_t max_words);

static void meta_decode_instr(DisasContext *dc, uint32_t instr);
static void meta_decoder(DisasContext *dc);
static void minim_decoder(DisasContext *dc);

static inline MetaDUArithMode meta_dc_duarithmode(DisasContext *dc)
{
    return (dc->tbflags & META_TBFLAG_DUARITHMODE) >> META_TBFLAG_DUMODE_SHIFT;
}

static inline bool meta_au_nonlinear(DisasContext *dc, int au)
{
    assert(au >= 0 && au < 2);

    if (dc->tbflags & META_TBFLAG_ISTAT) {
        /* always linear with IStat set */
        return false;
    }

    /* find whether we're using linear addressing */
    if (au) {
        return dc->tbflags & META_TBFLAG_AU1NLINEAR;
    }
    return dc->tbflags & META_TBFLAG_AU0NLINEAR;
}

static void gen_counters(DisasContext *dc)
{
#ifdef COUNT_ACTCYC
    tcg_gen_addi_i32(cpu_cregs[META_TXTACTCYC], cpu_cregs[META_TXTACTCYC],
                     dc->instr_count);
#endif
}

/*
 * Update any delayed state changes made during the course of the TB.
 * This writes translation-time things that are updated frequently enough that
 * you don't want to keep writing them to memory between memory accesses etc
 * but that need to be written before exiting the TB.
 * In the event of a fault, these should be restored by restore_state_to_opc.
 */
static void gen_update_state(DisasContext *dc)
{
    int flags_diff = dc->tbflags ^ dc->tb_flags;
    /* load/store multiple step number */
    if (flags_diff & META_TBFLAG_LSM) {
        tcg_gen_movi_i32(cpu_lsmstep,
                    (dc->tbflags & META_TBFLAG_LSM) >> META_TBFLAG_LSM_SHIFT);
    }
}

static inline void t_gen_raise_exception(uint32_t index)
{
        TCGv_i32 tmp = meta_const_i32(index);
        gen_helper_raise_exception(tcg_env, tmp);
}

static void gen_single_step(void)
{
#if defined(CONFIG_USER_ONLY)
        t_gen_raise_exception(EXCP_DEBUG);
#else
        gen_helper_thread_stop(tcg_env);
#endif
}

static void gen_hwloop_dec(DisasContext *dc)
{
    if ((dc->tbflags & META_TBFLAG_TXL2COUNTNZ) &&
        (dc->pc == dc->env->cregs[META_TXL2END])) {
        tcg_gen_subi_i32(cpu_cregs[META_TXL2COUNT],
                         cpu_cregs[META_TXL2COUNT], 1);
    } else if ((dc->tbflags & META_TBFLAG_TXL1COUNTNZ) &&
               (dc->pc == dc->env->cregs[META_TXL1END])) {
        tcg_gen_subi_i32(cpu_cregs[META_TXL1COUNT],
                         cpu_cregs[META_TXL1COUNT], 1);
    }
}

static void gen_goto_tb(DisasContext *dc, int n, target_ulong dest)
{
    int chain;
    target_ulong dest_addr = meta_pc_to_virt(dc->env, dest);
    if (dc->tbflags & META_TBFLAG_STEP) {
        /* don't chain while hardware single stepping */
        chain = 0;
        gen_single_step();
    } else {
        chain = translator_use_goto_tb(&dc->base, dest_addr);
    }

    gen_update_state(dc);
    gen_counters(dc);
    if (chain) {
        tcg_gen_goto_tb(n);
        tcg_gen_movi_tl(cpu_pc[META_PC], dest);
        tcg_gen_exit_tb(dc->base.tb, n);
    } else {
        tcg_gen_movi_tl(cpu_pc[META_PC], dest);
        tcg_gen_exit_tb(NULL, 0);
    }
}

/*
 * Each instruction records for a fault inside it: the PC to resume at (META
 * completes the instruction and replays the access from the catch buffer),
 * LSM_STEP, and the catch state.  The three insn_start words are filled in
 * once the instruction has been decoded.
 */
static void meta_tr_init_disas_context(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *dc = container_of(dcbase, DisasContext, base);
    CPUArchState *env = cpu_env(cs);
    uint64_t flags = dc->base.tb->flags |
                     ((uint64_t)dc->base.tb->cs_base << 32);

    if (flags & META_TBFLAG_MINIM) {
        dc->decoder = minim_decoder;
    } else {
        dc->decoder = meta_decoder;
    }
    dc->env = env;
    dc->core = META_THREAD2CORE(env);
    dc->is_jmp = DISAS_NEXT;
    dc->jmp = JMP_NOJMP;
    dc->tb_flags = flags;
    dc->tbflags = flags;
    dc->delayed_tbflags = 0;
    dc->reg_defr = 0;
    dc->reg_kick = 0;
    dc->reg_priv_exception = 0;
    dc->last_pc = -1;
    dc->pc = meta_virt_to_pc(env, dc->base.pc_first);
    dc->next_pc = dc->pc;
    dc->instr_count = 0;
    dc->catching = 1;
    dc->caught = 0;

    /* single step */
    if (dc->tbflags & META_TBFLAG_STEP) {
        dc->base.max_insns = 1;
    }

    LOG_DIS("T%d IN:\n", env->thread_num);
}

static void meta_tr_tb_start(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *dc = container_of(dcbase, DisasContext, base);

    /* catch replay triggered by thread enable */
    if (dc->tbflags & META_TBFLAG_REPLAY) {
        gen_helper_replay_catch(tcg_env);
        if (!(dc->tbflags & META_TBFLAG_ISTAT)) {
            TCGv temp = tcg_temp_new_i32();
            gen_helper_unmask_itrigger(temp, tcg_env);
        }
        tcg_gen_exit_tb(NULL, 0);
        LOG_DIS(" Catch Replay\n");
        dc->base.is_jmp = DISAS_NORETURN;
    }
}

static void meta_tr_insn_start(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *dc = container_of(dcbase, DisasContext, base);

    /* the PC and LSM_STEP the instruction starts with */
    tcg_gen_insn_start(dc->pc,
                       (uint64_t)((dc->tbflags & META_TBFLAG_LSM) >>
                                  META_TBFLAG_LSM_SHIFT) << 32, 0);
}

static void meta_tr_translate_insn(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *dc = container_of(dcbase, DisasContext, base);
    CPUArchState *env = dc->env;

    if (dc->base.is_jmp == DISAS_NORETURN) {
        /* catch replay TB: no instruction, but give it a size */
        dc->base.pc_next += 4;
        return;
    }
    /* delayed update to tbflags */
    if (dc->delayed_tbflags) {
        dc->tbflags ^= dc->delayed_tbflags;
        dc->delayed_tbflags = 0;
    }
    if (dc->pc != dc->last_pc) {
        LOG_DIS(" ");
    }
    dc->hwloop_handled = false;
    if (!handle_breakpoint(dc)) {
        dc->decoder(dc);
    }
    if (((dc->tbflags & META_TBFLAG_TXL2COUNTNZ) &&
         (dc->pc == env->cregs[META_TXL2END])) ||
        ((dc->tbflags & META_TBFLAG_TXL1COUNTNZ) &&
         (dc->pc == env->cregs[META_TXL1END]))) {
        dc->is_jmp = DISAS_JUMP;
    }
    dc->last_pc = dc->pc;
    dc->pc = dc->next_pc;

    /*
     * Where a fault in this instruction resumes: META completes it, so the
     * next PC and LSM_STEP, plus the catch state for the replay.
     */
    {
        TCGOp *op = dc->base.insn_start;
        uint64_t w0 = (uint32_t)tcg_get_insn_start_param(op, 0) |
                      ((uint64_t)dc->pc << 32);
        uint64_t w1 = tcg_get_insn_start_param(op, 1) |
            ((uint64_t)((dc->tbflags & META_TBFLAG_LSM) >>
                        META_TBFLAG_LSM_SHIFT) << 36);

        if (dc->caught) {
            w1 |= dc->catch0;
            tcg_set_insn_start_param(dc->base.insn_start, 2, dc->catch1);
        }
        tcg_set_insn_start_param(dc->base.insn_start, 0, w0);
        tcg_set_insn_start_param(dc->base.insn_start, 1, w1);
    }
    dc->caught = 0;

    dc->base.pc_next = meta_pc_to_virt(env, dc->pc);
    if (dc->jmp || dc->is_jmp) {
        dc->base.is_jmp = DISAS_TARGET_0;
    } else if ((dc->base.pc_next & TARGET_PAGE_MASK) !=
               (dc->base.pc_first & TARGET_PAGE_MASK)) {
        dc->base.is_jmp = DISAS_TOO_MANY;
    }
}

static void meta_tr_tb_stop(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *dc = container_of(dcbase, DisasContext, base);
    target_ulong npc;

    if (dc->base.is_jmp == DISAS_NORETURN) {
        return;
    }

    dc->pc = dc->last_pc;
    if (!(dc->tbflags & META_TBFLAG_LSM) &&
        !dc->hwloop_handled) {
        gen_hwloop_dec(dc);
    }
    npc = dc->next_pc;

    /*
     * code added here will probably also need to be added to
     * meta_set_safe_label
     */
    switch (dc->is_jmp) {
    case DISAS_NEXT:
        gen_goto_tb(dc, 1, npc);
        break;
    default:
    case DISAS_UPDATE:
        tcg_gen_movi_tl(cpu_pc[META_PC], npc);
        /* fall through */
    case DISAS_JUMP:
        /* indicate that the hash table must be used to find the next TB */
        gen_update_state(dc);
        gen_counters(dc);
        /* stop thread if hardware single stepping */
        if (dc->tbflags & META_TBFLAG_STEP) {
            gen_single_step();
        }
        tcg_gen_exit_tb(NULL, 0);
        break;
    case DISAS_TB_JUMP:
        /* nothing more to generate */
        break;
    }
}

static const TranslatorOps meta_tr_ops = {
    .init_disas_context = meta_tr_init_disas_context,
    .tb_start           = meta_tr_tb_start,
    .insn_start         = meta_tr_insn_start,
    .translate_insn     = meta_tr_translate_insn,
    .tb_stop            = meta_tr_tb_stop,
};

void meta_translate_code(CPUState *cs, TranslationBlock *tb,
                         int *max_insns, vaddr pc, void *host_pc)
{
    DisasContext dc;

    translator_loop(cs, tb, max_insns, pc, host_pc, &meta_tr_ops, &dc.base,
                    TCG_TYPE_VA);
}

static const uint32_t meta_unit_to_lddst[16] = {
    [META_UNIT_CT]      = META_TXCATCH0_LDDST_CT_MASK,
    [META_UNIT_D0]      = META_TXCATCH0_LDDST_D0_MASK,
    [META_UNIT_D1]      = META_TXCATCH0_LDDST_D1_MASK,
    [META_UNIT_A0]      = META_TXCATCH0_LDDST_A0_MASK,
    [META_UNIT_A1]      = META_TXCATCH0_LDDST_A1_MASK,
    [META_UNIT_PC]      = META_TXCATCH0_LDDST_PC_MASK,
    [META_UNIT_TR]      = META_TXCATCH0_LDDST_TR_MASK,
    [META_UNIT_FX]      = META_TXCATCH0_LDDST_FX_MASK,
};

/*
 * Catch state emulation.
 * When restoring CPU state dc->catching will be set. Each instruction that can
 * generate catch state should set the catch variables in dc.
 * See replay_catchbuffer in arch/metag/kernel/traps.c of Linux kernel for
 * a software replay of the catch state.
 */

/* catch a memory access address in a register */
static void gen_addr_catch(DisasContext *dc, MetaUnit ub, int rb)
{
    uint32_t catch1;

    if (likely(!dc->catching)) {
        return;
    }

    catch1 = ub << CATCH0_UB_SHIFT;
    catch1 |= rb << CATCH0_RB_SHIFT;

    dc->catch1 = catch1;
}

/* catch a memory access address in a register with an immediate offset */
static void gen_addr_catch_imm(DisasContext *dc, int ua, int pp,
                               MetaUnit ub, int rb, int imm)
{
    uint32_t catch1;

    if (likely(!dc->catching)) {
        return;
    }

    catch1 = ub << CATCH0_UB_SHIFT;
    catch1 |= rb << CATCH0_RB_SHIFT;
    if (ua) {
        catch1 |= CATCH0_UA_MASK;
    }
    if (pp) {
        catch1 |= CATCH0_PP_MASK;
    }
    catch1 |= CATCH0_OIMM_MASK;
    catch1 |= imm << CATCH0_IMM_SHIFT;

    dc->catch1 = catch1;
}

/* catch a memory access address in a register with a register offset */
static void gen_addr_catch_reg(DisasContext *dc, int ua, int pp,
                               MetaUnit ub, int rb,
                               MetaUnit uo, int ro)
{
    uint32_t catch1;

    if (likely(!dc->catching)) {
        return;
    }

    catch1 = ub << CATCH0_UB_SHIFT;
    catch1 |= rb << CATCH0_RB_SHIFT;
    if (ua) {
        catch1 |= CATCH0_UA_MASK;
    }
    if (pp) {
        catch1 |= CATCH0_PP_MASK;
    }
    catch1 |= CATCH0_OREG_MASK;
    catch1 |= uo << CATCH0_UO_SHIFT;
    catch1 |= ro << CATCH0_RO_SHIFT;

    dc->catch1 = catch1;
}

static void gen_getset_catch(DisasContext *dc, int set,
                             MetaUnit ud1, int rd, int l)
{
    uint32_t catch0 = 0;
    MetaUnit ud2;

    if (unlikely(!meta_unit_to_lddst[ud1] && (ud1 != META_UNIT_RA))) {
        cpu_abort(env_cpu(dc->env), "%s: Unimplemented unit %d "
                  "on %08x at %08x\n",
                  __func__,
                  ud1,
                  dc->instr, dc->pc);
        return;
    }
    catch0 |= meta_unit_to_lddst[ud1];

    if (l == 3 && ud1 != META_UNIT_FX) {
        ud2 = meta_unit_partner(ud1);
        if (unlikely((ud2 != META_UNIT_RA) && !meta_unit_to_lddst[ud2])) {
            cpu_abort(env_cpu(dc->env), "%s: Unimplemented partner unit %d of unit %d "
                      "on %08x at %08x\n",
                      __func__,
                      ud2, ud1,
                      dc->instr, dc->pc);
            return;
        }
        catch0 |= meta_unit_to_lddst[ud2];
    }
    /* second in pair */
    if (ud1 == META_UNIT_D1 || ud1 == META_UNIT_A1) {
        catch0 |= META_TXCATCH0_LDM16_MASK;
    }

    if (likely(!dc->catching)) {
        return;
    }


    dc->caught = 1;

    catch0 |= rd << META_TXCATCH0_LDRXX_SHIFT;
    if (dc->tbflags & META_TBFLAG_PSTAT) {
        catch0 |= META_TXCATCH0_PRIV_MASK;
    }
    if (!set) {
        catch0 |= META_TXCATCH0_READ_MASK;
    }
    catch0 |= l;

    dc->catch0 = catch0;
}

static void gen_read_catch(DisasContext *dc, int write, int ra)
{
    uint32_t catch0 = 0;

    if (likely(!dc->catching)) {
        return;
    }

    dc->caught = 1;

    if (dc->tbflags & META_TBFLAG_PSTAT) {
        catch0 |= META_TXCATCH0_PRIV_MASK;
    }
    if (!write) {
        catch0 |= META_TXCATCH0_READ_MASK;
    }
    catch0 |= ra;

    dc->catch0 = catch0;
}

/* same as gen_getset_catch, but sets LNK flag */
static void gen_lnkgetset_catch(DisasContext *dc, int set,
                                MetaUnit ud1, int rd, int l)
{
    gen_getset_catch(dc, set, ud1, rd, l);
    if (set) {
        dc->catch1 |= CATCH0_LNK_MASK;
    } else {
        dc->catch0 |= META_TXCATCH0_LNKGET_MASK;
    }
}

/* get pointers to dst registers of a load, return 0 on success */
static int get_catch_ld_dsts(CPUArchState *env, uint32_t catch0,
                             uint32_t **dstp1, uint32_t **dstp2)
{
    unsigned int dst;
    MetaUnit ud1, ud2;
    int rd, l;
    uint32_t *dst1, *dst2 = NULL;
    int preproc;

    preproc = catch0 & META_TXCATCH0_PREPROC_MASK;
    rd = (catch0 & META_TXCATCH0_LDRXX_MASK) >> META_TXCATCH0_LDRXX_SHIFT;
    l = (catch0 & META_TXCATCH0_LDL2L1_MASK) >> META_TXCATCH0_LDL2L1_SHIFT;
    dst = catch0 & META_TXCATCH0_LDDST_MASK;
    ud2 = -1;
    switch (dst) {
    /* control and trigger units not supported yet */
    case META_TXCATCH0_LDDST_D0_MASK:
        ud1 = META_UNIT_D0;
        break;
    case META_TXCATCH0_LDDST_D1_MASK:
        ud1 = META_UNIT_D1;
        break;
    case META_TXCATCH0_LDDST_A0_MASK:
        ud1 = META_UNIT_A0;
        break;
    case META_TXCATCH0_LDDST_A1_MASK:
        ud1 = META_UNIT_A1;
        break;
    case META_TXCATCH0_LDDST_PC_MASK:
        ud1 = META_UNIT_PC;
        break;
    /* long ops */
    case META_TXCATCH0_LDDST_D_MASK:
        assert(l == 3);
        ud1 = preproc ? META_UNIT_D1 : META_UNIT_D0;
        ud2 = preproc ? META_UNIT_D0 : META_UNIT_D1;
        break;
    case META_TXCATCH0_LDDST_A_MASK:
        assert(l == 3);
        ud1 = preproc ? META_UNIT_A1 : META_UNIT_A0;
        ud2 = preproc ? META_UNIT_A0 : META_UNIT_A1;
        break;
    default:
        cpu_abort(env_cpu(env), "%s: Unimplemented catch register units 0x%x, "
                  "pc=0x%08x\n",
                  __func__,
                  dst, env->pc[META_PC]);
        return 1;
    }
    dst1 = meta_raw_get_reg(env, ud1, rd);
    if (unlikely(!dst1)) {
        cpu_abort(env_cpu(env), "%s: Bad catch dst1 register unit %d reg %d, "
                  "pc=0x%08x\n",
                  __func__,
                  ud1, rd, env->pc[META_PC]);
        return 1;
    }
    if (l == 3) {
        dst2 = meta_raw_get_reg(env, ud2, rd);
        if (unlikely(!dst2)) {
            cpu_abort(env_cpu(env), "%s: Bad catch dst2 register unit %d reg %d, "
                      "pc=0x%08x\n",
                      __func__,
                      ud2, rd, env->pc[META_PC]);
            return 1;
        }
    }

    *dstp1 = dst1;
    *dstp2 = dst2;
    return 0;
}

/* Update fault reason code in catch0 */
void meta_update_txcatch_signum(MetaSigNum signum, uint32_t *catch0)
{
    MetaFReason freason = META_SIGNUM_FR(signum);

    *catch0 = (*catch0 & ~META_TXCATCH0_FAULT_MASK)
            | (freason << META_TXCATCH0_FAULT_SHIFT);
}

/* update TXCATCH registers from internal catch state */
static void update_txcatch(CPUArchState *env, uint32_t catch0, uint32_t catch1)
{
    target_ulong addr;
    MetaUnit ub, uo;
    int rb, ro, l, ua, pp, lnk = 0;
    uint32_t *base, *offs, *src1, *src2;
    uint32_t wmask, catch2, catch3;
    MetaFReason freason;

    freason = (env->cregs[META_TXSTATUS] & META_TXSTATUS_FREASON_MASK)
                    >> META_TXSTATUS_FREASON_SHIFT;
    meta_update_txcatch_signum(META_SIGNUM(freason, 0), &catch0);

    /* Calculate the address of the caught memory access */
    ub = (catch1 & CATCH0_UB_MASK) >> CATCH0_UB_SHIFT;
    rb = (catch1 & CATCH0_RB_MASK) >> CATCH0_RB_SHIFT;
    base = meta_raw_get_reg(env, ub,  rb);
    if (!base) {
        cpu_abort(env_cpu(env), "%s: Bad base register unit %d reg %d, pc=0x%08x\n",
                  __func__,
                  ub, rb, env->pc[META_PC]);
        return;
    }
    addr = *base;
    ua = catch1 & CATCH0_UA_MASK;
    pp = catch1 & CATCH0_PP_MASK;
    if (catch1 & CATCH0_OIMM_MASK) {
        /* immediate offset */
        int32_t offset = (int32_t)(catch1 & CATCH0_IMM_MASK)
                                >> CATCH0_IMM_SHIFT;
        if (!ua || !pp) {
            addr = helper_au_add(env, ub - META_UNIT_A0, addr, offset);
        }
        if (ua) {
            *base = helper_au_add(env, ub - META_UNIT_A0, *base, offset);
        }
    } else if (catch1 & CATCH0_OREG_MASK) {
        /* register offset */
        uo = (catch1 & CATCH0_UO_MASK) >> CATCH0_UO_SHIFT;
        ro = (catch1 & CATCH0_RO_MASK) >> CATCH0_RO_SHIFT;
        offs = meta_raw_get_reg(env, uo, ro);
        if (!offs) {
            cpu_abort(env_cpu(env),
                      "%s: Bad offset register unit %d reg %d, pc=0x%08x\n",
                      __func__,
                      uo, ro, env->pc[META_PC]);
            return;
        }
        if (!ua || !pp) {
            addr = helper_au_add(env, ub - META_UNIT_A0, addr, *offs);
        }
        if (ua) {
            *base = helper_au_add(env, ub - META_UNIT_A0, *base, *offs);
        }
    } else {
        /* no offset */
        lnk = catch1 & CATCH0_LNK_MASK;
    }
    catch1 = addr;

    /* If writing, convert internal catch to META catch */
    catch2 = 0;
    catch3 = 0;
    if (!(catch0 & META_TXCATCH0_READ_MASK)) {
        if (unlikely(get_catch_ld_dsts(env, catch0, &src1, &src2))) {
            return;
        }
        l = (catch0 & META_TXCATCH0_LDL2L1_MASK) >> META_TXCATCH0_LDL2L1_SHIFT;
        if (src1) {
            catch2 = *src1;
        }
        if (src2) {
            catch3 = *src2;
        }
        catch0 &= ~(META_TXCATCH0_LDRXX_MASK |
                    META_TXCATCH0_LDDST_MASK |
                    META_TXCATCH0_WMASK_MASK);
        /* write mask is active low byte lane mask */
        if (lnk) {
            wmask = 0xff;   /* LNKSET */
        } else {
            wmask = ~(((1 << (1 << l)) - 1) << (catch1 & 0x7 & (-1 << l)));
        }
        catch0 |= wmask & META_TXCATCH0_WMASK_MASK;
        /* rearrange the bytes with same unaligned behaviour as META */
        if (catch1 & 0x1) { /* catch2[8..15] = catch2[0..7] */
            catch2 &= 0xffff00ff;
            catch2 |= (catch2 & 0xff) << 8;
        }
        if (catch1 & 0x2) { /* catch2[16..31] = catch2[0..15] */
            catch2 &= 0x0000ffff;
            catch2 |= catch2 << 16;
        }
        if (catch1 & 0x4) {
            catch3 = catch2;
        }
        /* mask out unused bytes */
        if (l < 3) {
            if (l < 2) {
                uint32_t mask = (1 << ((1 << l) << 3)) - 1;
                unsigned int shift = 0x3 & catch1 & (-1 << l);
                mask <<= (shift << 3);
                catch2 &= mask;
                catch3 &= mask;
            }
            if (catch1 & 0x4) {
                catch2 = 0;
            } else {
                catch3 = 0;
            }
        }
    }

    /* Update TXCATCH registers */
    env->cregs[META_TXCATCH0] = catch0;
    env->cregs[META_TXCATCH1] = catch1;
    env->cregs[META_TXCATCH2] = catch2;
    env->cregs[META_TXCATCH3] = catch3;
#if DEBUG_LEVEL >= 3
    DBGLOG("CATCH: %08x %08x %08x %08x pc=0x%08x\n",
           catch0, catch1, catch2, catch3,
           env->pc[META_PC]);
#endif

    /* Update TXSTATUS to show what type of catch data has been caught */
    if (env->cregs[META_TXSTATUS] & META_TXSTATUS_ISTAT_MASK) {
        env->cregs[META_TXSTATUS] |= META_TXSTATUS_CBIMARKER_MASK;
    } else {
        env->cregs[META_TXSTATUS] |= META_TXSTATUS_CBMARKER_MASK;
    }
}

/*
 * insn_start words: [0] PC | next PC << 32, [1] TXCATCH0 | LSM_STEP before
 * << 32 | LSM_STEP after << 36, [2] TXCATCH1.  Restoring goes back to the
 * start of the instruction (as an I/O rewind needs); a memory fault then
 * calls meta_restore_fault_state() to complete it the META way.
 */
void meta_restore_state_to_opc(CPUState *cs, const TranslationBlock *tb,
                               const uint64_t *data)
{
    CPUArchState *env = cpu_env(cs);

    env->pc[META_PC] = (uint32_t)data[0];
    env->lsmstep = (data[1] >> 32) & 0xf;

    env->fault_next_pc = data[0] >> 32;
    env->fault_lsmstep = (data[1] >> 36) & 0xf;
    env->fault_catch0 = (uint32_t)data[1];
    env->fault_catch1 = data[2];
}

void meta_restore_fault_state(CPUArchState *env)
{
    env->pc[META_PC] = env->fault_next_pc;
    env->lsmstep = env->fault_lsmstep;

    /* update TXCATCH registers */
    if (env->fault_catch0) {
        update_txcatch(env, env->fault_catch0, env->fault_catch1);
    }
}

static const struct MetaUnitInfo {
    /*
     * max 32 registers per unit
     * registers 0..(num_locals-1) are local to the thread
     * registers num_locals..(num_locals+num_globals) are global to all threads
     * registers dsp_mask are useable only on DSP capable threads
     */
    uint8_t num_locals;     /* number of local registers */
    uint8_t num_globals;    /* number of global registers */
    uint32_t dsp_mask;      /* DSP only local registers */
} meta_unit_info[16] = {
    [META_UNIT_CT]       = { 31,  1, .dsp_mask = 0x0000d7e2},
    [META_UNIT_D0]       = { 16, 16, .dsp_mask = 0x0000ff00},
    [META_UNIT_D1]       = { 16, 16, .dsp_mask = 0x0000ff00},
    [META_UNIT_A0]       = {  8,  8, .dsp_mask = 0x000000f0},
    [META_UNIT_A1]       = {  8,  8, .dsp_mask = 0x000000f0},
    [META_UNIT_PC]       = {  2 },
    [META_UNIT_RA]       = {  32 },
    [META_UNIT_TR]       = {  8 },
    [META_UNIT_TT]       = {  4,  1 },
    [META_UNIT_FX]       = { 16 },
};

/* return number of registers in a unit */
static inline unsigned int meta_unit_registers(MetaUnit unit)
{
    if (unlikely(unit >= META_UNIT_MAX)) {
        return 0;
    }
    return meta_unit_info[unit].num_locals + meta_unit_info[unit].num_globals;
}

/* bitwise OR'able register information */
typedef enum MetaRegisterType {
    META_REG_VALID  = 1, /* a valid register */
    META_REG_GLOBAL = 2, /* global to all threads */
    META_REG_DSP    = 4, /* only available on DSP threads */
    META_REG_FX     = 8, /* only available on cores with FPU */
} MetaRegisterType;

/* get a bit vector describing a register */
static MetaRegisterType meta_unit_regtype(MetaUnit unit, int reg)
{
    MetaRegisterType ret;
    if (unlikely(unit >= META_UNIT_MAX || reg >= meta_unit_registers(unit))) {
        return 0;
    }

    ret = META_REG_VALID;
    if (reg >= meta_unit_info[unit].num_locals) {
        ret |= META_REG_GLOBAL;
    }
    if (meta_unit_info[unit].dsp_mask & (1 << reg)) {
        ret |= META_REG_DSP;
    }
    if (unit == META_UNIT_FX) {
        ret |= META_REG_FX;
    }
    return ret;
}

/* return 1 if the register is global, 0 otherwise */
static int meta_reg_global(MetaUnit unit, int reg)
{
    if (unlikely(unit >= META_UNIT_MAX || reg >= meta_unit_registers(unit))) {
        return 0;
    }
    return reg >= meta_unit_info[unit].num_locals;
}

static const char *meta_unit_names[META_UNIT_MAX] = {
    [META_UNIT_CT]     = "CT",
    [META_UNIT_D0]     = "D0",
    [META_UNIT_D1]     = "D1",
    [META_UNIT_A0]     = "A0",
    [META_UNIT_A1]     = "A1",
    [META_UNIT_PC]     = "PC",
    [META_UNIT_RA]     = "RA",
    [META_UNIT_TR]     = "TR",
    [META_UNIT_TT]     = "TT",
    [META_UNIT_FX]     = "FX",
};

static const char *meta_creg_names[META_CT_MAX] = {
    "TXENABLE",     "TXMODE",       "TXSTATUS",     "TXRPT",
    "TXTIMER",      "TXL1START",    "TXL1END",      "TXL1COUNT",
    "TXL2START",    "TXL2END",      "TXL2COUNT",    "TXBPOBITS",
    "TXMRSIZE",     "TXTIMERI",     "TXDRCTRL",     "TXDRSIZE",
    "TXCATCH0",     "TXCATCH1",     "TXCATCH2",     "TXCATCH3",
    "TXDEFR",       "CT.21",        "TXCLKCTRL",    "TXINTERN0",
    "TXAMAREG0",    "TXAMAREG1",    "TXAMAREG2",    "TXAMAREG3",
    "TXDIVTIME",    "TXPRIVEXT",    "TXTACTCYC",    "TXIDLECYC",
};

static const char *meta_dreg_names[2][32] = {
    { "D0Re0", "D0Ar6", "D0Ar4", "D0Ar2", "D0FrT", "D0.5",  "D0.6",  "D0.7",
      "D0.8",  "D0.9",  "D0.10", "D0.11", "D0.12", "D0.13", "D0.14", "D0.15",
      "D0.16", "D0.17", "D0.18", "D0.19", "D0.20", "D0.21", "D0.22", "D0.23",
      "D0.24", "D0.25", "D0.26", "D0.27", "D0.28", "D0.29", "D0.30", "D0.31", },
    { "D1Re0", "D1Ar5", "D1Ar3", "D1Ar1", "D1RtP", "D1.5",  "D1.6",  "D1.7",
      "D1.8",  "D1.9",  "D1.10", "D1.11", "D1.12", "D1.13", "D1.14", "D1.15",
      "D1.16", "D1.17", "D1.18", "D1.19", "D1.20", "D1.21", "D1.22", "D1.23",
      "D1.24", "D1.25", "D1.26", "D1.27", "D1.28", "D1.29", "D1.30", "D1.31", },
};

static const char *meta_areg_names[2][16] = {
    { "A0StP", "A0FrP", "A0.2",  "A0.3",  "A0.4",  "A0.5",  "A0.6",  "A0.7",
      "A0.8",  "A0.9",  "A0.10", "A0.11", "A0.12", "A0.13", "A0.14", "A0.15", },
    { "A1GbP", "A1LbP", "A1.2",  "A1.3",  "A1.4",  "A1.5",  "A1.6",  "A1.7",
      "A1.8",  "A1.9",  "A1.10", "A1.11", "A1.12", "A1.13", "A1.14", "A1.15", },
};

static const char *meta_accreg_names[2][4] = {
    { "AC0.0", "AC0.1", "AC0.2", "AC0.3" },
    { "AC1.0", "AC1.1", "AC1.2", "AC1.3" },
};

static const char *meta_pcreg_names[META_PC_MAX] = {
    "PC",   "PCX",
};

static const char *meta_port_names[2][META_RA_MAX] = {
    {
        [META_RA]       = "RD",      [META_RAPF]     = "?",

        [META_RAM8X32]  = "?",       [META_RAM8X]    = "?",
        [META_RABZ]     = "?",       [META_RAWZ]     = "?",
        [META_RADZ]     = "?",

        [META_RABX]     = "?",       [META_RAWX]     = "?",
        [META_RADX]     = "?",       [META_RAM16X]   = "?",
    },
    {
        [META_RA]       = "RA",      [META_RAPF]     = "RAPF",

        [META_RAM8X32]  = "RAM8X32", [META_RAM8X]    = "RAM8X",
        [META_RABZ]     = "RABZ",    [META_RAWZ]     = "RAWZ",
        [META_RADZ]     = "RADZ",

        [META_RABX]     = "RABX",    [META_RAWX]     = "RAWX",
        [META_RADX]     = "RADX",    [META_RAM16X]   = "RAM16X",
    },
};

static const char *meta_treg_names[META_TR_MAX] = {
    "TXSTAT",       "TXMASK",       "TXSTATI",      "TXMASKI",
    "TXPOLL",       "TXGPIOI",      "TXPOLLI",      "TXGPIOO",
};

static const char *meta_ttreg_names[META_TT_MAX] = {
    "TTEXEC",       "TTCTRL",       "TTMARK",       "TTREC",
    "GTEXEC",
};

static const char *meta_fxreg_names[16] = {
    "FX.0",  "FX.1",  "FX.2",  "FX.3",  "FX.4",  "FX.5",  "FX.6",  "FX.7",
    "FX.8",  "FX.9",  "FX.10", "FX.11", "FX.12", "FX.13", "FX.14", "FX.15",
};

static const char *meta_codebcount_names[4] = {
    "CODEB0Count", "CODEB1Count", "CODEB2Count", "CODEB3Count",
};

static const char *meta_lnkaddr_names[META_T2T_NR] = {
#if META_MAX_THREADS > 3
    [META_T2T(-3, 0)] = "_LNKADDR[-3]",
#endif
#if META_MAX_THREADS > 2
    [META_T2T(-2, 0)] = "_LNKADDR[-2]",
#endif
#if META_MAX_THREADS > 1
    [META_T2T(-1, 0)] = "_LNKADDR[-1]",
#endif
    [META_T2T(0, 0)] = "_LNKADDR",
#if META_MAX_THREADS > 1
    [META_T2T(1, 0)] = "_LNKADDR[+1]",
#endif
#if META_MAX_THREADS > 2
    [META_T2T(2, 0)] = "_LNKADDR[+2]",
#endif
#if META_MAX_THREADS > 3
    [META_T2T(3, 0)] = "_LNKADDR[+3]",
#endif
};

static const char **meta_reg_names[] = {
    [META_UNIT_CT]    = meta_creg_names,
    [META_UNIT_D0]    = meta_dreg_names[0],
    [META_UNIT_D1]    = meta_dreg_names[1],
    [META_UNIT_A0]    = meta_areg_names[0],
    [META_UNIT_A1]    = meta_areg_names[1],
    [META_UNIT_PC]    = meta_pcreg_names,
    [META_UNIT_RA]    = meta_port_names[1],
    [META_UNIT_TR]    = meta_treg_names,
    [META_UNIT_TT]    = meta_ttreg_names,
    [META_UNIT_FX]    = meta_fxreg_names,
};

static const char *meta_dspram_dreg_names[2][2][16] = {
    {
        {
            "[D0AR.0]", "[D0AR.0++]", "[D0AR.0+D0ARI.0++]", "[D0AR.0+D0ARI.1++]",
            "[D0AR.1]", "[D0AR.1++]", "[D0AR.1+D0ARI.0++]", "[D0AR.1+D0ARI.1++]",
            "[D0BR.0]", "[D0BR.0++]", "[D0BR.0+D0BRI.0++]", "[D0BR.0+D0BRI.1++]",
            "[D0BR.1]", "[D0BR.1++]", "[D0BR.1+D0BRI.0++]", "[D0BR.1+D0BRI.1++]",
        },
        {
            "[D1AR.0]", "[D1AR.0++]", "[D1AR.0+D1ARI.0++]", "[D1AR.0+D1ARI.1++]",
            "[D1AR.1]", "[D1AR.1++]", "[D1AR.1+D1ARI.0++]", "[D1AR.1+D1ARI.1++]",
            "[D1BR.0]", "[D1BR.0++]", "[D1BR.0+D1BRI.0++]", "[D1BR.0+D1BRI.1++]",
            "[D1BR.1]", "[D1BR.1++]", "[D1BR.1+D1BRI.0++]", "[D1BR.1+D1BRI.1++]",
        },
    },
    {
        {
            "[D0AW.0]", "[D0AW.0++]", "[D0AW.0+D0AWI.0++]", "[D0AW.0+D0AWI.1++]",
            "[D0AW.1]", "[D0AW.1++]", "[D0AW.1+D0AWI.0++]", "[D0AW.1+D0AWI.1++]",
            "[D0BW.0]", "[D0BW.0++]", "[D0BW.0+D0BWI.0++]", "[D0BW.0+D0BWI.1++]",
            "[D0BW.1]", "[D0BW.1++]", "[D0BW.1+D0BWI.0++]", "[D0BW.1+D0BWI.1++]",
        },
        {
            "[D1AW.0]", "[D1AW.0++]", "[D1AW.0+D1AWI.0++]", "[D1AW.0+D1AWI.1++]",
            "[D1AW.1]", "[D1AW.1++]", "[D1AW.1+D1AWI.0++]", "[D1AW.1+D1AWI.1++]",
            "[D1BW.0]", "[D1BW.0++]", "[D1BW.0+D1BWI.0++]", "[D1BW.0+D1BWI.1++]",
            "[D1BW.1]", "[D1BW.1++]", "[D1BW.1+D1BWI.0++]", "[D1BW.1+D1BWI.1++]",
        },
    },
};

static const char *meta_dspram_rp_names[2][2][2] = {
    { { "D0AR.0", "D0AR.1" }, { "D0BR.0", "D0BR.1" } },
    { { "D1AR.0", "D1AR.1" }, { "D1BR.0", "D1BR.1" } },
};

static const char *meta_dspram_rpi_names[2][2][2] = {
    { { "D0ARI.0", "D0ARI.1" }, { "D0BRI.0", "D0BRI.1" } },
    { { "D1ARI.0", "D1ARI.1" }, { "D1BRI.0", "D1BRI.1" } },
};

static const char *meta_dspram_wp_names[2][2][2] = {
    { { "D0AW.0", "D0AW.1" }, { "D0BW.0", "D0BW.1" } },
    { { "D1AW.0", "D1AW.1" }, { "D1BW.0", "D1BW.1" } },
};

static const char *meta_dspram_wpi_names[2][2][2] = {
    { { "D0AWI.0", "D0AWI.1" }, { "D0BWI.0", "D0BWI.1" } },
    { { "D1AWI.0", "D1AWI.1" }, { "D1BWI.0", "D1BWI.1" } },
};

static const char *meta_dspram_fetched_names[2][2][2] = {
    {
        { "D0AR.0:data", "D0AR.1:data" },
        { "D0BR.0:data", "D0BR.1:data" },
    },
    {
        { "D1AR.0:data", "D1AR.1:data" },
        { "D1BR.0:data", "D1BR.1:data" },
    },
};

static const char *meta_dspram_off_and_names[2] = {
    "D0OffsAND", "D1OffsAND",
};

static const char *meta_dspram_off_or_names[2] = {
    "D0OffsOR", "D1OffsOR",
};

static const char **meta_dspram_reg_names[2][2] = {
    {
        [0] = meta_dspram_dreg_names[0][0],
        [1] = meta_dspram_dreg_names[0][1],
    },
    {
        [0] = meta_dspram_dreg_names[1][0],
        [1] = meta_dspram_dreg_names[1][1],
    },
};

static const char *meta_cf_names[5] = {
    [META_CF_C] = "CF_C",
    [META_CF_V] = "CF_V",
    [META_CF_N] = "CF_N",
    [META_CF_Z] = "CF_Z",
    [META_CF_SCC] = "SCC",
};

/* bitmask of lengths that can be GET/SET for each unit */
static const unsigned char meta_unit_getset_lens[16] = {
    [META_UNIT_CT]    =         4,
    [META_UNIT_D0]    = 1 | 2 | 4 | 8,
    [META_UNIT_D1]    = 1 | 2 | 4 | 8,
    [META_UNIT_A0]    = 1 | 2 | 4 | 8,
    [META_UNIT_A1]    = 1 | 2 | 4 | 8,
    [META_UNIT_PC]    =         4,
    [META_UNIT_RA]    = 1 | 2 | 4 | 8,
    [META_UNIT_TR]    =         4,
    [META_UNIT_TT]    =         4,
    [META_UNIT_FX]    = 1 | 2 | 4 | 8,
};

static const char *meta_srcdst_name(MetaUnit unit, int regno,
                                    bool dsp, bool write)
{
    if (unlikely(unit >= META_UNIT_MAX)) {
        return NULL;
    }
    if (unit == META_UNIT_D0 || unit == META_UNIT_D1) {
        if (dsp && (regno & 0x10)) {
            /* DSP RAM */
            return meta_dspram_reg_names[!!write][unit - META_UNIT_D0][regno & 0xf];
        }
    }
    if (unlikely(regno >= meta_unit_registers(unit))) {
        return NULL;
    }
    return meta_reg_names[unit][regno];
}

/* TODO: scrap meta_reg_name */
static const char *meta_reg_name(MetaUnit unit, int regno)
{
    return meta_srcdst_name(unit, regno, false, false);
}


void meta_cpu_dump_state(CPUState *cs, FILE *f, int flags)
{
    CPUArchState *env = cpu_env(cs);
    int i;
    qemu_fprintf(f, "PC\t%08x\tPCX\t%08x\n",
                env->pc[0], env->pc[1]);

    if (meta_dsp_supported(env)) {
        const char *duarithmode_names[] = {
            "16x16",
            "Split-16",
            "32x32 High",
            "32x32 Low",
        };
        const char *aumode_names[] = {
            "",
            "RESERVED",
            "RESERVED",
            "modulo",
            "bit-reversed-8b",
            "bit-reversed-16b",
            "bit-reversed-32b",
            "bit-reversed-64b",
        };
        uint32_t mode = env->cregs[META_TXMODE];
        bool prodshift = !!(mode & META_TXMODE_DUPRODSHIFT_MASK);
        bool rounding = !!(mode & META_TXMODE_DUROUNDING_MASK);
        bool saturation = !!(mode & META_TXMODE_DUSATURATION_MASK);
        bool accsat = !!(mode & META_TXMODE_DUACCSAT_MASK);
        bool m8 = !!(mode & META_TXMODE_M8_MASK);
        uint8_t au0mode = (mode >> META_TXMODE_AU0ADDRMODE_SHIFT) &
            META_TXMODE_AU0ADDRMODE_MASK;
        uint8_t au1mode = (mode >> META_TXMODE_AU1ADDRMODE_SHIFT) &
            META_TXMODE_AU1ADDRMODE_MASK;
        bool dsprradix = !!(mode & META_TXMODE_DSPRRADIX_MASK);

        qemu_fprintf(f, "TXMODE\t%08x (%s%s%s%s%s%s%s%s%s%s%s)\n",
                    mode,
                    duarithmode_names[mode & 0x3],
                    prodshift ? ", ProdShift" : "",
                    rounding ? ", Rounding" : "",
                    saturation ? ", Saturation" : "",
                    accsat ? ", AccSat" : "",
                    m8 ? ", M8" : "",
                    au0mode ? ", AU0Mode=" : "",
                    aumode_names[au0mode],
                    au1mode ? ", AU1Mode=" : "",
                    aumode_names[au1mode],
                    dsprradix ? ", DSPRRadix" : "");
    }

    if (env->readport_count) {
        qemu_fprintf(f, "RPCount\t%d\n", env->readport_count);
    }

    qemu_fprintf(f, "TXSTATUS %c%c%c%c\t\tTXRPT\t%d\n",
                env->cf[META_CF_Z] ? 'Z' : 'z',
                env->cf[META_CF_N] ? 'N' : 'n',
                env->cf[META_CF_V] ? 'O' : 'o',
                env->cf[META_CF_C] ? 'C' : 'c',
                env->cregs[META_TXRPT]);
    qemu_fprintf(f, "TXTACTCYC %08x\n",
                env->cregs[META_TXTACTCYC]);
    qemu_fprintf(f, "TXSTAT\t%08x\tTXSTATI\t%08x\n",
                env->tregs[META_TXSTAT],
                env->tregs[META_TXSTATI]);
    qemu_fprintf(f, "TXMASK\t%08x\tTXMASKI\t%08x\n",
                env->tregs[META_TXMASK],
                env->tregs[META_TXMASKI]);

    for (i = 0; i < 4; i++) {
        qemu_fprintf(f, "%s\t%08x\t%s\t%08x\n",
                    meta_reg_name(META_UNIT_A0, i), env->aregs[0][i],
                    meta_reg_name(META_UNIT_A1, i), env->aregs[1][i]);
    }

    for (i = 0; i < 8; i++) {
        qemu_fprintf(f, "%s\t%08x\t%s\t%08x\n",
                    meta_reg_name(META_UNIT_D0, i), env->dregs[0][i],
                    meta_reg_name(META_UNIT_D1, i), env->dregs[1][i]);
    }

    if (meta_dsp_supported(env)) {
        qemu_fprintf(f, "AC0.0\t%010llx\tAC1.0\t%010llx\n",
                    env->accregs[0][0] & 0xffffffffffllu,
                    env->accregs[1][0] & 0xffffffffffllu);
    }

    if (meta_fpu_supported(env)) {
        for (i = 0; i < 16; i += 2) {
            qemu_fprintf(f, "FX.%d\t%08x\tFX.%d\t%08x\n",
                        i, env->fxregs[i],
                        i + 1, env->fxregs[i + 1]);
        }
    }
}

/*
 * Generate restoration code to update PC and counters as we're going to stop
 * on this instruction (important: on, not after, so instr_count is
 * decremented)
 */
static void meta_gen_restore_state(DisasContext *dc)
{
    /* put PC at current instruction */
    tcg_gen_movi_i32(cpu_pc[META_PC], dc->pc);

    /* update translation-time state */
    gen_update_state(dc);

    /* update counters, excluding current instruction */
    --dc->instr_count;
    gen_counters(dc);
    ++dc->instr_count;
}

#if !defined(CONFIG_USER_ONLY)
/* Return to the main loop (without stopping for single step) */
static void meta_gen_return(DisasContext *dc)
{
    tcg_gen_exit_tb(NULL, 0);
    dc->jmp = JMP_INDIRECT;
}

/* Generate a block condition */
static void meta_gen_block_condition(DisasContext *dc, MetaBlock block)
{
    TCGv tmp;

    meta_gen_restore_state(dc);

    /* use helper to block on the condition */
    tmp = meta_const_i32(block);
    gen_helper_block(tcg_env, tmp);

    meta_gen_return(dc);
}
#endif /* !CONFIG_USER_ONLY */

/* Generate a halt or a memory fault */
static void meta_gen_halt_fault(DisasContext *dc, MetaHReason hreason,
                                MetaFReason freason)
{
#if !defined(CONFIG_USER_ONLY)
    TCGv tmp_signum;
#endif

    meta_gen_restore_state(dc);

#if defined(CONFIG_USER_ONLY)
    t_gen_raise_exception(EXCP_FAULT_BASE + META_SIGNUM(freason, hreason));
#else
    /* use helper to trigger the halt */
    tmp_signum = meta_const_i32(META_SIGNUM(freason, hreason));
    gen_helper_halt_signum(tcg_env, tmp_signum);
    /* halt_signum will exit the cpu loop */
#endif
}

/* Generate a halt */
static void meta_gen_halt(DisasContext *dc, MetaHReason hreason)
{
    meta_gen_halt_fault(dc, hreason, META_FREASON_NOERROR);
}

/* Generate an unknown instruction halt */
static void meta_illegal(DisasContext *dc, const char *description)
{
    DBGLOG("T%d Generating unknown instruction halt on %08x at %08x (%s)\n",
           dc->env->thread_num, dc->instr, dc->pc, description);
    LOG_DIS("\tDWORD\t0x%08x\n", dc->instr);
    meta_gen_halt(dc, META_HREASON_UNKNOWN);
    dc->jmp = JMP_INDIRECT;
}

/* Generate a hard breakpoint */
static void meta_breakpoint(DisasContext *dc)
{
    if (dc->last_pc != dc->pc) {
        LOG_DIS("/*HWBP*/ ");
    }
    meta_gen_halt_fault(dc, META_HREASON_UNKNOWN, META_FREASON_PROTECT);
}

/* Generate a priv halt */
static void meta_priv(DisasContext *dc, const char *description)
{
    DBGLOG("T%d Generating potential priv halt on %08x at %08x (%s)\n",
           dc->env->thread_num, dc->instr, dc->pc, description);
    meta_gen_halt(dc, META_HREASON_PRIV);
}

static void meta_unimplemented(DisasContext *dc, const char *description)
{
#if DEBUG_LEVEL >= 3
    cpu_abort(env_cpu(dc->env), "T%d unimplemented instruction %08x at %08x (%s)",
              dc->env->thread_num, dc->instr, dc->pc, description);
#else
    meta_illegal(dc, description);
#endif
}

/* Translation-time breakpoint check and code gen to break */
/*
 * Hardware code breakpoints (CODEBxADDR/CODEBxCTRL).  gdb breakpoints are
 * handled by the generic code before the TB runs.  Changing a CODEB register
 * flushes all TBs, so the core registers can be read at translation time.
 */
static bool meta_codeb_hit(DisasContext *dc, int i)
{
    uint32_t ctrl = dc->core->codebctrl[i];
    uint32_t mask = 0x3 | (ctrl & META_CODEBXCTRL_MASK_MASK);

    if (!(ctrl & META_CODEBXCTRL_EN_MASK)) {
        return false;
    }
    if ((ctrl & META_CODEBXCTRL_TONLY_MASK) &&
        (ctrl & META_CODEBXCTRL_THREAD_MASK) != (dc->tbflags & META_TBFLAG_TX)) {
        return false;
    }
    return (dc->pc & ~mask) == (dc->core->codebaddr[i] & ~mask);
}

static int handle_breakpoint(DisasContext *dc)
{
    int i;
    int count = 0;
    TCGLabel *else_label = NULL, *end_label;
    TCGv codebcount, temp;

    /*
     * If there is only one matching breakpoint (most common), we generate a
     * single if-then-else block. If there are multiple we set temp to
     * whether a breakpoint is hit using an if-then-else for each, and
     * handle it after all the counters have been incremented.
     */
    temp = tcg_temp_new_i32();
    for (i = 0; i < 4; ++i) {
        if (!meta_codeb_hit(dc, i)) {
            continue;
        }
        if (count == 1) {
            /*     temp = 0; */
            tcg_gen_movi_i32(temp, 0);

            /* } else { */
            end_label = gen_new_label();
            tcg_gen_br(end_label);
            gen_set_label(else_label);
            /*     temp = 1; */
            tcg_gen_movi_i32(temp, 1);
            /* } */
            gen_set_label(end_label);
        }
        codebcount = cpu_codebcount[dc->tbflags & META_TBFLAG_TX][i];

        /* if (counter < 255) { */
        else_label = gen_new_label();
        tcg_gen_brcondi_i32(TCG_COND_EQ, codebcount, 255, else_label);
        /*     ++counter */
        tcg_gen_addi_i32(codebcount, codebcount, 1);
        /* don't generate the else block yet the first time around,
         * as if it's the only breakpoint we can put the break
         * directly in the else
         */
        if (count) {
            /* } else { */
            end_label = gen_new_label();
            tcg_gen_br(end_label);
            gen_set_label(else_label);
            /*     temp = 1; */
            tcg_gen_movi_i32(temp, 1);
            /* } */
            gen_set_label(end_label);
        }
        ++count;
    }
    if (count) {
        end_label = gen_new_label();
        if (count == 1) {
            /* } else { */
            tcg_gen_br(end_label);
            gen_set_label(else_label);
        } else {
            /* if (temp) { */
            tcg_gen_brcondi_i32(TCG_COND_EQ, temp, 0, end_label);
        }
        /*     break */
        ++dc->instr_count;
        meta_breakpoint(dc);
        --dc->instr_count;
        /* } */
        gen_set_label(end_label);
    }
    /* CPU breakpoints are conditional so we should still continue */
    return 0;
}

/* Generate a priv halt and return 0 if don't have privilege */
static int meta_privd(DisasContext *dc, const char *description)
{
    if (!(dc->tbflags & META_TBFLAG_PSTAT)) {
        meta_priv(dc, description);
        return 1;
    }
    return 0;
}

/* Generate a priv halt if priv is required for this instruction */
static void meta_priv_check(DisasContext *dc,
                            TCGv privreg, unsigned int mask,
                            const char *description)
{
    if (!(dc->tbflags & META_TBFLAG_PSTAT)) {
        TCGv temp = tcg_temp_new_i32();
        TCGLabel *l1 = gen_new_label();
        tcg_gen_andi_i32(temp, privreg, mask);
        tcg_gen_brcondi_i32(TCG_COND_EQ, temp, 0, l1);
        {
            /* generate priv halt and exit */
            meta_priv(dc, description);
        }
        gen_set_label(l1);
    }
}

/* get a TCGv register from a unit and register number, return 0 on success */
static int meta_get_reg(DisasContext *dc, MetaUnit unit, int reg, TCGv *tcgv)
{
    const struct MetaUnitInfo *info = &meta_unit_info[unit];
    MetaRegisterType type = meta_unit_regtype(unit, reg);

    if (unlikely(!(type & META_REG_VALID))) {
        meta_illegal(dc, "invalid register");
        DBGLOG("unit=%d, reg=%d\n", unit, reg);
        return 1;
    }

    if (!meta_fpu_supported(dc->env) && (type & META_REG_FX)) {
        meta_illegal(dc, "FX register on core without FPU");
        return 1;
    }

    if (type & META_REG_GLOBAL) {
        int t = dc->tbflags & META_TBFLAG_TX;
        reg -= info->num_locals;
        switch (unit) {
        case META_UNIT_CT:
            *tcgv = cpu_gcregs[t][reg];
            return 0;
        case META_UNIT_D0:
            *tcgv = cpu_gdregs[t][0][reg];
            return 0;
        case META_UNIT_D1:
            *tcgv = cpu_gdregs[t][1][reg];
            return 0;
        case META_UNIT_A0:
            *tcgv = cpu_garegs[t][0][reg];
            return 0;
        case META_UNIT_A1:
            *tcgv = cpu_garegs[t][1][reg];
            return 0;
        case META_UNIT_TT:
            *tcgv = cpu_gttregs[t][reg];
            return 0;
        default:
            return 1;
        };
    } else {
        switch (unit) {
        case META_UNIT_CT:
            *tcgv = cpu_cregs[reg];
            return 0;
        case META_UNIT_D0:
            *tcgv = cpu_dregs[0][reg];
            return 0;
        case META_UNIT_D1:
            *tcgv = cpu_dregs[1][reg];
            return 0;
        case META_UNIT_A0:
            *tcgv = cpu_aregs[0][reg];
            return 0;
        case META_UNIT_A1:
            *tcgv = cpu_aregs[1][reg];
            return 0;
        case META_UNIT_PC:
            *tcgv = cpu_pc[reg];
            return 0;
        case META_UNIT_RA:
            return 0;
        case META_UNIT_TR:
            *tcgv = cpu_tregs[reg];
            return 0;
        case META_UNIT_TT:
            *tcgv = cpu_ttregs[reg];
            return 0;
        case META_UNIT_FX:
            *tcgv = cpu_fxregs[reg];
            return 0;
        default:
            return 1;
        };
    }
}

/*
 * get a raw pointer to a register from a unit and register number, return NULL
 * on failure
 */
static uint32_t *meta_raw_get_reg(CPUMETAState *env, MetaUnit unit, int reg)
{
    const struct MetaUnitInfo *info = &meta_unit_info[unit];
    MetaRegisterType type = meta_unit_regtype(unit, reg);

    if (unlikely(!(type & META_REG_VALID))) {
        return NULL;
    }
    if (unlikely(env->tcaps != META_TCAPS_DSP && (type & META_REG_DSP))) {
        return NULL;
    }

    if (!meta_fpu_supported(env) && (type & META_REG_FX)) {
        return NULL;
    }

    if (type & META_REG_GLOBAL) {
        reg -= info->num_locals;
        switch (unit) {
        case META_UNIT_CT:    return &env->global->cregs[reg];
        case META_UNIT_D0:    return &env->global->dregs[0][reg];
        case META_UNIT_D1:    return &env->global->dregs[1][reg];
        case META_UNIT_A0:    return &env->global->aregs[0][reg];
        case META_UNIT_A1:    return &env->global->aregs[1][reg];
        case META_UNIT_TT:    return &env->global->ttregs[reg];
        default:              return NULL;
        };
    } else {
        switch (unit) {
        case META_UNIT_CT:    return &env->cregs[reg];
        case META_UNIT_D0:    return &env->dregs[0][reg];
        case META_UNIT_D1:    return &env->dregs[1][reg];
        case META_UNIT_A0:    return &env->aregs[0][reg];
        case META_UNIT_A1:    return &env->aregs[1][reg];
        case META_UNIT_PC:    return &env->pc[reg];
        case META_UNIT_TR:    return &env->tregs[reg];
        case META_UNIT_TT:    return &env->ttregs[reg];
        case META_UNIT_FX:    return &env->fxregs[reg];
        default:              return NULL;
        };
    }
}

uint32_t meta_minim_decode_insn(uint16_t **minim_ptr, size_t max_words)
{
    uint16_t insn_core, insn_ext = 0, insn_long = 0;
    uint16_t *minim_code = *minim_ptr;
    uint32_t enc_core, enc_ext, enc, se_mask, se_flag, se_lower;
    uint32_t ret = -1;
    uint32_t copy;
    size_t insn_words = 1;
    minim_insn_desc_t *desc;
    int bit;

    if (max_words >= 1) {
        insn_core = minim_code[0];
        insn_words = meta_minim_decode_size(insn_core);
    }

    if (insn_words > 1) {
        if ((insn_core & 0xf000) == 0xb000) {
            /* long instruction */
            assert(insn_words == 2);
            insn_long = insn_core;
            insn_core = minim_code[1];
            insn_ext = 0;
        } else if ((insn_core & 0xc000) == 0xc000) {
            /* extended instruction */
            assert(insn_words == 2);
            insn_ext = insn_core;
            insn_core = minim_code[1];
            insn_long = 0;
        } else {
            assert(0);
        }
    }

    *minim_ptr += insn_words;

    if (insn_words > max_words) {
        fprintf(stderr, "Required %d words, only have %d\n",
                (int)insn_words, (int)max_words);
        return ret;
    }

    /* find description table entry */
    for (desc = minim_table; desc->prefix != 0xffff; desc++) {
        if ((desc->dec_mask_core & insn_core) != desc->dec_val_core) {
            continue;
        }

        if (insn_words == 1) {
            /* core instruction matched */
            break;
        }

        if ((desc->dec_mask_ext & insn_ext) == desc->dec_val_ext) {
            /* extended instruction matched */
            insn_ext &= ~0xc000;
            break;
        }

        if ((desc->dec_mask_long & insn_long) == desc->dec_val_long) {
            /* long instruction matched */
            insn_ext = insn_long & ~0xf000;
            break;
        }
    }

    if (!desc || (desc->prefix == 0xffff)) {
        fprintf(stderr, "Unmatched instruction 0x%04x\n", insn_core);
        return ret;
    }

    enc_core = desc->enc_core;
    enc_ext = desc->enc_ext;

    if (insn_words == 1) {
        ret = desc->core_mask & desc->core_val & ~enc_core;
        enc_ext = 0;
    } else {
        ret = desc->ext_mask & desc->ext_val & ~(enc_core | enc_ext);
    }

    for (bit = 0, enc = enc_core | enc_ext; enc;
         enc >>= 1, enc_core >>= 1, enc_ext >>= 1, bit++) {
        if (enc_core & 0x1) {
            ret |= ((uint32_t)(insn_core & 0x1)) << bit;
            insn_core >>= 1;
            continue;
        }

        if (enc_ext & 0x1) {
            ret |= ((uint32_t)(insn_ext & 0x1)) << bit;
            insn_ext >>= 1;
            continue;
        }
    }

    if ((insn_words == 1) && desc->se_bits) {
        /* sign extend constants */

        se_mask = (1UL << desc->se_bits) - 1;
        se_flag = (desc->se_bit < 0) ? ~0 : (ret & (1UL << desc->se_bit));
        se_lower = ret & (1UL << (desc->se_msb - desc->se_bits));

        if (se_flag && se_lower) {
            ret |= (se_mask << ((desc->se_msb - desc->se_bits) + 1));

            /* set se flag in instruction */
            if (se_flag != ~0) {
                ret |= se_flag;
            }
        }
    }

    if (desc->copy_bits) {
        copy = (ret >> desc->copy_src) & ((1 << desc->copy_bits) - 1);
        ret |= copy << desc->copy_dst;
    }

    return ret;
}

typedef struct {
    int old_is_jmp;
    TCGLabel *label;
} MetaSafeLabel;
#define DEFINE_SAFE_LABEL(X) MetaSafeLabel X = { DISAS_NEXT, NULL }

/* Create a new safe label, but can be called multiple times (or no times) */
static inline void meta_init_safe_label(DisasContext *dc, MetaSafeLabel *label)
{
    if (label->label == NULL) {
        label->old_is_jmp = dc->is_jmp;
        label->label = gen_new_label();
    }
}

/* Set a safe label, handling any jump state we might be in */
static void meta_set_safe_label(DisasContext *dc, int is_end,
                                MetaSafeLabel *label)
{
    /*
     * If it's an end label, we can safely assume there is nothing interesting
     * that needs to be done after this point. We can therefore skip handling
     * the else condition, and just ensure that any jumps in the condition are
     * handled before the label as if we'd finished the block.  The else case
     * (after the label) can then finish more efficiently, such as allowing
     * chaining to the next block.
     *
     * If it isn't an end label, just make the jump dynamic and handle the else
     * case for now. Ideally this needs to hook in some code at the end to
     * handle the jumping efficiently and dynamically.
     */
    if (label->label != NULL) {
        int else_update = 0;
        TCGLabel *else_label = NULL;
        /* ensure that any jump in the conditional part has exited the tb */
        switch (dc->is_jmp) {
        case DISAS_UPDATE:
            /* more to do: don't exit the tb or update PC */
            if (!is_end) {
                break;
            }
            if (label->old_is_jmp != DISAS_JUMP) {
                tcg_gen_movi_tl(cpu_pc[META_PC], dc->next_pc);
            }

            /* fall through */
        case DISAS_JUMP:
            /* if its a jump whether the branch is taken or not, do nothing */
            if (label->old_is_jmp == DISAS_JUMP) {
                break;
            }
            /* more to do: don't exit the tb but do update PC in else case */
            if (!is_end) {
                else_update = 1;
                break;
            }
            /* exit tb immediately */
            gen_update_state(dc);
            gen_counters(dc);
            /* stop thread if hardware single stepping */
            if (dc->tbflags & META_TBFLAG_STEP) {
                gen_single_step();
            }
            tcg_gen_exit_tb(NULL, 0);

            /* fall through */
        case DISAS_TB_JUMP:
            /*
             * stop decoding, and continue to next block if condition did not
             * match
             */
            dc->is_jmp = DISAS_NEXT;
            dc->jmp = JMP_INDIRECT;
            break;
        }
        if (else_update) {
            else_label = gen_new_label();
            tcg_gen_br(else_label);
        }
        gen_set_label(label->label);
        if (else_update) {
            tcg_gen_movi_i32(cpu_pc[META_PC], dc->next_pc);
            gen_set_label(else_label);
        }
    }
}

/* special register handling */

typedef void MetaGenRegAccess(DisasContext *dc, MetaUnit unit, int id,
                              TCGv reg);
typedef void MetaGenRegLAccess(DisasContext *dc, MetaUnit ud, int rd,
                               MetaUnit us, int rs, TCGv dl, TCGv dh);
typedef void MetaGenRegWriteImm(DisasContext *dc, MetaUnit unit, int id,
                                uint32_t val);
typedef uint32_t MetaRegRead(CPUMETAState *env);
typedef void MetaRegWrite(CPUMETAState *env, uint32_t val);
typedef void MetaRegWritel(CPUMETAState *env, uint32_t lo, uint32_t hi);
typedef uint64_t MetaRegRead64(CPUMETAState *env);

/* Generated code reaches the register handlers through these. */
uint32_t HELPER(reg_read)(CPUMETAState *env, void *fn)
{
    return ((MetaRegRead *)fn)(env);
}

uint64_t HELPER(reg_read64)(CPUMETAState *env, void *fn)
{
    return ((MetaRegRead64 *)fn)(env);
}

void HELPER(reg_write)(CPUMETAState *env, void *fn, uint32_t val)
{
    ((MetaRegWrite *)fn)(env, val);
}

void HELPER(reg_writel)(CPUMETAState *env, void *fn, uint32_t lo, uint32_t hi)
{
    ((MetaRegWritel *)fn)(env, lo, hi);
}

/* MetaRegHandler::priv encoding */
#define META_REG_PRIV_WHEN_MASK         0x00f
#define META_REG_PRIV_WHEN_NONE             0x0
#define META_REG_PRIV_WHEN_ALWAYS           0x1
#define META_REG_PRIV_WHEN_TXPRIVEXT        0x2
#define META_REG_PRIV_BIT_SHIFT         4
#define META_REG_PRIV_BIT_MASK          (0x1f << META_REG_PRIV_BIT_SHIFT)
#define META_REG_PRIV_WRITE_MASK        0x10000
#define META_REG_PRIV_READ_MASK         0x20000
#define META_REG_PRIV_RW_R              META_REG_PRIV_READ_MASK
#define META_REG_PRIV_RW_W              META_REG_PRIV_WRITE_MASK
#define META_REG_PRIV_RW_RW             (META_REG_PRIV_READ_MASK | \
                                         META_REG_PRIV_WRITE_MASK)
#define META_REG_PRIV_EXCEPT_MASK       0x40000 /* see reg_priv_exception */
/* privilege requirement depends on bit BIT in REG for RW */
#define META_REG_PRIV_REGBIT(RW, REG, BIT) \
            (META_REG_PRIV_WHEN_##REG | \
             ((META_##REG##_##BIT) << META_REG_PRIV_BIT_SHIFT) | \
             META_REG_PRIV_RW_##RW)
/* privilege required WHEN for RW */
#define META_REG_PRIV(RW, WHEN) \
            (META_REG_PRIV_WHEN_##WHEN | \
             META_REG_PRIV_RW_##RW)

typedef struct {
    /* special TCG generation for writing immediate, overrides gen_write */
    MetaGenRegWriteImm  *gen_write_imm;
    /* special TCG generation for reading, overrides read */
    MetaGenRegAccess    *gen_read;
    /* special TCG generation for writing, overrides write */
    MetaGenRegAccess    *gen_write;
    /* special TCG generation for long writing, overrides write */
    MetaGenRegLAccess   *gen_writel;
    /* priv requirements for writing this register */
    uint32_t            priv;

    /* callback for reading */
    MetaRegRead         *read;
    /* callback for writing */
    MetaRegWrite        *write;
    /* callback for long writing */
    MetaRegWritel       *writel;
    /* read TCG helper flags */
    int                 read_flags;
    /* write TCG helper flags */
    int                 write_flags;
    /* writel TCG helper flags */
    int                 writel_flags;
} MetaRegHandler;

#if !defined(CONFIG_USER_ONLY)
static void meta_gen_reg_set_readonly(DisasContext *dc, MetaUnit unit, int id,
                                      TCGv reg)
{
    /* read only registers cannot be written to, so do nothing with the value */
}
#endif

static void meta_gen_reg_access_unimplemented(DisasContext *dc, MetaUnit unit,
                                              int id, TCGv reg)
{
    DBGLOG("reg=%s\n", meta_reg_name(unit, id));
    meta_unimplemented(dc, "access to special register");
    tcg_gen_movi_i32(reg, 0);
}

static void meta_gen_reg_read_warn(DisasContext *dc, MetaUnit unit, int id,
                                   TCGv reg)
{
    DBGLOG("T%d Unimplemented read from %s in %08x at %08x\n",
            dc->env->thread_num, meta_reg_name(unit, id), dc->instr, dc->pc);
    tcg_gen_movi_i32(reg, 0);
}

static void meta_gen_reg_write_warn(DisasContext *dc, MetaUnit unit, int id,
                                    TCGv reg)
{
    DBGLOG("T%d Unimplemented write to %s in %08x at %08x\n",
            dc->env->thread_num, meta_reg_name(unit, id), dc->instr, dc->pc);
}

#define UNIMPLEMENTED_REGHANDLER_READ \
    .gen_read = meta_gen_reg_access_unimplemented
#define UNIMPLEMENTED_REGHANDLER_WRITE \
    .gen_write = meta_gen_reg_access_unimplemented
#define UNIMPLEMENTED_REGHANDLER(REG) \
    [REG] = { \
        UNIMPLEMENTED_REGHANDLER_READ, \
        UNIMPLEMENTED_REGHANDLER_WRITE, \
    }

#define WARNING_REGHANDLER(REG) \
    [REG] = { \
        .gen_read = meta_gen_reg_read_warn, \
        .gen_write = meta_gen_reg_write_warn, \
    }

/*
 * bits 31-3 of TXENABLE are static, the remaining few bits are in
 * cregs[META_TXENABLE].
 */
static uint32_t meta_txenable_base(CPUMETAState *env)
{
    uint32_t ret;
    ret = env->global->core_rev & 0xffff0000;
    ret |= env->tcaps << META_TXENABLE_TCAPS_SHIFT;
    ret |= env->thread_num << 8;
    ret |= (env->global->core_rev & 0x00000f00) >> 4;
    return ret;
}

static void meta_gen_txenable_get(DisasContext *dc, MetaUnit unit,
                                  int id, TCGv reg)
{
    tcg_gen_ori_i32(reg, cpu_cregs[META_TXENABLE], meta_txenable_base(dc->env));
}

static void meta_gen_txenable_set(DisasContext *dc, MetaUnit unit,
                                  int id, TCGv reg)
{
#if !defined(CONFIG_USER_ONLY)
    TCGLabel *l1 = gen_new_label();
    TCGv temp = tcg_temp_new_i32();

    /* we can assume that thread is enabled, or we wouldn't be running code */
    tcg_gen_andi_i32(temp, reg, META_TXENABLE_THREADEN_MASK);
    tcg_gen_brcondi_i32(TCG_COND_NE, temp, 0, l1);
    {
        gen_helper_thread_stop(tcg_env);
    }
    gen_set_label(l1);
    /* stop decoding this block */
    dc->jmp = JMP_INDIRECT;
#endif
}

static void meta_gen_txenable_set_imm(DisasContext *dc, MetaUnit unit,
                                  int id, uint32_t val)
{
#if !defined(CONFIG_USER_ONLY)
    /* we can assume that thread is enabled, or we wouldn't be running code */
    if (!(val & META_TXENABLE_THREADEN_MASK)) {
        gen_helper_thread_stop(tcg_env);
        /* stop decoding this block */
        dc->jmp = JMP_INDIRECT;
    }
#endif
}

static uint32_t meta_txenable_read(CPUArchState *env)
{
    return env->cregs[META_TXENABLE] | meta_txenable_base(env);
}

#if !defined(CONFIG_USER_ONLY)
static void meta_txenable_write(CPUArchState *env, uint32_t val)
{
    /*
     * may be called from a different thread, so we cannot assume that the
     * thread is already enabled.
     */
    uint32_t diff = val ^ env->cregs[META_TXENABLE];
    if (diff & META_TXENABLE_THREADEN_MASK) {
        if (val & META_TXENABLE_THREADEN_MASK) {
            HELPER(thread_enable)(env);
        } else {
            HELPER(thread_stop)(env);
        }
    }
}
#else
#define meta_txenable_write NULL
#endif

static uint32_t meta_txmode_read(CPUMETAState *env)
{
    return env->cregs[META_TXMODE];
}

static void meta_txmode_write(CPUMETAState *env, uint32_t val)
{
    int rmode;

    const uint32_t always_update_bits =
        META_TXMODE_DSPRRADIX_MASK |
        (META_TXMODE_AU1ADDRMODE_MASK << META_TXMODE_AU1ADDRMODE_SHIFT) |
        (META_TXMODE_AU0ADDRMODE_MASK << META_TXMODE_AU0ADDRMODE_SHIFT) |
        META_TXMODE_DUMODE_MASK;

    if ((val & always_update_bits) ^
        (env->cregs[META_TXMODE] & always_update_bits)) {
        env->cregs[META_TXMODE] &= ~always_update_bits;
        env->cregs[META_TXMODE] |= val & always_update_bits;
    }

    if (val & META_TXMODE_FPURMODEGUARD_MASK) {
        rmode = val >> META_TXMODE_FPURMODE_SHIFT;
        rmode &= META_TXMODE_FPURMODE_MASK;

        env->cregs[META_TXMODE] &= ~(META_TXMODE_FPURMODE_MASK << META_TXMODE_FPURMODE_SHIFT);
        env->cregs[META_TXMODE] |= rmode << META_TXMODE_FPURMODE_SHIFT;

        switch (rmode) {
        case 0: rmode = float_round_nearest_even; break;
        case 1: rmode = float_round_to_zero; break;
        case 2: rmode = float_round_up; break;
        case 3: rmode = float_round_down; break;
        }

        set_float_rounding_mode(rmode, &env->fx.status);
    }
}

static void meta_gen_txmode_set(DisasContext *dc, MetaUnit unit,
                                int id, TCGv reg)
{
    gen_helper_reg_write(tcg_env, tcg_constant_ptr(meta_txmode_write), reg);

    /* tbflags may change */
    dc->is_jmp = DISAS_UPDATE;
}

static void meta_gen_txstatus_get(DisasContext *dc, MetaUnit unit,
                                  int id, TCGv reg)
{
    int lsmstep = (dc->tbflags & META_TBFLAG_LSM) >> META_TBFLAG_LSM_SHIFT;
    TCGv temp = tcg_temp_new_i32();
    /* shift the condition flags in, then OR with TXSTATUS */
    tcg_gen_shli_i32(temp, cpu_cf[4], 1);
    tcg_gen_or_i32(temp, temp, cpu_cf[3]);
    tcg_gen_shli_i32(temp, temp, 1);
    tcg_gen_or_i32(temp, temp, cpu_cf[2]);
    tcg_gen_shli_i32(temp, temp, 1);
    tcg_gen_or_i32(temp, temp, cpu_cf[1]);
    tcg_gen_shli_i32(temp, temp, 1);
    tcg_gen_or_i32(temp, temp, cpu_cf[0]);
    tcg_gen_or_i32(temp, temp, cpu_cregs[META_TXSTATUS]);
    /* lsmstep is separate, so OR that in too */
    tcg_gen_ori_i32(reg, temp, lsmstep << META_TXSTATUS_LSMSTEP_SHIFT);
}

static void meta_gen_txstatus_set(DisasContext *dc, MetaUnit unit,
                                  int id, TCGv reg)
{
    TCGv temp = tcg_temp_new_i32();
    int i;

    /* Update main fields */
    tcg_gen_xor_i32(temp, cpu_cregs[META_TXSTATUS], reg);
    tcg_gen_andi_i32(temp, temp, ~META_TXSTATUS_ROMASK);
    tcg_gen_xor_i32(cpu_cregs[META_TXSTATUS], cpu_cregs[META_TXSTATUS], temp);
    /* Update flags */
    for (i = 0; i < 5; ++i) {
        tcg_gen_andi_i32(temp, reg, 1 << i);
        tcg_gen_setcondi_i32(TCG_COND_NE, cpu_cf[i], temp, 0);
    }
    /* Update lsmstep */
    tcg_gen_andi_i32(temp, reg, META_TXSTATUS_LSMSTEP_MASK);
    tcg_gen_shri_i32(cpu_lsmstep, temp, META_TXSTATUS_LSMSTEP_SHIFT);

    /* lsmstep has been set, ensure we don't lazily clobber the new value */
    dc->tbflags &= ~META_TBFLAG_LSM;
    dc->tbflags |= dc->tb_flags & META_TBFLAG_LSM;
    dc->is_jmp = DISAS_UPDATE;
}

static uint32_t meta_txstatus_read(CPUArchState *env)
{
    uint32_t res;
    /* shift the condition flags in, then OR with TXSTATUS */
    res = env->cf[4] << 1;
    res |= env->cf[3];
    res <<= 1;
    res |= env->cf[2];
    res <<= 1;
    res |= env->cf[1];
    res <<= 1;
    res |= env->cf[0];
    res |= env->cregs[META_TXSTATUS];
    res |= env->lsmstep << META_TXSTATUS_LSMSTEP_SHIFT;
    return res;
}

static void meta_txstatus_write(CPUArchState *env, uint32_t val)
{
    uint32_t diff;
    int i;

    /* Update main fields */
    diff = env->cregs[META_TXSTATUS] ^ val;
    if (env->cregs[META_TXENABLE] & 1) {
        diff &= ~META_TXSTATUS_ROMASK;
    } else {
        /* thread not running, we can do more */
        /* FIXME also priv restrictions apply here */
        diff &= ~META_TXSTATUS_STOPPEDROMASK;
    }
    env->cregs[META_TXSTATUS] ^= diff;

    /* Update flags */
    for (i = 0; i < 5; ++i) {
        env->cf[i] = (val >> i) & 1;
    }

    /* Update lsmstep */
    env->lsmstep = (val & META_TXSTATUS_LSMSTEP_MASK)
                        >> META_TXSTATUS_LSMSTEP_SHIFT;
}

static void meta_txmrsize_write(CPUMETAState *env, uint32_t val)
{
    uint32_t mrsize, nbits, modsz;

    mrsize = val & 0xffff;
    env->cregs[META_TXMRSIZE] = mrsize;

    /* update aumod_mask */
    nbits = 32 - clz32(mrsize);
    modsz = nbits - !(mrsize & (mrsize - 1));
    env->aumod_mask = ~((1 << modsz) - 1);
}

static void meta_gen_txmrsize_write(DisasContext *dc, MetaUnit unit,
                                    int id, TCGv reg)
{
    gen_helper_reg_write(tcg_env, tcg_constant_ptr(meta_txmrsize_write), reg);

    /* tbflags may change */
    dc->is_jmp = DISAS_UPDATE;
}

/* Timers */

#if defined(CONFIG_USER_ONLY)

#define meta_txtimer_read    NULL
#define meta_txtimer_write   NULL
#define meta_txtimeri_read   NULL
#define meta_txtimeri_write  NULL
#define meta_txdivtime_read NULL
#define meta_txdivtime_write NULL

#else /* CONFIG_USER_ONLY */

/*
 * TXTIMER and TXTIMERI count up and trigger when they wrap.  Each is a ptimer
 * counting down from 2^32, so the register reads as minus the ptimer count.
 * The ptimers want the BQL, and these run from generated code too.
 */
static bool meta_timer_lock(void)
{
    if (bql_locked()) {
        return false;
    }
    bql_lock();
    return true;
}

static void meta_timer_unlock(bool locked)
{
    if (locked) {
        bql_unlock();
    }
}

static void meta_update_timer_freq(CPUMETAState *env)
{
    int divide = env->cregs[META_TXDIVTIME] & META_TXDIVTIME_RFCTRL_MASK;
    uint64_t timer;
    bool locked;

    if (!divide) {
        divide = META_TXDIVTIME_RFCTRL_MASK+1;
    }
    env->timer_period = META_THREAD2CORE(env)->timer_period * divide;

    locked = meta_timer_lock();
    ptimer_transaction_begin(env->txtimer);
    timer = ptimer_get_count(env->txtimer);
    ptimer_set_period(env->txtimer,  env->timer_period);
    ptimer_set_count(env->txtimer, timer);
    ptimer_transaction_commit(env->txtimer);

    ptimer_transaction_begin(env->txtimeri);
    timer = ptimer_get_count(env->txtimeri);
    ptimer_set_period(env->txtimeri, env->timer_period);
    ptimer_set_count(env->txtimeri, timer);
    ptimer_transaction_commit(env->txtimeri);
    meta_timer_unlock(locked);
}

static uint32_t meta_txdivtime_read(CPUMETAState *env)
{
    uint32_t val = env->cregs[META_TXDIVTIME];

    /* set RPMask bits */
    val |= ((1 << env->readport_count) - 1) << META_TXDIVTIME_RPMASK_SHIFT;

    return val;
}

static void meta_txdivtime_write(CPUMETAState *env, uint32_t val)
{
    uint32_t diff = val ^ env->cregs[META_TXDIVTIME];
    diff &= ~META_TXDIVTIME_ROMASK;
    env->cregs[META_TXDIVTIME] ^= diff;
    if (diff & META_TXDIVTIME_RFCTRL_MASK) {
        meta_update_timer_freq(env);
    }
}

static uint32_t meta_timer_read(ptimer_state *pt)
{
    bool locked = meta_timer_lock();
    uint32_t val = -ptimer_get_count(pt);

    meta_timer_unlock(locked);
    return val;
}

static void meta_timer_write(ptimer_state *pt, uint32_t val)
{
    bool locked = meta_timer_lock();

    ptimer_transaction_begin(pt);
    ptimer_set_count(pt, 0x100000000llu - val);
    ptimer_transaction_commit(pt);
    meta_timer_unlock(locked);
}

static uint32_t meta_txtimer_read(CPUMETAState *env)
{
    return meta_timer_read(env->txtimer);
}

static void meta_txtimer_write(CPUMETAState *env, uint32_t val)
{
    meta_timer_write(env->txtimer, val);
}

static uint32_t meta_txtimeri_read(CPUMETAState *env)
{
    return meta_timer_read(env->txtimeri);
}

static void meta_txtimeri_write(CPUMETAState *env, uint32_t val)
{
    meta_timer_write(env->txtimeri, val);
}

static void meta_txtimer_trigger(void *opaque)
{
    CPUArchState *env = opaque;
    meta_trigger_async(env, META_TRIGGER_TIMER_MASK, META_TRIGGER_BG);
}

static void meta_txtimeri_trigger(void *opaque)
{
    CPUArchState *env = opaque;
    meta_trigger_async(env, META_TRIGGER_TIMER_MASK, META_TRIGGER_INT);
}

static void meta_init_timers(CPUMETAState *env)
{
    env->txtimer = ptimer_init(meta_txtimer_trigger, env, PTIMER_POLICY_LEGACY);
    env->txtimeri = ptimer_init(meta_txtimeri_trigger, env,
                                PTIMER_POLICY_LEGACY);
    /* Set up timers */
    env->cregs[META_TXDIVTIME] = 0x00000001;
    meta_update_timer_freq(env);
    ptimer_transaction_begin(env->txtimer);
    ptimer_set_limit(env->txtimer,  0x100000000llu, 0);
    ptimer_set_count(env->txtimer, 0x100000000llu);
    ptimer_run(env->txtimer,  0);
    ptimer_transaction_commit(env->txtimer);
    ptimer_transaction_begin(env->txtimeri);
    ptimer_set_limit(env->txtimeri, 0x100000000llu, 0);
    ptimer_set_count(env->txtimeri, 0x100000000llu);
    ptimer_run(env->txtimeri, 0);
    ptimer_transaction_commit(env->txtimeri);
}

#endif /* !CONFIG_USER_ONLY */

static void meta_txl1start_write(CPUMETAState *env, uint32_t val)
{
    if (!meta_dsp_supported(env)) {
        return;
    }

    env->cregs[META_TXL1START] = val & 0xfffffffc;
}

static void meta_txl1end_write(CPUMETAState *env, uint32_t val)
{
    if (!meta_dsp_supported(env)) {
        return;
    }

    env->cregs[META_TXL1END] = val & 0xfffffffc;
}

static void meta_txl1count_write(CPUMETAState *env, uint32_t val)
{
    if (!meta_dsp_supported(env)) {
        return;
    }

    env->cregs[META_TXL1COUNT] = val;
}

static void meta_txl2start_write(CPUMETAState *env, uint32_t val)
{
    if (!meta_dsp_supported(env)) {
        return;
    }

    env->cregs[META_TXL2START] = val & 0xfffffffc;
}

static void meta_txl2end_write(CPUMETAState *env, uint32_t val)
{
    if (!meta_dsp_supported(env)) {
        return;
    }

    env->cregs[META_TXL2END] = val & 0xfffffffc;
}

static void meta_txl2count_write(CPUMETAState *env, uint32_t val)
{
    if (!meta_dsp_supported(env)) {
        return;
    }

    env->cregs[META_TXL2COUNT] = val;
}

static void meta_txlxcount_gen_write(DisasContext *dc, MetaUnit unit,
                                     int id, TCGv reg)
{
    if (!meta_dsp_supported(dc->env)) {
        return;
    }

    tcg_gen_mov_i32(cpu_cregs[id], reg);

    /* count may now be non-zero, tbflags change */
    dc->is_jmp = DISAS_UPDATE;
}

static uint32_t meta_txdrctrl_read(CPUMETAState *env)
{
    uint8_t addr_bits;
    uint32_t txdrctrl;

    addr_bits = env->global->dspram_addr_bits;
    txdrctrl = 0;

    txdrctrl |= addr_bits << META_TXDRCTRL_DSPRAMSIZE_SHIFT;

    txdrctrl |= (env->dspram_off_and[0] >> (addr_bits - 4))
                << META_TXDRCTRL_D0OFFSAND_SHIFT;
    txdrctrl |= (env->dspram_off_and[1] >> (addr_bits - 4))
                << META_TXDRCTRL_D1OFFSAND_SHIFT;
    txdrctrl |= (env->dspram_off_or[0] >> (addr_bits - 4))
                << META_TXDRCTRL_D0OFFSOR_SHIFT;
    txdrctrl |= (env->dspram_off_or[1] >> (addr_bits - 4))
                << META_TXDRCTRL_D1OFFSOR_SHIFT;

    return txdrctrl;
}

static void meta_txdrctrl_write(CPUMETAState *env, uint32_t val)
{
    uint8_t low_bits;

    low_bits = env->global->dspram_addr_bits - 4;

    env->dspram_off_and[0] = ((val & META_TXDRCTRL_D0OFFSAND_MASK)
                              >> META_TXDRCTRL_D0OFFSAND_SHIFT)
                              << low_bits;
    env->dspram_off_and[0] |= (1 << low_bits) - 1;
    env->dspram_off_and[1] = ((val & META_TXDRCTRL_D1OFFSAND_MASK)
                              >> META_TXDRCTRL_D1OFFSAND_SHIFT)
                              << low_bits;
    env->dspram_off_and[1] |= (1 << low_bits) - 1;

    env->dspram_off_or[0] = ((val & META_TXDRCTRL_D0OFFSOR_MASK)
                             >> META_TXDRCTRL_D0OFFSOR_SHIFT)
                             << low_bits;
    env->dspram_off_or[1] = ((val & META_TXDRCTRL_D1OFFSOR_MASK)
                             >> META_TXDRCTRL_D1OFFSOR_SHIFT)
                             << low_bits;
}

static void meta_txdrsize_write(CPUMETAState *env, uint32_t val)
{
    env->cregs[META_TXDRSIZE] = val & 0x7fff7fff;
}

static void meta_gen_txdrsize_write(DisasContext *dc, MetaUnit unit,
                                    int id, TCGv reg)
{
    gen_helper_reg_write(tcg_env, tcg_constant_ptr(meta_txdrsize_write), reg);

    /* tbflags may change */
    dc->is_jmp = DISAS_UPDATE;
}

/* deferred triggers */

/*
 * TXDEFR contains the master state, and TXSTAT/TXSTATI reflect some of that
 * state under certain circumstances.
 */

static void meta_txdefr_write(CPUMETAState *env, uint32_t val)
{
    uint32_t temp;
    uint32_t txdefr;
    uint32_t exflags;

    if (unlikely(!(meta_supported_isas(env) & (1 << META2)))) {
        return;
    }

    /* overwrite the writeable bits */
    txdefr = env->cregs[META_TXDEFR];
    temp = val ^ txdefr;
    temp &= ~META_TXDEFR_ROMASK;
    /* ensure same thing happens to MSB as error bit */
    temp |= (temp << 7) & META_TXDEFR_BUSERR_MASK;

    txdefr ^= temp;

    temp = txdefr >> META_TXDEFR_TRIGSTAT_SHIFT;
    if ((temp & META_DEFR_FPU_MASK) ^
        ((env->cregs[META_TXDEFR] >> META_TXDEFR_TRIGSTAT_SHIFT) &
         META_DEFR_FPU_MASK)) {
        exflags = get_float_exception_flags(&env->fx.status);

        if (!(temp & META_DEFR_FPU_DENORMAL_MASK))
            exflags &= ~(float_flag_input_denormal_flushed |
                         float_flag_output_denormal_flushed);
        if (!(temp & META_DEFR_FPU_INVALID_MASK))
            exflags &= ~float_flag_invalid;
        if (!(temp & META_DEFR_FPU_DIVZERO_MASK))
            exflags &= ~float_flag_divbyzero;
        if (!(temp & META_DEFR_FPU_OVERFLOW_MASK))
            exflags &= ~float_flag_overflow;
        if (!(temp & META_DEFR_FPU_UNDERFLOW_MASK))
            exflags &= ~float_flag_underflow;
        if (!(temp & META_DEFR_FPU_INEXACT_MASK))
            exflags &= ~float_flag_inexact;

        set_float_exception_flags(exflags, &env->fx.status);

        env->cregs[META_TXSTATUS] |= META_TXSTATUS_FPACTIVE_MASK;
    }

    env->cregs[META_TXDEFR] = txdefr;

    /* update deferred trigger bits */

    /* first find active deferred triggers */
    temp = txdefr >> 16;

    /* update background deferred trigger */
    if (temp & (txdefr ^ META_TXDEFR_TRIGICTRL_MASK)) {
        do_bgtrigger(env, META_TRIGGER_DEFR_MASK);
    } else {
        env->tregs[META_TXSTAT] &= ~META_TRIGGER_DEFR_MASK;
    }

    /* update interrupt deferred trigger */
    if (temp & txdefr) {
        do_itrigger(env, META_TRIGGER_DEFR_MASK);
    } else {
        env->tregs[META_TXSTATI] &= ~META_TRIGGER_DEFR_MASK;
    }
}

static void meta_gen_defr_trigger(DisasContext *dc, int defr)
{
    TCGv temp;

    temp = meta_const_i32(defr);
    gen_helper_defr_trigger(tcg_env, temp);

    /* this might trigger an interrupt */
    dc->is_jmp = DISAS_UPDATE;
}

static void meta_gen_txprivext_set(DisasContext *dc, MetaUnit unit,
                                   int id, TCGv reg)
{
    TCGv temp;

    temp = tcg_temp_new_i32();
    tcg_gen_xor_tl(temp, cpu_cregs[META_TXPRIVEXT], reg);
    tcg_gen_andi_tl(temp, temp, ~META_TXPRIVEXT_ROMASK(dc->env));
    tcg_gen_xor_tl(cpu_cregs[META_TXPRIVEXT], cpu_cregs[META_TXPRIVEXT], temp);

    /* single stepping/minim/static branch prediction may have been changed */
    dc->is_jmp = DISAS_UPDATE;
}

static void meta_gen_txprivext_set_imm(DisasContext *dc, MetaUnit unit,
                                       int id, uint32_t val)
{
    MetaISA isa;
    TCGv temp;

    temp = tcg_temp_new_i32();
    tcg_gen_xori_tl(temp, cpu_cregs[META_TXPRIVEXT], val);
    tcg_gen_andi_tl(temp, temp, ~META_TXPRIVEXT_ROMASK(dc->env));
    tcg_gen_xor_tl(cpu_cregs[META_TXPRIVEXT], cpu_cregs[META_TXPRIVEXT], temp);

    /*
     * MiniM enable bit requires an update if we'd be in MiniM with it set, and
     * the new value doesn't match the MiniM TB flag.
     */
    isa = meta_current_isa_pc_flags(dc->env, dc->pc,
                                    META_ISAFLAGS_IGNORE_MINIMENABLE);
    if (isa == MINIM && !(dc->tbflags & META_TBFLAG_MINIM)
                            != !(val & META_TXPRIVEXT_MINIMENABLE_MASK)) {
        dc->is_jmp = DISAS_UPDATE;
    }

    if (!(dc->tbflags & META_TBFLAG_STEP) && (val & META_TXPRIVEXT_STEP_MASK)) {
        /* switch on single stepping after this instruction */
        dc->delayed_tbflags |= META_TBFLAG_STEP;
    }
    /* switching single stepping off still stops once more */
}

static void meta_txprivext_write(CPUMETAState *env, uint32_t val)
{
    /* FIXME ensure thread is stopped while we do this */
    uint32_t diff = env->cregs[META_TXPRIVEXT] ^ val;
    if (env->cregs[META_TXENABLE] == META_TXENABLE_ENABLED) {
        diff &= ~META_TXPRIVEXT_RUNNINGROMASK(env);
    } else {
        diff &= ~META_TXPRIVEXT_STOPPEDROMASK(env);
    }
    env->cregs[META_TXPRIVEXT] ^= diff;
}

/* Counters */

#ifdef COUNT_ACTCYC
static void meta_gen_txtactcyc_get(DisasContext *dc, MetaUnit unit,
                                      int id, TCGv reg)
{
    /* value read from TXTACTCYC does NOT include current instruction */
    tcg_gen_addi_i32(reg, cpu_cregs[META_TXTACTCYC],
                     dc->instr_count - 1);
}

static void meta_gen_txtactcyc_set(DisasContext *dc, MetaUnit unit,
                                      int id, TCGv reg)
{
    /* TXTACTCYC is incremented for this cycle after the set */
    tcg_gen_subi_i32(cpu_cregs[META_TXTACTCYC], reg,
                     dc->instr_count - 1);
}

static TCGv meta_gen_actcyc_get(DisasContext *dc)
{
    TCGv actcyc = tcg_temp_new();
    meta_gen_txtactcyc_get(dc, META_UNIT_CT, META_TXTACTCYC, actcyc);
    return actcyc;
}

#else
#define meta_gen_txtactcyc_get NULL
#define meta_gen_txtactcyc_set NULL
#endif

static void meta_gen_txidlecyc_get(DisasContext *dc, MetaUnit unit,
                                      int id, TCGv reg)
{
    TCGv txidlecyc;
    meta_get_reg(dc, META_UNIT_CT, META_TXIDLECYC, &txidlecyc);
    tcg_gen_mov_i32(reg, txidlecyc);
}

static void meta_gen_txidlecyc_set(DisasContext *dc, MetaUnit unit,
                                      int id, TCGv reg)
{
    TCGv txidlecyc;
    meta_get_reg(dc, META_UNIT_CT, META_TXIDLECYC, &txidlecyc);
    tcg_gen_movi_i32(txidlecyc, 0);
}

static MetaRegHandler meta_ct_handlers[META_CT_MAX] = {
    [META_TXENABLE] = {
        .gen_read = meta_gen_txenable_get,
        .gen_write = meta_gen_txenable_set,
        .gen_write_imm = meta_gen_txenable_set_imm,
        .priv = META_REG_PRIV_REGBIT(W, TXPRIVEXT, TENW),
        .read = meta_txenable_read,
        .write = meta_txenable_write,
        .read_flags = TCG_CALL_NO_RWG,
    },
    [META_TXMODE] = {
        .gen_write = meta_gen_txmode_set,
        .read = meta_txmode_read,
        .write = meta_txmode_write,
        .read_flags = TCG_CALL_NO_RWG,
    },
    [META_TXSTATUS] = {
        .gen_read = meta_gen_txstatus_get,
        .gen_write = meta_gen_txstatus_set,
        .priv = META_REG_PRIV_REGBIT(W, TXPRIVEXT, TSTATUS),
        .read = meta_txstatus_read,
        .write = meta_txstatus_write,
        .read_flags = TCG_CALL_NO_RWG,
    },
    [META_TXRPT] = {},
    [META_TXTIMER] = {
        .read = meta_txtimer_read,
        .write = meta_txtimer_write,
        .read_flags = TCG_CALL_NO_RWG,
    },
    [META_TXL1START] = {
        .write = meta_txl1start_write,
    },
    [META_TXL1END] = {
        .write = meta_txl1end_write,
    },
    [META_TXL1COUNT] = {
        .gen_write = meta_txlxcount_gen_write,
        .write = meta_txl1count_write,
    },
    [META_TXL2START] = {
        .write = meta_txl2start_write,
    },
    [META_TXL2END] = {
        .write = meta_txl2end_write,
    },
    [META_TXL2COUNT] = {
        .gen_write = meta_txlxcount_gen_write,
        .write = meta_txl2count_write,
    },
    WARNING_REGHANDLER(META_TXBPOBITS),
    [META_TXMRSIZE] = {
        .write = meta_txmrsize_write,
        .gen_write = meta_gen_txmrsize_write,
    },
    [META_TXTIMERI] = {
        .read = meta_txtimeri_read,
        .write = meta_txtimeri_write,
        .priv = META_REG_PRIV_REGBIT(W, TXPRIVEXT, ITIMER),
        .read_flags = TCG_CALL_NO_RWG,
    },
    [META_TXDRCTRL] = {
        .read = meta_txdrctrl_read,
        .write = meta_txdrctrl_write,
    },
    [META_TXDRSIZE] = {
        .write = meta_txdrsize_write,
        .gen_write = meta_gen_txdrsize_write,
    },
    [META_TXCATCH0 ... META_TXCATCH3] = {
        .priv = META_REG_PRIV_REGBIT(W, TXPRIVEXT, TSTATUS),
    },
    [META_TXDEFR] = {
        .write = meta_txdefr_write,
        .priv = META_REG_PRIV_REGBIT(RW, TXPRIVEXT, TRIG),
    },
    UNIMPLEMENTED_REGHANDLER(META_CT21),
    UNIMPLEMENTED_REGHANDLER(META_TXCLKCTRL),
    UNIMPLEMENTED_REGHANDLER(META_TXINTERN0),
    UNIMPLEMENTED_REGHANDLER(META_TXAMAREG0),
    UNIMPLEMENTED_REGHANDLER(META_TXAMAREG1),
    UNIMPLEMENTED_REGHANDLER(META_TXAMAREG2),
    UNIMPLEMENTED_REGHANDLER(META_TXAMAREG3),
    [META_TXDIVTIME] = {
        .read = meta_txdivtime_read,
        .write = meta_txdivtime_write,
        .priv = META_REG_PRIV_REGBIT(W, TXPRIVEXT, TCB),
    },
    [META_TXPRIVEXT] = {
        .gen_write = meta_gen_txprivext_set,
        .gen_write_imm = meta_gen_txprivext_set_imm,
        .write = meta_txprivext_write,
        .priv = META_REG_PRIV(W, ALWAYS),
    },
    [META_TXTACTCYC] = {
        .gen_read = meta_gen_txtactcyc_get,
        .gen_write = meta_gen_txtactcyc_set,
        .priv = META_REG_PRIV_REGBIT(W, TXPRIVEXT, TICICYC),
    },
    [META_TXIDLECYC] = {
        .gen_read = meta_gen_txidlecyc_get,
        .gen_write = meta_gen_txidlecyc_set,
        .priv = META_REG_PRIV_REGBIT(W, TXPRIVEXT, TICICYC),
    },
};

static void meta_gen_pc_get(DisasContext *dc, MetaUnit unit, int id, TCGv reg)
{
    /* current PC is known immediately */
    tcg_gen_movi_i32(reg, dc->next_pc);
}

static void meta_gen_pc_set(DisasContext *dc, MetaUnit unit, int id, TCGv reg)
{
    tcg_gen_andi_tl(cpu_pc[META_PC], reg, -4);
    dc->is_jmp = DISAS_JUMP;
}

static void meta_gen_pc_set_imm(DisasContext *dc, MetaUnit unit, int id,
                                uint32_t val)
{
    val &= -4;
    tcg_gen_movi_tl(cpu_pc[META_PC], val);
    gen_goto_tb(dc, 0, val);
    dc->is_jmp = DISAS_TB_JUMP;
}

static uint32_t meta_pc_read(CPUMETAState *env)
{
    return env->pc[META_PC];
}

static void meta_pc_write(CPUMETAState *env, uint32_t val)
{
    if (env->cregs[META_TXENABLE] == META_TXENABLE_ENABLED) {
        DBGLOG("unimplemented writing to PC of running thread T%d\n",
               env->thread_num);
        return;
    }
    env->pc[META_PC] = val & -4;
}

static void meta_gen_pcx_set(DisasContext *dc, MetaUnit unit, int id, TCGv reg)
{
    tcg_gen_andi_tl(cpu_pc[META_PCX], reg, -4);
}

static void meta_gen_pcx_set_imm(DisasContext *dc, MetaUnit unit, int id,
                                 uint32_t val)
{
    tcg_gen_movi_tl(cpu_pc[META_PCX], val & -4);
}

static void meta_pcx_write(CPUMETAState *env, uint32_t val)
{
    env->pc[META_PCX] = val & -4;
}

static MetaRegHandler meta_pc_handlers[2] = {
    [META_PC] = {
        .gen_read = meta_gen_pc_get,
        .gen_write = meta_gen_pc_set,
        .gen_write_imm = meta_gen_pc_set_imm,
        .read = meta_pc_read,
        .write = meta_pc_write,
        .read_flags = TCG_CALL_NO_RWG,
    },
    [META_PCX] = {
        .gen_write = meta_gen_pcx_set,
        .gen_write_imm = meta_gen_pcx_set_imm,
        .write = meta_pcx_write,
        .priv = META_REG_PRIV_REGBIT(RW, TXPRIVEXT, PTOGGLE),
    },
};

/* Memory operation wrappers */

/* Unaligned accesses fault only when TXPRIVEXT.UnalignFault is set. */
static inline MemOp meta_mo(DisasContext *dc, MemOp size)
{
    MemOp mop = MO_TE | size;

    if (size != MO_UB && (dc->tbflags & META_TBFLAG_ALIGNFAULT)) {
        mop |= MO_ALIGN;
    }
    return mop;
}

static void gen_load(DisasContext *dc, TCGv dst1, TCGv dst2, TCGv addr,
                     unsigned int l)
{
    switch (l) {
    case 0:
        tcg_gen_qemu_ld_i32(dst1, addr, IS_USER(dc), meta_mo(dc, MO_UB));
        break;
    case 1:
        tcg_gen_qemu_ld_i32(dst1, addr, IS_USER(dc), meta_mo(dc, MO_UW));
        break;
    case 2:
        tcg_gen_qemu_ld_i32(dst1, addr, IS_USER(dc), meta_mo(dc, MO_UL));
        break;
    case 3: {
            TCGv_i64 dst64 = tcg_temp_new_i64();
            tcg_gen_qemu_ld_i64(dst64, addr, IS_USER(dc), meta_mo(dc, MO_UQ));
            tcg_gen_extr_i64_i32(dst1, dst2, dst64);
        }
    }

#if !defined(CONFIG_USER_ONLY)
    if (dc->tbflags & META_TBFLAG_DMTR) {
        TCGv actcyc = meta_gen_actcyc_get(dc);
        TCGv cl = meta_const_tl(l);
        TCGv pc = meta_const_tl(dc->pc);
        gen_helper_dload(tcg_env, addr, cl, pc, actcyc);
    }
#endif /* !CONFIG_USER_ONLY */
}

static void gen_load_i64(DisasContext *dc, TCGv_i64 dst, TCGv addr,
                         unsigned int l)
{
    TCGv tmp;

    if (l == 3) {
        tcg_gen_qemu_ld_i64(dst, addr, IS_USER(dc), meta_mo(dc, MO_UQ));
        return;
    }

    assert(l < 3);
    tmp = tcg_temp_new();

    switch (l) {
    case 0:
        tcg_gen_qemu_ld_i32(tmp, addr, IS_USER(dc), meta_mo(dc, MO_UB));
        break;
    case 1:
        tcg_gen_qemu_ld_i32(tmp, addr, IS_USER(dc), meta_mo(dc, MO_UW));
        break;
    case 2:
        tcg_gen_qemu_ld_i32(tmp, addr, IS_USER(dc), meta_mo(dc, MO_UL));
        break;
    }

    tcg_gen_extu_i32_i64(dst, tmp);
}

static void gen_store(DisasContext *dc, TCGv val1, TCGv val2, TCGv addr,
                      unsigned int l)
{
    switch (l) {
    case 0:
        tcg_gen_qemu_st_i32(val1, addr, IS_USER(dc), meta_mo(dc, MO_UB));
        break;
    case 1:
        tcg_gen_qemu_st_i32(val1, addr, IS_USER(dc), meta_mo(dc, MO_UW));
        break;
    case 2:
        tcg_gen_qemu_st_i32(val1, addr, IS_USER(dc), meta_mo(dc, MO_UL));
        break;
    case 3: {
            TCGv_i64 val64 = tcg_temp_new_i64();
            tcg_gen_concat_i32_i64(val64, val1, val2);
            tcg_gen_qemu_st_i64(val64, addr, IS_USER(dc), meta_mo(dc, MO_UQ));
        }
    }

#if !defined(CONFIG_USER_ONLY)
    if (dc->tbflags & META_TBFLAG_DMTR) {
        TCGv actcyc = meta_gen_actcyc_get(dc);
        TCGv cl = meta_const_tl(l);
        TCGv pc = meta_const_tl(dc->pc);
        gen_helper_dstore(tcg_env, addr, cl, pc, actcyc);
    }
#endif /* !CONFIG_USER_ONLY */
}

#if !defined(CONFIG_USER_ONLY)

/*
 * Trigger unit registers
 * tregs[META_TXSTAT] holds unmasked trigger status for background triggers
 * tregs[META_TXSTATI] holds unmasked trigger status for interrupt triggers
 * TXPOLL = tregs[META_TXSTAT] & tregs[META_TXMASK]
 * TXPOLLI = tregs[META_TXSTATI] & tregs[META_TXMASKI]
 * kick counts are in kicks and ikicks
 */

/* blocking read of TXSTAT, TXSTATI, TXPOLL, or TXPOLLI */
static void meta_gen_txstatpoll_get(DisasContext *dc, MetaUnit unit, int id,
                                    TCGv reg)
{
    TCGLabel *l1;
    TCGv kickreg, txstat, txmask;
    TCGv temp;
    int block_condition;
    int has_defr = (meta_supported_isas(dc->env) & (1 << META2));

    if (id & 2) {
        /* TXSTATI/TXPOLLI */
        kickreg = cpu_ikicks;
        txstat = cpu_tregs[META_TXSTATI];
        txmask = cpu_tregs[META_TXMASKI];
        block_condition = META_BLOCK_TXSTATI;
    } else {
        /* TXSTAT/TXPOLL */
        kickreg = cpu_kicks;
        txstat = cpu_tregs[META_TXSTAT];
        txmask = cpu_tregs[META_TXMASK];
        block_condition = META_BLOCK_TXSTAT;
    }

    if (!dc->reg_defr && !(id & 4)) {
        /* TXSTAT/TXSTATI block when read */
        l1 = gen_new_label();
        temp = tcg_temp_new_i32();
        tcg_gen_and_i32(temp, cpu_tregs[id], txmask);
        tcg_gen_brcondi_i32(TCG_COND_NE, temp, 0, l1);
        {
            meta_gen_block_condition(dc, block_condition);
        }
        gen_set_label(l1);
    }

    /*
     * With deferred exception bits this is made much more complicated, so just
     * use some helpers to make it much more readable and probably faster too.
     */
    if (has_defr && !dc->reg_kick) {
        if (dc->reg_defr) {
            if (id & 4) {
                if (id & 2) {
                    gen_helper_defr_txpolli(reg, tcg_env);
                } else {
                    gen_helper_defr_txpoll(reg, tcg_env);
                }
            } else {
                if (id & 2) {
                    gen_helper_defr_txstati(reg, tcg_env);
                } else {
                    gen_helper_defr_txstat(reg, tcg_env);
                }
            }
        } else {
            if (id & 2) {
                gen_helper_get_txpolli(reg, tcg_env);
            } else {
                gen_helper_get_txpoll(reg, tcg_env);
            }
        }
    } else {
        if (dc->reg_kick) {
            /*
             * "This instruction is a short-hand representation for a blocking
             * trigger read that only has the kick active in the trigger unit."
             */
            tcg_gen_andi_i32(reg, txstat, META_TRIGGER_KICK_MASK);
        } else {
            tcg_gen_and_i32(reg, txstat, txmask);
        }

        /* or in the kick count */
        temp = tcg_temp_new_i32();
        tcg_gen_shli_i32(temp, kickreg, 16);
        tcg_gen_or_i32(reg, reg, temp);
    }

    /* Update IRQEnc with highest priority interrupt trigger */
    if (id == META_TXSTATI) {
        gen_helper_update_irqenc(tcg_env);
    }
}

static void meta_gen_txstat_set(DisasContext *dc, MetaUnit unit, int id,
                                TCGv reg)
{
    TCGLabel *l1;
    TCGv temp, kicktemp, defrtemp;
    TCGv kickreg, txstat, txmask;
    int has_defr = (meta_supported_isas(dc->env) & (1 << META2));

    if (id & 2) {
        /* TXSTATI */
        kickreg = cpu_ikicks;
        txstat = cpu_tregs[META_TXSTATI];
        txmask = cpu_tregs[META_TXMASKI];
    } else {
        /* TXSTAT */
        kickreg = cpu_kicks;
        txstat = cpu_tregs[META_TXSTAT];
        txmask = cpu_tregs[META_TXMASK];
    }

    if (has_defr) {
        /*
         * kicktemp needs to be local to be preserved across the branch in the
         * DEFR handling.
         */
        kicktemp = tcg_temp_new_i32();
        defrtemp = tcg_temp_new_i32();
        /*
         * For the DEFR trigger we should ignore TXMASK*:
         * "Writes to the upper bits (and bit 3) to clear deferred state are
         * expected and allowed when TXMASK* bit 3 is zero."
         */
        tcg_gen_andi_i32(defrtemp, reg, META_TRIGGER_DEFR_MASK);
    } else {
        kicktemp = tcg_temp_new_i32();
        defrtemp = NULL;
    }

    /* clear unmasked TXSTAT bits written to */
    temp = tcg_temp_new_i32();
    tcg_gen_and_i32(temp, reg, txmask);
    tcg_gen_andi_i32(kicktemp, temp, META_TRIGGER_KICK_MASK);
    tcg_gen_not_i32(temp, temp);
    if (has_defr) {
        /* don't clear defr bit, that'd done by HELPER(defr_ack_txstat*)() */
        tcg_gen_ori_i32(temp, temp, META_TRIGGER_DEFR_MASK);
    }
    tcg_gen_and_i32(txstat, txstat, temp);

    if (has_defr) {
        /* only ack if 1 is written to DEFR trigger */
        l1 = gen_new_label();
        tcg_gen_brcondi_i32(TCG_COND_EQ, defrtemp, 0, l1);
        if (id & 2) {
            gen_helper_ack_defr_txstati(tcg_env, reg);
        } else {
            gen_helper_ack_defr_txstat(tcg_env, reg);
        }
        gen_set_label(l1);
    }


    /* decrement kick count */
    l1 = gen_new_label();
    tcg_gen_brcondi_i32(TCG_COND_EQ, kicktemp, 0, l1);
    tcg_gen_subi_i32(kickreg, kickreg, 1);
    tcg_gen_brcondi_i32(TCG_COND_EQ, kickreg, 0, l1);

    /* more kicks to come */
    tcg_gen_ori_i32(txstat, txstat, META_TRIGGER_KICK_MASK);
    gen_set_label(l1);
}

static void meta_txstat_write(CPUMETAState *env, uint32_t val)
{
    if (val & META_TRIGGER_KICK_MASK) {
        env->kicks--;
        if (env->kicks) {
            val &= ~META_TRIGGER_KICK_MASK;
        }
    }
    if (val & META_TRIGGER_DEFR_MASK) {
        HELPER(txstat_ack_defr)(env, val);
        val &= ~META_TRIGGER_DEFR_MASK;
    }

    /* clear unmasked TXSTAT bits written to */
    env->tregs[META_TXSTAT] &= ~(val & env->tregs[META_TXMASK]);
}

static void meta_txstati_write(CPUMETAState *env, uint32_t val)
{
    if (val & META_TRIGGER_KICK_MASK) {
        env->ikicks--;
        if (env->ikicks) {
            val &= ~META_TRIGGER_KICK_MASK;
        }
    }
    if (val & META_TRIGGER_DEFR_MASK) {
        HELPER(txstati_ack_defr)(env, val);
        val &= ~META_TRIGGER_DEFR_MASK;
    }

    /* clear unmasked TXSTATI bits written to */
    env->tregs[META_TXSTATI] &= ~(val & env->tregs[META_TXMASKI]);
}

static void meta_gen_txmask_set(DisasContext *dc, MetaUnit unit, int id,
                                TCGv reg)
{
    TCGv temp = tcg_temp_new_i32();
    tcg_gen_xor_i32(temp, cpu_tregs[META_TXMASK], reg);
    tcg_gen_andi_i32(temp, temp, ~META_TXMASK_ROMASK);
    tcg_gen_xor_i32(cpu_tregs[META_TXMASK], cpu_tregs[META_TXMASK], temp);
}

static void meta_txmask_write(CPUMETAState *env, uint32_t val)
{
    uint32_t diff = env->tregs[META_TXMASK] ^ val;
    diff &= ~META_TXMASK_ROMASK;
    env->tregs[META_TXMASK] ^= diff;
}

/*
 * Check trigger status, and if one is unmasked, interrupt after this
 * instruction.
 */
static void meta_gen_check_triggers(DisasContext *dc, int at_end)
{
    if (!(dc->tbflags & META_TBFLAG_ISTAT)) {
        TCGv temp;
        DEFINE_SAFE_LABEL(l1);

        /* PC is needed by HELPER(unmask_itrigger) to swap with PCX */
        if (dc->is_jmp != DISAS_JUMP) {
            tcg_gen_movi_i32(cpu_pc[META_PC], dc->next_pc);
        }
        temp = tcg_temp_new_i32();
        gen_helper_unmask_itrigger(temp, tcg_env);
        if (at_end) {
            /*
             * if we're at the end, this conditional will allow us to direct
             * chain to the next tb in the case of there being no triggers.
             */
            meta_init_safe_label(dc, &l1);
            tcg_gen_brcondi_i32(TCG_COND_EQ, temp, 0, l1.label);
        }
        dc->is_jmp = DISAS_JUMP;
        meta_set_safe_label(dc, at_end, &l1);
    }
}

static void meta_gen_txmaski_set(DisasContext *dc, MetaUnit unit, int id,
                                 TCGv reg)
{
    TCGv temp = tcg_temp_new_i32();
    tcg_gen_xor_i32(temp, cpu_tregs[META_TXMASKI], reg);
    tcg_gen_andi_i32(temp, temp, ~META_TXMASK_ROMASK);
    tcg_gen_xor_i32(cpu_tregs[META_TXMASKI], cpu_tregs[META_TXMASKI], temp);

    /* trigger may be pending */
    meta_gen_check_triggers(dc, 0);
}

static void meta_txmaski_write(CPUMETAState *env, uint32_t val)
{
    uint32_t diff = env->tregs[META_TXMASKI] ^ val;
    diff &= ~META_TXMASK_ROMASK;
    env->tregs[META_TXMASKI] ^= diff;
}

static MetaRegHandler meta_trig_handlers[META_TR_MAX] = {
    [META_TXSTAT] = {
        .gen_read = meta_gen_txstatpoll_get,
        .gen_write = meta_gen_txstat_set,
        .read = HELPER(txpoll_read),
        .read_flags = TCG_CALL_NO_RWG,
        .write = meta_txstat_write,
        .priv = META_REG_PRIV_EXCEPT_MASK,
    },
    [META_TXMASK] = {
        .gen_write = meta_gen_txmask_set,
        .write = meta_txmask_write,
    },
    [META_TXSTATI] = {
        .gen_read = meta_gen_txstatpoll_get,
        .gen_write = meta_gen_txstat_set,
        .read = HELPER(txpolli_read),
        .read_flags = TCG_CALL_NO_RWG,
        .write = meta_txstati_write,
        .priv = META_REG_PRIV_REGBIT(RW, TXPRIVEXT, PTOGGLE),
    },
    [META_TXMASKI] = {
        /* write may trigger an interrupt */
        .gen_write = meta_gen_txmaski_set,
        .write = meta_txmaski_write,
        .priv = META_REG_PRIV_REGBIT(W, TXPRIVEXT, PTOGGLE),
    },
    [META_TXPOLL] = {
        .gen_read = meta_gen_txstatpoll_get,
        .gen_write = meta_gen_reg_set_readonly,
        .read = HELPER(txpoll_read),
        .read_flags = TCG_CALL_NO_RWG,
        .priv = META_REG_PRIV_EXCEPT_MASK,
    },
    UNIMPLEMENTED_REGHANDLER(META_TXGPIOI),
    [META_TXPOLLI] = {
        .gen_read = meta_gen_txstatpoll_get,
        .gen_write = meta_gen_reg_set_readonly,
        .read = HELPER(txpolli_read),
        .read_flags = TCG_CALL_NO_RWG,
        .priv = META_REG_PRIV_REGBIT(RW, TXPRIVEXT, PTOGGLE),
    },
    UNIMPLEMENTED_REGHANDLER(META_TXGPIOO),
};

#else /* !CONFIG_USER_ONLY */
#define meta_trig_handlers NULL
static void meta_gen_check_triggers(DisasContext *dc, int at_end)
{
}
#endif /* CONFIG_USER_ONLY */

/* Read port registers */

static uint64_t meta_rd_read_i64(CPUMETAState *env)
{
    uint64_t data;

    if (!env->readport_count) {
        if (!(env->cregs[META_TXDIVTIME] & META_TXDIVTIME_RPDIRTY_MASK)) {
            MetaSigNum signum = META_SIGNUM(META_FREASON_GENERAL,
                                            META_HREASON_UNKNOWN);
#if defined(CONFIG_USER_ONLY)
            helper_raise_exception(env, EXCP_FAULT_BASE + signum);
#else
            helper_halt_signum(env, signum);
#endif
        }
        return 0;
    }

    data = env->readport_data[env->readport_idx_r++];
    env->readport_idx_r %= META_READPORT_DEPTH;
    env->readport_count--;
    return data;
}

static uint32_t meta_rd_read_i32(CPUMETAState *env)
{
    return (uint32_t)meta_rd_read_i64(env);
}

static void meta_gen_rd_read_i64(DisasContext *dc, TCGv_i64 dst)
{
    gen_helper_reg_read64(dst, tcg_env, tcg_constant_ptr(meta_rd_read_i64));
}

static void meta_gen_rd_read_i32(DisasContext *dc, TCGv dst)
{
    gen_helper_reg_read(dst, tcg_env, tcg_constant_ptr(meta_rd_read_i32));
}

static void meta_gen_rapf_set(DisasContext *dc, MetaUnit unit, int id, TCGv reg)
{
#if !defined(CONFIG_USER_ONLY)
    if (dc->tbflags & META_TBFLAG_OTHERTR) {
        TCGv actcyc = meta_gen_actcyc_get(dc);
        TCGv pc = meta_const_tl(dc->pc);
        gen_helper_dprefetch(tcg_env, reg, pc, actcyc);
    }
#endif /* !CONFIG_USER_ONLY */
}

#define DECLARE_READPORT_WRITE(name) \
static void meta_gen_ ## name ## _write(DisasContext *dc, TCGv reg, \
                                        MetaOpWidth sz) \
{ \
    TCGv_i64 data = tcg_temp_new_i64(); \
    gen_load_i64(dc, data, reg, sz); \
    gen_helper_read_append_ ## name(tcg_env, data); \
} \
\
static void meta_gen_ ## name ## _write32(DisasContext *dc, MetaUnit unit, \
                                          int id, TCGv reg) \
{ \
    meta_gen_ ## name ## _write(dc, reg, WIDTH_32B); \
}

DECLARE_READPORT_WRITE(ra)
DECLARE_READPORT_WRITE(rabz)
DECLARE_READPORT_WRITE(rawz)
DECLARE_READPORT_WRITE(radz)
DECLARE_READPORT_WRITE(rabx)
DECLARE_READPORT_WRITE(rawx)
DECLARE_READPORT_WRITE(radx)
DECLARE_READPORT_WRITE(ram8x)
DECLARE_READPORT_WRITE(ram8x32)
DECLARE_READPORT_WRITE(ram16x)

static MetaRegHandler meta_port_handlers[32] = {
    UNIMPLEMENTED_REGHANDLER(0 ... 31),
    [META_RA] = {
        .read = meta_rd_read_i32,
        .gen_write = meta_gen_ra_write32,
    },
    [META_RAPF] = {
        UNIMPLEMENTED_REGHANDLER_READ,
        .gen_write = meta_gen_rapf_set,
    },
    [META_RABZ] = {
        UNIMPLEMENTED_REGHANDLER_READ,
        .gen_write = meta_gen_rabz_write32,
    },
    [META_RAWZ] = {
        UNIMPLEMENTED_REGHANDLER_READ,
        .gen_write = meta_gen_rawz_write32,
    },
    [META_RADZ] = {
        UNIMPLEMENTED_REGHANDLER_READ,
        .gen_write = meta_gen_radz_write32,
    },
    [META_RABX] = {
        UNIMPLEMENTED_REGHANDLER_READ,
        .gen_write = meta_gen_rabx_write32,
    },
    [META_RAWX] = {
        UNIMPLEMENTED_REGHANDLER_READ,
        .gen_write = meta_gen_rawx_write32,
    },
    [META_RADX] = {
        UNIMPLEMENTED_REGHANDLER_READ,
        .gen_write = meta_gen_radx_write32,
    },
    [META_RAM8X] = {
        UNIMPLEMENTED_REGHANDLER_READ,
        .gen_write = meta_gen_ram8x_write32,
    },
    [META_RAM16X] = {
        UNIMPLEMENTED_REGHANDLER_READ,
        .gen_write = meta_gen_ram16x_write32,
    },
    [META_RAM8X32] = {
        UNIMPLEMENTED_REGHANDLER_READ,
        .gen_write = meta_gen_ram8x32_write32,
    },
};

static void meta_gen_unhandled_port_write(DisasContext *dc, TCGv reg,
                                          MetaOpWidth sz)
{
    meta_unimplemented(dc, "unhandled port");
}

static void meta_gen_rapf_write(DisasContext *dc, TCGv reg, MetaOpWidth sz)
{
    /* NOP */
}

static void (*meta_port_gen_write[32])(DisasContext *dx, TCGv reg,
                                       MetaOpWidth sz) = {
    [0 ... 31] = meta_gen_unhandled_port_write,
    [META_RA] = meta_gen_ra_write,
    [META_RAPF] = meta_gen_rapf_write,
    [META_RABZ] = meta_gen_rabz_write,
    [META_RAWZ] = meta_gen_rawz_write,
    [META_RADZ] = meta_gen_radz_write,
    [META_RABX] = meta_gen_rabx_write,
    [META_RAWX] = meta_gen_rawx_write,
    [META_RADX] = meta_gen_radx_write,
    [META_RAM8X] = meta_gen_ram8x_write,
    [META_RAM16X] = meta_gen_ram16x_write,
    [META_RAM8X32] = meta_gen_ram8x32_write,
};

#if !defined(CONFIG_USER_ONLY)

/* Trace data */

#define META_TRACE_HEADER_EVENT_SHIFT   12
#define META_TRACE_HEADER_DATA_SHIFT    4
#define META_TRACE_HEADER_TN_SHIFT      2
#define META_TRACE_HEADER_P_MASK        (1 << 1)
#define META_TRACE_HEADER_T_MASK        (1 << 0)

typedef enum {
    META_TRACE_MOV_TTDREG   = 0x8,
    META_TRACE_MOV_TTMARK   = 0xa,
    META_TRACE_TTMOV        = 0xb,
    META_TRACE_MOVL_TTREC   = 0xe,
    META_TRACE_TRACEALL     = 0xf,
} MetaTraceEvents;

#define META_TRACE_TAG_CONDFALSE_MASK   (1 << 0);
#define META_TRACE_TAG_BRFALSE_MASK     (1 << 1);
#define META_TRACE_TAG_PREDFETCH_MASK   (1 << 2);
#define META_TRACE_TAG_ISTAT_MASK       (1 << 3);

static uint16_t meta_ttevent_header(DisasContext *dc, MetaTraceEvents event,
                                    uint8_t data)
{
    uint32_t header;
    header = event << META_TRACE_HEADER_EVENT_SHIFT;
    header |= data << META_TRACE_HEADER_DATA_SHIFT;
    header |= (dc->tbflags & META_TBFLAG_TX) << META_TRACE_HEADER_TN_SHIFT;
    if (dc->tbflags & META_TBFLAG_TRPC || event == META_TRACE_TTMOV) {
        header |= META_TRACE_HEADER_P_MASK;
    }
    if (dc->tbflags & META_TBFLAG_TRTAG) {
        header |= META_TRACE_HEADER_T_MASK;
    }
    return header;
}

static uint32_t meta_ttevent_tag(DisasContext *dc)
{
    uint32_t tag = 0;
    if (dc->tbflags & META_TBFLAG_ISTAT) {
        tag |= META_TRACE_TAG_ISTAT_MASK;
    }
    return tag;
}

/* Trace event with only DL, no DH (only bits 15:4 of header) */
static void meta_gen_ttevent1(DisasContext *dc, uint16_t header, TCGv dl)
{
    int nargs = 0;
    TCGv actcyc = meta_gen_actcyc_get(dc);
    TCGv head = meta_const_i32(header);
    TCGv args[2];
    if (header & META_TRACE_HEADER_P_MASK) {
        args[nargs++] = meta_const_i32(dc->pc);
    }
    if (header & META_TRACE_HEADER_T_MASK) {
        args[nargs++] = meta_const_i32(meta_ttevent_tag(dc));
    }
    switch (nargs) {
    case 0:
        gen_helper_ttevent1(actcyc, head, dl);
        break;
    case 1:
        gen_helper_ttevent2(actcyc, head, args[0], dl);
        break;
    case 2:
        gen_helper_ttevent3(actcyc, head, args[0], args[1], dl);
        break;
    }
}

/* Trace event with both DL and DH (only bits 15:4 of header) */
static void meta_gen_ttevent2(DisasContext *dc, uint16_t header, TCGv dl, TCGv dh)
{
    int nargs = 0;
    TCGv actcyc = meta_gen_actcyc_get(dc);
    TCGv head = meta_const_i32(header);
    TCGv args[2];
    if (header & META_TRACE_HEADER_P_MASK) {
        args[nargs++] = meta_const_i32(dc->pc);
    }
    if (header & META_TRACE_HEADER_T_MASK) {
        args[nargs++] = meta_const_i32(meta_ttevent_tag(dc));
    }
    switch (nargs) {
    case 0:
        gen_helper_ttevent2(actcyc, head, dl, dh);
        break;
    case 1:
        gen_helper_ttevent3(actcyc, head, args[0], dl, dh);
        break;
    case 2:
        gen_helper_ttevent4(actcyc, head, args[0], args[1], dl, dh);
        break;
    }
}

static void meta_gen_ttmark_set(DisasContext *dc, MetaUnit unit, int id,
                                TCGv reg)
{
    if (dc->tbflags & META_TBFLAG_TREN) {
        uint16_t header = meta_ttevent_header(dc, META_TRACE_MOV_TTMARK, 0);
        meta_gen_ttevent1(dc, header, reg);
    }
}

static void meta_gen_tt_set(DisasContext *dc, MetaUnit unit, int id, TCGv dl)
{
    if (dc->tbflags & META_TBFLAG_TREN) {
        uint16_t header = meta_ttevent_header(dc, META_TRACE_MOV_TTDREG, 0xc0 | id);
        meta_gen_ttevent1(dc, header, dl);
    }
}

static void meta_gen_ttrec_setl(DisasContext *dc, MetaUnit ud, int rd,
                                MetaUnit us, int rs, TCGv dl, TCGv dh)
{
    if (dc->tbflags & META_TBFLAG_TREN) {
        uint16_t header = meta_ttevent_header(dc, META_TRACE_MOVL_TTREC,
                                              (us << 5) | rs);
        meta_gen_ttevent2(dc, header, dl, dh);
    }
}

static MetaRegHandler meta_trace_handlers[META_TT_MAX] = {
    /*
     * Privileges aren't actually this simple. It depends on TXMASKI.HALT and
     * the reason codes vary by register. For now though we'll settle for
     * blanket priv required (except TTMARK and TTREC).
     */
    [META_TTEXEC] = {
        UNIMPLEMENTED_REGHANDLER_WRITE,
        .priv = META_REG_PRIV(RW, ALWAYS),
    },
    [META_TTCTRL] = {
        UNIMPLEMENTED_REGHANDLER_WRITE,
        .priv = META_REG_PRIV(RW, ALWAYS),
    },
    [META_TTMARK] = {
        .gen_write = meta_gen_ttmark_set,
    },
    [META_TTREC] = {
        .gen_write = meta_gen_tt_set,
        .gen_writel = meta_gen_ttrec_setl,
    },
    [META_GTEXEC] = {
        UNIMPLEMENTED_REGHANDLER_WRITE,
        .priv = META_REG_PRIV(RW, ALWAYS),
    },
};

#else /* !CONFIG_USER_ONLY */

#define meta_trace_handlers NULL

#endif /* CONFIG_USER_ONLY */

typedef struct MetaUnitHandler {
    MetaRegHandler *handlers;
    /* privilege requirement for local [0] and global [1] registers */
    uint32_t priv[2];
} MetaUnitHandler;

MetaUnitHandler meta_reg_handlers[META_UNIT_MAX] = {
    [META_UNIT_CT] = {
        meta_ct_handlers,
    },
    [META_UNIT_D0 ... META_UNIT_A1] = {
        /* global data/address registers can be protected */
        /* FIXME most of the data/address unit instructions bypass this priv */
        .priv = { [1] = META_REG_PRIV_REGBIT(RW, TXPRIVEXT, GCR) },
    },
    [META_UNIT_PC] = {
        meta_pc_handlers,
    },
    [META_UNIT_TR] = {
        meta_trig_handlers,
        /* the whole trigger unit is protected by TXPRIVEXT.TrigPriv */
        { [0] = META_REG_PRIV_REGBIT(RW, TXPRIVEXT, TRIG)
                | META_REG_PRIV_EXCEPT_MASK },
    },
    [META_UNIT_RA] = {
        meta_port_handlers,
    },
    [META_UNIT_TT] = {
        meta_trace_handlers,
    },
};

/* Read an internal register (called by TXUXXRX register port) */
uint32_t meta_core_intreg_read(CPUArchState *env, MetaUnit unit, int id)
{
    MetaRegHandler *handler;
    int gp;
    if (unlikely(unit >= META_UNIT_MAX ||
                 id >= meta_unit_registers(unit))) {
        DBGLOG("attempt to read T%d R%d.%d out of range\n",
               env->thread_num,
               unit, id);
        return 0;
    }
    if (meta_reg_handlers[unit].handlers) {
        handler = &meta_reg_handlers[unit].handlers[id];
        if (likely(handler->read)) {
            return handler->read(env);
        }
        gp = !handler->gen_read;
    } else {
        gp = 1;
    }
    if (gp) {
        uint32_t *regp = meta_raw_get_reg(env, unit, id);
        if (regp) {
            return *regp;
        }
    }
    DBGLOG("attempt to read T%d %s unimplemented\n",
           env->thread_num,
           meta_reg_name(unit, id));
    return 0;
}

/* Write an internal register (called by TXUXXRX register port) */
void meta_core_intreg_write(CPUArchState *env, MetaUnit unit, int id, uint32_t val)
{
    MetaRegHandler *handler;
    int gp;
    if (unlikely(unit >= META_UNIT_MAX ||
                 id >= meta_unit_registers(unit))) {
        DBGLOG("attempt to write T%d R%d.%d out of range\n",
               env->thread_num,
               unit, id);
        return;
    }
    if (meta_reg_handlers[unit].handlers) {
        handler = &meta_reg_handlers[unit].handlers[id];
        if (likely(handler->write)) {
            handler->write(env, val);
            return;
        }
        gp = !handler->gen_write;
    } else {
        gp = 1;
    }
    if (gp) {
        uint32_t *regp = meta_raw_get_reg(env, unit, id);
        if (regp) {
            *regp = val;
            return;
        }
    }
    DBGLOG("attempt to write T%d %s unimplemented\n",
           env->thread_num,
           meta_reg_name(unit, id));
    return;
}

/* handle priv for a register access, and return 1 if priv halt known */
static int meta_gen_reg_priv(DisasContext *dc, int write,
                                const char *name, uint32_t priv)
{
    if (write && !(priv & META_REG_PRIV_WRITE_MASK)) {
        return 0;
    }
    if (!write && !(priv & META_REG_PRIV_READ_MASK)) {
        return 0;
    }
    if (!(dc->tbflags & META_TBFLAG_PSTAT)) {
        switch (priv & META_REG_PRIV_WHEN_MASK) {
        case META_REG_PRIV_WHEN_ALWAYS:
            /* privilege always required */
            return meta_privd(dc, name);

        case META_REG_PRIV_WHEN_TXPRIVEXT:
            /* whether previlege is required depends on a bit in TXPRIVEXT */
            meta_priv_check(dc, cpu_cregs[META_TXPRIVEXT],
                            1 << ((priv & META_REG_PRIV_BIT_MASK)
                                       >> META_REG_PRIV_BIT_SHIFT),
                            name);
            break;
        }
    }
    return 0;
}

static int meta_gen_reg_read(DisasContext *dc, MetaUnit unit, int id, TCGv dst)
{
    MetaUnitHandler *unit_handler;
    MetaRegHandler *handler;
    uint32_t unit_priv;
    uint32_t reg_priv;
    TCGv reg;
    int global;
    int exception;

    /* catch out the uncommon case of a bad register */
    if (unlikely(meta_get_reg(dc, unit, id, &reg))) {
        return 1;
    }

    /* handle unit priv checking */
    global = meta_reg_global(unit, id);
    unit_handler = &meta_reg_handlers[unit];
    unit_priv = unit_handler->priv[global];
    exception = dc->reg_priv_exception &&
                (unit_priv & META_REG_PRIV_EXCEPT_MASK);
    if (unit_priv && !exception && meta_gen_reg_priv(dc, 0,
                        meta_unit_names[unit], unit_priv)) {
        return 1;
    }

    if (unit_handler->handlers) {
        /* handle register priv checking */
        handler = &unit_handler->handlers[id];
        reg_priv = handler->priv;
        /* if the unit is excepted, but not the register, redo unit priv */
        if (exception) {
            if (!(reg_priv & META_REG_PRIV_EXCEPT_MASK)) {
                if (unit_priv && meta_gen_reg_priv(dc, 0,
                                    meta_unit_names[unit], unit_priv)) {
                    return 1;
                }
                exception = 0;
            }
        }
        if (reg_priv && !exception && meta_gen_reg_priv(dc, 0,
                            meta_reg_name(unit, id), reg_priv)) {
            return 1;
        }

        /* try using the gen_read callback to generate the code for us */
        if (handler->gen_read) {
            handler->gen_read(dc, unit, id, dst);
            return 0;
        }

        /* try generating code to call the read function */
        if (handler->read) {
            gen_helper_reg_read(dst, tcg_env, tcg_constant_ptr(handler->read));
            return 0;
        }
    }

    /* general purpose register, do the mov directly */
    tcg_gen_mov_i32(dst, reg);
    return 0;
}

/* src must be a local temp if privilege checking may be required */
static int meta_gen_reg_write(DisasContext *dc, MetaUnit unit, int id, TCGv src)
{
    MetaUnitHandler *unit_handler;
    MetaRegHandler *handler;
    TCGv reg;
    int global;

    /* catch out the uncommon case of a bad register */
    if (unlikely(meta_get_reg(dc, unit, id, &reg))) {
        return 1;
    }

    /* handle unit priv checking */
    global = meta_reg_global(unit, id);
    unit_handler = &meta_reg_handlers[unit];
    if (unit_handler->priv[global] && meta_gen_reg_priv(dc, 1,
                        meta_unit_names[unit], unit_handler->priv[global])) {
        return 1;
    }

    if (meta_reg_handlers[unit].handlers) {
        /* handle register priv checking */
        handler = &unit_handler->handlers[id];
        if (handler->priv && meta_gen_reg_priv(dc, 1,
                            meta_reg_name(unit, id), handler->priv)) {
            return 1;
        }

        /* try using the gen_write callback to generate the code for us */
        if (handler->gen_write) {
            handler->gen_write(dc, unit, id, src);
            return 0;
        }

        /* try generating code to call the write function */
        if (handler->write) {
            gen_helper_reg_write(tcg_env, tcg_constant_ptr(handler->write), src);
            return 0;
        }
    }

    /* general purpose register, do the mov directly */
    tcg_gen_mov_i32(reg, src);
    return 0;
}

/* src must be a local temp if privilege checking may be required */
static int meta_gen_reg_writel(DisasContext *dc,
                               MetaUnit unit, int id,
                               MetaUnit us, int rs,
                               TCGv lo, TCGv hi)
{
    MetaUnitHandler *unit_handler;
    MetaRegHandler *handler;
    TCGv reg;
    int global;

    /* catch out the uncommon case of a bad register */
    if (unlikely(meta_get_reg(dc, unit, id, &reg))) {
        return 1;
    }

    /* handle unit priv checking */
    global = meta_reg_global(unit, id);
    unit_handler = &meta_reg_handlers[unit];
    if (unit_handler->priv[global] && meta_gen_reg_priv(dc, 1,
                        meta_unit_names[unit], unit_handler->priv[global])) {
        return 1;
    }

    if (meta_reg_handlers[unit].handlers) {
        /* handle register priv checking */
        handler = &unit_handler->handlers[id];
        if (handler->priv && meta_gen_reg_priv(dc, 1,
                            meta_reg_name(unit, id), handler->priv)) {
            return 1;
        }

        /* try using the gen_writel callback to generate the code for us */
        if (handler->gen_writel) {
            handler->gen_writel(dc, unit, id, us, rs, lo, hi);
            return 0;
        }

        /* try generating code to call the writel function */
        if (handler->writel) {
            gen_helper_reg_writel(tcg_env, tcg_constant_ptr(handler->writel), lo, hi);
            return 0;
        }
    }

    /* can't do this general purpose, no registers are actually 64bit */
    meta_illegal(dc, "invalid writel register");
    return 0;
}

static int meta_gen_reg_write_imm(DisasContext *dc, MetaUnit unit, int id,
                                  uint32_t imm)
{
    MetaUnitHandler *unit_handler;
    MetaRegHandler *handler;
    TCGv reg;
    int global;

    /* catch out the uncommon case of a bad register */
    if (unlikely(meta_get_reg(dc, unit, id, &reg))) {
        return 1;
    }

    /* handle unit priv checking */
    global = meta_reg_global(unit, id);
    unit_handler = &meta_reg_handlers[unit];
    if (unit_handler->priv[global] && meta_gen_reg_priv(dc, 1,
                        meta_unit_names[unit], unit_handler->priv[global])) {
        return 1;
    }

    if (meta_reg_handlers[unit].handlers) {
        /* handle register priv checking */
        handler = &unit_handler->handlers[id];
        if (handler->priv && meta_gen_reg_priv(dc, 1,
                            meta_reg_name(unit, id), handler->priv)) {
            return 1;
        }

        /*
         * try using the gen_write_imm callback to generate optimised code for
         * us
         */
        if (handler->gen_write_imm) {
            handler->gen_write_imm(dc, unit, id, imm);
            return 0;
        }

        /* try using the gen_write callback to generate the code for us */
        if (handler->gen_write) {
            reg = meta_const_i32(imm);
            handler->gen_write(dc, unit, id, reg);
            return 0;
        }

        /* try generating code to call the write function */
        if (handler->write) {
            gen_helper_reg_write(tcg_env, tcg_constant_ptr(handler->write),
                                 tcg_constant_i32(imm));
            return 0;
        }
    }

    /* general purpose register, do the mov directly */
    tcg_gen_movi_i32(reg, imm);
    return 0;
}

static bool meta_reg_req_readwrite(DisasContext *dc, MetaUnit unit, int id, bool write)
{
    MetaUnitHandler *unit_handler;
    MetaRegHandler *handler;
    uint32_t unit_priv;
    uint32_t reg_priv;
    TCGv reg;
    int global;
    int exception;

    /* catch out the uncommon case of a bad register */
    if (unlikely(meta_get_reg(dc, unit, id, &reg))) {
        /* something's gone horribly wrong */
        return false;
    }

    /* handle unit priv checking */
    global = meta_reg_global(unit, id);
    unit_handler = &meta_reg_handlers[unit];
    unit_priv = unit_handler->priv[global];
    exception = dc->reg_priv_exception &&
                (unit_priv & META_REG_PRIV_EXCEPT_MASK);
    if (unit_priv && !exception &&
        ((write && (unit_priv & META_REG_PRIV_WRITE_MASK)) ||
         (!write && (unit_priv & META_REG_PRIV_READ_MASK)))) {
        return true;
    }

    if (unit_handler->handlers) {
        /* handle register priv checking */
        handler = &unit_handler->handlers[id];
        reg_priv = handler->priv;

        if (reg_priv &&
            (!exception || !(reg_priv & META_REG_PRIV_EXCEPT_MASK)) &&
            ((write && (reg_priv & META_REG_PRIV_WRITE_MASK)) ||
             (!write && (reg_priv & META_REG_PRIV_READ_MASK)))) {
            return true;
        }

        if (!write && (handler->gen_read || handler->read)) {
            return true;
        }
        if (write && (handler->gen_write || handler->write)) {
            return true;
        }
    }

    return false;
}

/* set scc flag */
static void meta_gen_set_scc(DisasContext *dc, bool scc)
{
    if (!meta_dsp_supported(dc->env)) {
        /* no DSP support, don't bother */
        assert(!scc);
        return;
    }

    if (!!(dc->tbflags & META_TBFLAG_SCC) == !!scc) {
        /* same state, do nothing */
        return;
    }

    /* set SCC flag */
    tcg_gen_movi_i32(cpu_cf[META_CF_SCC], !!scc);

    /* update tbflags */
    if (scc) {
        dc->tbflags |= META_TBFLAG_SCC;
    } else {
        dc->tbflags &= ~META_TBFLAG_SCC;
    }
}

/* set zero and negative flags from the result */
static void meta_gen_set_cf_nz(TCGv dst)
{
    tcg_gen_setcondi_i32(TCG_COND_LT, cpu_cf[META_CF_N], dst, 0);
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_cf[META_CF_Z], dst, 0);
}

/* set zero and negative flags from an immediate result */
static void meta_gen_set_cf_nz_imm(int32_t dst)
{
    tcg_gen_movi_i32(cpu_cf[META_CF_N], dst < 0);
    tcg_gen_movi_i32(cpu_cf[META_CF_Z], dst == 0);
}

/* clear overflow and carry flags for AND/OR/XOR etc */
static void meta_gen_clear_cf_vc(void)
{
    tcg_gen_movi_i32(cpu_cf[META_CF_C], 0);
    tcg_gen_movi_i32(cpu_cf[META_CF_V], 0);
}

/* set overflow and carry flags for ADD registers (before)
 * clobbers Z flag */
static void meta_gen_set_cf_vc_add_pre_i32(TCGv src1, TCGv src2)
{
    /* unsigned, wraps if one is larger than ~other */
    /* C = src2 > 0xffffffff - src1 */
    TCGv temp = tcg_temp_new_i32();
    tcg_gen_not_i32(temp, src1);
    tcg_gen_setcond_i32(TCG_COND_GTU, cpu_cf[META_CF_C], src2, temp);

    /* signed,
     * wraps if operands have same sign and result has differnt sign
     * store !xor in Z, indicates if the sign bits are the same
     */
    tcg_gen_xor_i32(cpu_cf[META_CF_Z], src1, src2);
    tcg_gen_not_i32(cpu_cf[META_CF_Z], cpu_cf[META_CF_Z]);
}

/* set overflow and carry flags for ADD registers (after) */
static void meta_gen_set_cf_vc_add_post_i32(TCGv dst, TCGv src2)
{
    /* Z = ~(src1 ^ src2)
     * set V = (Z & (src2 ^ dst)) < 0
     */
    tcg_gen_xor_i32(cpu_cf[META_CF_V], dst, src2);
    tcg_gen_and_i32(cpu_cf[META_CF_V], cpu_cf[META_CF_V], cpu_cf[META_CF_Z]);
    tcg_gen_discard_i32(cpu_cf[META_CF_Z]);
    tcg_gen_setcondi_i32(TCG_COND_LT, cpu_cf[META_CF_V], cpu_cf[META_CF_V], 0);
}

/* set overflow and carry flags for SUB registers (before)
 * clobbers Z flag */
static void meta_gen_set_cf_vc_sub_pre_i32(TCGv src1, TCGv src2)
{
    /* unsigned, wraps if src1 < src2 */
    tcg_gen_setcond_i32(TCG_COND_LTU, cpu_cf[META_CF_C], src1, src2);

    /* signed,
     * wraps if operands have different signs and result same sign as src2.
     * store xor in Z, indicates if the sign bits differ
     */
    tcg_gen_xor_i32(cpu_cf[META_CF_Z], src1, src2);
}

/* set overflow and carry flags for SUB registers (after) */
static void meta_gen_set_cf_vc_sub_post_i32(TCGv dst, TCGv src2)
{
    /* Z = src1 ^ src2
     * set V = (Z & !(src2 ^ dst)) < 0
     */
    tcg_gen_xor_i32(cpu_cf[META_CF_V], dst, src2);
    tcg_gen_not_i32(cpu_cf[META_CF_V], cpu_cf[META_CF_V]);
    tcg_gen_and_i32(cpu_cf[META_CF_V], cpu_cf[META_CF_V], cpu_cf[META_CF_Z]);
    tcg_gen_discard_i32(cpu_cf[META_CF_Z]);
    tcg_gen_setcondi_i32(TCG_COND_LT, cpu_cf[META_CF_V], cpu_cf[META_CF_V], 0);
}

/* set overflow and carry flags for ADD zero with reg (NEG) */
static void meta_gen_set_cf_vc_addz(TCGv src2)
{
    /* dst = 0 - src2 */
    /* never wraps from 0 */
    tcg_gen_movi_i32(cpu_cf[META_CF_C], 0);
    tcg_gen_movi_i32(cpu_cf[META_CF_V], 0);
}

/* set overflow and carry flags for ADD zero with immediate (MOV) */
static void meta_gen_set_cf_vc_addzi(uint32_t src2)
{
    /* dst = 0 + src2 */
    /* never wraps from 0 */
    tcg_gen_movi_i32(cpu_cf[META_CF_C], 0);
    tcg_gen_movi_i32(cpu_cf[META_CF_V], 0);
}

/* set overflow and carry flags for CMP/SUB zero with reg (NEG) */
static void meta_gen_set_cf_vc_subz(TCGv src2)
{
    /* dst = 0 - src2 */
    /* unsigned, almost always wraps */
    tcg_gen_setcondi_i32(TCG_COND_NE, cpu_cf[META_CF_C], src2, 0);
    /* signed, only wraps at extreme negative */
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_cf[META_CF_V], src2, 0x80000000);
}

/* set overflow and carry flags for CMP/SUB zero with immediate (NEG) */
static void meta_gen_set_cf_vc_subzi(uint32_t src2)
{
    /* dst = 0 - src2 */
    /* unsigned, almost always wraps */
    tcg_gen_movi_i32(cpu_cf[META_CF_C], src2 != 0);
    /* signed, only wraps at extreme negative */
    tcg_gen_movi_i32(cpu_cf[META_CF_V], src2 == 0x80000000);
}

/* set overflow and carry flags for ADD immediate */
static void meta_gen_set_cf_vc_addi_i32(TCGv src1, uint32_t src2)
{
    /* dst = src1 + src2 */
    if (src2) {
        /* unsigned, wraps if src1 > ~src2 */
        tcg_gen_setcondi_i32(TCG_COND_GTU, cpu_cf[META_CF_C], src1, ~src2);
        /* signed overflow... */
        if ((int32_t)src2 >= 0) {
            /* wraps if src1+src2 > 0x7fffffff, src1 > 0x7fffffff - src2 */
            tcg_gen_setcondi_i32(TCG_COND_GT, cpu_cf[META_CF_V], src1,
                                 0x7fffffff - src2);
        } else {
            /* wraps if src1+src2 < -0x80000000, src1 < -0x80000000 - src2 */
            tcg_gen_setcondi_i32(TCG_COND_LT, cpu_cf[META_CF_V], src1,
                                 0x80000000 - src2);
        }
    } else {
        /* zero immediate operand, never wraps */
        tcg_gen_movi_i32(cpu_cf[META_CF_C], 0);
        tcg_gen_movi_i32(cpu_cf[META_CF_V], 0);
    }
}

/* set overflow and carry flags for CMP/SUB immediate */
static void meta_gen_set_cf_vc_subi_i32(TCGv src1, uint32_t src2)
{
    /* dst = src1 - src2 */
    /* unsigned, wraps if src2 is greater */
    tcg_gen_setcondi_i32(TCG_COND_LTU, cpu_cf[META_CF_C], src1, src2);
    /* signed overflow... */
    if ((int32_t)src2 >= 0) {
        /* wraps if src1-src2 < -0x80000000, src1 < -0x80000000 + src2 */
        tcg_gen_setcondi_i32(TCG_COND_LT, cpu_cf[META_CF_V], src1,
                             0x80000000 + src2);
    } else {
        /* wraps if src1-src2 > 0x7fffffff, src1 > 0x7fffffff + src2 */
        tcg_gen_setcondi_i32(TCG_COND_GT, cpu_cf[META_CF_V], src1,
                             0x7fffffff + src2);
    }
}

/* set all flags for CMP/SUB immediate */
static void meta_gen_set_cf_subi(TCGv src1, uint32_t src2)
{
    TCGv temp = tcg_temp_new_i32();
    meta_gen_set_cf_vc_subi_i32(src1, src2);
    /* do the sub to find neg flag */
    tcg_gen_subi_i32(temp, src1, src2);
    meta_gen_set_cf_nz(temp);
}

/* set all flags for TST/AND immediate */
static void meta_gen_set_cf_andi(TCGv src1, uint32_t src2)
{
    TCGv temp;

    /* we can work out negative from the inputs */
    if (src2 & 0x80000000) {
        tcg_gen_setcondi_i32(TCG_COND_LT, cpu_cf[META_CF_N], src1, 0);
    } else {
        tcg_gen_movi_i32(cpu_cf[META_CF_N], 0);
    }

    /* need to do the AND to find zero flag */
    temp = tcg_temp_new_i32();
    tcg_gen_andi_i32(temp, src1, src2);
    if (!src2) {
        tcg_gen_movi_i32(cpu_cf[META_CF_Z], 1);
    } else {
        tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_cf[META_CF_Z], temp, 0);
    }

    meta_gen_clear_cf_vc();
}

/* set overflow and carry flags for LSR/ASR */
static void meta_gen_set_cf_vc_shift(int sr, TCGv src1, TCGv masked_src2)
{
    TCGLabel *l1 = gen_new_label();

    tcg_gen_movi_i32(cpu_cf[META_CF_C], 0);

    /* if (masked_src2) */
    tcg_gen_brcondi_i32(TCG_COND_EQ, masked_src2, 0, l1);
    {
        TCGv temp = tcg_temp_new();
        TCGv one = meta_const_i32(1);
        if (sr) {
            /* if (src1 & (1 << (src2-1))) */
            tcg_gen_subi_i32(temp, masked_src2, 1);
            tcg_gen_shl_i32(temp, one, temp);
            tcg_gen_and_i32(temp, src1, temp);
        } else {
            /* if (src1 & (1 << (32-src2))) */
            tcg_gen_subfi_i32(temp, 32, masked_src2);
            tcg_gen_shl_i32(temp, one, temp);
            tcg_gen_and_i32(temp, src1, temp);
        }
        tcg_gen_setcondi_i32(TCG_COND_NE, cpu_cf[META_CF_C], temp, 0);
    }
    gen_set_label(l1);
    tcg_gen_movi_i32(cpu_cf[META_CF_V], 0);
}

/* set overflow and carry flags for LSR/ASR immediate */
static void meta_gen_set_cf_vc_shifti(int sr, TCGv src, uint32_t imm)
{
    TCGv temp;

    /* carry set to bit imm-1 */
    if (imm) {
        temp = tcg_temp_new();
        tcg_gen_andi_i32(temp, src, (1 << (sr ? (imm-1) : (32-imm))));
        tcg_gen_setcondi_i32(TCG_COND_NE, cpu_cf[META_CF_C], temp, 0);
    } else {
        tcg_gen_movi_i32(cpu_cf[META_CF_C], 0);
    }
    tcg_gen_movi_i32(cpu_cf[META_CF_V], 0);
}

/* invert a condition code */
static MetaCc meta_cc_inv(MetaCc cc)
{
    /* increment, flip lsb, decrement, mask */
    return (((cc + 1) ^ 1) - 1) & 0xf;
}

/* branch to label if condition code PASSES */
static void meta_gen_br_cc_pass(DisasContext *dc, MetaCc cc, TCGLabel *label)
{
    TCGv temp;

    if (dc->tbflags & META_TBFLAG_SCC) {
        /* use split condition codes */
        switch ((MetaScc)cc) {
        case META_SCC_A: /* 1 */
            tcg_gen_br(label);
            break;
        case META_SCC_LEQ: /* Z_L */
            tcg_gen_brcondi_tl(TCG_COND_NE, cpu_cf[META_SCF_LZ], 0, label);
            break;
        case META_SCC_LNE: /* !Z_L */
            tcg_gen_brcondi_tl(TCG_COND_EQ, cpu_cf[META_SCF_LZ], 0, label);
            break;
        case META_SCC_LCS: /* C_L */
            tcg_gen_brcondi_tl(TCG_COND_NE, cpu_cf[META_SCF_LC], 0, label);
            break;
        case META_SCC_LCC: /* !C_L */
            tcg_gen_brcondi_tl(TCG_COND_EQ, cpu_cf[META_SCF_LC], 0, label);
            break;
        case META_SCC_HEQ: /* Z_H */
            tcg_gen_brcondi_tl(TCG_COND_NE, cpu_cf[META_SCF_HZ], 0, label);
            break;
        case META_SCC_HNE: /* !Z_H */
            tcg_gen_brcondi_tl(TCG_COND_EQ, cpu_cf[META_SCF_HZ], 0, label);
            break;
        case META_SCC_HCS: /* C_H */
            tcg_gen_brcondi_tl(TCG_COND_NE, cpu_cf[META_SCF_HC], 0, label);
            break;
        case META_SCC_HCC: /* !C_H */
            tcg_gen_brcondi_tl(TCG_COND_EQ, cpu_cf[META_SCF_HC], 0, label);
            break;
        case META_SCC_LHI: /* !(C_L | Z_L) */
            temp = tcg_temp_new_i32();
            tcg_gen_or_tl(temp, cpu_cf[META_SCF_LC], cpu_cf[META_SCF_LZ]);
            tcg_gen_brcondi_tl(TCG_COND_EQ, temp, 0, label);
            break;
        case META_SCC_LLS: /* C_L | Z_L */
            tcg_gen_brcondi_tl(TCG_COND_NE, cpu_cf[META_SCF_LC], 0, label);
            tcg_gen_brcondi_tl(TCG_COND_NE, cpu_cf[META_SCF_LZ], 0, label);
            break;
        case META_SCC_HHI: /* !(C_H | Z_H) */
            temp = tcg_temp_new_i32();
            tcg_gen_or_tl(temp, cpu_cf[META_SCF_HC], cpu_cf[META_SCF_HZ]);
            tcg_gen_brcondi_tl(TCG_COND_EQ, temp, 0, label);
            break;
        case META_SCC_HLS: /* C_H | Z_H */
            tcg_gen_brcondi_tl(TCG_COND_NE, cpu_cf[META_SCF_HC], 0, label);
            tcg_gen_brcondi_tl(TCG_COND_NE, cpu_cf[META_SCF_HZ], 0, label);
            break;
        case META_SCC_EEQ: /* Z_L | Z_H */
            tcg_gen_brcondi_tl(TCG_COND_NE, cpu_cf[META_SCF_LZ], 0, label);
            tcg_gen_brcondi_tl(TCG_COND_NE, cpu_cf[META_SCF_HZ], 0, label);
            break;
        case META_SCC_ECS: /* C_L | C_H */
            tcg_gen_brcondi_tl(TCG_COND_NE, cpu_cf[META_SCF_LC], 0, label);
            tcg_gen_brcondi_tl(TCG_COND_NE, cpu_cf[META_SCF_HC], 0, label);
            break;
        case META_SCC_NV: /* 0 */
        default:
            break;
        }
        return;
    }

    switch (cc) {
    case META_CC_A:  /* 1 */
        tcg_gen_br(label);
        break;
    case META_CC_EQ: /* Z */
        tcg_gen_brcondi_tl(TCG_COND_NE, cpu_cf[META_CF_Z], 0, label);
        break;
    case META_CC_NE: /* !Z */
        tcg_gen_brcondi_tl(TCG_COND_EQ, cpu_cf[META_CF_Z], 0, label);
        break;
    case META_CC_CS: /* C */
        tcg_gen_brcondi_tl(TCG_COND_NE, cpu_cf[META_CF_C], 0, label);
        break;
    case META_CC_CC: /* !C */
        tcg_gen_brcondi_tl(TCG_COND_EQ, cpu_cf[META_CF_C], 0, label);
        break;
    case META_CC_N: /* N */
        tcg_gen_brcondi_tl(TCG_COND_NE, cpu_cf[META_CF_N], 0, label);
        break;
    case META_CC_PL: /* !N */
        tcg_gen_brcondi_tl(TCG_COND_EQ, cpu_cf[META_CF_N], 0, label);
        break;
    case META_CC_VS: /* V */
        tcg_gen_brcondi_tl(TCG_COND_NE, cpu_cf[META_CF_V], 0, label);
        break;
    case META_CC_VC: /* !V */
        tcg_gen_brcondi_tl(TCG_COND_EQ, cpu_cf[META_CF_V], 0, label);
        break;
    case META_CC_HI: /* !(C | Z) */
        temp = tcg_temp_new_i32();
        tcg_gen_or_tl(temp, cpu_cf[META_CF_C], cpu_cf[META_CF_Z]);
        tcg_gen_brcondi_tl(TCG_COND_EQ, temp, 0, label);
        break;
    case META_CC_LS: /* C | Z */
        tcg_gen_brcondi_tl(TCG_COND_NE, cpu_cf[META_CF_C], 0, label);
        tcg_gen_brcondi_tl(TCG_COND_NE, cpu_cf[META_CF_Z], 0, label);
        break;
    case META_CC_GE: /* !(N ^ V) */
        temp = tcg_temp_new_i32();
        tcg_gen_xor_tl(temp, cpu_cf[META_CF_N], cpu_cf[META_CF_V]);
        tcg_gen_brcondi_tl(TCG_COND_EQ, temp, 0, label);
        break;
    case META_CC_LT: /* N ^ V */
        temp = tcg_temp_new_i32();
        tcg_gen_xor_tl(temp, cpu_cf[META_CF_N], cpu_cf[META_CF_V]);
        tcg_gen_brcondi_tl(TCG_COND_NE, temp, 0, label);
        break;
    case META_CC_GT: /* !(Z | (N ^ V)) */
        temp = tcg_temp_new_i32();
        tcg_gen_xor_tl(temp, cpu_cf[META_CF_N], cpu_cf[META_CF_V]);
        tcg_gen_or_tl(temp, temp, cpu_cf[META_CF_Z]);
        tcg_gen_brcondi_tl(TCG_COND_EQ, temp, 0, label);
        break;
    case META_CC_LE: /* Z | (N ^ V) */
        temp = tcg_temp_new_i32();
        tcg_gen_xor_tl(temp, cpu_cf[META_CF_N], cpu_cf[META_CF_V]);
        tcg_gen_or_tl(temp, temp, cpu_cf[META_CF_Z]);
        tcg_gen_brcondi_tl(TCG_COND_NE, temp, 0, label);
        break;
    case META_CC_NV: /* 0 */
    default:
        break;
    }
}

/* branch to label if floating point condition code PASSES */
static void meta_gen_br_fxcc_pass(DisasContext *dc, MetaFxCc cc, TCGLabel *label)
{
    TCGv temp;
    switch (cc) {
    case META_FXCC_A:  /* 1 */
        tcg_gen_br(label);
        break;
    case META_FXCC_FEQ: /* Z */
        tcg_gen_brcondi_tl(TCG_COND_NE, cpu_cf[META_CF_Z], 0, label);
        break;
    case META_FXCC_UNE: /* !Z */
        tcg_gen_brcondi_tl(TCG_COND_EQ, cpu_cf[META_CF_Z], 0, label);
        break;
    case META_FXCC_FLT: /* N */
    case META_FXCC_FMI: /* N */
        tcg_gen_brcondi_tl(TCG_COND_NE, cpu_cf[META_CF_N], 0, label);
        break;
    case META_FXCC_UGE: /* !N */
    case META_FXCC_UPL: /* !N */
        tcg_gen_brcondi_tl(TCG_COND_EQ, cpu_cf[META_CF_N], 0, label);
        break;
    case META_FXCC_UVS: /* V */
        tcg_gen_brcondi_tl(TCG_COND_NE, cpu_cf[META_CF_V], 0, label);
        break;
    case META_FXCC_FVC: /* !V */
        tcg_gen_brcondi_tl(TCG_COND_EQ, cpu_cf[META_CF_V], 0, label);
        break;
    case META_FXCC_UGT: /* !(N | Z) */
        temp = tcg_temp_new_i32();
        tcg_gen_or_tl(temp, cpu_cf[META_CF_N], cpu_cf[META_CF_Z]);
        tcg_gen_brcondi_tl(TCG_COND_EQ, temp, 0, label);
        break;
    case META_FXCC_FLE: /* N | Z */
        tcg_gen_brcondi_tl(TCG_COND_NE, cpu_cf[META_CF_N], 0, label);
        tcg_gen_brcondi_tl(TCG_COND_NE, cpu_cf[META_CF_Z], 0, label);
        break;
    case META_FXCC_FGE: /* !(N ^ V) */
        temp = tcg_temp_new_i32();
        tcg_gen_xor_tl(temp, cpu_cf[META_CF_N], cpu_cf[META_CF_V]);
        tcg_gen_brcondi_tl(TCG_COND_EQ, temp, 0, label);
        break;
    case META_FXCC_ULT: /* N ^ V */
        temp = tcg_temp_new_i32();
        tcg_gen_xor_tl(temp, cpu_cf[META_CF_N], cpu_cf[META_CF_V]);
        tcg_gen_brcondi_tl(TCG_COND_NE, temp, 0, label);
        break;
    case META_FXCC_FGT: /* !(Z | (N ^ V)) */
        temp = tcg_temp_new_i32();
        tcg_gen_xor_tl(temp, cpu_cf[META_CF_N], cpu_cf[META_CF_V]);
        tcg_gen_or_tl(temp, temp, cpu_cf[META_CF_Z]);
        tcg_gen_brcondi_tl(TCG_COND_EQ, temp, 0, label);
        break;
    case META_FXCC_ULE: /* Z | (N ^ V) */
        temp = tcg_temp_new_i32();
        tcg_gen_xor_tl(temp, cpu_cf[META_CF_N], cpu_cf[META_CF_V]);
        tcg_gen_or_tl(temp, temp, cpu_cf[META_CF_Z]);
        tcg_gen_brcondi_tl(TCG_COND_NE, temp, 0, label);
        break;
    case META_FXCC_NV: /* 0 */
    default:
        break;
    }
}

/* branch to label if paired floating point condition code PASSES */
static void meta_gen_br_fxpcc_pass(DisasContext *dc, MetaFxCc cc, TCGLabel *label)
{
    switch (cc) {
    case META_FXPCC_A:  /* 1 */
        tcg_gen_br(label);
        break;
    case META_FXPCC_LEQ: /* Z_L */
    case META_FXPCC_LNE: /* !Z_L */
    case META_FXPCC_LLO: /* N_L */
    case META_FXPCC_LHS: /* !N_L */
    case META_FXPCC_HEQ: /* Z_H */
    case META_FXPCC_HNE: /* !Z_H */
    case META_FXPCC_HLO: /* N_H */
    case META_FXPCC_HHS: /* !N_H */
    case META_FXPCC_LGR: /* !(N_L | Z_L) */
    case META_FXPCC_LLE: /* N_L | Z_L */
    case META_FXPCC_HGR: /* !(N_H | Z_H) */
    case META_FXPCC_HLE: /* N_H | Z_H */
    case META_FXPCC_EEQ: /* Z_L | Z_H */
    case META_FXPCC_ELO: /* N_L | N_H */
        meta_unimplemented(dc, "FXPCC");
        break;
    case META_FXPCC_NV: /* 0 */
    default:
        break;
    }
}

/* branch to label if condition code FAILS */
static inline void meta_gen_br_cc_fail(DisasContext *dc, MetaCc cc, TCGLabel *label)
{
    meta_gen_br_cc_pass(dc, meta_cc_inv(cc), label);
}

static inline void meta_gen_br_fxcc_fail(DisasContext *dc, MetaFxCc cc, TCGLabel *label)
{
    meta_gen_br_fxcc_pass(dc, meta_cc_inv((MetaCc)cc), label);
}

static inline void meta_gen_br_fxpcc_fail(DisasContext *dc, MetaFxPairedCc cc, TCGLabel *label)
{
    meta_gen_br_fxpcc_pass(dc, meta_cc_inv((MetaCc)cc), label);
}

/* start a conditional operation, returns label for meta_end_cc */
static inline void meta_start_cc(DisasContext *dc, MetaCc cc, MetaSafeLabel *l1)
{
    if (cc) {
        meta_init_safe_label(dc, l1);
        meta_gen_br_cc_fail(dc, cc, l1->label);
    }
}

static inline void meta_start_fxcc(DisasContext *dc, MetaFxCc cc, MetaSafeLabel *l1)
{
    if (cc) {
        meta_init_safe_label(dc, l1);
        meta_gen_br_fxcc_fail(dc, cc, l1->label);
    }
}

static inline void meta_start_fxpcc(DisasContext *dc, MetaFxPairedCc cc, MetaSafeLabel *l1)
{
    if (cc) {
        meta_init_safe_label(dc, l1);
        meta_gen_br_fxpcc_fail(dc, cc, l1->label);
    }
}

static inline void meta_end_cc(DisasContext *dc, MetaSafeLabel *l1)
{
    meta_set_safe_label(dc, 1, l1);
}

static TCGv_i64 meta_get_acc(DisasContext *dc, MetaUnit u, int idx)
{
    if (idx < 1) {
        /* local */
        return cpu_accregs[u - META_UNIT_D0][idx];
    }

    /* global */
    int t = dc->tbflags & META_TBFLAG_TX;
    return cpu_gaccregs[t][u - META_UNIT_D0][idx - 1];
}

static void meta_gen_addsub_sat32(DisasContext *dc,
                                  TCGv dst, TCGv result,
                                  TCGv src1, TCGv src2,
                                  bool sub, bool setflags)
{
    TCGv tmp1 = tcg_temp_new();
    TCGv tmp2 = tcg_temp_new();
    TCGv zero = meta_const_i32(0);

    tcg_gen_xor_i32(tmp1, src1, result);
    if (sub) {
        tcg_gen_not_i32(tmp2, src2);
        tcg_gen_xor_i32(tmp2, tmp2, result);
    } else {
        tcg_gen_not_i32(tmp2, src1);
        tcg_gen_xor_i32(tmp2, tmp2, src2);
    }
    tcg_gen_and_i32(tmp1, tmp1, tmp2);

    tcg_gen_sari_i32(tmp2, src1, 31);
    tcg_gen_xori_i32(tmp2, tmp2, (1u << 31) - 1);

    tcg_gen_movcond_i32(TCG_COND_LT, dst, tmp1, zero, tmp2, result);

    if (setflags) {
        tcg_gen_setcond_i32(TCG_COND_LT, cpu_cf[META_CF_V], tmp1, zero);
    }

}

static void meta_gen_addr_inc(DisasContext *dc, TCGv dst,
                              TCGv base, TCGv inc, MetaUnit ua)
{
    if ((ua == META_UNIT_A0 || ua == META_UNIT_A1) &&
        meta_au_nonlinear(dc, ua - META_UNIT_A0)) {
        TCGv rau = meta_const_i32(ua - META_UNIT_A0);
        gen_helper_au_add(dst, tcg_env, rau, base, inc);
    } else {
        tcg_gen_add_i32(dst, base, inc);
    }
}

static void meta_gen_addr_inci(DisasContext *dc, TCGv dst,
                               TCGv base, int32_t inc, MetaUnit ua)
{
    if ((ua == META_UNIT_A0 || ua == META_UNIT_A1) &&
        meta_au_nonlinear(dc, ua - META_UNIT_A0)) {
        TCGv rinc = meta_const_i32(inc);
        meta_gen_addr_inc(dc, dst, base, rinc, ua);
    } else {
        tcg_gen_addi_i32(dst, base, inc);
    }
}

static void meta_gen_fx_handle_exception(DisasContext *dc,
                                         const MetaFxInstInfo *info)
{
    TCGv r_op, r_args;

    r_op = meta_const_i32(info->op.raw);
    r_args = meta_const_i32(info->args.raw);
#ifdef CONFIG_USER_ONLY
    /* signal.c needs the correct PC for return */
    meta_gen_restore_state(dc);
#endif
    gen_helper_fx_handle_exception(tcg_env, r_op, r_args);

    /* this might trigger an interrupt */
    dc->is_jmp = DISAS_UPDATE;
}

static void meta_gen_trunc_i40_i32(DisasContext *dc, TCGv_i32 dst, TCGv_i64 src)
{
    TCGv_i64 tmp1, tmp2, want;

    if (!(dc->tbflags & META_TBFLAG_DUSATURATION)) {
        /* no saturation, straightforward */
        tcg_gen_extrl_i64_i32(dst, src);
        return;
    }

    tmp1 = tcg_temp_new_i64();
    tmp2 = tcg_temp_new_i64();
    want = tcg_temp_new_i64();

    /* tmp1 = sign bit ? -1 : 0 */
    tcg_gen_shli_i64(tmp1, src, 24);
    tcg_gen_sari_i64(tmp1, tmp1, 63);

    /* want = desired top 9 bits */
    tcg_gen_andi_i64(want, tmp1, 0x1ffllu << 31);

    /* tmp2 = top 9 bits */
    tcg_gen_andi_i64(tmp2, src, 0x1ffllu << 31);

    /* tmp1 = sign bit ? 0x80000000 : 0x7fffffff */
    tcg_gen_xori_i64(tmp1, tmp1, 0x7fffffff);
    tcg_gen_andi_i64(tmp1, tmp1, 0xffffffff);

    /* choose whether to saturate or not */
    tcg_gen_movcond_i64(TCG_COND_NE, tmp1, tmp2, want, tmp1, src);

    /* truncate result */
    tcg_gen_extrl_i64_i32(dst, tmp1);

}

static TCGv meta_get_source(DisasContext *dc,
                            const MetaInstructionSource src,
                            bool *tmp)
{
    switch (src.type) {
    case SRC_IMM:
        *tmp = true;
        return meta_const_i32(src.i);

    case SRC_DSPRAM: {
            TCGv rspec = meta_const_i32(src.i);
            TCGv rdu = meta_const_i32(src.u - META_UNIT_D0);
            TCGv reg = tcg_temp_new();

            *tmp = true;
            gen_helper_dspram_read(reg, tcg_env, rdu, rspec);

            return reg;
        }

    case SRC_ACCUM: {
            TCGv_i64 acc = meta_get_acc(dc, src.u, src.i);
            TCGv reg = tcg_temp_new();
            *tmp = true;

            if (dc->tbflags & META_TBFLAG_DUACCSAT) {
                /* 32-bit mode, shr 8 */
                TCGv_i64 tmp64 = tcg_temp_new_i64();
                tcg_gen_shri_i64(tmp64, acc, 8);
                tcg_gen_extrl_i64_i32(reg, tmp64);
            } else {
                /* 40-bit, possibly saturate */
                meta_gen_trunc_i40_i32(dc, reg, acc);
            }
            return reg;
        }

    case SRC_REG:
        if (src.u == META_UNIT_RA) {
            TCGv reg = tcg_temp_new();
            *tmp = true;
            meta_gen_rd_read_i32(dc, reg);
            return reg;
        } else if (meta_reg_req_readwrite(dc, src.u, src.i, false)) {
            TCGv reg = tcg_temp_new();
            *tmp = true;
            if (meta_gen_reg_read(dc, src.u, src.i, reg)) {
                meta_unimplemented(dc, "failed register read");
            }
            return reg;
        } else {
            TCGv reg;
            int ret = meta_get_reg(dc, src.u, src.i, &reg);
            *tmp = false;
            if (ret) {
                meta_unimplemented(dc, "no source register");
                return NULL;
            }
            return reg;
        }

    default:
        assert(false);
    }

    /* never happens, but keep dumb compilers happy */
    assert(false);
    return NULL;
}

typedef struct {
    TCGv src1[2];
    bool src1_tmp[2];

    TCGv src2[2];
    bool src2_tmp[2];
} MetaSrcData;

static void meta_read_sources(DisasContext *dc, MetaSrcData *data,
                              const MetaInstruction *inst,
                              bool want_src1, bool want_src2)
{
    data->src1_tmp[0] = data->src2_tmp[0] = false;
    data->src1_tmp[1] = data->src2_tmp[1] = false;

#define GET_SRC(num) \
    if (want_src ## num) { \
        if (inst->dual && \
            (inst->src ## num.type == SRC_REG) && \
            (inst->src ## num.u == META_UNIT_RA)) { \
            /* 64-bit read port data, split between units */ \
            TCGv_i64 data64 = tcg_temp_new_i64(); \
            meta_gen_rd_read_i64(dc, data64); \
            data->src ## num[0] = tcg_temp_new(); \
            data->src ## num[1] = tcg_temp_new(); \
            data->src ## num ## _tmp[0] = true; \
            data->src ## num ## _tmp[1] = true; \
            tcg_gen_extr_i64_i32(data->src ## num[0], \
                                  data->src ## num[1], data64); \
        } else { \
            data->src ## num[0] = meta_get_source(dc, inst->src ## num, \
                    &data->src ## num ## _tmp[0]); \
            if (inst->dual) { \
                data->src ## num[1] = meta_get_source(dc, \
                        meta_hi_source(inst->src ## num), \
                        &data->src ## num ## _tmp[1]); \
            } \
        } \
    }

    GET_SRC(1)
    GET_SRC(2)

#undef GET_SRC
}

static void meta_cleanup_sources(DisasContext *dc, MetaSrcData *data,
                                 const MetaInstruction *inst)
{
    int di;

    for (di = 0; di < 2; di++) {
    }
}

static TCGv meta_get_dest(DisasContext *dc,
                          const MetaInstructionDest dst,
                          bool *temp)
{
    switch (dst.type) {
    case DST_DSPRAM:
    case DST_ACCUM:
        *temp = true;
        break;

    case DST_REG:
        if (dst.u == META_UNIT_RA) {
            *temp = true;
        } else {
            *temp = meta_reg_req_readwrite(dc, dst.u, dst.i, true);
        }
        break;

    default:
        assert(false);
    }

    if (!*temp) {
        TCGv reg;
        int ret = meta_get_reg(dc, dst.u, dst.i, &reg);
        if (!ret) {
            /* standard register */
            return reg;
        }
        /* uh-oh, carry on & let meta_set_dest try to handle this */
    }

    *temp = true;
    return tcg_temp_new();
}

static void meta_set_dest(DisasContext *dc,
                          const MetaInstructionDest dst,
                          TCGv val)
{
    switch (dst.type) {
    case DST_DSPRAM: {
            TCGv rdu = meta_const_i32(dst.u - META_UNIT_D0);
            TCGv rspec = meta_const_i32(dst.i);

            gen_helper_dspram_write(tcg_env, rdu, rspec, val);

            break;
        }

    case DST_ACCUM:
        meta_unimplemented(dc, "DST_ACCUM");
        break;

    case DST_REG:
        if (dst.u == META_UNIT_RA) {
            meta_port_gen_write[dst.i](dc, val,
                    readport_info[dst.i].width + readport_info[dst.i].heed_l1);
        } else {
            meta_gen_reg_write(dc, dst.u, dst.i, val);
        }
        break;

    default:
        assert(false);
    }
}

static void meta_dest_from_src(DisasContext *dc,
                               MetaInstructionDest *dst,
                               const MetaInstructionSource *src)
{
    dst->u = src->u;
    dst->i = src->i;

    switch (src->type) {
    default:
    case SRC_NONE:
        meta_illegal(dc, "destination from no source?");
        break;

    case SRC_IMM:
        meta_illegal(dc, "destination from immediate source?");
        break;

    case SRC_REG:
        dst->type = DST_REG;
        break;

    case SRC_ACCUM:
        dst->type = DST_ACCUM;
        break;

    case SRC_DSPREG:
        dst->type = DST_DSPREG;
        break;

    case SRC_DSPRAM:
        dst->type = DST_DSPRAM;
        break;
    }
}

static void meta_gen_dsp_round_hi16(DisasContext *dc, TCGv dst, TCGv src,
                                    bool check_saturated)
{
    TCGv tmp1 = tcg_temp_new();
    TCGv round = tcg_temp_new();

    /* (src & 0x18000) == 0x08000 */
    tcg_gen_shri_i32(round, src, 16);
    tcg_gen_not_i32(round, round);
    tcg_gen_shri_i32(tmp1, src, 15);
    tcg_gen_and_i32(round, round, tmp1);
    tcg_gen_andi_i32(round, round, 0x1);

    /* (src & 0xffff) > 0x8000 */
    tcg_gen_ext16u_i32(tmp1, src);
    tcg_gen_setcondi_i32(TCG_COND_GTU, tmp1, tmp1, 0x8000);
    tcg_gen_or_i32(round, round, tmp1);

    if (check_saturated) {
        /* (src & 0xffff0000) != 0x7fff0000 */
        tcg_gen_andi_i32(tmp1, src, 0xffff0000);
        tcg_gen_setcondi_i32(TCG_COND_NE, tmp1, tmp1, 0x7fff0000);
        tcg_gen_and_i32(round, round, tmp1);
    }

    tcg_gen_shli_i32(round, round, 16);
    tcg_gen_add_i32(dst, src, round);

}

static void meta_gen_addsub_pshift(DisasContext *dc, TCGv dst)
{
    if (dc->tbflags & META_TBFLAG_DUROUNDING) {
        TCGv round_bit = tcg_temp_new();
        tcg_gen_andi_i32(round_bit, dst, 0x1);
        tcg_gen_sari_i32(dst, dst, 1);
        tcg_gen_or_i32(dst, dst, round_bit);
    } else {
        tcg_gen_sari_i32(dst, dst, 1);
    }
}

static void meta_gen_dsp_add_i16(DisasContext *dc, bool sub,
                                 TCGv cf_c, TCGv cf_v, TCGv cf_n, TCGv cf_z,
                                 TCGv dst, TCGv src1, TCGv src2,
                                 bool pshift, bool overscale)
{
    bool saturate = dc->tbflags & META_TBFLAG_DUSATURATION;
    TCGv src1_ext, src2_ext, dst_pretrunc;

    src1_ext = tcg_temp_new();
    src2_ext = tcg_temp_new();
    dst_pretrunc = tcg_temp_new();
    tcg_gen_ext16u_i32(src1_ext, src1);
    tcg_gen_ext16u_i32(src2_ext, src2);

    if (sub) {
        tcg_gen_sub_i32(dst_pretrunc, src1_ext, src2_ext);
    } else {
        tcg_gen_add_i32(dst_pretrunc, src1_ext, src2_ext);
    }

    if (cf_c != NULL) {
        /* C=1 if result exceeds 16 bits */
        if (sub) {
            tcg_gen_setcond_i32(TCG_COND_GTU, cf_c, src2_ext, src1_ext);
        } else {
            tcg_gen_setcondi_i32(TCG_COND_GTU, cf_c, dst_pretrunc, 0xffff);
        }
    }

    if (cf_v != NULL) {
        if (sub) {
            gen_helper_sub16_overflow(cf_v, dst_pretrunc, src1_ext, src2_ext);
        } else {
            gen_helper_add16_overflow(cf_v, dst_pretrunc, src1_ext, src2_ext);
        }
    }

    if (saturate) {
        if (sub) {
            gen_helper_sub16_sat(dst, dst_pretrunc, src1_ext, src2_ext);
        } else {
            gen_helper_add16_sat(dst, dst_pretrunc, src1_ext, src2_ext);
        }
    } else {
        tcg_gen_ext16u_i32(dst, dst_pretrunc);
    }

    if (pshift) {
        gen_helper_addsub16_pshiftse(dst, dst);
        meta_gen_addsub_pshift(dc, dst);
    }


    if (cf_n != NULL) {
        tcg_gen_shri_i32(cf_n, dst, 15);
        tcg_gen_andi_i32(cf_n, cf_n, 0x1);
    }

    if (cf_z != NULL) {
        tcg_gen_setcondi_i32(TCG_COND_EQ, cf_z, dst, 0);
    }
}

static void meta_gen_dsp_add_i32(DisasContext *dc, bool sub, bool setflags,
                                 TCGv dst, TCGv src1, TCGv src2,
                                 bool pshift, bool heed_saturate)
{
    bool saturate = heed_saturate && (dc->tbflags & META_TBFLAG_DUSATURATION);
    TCGv dst_tmp = dst;

    if (setflags) {
        if (sub) {
            meta_gen_set_cf_vc_sub_pre_i32(src1, src2);
        } else {
            meta_gen_set_cf_vc_add_pre_i32(src1, src2);
        }
    }

    if (((dst == src1) && saturate) ||
        ((dst == src2) && (saturate || setflags))) {
        dst_tmp = tcg_temp_new();
    }

    if (sub) {
        tcg_gen_sub_i32(dst_tmp, src1, src2);
    } else {
        tcg_gen_add_i32(dst_tmp, src1, src2);
    }

    if (setflags) {
        if (sub) {
            meta_gen_set_cf_vc_sub_post_i32(dst_tmp, src2);
        } else {
            meta_gen_set_cf_vc_add_post_i32(dst_tmp, src2);
        }
    }

    if (saturate) {
        meta_gen_addsub_sat32(dc, dst_tmp, dst_tmp, src1, src2, sub, setflags);
    }

    if (pshift) {
        meta_gen_addsub_pshift(dc, dst_tmp);
    }

    if (setflags) {
        meta_gen_set_cf_nz(dst_tmp);
        meta_gen_set_scc(dc, false);
    }

    if (!(dst == dst_tmp)) {
        tcg_gen_mov_i32(dst, dst_tmp);
    }
}

static void meta_gen_dsp_add_i40(DisasContext *dc, bool sub,
                                 TCGv_i64 dst, TCGv_i64 src1, TCGv_i64 src2,
                                 bool heed_accsat)
{
    TCGv_i64 s1 = src1;
    TCGv_i64 s2 = src2;
    TCGv_i64 dst_add = dst;

    if (heed_accsat && (dc->tbflags & META_TBFLAG_DUACCSAT)) {
        s2 = tcg_temp_new_i64();
        tcg_gen_shli_i64(s2, src2, 8);
    }
    if ((dc->tbflags & META_TBFLAG_DUSATURATION) &&
        ((dst == s1) || (dst == s2))) {
        dst_add = tcg_temp_new_i64();
    }

    if (sub) {
        tcg_gen_sub_i64(dst_add, s1, s2);
    } else {
        tcg_gen_add_i64(dst_add, s1, s2);
    }

    if (dc->tbflags & META_TBFLAG_DUSATURATION) {
        /* 40-bit saturation */
        TCGv_i64 tmp1 = tcg_temp_new_i64();
        TCGv_i64 tmp2 = tcg_temp_new_i64();
        TCGv_i64 zero = meta_const_i64(0);

        /* tmp1 = whether to saturate */
        tcg_gen_xor_i64(tmp1, dst_add, s1);
        tcg_gen_xor_i64(tmp2, s1, s2);
        if (!sub) {
            tcg_gen_not_i64(tmp2, tmp2);
        }
        tcg_gen_and_i64(tmp1, tmp1, tmp2);
        tcg_gen_andi_i64(tmp1, tmp1, 0x8000000000llu);

        /* tmp2 = saturated value */
        tcg_gen_shli_i64(tmp2, s1, 24);
        tcg_gen_sari_i64(tmp2, tmp2, 63);
        tcg_gen_shri_i64(tmp2, tmp2, 24);
        tcg_gen_xori_i64(tmp2, tmp2, 0x7fffffffffllu);

        /* decide whether to saturate or not */
        tcg_gen_movcond_i64(TCG_COND_NE, dst, tmp1, zero, tmp2, dst_add);

    }

    /* sign extend 40-bits to 64-bits */
    tcg_gen_shli_i64(dst, dst, 24);
    tcg_gen_sari_i64(dst, dst, 24);

}

static void meta_gen_dsp_shift_trunc_i40_i32(DisasContext *dc,
                                             const MetaInstruction *inst,
                                             TCGv dst, TCGv_i64 val, TCGv_i64 rspp)
{
    const MetaInstructionShift *sh = &inst->shift;

    if (sh->arithmetic &&
        (dc->tbflags & META_TBFLAG_DUSATURATION)) {
        /* saturate 40-bit to 32-bit */
        TCGv_i64 tmp1 = tcg_temp_new_i64();
        TCGv_i64 wanted = tcg_temp_new_i64();
        TCGv_i64 satval = tcg_temp_new_i64();
        TCGv_i64 mask = meta_const_i64(0xff80000000ll);

        /* wanted = (val & 0x8000000000) ? 0xff80000000 : 0x0000000000 */
        tcg_gen_shli_i64(tmp1, val, 24);
        tcg_gen_sari_i64(wanted, tmp1, 8);
        tcg_gen_shri_i64(wanted, wanted, 24);
        tcg_gen_and_i64(wanted, wanted, mask);

        /* satval = (src & 0x8000000000) ? 0x80000000 : 0x7fffffff */
        tcg_gen_sari_i64(satval, tmp1, 31);
        tcg_gen_shri_i64(satval, satval, 32);
        tcg_gen_xori_i64(satval, satval, 0x7fffffff);

        /* tmp1 = src & mask */
        tcg_gen_and_i64(tmp1, val, mask);

        /* tmp1 = (tmp1 == wanted) ? src : satval */
        tcg_gen_movcond_i64(TCG_COND_EQ, tmp1, tmp1, wanted, rspp, satval);

        /* set the destination */
        tcg_gen_extrl_i64_i32(dst, tmp1);

    } else {
        /* truncate 40-bit to 32-bit */
        tcg_gen_extrl_i64_i32(dst, rspp);
    }
}

static void meta_gen_dsp_shift_left_i40(DisasContext *dc,
                                        const MetaInstruction *inst,
                                        TCGv dst, TCGv_i64 src1, TCGv src2)
{
    const MetaInstructionShift *sh = &inst->shift;
    TCGv_i64 tmp1 = tcg_temp_new_i64();
    TCGv_i64 tmp2 = tcg_temp_new_i64();

    /* extend source 1 as appropriate */
    tcg_gen_shli_i64(tmp1, src1, 24);
    if (sh->arithmetic) {
        tcg_gen_sari_i64(tmp1, tmp1, 24);
    } else {
        tcg_gen_shri_i64(tmp1, tmp1, 24);
    }

    /* extend source 2 */
    tcg_gen_extu_i32_i64(tmp2, src2);

    /* perform the shift */
    tcg_gen_shl_i64(tmp1, tmp1, tmp2);

    if (sh->setflags) {
        /* set the carry flag */
        tcg_gen_shri_i64(tmp2, tmp1, 32);
        tcg_gen_extrl_i64_i32(cpu_cf[META_CF_C], tmp2);
        tcg_gen_andi_i32(cpu_cf[META_CF_C], cpu_cf[META_CF_C], 0x1);
    }

    if (sh->arithmetic &&
        (dc->tbflags & META_TBFLAG_DUSATURATION)) {
        /* 40-bit saturation */
        gen_helper_saturate_asl_i40(tmp1, tmp1, src1, src2);
    }

    /* set the destination */
    meta_gen_dsp_shift_trunc_i40_i32(dc, inst, dst, tmp1, tmp1);

    if (sh->setflags) {
        tcg_gen_movi_i32(cpu_cf[META_CF_V], 0);
        meta_gen_set_cf_nz(dst);
        meta_gen_set_scc(dc, false);
    }

}

static void meta_gen_dsp_shift_right_i40(DisasContext *dc,
                                         const MetaInstruction *inst,
                                         TCGv dst, TCGv_i64 src1, TCGv src2)
{
    const MetaInstructionShift *sh = &inst->shift;
    TCGv_i64 tmp1 = tcg_temp_new_i64();
    TCGv_i64 tmp2 = tcg_temp_new_i64();
    TCGv_i64 src2_64 = tcg_temp_new_i64();
    TCGv_i64 carry = NULL;
    TCGv_i64 rspp_tmp;

    /* extend source 1 as appropriate */
    tcg_gen_shli_i64(tmp1, src1, 24);
    if (sh->arithmetic) {
        tcg_gen_sari_i64(tmp1, tmp1, 24);
    } else {
        tcg_gen_shri_i64(tmp1, tmp1, 24);
    }

    /* extend source 2 */
    tcg_gen_extu_i32_i64(src2_64, src2);

    /* perform the shift */
    if (sh->arithmetic) {
        /* assumes the accumulator is stored sign extended to 64-bits */
        tcg_gen_sar_i64(tmp1, tmp1, src2_64);
    } else {
        tcg_gen_shr_i64(tmp1, tmp1, src2_64);
    }

    if (sh->setflags || (sh->rspp == RSPP_ROUND)) {
        carry = tcg_temp_new_i64();
        tcg_gen_subi_i64(tmp2, src2_64, 1);
        if (sh->arithmetic) {
            /* assumes the accumulator is stored sign extended to 64-bits */
            tcg_gen_sar_i64(carry, src1, tmp2);
        } else {
            tcg_gen_shr_i64(carry, src1, tmp2);
        }
        tcg_gen_andi_i64(carry, carry, 0x1);

        /* carry always 0 if we shifted by 0 */
        tcg_gen_movi_i64(tmp2, 0);
        tcg_gen_movcond_i64(TCG_COND_EQ, carry, src2_64, tmp2, tmp2, carry);

        tcg_gen_extrl_i64_i32(cpu_cf[META_CF_C], carry);
    }

    rspp_tmp = tmp1;
    switch (sh->rspp) {
    case RSPP_ROUND:
        /* round by adding the carry bit */
        if (sh->arithmetic &&
            (dc->tbflags & META_TBFLAG_DUSATURATION)) {
            TCGv_i64 tmp3 = tcg_temp_new_i64();

            /* add carry to a temporary */
            tcg_gen_add_i64(tmp2, tmp1, carry);

            /* and use it if not saturated */
            tcg_gen_movi_i64(tmp3, 0x7fffffffllu);
            tcg_gen_movcond_i64(TCG_COND_NE, tmp2, tmp1, tmp3, tmp2, tmp1);

        } else {
            /* add carry directly */
            tcg_gen_add_i64(tmp2, tmp1, carry);
        }
        rspp_tmp = tmp2;
        break;

    case RSPP_SATS9:
        if (sh->arithmetic) {
            gen_helper_shift_rspp_sats9(tmp2, src1, src2, tmp1);
            rspp_tmp = tmp2;
        }
        break;

    case RSPP_SATU8:
        if (sh->arithmetic) {
            gen_helper_shift_rspp_satu8(tmp2, src1, src2, tmp1);
            rspp_tmp = tmp2;
        }
        break;

    default:
        break;
    }

    /* set the destination */
    meta_gen_dsp_shift_trunc_i40_i32(dc, inst, dst, tmp1, rspp_tmp);

    if (sh->setflags) {
        tcg_gen_movi_i32(cpu_cf[META_CF_V], 0);
        meta_gen_set_cf_nz(dst);
        meta_gen_set_scc(dc, false);
    }

}

static void meta_gen_dsp_shift_left_i32(DisasContext *dc,
                                        const MetaInstruction *inst,
                                        TCGv dst, TCGv src1, TCGv src2)
{
    const MetaInstructionShift *sh = &inst->shift;
    TCGv dst_shift = dst;

    if (sh->arithmetic &&
        (dc->tbflags & META_TBFLAG_DUSATURATION) &&
        ((dst == src1) ||
         (dst == src2))) {
        /* use a temp to preserve the source data */
        dst_shift = tcg_temp_new();
    }

    if (sh->setflags) {
        TCGv tmp1 = tcg_temp_new();

        tcg_gen_subfi_i32(tmp1, 32, src2);
        tcg_gen_shr_i32(tmp1, src1, tmp1);
        tcg_gen_andi_i32(cpu_cf[META_CF_C], tmp1, 0x1);

    }

    /* perform the shift */
    tcg_gen_shl_i32(dst_shift, src1, src2);

    if (sh->arithmetic &&
        (dc->tbflags & META_TBFLAG_DUSATURATION)) {
        TCGv tmp1 = tcg_temp_new();
        TCGv tmp2 = tcg_temp_new();
        TCGv zero = meta_const_i32(0);

        /* tmp1 = lost bits */
        tcg_gen_subfi_i32(tmp1, 31, src2);
        tcg_gen_sar_i32(tmp1, src1, tmp1);

        /* tmp1 = (tmp1 < 0) ? ~tmp1 : tmp1 */
        tcg_gen_not_i32(tmp2, tmp1);
        tcg_gen_movcond_i32(TCG_COND_LT, tmp1, tmp1, zero, tmp2, tmp1);

        /* tmp2 = saturated value */
        tcg_gen_sari_i32(tmp2, src1, 31);
        tcg_gen_xori_i32(tmp2, tmp2, 0x7fffffff);

        /* decide whether to saturate */
        tcg_gen_movcond_i32(TCG_COND_NE, dst, tmp1, zero, tmp2, dst_shift);


    }

    if (sh->setflags) {
        tcg_gen_movi_i32(cpu_cf[META_CF_V], 0);
        meta_gen_set_cf_nz(dst);
        meta_gen_set_scc(dc, false);
    }
}

static void meta_gen_dsp_shift_right_i32(DisasContext *dc,
                                         const MetaInstruction *inst,
                                         TCGv dst, TCGv src1, TCGv src2)
{
    const MetaInstructionShift *sh = &inst->shift;
    TCGv carry = NULL;

    if (sh->rspp == RSPP_ROUND) {
        /* want the carry bit, but without changing flags */
        carry = tcg_temp_new();
    }

    if (sh->setflags || (sh->rspp == RSPP_ROUND)) {
        TCGv tmp1 = tcg_temp_new();

        /* most significant lost bit */
        tcg_gen_subi_i32(tmp1, src2, 1);
        if (sh->arithmetic) {
            tcg_gen_sar_i32(tmp1, src1, tmp1);
        } else {
            tcg_gen_shr_i32(tmp1, src1, tmp1);
        }
        tcg_gen_andi_i32(cpu_cf[META_CF_C], tmp1, 0x1);

        if (sh->rspp == RSPP_ROUND) {
            /* and src2 != 0 */
            tcg_gen_setcondi_i32(TCG_COND_NE, tmp1, src2, 0);
            tcg_gen_and_i32(carry, cpu_cf[META_CF_C], tmp1);
        }

    }

    /* perform the shift */
    if (sh->arithmetic) {
        tcg_gen_sar_i32(dst, src1, src2);
    } else {
        tcg_gen_shr_i32(dst, src1, src2);
    }

    if (sh->rspp == RSPP_ROUND) {
        /* round by adding the carry bit */
        tcg_gen_add_i32(dst, dst, carry);
    } else if (sh->rspp) {
        /* saturate */
        int32_t max = 255;
        int32_t min = (sh->rspp == RSPP_SATS9) ? -256 : 0;
        TCGv tmp1 = tcg_temp_new();

        /* dst = MIN(dst, max) */
        tcg_gen_movi_i32(tmp1, max);
        tcg_gen_movcond_i32(TCG_COND_LT, dst, dst, tmp1, dst, tmp1);

        /* dst = MAX(dst, min) */
        tcg_gen_movi_i32(tmp1, min);
        tcg_gen_movcond_i32(TCG_COND_GT, dst, dst, tmp1, dst, tmp1);

    }

    if (sh->setflags) {
        tcg_gen_movi_i32(cpu_cf[META_CF_V], 0);
        meta_gen_set_cf_nz(dst);
        meta_gen_set_scc(dc, false);
    }
}

static void meta_gen_dsp_shift_left_i16(DisasContext *dc,
                                        const MetaInstruction *inst,
                                        TCGv dst, TCGv src1, TCGv src2,
                                        TCGv cf_c, TCGv cf_z)
{
    const MetaInstructionShift *sh = &inst->shift;
    TCGv dst_tmp = tcg_temp_new();

    if (sh->arithmetic &&
        (dc->tbflags & META_TBFLAG_DUSATURATION)) {
        /* we'll need the upper bits later */
        tcg_gen_ext16s_i32(dst_tmp, src1);
        tcg_gen_shl_i32(dst_tmp, dst_tmp, src2);
    } else {
        /* straightforward shift */
        tcg_gen_shl_i32(dst_tmp, src1, src2);
    }

    /* evaluate carry */
    if (cf_c != NULL) {
        tcg_gen_shri_i32(cf_c, dst_tmp, 16);
        tcg_gen_andi_i32(cf_c, cf_c, 0x1);
    }

    /* saturate */
    if (sh->arithmetic &&
        (dc->tbflags & META_TBFLAG_DUSATURATION)) {
        TCGv tmp1 = tcg_temp_new();
        TCGv tmp2 = tcg_temp_new();
        TCGv zero = meta_const_i32(0);

        /* tmp1 = dst_tmp >> 15
         * tmp1 = MAX(tmp1, ~tmp1);
         */
        tcg_gen_sari_i32(tmp1, dst_tmp, 15);
        tcg_gen_not_i32(tmp2, tmp1);
        tcg_gen_movcond_i32(TCG_COND_GT, tmp1, tmp1, tmp2, tmp1, tmp2);

        /* tmp2 = (((src1 << 16) >> 31) ^ 0x7fff) & 0xffff */
        tcg_gen_shli_i32(tmp2, src1, 16);
        tcg_gen_sari_i32(tmp2, tmp2, 31);
        tcg_gen_xori_i32(tmp2, tmp2, 0x7fff);

        /* dst_tmp = tmp1 ? tmp2 : dst_tmp */
        tcg_gen_movcond_i32(TCG_COND_NE, dst_tmp, tmp1, zero, tmp2, dst_tmp);

    }

    /* truncate result */
    tcg_gen_ext16u_i32(dst, dst_tmp);

    /* evaluate zero */
    if (cf_z != NULL) {
        tcg_gen_setcondi_i32(TCG_COND_EQ, cf_z, dst, 0);
    }
}

static void meta_gen_dsp_shift_right_i16(DisasContext *dc,
                                         const MetaInstruction *inst,
                                         TCGv dst, TCGv src1, TCGv src2,
                                         TCGv cf_c, TCGv cf_z)
{
    const MetaInstructionShift *sh = &inst->shift;
    TCGv tmp_c = NULL;

    /* force carry flag evaluation for rspp rounds */
    if ((sh->rspp == RSPP_ROUND) &&
        (cf_c == NULL)) {
        cf_c = tmp_c = tcg_temp_new();
    }

    /* evaluate carry */
    if (cf_c != NULL) {
        TCGv tmp1 = tcg_temp_new();

        /* C = (src1 >> (src2 - 1)) & 0x1 */
        tcg_gen_subi_i32(tmp1, src2, 1);
        tcg_gen_shr_i32(tmp1, src1, tmp1);
        tcg_gen_andi_i32(cf_c, tmp1, 0x1);

        /* C &= !!src2 */
        tcg_gen_setcondi_i32(TCG_COND_NE, tmp1, src2, 0);
        tcg_gen_and_i32(cf_c, cf_c, tmp1);

    }

    /* sign extend if necessary, shift */
    if (sh->arithmetic) {
        TCGv tmp1 = tcg_temp_new();
        tcg_gen_ext16s_i32(tmp1, src1);
        tcg_gen_shr_i32(dst, tmp1, src2);
    } else {
        tcg_gen_shr_i32(dst, src1, src2);
    }

    /* truncate result */
    tcg_gen_ext16u_i32(dst, dst);

    /* post-process */
    switch (sh->rspp) {
    case RSPP_SATU8:
        if (sh->arithmetic) {
            TCGv tmp1 = tcg_temp_new();
            TCGv tmp2 = tcg_temp_new();

            /* tmp1 = ~((dst << 16) >> 31) */
            tcg_gen_shli_i32(tmp1, dst, 16);
            tcg_gen_sari_i32(tmp1, tmp1, 31);
            tcg_gen_not_i32(tmp1, tmp1);

            /* tmp2 = MIN(dst, 0xff) */
            tcg_gen_movi_i32(tmp2, 0xff);
            tcg_gen_movcond_i32(TCG_COND_LTU, tmp2, dst, tmp2, dst, tmp2);

            /* dst = tmp1 & tmp2 */
            tcg_gen_and_i32(dst, tmp1, tmp2);

        }
        break;

    case RSPP_SATS9:
        if (sh->arithmetic) {
            TCGv tmp1 = tcg_temp_new();

            /* dst = (int32_t)dst */
            tcg_gen_ext16s_i32(dst, dst);

            /* dst = MIN(dst, 255) */
            tcg_gen_movi_i32(tmp1, 255);
            tcg_gen_movcond_i32(TCG_COND_LT, dst, tmp1, dst, tmp1, dst);

            /* dst = MAX(dst, -256) */
            tcg_gen_movi_i32(tmp1, -256);
            tcg_gen_movcond_i32(TCG_COND_GT, dst, tmp1, dst, tmp1, dst);

            /* truncate result */
            tcg_gen_ext16u_i32(dst, dst);

        }
        break;

    case RSPP_ROUND:
        tcg_gen_add_i32(dst, dst, cf_c);
        tcg_gen_ext16u_i32(dst, dst);
        break;

    default:
        /* NOP */
        break;
    }

    /* evaluate zero */
    if (cf_z != NULL) {
        tcg_gen_setcondi_i32(TCG_COND_EQ, cf_z, dst, 0);
    }

    /* cleanup */
}

static void meta_gen_dsp_mul_i32(DisasContext *dc,
                                 const MetaInstruction *inst,
                                 TCGv_i64 dst, TCGv src1, TCGv src2,
                                 bool heed_prodshift, bool heed_saturation,
                                 bool heed_round)
{
    TCGv_i64 tmp1 = tcg_temp_new_i64();
    TCGv_i64 tmp2 = tcg_temp_new_i64();

    /* extend sources appropriately to 64-bits */
    if (inst->mul.sign) {
        tcg_gen_ext_i32_i64(tmp1, src1);
        tcg_gen_ext_i32_i64(tmp2, src2);
    } else {
        tcg_gen_extu_i32_i64(tmp1, src1);
        tcg_gen_extu_i32_i64(tmp2, src2);
    }

    /* perform the multiply */
    tcg_gen_mul_i64(dst, tmp1, tmp2);

    if (heed_prodshift && (dc->tbflags & META_TBFLAG_DUPRODSHIFT)) {
        if (heed_saturation &&
            inst->mul.sign &&
            (dc->tbflags & META_TBFLAG_DUSATURATION)) {
            /* signed 32x32 saturation */
            TCGv_i64 tmp3 = meta_const_i64(0x7fffffffffffffffllu);
            tcg_gen_shli_i64(tmp1, dst, 1);
            tcg_gen_movi_i64(tmp2, 0x4000000000000000llu);
            tcg_gen_movcond_i64(TCG_COND_EQ, dst, dst, tmp2, tmp3, tmp1);
        } else {
            tcg_gen_shli_i64(dst, dst, 1);
        }
    }

    if (heed_round && (dc->tbflags & META_TBFLAG_DUROUNDING)) {
        /* (dst & 0x180000000) == 0x080000000 */
        tcg_gen_shri_i64(tmp1, dst, 31);
        tcg_gen_shri_i64(tmp2, dst, 32);
        tcg_gen_not_i64(tmp2, tmp2);
        tcg_gen_and_i64(tmp1, tmp1, tmp2);
        tcg_gen_andi_i64(tmp1, tmp1, 0x1);

        /* (dst & 0xffffffff) > 0x80000000 */
        tcg_gen_ext32u_i64(tmp2, dst);
        tcg_gen_setcondi_i64(TCG_COND_GTU, tmp2, tmp2, 0x80000000);
        tcg_gen_or_i64(tmp1, tmp1, tmp2);

        if (inst->mul.sign) {
            /* ((unsigned)dst >> 32) != 0x7fffffff */
            tcg_gen_shri_i64(tmp2, dst, 32);
            tcg_gen_setcondi_i64(TCG_COND_NE, tmp2, tmp2, 0x7fffffff);
            tcg_gen_and_i64(tmp1, tmp1, tmp2);
        }

        /* increment top if required */
        tcg_gen_shli_i64(tmp1, tmp1, 32);
        tcg_gen_add_i64(dst, dst, tmp1);
    }

}

static void meta_gen_dsp_mul_i31(DisasContext *dc,
                                 const MetaInstruction *inst,
                                 TCGv dst, TCGv src1, TCGv src2,
                                 bool heed_prodshift, bool heed_saturation,
                                 bool heed_round, bool is_signed)
{
    TCGv s1l = tcg_temp_new();
    TCGv s1h = tcg_temp_new();
    TCGv s2l = tcg_temp_new();
    TCGv s2h = tcg_temp_new();
    TCGv tmp1 = tcg_temp_new();
    TCGv tmp2 = tcg_temp_new();
    TCGv tmp3 = tcg_temp_new();
    TCGv mid_sum = tcg_temp_new();

    /* extract values */
    tcg_gen_andi_i32(s1l, src1, 0xfffe);
    tcg_gen_andi_i32(s2l, src2, 0xfffe);

    if (is_signed) {
        tcg_gen_sari_i32(s1h, src1, 16);
        tcg_gen_sari_i32(s2h, src2, 16);
    } else {
        tcg_gen_shri_i32(s1h, src1, 16);
        tcg_gen_shri_i32(s2h, src2, 16);
    }

    /* arithmetic */
    tcg_gen_mul_i32(tmp1, s1l, s2h);
    tcg_gen_ext16u_i32(dst, tmp1);
    tcg_gen_mul_i32(tmp2, s2l, s1h);
    tcg_gen_ext16u_i32(tmp3, tmp2);
    tcg_gen_add_i32(mid_sum, dst, tmp3);
    tcg_gen_shri_i32(dst, mid_sum, 16);
    tcg_gen_mul_i32(tmp3, s1h, s2h);

    if (is_signed) {
        tcg_gen_sari_i32(tmp1, tmp1, 16);
        tcg_gen_sari_i32(tmp2, tmp2, 16);
    } else {
        tcg_gen_shri_i32(tmp1, tmp1, 16);
        tcg_gen_shri_i32(tmp2, tmp2, 16);
    }

    tcg_gen_add_i32(dst, dst, tmp1);
    tcg_gen_add_i32(dst, dst, tmp2);
    tcg_gen_add_i32(dst, dst, tmp3);

    if (heed_prodshift && (dc->tbflags & META_TBFLAG_DUPRODSHIFT)) {
        TCGv tmp4 = tcg_temp_new();

        tcg_gen_shli_i32(tmp3, dst, 1);
        tcg_gen_shri_i32(tmp4, mid_sum, 15);
        tcg_gen_andi_i32(tmp4, tmp4, 0x1);

        if (heed_saturation &&
            inst->mul.sign &&
            (dc->tbflags & META_TBFLAG_DUSATURATION)) {
            /* signed 32x32 saturation */
            tcg_gen_movi_i32(tmp1, 0x7fffffff);
            tcg_gen_movi_i32(tmp2, 0x40000000);
            tcg_gen_or_i32(tmp3, tmp3, tmp4);
            tcg_gen_movcond_i32(TCG_COND_EQ, dst, dst, tmp2, tmp1, tmp3);
        } else {
            tcg_gen_or_i32(dst, tmp3, tmp4);
        }

        if (heed_round && (dc->tbflags & META_TBFLAG_DUROUNDING)) {
            tcg_gen_shli_i32(mid_sum, mid_sum, 1);
        }

    }

    if (heed_round && (dc->tbflags & META_TBFLAG_DUROUNDING)) {
        /* mid_add[15] & !dst[0] */
        tcg_gen_shri_i32(tmp1, mid_sum, 15);
        tcg_gen_not_i32(tmp2, dst);
        tcg_gen_and_i32(tmp1, tmp1, tmp2);
        tcg_gen_andi_i32(tmp1, tmp1, 0x1);

        /* mid_add[15:0] > 0x8000 */
        tcg_gen_ext16u_i32(tmp2, mid_sum);
        tcg_gen_setcondi_i32(TCG_COND_GT, tmp2, tmp2, 0x8000);
        tcg_gen_or_i32(tmp1, tmp1, tmp2);

        if (is_signed) {
            /* dst != max +ve */
            tcg_gen_setcondi_i32(TCG_COND_NE, tmp2, dst, 0x7fffffff);
            tcg_gen_and_i32(tmp1, tmp1, tmp2);
        }

        tcg_gen_add_i32(dst, dst, tmp1);
    }

}

static void meta_gen_dsp_mul_i16(DisasContext *dc,
                                 const MetaInstruction *inst,
                                 TCGv dst, TCGv src1, TCGv src2,
                                 bool heed_prodshift, bool heed_saturation,
                                 bool heed_round)
{
    TCGv tmp1 = tcg_temp_new();
    TCGv tmp2 = tcg_temp_new();

    /* extend sources appropriately */
    if (inst->mul.sign) {
        tcg_gen_ext16s_i32(tmp1, src1);
        tcg_gen_ext16s_i32(tmp2, src2);
    } else {
        tcg_gen_ext16u_i32(tmp1, src1);
        tcg_gen_ext16u_i32(tmp2, src2);
    }

    /* perform the multiply */
    tcg_gen_mul_i32(dst, tmp1, tmp2);

    if (heed_prodshift && (dc->tbflags & META_TBFLAG_DUPRODSHIFT)) {
        if (inst->mul.sign &&
            heed_saturation &&
            (dc->tbflags & META_TBFLAG_DUSATURATION)) {
            /* signed 16x16 saturation */
            TCGv tmp3 = meta_const_i32(0x7fffffff);
            tcg_gen_shli_i32(tmp1, dst, 1);
            tcg_gen_movi_i32(tmp2, 0x40000000);
            tcg_gen_movcond_i32(TCG_COND_EQ, dst, dst, tmp2, tmp3, tmp1);
        } else {
            tcg_gen_shli_i32(dst, dst, 1);
        }
    }

    if (heed_round && (dc->tbflags & META_TBFLAG_DUROUNDING)) {
        meta_gen_dsp_round_hi16(dc, dst, dst, inst->mul.sign);
    }

}

static void meta_get_split8(TCGv *reg, bool *is_temp,
                            bool src_upper, bool inst_upper)
{
    TCGv val = *reg;

    if (!*is_temp) {
        val = tcg_temp_new();
    }

    if (src_upper) {
        tcg_gen_andi_i32(val, *reg, 0xff00ff00);
        if (!inst_upper) {
            tcg_gen_shri_i32(val, val, 8);
        }
    } else {
        tcg_gen_andi_i32(val, *reg, 0x00ff00ff);
        if (inst_upper) {
            tcg_gen_shli_i32(val, val, 8);
        }
    }

    if (!*is_temp) {
        *reg = val;
        *is_temp = true;
    }
}

static void meta_gen_duaddsub(DisasContext *dc, const MetaInstruction *inst)
{
    const MetaInstructionDUAddSub *as = &inst->duaddsub;
    MetaDUArithMode am = meta_dc_duarithmode(dc);
    MetaSrcData sdata;
    TCGv dst[2];
    bool dst_tmp[2] = { 0 };
    bool movneg = (inst->src1.type == SRC_IMM) && !inst->src1.i;
    bool isacc = inst->dsp && (inst->dst.type == DST_ACCUM);
    DEFINE_SAFE_LABEL(lbl_cc);
    int di;

    /* read sources */
    meta_read_sources(dc, &sdata, inst,
                      !isacc && (!movneg || inst->dsp),
                      !isacc && ((inst->src2.type != SRC_IMM) || inst->dsp));

    /* branch if condition fails */
    if (inst->cc) {
        meta_start_cc(dc, inst->cc, &lbl_cc);
    }

    /* get destination */
    if (!isacc &&
        !(inst->dsp && as->d.mod && (am == META_DUARITH_SPLIT16))) {
        dst[0] = meta_get_dest(dc, inst->dst, &dst_tmp[0]);
        if (inst->dual) {
            dst[1] = meta_get_dest(dc, meta_hi_dest(inst->dst), &dst_tmp[1]);
        }
    }

    if (!inst->dsp) {
        /* GP instruction */
        if (movneg) {
            /* MOV or NEG */
            if (as->setflags) {
                if (inst->src2.type == SRC_IMM) {
                    /* optimised function for flags set from immediates */
                    if (as->sub) {
                        meta_gen_set_cf_vc_subzi(inst->src2.i);
                    } else {
                        meta_gen_set_cf_vc_addzi(inst->src2.i);
                    }
                } else {
                    if (as->sub) {
                        meta_gen_set_cf_vc_subz(sdata.src2[0]);
                    } else {
                        meta_gen_set_cf_vc_addz(sdata.src2[0]);
                    }
                }
            }

            if (inst->src2.type == SRC_IMM) {
                if (as->sub) {
                    tcg_gen_movi_i32(dst[0], -inst->src2.i);
                } else {
                    tcg_gen_movi_i32(dst[0], inst->src2.i);
                }
            } else {
                if (as->sub) {
                    tcg_gen_neg_i32(dst[0], sdata.src2[0]);
                } else {
                    tcg_gen_mov_i32(dst[0], sdata.src2[0]);
                }
            }
        } else {
            /* ADD or SUB */
            if (as->setflags) {
                if (inst->src2.type == SRC_IMM) {
                    /* optimised function for flags set from immediates */
                    if (as->sub) {
                        meta_gen_set_cf_vc_subi_i32(sdata.src1[0],
                                                    inst->src2.i);
                    } else {
                        meta_gen_set_cf_vc_addi_i32(sdata.src1[0],
                                                    inst->src2.i);
                    }
                } else {
                    if (as->sub) {
                        meta_gen_set_cf_vc_sub_pre_i32(sdata.src1[0],
                                                       sdata.src2[0]);
                    } else {
                        meta_gen_set_cf_vc_add_pre_i32(sdata.src1[0],
                                                       sdata.src2[0]);
                    }
                }
            }

            if (inst->src2.type == SRC_IMM) {
                if (as->sub) {
                    tcg_gen_subi_i32(dst[0], sdata.src1[0], inst->src2.i);
                } else {
                    tcg_gen_addi_i32(dst[0], sdata.src1[0], inst->src2.i);
                }
            } else {
                if (as->sub) {
                    tcg_gen_sub_i32(dst[0], sdata.src1[0], sdata.src2[0]);
                } else {
                    tcg_gen_add_i32(dst[0], sdata.src1[0], sdata.src2[0]);
                }
            }

            if (as->setflags && (inst->src2.type != SRC_IMM)) {
                if (as->sub) {
                    meta_gen_set_cf_vc_sub_post_i32(dst[0],
                                                    sdata.src2[0]);
                } else {
                    meta_gen_set_cf_vc_add_post_i32(dst[0],
                                                    sdata.src2[0]);
                }
            }
        }

        if (as->setflags) {
            meta_gen_set_cf_nz(dst[0]);
            meta_gen_set_scc(dc, false);
        }
    } else {
        /* DSP instruction */
        /* work in each applicable unit */
        for (di = 0; di <= inst->dual; di++) {
            bool setflags = as->setflags && !di;

            if (isacc) {
                /* cross-unit accumulation */
                MetaUnit u_d = inst->dst.u;
                MetaUnit u_s1 = inst->src1.u;
                MetaUnit u_s2 = inst->src2.u;
                TCGv_i64 acc_d, acc_s1, acc_s2;

                if (di) {
                    u_d = meta_unit_partner(u_d);
                    u_s1 = meta_unit_partner(u_s1);
                    u_s2 = meta_unit_partner(u_s2);
                }

                acc_d = meta_get_acc(dc, u_d, inst->dst.i);
                acc_s1 = meta_get_acc(dc, u_s1, inst->src1.i);
                acc_s2 = meta_get_acc(dc, u_s2, inst->src2.i);

                meta_gen_dsp_add_i40(dc, as->sub,
                                     acc_d, acc_s1, acc_s2,
                                     false /* no DUAccSat */);
                continue;
            }

            if (as->d.split8) {
                if (as->d.split8_src1) {
                    meta_get_split8(&sdata.src1[di], &sdata.src1_tmp[di],
                                    as->d.split8_upper, false);
                }

                meta_get_split8(&sdata.src2[di], &sdata.src2_tmp[di],
                                as->d.split8_upper, false);
            }

            if (am == META_DUARITH_SPLIT16) {
                /* split-16 */
                TCGv low1 = tcg_temp_new();
                TCGv low2 = tcg_temp_new();
                TCGv high1 = tcg_temp_new();
                TCGv high2 = tcg_temp_new();
                TCGv tmp1 = tcg_temp_new();
                TCGv cf_lz = setflags ?
                    cpu_cf[META_SCF_LZ] : NULL;
                TCGv cf_hz = setflags ?
                    cpu_cf[META_SCF_HZ] : NULL;
                TCGv cf_lc = setflags ?
                    cpu_cf[META_SCF_LC] : NULL;
                TCGv cf_hc = setflags ?
                    cpu_cf[META_SCF_HC] : NULL;

                /* unpack values */
                tcg_gen_ext16u_i32(low1, sdata.src1[di]);
                tcg_gen_ext16u_i32(low2, sdata.src2[di]);
                tcg_gen_shri_i32(high1, sdata.src1[di], 16);
                tcg_gen_shri_i32(high2, sdata.src2[di], 16);

                if (!as->d.mod) {
                    /* un-modified */
                    meta_gen_dsp_add_i16(dc, as->sub,
                                         cf_lc, NULL,
                                         NULL, cf_lz,
                                         dst[di], low1, low2,
                                         as->d.pshift, true);
                    meta_gen_dsp_add_i16(dc, as->sub,
                                         cf_hc, NULL,
                                         NULL, cf_hz,
                                         tmp1, high1, high2,
                                         as->d.pshift, true);

                    /* pack result */
                    tcg_gen_deposit_i32(dst[di], dst[di], tmp1, 16, 16);
                } else {
                    /* modified, complex butterfly */
                    TCGv ret1, ret2;
                    bool ret1_tmp = false, ret2_tmp = false;
                    MetaInstructionDest sdst1, sdst2;

                    meta_dest_from_src(dc, &sdst1, &inst->src1);
                    meta_dest_from_src(dc, &sdst2, &inst->src2);
                    ret1 = meta_get_dest(dc,
                            di ? meta_hi_dest(sdst1) : sdst1, &ret1_tmp);
                    ret2 = meta_get_dest(dc,
                            di ? meta_hi_dest(sdst2) : sdst2, &ret2_tmp);

                    /* first operation, as encoded */
                    meta_gen_dsp_add_i16(dc, as->sub,
                                         cf_lc, NULL,
                                         NULL, cf_lz,
                                         ret1, low1, low2,
                                         as->d.pshift, false);
                    meta_gen_dsp_add_i16(dc, as->sub,
                                         cf_hc, NULL,
                                         NULL, cf_hz,
                                         tmp1, high1, high2,
                                         as->d.pshift, false);

                    /* pack first result */
                    tcg_gen_deposit_i32(ret1, ret1, tmp1, 16, 16);

                    /* second operation, complementary to encoded */
                    meta_gen_dsp_add_i16(dc, !as->sub,
                                         NULL, NULL,
                                         NULL, NULL,
                                         ret2, low1, low2,
                                         as->d.pshift, false);
                    meta_gen_dsp_add_i16(dc, !as->sub,
                                         NULL, NULL,
                                         NULL, NULL,
                                         tmp1, high1, high2,
                                         as->d.pshift, false);

                    /* pack second result */
                    tcg_gen_deposit_i32(ret2, ret2, tmp1, 16, 16);

                    /* write result, if necessary */
                    meta_set_dest(dc, di ? meta_hi_dest(sdst1) : sdst1, ret1);
                    meta_set_dest(dc, di ? meta_hi_dest(sdst2) : sdst2, ret2);

                }

            } else {
                /* non-split-16 */
                bool b16 = (am == META_DUARITH_16X16) && as->d.mod;

                if (b16) {
                    /* 16-bit add */
                    meta_gen_dsp_add_i16(dc, as->sub,
                                         cpu_cf[META_CF_C], cpu_cf[META_CF_V],
                                         cpu_cf[META_CF_N], cpu_cf[META_CF_Z],
                                         dst[di],
                                         sdata.src1[di], sdata.src2[di],
                                         as->d.pshift, false);
                } else {
                    /* 32-bit add */
                    meta_gen_dsp_add_i32(dc, as->sub, setflags, dst[di],
                                         sdata.src1[di], sdata.src2[di],
                                         as->d.pshift,
                                         true); /* can saturate */
                }
            }

            if (setflags) {
                meta_gen_set_scc(dc, am == META_DUARITH_SPLIT16);
            }
        }
    }

    /* if necessary, write the result */
    if (!isacc &&
        !(inst->dsp && as->d.mod && (am == META_DUARITH_SPLIT16))) {
        meta_set_dest(dc, inst->dst, dst[0]);
        if (inst->dual) {
            meta_set_dest(dc, meta_hi_dest(inst->dst), dst[1]);
        }
    }

    /* clean up */
    meta_cleanup_sources(dc, &sdata, inst);

    /* place the label for failed conditions */
    if (inst->cc) {
        meta_end_cc(dc, &lbl_cc);
    }
}

static void meta_gen_andorxor(DisasContext *dc, const MetaInstruction *inst)
{
    const MetaInstructionAndOrXor *aox = &inst->andorxor;
    MetaDUArithMode am = meta_dc_duarithmode(dc);
    MetaSrcData sdata;
    TCGv dst[2];
    bool dst_tmp[2] = { 0 };
    DEFINE_SAFE_LABEL(lbl_cc);
    int di;
    uint32_t s2_imm = inst->src2.i;

    /* read sources */
    meta_read_sources(dc, &sdata, inst, true,
                      inst->src2.type != SRC_IMM);

    /* branch if condition fails */
    if (inst->cc) {
        meta_start_cc(dc, inst->cc, &lbl_cc);
    }

    /* get destination */
    dst[0] = meta_get_dest(dc, inst->dst, &dst_tmp[0]);
    if (inst->dual) {
        dst[1] = meta_get_dest(dc, meta_hi_dest(inst->dst), &dst_tmp[1]);
    }

    if (inst->dsp &&
        (am == META_DUARITH_SPLIT16) &&
        (inst->src2.type == SRC_IMM)) {
        /* split-16 uses the same immediate for both halves */
        s2_imm = (s2_imm << 16) | (s2_imm & 0xffff);
    }

    for (di = 0; di <= inst->dual; di++) {
        switch (aox->op) {
        case OP_AND:
            if (inst->src2.type == SRC_IMM) {
                tcg_gen_andi_i32(dst[di], sdata.src1[di], s2_imm);
            } else {
                tcg_gen_and_i32(dst[di], sdata.src1[di], sdata.src2[di]);
            }
            break;

        case OP_OR:
            if (inst->src2.type == SRC_IMM) {
                tcg_gen_ori_i32(dst[di], sdata.src1[di], s2_imm);
            } else {
                tcg_gen_or_i32(dst[di], sdata.src1[di], sdata.src2[di]);
            }
            break;

        case OP_XOR:
            if (inst->src2.type == SRC_IMM) {
                tcg_gen_xori_i32(dst[di], sdata.src1[di], s2_imm);
            } else {
                tcg_gen_xor_i32(dst[di], sdata.src1[di], sdata.src2[di]);
            }
            break;
        }
    }

    if (aox->setflags) {
        if (inst->dsp && (am == META_DUARITH_SPLIT16)) {
            TCGv tmp = tcg_temp_new();

            tcg_gen_ext16u_i32(tmp, dst[0]);
            tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_cf[META_SCF_LZ],
                                 tmp, 0);

            tcg_gen_shri_i32(tmp, dst[0], 16);
            tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_cf[META_SCF_HZ],
                                 tmp, 0);


            tcg_gen_movi_i32(cpu_cf[META_SCF_LC], 0);
            tcg_gen_movi_i32(cpu_cf[META_SCF_HC], 0);

            meta_gen_set_scc(dc, true);
        } else {
            meta_gen_set_cf_nz(dst[0]);
            meta_gen_clear_cf_vc();
            meta_gen_set_scc(dc, false);
        }
    }

    /* if necessary, write the result */
    meta_set_dest(dc, inst->dst, dst[0]);
    if (inst->dual) {
        meta_set_dest(dc, meta_hi_dest(inst->dst), dst[1]);
    }

    /* clean up */
    meta_cleanup_sources(dc, &sdata, inst);

    /* place the label for failed conditions */
    if (inst->cc) {
        meta_end_cc(dc, &lbl_cc);
    }
}

static void meta_gen_shift(DisasContext *dc, const MetaInstruction *inst)
{
    const MetaInstructionShift *sh = &inst->shift;
    MetaDUArithMode am = meta_dc_duarithmode(dc);
    MetaSrcData sdata;
    TCGv dst[2];
    bool dst_tmp[2] = { 0 };
    DEFINE_SAFE_LABEL(lbl_cc);
    int di;

    /* read sources */
    meta_read_sources(dc, &sdata, inst,
                      inst->src1.type != SRC_ACCUM,
                      inst->src2.type != SRC_IMM);

    /* branch if condition fails */
    if (inst->cc) {
        meta_start_cc(dc, inst->cc, &lbl_cc);
    }

    /* get destination */
    dst[0] = meta_get_dest(dc, inst->dst, &dst_tmp[0]);
    if (inst->dual) {
        dst[1] = meta_get_dest(dc, meta_hi_dest(inst->dst), &dst_tmp[1]);
    }

    if (inst->dsp) {
        /* DSP instruction */
        uint32_t max_shift = 31;
        bool want_right, want_left;
        int s2imm;

        /* clamp the immediate appropriately */
        if ((inst->src1.type != SRC_ACCUM) && (am == META_DUARITH_SPLIT16)) {
            max_shift = 15;
        }
        s2imm = MIN(inst->src2.i, max_shift);

        /* if this is a shift by immediate choose whether it's right or left
           if it's a shift by register emit both because we can't know now */
        want_right = (inst->src2.type != SRC_IMM) || !s2imm || sh->right;
        want_left = (inst->src2.type != SRC_IMM) || !sh->right;

        for (di = 0; di <= inst->dual; di++) {
            TCGv s2 = NULL;
            TCGLabel *lbl_left = NULL, *lbl_done = NULL;
            MetaUnit s1u = di ? meta_unit_partner(inst->src1.u) : inst->src1.u;

            /* clamp source 2 & negate if necessary */
            if (inst->src2.type != SRC_IMM) {
                TCGv tmp = tcg_temp_new();
                s2 = tcg_temp_new();
                lbl_left = gen_new_label();
                lbl_done = gen_new_label();

                /* s2 = (src2 > max) ? max : src2 */
                tcg_gen_movi_i32(tmp, max_shift);
                tcg_gen_movcond_i32(TCG_COND_GT, s2,
                                    sdata.src2[di], tmp,
                                    tmp, sdata.src2[di]);

                /* s2 = (s2 < -max) ? -max : s2 */
                tcg_gen_movi_i32(tmp, -max_shift);
                tcg_gen_movcond_i32(TCG_COND_LT, s2, s2, tmp, tmp, s2);

                if (sh->right) {
                    tcg_gen_neg_i32(s2, s2);
                }


                /* if (s2 <= 0) { */
                tcg_gen_brcondi_i32(TCG_COND_GT, s2, 0, lbl_left);
                tcg_gen_neg_i32(s2, s2);
            } else {
                s2 = meta_const_i32(s2imm);
            }

            if (want_right) {
                if (inst->src1.type == SRC_ACCUM) {
                    TCGv_i64 acc = meta_get_acc(dc, s1u, inst->src1.i);
                    meta_gen_dsp_shift_right_i40(dc, inst,
                                                 dst[di], acc, s2);
                } else if (am == META_DUARITH_SPLIT16) {
                    TCGv high = tcg_temp_new();
                    TCGv low = tcg_temp_new();

                    tcg_gen_shri_i32(high, sdata.src1[di], 16);
                    meta_gen_dsp_shift_right_i16(dc, inst, high, high, s2,
                                                 cpu_cf[META_SCF_HC],
                                                 cpu_cf[META_SCF_HZ]);

                    tcg_gen_ext16u_i32(low, sdata.src1[di]);
                    meta_gen_dsp_shift_right_i16(dc, inst, low, low, s2,
                                                 cpu_cf[META_SCF_LC],
                                                 cpu_cf[META_SCF_LZ]);

                    tcg_gen_deposit_i32(dst[di], low, high, 16, 16);

                    if (sh->setflags) {
                        meta_gen_set_scc(dc, true);
                    }
                } else {
                    meta_gen_dsp_shift_right_i32(dc, inst,
                                                 dst[di], sdata.src1[di], s2);
                }
            }

            if (inst->src2.type != SRC_IMM) {
                /* } else { */
                tcg_gen_br(lbl_done);
                gen_set_label(lbl_left);
            }

            if (want_left) {
                if (inst->src1.type == SRC_ACCUM) {
                    TCGv_i64 acc = meta_get_acc(dc, s1u, inst->src1.i);
                    meta_gen_dsp_shift_left_i40(dc, inst,
                                                dst[di], acc, s2);
                } else if (am == META_DUARITH_SPLIT16) {
                    TCGv high = tcg_temp_new();
                    TCGv low = tcg_temp_new();

                    tcg_gen_shri_i32(high, sdata.src1[di], 16);
                    meta_gen_dsp_shift_left_i16(dc, inst, high, high, s2,
                                                cpu_cf[META_SCF_HC],
                                                cpu_cf[META_SCF_HZ]);

                    tcg_gen_ext16u_i32(low, sdata.src1[di]);
                    meta_gen_dsp_shift_left_i16(dc, inst, low, low, s2,
                                                cpu_cf[META_SCF_LC],
                                                cpu_cf[META_SCF_LZ]);

                    tcg_gen_deposit_i32(dst[di], low, high, 16, 16);

                    if (sh->setflags) {
                        meta_gen_set_scc(dc, true);
                    }
                } else {
                    meta_gen_dsp_shift_left_i32(dc, inst,
                                                dst[di], sdata.src1[di], s2);
                }
            }

            if (inst->src2.type != SRC_IMM) {
                /* } */
                gen_set_label(lbl_done);
            }

        }
    } else {
        /* GP instruction */
        TCGv s2 = NULL;

        if (inst->src2.type != SRC_IMM) {
            s2 = tcg_temp_new();
            tcg_gen_andi_i32(s2, sdata.src2[0], 0x1f);

            if (sh->setflags) {
                meta_gen_set_cf_vc_shift(sh->right, sdata.src1[0], s2);
            }
        } else if (sh->setflags) {
            meta_gen_set_cf_vc_shifti(sh->right, sdata.src1[0], inst->src2.i);
        }

        if (sh->right && sh->arithmetic) {
            if (inst->src2.type == SRC_IMM) {
                tcg_gen_sari_i32(dst[0], sdata.src1[0], inst->src2.i);
            } else {
                tcg_gen_sar_i32(dst[0], sdata.src1[0], s2);
            }
        } else if (sh->right) {
            if (inst->src2.type == SRC_IMM) {
                tcg_gen_shri_i32(dst[0], sdata.src1[0], inst->src2.i);
            } else {
                tcg_gen_shr_i32(dst[0], sdata.src1[0], s2);
            }
        } else {
            if (inst->src2.type == SRC_IMM) {
                tcg_gen_shli_i32(dst[0], sdata.src1[0], inst->src2.i);
            } else {
                tcg_gen_shl_i32(dst[0], sdata.src1[0], s2);
            }
        }

        if (sh->setflags) {
            meta_gen_set_cf_nz(dst[0]);
            meta_gen_set_scc(dc, false);
        }

    }

    /* if necessary, write the result */
    meta_set_dest(dc, inst->dst, dst[0]);
    if (inst->dual) {
        meta_set_dest(dc, meta_hi_dest(inst->dst), dst[1]);
    }

    /* clean up */
    meta_cleanup_sources(dc, &sdata, inst);

    /* place the label for failed conditions */
    if (inst->cc) {
        meta_end_cc(dc, &lbl_cc);
    }
}

static void meta_gen_mul(DisasContext *dc, const MetaInstruction *inst)
{
    const MetaInstructionMul *mul = &inst->mul;
    MetaDUArithMode am = meta_dc_duarithmode(dc);
    MetaSrcData sdata;
    TCGv dst[2];
    bool dst_tmp[2] = { 0 };
    DEFINE_SAFE_LABEL(lbl_cc);
    int di;

    /* read sources */
    meta_read_sources(dc, &sdata, inst, true,
                      inst->src2.type != SRC_IMM);

    /* branch if condition fails */
    if (inst->cc) {
        meta_start_cc(dc, inst->cc, &lbl_cc);
    }

    if (inst->dst.type != DST_ACCUM) {
        /* get destination */
        dst[0] = meta_get_dest(dc, inst->dst, &dst_tmp[0]);
        if (inst->dual) {
            dst[1] = meta_get_dest(dc, meta_hi_dest(inst->dst), &dst_tmp[1]);
        }
    } else {
        /* use a 32-bit temp destination */
        for (di = 0; di <= inst->dual; di++) {
            dst[di] = tcg_temp_new();
            dst_tmp[di] = true;
        }
    }

    if (inst->dsp) {
        /* DSP instruction */

        for (di = 0; di <= inst->dual; di++) {
            uint32_t s2_imm = inst->src2.i;

            if (mul->d.split8) {
                if (mul->d.split8_src1) {
                    meta_get_split8(&sdata.src1[di], &sdata.src1_tmp[di],
                                    mul->d.split8_upper, true);
                }

                if (inst->src2.type == SRC_IMM) {
                    if (mul->d.split8_upper) {
                        s2_imm = s2_imm & 0xff00ff00;
                    } else {
                        s2_imm = (s2_imm & 0x00ff00ff) << 8;
                    }
                } else {
                    meta_get_split8(&sdata.src2[di], &sdata.src2_tmp[di],
                                    mul->d.split8_upper, true);
                }
            }

            if (am == META_DUARITH_SPLIT16) {
                /* split-16 */
                TCGv low1 = tcg_temp_new();
                TCGv low2 = tcg_temp_new();
                TCGv high1 = tcg_temp_new();
                TCGv high2;

                /* unpack src1 */
                tcg_gen_ext16u_i32(low1, sdata.src1[di]);
                tcg_gen_shri_i32(high1, sdata.src1[di], 16);

                /* unpack src2 */
                if (inst->src2.type == SRC_IMM) {
                    tcg_gen_movi_i32(low2, ((int32_t)s2_imm << 16) >> 16);
                    high2 = low2;
                } else {
                    high2 = tcg_temp_new();
                    tcg_gen_ext16u_i32(low2, sdata.src2[di]);
                    tcg_gen_shri_i32(high2, sdata.src2[di], 16);
                }

                if (mul->d.mod && !mul->d.acc) {
                    /* modified, no accumulate, complex butterfly */
                    TCGv res_add = tcg_temp_new();
                    TCGv res_sub = tcg_temp_new();
                    TCGv tmp1 = tcg_temp_new();

                    meta_gen_dsp_mul_i16(dc, inst, res_sub, high1, high2,
                                         false, /* can't product shift */
                                         false, /* can't saturate */
                                         false); /* can't round */
                    meta_gen_dsp_mul_i16(dc, inst, tmp1, low1, low2,
                                         false, /* can't product shift */
                                         false, /* can't saturate */
                                         false); /* can't round */
                    meta_gen_dsp_add_i32(dc, true, false,
                                         res_sub, res_sub, tmp1,
                                         false, /* no product shift */
                                         false); /* can't saturate */

                    meta_gen_dsp_mul_i16(dc, inst, res_add, high1, low2,
                                         false, /* can't product shift */
                                         false, /* can't saturate */
                                         false); /* can't round */
                    meta_gen_dsp_mul_i16(dc, inst, tmp1, low1, high2,
                                         false, /* can't product shift */
                                         false, /* can't saturate */
                                         false); /* can't round */
                    meta_gen_dsp_add_i32(dc, false, false,
                                         res_add, res_add, tmp1,
                                         false, /* no product shift */
                                         false); /* can't saturate */

                    if (dc->tbflags & META_TBFLAG_DUPRODSHIFT) {
                        if (mul->sign &&
                            (dc->tbflags & META_TBFLAG_DUSATURATION)) {
                            /* saturate */
                            gen_helper_mul_ps_sat16(res_add, res_add);
                            gen_helper_mul_ps_sat16(res_sub, res_sub);
                        } else {
                            tcg_gen_shli_i32(res_add, res_add, 1);
                            tcg_gen_shli_i32(res_sub, res_sub, 1);
                        }
                    }

                    if (dc->tbflags & META_TBFLAG_DUROUNDING) {
                        bool sat = dc->tbflags & META_TBFLAG_DUSATURATION;
                        meta_gen_dsp_round_hi16(dc, res_add, res_add, sat);
                        meta_gen_dsp_round_hi16(dc, res_sub, res_sub, sat);
                    }

                    /* pack the result */
                    tcg_gen_shri_i32(res_add, res_add, 16);
                    tcg_gen_deposit_i32(dst[di], res_sub, res_add, 0, 16);

                } else if (mul->d.acc) {
                    /* un-modified, with accumulate */
                    TCGv tmp1 = tcg_temp_new();

                    meta_gen_dsp_mul_i16(dc, inst, dst[di], low1, low2,
                                         false, /* can't product shift */
                                         false, /* can't saturate */
                                         false); /* can't round */
                    meta_gen_dsp_mul_i16(dc, inst, tmp1, high1, high2,
                                         false, /* can't product shift */
                                         false, /* can't saturate */
                                         false); /* can't round */
                    meta_gen_dsp_add_i32(dc, false, false,
                                         dst[di], dst[di], tmp1,
                                         false, /* no product shift */
                                         mul->sign); /* saturate if signed */

                    if (dc->tbflags & META_TBFLAG_DUPRODSHIFT) {
                        if (mul->sign &&
                            (dc->tbflags & META_TBFLAG_DUSATURATION)) {
                            /* saturate */
                            gen_helper_mul_ps_sat16(dst[di], dst[di]);
                        } else {
                            tcg_gen_shli_i32(dst[di], dst[di], 1);
                        }
                    }

                } else {
                    /* un-modified, no accumulate */

                    meta_gen_dsp_mul_i16(dc, inst, low1, low1, low2,
                                         true, /* can product shift */
                                         true, /* can saturate */
                                         true); /* can round */
                    meta_gen_dsp_mul_i16(dc, inst, high1, high1, high2,
                                         true, /* can product shift */
                                         true, /* can saturate */
                                         true); /* can round */

                    /* pack result */
                    tcg_gen_shri_i32(low1, low1, 16);
                    tcg_gen_deposit_i32(dst[di], high1, low1, 0, 16);
                }

            } else if (am == META_DUARITH_16X16) {
                TCGv s2 = sdata.src2[di];

                if (inst->src2.type == SRC_IMM) {
                    s2 = meta_const_i32(s2_imm);
                }

                if (mul->d.mod) {
                    /* 31x31 */
                    meta_gen_dsp_mul_i31(dc, inst, dst[di],
                                         sdata.src1[di], s2,
                                         true, /* can product shift */
                                         true, /* can saturate */
                                         true, /* can round */
                                         mul->sign); /* signed? */
                } else {
                    /* 16x16 */
                    meta_gen_dsp_mul_i16(dc, inst, dst[di],
                                         sdata.src1[di], s2,
                                         true, /* can product shift */
                                         true, /* can saturate */
                                         false); /* can't round */
                }

            } else {
                /* 32x32 */
                bool high = (am == META_DUARITH_32X32H) || mul->d.mod;
                TCGv_i64 res64 = tcg_temp_new_i64();
                TCGv s2 = sdata.src2[di];

                if (inst->src2.type == SRC_IMM) {
                    s2 = meta_const_i32(s2_imm);
                }

                meta_gen_dsp_mul_i32(dc, inst, res64, sdata.src1[di], s2,
                                     true, /* can product shift */
                                     true, /* can saturate */
                                     false); /* can't round */


                if (high) {
                    tcg_gen_shri_i64(res64, res64, 32);
                }
                tcg_gen_extrl_i64_i32(dst[di], res64);
            }

            if (mul->d.acc) {
                MetaUnit u = di ? meta_unit_partner(inst->dst.u) : inst->dst.u;
                TCGv_i64 acc = meta_get_acc(dc, u, inst->dst.i);
                TCGv_i64 s1 = acc;
                TCGv_i64 s2 = tcg_temp_new_i64();

                if (mul->d.acc == ACC_Z) {
                    s1 = meta_const_i64(0);
                }

                /* extend the result from 32-bits to 40=bits */
                if (mul->sign) {
                    tcg_gen_ext_i32_i64(s2, dst[di]);
                } else {
                    tcg_gen_extu_i32_i64(s2, dst[di]);
                }

                meta_gen_dsp_add_i40(dc, mul->d.acc == ACC_N,
                                     acc, s1, s2, true);

            }
        }
    } else {
        /* GP instruction */
        TCGv s1 = sdata.src1[0];
        TCGv s2 = sdata.src2[0];
        uint32_t imm = inst->src2.i;

        if (!mul->l2) {
            /* 16x16 multiply */
            s1 = sdata.src1_tmp[0] ? sdata.src1[0] : tcg_temp_new();
            tcg_gen_ext16u_i32(s1, sdata.src1[0]);

            if (inst->src2.type != SRC_IMM) {
                s2 = sdata.src2_tmp[0] ? sdata.src2[0] : tcg_temp_new();
                tcg_gen_ext16u_i32(s2, sdata.src2[0]);
            } else {
                imm &= 0xffff;
            }
        }

        if (inst->src2.type == SRC_IMM) {
            tcg_gen_muli_i32(dst[0], s1, imm);
        } else {
            tcg_gen_mul_i32(dst[0], s1, s2);
        }

    }

    /* if necessary, write the result */
    if (inst->dst.type != DST_ACCUM) {
        meta_set_dest(dc, inst->dst, dst[0]);
        if (inst->dual) {
            meta_set_dest(dc, meta_hi_dest(inst->dst), dst[1]);
        }
    }

    /* clean up */
    meta_cleanup_sources(dc, &sdata, inst);

    /* place the label for failed conditions */
    if (inst->cc) {
        meta_end_cc(dc, &lbl_cc);
    }
}

static void meta_gen_abs_i32(DisasContext *dc, TCGv dst, TCGv src,
                             bool saturate)
{
    TCGv tmp1 = tcg_temp_new();
    TCGv zero = meta_const_i32(0);

    /* negate */
    tcg_gen_neg_i32(tmp1, src);

    /* choose whether to use it */
    tcg_gen_movcond_i32(TCG_COND_LT, dst, src, zero, tmp1, src);

    if (saturate) {
        tcg_gen_movi_i32(tmp1, 0x7fffffff);
        tcg_gen_movcond_i32(TCG_COND_LT, dst, dst, zero, tmp1, dst);
    }

}

static void meta_gen_cmptst(DisasContext *dc, const MetaInstruction *inst)
{
    const MetaInstructionCmpTst *ct = &inst->cmptst;
    MetaDUArithMode am = meta_dc_duarithmode(dc);
    MetaSrcData sdata;
    TCGv dst[2];
    bool dst_tmp[2] = { 0 };
    DEFINE_SAFE_LABEL(lbl_cc);
    int di;
    uint32_t s2_imm;

    /* read sources */
    meta_read_sources(dc, &sdata, inst, true,
                      ct->src2_cmp.type != SRC_IMM || ct->ext_op == OP_NMIN);

    /* branch if condition fails */
    if (inst->cc) {
        meta_start_cc(dc, inst->cc, &lbl_cc);
    }

    if (inst->dst.type == DST_NONE) {
        /* use temps for destination */
        for (di = 0; di <= inst->dual; di++) {
            dst[di] = tcg_temp_new();
            dst_tmp[di] = true;
        }
    } else {
        /* get destination */
        dst[0] = meta_get_dest(dc, inst->dst, &dst_tmp[0]);
        if (inst->dual) {
            dst[1] = meta_get_dest(dc, meta_hi_dest(inst->dst), &dst_tmp[1]);
        }
    }

    if (ct->src2_maxneg) {
        if (am == META_DUARITH_SPLIT16) {
            s2_imm = 0x8000;
        } else {
            s2_imm = 0x80000000;
        }
    } else {
        s2_imm = ct->src2_cmp.i;
    }

    if (ct->tst) {
        if (inst->dsp && (am == META_DUARITH_SPLIT16)) {
            /* high */
            tcg_gen_shri_i32(dst[0], sdata.src1[0], 16);
            if (ct->src2_cmp.type == SRC_IMM) {
                tcg_gen_andi_i32(dst[0], dst[0], s2_imm);
            } else {
                tcg_gen_and_i32(dst[0], dst[0], sdata.src2[0]);
            }
            tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_cf[META_SCF_HZ], dst[0], 0);
            tcg_gen_movi_i32(cpu_cf[META_SCF_HC], 0);

            /* low */
            tcg_gen_ext16u_i32(dst[0], sdata.src1[0]);
            if (ct->src2_cmp.type == SRC_IMM) {
                tcg_gen_andi_i32(dst[0], dst[0], s2_imm);
            } else {
                tcg_gen_and_i32(dst[0], dst[0], sdata.src2[0]);
            }
            tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_cf[META_SCF_LZ], dst[0], 0);
            tcg_gen_movi_i32(cpu_cf[META_SCF_LC], 0);

            meta_gen_set_scc(dc, true);
        } else {
            if (ct->src2_cmp.type == SRC_IMM) {
                meta_gen_set_cf_andi(sdata.src1[0], s2_imm);
            } else {
                tcg_gen_and_i32(dst[0], sdata.src1[0], sdata.src2[0]);
                meta_gen_set_cf_nz(dst[0]);
                meta_gen_clear_cf_vc();
            }
            meta_gen_set_scc(dc, false);
        }
    } else {
        TCGv tmp1 = tcg_temp_new();

        if (inst->dsp && (am == META_DUARITH_SPLIT16) &&
            (ct->ext_op == OP_ABS)) {
            /* split-16 ABS */
            tcg_gen_shri_i32(tmp1, sdata.src1[0], 16);
            tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_cf[META_SCF_HZ],
                                 tmp1, 0x8000);
            tcg_gen_setcondi_i32(TCG_COND_GTU, cpu_cf[META_SCF_HC],
                                 tmp1, 0x8000);

            tcg_gen_ext16u_i32(tmp1, sdata.src1[0]);
            tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_cf[META_SCF_LZ],
                                 tmp1, 0x8000);
            tcg_gen_setcondi_i32(TCG_COND_GTU, cpu_cf[META_SCF_LC],
                                 tmp1, 0x8000);
        } else if (inst->dsp && (am == META_DUARITH_SPLIT16) &&
                   (ct->ext_op == OP_MIN || ct->ext_op == OP_MAX)) {
            /* split-16 MIN,MAX */
            /* handled later */
            meta_gen_set_scc(dc, true);
        } else if (inst->dsp && (am == META_DUARITH_SPLIT16)) {
            /* split-16 non-ABS non-MIN,MAX */
            TCGv tmp2 = tcg_temp_new();

            if (ct->src2_cmp.type == SRC_IMM) {
                tcg_gen_movi_i32(tmp2, s2_imm);
            }

            tcg_gen_sari_i32(tmp1, sdata.src1[0], 16);
            if (ct->src2_cmp.type != SRC_IMM) {
                tcg_gen_shri_i32(tmp2, sdata.src2[0], 16);
            }
            meta_gen_dsp_add_i16(dc, true,
                                 cpu_cf[META_SCF_HC], NULL,
                                 NULL, cpu_cf[META_SCF_HZ],
                                 tmp1, tmp1, tmp2, false, false);

            tcg_gen_ext16s_i32(tmp1, sdata.src1[0]);
            if (ct->src2_cmp.type != SRC_IMM) {
                tcg_gen_ext16u_i32(tmp2, sdata.src2[0]);
            }
            meta_gen_dsp_add_i16(dc, true,
                                 cpu_cf[META_SCF_LC], NULL,
                                 NULL, cpu_cf[META_SCF_LZ],
                                 tmp1, tmp1, tmp2, false, false);

            meta_gen_set_scc(dc, true);
        } else if ((inst->src1.type == inst->src2.type) &&
                (inst->src1.u == inst->src2.u) &&
                (inst->src1.i == inst->src2.i)) {
            /* non-split-16 CMP X,X: cf=Znoc */
            meta_gen_set_cf_nz_imm(0);
            meta_gen_clear_cf_vc();
            meta_gen_set_scc(dc, false);
        } else if (!inst->dsp && (ct->src2_cmp.type == SRC_IMM)) {
            /* non-DSP CMP with immediate */
            meta_gen_set_cf_subi(sdata.src1[0], s2_imm);
            meta_gen_set_scc(dc, false);
        } else {
            /* non-split-16 */
            TCGv s2;

            if (ct->src2_cmp.type == SRC_IMM) {
                s2 = meta_const_i32(s2_imm);
            } else {
                s2 = sdata.src2[0];
            }

            meta_gen_dsp_add_i32(dc, true, true, tmp1, sdata.src1[0], s2,
                                 false,      /* no product shift */
                                 inst->dsp); /* DSP can saturate */

        }

    }

    /* perform extended op */
    for (di = 0; di <= inst->dual; di++) {
        if (inst->dsp && (am == META_DUARITH_SPLIT16)) {
            switch (ct->ext_op) {
            case OP_NONE:
                break;

            case OP_ABS: {
                    TCGv low = tcg_temp_new();
                    TCGv high = tcg_temp_new();

                    /* low */
                    tcg_gen_shli_i32(low, sdata.src1[di], 16);
                    meta_gen_abs_i32(dc, low, low,
                                     inst->dsp &&
                                     (dc->tbflags & META_TBFLAG_DUSATURATION));

                    /* high */
                    tcg_gen_andi_i32(high, sdata.src1[di], 0xffff0000);
                    meta_gen_abs_i32(dc, high, high,
                                     inst->dsp &&
                                     (dc->tbflags & META_TBFLAG_DUSATURATION));

                    /* pack result */
                    tcg_gen_shri_i32(low, low, 16);
                    tcg_gen_deposit_i32(dst[di], high, low, 0, 16);

                    break;
                }

            case OP_FFB:
            case OP_NORM: {
                    TCGv low = tcg_temp_new();
                    TCGv high = tcg_temp_new();

                    /* extract halves */
                    tcg_gen_sari_i32(high, sdata.src1[di], 16);
                    tcg_gen_ext16s_i32(low, sdata.src1[di]);

                    /* perform op */
                    if (ct->ext_op == OP_FFB) {
                        gen_helper_ffb16(low, low);
                        gen_helper_ffb16(high, high);
                    } else {
                        gen_helper_norm16(low, low);
                        gen_helper_norm16(high, high);
                    }

                    /* pack result */
                    tcg_gen_deposit_i32(dst[di], low, high, 16, 16);

                    break;
                }

            case OP_MAX:
            case OP_MIN: {
                    TCGv low1 = tcg_temp_new();
                    TCGv low2 = tcg_temp_new();
                    TCGv high1 = tcg_temp_new();
                    TCGv high2 = tcg_temp_new();
                    TCGCond cond;

                    if (ct->ext_op == OP_MIN) {
                        cond = TCG_COND_LT;
                    } else {
                        cond = TCG_COND_GT;
                    }

                    /* extract halves */
                    tcg_gen_sari_i32(high1, sdata.src1[di], 16);
                    tcg_gen_ext16s_i32(low1, sdata.src1[di]);
                    tcg_gen_sari_i32(high2, sdata.src2[di], 16);
                    tcg_gen_ext16s_i32(low2, sdata.src2[di]);

                    /* set flags */
                    tcg_gen_setcond_i32(TCG_COND_EQ, cpu_cf[META_SCF_LZ],
                                        low1, low2);
                    tcg_gen_setcond_i32(TCG_COND_LT, cpu_cf[META_SCF_LC],
                                        low1, low2);
                    tcg_gen_setcond_i32(TCG_COND_EQ, cpu_cf[META_SCF_HZ],
                                        high1, high2);
                    tcg_gen_setcond_i32(TCG_COND_LT, cpu_cf[META_SCF_HC],
                                        high1, high2);

                    /* perform op */
                    tcg_gen_movcond_i32(cond, low1,
                                        low1, low2, low1, low2);
                    tcg_gen_movcond_i32(cond, high1,
                                        high1, high2, high1, high2);

                    /* pack result */
                    tcg_gen_deposit_i32(dst[di], low1, high1, 16, 16);

                    break;
                }

            case OP_NMIN: {
                    TCGv low1 = tcg_temp_new();
                    TCGv low2 = tcg_temp_new();
                    TCGv high1 = tcg_temp_new();
                    TCGv high2 = tcg_temp_new();

                    /* extract halves */
                    tcg_gen_sari_i32(high1, sdata.src1[di], 16);
                    tcg_gen_ext16s_i32(low1, sdata.src1[di]);
                    tcg_gen_sari_i32(high2, sdata.src2[di], 16);
                    tcg_gen_ext16s_i32(low2, sdata.src2[di]);

                    /* NORM */
                    gen_helper_norm16(low1, low1);
                    gen_helper_norm16(high1, high1);

                    /* MIN */
                    tcg_gen_movcond_i32(TCG_COND_LT, low1,
                                        low1, low2, low1, low2);
                    tcg_gen_movcond_i32(TCG_COND_LT, high1,
                                        high1, high2, high1, high2);

                    /* pack result */
                    tcg_gen_deposit_i32(dst[di], low1, high1, 16, 16);

                    break;
                }

            default:
                meta_unimplemented(dc, "unhandled split-16 ext_op");
            }
        } else {
            switch (ct->ext_op) {
            case OP_NONE:
                break;

            case OP_ABS:
                meta_gen_abs_i32(dc, dst[di], sdata.src1[di],
                                 inst->dsp &&
                                 (dc->tbflags & META_TBFLAG_DUSATURATION));
                break;

            case OP_FFB:
                gen_helper_ffb(dst[di], sdata.src1[di]);
                break;

            case OP_NORM:
                gen_helper_norm(dst[di], sdata.src1[di]);
                break;

            case OP_MIN:
                tcg_gen_movcond_i32(TCG_COND_LT, dst[di],
                                    sdata.src1[di], sdata.src2[di],
                                    sdata.src1[di], sdata.src2[di]);
                break;

            case OP_MAX:
                tcg_gen_movcond_i32(TCG_COND_GT, dst[di],
                                    sdata.src1[di], sdata.src2[di],
                                    sdata.src1[di], sdata.src2[di]);
                break;

            case OP_NMIN: {
                    TCGv tmp1 = tcg_temp_new();

                    /* NORM */
                    gen_helper_norm(tmp1, sdata.src1[di]);

                    /* MIN */
                    tcg_gen_movcond_i32(TCG_COND_LT, dst[di],
                                        tmp1, sdata.src2[di],
                                        tmp1, sdata.src2[di]);

                    break;
                }

            default:
                meta_unimplemented(dc, "unhandled ext_op");
            }
        }
    }

    /* if necessary, write the result */
    if (inst->dst.type != DST_NONE) {
        meta_set_dest(dc, inst->dst, dst[0]);
        if (inst->dual) {
            meta_set_dest(dc, meta_hi_dest(inst->dst), dst[1]);
        }
    }

    /* clean up */
    meta_cleanup_sources(dc, &sdata, inst);

    /* place the label for failed conditions */
    if (inst->cc) {
        meta_end_cc(dc, &lbl_cc);
    }
}

static void meta_gen_dspreg_init(DisasContext *dc, const MetaInstruction *inst)
{
    MetaSrcData sdata;
    int di;

    /* read sources */
    meta_read_sources(dc, &sdata, inst,
                      inst->src1.type != SRC_IMM,
                      false);

    /* perform register init */
    for (di = 0; di <= inst->dual; di++) {
        MetaUnit u = inst->dst.u;

        if (di) {
            u = meta_unit_partner(u);
        }

        if (inst->dst.i & 0x10) {
            /* accumulator */
            TCGv_i64 acc = meta_get_acc(dc, u, inst->dst.i & 0x3);

            if (dc->tbflags & META_TBFLAG_DUACCSAT) {
                /* shift left 8 bits */
                if (inst->src1.type != SRC_IMM) {
                    tcg_gen_extu_i32_i64(acc, sdata.src1[di]);
                    tcg_gen_shli_i64(acc, acc, 32);
                    tcg_gen_sari_i64(acc, acc, 24);
                } else {
                    tcg_gen_movi_i64(acc, inst->src1.i << 8);
                }
            } else {
                /* sign extend 32-bits */
                if (inst->src1.type != SRC_IMM) {
                    tcg_gen_ext_i32_i64(acc, sdata.src1[di]);
                } else {
                    tcg_gen_movi_i64(acc, inst->src1.i);
                }
            }
        } else if (inst->dst.i & 0x8) {
            /* DSP RAM increment */
            int bank = (inst->dst.i >> 2) & 0x1;
            bool write = (inst->dst.i >> 1) & 0x1;
            int inc_idx = inst->dst.i & 0x1;
            int du = u - META_UNIT_D0;
            uint32_t mask;
            TCGv reg;

            if (write) {
                reg = cpu_dspram_wpi[du][bank][inc_idx];
                mask = 0xffff;
            } else {
                reg = cpu_dspram_rpi[du][bank][inc_idx];
                mask = 0x33dfffff;
            }

            if (inst->src1.type != SRC_IMM) {
                tcg_gen_andi_i32(reg, sdata.src1[di], mask);
            } else {
                tcg_gen_movi_i32(reg, inst->src1.i & mask);
            }
        } else {
            /* DSP RAM pointer */
            int bank = (inst->dst.i >> 2) & 0x1;
            bool write = (inst->dst.i >> 1) & 0x1;
            int ptr_idx = inst->dst.i & 0x1;
            int du = u - META_UNIT_D0;
            TCGv rdu = meta_const_i32(du);
            TCGv rbank = meta_const_i32(bank);
            TCGv ridx = meta_const_i32(ptr_idx);
            TCGv rval = tcg_temp_new();
            TCGv_ptr rnull = tcg_constant_ptr(0);

            if (inst->src1.type != SRC_IMM) {
                tcg_gen_ext16u_i32(rval, sdata.src1[di]);
            } else {
                tcg_gen_movi_i32(rval, inst->src1.i & 0xffff);
            }

            if (write) {
                gen_helper_dspram_ptr_w_set(tcg_env, rdu, rbank, ridx,
                                            rval, rnull);
            } else {
                gen_helper_dspram_ptr_r_set(rval, /* clobber */
                                            tcg_env, rdu, rbank, ridx,
                                            rval, rnull);
            }

        }
    }

    /* clean up */
    meta_cleanup_sources(dc, &sdata, inst);
}

static void meta_gen_dspreg_copy(DisasContext *dc, const MetaInstruction *inst)
{
    bool dst_tmp[2] = { 0 };
    TCGv dst[2];
    int di;

    /* get destination */
    dst[0] = meta_get_dest(dc, inst->dst, &dst_tmp[0]);
    if (inst->dual) {
        dst[1] = meta_get_dest(dc, meta_hi_dest(inst->dst), &dst_tmp[1]);
    }

    /* perform register copy */
    for (di = 0; di <= inst->dual; di++) {
        MetaUnit u = inst->dst.u;

        if (di) {
            u = meta_unit_partner(u);
        }

        if (inst->src1.i & 0x10) {
            /* accumulator */
            TCGv_i64 acc = meta_get_acc(dc, u, inst->src1.i & 0x3);

            if (dc->tbflags & META_TBFLAG_DUACCSAT) {
                /* shift right 8 bits */
                TCGv_i64 tmp1 = tcg_temp_new_i64();
                tcg_gen_shri_i64(tmp1, acc, 8);
                tcg_gen_extrl_i64_i32(dst[di], tmp1);
            } else {
                tcg_gen_extrl_i64_i32(dst[di], acc);
            }
        } else {
            /* DSP RAM pointer or increment */
            bool inc = (inst->src1.i >> 3) & 0x1;
            int bank = (inst->src1.i >> 2) & 0x1;
            bool write = (inst->src1.i >> 1) & 0x1;
            int idx = inst->src1.i & 0x1;
            int du = u - META_UNIT_D0;
            TCGv reg;

            if (write && inc) {
                reg = cpu_dspram_wpi[du][bank][idx];
            } else if (write) {
                reg = cpu_dspram_wp[du][bank][idx];
            } else if (inc) {
                reg = cpu_dspram_rpi[du][bank][idx];
            } else {
                reg = cpu_dspram_rp[du][bank][idx];
            }

            tcg_gen_mov_i32(dst[di], reg);
        }
    }

    /* if necessary, write the result */
    meta_set_dest(dc, inst->dst, dst[0]);
    if (inst->dual) {
        meta_set_dest(dc, meta_hi_dest(inst->dst), dst[1]);
    }
}

static void meta_gen_dspreg_get(DisasContext *dc, const MetaInstruction *inst)
{
    const MetaInstructionDspGetSet *gs = &inst->dspgetset;
    bool a_base_tmp = false;
    TCGv a_base, data[2] = { NULL, NULL };
    int di;

    /* retrieve the address */
    a_base = meta_get_source(dc, gs->a_base, &a_base_tmp);

    /* this should always be a standard A register */
    assert(!a_base_tmp);

    data[0] = tcg_temp_new();
    if (gs->l1) {
        data[1] = tcg_temp_new();
    }

    /* perform the load */
    gen_load(dc, data[0], data[1], a_base, 2 + gs->l1);

    /* store values in the DSP registers */
    for (di = 0; di <= gs->l1; di++) {
        MetaUnit u = inst->dst.u;

        if (di) {
            u = meta_unit_partner(u);
        }

        if (inst->dst.i & 0x10) {
            /* accumulator */
            bool top = (inst->dst.i >> 3) & 0x1;
            TCGv_i64 tmp1 = tcg_temp_new_i64();
            TCGv_i64 acc = meta_get_acc(dc, u, inst->dst.i & 0x3);

            tcg_gen_extu_i32_i64(tmp1, data[di]);

            if (top) {
                tcg_gen_deposit_i64(acc, acc, tmp1, 32, 8);

                /* sign extend to 64-bit */
                tcg_gen_shli_i64(acc, acc, 24);
                tcg_gen_sari_i64(acc, acc, 24);
            } else {
                tcg_gen_deposit_i64(acc, acc, tmp1, 0, 32);
            }

        } else if (inst->dst.i & 0x8) {
            /* DSP RAM increment */
            int bank = (inst->dst.i >> 2) & 0x1;
            bool write = (inst->dst.i >> 1) & 0x1;
            int inc_idx = inst->dst.i & 0x1;
            int du = u - META_UNIT_D0;
            uint32_t mask;
            TCGv reg;

            if (write) {
                reg = cpu_dspram_wpi[du][bank][inc_idx];
                mask = 0xffff;
            } else {
                reg = cpu_dspram_rpi[du][bank][inc_idx];
                mask = 0x33dfffff;
            }

            tcg_gen_andi_i32(reg, data[di], mask);
        } else {
            /* DSP RAM pointer */
            int bank = (inst->dst.i >> 2) & 0x1;
            bool write = (inst->dst.i >> 1) & 0x1;
            int ptr_idx = inst->dst.i & 0x1;
            int du = u - META_UNIT_D0;
            TCGv rdu = meta_const_i32(du);
            TCGv rbank = meta_const_i32(bank);
            TCGv ridx = meta_const_i32(ptr_idx);
            TCGv rval = tcg_temp_new();
            TCGv_ptr rnull = tcg_constant_ptr(0);

            tcg_gen_ext16u_i32(rval, data[di]);

            if (write) {
                gen_helper_dspram_ptr_w_set(tcg_env, rdu, rbank, ridx,
                                            rval, rnull);
            } else {
                gen_helper_dspram_ptr_r_set(rval, /* clobber */
                                            tcg_env, rdu, rbank, ridx,
                                            rval, rnull);
            }

        }
    }


    if (gs->a_off.type != SRC_IMM) {
        bool a_off_tmp = false;
        TCGv a_off = meta_get_source(dc, gs->a_off, &a_off_tmp);

        assert(!a_off_tmp);
        meta_gen_addr_inc(dc, a_base, a_base, a_off, gs->a_base.u);
    } else if (gs->a_off.i) {
        meta_gen_addr_inci(dc, a_base, a_base,
                           gs->a_off.i << (2 + gs->l1), gs->a_base.u);
    }
}

static void meta_gen_dspreg_set(DisasContext *dc, const MetaInstruction *inst)
{
    const MetaInstructionDspGetSet *gs = &inst->dspgetset;
    bool a_base_tmp = false;
    TCGv a_base;
    int di;
    TCGv data[2];

    /* retrieve the address */
    a_base = meta_get_source(dc, gs->a_base, &a_base_tmp);

    /* this should always be a standard A register */
    assert(!a_base_tmp);

    /* select the DSP registers */
    for (di = 0; di <= gs->l1; di++) {
        MetaUnit u = inst->src1.u;

        if (di) {
            u = meta_unit_partner(u);
        }

        if (inst->src1.i & 0x10) {
            /* accumulator */
            bool top = (inst->src1.i >> 3) & 0x1;
            TCGv_i64 acc = meta_get_acc(dc, u, inst->src1.i & 0x3);

            data[di] = tcg_temp_new();

            if (top) {
                TCGv_i64 tmp1 = tcg_temp_new_i64();
                tcg_gen_shri_i64(tmp1, acc, 32);
                tcg_gen_andi_i64(tmp1, tmp1, 0xff);
                tcg_gen_extrl_i64_i32(data[di], tmp1);
            } else {
                tcg_gen_extrl_i64_i32(data[di], acc);
            }
        } else {
            /* DSP RAM pointer or increment */
            bool inc = (inst->src1.i >> 3) & 0x1;
            int bank = (inst->src1.i >> 2) & 0x1;
            bool write = (inst->src1.i >> 1) & 0x1;
            int idx = inst->src1.i & 0x1;
            int du = u - META_UNIT_D0;

            if (write && inc) {
                data[di] = cpu_dspram_wpi[du][bank][idx];
            } else if (write) {
                data[di] = cpu_dspram_wp[du][bank][idx];
            } else if (inc) {
                data[di] = cpu_dspram_rpi[du][bank][idx];
            } else {
                data[di] = cpu_dspram_rp[du][bank][idx];
            }
        }
    }

    /* perform the store */
    gen_store(dc, data[0], data[1], a_base, 2 + gs->l1);


    if (gs->a_off.type != SRC_IMM) {
        bool a_off_tmp = false;
        TCGv a_off = meta_get_source(dc, gs->a_off, &a_off_tmp);

        assert(!a_off_tmp);
        meta_gen_addr_inc(dc, a_base, a_base, a_off, gs->a_base.u);
    } else if (gs->a_off.i) {
        meta_gen_addr_inci(dc, a_base, a_base,
                           gs->a_off.i << (2 + gs->l1), gs->a_base.u);
    }
}

static void meta_gen_xsd(DisasContext *dc, const MetaInstruction *inst)
{
    const MetaInstructionXsd *xsd = &inst->xsd;
    MetaSrcData sdata;
    TCGv dst[2];
    bool dst_tmp[2] = { 0 };
    int di;

    /* read sources */
    meta_read_sources(dc, &sdata, inst, true, false);

    /* get destination */
    dst[0] = meta_get_dest(dc, inst->dst, &dst_tmp[0]);
    if (inst->dual) {
        dst[1] = meta_get_dest(dc, meta_hi_dest(inst->dst), &dst_tmp[1]);
    }

    /* perform op */
    for (di = 0; di <= inst->dual; di++) {
        switch (xsd->width) {
        case XSDB:
            tcg_gen_ext8s_i32(dst[di], sdata.src1[di]);
            break;

        case XSDW:
            tcg_gen_ext16s_i32(dst[di], sdata.src1[di]);
            break;
        }
    }

    /* set flags */
    if (xsd->setflags) {
        meta_gen_set_cf_nz(dst[0]);
        meta_gen_clear_cf_vc();
        meta_gen_set_scc(dc, false);
    }

    /* if necessary, write the result */
    meta_set_dest(dc, inst->dst, dst[0]);
    if (inst->dual) {
        meta_set_dest(dc, meta_hi_dest(inst->dst), dst[1]);
    }

    /* clean up */
    meta_cleanup_sources(dc, &sdata, inst);
}

static void meta_gen_bex(DisasContext *dc, const MetaInstruction *inst)
{
    const MetaInstructionBex *bex = &inst->bex;
    MetaSrcData sdata;
    TCGv dst[2];
    bool dst_tmp[2] = { 0 };

    /* read sources */
    meta_read_sources(dc, &sdata, inst, true, false);

    /* get destination */
    dst[0] = meta_get_dest(dc, inst->dst, &dst_tmp[0]);
    if (inst->dual) {
        dst[1] = meta_get_dest(dc, meta_hi_dest(inst->dst), &dst_tmp[1]);
    } else {
        dst[1] = NULL;
    }

    /* perform op */
    if (inst->dual &&
        (inst->dst.i == inst->src1.i)) {
        /* 64-bit, careful about clobbering source data */
        TCGv tmp = tcg_temp_new();
        tcg_gen_bswap32_i32(tmp, sdata.src1[0]);
        tcg_gen_bswap32_i32(dst[0], sdata.src1[1]);
        tcg_gen_mov_i32(dst[1], tmp);
    } else if (inst->dual) {
        /* 64-bit, no need to be so careful */
        tcg_gen_bswap32_i32(dst[0], sdata.src1[1]);
        tcg_gen_bswap32_i32(dst[1], sdata.src1[0]);
    } else {
        tcg_gen_bswap32_i32(dst[0], sdata.src1[0]);
    }

    /* set flags */
    if (bex->setflags) {
        meta_gen_set_cf_nz(dst[0]);
        meta_gen_clear_cf_vc();
        meta_gen_set_scc(dc, false);
    }

    /* if necessary, write the result */
    meta_set_dest(dc, inst->dst, dst[0]);
    if (inst->dual) {
        meta_set_dest(dc, meta_hi_dest(inst->dst), dst[1]);
    }

    /* clean up */
    meta_cleanup_sources(dc, &sdata, inst);
}

static void meta_gen_rtd(DisasContext *dc, const MetaInstruction *inst)
{
    const MetaInstructionRtd *rtd = &inst->rtd;
    MetaSrcData sdata;
    TCGv dst[2], tmp1;
    bool dst_tmp[2] = { 0 };
    int di;

    /* read sources */
    meta_read_sources(dc, &sdata, inst, true, false);

    /* get destination */
    dst[0] = meta_get_dest(dc, inst->dst, &dst_tmp[0]);
    if (inst->dual) {
        dst[1] = meta_get_dest(dc, meta_hi_dest(inst->dst), &dst_tmp[1]);
    }

    tmp1 = tcg_temp_new();

    /* perform op */
    for (di = 0; di <= inst->dual; di++) {
        tcg_gen_shri_i32(tmp1, sdata.src1[di], 16);
        tcg_gen_shli_i32(dst[di], sdata.src1[di], 16);
        tcg_gen_or_i32(dst[di], dst[di], tmp1);
    }


    /* set flags */
    if (rtd->setflags) {
        meta_gen_set_cf_nz(dst[0]);
        meta_gen_clear_cf_vc();
        meta_gen_set_scc(dc, false);
    }

    /* if necessary, write the result */
    meta_set_dest(dc, inst->dst, dst[0]);
    if (inst->dual) {
        meta_set_dest(dc, meta_hi_dest(inst->dst), dst[1]);
    }

    /* clean up */
    meta_cleanup_sources(dc, &sdata, inst);
}

static void meta_gen_porttounit(DisasContext *dc, const MetaInstruction *inst)
{
    const MetaInstructionPortToUnit *ptu = &inst->porttounit;
    DEFINE_SAFE_LABEL(lbl_cc);
    TCGv dst[2];
    bool dst_tmp[2] = { 0 };

    if (inst->cc) {
        /* read to a temp in case condition is false */
        dst[0] = tcg_temp_new();
        dst_tmp[0] = true;

        if (ptu->width == WIDTH_64B) {
            dst[1] = tcg_temp_new();
            dst_tmp[1] = true;
        } else {
            dst[1] = NULL;
        }
    } else {
        /* get destination */
        dst[0] = meta_get_dest(dc, inst->dst, &dst_tmp[0]);

        if (ptu->width == WIDTH_64B) {
            dst[1] = meta_get_dest(dc, ptu->dst2, &dst_tmp[1]);
        } else {
            dst[1] = NULL;
        }
    }

    /* perform the read */
    if (inst->src1.type == SRC_COPROC) {
        meta_unimplemented(dc, "co-processor read");
    } else if (ptu->width == WIDTH_64B) {
        TCGv_i64 data = tcg_temp_new_i64();
        meta_gen_rd_read_i64(dc, data);
        tcg_gen_extr_i64_i32(dst[0], dst[1], data);
    } else {
        meta_gen_rd_read_i32(dc, dst[0]);
    }

    /* branch if condition fails */
    if (inst->cc) {
        meta_start_cc(dc, inst->cc, &lbl_cc);
    }

    /* truncate data */
    switch (ptu->width) {
    case WIDTH_64B:
    case WIDTH_32B:
        break;

    case WIDTH_16B:
        tcg_gen_ext16u_i32(dst[0], dst[0]);
        break;

    case WIDTH_8B:
        tcg_gen_ext8u_i32(dst[0], dst[0]);
        break;
    }

    /* if necessary, write the result */
    meta_set_dest(dc, inst->dst, dst[0]);
    if (ptu->width == WIDTH_64B) {
        meta_set_dest(dc, ptu->dst2, dst[1]);
    }

    /* cleanup */

    /* place the label for failed conditions */
    if (inst->cc) {
        meta_end_cc(dc, &lbl_cc);
    }
}

static void meta_gen_branch(DisasContext *dc, const MetaInstruction *inst)
{
    const MetaInstructionBranch *br = &inst->branch;
    DEFINE_SAFE_LABEL(lbl_end);
    target_ulong jmp_pc = dc->pc + br->offset;

    if (br->repeat) {
        TCGv txrpt = cpu_cregs[META_TXRPT];
        meta_init_safe_label(dc, &lbl_end);
        tcg_gen_brcondi_i32(TCG_COND_EQ, txrpt, 0, lbl_end.label);
        tcg_gen_subi_i32(txrpt, txrpt, 1);

        if (inst->cc &&
            (((dc->tbflags & META_TBFLAG_TXL2COUNTNZ) &&
              (dc->pc == dc->env->cregs[META_TXL2END])) ||
             ((dc->tbflags & META_TBFLAG_TXL1COUNTNZ) &&
              (dc->pc == dc->env->cregs[META_TXL1END])))) {
            /* repeating branches with false conditions and which end a
             * hardware loop cause the hardware loop branch to be taken
             * without decrementing the hardware loop counter */
            TCGLabel *lbl_repeathwldone = gen_new_label();
            meta_gen_br_cc_pass(dc, inst->cc, lbl_repeathwldone);
            if ((dc->tbflags & META_TBFLAG_TXL2COUNTNZ) &&
                (dc->pc == dc->env->cregs[META_TXL2END])) {
                gen_goto_tb(dc, 0, dc->env->cregs[META_TXL2START]);
                dc->is_jmp = DISAS_TB_JUMP;
            } else {
                gen_goto_tb(dc, 0, dc->env->cregs[META_TXL1START]);
                dc->is_jmp = DISAS_TB_JUMP;
            }
            gen_set_label(lbl_repeathwldone);
        }
    }

    if (likely(jmp_pc != dc->next_pc && inst->cc != META_CC_NV)) {
        /* branch may change PC */
        if (inst->cc) {
            meta_init_safe_label(dc, &lbl_end);
            /* branch if condition fails */
            meta_gen_br_cc_fail(dc, inst->cc, lbl_end.label);
        }
        gen_goto_tb(dc, 0, jmp_pc);
        dc->is_jmp = DISAS_TB_JUMP;
    }

    /* place the label for failed conditions */
    meta_set_safe_label(dc, 1, &lbl_end);

    /* decrement the hardware loops if the condition is false */
    gen_hwloop_dec(dc);
    dc->hwloop_handled = true;
}

static void meta_gen_auaddsub(DisasContext *dc, const MetaInstruction *inst)
{
    const MetaInstructionAUAddSub *as = &inst->auaddsub;
    DEFINE_SAFE_LABEL(lbl_cc);
    MetaSrcData sdata;
    TCGv dst;
    bool s1_isimm = as->pc_src1 || (inst->src1.type == SRC_IMM);
    bool s2_isimm = as->pc_src2 || (inst->src2.type == SRC_IMM);
    uint32_t imm1 = inst->src1.i;
    uint32_t imm2 = inst->src2.i;
    bool dst_tmp = false;
    bool nonlinear = meta_au_nonlinear(dc, inst->src1.u - META_UNIT_A0);

    /* branch if condition fails */
    if (inst->cc) {
        meta_start_cc(dc, inst->cc, &lbl_cc);
    }

    /* get destination */
    dst = meta_get_dest(dc, inst->dst, &dst_tmp);

    /* we'll call a helper so force temps */
    if (nonlinear && !as->pc_src1) {
        s1_isimm = false;
    }
    if (nonlinear && !as->pc_src2) {
        s2_isimm = false;
    }

    /* read sources */
    meta_read_sources(dc, &sdata, inst, !s1_isimm, !s2_isimm);

    /* handle PC overrides */
    if (as->pc_src1) {
        if (nonlinear) {
            sdata.src1_tmp[0] = true;
            sdata.src1[0] = meta_const_i32(dc->pc);
        } else {
            imm1 = dc->pc;
        }
    }
    if (as->pc_src2) {
        if (nonlinear) {
            sdata.src2_tmp[0] = true;
            if (as->sub) {
                sdata.src2[0] = meta_const_i32(-((int32_t)dc->pc));
            } else {
                sdata.src2[0] = meta_const_i32(dc->pc);
            }
        } else {
            imm2 = dc->pc;
        }
    }

    /* perform arithmetic */
    if (nonlinear) {
        /* using a helper to handle modulo or bit-reversed addressing */
        TCGv rau = meta_const_i32(inst->src1.u - META_UNIT_A0);
        TCGv rs2;

        if (as->sub && !as->pc_src2) {
            rs2 = tcg_temp_new();
            tcg_gen_neg_i32(rs2, sdata.src2[0]);
        } else {
            rs2 = sdata.src2[0];
        }

        gen_helper_au_add(dst, tcg_env, rau, sdata.src1[0], rs2);

    } else if (s1_isimm && s2_isimm) {
        /* MOV or NEG */
        if (as->sub) {
            tcg_gen_movi_i32(dst, imm1 - imm2);
        } else {
            tcg_gen_movi_i32(dst, imm1 + imm2);
        }
    } else if (s1_isimm) {
        if (as->sub) {
            tcg_gen_subfi_i32(dst, imm1, sdata.src2[0]);
        } else {
            tcg_gen_addi_i32(dst, sdata.src2[0], imm1);
        }
    } else if (s2_isimm) {
        if (as->sub) {
            tcg_gen_subi_i32(dst, sdata.src1[0], imm2);
        } else {
            tcg_gen_addi_i32(dst, sdata.src1[0], imm2);
        }
    } else {
        if (as->sub) {
            tcg_gen_sub_i32(dst, sdata.src1[0], sdata.src2[0]);
        } else {
            tcg_gen_add_i32(dst, sdata.src1[0], sdata.src2[0]);
        }
    }

    /* if necessary, write the result */
    meta_set_dest(dc, inst->dst, dst);

    /* cleanup dst */

    /* clean up */
    meta_cleanup_sources(dc, &sdata, inst);

    /* place the label for failed conditions */
    if (inst->cc) {
        meta_end_cc(dc, &lbl_cc);
    }
}

static void meta_gen_xfr(DisasContext *dc, const MetaInstruction *inst)
{
    const MetaInstructionXfr *xfr = &inst->xfr;
    int lsmstep = (dc->tbflags & META_TBFLAG_LSM) >> META_TBFLAG_LSM_SHIFT;
    TCGv a_base, a_off, addr;
    bool a_base_tmp = false, a_off_tmp = false;
    bool update_addr;
    MetaUnit addr_unit;

    /* select address for this step */
    if (!lsmstep) {
        a_base = meta_get_source(dc, xfr->base_dst, &a_base_tmp);
        a_off = meta_get_source(dc, xfr->off_dst, &a_off_tmp);
        update_addr = xfr->update_dst;
        addr_unit = xfr->base_dst.u;
    } else {
        a_base = meta_get_source(dc, xfr->base_src, &a_base_tmp);
        a_off = meta_get_source(dc, xfr->off_src, &a_off_tmp);
        update_addr = xfr->update_src;
        addr_unit = xfr->base_src.u;
    }

    /* reliant upon these being direct A registers for update */
    assert(!a_base_tmp && !a_off_tmp);

    /* calculate address, possibly pre-increment */
    if (!update_addr) {
        addr = tcg_temp_new();
        meta_gen_addr_inc(dc, addr, a_base, a_off, addr_unit);
    } else {
        if (!xfr->post) {
            meta_gen_addr_inc(dc, a_base, a_base, a_off, addr_unit);
        }
        addr = a_base;
    }

    if (!lsmstep) {
        /* write step */
        TCGv data[2];

        /* read data from the pipeline */
        if (xfr->l1) {
            TCGv_i64 data64 = tcg_temp_new_i64();
            meta_gen_rd_read_i64(dc, data64);
            data[0] = tcg_temp_new();
            data[1] = tcg_temp_new();
            tcg_gen_extr_i64_i32(data[0], data[1], data64);
        } else {
            data[0] = tcg_temp_new();
            meta_gen_rd_read_i32(dc, data[0]);
            data[1] = NULL;
        }

        /* store the data */
        gen_store(dc, data[0], data[1], addr, 2 + xfr->l1);

        /* tidy up */

        /* re-issue for read step */
        dc->tbflags &= ~META_TBFLAG_LSM;
        dc->tbflags |= 1 << META_TBFLAG_LSM_SHIFT;
        dc->next_pc = dc->pc;
    } else {
        /* read step */

        /* issue the new address */
        meta_port_gen_write[META_RA](dc, addr, WIDTH_32B + xfr->l1);

        /* reset LSMStep */
        dc->tbflags &= ~META_TBFLAG_LSM;
    }

    /* post-increment */
    if (update_addr && xfr->post) {
        meta_gen_addr_inc(dc, a_base, a_base, a_off, addr_unit);
    }

    if (lsmstep) {
        /* ensure looping happens after any post-increment */
        DEFINE_SAFE_LABEL(lend);

        /* all done if !TXRPT, skip to the end */
        meta_init_safe_label(dc, &lend);
        tcg_gen_brcondi_i32(TCG_COND_EQ, cpu_cregs[META_TXRPT], 0, lend.label);
        tcg_gen_subi_i32(cpu_cregs[META_TXRPT], cpu_cregs[META_TXRPT], 1);

        /* else loop back to this instruction */
        gen_goto_tb(dc, 0, dc->pc);
        dc->is_jmp = DISAS_TB_JUMP;

        /* et voila, all done! */
        meta_set_safe_label(dc, 1, &lend);
    }
}

static void meta_gen_addr_source_inc(DisasContext *dc, TCGv dst, TCGv addr,
                                     const MetaInstructionSource *off,
                                     TCGv off_reg, int32_t off_int)
{
    if (off->type == SRC_NONE) {
        tcg_gen_mov_i32(dst, addr);
    } else if (off->type == SRC_IMM) {
        meta_gen_addr_inci(dc, dst, addr, off_int, off->u);
    } else {
        meta_gen_addr_inc(dc, dst, addr, off_reg, off->u);
    }
}

static void meta_gen_store_mx(DisasContext *dc, TCGv addr_base,
                              TCGv low, TCGv high, bool dual)
{
    bool have_high = high != NULL;
    TCGv tmp = tcg_temp_new();
    TCGv addr = tcg_temp_new();

    if (dc->tbflags & META_TBFLAG_M8) {
        if (meta_dc_duarithmode(dc) == META_DUARITH_SPLIT16) {
            /* 8-bits from each 16-bit quarter */
            tcg_gen_ext8u_i32(tmp, low);
            tcg_gen_qemu_st_i32(tmp, addr_base, IS_USER(dc), meta_mo(dc, MO_UB));

            tcg_gen_addi_i32(addr, addr_base, 1);
            tcg_gen_shri_i32(tmp, low, 16);
            tcg_gen_ext8u_i32(tmp, tmp);
            tcg_gen_qemu_st_i32(tmp, addr, IS_USER(dc), meta_mo(dc, MO_UB));

            if (dual) {
                tcg_gen_addi_i32(addr, addr, 1);
                if (have_high) {
                    tcg_gen_ext8u_i32(tmp, high);
                } else {
                    tcg_gen_movi_i32(tmp, 0);
                }
                tcg_gen_qemu_st_i32(tmp, addr, IS_USER(dc), meta_mo(dc, MO_UB));

                tcg_gen_addi_i32(addr, addr, 1);
                if (have_high) {
                    tcg_gen_shri_i32(tmp, high, 16);
                    tcg_gen_ext8u_i32(tmp, tmp);
                }
                tcg_gen_qemu_st_i32(tmp, addr, IS_USER(dc), meta_mo(dc, MO_UB));
            }
        } else {
            /* 8-bits from each half */
            tcg_gen_ext8u_i32(tmp, low);
            tcg_gen_qemu_st_i32(tmp, addr_base, IS_USER(dc), meta_mo(dc, MO_UB));

            if (dual) {
                tcg_gen_addi_i32(addr, addr_base, 1);
                if (have_high) {
                    tcg_gen_ext8u_i32(tmp, high);
                } else {
                    tcg_gen_movi_i32(tmp, 0);
                }
                tcg_gen_qemu_st_i32(tmp, addr, IS_USER(dc), meta_mo(dc, MO_UB));
            }
        }
    } else {
        /* upper 16-bits of each half */
        tcg_gen_shri_i32(tmp, low, 16);
        tcg_gen_qemu_st_i32(tmp, addr_base, IS_USER(dc), meta_mo(dc, MO_UW));

        if (dual) {
            tcg_gen_addi_i32(addr, addr_base, 2);
            if (have_high) {
                tcg_gen_shri_i32(tmp, high, 16);
            } else {
                tcg_gen_movi_i32(tmp, 0);
            }
            tcg_gen_qemu_st_i32(tmp, addr, IS_USER(dc), meta_mo(dc, MO_UW));
        }
    }
}

static void meta_gen_getset(DisasContext *dc, const MetaInstruction *inst)
{
    const MetaInstructionGet *get = &inst->get;
    MetaInstructionSource step_src = inst->src1;
    MetaInstructionDest step_dst = inst->dst;
    DEFINE_SAFE_LABEL(lbl_cc);
    TCGv a_base, addr, off = NULL;
    bool a_base_tmp = false, addr_tmp = false, off_tmp = false;
    TCGv dat[2];
    bool dat_tmp[2] = { 0 };
    TCGLabel *lbl_lnkout = NULL;
    int offi = get->off.i;
    bool l1 = get->width == WIDTH_64B;

    /* branch if condition fails */
    if (inst->cc) {
        meta_start_cc(dc, inst->cc, &lbl_cc);
    }

    /* skip to the correct registers for this step */
    if (inst->multistep) {
        step_dst.i += dc->lsm_idx * meta_lsm_reg_inc(step_dst.u, get->width);
        step_src.i += dc->lsm_idx * meta_lsm_reg_inc(step_src.u, get->width);
    }

    /* get destination */
    if ((inst->op == OP_GET) &&
        (get->read != READ_PRIME) &&
        (get->read != READ_FLUSH) &&
        (inst->dst.u != META_UNIT_RA)) {
        dat[0] = meta_get_dest(dc, step_dst, &dat_tmp[0]);
        if (get->width == WIDTH_64B) {
            dat[1] = meta_get_dest(dc, meta_hi_dest(step_dst), &dat_tmp[1]);
        } else {
            dat[1] = NULL;
        }
    } else {
        dat[0] = NULL;
        dat[1] = NULL;
    }

    /* get address */
    a_base = meta_get_source(dc, get->base, &a_base_tmp);

    /* reliant upon this being a direct A register for update */
    assert(!a_base_tmp);

    /* get offset */
    if (get->off.type == SRC_REG) {
        off = meta_get_source(dc, get->off, &off_tmp);

        if (inst->op == OP_GET) {
            bool off_clobber = (dat[0] == off);
            off_clobber |= (get->width == WIDTH_64B) &&
                (dat[1] == off);

            if (off_clobber && !off_tmp) {
                /* ensure offset isn't clobbered */
                TCGv tmp = tcg_temp_new();
                tcg_gen_mov_i32(tmp, off);
                off = tmp;
                off_tmp = true;
            }
        }
    }

    /* update internal catch state */
    if (((inst->op == OP_GET) && (inst->dst.u == META_UNIT_RA)) ||
        ((inst->op == OP_SET) && (inst->src1.u == META_UNIT_RA))) {
        gen_addr_catch_imm(dc, get->update_addr, get->post,
                           0, 0,
                           get->off.i);
    } else if (get->linked || get->off.type == SRC_NONE) {
        gen_addr_catch(dc, get->base.u, get->base.i);
    } else if (get->off.type == SRC_IMM) {
        gen_addr_catch_imm(dc, get->update_addr, get->post,
                           get->base.u, get->base.i,
                           get->off.i);
    } else {
        assert(get->off.type == SRC_REG);
        gen_addr_catch_reg(dc, get->update_addr, get->post,
                           get->base.u, get->base.i,
                           get->off.u, get->off.i);
    }

    /* calculate address, possibly pre-increment */
    if (!get->update_addr &&
        (((get->off.type == SRC_IMM) && !get->off.i) ||
          (get->off.type == SRC_NONE)) &&
        !((get->width == WIDTH_64B) && (a_base == dat[0]))) {
        addr = a_base;
    } else {
        addr = tcg_temp_new();
        addr_tmp = true;
        if (!get->update_addr || !get->post) {
            meta_gen_addr_source_inc(dc, addr, a_base, &get->off, off, offi);
        } else {
            tcg_gen_mov_i32(addr, a_base);
        }
    }

    /* check for LNKSET failure */
    if (get->linked && (inst->op == OP_SET)) {
        int t;
        TCGLabel *lbl_match;
        int thread = dc->env->thread_num;
        TCGv txdefr = cpu_cregs[META_TXDEFR];
        TCGv zero;

        lbl_match = gen_new_label();
        lbl_lnkout = gen_new_label();

        /* if the thread's lnkaddr register doesn't match, the write fails */
        tcg_gen_brcond_i32(TCG_COND_EQ, addr, cpu_lnkaddr[META_T2T_MID],
                           lbl_match);

        /* Update bus error state in TXDEFR to indicate failure */
        tcg_gen_andi_i32(txdefr, txdefr, ~META_TXDEFR_BUSERR_ALL_MASK);
        tcg_gen_ori_i32(txdefr, txdefr, META_TXDEFR_LNKSET_FAIL);

        tcg_gen_br(lbl_lnkout);
        gen_set_label(lbl_match);

        /* if another thread already has this lnkaddr, we won, so unlink them */
        zero = meta_const_i32(0);
        for (t = 0; t < dc->core->num_threads; t++) {
            TCGv lnkaddr = cpu_lnkaddr[META_T2T(t, thread)];
            if (t == thread) {
                continue;
            }
            tcg_gen_movcond_i32(TCG_COND_EQ, lnkaddr,
                                addr, lnkaddr, zero, lnkaddr);
        }
    }

    if (get->read == READ_PRIME) {
        int port = step_dst.i;
        offi = (4 >> get->mx) << l1;

        /* select the port to issue to */
        if (get->mx) {
            if (dc->tbflags & META_TBFLAG_M8) {
                if (meta_dc_duarithmode(dc) == META_DUARITH_SPLIT16) {
                    port = META_RAM8X;
                } else {
                    port = META_RAM8X32;
                    offi >>= 1;
                }
            } else {
                port = META_RAM16X;
            }
        }

        /* update internal catch state */
        gen_read_catch(dc, false, port);

        /* issue the address */
        meta_port_gen_write[port](dc, addr, readport_info[port].width +
                (l1 && readport_info[port].heed_l1));
    } else if ((get->read == READ_DRAIN) ||
               (get->read == READ_FLUSH)) {
        /* update internal catch state */
        gen_read_catch(dc, false, step_dst.i);

        /* perform the read */
        if (get->read == READ_FLUSH) {
            TCGv_i64 data64 = tcg_temp_new_i64();
            meta_gen_rd_read_i64(dc, data64);
        } else if (get->width == WIDTH_64B) {
            TCGv_i64 data64 = tcg_temp_new_i64();
            meta_gen_rd_read_i64(dc, data64);
            tcg_gen_extr_i64_i32(dat[0], dat[1], data64);
        } else {
            meta_gen_rd_read_i32(dc, dat[0]);
        }
    } else if (inst->op == OP_GET) {
        if (unlikely(!(meta_unit_getset_lens[inst->dst.u] &
                       (1 << get->width)))) {
            meta_illegal(dc, "GET bad register unit for this length");
            return;
        }

        if (inst->dst.u == META_UNIT_RA) {
            /* update internal catch state */
            gen_read_catch(dc, false, step_dst.i);

            /* issue the address */
            meta_port_gen_write[inst->dst.i](dc, addr,
                    readport_info[inst->dst.i].width +
                    readport_info[inst->dst.i].heed_l1);
        } else {
            /* update internal catch state */
            if (get->linked) {
                gen_lnkgetset_catch(dc, false, inst->dst.u, step_dst.i,
                                    get->width);
            } else {
                gen_getset_catch(dc, false, inst->dst.u, step_dst.i,
                                 get->width);
            }

            /* perform the load */
            gen_load(dc, dat[0], dat[1], addr, get->width);
        }
    } else {
        assert(inst->op == OP_SET);

        if (unlikely(!(meta_unit_getset_lens[inst->src1.u] &
                       (1 << get->width)))) {
            meta_illegal(dc, "SET bad register unit for this length");
            return;
        }

        /* source the data */
        if (inst->src1.u == META_UNIT_RA) {
            TCGv_i64 data = tcg_temp_new_i64();
            meta_gen_rd_read_i64(dc, data);
            dat[0] = tcg_temp_new();
            dat_tmp[0] = true;
            if (get->width == WIDTH_64B) {
                dat[1] = tcg_temp_new();
                dat_tmp[1] = true;
                tcg_gen_extr_i64_i32(dat[0], dat[1], data);
            } else {
                tcg_gen_extrl_i64_i32(dat[0], data);
            }
        } else {
            dat[0] = meta_get_source(dc, step_src, &dat_tmp[0]);
            if (get->width == WIDTH_64B) {
                dat[1] = meta_get_source(dc, meta_hi_source(step_src),
                                         &dat_tmp[1]);
            } else {
                dat[1] = NULL;
            }
        }

        /* update internal catch state */
        if (inst->src1.u == META_UNIT_RA) {
            gen_read_catch(dc, true, step_src.i);
        } else if (get->linked) {
            gen_lnkgetset_catch(dc, true, inst->src1.u, step_src.i,
                                get->width);
        } else {
            gen_getset_catch(dc, true, inst->src1.u, step_src.i,
                             get->width);
        }

        /* perform the store */
        if (get->mx) {
            meta_gen_store_mx(dc, addr, dat[0], dat[1], l1);
        } else {
            gen_store(dc, dat[0], dat[1], addr, get->width);
        }
    }

    if ((inst->op == OP_GET) &&
        (get->read != READ_PRIME) &&
        (get->read != READ_FLUSH) &&
        (inst->dst.u != META_UNIT_RA)) {
        /* write the result, if necessary */
        meta_set_dest(dc, step_dst, dat[0]);
        if (get->width == WIDTH_64B) {
            meta_set_dest(dc, meta_hi_dest(step_dst), dat[1]);
        }
    }

    if (get->linked && (inst->op == OP_SET)) {
        TCGv txdefr = cpu_cregs[META_TXDEFR];

        /* Update bus error state in TXDEFR to indicate success */
        tcg_gen_andi_i32(txdefr, txdefr, ~META_TXDEFR_BUSERR_ALL_MASK);
        tcg_gen_ori_i32(txdefr, txdefr, META_TXDEFR_LNKSET_SUCCESS);

        gen_set_label(lbl_lnkout);
        meta_gen_defr_trigger(dc, META_DEFR_BUSERR);
    } else if (get->linked) {
        /* set this thread's lnk address */
        tcg_gen_mov_i32(cpu_lnkaddr[META_T2T_MID], addr);
    }

    /* address increment */
    if (get->update_addr &&
        !(dat[0] == a_base) &&
        !((dat[1] == a_base) && (get->width == WIDTH_64B))) {
        if (get->post) {
            meta_gen_addr_source_inc(dc, a_base, addr, &get->off, off, offi);
        } else {
            assert(addr_tmp);
            /* we can't just increment a_base before the memory access
               because if the access triggers a catch replay we will
               require the original value of a_base... */
            tcg_gen_mov_i32(a_base, addr);
        }
    }

    /* cleanup */

    /* place the label for failed conditions */
    if (inst->cc) {
        meta_end_cc(dc, &lbl_cc);
    }
}

static void meta_gen_multimov(DisasContext *dc, const MetaInstruction *inst)
{
    int lsmstep = (dc->tbflags & META_TBFLAG_LSM) >> META_TBFLAG_LSM_SHIFT;
    MetaInstructionSource step_src = inst->src1;
    MetaInstructionDest step_dst = inst->dst;
    MetaOpWidth width = inst->multimov.width;
    DEFINE_SAFE_LABEL(lbl_cc);
    TCGv src[2];
    bool src_tmp[2] = { 0 };

    /* branch if condition fails */
    if (inst->cc) {
        meta_start_cc(dc, inst->cc, &lbl_cc);
    }

    if (step_dst.u == META_UNIT_FX) {
        step_dst.i += lsmstep * meta_lsm_reg_inc(step_dst.u, width);
    } else {
        step_dst.i += dc->lsm_idx * meta_lsm_reg_inc(step_dst.u, width);
    }

    if (step_src.u == META_UNIT_FX) {
        step_src.i += lsmstep * meta_lsm_reg_inc(step_src.u, width);
    } else {
        step_src.i += dc->lsm_idx * meta_lsm_reg_inc(step_src.u, width);
    }

    /* get source */
    src[0] = meta_get_source(dc, step_src, &src_tmp[0]);
    if (width == WIDTH_64B) {
        src[1] = meta_get_source(dc, meta_hi_source(step_src), &src_tmp[1]);
    } else {
        src[1] = NULL;
    }

    /* set destination */
    meta_set_dest(dc, step_dst, src[0]);
    if (width == WIDTH_64B) {
        meta_set_dest(dc, meta_hi_dest(step_dst), src[1]);
    }

    /* cleanup */

    /* place the label for failed conditions */
    if (inst->cc) {
        meta_end_cc(dc, &lbl_cc);
    }
}

static void meta_gen_unittounit(DisasContext *dc, const MetaInstruction *inst)
{
    const MetaInstructionUnitToUnit *utu = &inst->unittounit;
    DEFINE_SAFE_LABEL(lbl_cc);
    TCGv src[2];
    bool src_tmp[2] = { 0 };

#if !defined(CONFIG_USER_ONLY)
    /* check for magic simulator SWAPNV */
    if (sim_magic &&
        utu->swap &&
        (inst->cc == META_CC_NV) &&
        (inst->dst.u == META_UNIT_D0) &&
        (inst->src1.u == META_UNIT_D1)) {
        switch (inst->src1.i) {
        case 0:
            gen_helper_sim_start(tcg_env);
            dc->is_jmp = DISAS_UPDATE;
            return;

        case 1:
            gen_helper_sim_stop(tcg_env);
            dc->is_jmp = DISAS_UPDATE;
            return;

        default:
            break;
        }
    }

    /* check for magic simulator MOV */
    if (sim_magic &&
        (dc->tbflags & META_TBFLAG_SIMACTIVE) &&
        !utu->swap && !utu->kick && !utu->defr &&
        (inst->dst.u == META_UNIT_D0) &&
        (inst->dst.i == 0) &&
        (inst->src1.u == META_UNIT_TR) &&
        (inst->src1.i == META_TXSTAT)) {
        gen_helper_sim_dump(tcg_env, cpu_dregs[1][0]);
        dc->is_jmp = DISAS_UPDATE;
        return;
    }
#endif

    /* skip MOV to TT unit when tracing is disabled */
    if (!utu->swap && !utu->kick && !utu->defr &&
        (inst->dst.u == META_UNIT_TT) &&
        !(dc->tbflags & META_TBFLAG_TREN)) {
        return;
    }

    /* branch if condition fails, pre-source for DEFR */
    if (inst->cc && utu->defr) {
        meta_start_cc(dc, inst->cc, &lbl_cc);
    }

    /* register access is different for kicks & defr */
    dc->reg_kick = utu->kick;
    dc->reg_defr = utu->defr;

    /* DEFR is allowed to read from TXSTAT and TXPOLL */
    dc->reg_priv_exception = utu->defr;

    /* get source */
    src[0] = meta_get_source(dc, inst->src1, &src_tmp[0]);
    if (utu->width == WIDTH_64B) {
        src[1] = meta_get_source(dc, inst->src2, &src_tmp[1]);
    } else {
        src[1] = NULL;
    }

    /* reset */
    dc->reg_kick = 0;
    dc->reg_defr = 0;
    dc->reg_priv_exception = 0;

    /* branch if condition fails, post-source for non-DEFR */
    if (inst->cc && !utu->defr) {
        meta_start_cc(dc, inst->cc, &lbl_cc);
    }

    if (!utu->swap && (utu->width == WIDTH_64B)) {
        MetaRegHandler *handlers = meta_reg_handlers[inst->dst.u].handlers;
        if (handlers && (handlers[inst->dst.i].gen_writel ||
                    handlers[inst->dst.i].writel)) {
            /* special case 64-bit register writes to do them whole */
            meta_gen_reg_writel(dc,
                                inst->dst.u, inst->dst.i,
                                inst->src1.u, inst->src1.i,
                                src[0], src[1]);
        } else {
            meta_set_dest(dc, inst->dst, src[0]);
            meta_set_dest(dc, meta_hi_dest(inst->dst), src[1]);
        }
    } else {
        if (utu->swap) {
            static const int effective_units[16] = {
                [META_UNIT_CT] = META_UNIT_CT,
                [META_UNIT_D0] = META_UNIT_D0,
                [META_UNIT_D1] = META_UNIT_D1,
                [META_UNIT_A0] = META_UNIT_A0,
                [META_UNIT_A1] = META_UNIT_A1,
                [META_UNIT_PC] = META_UNIT_CT,
                [META_UNIT_RA] = META_UNIT_RA,
                [META_UNIT_TR] = META_UNIT_CT,
            };
            MetaInstructionSource src2;
            MetaInstructionDest dst2;
            TCGv tmp1;

            /* check validity */
            assert(utu->width == WIDTH_32B);
            if (unlikely(effective_units[inst->src1.u] ==
                         effective_units[inst->dst.u])) {
                meta_illegal(dc, "SWAP within effective unit");
                return;
            }

            if ((inst->dst.u == META_UNIT_PC) &&
                (inst->dst.i == META_PC) &&
                !src_tmp[0]) {
                /* optimise common case */
                meta_set_dest(dc, inst->dst, src[0]);
                tcg_gen_movi_i32(src[0], dc->next_pc);
            } else {
                /* get second source */
                src2.type = SRC_REG;
                src2.u = inst->dst.u;
                src2.i = inst->dst.i;
                src[1] = meta_get_source(dc, src2, &src_tmp[1]);

                /* get second destination */
                dst2.type = DST_REG;
                dst2.u = inst->src1.u;
                dst2.i = inst->src1.i;

                /* swap the values */
                tmp1 = tcg_temp_new();
                tcg_gen_mov_i32(tmp1, src[0]);
                meta_set_dest(dc, dst2, src[1]);
                meta_set_dest(dc, inst->dst, tmp1);
            }
        } else {
            if (utu->tt) {
                /* TODO */
            }

            if (inst->cc &&
                (inst->dst.u == META_UNIT_PC) &&
                (inst->dst.i == META_PC)) {
                /* if executed we'll exit the tb, decrement loops first */
                gen_hwloop_dec(dc);
            }

            /* write the result, if necessary */
            meta_set_dest(dc, inst->dst, src[0]);
        }
    }

    /* cleanup */

    /* place the label for failed conditions */
    if (inst->cc) {
        meta_end_cc(dc, &lbl_cc);
    }
}

static void meta_gen_rti(DisasContext *dc, const MetaInstruction *inst)
{
    if (dc->tbflags & META_TBFLAG_ISTAT) {
        TCGv temp;
        TCGLabel *l1;

        /* Zero ISTAT bit */
        tcg_gen_andi_i32(cpu_cregs[META_TXSTATUS], cpu_cregs[META_TXSTATUS],
                         ~META_TXSTATUS_ISTAT_MASK);
        dc->tbflags &= ~META_TBFLAG_ISTAT;

        /* toggle PSTAT bit if TXPRIVEXT.PToggle */
        temp = tcg_temp_new_i32();
        tcg_gen_andi_i32(temp, cpu_cregs[META_TXPRIVEXT],
                         META_TXPRIVEXT_PTOGGLE_MASK);
        /* META_TXSTATUS_PSTAT_SHIFT > META_TXPRIVEXT_PTOGGLE */
        tcg_gen_shli_i32(temp, temp,
                         META_TXSTATUS_PSTAT_SHIFT - META_TXPRIVEXT_PTOGGLE);
        tcg_gen_xor_i32(cpu_cregs[META_TXSTATUS], cpu_cregs[META_TXSTATUS],
                        temp);

        /* Clear RPDirty bit */
        tcg_gen_andi_i32(cpu_cregs[META_TXDIVTIME], cpu_cregs[META_TXDIVTIME],
                         ~META_TXDIVTIME_RPDIRTY_MASK);

        /* Swap PC and PCX */
        tcg_gen_mov_i32(cpu_pc[META_PC], cpu_pc[META_PCX]);
        tcg_gen_movi_i32(cpu_pc[META_PCX], dc->next_pc);

        /* replay catch state if CBMARKER is set */
        l1 = gen_new_label();
        tcg_gen_andi_i32(temp, cpu_cregs[META_TXSTATUS],
                         META_TXSTATUS_CBMARKER_MASK);
        tcg_gen_brcondi_i32(TCG_COND_EQ, temp, 0, l1);
        {
            gen_helper_replay_catch(tcg_env);
        }
        gen_set_label(l1);

        /* Computed jump, (and flags updated) */
        dc->is_jmp = DISAS_JUMP;
        meta_gen_check_triggers(dc, 1);

    } else {
        /* RTI when not in interrupt mode generates unknown instruction halt */
        meta_gen_halt(dc, META_HREASON_UNKNOWN);
        dc->jmp = JMP_INDIRECT;
    }
}

static void meta_gen_rth(DisasContext *dc, const MetaInstruction *inst)
{
    /* FIXME priv check? */

    /* Toggle ISTAT bit */
    tcg_gen_xori_i32(cpu_cregs[META_TXSTATUS], cpu_cregs[META_TXSTATUS],
                     META_TXSTATUS_ISTAT_MASK);
    dc->tbflags ^= META_TBFLAG_ISTAT;

    meta_gen_check_triggers(dc, 1);
}

static void meta_gen_lock(DisasContext *dc, const MetaInstruction *inst)
{
#if !defined(CONFIG_USER_ONLY)
    const MetaInstructionLock *lock = &inst->lock;
    const int thread = dc->env->thread_num;
    TCGv glock = cpu_glock[thread];
    TCGv temp;
#endif

    meta_priv_check(dc, cpu_cregs[META_TXPRIVEXT],
                    META_TXPRIVEXT_ILOCK_MASK, "LOCK");

#if !defined(CONFIG_USER_ONLY)
    if (lock->n) {
        /* LOCK1 or LOCK2: check if another thread has LOCK1 */
        TCGLabel *l1 = gen_new_label();
        temp = tcg_temp_new_i32();
        tcg_gen_andi_i32(temp, glock, META_LOCK1_OTH(thread));
        tcg_gen_brcondi_i32(TCG_COND_EQ, temp, 0, l1);
        {
            /*
             * Another thread has LOCK1. Yield and wait for it.
             * This thread won't get scheduled until it's given the lock.
             */
            tcg_gen_ori_i32(glock, glock, META_LOCKW_THR(thread));
            meta_gen_restore_state(dc);
            t_gen_raise_exception(EXCP_HALTED);
            /*
             * Continue decoding, on the assumption that most of the time
             * there'll be no lock contention.
             */
        }
        gen_set_label(l1);
        if (lock->n == 0x2) {
            /* LOCK2: acquire both LOCK1 and LOCK2 */
            tcg_gen_ori_i32(glock, glock, META_LOCK_THR(thread));
        } else {
            /* LOCK1: acquire just LOCK1 and release LOCK2 */
            tcg_gen_ori_i32(glock, glock, META_LOCK1_THR(thread));
            tcg_gen_andi_i32(glock, glock, ~META_LOCK2_THR(thread));
        }
    } else {
        /* LOCK0: release all locks (LOCK2, LOCK1, and LOCKW) */
        tcg_gen_andi_i32(glock, glock, ~META_LOCKA_THR(thread));
        /* if another thread is waiting, give it the lock and unblock it. */
        temp = meta_const_i32(META_LOCKW_OTH(thread));
        gen_helper_unlock(glock, tcg_env, glock, temp);
    }
#endif
}

static void meta_gen_jump(DisasContext *dc, const MetaInstruction *inst)
{
    const MetaInstructionJump *jump = &inst->jump;
    TCGv src, npc = NULL;
    bool src_tmp = false;

    dc->hwloop_handled = true;

    /* get source */
    src = meta_get_source(dc, inst->src1, &src_tmp);

    /* must be a direct register for update */
    assert(!src_tmp);

    if (!jump->rel) {
        npc = tcg_temp_new();
        tcg_gen_addi_i32(npc, src, jump->offset);
    }

    /* if calling, put return address in src */
    if (jump->call) {
        if (jump->rel) {
            /* CALLR doesn't honor hardware loops */
            tcg_gen_movi_i32(src, dc->next_pc_nohwloop);
        } else {
            tcg_gen_movi_i32(src, dc->next_pc);
        }
    }

    if (jump->rel) {
        /* jump to pc + imm */
        meta_gen_reg_write_imm(dc, META_UNIT_PC, META_PC,
                               dc->pc + jump->offset);
    } else {
        /* jump to src address */
        meta_gen_reg_write(dc, META_UNIT_PC, META_PC, npc);
    }

    /* decrement the hardware loops */
    gen_hwloop_dec(dc);
}

static void meta_gen_switch(DisasContext *dc, const MetaInstruction *inst)
{
    /* record switch code for external handler */
    if (dc->tbflags & META_TBFLAG_MINIM) {
        /* recreate the MiniM core encoding */
        tcg_gen_movi_i32(cpu_switch_code, 0x9f00 |
                (((inst->raw >> 22) & 0x3) << 6) |
                (((inst->raw >> 16) & 0x3) << 4) |
                (((inst->raw >> 9) & 0x1) << 3) |
                (((inst->raw >> 1) & 0x7) << 0));
    } else {
        tcg_gen_movi_i32(cpu_switch_code, inst->raw);
    }

    tcg_gen_movi_i32(cpu_switch_nextpc, dc->next_pc);

    meta_gen_halt(dc, META_HREASON_SWITCH);
    dc->jmp = JMP_INDIRECT;
}

static void meta_gen_cache(DisasContext *dc, const MetaInstruction *inst)
{
    const MetaInstructionCache *cache = &inst->cache;
    TCGv addr, base;
    bool base_tmp = false;

    if (cache->op == ICACHE) {
        /* instruction prefetch is a no-op */
        return;
    }

    /* calculate address */
    base = meta_get_source(dc, cache->base, &base_tmp);
    addr = tcg_temp_new();
    meta_gen_addr_inci(dc, addr, base, cache->off.i, cache->base.u);

    if (cache->op == CACHE_R) {
        TCGv_i64 res;
        TCGv dst[2];
        bool dst_tmp[2] = { 0 };

        /* get destination */
        dst[0] = meta_get_dest(dc, inst->dst, &dst_tmp[0]);
        if (cache->width == WIDTH_64B) {
            dst[1] = meta_get_dest(dc, meta_hi_dest(inst->dst), &dst_tmp[1]);
        } else {
            dst[1] = NULL;
        }

        /* do the hard work in a helper */
        res = tcg_temp_new_i64();
        gen_helper_cacherl(res, tcg_env, addr);
        if (cache->width == WIDTH_64B) {
            tcg_gen_extr_i64_i32(dst[0], dst[1], res);
        } else {
            tcg_gen_extrl_i64_i32(dst[0], res);
        }

        /* write the result, if necessary */
        meta_set_dest(dc, inst->dst, dst[0]);
        if (cache->width == WIDTH_64B) {
            meta_set_dest(dc, meta_hi_dest(inst->dst), dst[1]);
        }

        /* cleanup */
    } else {
#if !defined(CONFIG_USER_ONLY)
        TCGv actcyc, src[2];
        bool src_tmp[2] = { 0 };

        /* get source data */
        src[0] = meta_get_source(dc, inst->src1, &src_tmp[0]);
        if (cache->width == WIDTH_64B) {
            src[1] = meta_get_source(dc, meta_hi_source(inst->src1),
                                     &src_tmp[1]);
        } else {
            src[1] = NULL;
        }

        /* do the hard work in a helper */
        actcyc = meta_gen_actcyc_get(dc);
        gen_helper_cachew(tcg_env, addr, src[0], actcyc);

        /*
         * The op may be an icache flush. The TBs should already be
         * invalidated, but end the TB in case a modification came from
         * this TB. We can still chain blocks, since it will be unchained
         * when the TB is invalidated.
         */
        dc->jmp = JMP_INDIRECT;

        /* cleanup */
#endif
    }

    /* cleanup */
}

static void meta_get_fpu_regs(int idx, bool pair,
                              TCGv *low, TCGv *high)
{
    if (pair) {
        *low = cpu_fxregs[idx & ~0x1];
        *high = cpu_fxregs[idx | 0x1];
    } else {
        *low = cpu_fxregs[idx];
        *high = NULL;
    }
}

static void meta_gen_fpu_mov_imm(DisasContext *dc, const MetaInstruction *inst)
{
    float_status fs;
    float16 val16 = make_float16(inst->src1.i);
    float32 val32 = float16_to_float32(val16, 1, &fs);
    TCGv dst[2];

    /* get destination */
    meta_get_fpu_regs(inst->dst.i, inst->fpu.dbl || inst->fpu.paired,
                      &dst[0], &dst[1]);

    if (inst->fpu.dbl) {
        /* float16 to float64 */
        float64 val64 = float32_to_float64(val32, &fs);
        uint64_t uival = float64_val(val64);

        tcg_gen_movi_i32(dst[0], uival);
        tcg_gen_movi_i32(dst[1], uival >> 32);
    } else {
        /* float16 to float32 */
        uint32_t uival = float32_val(val32);

        tcg_gen_movi_i32(dst[0], uival);
        if (inst->fpu.paired) {
            tcg_gen_movi_i32(dst[1], uival);
        }
    }
}

static void meta_gen_fpu_absmovnegswap(DisasContext *dc,
                                       const MetaInstruction *inst)
{
    DEFINE_SAFE_LABEL(lbl_cc);
    TCGv src[2], dst[2];
    int i;

    /* branch if condition fails */
    if (inst->cc) {
        meta_start_cc(dc, inst->cc, &lbl_cc);
    }

    /* get destination */
    meta_get_fpu_regs(inst->dst.i, inst->fpu.dbl || inst->fpu.paired,
                      &dst[0], &dst[1]);

    /* get source */
    meta_get_fpu_regs(inst->src1.i, inst->fpu.dbl || inst->fpu.paired,
                      &src[0], &src[1]);

    /* perform op */
    if (inst->op == OP_FPU_SWAP) {
        if (inst->dst.i == inst->src1.i) {
            /* careful not to clobber source */
            TCGv tmp1 = tcg_temp_new();
            tcg_gen_mov_i32(tmp1, src[1]);
            tcg_gen_mov_i32(dst[1], src[0]);
            tcg_gen_mov_i32(dst[0], tmp1);
        } else {
            tcg_gen_mov_i32(dst[0], src[1]);
            tcg_gen_mov_i32(dst[1], src[0]);
        }
    } else if (inst->fpu.dbl) {
        /* low */
        tcg_gen_mov_i32(dst[0], src[0]);

        /* high */
        switch (inst->op) {
        case OP_FPU_MOV_REG:
            tcg_gen_mov_i32(dst[1], src[1]);
            break;

        case OP_FPU_ABS:
            tcg_gen_andi_i32(dst[1], src[1], 0x7fffffff);
            break;

        case OP_FPU_NEG:
            tcg_gen_xori_i32(dst[1], src[1], 0x80000000);
            break;

        default:
            assert(0);
        }
    } else {
        /* for each register */
        for (i = 0; i <= inst->fpu.paired; i++) {
            switch (inst->op) {
            case OP_FPU_MOV_REG:
                tcg_gen_mov_i32(dst[i], src[i]);
                break;

            case OP_FPU_ABS:
                tcg_gen_andi_i32(dst[i], src[i], 0x7fffffff);
                break;

            case OP_FPU_NEG:
                tcg_gen_xori_i32(dst[i], src[i], 0x80000000);
                break;

            default:
                assert(0);
            }
        }
    }

    /* place the label for failed conditions */
    if (inst->cc) {
        meta_end_cc(dc, &lbl_cc);
    }
}

static void meta_gen_fpu_pack(DisasContext *dc, const MetaInstruction *inst)
{
    TCGv src[2], dst[2];

    /* get destination */
    meta_get_fpu_regs(inst->dst.i, inst->fpu.dbl || inst->fpu.paired,
                      &dst[0], &dst[1]);

    /* get sources */
    src[0] = cpu_fxregs[inst->src1.i];
    src[1] = cpu_fxregs[inst->src2.i];

    /* perform pack */
    if ((dst[0] == src[0])) {
        /* careful not to clobber source */
        TCGv tmp1 = tcg_temp_new();
        tcg_gen_mov_i32(tmp1, src[0]);
        tcg_gen_mov_i32(dst[0], src[1]);
        tcg_gen_mov_i32(dst[1], tmp1);
    } else {
        tcg_gen_mov_i32(dst[0], src[1]);
        tcg_gen_mov_i32(dst[1], src[0]);
    }
}

static void meta_gen_fpu_arith(DisasContext *dc, const MetaInstruction *inst)
{
    DEFINE_SAFE_LABEL(lbl_cc);
    TCGv src1[2], src2[2], dst[2];

    /* get destination */
    meta_get_fpu_regs(inst->dst.i, inst->fpu.dbl || inst->fpu.paired,
                      &dst[0], &dst[1]);

    /* get sources */
    if (inst->fpu.arith.reduction) {
        meta_get_fpu_regs(inst->src1.i, true,
                          &src1[0], &src2[0]);
        meta_get_fpu_regs(inst->src2.i, true,
                          &src1[1], &src2[1]);
    } else {
        meta_get_fpu_regs(inst->src1.i, inst->fpu.dbl || inst->fpu.paired,
                          &src1[0], &src1[1]);
        meta_get_fpu_regs(inst->src2.i, inst->fpu.dbl || inst->fpu.paired,
                          &src2[0], &src2[1]);
    }

    /* branch if condition fails */
    if (inst->cc) {
        meta_start_cc(dc, inst->cc, &lbl_cc);
    }

    if (inst->fpu.dbl) {
        TCGv_i64 tmp1, tmp2;

        tmp1 = tcg_temp_new_i64();
        tmp2 = tcg_temp_new_i64();
        tcg_gen_concat_i32_i64(tmp1, src1[0], src1[1]);
        tcg_gen_concat_i32_i64(tmp2, src2[0], src2[1]);

        switch (inst->op) {
        case OP_FPU_ADD:
            gen_helper_fx_add64(tmp1, tcg_env, tmp1, tmp2);
            break;
        case OP_FPU_MUL:
            gen_helper_fx_mul64(tmp1, tcg_env, tmp1, tmp2);
            break;
        case OP_FPU_SUB:
            gen_helper_fx_sub64(tmp1, tcg_env, tmp1, tmp2);
            break;
        default:
            assert(0);
        }

        if (inst->fpu.arith.invert) {
            tcg_gen_xori_i64(tmp1, tmp1, 0x8000000000000000llu);
        }

        tcg_gen_extr_i64_i32(dst[0], dst[1], tmp1);
    } else {
        int i;

        for (i = 0; i <= inst->fpu.paired; i++) {
            switch (inst->op) {
            case OP_FPU_ADD:
                gen_helper_fx_add32(dst[i], tcg_env, src1[i], src2[i]);
                break;
            case OP_FPU_MUL:
                gen_helper_fx_mul32(dst[i], tcg_env, src1[i], src2[i]);
                break;
            case OP_FPU_SUB:
                gen_helper_fx_sub32(dst[i], tcg_env, src1[i], src2[i]);
                break;
            default:
                assert(0);
            }

            if (inst->fpu.arith.invert) {
                tcg_gen_xori_i32(dst[i], dst[i], 0x80000000);
            }
        }
    }

    meta_gen_fx_handle_exception(dc, &inst->fpu.info);

    /* place the label for failed conditions */
    if (inst->cc) {
        meta_end_cc(dc, &lbl_cc);
    }
}

static void meta_gen_fpu_cmp(DisasContext *dc, const MetaInstruction *inst)
{
    DEFINE_SAFE_LABEL(lbl_cc);
    TCGv src1[2], src2[2], rflags;

    /* get sources */
    meta_get_fpu_regs(inst->src1.i, inst->fpu.dbl || inst->fpu.paired,
                      &src1[0], &src1[1]);
    if (inst->src2.type != SRC_IMM) {
        meta_get_fpu_regs(inst->src2.i, inst->fpu.dbl || inst->fpu.paired,
                          &src2[0], &src2[1]);
    }

    /* branch if condition fails */
    if (inst->cc) {
        meta_start_cc(dc, inst->cc, &lbl_cc);
    }

    if (inst->fpu.dbl) {
        TCGv_i64 tmp1, tmp2;

        tmp1 = tcg_temp_new_i64();
        tcg_gen_concat_i32_i64(tmp1, src1[0], src1[1]);

        if (inst->src2.type == SRC_IMM) {
            tmp2 = meta_const_i64(0);
        } else {
            tmp2 = tcg_temp_new_i64();
            tcg_gen_concat_i32_i64(tmp2, src2[0], src2[1]);
        }

        rflags = meta_const_i32(inst->fpu.info.op.flags);
        gen_helper_fx_cmp64(tmp1, tcg_env, tmp1, tmp2, rflags);

    } else {
        int i;

        for (i = 0; i <= inst->fpu.paired; i++) {
            if (inst->src2.type == SRC_IMM) {
                src2[i] = meta_const_i32(0);
            }

            rflags = meta_const_i32(inst->fpu.info.op.flags |
                                   (i ? META_FXINST_HIGH : 0));
            gen_helper_fx_cmp32(rflags, tcg_env, src1[i], src2[i], rflags);

        }
    }

    meta_gen_fx_handle_exception(dc, &inst->fpu.info);

    /* place the label for failed conditions */
    if (inst->cc) {
        meta_end_cc(dc, &lbl_cc);
    }
}

static void meta_gen_fpu_minmax(DisasContext *dc, const MetaInstruction *inst)
{
    DEFINE_SAFE_LABEL(lbl_cc);
    TCGv dst[2], src1[2], src2[2], rflags;

    /* get destination */
    meta_get_fpu_regs(inst->dst.i, inst->fpu.dbl || inst->fpu.paired,
                      &dst[0], &dst[1]);

    /* get sources */
    meta_get_fpu_regs(inst->src1.i, inst->fpu.dbl || inst->fpu.paired,
                      &src1[0], &src1[1]);
    meta_get_fpu_regs(inst->src2.i, inst->fpu.dbl || inst->fpu.paired,
                      &src2[0], &src2[1]);

    /* branch if condition fails */
    if (inst->cc) {
        meta_start_cc(dc, inst->cc, &lbl_cc);
    }

    if (inst->fpu.dbl) {
        TCGv_i64 tmp1, tmp2;

        tmp1 = tcg_temp_new_i64();
        tcg_gen_concat_i32_i64(tmp1, src1[0], src1[1]);
        tmp2 = tcg_temp_new_i64();
        tcg_gen_concat_i32_i64(tmp2, src2[0], src2[1]);

        rflags = meta_const_i32(inst->fpu.info.op.flags);
        gen_helper_fx_cmp64(tmp1, tcg_env, tmp1, tmp2, rflags);

        tcg_gen_extr_i64_i32(dst[0], dst[1], tmp1);
    } else {
        int i;

        for (i = 0; i <= inst->fpu.paired; i++) {
            rflags = meta_const_i32(inst->fpu.info.op.flags |
                                   (i ? META_FXINST_HIGH : 0));
            gen_helper_fx_cmp32(dst[i], tcg_env, src1[i], src2[i], rflags);
        }
    }

    meta_gen_fx_handle_exception(dc, &inst->fpu.info);

    /* place the label for failed conditions */
    if (inst->cc) {
        meta_end_cc(dc, &lbl_cc);
    }
}

static void meta_gen_fpu_reciprocal(DisasContext *dc,
                                    const MetaInstruction *inst)
{
    const MetaInstructionFpuRcp *rcp = &inst->fpu.rcp;
    TCGv dst[2], src[2], rflags;

    /* get destination */
    meta_get_fpu_regs(inst->dst.i, inst->fpu.dbl || inst->fpu.paired,
                      &dst[0], &dst[1]);

    /* get source */
    meta_get_fpu_regs(inst->src1.i, inst->fpu.dbl || inst->fpu.paired,
                      &src[0], &src[1]);

    if (inst->fpu.dbl) {
        TCGv_i64 tmp1 = tcg_temp_new_i64();
        tcg_gen_concat_i32_i64(tmp1, src[0], src[1]);
        rflags = meta_const_i32(inst->fpu.info.op.flags);

        if (inst->op == OP_FPU_RSQ) {
            gen_helper_fx_rsq64(tmp1, tcg_env, tmp1, rflags);
        } else {
            gen_helper_fx_rcp64(tmp1, tcg_env, tmp1, rflags);
        }


        if (rcp->invert) {
            tcg_gen_xori_i64(tmp1, tmp1, 0x8000000000000000llu);
        }

        tcg_gen_extr_i64_i32(dst[0], dst[1], tmp1);
    } else {
        int i;

        for (i = 0; i <= inst->fpu.paired; i++) {
            rflags = meta_const_i32(inst->fpu.info.op.flags);

            if (inst->op == OP_FPU_RSQ) {
                gen_helper_fx_rsq32(dst[i], tcg_env, src[i], rflags);
            } else {
                gen_helper_fx_rcp32(dst[i], tcg_env, src[i], rflags);
            }

            if (rcp->invert) {
                tcg_gen_xori_i32(dst[i], dst[i], 0x80000000);
            }

        }
    }

    meta_gen_fx_handle_exception(dc, &inst->fpu.info);
}

static void meta_gen_fpu_muz(DisasContext *dc, const MetaInstruction *inst)
{
    const MetaInstructionFpuMuz *muz = &inst->fpu.muz;
    TCGv dst[2], src1[2], src2[2], rflags;

    /* get destination */
    meta_get_fpu_regs(inst->dst.i, inst->fpu.dbl || inst->fpu.paired,
                      &dst[0], &dst[1]);

    /* get sources */
    meta_get_fpu_regs(inst->src1.i, inst->fpu.dbl || inst->fpu.paired,
                      &src1[0], &src1[1]);
    meta_get_fpu_regs(inst->src2.i, inst->fpu.dbl || inst->fpu.paired,
                      &src2[0], &src2[1]);

    if (inst->fpu.dbl) {
        TCGv_i64 tmp1, tmp2, tmp3;

        tmp1 = tcg_temp_new_i64();
        tcg_gen_concat_i32_i64(tmp1, src1[0], src1[1]);
        tmp2 = tcg_temp_new_i64();
        tcg_gen_concat_i32_i64(tmp2, src2[0], src2[1]);

        if (muz->o3o) {
            tmp3 = meta_const_i64(float64_val(1.0));
        } else {
            tmp3 = tcg_temp_new_i64();
            tcg_gen_concat_i32_i64(tmp3, dst[0], dst[1]);
        }

        rflags = meta_const_i32(inst->fpu.info.op.flags);
        gen_helper_fx_muz64(tmp3, tcg_env, tmp1, tmp2, tmp3, rflags);

        if (muz->invert) {
            tcg_gen_xori_i64(tmp3, tmp3, 0x8000000000000000llu);
        }

        tcg_gen_extr_i64_i32(dst[0], dst[1], tmp3);
    } else {
        int i;

        for (i = 0; i <= inst->fpu.paired; i++) {
            TCGv s3 = dst[i];

            if (muz->o3o) {
                s3 = meta_const_i32(float32_val(1.0f));
            }

            rflags = meta_const_i32(inst->fpu.info.op.flags);
            gen_helper_fx_muz32(dst[i], tcg_env, src1[i], src2[i], s3, rflags);

            if (muz->invert) {
                tcg_gen_xori_i32(dst[i], dst[i], 0x80000000llu);
            }

        }
    }

    meta_gen_fx_handle_exception(dc, &inst->fpu.info);
}

static void meta_gen_fpu_convert(DisasContext *dc, const MetaInstruction *inst)
{
    const MetaInstructionFpuConvert *conv = &inst->fpu.convert;
    TCGv dst[2], src[2];
    TCGv_i64 tmp1 = NULL;
    DEFINE_SAFE_LABEL(lbl_cc);

    /* branch if condition fails */
    if (inst->cc) {
        meta_start_cc(dc, inst->cc, &lbl_cc);
    }

    /* get destination */
    meta_get_fpu_regs(inst->dst.i, conv->dst_type & FPU_64B,
                      &dst[0], &dst[1]);

    /* get source */
    meta_get_fpu_regs(inst->src1.i, conv->src_type & FPU_64B,
                      &src[0], &src[1]);

    /* prepare for 64-bit data */
    if (conv->src_type & FPU_64B) {
        tmp1 = tcg_temp_new_i64();
        tcg_gen_concat_i32_i64(tmp1, src[0], src[1]);
    } else if (conv->dst_type & FPU_64B) {
        tmp1 = tcg_temp_new_i64();
    }

#define CONV(f, t) (((FPU_ ## f) << 4) | (FPU_ ## t))

    /* perform the conversion */
    switch ((conv->src_type << 4) | conv->dst_type) {
    case CONV(D, F):
        gen_helper_fx_dtof(dst[0], tcg_env, tmp1);
        break;

    case CONV(F, D):
        gen_helper_fx_ftod(tmp1, tcg_env, src[0]);
        break;

    case CONV(D, H):
        gen_helper_fx_dtoh(dst[0], tcg_env, tmp1);
        break;

    case CONV(F, H):
        gen_helper_fx_ftoh(dst[0], tcg_env, src[0]);
        break;

    case CONV(D, I): {
            TCGv rz = meta_const_i32(conv->z_rounding);
            gen_helper_fx_dtoi(dst[0], tcg_env, tmp1, rz);
            break;
        }

    case CONV(F, I): {
            TCGv rz = meta_const_i32(conv->z_rounding);
            gen_helper_fx_ftoi(dst[0], tcg_env, src[0], rz);
            break;
        }

    case CONV(D, L): {
            TCGv rz = meta_const_i32(conv->z_rounding);
            gen_helper_fx_dtol(tmp1, tcg_env, tmp1, rz);
            break;
        }

    case CONV(D, X): {
            TCGv rbits = meta_const_i32(inst->src2.i);
            gen_helper_fx_dtox(dst[0], tcg_env, tmp1, rbits);
            break;
        }

    case CONV(F, X): {
            TCGv rbits = meta_const_i32(inst->src2.i);
            gen_helper_fx_ftox(dst[0], tcg_env, src[0], rbits);
            break;
        }

    case CONV(D, XL): {
            TCGv rbits = meta_const_i32(inst->src2.i);
            gen_helper_fx_dtoxl(tmp1, tcg_env, tmp1, rbits);
            break;
        }

    case CONV(H, D):
        gen_helper_fx_htod(tmp1, tcg_env, src[0]);
        break;

    case CONV(H, F):
        gen_helper_fx_htof(dst[0], tcg_env, src[0]);
        break;

    case CONV(I, D):
        gen_helper_fx_itod(tmp1, tcg_env, src[0]);
        break;

    case CONV(I, F):
        gen_helper_fx_itof(dst[0], tcg_env, src[0]);
        break;

    case CONV(L, D):
        gen_helper_fx_ltod(tmp1, tcg_env, tmp1);
        break;

    case CONV(X, D): {
            TCGv rbits = meta_const_i32(inst->src2.i);
            gen_helper_fx_xtod(tmp1, tcg_env, src[0], rbits);
            break;
        }

    case CONV(X, F): {
            TCGv rbits = meta_const_i32(inst->src2.i);
            gen_helper_fx_xtof(dst[0], tcg_env, src[0], rbits);
            break;
        }

    case CONV(XL, D): {
            TCGv rbits = meta_const_i32(inst->src2.i);
            gen_helper_fx_xltod(tmp1, tcg_env, tmp1, rbits);
            break;
        }

    default:
        meta_unimplemented(dc, "unknown conversion");
    }

#undef CONV

    if (conv->dst_type & FPU_64B) {
        tcg_gen_extr_i64_i32(dst[0], dst[1], tmp1);
    }

    meta_gen_fx_handle_exception(dc, &inst->fpu.info);

    /* place the label for failed conditions */
    if (inst->cc) {
        meta_end_cc(dc, &lbl_cc);
    }
}

typedef void (meta_gen_fn)(DisasContext *dc, const MetaInstruction *instr);

static meta_gen_fn *meta_gen_functions[OP_COUNT] = {
    [0 ... OP_COUNT-1] = NULL,

    [OP_ANDORXOR]      = meta_gen_andorxor,
    [OP_AUADDSUB]      = meta_gen_auaddsub,
    [OP_BEX]           = meta_gen_bex,
    [OP_BRANCH]        = meta_gen_branch,
    [OP_CACHE]         = meta_gen_cache,
    [OP_CMPTST]        = meta_gen_cmptst,
    [OP_DSPREG_COPY]   = meta_gen_dspreg_copy,
    [OP_DSPREG_GET]    = meta_gen_dspreg_get,
    [OP_DSPREG_INIT]   = meta_gen_dspreg_init,
    [OP_DSPREG_SET]    = meta_gen_dspreg_set,
    [OP_DUADDSUB]      = meta_gen_duaddsub,
    [OP_FPU_ABS]       = meta_gen_fpu_absmovnegswap,
    [OP_FPU_ADD]       = meta_gen_fpu_arith,
    [OP_FPU_CMP]       = meta_gen_fpu_cmp,
    [OP_FPU_CONVERT]   = meta_gen_fpu_convert,
    [OP_FPU_MAX]       = meta_gen_fpu_minmax,
    [OP_FPU_MIN]       = meta_gen_fpu_minmax,
    [OP_FPU_MOV_IMM]   = meta_gen_fpu_mov_imm,
    [OP_FPU_MOV_REG]   = meta_gen_fpu_absmovnegswap,
    [OP_FPU_MUL]       = meta_gen_fpu_arith,
    [OP_FPU_MUZ]       = meta_gen_fpu_muz,
    [OP_FPU_NEG]       = meta_gen_fpu_absmovnegswap,
    [OP_FPU_PACK]      = meta_gen_fpu_pack,
    [OP_FPU_RCP]       = meta_gen_fpu_reciprocal,
    [OP_FPU_RSQ]       = meta_gen_fpu_reciprocal,
    [OP_FPU_SUB]       = meta_gen_fpu_arith,
    [OP_FPU_SWAP]      = meta_gen_fpu_absmovnegswap,
    [OP_GET]           = meta_gen_getset,
    [OP_JUMP]          = meta_gen_jump,
    [OP_LOCK]          = meta_gen_lock,
    [OP_MMOV]          = meta_gen_multimov,
    [OP_MUL]           = meta_gen_mul,
    [OP_PORTTOUNIT]    = meta_gen_porttounit,
    [OP_RTD]           = meta_gen_rtd,
    [OP_RTH]           = meta_gen_rth,
    [OP_RTI]           = meta_gen_rti,
    [OP_SET]           = meta_gen_getset,
    [OP_SHIFT]         = meta_gen_shift,
    [OP_SWITCH]        = meta_gen_switch,
    [OP_UNITTOUNIT]    = meta_gen_unittounit,
    [OP_XFR]           = meta_gen_xfr,
    [OP_XSD]           = meta_gen_xsd,
};

static void meta_decode_instr(DisasContext *dc, uint32_t instr)
{
    MetaInstruction _inst;
    int lsmstep = (dc->tbflags & META_TBFLAG_LSM) >> META_TBFLAG_LSM_SHIFT;
    int lsmcount = 0;
    int ret;

    ret = meta_decode(&_inst, instr);
    if (ret) {
        meta_unimplemented(dc, "Instruction decode failed");
        return;
    }

    assert(_inst.op != OP_BAD);

    if (_inst.dsp && !meta_dsp_supported(dc->env)) {
        meta_illegal(dc, "DSP instruction on non-DSP thread");
        return;
    }
    if (_inst.fx && !meta_fpu_supported(dc->env)) {
        meta_illegal(dc, "FPU instruction on non-FPU thread");
        return;
    }

    /* calculate which LSMStep this is */
    if (_inst.multistep == MULTISTEP_RMASK) {
        for (dc->lsm_idx = 0;
             !(_inst.step & (1 << dc->lsm_idx)) || lsmcount < lsmstep;
             dc->lsm_idx++) {
            if (unlikely(dc->lsm_idx >= 8)) {
                /* bad LSM STEP values cause priv halts */
                meta_priv(dc, "LSMSTEP out of range");
                return;
            }
            if (_inst.step & (1 << dc->lsm_idx)) {
                lsmcount++;
            }
        }
    }

    /* log instruction if not a repeated issue (ie. LSMSTEP) */
    if (dc->pc != dc->last_pc) {
        meta_log_instruction(CPU_LOG_TB_IN_ASM, dc->pc, &_inst);
    }

    meta_gen_functions[_inst.op](dc, &_inst);

    if (_inst.multistep == MULTISTEP_RMASK) {
        int i = dc->lsm_idx + 1;
        dc->tbflags &= ~META_TBFLAG_LSM;
        lsmstep = 0;
        for (; i < 8; i++) {
            if (_inst.step & (1 << i)) {
                /* still more steps to do */
                lsmstep = lsmcount + 1;
                dc->tbflags |= lsmstep << META_TBFLAG_LSM_SHIFT;
                dc->next_pc = dc->pc;
                dc->is_jmp = DISAS_NEXT;
                tcg_gen_movi_i32(cpu_pc[META_PC], dc->pc);
                break;
            }
        }
    }
}

static void meta_decoder(DisasContext *dc)
{
#if !defined(CONFIG_USER_ONLY)
    /* FIXME if just jumped here to add instruction, still need to fetch */
    if (dc->tbflags & META_TBFLAG_IMTR) {
        /* fetch 8 bytes at a time */
        if ((dc->pc & 0x4) == 0) {
            /*
             * don't bother recording fetches in the same cache line, we know
             * they'll hit
             */
            if ((dc->pc ^ dc->last_fetch) & -64) {
                TCGv actcyc = meta_gen_actcyc_get(dc);
                TCGv pc = meta_const_tl(dc->pc);
                gen_helper_ifetch(tcg_env, pc, actcyc);
                dc->last_fetch = dc->pc;
            }
        }
    }
#endif /* !CONFIG_USER_ONLY */
    dc->instr = translator_ldl(dc->env, &dc->base, dc->pc);
    ++dc->instr_count;

    dc->next_pc = dc->next_pc_nohwloop = dc->pc + 4;
    if ((dc->tbflags & META_TBFLAG_TXL2COUNTNZ) &&
        (dc->pc == dc->env->cregs[META_TXL2END])) {
        dc->next_pc = dc->env->cregs[META_TXL2START];
        tcg_gen_mov_tl(cpu_pc[META_PC], cpu_cregs[META_TXL2START]);
    } else if ((dc->tbflags & META_TBFLAG_TXL1COUNTNZ) &&
               (dc->pc == dc->env->cregs[META_TXL1END])) {
        dc->next_pc = dc->env->cregs[META_TXL1START];
        tcg_gen_mov_tl(cpu_pc[META_PC], cpu_cregs[META_TXL1START]);
    }

    if (dc->pc != dc->last_pc) {
        LOG_DIS("%08x:\t%08x\t",
                dc->pc,
                dc->instr);
    }
    meta_decode_instr(dc, dc->instr);
}

static void minim_decoder(DisasContext *dc)
{
    uint16_t minim_insns[2];
    uint16_t *minim_ptr = minim_insns;
    size_t minim_words, i;
    target_ulong minim_addr;

    minim_addr = meta_pc_to_virt(dc->env, dc->pc);
    minim_insns[0] = translator_lduw(dc->env, &dc->base, minim_addr);
    minim_words = meta_minim_decode_size(minim_insns[0]);

    for (i = 1; i < minim_words; i++) {
        minim_insns[i] = translator_lduw(dc->env, &dc->base,
                                         minim_addr + (i * 2));
    }

    dc->instr = meta_minim_decode_insn(&minim_ptr, minim_words);

/*#define DEBUG_MINIM*/
    if (dc->pc != dc->last_pc) {
        LOG_DIS("%08x", dc->pc);
#ifdef DEBUG_MINIM
        /* debug: add the MiniM code address in brackets */
        LOG_DIS(" (%08x)", minim_addr);
#endif
        LOG_DIS(":\t");
        /* CODESCAPE style MiniM encoding (second word first with f's) */
        for (i = 0; i < 2; i++) {
            if (i < minim_words) {
                LOG_DIS("ffff%04x ", minim_insns[minim_words-i-1]);
            } else {
                LOG_DIS("         ");
            }
        }
#ifdef DEBUG_MINIM
        /* debug: add the META encoding in brackets */
        LOG_DIS(" (%08x)", dc->instr);
#endif
        LOG_DIS("\t");
    }

    ++dc->instr_count;

    dc->next_pc = dc->next_pc_nohwloop = dc->pc + (4 * minim_words);
    if ((dc->tbflags & META_TBFLAG_TXL2COUNTNZ) &&
        (dc->pc == dc->env->cregs[META_TXL2END])) {
        dc->next_pc = dc->env->cregs[META_TXL2START];
        tcg_gen_mov_tl(cpu_pc[META_PC], cpu_cregs[META_TXL2START]);
    } else if ((dc->tbflags & META_TBFLAG_TXL1COUNTNZ) &&
               (dc->pc == dc->env->cregs[META_TXL1END])) {
        dc->next_pc = dc->env->cregs[META_TXL1START];
        tcg_gen_mov_tl(cpu_pc[META_PC], cpu_cregs[META_TXL1START]);
    }

    meta_decode_instr(dc, dc->instr);
}

#if !defined(CONFIG_USER_ONLY)
void cpu_meta_core_set_timer_freq(MetaCore *core, uint32_t timer_freq)
{
    int i;
    core->timer_period = 1000000000LL / timer_freq;
    for (i = 0; i < core->num_threads; ++i) {
        meta_update_timer_freq(&core->threads[i].env);
    }
}
#endif

void cpu_meta_core_reset(MetaCore *core)
{
    int i;
    for (i = 0; i < core->num_threads; ++i) {
        cpu_reset(CPU(&core->threads[i]));
    }
#if !defined(CONFIG_USER_ONLY)
    core->global.lock = 0;
    core->global.mmu = 0;
    core->global.mmu_flags = 0;
    core->global.mmu_ptroot = 0;

    meta_reset_core_registers(core);
    meta_triggers_reset(core);

    /* fake the presence of a DA */
    if (meta_switch_bus_has_device(core->switch_bus)) {
        for (i = 0; i < core->num_threads; ++i) {
            /* bottom nibbles 0xPxIB */
            core->triggers.int_vec[i] = 0x1011;
        }
    }
#endif
}

void cpu_state_reset(CPUMETAState *env)
{
    int i;

    if (qemu_loglevel_mask(CPU_LOG_RESET)) {
        qemu_log("CPU Reset (CPU %d)\n", env_cpu(env)->cpu_index);
        log_cpu_state(env_cpu(env), 0);
    }

    /* clear trigger registers */
    for (i = 0; i < META_TR_MAX; ++i) {
        env->tregs[i] = 0;
    }

    env->cregs[META_TXENABLE] = META_TXENABLE_OFF;
#if !defined(CONFIG_USER_ONLY)
    env_cpu(env)->halted = 1;
#endif
    env->block = META_BLOCK_OFF;
    env->pc[META_PC] = 0;
    env->pc[META_PCX] = 0;
    if (!env->thread_num) {
        env->cregs[META_TXSTATUS] = 0x00020000;
    } else {
        env->cregs[META_TXSTATUS] = 0x00000000;
    }
    env->cregs[META_TXDIVTIME] = 0x00000001;
#if !defined(CONFIG_USER_ONLY)
    meta_update_timer_freq(env);
#endif
    env->ttregs[META_TTCTRL] = META_TTCTRL_ENABLE_MASK
                             | META_TTCTRL_ENABLEI_MASK;

    for (i = 0; i < 4; ++i) {
        env->mmu_tblphys[i] = 0;
    }

    if (meta_dsp_supported(env)) {
        int low_bits = env->global->dspram_addr_bits - 4;

        env->dspram_off_and[0] = (0xf << low_bits) | ((1 << low_bits) - 1);
        env->dspram_off_and[1] = env->dspram_off_and[0];
    }

    env->readport_idx_r = 0;
    env->readport_idx_w = 0;
    env->readport_count = 0;
}

static struct MetaCoreConfig
{
    /* Name of core or description */
    const char *name;
    /* Core revision */
    uint32_t core_rev;
    /* Core ID code */
    uint8_t  core_id;
    /* Core hardware configuration */
    uint16_t core_config;
    uint32_t core_config2;
    /* Default number of threads */
    uint8_t  num_threads;
    /* DSP RAM size */
    uint8_t dspram_size;
    /* Thread capabilities */
    MetaTCaps tcaps[META_MAX_THREADS];
} meta_cores[] = {
    {
        "meta122",
        META_COREREV(1, 2, 2, 0),
        0x03, 0x0000, 0x00000000,
        META_MAX_THREADS, 8,
#if !defined(CONFIG_USER_ONLY)
        { [3] = META_TCAPS_GP },
#endif
    },
    {
        "meta213",
        META_COREREV(2, 1, 3, 2),
        0x01, 0x0040, 0x00000000,
        META_MAX_THREADS, 8,
    },
    {   /* The core in Toumaz Zenif (Comet) */
        "harrier",
        META_COREREV(2, 1, 3, 2),
        0x05, 0x0040, 0x00025D86,
        2, 8,
    },
    {   /* As harrier, but with an FPU */
        "harrier+fpu",
        META_COREREV(2, 1, 3, 2),
        0x05, 0x0140, 0x00025D86,
        2, 8,
    },
    {
        "meta214",
        META_COREREV(2, 1, 4, 2),
        0x01, 0x0040, 0x00000000,
        META_MAX_THREADS, 8,
    },
    {
        "frisa2thd",
        META_COREREV(2, 1, 4, 3),
        0x06, 0x0540, 0x00049d86,
        2, 9,
    },
    {
        "frisa",
        META_COREREV(2, 1, 4, 3),
        0x06, 0x0140, 0x00049d86,
        4, 9,
    },
};


void meta_tcg_init(void)
{
    int i, j, k, t;
    static int done_init;

    if (done_init) {
        return;
    }
    /* thread2thread regs - accessed relatively between threads */
    for (t = 0; t < META_T2T_NR; ++t) {
        int t2t_offset = offsetof(MetaCore, threads[t])
                       - offsetof(MetaCore, threads[META_T2T_MID]);
        cpu_lnkaddr[t] = tcg_global_mem_new(tcg_env,
                t2t_offset + offsetof(CPUArchState, lnkaddr),
                meta_lnkaddr_names[t]);
    }
    /*
     * global regs - accessed differently per thread since not in thread's env
     */
    for (t = 0; t < META_MAX_THREADS; ++t) {
        intptr_t global_offset = offsetof(MetaCore, global) -
                                 offsetof(MetaCore, threads[t]) -
                                 offsetof(METACPU, env);
        const struct MetaUnitInfo *info;

        info = &meta_unit_info[META_UNIT_CT];
        for (i = 0; i < info->num_globals; ++i) {
            cpu_gcregs[t][i] = tcg_global_mem_new(tcg_env,
                    global_offset + offsetof(MetaGlobalState, cregs[i]),
                    meta_creg_names[info->num_locals + i]);
        }
        for (i = 0; i < 2; ++i) {
            info = &meta_unit_info[META_UNIT_D0+i];
            for (j = 0; j < info->num_globals; ++j) {
                cpu_gdregs[t][i][j] = tcg_global_mem_new(tcg_env,
                        global_offset + offsetof(MetaGlobalState, dregs[i][j]),
                        meta_dreg_names[i][info->num_locals + j]);
            }
            info = &meta_unit_info[META_UNIT_A0+i];
            for (j = 0; j < info->num_globals; ++j) {
                cpu_garegs[t][i][j] = tcg_global_mem_new(tcg_env,
                        global_offset + offsetof(MetaGlobalState, aregs[i][j]),
                        meta_areg_names[i][info->num_locals + j]);
            }
            for (j = 0; j < 3; ++j) {
                cpu_gaccregs[t][i][j] = tcg_global_mem_new_i64(tcg_env,
                        global_offset + offsetof(MetaGlobalState, accregs[i][j]),
                        meta_accreg_names[i][1+j]);
            }
        }
        info = &meta_unit_info[META_UNIT_TT];
        for (i = 0; i < info->num_globals; ++i) {
            cpu_gttregs[t][i] = tcg_global_mem_new(tcg_env,
                    global_offset + offsetof(MetaGlobalState, ttregs[i]),
                    meta_ttreg_names[info->num_locals + i]);
        }
#if !defined(CONFIG_USER_ONLY)
        cpu_glock[t] = tcg_global_mem_new(tcg_env,
                global_offset + offsetof(MetaGlobalState, lock),
                "_LOCK");
#endif
        for (i = 0; i < 4; ++i) {
            cpu_codebcount[t][i] = tcg_global_mem_new(tcg_env,
                    global_offset + offsetof(MetaGlobalState, codebcount[i]),
                    meta_codebcount_names[i]);
        }
    }
    /* local regs */
    for (i = 0; i < meta_unit_info[META_UNIT_CT].num_locals; ++i) {
        cpu_cregs[i] = tcg_global_mem_new(tcg_env,
                                          offsetof(CPUArchState, cregs[i]),
                                          meta_creg_names[i]);
    }
    for (i = 0; i < 2; ++i) {
        for (j = 0; j < meta_unit_info[META_UNIT_D0 + i].num_locals; ++j) {
            cpu_dregs[i][j] = tcg_global_mem_new(tcg_env,
                                                 offsetof(CPUArchState,
                                                          dregs[i][j]),
                                                 meta_dreg_names[i][j]);
        }
        for (j = 0; j < meta_unit_info[META_UNIT_A0 + i].num_locals; ++j) {
            cpu_aregs[i][j] = tcg_global_mem_new(tcg_env,
                                                 offsetof(CPUArchState,
                                                          aregs[i][j]),
                                                 meta_areg_names[i][j]);
        }
        for (j = 0; j < 1; ++j) {
            cpu_accregs[i][j] = tcg_global_mem_new_i64(tcg_env,
                                                   offsetof(CPUArchState,
                                                            accregs[i][j]),
                                                   meta_accreg_names[i][j]);
        }
    }
    for (i = 0; i < meta_unit_info[META_UNIT_PC].num_locals; ++i) {
        cpu_pc[i] = tcg_global_mem_new(tcg_env,
                                       offsetof(CPUArchState, pc[i]),
                                       meta_pcreg_names[i]);
    }
    for (i = 0; i < 5; ++i) {
        cpu_cf[i] = tcg_global_mem_new(tcg_env,
                                       offsetof(CPUArchState, cf[i]),
                                       meta_cf_names[i]);
    }
    for (i = 0; i < meta_unit_info[META_UNIT_TR].num_locals; ++i) {
        cpu_tregs[i] = tcg_global_mem_new(tcg_env,
                                          offsetof(CPUArchState, tregs[i]),
                                          meta_treg_names[i]);
    }
    for (i = 0; i < meta_unit_info[META_UNIT_TT].num_locals; ++i) {
        cpu_ttregs[i] = tcg_global_mem_new(tcg_env,
                                           offsetof(CPUArchState, ttregs[i]),
                                           meta_ttreg_names[i]);
    }
    for (i = 0; i < meta_unit_info[META_UNIT_FX].num_locals; ++i) {
        cpu_fxregs[i] = tcg_global_mem_new(tcg_env,
                                           offsetof(CPUArchState, fxregs[i]),
                                           meta_fxreg_names[i]);
    }
    cpu_lsmstep = tcg_global_mem_new(tcg_env,
                                     offsetof(CPUArchState, lsmstep),
                                     "LSM_STEP");
    cpu_repcyc = tcg_global_mem_new(tcg_env,
                                    offsetof(CPUArchState, repcyc),
                                    "TXTREPCYC");
    cpu_kicks = tcg_global_mem_new(tcg_env,
                                   offsetof(CPUArchState, kicks),
                                   "_TXKICKS");
    cpu_ikicks = tcg_global_mem_new(tcg_env,
                                    offsetof(CPUArchState, ikicks),
                                    "_TXIKICKS");
    cpu_switch_code = tcg_global_mem_new(tcg_env,
                                         offsetof(CPUArchState, switch_code),
                                         "_TXSWITCHCODE");
    cpu_switch_nextpc = tcg_global_mem_new(tcg_env,
                                           offsetof(CPUArchState, switch_nextpc),
                                           "_TXSWITCHNEXTPC");
    for (i = 0; i < 2; i++) {
        for (j = 0; j < 2; j++) {
            for (k = 0; k < 2; k++) {
                cpu_dspram_rp[i][j][k] = tcg_global_mem_new(tcg_env,
                                                            offsetof(CPUArchState, dspram_rp[i][j][k]),
                                                            meta_dspram_rp_names[i][j][k]);
                cpu_dspram_rpi[i][j][k] = tcg_global_mem_new(tcg_env,
                                                             offsetof(CPUArchState, dspram_rpi[i][j][k]),
                                                             meta_dspram_rpi_names[i][j][k]);
                cpu_dspram_wp[i][j][k] = tcg_global_mem_new(tcg_env,
                                                            offsetof(CPUArchState, dspram_wp[i][j][k]),
                                                            meta_dspram_wp_names[i][j][k]);
                cpu_dspram_wpi[i][j][k] = tcg_global_mem_new(tcg_env,
                                                             offsetof(CPUArchState, dspram_wpi[i][j][k]),
                                                             meta_dspram_wpi_names[i][j][k]);
                cpu_dspram_fetched[i][j][k] = tcg_global_mem_new(tcg_env,
                                                                 offsetof(CPUArchState, dspram_fetched[i][j][k]),
                                                                 meta_dspram_fetched_names[i][j][k]);
            }
        }
        cpu_dspram_off_and[i] = tcg_global_mem_new(tcg_env,
                                                   offsetof(CPUArchState, dspram_off_and[i]),
                                                   meta_dspram_off_and_names[i]);
        cpu_dspram_off_or[i] = tcg_global_mem_new(tcg_env,
                                                  offsetof(CPUArchState, dspram_off_or[i]),
                                                  meta_dspram_off_or_names[i]);
    }
    cpu_aumod_mask = tcg_global_mem_new(tcg_env,
                                        offsetof(CPUArchState, aumod_mask),
                                        "AUModMask");
    cpu_readport_idx_r = tcg_global_mem_new(tcg_env,
                                            offsetof(CPUArchState, readport_idx_r),
                                            "RD_idx_r");
    cpu_readport_idx_w = tcg_global_mem_new(tcg_env,
                                            offsetof(CPUArchState, readport_idx_w),
                                            "RD_idx_w");
    cpu_readport_count = tcg_global_mem_new(tcg_env,
                                            offsetof(CPUArchState, readport_count),
                                            "RD_count");


    done_init = 1;
}


void meta_minim_init_table(void)
{
    minim_insn_desc_t *desc;
    uint16_t match_mask, match_val;
    uint32_t mask_mask, val_mask, enc_mask;

    for (desc = minim_table; desc->prefix != 0xffff; desc++) {
        match_mask = 0xffff;
        match_val = 0;
        mask_mask = desc->core_mask & desc->enc_core;
        val_mask = desc->core_val & mask_mask;

        if (!desc->core_mask) {
            mask_mask = desc->ext_mask & desc->enc_core;
            val_mask = desc->ext_val & mask_mask;
        } else if (desc->se_bit > 0) {
            /* remove se bit */
            mask_mask &= ~(1UL << desc->se_bit);
        }

        for (enc_mask = desc->enc_core; enc_mask;
             enc_mask <<= 1, mask_mask <<= 1, val_mask <<= 1) {
            if (!(enc_mask & 0x80000000UL)) {
                /* non-encoded bit */
                continue;
            }

            match_mask <<= 1;
            match_val <<= 1;

            match_mask |= (uint16_t)((mask_mask >> 31) & 0x1);
            match_val |= (uint16_t)((val_mask >> 31) & 0x1);
        }

        match_val |= desc->prefix;

        desc->dec_mask_core = match_mask;
        desc->dec_val_core = match_val;

        /* extended cases */
        match_mask = match_val = 0;
        mask_mask = desc->ext_mask & desc->enc_ext;
        val_mask = desc->ext_val & mask_mask;

        for (enc_mask = desc->enc_ext; enc_mask;
             enc_mask <<= 1, mask_mask <<= 1, val_mask <<= 1) {
            if (!(enc_mask & 0x80000000UL)) {
                /* non-encoded bit */
                continue;
            }

            match_mask <<= 1;
            match_val <<= 1;

            match_mask |= (uint16_t)((mask_mask >> 31) & 0x1);
            match_val |= (uint16_t)((val_mask >> 31) & 0x1);
        }

        /* begin never matching longer encodings */
        desc->dec_mask_ext = desc->dec_mask_long = 0;
        desc->dec_val_ext = desc->dec_val_long = 0xffff;

        if (desc->enc_core == 0x00003fff) {
            /* long instruction */
            desc->dec_mask_long = (uint16_t)(match_mask | 0xf000);
            desc->dec_val_long = (uint16_t)(match_val | 0xb000);
        } else if (desc->ext_mask) {
            /* extended instruction */
            desc->dec_mask_ext = (uint16_t)(match_mask | 0xc000);
            desc->dec_val_ext = (uint16_t)(match_val | 0xc000);
        }
    }
}



MetaCore *cpu_meta_core_init(const char *cpu_model, unsigned int extirqs)
{
    MetaCore *core;
    int i, t;
    struct MetaCoreConfig *cpu_info;
    int threads;

    /* find matching core id */
    if (cpu_model) {
        for (i = 0; i < ARRAY_SIZE(meta_cores); ++i) {
            if (!strcmp(meta_cores[i].name, cpu_model)) {
                cpu_info = &meta_cores[i];
                goto core_found;
            }
        }
        return NULL;
    } else {
        /* latest and "greatest" */
        cpu_info = &meta_cores[ARRAY_SIZE(meta_cores) - 1];
    }
core_found:
    if (current_machine->smp.threads > 1) {
        /* -smp x,threads=n argument takes priority */
        threads = current_machine->smp.threads;
    } else {
        /* otherwise use default for the core config */
        threads = cpu_info->num_threads;
    }
    if (threads < 1) {
        threads = 1;
    } else if (threads > META_MAX_THREADS) {
        threads = META_MAX_THREADS;
    }

    core = g_malloc0(sizeof(MetaCore));
    core->global.core_rev = cpu_info->core_rev;
    core->global.core_id = (0x14 << 24)
                         | ((uint32_t)cpu_info->core_id << 16)
                         | cpu_info->core_config;
    core->global.core_config2 = cpu_info->core_config2;
    core->num_threads = threads;
    core->global.dspram_addr_bits = cpu_info->dspram_size;
    core->timer_period = 1000; /* 1us, 1MHz */

    meta_minim_init_table();

    for (t = 0; t < threads; ++t) {
        CPUArchState *env = &core->threads[t].env;

        object_initialize(&core->threads[t], sizeof(core->threads[t]),
                          TYPE_META_CPU);
        env->global = &core->global;
        env->thread_num = t;
        env->tcaps = cpu_info->tcaps[t];
        meta_init_timers(env);
        qdev_realize(DEVICE(&core->threads[t]), NULL, &error_fatal);
    }

    meta_init_core_registers(core, extirqs);
    core->switch_bus = meta_switch_bus_new(NULL);

    return core;
}
