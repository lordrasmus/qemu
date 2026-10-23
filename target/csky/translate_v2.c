/*
 * CSKY translation
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
#include "translate.h"

#include "disas/disas.h"
#include "accel/tcg/cpu-ldst.h"
#include "exec/log.h"
#include "exec/translator.h"
#include "exec/translation-block.h"
#include "tcg/tcg-op.h"
#include "qemu/log.h"
#include "qemu/qemu-print.h"
#include <math.h>
#include "gdbstub/helpers.h"

#include "exec/helper-proto.h"
#include "exec/helper-gen.h"

#define HELPER_H "helper.h"
#include "exec/helper-info.c.inc"
#undef  HELPER_H

/*
 * The C-SKY fork's gdbstub exported whether a gdb server had been started;
 * a guest bkpt then raised a debug exception instead of ending the run.
 * That hook is not carried over, so bkpt always ends the run.
 */
static const bool csky_gdbserver_started;

/* Writable temporaries holding a value; tcg_constant_*() are read-only.  */
static inline TCGv_i32 csky_temp_const_i32(int32_t val)
{
    TCGv_i32 t = tcg_temp_new_i32();

    tcg_gen_movi_i32(t, val);
    return t;
}

static inline TCGv csky_temp_const_tl(target_ulong val)
{
    TCGv t = tcg_temp_new();

    tcg_gen_movi_tl(t, val);
    return t;
}

/*******************************/
#define sp 14
#define SVBR 30
#define FP 23
#define TOP 24
#define BSP 25
#define WORD_MASK 0x3
#define REG_MASK   0x1c
/*******************************************/

/* is_jmp field values */
#define DISAS_JUMP    DISAS_TARGET_0 /* only pc was modified dynamically */
#define DISAS_UPDATE  DISAS_TARGET_1 /* cpu state was modified dynamically */
#define DISAS_TB_JUMP DISAS_TARGET_2 /* only pc was modified statically */

static TCGv_i32 cpu_R[32];
static TCGv_i32 cpu_c;
static TCGv_i32 cpu_v;
static TCGv_i32 cpu_hi ;
static TCGv_i32 cpu_lo;
static TCGv_i32 cpu_hi_guard;
static TCGv_i32 cpu_lo_guard;

static TCGv_i32 cpu_F0s, cpu_F1s;
static TCGv_i64 cpu_F0d, cpu_F1d;
TCGv_i32 excl_val;
TCGv_i32 excl_addr;


static const char *regnames[] = {
    "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7",
    "r8", "r9", "r10", "r11", "r12", "r13", "sp", "r15",
    "r16", "r17", "r18", "r19", "r20", "r21", "r22", "r23",
    "r24", "r25", "r26", "r27", "r28", "r29", "r30", "r31" };

#if defined(CONFIG_USER_ONLY)
#define IS_SUPER(ctx)   0
#define IS_TRUST(ctx)   0
#else
#define IS_SUPER(ctx)   (ctx->super)
#define IS_TRUST(ctx)   (ctx->trust)
#endif

static inline void csky_concat_i32_i64(TCGv_i64 dest, TCGv_i32 first,
                                        TCGv_i32 second)
{
#if TARGET_BIG_ENDIAN
    tcg_gen_concat_i32_i64(dest, second, first);
#else
    tcg_gen_concat_i32_i64(dest, first, second);
#endif
}

static inline void csky_extr_i64_i32(TCGv_i32 first, TCGv_i32 second,
                                        TCGv_i64 arg)
{
#if TARGET_BIG_ENDIAN
    tcg_gen_extr_i64_i32(second, first, arg);
#else
    tcg_gen_extr_i64_i32(first, second, arg);
#endif

}

static inline void mulsha(int rx, int ry)
{
    TCGv t0 = tcg_temp_new();
    TCGv t1 = tcg_temp_new();
    TCGv_i64 t2 = tcg_temp_new_i64();
    TCGv_i64 t3 = tcg_temp_new_i64();
    TCGLabel *l1 = gen_new_label();

    tcg_gen_ext16s_tl(t0, cpu_R[rx]);
    tcg_gen_ext16s_tl(t1, cpu_R[ry]);
    tcg_gen_mul_tl(t0, t0, t1);
    tcg_gen_ext_tl_i64(t2, t0);
    tcg_gen_concat_i32_i64(t3, cpu_lo, cpu_lo_guard);
    tcg_gen_add_i64(t2, t3, t2);
    tcg_gen_extrl_i64_i32(cpu_lo, t2);
    tcg_gen_shri_i64(t3, t2, 31);
    tcg_gen_shri_i64(t2, t2, 32);
    tcg_gen_extrl_i64_i32(cpu_lo_guard, t2);
    tcg_gen_movi_i32(cpu_v, 0);
    tcg_gen_brcondi_i64(TCG_COND_EQ, t3, 0x0, l1);
    tcg_gen_brcondi_i64(TCG_COND_EQ, t3, 0x1ffffffffLL, l1);
    tcg_gen_movi_i32(cpu_v, 1);
    gen_set_label(l1);

}

static inline void mulshs(int rx, int ry)
{
    TCGv t0 = tcg_temp_new();
    TCGv t1 = tcg_temp_new();
    TCGv_i64 t2 = tcg_temp_new_i64();
    TCGv_i64 t3 = tcg_temp_new_i64();
    TCGLabel *l1 = gen_new_label();

    tcg_gen_ext16s_tl(t0, cpu_R[rx]);
    tcg_gen_ext16s_tl(t1, cpu_R[ry]);
    tcg_gen_mul_tl(t0, t0, t1);
    tcg_gen_ext_tl_i64(t2, t0);
    tcg_gen_concat_i32_i64(t3, cpu_lo, cpu_lo_guard);
    tcg_gen_sub_i64(t2, t3, t2);
    tcg_gen_extrl_i64_i32(cpu_lo, t2);
    tcg_gen_shri_i64(t3, t2, 31);
    tcg_gen_shri_i64(t2, t2, 32);
    tcg_gen_extrl_i64_i32(cpu_lo_guard, t2);
    tcg_gen_movi_i32(cpu_v, 0);
    tcg_gen_brcondi_i64(TCG_COND_EQ, t3, 0x0, l1);
    tcg_gen_brcondi_i64(TCG_COND_EQ, t3, 0x1ffffffffLL, l1);
    tcg_gen_movi_i32(cpu_v, 1);
    gen_set_label(l1);


}

static inline void mulsw(int rz, int rx, int ry)
{
    TCGv_i64 t0 = tcg_temp_new_i64();
    TCGv_i64 t1 = tcg_temp_new_i64();
    TCGv t2 = tcg_temp_new();

    tcg_gen_ext16s_tl(t2, cpu_R[rx]);
    tcg_gen_ext_tl_i64(t0, t2);
    tcg_gen_ext_tl_i64(t1, cpu_R[ry]);
    tcg_gen_mul_i64(t0, t0, t1);
    tcg_gen_shri_i64(t0, t0, 16);
    tcg_gen_extrl_i64_i32(cpu_R[rz], t0);

}
static inline void mulswa(int rx, int ry)
{
    TCGv t0 = tcg_temp_new();
    TCGv_i64 t1 = tcg_temp_new_i64();
    TCGv_i64 t2 = tcg_temp_new_i64();
    TCGLabel *l1 = gen_new_label();

    tcg_gen_ext16s_tl(t0, cpu_R[rx]) ;
    tcg_gen_ext_tl_i64(t1, t0);
    tcg_gen_ext_tl_i64(t2, cpu_R[ry]);
    tcg_gen_mul_i64(t1, t1, t2) ;
    tcg_gen_shri_i64(t1, t1, 16);
    tcg_gen_concat_i32_i64(t2, cpu_lo, cpu_lo_guard);
    tcg_gen_add_i64(t2, t2, t1);
    tcg_gen_extrl_i64_i32(cpu_lo, t2);
    tcg_gen_shri_i64(t1, t2, 31);
    tcg_gen_shri_i64(t2, t2, 32);
    tcg_gen_extrl_i64_i32(cpu_lo_guard, t2);
    tcg_gen_movi_i32(cpu_v, 0);
    tcg_gen_brcondi_i64(TCG_COND_EQ, t1, 0x0, l1);
    tcg_gen_brcondi_i64(TCG_COND_EQ, t1, 0x1ffffffffLL, l1);
    tcg_gen_movi_i32(cpu_v, 1);
    gen_set_label(l1);


}
static inline void mulsws(int rx, int ry)
{
    TCGv t0 = tcg_temp_new();
    TCGv_i64 t1 = tcg_temp_new_i64();
    TCGv_i64 t2 = tcg_temp_new_i64();
    TCGLabel *l1 = gen_new_label();

    tcg_gen_ext16s_tl(t0, cpu_R[rx]);
    tcg_gen_ext_i32_i64(t1, t0);
    tcg_gen_ext_i32_i64(t2, cpu_R[ry]);
    tcg_gen_mul_i64(t1, t1, t2);
    tcg_gen_shri_i64(t1, t1, 16);
    tcg_gen_concat_i32_i64(t2, cpu_lo, cpu_lo_guard);
    tcg_gen_sub_i64(t2, t2, t1);
    tcg_gen_extrl_i64_i32(cpu_lo, t2);
    tcg_gen_shri_i64(t1, t2, 31);
    tcg_gen_shri_i64(t2, t2, 32);
    tcg_gen_extrl_i64_i32(cpu_lo_guard, t2);
    tcg_gen_movi_i32(cpu_v, 0);
    tcg_gen_brcondi_i64(TCG_COND_EQ, t1, 0x0, l1);
    tcg_gen_brcondi_i64(TCG_COND_EQ, t1, 0x1ffffffffLL, l1);
    tcg_gen_movi_i32(cpu_v, 1);
    gen_set_label(l1);

}

/* initialize TCG globals. */
void csky_translate_init(void)
{
    int i;

    for (i = 0; i < 32; i++) {
        cpu_R[i] = tcg_global_mem_new_i32(tcg_env,
            offsetof(CPUCSKYState, regs[i]),
            regnames[i]);
    }

    cpu_c = tcg_global_mem_new_i32(tcg_env,
        offsetof(CPUCSKYState, psr_c), "cpu_c");
    cpu_v = tcg_global_mem_new_i32(tcg_env,
        offsetof(CPUCSKYState, dcsr_v), "cpu_v");
    cpu_hi = tcg_global_mem_new_i32(tcg_env,
        offsetof(CPUCSKYState, hi), "cpu_hi");
    cpu_lo = tcg_global_mem_new_i32(tcg_env,
        offsetof(CPUCSKYState, lo), "cpu_lo");
    cpu_hi_guard = tcg_global_mem_new_i32(tcg_env,
        offsetof(CPUCSKYState, hi_guard), "cpu_hi_guard");
    cpu_lo_guard = tcg_global_mem_new_i32(tcg_env,
        offsetof(CPUCSKYState, lo_guard), "cpu_lo_guard");
    excl_addr = tcg_global_mem_new_i32(tcg_env,
        offsetof(CPUCSKYState, excl_addr), "excl_addr");
    excl_val = tcg_global_mem_new_i32(tcg_env,
        offsetof(CPUCSKYState, excl_val), "excl_val");
}

static TCGv_i32 new_tmp(void)
{
    return tcg_temp_new_i32();
}

static void dead_tmp(TCGv tmp)
{
}


static inline TCGv load_cpu_offset(int offset)
{
    TCGv tmp = tcg_temp_new();
    tcg_gen_ld_i32(tmp, tcg_env, offset);
    return tmp;
}

#define load_cpu_field(name) load_cpu_offset(offsetof(CPUCSKYState, name))

static inline void store_cpu_offset(TCGv var, int offset)
{
    tcg_gen_st_i32(var, tcg_env, offset);
}

#define store_cpu_field(var, name) \
store_cpu_offset(var, offsetof(CPUCSKYState, name))

static inline void gen_save_pc(target_ulong pc)
{
    TCGv t0 = csky_temp_const_tl(pc);
    store_cpu_field(t0, pc);

}

static inline void generate_exception(DisasContext *ctx, int excp)
{
    TCGv t0;

    print_exception(ctx, excp);

    t0 = csky_temp_const_tl(excp);
    gen_save_pc(ctx->pc);
    gen_helper_exception(tcg_env, t0);
    ctx->base.is_jmp = DISAS_NORETURN;

}

static inline bool use_goto_tb(DisasContext *s, uint32_t dest)
{
    return translator_use_goto_tb(&s->base, dest);
}

static inline void gen_goto_tb(DisasContext *ctx, int n, uint32_t dest)
{
    TranslationBlock *tb;
    TCGv t0;

    tb = ctx->base.tb;

#if !defined(CONFIG_USER_ONLY)
    if (unlikely((ctx->trace_mode == INST_TRACE_MODE)
        || (ctx->trace_mode == BRAN_TRACE_MODE))) {
        gen_save_pc(dest);
        t0 = csky_temp_const_tl(EXCP_CSKY_TRACE);
        gen_helper_exception(tcg_env, t0);
        ctx->maybe_change_flow = 1;
    } else
#endif
    if (use_goto_tb(ctx, dest)) {
        tcg_gen_goto_tb(n);
        gen_save_pc(dest);
        tcg_gen_exit_tb(tb, n);
    } else {
        gen_save_pc(dest);
        tcg_gen_exit_tb(NULL, 0);
    }

}

/*
 * The ld/st forms the old tcg-op.h had, with the context's alignment
 * requirement added (see DisasContext.align).
 */
#define GEN_CSKY_LD(name, mop)                                          \
static inline void gen_csky_##name(DisasContext *ctx, TCGv ret, TCGv addr) \
{                                                                       \
    tcg_gen_qemu_ld_tl(ret, addr, ctx->mem_idx, (mop) | ctx->align);    \
}
#define GEN_CSKY_ST(name, mop)                                          \
static inline void gen_csky_##name(DisasContext *ctx, TCGv val, TCGv addr) \
{                                                                       \
    tcg_gen_qemu_st_tl(val, addr, ctx->mem_idx, (mop) | ctx->align);    \
}
GEN_CSKY_LD(ld8u, MO_UB)
GEN_CSKY_LD(ld8s, MO_SB)
GEN_CSKY_LD(ld16u, MO_TEUW)
GEN_CSKY_LD(ld16s, MO_TESW)
GEN_CSKY_LD(ld32u, MO_TEUL)
GEN_CSKY_ST(st8, MO_UB)
GEN_CSKY_ST(st16, MO_TEUW)
GEN_CSKY_ST(st32, MO_TEUL)

#define ldst(name, rx, rz, imm, isize)                             \
do {                                                               \
    if (ctx->bctm) {                                               \
        TCGLabel *l1 = gen_new_label();                            \
        tcg_gen_brcondi_tl(TCG_COND_NE, cpu_R[rx], 0, l1);         \
        tcg_gen_movi_tl(cpu_R[15], ctx->pc + isize);               \
        tcg_gen_subi_tl(t0, cpu_R[SVBR], 4);                       \
        store_cpu_field(t0, pc);                                   \
        tcg_gen_exit_tb(NULL, 0);                                  \
        gen_set_label(l1);                                         \
        tcg_gen_addi_tl(t0, cpu_R[rx], imm);                       \
        gen_csky_##name(ctx, cpu_R[rz], t0);                        \
        if (gen_mem_trace()) {                                     \
            TCGv_i32 t2 = csky_temp_const_i32(ctx->pc);                  \
            gen_helper_trace_##name(tcg_env, t2, cpu_R[rz], t0);   \
                                             \
        }                                                          \
        gen_goto_tb(ctx, 1, ctx->pc + isize);                      \
        ctx->base.is_jmp = DISAS_TB_JUMP;                          \
    } else {                                                       \
        tcg_gen_addi_tl(t0, cpu_R[rx], imm);                       \
        gen_csky_##name(ctx, cpu_R[rz], t0);                        \
        if (gen_mem_trace()) {                                     \
            TCGv_i32 t2 = csky_temp_const_i32(ctx->pc);                  \
            gen_helper_trace_##name(tcg_env, t2, cpu_R[rz], t0);   \
                                             \
        }                                                          \
    }                                                              \
} while (0)

#define ldrstr(name, rx, ry, rz, imm)                              \
do {                                                               \
    if (ctx->bctm) {                                               \
        TCGLabel *l1 = gen_new_label();                            \
        tcg_gen_brcondi_tl(TCG_COND_NE, cpu_R[rx], 0, l1);         \
        tcg_gen_movi_tl(cpu_R[15], ctx->pc + 4);                   \
        tcg_gen_subi_tl(t0, cpu_R[SVBR], 4);                       \
        store_cpu_field(t0, pc);                                   \
        tcg_gen_exit_tb(NULL, 0);                                  \
        gen_set_label(l1);                                         \
        tcg_gen_shli_tl(t0, cpu_R[ry], imm);                       \
        tcg_gen_add_tl(t0, cpu_R[rx], t0);                         \
        gen_csky_##name(ctx, cpu_R[rz], t0);                        \
        if (gen_mem_trace()) {                                     \
            TCGv_i32 t2 = csky_temp_const_i32(ctx->pc);                  \
            gen_helper_trace_##name(tcg_env, t2, cpu_R[rz], t0);   \
                                             \
        }                                                          \
        gen_goto_tb(ctx, 1, ctx->pc + 4);                          \
        ctx->base.is_jmp = DISAS_TB_JUMP;                          \
        break;                                                     \
    } else {                                                       \
        tcg_gen_shli_tl(t0, cpu_R[ry], imm);                       \
        tcg_gen_add_tl(t0, cpu_R[rx], t0);                         \
        gen_csky_##name(ctx, cpu_R[rz], t0);                        \
        if (gen_mem_trace()) {                                     \
            TCGv_i32 t2 = csky_temp_const_i32(ctx->pc);                  \
            gen_helper_trace_##name(tcg_env, t2, cpu_R[rz], t0);   \
                                             \
        }                                                          \
    }                                                              \
} while (0)

/* The insn support the cpu contained in the argument 'flags'.  */
static inline void check_insn(DisasContext *ctx, uint64_t flags)
{
    if (unlikely(!has_insn(ctx, flags))) {
        generate_exception(ctx, EXCP_CSKY_UDEF);
    }
}

/* The insn not support the cpu contained in the argument 'flags'.  */
static inline void check_insn_except(DisasContext *ctx, uint64_t flags)
{
    if (unlikely(has_insn(ctx, flags))) {
        generate_exception(ctx, EXCP_CSKY_UDEF);
    }
}

/* Set a variable to the value of a CPU register.  */
static void load_reg_var(DisasContext *s, TCGv var, int reg)
{
    tcg_gen_mov_i32(var, cpu_R[reg]);
}

/* Create a new temporary and set it to the value of a CPU register.  */
static inline TCGv load_reg(DisasContext *s, int reg)
{
    TCGv tmp = new_tmp();
    load_reg_var(s, tmp, reg);
    return tmp;
}

static inline void addc(int rz, int rx, int ry)
{
    TCGv t0 = tcg_temp_new();
    TCGv t1 = tcg_temp_new();
    TCGLabel *l1 = gen_new_label();
    TCGLabel *l2 = gen_new_label();

    tcg_gen_mov_tl(t1, cpu_R[rx]);
    tcg_gen_add_tl(t0, t1, cpu_R[ry]);
    tcg_gen_add_tl(cpu_R[rz], t0, cpu_c);
    tcg_gen_brcondi_tl(TCG_COND_NE, cpu_c, 0, l1);
    tcg_gen_setcond_tl(TCG_COND_LTU, cpu_c, cpu_R[rz], t1);
    tcg_gen_br(l2);
    gen_set_label(l1);
    tcg_gen_setcond_tl(TCG_COND_LEU, cpu_c, cpu_R[rz], t1);
    gen_set_label(l2);

}

static inline void subc(int rz, int rx, int ry)
{
    TCGv t0 = tcg_temp_new();
    TCGv t1 = tcg_temp_new();
    TCGv t2 = tcg_temp_new();
    TCGLabel *l1 = gen_new_label();
    TCGLabel *l2 = gen_new_label();

    tcg_gen_subfi_tl(t0, 1, cpu_c);
    tcg_gen_sub_tl(t1, cpu_R[rx], cpu_R[ry]);
    tcg_gen_sub_tl(t2, t1, t0);
    tcg_gen_brcondi_tl(TCG_COND_NE, cpu_c, 0, l1);
    tcg_gen_setcond_tl(TCG_COND_GTU, cpu_c, cpu_R[rx], cpu_R[ry]);
    tcg_gen_br(l2);
    gen_set_label(l1);
    tcg_gen_setcond_tl(TCG_COND_GEU, cpu_c, cpu_R[rx], cpu_R[ry]);
    gen_set_label(l2);
    tcg_gen_mov_tl(cpu_R[rz], t2);


}


static inline void tstnbz(int rx)
{
    TCGv t0 = tcg_temp_new();
    TCGLabel *l1 = gen_new_label();

    tcg_gen_movi_tl(cpu_c, 0);

    tcg_gen_andi_tl(t0, cpu_R[rx], 0xff000000);
    tcg_gen_brcondi_tl(TCG_COND_EQ, t0, 0, l1);

    tcg_gen_andi_tl(t0, cpu_R[rx], 0x00ff0000);
    tcg_gen_brcondi_tl(TCG_COND_EQ, t0, 0, l1);

    tcg_gen_andi_tl(t0, cpu_R[rx], 0x0000ff00);
    tcg_gen_brcondi_tl(TCG_COND_EQ, t0, 0, l1);

    tcg_gen_andi_tl(t0, cpu_R[rx], 0x000000ff);
    tcg_gen_brcondi_tl(TCG_COND_EQ, t0, 0, l1);

    tcg_gen_movi_tl(cpu_c, 1);
    gen_set_label(l1);

}

static inline void lsl(int rz, int rx, int ry)
{
    TCGv t0 = tcg_temp_new();
    TCGv t1 = tcg_temp_new();

    TCGLabel *l1 = gen_new_label();

    tcg_gen_mov_tl(t1, cpu_R[rx]);
    tcg_gen_andi_tl(t0, cpu_R[ry], 0x3f);
    tcg_gen_movi_tl(cpu_R[rz], 0);
    tcg_gen_brcondi_tl(TCG_COND_GTU, t0, 31, l1);
    tcg_gen_shl_tl(cpu_R[rz], t1, t0);
    gen_set_label(l1);

}

static inline void lsr(int rz, int rx, int ry)
{
    TCGv t0 = tcg_temp_new();
    TCGv t1 = tcg_temp_new();

    TCGLabel *l1 = gen_new_label();

    tcg_gen_mov_tl(t1, cpu_R[rx]);
    tcg_gen_andi_tl(t0, cpu_R[ry], 0x3f);
    tcg_gen_movi_tl(cpu_R[rz], 0);
    tcg_gen_brcondi_tl(TCG_COND_GTU, t0, 31, l1);
    tcg_gen_shr_tl(cpu_R[rz], t1, t0);
    gen_set_label(l1);

}

static inline void asr(int rz, int rx, int ry)
{
    TCGv t0 = tcg_temp_new();

    TCGLabel *l1 = gen_new_label();
    tcg_gen_andi_tl(t0, cpu_R[ry], 0x3f);
    tcg_gen_brcondi_tl(TCG_COND_LEU, t0, 31, l1);
    tcg_gen_movi_tl(t0, 31);
    gen_set_label(l1);
    tcg_gen_sar_tl(cpu_R[rz], cpu_R[rx], t0);

}

static inline void rotl(int rz, int rx, int ry)
{
    TCGv t0 = tcg_temp_new();
    TCGv t1 = tcg_temp_new();

    TCGLabel *l1 = gen_new_label();

    tcg_gen_mov_tl(t1, cpu_R[rx]);
    tcg_gen_andi_tl(t0, cpu_R[ry], 0x3f);
    tcg_gen_movi_tl(cpu_R[rz], 0);
    tcg_gen_brcondi_tl(TCG_COND_GTU, t0, 31, l1);
    tcg_gen_rotl_tl(cpu_R[rz], t1, t0);
    gen_set_label(l1);

}

static inline void branch16(DisasContext *ctx, TCGCond cond, int offset)
{
    int val;
    TCGLabel *l1 = gen_new_label();

    val = offset << 1;
    if (val & 0x400) {
        val |= 0xfffffc00;
    }
    val += ctx->pc;

    tcg_gen_brcondi_tl(cond, cpu_c, 0, l1);
    gen_goto_tb(ctx, 1, ctx->pc + 2);
    gen_set_label(l1);

    gen_goto_tb(ctx, 0, val);

    ctx->base.is_jmp = DISAS_TB_JUMP;

}

static inline void bsr16(DisasContext *ctx, int offset)
{
    int val;

    val = offset << 1;
    if (val & 0x400) {
        val |= 0xfffffc00;
    }
    val += ctx->pc;

    gen_goto_tb(ctx, 0, val);

    ctx->base.is_jmp = DISAS_TB_JUMP;
}

static inline void pop16(DisasContext *ctx, int imm)
{
    int i;
    TCGv t0 = tcg_temp_new();
    tcg_gen_mov_tl(t0, cpu_R[sp]);

    TCGv t1 = tcg_temp_new();
    tcg_gen_movi_tl(t1, 0);

    TCGv t2 = csky_temp_const_i32(DATA_RADDR);
    TCGv t3 = csky_temp_const_i32(ctx->pc);
    if (imm & 0x10) {
        tcg_gen_addi_tl(t1, t1, 1);
    }
    if (imm & 0xf) {
        tcg_gen_addi_tl(t1, t1, imm & 0xf);
    }
    if (gen_mem_trace()) {
        gen_helper_trace_m_addr(tcg_env, t3, t0, t1, t2);
    }
    if (imm & 0xf) {
        for (i = 0; i < (imm & 0xf); i++) {
            tcg_gen_qemu_ld_tl(cpu_R[i + 4], t0, ctx->mem_idx, MO_TEUL | ctx->align);
            if (gen_mem_trace()) {
                gen_helper_trace_m_value(tcg_env, t3, t0, cpu_R[i + 4]);
            }
            tcg_gen_addi_i32(t0, t0, 4);
        }
    }
    if (imm & 0x10) {
        tcg_gen_qemu_ld_tl(cpu_R[15], t0, ctx->mem_idx, MO_TEUL | ctx->align);
        if (gen_mem_trace()) {
            gen_helper_trace_m_value(tcg_env, t3, t0, cpu_R[15]);
        }
        tcg_gen_addi_i32(t0, t0, 4);
    }
    tcg_gen_mov_tl(cpu_R[sp], t0);

    tcg_gen_andi_tl(t0, cpu_R[15], 0xfffffffe);
    store_cpu_field(t0, pc);
    ctx->base.is_jmp = DISAS_JUMP;
}

static inline void push16(DisasContext *ctx, int imm)
{
    int i;
    TCGv t0 = tcg_temp_new();
    tcg_gen_mov_tl(t0, cpu_R[sp]);

    TCGv t1 = tcg_temp_new();
    tcg_gen_movi_tl(t1, 0);

    TCGv t2 = csky_temp_const_i32(DATA_WADDR);

    TCGv t3 = csky_temp_const_i32(ctx->pc);
    if (imm & 0x10) {
        tcg_gen_addi_tl(t1, t1, 1);
    }
    if (imm & 0xf) {
        tcg_gen_addi_tl(t1, t1, imm & 0xf);
    }
    if (gen_mem_trace()) {
        gen_helper_trace_m_addr(tcg_env, t3, t0, t1, t2);
    }
    if (imm & 0x10) {
        tcg_gen_subi_i32(t0, t0, 4);
        tcg_gen_qemu_st_tl(cpu_R[15], t0, ctx->mem_idx, MO_TEUL | ctx->align);
        if (gen_mem_trace()) {
            gen_helper_trace_m_value(tcg_env, t3, t0, cpu_R[15]);
        }
    }

    if (imm & 0xf) {
        for (i = (imm & 0xf); i > 0; i--) {
            tcg_gen_subi_i32(t0, t0, 4);
            tcg_gen_qemu_st_tl(cpu_R[i + 3], t0, ctx->mem_idx, MO_TEUL | ctx->align);
            if (gen_mem_trace()) {
                gen_helper_trace_m_value(tcg_env, t3, t0, cpu_R[i + 3]);
            }
        }
    }
    tcg_gen_mov_tl(cpu_R[sp], t0);
}

static inline void gen_cmp16(DisasContext *ctx, uint32_t sop, int rz, int rx)
{
    switch (sop) {
    case 0x0:
        /* cmphs */
        tcg_gen_setcond_tl(TCG_COND_GEU, cpu_c, cpu_R[rx], cpu_R[rz]);
        break;
    case 0x1:
        /* cmplt */
        tcg_gen_setcond_tl(TCG_COND_LT, cpu_c, cpu_R[rx], cpu_R[rz]);
        break;
    case 0x2:
        /* cmpne */
        tcg_gen_setcond_tl(TCG_COND_NE, cpu_c, cpu_R[rx], cpu_R[rz]);
        break;
    case 0x3:
        /* mvcv */
        tcg_gen_subfi_tl(cpu_R[rz], 1, cpu_c);
        break;
    default:
        break;
    }
}

static inline void gen_logic_and16(DisasContext *ctx, uint32_t sop,
                                   int rz, int rx)
{
    switch (sop) {
    case 0x0:
        /* and */
        tcg_gen_and_tl(cpu_R[rz], cpu_R[rz], cpu_R[rx]);
        break;
    case 0x1:
        /* andn */
        tcg_gen_andc_tl(cpu_R[rz], cpu_R[rz], cpu_R[rx]);
        break;
    case 0x2:
        /* tst */
        {
            TCGv t0 = tcg_temp_new();
            tcg_gen_and_tl(t0, cpu_R[rx], cpu_R[rz]);
            tcg_gen_setcondi_tl(TCG_COND_NE, cpu_c, t0, 0);

        }
        break;
    case 0x3:
        /* tstnbz */
        tstnbz(rx);
        break;
    default:
        break;
    }
}

static inline void gen_logic_or16(DisasContext *ctx, uint32_t sop,
                                  int rz, int rx)
{
    switch (sop) {
    case 0x0:
        /* or */
        tcg_gen_or_tl(cpu_R[rz], cpu_R[rz], cpu_R[rx]);
        break;
    case 0x1:
        /* xor */
        tcg_gen_xor_tl(cpu_R[rz], cpu_R[rz], cpu_R[rx]);
        break;
    case 0x2:
        /* nor */
        tcg_gen_nor_tl(cpu_R[rz], cpu_R[rz], cpu_R[rx]);
        break;
    case 0x3:
        /* mov */
        tcg_gen_mov_tl(cpu_R[rz], cpu_R[rx]);
        break;
    default:
        generate_exception(ctx, EXCP_CSKY_UDEF);
        break;
    }
}

static inline void gen_shift_reg16(DisasContext *ctx, uint32_t sop,
                                   int rz, int rx)
{
    switch (sop) {
    case 0x0:
        /* lsl */
        lsl(rz, rz, rx);
        break;
    case 0x1:
        /* lsr */
        lsr(rz, rz, rx);
        break;
    case 0x2:
        /* asr */
        asr(rz, rz, rx);
        break;
    case 0x3:
        /* rotl */
        rotl(rz, rz, rx);
        break;
    default:
        break;
    }
}

static inline void gen_ext16(DisasContext *ctx, uint32_t sop, int rz, int rx)
{
    switch (sop) {
    case 0x0:
        /* zextb */
        tcg_gen_andi_tl(cpu_R[rz], cpu_R[rx], 0xff);
        break;
    case 0x1:
        /* zexth */
        tcg_gen_andi_tl(cpu_R[rz], cpu_R[rx], 0xffff);
        break;
    case 0x2:
        /* sextb */
        tcg_gen_shli_tl(cpu_R[rz], cpu_R[rx], 24);
        tcg_gen_sari_tl(cpu_R[rz], cpu_R[rz], 24);
        break;
    case 0x3:
        /* sexth */
        tcg_gen_shli_tl(cpu_R[rz], cpu_R[rx], 16);
        tcg_gen_sari_tl(cpu_R[rz], cpu_R[rz], 16);
        break;
    default:
        break;
    }
}

static inline void gen_arith_misc16(DisasContext *ctx, uint32_t sop,
                                    int rz, int rx)
{
    switch (sop) {
    case 0x0:
        /* jmp */
        {
            TCGv t0 = tcg_temp_new();
            tcg_gen_andi_tl(t0, cpu_R[rx], 0xfffffffe);
            store_cpu_field(t0, pc);

#if !defined(CONFIG_USER_ONLY)
            if ((ctx->trace_mode == BRAN_TRACE_MODE)
                || (ctx->trace_mode == INST_TRACE_MODE)) {
                tcg_gen_movi_i32(t0, EXCP_CSKY_TRACE);
                gen_helper_exception(tcg_env, t0);
            }
            ctx->maybe_change_flow = 1;
#endif
            ctx->base.is_jmp = DISAS_JUMP;
        }
        break;
    case 0x1:
        /* jsr */
        {
            TCGv t0 = tcg_temp_new();
            tcg_gen_andi_tl(t0, cpu_R[rx], 0xfffffffe);
            tcg_gen_movi_tl(cpu_R[15], ctx->pc + 2);
            store_cpu_field(t0, pc);

#if !defined(CONFIG_USER_ONLY)
            if ((ctx->trace_mode == BRAN_TRACE_MODE)
                || (ctx->trace_mode == INST_TRACE_MODE)) {
                tcg_gen_movi_i32(t0, EXCP_CSKY_TRACE);
                gen_helper_exception(tcg_env, t0);
            }
            ctx->maybe_change_flow = 1;
#endif
            ctx->base.is_jmp = DISAS_JUMP;
            break;
        }
        break;
    case 0x2:
        /* revb */
        check_insn_except(ctx, CPU_E801);
        tcg_gen_bswap32_i32(cpu_R[rz], cpu_R[rx]);
        break;
    case 0x3:
        /* revh */
        {
            check_insn_except(ctx, CPU_E801);
            TCGv t0 = tcg_temp_new();
            TCGv t1 = tcg_temp_new();

            tcg_gen_bswap32_i32(t0, cpu_R[rx]);
            tcg_gen_shri_tl(t1, t0, 16);
            tcg_gen_shli_tl(t0, t0, 16);
            tcg_gen_or_tl(cpu_R[rz], t0, t1);

        }
        break;
    default:
        generate_exception(ctx, EXCP_CSKY_UDEF);
        break;
    }
}

static inline void gen_mul16(DisasContext *ctx, uint32_t sop, int rz, int rx)
{
    switch (sop) {
    case 0x0:
        /* mult */
        tcg_gen_mul_tl(cpu_R[rz], cpu_R[rz], cpu_R[rx]);
        break;
    case 0x1:
        /* mulsh */
        check_insn_except(ctx, CPU_E801 | CPU_E802);
        {
            TCGv t0 = tcg_temp_new();

            tcg_gen_ext16s_tl(t0, cpu_R[rx]);
            tcg_gen_ext16s_tl(cpu_R[rz], cpu_R[rz]);
            tcg_gen_mul_tl(cpu_R[rz], cpu_R[rz], t0);

        }
        break;
    default:
        generate_exception(ctx, EXCP_CSKY_UDEF);
        break;
    }
}

static void gen_branch16(DisasContext *ctx, uint32_t op, int offset)
{
    switch (op) {
    case 0x0:
        if (offset == 0) {
            /*bkpt16*/
            if (csky_gdbserver_started) {
                generate_exception(ctx, EXCP_DEBUG);
            } else {
		gen_helper_bkpt_exit();
                //generate_exception(ctx, EXCP_CSKY_BKPT);
            }
#if !defined(CONFIG_USER_ONLY)
            ctx->cannot_be_traced = 1;
#endif
        } else if (!has_insn(ctx, ABIV2_ELRW)) {
            /*bsr16*/
            tcg_gen_movi_tl(cpu_R[15], ctx->pc + 2);
            bsr16(ctx, offset);
        } else {
            /* lrw16 extend */
            unsigned int imm, rz, addr;
            TCGv t0 = tcg_temp_new();

            imm = ((ctx->insn & 0x300) >> 3) | (ctx->insn & 0x1f);
            imm = (~imm & 0x7f) | 0x80;
            rz = (ctx->insn >> 5) & 0x7;
            addr = (ctx->pc + (imm << 2)) & 0xfffffffc ;
            tcg_gen_movi_tl(t0, addr);
            tcg_gen_qemu_ld_tl(cpu_R[rz], t0, ctx->mem_idx, MO_TEUL | ctx->align);
        }
        break;
    case 0x1:
        /*br16*/
        {
            int val;

            val = offset << 1;
            if (val & 0x400) {
                val |= 0xfffffc00;
            }
            val += ctx->pc;

            gen_goto_tb(ctx, 0, val);

            ctx->base.is_jmp = DISAS_TB_JUMP;
        }
        break;
    case 0x2:
        /*bt16*/
        branch16(ctx, TCG_COND_NE, offset);
        break;
    case 0x3:
        /*bf16*/
        branch16(ctx, TCG_COND_EQ, offset);
        break;
    default:
        generate_exception(ctx, EXCP_CSKY_UDEF);
        break;
    }
}

static inline void gen_nvic_insn(DisasContext *ctx, uint32_t op, int imm)
{
    int i;
    TCGv_i32 t0 = tcg_temp_new_i32();
    TCGv_i32 t1 = tcg_temp_new_i32();
    tcg_gen_mov_i32(t0, cpu_R[sp]);

    switch (imm) {
    case 0x0:
        /* nie */
        t1 = load_cpu_field(cp0.epc);
        tcg_gen_subi_i32(t0, t0, 4);
        tcg_gen_qemu_st_tl(t1, t0, ctx->mem_idx, MO_TEUL | ctx->align);
        t1 = load_cpu_field(cp0.epsr);
        tcg_gen_subi_i32(t0, t0, 4);
        tcg_gen_qemu_st_tl(t1, t0, ctx->mem_idx, MO_TEUL | ctx->align);
        tcg_gen_mov_i32(cpu_R[sp], t0);
        t1 = load_cpu_field(cp0.psr);
        tcg_gen_ori_i32(t1, t1, PSR_EE_MASK);
        tcg_gen_ori_i32(t1, t1, PSR_IE_MASK);
        store_cpu_field(t1, cp0.psr);
        break;
    case 0x1:
        /* nir */
#ifndef CONFIG_USER_ONLY
        if (!IS_SUPER(ctx)) {
            generate_exception(ctx, EXCP_CSKY_PRIVILEGE);
        } else {
            tcg_gen_qemu_ld_tl(t1, t0, ctx->mem_idx, MO_TEUL | ctx->align);
            store_cpu_field(t1, cp0.epsr);
            tcg_gen_addi_i32(t0, t0, 4);
            tcg_gen_qemu_ld_tl(t1, t0, ctx->mem_idx, MO_TEUL | ctx->align);
            store_cpu_field(t1, cp0.epc);
            tcg_gen_addi_i32(t0, t0, 4);
            tcg_gen_mov_i32(cpu_R[sp], t0);
            tcg_gen_movi_i32(t0, 0);
            store_cpu_field(t0, idly4_counter);
            gen_helper_rte(tcg_env);
            ctx->base.is_jmp = DISAS_UPDATE;
        }
#else
        generate_exception(ctx, EXCP_CSKY_PRIVILEGE);
#endif
        break;
    case 0x2:
        /* ipush */
        tcg_gen_subi_i32(t0, t0, 4);
        tcg_gen_qemu_st_tl(cpu_R[13], t0, ctx->mem_idx, MO_TEUL | ctx->align);
        tcg_gen_subi_i32(t0, t0, 4);
        tcg_gen_qemu_st_tl(cpu_R[12], t0, ctx->mem_idx, MO_TEUL | ctx->align);
        for (i = 3; i >= 0; i--) {
            tcg_gen_subi_i32(t0, t0, 4);
            tcg_gen_qemu_st_tl(cpu_R[i], t0, ctx->mem_idx, MO_TEUL | ctx->align);
        }
        tcg_gen_mov_i32(cpu_R[sp], t0);
        break;
    case 0x3:
        /* ipop */
        for (i = 0; i <= 3; i++) {
            tcg_gen_qemu_ld_tl(cpu_R[i], t0, ctx->mem_idx, MO_TEUL | ctx->align);
            tcg_gen_addi_i32(t0, t0, 4);
        }
        tcg_gen_qemu_ld_tl(cpu_R[12], t0, ctx->mem_idx, MO_TEUL | ctx->align);
        tcg_gen_addi_i32(t0, t0, 4);
        tcg_gen_qemu_ld_tl(cpu_R[13], t0, ctx->mem_idx, MO_TEUL | ctx->align);
        tcg_gen_addi_i32(t0, t0, 4);
        tcg_gen_mov_i32(cpu_R[sp], t0);
        break;
    default:
        generate_exception(ctx, EXCP_CSKY_UDEF);
        break;
    }
}

static void gen_imm7_arith16(DisasContext *ctx, uint32_t op, uint16_t sop,
                             int imm)
{
    switch (sop) {
    case 0x0:
        /*addisp(2)*/
        tcg_gen_addi_tl(cpu_R[sp], cpu_R[sp], imm << 2);
        break;
    case 0x1:
        /*subisp*/
        tcg_gen_subi_tl(cpu_R[sp], cpu_R[sp], imm << 2);
        break;
    case 0x3:
        check_insn(ctx, CPU_E801 | CPU_E802 | CPU_E803);
        gen_nvic_insn(ctx, op, imm);
        break;
    case 0x4:
        /*pop16*/
        if (ctx->bctm) {
            TCGv t0 = tcg_temp_new();
            TCGLabel *l1 = gen_new_label();
            tcg_gen_brcondi_tl(TCG_COND_NE, cpu_R[sp], 0, l1);
            tcg_gen_movi_tl(cpu_R[15], ctx->pc + 2);
            tcg_gen_subi_tl(t0, cpu_R[SVBR], 4);
            store_cpu_field(t0, pc);
            tcg_gen_exit_tb(NULL, 0);
            gen_set_label(l1);
            pop16(ctx, imm & 0x1f);
            break;
        } else {
            pop16(ctx, imm & 0x1f);
            break;
        }
        QEMU_FALLTHROUGH;
    case 0x6:
        /*push16*/
        if (ctx->bctm) {
            TCGv t0 = tcg_temp_new();
            TCGLabel *l1 = gen_new_label();
            tcg_gen_brcondi_tl(TCG_COND_NE, cpu_R[sp], 0, l1);
            tcg_gen_movi_tl(cpu_R[15], ctx->pc + 2);
            tcg_gen_subi_tl(t0, cpu_R[SVBR], 4);
            store_cpu_field(t0, pc);
            tcg_gen_exit_tb(NULL, 0);
            gen_set_label(l1);
            push16(ctx, imm & 0x1f);
            gen_goto_tb(ctx, 1, ctx->pc + 2);
            ctx->base.is_jmp = DISAS_TB_JUMP;
            break;
        } else {
            push16(ctx, imm & 0x1f);
            break;
        }
    default:
        generate_exception(ctx, EXCP_CSKY_UDEF);
        break;
    }
}

static void gen_imm8_arith16(DisasContext *ctx, uint32_t op, int rz, int imm)
{
    switch (op) {
    case 0x6:
    case 0x7:
        /*addisp(1)*/
        tcg_gen_addi_tl(cpu_R[rz], cpu_R[sp], imm << 2);
        break;
    case 0x8:
    case 0x9:
        /*addi16(1)*/
        tcg_gen_addi_tl(cpu_R[rz], cpu_R[rz], imm + 1);
        break;
    case 0xa:
    case 0xb:
        /*subi16(1)*/
        tcg_gen_subi_tl(cpu_R[rz], cpu_R[rz], imm + 1);
        break;
    case 0xc:
    case 0xd:
        /*movi16*/
        tcg_gen_movi_tl(cpu_R[rz], imm);
        break;
    default:
        generate_exception(ctx, EXCP_CSKY_UDEF);
        break;
    }
}

static void gen_imm5_arith16(DisasContext *ctx, uint32_t op, uint16_t sop,
                             int rx, int imm)
{
    switch (sop) {
    case 0x0:
        /*cmphsi16*/
        tcg_gen_setcondi_tl(TCG_COND_GEU, cpu_c, cpu_R[rx], imm + 1);
        break;
    case 0x1:
        /*cmplti16*/
        tcg_gen_setcondi_tl(TCG_COND_LT, cpu_c, cpu_R[rx], imm + 1);
        break;
    case 0x2:
        /*cmpnei16*/
        tcg_gen_setcondi_tl(TCG_COND_NE, cpu_c, cpu_R[rx], imm);
        break;
    case 0x4:
        /*bclri16*/
        tcg_gen_andi_tl(cpu_R[rx], cpu_R[rx], ~(1 << imm));
        break;
    case 0x5:
        /*bseti16*/
        tcg_gen_ori_tl(cpu_R[rx], cpu_R[rx], 1 << imm);
        break;
    case 0x7:
        /* jmpix16 */
        check_insn(ctx, ABIV2_JAVA);
        if (ctx->bctm) {
            TCGv t0 = tcg_temp_new();
            TCGv t1 = tcg_temp_new();
            /*
            tcg_gen_addi_tl(t0, cpu_R[rx], 0xff);
            tcg_gen_and_tl(t0, t0, (16 + ((imm & 0x3) << 3)));
            tcg_gen_add_tl(t0, cpu_R[SVBR], t0);
            */
            tcg_gen_andi_tl(t0, cpu_R[rx], 0xff);
            switch (imm & 0x3) {
            case 0x0:
                tcg_gen_shli_tl(t0, t0, 4);
                break;
            case 0x1:
                tcg_gen_shli_tl(t1, t0, 4);
                tcg_gen_shli_tl(t0, t0, 3);
                tcg_gen_add_tl(t0, t0, t1);
                break;
            case 0x2:
                tcg_gen_shli_tl(t0, t0, 5);
                break;
            case 0x3:
                tcg_gen_shli_tl(t1, t0, 5);
                tcg_gen_shli_tl(t0, t0, 3);
                tcg_gen_add_tl(t0, t0, t1);
                break;
            default:
                break;
            }
            tcg_gen_add_tl(t0, cpu_R[SVBR], t0);
            store_cpu_field(t0, pc);

            ctx->base.is_jmp = DISAS_JUMP;
            break;
        }
        QEMU_FALLTHROUGH;
    case 0x6:
        if (has_insn(ctx, ABIV2_ELRW)) {
            /* btsti16 */
            tcg_gen_andi_tl(cpu_c, cpu_R[rx], 1 << imm);
            tcg_gen_shri_tl(cpu_c, cpu_c, imm);
            break;
        }
        QEMU_FALLTHROUGH;
    default:
        generate_exception(ctx, EXCP_CSKY_UDEF);
        break;
    }
}

static void gen_reg1_arith16(DisasContext *ctx, uint32_t op, int rz,
                             int rx, int imm)
{
    switch (op) {
    case 0x10:
    case 0x11:
        /*lsli16*/
        tcg_gen_shli_tl(cpu_R[rz], cpu_R[rx], imm);
        break;
    case 0x12:
    case 0x13:
        /*lsri16*/
        tcg_gen_shri_tl(cpu_R[rz], cpu_R[rx], imm);
        break;
    case 0x14:
    case 0x15:
        /*asri16*/
        tcg_gen_sari_tl(cpu_R[rz], cpu_R[rx], imm);
        break;
    default:
        generate_exception(ctx, EXCP_CSKY_UDEF);
        break;
    }
}

static void gen_reg3_arith16(DisasContext *ctx, uint32_t op, uint16_t sop,
                             int rz, int rx, int ry, int imm)
{
    switch (sop) {
    case 0x0:
        /*addu16(2)*/
        tcg_gen_add_tl(cpu_R[rz], cpu_R[rx], cpu_R[ry]);
        break;
    case 0x1:
        /*subu16(2)*/
        tcg_gen_sub_tl(cpu_R[rz], cpu_R[rx], cpu_R[ry]);
        break;
    case 0x2:
        /*addi16(2)*/
        tcg_gen_addi_tl(cpu_R[rz], cpu_R[rx], imm + 1);
        break;
    case 0x3:
        /*subi16(2)*/
        tcg_gen_subi_tl(cpu_R[rz], cpu_R[rx], imm + 1);
        break;
    default:
        generate_exception(ctx, EXCP_CSKY_UDEF);
        break;
    }
}

static void gen_reg2_arith16(DisasContext *ctx, uint32_t op, uint16_t sop,
                             int rx, int rz)
{
    switch (op) {
    case 0x18:
        switch (sop) {
        case 0x0:
            /*addu16(1)*/
            tcg_gen_add_tl(cpu_R[rz], cpu_R[rz], cpu_R[rx]);
            break;
        case 0x1:
            /*addc16*/
            addc(rz, rz, rx);
            break;
        case 0x2:
            /*subu16(1)*/
            tcg_gen_sub_tl(cpu_R[rz], cpu_R[rz], cpu_R[rx]);
            break;
        case 0x3:
            /*subc16*/
            subc(rz, rz, rx);
            break;
        default:
            generate_exception(ctx, EXCP_CSKY_UDEF);
            break;
        }
        break;
    case 0x19:
        gen_cmp16(ctx, sop, rz, rx);
        break;
    case 0x1a:
        gen_logic_and16(ctx, sop, rz, rx);
        break;
    case 0x1b:
        gen_logic_or16(ctx, sop, rz, rx);
        break;
    case 0x1c:
        gen_shift_reg16(ctx, sop, rz, rx);
        break;
    case 0x1d:
        gen_ext16(ctx, sop, rz, rx);
        break;
    case 0x1e:
        gen_arith_misc16(ctx, sop, rz, rx);
        break;
    case 0x1f:
        gen_mul16(ctx, sop, rz, rx);
        break;
    default:
        generate_exception(ctx, EXCP_CSKY_UDEF);
        break;
    }
}

static void gen_ldst16(DisasContext *ctx, uint32_t op, int rz, int rx, int imm)
{
    TCGv t0 = tcg_temp_new();

    switch (op) {
    case 0x20:
    case 0x21:
        /*ld.b16*/
        ldst(ld8u, rx, rz, imm, 2);
        break;
    case 0x22:
    case 0x23:
        /*ld.h16*/
        ldst(ld16u, rx, rz, imm << 1, 2);
        break;
    case 0x24:
    case 0x25:
        /*ld.w16*/
        ldst(ld32u, rx, rz, imm << 2, 2);
        break;
    case 0x28:
    case 0x29:
        /*st.b16*/
        ldst(st8, rx, rz, imm, 2);
        break;
    case 0x2a:
    case 0x2b:
        /*st.h16*/
        ldst(st16, rx, rz, imm << 1, 2);
        break;
    case 0x2c:
    case 0x2d:
        /*st.w16*/
        ldst(st32, rx, rz, imm << 2, 2);
        break;
    default:
        generate_exception(ctx, EXCP_CSKY_UDEF);
        break;
    }


}

static void disas_csky_16_insn(CPUCSKYState *env, DisasContext *ctx)
{
    unsigned int insn, rz, rx, ry;
    uint32_t op, sop;
    int imm;
    target_ulong addr;
    TCGv t0;

    insn = ctx->insn;

    op = (insn >> 10) & 0x3f;
    switch (op) {
    case 0x0 ... 0x3:
        imm = insn & 0x3ff;
        gen_branch16(ctx, op, imm);
        break;
    case 0x4:
        /*lrw16*/
        t0 = tcg_temp_new();
        TCGv_i32 t3 = csky_temp_const_i32(ctx->pc);
        imm = ((insn & 0x300) >> 3) | (insn & 0x1f);
        rz = (insn >> 5) & 0x7;
        addr = (ctx->pc + (imm << 2)) & 0xfffffffc ;
        tcg_gen_movi_tl(t0, addr);
        tcg_gen_qemu_ld_tl(cpu_R[rz], t0, ctx->mem_idx, MO_TEUL | ctx->align);
        if (gen_mem_trace()) {
            gen_helper_trace_ld32u(tcg_env, t3, cpu_R[rz], t0);
        }
        break;
    case 0x5:
        imm = ((insn >> 3) & 0x60) | (insn & 0x1f);
        sop = (insn >> 5) & 0x7;
        gen_imm7_arith16(ctx, op, sop, imm);
        break;
    case 0x6 ... 0xd:
        imm = insn & 0xff;
        rz = (insn >> 8) & 0x7;
        gen_imm8_arith16(ctx, op, rz, imm);
        break;
    case 0xe:
    case 0xf:
        imm = insn & 0x1f;
        sop = (insn >> 5) & 0x7;
        rx = (insn >> 8) & 0x7;
        gen_imm5_arith16(ctx, op, sop, rx, imm);
        break;
    case 0x10 ... 0x15:
        imm = insn & 0x1f;
        rz = (insn >> 5) & 0x7;
        rx = (insn >> 8) & 0x7;
        gen_reg1_arith16(ctx, op, rz, rx, imm);
        break;
    case 0x16:
    case 0x17:
        sop = insn & 0x3;
        imm = (insn >> 2) & 0x7;
        ry = (insn >> 2) & 0x7;
        rz = (insn >> 5) & 0x7;
        rx = (insn >> 8) & 0x7;
        gen_reg3_arith16(ctx, op, sop, rz, rx, ry, imm);
        break;
    case 0x18 ... 0x1f:
        sop = insn & 0x3;
        rx = (insn >> 2) & 0xf;
        rz = (insn >> 6) & 0xf;
        gen_reg2_arith16(ctx, op, sop, rx, rz);
        break;
    case 0x20 ... 0x25:
        imm = insn & 0x1f;
        rz = (insn >> 5) & 0x7;
        rx = (insn >> 8) & 0x7;
        gen_ldst16(ctx, op, rz, rx, imm);
        break;
    case 0x26:
    case 0x27:
        /*ld16.w(sp)*/
        t0 = tcg_temp_new();
        imm = ((insn & 0x700) >> 3) | (insn & 0x1f);
        rz = (insn >> 5) & 0x7;
        ldst(ld32u, sp, rz, imm << 2, 2);
        break;
    case 0x28 ... 0x2d:
        imm = insn & 0x1f;
        rz = (insn >> 5) & 0x7;
        rx = (insn >> 8) & 0x7;
        gen_ldst16(ctx, op, rz, rx, imm);
        break;
    case 0x2e:
    case 0x2f:
        /*st16.w(sp)*/
        t0 = tcg_temp_new();
        imm = ((insn & 0x700) >> 3) | (insn & 0x1f);
        rz = (insn >> 5) & 0x7;
        ldst(st32, sp, rz, imm << 2, 2);
        break;
    default:
        generate_exception(ctx, EXCP_CSKY_UDEF);
        break;
    }
}

static inline void bnezad(DisasContext *ctx, int rx, int offset)
{
    int val;
    TCGLabel *l1 = gen_new_label();

    val = offset << 1;
    if (val & 0x10000) {
        val |= 0xffff0000;
    }
    val += ctx->pc;

    tcg_gen_subi_tl(cpu_R[rx], cpu_R[rx], 1);

    tcg_gen_brcondi_tl(TCG_COND_GTU, cpu_R[rx], 0, l1);
    gen_goto_tb(ctx, 1, ctx->pc + 4);
    gen_set_label(l1);
    gen_goto_tb(ctx, 0, val);

    ctx->base.is_jmp = DISAS_TB_JUMP;
}

static inline void bnerai(DisasContext *ctx, int rx, int ry, int offset)
{
    int val;
    TCGLabel *l1 = gen_new_label();

    val = offset << 1;
    if (val & 0x10000) {
        val |= 0xffff0000;
    }
    val += ctx->pc;

    tcg_gen_addi_tl(cpu_R[rx], cpu_R[rx], 1);
    tcg_gen_brcond_tl(TCG_COND_NE, cpu_R[rx], cpu_R[ry], l1);
    gen_goto_tb(ctx, 1, ctx->pc + 4);
    gen_set_label(l1);
    gen_goto_tb(ctx, 0, val);

    ctx->base.is_jmp = DISAS_TB_JUMP;
}

static inline void branch32(DisasContext *ctx, TCGCond cond, int rx, int offset)
{
    int val;
    TCGLabel *l1 = gen_new_label();

    val = offset << 1;
    if (val & 0x10000) {
        val |= 0xffff0000;
    }
    val += ctx->pc;

    if (rx != -1) {
        tcg_gen_brcondi_tl(cond, cpu_R[rx], 0, l1);
    } else {
        tcg_gen_brcondi_tl(cond, cpu_c, 0, l1);
    }
    gen_goto_tb(ctx, 1, ctx->pc + 4);
    gen_set_label(l1);

    gen_goto_tb(ctx, 0, val);

    ctx->base.is_jmp = DISAS_TB_JUMP;


}

static inline void bsr32(DisasContext *ctx, int offset)
{
    int val;

    val = offset << 1;
    if (val & 0x4000000) {
        val |= 0xfc000000;
    }
    val += ctx->pc;

    gen_goto_tb(ctx, 0, val);

    ctx->base.is_jmp = DISAS_TB_JUMP;
}

static inline void sce(DisasContext *ctx, int cond)
{
    TCGv t0 = tcg_temp_new();
    TCGLabel *l1 = gen_new_label();

    tcg_gen_movi_tl(t0, cond);
    tcg_gen_brcondi_tl(TCG_COND_NE, cpu_c, 0, l1);
    tcg_gen_not_tl(t0, t0);
    tcg_gen_andi_tl(t0, t0, 0xf);
    gen_set_label(l1);
    tcg_gen_ori_tl(t0, t0, 0x10);
    store_cpu_field(t0, sce_condexec_bits);

    gen_save_pc(ctx->pc + 4);
    ctx->base.is_jmp = DISAS_UPDATE;
}

#ifndef CONFIG_USER_ONLY
static inline void gen_mfcr_cpu(DisasContext *ctx, uint32_t rz, uint32_t cr_num)
{
    TCGv t0;

    switch (cr_num) {
    case 0x0:
        /* cr0 psr */
        gen_helper_mfcr_cr0(cpu_R[rz], tcg_env);
        break;
    case 0x1:
        /* vbr */
        t0 = load_cpu_field(cp0.vbr);
        tcg_gen_mov_tl(cpu_R[rz], t0);

        break;
    case 0x2:
        /* epsr */
        t0 = load_cpu_field(cp0.epsr);
        tcg_gen_mov_tl(cpu_R[rz], t0);

        break;
    case 0x3:
        /* fpsr */
        t0 = load_cpu_field(cp0.fpsr);
        tcg_gen_mov_tl(cpu_R[rz], t0);

        break;
    case 0x4:
        /* epc */
        t0 = load_cpu_field(cp0.epc);
        tcg_gen_mov_tl(cpu_R[rz], t0);

        break;
    case 0x5:
        /* fpc */
        t0 = load_cpu_field(cp0.fpc);
        tcg_gen_mov_tl(cpu_R[rz], t0);

        break;
    case 0x6:
        /* ss0 */
        t0 = load_cpu_field(cp0.ss0);
        tcg_gen_mov_tl(cpu_R[rz], t0);

        break;
    case 0x7:
        /* ss1 */
        t0 = load_cpu_field(cp0.ss1);
        tcg_gen_mov_tl(cpu_R[rz], t0);

        break;
    case 0x8:
        /* ss2 */
        t0 = load_cpu_field(cp0.ss2);
        tcg_gen_mov_tl(cpu_R[rz], t0);

        break;
    case 0x9:
        /* ss3 */
        t0 = load_cpu_field(cp0.ss3);
        tcg_gen_mov_tl(cpu_R[rz], t0);

        break;
    case 0xa:
        /* ss4 */
        t0 = load_cpu_field(cp0.ss4);
        tcg_gen_mov_tl(cpu_R[rz], t0);

        break;
    case 0xb:
        /* gcr */
        t0 = load_cpu_field(cp0.gcr);
        tcg_gen_mov_tl(cpu_R[rz], t0);

        break;
    case 0xc:
        /* gsr */
        t0 = load_cpu_field(cp0.gsr);
        tcg_gen_mov_tl(cpu_R[rz], t0);

        break;
    case 0xd:
        /* cpidr */
        gen_helper_mfcr_cpidr(cpu_R[rz], tcg_env);
        break;
    case 0xe:
        /* dcsr */
        t0 = load_cpu_field(cp0.dcsr);
        tcg_gen_mov_tl(cpu_R[rz], t0);

        break;
    case 0xf:
        /* cpwr */
        t0 = load_cpu_field(cp0.cpwr);
        tcg_gen_mov_tl(cpu_R[rz], t0);

        break;
    case 0x10:
        /* no CR16 */
        break;
    case 0x11:
        /* cfr */
        t0 = load_cpu_field(cp0.cfr);
        tcg_gen_mov_tl(cpu_R[rz], t0);

        break;
    case 0x12:
        /* ccr */
        t0 = load_cpu_field(cp0.ccr);
        tcg_gen_mov_tl(cpu_R[rz], t0);

        break;
    case 0x13:
        /* capr */
        if (ctx->features & ABIV2_TEE) {
            gen_helper_tee_mfcr_cr19(cpu_R[rz], tcg_env);
        } else {
            t0 = load_cpu_field(cp0.capr);
            tcg_gen_mov_tl(cpu_R[rz], t0);
        }
        break;
    case 0x14:
        /* cr20 pacr */
        gen_helper_mfcr_cr20(cpu_R[rz], tcg_env);
        break;
    case 0x15:
        /* prsr */
        t0 = load_cpu_field(cp0.prsr);
        tcg_gen_mov_tl(cpu_R[rz], t0);

        break;
    case 0x1c:
        /* rvbr */
        gen_helper_mfcr_cr28(cpu_R[rz], tcg_env);
        break;
    case 0x1d:
        /* rmr */
        gen_helper_mfcr_cr29(cpu_R[rz], tcg_env);
        break;
    case 0x1e:
        gen_helper_mfcr_cr30(cpu_R[rz], tcg_env);
        break;
    case 0x1f:
        gen_helper_mfcr_cr31(cpu_R[rz], tcg_env);
        break;
    default:
        break;
    }
}

static inline void gen_mfcr_tee(DisasContext *ctx, uint32_t rz, uint32_t cr_num)
{
    TCGv t0;
    switch (cr_num) {
    case 0x0:
        /* NT_PSR */
        t0 = load_cpu_field(tee.nt_psr);
        tcg_gen_mov_tl(cpu_R[rz], t0);
        break;
    case 0x1:
        /* NT_VBR */
        t0 = load_cpu_field(tee.nt_vbr);
        tcg_gen_mov_tl(cpu_R[rz], t0);
        break;
    case 0x2:
        /* NT_EPSR */
        t0 = load_cpu_field(tee.nt_epsr);
        tcg_gen_mov_tl(cpu_R[rz], t0);
        break;
    case 0x4:
        /* NT_EPC */
        t0 = load_cpu_field(tee.nt_epc);
        tcg_gen_mov_tl(cpu_R[rz], t0);
        break;
    case 0x6:
        /* NT_SSP */
        t0 = load_cpu_field(stackpoint.nt_ssp);
        tcg_gen_mov_tl(cpu_R[rz], t0);
        break;
    case 0x7:
        /* T_USP */
        t0 = load_cpu_field(stackpoint.t_usp);
        tcg_gen_mov_tl(cpu_R[rz], t0);
        break;
    case 0x8:
        /* T_DCR */
        t0 = load_cpu_field(tee.t_dcr);
        tcg_gen_mov_tl(cpu_R[rz], t0);
        break;
    case 0x9:
        /* T_PCR */
        t0 = load_cpu_field(tee.t_pcr);
        tcg_gen_mov_tl(cpu_R[rz], t0);
        break;
    case 0xa:
        /* NT_EBR */
        t0 = load_cpu_field(tee.nt_ebr);
        tcg_gen_mov_tl(cpu_R[rz], t0);
        break;

    default:
        break;
    }
}

/* Read MMU Coprocessor Contronl Registers */
static inline void
gen_mfcr_mmu(DisasContext *ctx, uint32_t rz, uint32_t cr_num)
{
    TCGv t0;

    switch (cr_num) {
    case 0x0:
        /* MIR */
        t0 = load_cpu_field(mmu.mir);
        tcg_gen_mov_tl(cpu_R[rz], t0);
        break;
    case 0x1:
        /* MRR */
        t0 = load_cpu_field(mmu.mrr);
        tcg_gen_mov_tl(cpu_R[rz], t0);
        break;
    case 0x2:
        /* MEL0 */
        t0 = load_cpu_field(mmu.mel0);
        tcg_gen_mov_tl(cpu_R[rz], t0);
        break;
    case 0x3:
        /* MEL1 */
        t0 = load_cpu_field(mmu.mel1);
        tcg_gen_mov_tl(cpu_R[rz], t0);
        break;
    case 0x4:
        /* MEH */
        t0 = load_cpu_field(mmu.meh);
        tcg_gen_mov_tl(cpu_R[rz], t0);
        break;
    case 0x5:
        /* MCR */
        t0 = load_cpu_field(mmu.mcr);
        tcg_gen_mov_tl(cpu_R[rz], t0);
        break;
    case 0x6:
        /* MPR */
        t0 = load_cpu_field(mmu.mpr);
        tcg_gen_mov_tl(cpu_R[rz], t0);
        break;
    case 0x7:
        /* MWR */
        t0 = load_cpu_field(mmu.mwr);
        tcg_gen_mov_tl(cpu_R[rz], t0);
        break;
    case 0x8:
        /* MCIR */
        t0 = load_cpu_field(mmu.mcir);
        tcg_gen_mov_tl(cpu_R[rz], t0);
        break;
    case 0x9:
        /* CP15_CR9 */
        t0 = load_cpu_field(mmu.cr9);
        tcg_gen_mov_tl(cpu_R[rz], t0);
        break;
    case 0xa:
        /* CP15_CR10 */
        t0 = load_cpu_field(mmu.cr10);
        tcg_gen_mov_tl(cpu_R[rz], t0);
        break;
    case 0xb:
        /* CP15_CR11 */
        t0 = load_cpu_field(mmu.cr11);
        tcg_gen_mov_tl(cpu_R[rz], t0);
        break;
    case 0xc:
        /* CP15_CR12 */
        t0 = load_cpu_field(mmu.cr12);
        tcg_gen_mov_tl(cpu_R[rz], t0);
        break;
    case 0xd:
        /* CP15_CR13 */
        t0 = load_cpu_field(mmu.cr13);
        tcg_gen_mov_tl(cpu_R[rz], t0);
        break;
    case 0xe:
        /* CP15_CR14 */
        t0 = load_cpu_field(mmu.cr14);
        tcg_gen_mov_tl(cpu_R[rz], t0);
        break;
    case 0xf:
        /* CP15_CR15 */
        t0 = load_cpu_field(mmu.cr15);
        tcg_gen_mov_tl(cpu_R[rz], t0);
        break;
    case 0x10:
        /* CP15_CR16 */
        t0 = load_cpu_field(mmu.cr16);
        tcg_gen_mov_tl(cpu_R[rz], t0);
        break;
    case 0x1c:
        /* CP15_CR28 */
        t0 = load_cpu_field(mmu.mpgd0);
        tcg_gen_mov_tl(cpu_R[rz], t0);
        break;
    case 0x1d:
        t0 = load_cpu_field(mmu.mpgd1);
        tcg_gen_mov_tl(cpu_R[rz], t0);
        break;
    case 0x1e:
        t0 = load_cpu_field(mmu.msa0);
        tcg_gen_mov_tl(cpu_R[rz], t0);
        break;
    case 0x1f:
        t0 = load_cpu_field(mmu.msa1);
        tcg_gen_mov_tl(cpu_R[rz], t0);
        break;
    default:
        t0 = tcg_temp_new();
        break;
    }
}

static inline void gen_mtcr_cpu(DisasContext *ctx, uint32_t cr_num, uint32_t rx)
{

    switch (cr_num) {
    case 0x0:
        /* psr */
        gen_helper_mtcr_cr0(tcg_env, cpu_R[rx]);
        break;
    case 0x1:
        /* vbr */
        {
            TCGv t0 = tcg_temp_new();

            tcg_gen_andi_tl(t0, cpu_R[rx], ~0x3ff);
            store_cpu_field(t0, cp0.vbr);

        }
        break;
    case 0x2:
        /* epsr */
        store_cpu_field(cpu_R[rx], cp0.epsr);
        break;
    case 0x3:
        /* fpsr */
        store_cpu_field(cpu_R[rx], cp0.fpsr);
        break;
    case 0x4:
        /* epc */
        store_cpu_field(cpu_R[rx], cp0.epc);
        break;
    case 0x5:
        /* fpc */
        store_cpu_field(cpu_R[rx], cp0.fpc);
        break;
    case 0x6:
        /* ss0 */
        store_cpu_field(cpu_R[rx], cp0.ss0);
        break;
    case 0x7:
        /* ss1 */
        store_cpu_field(cpu_R[rx], cp0.ss1);
        break;
    case 0x8:
        /* ss2 */
        store_cpu_field(cpu_R[rx], cp0.ss2);
        break;
    case 0x9:
        /* ss3 */
        store_cpu_field(cpu_R[rx], cp0.ss3);
        break;
    case 0xa:
        /* ss4 */
        store_cpu_field(cpu_R[rx], cp0.ss4);
        break;
    case 0xb:
        /* gcr */
        store_cpu_field(cpu_R[rx], cp0.gcr);
        break;
    case 0xc:
        /* gsr */
        /* Read only */
        break;
    case 0xd:
        /* cpidr */
        /* Read only */
        break;
    case 0xe:
        /* dcsr */
        store_cpu_field(cpu_R[rx], cp0.dcsr);
        break;
    case 0xf:
        /* cpwr */
        /* FIXME */
        store_cpu_field(cpu_R[rx], cp0.cpwr);
        break;
    case 0x10:
        /* no CR16 */
        break;
    case 0x11:
        /* cfr */
        store_cpu_field(cpu_R[rx], cp0.cfr);
        break;
    case 0x12:
        /* ccr */
        gen_helper_mtcr_cr18(tcg_env, cpu_R[rx]);
        break;
    case 0x13:
        /* capr */
        if (ctx->features & ABIV2_TEE) {
            gen_helper_tee_mtcr_cr19(tcg_env, cpu_R[rx]);
        } else {
            store_cpu_field(cpu_R[rx], cp0.capr);
        }
        break;
    case 0x14:
        /* pacr */
        gen_helper_mtcr_cr20(tcg_env, cpu_R[rx]);
        break;
    case 0x15:
        /* prsr */
        store_cpu_field(cpu_R[rx], cp0.prsr);
        break;
    case 0x1c:
        /* rvbr */
        gen_helper_mtcr_cr28(tcg_env, cpu_R[rx]);
        break;
    case 0x1d:
        /* rmr */
        gen_helper_mtcr_cr29(tcg_env, cpu_R[rx]);
        break;
    case 0x1f:
        /* chr */
        gen_helper_mtcr_cr31(tcg_env, cpu_R[rx]);
    default:
        break;
    }
}

static inline void gen_mtcr_tee(DisasContext *ctx, uint32_t cr_num, uint32_t rx)
{
    TCGv t0 = tcg_temp_new();
    switch (cr_num) {
    case 0x0:
        /* NT_PSR */
        if (!(ctx->features & ABIV2_JAVA)) {
            tcg_gen_andi_tl(t0, cpu_R[rx], ~0x400);
            store_cpu_field(t0, tee.nt_psr);
        } else {
            store_cpu_field(cpu_R[rx], tee.nt_psr);
        }
        break;
    case 0x1:
        /* NT_VBR */
        tcg_gen_andi_tl(t0, cpu_R[rx], ~0x3ff);
        store_cpu_field(t0, tee.nt_vbr);
        break;
    case 0x2:
        /* NT_EPSR */
        store_cpu_field(cpu_R[rx], tee.nt_epsr);
        break;
    case 0x4:
        /* NT_EPC */
        store_cpu_field(cpu_R[rx], tee.nt_epc);
        break;
    case 0x6:
        /* NT_SSP */
        store_cpu_field(cpu_R[rx], stackpoint.nt_ssp);
        break;
    case 0x7:
        /* T_USP */
        store_cpu_field(cpu_R[rx], stackpoint.t_usp);
        break;
    case 0x8:
        /* T_DCR */
        tcg_gen_andi_tl(t0, cpu_R[rx], 0x3);
        store_cpu_field(t0, tee.t_dcr);
        break;
    case 0x9:
        /* T_PCR */
        tcg_gen_andi_tl(t0, cpu_R[rx], 0x1);
        store_cpu_field(t0, tee.t_pcr);
        break;
    case 0xa:
        /* NT_EBR */
        store_cpu_field(cpu_R[rx], tee.nt_ebr);
        break;

    default:
        break;
    }
}

static inline void gen_mtcr_mmu(DisasContext *ctx, uint32_t cr_num, uint32_t rx)
{
    switch (cr_num) {
    case 0x0:
        /* MIR */
        store_cpu_field(cpu_R[rx], mmu.mir);
        break;
    case 0x1:
        /* MRR */

        store_cpu_field(cpu_R[rx], mmu.mrr);
        break;
    case 0x2:
        /* MEL0 */
        store_cpu_field(cpu_R[rx], mmu.mel0);
        break;
    case 0x3:
        /* MEL1 */
        store_cpu_field(cpu_R[rx], mmu.mel1);
        break;
    case 0x4:
        /* MEH */
        gen_helper_meh_write(tcg_env, cpu_R[rx]);
        gen_save_pc(ctx->pc + 4);
        ctx->base.is_jmp = DISAS_UPDATE;
        break;
    case 0x5:
        /* MCR */
        store_cpu_field(cpu_R[rx], mmu.mcr);
        break;
    case 0x6:
        /* MPR */
        store_cpu_field(cpu_R[rx], mmu.mpr);
        break;
    case 0x7:
        /* MWR */
        store_cpu_field(cpu_R[rx], mmu.mwr);
        break;
    case 0x8:
        /* MCIR */
        gen_helper_mcir_write(tcg_env, cpu_R[rx]);
        gen_save_pc(ctx->pc + 4);
        ctx->base.is_jmp = DISAS_UPDATE;
        break;
    case 0x9:
        /* FIXME SPM is not implement yet */
        /* CP15_CR9 */
        store_cpu_field(cpu_R[rx], mmu.cr9);
        break;
    case 0xa:
        /* CP15_CR10 */
        store_cpu_field(cpu_R[rx], mmu.cr10);
        break;
    case 0xb:
        /* CP15_CR11 */
        store_cpu_field(cpu_R[rx], mmu.cr11);
        break;
    case 0xc:
        /* CP15_CR12 */
        store_cpu_field(cpu_R[rx], mmu.cr12);
        break;
    case 0xd:
        /* CP15_CR13 */
        store_cpu_field(cpu_R[rx], mmu.cr13);
        break;
    case 0xe:
        /* CP15_CR14 */
        store_cpu_field(cpu_R[rx], mmu.cr14);
        break;
    case 0xf:
        /* CP15_CR15 */
        store_cpu_field(cpu_R[rx], mmu.cr15);
        break;
    case 0x10:
        /* CP15_CR16 */
        store_cpu_field(cpu_R[rx], mmu.cr16);
        break;
    case 0x1c:
        /* CP15_CR28 */
        store_cpu_field(cpu_R[rx], mmu.mpgd0);
        gen_save_pc(ctx->pc + 4);
        ctx->base.is_jmp = DISAS_UPDATE;
        break;
    case 0x1d:
        store_cpu_field(cpu_R[rx], mmu.mpgd1);
        gen_save_pc(ctx->pc + 4);
        ctx->base.is_jmp = DISAS_UPDATE;
        break;
    case 0x1e:
        store_cpu_field(cpu_R[rx], mmu.msa0);
        break;
    case 0x1f:
        store_cpu_field(cpu_R[rx], mmu.msa1);
        break;
    default:
        break;
    }
}

static inline void gen_mtcr_mptimer(DisasContext *ctx,
    uint32_t rz, uint32_t rx)
{
    int pos = rz;
    TCGv t0;
    switch (pos) {
    case 0x0:
        /* PTIM_CTLR */
        store_cpu_field(cpu_R[rx], mptimer.ctlr);
        break;
    case 0x1:
        /* PTIM_TSR */
        store_cpu_field(cpu_R[rx], mptimer.isr);
        break;
    case 0x4:
        /* PTIM_CMVR_HI */
        store_cpu_field(cpu_R[rx], mptimer.cmvr_hi);
        break;
    case 0x5:
        /* PTIM_CMVR_LO */
        store_cpu_field(cpu_R[rx], mptimer.cmvr_lo);
        break;
    case 0x6:
        /* PTIM_CMVR_LVR */
        store_cpu_field(cpu_R[rx], mptimer.lvr);
        break;
    default:
        generate_exception(ctx, EXCP_CSKY_UDEF);
        qemu_log_mask(LOG_GUEST_ERROR,
                      "wrong mtcr control register: pc=%x rx=%d\n",
                      ctx->pc, rx);
        break;
    }

    t0 = csky_temp_const_i32(pos);
    gen_helper_mtcr_mptimer(tcg_env, t0, cpu_R[rx]);
}

static inline void gen_mfcr_mptimer(DisasContext *ctx, uint32_t rz, uint32_t rx)
{
    int pos = rx;
    TCGv t0, t1;
    switch (pos) {
    case 0x0: case 0x1: case 0x2: case 0x3:
        /* PTIM_CTLR/PTIM_ISR/PTIM_CCVR */
        t1 = tcg_temp_new();
        t0 = csky_temp_const_i32(pos);
        gen_helper_mfcr_mptimer(t1, tcg_env, t0);
        tcg_gen_mov_i32(cpu_R[rz], t1);
        break;
    case 0x1f:
        tcg_gen_movi_i32(cpu_R[rz], 0xfffe0000);
        break;
    default:
        generate_exception(ctx, EXCP_CSKY_UDEF);
        qemu_log_mask(LOG_GUEST_ERROR,
                      "wrong mfcr control register: pc=%x rx=%d\n",
                      ctx->pc, rx);
        break;
    }
}
#endif /* !CONFIG_USER_ONLY */

static inline void gen_mfcr_vfp(DisasContext *ctx,  int rz, int rx)
{
    TCGv t0;

    switch (rx) {
    case 0x0:
        /* fid*/
        t0 = load_cpu_field(vfp.fid);
        tcg_gen_mov_tl(cpu_R[rz], t0);
        break;
    case 0x1:
        /* fcr*/
        t0 = load_cpu_field(vfp.fcr);
        tcg_gen_mov_tl(cpu_R[rz], t0);
        break;
    case 0x2:
        /* fesr*/
        t0 = load_cpu_field(vfp.fesr);
        tcg_gen_mov_tl(cpu_R[rz], t0);
        break;
    default:
        generate_exception(ctx, EXCP_CSKY_UDEF);
        qemu_log_mask(LOG_GUEST_ERROR,
                      "wrong mfcr vfp control register: pc=%x rx=%d\n",
                      ctx->pc, rx);
        break;
    }
}

static inline void gen_mfcr_tp(DisasContext *ctx, int rz, int rx)
{
    switch (rz) {
    case 0x0:
        /*load_cpu_field(cpu_R[rx], cp14.tcr);*/
        break;
    default:
        break;
    }
}
static inline void gen_mtcr_tp(DisasContext *ctx, int rz, int rx)
{
    switch (rz) {
    case 0x0:
        store_cpu_field(cpu_R[rx], cp14.tcr);
        gen_helper_trace_update_tcr(tcg_env);
        gen_save_pc(ctx->pc + 4);
        ctx->base.is_jmp = DISAS_UPDATE;
        break;
    case 0x1:
        store_cpu_field(cpu_R[rx], cp14.ter);
        break;
    case 0x2:
        store_cpu_field(cpu_R[rx], cp14.addr_cmpr_config[0]);
        break;
    case 0x3:
        store_cpu_field(cpu_R[rx], cp14.addr_cmpr[0]);
        break;
    case 0x4:
        store_cpu_field(cpu_R[rx], cp14.addr_cmpr[1]);
        break;
    case 0x5:
        store_cpu_field(cpu_R[rx], cp14.data_cmpr_config[0]);
        break;
    case 0x6:
        store_cpu_field(cpu_R[rx], cp14.data_cmpr[0]);
        break;
    case 0x7:
        store_cpu_field(cpu_R[rx], cp14.data_cmpr[1]);
        break;
    case 0x8:
        store_cpu_field(cpu_R[rx], cp14.asid);
        break;
    default:
        break;
    }
}

static inline void gen_mtcr_vfp(DisasContext *ctx,  int rz, int rx)
{
    switch (rz) {
    case 0x1:
        /* store_cpu_field(cpu_R[rx], vfp.fcr); */
        store_cpu_field(cpu_R[rx], vfp.fcr);
        gen_helper_vfp_update_fcr(tcg_env);
        gen_save_pc(ctx->pc + 4);
        ctx->base.is_jmp = DISAS_UPDATE;
        break;
    case 0x2:
        store_cpu_field(cpu_R[rx], vfp.fesr);
        break;
    default:
        generate_exception(ctx, EXCP_CSKY_UDEF);
        qemu_log_mask(LOG_GUEST_ERROR,
                      "wrong mfcr vfp control register: pc=%x rz=%d\n",
                      ctx->pc, rz);
        break;
    }
}

static inline void add_ix(int rz, int rx, int ry, int imm)
{
    TCGv t0 = tcg_temp_new();

    tcg_gen_shli_tl(t0, cpu_R[ry], imm);
    tcg_gen_add_tl(cpu_R[rz], cpu_R[rx], t0);

}

static inline void lslc(int rz, int rx, int imm)
{
    TCGv t0 = tcg_temp_new();
    TCGv t1 = tcg_temp_new();
    TCGv t2 = tcg_temp_new();
    TCGLabel *l1 = gen_new_label();

    tcg_gen_movi_tl(t1, imm);
    tcg_gen_mov_tl(t0, cpu_R[rx]);
    tcg_gen_andi_tl(cpu_c, t0, 0x1);
    tcg_gen_movi_tl(cpu_R[rz], 0);
    tcg_gen_brcondi_tl(TCG_COND_EQ, t1, 32, l1);
    tcg_gen_shl_tl(cpu_R[rz], t0, t1);
    tcg_gen_rotl_tl(t2, t0, t1);
    tcg_gen_andi_tl(cpu_c, t2, 0x1);
    gen_set_label(l1);

}

static inline void lsrc(int rz, int rx, int imm)
{
    TCGv t0 = tcg_temp_new();
    TCGv t1 = tcg_temp_new();
    TCGv t2 = tcg_temp_new();
    TCGLabel *l1 = gen_new_label();

    tcg_gen_movi_tl(t1, imm);
    tcg_gen_mov_tl(t0, cpu_R[rx]);
    tcg_gen_andi_tl(cpu_c, t0, 0x80000000);
    tcg_gen_shri_tl(cpu_c, cpu_c, 31);
    tcg_gen_movi_tl(cpu_R[rz], 0);
    tcg_gen_brcondi_tl(TCG_COND_EQ, t1, 32, l1);
    tcg_gen_shr_tl(cpu_R[rz], t0, t1);
    tcg_gen_shri_tl(t2, t0, imm - 1);
    tcg_gen_andi_tl(cpu_c, t2, 0x1);
    gen_set_label(l1);

}

static inline void asrc(int rz, int rx, int imm)
{
    TCGv t0 = tcg_temp_new();
    TCGv t1 = tcg_temp_new();
    TCGv t2 = tcg_temp_new();
    TCGLabel *l1 = gen_new_label();

    tcg_gen_movi_tl(t1, imm);
    tcg_gen_mov_tl(t0, cpu_R[rx]);
    tcg_gen_sari_tl(cpu_R[rz], t0, 31);
    tcg_gen_andi_tl(cpu_c, cpu_R[rz], 0x1);
    tcg_gen_brcondi_tl(TCG_COND_EQ, t1, 32, l1);
    tcg_gen_sar_tl(cpu_R[rz], t0, t1);
    tcg_gen_shri_tl(t2, t0, imm - 1);
    tcg_gen_andi_tl(cpu_c, t2, 0x1);
    gen_set_label(l1);

}

static inline void divu(DisasContext *ctx, int rz, int rx, int ry)
{
    TCGLabel *l1 = gen_new_label();
    TCGLabel *l2 = gen_new_label();
    tcg_gen_brcondi_tl(TCG_COND_EQ, cpu_R[ry], 0, l1);
    tcg_gen_divu_tl(cpu_R[rz], cpu_R[rx], cpu_R[ry]);
    tcg_gen_br(l2);
    gen_set_label(l1);

    TCGv t0 = tcg_temp_new();

    tcg_gen_movi_tl(t0, EXCP_CSKY_DIV);
    gen_save_pc(ctx->pc);
    gen_helper_exception(tcg_env, t0);
    ctx->base.is_jmp = DISAS_NEXT;


    gen_set_label(l2);
}

static inline void divs(DisasContext *ctx, int rz, int rx, int ry)
{
    TCGLabel *l1 = gen_new_label();
    TCGLabel *l2 = gen_new_label();
    tcg_gen_brcondi_tl(TCG_COND_EQ, cpu_R[ry], 0, l1);
    TCGv_i64 t0 = tcg_temp_new_i64();
    TCGv_i64 t1 = tcg_temp_new_i64();
    tcg_gen_ext_i32_i64(t0, cpu_R[rx]);
    tcg_gen_ext_i32_i64(t1, cpu_R[ry]);
    tcg_gen_div_i64(t0, t0, t1);
    tcg_gen_extrl_i64_i32(cpu_R[rz], t0);
    tcg_gen_br(l2);
    gen_set_label(l1);

    TCGv t2 = tcg_temp_new();

    tcg_gen_movi_tl(t2, EXCP_CSKY_DIV);
    gen_save_pc(ctx->pc);
    gen_helper_exception(tcg_env, t2);
    ctx->base.is_jmp = DISAS_NEXT;


    gen_set_label(l2);
}

static inline int csky_log2(uint32_t s)
{
    int i = 0;
    if (s == 0) {
        return -1;
    }

    while (s != 1) {
        s >>= 1;
        i++;
    }
    return i;
}
#define ldbistbi(name, rx, rz, imm)                             \
do {                                                            \
    gen_csky_##name(ctx, cpu_R[rz], cpu_R[rx]);    \
    if (gen_mem_trace()) {                                      \
        TCGv_i32 t_pc;                     \
        t_pc = csky_temp_const_i32(ctx->pc);                          \
        gen_helper_trace_##name(tcg_env, t_pc, cpu_R[rz], cpu_R[rx]);  \
                                          \
    }                                                           \
    tcg_gen_addi_tl(cpu_R[rx], cpu_R[rx], imm);                 \
} while (0)

#define ldbirstbir(name, rx, rz, ry)                            \
do {                                                            \
    tcg_gen_mov_tl(t0, cpu_R[ry]);                              \
    gen_csky_##name(ctx, cpu_R[rz], cpu_R[rx]);    \
    if (gen_mem_trace()) {                                      \
        TCGv_i32 t_pc;                     \
        t_pc = csky_temp_const_i32(ctx->pc);                          \
        gen_helper_trace_##name(tcg_env, t_pc, cpu_R[rz], cpu_R[rx]);  \
                                          \
    }                                                           \
    tcg_gen_add_tl(cpu_R[rx], cpu_R[rx], t0);                   \
} while (0)

static inline void dspv2_insn_pldbi_d(DisasContext *s, int rz, int rx)
{
    /* Rz[31:0] <- mem(Rx)
     * Rz+1[31:0] <- mem(Rx + 4)
     * Rx[31:0] <- Rx[31:0] + 8 */
    TCGv_i32 t2 = csky_temp_const_i32(s->pc);
    tcg_gen_qemu_ld_tl(cpu_R[rz], cpu_R[rx], s->mem_idx, MO_TEUL | s->align);
    if (gen_mem_trace()) {
        gen_helper_trace_ld32u(tcg_env, t2, cpu_R[rz], cpu_R[rx]);
    }
    tcg_gen_addi_i32(cpu_R[rx], cpu_R[rx], 4);
    tcg_gen_qemu_ld_tl(cpu_R[(rz + 1) % 32], cpu_R[rx], s->mem_idx, MO_TEUL | s->align);
    if (gen_mem_trace()) {
        gen_helper_trace_ld32u(tcg_env, t2, cpu_R[(rz + 1) % 32], cpu_R[rx]);
    }
    tcg_gen_addi_i32(cpu_R[rx], cpu_R[rx], 4);
}

static inline void dspv2_insn_pldbir_d(DisasContext *s, int rz, int rx, int ry)
{
    /* Rz[31:0] <- mem(Rx)
     * Rz+1[31:0] <- mem(Rx + Ry)
     * Rx[31:0] <- Rx[31:0] + 2*Ry */
    TCGv_i32 t0 = tcg_temp_new_i32();
    TCGv_i32 t2 = csky_temp_const_i32(s->pc);
    tcg_gen_mov_i32(t0, cpu_R[ry]);
    tcg_gen_qemu_ld_tl(cpu_R[rz], cpu_R[rx], s->mem_idx, MO_TEUL | s->align);
    if (gen_mem_trace()) {
        gen_helper_trace_ld32u(tcg_env, t2, cpu_R[rz], cpu_R[rx]);
    }
    tcg_gen_add_i32(cpu_R[rx], cpu_R[rx], t0);
    tcg_gen_qemu_ld_tl(cpu_R[(rz + 1) % 32], cpu_R[rx], s->mem_idx, MO_TEUL | s->align);
    if (gen_mem_trace()) {
        gen_helper_trace_ld32u(tcg_env, t2, cpu_R[(rz + 1) % 32], cpu_R[rx]);
    }
    tcg_gen_add_i32(cpu_R[rx], cpu_R[rx], t0);
}

static void ldr(DisasContext *ctx, uint32_t sop,
                 uint32_t pcode, int rz, int rx, int ry)
{
    TCGv t0 = tcg_temp_new();
    int imm = 0;
    if (sop < 32) {
        /* basic ld instructions. */
        imm = csky_log2(pcode);
        if (imm == -1) {
            generate_exception(ctx, EXCP_CSKY_UDEF);
            return;
        }
    } else {
        check_insn(ctx, ABIV2_DSP2);
        if (pcode != 0) {
            generate_exception(ctx, EXCP_CSKY_UDEF);
            return;
        }
    }

    switch (sop) {
    case 0x0:
        /* ldr.b */
        check_insn_except(ctx, CPU_E801 | CPU_E802);
        ldrstr(ld8u, rx, ry, rz, imm);
        break;
    case 0x1:
        /* ldr.h */
        check_insn_except(ctx, CPU_E801 | CPU_E802);
        ldrstr(ld16u, rx, ry, rz, imm);
        break;
    case 0x2:
        /* ldr.w */
        check_insn_except(ctx, CPU_E801 | CPU_E802);
        ldrstr(ld32u, rx, ry, rz, imm);
        break;
    case 0x3:
        /* ldr.d */
        check_insn_except(ctx, CPU_E801 | CPU_E802);
        tcg_gen_shli_tl(t0, cpu_R[ry], imm);
        tcg_gen_add_tl(t0, cpu_R[rx], t0);
        tcg_gen_qemu_ld_tl(cpu_R[rz], t0, ctx->mem_idx, MO_TEUL | ctx->align);
        tcg_gen_addi_tl(t0, t0, 4);
        tcg_gen_qemu_ld_tl(cpu_R[rz + 1], t0, ctx->mem_idx, MO_TEUL | ctx->align);
        if (gen_mem_trace()) {
            TCGv t1, t2, t3;
            t1 = csky_temp_const_i32(2);
            t2 = csky_temp_const_i32(DATA_RADDR);
            t3 = csky_temp_const_i32(ctx->pc);
            tcg_gen_subi_tl(t0, t0, 4);
            gen_helper_trace_m_addr(tcg_env, t3, t0, t1, t2);
            gen_helper_trace_m_value(tcg_env, t3, t0, cpu_R[rz]);
            tcg_gen_addi_tl(t0, t0, 4);
            gen_helper_trace_m_value(tcg_env, t3, t0, cpu_R[rz + 1]);
        }
        break;
    case 0x4:
        /* ldr.bs */
        check_insn_except(ctx, CPU_E801 | CPU_E802);
        ldrstr(ld8s, rx, ry, rz, imm);
        break;
    case 0x5:
        /* ldr.hs */
        check_insn_except(ctx, CPU_E801 | CPU_E802);
        ldrstr(ld16s, rx, ry, rz, imm);
        break;
    case 0x7:
        /* ldm or ldq */
        check_insn_except(ctx, CPU_E801);

        TCGv t1 = csky_temp_const_i32(rz + 1);
        TCGv t2 = csky_temp_const_i32(DATA_RADDR);
        TCGv t3 = csky_temp_const_i32(ctx->pc);
        if (ctx->bctm) {
            TCGLabel *l1 = gen_new_label();
            tcg_gen_brcondi_tl(TCG_COND_NE, cpu_R[rx], 0, l1);
            tcg_gen_movi_tl(cpu_R[15], ctx->pc + 4);
            tcg_gen_subi_tl(t0, cpu_R[SVBR], 4);
            store_cpu_field(t0, pc);
            tcg_gen_exit_tb(NULL, 0);
            gen_set_label(l1);
            tcg_gen_mov_tl(t0, cpu_R[rx]);
            if (gen_mem_trace()) {
                gen_helper_trace_m_addr(tcg_env, t3, t0, t1, t2);
            }
            for (imm = 0; imm <= rz; imm++) {
                tcg_gen_qemu_ld_tl(cpu_R[ry + imm], t0, ctx->mem_idx, MO_TEUL | ctx->align);
                if (gen_mem_trace()) {
                    gen_helper_trace_m_value(tcg_env, t3, t0, cpu_R[ry + imm]);
                }
                tcg_gen_addi_tl(t0, t0, 4);
            }
            gen_goto_tb(ctx, 1, ctx->pc + 4);
            ctx->base.is_jmp = DISAS_TB_JUMP;
            break;
        } else {
            tcg_gen_mov_tl(t0, cpu_R[rx]);
            if (gen_mem_trace()) {
                gen_helper_trace_m_addr(tcg_env, t3, t0, t1, t2);
            }
            for (imm = 0; imm <= rz; imm++) {
                tcg_gen_qemu_ld_tl(cpu_R[ry + imm], t0, ctx->mem_idx, MO_TEUL | ctx->align);
                if (gen_mem_trace()) {
                    gen_helper_trace_m_value(tcg_env, t3, t0, cpu_R[ry + imm]);
                }
                tcg_gen_addi_tl(t0, t0, 4);
            }
            break;
        }
    case OP_LDBI_B:
        ldbistbi(ld8u, rx, rz, 1);
        break;
    case OP_LDBI_H:
        ldbistbi(ld16u, rx, rz, 2);
        break;
    case OP_LDBI_W:
        ldbistbi(ld32u, rx, rz, 4);
        break;
    case OP_PLDBI_D:
        dspv2_insn_pldbi_d(ctx, rz, rx);
        break;
    case OP_LDBI_BS:
        ldbistbi(ld8s, rx, rz, 1);
        break;
    case OP_LDBI_HS:
        ldbistbi(ld16s, rx, rz, 2);
        break;
    case OP_LDBIR_B:
        ldbirstbir(ld8u, rx, rz, ry);
        break;
    case OP_LDBIR_H:
        ldbirstbir(ld16u, rx, rz, ry);
        break;
    case OP_LDBIR_W:
        ldbirstbir(ld32u, rx, rz, ry);
        break;
    case OP_PLDBIR_D:
        dspv2_insn_pldbir_d(ctx, rz, rx, ry);
        break;
    case OP_LDBIR_BS:
        ldbirstbir(ld8s, rx, rz, ry);
        break;
    case OP_LDBIR_HS:
        ldbirstbir(ld16s, rx, rz, ry);
        break;
    default:
        generate_exception(ctx, EXCP_CSKY_UDEF);
        break;
    }
}

static void str(DisasContext *ctx, uint32_t sop,
                 uint32_t pcode, int rz, int rx, int ry)
{
    TCGv t0 = tcg_temp_new();
    int imm = 0;

    if (sop < 32) {
        imm = csky_log2(pcode);

        if (imm == -1) {
            generate_exception(ctx, EXCP_CSKY_UDEF);
            return;
        }
    } else {
        check_insn(ctx, ABIV2_DSP2);
        if (pcode != 0) {
            generate_exception(ctx, EXCP_CSKY_UDEF);
            return;
        }
    }

    switch (sop) {
    case 0x0:
        /* str.b */
        check_insn_except(ctx, CPU_E801 | CPU_E802);
        ldrstr(st8, rx, ry, rz, imm);
        break;
    case 0x1:
        /* str.h */
        check_insn_except(ctx, CPU_E801 | CPU_E802);
        ldrstr(st16, rx, ry, rz, imm);
        break;
    case 0x2:
        /* str.w */
        check_insn_except(ctx, CPU_E801 | CPU_E802);
        ldrstr(st32, rx, ry, rz, imm);
        break;
    case 0x3:
        /* str.d */
        check_insn_except(ctx, CPU_E801 | CPU_E802);
        tcg_gen_shli_tl(t0, cpu_R[ry], imm);
        tcg_gen_add_tl(t0, cpu_R[rx], t0);
        tcg_gen_qemu_st_tl(cpu_R[rz], t0, ctx->mem_idx, MO_TEUL | ctx->align);
        tcg_gen_addi_tl(t0, t0, 4);
        tcg_gen_qemu_st_tl(cpu_R[rz + 1], t0, ctx->mem_idx, MO_TEUL | ctx->align);
        if (gen_mem_trace()) {
            TCGv t1, t2, t3;
            t1 = csky_temp_const_i32(2);
            t2 = csky_temp_const_i32(DATA_WADDR);
            t3 = csky_temp_const_i32(ctx->pc);
            tcg_gen_subi_tl(t0, t0, 4);
            gen_helper_trace_m_addr(tcg_env, t3, t0, t1, t2);
            gen_helper_trace_m_value(tcg_env, t3, t0, cpu_R[rz]);
            tcg_gen_addi_tl(t0, t0, 4);
            gen_helper_trace_m_value(tcg_env, t3, t0, cpu_R[rz + 1]);
        }
        break;
    case 0x7:
        /* stm or stq */
        check_insn_except(ctx, CPU_E801);
        TCGv t1;
        TCGv t2 = csky_temp_const_i32(DATA_WADDR);
        TCGv t3 = csky_temp_const_i32(ctx->pc);
        if (ctx->bctm) {
            TCGLabel *l1 = gen_new_label();
            tcg_gen_brcondi_tl(TCG_COND_NE, cpu_R[rx], 0, l1);
            tcg_gen_movi_tl(cpu_R[15], ctx->pc + 4);
            tcg_gen_subi_tl(t0, cpu_R[SVBR], 4);
            store_cpu_field(t0, pc);
            tcg_gen_exit_tb(NULL, 0);
            gen_set_label(l1);
            tcg_gen_mov_tl(t0, cpu_R[rx]);
            if (gen_mem_trace()) {
                t1 = csky_temp_const_i32(rz + 1);
                gen_helper_trace_m_addr(tcg_env, t3, t0, t1, t2);
            }
            for (imm = 0; imm <= rz; imm++) {
                tcg_gen_qemu_st_tl(cpu_R[ry + imm], t0, ctx->mem_idx, MO_TEUL | ctx->align);
                if (gen_mem_trace()) {
                    gen_helper_trace_m_value(tcg_env, t3, t0, cpu_R[ry + imm]);
                }
                tcg_gen_addi_tl(t0, t0, 4);
            }
            gen_goto_tb(ctx, 1, ctx->pc + 4);
            ctx->base.is_jmp = DISAS_TB_JUMP;
        } else {
            tcg_gen_mov_tl(t0, cpu_R[rx]);
            if (gen_mem_trace()) {
                t1 = csky_temp_const_tl(rz + 1);
                gen_helper_trace_m_addr(tcg_env, t3, t0, t1, t2);
            }
            for (imm = 0; imm <= rz; imm++) {
                tcg_gen_qemu_st_tl(cpu_R[ry + imm], t0, ctx->mem_idx, MO_TEUL | ctx->align);
                if (gen_mem_trace()) {
                    gen_helper_trace_m_value(tcg_env, t3, t0, cpu_R[ry + imm]);
                }
                tcg_gen_addi_tl(t0, t0, 4);
            }
        }
        break;
    case OP_STBI_B:
        ldbistbi(st8, rx, rz, 1);
        break;
    case OP_STBI_H:
        ldbistbi(st16, rx, rz, 2);
        break;
    case OP_STBI_W:
        ldbistbi(st32, rx, rz, 4);
        break;
    case OP_STBIR_B:
        ldbirstbir(st8, rx, rz, ry);
        break;
    case OP_STBIR_H:
        ldbirstbir(st16, rx, rz, ry);
        break;
    case OP_STBIR_W:
        ldbirstbir(st32, rx, rz, ry);
        break;
    default:
        generate_exception(ctx, EXCP_CSKY_UDEF);
        break;
    }
}

static inline void pop(DisasContext *ctx, int imm)
{
    int i;
    TCGv t0 = tcg_temp_new();
    TCGv t1 = tcg_temp_new();
    TCGv t2 = csky_temp_const_i32(DATA_RADDR);
    TCGv t3 = csky_temp_const_i32(ctx->pc);

    tcg_gen_mov_tl(t0, cpu_R[sp]);
    tcg_gen_movi_tl(t1, 0);
    if (imm & 0x100) {
        tcg_gen_addi_tl(t1, t1, 1);
    }
    if ((imm >> 5) & 0x7) {
        tcg_gen_addi_tl(t1, t1, (imm >> 5) & 0x7);
    }
    if (imm & 0x10) {
        tcg_gen_addi_tl(t1, t1, 1);
    }
    if (imm & 0xf) {
        tcg_gen_addi_tl(t1, t1, imm & 0xf);
    }
    if (gen_mem_trace()) {
        gen_helper_trace_m_addr(tcg_env, t3, t0, t1, t2);
    }
    if (imm & 0xf) {
        for (i = 0; i < (imm & 0xf); i++) {
            tcg_gen_qemu_ld_tl(cpu_R[4 + i], t0, ctx->mem_idx, MO_TEUL | ctx->align);
            if (gen_mem_trace()) {
                gen_helper_trace_m_value(tcg_env, t3, t0, cpu_R[4 + i]);
            }
            tcg_gen_addi_i32(t0, t0, 4);
        }
    }

    if (imm & 0x10) {
        tcg_gen_qemu_ld_tl(cpu_R[15], t0, ctx->mem_idx, MO_TEUL | ctx->align);
        if (gen_mem_trace()) {
            gen_helper_trace_m_value(tcg_env, t3, t0, cpu_R[15]);
        }
        tcg_gen_addi_i32(t0, t0, 4);
    }

    if ((imm >> 5) & 0x7) {
        for (i = 0; i < ((imm >> 5) & 0x7); i++) {
            tcg_gen_qemu_ld_tl(cpu_R[16 + i], t0, ctx->mem_idx, MO_TEUL | ctx->align);
            if (gen_mem_trace()) {
                gen_helper_trace_m_value(tcg_env, t3, t0, cpu_R[16 + i]);
            }
            tcg_gen_addi_i32(t0, t0, 4);
        }
    }

    if (imm & 0x100) {
        tcg_gen_qemu_ld_tl(cpu_R[28], t0, ctx->mem_idx, MO_TEUL | ctx->align);
        if (gen_mem_trace()) {
            gen_helper_trace_m_value(tcg_env, t3, t0, cpu_R[28]);
        }
        tcg_gen_addi_i32(t0, t0, 4);
    }
    tcg_gen_mov_tl(cpu_R[sp], t0);

    tcg_gen_andi_tl(t0, cpu_R[15], 0xfffffffe);
    store_cpu_field(t0, pc);
    ctx->base.is_jmp = DISAS_JUMP;
}

static inline void ldex_w(DisasContext *ctx,
                          uint32_t rz, uint32_t rx, uint32_t imm)
{
    tcg_gen_addi_tl(excl_addr, cpu_R[rx], imm << 2);
    tcg_gen_qemu_ld_tl(excl_val, excl_addr, ctx->mem_idx, MO_TEUL | ctx->align);
    tcg_gen_mov_i32(cpu_R[rz], excl_val);
}

static void ldi(DisasContext *ctx, uint32_t sop,
                int rz, int rx, int imm)
{
    TCGv t0 = tcg_temp_new();

    switch (sop) {
    case 0x0:
        /* ld.b */
        ldst(ld8u, rx, rz, imm, 4);
        break;
    case 0x1:
        /* ld.h */
        ldst(ld16u, rx, rz, imm << 1, 4);
        break;
    case 0x2:
        /* ld.w */
        ldst(ld32u, rx, rz, imm << 2, 4);
        break;
    case 0x3:
        /* ld.d */
        check_insn(ctx, CPU_C810 | CPU_C807 | CPU_C860);
        tcg_gen_addi_tl(t0, cpu_R[rx], imm << 2);
        tcg_gen_qemu_ld_tl(cpu_R[rz], t0, ctx->mem_idx, MO_TEUL | ctx->align);
        tcg_gen_addi_tl(t0, t0, 4);
        tcg_gen_qemu_ld_tl(cpu_R[(rz + 1) % 32], t0, ctx->mem_idx, MO_TEUL | ctx->align);
        break;
    case 0x4:
        /* ld.bs */
        check_insn_except(ctx, CPU_E801);
        ldst(ld8s, rx, rz, imm, 4);
        break;
    case 0x5:
        /* ld.hs */
        check_insn_except(ctx, CPU_E801);
        ldst(ld16s, rx, rz, imm << 1, 4);
        break;
    case 0x6:
        /* pldr */    /*ignore this instruction*/
        check_insn(ctx, CPU_C807 | CPU_C810 | CPU_C860);
        break;
    case 0x7:
        /* ldex.w */
        check_insn(ctx, CPU_C860);
        ldex_w(ctx, rz, rx, imm);
        break;
    default:
        generate_exception(ctx, EXCP_CSKY_UDEF);
        break;
    }

}

static inline void push(DisasContext *ctx, int imm)
{
    TCGv t0 =  tcg_temp_new();
    int i;

    tcg_gen_mov_tl(t0, cpu_R[sp]);

    TCGv t1 = tcg_temp_new();
    tcg_gen_movi_tl(t1, 0);
    TCGv t2 = csky_temp_const_i32(DATA_WADDR);
    TCGv t3 = csky_temp_const_i32(ctx->pc);
    if (imm & 0x100) {
        tcg_gen_addi_tl(t1, t1, 1);
    }
    if ((imm >> 5) & 0x7) {
        tcg_gen_addi_tl(t1, t1, (imm >> 5) & 0x7);
    }
    if (imm & 0x10) {
        tcg_gen_addi_tl(t1, t1, 1);
    }
    if (imm & 0xf) {
        tcg_gen_addi_tl(t1, t1, imm & 0xf);
    }
    if (gen_mem_trace()) {
        gen_helper_trace_m_addr(tcg_env, t3, t0, t1, t2);
    }
    if (imm & 0x100) {
        tcg_gen_subi_i32(t0, t0, 4);
        tcg_gen_qemu_st_tl(cpu_R[28], t0, ctx->mem_idx, MO_TEUL | ctx->align);
        if (gen_mem_trace()) {
            gen_helper_trace_m_value(tcg_env, t3, t0, cpu_R[28]);
        }
    }

    if ((imm >> 5) & 0x7) {
        for (i = (imm >> 5) & 0x7; i > 0; i--) {
            tcg_gen_subi_i32(t0, t0, 4);
            tcg_gen_qemu_st_tl(cpu_R[15 + i], t0, ctx->mem_idx, MO_TEUL | ctx->align);
            if (gen_mem_trace()) {
                gen_helper_trace_m_value(tcg_env, t3, t0, cpu_R[15 + i]);
            }
        }
    }

    if (imm & 0x10) {
        tcg_gen_subi_i32(t0, t0, 4);
        tcg_gen_qemu_st_tl(cpu_R[15], t0, ctx->mem_idx, MO_TEUL | ctx->align);
        if (gen_mem_trace()) {
            gen_helper_trace_m_value(tcg_env, t3, t0, cpu_R[15]);
        }
    }

    if (imm & 0xf) {
        for (i = (imm & 0xf); i > 0; i--) {
            tcg_gen_subi_i32(t0, t0, 4);
            tcg_gen_qemu_st_tl(cpu_R[3 + i], t0, ctx->mem_idx, MO_TEUL | ctx->align);
            if (gen_mem_trace()) {
                gen_helper_trace_m_value(tcg_env, t3, t0, cpu_R[3 + i]);
            }
        }
    }
    tcg_gen_mov_tl(cpu_R[sp], t0);

}

static inline void stex_w(DisasContext *ctx,
                          uint32_t rz, uint32_t rx, uint32_t imm)
{
    TCGv_i32 t0, t1;
    TCGLabel *l3 = gen_new_label();
    TCGLabel *l4 = gen_new_label();

    MemOp opc = MO_32 | MO_LE | ctx->align;

    t0 = tcg_temp_new_i32();
    t1 = tcg_temp_new_i32();
    tcg_gen_addi_i32(t0, cpu_R[rx], imm << 2);
    tcg_gen_brcond_i32(TCG_COND_NE, t0, excl_addr, l3);

    tcg_gen_atomic_cmpxchg_i32(t1, excl_addr, excl_val, cpu_R[rz],
        ctx->mem_idx, opc);
    tcg_gen_setcond_i32(TCG_COND_EQ, t1, t1, excl_val);
    tcg_gen_mov_i32(cpu_R[rz], t1);
    tcg_gen_br(l4);

    gen_set_label(l3);
    tcg_gen_movi_i32(cpu_R[rz], 0);

    gen_set_label(l4);
    tcg_gen_movi_i32(excl_addr, -1);

}

static void sti(DisasContext *ctx, uint32_t sop, int rz, int rx, int imm)
{
    TCGv t0 = tcg_temp_new();

    switch (sop) {
    case 0x0:
        /* st.b */
        ldst(st8, rx, rz, imm, 4);
        break;
    case 0x1:
        /* st.h */
        ldst(st16, rx, rz, imm << 1, 4);
        break;
    case 0x2:
        /* st.w */
        ldst(st32, rx, rz, imm << 2, 4);
        break;
    case 0x3:
        /* st.d */
        check_insn(ctx, CPU_C810 | CPU_C807 | CPU_C860);
        tcg_gen_addi_tl(t0, cpu_R[rx], imm << 2);
        tcg_gen_qemu_st_tl(cpu_R[rz], t0, ctx->mem_idx, MO_TEUL | ctx->align);
        tcg_gen_addi_tl(t0, t0, 4);
        tcg_gen_qemu_st_tl(cpu_R[(rz + 1) % 32], t0, ctx->mem_idx, MO_TEUL | ctx->align);
        break;
    case 0x6:
        /* pldw */    /*ignore this instruction*/
        check_insn(ctx, CPU_C807 | CPU_C810 | CPU_C860);
        break;
    case 0x7:
        /* stex.w */
        check_insn(ctx, CPU_C860);
        stex_w(ctx, rz, rx, imm);
        break;
    default:
        generate_exception(ctx, EXCP_CSKY_UDEF);
        break;
    }
}

static inline void special(DisasContext *ctx, int rx, uint32_t sop,
                            int rz, int ry)
{
    /* ry:25-21, rx:20-16, sop:15-10, rz:4-0 */
    TCGv t0;

    switch (sop) {
    case 0x1:
        /* sync */
        break;
    case 0x4:
        /* bmset */
        check_insn(ctx, ABIV2_JAVA);

        t0 = csky_temp_const_tl(1);
        store_cpu_field(t0, psr_bm);
        ctx->base.is_jmp = DISAS_UPDATE;

        break;
    case 0x5:
        /* bmclr */
        check_insn(ctx, ABIV2_JAVA);

        t0 = csky_temp_const_tl(0);
        store_cpu_field(t0, psr_bm);
        ctx->base.is_jmp = DISAS_UPDATE;

        break;
    case 0x6:
        /* sce */
        check_insn(ctx, CPU_C810 | CPU_E803 | CPU_C807);
        sce(ctx, ry & 0xf);
        break;
    case 0x7:
        /* idly */
        check_insn_except(ctx, CPU_E801 | CPU_E802);
#if !defined(CONFIG_USER_ONLY)
        if (ctx->trace_mode == NORMAL_MODE) {
            TCGLabel *l1 = gen_new_label();

            t0 = load_cpu_field(idly4_counter);
            tcg_gen_brcondi_tl(TCG_COND_NE, t0, 0, l1);

            tcg_gen_movi_tl(t0, 4);
            store_cpu_field(t0, idly4_counter);
            tcg_gen_movi_tl(cpu_c, 0);

            gen_set_label(l1);
            ctx->base.is_jmp = DISAS_UPDATE;
            gen_save_pc(ctx->pc + 4);
        }
#endif
        break;
    case 0x8:
        /* trap 0 */
        generate_exception(ctx, EXCP_CSKY_TRAP0);
#if !defined(CONFIG_USER_ONLY)
        ctx->cannot_be_traced = 1;
#endif
        break;
    case 0x9:
        /* trap 1 */
#if !defined(CONFIG_USER_ONLY)
        generate_exception(ctx, EXCP_CSKY_TRAP1);
        ctx->cannot_be_traced = 1;
#endif
        break;
    case 0xa:
        /* trap 2 */
        generate_exception(ctx, EXCP_CSKY_TRAP2);
#if !defined(CONFIG_USER_ONLY)
        ctx->cannot_be_traced = 1;
#endif
        break;
    case 0xb:
        /* trap 3 */
        generate_exception(ctx, EXCP_CSKY_TRAP3);
#if !defined(CONFIG_USER_ONLY)
        ctx->cannot_be_traced = 1;
#endif
        break;
    case 0xf:
        /* wsc */
        check_insn(ctx, ABIV2_TEE);
#ifndef CONFIG_USER_ONLY
        t0 = tcg_temp_new();
        tcg_gen_movi_tl(t0, ctx->pc);
        store_cpu_field(t0, pc);
        tcg_gen_movi_tl(t0, 0);
        store_cpu_field(t0, idly4_counter);
        gen_helper_wsc(tcg_env);
        ctx->base.is_jmp = DISAS_UPDATE;
        ctx->cannot_be_traced = 1;
#else
        generate_exception(ctx, EXCP_CSKY_PRIVILEGE);
#endif
        break;
    case 0x15:
        /* we */
        break;
    case 0x16:
        /* se */
        break;
    case 0x10:
        /* rte */
#ifndef CONFIG_USER_ONLY
        if (!IS_SUPER(ctx)) {
            generate_exception(ctx, EXCP_CSKY_PRIVILEGE);
        } else {
            t0 = csky_temp_const_tl(0);
            store_cpu_field(t0, idly4_counter);

            gen_helper_rte(tcg_env);
            ctx->base.is_jmp = DISAS_UPDATE;
            ctx->cannot_be_traced = 1;
        }
#else
        generate_exception(ctx, EXCP_CSKY_PRIVILEGE);
#endif
        break;
    case 0x11:
        /* rfi */
        check_insn_except(ctx, CPU_E801 | CPU_E802);
#ifndef CONFIG_USER_ONLY
        if (!IS_SUPER(ctx)) {
            generate_exception(ctx, EXCP_CSKY_PRIVILEGE);
        } else {
            t0 = csky_temp_const_tl(0);
            store_cpu_field(t0, idly4_counter);

            gen_helper_rfi(tcg_env);
            ctx->base.is_jmp = DISAS_UPDATE;
            ctx->cannot_be_traced = 1;
        }
#else
        generate_exception(ctx, EXCP_CSKY_PRIVILEGE);
#endif
        break;
    case 0x12:
        /* stop */
#ifndef CONFIG_USER_ONLY
        if (!IS_SUPER(ctx)) {
            generate_exception(ctx, EXCP_CSKY_PRIVILEGE);
        } else {
            t0 = csky_temp_const_tl(0);
            store_cpu_field(t0, idly4_counter);

            gen_save_pc(ctx->pc + 4);
            gen_helper_stop(tcg_env);
            ctx->base.is_jmp = DISAS_UPDATE;
            ctx->cannot_be_traced = 1;
        }
#else
        generate_exception(ctx, EXCP_CSKY_PRIVILEGE);
#endif
        break;
    case 0x13:
        /* wait */
#ifndef CONFIG_USER_ONLY
        if (!IS_SUPER(ctx)) {
            generate_exception(ctx, EXCP_CSKY_PRIVILEGE);
        } else {
            t0 = csky_temp_const_tl(0);
            store_cpu_field(t0, idly4_counter);

            gen_save_pc(ctx->pc + 4);
            gen_helper_wait(tcg_env);
            ctx->base.is_jmp = DISAS_UPDATE;
            ctx->cannot_be_traced = 1;
        }
#else
        generate_exception(ctx, EXCP_CSKY_PRIVILEGE);
#endif
        break;
    case 0x14:
        /* doze */
#ifndef CONFIG_USER_ONLY
        if (!IS_SUPER(ctx)) {
            generate_exception(ctx, EXCP_CSKY_PRIVILEGE);
        } else {
            t0 = csky_temp_const_tl(0);
            store_cpu_field(t0, idly4_counter);

            gen_save_pc(ctx->pc + 4);
            gen_helper_doze(tcg_env);
            ctx->base.is_jmp = DISAS_UPDATE;
            ctx->cannot_be_traced = 1;
        }
#else
        generate_exception(ctx, EXCP_CSKY_PRIVILEGE);
#endif
        break;
    case 0x18:
        /* mfcr */
#ifndef CONFIG_USER_ONLY

        if (ry == 2) {
            check_insn(ctx, ABIV2_FLOAT_S);
            gen_mfcr_vfp(ctx, rz, rx);
            break;
        }
        if (!IS_SUPER(ctx)) {
            generate_exception(ctx, EXCP_CSKY_PRIVILEGE);
        }

        if (ry == 0) { /* sel */
            gen_mfcr_cpu(ctx, rz, rx);
        } else if (ry == 3) {  /* tee */
            check_insn(ctx, ABIV2_TEE);
            if (IS_TRUST(ctx)) {
                gen_mfcr_tee(ctx, rz, rx);
            } else {
                generate_exception(ctx, EXCP_CSKY_PRIVILEGE);
            }
        } else if (ry == 14) {
            gen_mfcr_mptimer(ctx, rz, rx);
        } else if (ry == 15) {
            check_insn(ctx, CSKY_MMU);
            gen_mfcr_mmu(ctx, rz, rx);
        } else if ((ry == 1) && (rx == 14)) {
            gen_helper_mfcr_cr14(cpu_R[rz], tcg_env);
        } else if ((ry == 1) && (rx == 15)) {
            gen_helper_mfcr_cr15(cpu_R[rz], tcg_env);
        } else if ((ry == 1) && (rx == 1)) {  /* mfcr cr<1, 1> */
            check_insn(ctx, ABIV2_TEE);
            TCGv t_ebr;
            if (IS_TRUST(ctx)) {
                t_ebr = load_cpu_field(tee.t_ebr);
            } else {
                t_ebr = load_cpu_field(tee.nt_ebr);
            }
            tcg_gen_mov_tl(cpu_R[rz], t_ebr);
        }
        break;
#else
        if (ry == 2) {
            check_insn(ctx, ABIV2_FLOAT_S);
            gen_mfcr_vfp(ctx, rz, rx);
        } else {
            generate_exception(ctx, EXCP_CSKY_PRIVILEGE);
        }
        break;
#endif
    case 0x19:
        /* mtcr */
#ifndef CONFIG_USER_ONLY
        if (ry == 2) {
            check_insn(ctx, ABIV2_FLOAT_S);
            gen_mtcr_vfp(ctx, rz, rx);
        } else if (!IS_SUPER(ctx)) {
            generate_exception(ctx, EXCP_CSKY_PRIVILEGE);
        } else {
            if (ry == 0) {  /* sel */
                gen_mtcr_cpu(ctx, rz, rx);
            } else if (ry == 3) {  /* tee */
                check_insn(ctx, ABIV2_TEE);
                if (IS_TRUST(ctx)) {
                    gen_mtcr_tee(ctx, rz, rx);
                } else {
                    generate_exception(ctx, EXCP_CSKY_PRIVILEGE);
                }
            } else if (ry == 14) { /* mptimer */
                gen_mtcr_mptimer(ctx, rz, rx);
            } else if (ry == 15) {
                check_insn(ctx, CSKY_MMU);
                gen_mtcr_mmu(ctx, rz, rx);
            } else if ((ry == 1) && (rz == 14)) { /* mtcr cr<14, 1> */
                gen_helper_mtcr_cr14(tcg_env, cpu_R[rx]);
            } else if ((ry == 1) && (rz == 15)) { /* mtcr cr<14, 1> */
                gen_helper_mtcr_cr15(tcg_env, cpu_R[rx]);
            } else if ((ry == 1) && (rz == 1)) {  /* mtcr cr<1, 1> */
                check_insn(ctx, ABIV2_TEE);
                TCGv t_ebr = tcg_temp_new();
                tcg_gen_andi_tl(t_ebr, cpu_R[rx], ~0x3);
                if (IS_TRUST(ctx)) {
                    store_cpu_field(t_ebr, tee.t_ebr);
                } else {
                    store_cpu_field(t_ebr, tee.nt_ebr);
                }
            }
        }
#else
        if (ry == 2) {
            check_insn(ctx, ABIV2_FLOAT_S);
            gen_mtcr_vfp(ctx, rz, rx);
        } else if (ry == 14) {
            gen_mtcr_tp(ctx, rz, rx);
        } else {
            generate_exception(ctx, EXCP_CSKY_PRIVILEGE);
        }
#endif
        if (ctx->base.is_jmp == DISAS_NEXT) {
            gen_save_pc(ctx->pc + 4);
            ctx->base.is_jmp = DISAS_UPDATE;
        }
        break;
    case 0x1c:
        /* psrclr */
#ifndef CONFIG_USER_ONLY
        if (!IS_SUPER(ctx)) {
            generate_exception(ctx, EXCP_CSKY_PRIVILEGE);
        } else {
            t0 = csky_temp_const_tl(ry);
            gen_helper_psrclr(tcg_env, t0);

            gen_save_pc(ctx->pc + 4);
            ctx->base.is_jmp = DISAS_UPDATE;
        }
#else
        generate_exception(ctx, EXCP_CSKY_PRIVILEGE);
#endif
        break;
    case 0x1d:
        /* psrset */
#ifndef CONFIG_USER_ONLY
        if (!IS_SUPER(ctx)) {
            generate_exception(ctx, EXCP_CSKY_PRIVILEGE);
        } else {
            t0 = csky_temp_const_tl(ry);
            gen_helper_psrset(tcg_env, t0);

            gen_save_pc(ctx->pc + 4);
            ctx->base.is_jmp = DISAS_UPDATE;
        }
#else
        generate_exception(ctx, EXCP_CSKY_PRIVILEGE);
#endif
        break;
    case 0x21:
        /* bar */
        check_insn(ctx, CPU_C860);
        /* ignore this instruction. */
        break;
    case 0x22:
        /* ck860 tlbi instructions */
#ifndef CONFIG_USER_ONLY
        check_insn(ctx, CPU_C860);
        t0 = csky_temp_const_tl(ry);

        gen_helper_mp_tlb_inv(tcg_env, cpu_R[rx], t0);

        gen_save_pc(ctx->pc + 4);
        ctx->base.is_jmp = DISAS_UPDATE;
#else
        generate_exception(ctx, EXCP_CSKY_PRIVILEGE);
#endif
        break;
    case 0x24:
        /* ck860 icache instructions: icache.iall(s)/icache iva */
        check_insn(ctx, CPU_C860);
        break;
    case 0x25:
        /* ck860 dcache instructions */
        check_insn(ctx, CPU_C860);
        /* ignore those instructions. */
        break;
    case 0x26:
        /* ck860 l2cache instructions */
        check_insn(ctx, CPU_C860);
        /* ignore those instructions. */
        break;
    default:
        generate_exception(ctx, EXCP_CSKY_UDEF);
        break;
    }
}

static inline void arth_reg32(DisasContext *ctx,  int ry, int rx, uint32_t sop,
                              uint32_t pcode, int rz)
{
    TCGv t0 = tcg_temp_new();
    TCGv t1 = tcg_temp_new();
    int lsb;
    int msb;

    switch (sop) {
    case 0x0:
        if (pcode == 0x1) {
            /* addu */
            check_insn_except(ctx, CPU_E801);
            tcg_gen_add_tl(cpu_R[rz], cpu_R[rx], cpu_R[ry]);
        } else if (pcode == 0x2) {
            /* addc */
            check_insn_except(ctx, CPU_E801);
            addc(rz, rx, ry);
        } else if (pcode == 0x4) {
            /* subu or rsub */
            check_insn_except(ctx, CPU_E801);
            tcg_gen_sub_tl(cpu_R[rz], cpu_R[rx], cpu_R[ry]);
        } else if (pcode == 0x8) {
            /* subc */
            check_insn_except(ctx, CPU_E801);
            subc(rz, rx, ry);
        } else if (pcode == 0x10) {
            /* abs */
            check_insn_except(ctx, CPU_E801 | CPU_E802);
            TCGLabel *l1 = gen_new_label();
            tcg_gen_mov_tl(cpu_R[rz], cpu_R[rx]);
            tcg_gen_brcondi_tl(TCG_COND_EQ, cpu_R[rx], 0x80000000, l1);
            tcg_gen_brcondi_tl(TCG_COND_GE, cpu_R[rx], 0, l1);
            tcg_gen_neg_tl(cpu_R[rz], cpu_R[rx]);
            gen_set_label(l1);
        } else {
            goto illegal_op;
        }
        break;
    case 0x1:
        if (pcode == 0x1) {
            /* cmphs */
            check_insn_except(ctx, CPU_E801 | CPU_E802);
            tcg_gen_setcond_tl(TCG_COND_GEU, cpu_c, cpu_R[rx], cpu_R[ry]);
        } else if (pcode == 0x2) {
            /* cmplt */
            check_insn_except(ctx, CPU_E801 | CPU_E802);
            tcg_gen_setcond_tl(TCG_COND_LT, cpu_c, cpu_R[rx], cpu_R[ry]);
        } else if (pcode == 0x4) {
            /* cmpne */
            check_insn_except(ctx, CPU_E801 | CPU_E802);
            tcg_gen_setcond_tl(TCG_COND_NE, cpu_c, cpu_R[rx], cpu_R[ry]);
        } else if (pcode == 0x8) {
            /* mvc */
            check_insn_except(ctx, CPU_E801);
            tcg_gen_mov_tl(cpu_R[rz], cpu_c);
        } else if (pcode == 0x10) {
            /* mvcv */
            check_insn_except(ctx, CPU_E801 | CPU_E802);
            tcg_gen_subfi_tl(cpu_R[rz], 1, cpu_c);
        } else {
            goto illegal_op;
        }
        break;
    case 0x2:
        if (pcode == 0x1) {
            /* ixh */
            check_insn_except(ctx, CPU_E801);
            add_ix(rz, rx, ry, 1);
        } else if (pcode == 0x2) {
            /* ixw */
            check_insn_except(ctx, CPU_E801);
            add_ix(rz, rx, ry, 2);
        } else if (pcode == 0x4) {
            /* ixd */
            check_insn_except(ctx, CPU_E801 | CPU_E802);
            add_ix(rz, rx, ry, 3);
        } else {
            goto illegal_op;
        }
        break;
    case 0x3:
        check_insn_except(ctx, CPU_E801);
        if (pcode == 0x1) {
            /* incf */
            TCGLabel *l1 = gen_new_label();
            tcg_gen_brcondi_tl(TCG_COND_NE, cpu_c, 0, l1);
            tcg_gen_addi_tl(cpu_R[ry], cpu_R[rx], rz);
            gen_set_label(l1);
        } else if (pcode == 0x2) {
            /* inct */
            TCGLabel *l1 = gen_new_label();
            tcg_gen_brcondi_tl(TCG_COND_EQ, cpu_c, 0, l1);
            tcg_gen_addi_tl(cpu_R[ry], cpu_R[rx], rz);
            gen_set_label(l1);
        } else if (pcode == 0x4) {
            /* decf */
            TCGLabel *l1 = gen_new_label();
            tcg_gen_brcondi_tl(TCG_COND_NE, cpu_c, 0, l1);
            tcg_gen_subi_tl(cpu_R[ry], cpu_R[rx], rz);
            gen_set_label(l1);
        } else if (pcode == 0x8) {
            /* dect */
            TCGLabel *l1 = gen_new_label();
            tcg_gen_brcondi_tl(TCG_COND_EQ, cpu_c, 0, l1);
            tcg_gen_subi_tl(cpu_R[ry], cpu_R[rx], rz);
            gen_set_label(l1);
        } else {
            goto illegal_op;
        }
        break;
    case 0x4:
        if (pcode == 0x1) {
            /* decgt */
            check_insn_except(ctx, CPU_E801 | CPU_E802);
            tcg_gen_subi_tl(cpu_R[rz], cpu_R[rx], ry);
            tcg_gen_setcondi_tl(TCG_COND_GT, cpu_c, cpu_R[rz], 0);
        } else if (pcode == 0x2) {
            /* declt */
            check_insn_except(ctx, CPU_E801 | CPU_E802);
            tcg_gen_subi_tl(cpu_R[rz], cpu_R[rx], ry);
            tcg_gen_setcondi_tl(TCG_COND_LT, cpu_c, cpu_R[rz], 0);
        } else if (pcode == 0x4) {
            /* decne */
            check_insn_except(ctx, CPU_E801 | CPU_E802);
            tcg_gen_subi_tl(cpu_R[rz], cpu_R[rx], ry);
            tcg_gen_setcondi_tl(TCG_COND_NE, cpu_c, cpu_R[rz], 0);
        } else {
            goto illegal_op;
        }
        break;
    case 0x7:
        if (pcode == 1) {
            /* cmpix */
            check_insn(ctx, ABIV2_JAVA);
            if (ctx->bctm) {
                TCGLabel *l1 = gen_new_label();
                tcg_gen_brcond_tl(TCG_COND_LT, cpu_R[rx], cpu_R[ry], l1);
                tcg_gen_movi_tl(cpu_R[15], ctx->pc + 4);
                tcg_gen_subi_tl(t0, cpu_R[SVBR], 8);
                store_cpu_field(t0, pc);
                ctx->base.is_jmp = DISAS_JUMP;
                gen_set_label(l1);
            } else {
                break;
            }
        } else {
            goto illegal_op;
        }
        break;
    case 0x8:
        if (pcode == 0x1) {
            /* and */
            check_insn_except(ctx, CPU_E801);
            tcg_gen_and_tl(cpu_R[rz], cpu_R[rx], cpu_R[ry]);
        } else if (pcode == 0x2) {
            /* andn */
            check_insn_except(ctx, CPU_E801);
            tcg_gen_andc_tl(cpu_R[rz], cpu_R[rx], cpu_R[ry]);
        } else if (pcode == 0x4) {
            /* tst */
            check_insn_except(ctx, CPU_E801 | CPU_E802);
            tcg_gen_and_tl(t0, cpu_R[rx], cpu_R[ry]);
            tcg_gen_setcondi_tl(TCG_COND_NE, cpu_c, t0, 0);
        } else if (pcode == 0x8) {
            /* tstnbz */
            check_insn_except(ctx, CPU_E801 | CPU_E802);
            tstnbz(rx);
        } else {
            goto illegal_op;
        }
        break;
    case 0x9:
        check_insn_except(ctx, CPU_E801);
        if (pcode == 0x1) {
            /* or */
            tcg_gen_or_tl(cpu_R[rz], cpu_R[rx], cpu_R[ry]);
        } else if (pcode == 0x2) {
            /* xor */
            tcg_gen_xor_tl(cpu_R[rz], cpu_R[rx], cpu_R[ry]);
        } else if (pcode == 0x4) {
            /* nor */
            tcg_gen_nor_tl(cpu_R[rz], cpu_R[rx], cpu_R[ry]);
        } else {
            goto illegal_op;
        }
        break;
    case 0xa:
        check_insn_except(ctx, CPU_E801);
        if (pcode == 0x1) {
            /* bclri */
            tcg_gen_andi_tl(cpu_R[rz], cpu_R[rx], ~(1 << ry));
        } else if (pcode == 0x2) {
            /* bseti */
            tcg_gen_ori_tl(cpu_R[rz], cpu_R[rx], 1 << ry);
        } else if (pcode == 0x4) {
            /* btsti */
            tcg_gen_andi_tl(cpu_c, cpu_R[rx], 1 << ry);
            tcg_gen_shri_tl(cpu_c, cpu_c, ry);
        } else {
            goto illegal_op;
        }
        break;
    case 0xb:
        if (pcode == 0x1) {
            /* clrf */
            check_insn_except(ctx, CPU_E801 | CPU_E802);
            TCGLabel *l1 = gen_new_label();
            tcg_gen_brcondi_tl(TCG_COND_NE, cpu_c, 0, l1);
            tcg_gen_movi_tl(cpu_R[ry], 0);
            gen_set_label(l1);
        } else if (pcode == 0x2) {
            /* clrt */
            check_insn_except(ctx, CPU_E801 | CPU_E802);
            TCGLabel *l1 = gen_new_label();
            tcg_gen_brcondi_tl(TCG_COND_EQ, cpu_c, 0, l1);
            tcg_gen_movi_tl(cpu_R[ry], 0);
            gen_set_label(l1);
        } else {
            goto illegal_op;
        }
        break;
    case 0x10:
        check_insn_except(ctx, CPU_E801);
        if (pcode == 0x1) {
            /* lsl */
            lsl(rz, rx, ry);
        } else if (pcode == 0x2) {
            /* lsr */
            lsr(rz, rx, ry);
        } else if (pcode == 0x4) {
            /* asr */
            asr(rz, rx, ry);
        } else if (pcode == 0x8) {
            /* rotl */
            rotl(rz, rx, ry);
        } else {
            goto illegal_op;
        }
        break;
    case 0x12:
        check_insn_except(ctx, CPU_E801);
        if (pcode == 0x1) {
            /* lsli */
            tcg_gen_shli_tl(cpu_R[rz], cpu_R[rx], ry);
        } else if (pcode == 0x2) {
            /* lsri */
            tcg_gen_shri_tl(cpu_R[rz], cpu_R[rx], ry);
        } else if (pcode == 0x4) {
            /* asri */
            tcg_gen_sari_tl(cpu_R[rz], cpu_R[rx], ry);
        } else if (pcode == 0x8) {
            /* rotli */
            tcg_gen_rotli_tl(cpu_R[rz], cpu_R[rx], ry);
        } else {
            goto illegal_op;
        }
        break;
    case 0x13:
        check_insn_except(ctx, CPU_E801);
        if (pcode == 0x1) {
            /* lslc */
            lslc(rz, rx, ry + 1);
        } else if (pcode == 0x2) {
            /* lsrc */
            lsrc(rz, rx, ry + 1);
        } else if (pcode == 0x4) {
            /* asrc */
            asrc(rz, rx, ry + 1);
        } else if (pcode == 0x8) {
            /* xsr */
            t0 = csky_temp_const_tl(ry + 1);
            gen_helper_xsr(cpu_R[rz], tcg_env, cpu_R[rx], t0);
        } else {
            goto illegal_op;
        }
        break;
    case 0x14:
        if (pcode == 0x1) {
            /* bmaski */
            check_insn_except(ctx, CPU_E801);
            ry += 1;
            if (ry == 32) {
                tcg_gen_movi_tl(cpu_R[rz], 0xffffffff);
            } else {
                tcg_gen_movi_tl(cpu_R[rz], (1 << ry) - 1);
            }
        } else if (pcode == 0x2) {
            /* bgenr */
            check_insn_except(ctx, CPU_E801 | CPU_E802);
            TCGv t2 = tcg_temp_new();
            TCGLabel *l1 = gen_new_label();

            tcg_gen_mov_tl(t2, cpu_R[rx]);
            tcg_gen_movi_tl(cpu_R[rz], 0);
            tcg_gen_andi_tl(t1, t2, 0x20);
            tcg_gen_brcondi_tl(TCG_COND_NE, t1, 0, l1);
            tcg_gen_movi_tl(t1, 1);
            tcg_gen_andi_tl(t2, t2, 0x1f);
            tcg_gen_shl_tl(cpu_R[rz], t1, t2);
            gen_set_label(l1);

        } else {
            goto illegal_op;
        }
        break;
    case 0x15:
        /* zext or zextb or zexth */
        check_insn_except(ctx, CPU_E801 | CPU_E802);
        lsb = ry;
        msb = pcode;
        if (lsb == 0 && msb == 31) {
            tcg_gen_mov_tl(cpu_R[rz], cpu_R[rx]);
        } else {
            tcg_gen_movi_tl(t0, 0);
            tcg_gen_shri_tl(cpu_R[rz], cpu_R[rx], lsb);
            tcg_gen_deposit_tl(cpu_R[rz], t0, cpu_R[rz], 0, msb - lsb + 1);
        }
        break;
    case 0x16:
        /* sext or sextb or sexth */
        check_insn_except(ctx, CPU_E801 | CPU_E802);
        lsb = ry;
        msb = pcode;
        if (lsb == 0 && msb == 31) {
            tcg_gen_mov_tl(cpu_R[rz], cpu_R[rx]);
        } else {
            tcg_gen_shri_tl(cpu_R[rz], cpu_R[rx], lsb);
            tcg_gen_movi_tl(t0, 0);
            tcg_gen_deposit_tl(t0, t0, cpu_R[rz], 0, msb - lsb + 1);
            tcg_gen_shli_tl(t0, t0, 32 - (msb - lsb + 1));
            tcg_gen_sari_tl(cpu_R[rz], t0, 32 - (msb - lsb + 1));
        }
        break;
    case 0x17:
        /* ins */
        check_insn_except(ctx, CPU_E801 | CPU_E802);
        lsb = rz;
        if (pcode == 31) {
            tcg_gen_mov_tl(cpu_R[ry], cpu_R[rx]);
        } else {
            tcg_gen_deposit_tl(cpu_R[ry], cpu_R[ry], cpu_R[rx], lsb, pcode + 1);
        }
        break;
    case 0x18:
        if (pcode == 0x4) {
            /* revb */
            check_insn_except(ctx, CPU_E801 | CPU_E802);
            tcg_gen_bswap32_i32(cpu_R[rz], cpu_R[rx]);
        } else if (pcode == 0x8) {
            /* revh */
            check_insn_except(ctx, CPU_E801 | CPU_E802);
            tcg_gen_bswap32_i32(t0, cpu_R[rx]);
            tcg_gen_shri_tl(t1, t0, 16);
            tcg_gen_shli_tl(t0, t0, 16);
            tcg_gen_or_tl(cpu_R[rz], t0, t1);
        } else if (pcode == 0x10) {
            /* brev */
            check_insn_except(ctx, CPU_E801 | CPU_E802);
            gen_helper_brev(cpu_R[rz], cpu_R[rx]);
        } else {
            goto illegal_op;
        }
        break;
    case 0x1c:
        check_insn_except(ctx, CPU_E801);
        if (pcode == 0x1) {
            /* xtrb0 */
            tcg_gen_shri_tl(cpu_R[rz], cpu_R[rx], 24);
            tcg_gen_setcondi_tl(TCG_COND_NE, cpu_c, cpu_R[rz], 0);
        } else if (pcode == 0x2) {
            /* xtrb1 */
            tcg_gen_andi_tl(cpu_R[rz], cpu_R[rx], 0x00ff0000);
            tcg_gen_shri_tl(cpu_R[rz], cpu_R[rz], 16);
            tcg_gen_setcondi_tl(TCG_COND_NE, cpu_c, cpu_R[rz], 0);
        } else if (pcode == 0x4) {
            /* xtrb2 */
            tcg_gen_andi_tl(cpu_R[rz], cpu_R[rx], 0x0000ff00);
            tcg_gen_shri_tl(cpu_R[rz], cpu_R[rz], 8);
            tcg_gen_setcondi_tl(TCG_COND_NE, cpu_c, cpu_R[rz], 0);
        } else if (pcode == 0x8) {
            /* xtrb3 */
            tcg_gen_andi_tl(cpu_R[rz], cpu_R[rx], 0xff);
            tcg_gen_setcondi_tl(TCG_COND_NE, cpu_c, cpu_R[rz], 0);
        } else {
            goto illegal_op;
        }
        break;
    case 0x1f:
        check_insn_except(ctx, CPU_E801);
        if (pcode == 0x1) {
            /* ff0 */
            gen_helper_ff0(cpu_R[rz], cpu_R[rx]);
        } else if (pcode == 0x2) {
            /* ff1 */
            gen_helper_ff1(cpu_R[rz], cpu_R[rx]);
        } else {
            goto illegal_op;
        }
        break;
    case 0x20:
        if (pcode == 0x1) {
            /* divu */
            check_insn_except(ctx, CPU_E801 | CPU_E802);
            divu(ctx, rz, rx, ry);
        } else if (pcode == 0x2) {
            /* divs */
            check_insn_except(ctx, CPU_E801 | CPU_E802);
            divs(ctx, rz, rx, ry);
        } else {
            goto illegal_op;
        }
        break;
    case 0x21:
        check_insn_except(ctx, CPU_E801);
        if (pcode == 0x1) {
            /* mult */
            tcg_gen_mul_tl(cpu_R[rz], cpu_R[rx], cpu_R[ry]);
        } else {
            goto illegal_op;
        }
        break;
    case 0x22:
        if (pcode == 0x1) {
            /* mulu */
            check_insn(ctx, CPU_C810 | CPU_C807 | ABIV2_DSP);
            TCGv_i64 t2 = tcg_temp_new_i64();
            TCGv_i64 t3 = tcg_temp_new_i64();

            tcg_gen_extu_tl_i64(t2, cpu_R[rx]);
            tcg_gen_extu_tl_i64(t3, cpu_R[ry]);
            tcg_gen_mul_i64(t2, t2, t3);
            tcg_gen_trunc_i64_tl(cpu_lo, t2);
            tcg_gen_shri_i64(t2, t2, 32);
            tcg_gen_trunc_i64_tl(cpu_hi, t2);
        } else if (pcode == 0x2) {
            /* mulua */
            check_insn(ctx, CPU_C810 | CPU_C807 | ABIV2_DSP);
            TCGv_i64 t2 = tcg_temp_new_i64();
            TCGv_i64 t3 = tcg_temp_new_i64();

            tcg_gen_extu_tl_i64(t2, cpu_R[rx]);
            tcg_gen_extu_tl_i64(t3, cpu_R[ry]);
            tcg_gen_mul_i64(t3, t3, t2);
            tcg_gen_concat_tl_i64(t2, cpu_lo, cpu_hi);
            tcg_gen_add_i64(t3, t3, t2);
            tcg_gen_trunc_i64_tl(cpu_lo, t3);
            tcg_gen_shri_i64(t3, t3, 32);
            tcg_gen_trunc_i64_tl(cpu_hi, t3);
        } else if (pcode == 0x4) {
            /* mulus */
            check_insn(ctx, CPU_C810 | CPU_C807 | ABIV2_DSP);
            TCGv_i64 t2 = tcg_temp_new_i64();
            TCGv_i64 t3 = tcg_temp_new_i64();

            tcg_gen_extu_tl_i64(t2, cpu_R[rx]);
            tcg_gen_extu_tl_i64(t3, cpu_R[ry]);
            tcg_gen_mul_i64(t3, t3, t2);
            tcg_gen_concat_tl_i64(t2, cpu_lo, cpu_hi);
            tcg_gen_sub_i64(t3, t2, t3);
            tcg_gen_trunc_i64_tl(cpu_lo, t3);
            tcg_gen_shri_i64(t3, t3, 32);
            tcg_gen_trunc_i64_tl(cpu_hi, t3);
        } else {
            goto illegal_op;
        }
        break;
    case 0x23:
        if (pcode == 0x1) {
            /* muls */
            check_insn(ctx, CPU_C810 | CPU_C807 | ABIV2_DSP);
            TCGv_i64 t2 = tcg_temp_new_i64();
            TCGv_i64 t3 = tcg_temp_new_i64();

            tcg_gen_ext_tl_i64(t2, cpu_R[rx]);
            tcg_gen_ext_tl_i64(t3, cpu_R[ry]);
            tcg_gen_mul_i64(t2, t2, t3);
            tcg_gen_trunc_i64_tl(cpu_lo, t2);
            tcg_gen_shri_i64(t2, t2, 32);
            tcg_gen_trunc_i64_tl(cpu_hi, t2);
        } else if (pcode == 0x2) {
            /* mulsa */
            check_insn(ctx, CPU_C810 | CPU_C807 | ABIV2_DSP);
            TCGv_i64 t2 = tcg_temp_new_i64();
            TCGv_i64 t3 = tcg_temp_new_i64();

            tcg_gen_ext_tl_i64(t2, cpu_R[rx]);
            tcg_gen_ext_tl_i64(t3, cpu_R[ry]);
            tcg_gen_mul_i64(t3, t3, t2);
            tcg_gen_concat_tl_i64(t2, cpu_lo, cpu_hi);
            tcg_gen_add_i64(t3, t3, t2);
            tcg_gen_trunc_i64_tl(cpu_lo, t3);
            tcg_gen_shri_i64(t3, t3, 32);
            tcg_gen_trunc_i64_tl(cpu_hi, t3);
        } else if (pcode == 0x4) {
            /* mulss */
            check_insn(ctx, CPU_C810 | CPU_C807 | ABIV2_DSP);
            TCGv_i64 t2 = tcg_temp_new_i64();
            TCGv_i64 t3 = tcg_temp_new_i64();

            tcg_gen_ext_tl_i64(t2, cpu_R[rx]);
            tcg_gen_ext_tl_i64(t3, cpu_R[ry]);
            tcg_gen_mul_i64(t3, t3, t2);
            tcg_gen_concat_tl_i64(t2, cpu_lo, cpu_hi);
            tcg_gen_sub_i64(t3, t2, t3);
            tcg_gen_trunc_i64_tl(cpu_lo, t3);
            tcg_gen_shri_i64(t3, t3, 32);
            tcg_gen_trunc_i64_tl(cpu_hi, t3);
        } else {
            goto illegal_op;
        }
        break;
    case 0x24:
        if (pcode == 0x1) {
            /* mulsh */
            check_insn_except(ctx, CPU_E801 | CPU_E802);
            tcg_gen_ext16s_tl(t0, cpu_R[rx]);
            tcg_gen_ext16s_tl(t1, cpu_R[ry]);
            tcg_gen_mul_tl(cpu_R[rz], t0, t1);
        } else if (pcode == 0x2) {
            /* mulsha */
            check_insn(ctx, CPU_C810 | CPU_C807 | ABIV2_DSP);
            mulsha(rx, ry);
        } else if (pcode == 0x4) {
            /* mulshs */
            check_insn(ctx, CPU_C810 | CPU_C807 | ABIV2_DSP);
            mulshs(rx, ry);
        } else {
            goto illegal_op;
        }
        break;
    case 0x25:
        if (pcode == 0x1) {
            /* mulsw */
            check_insn(ctx, CPU_C810 | CPU_C807 | CPU_C860 | ABIV2_DSP);
            mulsw(rz, rx, ry);
        } else if (pcode == 0x2) {
            /* mulswa */
            check_insn(ctx, CPU_C810 | CPU_C807 | ABIV2_DSP);
            mulswa(rx, ry);
        } else if (pcode == 0x4) {
            /* mulsws */
            check_insn(ctx, CPU_C810 | CPU_C807 | ABIV2_DSP);
            mulsws(rx, ry);
        } else {
            goto illegal_op;
        }
        break;
    case 0x26:
        if (pcode == 0x10) {
            /*mvtc*/
            check_insn(ctx, CPU_C807 | CPU_C810 | CPU_C860 | ABIV2_DSP);
            tcg_gen_mov_tl(cpu_c, cpu_v);
        } else {
            goto illegal_op;
        }
        break;
    case 0x27:
        if (pcode == 0x1) {
            /* mfhi */
            check_insn(ctx, CPU_C807 | CPU_C810 | ABIV2_DSP);
            tcg_gen_mov_tl(cpu_R[rz], cpu_hi);
        } else if (pcode == 0x2) {
            /* mthi */
            check_insn(ctx, CPU_C807 | CPU_C810 | ABIV2_DSP);
            tcg_gen_mov_tl(cpu_hi, cpu_R[rx]);
        } else if (pcode == 0x4) {
            /* mflo */
            check_insn(ctx, CPU_C807 | CPU_C810 | ABIV2_DSP);
            tcg_gen_mov_tl(cpu_R[rz], cpu_lo);
        } else if (pcode == 0x8) {
            /* mtlo */
            check_insn(ctx, CPU_C807 | CPU_C810 | ABIV2_DSP);
            tcg_gen_mov_tl(cpu_lo, cpu_R[rx]);
        } else {
            goto illegal_op;
        }
        break;
    default:
illegal_op:
        generate_exception(ctx, EXCP_CSKY_UDEF);
        break;
    }


}

static inline void lrs(DisasContext *ctx, int rz, uint32_t sop, int imm)
{
    TCGv t0 = tcg_temp_new();
    switch (sop) {
    case 0x0: /*lrs.b*/
        check_insn_except(ctx, CPU_E801 | CPU_E802);
        ldst(ld8u, 28, rz, imm, 4);
        break;
    case 0x1: /*lrs.h*/
        check_insn_except(ctx, CPU_E801 | CPU_E802);
        ldst(ld16u, 28, rz, imm << 1, 4);
        break;
    case 0x2: /*lrs.w*/
        check_insn_except(ctx, CPU_E801 | CPU_E802);
        ldst(ld32u, 28, rz, imm << 2, 4);
        break;
    case 0x3: /*grs*/
        {
            check_insn_except(ctx, CPU_E801 | CPU_E802);
            int t1;
            t1 = imm << 1;
            if (t1 & 0x40000) {
                t1 |= 0xfffc0000;
            }
            t1 += ctx->pc;
            tcg_gen_movi_tl(cpu_R[rz], t1);
        }
        break;
    case 0x4: /*srs.b*/
        check_insn_except(ctx, CPU_E801 | CPU_E802);
        ldst(st8, 28, rz, imm, 4);
        break;
    case 0x5: /*srs.h*/
        check_insn_except(ctx, CPU_E801 | CPU_E802);
        ldst(st16, 28, rz, imm << 1, 4);
        break;
    case 0x6: /*srs.w*/
        check_insn_except(ctx, CPU_E801 | CPU_E802);
        ldst(st32, 28, rz, imm << 2, 4);
        break;
    case 0x7:  /* addi */
        tcg_gen_addi_tl(cpu_R[rz], cpu_R[28], imm + 1);
        break;
    default:
        generate_exception(ctx, EXCP_CSKY_UDEF);
        break;
    }
}

static inline void imm_2op(DisasContext *ctx, int rz, int rx, uint32_t sop,
                            int imm)
{
    TCGv t0;

    check_insn_except(ctx, CPU_E801);
    switch (sop) {
    case 0x0:  /* addi */
        tcg_gen_addi_tl(cpu_R[rz], cpu_R[rx], imm + 1);
        break;
    case 0x1: /* subi */
        tcg_gen_subi_tl(cpu_R[rz], cpu_R[rx], imm + 1);
        break;
    case 0x2: /* andi */
        tcg_gen_andi_tl(cpu_R[rz], cpu_R[rx], imm);
        break;
    case 0x3: /* andni */
        t0 = csky_temp_const_tl(imm);
        tcg_gen_andc_tl(cpu_R[rz], cpu_R[rx], t0);
        break;
    case 0x4: /* xori */
        tcg_gen_xori_tl(cpu_R[rz], cpu_R[rx], imm);
        break;
    default:
        generate_exception(ctx, EXCP_CSKY_UDEF);
        break;
    }
}

static inline void imm_1op(DisasContext *ctx, uint32_t sop, int rx, int imm)
{
    target_ulong addr;
    int val = 0;
    TCGv_i32 t0, t1, t3;

    switch (sop) {
    case 0x0: /* br */
        val = imm << 1;
        if (val & 0x10000) {
            val |= 0xffff0000;
        }
        val += ctx->pc;

        gen_goto_tb(ctx, 0, val);
        ctx->base.is_jmp = DISAS_TB_JUMP;
        break;
    case 0x1: /* bnezad */
        check_insn(ctx, CPU_C860 | CPU_E803);
        bnezad(ctx, rx, imm);
        break;
    case 0x2: /* bf */
        check_insn_except(ctx, CPU_E801);
        branch32(ctx, TCG_COND_EQ, -1, imm);
        break;
    case 0x3: /* bt */
        check_insn_except(ctx, CPU_E801);
        branch32(ctx, TCG_COND_NE, -1, imm);
        break;
    case 0x6: /* jmp */
        check_insn_except(ctx, CPU_E801 | CPU_E802);
        t0 = tcg_temp_new();
        tcg_gen_andi_tl(t0, cpu_R[rx], 0xfffffffe);
        store_cpu_field(t0, pc);

#if !defined(CONFIG_USER_ONLY)
        if ((ctx->trace_mode == BRAN_TRACE_MODE)
            || (ctx->trace_mode == INST_TRACE_MODE)) {
            t0 = csky_temp_const_i32(EXCP_CSKY_TRACE);
            gen_helper_exception(tcg_env, t0);
        }
        ctx->maybe_change_flow = 1;
#endif
        ctx->base.is_jmp = DISAS_JUMP;
        break;
    case 0x7:/* jsr */
        check_insn_except(ctx, CPU_E801 | CPU_E802);
        t0 = tcg_temp_new();
        tcg_gen_andi_tl(t0, cpu_R[rx], 0xfffffffe);
        tcg_gen_movi_tl(cpu_R[15], ctx->pc + 4);
        store_cpu_field(t0, pc);

#if !defined(CONFIG_USER_ONLY)
        if ((ctx->trace_mode == BRAN_TRACE_MODE)
            || (ctx->trace_mode == INST_TRACE_MODE)) {
            t0 = csky_temp_const_i32(EXCP_CSKY_TRACE);
            gen_helper_exception(tcg_env, t0);
        }
        ctx->maybe_change_flow = 1;
#endif
        ctx->base.is_jmp = DISAS_JUMP;
        break;
    case 0x8: /* bez */
        check_insn_except(ctx, CPU_E801 | CPU_E802);
        branch32(ctx, TCG_COND_EQ, rx, imm);
        break;
    case 0x9: /* bnez */
        check_insn_except(ctx, CPU_E801 | CPU_E802);
        branch32(ctx, TCG_COND_NE, rx, imm);
        break;
    case 0xa:  /* bhz */
        check_insn_except(ctx, CPU_E801 | CPU_E802);
        branch32(ctx, TCG_COND_GT, rx, imm);
        break;
    case 0xb: /* blsz */
        check_insn_except(ctx, CPU_E801 | CPU_E802);
        branch32(ctx, TCG_COND_LE, rx, imm);
        break;
    case 0xc: /* blz */
        check_insn_except(ctx, CPU_E801 | CPU_E802);
        branch32(ctx, TCG_COND_LT, rx, imm);
        break;
    case 0xd: /* bhsz */
        check_insn_except(ctx, CPU_E801 | CPU_E802);
        branch32(ctx, TCG_COND_GE, rx, imm);
        break;
    case 0xe: /* bloop */
        check_insn(ctx, ABIV2_DSP2 | ABIV2_VDSP2);
        if (imm & 0x800) {
            val = imm | 0xf000;
        }
        tcg_gen_subi_tl(cpu_R[rx], cpu_R[rx], 1);
        branch32(ctx, TCG_COND_NE, rx, val);
        break;
    case 0xf: /* jmpix */
        check_insn(ctx, ABIV2_JAVA);
        if (ctx->bctm) {
            t0 = tcg_temp_new();
            t1 = tcg_temp_new();

            tcg_gen_andi_tl(t0, cpu_R[rx], 0xff);
            switch (imm & 0x3) {
            case 0x0:
                tcg_gen_shli_tl(t0, t0, 4);
                break;
            case 0x1:
                tcg_gen_shli_tl(t1, t0, 4);
                tcg_gen_shli_tl(t0, t0, 3);
                tcg_gen_add_tl(t0, t0, t1);
                break;
            case 0x2:
                tcg_gen_shli_tl(t0, t0, 5);
                break;
            case 0x3:
                tcg_gen_shli_tl(t1, t0, 5);
                tcg_gen_shli_tl(t0, t0, 3);
                tcg_gen_add_tl(t0, t0, t1);
                break;
            default:
                break;
            }
            tcg_gen_add_tl(t0, cpu_R[SVBR], t0);
            store_cpu_field(t0, pc);

            ctx->base.is_jmp = DISAS_JUMP;
            break;
        } else {
            generate_exception(ctx, EXCP_CSKY_UDEF);
            break;
        }
    case 0x10: /* movi */
        check_insn_except(ctx, CPU_E801);
        tcg_gen_movi_tl(cpu_R[rx], imm);
        break;
    case 0x11: /* movih */
        check_insn_except(ctx, CPU_E801);
        tcg_gen_movi_tl(cpu_R[rx], imm << 16);
        break;
    case 0x14:/* lrw */
        t0 = tcg_temp_new();
        t3 = csky_temp_const_i32(ctx->pc);
        addr = (ctx->pc + (imm << 2)) & 0xfffffffc ;
        tcg_gen_movi_tl(t0, addr);
        tcg_gen_qemu_ld_tl(cpu_R[rx], t0, ctx->mem_idx, MO_TEUL | ctx->align);
        if (gen_mem_trace()) {
            gen_helper_trace_ld32u(tcg_env, t3, cpu_R[rx], t0);
        }
        break;
    case 0x16: /* jmpi */
        check_insn_except(ctx, CPU_E801 | CPU_E802);
        t0 = tcg_temp_new();
        t3 = csky_temp_const_i32(ctx->pc);

        addr = (ctx->pc + (imm << 2)) & 0xfffffffc ;
        t1 = tcg_temp_new();
        tcg_gen_movi_tl(t1, addr);
        tcg_gen_movi_tl(t0, addr);
        tcg_gen_qemu_ld_tl(t0, t0, ctx->mem_idx, MO_TEUL | ctx->align);
        if (gen_mem_trace()) {
            gen_helper_trace_ld32u(tcg_env, t3, t0, t1);
        }
        store_cpu_field(t0, pc);

#if !defined(CONFIG_USER_ONLY)
        if ((ctx->trace_mode == BRAN_TRACE_MODE)
            || (ctx->trace_mode == INST_TRACE_MODE)) {
            t0 = csky_temp_const_i32(EXCP_CSKY_TRACE);
            gen_helper_exception(tcg_env, t0);
        }
        ctx->maybe_change_flow = 1;
#endif
        ctx->base.is_jmp = DISAS_JUMP;

        break;
    case 0x17: /* jsri */
        check_insn_except(ctx, CPU_E801 | CPU_E802);
        t0 = tcg_temp_new();
        t1 = tcg_temp_new();
        t3 = csky_temp_const_i32(ctx->pc);
        addr =  (ctx->pc + (imm << 2)) & 0xfffffffc;
        tcg_gen_movi_tl(cpu_R[15], ctx->pc + 4);
        tcg_gen_movi_tl(t1, addr);
        tcg_gen_movi_tl(t0, addr);
        tcg_gen_qemu_ld_tl(t0, t0, ctx->mem_idx, MO_TEUL | ctx->align);
        if (gen_mem_trace()) {
            gen_helper_trace_ld32u(tcg_env, t3, t0, t1);
        }
        store_cpu_field(t0, pc);

#if !defined(CONFIG_USER_ONLY)
        if ((ctx->trace_mode == BRAN_TRACE_MODE)
            || (ctx->trace_mode == INST_TRACE_MODE)) {
            tcg_gen_movi_i32(t0, EXCP_CSKY_TRACE);
            gen_helper_exception(tcg_env, t0);
        }
        ctx->maybe_change_flow = 1;
#endif
        ctx->base.is_jmp = DISAS_JUMP;

        break;
    case 0x18: /* cmphsi */
        check_insn_except(ctx, CPU_E801);
        tcg_gen_setcondi_tl(TCG_COND_GEU, cpu_c, cpu_R[rx], imm + 1);
        break;
    case 0x19:  /* cmplti */
        check_insn_except(ctx, CPU_E801);
        tcg_gen_setcondi_tl(TCG_COND_LT, cpu_c, cpu_R[rx], imm + 1);
        break;
    case 0x1a: /* cmpnei */
        check_insn_except(ctx, CPU_E801);
        tcg_gen_setcondi_tl(TCG_COND_NE, cpu_c, cpu_R[rx], imm);
        break;
    case 0x1e:   /* pop */
        check_insn_except(ctx, CPU_E801 | CPU_E802);
        if (ctx->bctm) {
            t0 = tcg_temp_new();
            TCGLabel *l1 = gen_new_label();
            tcg_gen_brcondi_tl(TCG_COND_NE, cpu_R[sp], 0, l1);
            tcg_gen_movi_tl(cpu_R[15], ctx->pc + 4);
            tcg_gen_subi_tl(t0, cpu_R[SVBR], 4);
            store_cpu_field(t0, pc);
            tcg_gen_exit_tb(NULL, 0);
            gen_set_label(l1);
            pop(ctx, imm & 0x1ff);
            break;
        } else {
            pop(ctx, imm & 0x1ff);
            break;
        }
    case 0x1f:    /* push */
        check_insn_except(ctx, CPU_E801 | CPU_E802);
        if (ctx->bctm) {
            t0 = tcg_temp_new();
            TCGLabel *l1 = gen_new_label();
            tcg_gen_brcondi_tl(TCG_COND_NE, cpu_R[sp], 0, l1);
            tcg_gen_movi_tl(cpu_R[15], ctx->pc + 4);
            tcg_gen_subi_tl(t0, cpu_R[SVBR], 4);
            store_cpu_field(t0, pc);
            tcg_gen_exit_tb(NULL, 0);
            gen_set_label(l1);
            push(ctx, imm & 0x1ff);
            gen_goto_tb(ctx, 1, ctx->pc + 4);
            ctx->base.is_jmp = DISAS_TB_JUMP;
            break;
        } else {
            push(ctx, imm & 0x1ff);
            break;
        }
    default:
        generate_exception(ctx, EXCP_CSKY_UDEF);
        break;
    }
}

static inline void gen_vfp_ld(DisasContext *s, int dp, TCGv addr)
{
    if (dp) {
        /*doesn't cause alignment exception*/
#if !TARGET_BIG_ENDIAN
        tcg_gen_qemu_ld_tl(cpu_F0s, addr, s->mem_idx, MO_TEUL | s->align);
        tcg_gen_addi_i32(addr, addr, 4);
        tcg_gen_qemu_ld_tl(cpu_F1s, addr, s->mem_idx, MO_TEUL | s->align);
        tcg_gen_concat_i32_i64(cpu_F0d, cpu_F0s, cpu_F1s);
#else
        tcg_gen_qemu_ld_tl(cpu_F0s, addr, s->mem_idx, MO_TEUL | s->align);
        tcg_gen_addi_i32(addr, addr, 4);
        tcg_gen_qemu_ld_tl(cpu_F1s, addr, s->mem_idx, MO_TEUL | s->align);
        tcg_gen_concat_i32_i64(cpu_F0d, cpu_F1s, cpu_F0s);
#endif
    } else {
        tcg_gen_qemu_ld_tl(cpu_F0s, addr, s->mem_idx, MO_TEUL | s->align);
    }
}

static inline void gen_vfp_st(DisasContext *s, int dp, TCGv addr)
{
    if (dp) {
        /*doesn't cause alignment exception*/
#if !TARGET_BIG_ENDIAN
        tcg_gen_extrl_i64_i32(cpu_F0s, cpu_F0d);
        tcg_gen_qemu_st_tl(cpu_F0s, addr, s->mem_idx, MO_TEUL | s->align);
        tcg_gen_shri_i64(cpu_F0d, cpu_F0d, 32);
        tcg_gen_extrl_i64_i32(cpu_F1s, cpu_F0d);
        tcg_gen_addi_i32(addr, addr, 4);
        tcg_gen_qemu_st_tl(cpu_F1s, addr, s->mem_idx, MO_TEUL | s->align);
#else
        tcg_gen_extrl_i64_i32(cpu_F0s, cpu_F0d);
        tcg_gen_shri_i64(cpu_F0d, cpu_F0d, 32);
        tcg_gen_extrl_i64_i32(cpu_F1s, cpu_F0d);
        tcg_gen_qemu_st_tl(cpu_F1s, addr, s->mem_idx, MO_TEUL | s->align);
        tcg_gen_addi_i32(addr, addr, 4);
        tcg_gen_qemu_st_tl(cpu_F0s, addr, s->mem_idx, MO_TEUL | s->align);
#endif
    } else {
        tcg_gen_qemu_st_tl(cpu_F0s, addr, s->mem_idx, MO_TEUL | s->align);
    }
}

#define vfp_reg_offset(reg) offsetof(CPUCSKYState, \
    vfp.reg[reg].fpu[0])

#define tcg_gen_ld_f32 tcg_gen_ld_i32
#define tcg_gen_ld_f64 tcg_gen_ld_i64
#define tcg_gen_st_f32 tcg_gen_st_i32
#define tcg_gen_st_f64 tcg_gen_st_i64

static inline void gen_mov_F0_vreg(int dp, int reg)
{
    if (dp) {
        tcg_gen_ld_f64(cpu_F0d, tcg_env, vfp_reg_offset(reg));
    } else {
        tcg_gen_ld_f32(cpu_F0s, tcg_env, vfp_reg_offset(reg));
    }
}

static inline void gen_mov_F0_vreg_hi(int dp, int reg)
{
    if (dp) {
        tcg_gen_ld_f64(cpu_F0d, tcg_env, vfp_reg_offset(reg) + 4);
    } else {
        tcg_gen_ld_f32(cpu_F0s, tcg_env, vfp_reg_offset(reg) + 4);
    }
}

static inline void gen_mov_F1_vreg(int dp, int reg)
{
    if (dp) {
        tcg_gen_ld_f64(cpu_F1d, tcg_env, vfp_reg_offset(reg));
    } else {
        tcg_gen_ld_f32(cpu_F1s, tcg_env, vfp_reg_offset(reg));
    }
}

static inline void gen_mov_F1_vreg_hi(int dp, int reg)
{
    if (dp) {
        tcg_gen_ld_f64(cpu_F1d, tcg_env, vfp_reg_offset(reg) + 4);
    } else {
        tcg_gen_ld_f32(cpu_F1s, tcg_env, vfp_reg_offset(reg) + 4);
    }
}

static inline void gen_mov_vreg_F0(int dp, int reg)
{
    if (dp) {
        tcg_gen_st_f64(cpu_F0d, tcg_env, vfp_reg_offset(reg));
    } else {
        tcg_gen_st_f32(cpu_F0s, tcg_env, vfp_reg_offset(reg));
    }
}

static inline void gen_mov_vreg_F0_hi(int dp, int reg)
{
    if (dp) {
        tcg_gen_st_f64(cpu_F0d, tcg_env, vfp_reg_offset(reg) + 4);
    } else {
        tcg_gen_st_f32(cpu_F0s, tcg_env, vfp_reg_offset(reg) + 4);
    }
}

static inline void gen_vfp_F1_ld0(int dp)
{
    if (dp) {
        tcg_gen_movi_i64(cpu_F1d, 0);
    } else {
        tcg_gen_movi_i32(cpu_F1s, 0);
    }
}

static inline void gen_vfp_add(int dp)
{
    if (dp) {
        gen_helper_vfp_addd(cpu_F0d, cpu_F0d, cpu_F1d, tcg_env);
    } else {
        gen_helper_vfp_adds(cpu_F0s, cpu_F0s, cpu_F1s, tcg_env);
    }
}

static inline void gen_vfp_sub(int dp)
{
    if (dp) {
        gen_helper_vfp_subd(cpu_F0d, cpu_F0d, cpu_F1d, tcg_env);
    } else {
        gen_helper_vfp_subs(cpu_F0s, cpu_F0s, cpu_F1s, tcg_env);
    }
}

static inline void gen_vfp_mul(int dp)
{
    if (dp) {
        gen_helper_vfp_muld(cpu_F0d, cpu_F0d, cpu_F1d, tcg_env);
    } else {
        gen_helper_vfp_muls(cpu_F0s, cpu_F0s, cpu_F1s, tcg_env);
    }
}

static inline void gen_vfp_div(int dp)
{
    if (dp) {
        gen_helper_vfp_divd(cpu_F0d, cpu_F0d, cpu_F1d, tcg_env);
    } else {
        gen_helper_vfp_divs(cpu_F0s, cpu_F0s, cpu_F1s, tcg_env);
    }
}



static inline void gen_vfp_abs(int dp)
{
    if (dp) {
        gen_helper_vfp_absd(cpu_F0d, cpu_F0d);
    } else {
        gen_helper_vfp_abss(cpu_F0s, cpu_F0s);
    }
}

static inline void gen_vfp_neg(int dp)
{
    if (dp) {
        gen_helper_vfp_negd(cpu_F0d, cpu_F0d);
    } else {
        gen_helper_vfp_negs(cpu_F0s, cpu_F0s);
    }
}

static inline void gen_vfp_sqrt(int dp)
{
    if (dp) {
        gen_helper_vfp_sqrtd(cpu_F0d, cpu_F0d, tcg_env);
    } else {
        gen_helper_vfp_sqrts(cpu_F0s, cpu_F0s, tcg_env);
    }
}

static inline void gen_vfp_recip(int dp)
{
    if (dp) {
        gen_helper_vfp_recipd(cpu_F0d, cpu_F0d, tcg_env);
    } else {
        gen_helper_vfp_recips(cpu_F0s, cpu_F0s, tcg_env);
    }
}


static inline void gen_vfp_cmp_ge(int dp)
{
    if (dp) {
        gen_helper_vfp_cmp_ged(cpu_F0d, cpu_F1d, tcg_env);
    } else {
        gen_helper_vfp_cmp_ges(cpu_F0s, cpu_F1s, tcg_env);
    }
}

static inline void gen_vfp_cmp_l(int dp)
{
    if (dp) {
        gen_helper_vfp_cmp_ld(cpu_F0d, cpu_F1d, tcg_env);
    } else {
        gen_helper_vfp_cmp_ls(cpu_F0s, cpu_F1s, tcg_env);
    }
}

static inline void gen_vfp_cmp_ls(int dp)
{
    if (dp) {
        gen_helper_vfp_cmp_lsd(cpu_F0d, cpu_F1d, tcg_env);
    } else {
        gen_helper_vfp_cmp_lss(cpu_F0s, cpu_F1s, tcg_env);
    }
}

static inline void gen_vfp_cmp_ne(int dp)
{
    if (dp) {
        gen_helper_vfp_cmp_ned(cpu_F0d, cpu_F1d, tcg_env);
    } else {
        gen_helper_vfp_cmp_nes(cpu_F0s, cpu_F1s, tcg_env);
    }
}

static inline void gen_vfp_cmp_isNAN(int dp)
{
    if (dp) {
        gen_helper_vfp_cmp_isNANd(cpu_F0d, cpu_F1d, tcg_env);
    } else {
        gen_helper_vfp_cmp_isNANs(cpu_F0s, cpu_F1s, tcg_env);
    }
}

static inline void gen_vfp_tosirn(int dp)
{
    if (dp) {
        gen_helper_vfp_tosirnd(cpu_F0s, cpu_F0d, tcg_env);
    } else {
        gen_helper_vfp_tosirns(cpu_F0s, cpu_F0s, tcg_env);
    }
}

static inline void gen_vfp_tosirz(int dp)
{
    if (dp) {
        gen_helper_vfp_tosirzd(cpu_F0s, cpu_F0d, tcg_env);
    } else {
        gen_helper_vfp_tosirzs(cpu_F0s, cpu_F0s, tcg_env);
    }
}

static inline void gen_vfp_tosirpi(int dp)
{
    if (dp) {
        gen_helper_vfp_tosirpid(cpu_F0s, cpu_F0d, tcg_env);
    } else {
        gen_helper_vfp_tosirpis(cpu_F0s, cpu_F0s, tcg_env);
    }
}

static inline void gen_vfp_tosirni(int dp)
{
    if (dp) {
        gen_helper_vfp_tosirnid(cpu_F0s, cpu_F0d, tcg_env);
    } else {
        gen_helper_vfp_tosirnis(cpu_F0s, cpu_F0s, tcg_env);
    }
}

static inline void gen_vfp_touirn(int dp)
{
    if (dp) {
        gen_helper_vfp_touirnd(cpu_F0s, cpu_F0d, tcg_env);
    } else {
        gen_helper_vfp_touirns(cpu_F0s, cpu_F0s, tcg_env);
    }
}

static inline void gen_vfp_touirz(int dp)
{
    if (dp) {
        gen_helper_vfp_touirzd(cpu_F0s, cpu_F0d, tcg_env);
    } else {
        gen_helper_vfp_touirzs(cpu_F0s, cpu_F0s, tcg_env);
    }
}

static inline void gen_vfp_touirpi(int dp)
{
    if (dp) {
        gen_helper_vfp_touirpid(cpu_F0s, cpu_F0d, tcg_env);
    } else {
        gen_helper_vfp_touirpis(cpu_F0s, cpu_F0s, tcg_env);
    }
}

static inline void gen_vfp_touirni(int dp)
{
    if (dp) {
        gen_helper_vfp_touirnid(cpu_F0s, cpu_F0d, tcg_env);
    } else {
        gen_helper_vfp_touirnis(cpu_F0s, cpu_F0s, tcg_env);
    }
}

static inline void gen_vfp_uito(int dp)
{
    if (dp) {
        gen_helper_vfp_uitod(cpu_F0d, cpu_F0s, tcg_env);
    } else {
        gen_helper_vfp_uitos(cpu_F0s, cpu_F0s, tcg_env);
    }
}

static inline void gen_vfp_sito(int dp)
{
    if (dp) {
        gen_helper_vfp_sitod(cpu_F0d, cpu_F0s, tcg_env);
    } else {
        gen_helper_vfp_sitos(cpu_F0s, cpu_F0s, tcg_env);
    }
}

/* Move between integer and VFP cores.  */
static TCGv gen_vfp_mrs(void)
{
    TCGv tmp = new_tmp();
    tcg_gen_mov_i32(tmp, cpu_F0s);
    return tmp;
}

static void gen_vfp_msr(TCGv tmp)
{
    tcg_gen_mov_i32(cpu_F0s, tmp);
    dead_tmp(tmp);
}

static inline void fpu_insn_fmovi(int32_t insn)
{
    int vrz = insn & 0xf;
    int imm = (((insn >> 21) & 0xf) << 7) | (((insn >> 4) & 0xf) << 3) | 0x800;
    int pos = (insn >> 16) & 0xf;
    int sign = (insn >> 20) & 0x1;
    int dp = (insn >> 9) & 0x1;
    TCGv_i32 t0, t1, t2;
    t0 = csky_temp_const_i32(imm);
    t1 = csky_temp_const_i32(pos);
    t2 = csky_temp_const_i32(sign);
    if (dp) {
        gen_helper_vfp_fmovid(cpu_F0d, t0, t1, t2, tcg_env);
    } else {
        gen_helper_vfp_fmovis(cpu_F0s, t0, t1, t2, tcg_env);
    }
    gen_mov_vreg_F0(dp, vrz);
}

static void disas_vfp_insn(CPUCSKYState *env, DisasContext *s, uint32_t insn)
{
    int op1, op2, dp, imm, shift, i;
    int vrx, vry, vrz;
    int rx, ry, rz;

    TCGv addr;
    TCGv tmp;
    op1 = (insn >> 8) & 0xff;
    op2 = (insn >> 4) & 0xf;

    switch (op1) {
    case 0x0: /*single-alu*/
        check_insn(s, ABIV2_FLOAT_S);
        switch (op2) {
        case 0x0:/* fadds */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vry =  (insn >> 21) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_F1_vreg(dp, vry);
            gen_vfp_add(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0x2:/* fsubs */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vry =  (insn >> 21) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_F1_vreg(dp, vry);
            gen_vfp_sub(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0x8:/* fmovs */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0xc:/* fabss */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_vfp_abs(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0xe:/* fnegs */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_vfp_neg(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
        default:
            goto wrong;
            break;
        }
        break;
    case 0x1:/*single-compare*/
        check_insn(s, ABIV2_FLOAT_S);
        switch (op2) {
        case 0x0:/* fcmpzhss */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_vfp_F1_ld0(dp);
            gen_vfp_cmp_ge(dp);
            break;
        case 0x2:/* fcmpzlss  */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_vfp_F1_ld0(dp);
            gen_vfp_cmp_ls(dp);
            break;
        case 0x4:/* fcmpznes  */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_vfp_F1_ld0(dp);
            gen_vfp_cmp_ne(dp);
            break;
        case 0x6:/* fcmpzuos  */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_F1_vreg(dp, vrx);
            gen_vfp_cmp_isNAN(dp);
            break;
        case 0x8:/* fcmphss */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vry =  (insn >> 21) & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_F1_vreg(dp, vry);
            gen_vfp_cmp_ge(dp);
            break;
        case 0xa:/* fcmplts */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vry =  (insn >> 21) & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_F1_vreg(dp, vry);
            gen_vfp_cmp_l(dp);
            break;
        case 0xc:/* fcmpnes */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vry =  (insn >> 21) & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_F1_vreg(dp, vry);
            gen_vfp_cmp_ne(dp);
            break;
        case 0xe:/* fcmpuos */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vry =  (insn >> 21) & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_F1_vreg(dp, vry);
            gen_vfp_cmp_isNAN(dp);
            break;
        default:
            goto wrong;
            break;
        }
        break;

    case 0x2:/* single-mul */
        check_insn(s, ABIV2_FLOAT_S);
        switch (op2) {
        case 0x0:/* fmuls */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vry =  (insn >> 21) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_F1_vreg(dp, vry);
            gen_vfp_mul(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0x2:/* fnmuls  */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vry =  (insn >> 21) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_F1_vreg(dp, vry);
            gen_vfp_mul(dp);
            gen_vfp_neg(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0x8:/* fmacs */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vry =  (insn >> 21) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_F1_vreg(dp, vry);
            gen_vfp_mul(dp);
            gen_mov_F1_vreg(dp, vrz);
            gen_vfp_add(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0xa:/* fmscs */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vry =  (insn >> 21) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_F1_vreg(dp, vry);
            gen_vfp_mul(dp);
            gen_mov_F1_vreg(dp, vrz);
            gen_vfp_sub(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0xc:/* fnmacs */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vry =  (insn >> 21) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_F1_vreg(dp, vry);
            gen_vfp_mul(dp);
            gen_vfp_neg(dp);
            gen_mov_F1_vreg(dp, vrz);
            gen_vfp_add(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0xe:/* fnmscs */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vry = (insn >> 21) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_F1_vreg(dp, vry);
            gen_vfp_mul(dp);
            gen_vfp_neg(dp);
            gen_mov_F1_vreg(dp, vrz);
            gen_vfp_sub(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
        default:
            goto wrong;
            break;
        }
        break;

    case 0x3:/* single-div */
        check_insn(s, ABIV2_FLOAT_S);
        switch (op2) {
        case 0x0:/* fdivs*/
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vry = (insn >> 21) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_F1_vreg(dp, vry);
            gen_vfp_div(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0x2:/* frecips  */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_vfp_recip(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0x4:/* fsqrts  */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_vfp_sqrt(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
        default:
            goto wrong;
            break;
        }
        break;

    case 0x8:/* double-alu */
        check_insn(s, ABIV2_FLOAT_D);
        switch (op2) {
        case 0x0:/* faddd */
            dp = 1;
            vrx = (insn >> 16) & 0xf;
            vry =  (insn >> 21) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_F1_vreg(dp, vry);
            gen_vfp_add(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0x2:/* fsubd */
            dp = 1;
            vrx = (insn >> 16) & 0xf;
            vry =  (insn >> 21) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_F1_vreg(dp, vry);
            gen_vfp_sub(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0x8:/* fmovd */
            dp = 1;
            vrx = (insn >> 16) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0xc:/* fabsd */
            dp = 1;
            vrx = (insn >> 16) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_vfp_abs(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0xe:/* fnegd */
            dp = 1;
            vrx = (insn >> 16) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_vfp_neg(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
        default:
            goto wrong;
            break;

        }
        break;

    case 0x9:/* double-compare */
        check_insn(s, ABIV2_FLOAT_D);
        switch (op2) {
        case 0x0:/* fcmpzhsd */
            dp = 1;
            vrx = (insn >> 16) & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_vfp_F1_ld0(dp);
            gen_vfp_cmp_ge(dp);
            break;
        case 0x2:/* fcmpzlsd  */
            dp = 1;
            vrx = (insn >> 16) & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_vfp_F1_ld0(dp);
            gen_vfp_cmp_ls(dp);
            break;
        case 0x4:/* fcmpzned  */
            dp = 1;
            vrx = (insn >> 16) & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_vfp_F1_ld0(dp);
            gen_vfp_cmp_ne(dp);
            break;
        case 0x6:/* fcmpzuod  */
            dp = 1;
            vrx = (insn >> 16) & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_F1_vreg(dp, vrx);
            gen_vfp_cmp_isNAN(dp);
            break;
        case 0x8:/* fcmphsd */
            dp = 1;
            vrx = (insn >> 16) & 0xf;
            vry = (insn >> 21) & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_F1_vreg(dp, vry);
            gen_vfp_cmp_ge(dp);
            break;
        case 0xa:/* fcmpltd */
            dp = 1;
            vrx = (insn >> 16) & 0xf;
            vry = (insn >> 21) & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_F1_vreg(dp, vry);
            gen_vfp_cmp_l(dp);
            break;
        case 0xc:/* fcmpned */
            dp = 1;
            vrx = (insn >> 16) & 0xf;
            vry = (insn >> 21) & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_F1_vreg(dp, vry);
            gen_vfp_cmp_ne(dp);
            break;
        case 0xe:/* fcmpuod */
            dp = 1;
            vrx = (insn >> 16) & 0xf;
            vry =  (insn >> 21) & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_F1_vreg(dp, vry);
            gen_vfp_cmp_isNAN(dp);
            break;
        default:
            goto wrong;
            break;

        }
        break;

    case 0xa:/* double-mul */
        check_insn(s, ABIV2_FLOAT_D);
        switch (op2) {
        case 0x0:/* fmuld */
            dp = 1;
            vrx = (insn >> 16) & 0xf;
            vry = (insn >> 21) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_F1_vreg(dp, vry);
            gen_vfp_mul(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0x2:/* fnmuld  */
            dp = 1;
            vrx = (insn >> 16) & 0xf;
            vry = (insn >> 21) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_F1_vreg(dp, vry);
            gen_vfp_mul(dp);
            gen_vfp_neg(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0x8:/* fmacd */
            dp = 1;
            vrx = (insn >> 16) & 0xf;
            vry = (insn >> 21) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_F1_vreg(dp, vry);
            gen_vfp_mul(dp);
            gen_mov_F1_vreg(dp, vrz);
            gen_vfp_add(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0xa:/* fmscd */
            dp = 1;
            vrx = (insn >> 16) & 0xf;
            vry = (insn >> 21) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_F1_vreg(dp, vry);
            gen_vfp_mul(dp);
            gen_mov_F1_vreg(dp, vrz);
            gen_vfp_sub(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0xc:/* fnmacd */
            dp = 1;
            vrx = (insn >> 16) & 0xf;
            vry = (insn >> 21) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_F1_vreg(dp, vry);
            gen_vfp_mul(dp);
            gen_vfp_neg(dp);
            gen_mov_F1_vreg(dp, vrz);
            gen_vfp_add(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0xe:/* fnmscd */
            dp = 1;
            vrx = (insn >> 16) & 0xf;
            vry = (insn >> 21) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_F1_vreg(dp, vry);
            gen_vfp_mul(dp);
            gen_vfp_neg(dp);
            gen_mov_F1_vreg(dp, vrz);
            gen_vfp_sub(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
        default:
            goto wrong;
            break;
        }
        break;

    case 0xb:/* double-div */
        check_insn(s, ABIV2_FLOAT_D);
        switch (op2) {
        case 0x0:/* fdivd*/
            dp = 1;
            vrx = (insn >> 16) & 0xf;
            vry = (insn >> 21) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_F1_vreg(dp, vry);
            gen_vfp_div(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0x2:/* frecipd  */
            dp = 1;
            vrx = (insn >> 16) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_vfp_recip(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0x4:/* fsqrtd  */
            dp = 1;
            vrx = (insn >> 16) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_vfp_sqrt(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
        default:
            goto wrong;
            break;
        }
        break;

    case 0x10:/* simd-alu */
        switch (op2) {
        case 0x0:/* faddm */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vry =  (insn >> 21) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_F1_vreg(dp, vry);
            gen_vfp_add(dp);
            gen_mov_vreg_F0(dp, vrz);
            gen_mov_F0_vreg_hi(dp, vrx);
            gen_mov_F1_vreg_hi(dp, vry);
            gen_vfp_add(dp);
            gen_mov_vreg_F0_hi(dp, vrz);
            break;
        case 0x2:/* fsubm */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vry =  (insn >> 21) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_F1_vreg(dp, vry);
            gen_vfp_sub(dp);
            gen_mov_vreg_F0(dp, vrz);
            gen_mov_F0_vreg_hi(dp, vrx);
            gen_mov_F1_vreg_hi(dp, vry);
            gen_vfp_sub(dp);
            gen_mov_vreg_F0_hi(dp, vrz);
            break;
        case 0x8:/* fmovm */
            dp = 1;
            vrx = (insn >> 16) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0xc:/* fabsm */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_vfp_abs(dp);
            gen_mov_vreg_F0(dp, vrz);
            gen_mov_F0_vreg_hi(dp, vrx);
            gen_vfp_abs(dp);
            gen_mov_vreg_F0_hi(dp, vrz);
            break;
        case 0xe:/* fnegm */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_vfp_neg(dp);
            gen_mov_vreg_F0(dp, vrz);
            gen_mov_F0_vreg_hi(dp, vrx);
            gen_vfp_neg(dp);
            gen_mov_vreg_F0_hi(dp, vrz);
            break;
        default:
            goto wrong;
            break;
        }
        break;

    case 0x12:/* simd-mul  --fixe case 0x11:*/
        switch (op2) {
        case 0x0:/* fmulm */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vry =  (insn >> 21) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_F1_vreg(dp, vry);
            gen_vfp_mul(dp);
            gen_mov_vreg_F0(dp, vrz);
            gen_mov_F0_vreg_hi(dp, vrx);
            gen_mov_F1_vreg_hi(dp, vry);
            gen_vfp_mul(dp);
            gen_mov_vreg_F0_hi(dp, vrz);
            break;
        case 0x2:/* fnmulm  */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vry =  (insn >> 21) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_F1_vreg(dp, vry);
            gen_vfp_mul(dp);
            gen_vfp_neg(dp);
            gen_mov_vreg_F0(dp, vrz);
            gen_mov_F0_vreg_hi(dp, vrx);
            gen_mov_F1_vreg_hi(dp, vry);
            gen_vfp_mul(dp);
            gen_vfp_neg(dp);
            gen_mov_vreg_F0_hi(dp, vrz);
            break;
        case 0x8:/* fmacm */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vry =  (insn >> 21) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_F1_vreg(dp, vry);
            gen_vfp_mul(dp);
            gen_mov_F1_vreg(dp, vrz);
            gen_vfp_add(dp);
            gen_mov_vreg_F0(dp, vrz);
            gen_mov_F0_vreg_hi(dp, vrx);
            gen_mov_F1_vreg_hi(dp, vry);
            gen_vfp_mul(dp);
            gen_mov_F1_vreg_hi(dp, vrz);
            gen_vfp_add(dp);
            gen_mov_vreg_F0_hi(dp, vrz);
            break;
        case 0xa:/* fmscm */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vry =  (insn >> 21) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_F1_vreg(dp, vry);
            gen_vfp_mul(dp);
            gen_mov_F1_vreg(dp, vrz);
            gen_vfp_sub(dp);
            gen_mov_vreg_F0(dp, vrz);
            gen_mov_F0_vreg_hi(dp, vrx);
            gen_mov_F1_vreg_hi(dp, vry);
            gen_vfp_mul(dp);
            gen_mov_F1_vreg_hi(dp, vrz);
            gen_vfp_sub(dp);
            gen_mov_vreg_F0_hi(dp, vrz);
            break;
        case 0xc:/* fnmacm */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vry =  (insn >> 21) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_F1_vreg(dp, vry);
            gen_vfp_mul(dp);
            gen_vfp_neg(dp);
            gen_mov_F1_vreg(dp, vrz);
            gen_vfp_add(dp);
            gen_mov_vreg_F0(dp, vrz);
            gen_mov_F0_vreg_hi(dp, vrx);
            gen_mov_F1_vreg_hi(dp, vry);
            gen_vfp_mul(dp);
            gen_vfp_neg(dp);
            gen_mov_F1_vreg_hi(dp, vrz);
            gen_vfp_add(dp);
            gen_mov_vreg_F0_hi(dp, vrz);
            break;
        case 0xe:/* fnmscm */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vry =  (insn >> 21) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_mov_F1_vreg(dp, vry);
            gen_vfp_mul(dp);
            gen_vfp_neg(dp);
            gen_mov_F1_vreg(dp, vrz);
            gen_vfp_sub(dp);
            gen_mov_vreg_F0(dp, vrz);
            gen_mov_F0_vreg_hi(dp, vrx);
            gen_mov_F1_vreg_hi(dp, vry);
            gen_vfp_mul(dp);
            gen_vfp_neg(dp);
            gen_mov_F1_vreg_hi(dp, vrz);
            gen_vfp_sub(dp);
            gen_mov_vreg_F0_hi(dp, vrz);
            break;
        default:
            goto wrong;
            break;

        }
        break;

    case 0x18:/* for-sti */
        switch (op2) {
            /* fstosi */
        case 0x0:/* fstosi.rn */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_vfp_tosirn(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0x2:/* fstosi.rz */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_vfp_tosirz(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0x4:/* fstosi.rpi */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_vfp_tosirpi(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0x6:/* fstosi.rni */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_vfp_tosirni(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
            /* fstoui */
        case 0x8:/* fstoui.rn */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_vfp_touirn(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0xa:/* fstoui.rz */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_vfp_touirz(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0xc:/* fstoui.rpi */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_vfp_touirpi(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0xe:/* fstoui.rni */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_vfp_touirni(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
        default:
            goto wrong;
            break;

        }
        break;

    case 0x19:/* for-dti */
        switch (op2) {
            /* fdtosi */
        case 0x0:/* fdtosi.rn */
            dp = 1;
            vrx = (insn >> 16) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_vfp_tosirn(dp);
            dp = 0;
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0x2:/* fdtosi.rz */
            dp = 1;
            vrx = (insn >> 16) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_vfp_tosirz(dp);
            dp = 0;
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0x4:/* fdtosi.rpi */
            dp = 1;
            vrx = (insn >> 16) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_vfp_tosirpi(dp);
            dp = 0;
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0x6:/* fdtosi.rni */
            dp = 1;
            vrx = (insn >> 16) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_vfp_tosirni(dp);
            dp = 0;
            gen_mov_vreg_F0(dp, vrz);
            break;
            /* fdtoui */
        case 0x8:/* fdtoui.rn */
            dp = 1;
            vrx = (insn >> 16) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_vfp_touirn(dp);
            dp = 0;
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0xa:/* fdtoui.rz */
            dp = 1;
            vrx = (insn >> 16) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_vfp_touirz(dp);
            dp = 0;
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0xc:/* fdtoui.rpi */
            dp = 1;
            vrx = (insn >> 16) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_vfp_touirpi(dp);
            dp = 0;
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0xe:/* fdtoui.rni */
            dp = 1;
            vrx = (insn >> 16) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_vfp_touirni(dp);
            dp = 0;
            gen_mov_vreg_F0(dp, vrz);
            break;
        default:
            goto wrong;
            break;
        }
        break;

    case 0x1a:/* for-misc */
        switch (op2) {
        case 0x0:/* fsitos */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_vfp_sito(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0x2:/* fuitos  */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            gen_vfp_uito(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0x8:/* fsitod */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            dp = 1;
            gen_vfp_sito(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0xa:/* fuitod */
            dp = 0;
            vrx = (insn >> 16) & 0xf;
            vrz = insn & 0xf;
            gen_mov_F0_vreg(dp, vrx);
            dp = 1;
            gen_vfp_uito(dp);
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0xc:/* fdtos */
            vrx = (insn >> 16) & 0xf;
            vrz = insn & 0xf;
            dp = 1;
            gen_mov_F0_vreg(dp, vrx);
            gen_helper_vfp_tosd(cpu_F0s, cpu_F0d, tcg_env);
            dp = 0;
            gen_mov_vreg_F0(dp, vrz);
            break;
        case 0xe:/* fstod */
            vrx = (insn >> 16) & 0xf;
            vrz = insn & 0xf;
            dp = 0;
            gen_mov_F0_vreg(dp, vrx);
            gen_helper_vfp_tods(cpu_F0d, cpu_F0s, tcg_env);
            dp = 1;
            gen_mov_vreg_F0(dp, vrz);
            break;
        default:
            goto wrong;
            break;
        }
        break;

    case 0x1b:/* for-fmvr */
        switch (op2) {
        case 0x0:
        case 0x1:/* fmfvrh*/
            vrx = (insn >> 16) & 0xf;
            rz = insn & 0x1f;
            gen_mov_F0_vreg_hi(0, vrx);
            tmp = gen_vfp_mrs();
            tcg_gen_mov_i32(cpu_R[rz], tmp);
            break;
        case 0x2:
        case 0x3:/* fmfvrl */
            vrx = (insn >> 16) & 0xf;
            rz = insn & 0x1f;
            gen_mov_F0_vreg(0, vrx);
            tmp = gen_vfp_mrs();
            tcg_gen_mov_i32(cpu_R[rz], tmp);
            break;
        case 0x4:/* fmtvrh */
            rx = (insn >> 16) & 0x1f;
            vrz = insn & 0xf;
            tmp = load_reg(s, rx);
            gen_vfp_msr(tmp);
            gen_mov_vreg_F0_hi(0, vrz);
            break;
        case 0x6:/* fmtvrl */
            rx = (insn >> 16) & 0x1f;
            vrz = insn & 0xf;
            tmp = load_reg(s, rx);
            gen_vfp_msr(tmp);
            gen_mov_vreg_F0(0, vrz);
            break;
        default:
            goto wrong;
            break;
        }
        break;
    /* fmovis */
    case 0x1c:
    case 0x1e:
        fpu_insn_fmovi(insn);
        break;
     /* flsu */
    case 0x20:/* flds */
        rx = (insn >> 16) & 0x1f;
        vrz = insn & 0xf;
        imm = ((insn >> 17) & 0xf0) | ((insn >> 4) & 0xf);
        addr =  load_reg(s, rx);
        tcg_gen_addi_i32(addr, addr, imm << 2);
        gen_vfp_ld(s, 0, addr);
        gen_mov_vreg_F0(0, vrz);
        dead_tmp(addr);
        break;
    case 0x21:/* fldd */
        rx = (insn >> 16) & 0x1f;
        vrz = insn & 0xf;
        imm = ((insn >> 17) & 0xf0) | ((insn >> 4) & 0xf);
        addr =  load_reg(s, rx);
        tcg_gen_addi_i32(addr, addr, imm << 2);
        gen_vfp_ld(s, 1, addr);
        gen_mov_vreg_F0(1, vrz);
        dead_tmp(addr);
        break;
    case 0x22:/* fldm */
        rx = (insn >> 16) & 0x1f;
        vrz = insn & 0xf;
        imm = ((insn >> 17) & 0xf0) | ((insn >> 4) & 0xf);
        addr =  load_reg(s, rx);
        tcg_gen_addi_i32(addr, addr, imm << 2);
        gen_vfp_ld(s, 1, addr);
        gen_mov_vreg_F0(1, vrz);
        dead_tmp(addr);
        break;
    case 0x24:/* fsts */
        rx = (insn >> 16) & 0x1f;
        vrz = insn & 0xf;
        imm = ((insn >> 17) & 0xf0) | ((insn >> 4) & 0xf);
        addr =  load_reg(s, rx);
        tcg_gen_addi_i32(addr, addr, imm << 2);
        gen_mov_F0_vreg(0, vrz);
        gen_vfp_st(s, 0, addr);
        dead_tmp(addr);
        break;
    case 0x25:/* fstd */
        rx = (insn >> 16) & 0x1f;
        vrz = insn & 0xf;
        imm = ((insn >> 17) & 0xf0) | ((insn >> 4) & 0xf);
        addr =  load_reg(s, rx);
        tcg_gen_addi_i32(addr, addr, imm << 2);
        gen_mov_F0_vreg(1, vrz);
        gen_vfp_st(s, 1, addr);
        dead_tmp(addr);
        break;
    case 0x26:/* fstm */
        rx = (insn >> 16) & 0x1f;
        vrz = insn & 0xf;
        imm = ((insn >> 17) & 0xf0) | ((insn >> 4) & 0xf);
        addr =  load_reg(s, rx);
        tcg_gen_addi_i32(addr, addr, imm << 2);
        gen_mov_F0_vreg(1, vrz);
        gen_vfp_st(s, 1, addr);
        dead_tmp(addr);
        break;
    case 0x28:/* fldrs */
        rx = (insn >> 16) & 0x1f;
        ry = (insn >> 21) & 0x1f;
        shift = (insn >> 0x5) & 0x3;
        vrz = insn & 0xf;
        addr =  load_reg(s, rx);
        tmp =  load_reg(s, ry);
        tcg_gen_shli_i32(tmp, tmp, shift);
        tcg_gen_add_i32(addr, addr, tmp);
        dead_tmp(tmp);
        gen_vfp_ld(s, 0, addr);
        gen_mov_vreg_F0(0, vrz);
        dead_tmp(addr);
        break;
    case 0x29:/* fldrd */
        rx = (insn >> 16) & 0x1f;
        ry = (insn >> 21) & 0x1f;
        shift = (insn >> 0x5) & 0x3;
        vrz = insn & 0xf;
        addr =  load_reg(s, rx);
        tmp =  load_reg(s, ry);
        tcg_gen_shli_i32(tmp, tmp, shift);
        tcg_gen_add_i32(addr, addr, tmp);
        dead_tmp(tmp);
        gen_vfp_ld(s, 1, addr);
        gen_mov_vreg_F0(1, vrz);
        dead_tmp(addr);
        break;
    case 0x2a:/* fldrm */
        rx = (insn >> 16) & 0x1f;
        ry = (insn >> 21) & 0x1f;
        shift = (insn >> 0x5) & 0x3;
        vrz = insn & 0xf;
        addr =  load_reg(s, rx);
        tmp =  load_reg(s, ry);
        tcg_gen_shli_i32(tmp, tmp, shift);
        tcg_gen_add_i32(addr, addr, tmp);
        dead_tmp(tmp);
        gen_vfp_ld(s, 1, addr);
        gen_mov_vreg_F0(1, vrz);
        dead_tmp(addr);
        break;
    case 0x2c:/* fstrs */
        rx = (insn >> 16) & 0x1f;
        ry = (insn >> 21) & 0x1f;
        shift = (insn >> 0x5) & 0x3;
        vrz = insn & 0xf;
        addr =  load_reg(s, rx);
        tmp =  load_reg(s, ry);
        tcg_gen_shli_i32(tmp, tmp, shift);
        tcg_gen_add_i32(addr, addr, tmp);
        dead_tmp(tmp);
        gen_mov_F0_vreg(0, vrz);
        gen_vfp_st(s, 0, addr);
        dead_tmp(addr);
        break;
    case 0x2d:/* fstrd */
        rx = (insn >> 16) & 0x1f;
        ry = (insn >> 21) & 0x1f;
        shift = (insn >> 0x5) & 0x3;
        vrz = insn & 0xf;
        addr =  load_reg(s, rx);
        tmp =  load_reg(s, ry);
        tcg_gen_shli_i32(tmp, tmp, shift);
        tcg_gen_add_i32(addr, addr, tmp);
        dead_tmp(tmp);
        gen_mov_F0_vreg(1, vrz);
        gen_vfp_st(s, 1, addr);
        dead_tmp(addr);
        break;
    case 0x2e:/* fstrm */
        rx = (insn >> 16) & 0x1f;
        ry = (insn >> 21) & 0x1f;
        shift = (insn >> 0x5) & 0x3;
        vrz = insn & 0xf;
        addr =  load_reg(s, rx);
        tmp =  load_reg(s, ry);
        tcg_gen_shli_i32(tmp, tmp, shift);
        tcg_gen_add_i32(addr, addr, tmp);
        dead_tmp(tmp);
        gen_mov_F0_vreg(1, vrz);
        gen_vfp_st(s, 1, addr);
        dead_tmp(addr);
        break;
    case 0x30:/* fldms */
        rx = (insn >> 16) & 0x1f;
        vrz = insn & 0xf;
        imm = (insn >> 21) & 0xf;
        addr =  load_reg(s, rx);
        for (i = 0; i <= imm; i++) {
            gen_vfp_ld(s, 0, addr);
            gen_mov_vreg_F0(0, vrz);
            tcg_gen_addi_i32(addr, addr, 4);
            vrz++;
        }
        dead_tmp(addr);
        break;
    case 0x31:/* fldmd */
        rx = (insn >> 16) & 0x1f;
        vrz = insn & 0xf;
        imm = (insn >> 21) & 0xf;
        addr =  load_reg(s, rx);
        for (i = 0; i <= imm; i++) {
            gen_vfp_ld(s, 1, addr);
            gen_mov_vreg_F0(1, vrz);
            tcg_gen_addi_i32(addr, addr, 4);
            vrz++;
        }
        dead_tmp(addr);
        break;
    case 0x32:/* fldmm */
        rx = (insn >> 16) & 0x1f;
        vrz = insn & 0xf;
        imm = (insn >> 21) & 0xf;
        addr =  load_reg(s, rx);
        for (i = 0; i <= imm; i++) {
            gen_vfp_ld(s, 1, addr);
            gen_mov_vreg_F0(1, vrz);
            tcg_gen_addi_i32(addr, addr, 4);
            vrz++;
        }
        dead_tmp(addr);
        break;
    case 0x34:/* fstms */
        rx = (insn >> 16) & 0x1f;
        vrz = insn & 0xf;
        imm = (insn >> 21) & 0xf;
        addr =  load_reg(s, rx);
        for (i = 0; i <= imm; i++) {
            gen_mov_F0_vreg(0, vrz);
            gen_vfp_st(s, 0, addr);
            tcg_gen_addi_i32(addr, addr, 4);
            vrz++;
        }
        dead_tmp(addr);
        break;
    case 0x35:/* fstmd */
        rx = (insn >> 16) & 0x1f;
        vrz = insn & 0xf;
        imm = (insn >> 21) & 0xf;
        addr =  load_reg(s, rx);
        for (i = 0; i <= imm; i++) {
            gen_mov_F0_vreg(1, vrz);
            gen_vfp_st(s, 1, addr);
            tcg_gen_addi_i32(addr, addr, 4);
            vrz++;
        }
        dead_tmp(addr);
        break;
    case 0x36:/* fstmm */
        rx = (insn >> 16) & 0x1f;
        vrz = insn & 0xf;
        imm = (insn >> 21) & 0xf;
        addr = load_reg(s, rx);
        for (i = 0; i <= imm; i++) {
            gen_mov_F0_vreg(1, vrz);
            gen_vfp_st(s, 1, addr);
            tcg_gen_addi_i32(addr, addr, 4);
            vrz++;
         }
        dead_tmp(addr);
        break;
    default:
wrong:
        generate_exception(s, EXCP_CSKY_UDEF);
        qemu_log_mask(LOG_GUEST_ERROR, "unknown vdsp insn pc=%x opc=%x\n",
                      s->pc, insn);
        break;
    }

}

static void disas_fpu3_insn(CPUCSKYState *env, DisasContext *s, uint32_t insn)
{
    int op1, op2;
    TCGv fpu3_insn = csky_temp_const_tl(insn);

    op1 = (insn >> 8) & 0xff;
    op2 = (insn >> 5) & 0x7;

    switch (op1) {
    case 0x0: /* single-alu */
        check_insn(s, ABIV2_FLOAT_S);
        switch (op2) {
        case 0x0: /* fadds */
            gen_helper_fpu3_fadds(tcg_env, fpu3_insn);
            break;
        case 0x1: /* fsubs */
            gen_helper_fpu3_fsubs(tcg_env, fpu3_insn);
            break;
        case 0x4: /* fmovs */
            gen_helper_fpu3_fmovs(tcg_env, fpu3_insn);
            break;
        case 0x6: /* fabss */
            gen_helper_fpu3_fabss(tcg_env, fpu3_insn);
            break;
        case 0x7: /* fnegs */
            gen_helper_fpu3_fnegs(tcg_env, fpu3_insn);
            break;
        default:
            goto wrong;
            break;
        }
        break;
    case 0x1: /* single-compare */
        check_insn(s, ABIV2_FLOAT_S);
        switch (op2) {
        case 0x0: /* fcmpzhss */
            gen_helper_fpu3_fcmpzhss(tcg_env, fpu3_insn);
            break;
        case 0x1: /* fcmpzlts  */
            gen_helper_fpu3_fcmpzlts(tcg_env, fpu3_insn);
            break;
        case 0x2: /* fcmpznes/fcmpnez */
            gen_helper_fpu3_fcmpznes(tcg_env, fpu3_insn);
            break;
        case 0x3: /* fcmpzuos  */
            gen_helper_fpu3_fcmpzuos(tcg_env, fpu3_insn);
            break;
        case 0x4: /* fcmphss */
            gen_helper_fpu3_fcmphss(tcg_env, fpu3_insn);
            break;
        case 0x5: /* fcmplts */
            gen_helper_fpu3_fcmplts(tcg_env, fpu3_insn);
            break;
        case 0x6: /* fcmpnes */
            gen_helper_fpu3_fcmpnes(tcg_env, fpu3_insn);
            break;
        case 0x7: /* fcmpuos */
            gen_helper_fpu3_fcmpuos(tcg_env, fpu3_insn);
            break;
        default:
            goto wrong;
            break;
        }
        break;

    case 0x2: /* single-mul */
        check_insn(s, ABIV2_FLOAT_S);
        switch (op2) {
        case 0x0: /* fmuls */
            gen_helper_fpu3_fmuls(tcg_env, fpu3_insn);
            break;
        case 0x1: /* fnmuls */
            gen_helper_fpu3_fnmuls(tcg_env, fpu3_insn);
            break;
        case 0x4: /* fmacs */
            gen_helper_fpu3_fmacs(tcg_env, fpu3_insn);
            break;
        case 0x5: /* fmscs */
            gen_helper_fpu3_fmscs(tcg_env, fpu3_insn);
            break;
        case 0x6: /* fnmacs */
            gen_helper_fpu3_fnmacs(tcg_env, fpu3_insn);
            break;
        case 0x7: /* fnmscs */
            gen_helper_fpu3_fnmscs(tcg_env, fpu3_insn);
            break;
        default:
            goto wrong;
            break;
        }
        break;

    case 0x3: /* single-div */
        check_insn(s, ABIV2_FLOAT_S);
        switch (op2) {
        case 0x0: /* fdivs*/
            gen_helper_fpu3_fdivs(tcg_env, fpu3_insn);
            break;
        case 0x1: /* frecips  */
            gen_helper_fpu3_frecips(tcg_env, fpu3_insn);
            break;
        case 0x2: /* fsqrts  */
            gen_helper_fpu3_fsqrts(tcg_env, fpu3_insn);
            break;
        case 0x3: /* fins.32 */
            gen_helper_fpu3_fins32(tcg_env, fpu3_insn);
            break;
        default:
            goto wrong;
            break;
        }
        break;

    case 0x5: /* single-compare */
        check_insn(s, ABIV2_FLOAT_S);
        switch (op2) {
        case 0x0: /* fmaxnm.32 */
            gen_helper_fpu3_fmaxnm32(tcg_env, fpu3_insn);
            break;
        case 0x1: /* fminnm.32 */
            gen_helper_fpu3_fminnm32(tcg_env, fpu3_insn);
            break;
        case 0x2: /* fcmphz.32 */
            gen_helper_fpu3_fcmphz32(tcg_env, fpu3_insn);
            break;
        case 0x3: /* fcmplsz.32 */
            gen_helper_fpu3_fcmplsz32(tcg_env, fpu3_insn);
            break;
        default:
            goto wrong;
            break;
        }
        break;

    case 0x6: /* single-mul */
        check_insn(s, ABIV2_FLOAT_S);
        switch (op2) {
        case 0x0: /* ffmula.32 */
            gen_helper_fpu3_ffmula32(tcg_env, fpu3_insn);
            break;
        case 0x1: /* ffmuls.32 */
            gen_helper_fpu3_ffmuls32(tcg_env, fpu3_insn);
            break;
        case 0x2: /* ffnmula.32 */
            gen_helper_fpu3_ffnmula32(tcg_env, fpu3_insn);
            break;
        case 0x3: /* ffnmuls.32 */
            gen_helper_fpu3_ffnmuls32(tcg_env, fpu3_insn);
            break;
        default:
            goto wrong;
            break;
        }
        break;

    case 0x7: /* single-misc */
        check_insn(s, ABIV2_FLOAT_S);
        switch (op2) {
        case 0x1: /* fsel.32 */
            gen_helper_fpu3_fsel32(tcg_env, fpu3_insn);
            break;
        default:
            goto wrong;
            break;
        }
        break;

    case 0x8: /* double-alu */
        check_insn(s, ABIV2_FLOAT_D);
        switch (op2) {
        case 0x0: /* faddd */
            gen_helper_fpu3_faddd(tcg_env, fpu3_insn);
            break;
        case 0x1: /* fsubd */
            gen_helper_fpu3_fsubd(tcg_env, fpu3_insn);
            break;
        case 0x4: /* fmovd */
            gen_helper_fpu3_fmovd(tcg_env, fpu3_insn);
            break;
        case 0x5: /* fmovx.32 */
            gen_helper_fpu3_fmovx32(tcg_env, fpu3_insn);
            break;
        case 0x6: /* fabsd */
            gen_helper_fpu3_fabsd(tcg_env, fpu3_insn);
            break;
        case 0x7: /* fnegd */
            gen_helper_fpu3_fnegd(tcg_env, fpu3_insn);
            break;
        default:
            goto wrong;
            break;
        }
        break;

    case 0x9: /* double-compare */
        check_insn(s, ABIV2_FLOAT_D);
        switch (op2) {
        case 0x0: /* fcmpzhsd */
            gen_helper_fpu3_fcmpzhsd(tcg_env, fpu3_insn);
            break;
        case 0x1: /* fcmpzltd  */
            gen_helper_fpu3_fcmpzltd(tcg_env, fpu3_insn);
            break;
        case 0x2: /* fcmpzned  */
            gen_helper_fpu3_fcmpzned(tcg_env, fpu3_insn);
            break;
        case 0x3: /* fcmpzuod  */
            gen_helper_fpu3_fcmpzuod(tcg_env, fpu3_insn);
            break;
        case 0x4: /* fcmphsd */
            gen_helper_fpu3_fcmphsd(tcg_env, fpu3_insn);
            break;
        case 0x5: /* fcmpltd */
            gen_helper_fpu3_fcmpltd(tcg_env, fpu3_insn);
            break;
        case 0x6: /* fcmpned */
            gen_helper_fpu3_fcmpned(tcg_env, fpu3_insn);
            break;
        case 0x7: /* fcmpuod */
            gen_helper_fpu3_fcmpuod(tcg_env, fpu3_insn);
            break;
        default:
            goto wrong;
            break;
        }
        break;

    case 0xa: /* double-mul */
        check_insn(s, ABIV2_FLOAT_D);
        switch (op2) {
        case 0x0: /* fmuld */
            gen_helper_fpu3_fmuld(tcg_env, fpu3_insn);
            break;
        case 0x1: /* fnmuld  */
            gen_helper_fpu3_fnmuld(tcg_env, fpu3_insn);
            break;
        case 0x4: /* fmacd */
            gen_helper_fpu3_fmacd(tcg_env, fpu3_insn);
            break;
        case 0x5: /* fmscd */
            gen_helper_fpu3_fmscd(tcg_env, fpu3_insn);
            break;
        case 0x6: /* fnmacd */
            gen_helper_fpu3_fnmacd(tcg_env, fpu3_insn);
            break;
        case 0x7: /* fnmscd */
            gen_helper_fpu3_fnmscd(tcg_env, fpu3_insn);
            break;
        default:
            goto wrong;
            break;
        }
        break;

    case 0xb: /* double-div */
        check_insn(s, ABIV2_FLOAT_D);
        switch (op2) {
        case 0x0: /* fdivd */
            gen_helper_fpu3_fdivd(tcg_env, fpu3_insn);
            break;
        case 0x1: /* frecipd */
            gen_helper_fpu3_frecipd(tcg_env, fpu3_insn);
            break;
        case 0x2: /* fsqrtd */
            gen_helper_fpu3_fsqrtd(tcg_env, fpu3_insn);
            break;
        default:
            goto wrong;
            break;
        }
        break;

    case 0xd: /* double-compare */
        check_insn(s, ABIV2_FLOAT_D);
        switch (op2) {
        case 0x0: /* fmaxnm.64 */
            gen_helper_fpu3_fmaxnm64(tcg_env, fpu3_insn);
            break;
        case 0x1: /* fminnm.64 */
            gen_helper_fpu3_fminnm64(tcg_env, fpu3_insn);
            break;
        case 0x2: /* fcmphz.64 */
            gen_helper_fpu3_fcmphz64(tcg_env, fpu3_insn);
            break;
        case 0x3: /* fcmplsz.64 */
            gen_helper_fpu3_fcmplsz64(tcg_env, fpu3_insn);
            break;
        default:
            goto wrong;
            break;
        }
        break;

    case 0xe: /* double-mul */
        check_insn(s, ABIV2_FLOAT_D);
        switch (op2) {
        case 0x0: /* ffmula.64 */
            gen_helper_fpu3_ffmula64(tcg_env, fpu3_insn);
            break;
        case 0x1: /* ffmuls.64 */
            gen_helper_fpu3_ffmuls64(tcg_env, fpu3_insn);
            break;
        case 0x2: /* ffnmula.64 */
            gen_helper_fpu3_ffnmula64(tcg_env, fpu3_insn);
            break;
        case 0x3: /* ffnmuls.64 */
            gen_helper_fpu3_ffnmuls64(tcg_env, fpu3_insn);
            break;
        default:
            goto wrong;
            break;
        }
        break;

    case 0xf: /* double-misc */
        check_insn(s, ABIV2_FLOAT_D);
        switch (op2) {
        case 0x1: /* fsel.64 */
            gen_helper_fpu3_fsel64(tcg_env, fpu3_insn);
            break;
        default:
            goto wrong;
            break;
        }
        break;

    case 0x18: /* for-sti */
        switch (op2) {
            /* fstosi */
        case 0x0: /* fstosi.rn */
            gen_helper_fpu3_fstosirn(tcg_env, fpu3_insn);
            break;
        case 0x1: /* fstosi.rz */
            gen_helper_fpu3_fstosirz(tcg_env, fpu3_insn);
            break;
        case 0x2: /* fstosi.rpi */
            gen_helper_fpu3_fstosirpi(tcg_env, fpu3_insn);
            break;
        case 0x3: /* fstosi.rni */
            gen_helper_fpu3_fstosirni(tcg_env, fpu3_insn);
            break;
            /* fstoui */
        case 0x4: /* fstoui.rn */
            gen_helper_fpu3_fstouirn(tcg_env, fpu3_insn);
            break;
        case 0x5: /* fstoui.rz */
            gen_helper_fpu3_fstouirz(tcg_env, fpu3_insn);
            break;
        case 0x6: /* fstoui.rpi */
            gen_helper_fpu3_fstouirpi(tcg_env, fpu3_insn);
            break;
        case 0x7: /* fstoui.rni */
            gen_helper_fpu3_fstouirni(tcg_env, fpu3_insn);
            break;
        default:
            goto wrong;
            break;
        }
        break;

    case 0x19: /* for-dti, fftoi.t1.t2.rm */
        switch (op2) {
            /* fdtosi */
        case 0x0: /* fdtosi.rn */
            gen_helper_fpu3_fdtosirn(tcg_env, fpu3_insn);
            break;
        case 0x1: /* fdtosi.rz */
            gen_helper_fpu3_fdtosirz(tcg_env, fpu3_insn);
            break;
        case 0x2: /* fdtosi.rpi */
            gen_helper_fpu3_fdtosirpi(tcg_env, fpu3_insn);
            break;
        case 0x3: /* fdtosi.rni */
            gen_helper_fpu3_fdtosirni(tcg_env, fpu3_insn);
            break;
            /* fdtoui */
        case 0x4: /* fdtoui.rn */
            gen_helper_fpu3_fdtouirn(tcg_env, fpu3_insn);
            break;
        case 0x5: /* fdtoui.rz */
            gen_helper_fpu3_fdtouirz(tcg_env, fpu3_insn);
            break;
        case 0x6: /* fdtoui.rpi */
            gen_helper_fpu3_fdtouirpi(tcg_env, fpu3_insn);
            break;
        case 0x7: /* fdtoui.rni */
            gen_helper_fpu3_fdtouirni(tcg_env, fpu3_insn);
            break;
        default:
            goto wrong;
            break;
        }
        break;

    case 0x1a: /* for-misc */
        switch (op2) {
        case 0x0: /* fsitos */
            gen_helper_fpu3_fsitos(tcg_env, fpu3_insn);
            break;
        case 0x1: /* fuitos */
            gen_helper_fpu3_fuitos(tcg_env, fpu3_insn);
            break;
        case 0x2: /* fhtos */
            gen_helper_fpu3_fhtos(tcg_env, fpu3_insn);
            break;
        case 0x3: /* fstoh */
            gen_helper_fpu3_fstoh(tcg_env, fpu3_insn);
            break;
        case 0x4: /* fsitod */
            gen_helper_fpu3_fsitod(tcg_env, fpu3_insn);
            break;
        case 0x5: /* fuitod */
            gen_helper_fpu3_fuitod(tcg_env, fpu3_insn);
            break;
        case 0x6: /* fdtos */
            gen_helper_fpu3_fdtos(tcg_env, fpu3_insn);
            break;
        case 0x7: /* fstod */
            gen_helper_fpu3_fstod(tcg_env, fpu3_insn);
            break;
        default:
            goto wrong;
            break;
        }
        break;

    case 0x1b: /* for-fmvr */
        switch (op2) {
        case 0x0: /* fmfvrh */
            gen_helper_fpu3_fmfvrh(tcg_env, fpu3_insn);
            break;
        case 0x1: /* fmfvrl */
            gen_helper_fpu3_fmfvrl(tcg_env, fpu3_insn);
            break;
        case 0x2: /* fmtvrh */
            gen_helper_fpu3_fmtvrh(tcg_env, fpu3_insn);
            break;
        case 0x3: /* fmtvrl */
            gen_helper_fpu3_fmtvrl(tcg_env, fpu3_insn);
            break;
        default:
            goto wrong;
            break;
        }
        break;

    case 0x1c: /* for-hti */
        switch (op2) {
            /* fhtosi */
        case 0x0: /* fhtosi.rn */
            gen_helper_fpu3_fhtosirn(tcg_env, fpu3_insn);
            break;
        case 0x1: /* fhtosi.rz */
            gen_helper_fpu3_fhtosirz(tcg_env, fpu3_insn);
            break;
        case 0x2: /* fhtosi.rpi */
            gen_helper_fpu3_fhtosirpi(tcg_env, fpu3_insn);
            break;
        case 0x3: /* fhtosi.rni */
            gen_helper_fpu3_fhtosirni(tcg_env, fpu3_insn);
            break;
            /* fhtoui */
        case 0x4: /* fhtoui.rn */
            gen_helper_fpu3_fhtouirn(tcg_env, fpu3_insn);
            break;
        case 0x5: /* fhtoui.rz */
            gen_helper_fpu3_fhtouirz(tcg_env, fpu3_insn);
            break;
        case 0x6: /* fhtoui.rpi */
            gen_helper_fpu3_fhtouirpi(tcg_env, fpu3_insn);
            break;
        case 0x7: /* fhtoui.rni */
            gen_helper_fpu3_fhtouirni(tcg_env, fpu3_insn);
            break;
        default:
            goto wrong;
            break;
        }
        break;

    case 0x1e:
        break;
    case 0x1f: /* fmvr */
        switch (op2) {
        case 0x0: /* fmfvr.64 */
            gen_helper_fpu3_fmfvr64(tcg_env, fpu3_insn);
            break;
        case 0x1: /* fmfvr.16 */
            gen_helper_fpu3_fmfvr16(tcg_env, fpu3_insn);
            break;
        case 0x2: /* fmfvr.32.2 */
            gen_helper_fpu3_fmfvr322(tcg_env, fpu3_insn);
            break;
        case 0x4: /* fmtvr.64 */
            gen_helper_fpu3_fmtvr64(tcg_env, fpu3_insn);
            break;
        case 0x5: /* fmtvr.16 */
            gen_helper_fpu3_fmtvr16(tcg_env, fpu3_insn);
            break;
        case 0x6: /* fmtvr.32.2 */
            gen_helper_fpu3_fmtvr322(tcg_env, fpu3_insn);
            break;
        default:
            goto wrong;
            break;
        }
        break;

     /* flsu */
    case 0x20: /* flds */
        gen_helper_fpu3_flds(tcg_env, fpu3_insn);
        break;
    case 0x21: /* fldd */
        gen_helper_fpu3_fldd(tcg_env, fpu3_insn);
        break;
    case 0x22: /* fldm */
        gen_helper_fpu3_fldm(tcg_env, fpu3_insn);
        break;
    case 0x23: /* fldh */
        gen_helper_fpu3_fldh(tcg_env, fpu3_insn);
        break;
    case 0x24: /* fsts */
        gen_helper_fpu3_fsts(tcg_env, fpu3_insn);
        break;
    case 0x25: /* fstd */
        gen_helper_fpu3_fstd(tcg_env, fpu3_insn);
        break;
    case 0x26: /* fstm */
        gen_helper_fpu3_fstm(tcg_env, fpu3_insn);
        break;
    case 0x27: /* fsth */
        gen_helper_fpu3_fsth(tcg_env, fpu3_insn);
        break;
    case 0x28: /* fldrs */
        gen_helper_fpu3_fldrs(tcg_env, fpu3_insn);
        break;
    case 0x29: /* fldrd */
        gen_helper_fpu3_fldrd(tcg_env, fpu3_insn);
        break;
    case 0x2a: /* fldrm */
        gen_helper_fpu3_fldrm(tcg_env, fpu3_insn);
        break;
    case 0x2b: /* fldrh */
        gen_helper_fpu3_fldrh(tcg_env, fpu3_insn);
        break;
    case 0x2c: /* fstrs */
        gen_helper_fpu3_fstrs(tcg_env, fpu3_insn);
        break;
    case 0x2d: /* fstrd */
        gen_helper_fpu3_fstrd(tcg_env, fpu3_insn);
        break;
    case 0x2e: /* fstrm */
        gen_helper_fpu3_fstrm(tcg_env, fpu3_insn);
        break;
    case 0x2f: /* fstrh */
        gen_helper_fpu3_fstrh(tcg_env, fpu3_insn);
        break;
    case 0x30:
        switch (op2) {
        case 0x0: /* fldms/fldm.32 */
            gen_helper_fpu3_fldms(tcg_env, fpu3_insn);
            break;
        case 0x4: /* fldmu.s/fldmu.32 */
            gen_helper_fpu3_fldmus(tcg_env, fpu3_insn);
            break;
        default:
            goto wrong;
            break;
        }
        break;
    case 0x31:
        switch (op2) {
        case 0x0: /* fldmd/fldm.64 */
            gen_helper_fpu3_fldmd(tcg_env, fpu3_insn);
            break;
        case 0x4: /* fldmu.d/fldmu.64 */
            gen_helper_fpu3_fldmud(tcg_env, fpu3_insn);
            break;
        default:
            goto wrong;
            break;
        }
        break;
    case 0x32: /* fldmm */
        gen_helper_fpu3_fldmm(tcg_env, fpu3_insn);
        break;
    case 0x33:
        switch (op2) {
        case 0x0: /* fldmh/fldm.16 */
            gen_helper_fpu3_fldmh(tcg_env, fpu3_insn);
            break;
        case 0x4: /* fldmu.h/fldmu.16 */
            gen_helper_fpu3_fldmuh(tcg_env, fpu3_insn);
            break;
        default:
            goto wrong;
            break;
        }
        break;
    case 0x34: /* fstms */
        switch (op2) {
        case 0x0: /* fstms/fstms.32 */
            gen_helper_fpu3_fstms(tcg_env, fpu3_insn);
            break;
        case 0x4: /* fstmu.s/fstmu.32 */
            gen_helper_fpu3_fstmus(tcg_env, fpu3_insn);
            break;
        default:
            goto wrong;
            break;
        }
        break;
    case 0x35: /* fstmd */
        switch (op2) {
        case 0x0: /* fstmd/fstms.64 */
            gen_helper_fpu3_fstmd(tcg_env, fpu3_insn);
            break;
        case 0x4: /* fstmu.d/fstmu.64 */
            gen_helper_fpu3_fstmud(tcg_env, fpu3_insn);
            break;
        default:
            goto wrong;
            break;
        }
        break;
    case 0x36: /* fstmm */
        gen_helper_fpu3_fstmm(tcg_env, fpu3_insn);
        break;
    case 0x37: /* fstmh */
        switch (op2) {
        case 0x0: /* fstmh/fstmh.16 */
            gen_helper_fpu3_fstmh(tcg_env, fpu3_insn);
            break;
        case 0x4: /* fstmu.h/fstmu.16 */
            gen_helper_fpu3_fstmuh(tcg_env, fpu3_insn);
            break;
        default:
            goto wrong;
            break;
        }
        break;
    case 0x38:
        gen_save_pc(s->pc);
        gen_helper_fpu3_flrws(tcg_env, fpu3_insn);
        break;
    case 0x39:
        gen_save_pc(s->pc);
        gen_helper_fpu3_flrwd(tcg_env, fpu3_insn);
        break;
    case 0x3a:
        gen_save_pc(s->pc);
        gen_helper_fpu3_flrwh(tcg_env, fpu3_insn);
        break;
    case 0x40: /* fftox.t1.t2 */
    case 0x41: /* fftox.t1.t2 */
        gen_helper_fpu3_fftox(tcg_env, fpu3_insn);
        break;
    case 0x42: /* fftoi.t1.t2 */
    case 0x43: /* fftoi.t1.t2 */
        gen_helper_fpu3_fftoi(tcg_env, fpu3_insn);
        break;
    case 0x44: /* fftofi.t.rm */
    case 0x45: /* fftofi.t.rm */
        gen_helper_fpu3_fftofi(tcg_env, fpu3_insn);
        break;
    case 0x48: /* fxtof.t1.t2 */
    case 0x49: /* fxtof.t1.t2 */
        gen_helper_fpu3_fxtof(tcg_env, fpu3_insn);
        break;
    case 0x4a: /* fitof.t1.t2 */
    case 0x4b: /* fitof.t1.t2 */
        gen_helper_fpu3_fitof(tcg_env, fpu3_insn);
        break;
    case 0xc8: /* half-alu */
        switch (op2) {
        case 0x0: /* faddh */
            gen_helper_fpu3_faddh(tcg_env, fpu3_insn);
            break;
        case 0x1: /* fsubh */
            gen_helper_fpu3_fsubh(tcg_env, fpu3_insn);
            break;
        case 0x4: /* fmovh */
            gen_helper_fpu3_fmovh(tcg_env, fpu3_insn);
            break;
        case 0x6: /* fabsh */
            gen_helper_fpu3_fabsh(tcg_env, fpu3_insn);
            break;
        case 0x7: /* fnegh */
            gen_helper_fpu3_fnegh(tcg_env, fpu3_insn);
            break;
        default:
            goto wrong;
            break;
        }
        break;
    case 0xc9: /* half-compare */
        switch (op2) {
        case 0x0: /* fcmpzhsh */
            gen_helper_fpu3_fcmpzhsh(tcg_env, fpu3_insn);
            break;
        case 0x1: /* fcmpzlth */
            gen_helper_fpu3_fcmpzlth(tcg_env, fpu3_insn);
            break;
        case 0x2: /* fcmpzneh */
            gen_helper_fpu3_fcmpzneh(tcg_env, fpu3_insn);
            break;
        case 0x3: /* fcmpzuoh */
            gen_helper_fpu3_fcmpzuoh(tcg_env, fpu3_insn);
            break;
        case 0x4: /* fcmphsh */
            gen_helper_fpu3_fcmphsh(tcg_env, fpu3_insn);
            break;
        case 0x5: /* fcmplth */
            gen_helper_fpu3_fcmplth(tcg_env, fpu3_insn);
            break;
        case 0x6: /* fcmpneh */
            gen_helper_fpu3_fcmpneh(tcg_env, fpu3_insn);
            break;
        case 0x7: /* fcmpuoh */
            gen_helper_fpu3_fcmpuoh(tcg_env, fpu3_insn);
            break;
        default:
            goto wrong;
            break;
        }
        break;

    case 0xca: /* half-mul */
        switch (op2) {
        case 0x0: /* fmulh */
            gen_helper_fpu3_fmulh(tcg_env, fpu3_insn);
            break;
        case 0x1: /* fnmulh */
            gen_helper_fpu3_fnmulh(tcg_env, fpu3_insn);
            break;
        case 0x4: /* fmach */
            gen_helper_fpu3_fmach(tcg_env, fpu3_insn);
            break;
        case 0x5: /* fmsch */
            gen_helper_fpu3_fmsch(tcg_env, fpu3_insn);
            break;
        case 0x6: /* fnmach */
            gen_helper_fpu3_fnmach(tcg_env, fpu3_insn);
            break;
        case 0x7: /* fnmsch */
            gen_helper_fpu3_fnmsch(tcg_env, fpu3_insn);
            break;
        default:
            goto wrong;
            break;
        }
        break;

    case 0xcb: /* half-div */
        switch (op2) {
        case 0x0: /* fdivh */
            gen_helper_fpu3_fdivh(tcg_env, fpu3_insn);
            break;
        case 0x1: /* freciph */
            gen_helper_fpu3_freciph(tcg_env, fpu3_insn);
            break;
        case 0x2: /* fsqrth */
            gen_helper_fpu3_fsqrth(tcg_env, fpu3_insn);
            break;
        default:
            goto wrong;
            break;
        }
        break;

    case 0xcd: /* half-compare */
        switch (op2) {
        case 0x0: /* fmaxnm.16 */
            gen_helper_fpu3_fmaxnm16(tcg_env, fpu3_insn);
            break;
        case 0x1: /* fminnm.16 */
            gen_helper_fpu3_fminnm16(tcg_env, fpu3_insn);
            break;
        case 0x2: /* fcmphz.16 */
            gen_helper_fpu3_fcmphz16(tcg_env, fpu3_insn);
            break;
        case 0x3: /* fcmplsz.16 */
            gen_helper_fpu3_fcmplsz16(tcg_env, fpu3_insn);
            break;
        default:
            goto wrong;
            break;
        }
        break;

    case 0xce: /* half-mul */
        switch (op2) {
        case 0x0: /* ffmula.16 */
            gen_helper_fpu3_ffmula16(tcg_env, fpu3_insn);
            break;
        case 0x1: /* ffmuls.16 */
            gen_helper_fpu3_ffmuls16(tcg_env, fpu3_insn);
            break;
        case 0x2: /* ffnmula.16 */
            gen_helper_fpu3_ffnmula16(tcg_env, fpu3_insn);
            break;
        case 0x3: /* ffnmuls.16 */
            gen_helper_fpu3_ffnmuls16(tcg_env, fpu3_insn);
            break;
        default:
            goto wrong;
            break;
        }
        break;

    case 0xcf: /* half-misc */
        switch (op2) {
        case 0x1: /* fsel.16 */
            gen_helper_fpu3_fsel16(tcg_env, fpu3_insn);
            break;
        default:
            goto wrong;
            break;
        }
        break;
    default:
wrong:
        generate_exception(s, EXCP_CSKY_UDEF);
        qemu_log_mask(LOG_GUEST_ERROR, "unknown fpu insn pc=%x opc=%x\n",
                      s->pc, insn);
        break;
    }
}

static inline void cp(DisasContext *ctx, int cprz, int rx, uint32_t sop,
                       int imm)
{
    switch (sop) {
    case 0x0: /*cprgr*/
        break;
    case 0x1: /*cpwgr*/
        break;
    case 0x2: /*cprcr*/
        break;
    case 0x3: /*cpwcr*/
        break;
    case 0x4: /*cprc*/
        break;
    case 0x8: /*ldcpr*/
        break;
    case 0xa: /*stcpr*/
        break;
    case 0xc: /*cpop*/
        break;
    default:
        generate_exception(ctx, EXCP_CSKY_UDEF);
        break;
    }
}

static void disas_csky_32_insn(CPUCSKYState *env, DisasContext *ctx)
{
    uint32_t insn, op, sop, pcode;
    int rz, rx, ry, imm;

    insn = ctx->insn;
    op = (insn >> 26) & 0xf; /* bits:29-26 */

    switch (op) {
    case 0x0: /*special*/
        rx = (insn >> 16) & 0x1f;
        ry = (insn >> 21) & 0x1f;
        sop = (insn >> 10) & 0x3f;
        rz = insn & 0x1f;
        special(ctx, rx, sop, rz, ry);
        break;
    case 0x1:/*arth_reg*/
        ry = (insn >> 21) & 0x1f;
        rx = (insn >> 16) & 0x1f;
        sop = (insn >> 10) & 0x3f;
        pcode = (insn >> 5) & 0x1f;
        rz = insn & 0x1f;
        arth_reg32(ctx, ry, rx, sop, pcode, rz);
        break;
    case 0x3:/*lrs_srs*/
        rz = (insn >> 21) & 0x1f;
        sop = (insn >> 18) & 0x7;
        imm = insn & 0x3ffff;
        lrs(ctx, rz, sop, imm);
        break;
    case 0x4:/*ldr*/
        ry = (insn >> 21) & 0x1f;
        rx = (insn >> 16) & 0x1f;
        sop = (insn >> 10) & 0x3f;
        pcode = (insn >> 5) & 0x1f;
        rz = insn & 0x1f;
        ldr(ctx, sop, pcode, rz, rx, ry);
        break;
    case 0x5:/*str*/
        ry = (insn >> 21) & 0x1f;
        rx = (insn >> 16) & 0x1f;
        sop = (insn >> 10) & 0x3f;
        pcode = (insn >> 5) & 0x1f;
        rz = insn & 0x1f;
        str(ctx, sop, pcode, rz, rx, ry);
        break;
    case 0x6:/*ldi*/
        rz = (insn >> 21) & 0x1f;
        rx = (insn >> 16) & 0x1f;
        sop = (insn >> 12) & 0xf;
        imm = insn & 0xfff;
        ldi(ctx, sop, rz, rx, imm);
        break;
    case 0x7:/*sti*/
        rz = (insn >> 21) & 0x1f;
        rx = (insn >> 16) & 0x1f;
        sop = (insn >> 12) & 0xf;
        imm = insn & 0xfff;
        sti(ctx, sop, rz, rx, imm);
        break;
    case 0x8:/*bsr*/
        imm = insn & 0x3ffffff;
        tcg_gen_movi_tl(cpu_R[15], ctx->pc + 4);
        bsr32(ctx, imm);
        break;
    case 0x9:/*imm_2op*/
        rz = (insn >> 21) & 0x1f;
        rx = (insn >> 16) & 0x1f;
        sop = (insn >> 12) & 0xf;
        imm = insn & 0xfff;
        imm_2op(ctx, rz, rx, sop, imm);
        break;
    case 0xa:/*imm_1op*/
        sop = (insn >> 21) & 0x1f;
        rx = (insn >> 16) & 0x1f;
        imm = insn & 0xffff;
        imm_1op(ctx, sop, rx, imm);
        break;
    case 0xb:/*ori*/
        check_insn_except(ctx, CPU_E801);
        rz = (insn >> 21) & 0x1f;
        rx = (insn >> 16) & 0x1f;
        imm = insn & 0xffff;
        tcg_gen_ori_tl(cpu_R[rz], cpu_R[rx], imm);
        break;
    case 0xd:/*vfp*/
        if (has_insn(ctx, ABIV2_FPU3)) {
            disas_fpu3_insn(env, ctx, insn);
        } else {
            disas_vfp_insn(env, ctx, insn);
        }
        gen_save_pc(ctx->pc);
        gen_helper_vfp_check_exception(tcg_env);
        break;
    case 0xf:/*cp*/
        rz = (insn >> 21) & 0x1f;
        rx = (insn >> 16) & 0x1f;
        sop = (insn >> 12) & 0xf;
        imm = insn & 0xfff;
        cp(ctx, rz, rx, sop, imm);
        break;
    default:
        generate_exception(ctx, EXCP_CSKY_UDEF);
        break;
    }
}

static void csky_trace_tb_start(CPUCSKYState *env, TranslationBlock *tb)
{
    uint32_t tb_pc = (uint32_t)tb->pc;
    TCGv t0;

    t0 = csky_temp_const_tl(tb_pc);
    gen_helper_trace_tb_start(tcg_env, t0);
}

static void csky_trace_tb_exit(uint32_t subtype, uint32_t offset)
{
    TCGv t0 = csky_temp_const_tl(subtype);
    TCGv t1 = csky_temp_const_tl(offset);

    gen_helper_trace_tb_exit(t0, t1);
}

static void csky_tb_start_tb(CPUCSKYState *env, TranslationBlock *tb)
{
    uint32_t tb_pc = (uint32_t)tb->pc;
    TCGv t0;

    t0 = csky_temp_const_tl(tb_pc);
    gen_helper_tb_trace(tcg_env, t0);
}

static void csky_dump_tb_map(DisasContextBase *dcbase)
{
    uint32_t tb_pc = (uint32_t)dcbase->pc_first;
    uint32_t tb_end = (uint32_t)dcbase->pc_next;
    uint32_t icount = dcbase->num_insns;

    qemu_log_mask(CPU_LOG_EXEC, "tb_map: 0x%.8x 0x%.8x %d\n",
                  tb_pc, tb_end, icount);
}

static TCGOp *jcount_start_insn;
static void gen_csky_jcount_start(DisasContext *dc, CPUCSKYState *env)
{
    TCGv t0 = tcg_temp_new_i32();
    TCGv t1 = tcg_temp_new_i32();
    /* We emit a movi with a dummy immediate argument. Keep the insn index
     * of the movi so that we later (when we know the actual insn count)
     * can update the immediate argument with the actual insn count.  */
    tcg_gen_movi_i32(t1, 0xdeadbeef);
    jcount_start_insn = tcg_last_op();

    tcg_gen_movi_tl(t0, dc->pc);
    if (env->jcount_start != 0) {
        gen_helper_jcount(tcg_env, t0, t1);
    }
    if (0 /* trace unsupported */ & CSKY_TRACE) {
        gen_helper_csky_trace_icount(tcg_env, t0, t1);
    }
}

static void gen_csky_jcount_end(int num_insns)
{
    tcg_set_insn_param(jcount_start_insn, 1, num_insns);
}

/* The trace filter is never enabled without the trace server.  */
static bool trace_match_range(CPUCSKYState *env, uint32_t pc)
{
    return false;
}

static void csky_tr_init_disas_context(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *dc = container_of(dcbase, DisasContext, base);
    CPUCSKYState *env = cpu_env(cs);
    TranslationBlock *tb = dcbase->tb;

    dc->pc = dc->base.pc_first;
    dc->bctm = CSKY_TBFLAG_PSR_BM(tb->flags);
    dc->features = env->features;

#ifndef CONFIG_USER_ONLY
    dc->super = CSKY_TBFLAG_PSR_S(tb->flags);
    dc->trust = CSKY_TBFLAG_PSR_T(tb->flags);
    dc->current_cp = CSKY_TBFLAG_CPID(tb->flags);
    dc->trace_mode = (TraceMode)CSKY_TBFLAG_PSR_TM(tb->flags);
#endif

#ifdef CONFIG_USER_ONLY
    dc->mem_idx = CSKY_USERMODE;
#else
    dc->mem_idx = dc->super;
#endif
    dc->align = (env->features & UNALIGNED_ACCESS) ? 0 : MO_ALIGN;

    dc->next_page_start =
        (dc->base.pc_first & TARGET_PAGE_MASK) + TARGET_PAGE_SIZE;

    cpu_F0s = tcg_temp_new_i32();
    cpu_F1s = tcg_temp_new_i32();
    cpu_F0d = tcg_temp_new_i64();
    cpu_F1d = tcg_temp_new_i64();
}

static void csky_tr_tb_start(DisasContextBase *dcbase, CPUState *cpu)
{
    CPUCSKYState *env = cpu_env(cpu);
    DisasContext *dc = container_of(dcbase, DisasContext, base);
    TranslationBlock *tb = dcbase->tb;

    dc->idly4_counter = env->idly4_counter;
    dc->condexec_cond = env->sce_condexec_bits;

    if (tfilter.enable) {
        dc->trace_match = trace_match_range(env, dc->pc);
    } else {
        dc->trace_match = false;
    }

    if (env->jcount_start != 0 || 0 /* trace unsupported */ & CSKY_TRACE) {
        gen_csky_jcount_start(dc, env);
    }

    if (gen_tb_trace()) {
        csky_trace_tb_start(env, tb);
    }

    if (env->tb_trace == 1 || env->pctrace == 1) {
        csky_tb_start_tb(env, tb);
    }
}

static void csky_tr_insn_start(DisasContextBase *dcbase, CPUState *cpu)
{
    DisasContext *dc = container_of(dcbase, DisasContext, base);

    tcg_gen_insn_start(dc->pc, 0, 0);
}

static bool csky_pre_translate_insn(DisasContext *dc, CPUState *cpu)
{
    CPUCSKYState *env = cpu_env(cpu);
#ifndef CONFIG_USER_ONLY
    dc->cannot_be_traced = 0;
    dc->maybe_change_flow = 0;
#endif
#ifdef CONFIG_USER_ONLY
    if (dc->pc >= 0x80000000) {
        generate_exception(dc, EXCP_CSKY_PRIVILEGE);
        return true;
    }
#endif

    if (env->exit_addr != 0) {
        if (env->exit_addr == dc->pc) {
            generate_exception(dc, EXCP_CSKY_EXIT);
            return true;
        }
    }

    /* if trace range match, break the tb. */
    if ((!dc->trace_match) && tfilter.enable) {
        if (unlikely(trace_match_range(env, dc->pc))) {
            gen_save_pc(dc->pc);
            dc->base.is_jmp = DISAS_UPDATE;
            dc->base.num_insns--;
            return true;
        }
    }

    /* if conditional not exec, pass it. */
    if (dc->condexec_cond != 1) {
        TCGv_i32 t0;
        dc->condlabel = gen_new_label();
        t0 = load_cpu_field(sce_condexec_bits);
        tcg_gen_andi_i32(t0, t0, 1);
        tcg_gen_brcondi_i32(TCG_COND_NE, t0, 0x1, dc->condlabel);
    }

    return false;
}

static void csky_post_translate_insn(DisasContext *dc)
{
    TCGv_i32 t0;
    dc->base.pc_next = dc->pc;

    /* update idly4 in env. */
    if (dc->idly4_counter != 0) {
        dc->idly4_counter--;
        t0 = csky_temp_const_tl(dc->idly4_counter);
        store_cpu_field(t0, idly4_counter);
        gen_save_pc(dc->pc);
        dc->base.is_jmp = DISAS_UPDATE;
    }
    /* if in conditional exec mode, shift cond after translate one insn. */
    if (dc->condexec_cond != 1) {
        gen_set_label(dc->condlabel);
        t0 = load_cpu_field(sce_condexec_bits);
        tcg_gen_shri_i32(t0, t0, 1);
        store_cpu_field(t0, sce_condexec_bits);
        gen_save_pc(dc->pc);
        dc->base.is_jmp = DISAS_UPDATE;
    }

#ifndef CONFIG_USER_ONLY
    if (dc->cannot_be_traced) {
        return;
    }
    if (dc->trace_mode == INST_TRACE_MODE) {
        if (!dc->maybe_change_flow) {
            generate_exception(dc, EXCP_CSKY_TRACE);
        }
    }
#endif

    return;
}

static void csky_tr_translate_insn(DisasContextBase *dcbase, CPUState *cpu)
{
    DisasContext *dc = container_of(dcbase, DisasContext, base);
    CPUCSKYState *env = cpu_env(cpu);

    if (csky_pre_translate_insn(dc, cpu)) {
        return;
    }

    dc->insn = translator_lduw(env, &dc->base, dc->pc);

    if ((dc->insn & 0xc000) != 0xc000) {
        /* 16 bit instruction*/
        disas_csky_16_insn(env, dc);
        dc->pc += 2;
    } else {
        /*32 bit instruction*/
        dc->insn = (dc->insn << 16) | translator_lduw(env, &dc->base, dc->pc + 2);
        disas_csky_32_insn(env, dc);
        dc->pc += 4;
    }

    csky_post_translate_insn(dc);
}

static void csky_tr_tb_stop(DisasContextBase *dcbase, CPUState *cpu)
{
    DisasContext *dc = container_of(dcbase, DisasContext, base);
    CPUCSKYState *env = cpu_env(cpu);

    dc->base.pc_next = dc->pc;

    /* send TB_EXIT trace when tb exit in special situation. */
    if (gen_tb_trace()) {
        if (dc->base.is_jmp == DISAS_NEXT
            || dc->base.is_jmp == DISAS_UPDATE) {
            /* exit on special insns. */
            csky_trace_tb_exit(0x2, dc->pc - dc->base.pc_first);
        } else if (dc->base.is_jmp == DISAS_TOO_MANY) {
            /* exit on too many insns. */
            csky_trace_tb_exit(0x3, dc->pc - dc->base.pc_first);
        }
    }
    {
        switch (dc->base.is_jmp) {
        case DISAS_NEXT:
        case DISAS_TOO_MANY:
            gen_goto_tb(dc, 1, dc->pc);
            break;
        case DISAS_JUMP:
            /* indicate that the hash table must be used to find the next TB */
            tcg_gen_lookup_and_goto_ptr();
            break;
        case DISAS_UPDATE:
            /* special insns like mtcr/wait/nir */
            tcg_gen_exit_tb(NULL, 0);
            break;
        case DISAS_NORETURN:
            /* exception */
        case DISAS_TB_JUMP:
            /* nothing more to generate */
            break;
        default:
            /* indicate that the hash table must be used to find the next TB */
            tcg_gen_exit_tb(NULL, 0);
            break;
        }
    }

    if (env->jcount_start != 0 || 0 /* trace unsupported */ & CSKY_TRACE) {
        gen_csky_jcount_end(dcbase->num_insns);
    }

    if (env->tb_trace == 1) {
        csky_dump_tb_map(dcbase);
    }
}

static const TranslatorOps csky_translator_ops = {
    .init_disas_context = csky_tr_init_disas_context,
    .tb_start           = csky_tr_tb_start,
    .insn_start         = csky_tr_insn_start,
    .translate_insn     = csky_tr_translate_insn,
    .tb_stop            = csky_tr_tb_stop,
};

void csky_translate_code(CPUState *cs, TranslationBlock *tb,
                         int *max_insns, vaddr pc, void *host_pc)
{
    DisasContext dc;

    translator_loop(cs, tb, max_insns, pc, host_pc, &csky_translator_ops,
                    &dc.base, TCG_TYPE_VA);
}

void csky_cpu_dump_state(CPUState *cs, FILE *f, int flags)
{
    CSKYCPU *cpu = CSKY_CPU(cs);
    CPUCSKYState *env = &cpu->env;
    int i;

    for (i = 0; i < 32; i++) {
        qemu_fprintf(f, "R%02d=0x%08x", i, env->regs[i]);
        if ((i % 4) == 3) {
            qemu_fprintf(f, "\n");
        } else {
            qemu_fprintf(f, " ");
        }
    }

    for (i = 0; i < 16; i++) {
        qemu_fprintf(f, "vr%02d=0x%16llx", i,
                    (long long unsigned int)env->vfp.reg[i].fpu[0]);
        if ((i % 3) == 2) {
            qemu_fprintf(f, "\n");
        } else {
            qemu_fprintf(f, " ");
        }
    }

    qemu_fprintf(f, "pc=%08x\n", env->pc);

    env->cp0.psr &= ~0x8000c401;
    env->cp0.psr |= env->psr_s << 31;
    env->cp0.psr |= env->psr_tm << 14;
    env->cp0.psr |= env->psr_bm << 10;
    env->cp0.psr |= env->psr_c;
    qemu_fprintf(f, "psr=%08x\n", env->cp0.psr);
    qemu_fprintf(f, "sp=%08x\n", env->regs[14]);
    qemu_fprintf(f, "spv_sp=%08x\n", env->stackpoint.nt_ssp);
    qemu_fprintf(f, "epsr=%08x ", env->cp0.epsr);
    qemu_fprintf(f, "epc=%08x ", env->cp0.epc);
    qemu_fprintf(f, "cr18=%08x\n", env->cp0.capr);
}

