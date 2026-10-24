/*
 * Blackfin translation
 *
 * Instruction semantics after the Blackfin Processor Programming
 * Reference, Rev. 2.2; encodings after its appendix C.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/qemu-print.h"
#include "cpu.h"
#include "tcg/tcg-op.h"
#include "exec/helper-proto.h"
#include "exec/helper-gen.h"
#include "exec/translator.h"
#include "exec/translation-block.h"
#include "exec/log.h"

#define HELPER_H "helper.h"
#include "exec/helper-info.c.inc"
#undef HELPER_H

typedef struct DisasContext {
    DisasContextBase base;
    CPUBfinState *env;
    uint32_t insn_len;          /* 2, 4 or 8 (multi-issue) */
    uint32_t insn;              /* the 32-bit instruction word */
    bool parallel;              /* translating slot 2 or 3 */
    int slot;                   /* 0 = slot 2 (Group1), 1 = slot 3 (Group2) */
    bool super;
    int mem_idx;
    uint32_t lb[2];             /* loop bottoms this TB was built for */
} DisasContext;

/* jumped away, or needs a lookup of cpu_pc */
#define DISAS_JUMP     DISAS_TARGET_0
/* state changed that the next TB must see (e.g. IPEND) */
#define DISAS_UPDATE   DISAS_TARGET_1
/* cpu_pc is set; leave the TB loop so pending interrupts are taken */
#define DISAS_EXIT     DISAS_TARGET_2
/* as DISAS_EXIT, but cpu_pc is a branch target */
#define DISAS_EXIT_JUMP DISAS_TARGET_3

/* what gen_hwloop() does when a loop goes round */
enum { HWLOOP_GOTO, HWLOOP_SETPC, HWLOOP_DEC };

static void gen_hwloop(DisasContext *ctx, uint32_t pc, int mode);

static TCGv_i32 cpu_reg[BFIN_NREGS];
static TCGv_i32 cpu_astat[32];
static TCGv_i32 cpu_pc;

static const char * const reg_names[BFIN_NREGS] = {
    "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7",
    "p0", "p1", "p2", "p3", "p4", "p5", "sp", "fp",
    "i0", "i1", "i2", "i3", "m0", "m1", "m2", "m3",
    "b0", "b1", "b2", "b3", "l0", "l1", "l2", "l3",
    "a0x", "a0w", "a1x", "a1w", NULL, NULL, NULL, "rets",
    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
    "lc0", "lt0", "lb0", "lc1", "lt1", "lb1", NULL, NULL,
    "usp", "seqstat", "syscfg", "reti", "retx", "retn", "rete", "emudat",
};

static const char * const astat_names[32] = {
    [ASTAT_AZ] = "az", [ASTAT_AN] = "an", [ASTAT_CC] = "cc",
    [ASTAT_AC0_COPY] = "ac0_copy", [ASTAT_V_COPY] = "v_copy",
    [ASTAT_AQ] = "aq", [ASTAT_RND_MOD] = "rnd_mod",
    [ASTAT_AC0] = "ac0", [ASTAT_AC1] = "ac1",
    [ASTAT_AV0] = "av0", [ASTAT_AV0S] = "av0s",
    [ASTAT_AV1] = "av1", [ASTAT_AV1S] = "av1s",
    [ASTAT_V] = "v", [ASTAT_VS] = "vs",
};

void bfin_tcg_init(void)
{
    for (int i = 0; i < BFIN_NREGS; i++) {
        if (reg_names[i]) {
            cpu_reg[i] = tcg_global_mem_new_i32(tcg_env,
                             offsetof(CPUBfinState, reg[i]), reg_names[i]);
        }
    }
    for (int i = 0; i < 32; i++) {
        if (astat_names[i]) {
            cpu_astat[i] = tcg_global_mem_new_i32(tcg_env,
                             offsetof(CPUBfinState, astat[i]), astat_names[i]);
        }
    }
    cpu_pc = tcg_global_mem_new_i32(tcg_env, offsetof(CPUBfinState, pc), "pc");
}

#include "decode-insn16.c.inc"
#include "decode-insn32.c.inc"

/* ------------------------------------------------------------- helpers */

static void gen_exception(DisasContext *ctx, int excause)
{
    tcg_gen_movi_i32(cpu_pc, ctx->base.pc_next);
    gen_helper_excpt(tcg_env, tcg_constant_i32(excause));
    ctx->base.is_jmp = DISAS_NORETURN;
}

#define EXCAUSE_ILLEGAL     0x21
#define EXCAUSE_PRIVILEGE   0x2e

static bool gen_illegal(DisasContext *ctx)
{
    gen_exception(ctx, EXCAUSE_ILLEGAL);
    return true;
}

static bool check_super(DisasContext *ctx)
{
    if (!ctx->super) {
        gen_exception(ctx, EXCAUSE_PRIVILEGE);
        return false;
    }
    return true;
}

static void gen_goto_tb(DisasContext *ctx, int n, uint32_t dest)
{
    if (translator_use_goto_tb(&ctx->base, dest)) {
        tcg_gen_goto_tb(n);
        tcg_gen_movi_i32(cpu_pc, dest);
        tcg_gen_exit_tb(ctx->base.tb, n);
    } else {
        tcg_gen_movi_i32(cpu_pc, dest);
        tcg_gen_lookup_and_goto_ptr();
    }
}

/* jump to a constant target; this ends the instruction */
static void gen_jump_imm(DisasContext *ctx, uint32_t dest)
{
    tcg_gen_movi_i32(cpu_pc, dest);
    ctx->base.is_jmp = DISAS_JUMP;
}

static void gen_jump_reg(DisasContext *ctx, TCGv_i32 dest)
{
    tcg_gen_andi_i32(cpu_pc, dest, ~1u);
    ctx->base.is_jmp = DISAS_JUMP;
}

/* the address of this instruction */
static uint32_t insn_pc(DisasContext *ctx)
{
    return ctx->base.pc_next;
}

/* -------------------------------------------------- unimplemented yet */

#define UNIMPL(name) \
    static bool trans_##name(DisasContext *ctx, arg_##name *a) \
    { \
        qemu_log_mask(LOG_UNIMP, "bfin: " #name " not implemented at %08x\n", \
                      insn_pc(ctx)); \
        return false; \
    }


/* ----------------------------------------------------- register file */

/* operand classes of the allreg encoding, chapter 9 "Move Register" */
static bool is_dreg(int r)   { return r < 8; }
static bool is_preg(int r)   { return r >= 8 && r < 16; }
static bool is_areg(int r)   { return r >= BFIN_A0X && r <= BFIN_A1W; }
static bool is_genreg(int r) { return r < 16 || is_areg(r); }
static bool is_dagreg(int r) { return r >= 16 && r < 32; }
static bool is_usp(int r)    { return r == BFIN_USP; }

static bool is_sysreg(int r)
{
    return r == BFIN_ASTAT || r == BFIN_RETS ||
           (r >= BFIN_LC0 && r <= BFIN_CYCLES2) ||
           (r >= BFIN_SEQSTAT && r <= BFIN_EMUDAT);
}

/* explicit accesses that need supervisor mode (chapter 9) */
static bool is_protected_reg(int r)
{
    return r == BFIN_USP || (r >= BFIN_SEQSTAT && r <= BFIN_RETE);
}

/* read any allreg register; A0.X/A1.X are sign extended from 8 bits */
static TCGv_i32 gen_get_reg(DisasContext *ctx, int r)
{
    TCGv_i32 t = tcg_temp_new_i32();

    switch (r) {
    case BFIN_A0X:
    case BFIN_A1X:
        tcg_gen_ext8s_i32(t, cpu_reg[r]);
        break;
    case BFIN_ASTAT:
        gen_helper_get_astat(t, tcg_env);
        break;
    case BFIN_CYCLES:
    case BFIN_CYCLES2:
        gen_helper_get_cycles(t, tcg_env, tcg_constant_i32(r == BFIN_CYCLES2));
        break;
    default:
        tcg_gen_mov_i32(t, cpu_reg[r]);
        break;
    }
    return t;
}

static void gen_set_reg(DisasContext *ctx, int r, TCGv_i32 val)
{
    switch (r) {
    case BFIN_A0X:
    case BFIN_A1X:
        tcg_gen_andi_i32(cpu_reg[r], val, 0xff);
        break;
    case BFIN_ASTAT:
        gen_helper_set_astat(tcg_env, val);
        break;
    case BFIN_CYCLES:
    case BFIN_CYCLES2:
        gen_helper_set_cycles(tcg_env, tcg_constant_i32(r == BFIN_CYCLES2),
                              val);
        break;
    case BFIN_LB0:
    case BFIN_LB1:
        /* the loop bottoms are part of the TB key */
        tcg_gen_mov_i32(cpu_reg[r], val);
        if (ctx->base.is_jmp == DISAS_NEXT) {
            ctx->base.is_jmp = DISAS_UPDATE;
        }
        break;
    default:
        tcg_gen_mov_i32(cpu_reg[r], val);
        break;
    }
}

/* genreg = genreg, dagreg, USP; dagreg = genreg, dagreg; USP = genreg;
   Dreg, Preg = sysreg; sysreg = Dreg, Preg, USP */
static bool regmv_ok(int dst, int src)
{
    if (is_genreg(dst) && (is_genreg(src) || is_dagreg(src) || is_usp(src))) {
        return true;
    }
    if (is_dagreg(dst) && (is_genreg(src) || is_dagreg(src))) {
        return true;
    }
    if (is_usp(dst) && is_genreg(src)) {
        return true;
    }
    if ((is_dreg(dst) || is_preg(dst)) && is_sysreg(src)) {
        return true;
    }
    if (is_sysreg(dst) && (is_dreg(src) || is_preg(src) || is_usp(src))) {
        return true;
    }
    return false;
}

static bool trans_REGMV(DisasContext *ctx, arg_REGMV *a)
{
    int dst = (a->gd << 3) | a->dst;
    int src = (a->gs << 3) | a->src;

    if (ctx->parallel || !regmv_ok(dst, src)) {
        return false;
    }
    if ((is_protected_reg(dst) || is_protected_reg(src)) && !check_super(ctx)) {
        return true;
    }
    gen_set_reg(ctx, dst, gen_get_reg(ctx, src));
    return true;
}

/* reg.L = uimm16, reg = imm16 (X), reg.H = uimm16, reg = uimm16 (Z) */
static bool trans_LDIMMhalf(DisasContext *ctx, arg_LDIMMhalf *a)
{
    int r = (a->grp << 3) | a->reg;
    TCGv_i32 t;

    switch ((a->z << 2) | (a->h << 1) | a->s) {
    case 0:                                         /* reg.L = uimm16 */
        t = tcg_temp_new_i32();
        tcg_gen_deposit_i32(t, cpu_reg[r], tcg_constant_i32(a->hword), 0, 16);
        break;
    case 1:                                         /* reg = imm16 (X) */
        t = tcg_constant_i32((int16_t)a->hword);
        break;
    case 2:                                         /* reg.H = uimm16 */
        t = tcg_temp_new_i32();
        tcg_gen_deposit_i32(t, cpu_reg[r], tcg_constant_i32(a->hword), 16, 16);
        break;
    case 4:                                         /* reg = uimm16 (Z) */
        t = tcg_constant_i32(a->hword);
        break;
    default:
        return false;
    }
    gen_set_reg(ctx, r, t);
    return true;
}

/* -------------------------------------------------------- load/store */

static void gen_ld(DisasContext *ctx, TCGv_i32 dst, TCGv_i32 addr, MemOp mop)
{
    tcg_gen_qemu_ld_i32(dst, addr, ctx->mem_idx, mop | MO_LE | MO_ALIGN);
}

static void gen_st(DisasContext *ctx, TCGv_i32 val, TCGv_i32 addr, MemOp mop)
{
    tcg_gen_qemu_st_i32(val, addr, ctx->mem_idx, mop | MO_LE | MO_ALIGN);
}

/* a copy of the address register, taken before any post-modify */
static TCGv_i32 addr_of(TCGv_i32 reg)
{
    TCGv_i32 t = tcg_temp_new_i32();

    tcg_gen_mov_i32(t, reg);
    return t;
}

/*
 * Ireg += Mval with circular buffer wraparound (chapter 5, "Addressing
 * Circular Buffers"); L = 0 means linear.
 */
static void gen_dag_add(int i, TCGv_i32 m)
{
    gen_helper_dag_add(cpu_reg[BFIN_I0 + i], cpu_reg[BFIN_I0 + i], m,
                       cpu_reg[BFIN_B0 + i], cpu_reg[BFIN_L0 + i]);
}

static void gen_dag_addi(int i, int32_t m)
{
    gen_dag_add(i, tcg_constant_i32(m));
}

/* sz: 0 word, 1 half, 2 byte */
static const MemOp sz_mop[3] = { MO_32, MO_16, MO_8 };

/*
 * LDST: [Preg], [Preg++], [Preg--] in all sizes.  Table C-22:
 * aop 3 is not used; for words Z selects a Preg destination/source,
 * for loads of halves and bytes it selects sign extension; stores of
 * halves and bytes have no Z form.
 */
static bool trans_LDST(DisasContext *ctx, arg_LDST *a)
{
    TCGv_i32 ptr = cpu_reg[BFIN_P0 + a->ptr];
    TCGv_i32 addr;
    int reg = a->reg;
    int step = 4 >> a->sz;
    MemOp mop;

    if (a->sz == 3 || a->aop == 3) {
        return false;
    }
    if (a->sz == 0) {
        mop = MO_32;
        if (a->z) {
            reg += 8;                               /* Preg */
        }
    } else {
        if (a->w && a->z) {
            return false;
        }
        mop = sz_mop[a->sz] | (a->z ? MO_SIGN : 0);
    }
    /* Preg = [Preg++/--] with the same register is undefined */
    if (!a->w && reg == BFIN_P0 + a->ptr && a->aop != 2) {
        return false;
    }
    if (ctx->parallel && ctx->slot != 0) {
        return false;                               /* Group1 only */
    }
    addr = addr_of(ptr);
    if (a->w) {
        gen_st(ctx, cpu_reg[reg], addr, mop);
    } else {
        gen_ld(ctx, cpu_reg[reg], addr, mop);
    }
    if (a->aop == 0) {
        tcg_gen_addi_i32(ptr, ptr, step);
    } else if (a->aop == 1) {
        tcg_gen_subi_i32(ptr, ptr, step);
    }
    return true;
}

/*
 * LDSTii: [Preg + uimm6m4] word, W[Preg + uimm5m2] (Z)/(X), Preg words.
 * Stores: op 0 Dreg, op 1 half, op 3 Preg (op 2 is LDSTiiFP).
 */
static bool trans_LDSTii(DisasContext *ctx, arg_LDSTii *a)
{
    static const MemOp mops[4] = { MO_32, MO_16, MO_16 | MO_SIGN, MO_32 };
    TCGv_i32 addr = tcg_temp_new_i32();
    int reg = a->reg + (a->op == 3 ? 8 : 0);
    int off = a->offset << (a->op == 1 || a->op == 2 ? 1 : 2);

    if (a->w && a->op == 2) {
        return false;
    }
    if (ctx->parallel && ctx->slot != 0) {
        return false;
    }
    tcg_gen_addi_i32(addr, cpu_reg[BFIN_P0 + a->ptr], off);
    if (a->w) {
        gen_st(ctx, cpu_reg[reg], addr, mops[a->op] & ~MO_SIGN);
    } else {
        gen_ld(ctx, cpu_reg[reg], addr, mops[a->op]);
    }
    return true;
}

/* Dreg/Preg = [FP - uimm7m4], [FP - uimm7m4] = Dreg/Preg */
static bool trans_LDSTiiFP(DisasContext *ctx, arg_LDSTiiFP *a)
{
    TCGv_i32 addr = tcg_temp_new_i32();

    if (ctx->parallel && ctx->slot != 0) {
        return false;
    }
    /* the 5-bit field is the negative offset in words, -32..-1 */
    tcg_gen_addi_i32(addr, cpu_reg[BFIN_FP], (a->offset - 32) * 4);
    if (a->w) {
        gen_st(ctx, cpu_reg[a->reg], addr, MO_32);
    } else {
        gen_ld(ctx, cpu_reg[a->reg], addr, MO_32);
    }
    return true;
}

/*
 * LDSTidxI: [Preg +/- uimm17m4], W[Preg +/- uimm16m2], B[Preg +/- uimm15].
 * Z selects Preg for words and sign extension for halves and bytes;
 * stores of halves and bytes have no Z form.
 */
static bool trans_LDSTidxI(DisasContext *ctx, arg_LDSTidxI *a)
{
    TCGv_i32 addr = tcg_temp_new_i32();
    int reg = a->reg;
    MemOp mop;

    if (a->sz == 3) {
        return false;
    }
    if (a->sz == 0) {
        mop = MO_32;
        if (a->z) {
            reg += 8;
        }
    } else {
        if (a->w && a->z) {
            return false;
        }
        mop = sz_mop[a->sz] | (a->z ? MO_SIGN : 0);
    }
    tcg_gen_addi_i32(addr, cpu_reg[BFIN_P0 + a->ptr], a->offset << (2 - a->sz));
    if (a->w) {
        gen_st(ctx, cpu_reg[reg], addr, mop);
    } else {
        gen_ld(ctx, cpu_reg[reg], addr, mop);
    }
    return true;
}

/*
 * LDSTpmod: [Preg ++ Preg] forms.  With the same register for both
 * operands the post-increment does not happen ("Indirect and
 * Post-Increment Index Addressing", chapter 8); for the half register
 * forms that is the W[Preg] encoding.
 *   W=0: aop 0 Dreg = [], 1 Dreg_lo = W[], 2 Dreg_hi = W[], 3 Dreg = W[] (Z)
 *   W=1: aop 0 [] = Dreg, 1 W[] = Dreg_lo, 2 W[] = Dreg_hi, 3 Dreg = W[] (X)
 */
static bool trans_LDSTpmod(DisasContext *ctx, arg_LDSTpmod *a)
{
    TCGv_i32 ptr = cpu_reg[BFIN_P0 + a->ptr];
    TCGv_i32 dreg = cpu_reg[BFIN_R0 + a->reg];
    TCGv_i32 addr = addr_of(ptr);
    TCGv_i32 t;

    if (ctx->parallel && ctx->slot != 0) {
        return false;
    }
    switch ((a->w << 2) | a->aop) {
    case 0:
        gen_ld(ctx, dreg, addr, MO_32);
        break;
    case 1:
    case 2:
        t = tcg_temp_new_i32();
        gen_ld(ctx, t, addr, MO_16);
        tcg_gen_deposit_i32(dreg, dreg, t, a->aop == 1 ? 0 : 16, 16);
        break;
    case 3:
        gen_ld(ctx, dreg, addr, MO_16);
        break;
    case 4:
        gen_st(ctx, dreg, addr, MO_32);
        break;
    case 5:
        gen_st(ctx, dreg, addr, MO_16);
        break;
    case 6:
        t = tcg_temp_new_i32();
        tcg_gen_shri_i32(t, dreg, 16);
        gen_st(ctx, t, addr, MO_16);
        break;
    case 7:
        gen_ld(ctx, dreg, addr, MO_16 | MO_SIGN);
        break;
    }
    if (a->idx != a->ptr) {
        tcg_gen_add_i32(ptr, ptr, cpu_reg[BFIN_P0 + a->idx]);
    }
    return true;
}

/*
 * dspLDST: Ireg addressed words and halves.
 *   aop 0 [Ireg++], 1 [Ireg--], 2 [Ireg], 3 [Ireg ++ Mreg] (words only)
 *   m 0 word, 1 low half, 2 high half
 * Both parallel slots accept these.
 */
static bool trans_dspLDST(DisasContext *ctx, arg_dspLDST *a)
{
    TCGv_i32 ireg = cpu_reg[BFIN_I0 + a->i];
    TCGv_i32 dreg = cpu_reg[BFIN_R0 + a->reg];
    TCGv_i32 addr = addr_of(ireg);
    TCGv_i32 t;
    int step;

    if (a->aop == 3) {
        step = 0;                                   /* m selects Mreg */
    } else if (a->m == 3) {
        return false;
    } else {
        step = a->m ? 2 : 4;
    }
    if (a->aop == 3 || a->m == 0) {
        if (a->w) {
            gen_st(ctx, dreg, addr, MO_32);
        } else {
            gen_ld(ctx, dreg, addr, MO_32);
        }
    } else if (a->w) {
        t = tcg_temp_new_i32();
        tcg_gen_shri_i32(t, dreg, a->m == 2 ? 16 : 0);
        gen_st(ctx, t, addr, MO_16);
    } else {
        t = tcg_temp_new_i32();
        gen_ld(ctx, t, addr, MO_16);
        tcg_gen_deposit_i32(dreg, dreg, t, a->m == 2 ? 16 : 0, 16);
    }
    switch (a->aop) {
    case 0:
        gen_dag_addi(a->i, step);
        break;
    case 1:
        gen_dag_addi(a->i, -step);
        break;
    case 3:
        gen_dag_add(a->i, cpu_reg[BFIN_M0 + a->m]);
        break;
    }
    return true;
}

/* Ireg += Mreg, Ireg -= Mreg, Ireg += Mreg (brev) */
static bool trans_dagMODim(DisasContext *ctx, arg_dagMODim *a)
{
    TCGv_i32 m = cpu_reg[BFIN_M0 + a->m];

    if (ctx->parallel && ctx->slot != 0) {
        return false;
    }
    if (a->br) {
        if (a->op) {
            return false;                           /* no brev decrement */
        }
        gen_helper_dag_brev(cpu_reg[BFIN_I0 + a->i], cpu_reg[BFIN_I0 + a->i],
                            m);
        return true;
    }
    if (a->op) {
        TCGv_i32 t = tcg_temp_new_i32();

        tcg_gen_neg_i32(t, m);
        gen_dag_add(a->i, t);
    } else {
        gen_dag_add(a->i, m);
    }
    return true;
}

/* Ireg += 2, Ireg -= 2, Ireg += 4, Ireg -= 4 */
static bool trans_dagMODik(DisasContext *ctx, arg_dagMODik *a)
{
    static const int8_t delta[4] = { 2, -2, 4, -4 };

    if (ctx->parallel && ctx->slot != 0) {
        return false;
    }
    gen_dag_addi(a->i, delta[a->op]);
    return true;
}

/* ------------------------------------------------------------- stack */

static bool is_mostreg(int r)
{
    return !(r < 16 || r == BFIN_CYCLES || r == BFIN_CYCLES2 ||
             r == BFIN_A0X + 4 || r == BFIN_A0X + 5 ||
             (r >= 40 && r < 48));
}

static bool is_stack_protected(int r)
{
    return is_protected_reg(r) || r == BFIN_EMUDAT;
}

/* [--SP] = allreg ; mostreg = [SP++] (chapter 10) */
static bool trans_PushPopReg(DisasContext *ctx, arg_PushPopReg *a)
{
    int r = (a->grp << 3) | a->reg;
    TCGv_i32 sp = cpu_reg[BFIN_SP];

    if (ctx->parallel) {
        return false;
    }
    if (a->w) {
        /* allreg: everything but SP and the reserved encodings */
        if (r == BFIN_SP || r == BFIN_A0X + 4 || r == BFIN_A0X + 5 ||
            (r >= 40 && r < 48)) {
            return false;
        }
    } else if (!is_mostreg(r)) {
        return false;
    }
    if (is_stack_protected(r) && !check_super(ctx)) {
        return true;
    }
    if (a->w) {
        TCGv_i32 addr = tcg_temp_new_i32();

        tcg_gen_subi_i32(addr, sp, 4);
        gen_st(ctx, gen_get_reg(ctx, r), addr, MO_32);
        tcg_gen_mov_i32(sp, addr);
        if (r == BFIN_RETI) {
            /* saving RETI re-enables interrupts */
            gen_helper_reti_saved(tcg_env, tcg_constant_i32(1));
            tcg_gen_movi_i32(cpu_pc, insn_pc(ctx) + 2);
            ctx->base.is_jmp = DISAS_EXIT;
        }
    } else {
        TCGv_i32 t = tcg_temp_new_i32();

        gen_ld(ctx, t, sp, MO_32);
        tcg_gen_addi_i32(sp, sp, 4);
        gen_set_reg(ctx, r, t);
        if (r == BFIN_RETI) {
            gen_helper_reti_saved(tcg_env, tcg_constant_i32(0));
        }
    }
    return true;
}

/*
 * [--SP] = (R7:dr, P5:pr) saves the lowest index first, data registers
 * before pointer registers; the pop goes the other way round.
 */
static bool trans_PushPopMultiple(DisasContext *ctx, arg_PushPopMultiple *a)
{
    TCGv_i32 sp = cpu_reg[BFIN_SP];
    TCGv_i32 addr;

    if (ctx->parallel || (!a->d && !a->p) || a->pr > 5) {
        return false;
    }
    addr = addr_of(sp);
    if (a->w) {
        if (a->d) {
            for (int i = a->dr; i <= 7; i++) {
                tcg_gen_subi_i32(addr, addr, 4);
                gen_st(ctx, cpu_reg[BFIN_R0 + i], addr, MO_32);
            }
        }
        if (a->p) {
            for (int i = a->pr; i <= 5; i++) {
                tcg_gen_subi_i32(addr, addr, 4);
                gen_st(ctx, cpu_reg[BFIN_P0 + i], addr, MO_32);
            }
        }
        tcg_gen_mov_i32(sp, addr);
    } else {
        /* load into temporaries first so an exception leaves no trace */
        TCGv_i32 val[14];
        int n = 0;

        if (a->p) {
            for (int i = 5; i >= a->pr; i--) {
                val[n] = tcg_temp_new_i32();
                gen_ld(ctx, val[n++], addr, MO_32);
                tcg_gen_addi_i32(addr, addr, 4);
            }
        }
        if (a->d) {
            for (int i = 7; i >= a->dr; i--) {
                val[n] = tcg_temp_new_i32();
                gen_ld(ctx, val[n++], addr, MO_32);
                tcg_gen_addi_i32(addr, addr, 4);
            }
        }
        n = 0;
        if (a->p) {
            for (int i = 5; i >= a->pr; i--) {
                tcg_gen_mov_i32(cpu_reg[BFIN_P0 + i], val[n++]);
            }
        }
        if (a->d) {
            for (int i = 7; i >= a->dr; i--) {
                tcg_gen_mov_i32(cpu_reg[BFIN_R0 + i], val[n++]);
            }
        }
        tcg_gen_mov_i32(sp, addr);
    }
    return true;
}

/* LINK uimm18m4 ; UNLINK */
static bool trans_linkage(DisasContext *ctx, arg_linkage *a)
{
    TCGv_i32 sp = cpu_reg[BFIN_SP];
    TCGv_i32 fp = cpu_reg[BFIN_FP];
    TCGv_i32 addr = tcg_temp_new_i32();

    if (a->r) {
        TCGv_i32 nfp = tcg_temp_new_i32();
        TCGv_i32 rets = tcg_temp_new_i32();

        if (a->framesize) {
            return false;
        }
        gen_ld(ctx, nfp, fp, MO_32);
        tcg_gen_addi_i32(addr, fp, 4);
        gen_ld(ctx, rets, addr, MO_32);
        tcg_gen_addi_i32(sp, fp, 8);
        tcg_gen_mov_i32(fp, nfp);
        tcg_gen_mov_i32(cpu_reg[BFIN_RETS], rets);
    } else {
        tcg_gen_subi_i32(addr, sp, 4);
        gen_st(ctx, cpu_reg[BFIN_RETS], addr, MO_32);
        tcg_gen_subi_i32(addr, addr, 4);
        gen_st(ctx, fp, addr, MO_32);
        tcg_gen_mov_i32(fp, addr);
        tcg_gen_subi_i32(sp, addr, a->framesize * 4);
    }
    return true;
}

/* ------------------------------------------------------ status bits */

static void gen_astat_clr(int bit)
{
    tcg_gen_movi_i32(cpu_astat[bit], 0);
}

/* an instruction result in AC0 or V also lands in its copy bit */
static void gen_ac0_copy(void)
{
    tcg_gen_mov_i32(cpu_astat[ASTAT_AC0_COPY], cpu_astat[ASTAT_AC0]);
}

static void gen_az_an(TCGv_i32 res)
{
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_astat[ASTAT_AZ], res, 0);
    tcg_gen_shri_i32(cpu_astat[ASTAT_AN], res, 31);
}

/* V from v (0/1); VS is sticky */
static void gen_v(TCGv_i32 v)
{
    tcg_gen_mov_i32(cpu_astat[ASTAT_V], v);
    tcg_gen_mov_i32(cpu_astat[ASTAT_V_COPY], v);
    tcg_gen_or_i32(cpu_astat[ASTAT_VS], cpu_astat[ASTAT_VS], v);
}

/* AZ, AN, AC0, V, VS of res = a + b */
static void gen_add_flags(TCGv_i32 res, TCGv_i32 a, TCGv_i32 b)
{
    TCGv_i32 t = tcg_temp_new_i32();
    TCGv_i32 u = tcg_temp_new_i32();

    gen_az_an(res);
    tcg_gen_setcond_i32(TCG_COND_LTU, cpu_astat[ASTAT_AC0], res, a);
    gen_ac0_copy();
    tcg_gen_xor_i32(t, res, a);
    tcg_gen_xor_i32(u, res, b);
    tcg_gen_and_i32(t, t, u);
    tcg_gen_shri_i32(t, t, 31);
    gen_v(t);
}

/* AZ, AN, AC0 (no borrow), V, VS of res = a - b */
static void gen_sub_flags(TCGv_i32 res, TCGv_i32 a, TCGv_i32 b)
{
    TCGv_i32 t = tcg_temp_new_i32();
    TCGv_i32 u = tcg_temp_new_i32();

    gen_az_an(res);
    tcg_gen_setcond_i32(TCG_COND_GEU, cpu_astat[ASTAT_AC0], a, b);
    gen_ac0_copy();
    tcg_gen_xor_i32(t, a, b);
    tcg_gen_xor_i32(u, res, a);
    tcg_gen_and_i32(t, t, u);
    tcg_gen_shri_i32(t, t, 31);
    gen_v(t);
}

/* shifts: AZ and AN from res, V cleared, AC0 untouched */
static void gen_shift_flags(TCGv_i32 res)
{
    gen_az_an(res);
    gen_astat_clr(ASTAT_V);
    gen_astat_clr(ASTAT_V_COPY);
}

/* AZ and AN from res, AC0 and V cleared: logical results */
static void gen_logic_flags(TCGv_i32 res)
{
    gen_az_an(res);
    gen_astat_clr(ASTAT_AC0);
    gen_astat_clr(ASTAT_AC0_COPY);
    gen_astat_clr(ASTAT_V);
    gen_astat_clr(ASTAT_V_COPY);
}

/* ---------------------------------------------------- 16-bit compute */

/* Dreg >>>= Dreg, >>= , <<= : the whole 32-bit magnitude counts */
static void gen_shift_reg(TCGv_i32 d, TCGv_i32 n, int kind)
{
    TCGv_i32 big = tcg_temp_new_i32();
    TCGv_i32 sh = tcg_temp_new_i32();
    TCGv_i32 res = tcg_temp_new_i32();
    TCGv_i32 fill = tcg_temp_new_i32();

    tcg_gen_andi_i32(sh, n, 31);
    switch (kind) {
    case 0:
        tcg_gen_sar_i32(res, d, sh);
        tcg_gen_sari_i32(fill, d, 31);
        break;
    case 1:
        tcg_gen_shr_i32(res, d, sh);
        tcg_gen_movi_i32(fill, 0);
        break;
    default:
        tcg_gen_shl_i32(res, d, sh);
        tcg_gen_movi_i32(fill, 0);
        break;
    }
    tcg_gen_setcondi_i32(TCG_COND_GTU, big, n, 31);
    tcg_gen_movcond_i32(TCG_COND_NE, d, big, tcg_constant_i32(0), fill, res);
}

static bool trans_ALU2op(DisasContext *ctx, arg_ALU2op *a)
{
    TCGv_i32 d = cpu_reg[BFIN_R0 + a->dst];
    TCGv_i32 s = cpu_reg[BFIN_R0 + a->src];
    TCGv_i32 a0 = tcg_temp_new_i32();
    TCGv_i32 t = tcg_temp_new_i32();

    if (ctx->parallel) {
        return false;
    }
    switch (a->opc) {
    case 0:                                         /* Dreg >>>= Dreg */
    case 1:                                         /* Dreg >>= Dreg */
    case 2:                                         /* Dreg <<= Dreg */
        gen_shift_reg(d, s, a->opc);
        gen_shift_flags(d);
        return true;
    case 3:                                         /* Dreg *= Dreg */
        tcg_gen_mul_i32(d, d, s);
        return true;
    case 4:                                         /* (Dreg + Dreg) << 1 */
    case 5:                                         /* (Dreg + Dreg) << 2 */
        {
            int n = a->opc == 4 ? 1 : 2;
            TCGv_i64 w = tcg_temp_new_i64();
            TCGv_i64 x = tcg_temp_new_i64();
            TCGv_i32 v = tcg_temp_new_i32();

            /* exact sum shifted, V if it does not fit in 32 bits */
            tcg_gen_ext_i32_i64(w, d);
            tcg_gen_ext_i32_i64(x, s);
            tcg_gen_add_i64(w, w, x);
            tcg_gen_shli_i64(w, w, n);
            tcg_gen_extrl_i64_i32(d, w);
            tcg_gen_ext_i32_i64(x, d);
            tcg_gen_setcond_i64(TCG_COND_NE, x, x, w);
            tcg_gen_extrl_i64_i32(v, x);
            gen_az_an(d);
            gen_v(v);
        }
        return true;
    case 8:                                         /* DIVQ */
    case 9:                                         /* DIVS */
        gen_helper_div(d, tcg_env, d, s, tcg_constant_i32(a->opc == 9));
        return true;
    case 10:                                        /* Dreg = Dreg_lo (X) */
        tcg_gen_ext16s_i32(d, s);
        gen_logic_flags(d);
        return true;
    case 11:                                        /* Dreg = Dreg_lo (Z) */
        tcg_gen_ext16u_i32(d, s);
        gen_logic_flags(d);
        return true;
    case 12:                                        /* Dreg = Dreg_byte (X) */
        tcg_gen_ext8s_i32(d, s);
        gen_logic_flags(d);
        return true;
    case 13:                                        /* Dreg = Dreg_byte (Z) */
        tcg_gen_ext8u_i32(d, s);
        gen_logic_flags(d);
        return true;
    case 14:                                        /* Dreg = -Dreg */
        tcg_gen_mov_i32(a0, s);
        tcg_gen_neg_i32(d, s);
        gen_az_an(d);
        tcg_gen_setcondi_i32(TCG_COND_EQ, t, a0, INT32_MIN);
        gen_v(t);
        tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_astat[ASTAT_AC0], a0, 0);
        gen_ac0_copy();
        return true;
    case 15:                                        /* Dreg = ~Dreg */
        tcg_gen_not_i32(d, s);
        gen_logic_flags(d);
        return true;
    }
    return false;                                   /* opc 6, 7 */
}

/* pointer arithmetic, no status bits */
static bool trans_PTR2op(DisasContext *ctx, arg_PTR2op *a)
{
    TCGv_i32 d = cpu_reg[BFIN_P0 + a->dst];
    TCGv_i32 s = cpu_reg[BFIN_P0 + a->src];

    if (ctx->parallel) {
        return false;
    }
    switch (a->opc) {
    case 0:
        tcg_gen_sub_i32(d, d, s);
        return true;
    case 1:
        tcg_gen_shli_i32(d, s, 2);
        return true;
    case 3:
        tcg_gen_shri_i32(d, s, 2);
        return true;
    case 4:
        tcg_gen_shri_i32(d, s, 1);
        return true;
    case 5:
        gen_helper_dag_brev(d, d, s);
        return true;
    case 6:
    case 7:
        tcg_gen_add_i32(d, d, s);
        tcg_gen_shli_i32(d, d, a->opc == 6 ? 1 : 2);
        return true;
    }
    return false;
}

/* bit operations and shifts by uimm5 */
static bool trans_LOGI2op(DisasContext *ctx, arg_LOGI2op *a)
{
    TCGv_i32 d = cpu_reg[BFIN_R0 + a->dst];
    int n = a->src;

    if (ctx->parallel) {
        return false;
    }
    switch (a->opc) {
    case 0:                                         /* CC = !BITTST */
    case 1:                                         /* CC = BITTST */
        tcg_gen_extract_i32(cpu_astat[ASTAT_CC], d, n, 1);
        if (a->opc == 0) {
            tcg_gen_xori_i32(cpu_astat[ASTAT_CC], cpu_astat[ASTAT_CC], 1);
        }
        return true;
    case 2:                                         /* BITSET */
        tcg_gen_ori_i32(d, d, 1u << n);
        gen_logic_flags(d);
        gen_astat_clr(ASTAT_AZ);
        return true;
    case 3:                                         /* BITTGL */
        tcg_gen_xori_i32(d, d, 1u << n);
        gen_logic_flags(d);
        return true;
    case 4:                                         /* BITCLR */
        tcg_gen_andi_i32(d, d, ~(1u << n));
        gen_logic_flags(d);
        return true;
    case 5:                                         /* Dreg >>>= uimm5 */
        tcg_gen_sari_i32(d, d, n);
        break;
    case 6:                                         /* Dreg >>= uimm5 */
        tcg_gen_shri_i32(d, d, n);
        break;
    case 7:                                         /* Dreg <<= uimm5 */
        tcg_gen_shli_i32(d, d, n);
        break;
    }
    gen_shift_flags(d);
    return true;
}

/* three operand add/sub/logic on Dregs, add/shift-add on Pregs */
static bool trans_COMP3op(DisasContext *ctx, arg_COMP3op *a)
{
    TCGv_i32 res = tcg_temp_new_i32();
    TCGv_i32 x, y;

    if (ctx->parallel) {
        return false;
    }
    if (a->opc >= 5) {
        x = cpu_reg[BFIN_P0 + a->src0];
        y = cpu_reg[BFIN_P0 + a->src1];
        tcg_gen_shli_i32(res, y, a->opc - 5);
        tcg_gen_add_i32(cpu_reg[BFIN_P0 + a->dst], x, res);
        return true;
    }
    x = cpu_reg[BFIN_R0 + a->src0];
    y = cpu_reg[BFIN_R0 + a->src1];
    switch (a->opc) {
    case 0:
        tcg_gen_add_i32(res, x, y);
        gen_add_flags(res, x, y);
        break;
    case 1:
        tcg_gen_sub_i32(res, x, y);
        gen_sub_flags(res, x, y);
        break;
    case 2:
        tcg_gen_and_i32(res, x, y);
        gen_logic_flags(res);
        break;
    case 3:
        tcg_gen_or_i32(res, x, y);
        gen_logic_flags(res);
        break;
    case 4:
        tcg_gen_xor_i32(res, x, y);
        gen_logic_flags(res);
        break;
    }
    tcg_gen_mov_i32(cpu_reg[BFIN_R0 + a->dst], res);
    return true;
}

/* Dreg = imm7 (X) ; Dreg += imm7 */
static bool trans_COMPI2opD(DisasContext *ctx, arg_COMPI2opD *a)
{
    TCGv_i32 d = cpu_reg[BFIN_R0 + a->dst];

    if (ctx->parallel) {
        return false;
    }
    if (a->op) {
        TCGv_i32 old = addr_of(d);
        TCGv_i32 imm = tcg_constant_i32(a->isrc);

        tcg_gen_addi_i32(d, d, a->isrc);
        gen_add_flags(d, old, imm);
    } else {
        tcg_gen_movi_i32(d, a->isrc);
    }
    return true;
}

/* Preg = imm7 (X) ; Preg += imm7 */
static bool trans_COMPI2opP(DisasContext *ctx, arg_COMPI2opP *a)
{
    TCGv_i32 d = cpu_reg[BFIN_P0 + a->dst];

    if (ctx->parallel) {
        return false;
    }
    if (a->op) {
        tcg_gen_addi_i32(d, d, a->src);
    } else {
        tcg_gen_movi_i32(d, a->src);
    }
    return true;
}

/*
 * CC = x == y, x < y, x <= y, unsigned (IU), against a register or a
 * 3-bit immediate; Pregs with G.  The Dreg versions also set AZ, AN and
 * AC0 from x - y.  Table 11-1 equates "less than" with AN = 1, so AN is
 * taken as the sign of the exact difference.  (unsicher: to be checked
 * against the hardware model)
 */
static bool trans_CCflag(DisasContext *ctx, arg_CCflag *a)
{
    static const TCGCond cond[5] = {
        TCG_COND_EQ, TCG_COND_LT, TCG_COND_LE, TCG_COND_LTU, TCG_COND_LEU
    };
    TCGv_i32 x, y, res;

    if (ctx->parallel) {
        return false;
    }
    if (a->opc >= 5) {
        /* CC = A0 == A1, A0 < A1, A0 <= A1 */
        if (a->i || a->g || a->x || a->y) {
            return false;
        }
        gen_helper_cc_acc(tcg_env, tcg_constant_i32(a->opc - 5));
        return true;
    }
    x = cpu_reg[(a->g ? BFIN_P0 : BFIN_R0) + a->x];
    if (a->i) {
        int imm = a->y;

        if (a->opc < 3) {
            imm = sextract32(imm, 0, 3);
        }
        y = tcg_constant_i32(imm);
    } else {
        y = cpu_reg[(a->g ? BFIN_P0 : BFIN_R0) + a->y];
    }
    res = tcg_temp_new_i32();
    tcg_gen_setcond_i32(cond[a->opc], res, x, y);
    if (!a->g) {
        TCGv_i32 diff = tcg_temp_new_i32();

        tcg_gen_sub_i32(diff, x, y);
        tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_astat[ASTAT_AZ], diff, 0);
        tcg_gen_setcond_i32(TCG_COND_LT, cpu_astat[ASTAT_AN], x, y);
        tcg_gen_setcond_i32(TCG_COND_GEU, cpu_astat[ASTAT_AC0], x, y);
        gen_ac0_copy();
    }
    tcg_gen_mov_i32(cpu_astat[ASTAT_CC], res);
    return true;
}

/* IF [!] CC dst = src, D- and P-registers */
static bool trans_ccMV(DisasContext *ctx, arg_ccMV *a)
{
    TCGv_i32 d = cpu_reg[(a->d ? BFIN_P0 : BFIN_R0) + a->dst];
    TCGv_i32 s = cpu_reg[(a->s ? BFIN_P0 : BFIN_R0) + a->src];

    if (ctx->parallel) {
        return false;
    }
    tcg_gen_movcond_i32(a->t ? TCG_COND_NE : TCG_COND_EQ, d,
                        cpu_astat[ASTAT_CC], tcg_constant_i32(0), s, d);
    return true;
}

/* Dreg = CC ; CC = Dreg ; CC = !CC */
static bool trans_CC2dreg(DisasContext *ctx, arg_CC2dreg *a)
{
    TCGv_i32 d = cpu_reg[BFIN_R0 + a->reg];

    if (ctx->parallel) {
        return false;
    }
    switch (a->op) {
    case 0:
        tcg_gen_mov_i32(d, cpu_astat[ASTAT_CC]);
        return true;
    case 1:
        tcg_gen_setcondi_i32(TCG_COND_NE, cpu_astat[ASTAT_CC], d, 0);
        return true;
    case 3:
        if (a->reg) {
            return false;
        }
        tcg_gen_xori_i32(cpu_astat[ASTAT_CC], cpu_astat[ASTAT_CC], 1);
        return true;
    }
    return false;
}

/*
 * CC op= statbit, statbit op= CC for AZ, AN, AC0, AC1, V, VS, AV0, AV0S,
 * AV1, AV1S and AQ (chapter 11, "Move CC").
 */
static bool trans_CC2stat(DisasContext *ctx, arg_CC2stat *a)
{
    static const uint32_t statbits =
        (1u << ASTAT_AZ) | (1u << ASTAT_AN) | (1u << ASTAT_AC0) |
        (1u << ASTAT_AC1) | (1u << ASTAT_V) | (1u << ASTAT_VS) |
        (1u << ASTAT_AV0) | (1u << ASTAT_AV0S) | (1u << ASTAT_AV1) |
        (1u << ASTAT_AV1S) | (1u << ASTAT_AQ);
    TCGv_i32 cc = cpu_astat[ASTAT_CC];
    TCGv_i32 sb;
    TCGv_i32 dst, src;

    if (ctx->parallel || !(statbits & (1u << a->cbit))) {
        return false;
    }
    sb = cpu_astat[a->cbit];
    dst = a->d ? sb : cc;
    src = a->d ? cc : sb;
    switch (a->op) {
    case 0:
        tcg_gen_mov_i32(dst, src);
        break;
    case 1:
        tcg_gen_or_i32(dst, dst, src);
        break;
    case 2:
        tcg_gen_and_i32(dst, dst, src);
        break;
    case 3:
        tcg_gen_xor_i32(dst, dst, src);
        break;
    }
    if (a->d && a->cbit == ASTAT_V) {
        tcg_gen_or_i32(cpu_astat[ASTAT_VS], cpu_astat[ASTAT_VS], sb);
    }
    return true;
}

/*
 * PREFETCH, FLUSHINV, FLUSH, IFLUSH [Preg] / [Preg++]: no caches are
 * modelled; the post-increment steps one cache line (32 bytes).
 */
static bool trans_CaCTRL(DisasContext *ctx, arg_CaCTRL *a)
{
    if (ctx->parallel) {
        return false;
    }
    if (a->a) {
        TCGv_i32 p = cpu_reg[BFIN_P0 + a->reg];

        tcg_gen_addi_i32(p, p, 32);
    }
    return true;
}

/*
 * LSETUP (pcrel5m2, lppcrel11m2) LCx [= Preg [>> 1]]: LT = PC + 2 *
 * soffset, LB = PC + 2 * eoffset (both unsigned); rop 0 leaves LC, 1
 * loads Preg, 3 loads Preg >> 1.  SP and FP are not allowed.
 */
static bool trans_LoopSetup(DisasContext *ctx, arg_LoopSetup *a)
{
    int n = a->c;

    /*
     * The PRM bars SP and FP as the count source, but only as assembler
     * syntax; gcc emits LC1 = FP, so take any Preg
     */
    if (a->rop == 2 || (a->rop == 0 && a->reg) || a->reg > 7) {
        return false;
    }
    tcg_gen_movi_i32(cpu_reg[BFIN_LT0 + 3 * n], insn_pc(ctx) + a->soffset * 2);
    tcg_gen_movi_i32(cpu_reg[BFIN_LB0 + 3 * n], insn_pc(ctx) + a->eoffset * 2);
    if (a->rop == 1) {
        tcg_gen_mov_i32(cpu_reg[BFIN_LC0 + 3 * n], cpu_reg[BFIN_P0 + a->reg]);
    } else if (a->rop == 3) {
        tcg_gen_shri_i32(cpu_reg[BFIN_LC0 + 3 * n], cpu_reg[BFIN_P0 + a->reg], 1);
    }
    /* the loop bottoms are part of the TB key */
    ctx->base.is_jmp = DISAS_UPDATE;
    return true;
}

/* --------------------------------------------------- 32-bit DSP */

#include "dsp-valid.c.inc"

/* forms listed in the manual but not modelled yet */
static bool dsp_unimpl(DisasContext *ctx, const char *what)
{
    qemu_log_mask(LOG_UNIMP, "bfin: %s %08x not implemented at %08x\n",
                  what, ctx->insn, insn_pc(ctx));
    return false;
}

/* in a bundle the D-register results wait for the 16-bit slots */
static void gen_dsp(DisasContext *ctx,
                    void (*fn)(TCGv_env, TCGv_i32))
{
    if (ctx->insn_len == 8) {
        gen_helper_dsp_defer(tcg_env);
    }
    fn(tcg_env, tcg_constant_i32(ctx->insn));
}

static bool trans_MNOP(DisasContext *ctx, arg_MNOP *a)
{
    return true;
}

static bool trans_dsp32alu(DisasContext *ctx, arg_dsp32alu *a)
{
    bool found = false;

    for (int i = 0; i < ARRAY_SIZE(dsp32alu_valid); i++) {
        const uint8_t *v = dsp32alu_valid[i];

        if (v[0] == a->hl && v[1] == a->aopcde && v[2] == a->aop &&
            v[3] == a->s && v[4] == a->x) {
            found = true;
            break;
        }
    }
    if (!found) {
        return false;
    }
    switch (a->aopcde) {
    case 0: case 1: case 2: case 3: case 4: case 5: case 6: case 7:
    case 8: case 9: case 10: case 11: case 12: case 14: case 15: case 16:
    case 17:
        break;
    case 18:
        if (a->aop != 3) {
            return dsp_unimpl(ctx, "dsp32alu");
        }
        break;
    default:
        return dsp_unimpl(ctx, "dsp32alu");
    }
    gen_dsp(ctx, gen_helper_dsp32alu);
    return true;
}

static bool shift_valid(const uint8_t (*tab)[3], int n, int sopcde, int sop,
                        int hls)
{
    for (int i = 0; i < n; i++) {
        if (tab[i][0] == sopcde && tab[i][1] == sop && tab[i][2] == hls) {
            return true;
        }
    }
    return false;
}

static bool trans_dsp32shift(DisasContext *ctx, arg_dsp32shift *a)
{
    if (!shift_valid(dsp32shift_valid, ARRAY_SIZE(dsp32shift_valid),
                     a->sopcde, a->sop, a->hls)) {
        return false;
    }
    switch (a->sopcde) {
    case 0: case 1: case 2: case 3: case 4: case 5: case 6: case 10: case 13:
        break;
    default:                    /* EXPADJ, BITMUX, VIT_MAX, BXOR */
        return dsp_unimpl(ctx, "dsp32shift");
    }
    gen_dsp(ctx, gen_helper_dsp32shift);
    return true;
}

static bool trans_dsp32shiftimm(DisasContext *ctx, arg_dsp32shiftimm *a)
{
    if (!shift_valid(dsp32shiftimm_valid, ARRAY_SIZE(dsp32shiftimm_valid),
                     a->sopcde, a->sop, a->hls)) {
        return false;
    }
    gen_dsp(ctx, gen_helper_dsp32shiftimm);
    return true;
}

/* the mmod values tables C-23 list */
static bool mmod_valid(int mmod)
{
    switch (mmod) {
    case 0: case 1: case 2: case 3: case 4: case 6: case 8: case 9:
    case 11: case 12:
        return true;
    }
    return false;
}

static bool trans_dsp32mac(DisasContext *ctx, arg_dsp32mac *a)
{
    if (!mmod_valid(a->mmod) || (a->p && (a->dst & 1)) ||
        (a->op1 == 3 && a->op0 == 3 && !a->w0 && !a->w1)) {
        return false;
    }
    gen_dsp(ctx, gen_helper_dsp32mac);
    return true;
}

static bool trans_dsp32mult(DisasContext *ctx, arg_dsp32mult *a)
{
    if (!mmod_valid(a->mmod) || (a->p && (a->dst & 1)) ||
        (!a->w0 && !a->w1) || a->mmod == 3) {
        return false;
    }
    gen_dsp(ctx, gen_helper_dsp32mult);
    return true;
}

/* ------------------------------------------------- program control */

/*
 * Most 16-bit instructions cannot be issued in parallel (chapter 20);
 * the ones that can say so with parallel_ok().
 */
static bool not_parallel(DisasContext *ctx)
{
    return !ctx->parallel;
}

static void gen_rtx(DisasContext *ctx, int ivg, int retreg)
{
    tcg_gen_mov_i32(cpu_pc, cpu_reg[retreg]);
    gen_helper_rtx(tcg_env, tcg_constant_i32(ivg));
    /* the mode and the interrupt state may have changed */
    ctx->base.is_jmp = DISAS_EXIT_JUMP;
}

/* Table C-22, 0x0000 to 0x00B5 */
static bool trans_ProgCtrl(DisasContext *ctx, arg_ProgCtrl *a)
{
    int r = a->poprnd;
    TCGv_i32 preg = cpu_reg[BFIN_P0 + (r & 7)];

    if (ctx->parallel) {
        /* only the 16-bit NOP, and only in slot 3 */
        return a->prgfunc == 0 && r == 0 && ctx->slot == 1;
    }
    switch (a->prgfunc) {
    case 0:
        return r == 0;                              /* NOP */
    case 1:
        switch (r) {
        case 0:                                     /* RTS */
            gen_jump_reg(ctx, cpu_reg[BFIN_RETS]);
            return true;
        case 1:                                     /* RTI */
            if (check_super(ctx)) {
                gen_rtx(ctx, 5, BFIN_RETI);
            }
            return true;
        case 2:                                     /* RTX */
            if (check_super(ctx)) {
                gen_rtx(ctx, 3, BFIN_RETX);
            }
            return true;
        case 3:                                     /* RTN */
            if (check_super(ctx)) {
                gen_rtx(ctx, 2, BFIN_RETN);
            }
            return true;
        case 4:                                     /* RTE: no emulation */
            gen_exception(ctx, EXCAUSE_PRIVILEGE);
            return true;
        }
        return false;
    case 2:
        switch (r) {
        case 0:                                     /* IDLE */
            if (check_super(ctx)) {
                tcg_gen_movi_i32(cpu_pc, insn_pc(ctx) + 2);
                gen_helper_idle(tcg_env);
                ctx->base.is_jmp = DISAS_NORETURN;
            }
            return true;
        case 3:                                     /* CSYNC */
        case 4:                                     /* SSYNC */
            return true;
        case 5:                                     /* EMUEXCPT: NOP */
            return true;
        case 15:                                    /* ABORT */
            return false;
        }
        return false;
    case 3:                                         /* CLI Dreg */
        if (r > 7) {
            return false;
        }
        if (check_super(ctx)) {
            gen_helper_cli(cpu_reg[BFIN_R0 + r], tcg_env);
        }
        return true;
    case 4:                                         /* STI Dreg */
        if (r > 7) {
            return false;
        }
        if (check_super(ctx)) {
            gen_helper_sti(tcg_env, cpu_reg[BFIN_R0 + r]);
            tcg_gen_movi_i32(cpu_pc, insn_pc(ctx) + 2);
            ctx->base.is_jmp = DISAS_EXIT;
        }
        return true;
    case 5:                                         /* JUMP (Preg) */
        if (r > 7) {
            return false;
        }
        gen_jump_reg(ctx, preg);
        return true;
    case 6:                                         /* CALL (Preg) */
        if (r > 7) {
            return false;
        }
        tcg_gen_andi_i32(cpu_pc, preg, ~1u);
        tcg_gen_movi_i32(cpu_reg[BFIN_RETS], insn_pc(ctx) + 2);
        ctx->base.is_jmp = DISAS_JUMP;
        return true;
    case 7:                                         /* CALL (PC + Preg) */
    case 8:                                         /* JUMP (PC + Preg) */
        if (r > 7) {
            return false;
        }
        tcg_gen_addi_i32(cpu_pc, preg, insn_pc(ctx));
        tcg_gen_andi_i32(cpu_pc, cpu_pc, ~1u);
        if (a->prgfunc == 7) {
            tcg_gen_movi_i32(cpu_reg[BFIN_RETS], insn_pc(ctx) + 2);
        }
        ctx->base.is_jmp = DISAS_JUMP;
        return true;
    case 9:                                         /* RAISE uimm4 */
        if (r == 0 || r == 3 || r == 4) {
            return false;                           /* reserved */
        }
        if (check_super(ctx)) {
            gen_helper_raise(tcg_env, tcg_constant_i32(r));
            tcg_gen_movi_i32(cpu_pc, insn_pc(ctx) + 2);
            ctx->base.is_jmp = DISAS_EXIT;
        }
        return true;
    case 10:                                        /* EXCPT uimm4 */
        /* a service exception: RETX is the next instruction */
        ctx->base.pc_next += 2;
        gen_exception(ctx, r);
        ctx->base.pc_next -= 2;
        return true;
    case 11:                                        /* TESTSET (Preg) */
        if (r > 5) {
            return false;
        }
        {
            TCGv_i32 old = tcg_temp_new_i32();

            tcg_gen_atomic_fetch_or_i32(old, preg, tcg_constant_i32(0x80),
                                        ctx->mem_idx, MO_UB);
            tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_astat[ASTAT_CC], old, 0);
        }
        return true;
    }
    return false;
}

/* JUMP.S pcrel13m2 */
static bool trans_UJUMP(DisasContext *ctx, arg_UJUMP *a)
{
    if (!not_parallel(ctx)) {
        return false;
    }
    gen_jump_imm(ctx, insn_pc(ctx) + a->offset * 2);
    return true;
}

/* IF [!] CC JUMP pcrel11m2 [(BP)] */
static bool trans_BRCC(DisasContext *ctx, arg_BRCC *a)
{
    TCGLabel *skip = gen_new_label();

    if (!not_parallel(ctx)) {
        return false;
    }
    tcg_gen_brcondi_i32(a->t ? TCG_COND_EQ : TCG_COND_NE,
                        cpu_astat[ASTAT_CC], 0, skip);
    gen_hwloop(ctx, insn_pc(ctx), HWLOOP_DEC);
    gen_goto_tb(ctx, 1, insn_pc(ctx) + a->offset * 2);
    gen_set_label(skip);
    gen_hwloop(ctx, insn_pc(ctx), HWLOOP_GOTO);
    gen_goto_tb(ctx, 0, insn_pc(ctx) + 2);
    ctx->base.is_jmp = DISAS_NORETURN;
    return true;
}

/* CALL / JUMP.L pcrel25m2 */
static bool trans_CALLa(DisasContext *ctx, arg_CALLa *a)
{
    int32_t off = ((a->msw << 16) | a->lsw) * 2;

    if (a->s) {
        tcg_gen_movi_i32(cpu_reg[BFIN_RETS], insn_pc(ctx) + 4);
    }
    gen_jump_imm(ctx, insn_pc(ctx) + off);
    return true;
}

/* ---------------------------------------------------------- the loop */

static uint16_t fetch16(DisasContext *ctx, CPUBfinState *env, uint32_t pc)
{
    return translator_lduw_end(env, &ctx->base, pc, MO_LE);
}

/* is the halfword the first of a 32-bit instruction? (appendix C) */
static bool is_insn32(uint16_t iw0)
{
    return (iw0 & 0xc000) == 0xc000;
}

/* the M bit makes a 32-bit DSP instruction the first of a 64-bit bundle */
static bool is_multi(uint16_t iw0)
{
    return (iw0 & 0xf000) == 0xc000 && (iw0 & 0x0800);
}

/*
 * Hardware loops (chapter 7, LSETUP): when the instruction at LBx has
 * executed and LCx is nonzero, LCx is decremented and, unless it reached
 * zero, execution continues at LTx.  The loop bottoms are part of the TB
 * key (cs_base), so whether an instruction is a loop end is known here.
 * With LB0 == LB1 loop 1 is the inner loop and is evaluated first.
 */
/*
 * The sequencer acts on the fetch address: at LBx with LCx nonzero it
 * decrements LCx and, unless that reached zero, continues at LTx.  The
 * decrement happens even when the instruction there branches away (PRM
 * "LSETUP", forbidden only on the BF535); the branch target then wins,
 * which strncpy relies on when it reads LC0 after leaving its loop.
 * Loop 1 is checked first, a loopback of it leaves loop 0 alone.
 */
static void gen_hwloop(DisasContext *ctx, uint32_t pc, int mode)
{
    TCGLabel *end = NULL;

    for (int n = 1; n >= 0; n--) {
        TCGLabel *done;
        TCGv_i32 lc;

        if (pc != ctx->lb[n]) {
            continue;
        }
        if (!end) {
            end = gen_new_label();
        }
        done = gen_new_label();
        lc = cpu_reg[BFIN_LC0 + 3 * n];
        tcg_gen_brcondi_i32(TCG_COND_EQ, lc, 0, done);
        tcg_gen_subi_i32(lc, lc, 1);
        tcg_gen_brcondi_i32(TCG_COND_EQ, lc, 0, done);
        if (mode != HWLOOP_DEC) {
            tcg_gen_mov_i32(cpu_pc, cpu_reg[BFIN_LT0 + 3 * n]);
        }
        if (mode == HWLOOP_GOTO) {
            tcg_gen_lookup_and_goto_ptr();
        } else {
            tcg_gen_br(end);
        }
        gen_set_label(done);
    }
    if (end) {
        gen_set_label(end);
        if (mode == HWLOOP_GOTO && ctx->base.is_jmp == DISAS_NEXT) {
            ctx->base.is_jmp = DISAS_TOO_MANY;
        }
    }
}

static void bfin_tr_init_disas_context(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *ctx = container_of(dcbase, DisasContext, base);

    ctx->env = cpu_env(cs);
    ctx->super = ctx->base.tb->flags & BFIN_TBFLAG_SUPER;
    ctx->mem_idx = ctx->super ? MMU_SUPER_IDX : MMU_USER_IDX;
    ctx->lb[0] = (uint32_t)ctx->base.tb->cs_base;
    ctx->lb[1] = ctx->base.tb->cs_base >> 32;
    ctx->parallel = false;
}

static void bfin_tr_tb_start(DisasContextBase *dcbase, CPUState *cs)
{
}

static void bfin_tr_insn_start(DisasContextBase *dcbase, CPUState *cs)
{
    tcg_gen_insn_start(dcbase->pc_next, 0, 0);
}

static void bfin_tr_translate_insn(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *ctx = container_of(dcbase, DisasContext, base);
    CPUBfinState *env = cpu_env(cs);
    uint32_t pc = ctx->base.pc_next;
    uint16_t iw0 = fetch16(ctx, env, pc);
    bool ok;

    if (is_insn32(iw0)) {
        uint32_t insn = ((uint32_t)iw0 << 16) | fetch16(ctx, env, pc + 2);

        ctx->insn_len = is_multi(iw0) ? 8 : 4;
        ctx->insn = insn;
        ok = decode32(ctx, insn);
        if (ok && ctx->insn_len == 8 && ctx->base.is_jmp == DISAS_NEXT) {
            ctx->parallel = true;
            for (int slot = 0; ok && slot < 2; slot++) {
                uint16_t iw = fetch16(ctx, env, pc + 4 + 2 * slot);

                ctx->slot = slot;
                ok = !is_insn32(iw) && decode16(ctx, iw);
            }
            ctx->parallel = false;
            /* the DSP slot's register results land after the others */
            gen_helper_dsp_commit(tcg_env);
        }
    } else {
        ctx->insn_len = 2;
        ok = decode16(ctx, iw0);
    }
    if (!ok) {
        qemu_log_mask(LOG_GUEST_ERROR, "bfin: illegal instruction %04x "
                      "at %08x\n", iw0, pc);
        gen_illegal(ctx);
        ctx->base.pc_next += 2;
        return;
    }
    ctx->base.pc_next += ctx->insn_len;
    if (pc != ctx->lb[0] && pc != ctx->lb[1]) {
        return;
    }
    switch (ctx->base.is_jmp) {
    case DISAS_NEXT:
    case DISAS_TOO_MANY:
        gen_hwloop(ctx, pc, HWLOOP_GOTO);
        break;
    case DISAS_UPDATE:
        tcg_gen_movi_i32(cpu_pc, ctx->base.pc_next);
        ctx->base.is_jmp = DISAS_EXIT;
        /* fall through */
    case DISAS_EXIT:
        gen_hwloop(ctx, pc, HWLOOP_SETPC);
        break;
    case DISAS_JUMP:
    case DISAS_EXIT_JUMP:
        gen_hwloop(ctx, pc, HWLOOP_DEC);
        break;
    default:                    /* BRCC has done it, exceptions don't */
        break;
    }
}

static void bfin_tr_tb_stop(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *ctx = container_of(dcbase, DisasContext, base);

    switch (ctx->base.is_jmp) {
    case DISAS_NEXT:
    case DISAS_TOO_MANY:
        gen_goto_tb(ctx, 0, ctx->base.pc_next);
        break;
    case DISAS_JUMP:
        tcg_gen_lookup_and_goto_ptr();
        break;
    case DISAS_UPDATE:
        tcg_gen_movi_i32(cpu_pc, ctx->base.pc_next);
        tcg_gen_exit_tb(NULL, 0);
        break;
    case DISAS_EXIT:
    case DISAS_EXIT_JUMP:
        tcg_gen_exit_tb(NULL, 0);
        break;
    case DISAS_NORETURN:
        break;
    default:
        g_assert_not_reached();
    }
}

static const TranslatorOps bfin_tr_ops = {
    .init_disas_context = bfin_tr_init_disas_context,
    .tb_start           = bfin_tr_tb_start,
    .insn_start         = bfin_tr_insn_start,
    .translate_insn     = bfin_tr_translate_insn,
    .tb_stop            = bfin_tr_tb_stop,
};

void bfin_translate_code(CPUState *cs, TranslationBlock *tb,
                         int *max_insns, vaddr pc, void *host_pc)
{
    DisasContext dc;

    translator_loop(cs, tb, max_insns, pc, host_pc, &bfin_tr_ops, &dc.base,
                    TCG_TYPE_I32);
}
