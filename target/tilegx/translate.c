/*
 * QEMU TILE-Gx CPU
 *
 *  Copyright (c) 2015 Chen Gang
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
 * License along with this library; if not, see
 * <http://www.gnu.org/licenses/lgpl-2.1.html>
 */

#include "qemu/osdep.h"
#include "cpu.h"
#include "qemu/log.h"
#include "exec/log.h"
#include "exec/translator.h"
#include "exec/translation-block.h"
#include "exec/target_page.h"
#include "tcg/tcg-op.h"
#include "exec/helper-proto.h"
#include "exec/helper-gen.h"

#define HELPER_H "helper.h"
#include "exec/helper-info.c.inc"
#undef HELPER_H

#include "opcode_tilegx.h"

#define FMT64X                          "%016" PRIx64

/*
 * What a pipe's decoder reports: nothing, or the interrupt the bundle
 * raises before any of its results are written.  Unknown and unimplemented
 * encodings are both an illegal instruction to the guest.
 */
typedef int TileExcp;
#define TILEGX_EXCP_NONE                    (-1)
#define TILEGX_EXCP_OPCODE_UNKNOWN          INT_ILL
#define TILEGX_EXCP_OPCODE_UNIMPLEMENTED    (TILEGX_NUM_INTERRUPTS + INT_ILL)

static TCGv cpu_pc;
static TCGv cpu_regs[TILEGX_R_COUNT];

static const char * const reg_names[64] = {
     "r0",  "r1",  "r2",  "r3",  "r4",  "r5",  "r6",  "r7",
     "r8",  "r9", "r10", "r11", "r12", "r13", "r14", "r15",
    "r16", "r17", "r18", "r19", "r20", "r21", "r22", "r23",
    "r24", "r25", "r26", "r27", "r28", "r29", "r30", "r31",
    "r32", "r33", "r34", "r35", "r36", "r37", "r38", "r39",
    "r40", "r41", "r42", "r43", "r44", "r45", "r46", "r47",
    "r48", "r49", "r50", "r51",  "bp",  "tp",  "sp",  "lr",
    "sn", "idn0", "idn1", "udn0", "udn1", "udn2", "udn2", "zero"
};

/* Modified registers are cached in temporaries until the end of the bundle. */
typedef struct {
    unsigned reg;
    TCGv val;
} DisasContextTemp;

#define MAX_WRITEBACK 4

/* This is the state at translation time.  */
typedef struct {
    uint64_t pc;		/* Current pc */

    TCGv zero;                  /* For zero register */

    DisasContextTemp wb[MAX_WRITEBACK];
    int num_wb;
    int mmuidx;
    bool exit_tb;
    bool end_tb;        /* state changed: end the TB after this bundle */
    int post_excp;      /* swint: raised after the bundle's results */

    struct {
        TCGCond cond;    /* branch condition */
        TCGv dest;       /* branch destination */
        TCGv val1;       /* value to be compared against zero, for cond */
    } jmp;               /* Jump object, only once in each TB block */
} DisasContext;


/* Differentiate the various pipe encodings.  */
#define TY_X0  0
#define TY_X1  1
#define TY_Y0  2
#define TY_Y1  3

/* Remerge the base opcode and extension fields for switching.
   The X opcode fields are 3 bits; Y0/Y1 opcode fields are 4 bits;
   Y2 opcode field is 2 bits.  */
#define OE(OP, EXT, XY) (TY_##XY + OP * 4 + EXT * 64)

/* Similar, but for Y2 only.  */
#define OEY2(OP, MODE) (OP + MODE * 4)

/* Similar, but make sure opcode names match up.  */
#define OE_RR_X0(E)    OE(RRR_0_OPCODE_X0, E##_UNARY_OPCODE_X0, X0)
#define OE_RR_X1(E)    OE(RRR_0_OPCODE_X1, E##_UNARY_OPCODE_X1, X1)
#define OE_RR_Y0(E)    OE(RRR_1_OPCODE_Y0, E##_UNARY_OPCODE_Y0, Y0)
#define OE_RR_Y1(E)    OE(RRR_1_OPCODE_Y1, E##_UNARY_OPCODE_Y1, Y1)
#define OE_RRR(E,N,XY) OE(RRR_##N##_OPCODE_##XY, E##_RRR_##N##_OPCODE_##XY, XY)
#define OE_IM(E,XY)    OE(IMM8_OPCODE_##XY, E##_IMM8_OPCODE_##XY, XY)
#define OE_SH(E,XY)    OE(SHIFT_OPCODE_##XY, E##_SHIFT_OPCODE_##XY, XY)

#define V1_IMM(X)      (((X) & 0xff) * 0x0101010101010101ull)
#define V2_IMM(X)      (((X) & 0xffff) * 0x0001000100010001ull)


/* Raise NUM with the PC of this bundle: nothing in it has taken effect. */
static void gen_exception(DisasContext *dc, int num)
{
    tcg_gen_movi_tl(cpu_pc, dc->pc);
    gen_helper_exception(tcg_env, tcg_constant_i32(num));
    dc->exit_tb = true;
}

static bool check_gr(DisasContext *dc, uint8_t reg)
{
    if (likely(reg < TILEGX_R_COUNT)) {
        return true;
    }

    switch (reg) {
    case TILEGX_R_SN:
    case TILEGX_R_ZERO:
        break;
    case TILEGX_R_IDN0:
    case TILEGX_R_IDN1:
        gen_exception(dc, INT_IDN_ACCESS);
        break;
    case TILEGX_R_UDN0:
    case TILEGX_R_UDN1:
    case TILEGX_R_UDN2:
    case TILEGX_R_UDN3:
        gen_exception(dc, INT_UDN_ACCESS);
        break;
    default:
        g_assert_not_reached();
    }
    return false;
}

static TCGv load_zero(DisasContext *dc)
{
    if (!dc->zero) {
        dc->zero = tcg_constant_i64(0);
    }
    return dc->zero;
}

static TCGv load_gr(DisasContext *dc, unsigned reg)
{
    if (check_gr(dc, reg)) {
        return cpu_regs[reg];
    }
    return load_zero(dc);
}

static TCGv dest_gr(DisasContext *dc, unsigned reg)
{
    int n;

    /* Skip the result, mark the exception if necessary, and continue */
    check_gr(dc, reg);

    n = dc->num_wb++;
    dc->wb[n].reg = reg;
    return dc->wb[n].val = tcg_temp_new_i64();
}

static void gen_saturate_op(TCGv tdest, TCGv tsrca, TCGv tsrcb,
                            void (*operate)(TCGv, TCGv, TCGv))
{
    TCGv t0 = tcg_temp_new();

    tcg_gen_ext32s_tl(tdest, tsrca);
    tcg_gen_ext32s_tl(t0, tsrcb);
    operate(tdest, tdest, t0);

    tcg_gen_movi_tl(t0, 0x7fffffff);
    tcg_gen_movcond_tl(TCG_COND_GT, tdest, tdest, t0, t0, tdest);
    tcg_gen_movi_tl(t0, -0x80000000LL);
    tcg_gen_movcond_tl(TCG_COND_LT, tdest, tdest, t0, t0, tdest);

}

/*
 * The atomics.  CMPEXCH compares against the CMPEXCH_VALUE SPR; FETCHADDGEZ
 * stores only if the sum is not negative.
 */
typedef enum {
    AT_CMPEXCH, AT_EXCH, AT_FETCHADD, AT_FETCHADDGEZ, AT_FETCHAND, AT_FETCHOR
} AtomicOp;

static void gen_atomic(DisasContext *dc, TCGv tdest, TCGv tsrca, TCGv tsrcb,
                       AtomicOp op, bool four)
{
    MemOp mop = (four ? MO_TESL : MO_TEUQ) | MO_ALIGN;
    TCGv t;

    switch (op) {
    case AT_CMPEXCH:
        t = tcg_temp_new();
        tcg_gen_ld_tl(t, tcg_env, offsetof(CPUTLGState, cmpexch_value));
        if (four) {
            tcg_gen_ext32s_tl(t, t);
        }
        tcg_gen_atomic_cmpxchg_tl(tdest, tsrca, t, tsrcb, dc->mmuidx, mop);
        break;
    case AT_EXCH:
        tcg_gen_atomic_xchg_tl(tdest, tsrca, tsrcb, dc->mmuidx, mop);
        break;
    case AT_FETCHADD:
        tcg_gen_atomic_fetch_add_tl(tdest, tsrca, tsrcb, dc->mmuidx, mop);
        break;
    case AT_FETCHAND:
        tcg_gen_atomic_fetch_and_tl(tdest, tsrca, tsrcb, dc->mmuidx, mop);
        break;
    case AT_FETCHOR:
        tcg_gen_atomic_fetch_or_tl(tdest, tsrca, tsrcb, dc->mmuidx, mop);
        break;
    case AT_FETCHADDGEZ:
        /* Single tile, no MTTCG: a load and a conditional store do. */
        t = tcg_temp_new();
        tcg_gen_qemu_ld_tl(tdest, tsrca, dc->mmuidx, mop);
        tcg_gen_add_tl(t, tdest, tsrcb);
        if (four) {
            tcg_gen_ext32s_tl(t, t);
        }
        tcg_gen_movcond_tl(TCG_COND_LT, t, t, tcg_constant_tl(0), tdest, t);
        tcg_gen_qemu_st_tl(t, tsrca, dc->mmuidx, mop);
        break;
    }
}

/* Shift the 128-bit value TSRCA:TSRCD right by the number of bytes
   specified by the bottom 3 bits of TSRCB, and set TDEST to the
   low 64 bits of the resulting value.  */
static void gen_dblalign(TCGv tdest, TCGv tsrcd, TCGv tsrca, TCGv tsrcb)
{
    TCGv t0 = tcg_temp_new();

    tcg_gen_andi_tl(t0, tsrcb, 7);
    tcg_gen_shli_tl(t0, t0, 3);
    tcg_gen_shr_tl(tdest, tsrcd, t0);

    /* We want to do "t0 = tsrca << (64 - t0)".  Two's complement
       arithmetic on a 6-bit field tells us that 64 - t0 is equal
       to (t0 ^ 63) + 1.  So we can do the shift in two parts,
       neither of which will be an invalid shift by 64.  */
    tcg_gen_xori_tl(t0, t0, 63);
    tcg_gen_shl_tl(t0, tsrca, t0);
    tcg_gen_shli_tl(t0, t0, 1);
    tcg_gen_or_tl(tdest, tdest, t0);

}

/* Similarly, except that the 128-bit value is TSRCA:TSRCB, and the
   right shift is an immediate.  */
static void gen_dblaligni(TCGv tdest, TCGv tsrca, TCGv tsrcb, int shr)
{
    TCGv t0 = tcg_temp_new();

    tcg_gen_shri_tl(t0, tsrcb, shr);
    tcg_gen_shli_tl(tdest, tsrca, 64 - shr);
    tcg_gen_or_tl(tdest, tdest, t0);

}

typedef enum {
    LU, LS, HU, HS
} MulHalf;

static void gen_ext_half(TCGv d, TCGv s, MulHalf h)
{
    switch (h) {
    case LU:
        tcg_gen_ext32u_tl(d, s);
        break;
    case LS:
        tcg_gen_ext32s_tl(d, s);
        break;
    case HU:
        tcg_gen_shri_tl(d, s, 32);
        break;
    case HS:
        tcg_gen_sari_tl(d, s, 32);
        break;
    }
}

static void gen_mul_half(TCGv tdest, TCGv tsrca, TCGv tsrcb,
                         MulHalf ha, MulHalf hb)
{
    TCGv t = tcg_temp_new();
    gen_ext_half(t, tsrca, ha);
    gen_ext_half(tdest, tsrcb, hb);
    tcg_gen_mul_tl(tdest, tdest, t);
}

static void gen_cmul2(TCGv tdest, TCGv tsrca, TCGv tsrcb, int sh, int rd)
{
    TCGv_i32 tsh = tcg_constant_i32(sh);
    TCGv_i32 trd = tcg_constant_i32(rd);
    gen_helper_cmul2(tdest, tsrca, tsrcb, tsh, trd);
}

static TileExcp gen_st_opcode(DisasContext *dc, unsigned dest, unsigned srca,
                              unsigned srcb, MemOp memop, const char *name)
{
    if (dest) {
        return TILEGX_EXCP_OPCODE_UNKNOWN;
    }

    tcg_gen_qemu_st_tl(load_gr(dc, srcb), load_gr(dc, srca),
                       dc->mmuidx, memop);

    qemu_log_mask(CPU_LOG_TB_IN_ASM, "%s %s, %s", name,
                  reg_names[srca], reg_names[srcb]);
    return TILEGX_EXCP_NONE;
}

static TileExcp gen_st_add_opcode(DisasContext *dc, unsigned srca, unsigned srcb,
                                  int imm, MemOp memop, const char *name)
{
    TCGv tsrca = load_gr(dc, srca);
    TCGv tsrcb = load_gr(dc, srcb);

    tcg_gen_qemu_st_tl(tsrcb, tsrca, dc->mmuidx, memop);
    tcg_gen_addi_tl(dest_gr(dc, srca), tsrca, imm);

    qemu_log_mask(CPU_LOG_TB_IN_ASM, "%s %s, %s, %d", name,
                  reg_names[srca], reg_names[srcb], imm);
    return TILEGX_EXCP_NONE;
}

/* Equality comparison with zero can be done quickly and efficiently.  */
static void gen_v1cmpeq0(TCGv v)
{
    TCGv m = tcg_constant_tl(V1_IMM(0x7f));
    TCGv c = tcg_temp_new();

    /* ~(((v & m) + m) | m | v).  Sets the msb for each byte == 0.  */
    tcg_gen_and_tl(c, v, m);
    tcg_gen_add_tl(c, c, m);
    tcg_gen_or_tl(c, c, m);
    tcg_gen_nor_tl(c, c, v);

    /* Shift the msb down to form the lsb boolean result.  */
    tcg_gen_shri_tl(v, c, 7);
}

static void gen_v1cmpne0(TCGv v)
{
    TCGv m = tcg_constant_tl(V1_IMM(0x7f));
    TCGv c = tcg_temp_new();

    /* (((v & m) + m) | v) & ~m.  Sets the msb for each byte != 0.  */
    tcg_gen_and_tl(c, v, m);
    tcg_gen_add_tl(c, c, m);
    tcg_gen_or_tl(c, c, v);
    tcg_gen_andc_tl(c, c, m);

    /* Shift the msb down to form the lsb boolean result.  */
    tcg_gen_shri_tl(v, c, 7);
}

/* Vector addition can be performed via arithmetic plus masking.  It is
   efficient this way only for 4 or more elements.  */
static void gen_v12add(TCGv tdest, TCGv tsrca, TCGv tsrcb, uint64_t sign)
{
    TCGv tmask = tcg_constant_tl(~sign);
    TCGv t0 = tcg_temp_new();
    TCGv t1 = tcg_temp_new();

    /* ((a & ~sign) + (b & ~sign)) ^ ((a ^ b) & sign).  */
    tcg_gen_and_tl(t0, tsrca, tmask);
    tcg_gen_and_tl(t1, tsrcb, tmask);
    tcg_gen_add_tl(tdest, t0, t1);
    tcg_gen_xor_tl(t0, tsrca, tsrcb);
    tcg_gen_andc_tl(t0, t0, tmask);
    tcg_gen_xor_tl(tdest, tdest, t0);

}

/* Similarly for vector subtraction.  */
static void gen_v12sub(TCGv tdest, TCGv tsrca, TCGv tsrcb, uint64_t sign)
{
    TCGv tsign = tcg_constant_tl(sign);
    TCGv t0 = tcg_temp_new();
    TCGv t1 = tcg_temp_new();

    /* ((a | sign) - (b & ~sign)) ^ ((a ^ ~b) & sign).  */
    tcg_gen_or_tl(t0, tsrca, tsign);
    tcg_gen_andc_tl(t1, tsrcb, tsign);
    tcg_gen_sub_tl(tdest, t0, t1);
    tcg_gen_eqv_tl(t0, tsrca, tsrcb);
    tcg_gen_and_tl(t0, t0, tsign);
    tcg_gen_xor_tl(tdest, tdest, t0);

}

static void gen_v4sh(TCGv d64, TCGv a64, TCGv b64,
                     void (*generate)(TCGv_i32, TCGv_i32, TCGv_i32))
{
    TCGv_i32 al = tcg_temp_new_i32();
    TCGv_i32 ah = tcg_temp_new_i32();
    TCGv_i32 bl = tcg_temp_new_i32();

    tcg_gen_extr_i64_i32(al, ah, a64);
    tcg_gen_extrl_i64_i32(bl, b64);
    tcg_gen_andi_i32(bl, bl, 31);
    generate(al, al, bl);
    generate(ah, ah, bl);
    tcg_gen_concat_i32_i64(d64, al, ah);

}

static void gen_v4op(TCGv d64, TCGv a64, TCGv b64,
                     void (*generate)(TCGv_i32, TCGv_i32, TCGv_i32))
{
    TCGv_i32 al = tcg_temp_new_i32();
    TCGv_i32 ah = tcg_temp_new_i32();
    TCGv_i32 bl = tcg_temp_new_i32();
    TCGv_i32 bh = tcg_temp_new_i32();

    tcg_gen_extr_i64_i32(al, ah, a64);
    tcg_gen_extr_i64_i32(bl, bh, b64);
    generate(al, al, bl);
    generate(ah, ah, bh);
    tcg_gen_concat_i32_i64(d64, al, ah);

}

/* ill, bpt and raise all trap; the kernel looks at the bundle itself. */
static TileExcp gen_specill(DisasContext *dc, unsigned dest, unsigned srca,
                            uint64_t bundle)
{
    qemu_log_mask(CPU_LOG_TB_IN_ASM, "ill");
    return INT_ILL;
}

static TileExcp gen_rr_opcode(DisasContext *dc, unsigned opext,
                              unsigned dest, unsigned srca, uint64_t bundle)
{
    TCGv tdest, tsrca;
    const char *mnemonic;
    MemOp memop;
    TileExcp ret = TILEGX_EXCP_NONE;
    bool prefetch_nofault = false;

    /* Eliminate instructions with no output before doing anything else.  */
    switch (opext) {
    case OE_RR_Y0(NOP):
    case OE_RR_Y1(NOP):
    case OE_RR_X0(NOP):
    case OE_RR_X1(NOP):
        mnemonic = "nop";
        goto done0;
    case OE_RR_Y0(FNOP):
    case OE_RR_Y1(FNOP):
    case OE_RR_X0(FNOP):
    case OE_RR_X1(FNOP):
        mnemonic = "fnop";
        goto done0;
    case OE_RR_X1(DRAIN):
        mnemonic = "drain";
        goto done0;
    case OE_RR_X1(FLUSHWB):
        mnemonic = "flushwb";
        goto done0;
    case OE_RR_X1(ILL):
        return gen_specill(dc, dest, srca, bundle);
    case OE_RR_Y1(ILL):
        qemu_log_mask(CPU_LOG_TB_IN_ASM, "ill");
        return INT_ILL;
    case OE_RR_X1(MF):
        mnemonic = "mf";
        goto done0;
    case OE_RR_X1(NAP):
        if (srca || dest) {
            return TILEGX_EXCP_OPCODE_UNKNOWN;
        }
        tcg_gen_movi_tl(cpu_pc, dc->pc);
        gen_helper_nap(tcg_env);
        dc->exit_tb = true;
        qemu_log_mask(CPU_LOG_TB_IN_ASM, "nap");
        return TILEGX_EXCP_NONE;
    case OE_RR_X1(IRET):
        if (srca || dest) {
            return TILEGX_EXCP_OPCODE_UNKNOWN;
        }
        tcg_gen_movi_tl(cpu_pc, dc->pc);
        gen_helper_iret(tcg_env);
        tcg_gen_exit_tb(NULL, 0);
        dc->exit_tb = true;
        qemu_log_mask(CPU_LOG_TB_IN_ASM, "iret");
        return TILEGX_EXCP_NONE;
    case OE_RR_X1(SWINT0):
        dc->post_excp = INT_SWINT_0;
        mnemonic = "swint0";
        goto done0;
    case OE_RR_X1(SWINT1):
        dc->post_excp = INT_SWINT_1;
        mnemonic = "swint1";
        goto done0;
    case OE_RR_X1(SWINT2):
        dc->post_excp = INT_SWINT_2;
        mnemonic = "swint2";
        goto done0;
    case OE_RR_X1(SWINT3):
        dc->post_excp = INT_SWINT_3;
        mnemonic = "swint3";
    done0:
        if (srca || dest) {
            return TILEGX_EXCP_OPCODE_UNKNOWN;
        }
        qemu_log_mask(CPU_LOG_TB_IN_ASM, "%s", mnemonic);
        return ret;

    case OE_RR_X1(DTLBPR):
        return TILEGX_EXCP_OPCODE_UNIMPLEMENTED;
    case OE_RR_X1(FINV):
        mnemonic = "finv";
        goto done1;
    case OE_RR_X1(FLUSH):
        mnemonic = "flush";
        goto done1;
    case OE_RR_X1(ICOH):
        mnemonic = "icoh";
        goto done1;
    case OE_RR_X1(INV):
        mnemonic = "inv";
        goto done1;
    case OE_RR_X1(WH64):
        mnemonic = "wh64";
        goto done1;
    case OE_RR_X1(JRP):
    case OE_RR_Y1(JRP):
        mnemonic = "jrp";
        goto do_jr;
    case OE_RR_X1(JR):
    case OE_RR_Y1(JR):
        mnemonic = "jr";
        goto do_jr;
    case OE_RR_X1(JALRP):
    case OE_RR_Y1(JALRP):
        mnemonic = "jalrp";
        goto do_jalr;
    case OE_RR_X1(JALR):
    case OE_RR_Y1(JALR):
        mnemonic = "jalr";
    do_jalr:
        tcg_gen_movi_tl(dest_gr(dc, TILEGX_R_LR),
                        dc->pc + TILEGX_BUNDLE_SIZE_IN_BYTES);
    do_jr:
        dc->jmp.cond = TCG_COND_ALWAYS;
        dc->jmp.dest = tcg_temp_new();
        tcg_gen_andi_tl(dc->jmp.dest, load_gr(dc, srca), ~7);
    done1:
        if (dest) {
            return TILEGX_EXCP_OPCODE_UNKNOWN;
        }
        qemu_log_mask(CPU_LOG_TB_IN_ASM, "%s %s", mnemonic, reg_names[srca]);
        return ret;
    }

    tdest = dest_gr(dc, dest);
    tsrca = load_gr(dc, srca);

    switch (opext) {
    case OE_RR_X0(CNTLZ):
    case OE_RR_Y0(CNTLZ):
        tcg_gen_clzi_tl(tdest, tsrca, TARGET_LONG_BITS);
        mnemonic = "cntlz";
        break;
    case OE_RR_X0(CNTTZ):
    case OE_RR_Y0(CNTTZ):
        tcg_gen_ctzi_tl(tdest, tsrca, TARGET_LONG_BITS);
        mnemonic = "cnttz";
        break;
    case OE_RR_X0(FSINGLE_PACK1):
    case OE_RR_Y0(FSINGLE_PACK1):
        gen_helper_fsingle_pack1(tdest, tsrca);
        mnemonic = "fsingle_pack1";
        break;
    case OE_RR_X1(LD1S):
        memop = MO_SB;
        mnemonic = "ld1s"; /* prefetch_l1_fault */
        goto do_load;
    case OE_RR_X1(LD1U):
        memop = MO_UB;
        mnemonic = "ld1u"; /* prefetch, prefetch_l1 */
        prefetch_nofault = (dest == TILEGX_R_ZERO);
        goto do_load;
    case OE_RR_X1(LD2S):
        memop = MO_TESW;
        mnemonic = "ld2s"; /* prefetch_l2_fault */
        goto do_load;
    case OE_RR_X1(LD2U):
        memop = MO_TEUW;
        mnemonic = "ld2u"; /* prefetch_l2 */
        prefetch_nofault = (dest == TILEGX_R_ZERO);
        goto do_load;
    case OE_RR_X1(LD4S):
        memop = MO_TESL;
        mnemonic = "ld4s"; /* prefetch_l3_fault */
        goto do_load;
    case OE_RR_X1(LD4U):
        memop = MO_TEUL;
        mnemonic = "ld4u"; /* prefetch_l3 */
        prefetch_nofault = (dest == TILEGX_R_ZERO);
        goto do_load;
    case OE_RR_X1(LDNT1S):
        memop = MO_SB;
        mnemonic = "ldnt1s";
        goto do_load;
    case OE_RR_X1(LDNT1U):
        memop = MO_UB;
        mnemonic = "ldnt1u";
        goto do_load;
    case OE_RR_X1(LDNT2S):
        memop = MO_TESW;
        mnemonic = "ldnt2s";
        goto do_load;
    case OE_RR_X1(LDNT2U):
        memop = MO_TEUW;
        mnemonic = "ldnt2u";
        goto do_load;
    case OE_RR_X1(LDNT4S):
        memop = MO_TESL;
        mnemonic = "ldnt4s";
        goto do_load;
    case OE_RR_X1(LDNT4U):
        memop = MO_TEUL;
        mnemonic = "ldnt4u";
        goto do_load;
    case OE_RR_X1(LDNT):
        memop = MO_TEUQ;
        mnemonic = "ldnt";
        goto do_load;
    case OE_RR_X1(LD):
        memop = MO_TEUQ;
        mnemonic = "ld";
    do_load:
        if (!prefetch_nofault) {
            tcg_gen_qemu_ld_tl(tdest, tsrca, dc->mmuidx, memop);
        }
        break;
    case OE_RR_X1(LDNA):
        tcg_gen_andi_tl(tdest, tsrca, ~7);
        tcg_gen_qemu_ld_tl(tdest, tdest, dc->mmuidx, MO_TEUQ);
        mnemonic = "ldna";
        break;
    case OE_RR_X1(LNK):
    case OE_RR_Y1(LNK):
        if (srca) {
            return TILEGX_EXCP_OPCODE_UNKNOWN;
        }
        tcg_gen_movi_tl(tdest, dc->pc + TILEGX_BUNDLE_SIZE_IN_BYTES);
        mnemonic = "lnk";
        break;
    case OE_RR_X0(PCNT):
    case OE_RR_Y0(PCNT):
        tcg_gen_ctpop_tl(tdest, tsrca);
        mnemonic = "pcnt";
        break;
    case OE_RR_X0(REVBITS):
    case OE_RR_Y0(REVBITS):
        gen_helper_revbits(tdest, tsrca);
        mnemonic = "revbits";
        break;
    case OE_RR_X0(REVBYTES):
    case OE_RR_Y0(REVBYTES):
        tcg_gen_bswap64_tl(tdest, tsrca);
        mnemonic = "revbytes";
        break;
    case OE_RR_X0(TBLIDXB0):
    case OE_RR_Y0(TBLIDXB0):
        tcg_gen_deposit_tl(tdest, load_gr(dc, dest), tsrca, 2, 8);
        mnemonic = "tblidxb0";
        break;
    case OE_RR_X0(TBLIDXB1):
    case OE_RR_Y0(TBLIDXB1):
        tcg_gen_shri_tl(tdest, tsrca, 8);
        tcg_gen_deposit_tl(tdest, load_gr(dc, dest), tdest, 2, 8);
        mnemonic = "tblidxb1";
        break;
    case OE_RR_X0(TBLIDXB2):
    case OE_RR_Y0(TBLIDXB2):
        tcg_gen_shri_tl(tdest, tsrca, 16);
        tcg_gen_deposit_tl(tdest, load_gr(dc, dest), tdest, 2, 8);
        mnemonic = "tblidxb2";
        break;
    case OE_RR_X0(TBLIDXB3):
    case OE_RR_Y0(TBLIDXB3):
        tcg_gen_shri_tl(tdest, tsrca, 24);
        tcg_gen_deposit_tl(tdest, load_gr(dc, dest), tdest, 2, 8);
        mnemonic = "tblidxb3";
        break;
    default:
        return TILEGX_EXCP_OPCODE_UNKNOWN;
    }

    qemu_log_mask(CPU_LOG_TB_IN_ASM, "%s %s, %s", mnemonic,
                  reg_names[dest], reg_names[srca]);
    return ret;
}

static TileExcp gen_rrr_opcode(DisasContext *dc, unsigned opext,
                               unsigned dest, unsigned srca, unsigned srcb)
{
    TCGv tdest = dest_gr(dc, dest);
    TCGv tsrca = load_gr(dc, srca);
    TCGv tsrcb = load_gr(dc, srcb);
    TCGv t0;
    const char *mnemonic;

    switch (opext) {
    case OE_RRR(ADDXSC, 0, X0):
    case OE_RRR(ADDXSC, 0, X1):
        gen_saturate_op(tdest, tsrca, tsrcb, tcg_gen_add_tl);
        mnemonic = "addxsc";
        break;
    case OE_RRR(ADDX, 0, X0):
    case OE_RRR(ADDX, 0, X1):
    case OE_RRR(ADDX, 0, Y0):
    case OE_RRR(ADDX, 0, Y1):
        tcg_gen_add_tl(tdest, tsrca, tsrcb);
        tcg_gen_ext32s_tl(tdest, tdest);
        mnemonic = "addx";
        break;
    case OE_RRR(ADD, 0, X0):
    case OE_RRR(ADD, 0, X1):
    case OE_RRR(ADD, 0, Y0):
    case OE_RRR(ADD, 0, Y1):
        tcg_gen_add_tl(tdest, tsrca, tsrcb);
        mnemonic = "add";
        break;
    case OE_RRR(AND, 0, X0):
    case OE_RRR(AND, 0, X1):
    case OE_RRR(AND, 5, Y0):
    case OE_RRR(AND, 5, Y1):
        tcg_gen_and_tl(tdest, tsrca, tsrcb);
        mnemonic = "and";
        break;
    case OE_RRR(CMOVEQZ, 0, X0):
    case OE_RRR(CMOVEQZ, 4, Y0):
        tcg_gen_movcond_tl(TCG_COND_EQ, tdest, tsrca, load_zero(dc),
                           tsrcb, load_gr(dc, dest));
        mnemonic = "cmoveqz";
        break;
    case OE_RRR(CMOVNEZ, 0, X0):
    case OE_RRR(CMOVNEZ, 4, Y0):
        tcg_gen_movcond_tl(TCG_COND_NE, tdest, tsrca, load_zero(dc),
                           tsrcb, load_gr(dc, dest));
        mnemonic = "cmovnez";
        break;
    case OE_RRR(CMPEQ, 0, X0):
    case OE_RRR(CMPEQ, 0, X1):
    case OE_RRR(CMPEQ, 3, Y0):
    case OE_RRR(CMPEQ, 3, Y1):
        tcg_gen_setcond_tl(TCG_COND_EQ, tdest, tsrca, tsrcb);
        mnemonic = "cmpeq";
        break;
    case OE_RRR(CMPEXCH4, 0, X1):
        gen_atomic(dc, tdest, tsrca, tsrcb, AT_CMPEXCH, true);
        mnemonic = "cmpexch4";
        break;
    case OE_RRR(CMPEXCH, 0, X1):
        gen_atomic(dc, tdest, tsrca, tsrcb, AT_CMPEXCH, false);
        mnemonic = "cmpexch";
        break;
    case OE_RRR(CMPLES, 0, X0):
    case OE_RRR(CMPLES, 0, X1):
    case OE_RRR(CMPLES, 2, Y0):
    case OE_RRR(CMPLES, 2, Y1):
        tcg_gen_setcond_tl(TCG_COND_LE, tdest, tsrca, tsrcb);
        mnemonic = "cmples";
        break;
    case OE_RRR(CMPLEU, 0, X0):
    case OE_RRR(CMPLEU, 0, X1):
    case OE_RRR(CMPLEU, 2, Y0):
    case OE_RRR(CMPLEU, 2, Y1):
        tcg_gen_setcond_tl(TCG_COND_LEU, tdest, tsrca, tsrcb);
        mnemonic = "cmpleu";
        break;
    case OE_RRR(CMPLTS, 0, X0):
    case OE_RRR(CMPLTS, 0, X1):
    case OE_RRR(CMPLTS, 2, Y0):
    case OE_RRR(CMPLTS, 2, Y1):
        tcg_gen_setcond_tl(TCG_COND_LT, tdest, tsrca, tsrcb);
        mnemonic = "cmplts";
        break;
    case OE_RRR(CMPLTU, 0, X0):
    case OE_RRR(CMPLTU, 0, X1):
    case OE_RRR(CMPLTU, 2, Y0):
    case OE_RRR(CMPLTU, 2, Y1):
        tcg_gen_setcond_tl(TCG_COND_LTU, tdest, tsrca, tsrcb);
        mnemonic = "cmpltu";
        break;
    case OE_RRR(CMPNE, 0, X0):
    case OE_RRR(CMPNE, 0, X1):
    case OE_RRR(CMPNE, 3, Y0):
    case OE_RRR(CMPNE, 3, Y1):
        tcg_gen_setcond_tl(TCG_COND_NE, tdest, tsrca, tsrcb);
        mnemonic = "cmpne";
        break;
    case OE_RRR(CMULAF, 0, X0):
        gen_helper_cmulaf(tdest, load_gr(dc, dest), tsrca, tsrcb);
        mnemonic = "cmulaf";
        break;
    case OE_RRR(CMULA, 0, X0):
        gen_helper_cmula(tdest, load_gr(dc, dest), tsrca, tsrcb);
        mnemonic = "cmula";
        break;
    case OE_RRR(CMULFR, 0, X0):
        gen_cmul2(tdest, tsrca, tsrcb, 15, 1 << 14);
        mnemonic = "cmulfr";
        break;
    case OE_RRR(CMULF, 0, X0):
        gen_cmul2(tdest, tsrca, tsrcb, 15, 0);
        mnemonic = "cmulf";
        break;
    case OE_RRR(CMULHR, 0, X0):
        gen_cmul2(tdest, tsrca, tsrcb, 16, 1 << 15);
        mnemonic = "cmulhr";
        break;
    case OE_RRR(CMULH, 0, X0):
        gen_cmul2(tdest, tsrca, tsrcb, 16, 0);
        mnemonic = "cmulh";
        break;
    case OE_RRR(CMUL, 0, X0):
        gen_helper_cmula(tdest, load_zero(dc), tsrca, tsrcb);
        mnemonic = "cmul";
        break;
    case OE_RRR(CRC32_32, 0, X0):
        gen_helper_crc32_32(tdest, tsrca, tsrcb);
        mnemonic = "crc32_32";
        break;
    case OE_RRR(CRC32_8, 0, X0):
        gen_helper_crc32_8(tdest, tsrca, tsrcb);
        mnemonic = "crc32_8";
        break;
    case OE_RRR(DBLALIGN2, 0, X0):
    case OE_RRR(DBLALIGN2, 0, X1):
        gen_dblaligni(tdest, tsrca, tsrcb, 16);
        mnemonic = "dblalign2";
        break;
    case OE_RRR(DBLALIGN4, 0, X0):
    case OE_RRR(DBLALIGN4, 0, X1):
        gen_dblaligni(tdest, tsrca, tsrcb, 32);
        mnemonic = "dblalign4";
        break;
    case OE_RRR(DBLALIGN6, 0, X0):
    case OE_RRR(DBLALIGN6, 0, X1):
        gen_dblaligni(tdest, tsrca, tsrcb, 48);
        mnemonic = "dblalign6";
        break;
    case OE_RRR(DBLALIGN, 0, X0):
        gen_dblalign(tdest, load_gr(dc, dest), tsrca, tsrcb);
        mnemonic = "dblalign";
        break;
    case OE_RRR(EXCH4, 0, X1):
        gen_atomic(dc, tdest, tsrca, tsrcb, AT_EXCH, true);
        mnemonic = "exch4";
        break;
    case OE_RRR(EXCH, 0, X1):
        gen_atomic(dc, tdest, tsrca, tsrcb, AT_EXCH, false);
        mnemonic = "exch";
        break;
    case OE_RRR(FDOUBLE_ADDSUB, 0, X0):
        gen_helper_fdouble_addsub(tdest, load_gr(dc, dest), tsrca, tsrcb);
        mnemonic = "fdouble_addsub";
        break;
    case OE_RRR(FDOUBLE_ADD_FLAGS, 0, X0):
        gen_helper_fdouble_add_flags(tdest, tsrca, tsrcb);
        mnemonic = "fdouble_add_flags";
        break;
    case OE_RRR(FDOUBLE_MUL_FLAGS, 0, X0):
        gen_helper_fdouble_mul_flags(tdest, tsrca, tsrcb);
        mnemonic = "fdouble_mul_flags";
        break;
    case OE_RRR(FDOUBLE_PACK1, 0, X0):
        gen_helper_fdouble_pack1(tdest, tsrca, tsrcb);
        mnemonic = "fdouble_pack1";
        break;
    case OE_RRR(FDOUBLE_PACK2, 0, X0):
        gen_helper_fdouble_pack2(tdest, load_gr(dc, dest), tsrca, tsrcb);
        mnemonic = "fdouble_pack2";
        break;
    case OE_RRR(FDOUBLE_SUB_FLAGS, 0, X0):
        gen_helper_fdouble_sub_flags(tdest, tsrca, tsrcb);
        mnemonic = "fdouble_sub_flags";
        break;
    case OE_RRR(FDOUBLE_UNPACK_MAX, 0, X0):
        gen_helper_fdouble_unpack_max(tdest, tsrca, tsrcb);
        mnemonic = "fdouble_unpack_max";
        break;
    case OE_RRR(FDOUBLE_UNPACK_MIN, 0, X0):
        gen_helper_fdouble_unpack_min(tdest, tsrca, tsrcb);
        mnemonic = "fdouble_unpack_min";
        break;
    case OE_RRR(FETCHADD4, 0, X1):
        gen_atomic(dc, tdest, tsrca, tsrcb, AT_FETCHADD, true);
        mnemonic = "fetchadd4";
        break;
    case OE_RRR(FETCHADDGEZ4, 0, X1):
        gen_atomic(dc, tdest, tsrca, tsrcb, AT_FETCHADDGEZ, true);
        mnemonic = "fetchaddgez4";
        break;
    case OE_RRR(FETCHADDGEZ, 0, X1):
        gen_atomic(dc, tdest, tsrca, tsrcb, AT_FETCHADDGEZ, false);
        mnemonic = "fetchaddgez";
        break;
    case OE_RRR(FETCHADD, 0, X1):
        gen_atomic(dc, tdest, tsrca, tsrcb, AT_FETCHADD, false);
        mnemonic = "fetchadd";
        break;
    case OE_RRR(FETCHAND4, 0, X1):
        gen_atomic(dc, tdest, tsrca, tsrcb, AT_FETCHAND, true);
        mnemonic = "fetchand4";
        break;
    case OE_RRR(FETCHAND, 0, X1):
        gen_atomic(dc, tdest, tsrca, tsrcb, AT_FETCHAND, false);
        mnemonic = "fetchand";
        break;
    case OE_RRR(FETCHOR4, 0, X1):
        gen_atomic(dc, tdest, tsrca, tsrcb, AT_FETCHOR, true);
        mnemonic = "fetchor4";
        break;
    case OE_RRR(FETCHOR, 0, X1):
        gen_atomic(dc, tdest, tsrca, tsrcb, AT_FETCHOR, false);
        mnemonic = "fetchor";
        break;
    case OE_RRR(FSINGLE_ADD1, 0, X0):
        gen_helper_fsingle_add1(tdest, tsrca, tsrcb);
        mnemonic = "fsingle_add1";
        break;
    case OE_RRR(FSINGLE_ADDSUB2, 0, X0):
        gen_helper_fsingle_addsub2(tdest, load_gr(dc, dest), tsrca, tsrcb);
        mnemonic = "fsingle_addsub2";
        break;
    case OE_RRR(FSINGLE_MUL1, 0, X0):
        gen_helper_fsingle_mul1(tdest, tsrca, tsrcb);
        mnemonic = "fsingle_mul1";
        break;
    case OE_RRR(FSINGLE_MUL2, 0, X0):
        gen_helper_fsingle_mul2(tdest, tsrca, tsrcb);
        mnemonic = "fsingle_mul2";
        break;
    case OE_RRR(FSINGLE_PACK2, 0, X0):
        gen_helper_fsingle_pack2(tdest, tsrca, tsrcb);
        mnemonic = "fsingle_pack2";
        break;
    case OE_RRR(FSINGLE_SUB1, 0, X0):
        gen_helper_fsingle_sub1(tdest, tsrca, tsrcb);
        mnemonic = "fsingle_sub1";
        break;
    case OE_RRR(MNZ, 0, X0):
    case OE_RRR(MNZ, 0, X1):
    case OE_RRR(MNZ, 4, Y0):
    case OE_RRR(MNZ, 4, Y1):
        t0 = load_zero(dc);
        tcg_gen_movcond_tl(TCG_COND_NE, tdest, tsrca, t0, tsrcb, t0);
        mnemonic = "mnz";
        break;
    case OE_RRR(MULAX, 0, X0):
    case OE_RRR(MULAX, 3, Y0):
        tcg_gen_mul_tl(tdest, tsrca, tsrcb);
        tcg_gen_add_tl(tdest, tdest, load_gr(dc, dest));
        tcg_gen_ext32s_tl(tdest, tdest);
        mnemonic = "mulax";
        break;
    case OE_RRR(MULA_HS_HS, 0, X0):
    case OE_RRR(MULA_HS_HS, 9, Y0):
        gen_mul_half(tdest, tsrca, tsrcb, HS, HS);
        tcg_gen_add_tl(tdest, tdest, load_gr(dc, dest));
        mnemonic = "mula_hs_hs";
        break;
    case OE_RRR(MULA_HS_HU, 0, X0):
        gen_mul_half(tdest, tsrca, tsrcb, HS, HU);
        tcg_gen_add_tl(tdest, tdest, load_gr(dc, dest));
        mnemonic = "mula_hs_hu";
        break;
    case OE_RRR(MULA_HS_LS, 0, X0):
        gen_mul_half(tdest, tsrca, tsrcb, HS, LS);
        tcg_gen_add_tl(tdest, tdest, load_gr(dc, dest));
        mnemonic = "mula_hs_ls";
        break;
    case OE_RRR(MULA_HS_LU, 0, X0):
        gen_mul_half(tdest, tsrca, tsrcb, HS, LU);
        tcg_gen_add_tl(tdest, tdest, load_gr(dc, dest));
        mnemonic = "mula_hs_lu";
        break;
    case OE_RRR(MULA_HU_HU, 0, X0):
    case OE_RRR(MULA_HU_HU, 9, Y0):
        gen_mul_half(tdest, tsrca, tsrcb, HU, HU);
        tcg_gen_add_tl(tdest, tdest, load_gr(dc, dest));
        mnemonic = "mula_hu_hu";
        break;
    case OE_RRR(MULA_HU_LS, 0, X0):
        gen_mul_half(tdest, tsrca, tsrcb, HU, LS);
        tcg_gen_add_tl(tdest, tdest, load_gr(dc, dest));
        mnemonic = "mula_hu_ls";
        break;
    case OE_RRR(MULA_HU_LU, 0, X0):
        gen_mul_half(tdest, tsrca, tsrcb, HU, LU);
        tcg_gen_add_tl(tdest, tdest, load_gr(dc, dest));
        mnemonic = "mula_hu_lu";
        break;
    case OE_RRR(MULA_LS_LS, 0, X0):
    case OE_RRR(MULA_LS_LS, 9, Y0):
        gen_mul_half(tdest, tsrca, tsrcb, LS, LS);
        tcg_gen_add_tl(tdest, tdest, load_gr(dc, dest));
        mnemonic = "mula_ls_ls";
        break;
    case OE_RRR(MULA_LS_LU, 0, X0):
        gen_mul_half(tdest, tsrca, tsrcb, LS, LU);
        tcg_gen_add_tl(tdest, tdest, load_gr(dc, dest));
        mnemonic = "mula_ls_lu";
        break;
    case OE_RRR(MULA_LU_LU, 0, X0):
    case OE_RRR(MULA_LU_LU, 9, Y0):
        gen_mul_half(tdest, tsrca, tsrcb, LU, LU);
        tcg_gen_add_tl(tdest, tdest, load_gr(dc, dest));
        mnemonic = "mula_lu_lu";
        break;
    case OE_RRR(MULX, 0, X0):
    case OE_RRR(MULX, 3, Y0):
        tcg_gen_mul_tl(tdest, tsrca, tsrcb);
        tcg_gen_ext32s_tl(tdest, tdest);
        mnemonic = "mulx";
        break;
    case OE_RRR(MUL_HS_HS, 0, X0):
    case OE_RRR(MUL_HS_HS, 8, Y0):
        gen_mul_half(tdest, tsrca, tsrcb, HS, HS);
        mnemonic = "mul_hs_hs";
        break;
    case OE_RRR(MUL_HS_HU, 0, X0):
        gen_mul_half(tdest, tsrca, tsrcb, HS, HU);
        mnemonic = "mul_hs_hu";
        break;
    case OE_RRR(MUL_HS_LS, 0, X0):
        gen_mul_half(tdest, tsrca, tsrcb, HS, LS);
        mnemonic = "mul_hs_ls";
        break;
    case OE_RRR(MUL_HS_LU, 0, X0):
        gen_mul_half(tdest, tsrca, tsrcb, HS, LU);
        mnemonic = "mul_hs_lu";
        break;
    case OE_RRR(MUL_HU_HU, 0, X0):
    case OE_RRR(MUL_HU_HU, 8, Y0):
        gen_mul_half(tdest, tsrca, tsrcb, HU, HU);
        mnemonic = "mul_hu_hu";
        break;
    case OE_RRR(MUL_HU_LS, 0, X0):
        gen_mul_half(tdest, tsrca, tsrcb, HU, LS);
        mnemonic = "mul_hu_ls";
        break;
    case OE_RRR(MUL_HU_LU, 0, X0):
        gen_mul_half(tdest, tsrca, tsrcb, HU, LU);
        mnemonic = "mul_hu_lu";
        break;
    case OE_RRR(MUL_LS_LS, 0, X0):
    case OE_RRR(MUL_LS_LS, 8, Y0):
        gen_mul_half(tdest, tsrca, tsrcb, LS, LS);
        mnemonic = "mul_ls_ls";
        break;
    case OE_RRR(MUL_LS_LU, 0, X0):
        gen_mul_half(tdest, tsrca, tsrcb, LS, LU);
        mnemonic = "mul_ls_lu";
        break;
    case OE_RRR(MUL_LU_LU, 0, X0):
    case OE_RRR(MUL_LU_LU, 8, Y0):
        gen_mul_half(tdest, tsrca, tsrcb, LU, LU);
        mnemonic = "mul_lu_lu";
        break;
    case OE_RRR(MZ, 0, X0):
    case OE_RRR(MZ, 0, X1):
    case OE_RRR(MZ, 4, Y0):
    case OE_RRR(MZ, 4, Y1):
        t0 = load_zero(dc);
        tcg_gen_movcond_tl(TCG_COND_EQ, tdest, tsrca, t0, tsrcb, t0);
        mnemonic = "mz";
        break;
    case OE_RRR(NOR, 0, X0):
    case OE_RRR(NOR, 0, X1):
    case OE_RRR(NOR, 5, Y0):
    case OE_RRR(NOR, 5, Y1):
        tcg_gen_nor_tl(tdest, tsrca, tsrcb);
        mnemonic = "nor";
        break;
    case OE_RRR(OR, 0, X0):
    case OE_RRR(OR, 0, X1):
    case OE_RRR(OR, 5, Y0):
    case OE_RRR(OR, 5, Y1):
        tcg_gen_or_tl(tdest, tsrca, tsrcb);
        mnemonic = "or";
        break;
    case OE_RRR(ROTL, 0, X0):
    case OE_RRR(ROTL, 0, X1):
    case OE_RRR(ROTL, 6, Y0):
    case OE_RRR(ROTL, 6, Y1):
        tcg_gen_andi_tl(tdest, tsrcb, 63);
        tcg_gen_rotl_tl(tdest, tsrca, tdest);
        mnemonic = "rotl";
        break;
    case OE_RRR(SHL1ADDX, 0, X0):
    case OE_RRR(SHL1ADDX, 0, X1):
    case OE_RRR(SHL1ADDX, 7, Y0):
    case OE_RRR(SHL1ADDX, 7, Y1):
        tcg_gen_shli_tl(tdest, tsrca, 1);
        tcg_gen_add_tl(tdest, tdest, tsrcb);
        tcg_gen_ext32s_tl(tdest, tdest);
        mnemonic = "shl1addx";
        break;
    case OE_RRR(SHL1ADD, 0, X0):
    case OE_RRR(SHL1ADD, 0, X1):
    case OE_RRR(SHL1ADD, 1, Y0):
    case OE_RRR(SHL1ADD, 1, Y1):
        tcg_gen_shli_tl(tdest, tsrca, 1);
        tcg_gen_add_tl(tdest, tdest, tsrcb);
        mnemonic = "shl1add";
        break;
    case OE_RRR(SHL2ADDX, 0, X0):
    case OE_RRR(SHL2ADDX, 0, X1):
    case OE_RRR(SHL2ADDX, 7, Y0):
    case OE_RRR(SHL2ADDX, 7, Y1):
        tcg_gen_shli_tl(tdest, tsrca, 2);
        tcg_gen_add_tl(tdest, tdest, tsrcb);
        tcg_gen_ext32s_tl(tdest, tdest);
        mnemonic = "shl2addx";
        break;
    case OE_RRR(SHL2ADD, 0, X0):
    case OE_RRR(SHL2ADD, 0, X1):
    case OE_RRR(SHL2ADD, 1, Y0):
    case OE_RRR(SHL2ADD, 1, Y1):
        tcg_gen_shli_tl(tdest, tsrca, 2);
        tcg_gen_add_tl(tdest, tdest, tsrcb);
        mnemonic = "shl2add";
        break;
    case OE_RRR(SHL3ADDX, 0, X0):
    case OE_RRR(SHL3ADDX, 0, X1):
    case OE_RRR(SHL3ADDX, 7, Y0):
    case OE_RRR(SHL3ADDX, 7, Y1):
        tcg_gen_shli_tl(tdest, tsrca, 3);
        tcg_gen_add_tl(tdest, tdest, tsrcb);
        tcg_gen_ext32s_tl(tdest, tdest);
        mnemonic = "shl3addx";
        break;
    case OE_RRR(SHL3ADD, 0, X0):
    case OE_RRR(SHL3ADD, 0, X1):
    case OE_RRR(SHL3ADD, 1, Y0):
    case OE_RRR(SHL3ADD, 1, Y1):
        tcg_gen_shli_tl(tdest, tsrca, 3);
        tcg_gen_add_tl(tdest, tdest, tsrcb);
        mnemonic = "shl3add";
        break;
    case OE_RRR(SHLX, 0, X0):
    case OE_RRR(SHLX, 0, X1):
        tcg_gen_andi_tl(tdest, tsrcb, 31);
        tcg_gen_shl_tl(tdest, tsrca, tdest);
        tcg_gen_ext32s_tl(tdest, tdest);
        mnemonic = "shlx";
        break;
    case OE_RRR(SHL, 0, X0):
    case OE_RRR(SHL, 0, X1):
    case OE_RRR(SHL, 6, Y0):
    case OE_RRR(SHL, 6, Y1):
        tcg_gen_andi_tl(tdest, tsrcb, 63);
        tcg_gen_shl_tl(tdest, tsrca, tdest);
        mnemonic = "shl";
        break;
    case OE_RRR(SHRS, 0, X0):
    case OE_RRR(SHRS, 0, X1):
    case OE_RRR(SHRS, 6, Y0):
    case OE_RRR(SHRS, 6, Y1):
        tcg_gen_andi_tl(tdest, tsrcb, 63);
        tcg_gen_sar_tl(tdest, tsrca, tdest);
        mnemonic = "shrs";
        break;
    case OE_RRR(SHRUX, 0, X0):
    case OE_RRR(SHRUX, 0, X1):
        t0 = tcg_temp_new();
        tcg_gen_andi_tl(t0, tsrcb, 31);
        tcg_gen_ext32u_tl(tdest, tsrca);
        tcg_gen_shr_tl(tdest, tdest, t0);
        tcg_gen_ext32s_tl(tdest, tdest);
        mnemonic = "shrux";
        break;
    case OE_RRR(SHRU, 0, X0):
    case OE_RRR(SHRU, 0, X1):
    case OE_RRR(SHRU, 6, Y0):
    case OE_RRR(SHRU, 6, Y1):
        tcg_gen_andi_tl(tdest, tsrcb, 63);
        tcg_gen_shr_tl(tdest, tsrca, tdest);
        mnemonic = "shru";
        break;
    case OE_RRR(SHUFFLEBYTES, 0, X0):
        gen_helper_shufflebytes(tdest, load_gr(dc, dest), tsrca, tsrca);
        mnemonic = "shufflebytes";
        break;
    case OE_RRR(SUBXSC, 0, X0):
    case OE_RRR(SUBXSC, 0, X1):
        gen_saturate_op(tdest, tsrca, tsrcb, tcg_gen_sub_tl);
        mnemonic = "subxsc";
        break;
    case OE_RRR(SUBX, 0, X0):
    case OE_RRR(SUBX, 0, X1):
    case OE_RRR(SUBX, 0, Y0):
    case OE_RRR(SUBX, 0, Y1):
        tcg_gen_sub_tl(tdest, tsrca, tsrcb);
        tcg_gen_ext32s_tl(tdest, tdest);
        mnemonic = "subx";
        break;
    case OE_RRR(SUB, 0, X0):
    case OE_RRR(SUB, 0, X1):
    case OE_RRR(SUB, 0, Y0):
    case OE_RRR(SUB, 0, Y1):
        tcg_gen_sub_tl(tdest, tsrca, tsrcb);
        mnemonic = "sub";
        break;
    case OE_RRR(V1ADDUC, 0, X0):
    case OE_RRR(V1ADDUC, 0, X1):
        return TILEGX_EXCP_OPCODE_UNIMPLEMENTED;
    case OE_RRR(V1ADD, 0, X0):
    case OE_RRR(V1ADD, 0, X1):
        gen_v12add(tdest, tsrca, tsrcb, V1_IMM(0x80));
        mnemonic = "v1add";
        break;
    case OE_RRR(V1ADIFFU, 0, X0):
    case OE_RRR(V1AVGU, 0, X0):
        return TILEGX_EXCP_OPCODE_UNIMPLEMENTED;
    case OE_RRR(V1CMPEQ, 0, X0):
    case OE_RRR(V1CMPEQ, 0, X1):
        tcg_gen_xor_tl(tdest, tsrca, tsrcb);
        gen_v1cmpeq0(tdest);
        mnemonic = "v1cmpeq";
        break;
    case OE_RRR(V1CMPLES, 0, X0):
    case OE_RRR(V1CMPLES, 0, X1):
    case OE_RRR(V1CMPLEU, 0, X0):
    case OE_RRR(V1CMPLEU, 0, X1):
    case OE_RRR(V1CMPLTS, 0, X0):
    case OE_RRR(V1CMPLTS, 0, X1):
    case OE_RRR(V1CMPLTU, 0, X0):
    case OE_RRR(V1CMPLTU, 0, X1):
        return TILEGX_EXCP_OPCODE_UNIMPLEMENTED;
    case OE_RRR(V1CMPNE, 0, X0):
    case OE_RRR(V1CMPNE, 0, X1):
        tcg_gen_xor_tl(tdest, tsrca, tsrcb);
        gen_v1cmpne0(tdest);
        mnemonic = "v1cmpne";
        break;
    case OE_RRR(V1DDOTPUA, 0, X0):
    case OE_RRR(V1DDOTPUSA, 0, X0):
    case OE_RRR(V1DDOTPUS, 0, X0):
    case OE_RRR(V1DDOTPU, 0, X0):
    case OE_RRR(V1DOTPA, 0, X0):
    case OE_RRR(V1DOTPUA, 0, X0):
    case OE_RRR(V1DOTPUSA, 0, X0):
    case OE_RRR(V1DOTPUS, 0, X0):
    case OE_RRR(V1DOTPU, 0, X0):
    case OE_RRR(V1DOTP, 0, X0):
        return TILEGX_EXCP_OPCODE_UNIMPLEMENTED;
    case OE_RRR(V1INT_H, 0, X0):
    case OE_RRR(V1INT_H, 0, X1):
        gen_helper_v1int_h(tdest, tsrca, tsrcb);
        mnemonic = "v1int_h";
        break;
    case OE_RRR(V1INT_L, 0, X0):
    case OE_RRR(V1INT_L, 0, X1):
        gen_helper_v1int_l(tdest, tsrca, tsrcb);
        mnemonic = "v1int_l";
        break;
    case OE_RRR(V1MAXU, 0, X0):
    case OE_RRR(V1MAXU, 0, X1):
    case OE_RRR(V1MINU, 0, X0):
    case OE_RRR(V1MINU, 0, X1):
    case OE_RRR(V1MNZ, 0, X0):
    case OE_RRR(V1MNZ, 0, X1):
        return TILEGX_EXCP_OPCODE_UNIMPLEMENTED;
    case OE_RRR(V1MULTU, 0, X0):
        gen_helper_v1multu(tdest, tsrca, tsrcb);
        mnemonic = "v1multu";
        break;
    case OE_RRR(V1MULUS, 0, X0):
    case OE_RRR(V1MULU, 0, X0):
    case OE_RRR(V1MZ, 0, X0):
    case OE_RRR(V1MZ, 0, X1):
    case OE_RRR(V1SADAU, 0, X0):
    case OE_RRR(V1SADU, 0, X0):
        return TILEGX_EXCP_OPCODE_UNIMPLEMENTED;
    case OE_RRR(V1SHL, 0, X0):
    case OE_RRR(V1SHL, 0, X1):
        gen_helper_v1shl(tdest, tsrca, tsrcb);
        mnemonic = "v1shl";
        break;
    case OE_RRR(V1SHRS, 0, X0):
    case OE_RRR(V1SHRS, 0, X1):
        gen_helper_v1shrs(tdest, tsrca, tsrcb);
        mnemonic = "v1shrs";
        break;
    case OE_RRR(V1SHRU, 0, X0):
    case OE_RRR(V1SHRU, 0, X1):
        gen_helper_v1shru(tdest, tsrca, tsrcb);
        mnemonic = "v1shru";
        break;
    case OE_RRR(V1SUBUC, 0, X0):
    case OE_RRR(V1SUBUC, 0, X1):
        return TILEGX_EXCP_OPCODE_UNIMPLEMENTED;
    case OE_RRR(V1SUB, 0, X0):
    case OE_RRR(V1SUB, 0, X1):
        gen_v12sub(tdest, tsrca, tsrcb, V1_IMM(0x80));
        mnemonic = "v1sub";
        break;
    case OE_RRR(V2ADDSC, 0, X0):
    case OE_RRR(V2ADDSC, 0, X1):
        return TILEGX_EXCP_OPCODE_UNIMPLEMENTED;
    case OE_RRR(V2ADD, 0, X0):
    case OE_RRR(V2ADD, 0, X1):
        gen_v12add(tdest, tsrca, tsrcb, V2_IMM(0x8000));
        mnemonic = "v2add";
        break;
    case OE_RRR(V2ADIFFS, 0, X0):
    case OE_RRR(V2AVGS, 0, X0):
    case OE_RRR(V2CMPEQ, 0, X0):
    case OE_RRR(V2CMPEQ, 0, X1):
    case OE_RRR(V2CMPLES, 0, X0):
    case OE_RRR(V2CMPLES, 0, X1):
    case OE_RRR(V2CMPLEU, 0, X0):
    case OE_RRR(V2CMPLEU, 0, X1):
    case OE_RRR(V2CMPLTS, 0, X0):
    case OE_RRR(V2CMPLTS, 0, X1):
    case OE_RRR(V2CMPLTU, 0, X0):
    case OE_RRR(V2CMPLTU, 0, X1):
    case OE_RRR(V2CMPNE, 0, X0):
    case OE_RRR(V2CMPNE, 0, X1):
    case OE_RRR(V2DOTPA, 0, X0):
    case OE_RRR(V2DOTP, 0, X0):
        return TILEGX_EXCP_OPCODE_UNIMPLEMENTED;
    case OE_RRR(V2INT_H, 0, X0):
    case OE_RRR(V2INT_H, 0, X1):
        gen_helper_v2int_h(tdest, tsrca, tsrcb);
        mnemonic = "v2int_h";
        break;
    case OE_RRR(V2INT_L, 0, X0):
    case OE_RRR(V2INT_L, 0, X1):
        gen_helper_v2int_l(tdest, tsrca, tsrcb);
        mnemonic = "v2int_l";
        break;
    case OE_RRR(V2MAXS, 0, X0):
    case OE_RRR(V2MAXS, 0, X1):
    case OE_RRR(V2MINS, 0, X0):
    case OE_RRR(V2MINS, 0, X1):
    case OE_RRR(V2MNZ, 0, X0):
    case OE_RRR(V2MNZ, 0, X1):
    case OE_RRR(V2MULFSC, 0, X0):
    case OE_RRR(V2MULS, 0, X0):
        return TILEGX_EXCP_OPCODE_UNIMPLEMENTED;
    case OE_RRR(V2MULTS, 0, X0):
        gen_helper_v2mults(tdest, tsrca, tsrcb);
        mnemonic = "v2mults";
        break;
    case OE_RRR(V2MZ, 0, X0):
    case OE_RRR(V2MZ, 0, X1):
    case OE_RRR(V2PACKH, 0, X0):
    case OE_RRR(V2PACKH, 0, X1):
    case OE_RRR(V2PACKL, 0, X0):
    case OE_RRR(V2PACKL, 0, X1):
    case OE_RRR(V2PACKUC, 0, X0):
    case OE_RRR(V2PACKUC, 0, X1):
    case OE_RRR(V2SADAS, 0, X0):
    case OE_RRR(V2SADAU, 0, X0):
    case OE_RRR(V2SADS, 0, X0):
    case OE_RRR(V2SADU, 0, X0):
    case OE_RRR(V2SHLSC, 0, X0):
    case OE_RRR(V2SHLSC, 0, X1):
        return TILEGX_EXCP_OPCODE_UNIMPLEMENTED;
    case OE_RRR(V2SHL, 0, X0):
    case OE_RRR(V2SHL, 0, X1):
        gen_helper_v2shl(tdest, tsrca, tsrcb);
        mnemonic = "v2shl";
        break;
    case OE_RRR(V2SHRS, 0, X0):
    case OE_RRR(V2SHRS, 0, X1):
        gen_helper_v2shrs(tdest, tsrca, tsrcb);
        mnemonic = "v2shrs";
        break;
    case OE_RRR(V2SHRU, 0, X0):
    case OE_RRR(V2SHRU, 0, X1):
        gen_helper_v2shru(tdest, tsrca, tsrcb);
        mnemonic = "v2shru";
        break;
    case OE_RRR(V2SUBSC, 0, X0):
    case OE_RRR(V2SUBSC, 0, X1):
        return TILEGX_EXCP_OPCODE_UNIMPLEMENTED;
    case OE_RRR(V2SUB, 0, X0):
    case OE_RRR(V2SUB, 0, X1):
        gen_v12sub(tdest, tsrca, tsrcb, V2_IMM(0x8000));
        mnemonic = "v2sub";
        break;
    case OE_RRR(V4ADDSC, 0, X0):
    case OE_RRR(V4ADDSC, 0, X1):
        return TILEGX_EXCP_OPCODE_UNIMPLEMENTED;
    case OE_RRR(V4ADD, 0, X0):
    case OE_RRR(V4ADD, 0, X1):
        gen_v4op(tdest, tsrca, tsrcb, tcg_gen_add_i32);
        mnemonic = "v4add";
        break;
    case OE_RRR(V4INT_H, 0, X0):
    case OE_RRR(V4INT_H, 0, X1):
        tcg_gen_shri_tl(tdest, tsrcb, 32);
        tcg_gen_deposit_tl(tdest, tsrca, tdest, 0, 32);
        mnemonic = "v4int_h";
        break;
    case OE_RRR(V4INT_L, 0, X0):
    case OE_RRR(V4INT_L, 0, X1):
        tcg_gen_deposit_tl(tdest, tsrcb, tsrca, 32, 32);
        mnemonic = "v4int_l";
        break;
    case OE_RRR(V4PACKSC, 0, X0):
    case OE_RRR(V4PACKSC, 0, X1):
    case OE_RRR(V4SHLSC, 0, X0):
    case OE_RRR(V4SHLSC, 0, X1):
        return TILEGX_EXCP_OPCODE_UNIMPLEMENTED;
    case OE_RRR(V4SHL, 0, X0):
    case OE_RRR(V4SHL, 0, X1):
        gen_v4sh(tdest, tsrca, tsrcb, tcg_gen_shl_i32);
        mnemonic = "v4shl";
        break;
    case OE_RRR(V4SHRS, 0, X0):
    case OE_RRR(V4SHRS, 0, X1):
        gen_v4sh(tdest, tsrca, tsrcb, tcg_gen_sar_i32);
        mnemonic = "v4shrs";
        break;
    case OE_RRR(V4SHRU, 0, X0):
    case OE_RRR(V4SHRU, 0, X1):
        gen_v4sh(tdest, tsrca, tsrcb, tcg_gen_shr_i32);
        mnemonic = "v4shru";
        break;
    case OE_RRR(V4SUBSC, 0, X0):
    case OE_RRR(V4SUBSC, 0, X1):
        return TILEGX_EXCP_OPCODE_UNIMPLEMENTED;
    case OE_RRR(V4SUB, 0, X0):
    case OE_RRR(V4SUB, 0, X1):
        gen_v4op(tdest, tsrca, tsrcb, tcg_gen_sub_i32);
        mnemonic = "v2sub";
        break;
    case OE_RRR(XOR, 0, X0):
    case OE_RRR(XOR, 0, X1):
    case OE_RRR(XOR, 5, Y0):
    case OE_RRR(XOR, 5, Y1):
        tcg_gen_xor_tl(tdest, tsrca, tsrcb);
        mnemonic = "xor";
        break;
    default:
        return TILEGX_EXCP_OPCODE_UNKNOWN;
    }

    qemu_log_mask(CPU_LOG_TB_IN_ASM, "%s %s, %s, %s", mnemonic,
                  reg_names[dest], reg_names[srca], reg_names[srcb]);
    return TILEGX_EXCP_NONE;
}

static TileExcp gen_rri_opcode(DisasContext *dc, unsigned opext,
                               unsigned dest, unsigned srca, int imm)
{
    TCGv tdest = dest_gr(dc, dest);
    TCGv tsrca = load_gr(dc, srca);
    bool prefetch_nofault = false;
    const char *mnemonic;
    MemOp memop;
    int i2, i3;
    TCGv t0;

    switch (opext) {
    case OE(ADDI_OPCODE_Y0, 0, Y0):
    case OE(ADDI_OPCODE_Y1, 0, Y1):
    case OE_IM(ADDI, X0):
    case OE_IM(ADDI, X1):
        tcg_gen_addi_tl(tdest, tsrca, imm);
        mnemonic = "addi";
        break;
    case OE(ADDXI_OPCODE_Y0, 0, Y0):
    case OE(ADDXI_OPCODE_Y1, 0, Y1):
    case OE_IM(ADDXI, X0):
    case OE_IM(ADDXI, X1):
        tcg_gen_addi_tl(tdest, tsrca, imm);
        tcg_gen_ext32s_tl(tdest, tdest);
        mnemonic = "addxi";
        break;
    case OE(ANDI_OPCODE_Y0, 0, Y0):
    case OE(ANDI_OPCODE_Y1, 0, Y1):
    case OE_IM(ANDI, X0):
    case OE_IM(ANDI, X1):
        tcg_gen_andi_tl(tdest, tsrca, imm);
        mnemonic = "andi";
        break;
    case OE(CMPEQI_OPCODE_Y0, 0, Y0):
    case OE(CMPEQI_OPCODE_Y1, 0, Y1):
    case OE_IM(CMPEQI, X0):
    case OE_IM(CMPEQI, X1):
        tcg_gen_setcondi_tl(TCG_COND_EQ, tdest, tsrca, imm);
        mnemonic = "cmpeqi";
        break;
    case OE(CMPLTSI_OPCODE_Y0, 0, Y0):
    case OE(CMPLTSI_OPCODE_Y1, 0, Y1):
    case OE_IM(CMPLTSI, X0):
    case OE_IM(CMPLTSI, X1):
        tcg_gen_setcondi_tl(TCG_COND_LT, tdest, tsrca, imm);
        mnemonic = "cmpltsi";
        break;
    case OE_IM(CMPLTUI, X0):
    case OE_IM(CMPLTUI, X1):
        tcg_gen_setcondi_tl(TCG_COND_LTU, tdest, tsrca, imm);
        mnemonic = "cmpltui";
        break;
    case OE_IM(LD1S_ADD, X1):
        memop = MO_SB;
        mnemonic = "ld1s_add"; /* prefetch_add_l1_fault */
        goto do_load_add;
    case OE_IM(LD1U_ADD, X1):
        memop = MO_UB;
        mnemonic = "ld1u_add"; /* prefetch_add_l1 */
        prefetch_nofault = (dest == TILEGX_R_ZERO);
        goto do_load_add;
    case OE_IM(LD2S_ADD, X1):
        memop = MO_TESW;
        mnemonic = "ld2s_add"; /* prefetch_add_l2_fault */
        goto do_load_add;
    case OE_IM(LD2U_ADD, X1):
        memop = MO_TEUW;
        mnemonic = "ld2u_add"; /* prefetch_add_l2 */
        prefetch_nofault = (dest == TILEGX_R_ZERO);
        goto do_load_add;
    case OE_IM(LD4S_ADD, X1):
        memop = MO_TESL;
        mnemonic = "ld4s_add"; /* prefetch_add_l3_fault */
        goto do_load_add;
    case OE_IM(LD4U_ADD, X1):
        memop = MO_TEUL;
        mnemonic = "ld4u_add"; /* prefetch_add_l3 */
        prefetch_nofault = (dest == TILEGX_R_ZERO);
        goto do_load_add;
    case OE_IM(LDNT1S_ADD, X1):
        memop = MO_SB;
        mnemonic = "ldnt1s_add";
        goto do_load_add;
    case OE_IM(LDNT1U_ADD, X1):
        memop = MO_UB;
        mnemonic = "ldnt1u_add";
        goto do_load_add;
    case OE_IM(LDNT2S_ADD, X1):
        memop = MO_TESW;
        mnemonic = "ldnt2s_add";
        goto do_load_add;
    case OE_IM(LDNT2U_ADD, X1):
        memop = MO_TEUW;
        mnemonic = "ldnt2u_add";
        goto do_load_add;
    case OE_IM(LDNT4S_ADD, X1):
        memop = MO_TESL;
        mnemonic = "ldnt4s_add";
        goto do_load_add;
    case OE_IM(LDNT4U_ADD, X1):
        memop = MO_TEUL;
        mnemonic = "ldnt4u_add";
        goto do_load_add;
    case OE_IM(LDNT_ADD, X1):
        memop = MO_TEUQ;
        mnemonic = "ldnt_add";
        goto do_load_add;
    case OE_IM(LD_ADD, X1):
        memop = MO_TEUQ;
        mnemonic = "ld_add";
    do_load_add:
        if (!prefetch_nofault) {
            tcg_gen_qemu_ld_tl(tdest, tsrca, dc->mmuidx, memop);
        }
        tcg_gen_addi_tl(dest_gr(dc, srca), tsrca, imm);
        break;
    case OE_IM(LDNA_ADD, X1):
        tcg_gen_andi_tl(tdest, tsrca, ~7);
        tcg_gen_qemu_ld_tl(tdest, tdest, dc->mmuidx, MO_TEUQ);
        tcg_gen_addi_tl(dest_gr(dc, srca), tsrca, imm);
        mnemonic = "ldna_add";
        break;
    case OE_IM(ORI, X0):
    case OE_IM(ORI, X1):
        tcg_gen_ori_tl(tdest, tsrca, imm);
        mnemonic = "ori";
        break;
    case OE_IM(V1ADDI, X0):
    case OE_IM(V1ADDI, X1):
        t0 = tcg_constant_tl(V1_IMM(imm));
        gen_v12add(tdest, tsrca, t0, V1_IMM(0x80));
        mnemonic = "v1addi";
        break;
    case OE_IM(V1CMPEQI, X0):
    case OE_IM(V1CMPEQI, X1):
        tcg_gen_xori_tl(tdest, tsrca, V1_IMM(imm));
        gen_v1cmpeq0(tdest);
        mnemonic = "v1cmpeqi";
        break;
    case OE_IM(V1CMPLTSI, X0):
    case OE_IM(V1CMPLTSI, X1):
    case OE_IM(V1CMPLTUI, X0):
    case OE_IM(V1CMPLTUI, X1):
    case OE_IM(V1MAXUI, X0):
    case OE_IM(V1MAXUI, X1):
    case OE_IM(V1MINUI, X0):
    case OE_IM(V1MINUI, X1):
        return TILEGX_EXCP_OPCODE_UNIMPLEMENTED;
    case OE_IM(V2ADDI, X0):
    case OE_IM(V2ADDI, X1):
        t0 = tcg_constant_tl(V2_IMM(imm));
        gen_v12add(tdest, tsrca, t0, V2_IMM(0x8000));
        mnemonic = "v2addi";
        break;
    case OE_IM(V2CMPEQI, X0):
    case OE_IM(V2CMPEQI, X1):
    case OE_IM(V2CMPLTSI, X0):
    case OE_IM(V2CMPLTSI, X1):
    case OE_IM(V2CMPLTUI, X0):
    case OE_IM(V2CMPLTUI, X1):
    case OE_IM(V2MAXSI, X0):
    case OE_IM(V2MAXSI, X1):
    case OE_IM(V2MINSI, X0):
    case OE_IM(V2MINSI, X1):
        return TILEGX_EXCP_OPCODE_UNIMPLEMENTED;
    case OE_IM(XORI, X0):
    case OE_IM(XORI, X1):
        tcg_gen_xori_tl(tdest, tsrca, imm);
        mnemonic = "xori";
        break;

    case OE_SH(ROTLI, X0):
    case OE_SH(ROTLI, X1):
    case OE_SH(ROTLI, Y0):
    case OE_SH(ROTLI, Y1):
        tcg_gen_rotli_tl(tdest, tsrca, imm);
        mnemonic = "rotli";
        break;
    case OE_SH(SHLI, X0):
    case OE_SH(SHLI, X1):
    case OE_SH(SHLI, Y0):
    case OE_SH(SHLI, Y1):
        tcg_gen_shli_tl(tdest, tsrca, imm);
        mnemonic = "shli";
        break;
    case OE_SH(SHLXI, X0):
    case OE_SH(SHLXI, X1):
        tcg_gen_shli_tl(tdest, tsrca, imm & 31);
        tcg_gen_ext32s_tl(tdest, tdest);
        mnemonic = "shlxi";
        break;
    case OE_SH(SHRSI, X0):
    case OE_SH(SHRSI, X1):
    case OE_SH(SHRSI, Y0):
    case OE_SH(SHRSI, Y1):
        tcg_gen_sari_tl(tdest, tsrca, imm);
        mnemonic = "shrsi";
        break;
    case OE_SH(SHRUI, X0):
    case OE_SH(SHRUI, X1):
    case OE_SH(SHRUI, Y0):
    case OE_SH(SHRUI, Y1):
        tcg_gen_shri_tl(tdest, tsrca, imm);
        mnemonic = "shrui";
        break;
    case OE_SH(SHRUXI, X0):
    case OE_SH(SHRUXI, X1):
        if ((imm & 31) == 0) {
            tcg_gen_ext32s_tl(tdest, tsrca);
        } else {
            tcg_gen_ext32u_tl(tdest, tsrca);
            tcg_gen_shri_tl(tdest, tdest, imm & 31);
        }
        mnemonic = "shlxi";
        break;
    case OE_SH(V1SHLI, X0):
    case OE_SH(V1SHLI, X1):
        i2 = imm & 7;
        i3 = 0xff >> i2;
        tcg_gen_andi_tl(tdest, tsrca, V1_IMM(i3));
        tcg_gen_shli_tl(tdest, tdest, i2);
        mnemonic = "v1shli";
        break;
    case OE_SH(V1SHRSI, X0):
    case OE_SH(V1SHRSI, X1):
        t0 = tcg_constant_tl(imm & 7);
        gen_helper_v1shrs(tdest, tsrca, t0);
        mnemonic = "v1shrsi";
        break;
    case OE_SH(V1SHRUI, X0):
    case OE_SH(V1SHRUI, X1):
        i2 = imm & 7;
        i3 = (0xff << i2) & 0xff;
        tcg_gen_andi_tl(tdest, tsrca, V1_IMM(i3));
        tcg_gen_shri_tl(tdest, tdest, i2);
        mnemonic = "v1shrui";
        break;
    case OE_SH(V2SHLI, X0):
    case OE_SH(V2SHLI, X1):
        i2 = imm & 15;
        i3 = 0xffff >> i2;
        tcg_gen_andi_tl(tdest, tsrca, V2_IMM(i3));
        tcg_gen_shli_tl(tdest, tdest, i2);
        mnemonic = "v2shli";
        break;
    case OE_SH(V2SHRSI, X0):
    case OE_SH(V2SHRSI, X1):
        t0 = tcg_constant_tl(imm & 15);
        gen_helper_v2shrs(tdest, tsrca, t0);
        mnemonic = "v2shrsi";
        break;
    case OE_SH(V2SHRUI, X0):
    case OE_SH(V2SHRUI, X1):
        i2 = imm & 15;
        i3 = (0xffff << i2) & 0xffff;
        tcg_gen_andi_tl(tdest, tsrca, V2_IMM(i3));
        tcg_gen_shri_tl(tdest, tdest, i2);
        mnemonic = "v2shrui";
        break;

    case OE(ADDLI_OPCODE_X0, 0, X0):
    case OE(ADDLI_OPCODE_X1, 0, X1):
        tcg_gen_addi_tl(tdest, tsrca, imm);
        mnemonic = "addli";
        break;
    case OE(ADDXLI_OPCODE_X0, 0, X0):
    case OE(ADDXLI_OPCODE_X1, 0, X1):
        tcg_gen_addi_tl(tdest, tsrca, imm);
        tcg_gen_ext32s_tl(tdest, tdest);
        mnemonic = "addxli";
        break;
    case OE(SHL16INSLI_OPCODE_X0, 0, X0):
    case OE(SHL16INSLI_OPCODE_X1, 0, X1):
        tcg_gen_shli_tl(tdest, tsrca, 16);
        tcg_gen_ori_tl(tdest, tdest, imm & 0xffff);
        mnemonic = "shl16insli";
        break;

    default:
        return TILEGX_EXCP_OPCODE_UNKNOWN;
    }

    qemu_log_mask(CPU_LOG_TB_IN_ASM, "%s %s, %s, %d", mnemonic,
                  reg_names[dest], reg_names[srca], imm);
    return TILEGX_EXCP_NONE;
}

static TileExcp gen_bf_opcode_x0(DisasContext *dc, unsigned ext,
                                 unsigned dest, unsigned srca,
                                 unsigned bfs, unsigned bfe)
{
    TCGv tdest = dest_gr(dc, dest);
    TCGv tsrca = load_gr(dc, srca);
    TCGv tsrcd;
    int len;
    const char *mnemonic;

    /* The bitfield is either between E and S inclusive,
       or up from S and down from E inclusive.  */
    if (bfs <= bfe) {
        len = bfe - bfs + 1;
    } else {
        len = (64 - bfs) + (bfe + 1);
    }

    switch (ext) {
    case BFEXTU_BF_OPCODE_X0:
        if (bfs == 0 && bfe == 7) {
            tcg_gen_ext8u_tl(tdest, tsrca);
        } else if (bfs == 0 && bfe == 15) {
            tcg_gen_ext16u_tl(tdest, tsrca);
        } else if (bfs == 0 && bfe == 31) {
            tcg_gen_ext32u_tl(tdest, tsrca);
        } else {
            int rol = 63 - bfe;
            if (bfs <= bfe) {
                tcg_gen_shli_tl(tdest, tsrca, rol);
            } else {
                tcg_gen_rotli_tl(tdest, tsrca, rol);
            }
            tcg_gen_shri_tl(tdest, tdest, (bfs + rol) & 63);
        }
        mnemonic = "bfextu";
        break;

    case BFEXTS_BF_OPCODE_X0:
        if (bfs == 0 && bfe == 7) {
            tcg_gen_ext8s_tl(tdest, tsrca);
        } else if (bfs == 0 && bfe == 15) {
            tcg_gen_ext16s_tl(tdest, tsrca);
        } else if (bfs == 0 && bfe == 31) {
            tcg_gen_ext32s_tl(tdest, tsrca);
        } else {
            int rol = 63 - bfe;
            if (bfs <= bfe) {
                tcg_gen_shli_tl(tdest, tsrca, rol);
            } else {
                tcg_gen_rotli_tl(tdest, tsrca, rol);
            }
            tcg_gen_sari_tl(tdest, tdest, (bfs + rol) & 63);
        }
        mnemonic = "bfexts";
        break;

    case BFINS_BF_OPCODE_X0:
        tsrcd = load_gr(dc, dest);
        if (bfs <= bfe) {
            tcg_gen_deposit_tl(tdest, tsrcd, tsrca, bfs, len);
        } else {
            tcg_gen_rotri_tl(tdest, tsrcd, bfs);
            tcg_gen_deposit_tl(tdest, tdest, tsrca, 0, len);
            tcg_gen_rotli_tl(tdest, tdest, bfs);
        }
        mnemonic = "bfins";
        break;

    case MM_BF_OPCODE_X0:
        tsrcd = load_gr(dc, dest);
        if (bfs == 0) {
            tcg_gen_deposit_tl(tdest, tsrca, tsrcd, 0, len);
        } else {
            uint64_t mask = len == 64 ? -1 : rol64((1ULL << len) - 1, bfs);
            TCGv tmp = tcg_temp_new();
            tcg_gen_movi_tl(tmp, mask);

            tcg_gen_and_tl(tdest, tsrcd, tmp);
            tcg_gen_andc_tl(tmp, tsrca, tmp);
            tcg_gen_or_tl(tdest, tdest, tmp);
        }
        mnemonic = "mm";
        break;

    default:
        return TILEGX_EXCP_OPCODE_UNKNOWN;
    }

    qemu_log_mask(CPU_LOG_TB_IN_ASM, "%s %s, %s, %u, %u", mnemonic,
                  reg_names[dest], reg_names[srca], bfs, bfe);
    return TILEGX_EXCP_NONE;
}

static TileExcp gen_branch_opcode_x1(DisasContext *dc, unsigned ext,
                                     unsigned srca, int off)
{
    target_ulong tgt = dc->pc + off * TILEGX_BUNDLE_SIZE_IN_BYTES;
    const char *mnemonic;

    dc->jmp.dest = tcg_constant_tl(tgt);
    dc->jmp.val1 = tcg_temp_new();
    tcg_gen_mov_tl(dc->jmp.val1, load_gr(dc, srca));

    /* Note that the "predict taken" opcodes have bit 0 clear.
       Therefore, fold the two cases together by setting bit 0.  */
    switch (ext | 1) {
    case BEQZ_BRANCH_OPCODE_X1:
        dc->jmp.cond = TCG_COND_EQ;
        mnemonic = "beqz";
        break;
    case BNEZ_BRANCH_OPCODE_X1:
        dc->jmp.cond = TCG_COND_NE;
        mnemonic = "bnez";
        break;
    case BGEZ_BRANCH_OPCODE_X1:
        dc->jmp.cond = TCG_COND_GE;
        mnemonic = "bgez";
        break;
    case BGTZ_BRANCH_OPCODE_X1:
        dc->jmp.cond = TCG_COND_GT;
        mnemonic = "bgtz";
        break;
    case BLEZ_BRANCH_OPCODE_X1:
        dc->jmp.cond = TCG_COND_LE;
        mnemonic = "blez";
        break;
    case BLTZ_BRANCH_OPCODE_X1:
        dc->jmp.cond = TCG_COND_LT;
        mnemonic = "bltz";
        break;
    case BLBC_BRANCH_OPCODE_X1:
        dc->jmp.cond = TCG_COND_EQ;
        tcg_gen_andi_tl(dc->jmp.val1, dc->jmp.val1, 1);
        mnemonic = "blbc";
        break;
    case BLBS_BRANCH_OPCODE_X1:
        dc->jmp.cond = TCG_COND_NE;
        tcg_gen_andi_tl(dc->jmp.val1, dc->jmp.val1, 1);
        mnemonic = "blbs";
        break;
    default:
        return TILEGX_EXCP_OPCODE_UNKNOWN;
    }

    if (qemu_loglevel_mask(CPU_LOG_TB_IN_ASM)) {
        qemu_log("%s%s %s, " TARGET_FMT_lx " <%s>",
                 mnemonic, ext & 1 ? "" : "t",
                 reg_names[srca], tgt, lookup_symbol(tgt));
    }
    return TILEGX_EXCP_NONE;
}

static TileExcp gen_jump_opcode_x1(DisasContext *dc, unsigned ext, int off)
{
    target_ulong tgt = dc->pc + off * TILEGX_BUNDLE_SIZE_IN_BYTES;
    const char *mnemonic = "j";

    /* The extension field is 1 bit, therefore we only have JAL and J.  */
    if (ext == JAL_JUMP_OPCODE_X1) {
        tcg_gen_movi_tl(dest_gr(dc, TILEGX_R_LR),
                        dc->pc + TILEGX_BUNDLE_SIZE_IN_BYTES);
        mnemonic = "jal";
    }
    dc->jmp.cond = TCG_COND_ALWAYS;
    dc->jmp.dest = tcg_constant_tl(tgt);

    if (qemu_loglevel_mask(CPU_LOG_TB_IN_ASM)) {
        qemu_log("%s " TARGET_FMT_lx " <%s>",
                 mnemonic, tgt, lookup_symbol(tgt));
    }
    return TILEGX_EXCP_NONE;
}

/* SPR accesses are checked against the MPLs at run time. */
static TileExcp gen_mtspr_x1(DisasContext *dc, unsigned spr, unsigned srca)
{
    tcg_gen_movi_tl(cpu_pc, dc->pc);
    gen_helper_mtspr(tcg_env, tcg_constant_i32(spr), load_gr(dc, srca));
    /* Masks, ICS and MPLs decide which interrupts are taken. */
    dc->end_tb = true;
    qemu_log_mask(CPU_LOG_TB_IN_ASM, "mtspr %#x, %s", spr, reg_names[srca]);
    return TILEGX_EXCP_NONE;
}

static TileExcp gen_mfspr_x1(DisasContext *dc, unsigned dest, unsigned spr)
{
    tcg_gen_movi_tl(cpu_pc, dc->pc);
    gen_helper_mfspr(dest_gr(dc, dest), tcg_env, tcg_constant_i32(spr));
    qemu_log_mask(CPU_LOG_TB_IN_ASM, "mfspr %s, %#x", reg_names[dest], spr);
    return TILEGX_EXCP_NONE;
}

static TileExcp decode_y0(DisasContext *dc, tilegx_bundle_bits bundle)
{
    unsigned opc = get_Opcode_Y0(bundle);
    unsigned ext = get_RRROpcodeExtension_Y0(bundle);
    unsigned dest = get_Dest_Y0(bundle);
    unsigned srca = get_SrcA_Y0(bundle);
    unsigned srcb;
    int imm;

    switch (opc) {
    case RRR_1_OPCODE_Y0:
        if (ext == UNARY_RRR_1_OPCODE_Y0) {
            ext = get_UnaryOpcodeExtension_Y0(bundle);
            return gen_rr_opcode(dc, OE(opc, ext, Y0), dest, srca, bundle);
        }
        /* fallthru */
    case RRR_0_OPCODE_Y0:
    case RRR_2_OPCODE_Y0:
    case RRR_3_OPCODE_Y0:
    case RRR_4_OPCODE_Y0:
    case RRR_5_OPCODE_Y0:
    case RRR_6_OPCODE_Y0:
    case RRR_7_OPCODE_Y0:
    case RRR_8_OPCODE_Y0:
    case RRR_9_OPCODE_Y0:
        srcb = get_SrcB_Y0(bundle);
        return gen_rrr_opcode(dc, OE(opc, ext, Y0), dest, srca, srcb);

    case SHIFT_OPCODE_Y0:
        ext = get_ShiftOpcodeExtension_Y0(bundle);
        imm = get_ShAmt_Y0(bundle);
        return gen_rri_opcode(dc, OE(opc, ext, Y0), dest, srca, imm);

    case ADDI_OPCODE_Y0:
    case ADDXI_OPCODE_Y0:
    case ANDI_OPCODE_Y0:
    case CMPEQI_OPCODE_Y0:
    case CMPLTSI_OPCODE_Y0:
        imm = (int8_t)get_Imm8_Y0(bundle);
        return gen_rri_opcode(dc, OE(opc, 0, Y0), dest, srca, imm);

    default:
        return TILEGX_EXCP_OPCODE_UNKNOWN;
    }
}

static TileExcp decode_y1(DisasContext *dc, tilegx_bundle_bits bundle)
{
    unsigned opc = get_Opcode_Y1(bundle);
    unsigned ext = get_RRROpcodeExtension_Y1(bundle);
    unsigned dest = get_Dest_Y1(bundle);
    unsigned srca = get_SrcA_Y1(bundle);
    unsigned srcb;
    int imm;

    switch (get_Opcode_Y1(bundle)) {
    case RRR_1_OPCODE_Y1:
        if (ext == UNARY_RRR_1_OPCODE_Y0) {
            ext = get_UnaryOpcodeExtension_Y1(bundle);
            return gen_rr_opcode(dc, OE(opc, ext, Y1), dest, srca, bundle);
        }
        /* fallthru */
    case RRR_0_OPCODE_Y1:
    case RRR_2_OPCODE_Y1:
    case RRR_3_OPCODE_Y1:
    case RRR_4_OPCODE_Y1:
    case RRR_5_OPCODE_Y1:
    case RRR_6_OPCODE_Y1:
    case RRR_7_OPCODE_Y1:
        srcb = get_SrcB_Y1(bundle);
        return gen_rrr_opcode(dc, OE(opc, ext, Y1), dest, srca, srcb);

    case SHIFT_OPCODE_Y1:
        ext = get_ShiftOpcodeExtension_Y1(bundle);
        imm = get_ShAmt_Y1(bundle);
        return gen_rri_opcode(dc, OE(opc, ext, Y1), dest, srca, imm);

    case ADDI_OPCODE_Y1:
    case ADDXI_OPCODE_Y1:
    case ANDI_OPCODE_Y1:
    case CMPEQI_OPCODE_Y1:
    case CMPLTSI_OPCODE_Y1:
        imm = (int8_t)get_Imm8_Y1(bundle);
        return gen_rri_opcode(dc, OE(opc, 0, Y1), dest, srca, imm);

    default:
        return TILEGX_EXCP_OPCODE_UNKNOWN;
    }
}

static TileExcp decode_y2(DisasContext *dc, tilegx_bundle_bits bundle)
{
    unsigned mode = get_Mode(bundle);
    unsigned opc = get_Opcode_Y2(bundle);
    unsigned srca = get_SrcA_Y2(bundle);
    unsigned srcbdest = get_SrcBDest_Y2(bundle);
    const char *mnemonic;
    MemOp memop;
    bool prefetch_nofault = false;

    switch (OEY2(opc, mode)) {
    case OEY2(LD1S_OPCODE_Y2, MODE_OPCODE_YA2):
        memop = MO_SB;
        mnemonic = "ld1s"; /* prefetch_l1_fault */
        goto do_load;
    case OEY2(LD1U_OPCODE_Y2, MODE_OPCODE_YA2):
        memop = MO_UB;
        mnemonic = "ld1u"; /* prefetch, prefetch_l1 */
        prefetch_nofault = (srcbdest == TILEGX_R_ZERO);
        goto do_load;
    case OEY2(LD2S_OPCODE_Y2, MODE_OPCODE_YA2):
        memop = MO_TESW;
        mnemonic = "ld2s"; /* prefetch_l2_fault */
        goto do_load;
    case OEY2(LD2U_OPCODE_Y2, MODE_OPCODE_YA2):
        memop = MO_TEUW;
        mnemonic = "ld2u"; /* prefetch_l2 */
        prefetch_nofault = (srcbdest == TILEGX_R_ZERO);
        goto do_load;
    case OEY2(LD4S_OPCODE_Y2, MODE_OPCODE_YB2):
        memop = MO_TESL;
        mnemonic = "ld4s"; /* prefetch_l3_fault */
        goto do_load;
    case OEY2(LD4U_OPCODE_Y2, MODE_OPCODE_YB2):
        memop = MO_TEUL;
        mnemonic = "ld4u"; /* prefetch_l3 */
        prefetch_nofault = (srcbdest == TILEGX_R_ZERO);
        goto do_load;
    case OEY2(LD_OPCODE_Y2, MODE_OPCODE_YB2):
        memop = MO_TEUQ;
        mnemonic = "ld";
    do_load:
        if (!prefetch_nofault) {
            tcg_gen_qemu_ld_tl(dest_gr(dc, srcbdest), load_gr(dc, srca),
                               dc->mmuidx, memop);
        }
        qemu_log_mask(CPU_LOG_TB_IN_ASM, "%s %s, %s", mnemonic,
                      reg_names[srcbdest], reg_names[srca]);
        return TILEGX_EXCP_NONE;

    case OEY2(ST1_OPCODE_Y2, MODE_OPCODE_YC2):
        return gen_st_opcode(dc, 0, srca, srcbdest, MO_UB, "st1");
    case OEY2(ST2_OPCODE_Y2, MODE_OPCODE_YC2):
        return gen_st_opcode(dc, 0, srca, srcbdest, MO_TEUW, "st2");
    case OEY2(ST4_OPCODE_Y2, MODE_OPCODE_YC2):
        return gen_st_opcode(dc, 0, srca, srcbdest, MO_TEUL, "st4");
    case OEY2(ST_OPCODE_Y2, MODE_OPCODE_YC2):
        return gen_st_opcode(dc, 0, srca, srcbdest, MO_TEUQ, "st");

    default:
        return TILEGX_EXCP_OPCODE_UNKNOWN;
    }
}

static TileExcp decode_x0(DisasContext *dc, tilegx_bundle_bits bundle)
{
    unsigned opc = get_Opcode_X0(bundle);
    unsigned dest = get_Dest_X0(bundle);
    unsigned srca = get_SrcA_X0(bundle);
    unsigned ext, srcb, bfs, bfe;
    int imm;

    switch (opc) {
    case RRR_0_OPCODE_X0:
        ext = get_RRROpcodeExtension_X0(bundle);
        if (ext == UNARY_RRR_0_OPCODE_X0) {
            ext = get_UnaryOpcodeExtension_X0(bundle);
            return gen_rr_opcode(dc, OE(opc, ext, X0), dest, srca, bundle);
        }
        srcb = get_SrcB_X0(bundle);
        return gen_rrr_opcode(dc, OE(opc, ext, X0), dest, srca, srcb);

    case SHIFT_OPCODE_X0:
        ext = get_ShiftOpcodeExtension_X0(bundle);
        imm = get_ShAmt_X0(bundle);
        return gen_rri_opcode(dc, OE(opc, ext, X0), dest, srca, imm);

    case IMM8_OPCODE_X0:
        ext = get_Imm8OpcodeExtension_X0(bundle);
        imm = (int8_t)get_Imm8_X0(bundle);
        return gen_rri_opcode(dc, OE(opc, ext, X0), dest, srca, imm);

    case BF_OPCODE_X0:
        ext = get_BFOpcodeExtension_X0(bundle);
        bfs = get_BFStart_X0(bundle);
        bfe = get_BFEnd_X0(bundle);
        return gen_bf_opcode_x0(dc, ext, dest, srca, bfs, bfe);

    case ADDLI_OPCODE_X0:
    case SHL16INSLI_OPCODE_X0:
    case ADDXLI_OPCODE_X0:
        imm = (int16_t)get_Imm16_X0(bundle);
        return gen_rri_opcode(dc, OE(opc, 0, X0), dest, srca, imm);

    default:
        return TILEGX_EXCP_OPCODE_UNKNOWN;
    }
}

static TileExcp decode_x1(DisasContext *dc, tilegx_bundle_bits bundle)
{
    unsigned opc = get_Opcode_X1(bundle);
    unsigned dest = get_Dest_X1(bundle);
    unsigned srca = get_SrcA_X1(bundle);
    unsigned ext, srcb;
    int imm;

    switch (opc) {
    case RRR_0_OPCODE_X1:
        ext = get_RRROpcodeExtension_X1(bundle);
        srcb = get_SrcB_X1(bundle);
        switch (ext) {
        case UNARY_RRR_0_OPCODE_X1:
            ext = get_UnaryOpcodeExtension_X1(bundle);
            return gen_rr_opcode(dc, OE(opc, ext, X1), dest, srca, bundle);
        case ST1_RRR_0_OPCODE_X1:
            return gen_st_opcode(dc, dest, srca, srcb, MO_UB, "st1");
        case ST2_RRR_0_OPCODE_X1:
            return gen_st_opcode(dc, dest, srca, srcb, MO_TEUW, "st2");
        case ST4_RRR_0_OPCODE_X1:
            return gen_st_opcode(dc, dest, srca, srcb, MO_TEUL, "st4");
        case STNT1_RRR_0_OPCODE_X1:
            return gen_st_opcode(dc, dest, srca, srcb, MO_UB, "stnt1");
        case STNT2_RRR_0_OPCODE_X1:
            return gen_st_opcode(dc, dest, srca, srcb, MO_TEUW, "stnt2");
        case STNT4_RRR_0_OPCODE_X1:
            return gen_st_opcode(dc, dest, srca, srcb, MO_TEUL, "stnt4");
        case STNT_RRR_0_OPCODE_X1:
            return gen_st_opcode(dc, dest, srca, srcb, MO_TEUQ, "stnt");
        case ST_RRR_0_OPCODE_X1:
            return gen_st_opcode(dc, dest, srca, srcb, MO_TEUQ, "st");
        }
        return gen_rrr_opcode(dc, OE(opc, ext, X1), dest, srca, srcb);

    case SHIFT_OPCODE_X1:
        ext = get_ShiftOpcodeExtension_X1(bundle);
        imm = get_ShAmt_X1(bundle);
        return gen_rri_opcode(dc, OE(opc, ext, X1), dest, srca, imm);

    case IMM8_OPCODE_X1:
        ext = get_Imm8OpcodeExtension_X1(bundle);
        imm = (int8_t)get_Dest_Imm8_X1(bundle);
        srcb = get_SrcB_X1(bundle);
        switch (ext) {
        case ST1_ADD_IMM8_OPCODE_X1:
            return gen_st_add_opcode(dc, srca, srcb, imm, MO_UB, "st1_add");
        case ST2_ADD_IMM8_OPCODE_X1:
            return gen_st_add_opcode(dc, srca, srcb, imm, MO_TEUW, "st2_add");
        case ST4_ADD_IMM8_OPCODE_X1:
            return gen_st_add_opcode(dc, srca, srcb, imm, MO_TEUL, "st4_add");
        case STNT1_ADD_IMM8_OPCODE_X1:
            return gen_st_add_opcode(dc, srca, srcb, imm, MO_UB, "stnt1_add");
        case STNT2_ADD_IMM8_OPCODE_X1:
            return gen_st_add_opcode(dc, srca, srcb, imm, MO_TEUW, "stnt2_add");
        case STNT4_ADD_IMM8_OPCODE_X1:
            return gen_st_add_opcode(dc, srca, srcb, imm, MO_TEUL, "stnt4_add");
        case STNT_ADD_IMM8_OPCODE_X1:
            return gen_st_add_opcode(dc, srca, srcb, imm, MO_TEUQ, "stnt_add");
        case ST_ADD_IMM8_OPCODE_X1:
            return gen_st_add_opcode(dc, srca, srcb, imm, MO_TEUQ, "st_add");
        case MFSPR_IMM8_OPCODE_X1:
            return gen_mfspr_x1(dc, dest, get_MF_Imm14_X1(bundle));
        case MTSPR_IMM8_OPCODE_X1:
            return gen_mtspr_x1(dc, get_MT_Imm14_X1(bundle), srca);
        }
        imm = (int8_t)get_Imm8_X1(bundle);
        return gen_rri_opcode(dc, OE(opc, ext, X1), dest, srca, imm);

    case BRANCH_OPCODE_X1:
        ext = get_BrType_X1(bundle);
        imm = sextract32(get_BrOff_X1(bundle), 0, 17);
        return gen_branch_opcode_x1(dc, ext, srca, imm);

    case JUMP_OPCODE_X1:
        ext = get_JumpOpcodeExtension_X1(bundle);
        imm = sextract32(get_JumpOff_X1(bundle), 0, 27);
        return gen_jump_opcode_x1(dc, ext, imm);

    case ADDLI_OPCODE_X1:
    case SHL16INSLI_OPCODE_X1:
    case ADDXLI_OPCODE_X1:
        imm = (int16_t)get_Imm16_X1(bundle);
        return gen_rri_opcode(dc, OE(opc, 0, X1), dest, srca, imm);

    default:
        return TILEGX_EXCP_OPCODE_UNKNOWN;
    }
}

static void notice_excp(DisasContext *dc, uint64_t bundle,
                        const char *type, TileExcp excp)
{
    if (likely(excp == TILEGX_EXCP_NONE) || dc->exit_tb) {
        return;
    }
    if (excp == TILEGX_EXCP_OPCODE_UNIMPLEMENTED) {
        qemu_log_mask(LOG_UNIMP, "UNIMP %s, [" FMT64X "] at " FMT64X "\n",
                      type, bundle, dc->pc);
        excp = INT_ILL;
    }
    gen_exception(dc, excp);
}

static void translate_one_bundle(DisasContext *dc, uint64_t bundle)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(dc->wb); i++) {
        DisasContextTemp *wb = &dc->wb[i];
        wb->reg = TILEGX_R_NOREG;
        wb->val = NULL;
    }
    dc->num_wb = 0;
    dc->post_excp = -1;
    dc->jmp.cond = TCG_COND_NEVER;
    dc->zero = NULL;

    qemu_log_mask(CPU_LOG_TB_IN_ASM, "  %" PRIx64 ":  { ", dc->pc);
    if (get_Mode(bundle)) {
        notice_excp(dc, bundle, "y0", decode_y0(dc, bundle));
        qemu_log_mask(CPU_LOG_TB_IN_ASM, " ; ");
        notice_excp(dc, bundle, "y1", decode_y1(dc, bundle));
        qemu_log_mask(CPU_LOG_TB_IN_ASM, " ; ");
        notice_excp(dc, bundle, "y2", decode_y2(dc, bundle));
    } else {
        notice_excp(dc, bundle, "x0", decode_x0(dc, bundle));
        qemu_log_mask(CPU_LOG_TB_IN_ASM, " ; ");
        notice_excp(dc, bundle, "x1", decode_x1(dc, bundle));
    }
    qemu_log_mask(CPU_LOG_TB_IN_ASM, " }\n");

    if (dc->exit_tb) {
        /* The bundle trapped, or (nap, iret) has done its own exit. */
        return;
    }

    for (i = dc->num_wb - 1; i >= 0; --i) {
        DisasContextTemp *wb = &dc->wb[i];
        if (wb->reg < TILEGX_R_COUNT) {
            tcg_gen_mov_i64(cpu_regs[wb->reg], wb->val);
        }
    }

    if (dc->post_excp >= 0) {
        /*
         * swint: the rest of the bundle has completed, but EX_CONTEXT
         * gets the PC of the swint bundle; the handler steps past it.
         */
        tcg_gen_movi_tl(cpu_pc, dc->pc);
        gen_helper_exception(tcg_env, tcg_constant_i32(dc->post_excp));
        dc->exit_tb = true;
    } else if (dc->jmp.cond != TCG_COND_NEVER) {
        if (dc->jmp.cond == TCG_COND_ALWAYS) {
            tcg_gen_mov_i64(cpu_pc, dc->jmp.dest);
        } else {
            TCGv next = tcg_constant_i64(dc->pc + TILEGX_BUNDLE_SIZE_IN_BYTES);
            tcg_gen_movcond_i64(dc->jmp.cond, cpu_pc,
                                dc->jmp.val1, load_zero(dc),
                                dc->jmp.dest, next);
        }
        tcg_gen_lookup_and_goto_ptr();
        dc->exit_tb = true;
    } else if (dc->end_tb) {
        tcg_gen_movi_tl(cpu_pc, dc->pc + TILEGX_BUNDLE_SIZE_IN_BYTES);
        tcg_gen_exit_tb(NULL, 0);
        dc->exit_tb = true;
    }
}

typedef struct {
    DisasContextBase base;
    DisasContext dc;
    CPUTLGState *env;
} TileGXDisasContext;

static void tilegx_tr_init_disas_context(DisasContextBase *dcbase,
                                         CPUState *cs)
{
    TileGXDisasContext *ctx = container_of(dcbase, TileGXDisasContext, base);
    DisasContext *dc = &ctx->dc;

    ctx->env = cpu_env(cs);
    dc->pc = ctx->base.pc_first;
    dc->mmuidx = ctx->base.tb->flags ? MMU_KERNEL_IDX : MMU_USER_IDX;
    dc->exit_tb = false;
    dc->end_tb = false;
    dc->jmp.cond = TCG_COND_NEVER;
    dc->jmp.dest = NULL;
    dc->jmp.val1 = NULL;
    dc->zero = NULL;

    /* Stay within the page: bundles never straddle one. */
    ctx->base.max_insns = MIN(ctx->base.max_insns,
        (TARGET_PAGE_SIZE - (dc->pc & ~TARGET_PAGE_MASK)) / 8);
}

static void tilegx_tr_tb_start(DisasContextBase *dcbase, CPUState *cs)
{
}

static void tilegx_tr_insn_start(DisasContextBase *dcbase, CPUState *cs)
{
    TileGXDisasContext *ctx = container_of(dcbase, TileGXDisasContext, base);

    tcg_gen_insn_start(ctx->dc.pc, 0, 0);
}

static void tilegx_tr_translate_insn(DisasContextBase *dcbase, CPUState *cs)
{
    TileGXDisasContext *ctx = container_of(dcbase, TileGXDisasContext, base);
    DisasContext *dc = &ctx->dc;
    CPUTLGState *env = ctx->env;
    uint64_t bundle;

    /* A jump into the hypervisor glue page is a hypervisor call. */
    if (env->hv_glue_va && dc->mmuidx != MMU_USER_IDX &&
        dc->pc - env->hv_glue_va < 0x10000) {
        tcg_gen_movi_tl(cpu_pc, dc->pc);
        gen_helper_exception(tcg_env, tcg_constant_i32(TILEGX_EXCP_HV_CALL));
        ctx->base.is_jmp = DISAS_NORETURN;
        ctx->base.pc_next += TILEGX_BUNDLE_SIZE_IN_BYTES;
        return;
    }

    bundle = translator_ldq(env, &ctx->base, dc->pc);
    ctx->base.pc_next += TILEGX_BUNDLE_SIZE_IN_BYTES;
    translate_one_bundle(dc, bundle);
    if (dc->exit_tb) {
        ctx->base.is_jmp = DISAS_NORETURN;
        return;
    }
    dc->pc += TILEGX_BUNDLE_SIZE_IN_BYTES;
}

static void tilegx_tr_tb_stop(DisasContextBase *dcbase, CPUState *cs)
{
    TileGXDisasContext *ctx = container_of(dcbase, TileGXDisasContext, base);

    if (ctx->base.is_jmp == DISAS_NORETURN) {
        return;
    }
    tcg_gen_movi_tl(cpu_pc, ctx->dc.pc);
    tcg_gen_lookup_and_goto_ptr();
}

static const TranslatorOps tilegx_tr_ops = {
    .init_disas_context = tilegx_tr_init_disas_context,
    .tb_start           = tilegx_tr_tb_start,
    .insn_start         = tilegx_tr_insn_start,
    .translate_insn     = tilegx_tr_translate_insn,
    .tb_stop            = tilegx_tr_tb_stop,
};

void tilegx_translate_code(CPUState *cs, TranslationBlock *tb,
                           int *max_insns, vaddr pc, void *host_pc)
{
    TileGXDisasContext ctx;

    translator_loop(cs, tb, max_insns, pc, host_pc, &tilegx_tr_ops,
                    &ctx.base, TCG_TYPE_VA);
}

void tilegx_translate_init(void)
{
    int i;

    cpu_pc = tcg_global_mem_new_i64(tcg_env, offsetof(CPUTLGState, pc), "pc");
    for (i = 0; i < TILEGX_R_COUNT; i++) {
        cpu_regs[i] = tcg_global_mem_new_i64(tcg_env,
                                             offsetof(CPUTLGState, regs[i]),
                                             reg_names[i]);
    }
}
