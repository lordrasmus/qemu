/*
 * FR-V instruction translation
 *
 * One qemu "instruction" is one VLIW packet: the instructions up to and
 * including the first one with the pack bit set.  They execute in
 * parallel, so every one of them reads the registers as they were before
 * the packet.  Where a later instruction of a packet reads a register an
 * earlier one writes, the write goes to a temporary that is committed at
 * the end of the packet.  Branches, traps and rett also take effect at
 * the end of the packet.
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include "qemu/osdep.h"
#include "cpu.h"
#include "tcg/tcg-op.h"
#include "exec/helper-proto.h"
#include "exec/helper-gen.h"
#include "exec/log.h"
#include "exec/translator.h"
#include "exec/translation-block.h"
#include "qemu/qemu-print.h"

#define HELPER_H "helper.h"
#include "exec/helper-info.c.inc"
#undef  HELPER_H

#define MAX_PKT 8

/* Instruction fields */
#define F_PACK(i)   (((i) >> 31) & 1)
#define F_GRK(i)    (((i) >> 25) & 63)
#define F_OP(i)     (((i) >> 18) & 127)
#define F_GRI(i)    (((i) >> 12) & 63)
#define F_GRJ(i)    ((i) & 63)
#define F_OPE1(i)   (((i) >> 6) & 63)
#define F_OPE2(i)   (((i) >> 6) & 15)
#define F_OPE3(i)   (((i) >> 13) & 7)
#define F_OPE4(i)   (((i) >> 6) & 3)
#define F_ICC1(i)   (((i) >> 10) & 3)
#define F_ICC2(i)   (((i) >> 25) & 3)
#define F_ICC3(i)   ((i) & 3)
#define F_CCI(i)    (((i) >> 9) & 7)
#define F_COND(i)   (((i) >> 8) & 1)
#define F_CC4(i)    (((i) >> 27) & 15)
#define F_CCOND(i)  (((i) >> 12) & 1)
#define F_LI(i)     (((i) >> 25) & 1)
#define F_S12(i)    sextract32(i, 0, 12)
#define F_S10(i)    sextract32(i, 0, 10)
#define F_U16(i)    ((i) & 0xffff)
#define F_S16(i)    sextract32(i, 0, 16)
#define F_CRI(i)    (((i) >> 12) & 7)
#define F_CRJ(i)    ((i) & 7)
#define F_CRK(i)    (((i) >> 25) & 7)
#define F_SPR(i)    ((((i) >> 25) & 63) << 6 | (((i) >> 12) & 63))
#define F_A(i)      (((i) >> 17) & 1)

typedef struct DisasContext {
    DisasContextBase base;
    uint32_t pkt;           /* address of the packet */
    uint32_t next;          /* address of the packet after it */
    uint32_t iaddr;         /* address of the instruction */
    uint32_t insn;
    int mem_idx;
    bool user;
    bool debug;

    /* writes held back until the end of the packet, NULL: write through */
    TCGv_i32 gr_pend[NUM_GR];
    TCGv_i32 fr_pend[NUM_FR];
    TCGv_i32 icc_pend[4];
    TCGv_i32 fcc_pend[4];
    TCGv_i32 cc_pend[8];
    TCGv_i32 lr_pend;
    TCGv_i32 lcr_pend;

    /*
     * branches of the packet: 0 none, 1 one static unconditional, 3 one
     * static conditional (br_taken says whether), 2 anything else
     */
    int br_kind;
    uint32_t br_target;
    TCGv_i32 br_taken;
    TCGv_i32 br_dest;

    /* trap at the end of the packet: tira, tne, ... */
    TCGv_i32 trap_val;
    TCGv_i32 trap_cond;     /* NULL: always */
    bool trap;
    bool brk;
    int rett;               /* 0 none, 1 rett, 2 rett #1 */
    bool exit_tb;           /* the CPU state changed, recheck interrupts */
    TCGOp *insn_start;
} DisasContext;

static TCGv_i32 cpu_gr[NUM_GR];
static TCGv_i32 cpu_fr[NUM_FR];
static TCGv_i32 cpu_pc;
static TCGv_i32 cpu_icc[4];
static TCGv_i32 cpu_fcc[4];
static TCGv_i32 cpu_cc[8];
static TCGv_i32 cpu_lr;
static TCGv_i32 cpu_lcr;

static TCGv_i32 tmp(void)
{
    return tcg_temp_new_i32();
}

static TCGv_i32 cst(uint32_t v)
{
    return tcg_constant_i32(v);
}

/* Registers: reads see the state before the packet */

static TCGv_i32 gr(int r)
{
    return r ? cpu_gr[r] : cst(0);
}

static void set_gr(DisasContext *dc, int r, TCGv_i32 v)
{
    if (r) {
        tcg_gen_mov_i32(dc->gr_pend[r] ? dc->gr_pend[r] : cpu_gr[r], v);
    }
}

/*
 * sethi and setlo write half of a register, so the two can share a
 * packet: the halves merge into the pending value.
 */
static void set_gr_half(DisasContext *dc, int r, uint32_t val, bool high)
{
    TCGv_i32 dst = dc->gr_pend[r] ? dc->gr_pend[r] : cpu_gr[r];

    if (r) {
        tcg_gen_deposit_i32(dst, dst, cst(val), high ? 16 : 0, 16);
    }
}

static void set_fr(DisasContext *dc, int r, TCGv_i32 v)
{
    tcg_gen_mov_i32(dc->fr_pend[r] ? dc->fr_pend[r] : cpu_fr[r], v);
}

static void set_icc(DisasContext *dc, int n, TCGv_i32 v)
{
    tcg_gen_mov_i32(dc->icc_pend[n] ? dc->icc_pend[n] : cpu_icc[n], v);
}

static void set_cc(DisasContext *dc, int n, TCGv_i32 v)
{
    tcg_gen_mov_i32(dc->cc_pend[n] ? dc->cc_pend[n] : cpu_cc[n], v);
}

static void set_lr(DisasContext *dc, TCGv_i32 v)
{
    tcg_gen_mov_i32(dc->lr_pend ? dc->lr_pend : cpu_lr, v);
}

static void set_lcr(DisasContext *dc, TCGv_i32 v)
{
    tcg_gen_mov_i32(dc->lcr_pend ? dc->lcr_pend : cpu_lcr, v);
}

/* Exceptions */

static void gen_raise(DisasContext *dc, int tt)
{
    gen_helper_raise(tcg_env, cst(tt));
    dc->base.is_jmp = DISAS_NORETURN;
}

static bool gen_illegal(DisasContext *dc)
{
    qemu_log_mask(LOG_UNIMP, "frv: illegal instruction %08x at %08x\n",
                  dc->insn, dc->iaddr);
    gen_raise(dc, TT_ILLEGAL_INSN);
    return true;
}

static bool gen_priv(DisasContext *dc)
{
    if (dc->user) {
        gen_raise(dc, TT_PRIV_INSN);
        return true;
    }
    return false;
}

/* Conditions */

/* An integer condition (int-cc numbering) on the flags N Z V C */
static TCGv_i32 gen_icond(int cond, TCGv_i32 icc)
{
    TCGv_i32 r = tmp(), t = tmp();

    switch (cond & 7) {
    case 0: /* never / always */
        tcg_gen_movi_i32(r, 0);
        break;
    case 1: /* c */
        tcg_gen_andi_i32(r, icc, 1);
        break;
    case 2: /* v */
        tcg_gen_extract_i32(r, icc, 1, 1);
        break;
    case 3: /* lt: n ^ v */
        tcg_gen_shri_i32(t, icc, 2);
        tcg_gen_xor_i32(r, icc, t);
        tcg_gen_extract_i32(r, r, 1, 1);
        break;
    case 4: /* eq */
        tcg_gen_extract_i32(r, icc, 2, 1);
        break;
    case 5: /* ls: c | z */
        tcg_gen_shri_i32(t, icc, 2);
        tcg_gen_or_i32(r, icc, t);
        tcg_gen_andi_i32(r, r, 1);
        break;
    case 6: /* n */
        tcg_gen_extract_i32(r, icc, 3, 1);
        break;
    case 7: /* le: z | (n ^ v) */
        tcg_gen_shri_i32(t, icc, 2);
        tcg_gen_xor_i32(r, icc, t);
        tcg_gen_shri_i32(r, r, 1);
        tcg_gen_or_i32(r, r, t);
        tcg_gen_andi_i32(r, r, 1);
        break;
    }
    if (cond & 8) {
        tcg_gen_xori_i32(r, r, 1);
    }
    return r;
}

/* A float condition (flt-cc numbering) on the flags E L G U */
static TCGv_i32 gen_fcond(int cond, TCGv_i32 fcc)
{
    TCGv_i32 r = tmp();

    if (cond == 15) {
        tcg_gen_movi_i32(r, 1);
    } else {
        tcg_gen_andi_i32(r, fcc, cond);
        tcg_gen_setcondi_i32(TCG_COND_NE, r, r, 0);
    }
    return r;
}

/* Conditional execution: skip unless CCi equals cond | 2 */
static TCGLabel *gen_cexec(DisasContext *dc)
{
    TCGLabel *skip = gen_new_label();

    tcg_gen_brcondi_i32(TCG_COND_NE, cpu_cc[F_CCI(dc->insn)],
                        F_COND(dc->insn) | 2, skip);
    return skip;
}

/* Integer condition codes */

/* N and Z from the result, V and C from cc */
static void gen_nz(TCGv_i32 out, TCGv_i32 cc, TCGv_i32 res)
{
    TCGv_i32 t = tmp();

    tcg_gen_andi_i32(out, cc, 3);
    tcg_gen_setcondi_i32(TCG_COND_EQ, t, res, 0);
    tcg_gen_shli_i32(t, t, 2);
    tcg_gen_or_i32(out, out, t);
    tcg_gen_shri_i32(t, res, 31);
    tcg_gen_shli_i32(t, t, 3);
    tcg_gen_or_i32(out, out, t);
}

/* N Z V C of a + b + cin (sub: a - b - cin, C is the borrow) */
static void gen_arith(TCGv_i32 res, TCGv_i32 icc, TCGv_i32 a, TCGv_i32 b,
                      TCGv_i32 cin, bool sub)
{
    TCGv_i64 wa = tcg_temp_new_i64(), wb = tcg_temp_new_i64();
    TCGv_i32 r = tmp(), c = tmp(), v = tmp(), t = tmp();

    tcg_gen_extu_i32_i64(wa, a);
    tcg_gen_extu_i32_i64(wb, b);
    if (sub) {
        tcg_gen_sub_i64(wa, wa, wb);
    } else {
        tcg_gen_add_i64(wa, wa, wb);
    }
    if (cin) {
        tcg_gen_extu_i32_i64(wb, cin);
        if (sub) {
            tcg_gen_sub_i64(wa, wa, wb);
        } else {
            tcg_gen_add_i64(wa, wa, wb);
        }
    }
    tcg_gen_extr_i64_i32(r, c, wa);
    tcg_gen_andi_i32(c, c, 1);
    if (sub) {
        tcg_gen_xor_i32(v, a, b);
        tcg_gen_xor_i32(t, a, r);
    } else {
        tcg_gen_xor_i32(v, a, r);
        tcg_gen_xor_i32(t, b, r);
    }
    tcg_gen_and_i32(v, v, t);
    tcg_gen_shri_i32(v, v, 31);
    tcg_gen_shli_i32(v, v, 1);
    tcg_gen_or_i32(c, c, v);
    gen_nz(icc, c, r);
    tcg_gen_mov_i32(res, r);
}

/* Integer operations */

enum {
    OP_ADD, OP_SUB, OP_AND, OP_OR, OP_XOR, OP_SLL, OP_SRL, OP_SRA,
    OP_SMUL, OP_UMUL, OP_SDIV, OP_UDIV, OP_ADDX, OP_SUBX, OP_SCAN, OP_NOT,
};

/*
 * Compute op on a and b into GRk (GRk:GRk+1 for the multiplications);
 * with icc >= 0 set that integer condition code register as well.
 */
static void gen_intop(DisasContext *dc, int op, TCGv_i32 a, TCGv_i32 b,
                      int k, int icc)
{
    TCGv_i32 r = tmp(), cc = tmp();
    TCGv_i32 old = icc >= 0 ? cpu_icc[icc] : NULL;

    switch (op) {
    case OP_ADD:
    case OP_SUB:
        if (icc >= 0) {
            gen_arith(r, cc, a, b, NULL, op == OP_SUB);
        } else if (op == OP_ADD) {
            tcg_gen_add_i32(r, a, b);
        } else {
            tcg_gen_sub_i32(r, a, b);
        }
        break;
    case OP_ADDX:
    case OP_SUBX: {
        TCGv_i32 cin = tmp();

        tcg_gen_andi_i32(cin, cpu_icc[F_ICC1(dc->insn)], 1);
        gen_arith(r, cc, a, b, cin, op == OP_SUBX);
        break;
    }
    case OP_AND:
    case OP_OR:
    case OP_XOR:
    case OP_NOT:
        if (op == OP_AND) {
            tcg_gen_and_i32(r, a, b);
        } else if (op == OP_OR) {
            tcg_gen_or_i32(r, a, b);
        } else if (op == OP_XOR) {
            tcg_gen_xor_i32(r, a, b);
        } else {
            tcg_gen_not_i32(r, b);
        }
        if (old) {
            gen_nz(cc, old, r);
        }
        break;
    case OP_SLL:
    case OP_SRL:
    case OP_SRA: {
        TCGv_i32 sh = tmp();

        tcg_gen_andi_i32(sh, b, 31);
        if (op == OP_SLL) {
            tcg_gen_shl_i32(r, a, sh);
        } else if (op == OP_SRL) {
            tcg_gen_shr_i32(r, a, sh);
        } else {
            tcg_gen_sar_i32(r, a, sh);
        }
        if (old) {
            /* left: V = bits shifted out; right: C = bits shifted out */
            TCGv_i64 w = tcg_temp_new_i64(), s64 = tcg_temp_new_i64();
            TCGv_i32 lo = tmp(), hi = tmp(), f = tmp();

            tcg_gen_extu_i32_i64(w, a);
            tcg_gen_extu_i32_i64(s64, sh);
            if (op == OP_SLL) {
                tcg_gen_shl_i64(w, w, s64);
                tcg_gen_extr_i64_i32(lo, hi, w);
                tcg_gen_setcondi_i32(TCG_COND_NE, f, hi, 0);
                tcg_gen_shli_i32(f, f, 1);
                tcg_gen_andi_i32(cc, old, 0xd);
            } else {
                tcg_gen_shli_i64(w, w, 32);
                tcg_gen_shr_i64(w, w, s64);
                tcg_gen_extr_i64_i32(lo, hi, w);
                tcg_gen_setcondi_i32(TCG_COND_NE, f, lo, 0);
                tcg_gen_andi_i32(cc, old, 0xe);
            }
            tcg_gen_or_i32(cc, cc, f);
            gen_nz(cc, cc, r);
        }
        break;
    }
    case OP_SMUL:
    case OP_UMUL: {
        TCGv_i32 hi = tmp();

        if (op == OP_SMUL) {
            tcg_gen_muls2_i32(r, hi, a, b);
        } else {
            tcg_gen_mulu2_i32(r, hi, a, b);
        }
        if (old) {
            /* N from bit 63, Z from all 64 bits */
            TCGv_i32 t = tmp();

            tcg_gen_or_i32(t, r, hi);
            gen_nz(cc, old, t);
            tcg_gen_andi_i32(cc, cc, 7);
            tcg_gen_shri_i32(t, hi, 31);
            tcg_gen_shli_i32(t, t, 3);
            tcg_gen_or_i32(cc, cc, t);
            set_icc(dc, icc, cc);
        }
        /* a pair starting at GR0 is not written, GR1 included */
        if (k) {
            set_gr(dc, k, hi);
            set_gr(dc, (k + 1) & 63, r);
        }
        return;
    }
    case OP_SDIV:
        gen_helper_sdiv(r, tcg_env, a, b);
        break;
    case OP_UDIV:
        gen_helper_udiv(r, tcg_env, a, b);
        break;
    case OP_SCAN: {
        TCGv_i32 t = tmp();

        tcg_gen_sari_i32(t, b, 1);
        tcg_gen_xor_i32(t, a, t);
        tcg_gen_clzi_i32(r, t, 63);
        break;
    }
    default:
        g_assert_not_reached();
    }
    if (old) {
        set_icc(dc, icc, cc);
    }
    set_gr(dc, k, r);
}

/* op 0x00 and 0x01 by OPE2, -1: not an instruction */
static const int8_t op00[16] = {
    OP_ADD, OP_ADD, OP_ADDX, OP_ADDX, OP_SUB, OP_SUB, OP_SUBX, OP_SUBX,
    OP_SMUL, OP_SMUL, OP_UMUL, OP_UMUL, -1, -1, OP_SDIV, OP_UDIV,
};
static const int8_t op01[16] = {
    OP_AND, OP_AND, OP_OR, OP_OR, OP_XOR, OP_XOR, OP_NOT, -1,
    OP_SLL, OP_SLL, OP_SRL, OP_SRL, OP_SRA, OP_SRA, -1, -1,
};

static bool trans_rr(DisasContext *dc, uint32_t insn)
{
    int ope = F_OPE2(insn);
    int op = F_OP(insn) ? op01[ope] : op00[ope];
    /* odd OPE2 set the condition codes, except sdiv and udiv */
    int icc = (ope & 1) && op != OP_UDIV ? F_ICC1(insn) : -1;

    if (F_OP(insn) == 0 && (ope == 0xc || ope == 0xd)) {
        /* cmpb, cmpba */
        TCGv_i32 x = tmp(), cc = tmp(), t = tmp();

        tcg_gen_xor_i32(x, gr(F_GRI(insn)), gr(F_GRJ(insn)));
        tcg_gen_movi_i32(cc, 0);
        for (int byte = 0; byte < 4; byte++) {
            tcg_gen_andi_i32(t, x, 0xffu << (8 * (3 - byte)));
            tcg_gen_setcondi_i32(TCG_COND_EQ, t, t, 0);
            if (ope == 0xc) {
                tcg_gen_shli_i32(t, t, 3 - byte);
            }
            tcg_gen_or_i32(cc, cc, t);
        }
        set_icc(dc, F_ICC1(insn), cc);
        return true;
    }
    if (op < 0) {
        return false;
    }
    if (op == OP_ADDX || op == OP_SUBX) {
        icc = (ope & 1) ? F_ICC1(insn) : -1;
        if (icc < 0) {
            /* addx/subx use the carry, but leave the flags */
            TCGv_i32 r = tmp(), cc = tmp(), cin = tmp();

            tcg_gen_andi_i32(cin, cpu_icc[F_ICC1(insn)], 1);
            gen_arith(r, cc, gr(F_GRI(insn)), gr(F_GRJ(insn)), cin,
                      op == OP_SUBX);
            set_gr(dc, F_GRK(insn), r);
            return true;
        }
    }
    gen_intop(dc, op, gr(F_GRI(insn)), gr(F_GRJ(insn)), F_GRK(insn), icc);
    return true;
}

/* addi ... reg/immediate forms, op 0x10-0x2d and scani */
static bool trans_ri(DisasContext *dc, uint32_t insn)
{
    static const struct {
        int8_t op;
        int8_t cc;      /* 1: sets ICCi_1 from a 10 bit immediate */
    } tab[0x30] = {
        [0x10] = { OP_ADD, 0 }, [0x11] = { OP_ADD, 1 },
        [0x12] = { OP_ADDX, 0 }, [0x13] = { OP_ADDX, 1 },
        [0x14] = { OP_SUB, 0 }, [0x15] = { OP_SUB, 1 },
        [0x16] = { OP_SUBX, 0 }, [0x17] = { OP_SUBX, 1 },
        [0x18] = { OP_SMUL, 0 }, [0x19] = { OP_SMUL, 1 },
        [0x1a] = { OP_UMUL, 0 }, [0x1b] = { OP_UMUL, 1 },
        [0x1e] = { OP_SDIV, 0 }, [0x1f] = { OP_UDIV, 0 },
        [0x20] = { OP_AND, 0 }, [0x21] = { OP_AND, 1 },
        [0x22] = { OP_OR, 0 }, [0x23] = { OP_OR, 1 },
        [0x24] = { OP_XOR, 0 }, [0x25] = { OP_XOR, 1 },
        [0x28] = { OP_SLL, 0 }, [0x29] = { OP_SLL, 1 },
        [0x2a] = { OP_SRL, 0 }, [0x2b] = { OP_SRL, 1 },
        [0x2c] = { OP_SRA, 0 }, [0x2d] = { OP_SRA, 1 },
    };
    int opc = F_OP(insn);
    int op, icc;
    TCGv_i32 imm;

    if (opc == 0x47) {
        gen_intop(dc, OP_SCAN, gr(F_GRI(insn)), cst(F_S12(insn)),
                  F_GRK(insn), -1);
        return true;
    }
    if (opc >= 0x30 || (!tab[opc].op && !tab[opc].cc && opc != 0x10)) {
        return false;
    }
    op = tab[opc].op;
    icc = tab[opc].cc ? F_ICC1(insn) : -1;
    /* the cc forms and addxi/subxi have ICCi_1 next to a 10 bit immediate */
    imm = cst(tab[opc].cc || op == OP_ADDX || op == OP_SUBX ? F_S10(insn)
                                                            : F_S12(insn));
    if (op == OP_ADDX || op == OP_SUBX) {
        if (icc < 0) {
            TCGv_i32 r = tmp(), cc = tmp(), cin = tmp();

            tcg_gen_andi_i32(cin, cpu_icc[F_ICC1(insn)], 1);
            gen_arith(r, cc, gr(F_GRI(insn)), imm, cin, op == OP_SUBX);
            set_gr(dc, F_GRK(insn), r);
            return true;
        }
    }
    gen_intop(dc, op, gr(F_GRI(insn)), imm, F_GRK(insn), icc);
    return true;
}

/* cadd ... conditional integer operations, op 0x58-0x5d and cscan */
static bool trans_cint(DisasContext *dc, uint32_t insn)
{
    static const int8_t tab[6][4] = {
        { OP_ADD, OP_SUB, OP_SMUL, OP_SDIV },   /* 0x58 */
        { OP_ADD, OP_SUB, OP_SMUL, OP_UDIV },   /* 0x59, cc but cudiv */
        { OP_AND, OP_OR, OP_XOR, OP_NOT },      /* 0x5a */
        { OP_AND, OP_OR, OP_XOR, -1 },          /* 0x5b, cc */
        { OP_SLL, OP_SRL, OP_SRA, -1 },         /* 0x5c */
        { OP_SLL, OP_SRL, OP_SRA, -1 },         /* 0x5d, cc */
    };
    int row = F_OP(insn) - 0x58, ope = F_OPE4(insn);
    int op = tab[row][ope];
    int icc = -1;
    TCGLabel *skip;

    if (op < 0) {
        return false;
    }
    if ((row & 1) && !(row == 1 && ope == 3)) {
        icc = F_CCI(insn) & 3;
    }
    skip = gen_cexec(dc);
    gen_intop(dc, op, gr(F_GRI(insn)), gr(F_GRJ(insn)), F_GRK(insn), icc);
    gen_set_label(skip);
    return true;
}

/* Loads and stores */

static TCGv_i32 gen_addr(DisasContext *dc, uint32_t insn, bool imm)
{
    TCGv_i32 a = tmp();

    if (imm) {
        tcg_gen_addi_i32(a, gr(F_GRI(insn)), F_S12(insn));
    } else {
        tcg_gen_add_i32(a, gr(F_GRI(insn)), gr(F_GRJ(insn)));
    }
    return a;
}

enum { R_GR, R_FR };

/*
 * Load or store one, two (double) or four (quad) words, a byte or a half
 * word between memory and register k of the given file.  Returns false
 * for combinations that do not exist.
 */
static void gen_ldst(DisasContext *dc, bool store, int file, int k,
                     TCGv_i32 addr, MemOp mop)
{
    TCGv_i32 *regs = file == R_GR ? cpu_gr : cpu_fr;

    mop |= MO_BE | MO_ALIGN;
    if ((mop & MO_SIZE) == MO_64) {
        TCGv_i64 v = tcg_temp_new_i64();
        int k1 = (k + 1) & 63;

        if (store) {
            /* the GR0 pair reads as zero, GR1 included */
            if (file == R_GR && k == 0) {
                tcg_gen_movi_i64(v, 0);
            } else {
                tcg_gen_concat_i32_i64(v, regs[k1], regs[k]);
            }
            tcg_gen_qemu_st_i64(v, addr, dc->mem_idx, mop);
        } else {
            TCGv_i32 hi = tmp(), lo = tmp();

            /* a double load into GR0 does nothing */
            if (file == R_GR && k == 0) {
                return;
            }
            tcg_gen_qemu_ld_i64(v, addr, dc->mem_idx, mop);
            tcg_gen_extr_i64_i32(lo, hi, v);
            if (file == R_GR) {
                set_gr(dc, k, hi);
                set_gr(dc, k1, lo);
            } else {
                set_fr(dc, k, hi);
                set_fr(dc, k1, lo);
            }
        }
        return;
    }
    if (store) {
        tcg_gen_qemu_st_i32(file == R_GR ? gr(k) : regs[k], addr,
                            dc->mem_idx, mop);
    } else {
        TCGv_i32 v = tmp();

        tcg_gen_qemu_ld_i32(v, addr, dc->mem_idx, mop);
        if (file == R_GR) {
            set_gr(dc, k, v);
        } else {
            set_fr(dc, k, v);
        }
    }
}

/* What OPE1 of op 0x02/0x03 means for loads and stores */
typedef struct LdSt {
    int8_t valid;
    int8_t file;
    MemOp mop;
    int8_t update;
} LdSt;

static const LdSt ld_tab[64] = {
    [0x00] = { 1, R_GR, MO_SB }, [0x01] = { 1, R_GR, MO_UB },
    [0x02] = { 1, R_GR, MO_SW }, [0x03] = { 1, R_GR, MO_UW },
    [0x04] = { 1, R_GR, MO_UL }, [0x05] = { 1, R_GR, MO_UQ },
    [0x08] = { 1, R_FR, MO_UB }, [0x09] = { 1, R_FR, MO_UW },
    [0x0a] = { 1, R_FR, MO_UL }, [0x0b] = { 1, R_FR, MO_UQ },
    [0x10] = { 1, R_GR, MO_SB, 1 }, [0x11] = { 1, R_GR, MO_UB, 1 },
    [0x12] = { 1, R_GR, MO_SW, 1 }, [0x13] = { 1, R_GR, MO_UW, 1 },
    [0x14] = { 1, R_GR, MO_UL, 1 }, [0x15] = { 1, R_GR, MO_UQ, 1 },
    [0x18] = { 1, R_FR, MO_UB, 1 }, [0x19] = { 1, R_FR, MO_UW, 1 },
    [0x1a] = { 1, R_FR, MO_UL, 1 }, [0x1b] = { 1, R_FR, MO_UQ, 1 },
};

static const LdSt st_tab[64] = {
    [0x00] = { 1, R_GR, MO_UB }, [0x01] = { 1, R_GR, MO_UW },
    [0x02] = { 1, R_GR, MO_UL }, [0x03] = { 1, R_GR, MO_UQ },
    [0x08] = { 1, R_FR, MO_UB }, [0x09] = { 1, R_FR, MO_UW },
    [0x0a] = { 1, R_FR, MO_UL }, [0x0b] = { 1, R_FR, MO_UQ },
    [0x10] = { 1, R_GR, MO_UB, 1 }, [0x11] = { 1, R_GR, MO_UW, 1 },
    [0x12] = { 1, R_GR, MO_UL, 1 }, [0x13] = { 1, R_GR, MO_UQ, 1 },
    [0x18] = { 1, R_FR, MO_UB, 1 }, [0x19] = { 1, R_FR, MO_UW, 1 },
    [0x1a] = { 1, R_FR, MO_UL, 1 }, [0x1b] = { 1, R_FR, MO_UQ, 1 },
};

static void gen_ldst_op(DisasContext *dc, uint32_t insn, bool store,
                        const LdSt *d, TCGv_i32 addr)
{
    int k = F_GRK(insn), i = F_GRI(insn);

    gen_ldst(dc, store, d->file, k, addr, d->mop);
    if (d->update) {
        /*
         * A GR load whose base is (one of) its targets leaves the
         * loaded value; a double load into GR0 does not update at all.
         */
        if (!store && d->file == R_GR) {
            bool dbl = (d->mop & MO_SIZE) == MO_64;

            if (i == k || (dbl && k == 0)) {
                return;
            }
        }
        set_gr(dc, i, addr);
    }
}

static bool trans_ldst_rr(DisasContext *dc, uint32_t insn, bool store)
{
    const LdSt *d = store ? &st_tab[F_OPE1(insn)] : &ld_tab[F_OPE1(insn)];

    if (!d->valid) {
        return false;
    }
    gen_ldst_op(dc, insn, store, d, gen_addr(dc, insn, false));
    return true;
}

/* ldsbi ... stdfi, the reg + 12 bit displacement forms */
static bool trans_ldst_ri(DisasContext *dc, uint32_t insn)
{
    static const struct {
        int8_t valid, store, file;
        MemOp mop;
    } tab[0x60] = {
        [0x30] = { 1, 0, R_GR, MO_SB }, [0x31] = { 1, 0, R_GR, MO_SW },
        [0x32] = { 1, 0, R_GR, MO_UL }, [0x33] = { 1, 0, R_GR, MO_UQ },
        [0x35] = { 1, 0, R_GR, MO_UB }, [0x36] = { 1, 0, R_GR, MO_UW },
        [0x38] = { 1, 0, R_FR, MO_UB }, [0x39] = { 1, 0, R_FR, MO_UW },
        [0x3a] = { 1, 0, R_FR, MO_UL }, [0x3b] = { 1, 0, R_FR, MO_UQ },
        [0x4e] = { 1, 1, R_FR, MO_UB }, [0x4f] = { 1, 1, R_FR, MO_UW },
        [0x50] = { 1, 1, R_GR, MO_UB }, [0x51] = { 1, 1, R_GR, MO_UW },
        [0x52] = { 1, 1, R_GR, MO_UL }, [0x53] = { 1, 1, R_GR, MO_UQ },
        [0x55] = { 1, 1, R_FR, MO_UL }, [0x56] = { 1, 1, R_FR, MO_UQ },
    };
    int op = F_OP(insn);

    if (op >= 0x60 || !tab[op].valid) {
        return false;
    }
    gen_ldst(dc, tab[op].store, tab[op].file, F_GRK(insn),
             gen_addr(dc, insn, true), tab[op].mop);
    return true;
}

/* Conditional loads and stores, op 0x5e-0x68 except 0x65 */
static bool trans_cldst(DisasContext *dc, uint32_t insn)
{
    static const struct {
        int8_t valid, store, file, update;
        MemOp mop;
    } tab[11][4] = {
        /* 0x5e cldsb cldub cldsh clduh */
        { { 1, 0, R_GR, 0, MO_SB }, { 1, 0, R_GR, 0, MO_UB },
          { 1, 0, R_GR, 0, MO_SW }, { 1, 0, R_GR, 0, MO_UW } },
        /* 0x5f cld cldd */
        { { 1, 0, R_GR, 0, MO_UL }, { 1, 0, R_GR, 0, MO_UQ } },
        /* 0x60 cldbf cldhf cldf clddf */
        { { 1, 0, R_FR, 0, MO_UB }, { 1, 0, R_FR, 0, MO_UW },
          { 1, 0, R_FR, 0, MO_UL }, { 1, 0, R_FR, 0, MO_UQ } },
        /* 0x61 cldsbu cldubu cldshu clduhu */
        { { 1, 0, R_GR, 1, MO_SB }, { 1, 0, R_GR, 1, MO_UB },
          { 1, 0, R_GR, 1, MO_SW }, { 1, 0, R_GR, 1, MO_UW } },
        /* 0x62 cldu clddu */
        { { 1, 0, R_GR, 1, MO_UL }, { 1, 0, R_GR, 1, MO_UQ } },
        /* 0x63 cldbfu cldhfu cldfu clddfu */
        { { 1, 0, R_FR, 1, MO_UB }, { 1, 0, R_FR, 1, MO_UW },
          { 1, 0, R_FR, 1, MO_UL }, { 1, 0, R_FR, 1, MO_UQ } },
        /* 0x64 cstb csth cst cstd */
        { { 1, 1, R_GR, 0, MO_UB }, { 1, 1, R_GR, 0, MO_UW },
          { 1, 1, R_GR, 0, MO_UL }, { 1, 1, R_GR, 0, MO_UQ } },
        /* 0x65 cswap and cscan, elsewhere */
        { },
        /* 0x66 cstbf csthf cstf cstdf */
        { { 1, 1, R_FR, 0, MO_UB }, { 1, 1, R_FR, 0, MO_UW },
          { 1, 1, R_FR, 0, MO_UL }, { 1, 1, R_FR, 0, MO_UQ } },
        /* 0x67 cstbu csthu cstu cstdu */
        { { 1, 1, R_GR, 1, MO_UB }, { 1, 1, R_GR, 1, MO_UW },
          { 1, 1, R_GR, 1, MO_UL }, { 1, 1, R_GR, 1, MO_UQ } },
        /* 0x68 cstbfu csthfu cstfu cstdfu */
        { { 1, 1, R_FR, 1, MO_UB }, { 1, 1, R_FR, 1, MO_UW },
          { 1, 1, R_FR, 1, MO_UL }, { 1, 1, R_FR, 1, MO_UQ } },
    };
    int row = F_OP(insn) - 0x5e;
    LdSt d;
    TCGLabel *skip;

    if (!tab[row][F_OPE4(insn)].valid) {
        return false;
    }
    d.valid = 1;
    d.file = tab[row][F_OPE4(insn)].file;
    d.mop = tab[row][F_OPE4(insn)].mop;
    d.update = tab[row][F_OPE4(insn)].update;
    skip = gen_cexec(dc);
    gen_ldst_op(dc, insn, tab[row][F_OPE4(insn)].store, &d,
                gen_addr(dc, insn, false));
    gen_set_label(skip);
    return true;
}

static void gen_swap(DisasContext *dc, uint32_t insn, TCGv_i32 addr)
{
    TCGv_i32 old = tmp();

    tcg_gen_atomic_xchg_i32(old, addr, gr(F_GRK(insn)), dc->mem_idx,
                            MO_BEUL | MO_ALIGN);
    set_gr(dc, F_GRK(insn), old);
}

/* Special purpose registers */

static bool spr_user_ok(int spr)
{
    return (spr >= SPR_CCR && spr <= SPR_IACC0L) ||
           (spr >= 432 && spr <= 435) ||
           (spr >= 1024 && spr < 1536);
}

/* SPRs whose writes need no fresh look at the CPU state */
static bool spr_plain(int spr)
{
    return spr_user_ok(spr) || (spr >= SPR_SR0 && spr < SPR_SR0 + 68) ||
           (spr >= SPR_EPCR0 && spr < SPR_EPCR0 + 128);
}

static bool trans_movgs(DisasContext *dc, uint32_t insn, bool to_spr)
{
    int spr = F_SPR(insn), j = F_GRJ(insn);

    if (dc->user && !spr_user_ok(spr)) {
        gen_raise(dc, TT_PRIV_INSN);
        return true;
    }
    if (to_spr) {
        if (spr == SPR_LR) {
            set_lr(dc, gr(j));
        } else if (spr == SPR_LCR) {
            set_lcr(dc, gr(j));
        } else {
            gen_helper_movgs(tcg_env, cst(spr), gr(j));
            if (!spr_plain(spr)) {
                dc->exit_tb = true;
            }
        }
    } else {
        TCGv_i32 v = tmp();

        if (spr == SPR_LR) {
            tcg_gen_mov_i32(v, cpu_lr);
        } else if (spr == SPR_LCR) {
            tcg_gen_mov_i32(v, cpu_lcr);
        } else {
            gen_helper_movsg(v, tcg_env, cst(spr));
        }
        set_gr(dc, j, v);
    }
    return true;
}

/* GR <-> FR moves */
static void gen_movgf(DisasContext *dc, uint32_t insn, bool to_fr, bool dbl)
{
    int k = F_GRK(insn), j = F_GRJ(insn);

    if (to_fr) {
        set_fr(dc, k, gr(j));
        if (dbl) {
            set_fr(dc, (k + 1) & 63, j ? gr((j + 1) & 63) : cst(0));
        }
    } else {
        if (dbl && j == 0) {
            return;
        }
        set_gr(dc, j, cpu_fr[k]);
        if (dbl) {
            set_gr(dc, (j + 1) & 63, cpu_fr[(k + 1) & 63]);
        }
    }
}

/* op 0x03 except the plain loads and stores */
static bool trans_op03(DisasContext *dc, uint32_t insn)
{
    switch (F_OPE1(insn)) {
    case 0x05: /* swap */
        gen_swap(dc, insn, gen_addr(dc, insn, false));
        return true;
    case 0x06: /* movgs */
        return trans_movgs(dc, insn, true);
    case 0x07: /* movsg */
        return trans_movgs(dc, insn, false);
    case 0x15: /* movgf */
        gen_movgf(dc, insn, true, false);
        return true;
    case 0x0d: /* movfg */
        gen_movgf(dc, insn, false, false);
        return true;
    case 0x16: /* movgfd */
        gen_movgf(dc, insn, true, true);
        return true;
    case 0x0e: /* movfgd */
        gen_movgf(dc, insn, false, true);
        return true;
    case 0x30: /* icpl */
    case 0x31: /* icul */
    case 0x34: /* dcpl */
    case 0x35: /* dcul */
    case 0x38: /* ici */
    case 0x39: /* icei */
    case 0x3a: /* dcei */
    case 0x3b: /* dcef */
    case 0x3c: /* dci */
    case 0x3d: /* dcf */
        /* no caches */
        return true;
    case 0x3e: /* bar */
    case 0x3f: /* membar */
        tcg_gen_mb(TCG_MO_ALL | TCG_BAR_SC);
        return true;
    default:
        return trans_ldst_rr(dc, insn, true);
    }
}

/* Branches */

/* Note a branch to dest (or to the static target) if cond holds */
static void gen_branch(DisasContext *dc, TCGv_i32 cond, TCGv_i32 dest,
                       uint32_t target, bool in_cexec)
{
    bool stat = !dest && !in_cexec;

    if (!dest) {
        dest = cst(target);
    }
    if (cond) {
        tcg_gen_movcond_i32(TCG_COND_NE, dc->br_dest, cond, cst(0), dest,
                            dc->br_dest);
        tcg_gen_or_i32(dc->br_taken, dc->br_taken, cond);
    } else {
        tcg_gen_mov_i32(dc->br_dest, dest);
        tcg_gen_movi_i32(dc->br_taken, 1);
    }
    if (dc->br_kind == 0 && stat) {
        dc->br_kind = cond ? 3 : 1;
        dc->br_target = target;
    } else {
        dc->br_kind = 2;
    }
}

static bool trans_bcc(DisasContext *dc, uint32_t insn, bool flt)
{
    int c = F_CC4(insn);
    uint32_t target = dc->iaddr + F_S16(insn) * 4;
    bool always = flt ? c == 15 : c == 8;
    bool never = c == 0;

    if (never) {
        return true;
    }
    if (always) {
        gen_branch(dc, NULL, NULL, target, false);
    } else {
        TCGv_i32 cond = flt ? gen_fcond(c, cpu_fcc[F_ICC2(insn)])
                            : gen_icond(c, cpu_icc[F_ICC2(insn)]);
        gen_branch(dc, cond, NULL, target, false);
    }
    return true;
}

/* op 0x0e: branches to LR, optionally counting down LCR */
static bool trans_blr(DisasContext *dc, uint32_t insn)
{
    int ope = F_OPE3(insn), c = F_CC4(insn);
    bool flt = ope & 4, ctr;
    TCGv_i32 cond;

    if (ope == 1) {
        /* bctrlr */
        ctr = true;
        cond = cst(1);
    } else if ((ope & 3) == 2 || (ope & 3) == 3) {
        ctr = (ope & 3) == 3;
        if (flt) {
            cond = gen_fcond(c, cpu_fcc[F_ICC2(insn)]);
        } else {
            cond = c == 8 ? cst(1) : gen_icond(c, cpu_icc[F_ICC2(insn)]);
        }
    } else {
        return false;
    }
    if (ctr) {
        /* LCR is decremented, the branch needs LCR != 0 or == 0 */
        TCGv_i32 n = tmp(), z = tmp(), t = tmp();

        tcg_gen_subi_i32(n, cpu_lcr, 1);
        set_lcr(dc, n);
        tcg_gen_setcondi_i32(F_CCOND(insn) ? TCG_COND_EQ : TCG_COND_NE,
                             z, n, 0);
        tcg_gen_and_i32(t, cond, z);
        cond = t;
    }
    gen_branch(dc, cond, cpu_lr, 0, false);
    return true;
}

/* jmpl, calll, jmpil, callil and their conditional forms */
static void gen_jmpl(DisasContext *dc, uint32_t insn, bool imm, bool cexec)
{
    TCGv_i32 dest = tmp();

    if (imm) {
        tcg_gen_addi_i32(dest, gr(F_GRI(insn)), F_S12(insn));
    } else {
        tcg_gen_add_i32(dest, gr(F_GRI(insn)), gr(F_GRJ(insn)));
    }
    tcg_gen_andi_i32(dest, dest, ~3u);
    if (F_LI(insn)) {
        set_lr(dc, cst(dc->next));
    }
    gen_branch(dc, NULL, dest, 0, cexec);
}

/* Condition code operations */

static bool trans_ck(DisasContext *dc, uint32_t insn, bool flt, bool cexec)
{
    int c = F_CC4(insn);
    int crj = flt ? F_ICC2(insn) : F_ICC2(insn) + 4;
    TCGv_i32 cond, v = tmp();
    TCGLabel *skip = NULL, *done = NULL;

    if (cexec) {
        /* the CR becomes undefined when the condition fails */
        skip = gen_new_label();
        done = gen_new_label();
        tcg_gen_brcondi_i32(TCG_COND_NE, cpu_cc[F_CCI(insn)],
                            F_COND(insn) | 2, skip);
    }
    if (flt) {
        cond = c == 0 ? cst(0) : gen_fcond(c, cpu_fcc[F_ICC3(insn)]);
    } else {
        cond = c == 8 ? cst(1) : gen_icond(c, cpu_icc[F_ICC3(insn)]);
    }
    tcg_gen_ori_i32(v, cond, 2);
    set_cc(dc, crj, v);
    if (cexec) {
        tcg_gen_br(done);
        gen_set_label(skip);
        set_cc(dc, crj, cst(0));
        gen_set_label(done);
    }
    return true;
}

static bool trans_crlogic(DisasContext *dc, uint32_t insn)
{
    static const int8_t ops[64] = {
        [0 ... 63] = -1,
        [0x08] = 0, [0x09] = 1, [0x0a] = 2, [0x0c] = 3, [0x0d] = 4,
        [0x10] = 5, [0x11] = 6, [0x14] = 7, [0x15] = 8, [0x0b] = 9,
    };
    int op = ops[F_OPE1(insn)];
    TCGv_i32 v = tmp();

    if (op < 0) {
        /* clrgr, clrfr, clrga, clrfa, commitgr, ...: NE flags, none here */
        return F_OPE1(insn) <= 7;
    }
    if (op == 9) {
        /* notcr */
        tcg_gen_xori_i32(v, cpu_cc[F_CRJ(insn)], 1);
    } else {
        gen_helper_crlogic(v, cst(op), cpu_cc[F_CRI(insn)],
                           cpu_cc[F_CRJ(insn)]);
    }
    set_cc(dc, F_CRK(insn), v);
    return true;
}

/* Traps */

static bool trans_trap(DisasContext *dc, uint32_t insn, bool imm, bool flt)
{
    int c = F_CC4(insn);
    TCGv_i32 val;

    if (!imm && F_OPE4(insn) >= 2) {
        if (F_OPE4(insn) == 3) {
            /* break */
            dc->brk = true;
            return true;
        }
        /* mtrap: media exceptions are not raised, nothing to report */
        return true;
    }
    if (c == 0) {
        return true;
    }
    val = tmp();
    if (imm) {
        tcg_gen_addi_i32(val, gr(F_GRI(insn)), F_S12(insn));
    } else {
        tcg_gen_add_i32(val, gr(F_GRI(insn)), gr(F_GRJ(insn)));
    }
    dc->trap_val = val;
    if ((flt && c == 15) || (!flt && c == 8)) {
        dc->trap_cond = NULL;
    } else {
        dc->trap_cond = flt ? gen_fcond(c, cpu_fcc[F_ICC2(insn)])
                            : gen_icond(c, cpu_icc[F_ICC2(insn)]);
    }
    dc->trap = true;
    return true;
}

/* Media */

static TCGv_i32 acc_reg(int n, bool guard)
{
    TCGv_i32 v = tmp();

    tcg_gen_ld_i32(v, tcg_env, offsetof(CPUFRVState,
                   spr[(guard ? SPR_ACCG0 : SPR_ACC0) + n]));
    return v;
}

static void set_acc(int n, bool guard, TCGv_i32 v)
{
    tcg_gen_st_i32(v, tcg_env, offsetof(CPUFRVState,
                   spr[(guard ? SPR_ACCG0 : SPR_ACC0) + n]));
}

static bool trans_media(DisasContext *dc, uint32_t insn)
{
    int k = F_GRK(insn), i = F_GRI(insn);
    TCGv_i32 v;

    switch (F_OPE1(insn)) {
    case 0x3b: /* mclracc, mnop */
        if (F_A(insn)) {
            if (k == 63) {
                return true;
            }
            for (int n = 0; n < 64; n++) {
                set_acc(n, false, cst(0));
                set_acc(n, true, cst(0));
            }
        } else {
            set_acc(k, false, cst(0));
            set_acc(k, true, cst(0));
        }
        return true;
    case 0x3c: /* mrdacc */
        set_fr(dc, k, acc_reg(i, false));
        return true;
    case 0x3e: /* mrdaccg */
        set_fr(dc, k, acc_reg(i, true));
        return true;
    case 0x3d: /* mwtacc */
        set_acc(k, false, cpu_fr[i]);
        return true;
    case 0x3f: /* mwtaccg */
        v = tmp();
        tcg_gen_andi_i32(v, cpu_fr[i], 0xff);
        set_acc(k, true, v);
        return true;
    default:
        return false;
    }
}

/* op 0x46: the FR400 audio instructions */
static bool trans_audio(DisasContext *dc, uint32_t insn)
{
    TCGv_i32 r = tmp(), a = gr(F_GRI(insn)), b = gr(F_GRJ(insn));

    switch (F_OPE1(insn)) {
    case 0x00:
        gen_helper_addss(r, a, b);
        break;
    case 0x01:
        gen_helper_subss(r, a, b);
        break;
    case 0x02:
        gen_helper_slass(r, a, b);
        break;
    case 0x04:
        gen_helper_scutss(r, tcg_env, b);
        break;
    case 0x05:
    case 0x06:
    case 0x07:
        gen_helper_iacc_mul(tcg_env, cst(F_OPE1(insn) - 5), a, b);
        return true;
    default:
        return false;
    }
    set_gr(dc, F_GRK(insn), r);
    return true;
}

/* Decode */

static bool decode(DisasContext *dc, uint32_t insn)
{
    int op = F_OP(insn);
    TCGLabel *skip;

    switch (op) {
    case 0x00:
    case 0x01:
        return trans_rr(dc, insn);
    case 0x02:
        return trans_ldst_rr(dc, insn, false);
    case 0x03:
        return trans_op03(dc, insn);
    case 0x04:
        return trans_trap(dc, insn, false, F_OPE4(insn) == 1);
    case 0x05: /* rett */
        if (gen_priv(dc)) {
            return true;
        }
        dc->rett = 1 + ((insn >> 25) & 1);
        return true;
    case 0x06:
    case 0x07:
        return trans_bcc(dc, insn, op == 7);
    case 0x08:
    case 0x09:
        return trans_ck(dc, insn, op == 9, false);
    case 0x0a:
        return trans_crlogic(dc, insn);
    case 0x0b: /* scan */
        gen_intop(dc, OP_SCAN, gr(F_GRI(insn)), gr(F_GRJ(insn)),
                  F_GRK(insn), -1);
        return true;
    case 0x0c:
    case 0x0d:
        gen_jmpl(dc, insn, op == 0x0d, false);
        return true;
    case 0x0e:
        return trans_blr(dc, insn);
    case 0x0f: { /* call */
        int32_t disp = sextract32((F_GRK(insn) << 18) | (insn & 0x3ffff),
                                  0, 24);

        set_lr(dc, cst(dc->next));
        gen_branch(dc, NULL, NULL, dc->iaddr + disp * 4, false);
        return true;
    }
    case 0x1c:
    case 0x1d:
        return trans_trap(dc, insn, true, op == 0x1d);
    case 0x3d: /* setlo */
    case 0x3e: /* sethi */
        set_gr_half(dc, F_GRK(insn), F_U16(insn), op == 0x3e);
        return true;
    case 0x3f: /* setlos */
        set_gr(dc, F_GRK(insn), cst(F_S16(insn)));
        return true;
    case 0x46:
        return trans_audio(dc, insn);
    case 0x47:
        return trans_ri(dc, insn);
    case 0x4d: /* swapi */
        gen_swap(dc, insn, gen_addr(dc, insn, true));
        return true;
    case 0x58 ... 0x5d:
        return trans_cint(dc, insn);
    case 0x5e ... 0x64:
    case 0x66 ... 0x68:
        return trans_cldst(dc, insn);
    case 0x65:
        if (F_OPE4(insn) == 2) {
            /* cswap */
            skip = gen_cexec(dc);
            gen_swap(dc, insn, gen_addr(dc, insn, false));
            gen_set_label(skip);
            return true;
        }
        if (F_OPE4(insn) == 3) {
            /* cscan */
            skip = gen_cexec(dc);
            gen_intop(dc, OP_SCAN, gr(F_GRI(insn)), gr(F_GRJ(insn)),
                      F_GRK(insn), -1);
            gen_set_label(skip);
            return true;
        }
        return false;
    case 0x69: { /* cmovgf, cmovgfd, cmovfg, cmovfgd */
        int ope = F_OPE4(insn);

        skip = gen_cexec(dc);
        gen_movgf(dc, insn, !(ope & 2), ope & 1);
        gen_set_label(skip);
        return true;
    }
    case 0x6a:
        if (F_OPE4(insn) == 2) {
            /* cjmpl, ccalll */
            skip = gen_cexec(dc);
            gen_jmpl(dc, insn, false, true);
            gen_set_label(skip);
            return true;
        }
        if (F_OPE4(insn) < 2) {
            return trans_ck(dc, insn, F_OPE4(insn) == 1, true);
        }
        return false;
    case 0x7b:
        return trans_media(dc, insn);
    default:
        if ((op >= 0x10 && op <= 0x2d) || (op >= 0x18 && op <= 0x1f)) {
            return trans_ri(dc, insn);
        }
        if ((op >= 0x30 && op <= 0x3b) || (op >= 0x4e && op <= 0x56)) {
            return trans_ldst_ri(dc, insn);
        }
        return false;
    }
}

/*
 * Which registers an instruction may touch: every register field it has,
 * an over-estimate that is good enough to find the hazards in a packet.
 */
static uint64_t reg_mask(uint32_t insn)
{
    int k = F_GRK(insn);
    uint64_t m = 0;

    for (int n = 0; n < 4; n++) {
        m |= 1ull << ((k + n) & 63);
    }
    m |= 1ull << F_GRI(insn);
    m |= 1ull << F_GRJ(insn);
    return m;
}

static bool uses_fr(uint32_t insn)
{
    int op = F_OP(insn), ope = F_OPE1(insn);

    if (op == 2 || op == 3) {
        return (ope & 0x08) || ope == 0x15 || ope == 0x16;
    }
    return (op >= 0x38 && op <= 0x3b) || op == 0x4e || op == 0x4f ||
           op == 0x55 || op == 0x56 || op == 0x60 || op == 0x63 ||
           op == 0x66 || op == 0x68 || op == 0x69 || op >= 0x70;
}

static bool writes_ccr(uint32_t insn)
{
    int op = F_OP(insn);

    return op <= 0x01 || op == 0x03 || (op >= 0x08 && op <= 0x0a) ||
           (op >= 0x11 && op <= 0x2d && (op & 1)) ||
           op == 0x59 || op == 0x5b || op == 0x5d || op == 0x6a;
}

static bool reads_ccr(uint32_t insn)
{
    int op = F_OP(insn);

    return op <= 0x01 || op == 0x03 || (op >= 0x04 && op <= 0x0a) ||
           op == 0x0e || (op >= 0x12 && op <= 0x17) || op == 0x1c ||
           op == 0x1d || (op >= 0x58 && op <= 0x77);
}

static bool writes_lr(uint32_t insn)
{
    int op = F_OP(insn);

    return op == 0x03 || (op >= 0x0c && op <= 0x0f) || op == 0x6a;
}

static bool reads_lr(uint32_t insn)
{
    int op = F_OP(insn);

    return op == 0x03 || op == 0x0e;
}

static bool is_branch(uint32_t insn)
{
    int op = F_OP(insn);

    return (op >= 0x06 && op <= 0x07) || (op >= 0x0c && op <= 0x0f) ||
           (op == 0x6a && F_OPE4(insn) == 2);
}

/* Set up the temporaries for the writes that must wait for packet end */
static void plan_packet(DisasContext *dc, const uint32_t *insns, int n)
{
    uint64_t gr_haz = 0, fr_haz = 0, gr_seen = 0, fr_seen = 0;
    bool cc_w = false, cc_haz = false, lr_w = false, lr_haz = false;
    bool branch = false;

    memset(dc->gr_pend, 0, sizeof(dc->gr_pend));
    memset(dc->fr_pend, 0, sizeof(dc->fr_pend));
    memset(dc->icc_pend, 0, sizeof(dc->icc_pend));
    memset(dc->fcc_pend, 0, sizeof(dc->fcc_pend));
    memset(dc->cc_pend, 0, sizeof(dc->cc_pend));
    dc->lr_pend = dc->lcr_pend = NULL;

    for (int i = 0; i < n; i++) {
        uint64_t m = reg_mask(insns[i]);

        gr_haz |= gr_seen & m;
        gr_seen |= m;
        if (uses_fr(insns[i])) {
            fr_haz |= fr_seen & m;
            fr_seen |= m;
        }
        cc_haz |= cc_w && reads_ccr(insns[i]);
        cc_w |= writes_ccr(insns[i]);
        lr_haz |= lr_w && reads_lr(insns[i]);
        lr_w |= writes_lr(insns[i]);
        branch |= is_branch(insns[i]);
    }
    gr_haz &= ~1ull;

    for (int r = 0; r < NUM_GR; r++) {
        if (gr_haz & (1ull << r)) {
            dc->gr_pend[r] = tmp();
            tcg_gen_mov_i32(dc->gr_pend[r], cpu_gr[r]);
        }
        if (fr_haz & (1ull << r)) {
            dc->fr_pend[r] = tmp();
            tcg_gen_mov_i32(dc->fr_pend[r], cpu_fr[r]);
        }
    }
    if (cc_haz) {
        for (int i = 0; i < 4; i++) {
            dc->icc_pend[i] = tmp();
            tcg_gen_mov_i32(dc->icc_pend[i], cpu_icc[i]);
            dc->fcc_pend[i] = tmp();
            tcg_gen_mov_i32(dc->fcc_pend[i], cpu_fcc[i]);
        }
        for (int i = 0; i < 8; i++) {
            dc->cc_pend[i] = tmp();
            tcg_gen_mov_i32(dc->cc_pend[i], cpu_cc[i]);
        }
    }
    if (lr_haz) {
        dc->lr_pend = tmp();
        tcg_gen_mov_i32(dc->lr_pend, cpu_lr);
        dc->lcr_pend = tmp();
        tcg_gen_mov_i32(dc->lcr_pend, cpu_lcr);
    }

    dc->br_kind = 0;
    dc->br_taken = dc->br_dest = NULL;
    if (branch) {
        dc->br_taken = tmp();
        dc->br_dest = tmp();
        tcg_gen_movi_i32(dc->br_taken, 0);
        tcg_gen_movi_i32(dc->br_dest, dc->next);
    }
}

static void commit_packet(DisasContext *dc)
{
    for (int r = 0; r < NUM_GR; r++) {
        if (dc->gr_pend[r]) {
            tcg_gen_mov_i32(cpu_gr[r], dc->gr_pend[r]);
        }
        if (dc->fr_pend[r]) {
            tcg_gen_mov_i32(cpu_fr[r], dc->fr_pend[r]);
        }
    }
    for (int i = 0; i < 4; i++) {
        if (dc->icc_pend[i]) {
            tcg_gen_mov_i32(cpu_icc[i], dc->icc_pend[i]);
            tcg_gen_mov_i32(cpu_fcc[i], dc->fcc_pend[i]);
        }
    }
    for (int i = 0; i < 8; i++) {
        if (dc->cc_pend[i]) {
            tcg_gen_mov_i32(cpu_cc[i], dc->cc_pend[i]);
        }
    }
    if (dc->lr_pend) {
        tcg_gen_mov_i32(cpu_lr, dc->lr_pend);
        tcg_gen_mov_i32(cpu_lcr, dc->lcr_pend);
    }
}

static void gen_goto(DisasContext *dc, int n, uint32_t dest)
{
    if (translator_use_goto_tb(&dc->base, dest)) {
        tcg_gen_goto_tb(n);
        tcg_gen_movi_i32(cpu_pc, dest);
        tcg_gen_exit_tb(dc->base.tb, n);
    } else {
        tcg_gen_movi_i32(cpu_pc, dest);
        tcg_gen_lookup_and_goto_ptr();
    }
}

static int packet_length(CPUFRVState *env, DisasContext *dc, uint32_t addr,
                         uint32_t *insns)
{
    for (int n = 0; n < MAX_PKT; n++) {
        insns[n] = translator_ldl(env, &dc->base, addr + 4 * n);
        if (F_PACK(insns[n])) {
            return n + 1;
        }
    }
    return -1;
}

static void frv_tr_init_disas_context(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *dc = container_of(dcbase, DisasContext, base);
    uint32_t flags = dc->base.tb->flags;

    dc->user = FIELD_EX32(flags, TBFLAGS, USER);
    dc->debug = FIELD_EX32(flags, TBFLAGS, DEBUG);
    dc->mem_idx = dc->user;
}

static void frv_tr_tb_start(DisasContextBase *db, CPUState *cs)
{
}

static void frv_tr_insn_start(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *dc = container_of(dcbase, DisasContext, base);

    /* the second word, filled in later, is the packet after this one */
    dc->pkt = dcbase->pc_next;
    tcg_gen_insn_start(dc->pkt, 0, 0);
    dc->insn_start = tcg_last_op();
}

static void frv_tr_translate_insn(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *dc = container_of(dcbase, DisasContext, base);
    CPUFRVState *env = cpu_env(cs);
    uint32_t insns[MAX_PKT];
    int n = packet_length(env, dc, dc->pkt, insns);

    dc->next = dc->pkt + 4 * (n > 0 ? n : 1);
    tcg_set_insn_start_param(dc->insn_start, 1, dc->next);
    dc->base.pc_next = dc->next;
    dc->trap = dc->brk = dc->exit_tb = false;
    dc->trap_cond = NULL;
    dc->rett = 0;

    if (n < 0) {
        dc->insn = insns[0];
        dc->iaddr = dc->pkt;
        gen_illegal(dc);
        return;
    }

    plan_packet(dc, insns, n);

    for (int i = 0; i < n; i++) {
        dc->insn = insns[i];
        dc->iaddr = dc->pkt + 4 * i;
        if (!decode(dc, insns[i])) {
            gen_illegal(dc);
            return;
        }
        if (dc->base.is_jmp == DISAS_NORETURN) {
            return;
        }
    }

    commit_packet(dc);

    if (dc->trap) {
        TCGLabel *no = NULL;

        if (dc->trap_cond) {
            no = gen_new_label();
            tcg_gen_brcondi_i32(TCG_COND_EQ, dc->trap_cond, 0, no);
        }
        gen_helper_trap(tcg_env, dc->trap_val);
        if (!no) {
            dc->base.is_jmp = DISAS_NORETURN;
            return;
        }
        gen_set_label(no);
    }
    if (dc->brk) {
        gen_raise(dc, TT_BREAK);
        return;
    }
    if (dc->rett) {
        gen_helper_rett(cpu_pc, tcg_env, cst(dc->rett - 1));
        tcg_gen_exit_tb(NULL, 0);
        dc->base.is_jmp = DISAS_NORETURN;
        return;
    }
    if (dc->br_kind == 1 && !dc->exit_tb) {
        gen_goto(dc, 0, dc->br_target);
        dc->base.is_jmp = DISAS_NORETURN;
        return;
    }
    if (dc->br_kind == 3 && !dc->exit_tb) {
        TCGLabel *not_taken = gen_new_label();

        tcg_gen_brcondi_i32(TCG_COND_EQ, dc->br_taken, 0, not_taken);
        gen_goto(dc, 0, dc->br_target);
        gen_set_label(not_taken);
        gen_goto(dc, 1, dc->next);
        dc->base.is_jmp = DISAS_NORETURN;
        return;
    }
    if (dc->br_kind) {
        tcg_gen_mov_i32(cpu_pc, dc->br_dest);
        if (dc->exit_tb) {
            tcg_gen_exit_tb(NULL, 0);
        } else {
            tcg_gen_lookup_and_goto_ptr();
        }
        dc->base.is_jmp = DISAS_NORETURN;
        return;
    }
    if (dc->exit_tb) {
        tcg_gen_movi_i32(cpu_pc, dc->next);
        tcg_gen_exit_tb(NULL, 0);
        dc->base.is_jmp = DISAS_NORETURN;
        return;
    }
    /* stay within the TB's first page */
    if ((dc->next ^ dc->base.pc_first) & TARGET_PAGE_MASK) {
        dc->base.is_jmp = DISAS_TOO_MANY;
    }
}

static void frv_tr_tb_stop(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *dc = container_of(dcbase, DisasContext, base);

    switch (dc->base.is_jmp) {
    case DISAS_NEXT:
    case DISAS_TOO_MANY:
        gen_goto(dc, 1, dc->base.pc_next);
        break;
    case DISAS_NORETURN:
        break;
    default:
        g_assert_not_reached();
    }
}

static const TranslatorOps frv_tr_ops = {
    .init_disas_context = frv_tr_init_disas_context,
    .tb_start           = frv_tr_tb_start,
    .insn_start         = frv_tr_insn_start,
    .translate_insn     = frv_tr_translate_insn,
    .tb_stop            = frv_tr_tb_stop,
};

void frv_translate_code(CPUState *cs, TranslationBlock *tb,
                        int *max_insns, vaddr pc, void *host_pc)
{
    DisasContext dc;

    translator_loop(cs, tb, max_insns, pc, host_pc, &frv_tr_ops, &dc.base,
                    TCG_TYPE_VA);
}

void frv_cpu_dump_state(CPUState *cs, FILE *f, int flags)
{
    CPUFRVState *env = cpu_env(cs);

    qemu_fprintf(f, "PC=%08x PSR=%08x TBR=%08x PCSR=%08x LR=%08x LCR=%08x "
                 "CCR=%08x CCCR=%04x\n", env->pc, env->spr[SPR_PSR],
                 env->spr[SPR_TBR], env->spr[SPR_PCSR], env->spr[SPR_LR],
                 env->spr[SPR_LCR], frv_spr_read(env, SPR_CCR),
                 frv_spr_read(env, SPR_CCCR));
    for (int i = 0; i < NUM_GR; i++) {
        qemu_fprintf(f, "gr%-2d=%08x%c", i, env->gr[i],
                     (i % 8) == 7 ? '\n' : ' ');
    }
    if (flags & CPU_DUMP_FPU) {
        for (int i = 0; i < NUM_FR; i++) {
            qemu_fprintf(f, "fr%-2d=%08x%c", i, env->fr[i],
                         (i % 8) == 7 ? '\n' : ' ');
        }
    }
}

void frv_tcg_init(void)
{
    static char gr_names[NUM_GR][6], fr_names[NUM_FR][6];
    static const char *const icc_names[4] = { "icc0", "icc1", "icc2", "icc3" };
    static const char *const fcc_names[4] = { "fcc0", "fcc1", "fcc2", "fcc3" };
    static const char *const cc_names[8] = {
        "cc0", "cc1", "cc2", "cc3", "cc4", "cc5", "cc6", "cc7"
    };

    for (int i = 0; i < NUM_GR; i++) {
        snprintf(gr_names[i], sizeof(gr_names[i]), "gr%d", i);
        cpu_gr[i] = tcg_global_mem_new_i32(tcg_env,
                                           offsetof(CPUFRVState, gr[i]),
                                           gr_names[i]);
    }
    for (int i = 0; i < NUM_FR; i++) {
        snprintf(fr_names[i], sizeof(fr_names[i]), "fr%d", i);
        cpu_fr[i] = tcg_global_mem_new_i32(tcg_env,
                                           offsetof(CPUFRVState, fr[i]),
                                           fr_names[i]);
    }
    for (int i = 0; i < 4; i++) {
        cpu_icc[i] = tcg_global_mem_new_i32(tcg_env,
                                            offsetof(CPUFRVState, icc[i]),
                                            icc_names[i]);
        cpu_fcc[i] = tcg_global_mem_new_i32(tcg_env,
                                            offsetof(CPUFRVState, fcc[i]),
                                            fcc_names[i]);
    }
    for (int i = 0; i < 8; i++) {
        cpu_cc[i] = tcg_global_mem_new_i32(tcg_env,
                                           offsetof(CPUFRVState, cc[i]),
                                           cc_names[i]);
    }
    cpu_pc = tcg_global_mem_new_i32(tcg_env, offsetof(CPUFRVState, pc), "pc");
    cpu_lr = tcg_global_mem_new_i32(tcg_env,
                                    offsetof(CPUFRVState, spr[SPR_LR]), "lr");
    cpu_lcr = tcg_global_mem_new_i32(tcg_env,
                                     offsetof(CPUFRVState, spr[SPR_LCR]),
                                     "lcr");
}
