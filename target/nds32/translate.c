/*
 * NDS32 (AndeStar V3) instruction translation
 *
 * Encodings follow binutils opcodes/nds32-asm.c; instructions are
 * stored big-endian in both data endiannesses.
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

/* Leave the TB and return to the main loop: state the TB depends on. */
#define DISAS_EXIT      DISAS_TARGET_0
/* cpu_pc already holds the target. */
#define DISAS_JUMP      DISAS_TARGET_1
/* Both: cpu_pc holds the target and the main loop must look again. */
#define DISAS_EXIT_JUMP DISAS_TARGET_2

typedef struct DisasContext {
    DisasContextBase base;
    uint32_t pc;        /* address of the current insn */
    uint32_t tbflags;
    int mem_idx;
    bool user;
    MemOp be;
    MemOp align;
} DisasContext;

static TCGv cpu_R[32];
static TCGv cpu_pc;
static TCGv cpu_D[4];
static TCGv cpu_lock_addr;
static TCGv cpu_lock_val;

static const char * const regnames[32] = {
    "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7",
    "r8", "r9", "r10", "r11", "r12", "r13", "r14", "ta",
    "r16", "r17", "r18", "r19", "r20", "r21", "r22", "r23",
    "r24", "r25", "p0", "p1", "fp", "gp", "lp", "sp",
};

static const int r45map[16] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 16, 17, 18, 19
};

static inline int32_t sext(uint32_t v, int bits)
{
    return sextract32(v, 0, bits);
}

/* Exceptions */

static void gen_raise(DisasContext *dc, int vec, uint32_t itype)
{
    tcg_gen_movi_i32(cpu_pc, dc->pc);
    gen_helper_raise(tcg_env, tcg_constant_i32(vec), tcg_constant_i32(itype),
                     tcg_constant_i32(0));
    dc->base.is_jmp = DISAS_NORETURN;
}

static void gen_reserved(DisasContext *dc)
{
    gen_raise(dc, VEC_GENERAL, ETYPE_RESERVED_INSN);
}

static bool check_priv(DisasContext *dc)
{
    if (dc->user) {
        gen_reserved(dc);
        return false;
    }
    return true;
}

/* Control flow */

static void gen_goto_tb(DisasContext *dc, int n, uint32_t dest)
{
    if (translator_use_goto_tb(&dc->base, dest)) {
        tcg_gen_goto_tb(n);
        tcg_gen_movi_i32(cpu_pc, dest);
        tcg_gen_exit_tb(dc->base.tb, n);
    } else {
        tcg_gen_movi_i32(cpu_pc, dest);
        tcg_gen_lookup_and_goto_ptr();
    }
    dc->base.is_jmp = DISAS_NORETURN;
}

static void gen_jump_imm(DisasContext *dc, uint32_t dest)
{
    gen_goto_tb(dc, 0, dest);
}

static void gen_brcond(DisasContext *dc, TCGCond cond, TCGv a, TCGv b,
                       uint32_t dest, int len)
{
    TCGLabel *l = gen_new_label();

    tcg_gen_brcond_i32(cond, a, b, l);
    gen_goto_tb(dc, 0, dc->pc + len);
    gen_set_label(l);
    gen_goto_tb(dc, 1, dest);
}

static void gen_brcondi(DisasContext *dc, TCGCond cond, TCGv a, int32_t b,
                        uint32_t dest, int len)
{
    gen_brcond(dc, cond, a, tcg_constant_i32(b), dest, len);
}

/* Memory */

static void gen_ld(DisasContext *dc, TCGv dst, TCGv addr, MemOp mop, int idx)
{
    if ((mop & MO_SIZE) != MO_8) {
        mop |= dc->be | dc->align;
    }
    tcg_gen_qemu_ld_i32(dst, addr, idx, mop);
}

static void gen_st(DisasContext *dc, TCGv val, TCGv addr, MemOp mop, int idx)
{
    if ((mop & MO_SIZE) != MO_8) {
        mop |= dc->be | dc->align;
    }
    tcg_gen_qemu_st_i32(val, addr, idx, mop);
}

/* Load/store with base + offset; post-modify ("bi") updates the base. */
static void gen_ldst(DisasContext *dc, bool store, MemOp mop, int rt,
                     TCGv base_reg, TCGv off, bool bi, int idx)
{
    TCGv addr = tcg_temp_new();

    if (bi) {
        tcg_gen_mov_i32(addr, base_reg);
    } else {
        tcg_gen_add_i32(addr, base_reg, off);
    }
    if (store) {
        gen_st(dc, cpu_R[rt], addr, mop, idx);
        if (bi) {
            tcg_gen_add_i32(base_reg, base_reg, off);
        }
    } else {
        TCGv val = tcg_temp_new();
        gen_ld(dc, val, addr, mop, idx);
        if (bi) {
            tcg_gen_add_i32(base_reg, base_reg, off);
        }
        tcg_gen_mov_i32(cpu_R[rt], val);
    }
}

static void gen_ldst_imm(DisasContext *dc, bool store, MemOp mop, int rt,
                         int ra, int32_t off, bool bi)
{
    gen_ldst(dc, store, mop, rt, cpu_R[ra], tcg_constant_i32(off), bi,
             dc->mem_idx);
}

/*
 * lmw/smw: registers rb..re (when below $fp) and then fp, gp, lp, sp as
 * selected by enable4, in ascending order from the lowest address.
 */
static void gen_lsmw(DisasContext *dc, uint32_t insn)
{
    int rb = extract32(insn, 20, 5);
    int ra = extract32(insn, 15, 5);
    int re = extract32(insn, 10, 5);
    int en4 = extract32(insn, 6, 4);
    bool store = extract32(insn, 5, 1);
    int mode = extract32(insn, 2, 3);
    bool modify = mode & 1, dec = mode & 2, after = mode & 4;
    int regs[32], n = 0;
    TCGv addr, vals[32];
    int32_t start;

    /* lmw/smw accept any alignment, lmwa/smwa trap on misaligned words. */
    MemOp mop = MO_UL | dc->be |
                (extract32(insn, 0, 2) == 1 ? MO_ALIGN : MO_UNALN);

    if (extract32(insn, 0, 2) == 2) {
        gen_reserved(dc);       /* lmwzb/smwzb */
        return;
    }
    if (rb < R_FP && re < R_FP) {
        for (int r = rb; r <= re; r++) {
            regs[n++] = r;
        }
    }
    for (int i = 3; i >= 0; i--) {
        if (en4 & (1 << i)) {
            regs[n++] = R_SP - i;
        }
    }
    if (n == 0) {
        return;
    }

    if (!dec) {
        start = after ? 4 : 0;
    } else {
        start = after ? -4 * n : -4 * (n - 1);
    }

    addr = tcg_temp_new();
    tcg_gen_addi_i32(addr, cpu_R[ra], start);
    for (int i = 0; i < n; i++) {
        if (store) {
            tcg_gen_qemu_st_i32(cpu_R[regs[i]], addr, dc->mem_idx, mop);
        } else {
            vals[i] = tcg_temp_new();
            tcg_gen_qemu_ld_i32(vals[i], addr, dc->mem_idx, mop);
        }
        if (i + 1 < n) {
            tcg_gen_addi_i32(addr, addr, 4);
        }
    }
    if (!store) {
        for (int i = 0; i < n; i++) {
            tcg_gen_mov_i32(cpu_R[regs[i]], vals[i]);
        }
    }
    if (modify) {
        tcg_gen_addi_i32(cpu_R[ra], cpu_R[ra], dec ? -4 * n : 4 * n);
    }
}

/* 32-bit groups */

static void gen_alu1(DisasContext *dc, uint32_t insn)
{
    int rt = extract32(insn, 20, 5);
    int ra = extract32(insn, 15, 5);
    int rb = extract32(insn, 10, 5);
    int sh = extract32(insn, 5, 5);
    TCGv t = tcg_temp_new();
    TCGv d = cpu_R[rt];
    TCGv a = cpu_R[ra];
    TCGv b = cpu_R[rb];

    switch (extract32(insn, 0, 5)) {
    case 0x00: /* add / add_slli */
        tcg_gen_shli_i32(t, b, sh);
        tcg_gen_add_i32(d, a, t);
        break;
    case 0x01:
        tcg_gen_shli_i32(t, b, sh);
        tcg_gen_sub_i32(d, a, t);
        break;
    case 0x02:
        tcg_gen_shli_i32(t, b, sh);
        tcg_gen_and_i32(d, a, t);
        break;
    case 0x03:
        tcg_gen_shli_i32(t, b, sh);
        tcg_gen_xor_i32(d, a, t);
        break;
    case 0x04:
        tcg_gen_shli_i32(t, b, sh);
        tcg_gen_or_i32(d, a, t);
        break;
    case 0x05:
        tcg_gen_nor_i32(d, a, b);
        break;
    case 0x06:
        tcg_gen_setcond_i32(TCG_COND_LTU, d, a, b);
        break;
    case 0x07:
        tcg_gen_setcond_i32(TCG_COND_LT, d, a, b);
        break;
    case 0x08:
        tcg_gen_shli_i32(d, a, rb);
        break;
    case 0x09:
        tcg_gen_shri_i32(d, a, rb);
        break;
    case 0x0a:
        tcg_gen_sari_i32(d, a, rb);
        break;
    case 0x0b:
        tcg_gen_rotri_i32(d, a, rb);
        break;
    case 0x0c:
        tcg_gen_andi_i32(t, b, 31);
        tcg_gen_shl_i32(d, a, t);
        break;
    case 0x0d:
        tcg_gen_andi_i32(t, b, 31);
        tcg_gen_shr_i32(d, a, t);
        break;
    case 0x0e:
        tcg_gen_andi_i32(t, b, 31);
        tcg_gen_sar_i32(d, a, t);
        break;
    case 0x0f:
        tcg_gen_andi_i32(t, b, 31);
        tcg_gen_rotr_i32(d, a, t);
        break;
    case 0x10:
        tcg_gen_ext8s_i32(d, a);
        break;
    case 0x11:
        tcg_gen_ext16s_i32(d, a);
        break;
    case 0x12:
        tcg_gen_andc_i32(d, a, b);
        break;
    case 0x13:
        tcg_gen_ext16u_i32(d, a);
        break;
    case 0x14: /* wsbh: swap the bytes of each halfword */
        {
            TCGv u = tcg_temp_new();
            tcg_gen_shri_i32(t, a, 8);
            tcg_gen_andi_i32(t, t, 0x00ff00ff);
            tcg_gen_shli_i32(u, a, 8);
            tcg_gen_andi_i32(u, u, 0xff00ff00);
            tcg_gen_or_i32(d, t, u);
        }
        break;
    case 0x15:
        tcg_gen_shri_i32(t, b, sh);
        tcg_gen_or_i32(d, a, t);
        break;
    case 0x16: /* divsr rt, rd, ra, rb */
    case 0x17: /* divr */
        {
            TCGv_i64 r = tcg_temp_new_i64();
            tcg_gen_movi_i32(cpu_pc, dc->pc);
            if ((insn & 0x1f) == 0x16) {
                gen_helper_divsr(r, tcg_env, a, b);
            } else {
                gen_helper_divr(r, tcg_env, a, b);
            }
            if (sh != rt) {
                tcg_gen_extrh_i64_i32(cpu_R[sh], r);
            }
            tcg_gen_extrl_i64_i32(cpu_R[rt], r);
        }
        break;
    case 0x18: /* sva: signed overflow of a + b */
        {
            TCGv s = tcg_temp_new();
            tcg_gen_add_i32(s, a, b);
            tcg_gen_xor_i32(t, s, a);
            tcg_gen_xor_i32(s, s, b);
            tcg_gen_and_i32(t, t, s);
            tcg_gen_shri_i32(d, t, 31);
        }
        break;
    case 0x19: /* svs: signed overflow of a - b */
        {
            TCGv s = tcg_temp_new();
            tcg_gen_sub_i32(s, a, b);
            tcg_gen_xor_i32(t, a, b);
            tcg_gen_xor_i32(s, s, a);
            tcg_gen_and_i32(t, t, s);
            tcg_gen_shri_i32(d, t, 31);
        }
        break;
    case 0x1a:
        tcg_gen_movcond_i32(TCG_COND_EQ, d, b, tcg_constant_i32(0), a, d);
        break;
    case 0x1b:
        tcg_gen_movcond_i32(TCG_COND_NE, d, b, tcg_constant_i32(0), a, d);
        break;
    case 0x1c:
        tcg_gen_shri_i32(t, b, sh);
        tcg_gen_add_i32(d, a, t);
        break;
    case 0x1d:
        tcg_gen_shri_i32(t, b, sh);
        tcg_gen_sub_i32(d, a, t);
        break;
    case 0x1e:
        tcg_gen_shri_i32(t, b, sh);
        tcg_gen_and_i32(d, a, t);
        break;
    case 0x1f:
        tcg_gen_shri_i32(t, b, sh);
        tcg_gen_xor_i32(d, a, t);
        break;
    }
}

static void gen_mul64(TCGv_i64 r, TCGv a, TCGv b, bool sign)
{
    TCGv lo = tcg_temp_new(), hi = tcg_temp_new();

    if (sign) {
        tcg_gen_muls2_i32(lo, hi, a, b);
    } else {
        tcg_gen_mulu2_i32(lo, hi, a, b);
    }
    tcg_gen_concat_i32_i64(r, lo, hi);
}

static void gen_alu2(DisasContext *dc, uint32_t insn)
{
    int rt = extract32(insn, 20, 5);
    int ra = extract32(insn, 15, 5);
    int rb = extract32(insn, 10, 5);
    int dt = extract32(insn, 21, 1);
    int grp = extract32(insn, 6, 4);
    int sub = extract32(insn, 0, 6);
    TCGv d = cpu_R[rt];
    TCGv a = cpu_R[ra];
    TCGv b = cpu_R[rb];
    TCGv t = tcg_temp_new();
    TCGv_i64 p = tcg_temp_new_i64();
    TCGv_i64 acc = tcg_temp_new_i64();

    if (grp == 1) {
        switch (sub) {
        case 0x28: /* mulsr64 */
        case 0x29: /* mulr64 */
            {
                TCGv lo = tcg_temp_new(), hi = tcg_temp_new();
                if (sub == 0x28) {
                    tcg_gen_muls2_i32(lo, hi, a, b);
                } else {
                    tcg_gen_mulu2_i32(lo, hi, a, b);
                }
                rt &= ~1;
                if (dc->be) {
                    tcg_gen_mov_i32(cpu_R[rt], hi);
                    tcg_gen_mov_i32(cpu_R[rt + 1], lo);
                } else {
                    tcg_gen_mov_i32(cpu_R[rt], lo);
                    tcg_gen_mov_i32(cpu_R[rt + 1], hi);
                }
            }
            return;
        case 0x33: /* maddr32 */
            tcg_gen_mul_i32(t, a, b);
            tcg_gen_add_i32(d, d, t);
            return;
        case 0x35: /* msubr32 */
            tcg_gen_mul_i32(t, a, b);
            tcg_gen_sub_i32(d, d, t);
            return;
        }
        gen_reserved(dc);
        return;
    }
    if (grp != 0) {
        gen_reserved(dc);
        return;
    }

    switch (sub) {
    case 0x00:
        tcg_gen_smax_i32(d, a, b);
        break;
    case 0x01:
        tcg_gen_smin_i32(d, a, b);
        break;
    case 0x02: /* ave: (a + b + 1) >> 1 in 33 bits */
        {
            TCGv_i64 x = tcg_temp_new_i64(), y = tcg_temp_new_i64();
            tcg_gen_ext_i32_i64(x, a);
            tcg_gen_ext_i32_i64(y, b);
            tcg_gen_add_i64(x, x, y);
            tcg_gen_addi_i64(x, x, 1);
            tcg_gen_sari_i64(x, x, 1);
            tcg_gen_extrl_i64_i32(d, x);
        }
        break;
    case 0x03:
        tcg_gen_abs_i32(d, a);
        break;
    case 0x04: /* clips: clamp to [-2^imm, 2^imm - 1] */
        tcg_gen_smin_i32(t, a, tcg_constant_i32((1u << rb) - 1));
        tcg_gen_smax_i32(d, t, tcg_constant_i32(-(1 << rb)));
        break;
    case 0x05: /* clip: clamp to [0, 2^imm - 1] */
        tcg_gen_smin_i32(t, a, tcg_constant_i32((1u << rb) - 1));
        tcg_gen_smax_i32(d, t, tcg_constant_i32(0));
        break;
    case 0x06:
        tcg_gen_not_i32(t, a);
        tcg_gen_clzi_i32(d, t, 32);
        break;
    case 0x07:
        tcg_gen_clzi_i32(d, a, 32);
        break;
    case 0x08:
        tcg_gen_ori_i32(d, a, 1u << rb);
        break;
    case 0x09:
        tcg_gen_andi_i32(d, a, ~(1u << rb));
        break;
    case 0x0a:
        tcg_gen_xori_i32(d, a, 1u << rb);
        break;
    case 0x0b:
        tcg_gen_extract_i32(d, a, rb, 1);
        break;
    case 0x20: /* mfusr */
    case 0x21: /* mtusr */
        {
            int group = extract32(insn, 10, 5);
            int usr = extract32(insn, 15, 5);
            bool to = sub == 0x21;

            if (group == 0 && usr < 4) {
                if (to) {
                    tcg_gen_mov_i32(cpu_D[usr], d);
                } else {
                    tcg_gen_mov_i32(d, cpu_D[usr]);
                }
            } else if (group == 0 && usr == 31) {
                if (to) {
                    tcg_gen_mov_i32(cpu_pc, d);
                    dc->base.is_jmp = DISAS_JUMP;
                } else {
                    tcg_gen_movi_i32(d, dc->pc);
                }
            } else if (!to) {
                tcg_gen_movi_i32(d, 0);
            }
        }
        break;
    case 0x24:
        tcg_gen_mul_i32(d, a, b);
        break;
    case 0x28: /* mults64 */
    case 0x29: /* mult64 */
        gen_mul64(p, a, b, sub == 0x28);
        tcg_gen_extr_i64_i32(cpu_D[2 * dt], cpu_D[2 * dt + 1], p);
        break;
    case 0x2a: /* madds64 */
    case 0x2b: /* madd64 */
    case 0x2c: /* msubs64 */
    case 0x2d: /* msub64 */
        gen_mul64(p, a, b, !(sub & 1));
        tcg_gen_concat_i32_i64(acc, cpu_D[2 * dt], cpu_D[2 * dt + 1]);
        if (sub < 0x2c) {
            tcg_gen_add_i64(acc, acc, p);
        } else {
            tcg_gen_sub_i64(acc, acc, p);
        }
        tcg_gen_extr_i64_i32(cpu_D[2 * dt], cpu_D[2 * dt + 1], acc);
        break;
    case 0x2e: /* divs */
    case 0x2f: /* div */
        tcg_gen_movi_i32(cpu_pc, dc->pc);
        if (sub == 0x2e) {
            gen_helper_divsr(p, tcg_env, a, b);
        } else {
            gen_helper_divr(p, tcg_env, a, b);
        }
        tcg_gen_extr_i64_i32(cpu_D[2 * dt], cpu_D[2 * dt + 1], p);
        break;
    case 0x31: /* mult32 */
        tcg_gen_mul_i32(cpu_D[2 * dt], a, b);
        break;
    case 0x33: /* madd32 */
        tcg_gen_mul_i32(t, a, b);
        tcg_gen_add_i32(cpu_D[2 * dt], cpu_D[2 * dt], t);
        break;
    case 0x35: /* msub32 */
        tcg_gen_mul_i32(t, a, b);
        tcg_gen_sub_i32(cpu_D[2 * dt], cpu_D[2 * dt], t);
        break;
    default:
        gen_reserved(dc);
        break;
    }
}

static void gen_mem(DisasContext *dc, uint32_t insn)
{
    int rt = extract32(insn, 20, 5);
    int ra = extract32(insn, 15, 5);
    int rb = extract32(insn, 10, 5);
    int sv = extract32(insn, 8, 2);
    int sub = extract32(insn, 0, 6);
    TCGv off = tcg_temp_new();
    int uidx = MMU_IDX_USER |
               (FIELD_EX32(dc->tbflags, TBFLAGS, DT) ? MMU_IDX_XLATE : 0);

    tcg_gen_shli_i32(off, cpu_R[rb], sv);

    switch (sub) {
    case 0x00: gen_ldst(dc, false, MO_UB, rt, cpu_R[ra], off, false, dc->mem_idx); break;
    case 0x01: gen_ldst(dc, false, MO_UW, rt, cpu_R[ra], off, false, dc->mem_idx); break;
    case 0x02: gen_ldst(dc, false, MO_UL, rt, cpu_R[ra], off, false, dc->mem_idx); break;
    case 0x04: gen_ldst(dc, false, MO_UB, rt, cpu_R[ra], off, true, dc->mem_idx); break;
    case 0x05: gen_ldst(dc, false, MO_UW, rt, cpu_R[ra], off, true, dc->mem_idx); break;
    case 0x06: gen_ldst(dc, false, MO_UL, rt, cpu_R[ra], off, true, dc->mem_idx); break;
    case 0x08: gen_ldst(dc, true, MO_UB, rt, cpu_R[ra], off, false, dc->mem_idx); break;
    case 0x09: gen_ldst(dc, true, MO_UW, rt, cpu_R[ra], off, false, dc->mem_idx); break;
    case 0x0a: gen_ldst(dc, true, MO_UL, rt, cpu_R[ra], off, false, dc->mem_idx); break;
    case 0x0c: gen_ldst(dc, true, MO_UB, rt, cpu_R[ra], off, true, dc->mem_idx); break;
    case 0x0d: gen_ldst(dc, true, MO_UW, rt, cpu_R[ra], off, true, dc->mem_idx); break;
    case 0x0e: gen_ldst(dc, true, MO_UL, rt, cpu_R[ra], off, true, dc->mem_idx); break;
    case 0x10: gen_ldst(dc, false, MO_SB, rt, cpu_R[ra], off, false, dc->mem_idx); break;
    case 0x11: gen_ldst(dc, false, MO_SW, rt, cpu_R[ra], off, false, dc->mem_idx); break;
    case 0x13: break; /* dpref */
    case 0x14: gen_ldst(dc, false, MO_SB, rt, cpu_R[ra], off, true, dc->mem_idx); break;
    case 0x15: gen_ldst(dc, false, MO_SW, rt, cpu_R[ra], off, true, dc->mem_idx); break;
    case 0x18: /* llw */
        {
            TCGv addr = tcg_temp_new(), val = tcg_temp_new();
            tcg_gen_add_i32(addr, cpu_R[ra], off);
            gen_ld(dc, val, addr, MO_UL | MO_ALIGN, dc->mem_idx);
            tcg_gen_mov_i32(cpu_lock_addr, addr);
            tcg_gen_mov_i32(cpu_lock_val, val);
            tcg_gen_mov_i32(cpu_R[rt], val);
        }
        break;
    case 0x19: /* scw: rt = 1 on success, 0 on failure */
        {
            TCGv addr = tcg_temp_new(), old = tcg_temp_new();
            TCGLabel *fail = gen_new_label(), *done = gen_new_label();

            tcg_gen_add_i32(addr, cpu_R[ra], off);
            tcg_gen_brcond_i32(TCG_COND_NE, addr, cpu_lock_addr, fail);
            tcg_gen_atomic_cmpxchg_i32(old, addr, cpu_lock_val, cpu_R[rt],
                                       dc->mem_idx,
                                       MO_UL | MO_ALIGN | dc->be);
            tcg_gen_setcond_i32(TCG_COND_EQ, cpu_R[rt], old, cpu_lock_val);
            tcg_gen_br(done);
            gen_set_label(fail);
            tcg_gen_movi_i32(cpu_R[rt], 0);
            gen_set_label(done);
            tcg_gen_movi_i32(cpu_lock_addr, -1);
        }
        break;
    case 0x20: gen_ldst(dc, false, MO_UB, rt, cpu_R[ra], off, false, uidx); break;
    case 0x22: gen_ldst(dc, false, MO_UL, rt, cpu_R[ra], off, false, uidx); break;
    case 0x28: gen_ldst(dc, true, MO_UB, rt, cpu_R[ra], off, false, uidx); break;
    case 0x2a: gen_ldst(dc, true, MO_UL, rt, cpu_R[ra], off, false, uidx); break;
    default:
        gen_reserved(dc);
        break;
    }
}

static void gen_misc(DisasContext *dc, uint32_t insn)
{
    int rt = extract32(insn, 20, 5);
    int ra = extract32(insn, 15, 5);
    int sr = extract32(insn, 10, 10);
    uint32_t swid = extract32(insn, 5, 15);

    switch (extract32(insn, 0, 5)) {
    case 0x00: /* standby */
        if (!dc->user) {
            tcg_gen_movi_i32(cpu_pc, dc->pc + 4);
            gen_helper_standby(tcg_env);
            dc->base.is_jmp = DISAS_NORETURN;
        }
        break;
    case 0x01: /* cctl */
        switch (extract32(insn, 5, 5)) {
        case 0x03: case 0x04: case 0x13: case 0x14:
            if (check_priv(dc)) {
                tcg_gen_movi_i32(cpu_R[rt], 0);
            }
            break;
        default:
            break;
        }
        break;
    case 0x02: /* mfsr */
        if (check_priv(dc)) {
            gen_helper_mfsr(cpu_R[rt], tcg_env, tcg_constant_i32(sr));
        }
        break;
    case 0x03: /* mtsr, setend, setgie */
        if (extract32(insn, 5, 2) == 0) {
            if (check_priv(dc)) {
                gen_helper_mtsr(tcg_env, tcg_constant_i32(sr), cpu_R[rt]);
                dc->base.is_jmp = DISAS_EXIT;
            }
        } else {
            int bit = extract32(insn, 5, 1) ? R_PSW_BE_SHIFT : R_PSW_GIE_SHIFT;
            TCGv psw = tcg_temp_new();

            if (bit == R_PSW_GIE_SHIFT && !check_priv(dc)) {
                break;
            }
            gen_helper_mfsr(psw, tcg_env, tcg_constant_i32(SR_PSW));
            tcg_gen_deposit_i32(psw, psw, tcg_constant_i32(rt & 1), bit, 1);
            gen_helper_mtsr(tcg_env, tcg_constant_i32(SR_PSW), psw);
            dc->base.is_jmp = DISAS_EXIT;
        }
        break;
    case 0x04: /* iret */
        if (check_priv(dc)) {
            gen_helper_iret(tcg_env);
            dc->base.is_jmp = DISAS_EXIT_JUMP;
        }
        break;
    case 0x05: /* trap */
        gen_raise(dc, VEC_GENERAL, ETYPE_TRAP | (swid << R_ITYPE_SWID_SHIFT));
        break;
    case 0x06: /* teqz */
    case 0x07: /* tnez */
        {
            TCGLabel *skip = gen_new_label();
            tcg_gen_brcondi_i32((insn & 1) ? TCG_COND_EQ : TCG_COND_NE,
                                cpu_R[rt], 0, skip);
            tcg_gen_movi_i32(cpu_pc, dc->pc);
            gen_helper_raise(tcg_env, tcg_constant_i32(VEC_GENERAL),
                             tcg_constant_i32(ETYPE_TRAP |
                                              (swid << R_ITYPE_SWID_SHIFT)),
                             tcg_constant_i32(0));
            gen_set_label(skip);
        }
        break;
    case 0x08: /* dsb */
    case 0x09: /* isb */
    case 0x0c: /* msync */
    case 0x0d: /* isync */
        break;
    case 0x0a: /* break */
        gen_raise(dc, VEC_DEBUG, swid << R_ITYPE_SWID_SHIFT);
        break;
    case 0x0b: /* syscall */
        gen_raise(dc, VEC_SYSCALL, swid << R_ITYPE_SWID_SHIFT);
        break;
    case 0x0e: /* tlbop */
        if (!check_priv(dc)) {
            break;
        }
        if (extract32(insn, 5, 5) == 5) {
            gen_helper_tlbop_probe(cpu_R[rt], tcg_env, cpu_R[ra]);
        } else {
            gen_helper_tlbop(tcg_env, cpu_R[ra],
                             tcg_constant_i32(extract32(insn, 5, 5)));
            dc->base.is_jmp = DISAS_EXIT;
        }
        break;
    default:
        gen_reserved(dc);
        break;
    }
}

/* Set or clear PSW.IT/DT for the dtiton/dtitoff forms of jr/jral. */
static void gen_dtit(int dtit, bool on)
{
    uint32_t mask = 0;
    TCGv psw = tcg_temp_new();

    if (dtit & 1) {
        mask |= R_PSW_IT_MASK;
    }
    if (dtit & 2) {
        mask |= R_PSW_DT_MASK;
    }
    gen_helper_mfsr(psw, tcg_env, tcg_constant_i32(SR_PSW));
    if (on) {
        tcg_gen_ori_i32(psw, psw, mask);
    } else {
        tcg_gen_andi_i32(psw, psw, ~mask);
    }
    gen_helper_mtsr(tcg_env, tcg_constant_i32(SR_PSW), psw);
}

static void gen_jreg(DisasContext *dc, uint32_t insn)
{
    int rt = extract32(insn, 20, 5);
    int rb = extract32(insn, 10, 5);
    int dtit = extract32(insn, 8, 2);
    int sub = extract32(insn, 0, 5);
    TCGv dest = tcg_temp_new();

    if (sub > 3) {
        gen_reserved(dc);
        return;
    }
    if (dtit && !check_priv(dc)) {
        return;
    }
    tcg_gen_mov_i32(dest, cpu_R[rb]);
    if (sub & 1) {
        tcg_gen_movi_i32(cpu_R[rt], dc->pc + 4);
    }
    if (sub & 2) {
        TCGLabel *l = gen_new_label();
        tcg_gen_brcondi_i32(TCG_COND_EQ, dest, 0, l);
        tcg_gen_mov_i32(cpu_pc, dest);
        tcg_gen_lookup_and_goto_ptr();
        gen_set_label(l);
        gen_goto_tb(dc, 0, dc->pc + 4);
        return;
    }
    if (dtit) {
        gen_dtit(dtit, sub & 1);
        tcg_gen_mov_i32(cpu_pc, dest);
        dc->base.is_jmp = DISAS_EXIT_JUMP;
        return;
    }
    tcg_gen_mov_i32(cpu_pc, dest);
    dc->base.is_jmp = DISAS_JUMP;
}

static void decode32(DisasContext *dc, uint32_t insn)
{
    int rt = extract32(insn, 20, 5);
    int ra = extract32(insn, 15, 5);
    int32_t imm15s = sext(insn, 15);
    uint32_t imm15u = insn & 0x7fff;
    uint32_t pc = dc->pc;

    switch (extract32(insn, 25, 6)) {
    case 0x00: gen_ldst_imm(dc, false, MO_UB, rt, ra, imm15s, false); break;
    case 0x01: gen_ldst_imm(dc, false, MO_UW, rt, ra, imm15s << 1, false); break;
    case 0x02: gen_ldst_imm(dc, false, MO_UL, rt, ra, imm15s << 2, false); break;
    case 0x04: gen_ldst_imm(dc, false, MO_UB, rt, ra, imm15s, true); break;
    case 0x05: gen_ldst_imm(dc, false, MO_UW, rt, ra, imm15s << 1, true); break;
    case 0x06: gen_ldst_imm(dc, false, MO_UL, rt, ra, imm15s << 2, true); break;
    case 0x08: gen_ldst_imm(dc, true, MO_UB, rt, ra, imm15s, false); break;
    case 0x09: gen_ldst_imm(dc, true, MO_UW, rt, ra, imm15s << 1, false); break;
    case 0x0a: gen_ldst_imm(dc, true, MO_UL, rt, ra, imm15s << 2, false); break;
    case 0x0c: gen_ldst_imm(dc, true, MO_UB, rt, ra, imm15s, true); break;
    case 0x0d: gen_ldst_imm(dc, true, MO_UW, rt, ra, imm15s << 1, true); break;
    case 0x0e: gen_ldst_imm(dc, true, MO_UL, rt, ra, imm15s << 2, true); break;
    case 0x10: gen_ldst_imm(dc, false, MO_SB, rt, ra, imm15s, false); break;
    case 0x11: gen_ldst_imm(dc, false, MO_SW, rt, ra, imm15s << 1, false); break;
    case 0x13: break; /* dprefi */
    case 0x14: gen_ldst_imm(dc, false, MO_SB, rt, ra, imm15s, true); break;
    case 0x15: gen_ldst_imm(dc, false, MO_SW, rt, ra, imm15s << 1, true); break;
    case 0x17: /* lbi.gp / lbsi.gp */
        gen_ldst_imm(dc, false, extract32(insn, 19, 1) ? MO_SB : MO_UB,
                     rt, R_GP, sext(insn, 19), false);
        break;
    case 0x18: case 0x19: case 0x1a: case 0x1b: case 0x35:
        gen_raise(dc, VEC_GENERAL, ETYPE_COPROC);
        break;
    case 0x1c:
        gen_mem(dc, insn);
        break;
    case 0x1d:
        gen_lsmw(dc, insn);
        break;
    case 0x1e: /* lhi.gp, lhsi.gp, shi.gp, lwi.gp, swi.gp */
        switch (extract32(insn, 18, 2)) {
        case 0:
            gen_ldst_imm(dc, false, MO_UW, rt, R_GP, sext(insn, 18) << 1, false);
            break;
        case 1:
            gen_ldst_imm(dc, false, MO_SW, rt, R_GP, sext(insn, 18) << 1, false);
            break;
        case 2:
            gen_ldst_imm(dc, true, MO_UW, rt, R_GP, sext(insn, 18) << 1, false);
            break;
        case 3:
            gen_ldst_imm(dc, extract32(insn, 17, 1), MO_UL, rt, R_GP,
                         sext(insn, 17) << 2, false);
            break;
        }
        break;
    case 0x1f: /* sbi.gp / addi.gp */
        if (extract32(insn, 19, 1)) {
            tcg_gen_addi_i32(cpu_R[rt], cpu_R[R_GP], sext(insn, 19));
        } else {
            gen_ldst_imm(dc, true, MO_UB, rt, R_GP, sext(insn, 19), false);
        }
        break;
    case 0x20:
        gen_alu1(dc, insn);
        break;
    case 0x21:
        gen_alu2(dc, insn);
        break;
    case 0x22: /* movi */
        tcg_gen_movi_i32(cpu_R[rt], sext(insn, 20));
        break;
    case 0x23: /* sethi */
        tcg_gen_movi_i32(cpu_R[rt], (insn & 0xfffff) << 12);
        break;
    case 0x24: /* j / jal */
        if (extract32(insn, 24, 1)) {
            tcg_gen_movi_i32(cpu_R[R_LP], pc + 4);
        }
        gen_jump_imm(dc, pc + (sext(insn, 24) << 1));
        break;
    case 0x25:
        gen_jreg(dc, insn);
        break;
    case 0x26: /* beq / bne */
        gen_brcond(dc, extract32(insn, 14, 1) ? TCG_COND_NE : TCG_COND_EQ,
                   cpu_R[rt], cpu_R[ra], pc + (sext(insn, 14) << 1), 4);
        break;
    case 0x27:
        {
            uint32_t dest = pc + (sext(insn, 16) << 1);
            static const TCGCond conds[8] = {
                [2] = TCG_COND_EQ, [3] = TCG_COND_NE, [4] = TCG_COND_GE,
                [5] = TCG_COND_LT, [6] = TCG_COND_GT, [7] = TCG_COND_LE,
            };
            int sub = extract32(insn, 16, 4);

            switch (sub) {
            case 2: case 3: case 4: case 5: case 6: case 7:
                gen_brcondi(dc, conds[sub], cpu_R[rt], 0, dest, 4);
                break;
            case 0xc: /* bgezal */
            case 0xd: /* bltzal */
                {
                    TCGv v = tcg_temp_new();
                    tcg_gen_mov_i32(v, cpu_R[rt]);
                    tcg_gen_movi_i32(cpu_R[R_LP], pc + 4);
                    gen_brcondi(dc, sub == 0xc ? TCG_COND_GE : TCG_COND_LT,
                                v, 0, dest, 4);
                }
                break;
            default:
                gen_reserved(dc);
                break;
            }
        }
        break;
    case 0x28:
        tcg_gen_addi_i32(cpu_R[rt], cpu_R[ra], imm15s);
        break;
    case 0x29:
        tcg_gen_subfi_i32(cpu_R[rt], imm15s, cpu_R[ra]);
        break;
    case 0x2a:
        tcg_gen_andi_i32(cpu_R[rt], cpu_R[ra], imm15u);
        break;
    case 0x2b:
        tcg_gen_xori_i32(cpu_R[rt], cpu_R[ra], imm15u);
        break;
    case 0x2c:
        tcg_gen_ori_i32(cpu_R[rt], cpu_R[ra], imm15u);
        break;
    case 0x2d: /* beqc / bnec */
        gen_brcondi(dc, extract32(insn, 19, 1) ? TCG_COND_NE : TCG_COND_EQ,
                    cpu_R[rt], sextract32(insn, 8, 11),
                    pc + (sext(insn, 8) << 1), 4);
        break;
    case 0x2e:
        tcg_gen_setcondi_i32(TCG_COND_LTU, cpu_R[rt], cpu_R[ra], imm15s);
        break;
    case 0x2f:
        tcg_gen_setcondi_i32(TCG_COND_LT, cpu_R[rt], cpu_R[ra], imm15s);
        break;
    case 0x32:
        gen_misc(dc, insn);
        break;
    case 0x33:
        tcg_gen_andi_i32(cpu_R[rt], cpu_R[ra], ~imm15u);
        break;
    default:
        gen_reserved(dc);
        break;
    }
}

/* 16-bit instructions */

static void decode16(DisasContext *dc, uint32_t h)
{
    uint32_t pc = dc->pc;
    int rt5 = extract32(h, 5, 5), ra5 = extract32(h, 0, 5);
    int rt4 = r45map[extract32(h, 5, 4)];
    int rt3 = extract32(h, 6, 3), ra3 = extract32(h, 3, 3), rb3 = h & 7;
    int rt38 = extract32(h, 8, 3);
    uint32_t imm5u = h & 0x1f, imm3u = h & 7;
    int32_t imm8s = sext(h, 8);

    switch (h >> 11) {
    case 0x18: /* beqz38 */
        gen_brcondi(dc, TCG_COND_EQ, cpu_R[rt38], 0, pc + (imm8s << 1), 2);
        return;
    case 0x19: /* bnez38 */
        gen_brcondi(dc, TCG_COND_NE, cpu_R[rt38], 0, pc + (imm8s << 1), 2);
        return;
    case 0x1a: /* beqs38, j8 */
        if (rt38 == 5) {
            gen_jump_imm(dc, pc + (imm8s << 1));
        } else {
            gen_brcond(dc, TCG_COND_EQ, cpu_R[rt38], cpu_R[5],
                       pc + (imm8s << 1), 2);
        }
        return;
    case 0x1b: /* bnes38, and the jr5 group */
        if (rt38 != 5) {
            gen_brcond(dc, TCG_COND_NE, cpu_R[rt38], cpu_R[5],
                       pc + (imm8s << 1), 2);
            return;
        }
        switch (extract32(h, 5, 3)) {
        case 0: /* jr5 */
        case 4: /* ret5 */
            tcg_gen_mov_i32(cpu_pc, cpu_R[ra5]);
            dc->base.is_jmp = DISAS_JUMP;
            return;
        case 1: /* jral5 */
            tcg_gen_mov_i32(cpu_pc, cpu_R[ra5]);
            tcg_gen_movi_i32(cpu_R[R_LP], pc + 2);
            dc->base.is_jmp = DISAS_JUMP;
            return;
        case 5: /* add5.pc */
            tcg_gen_addi_i32(cpu_R[ra5], cpu_R[ra5], pc);
            return;
        }
        gen_reserved(dc);
        return;
    }

    switch (h >> 9) {
    case 0x40: case 0x41: /* mov55 */
        tcg_gen_mov_i32(cpu_R[rt5], cpu_R[ra5]);
        return;
    case 0x42: case 0x43: /* movi55 */
        tcg_gen_movi_i32(cpu_R[rt5], sext(h, 5));
        return;
    case 0x44:
        tcg_gen_add_i32(cpu_R[rt4], cpu_R[rt4], cpu_R[ra5]);
        return;
    case 0x45:
        tcg_gen_sub_i32(cpu_R[rt4], cpu_R[rt4], cpu_R[ra5]);
        return;
    case 0x46:
        tcg_gen_addi_i32(cpu_R[rt4], cpu_R[rt4], imm5u);
        return;
    case 0x47:
        tcg_gen_subi_i32(cpu_R[rt4], cpu_R[rt4], imm5u);
        return;
    case 0x48:
        tcg_gen_sari_i32(cpu_R[rt4], cpu_R[rt4], imm5u);
        return;
    case 0x49:
        tcg_gen_shri_i32(cpu_R[rt4], cpu_R[rt4], imm5u);
        return;
    case 0x4a:
        tcg_gen_shli_i32(cpu_R[rt3], cpu_R[ra3], imm3u);
        return;
    case 0x4b: /* zeb33 ... fexti33 */
        switch (h & 7) {
        case 0: tcg_gen_ext8u_i32(cpu_R[rt3], cpu_R[ra3]); return;
        case 1: tcg_gen_ext16u_i32(cpu_R[rt3], cpu_R[ra3]); return;
        case 2: tcg_gen_ext8s_i32(cpu_R[rt3], cpu_R[ra3]); return;
        case 3: tcg_gen_ext16s_i32(cpu_R[rt3], cpu_R[ra3]); return;
        case 4: tcg_gen_andi_i32(cpu_R[rt3], cpu_R[ra3], 1); return;
        case 5: tcg_gen_andi_i32(cpu_R[rt3], cpu_R[ra3], 0x7ff); return;
        case 6: tcg_gen_andi_i32(cpu_R[rt3], cpu_R[rt3], 1u << ra3); return;
        case 7:
            tcg_gen_andi_i32(cpu_R[rt3], cpu_R[rt3], (2u << ra3) - 1);
            return;
        }
        break;
    case 0x4c:
        tcg_gen_add_i32(cpu_R[rt3], cpu_R[ra3], cpu_R[rb3]);
        return;
    case 0x4d:
        tcg_gen_sub_i32(cpu_R[rt3], cpu_R[ra3], cpu_R[rb3]);
        return;
    case 0x4e:
        tcg_gen_addi_i32(cpu_R[rt3], cpu_R[ra3], imm3u);
        return;
    case 0x4f:
        tcg_gen_subi_i32(cpu_R[rt3], cpu_R[ra3], imm3u);
        return;
    case 0x50: gen_ldst_imm(dc, false, MO_UL, rt3, ra3, imm3u << 2, false); return;
    case 0x51: gen_ldst_imm(dc, false, MO_UL, rt3, ra3, imm3u << 2, true); return;
    case 0x52: gen_ldst_imm(dc, false, MO_UW, rt3, ra3, imm3u << 1, false); return;
    case 0x53: gen_ldst_imm(dc, false, MO_UB, rt3, ra3, imm3u, false); return;
    case 0x54: gen_ldst_imm(dc, true, MO_UL, rt3, ra3, imm3u << 2, false); return;
    case 0x55: gen_ldst_imm(dc, true, MO_UL, rt3, ra3, imm3u << 2, true); return;
    case 0x56: gen_ldst_imm(dc, true, MO_UW, rt3, ra3, imm3u << 1, false); return;
    case 0x57: gen_ldst_imm(dc, true, MO_UB, rt3, ra3, imm3u, false); return;
    case 0x58: /* addri36.sp */
        tcg_gen_addi_i32(cpu_R[rt3], cpu_R[R_SP], (h & 0x3f) << 2);
        return;
    case 0x59: /* lwi45.fe */
        gen_ldst_imm(dc, false, MO_UL, rt4, 8, (imm5u << 2) - 128, false);
        return;
    case 0x5a: /* lwi450 */
        gen_ldst_imm(dc, false, MO_UL, rt4, ra5, 0, false);
        return;
    case 0x5b: /* swi450 */
        gen_ldst_imm(dc, true, MO_UL, rt4, ra5, 0, false);
        return;
    case 0x5c: case 0x5d: case 0x5e: case 0x5f: /* lwi37 / swi37 */
        gen_ldst_imm(dc, extract32(h, 7, 1), MO_UL, rt38, R_FP,
                     (h & 0x7f) << 2, false);
        return;
    case 0x70: /* slts45 */
        tcg_gen_setcond_i32(TCG_COND_LT, cpu_R[R_TA], cpu_R[rt4], cpu_R[ra5]);
        return;
    case 0x71: /* slt45 */
        tcg_gen_setcond_i32(TCG_COND_LTU, cpu_R[R_TA], cpu_R[rt4], cpu_R[ra5]);
        return;
    case 0x72: /* sltsi45 */
        tcg_gen_setcondi_i32(TCG_COND_LT, cpu_R[R_TA], cpu_R[rt4], imm5u);
        return;
    case 0x73: /* slti45 */
        tcg_gen_setcondi_i32(TCG_COND_LTU, cpu_R[R_TA], cpu_R[rt4], imm5u);
        return;
    case 0x74: /* beqzs8 / bnezs8 */
        gen_brcondi(dc, extract32(h, 8, 1) ? TCG_COND_NE : TCG_COND_EQ,
                    cpu_R[R_TA], 0, pc + (imm8s << 1), 2);
        return;
    case 0x75: /* break16 */
        if (h & 0x1e0) {
            break;
        }
        gen_raise(dc, VEC_DEBUG, imm5u << R_ITYPE_SWID_SHIFT);
        return;
    case 0x76: case 0x77: /* addi10.sp */
        tcg_gen_addi_i32(cpu_R[R_SP], cpu_R[R_SP], sext(h, 10));
        return;
    case 0x78: case 0x79: case 0x7a: case 0x7b: /* lwi37.sp / swi37.sp */
        gen_ldst_imm(dc, extract32(h, 7, 1), MO_UL, rt38, R_SP,
                     (h & 0x7f) << 2, false);
        return;
    case 0x7d: /* movpi45 */
        tcg_gen_movi_i32(cpu_R[rt4], imm5u + 16);
        return;
    case 0x7e: /* push25 / pop25 / movd44 */
        if (!extract32(h, 8, 1)) {
            static const int re_map[4] = { 6, 8, 10, 14 };
            int re = re_map[extract32(h, 5, 2)];
            bool pop = extract32(h, 7, 1);
            /* smw.adm $r6, [$sp], re, #0xe / lmw.bim $r6, [$sp], re, #0xe */
            uint32_t lsmw = (0x1d << 25) | (6 << 20) | (R_SP << 15) |
                            (re << 10) | (0xe << 6);

            if (pop) {
                tcg_gen_addi_i32(cpu_R[R_SP], cpu_R[R_SP], imm5u << 3);
                gen_lsmw(dc, lsmw | (1 << 2));
                tcg_gen_mov_i32(cpu_pc, cpu_R[R_LP]);
                dc->base.is_jmp = DISAS_JUMP;
            } else {
                gen_lsmw(dc, lsmw | (1 << 5) | (7 << 2));
                tcg_gen_subi_i32(cpu_R[R_SP], cpu_R[R_SP], imm5u << 3);
            }
        } else {
            int rt = extract32(h, 4, 4) * 2, ra = extract32(h, 0, 4) * 2;
            TCGv lo = tcg_temp_new(), hi = tcg_temp_new();

            tcg_gen_mov_i32(lo, cpu_R[ra]);
            tcg_gen_mov_i32(hi, cpu_R[ra + 1]);
            tcg_gen_mov_i32(cpu_R[rt], lo);
            tcg_gen_mov_i32(cpu_R[rt + 1], hi);
        }
        return;
    case 0x7f: /* neg33 ... or33 */
        switch (h & 7) {
        case 2: tcg_gen_neg_i32(cpu_R[rt3], cpu_R[ra3]); return;
        case 3: tcg_gen_not_i32(cpu_R[rt3], cpu_R[ra3]); return;
        case 4: tcg_gen_mul_i32(cpu_R[rt3], cpu_R[rt3], cpu_R[ra3]); return;
        case 5: tcg_gen_xor_i32(cpu_R[rt3], cpu_R[rt3], cpu_R[ra3]); return;
        case 6: tcg_gen_and_i32(cpu_R[rt3], cpu_R[rt3], cpu_R[ra3]); return;
        case 7: tcg_gen_or_i32(cpu_R[rt3], cpu_R[rt3], cpu_R[ra3]); return;
        }
        break;
    }
    gen_reserved(dc);
}

/* Translator glue */

static void nds32_tr_init_disas_context(DisasContextBase *dcbase,
                                        CPUState *cs)
{
    DisasContext *dc = container_of(dcbase, DisasContext, base);
    uint32_t flags = dc->base.tb->flags;

    dc->tbflags = flags;
    dc->user = FIELD_EX32(flags, TBFLAGS, USER);
    dc->mem_idx = (dc->user ? MMU_IDX_USER : 0) |
                  (FIELD_EX32(flags, TBFLAGS, DT) ? MMU_IDX_XLATE : 0);
    dc->be = FIELD_EX32(flags, TBFLAGS, BE) ? MO_BE : MO_LE;
    dc->align = FIELD_EX32(flags, TBFLAGS, UNA) ? MO_UNALN : MO_ALIGN;
}

static void nds32_tr_tb_start(DisasContextBase *db, CPUState *cs)
{
}

static void nds32_tr_insn_start(DisasContextBase *dcbase, CPUState *cs)
{
    tcg_gen_insn_start(dcbase->pc_next, 0, 0);
}

static void nds32_tr_translate_insn(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *dc = container_of(dcbase, DisasContext, base);
    CPUNDS32State *env = cpu_env(cs);
    uint32_t pc = dc->base.pc_next;
    uint32_t insn;

    dc->pc = pc;
    insn = translator_lduw_end(env, &dc->base, pc, MO_BE);
    if (insn & 0x8000) {
        dc->base.pc_next = pc + 2;
        decode16(dc, insn);
    } else {
        insn = (insn << 16) | translator_lduw_end(env, &dc->base, pc + 2,
                                                  MO_BE);
        dc->base.pc_next = pc + 4;
        decode32(dc, insn);
    }

    /* A 32-bit insn must not straddle the end of this TB's last page. */
    if (dc->base.is_jmp == DISAS_NEXT &&
        (dc->base.pc_next & ~TARGET_PAGE_MASK) > TARGET_PAGE_SIZE - 4) {
        dc->base.is_jmp = DISAS_TOO_MANY;
    }
}

static void nds32_tr_tb_stop(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *dc = container_of(dcbase, DisasContext, base);

    switch (dc->base.is_jmp) {
    case DISAS_NEXT:
    case DISAS_TOO_MANY:
        gen_goto_tb(dc, 0, dc->base.pc_next);
        break;
    case DISAS_JUMP:
        tcg_gen_lookup_and_goto_ptr();
        break;
    case DISAS_EXIT:
        tcg_gen_movi_i32(cpu_pc, dc->base.pc_next);
        tcg_gen_exit_tb(NULL, 0);
        break;
    case DISAS_EXIT_JUMP:
        tcg_gen_exit_tb(NULL, 0);
        break;
    case DISAS_NORETURN:
        break;
    default:
        g_assert_not_reached();
    }
}

static const TranslatorOps nds32_tr_ops = {
    .init_disas_context = nds32_tr_init_disas_context,
    .tb_start           = nds32_tr_tb_start,
    .insn_start         = nds32_tr_insn_start,
    .translate_insn     = nds32_tr_translate_insn,
    .tb_stop            = nds32_tr_tb_stop,
};

void nds32_translate_code(CPUState *cs, TranslationBlock *tb,
                          int *max_insns, vaddr pc, void *host_pc)
{
    DisasContext dc;

    translator_loop(cs, tb, max_insns, pc, host_pc, &nds32_tr_ops, &dc.base,
                    TCG_TYPE_VA);
}

void nds32_cpu_dump_state(CPUState *cs, FILE *f, int flags)
{
    CPUNDS32State *env = cpu_env(cs);

    qemu_fprintf(f, "PC=%08x PSW=%08x IPC=%08x IPSW=%08x ITYPE=%08x "
                 "EVA=%08x\n", env->pc, env->sr[SR_PSW], env->sr[SR_IPC],
                 env->sr[SR_IPSW], env->sr[SR_ITYPE], env->sr[SR_EVA]);
    for (int i = 0; i < 32; i++) {
        qemu_fprintf(f, "%4s=%08x%c", regnames[i], env->gpr[i],
                     (i % 8) == 7 ? '\n' : ' ');
    }
}

void nds32_tcg_init(void)
{
    static const char * const dnames[4] = {
        "d0.lo", "d0.hi", "d1.lo", "d1.hi"
    };

    for (int i = 0; i < 32; i++) {
        cpu_R[i] = tcg_global_mem_new_i32(tcg_env,
                                          offsetof(CPUNDS32State, gpr[i]),
                                          regnames[i]);
    }
    for (int i = 0; i < 4; i++) {
        cpu_D[i] = tcg_global_mem_new_i32(tcg_env,
                                          offsetof(CPUNDS32State, d[i]),
                                          dnames[i]);
    }
    cpu_pc = tcg_global_mem_new_i32(tcg_env, offsetof(CPUNDS32State, pc),
                                    "pc");
    cpu_lock_addr = tcg_global_mem_new_i32(tcg_env,
                                           offsetof(CPUNDS32State, lock_addr),
                                           "lock_addr");
    cpu_lock_val = tcg_global_mem_new_i32(tcg_env,
                                          offsetof(CPUNDS32State, lock_val),
                                          "lock_val");
}
