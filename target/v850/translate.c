/*
 * NEC V850E1 translation
 *
 * Instruction encodings from the V850E1 Architecture User's Manual
 * (U14559EJ), chapter 5.
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/helper-proto.h"
#include "exec/translator.h"
#include "exec/translation-block.h"
#include "tcg/tcg-op.h"
#include "qemu/qemu-print.h"
#include "exec/helper-gen.h"
#include "exec/log.h"

#define HELPER_H "helper.h"
#include "exec/helper-info.c.inc"
#undef  HELPER_H

/* pc was set; leave through the main loop so interrupts are looked at */
#define DISAS_UPDATE    DISAS_TARGET_0
/* pc was set to a computed address */
#define DISAS_JUMP      DISAS_TARGET_1

static TCGv_i32 cpu_gpr[NUM_GPR];
static TCGv_i32 cpu_pc;
static TCGv_i32 cpu_zf, cpu_sf, cpu_ovf, cpu_cyf, cpu_sat;

typedef struct DisasContext {
    DisasContextBase base;
    uint32_t pc;        /* this insn */
    uint32_t next;      /* the following insn */
} DisasContext;

static const char * const regnames[NUM_GPR] = {
    "r0", "r1", "r2", "sp", "gp", "r5", "r6", "r7",
    "r8", "r9", "r10", "r11", "r12", "r13", "r14", "r15",
    "r16", "r17", "r18", "r19", "r20", "r21", "r22", "r23",
    "r24", "r25", "r26", "r27", "r28", "r29", "ep", "lp",
};

void v850_tcg_init(void)
{
    for (int i = 1; i < NUM_GPR; i++) {
        cpu_gpr[i] = tcg_global_mem_new_i32(tcg_env,
                                            offsetof(CPUV850State, gpr[i]),
                                            regnames[i]);
    }
    cpu_pc = tcg_global_mem_new_i32(tcg_env, offsetof(CPUV850State, pc),
                                    "pc");
    cpu_zf = tcg_global_mem_new_i32(tcg_env, offsetof(CPUV850State, zf),
                                    "zf");
    cpu_sf = tcg_global_mem_new_i32(tcg_env, offsetof(CPUV850State, sf),
                                    "sf");
    cpu_ovf = tcg_global_mem_new_i32(tcg_env, offsetof(CPUV850State, ovf),
                                     "ovf");
    cpu_cyf = tcg_global_mem_new_i32(tcg_env, offsetof(CPUV850State, cyf),
                                     "cyf");
    cpu_sat = tcg_global_mem_new_i32(tcg_env, offsetof(CPUV850State, sat),
                                     "sat");
}

static TCGv_i32 reg(int r)
{
    return r ? cpu_gpr[r] : tcg_constant_i32(0);
}

static void set_reg(int r, TCGv_i32 v)
{
    if (r) {
        tcg_gen_mov_i32(cpu_gpr[r], v);
    }
}

/* A destination that may be r0: writes to it are dropped. */
static TCGv_i32 dest(int r)
{
    return r ? cpu_gpr[r] : tcg_temp_new_i32();
}

static void gen_exception(DisasContext *dc, int excp, uint32_t pc)
{
    tcg_gen_movi_i32(cpu_pc, pc);
    gen_helper_raise_exception(tcg_env, tcg_constant_i32(excp));
    dc->base.is_jmp = DISAS_NORETURN;
}

static void gen_illegal(DisasContext *dc)
{
    qemu_log_mask(LOG_GUEST_ERROR, "v850: illegal insn at %08x\n", dc->pc);
    gen_exception(dc, EXCP_ILGOP, dc->next);
}

static void gen_goto_tb(DisasContext *dc, int n, uint32_t dest_pc)
{
    if (translator_use_goto_tb(&dc->base, dest_pc)) {
        tcg_gen_goto_tb(n);
        tcg_gen_movi_i32(cpu_pc, dest_pc);
        tcg_gen_exit_tb(dc->base.tb, n);
    } else {
        tcg_gen_movi_i32(cpu_pc, dest_pc);
        tcg_gen_lookup_and_goto_ptr();
    }
}

/* res = a + b with all flags */
static void gen_add_cc(TCGv_i32 res, TCGv_i32 a, TCGv_i32 b)
{
    TCGv_i32 t = tcg_temp_new_i32();
    TCGv_i32 r = tcg_temp_new_i32();

    tcg_gen_add_i32(r, a, b);
    tcg_gen_setcond_i32(TCG_COND_LTU, cpu_cyf, r, a);
    tcg_gen_xor_i32(cpu_ovf, r, a);
    tcg_gen_xor_i32(t, a, b);
    tcg_gen_andc_i32(cpu_ovf, cpu_ovf, t);
    tcg_gen_mov_i32(cpu_zf, r);
    tcg_gen_mov_i32(cpu_sf, r);
    tcg_gen_mov_i32(res, r);
}

/* res = a - b with all flags */
static void gen_sub_cc(TCGv_i32 res, TCGv_i32 a, TCGv_i32 b)
{
    TCGv_i32 t = tcg_temp_new_i32();
    TCGv_i32 r = tcg_temp_new_i32();

    tcg_gen_sub_i32(r, a, b);
    tcg_gen_setcond_i32(TCG_COND_LTU, cpu_cyf, a, b);
    tcg_gen_xor_i32(cpu_ovf, a, b);
    tcg_gen_xor_i32(t, a, r);
    tcg_gen_and_i32(cpu_ovf, cpu_ovf, t);
    tcg_gen_mov_i32(cpu_zf, r);
    tcg_gen_mov_i32(cpu_sf, r);
    tcg_gen_mov_i32(res, r);
}

/* S and Z from res, OV cleared */
static void gen_logic_cc(TCGv_i32 res)
{
    tcg_gen_mov_i32(cpu_zf, res);
    tcg_gen_mov_i32(cpu_sf, res);
    tcg_gen_movi_i32(cpu_ovf, 0);
}

/* 0 or 1 in res for condition code cc */
static void gen_cond(TCGv_i32 res, int cc)
{
    TCGv_i32 t = tcg_temp_new_i32();

    switch (cc & 7) {
    case 0:     /* V */
        tcg_gen_shri_i32(res, cpu_ovf, 31);
        break;
    case 1:     /* C */
        tcg_gen_mov_i32(res, cpu_cyf);
        break;
    case 2:     /* Z */
        tcg_gen_setcondi_i32(TCG_COND_EQ, res, cpu_zf, 0);
        break;
    case 3:     /* NH: CY or Z */
        tcg_gen_setcondi_i32(TCG_COND_EQ, t, cpu_zf, 0);
        tcg_gen_or_i32(res, t, cpu_cyf);
        break;
    case 4:     /* N */
        tcg_gen_shri_i32(res, cpu_sf, 31);
        break;
    case 5:     /* T */
        tcg_gen_movi_i32(res, 1);
        break;
    case 6:     /* LT: S xor OV */
        tcg_gen_xor_i32(t, cpu_sf, cpu_ovf);
        tcg_gen_shri_i32(res, t, 31);
        break;
    case 7:     /* LE: (S xor OV) or Z */
        tcg_gen_xor_i32(t, cpu_sf, cpu_ovf);
        tcg_gen_shri_i32(t, t, 31);
        tcg_gen_setcondi_i32(TCG_COND_EQ, res, cpu_zf, 0);
        tcg_gen_or_i32(res, res, t);
        break;
    }
    if (cc == 0xd) {            /* SA: the one that is not an inverse */
        tcg_gen_mov_i32(res, cpu_sat);
    } else if (cc & 8) {
        tcg_gen_xori_i32(res, res, 1);
    }
}

static void gen_ld(TCGv_i32 dst, TCGv_i32 base, int32_t disp, MemOp mop)
{
    TCGv_i32 addr = tcg_temp_new_i32();

    tcg_gen_addi_i32(addr, base, disp);
    tcg_gen_qemu_ld_i32(dst, addr, 0, mop | MO_LE);
}

static void gen_st(TCGv_i32 val, TCGv_i32 base, int32_t disp, MemOp mop)
{
    TCGv_i32 addr = tcg_temp_new_i32();

    tcg_gen_addi_i32(addr, base, disp);
    tcg_gen_qemu_st_i32(val, addr, 0, mop | MO_LE);
}

static void gen_load_reg(int r2, TCGv_i32 base, int32_t disp, MemOp mop)
{
    TCGv_i32 t = tcg_temp_new_i32();

    gen_ld(t, base, disp, mop);
    set_reg(r2, t);
}

/* SET1 (0), NOT1 (1), CLR1 (2), TST1 (3); Z is the old bit inverted */
static void gen_bitop(int op, TCGv_i32 addr, TCGv_i32 bit)
{
    TCGv_i32 v = tcg_temp_new_i32();
    TCGv_i32 m = tcg_temp_new_i32();

    tcg_gen_shl_i32(m, tcg_constant_i32(1), bit);
    tcg_gen_qemu_ld_i32(v, addr, 0, MO_UB);
    tcg_gen_and_i32(cpu_zf, v, m);
    switch (op) {
    case 0:
        tcg_gen_or_i32(v, v, m);
        break;
    case 1:
        tcg_gen_xor_i32(v, v, m);
        break;
    case 2:
        tcg_gen_andc_i32(v, v, m);
        break;
    default:
        return;
    }
    tcg_gen_qemu_st_i32(v, addr, 0, MO_UB);
}

static void gen_satop(int r2, TCGv_i32 a, TCGv_i32 b, bool sub)
{
    TCGv_i32 t = tcg_temp_new_i32();

    if (sub) {
        gen_helper_satsub(t, tcg_env, a, b);
    } else {
        gen_helper_satadd(t, tcg_env, a, b);
    }
    set_reg(r2, t);
}

static void gen_mulh(int r2, TCGv_i32 a, TCGv_i32 b)
{
    TCGv_i32 x = tcg_temp_new_i32();
    TCGv_i32 y = tcg_temp_new_i32();

    tcg_gen_ext16s_i32(x, a);
    tcg_gen_ext16s_i32(y, b);
    tcg_gen_mul_i32(x, x, y);
    set_reg(r2, x);
}

/* Formats I and II plus the 16-bit loads and stores */
static void decode16(DisasContext *dc, uint32_t hw)
{
    int r1 = hw & 0x1f, r2 = hw >> 11, op = (hw >> 5) & 0x3f;
    int32_t imm5 = sextract32(hw, 0, 5);
    TCGv_i32 t;

    if (op >= 0x18) {
        switch ((hw >> 7) & 0xf) {
        case 0x6:       /* SLD.B */
            gen_load_reg(r2, cpu_gpr[R_EP], hw & 0x7f, MO_SB);
            return;
        case 0x7:       /* SST.B */
            gen_st(reg(r2), cpu_gpr[R_EP], hw & 0x7f, MO_UB);
            return;
        case 0x8:       /* SLD.H */
            gen_load_reg(r2, cpu_gpr[R_EP], (hw & 0x7f) << 1, MO_SW);
            return;
        case 0x9:       /* SST.H */
            gen_st(reg(r2), cpu_gpr[R_EP], (hw & 0x7f) << 1, MO_UW);
            return;
        case 0xa:       /* SLD.W / SST.W */
            if (hw & 1) {
                gen_st(reg(r2), cpu_gpr[R_EP], (hw & 0x7e) << 1, MO_UL);
            } else {
                gen_load_reg(r2, cpu_gpr[R_EP], (hw & 0x7e) << 1, MO_UL);
            }
            return;
        case 0xb: {     /* Bcond disp9 */
            int32_t disp = sextract32(((hw >> 11) << 4) | ((hw >> 4) & 7) << 1,
                                      0, 9);
            int cc = hw & 0xf;

            if (cc == 5) {
                gen_goto_tb(dc, 0, dc->pc + disp);
            } else {
                TCGLabel *l = gen_new_label();

                t = tcg_temp_new_i32();
                gen_cond(t, cc);
                tcg_gen_brcondi_i32(TCG_COND_NE, t, 0, l);
                gen_goto_tb(dc, 0, dc->next);
                gen_set_label(l);
                gen_goto_tb(dc, 1, dc->pc + disp);
            }
            dc->base.is_jmp = DISAS_NORETURN;
            return;
        }
        }
        g_assert_not_reached();
    }

    switch (op) {
    case 0x00:          /* MOV reg1, reg2 / NOP */
        set_reg(r2, reg(r1));
        break;
    case 0x01:          /* NOT */
        t = dest(r2);
        tcg_gen_not_i32(t, reg(r1));
        gen_logic_cc(t);
        break;
    case 0x02:
        if (r2 == 0) {
            if (r1 == 0) {
                gen_illegal(dc);
                break;
            }
            /* SWITCH reg1 */
            t = tcg_temp_new_i32();
            tcg_gen_shli_i32(t, cpu_gpr[r1], 1);
            gen_ld(t, t, dc->next, MO_SW);
            tcg_gen_shli_i32(t, t, 1);
            tcg_gen_addi_i32(cpu_pc, t, dc->next);
            dc->base.is_jmp = DISAS_JUMP;
        } else if (r1 == 0) {
            if (r2 == 31) {     /* DBTRAP */
                gen_exception(dc, EXCP_DBTRAP, dc->next);
            } else {
                gen_illegal(dc);
            }
        } else {        /* DIVH reg1, reg2 */
            gen_helper_div(tcg_env, tcg_constant_i32(4), tcg_constant_i32(r1),
                           tcg_constant_i32(r2), tcg_constant_i32(0));
        }
        break;
    case 0x03:
        if (r2 == 0) {  /* JMP [reg1] */
            tcg_gen_andi_i32(cpu_pc, reg(r1), ~1);
            dc->base.is_jmp = DISAS_JUMP;
        } else if (hw & 0x10) {         /* SLD.HU */
            gen_load_reg(r2, cpu_gpr[R_EP], (hw & 0xf) << 1, MO_UW);
        } else {                        /* SLD.BU */
            gen_load_reg(r2, cpu_gpr[R_EP], hw & 0xf, MO_UB);
        }
        break;
    case 0x04:
        if (r2 == 0) {  /* ZXB */
            tcg_gen_ext8u_i32(dest(r1), reg(r1));
        } else {        /* SATSUBR */
            gen_satop(r2, reg(r1), reg(r2), true);
        }
        break;
    case 0x05:
        if (r2 == 0) {  /* SXB */
            tcg_gen_ext8s_i32(dest(r1), reg(r1));
        } else {        /* SATSUB */
            gen_satop(r2, reg(r2), reg(r1), true);
        }
        break;
    case 0x06:
        if (r2 == 0) {  /* ZXH */
            tcg_gen_ext16u_i32(dest(r1), reg(r1));
        } else {        /* SATADD */
            gen_satop(r2, reg(r2), reg(r1), false);
        }
        break;
    case 0x07:
        if (r2 == 0) {  /* SXH */
            tcg_gen_ext16s_i32(dest(r1), reg(r1));
        } else {        /* MULH */
            gen_mulh(r2, reg(r2), reg(r1));
        }
        break;
    case 0x08:          /* OR */
        t = dest(r2);
        tcg_gen_or_i32(t, reg(r2), reg(r1));
        gen_logic_cc(t);
        break;
    case 0x09:          /* XOR */
        t = dest(r2);
        tcg_gen_xor_i32(t, reg(r2), reg(r1));
        gen_logic_cc(t);
        break;
    case 0x0a:          /* AND */
        t = dest(r2);
        tcg_gen_and_i32(t, reg(r2), reg(r1));
        gen_logic_cc(t);
        break;
    case 0x0b:          /* TST */
        t = tcg_temp_new_i32();
        tcg_gen_and_i32(t, reg(r2), reg(r1));
        gen_logic_cc(t);
        break;
    case 0x0c:          /* SUBR */
        gen_sub_cc(dest(r2), reg(r1), reg(r2));
        break;
    case 0x0d:          /* SUB */
        gen_sub_cc(dest(r2), reg(r2), reg(r1));
        break;
    case 0x0e:          /* ADD */
        gen_add_cc(dest(r2), reg(r2), reg(r1));
        break;
    case 0x0f:          /* CMP */
        gen_sub_cc(tcg_temp_new_i32(), reg(r2), reg(r1));
        break;
    case 0x10:
    case 0x11:
        if (r2 == 0) {  /* CALLT imm6 */
            gen_helper_callt(cpu_pc, tcg_env, tcg_constant_i32(hw & 0x3f),
                             tcg_constant_i32(dc->next));
            dc->base.is_jmp = DISAS_JUMP;
        } else if (op == 0x10) {        /* MOV imm5 */
            tcg_gen_movi_i32(cpu_gpr[r2], imm5);
        } else {                        /* SATADD imm5 */
            gen_satop(r2, reg(r2), tcg_constant_i32(imm5), false);
        }
        break;
    case 0x12:          /* ADD imm5 */
        gen_add_cc(dest(r2), reg(r2), tcg_constant_i32(imm5));
        break;
    case 0x13:          /* CMP imm5 */
        gen_sub_cc(tcg_temp_new_i32(), reg(r2), tcg_constant_i32(imm5));
        break;
    case 0x14:          /* SHR imm5 */
    case 0x15:          /* SAR imm5 */
    case 0x16:          /* SHL imm5 */
        t = dest(r2);
        if (op == 0x14) {
            gen_helper_shr(t, tcg_env, reg(r2), tcg_constant_i32(r1));
        } else if (op == 0x15) {
            gen_helper_sar(t, tcg_env, reg(r2), tcg_constant_i32(r1));
        } else {
            gen_helper_shl(t, tcg_env, reg(r2), tcg_constant_i32(r1));
        }
        break;
    case 0x17:          /* MULH imm5 */
        if (r2 == 0) {
            gen_illegal(dc);
        } else {
            gen_mulh(r2, reg(r2), tcg_constant_i32(imm5));
        }
        break;
    default:
        g_assert_not_reached();
    }
}

/* Format IX/X/XI/XII: opcode 111111, bit 16 clear */
static void decode_ext(DisasContext *dc, uint32_t hw1, uint32_t hw2)
{
    int r1 = hw1 & 0x1f, r2 = hw1 >> 11, r3 = hw2 >> 11;
    uint32_t sub = hw2 & 0x7ff;
    TCGv_i32 t;

    switch (sub & 0x7c3) {
    case 0x240:         /* MUL imm9, reg2, reg3 */
    case 0x242: {       /* MULU imm9, reg2, reg3 */
        uint32_t imm9 = r1 | ((hw2 >> 2) & 0xf) << 5;
        TCGv_i32 lo = tcg_temp_new_i32(), hi = tcg_temp_new_i32();

        if (sub & 2) {
            tcg_gen_mulu2_i32(lo, hi, reg(r2), tcg_constant_i32(imm9));
        } else {
            tcg_gen_muls2_i32(lo, hi, reg(r2),
                              tcg_constant_i32(sextract32(imm9, 0, 9)));
        }
        set_reg(r2, lo);
        set_reg(r3, hi);
        return;
    }
    }

    switch (sub & 0x7e1) {
    case 0x300:         /* CMOV cccc, imm5, reg2, reg3 */
    case 0x320:         /* CMOV cccc, reg1, reg2, reg3 */
        t = tcg_temp_new_i32();
        gen_cond(t, (hw2 >> 1) & 0xf);
        tcg_gen_movcond_i32(TCG_COND_NE, dest(r3), t, tcg_constant_i32(0),
                            sub & 0x20 ? reg(r1)
                                       : tcg_constant_i32(sextract32(r1, 0, 5)),
                            reg(r2));
        return;
    }

    switch (sub) {
    case 0x000:         /* SETF cccc, reg2 */
        gen_cond(dest(r2), r1 & 0xf);
        break;
    case 0x200:         /* SASF cccc, reg2 */
        if (r2) {
            t = tcg_temp_new_i32();
            gen_cond(t, r1 & 0xf);
            tcg_gen_shli_i32(cpu_gpr[r2], cpu_gpr[r2], 1);
            tcg_gen_or_i32(cpu_gpr[r2], cpu_gpr[r2], t);
        }
        break;
    case 0x020:         /* LDSR reg2 (in the reg1 field), regID */
        gen_helper_ldsr(tcg_env, tcg_constant_i32(r2), reg(r1));
        tcg_gen_movi_i32(cpu_pc, dc->next);
        dc->base.is_jmp = DISAS_UPDATE;
        break;
    case 0x040:         /* STSR regID, reg2 */
        gen_helper_stsr(dest(r2), tcg_env, tcg_constant_i32(r1));
        break;
    case 0x080:         /* SHR reg1, reg2 */
        gen_helper_shr(dest(r2), tcg_env, reg(r2), reg(r1));
        break;
    case 0x0a0:         /* SAR reg1, reg2 */
        gen_helper_sar(dest(r2), tcg_env, reg(r2), reg(r1));
        break;
    case 0x0c0:         /* SHL reg1, reg2 */
        gen_helper_shl(dest(r2), tcg_env, reg(r2), reg(r1));
        break;
    case 0x0e0:         /* SET1 reg2, [reg1] */
    case 0x0e2:         /* NOT1 */
    case 0x0e4:         /* CLR1 */
    case 0x0e6:         /* TST1 */
        t = tcg_temp_new_i32();
        tcg_gen_andi_i32(t, reg(r2), 7);
        gen_bitop((sub >> 1) & 3, reg(r1), t);
        break;
    case 0x100:         /* TRAP vector */
        tcg_gen_movi_i32(cpu_pc, dc->next);
        gen_helper_trap(tcg_env, tcg_constant_i32(r1));
        dc->base.is_jmp = DISAS_UPDATE;
        break;
    case 0x120:         /* HALT */
        tcg_gen_movi_i32(cpu_pc, dc->next);
        gen_helper_halt(tcg_env);
        dc->base.is_jmp = DISAS_NORETURN;
        break;
    case 0x140:         /* RETI */
    case 0x144:         /* CTRET */
    case 0x146:         /* DBRET */
        gen_helper_ret_from(tcg_env, tcg_constant_i32((sub >> 1) & 3));
        dc->base.is_jmp = DISAS_UPDATE;
        break;
    case 0x160:         /* DI / EI */
        t = tcg_temp_new_i32();
        tcg_gen_ld_i32(t, tcg_env, offsetof(CPUV850State, psw));
        if (hw1 & 0x8000) {
            tcg_gen_andi_i32(t, t, ~PSW_ID);
        } else {
            tcg_gen_ori_i32(t, t, PSW_ID);
        }
        tcg_gen_st_i32(t, tcg_env, offsetof(CPUV850State, psw));
        tcg_gen_movi_i32(cpu_pc, dc->next);
        dc->base.is_jmp = DISAS_UPDATE;
        break;
    case 0x220:         /* MUL reg1, reg2, reg3 */
    case 0x222: {       /* MULU */
        TCGv_i32 lo = tcg_temp_new_i32(), hi = tcg_temp_new_i32();

        if (sub & 2) {
            tcg_gen_mulu2_i32(lo, hi, reg(r2), reg(r1));
        } else {
            tcg_gen_muls2_i32(lo, hi, reg(r2), reg(r1));
        }
        set_reg(r2, lo);
        set_reg(r3, hi);
        break;
    }
    case 0x280:         /* DIVH reg1, reg2, reg3 */
    case 0x282:         /* DIVHU */
    case 0x2c0:         /* DIV */
    case 0x2c2: {       /* DIVU */
        static const uint8_t kind[] = { 2, 3, 0, 1 };

        gen_helper_div(tcg_env, tcg_constant_i32(kind[((sub >> 5) & 2) |
                                                      ((sub >> 1) & 1)]),
                       tcg_constant_i32(r1), tcg_constant_i32(r2),
                       tcg_constant_i32(r3));
        break;
    }
    case 0x340:         /* BSW reg2, reg3 */
    case 0x342:         /* BSH */
    case 0x344:         /* HSW */
        gen_helper_swap(dest(r3), tcg_env, reg(r2),
                        tcg_constant_i32((sub >> 1) & 3));
        break;
    default:
        gen_illegal(dc);
        break;
    }
}

static uint32_t fetch16(DisasContext *dc, CPUState *cs, uint32_t addr)
{
    return translator_lduw(cpu_env(cs), &dc->base, addr);
}

static void decode32(DisasContext *dc, CPUState *cs, uint32_t hw1,
                     uint32_t hw2)
{
    int r1 = hw1 & 0x1f, r2 = hw1 >> 11, op = (hw1 >> 5) & 0x3f;
    int32_t imm16 = (int16_t)hw2;
    TCGv_i32 t;

    switch (op) {
    case 0x30:          /* ADDI */
        gen_add_cc(dest(r2), reg(r1), tcg_constant_i32(imm16));
        break;
    case 0x31:
        if (r2 == 0) {  /* MOV imm32, reg1 */
            uint32_t hw3 = fetch16(dc, cs, dc->pc + 4);

            dc->next = dc->pc + 6;
            if (r1) {
                tcg_gen_movi_i32(cpu_gpr[r1], hw2 | hw3 << 16);
            }
        } else {        /* MOVEA */
            tcg_gen_addi_i32(cpu_gpr[r2], reg(r1), imm16);
        }
        break;
    case 0x32:
    case 0x33:
        if (r2 == 0) {  /* DISPOSE imm5, list12 [, [reg1]] */
            uint32_t list = (hw2 & 0xffe0) << 16 | (hw1 & 1);

            gen_helper_dispose(tcg_env, tcg_constant_i32(list),
                               tcg_constant_i32((hw1 >> 1) & 0x1f));
            if (hw2 & 0x1f) {
                tcg_gen_andi_i32(cpu_pc, cpu_gpr[hw2 & 0x1f], ~1);
                dc->base.is_jmp = DISAS_JUMP;
            }
        } else if (op == 0x32) {        /* MOVHI */
            tcg_gen_addi_i32(cpu_gpr[r2], reg(r1), hw2 << 16);
        } else {                        /* SATSUBI */
            gen_satop(r2, reg(r1), tcg_constant_i32(imm16), true);
        }
        break;
    case 0x34:          /* ORI */
        t = dest(r2);
        tcg_gen_ori_i32(t, reg(r1), hw2);
        gen_logic_cc(t);
        break;
    case 0x35:          /* XORI */
        t = dest(r2);
        tcg_gen_xori_i32(t, reg(r1), hw2);
        gen_logic_cc(t);
        break;
    case 0x36:          /* ANDI */
        t = dest(r2);
        tcg_gen_andi_i32(t, reg(r1), hw2);
        gen_logic_cc(t);
        break;
    case 0x37:          /* MULHI */
        if (r2 == 0) {
            gen_illegal(dc);
        } else {
            gen_mulh(r2, reg(r1), tcg_constant_i32(imm16));
        }
        break;
    case 0x38:          /* LD.B */
        gen_load_reg(r2, reg(r1), imm16, MO_SB);
        break;
    case 0x39:          /* LD.H / LD.W */
        gen_load_reg(r2, reg(r1), imm16 & ~1, hw2 & 1 ? MO_UL : MO_SW);
        break;
    case 0x3a:          /* ST.B */
        gen_st(reg(r2), reg(r1), imm16, MO_UB);
        break;
    case 0x3b:          /* ST.H / ST.W */
        gen_st(reg(r2), reg(r1), imm16 & ~1, hw2 & 1 ? MO_UL : MO_UW);
        break;
    case 0x3c:
    case 0x3d:
        if (!(hw2 & 1)) {       /* JR / JARL disp22, reg2 */
            int32_t disp = sextract32((hw1 & 0x3f) << 16 | (hw2 & 0xfffe),
                                      0, 22);

            if (r2) {
                tcg_gen_movi_i32(cpu_gpr[r2], dc->next);
            }
            gen_goto_tb(dc, 0, dc->pc + disp);
            dc->base.is_jmp = DISAS_NORETURN;
        } else if (r2) {        /* LD.BU */
            gen_load_reg(r2, reg(r1),
                         (int16_t)((hw2 & 0xfffe) | ((hw1 >> 5) & 1)), MO_UB);
        } else {                /* PREPARE list12, imm5 [, sp/imm] */
            uint32_t list = (hw2 & 0xffe0) << 16 | (hw1 & 1);
            uint32_t ff = (hw2 >> 3) & 3, val = 0;

            if ((hw2 & 0x1f) == 1) {
                ff = 4;
            } else if ((hw2 & 7) != 3) {
                gen_illegal(dc);
                break;
            }
            if (ff == 1 || ff == 2) {
                val = fetch16(dc, cs, dc->pc + 4);
                val = ff == 1 ? (int16_t)val : val << 16;
                dc->next = dc->pc + 6;
            } else if (ff == 3) {
                val = fetch16(dc, cs, dc->pc + 4);
                val |= fetch16(dc, cs, dc->pc + 6) << 16;
                dc->next = dc->pc + 8;
            }
            gen_helper_prepare(tcg_env, tcg_constant_i32(list),
                               tcg_constant_i32((hw1 >> 1) & 0x1f));
            if (ff == 0) {
                tcg_gen_mov_i32(cpu_gpr[R_EP], cpu_gpr[R_SP]);
            } else if (ff != 4) {
                tcg_gen_movi_i32(cpu_gpr[R_EP], val);
            }
        }
        break;
    case 0x3e: {        /* SET1/NOT1/CLR1/TST1 bit#3, disp16[reg1] */
        TCGv_i32 addr = tcg_temp_new_i32();

        tcg_gen_addi_i32(addr, reg(r1), imm16);
        gen_bitop(hw1 >> 14, addr, tcg_constant_i32((hw1 >> 11) & 7));
        break;
    }
    case 0x3f:
        if (hw2 & 1) {  /* LD.HU */
            if (r2 == 0) {
                gen_illegal(dc);
            } else {
                gen_load_reg(r2, reg(r1), imm16 & ~1, MO_UW);
            }
        } else {
            decode_ext(dc, hw1, hw2);
        }
        break;
    default:
        g_assert_not_reached();
    }
}

static void v850_tr_init_disas_context(DisasContextBase *dcbase, CPUState *cs)
{
}

static void v850_tr_tb_start(DisasContextBase *db, CPUState *cs)
{
}

static void v850_tr_insn_start(DisasContextBase *dcbase, CPUState *cs)
{
    tcg_gen_insn_start(dcbase->pc_next, 0, 0);
}

static void v850_tr_translate_insn(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *dc = container_of(dcbase, DisasContext, base);
    uint32_t hw1, hw2;

    dc->pc = dc->base.pc_next;
    hw1 = fetch16(dc, cs, dc->pc);
    if (((hw1 >> 5) & 0x3f) < 0x30) {
        dc->next = dc->pc + 2;
        decode16(dc, hw1);
    } else {
        hw2 = fetch16(dc, cs, dc->pc + 2);
        dc->next = dc->pc + 4;
        decode32(dc, cs, hw1, hw2);
    }
    dc->base.pc_next = dc->next;

    /* stop at a page boundary so a TB never spans two pages */
    if (dc->base.is_jmp == DISAS_NEXT &&
        ((dc->base.pc_next ^ dc->base.pc_first) & TARGET_PAGE_MASK ||
         (dc->base.pc_next & ~TARGET_PAGE_MASK) > TARGET_PAGE_SIZE - 8)) {
        dc->base.is_jmp = DISAS_TOO_MANY;
    }
}

static void v850_tr_tb_stop(DisasContextBase *dcbase, CPUState *cs)
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
    case DISAS_UPDATE:
        tcg_gen_exit_tb(NULL, 0);
        break;
    case DISAS_NORETURN:
        break;
    default:
        g_assert_not_reached();
    }
}

static const TranslatorOps v850_tr_ops = {
    .init_disas_context = v850_tr_init_disas_context,
    .tb_start           = v850_tr_tb_start,
    .insn_start         = v850_tr_insn_start,
    .translate_insn     = v850_tr_translate_insn,
    .tb_stop            = v850_tr_tb_stop,
};

void v850_translate_code(CPUState *cs, TranslationBlock *tb,
                         int *max_insns, vaddr pc, void *host_pc)
{
    DisasContext dc;

    translator_loop(cs, tb, max_insns, pc, host_pc, &v850_tr_ops, &dc.base,
                    TCG_TYPE_I32);
}

void v850_cpu_dump_state(CPUState *cs, FILE *f, int flags)
{
    CPUV850State *env = cpu_env(cs);

    qemu_fprintf(f, "pc=%08x psw=%08x eipc=%08x eipsw=%08x ecr=%08x\n",
                 env->pc, v850_psw_read(env), env->sr[SR_EIPC],
                 env->sr[SR_EIPSW], env->sr[SR_ECR]);
    for (int i = 0; i < NUM_GPR; i++) {
        qemu_fprintf(f, "%-3s=%08x%c", regnames[i], env->gpr[i],
                     (i & 7) == 7 ? '\n' : ' ');
    }
}
