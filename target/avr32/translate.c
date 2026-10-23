/*
 * QEMU AVR CPU
 *
 * Copyright (c) 2022-2023 Florian Göhler, Johannes Willbold
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
#include "qemu/error-report.h"
#include "cpu.h"
#include "tcg/tcg-op.h"
#include "exec/helper-proto.h"
#include "exec/helper-gen.h"
#include "exec/translation-block.h"
#include "exec/translator.h"
#include "exec/log.h"
#include "helper_conditions.h"

#define HELPER_H "helper.h"
#include "exec/helper-info.c.inc"
#undef  HELPER_H

#define NUM_REG_PAGE_SIZE 16
#define PC_REG 15
#define LR_REG 14
#define SP_REG 13

#define SYS_MODE 11

#define MMU_IDX 0


static TCGv cpu_sflags[32];

static const char avr32_cpu_r_names[AVR32_NREGS][8] = {
    "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7",
    "r8", "r9", "r10", "r11", "r12", "sp", "lr", "pc",
};

static const char avr32_cpu_sr_flag_names[32][8] = {
    "sregC", "sregZ", "sregN", "sregV", "sregQ", "sregL", "sreg6", "sreg7",
    "sreg8", "sreg9", "sreg10", "sreg11", "sreg12", "sreg13", "sregT", "sregR",
    "sregGM", "sregI0M", "sregI1M", "sregI2M", "sregI3M", "sregEM", "sregM0",
    "sregM1", "sregM2", "sreg25", "sregD", "sregDM", "sregJ", "sregH",
    "sreg30", "sregSS",
};

static TCGv cpu_r[NUM_REG_PAGE_SIZE];


enum {
    // Only the PC was modified, keep running without interrupt
    DISAS_JUMP = DISAS_TARGET_0,
    DISAS_EXIT = DISAS_TARGET_1,
    DISAS_CHAIN = DISAS_TARGET_2,
    /* PC = pc_next, then back to the main loop (mode, MMU or IRQ change) */
    DISAS_UPDATE = DISAS_TARGET_3,

};

typedef struct DisasContext DisasContext;

/* This is the state at translation time. */
struct DisasContext {
    DisasContextBase base;

    CPUAVR32State *env;
    CPUState *cs;

    uint32_t pc;
    int mem_idx;
    bool priv;
};

void avr32_tcg_init(void){
    int i;

    for(i = 0;i < NUM_REG_PAGE_SIZE; ++i) {
        cpu_r[i] = tcg_global_mem_new_i32(tcg_env,
                                          offsetof(CPUAVR32State, r[i]),
                                          avr32_cpu_r_names[i]);
    }

    for(i = 0;i < 32; ++i) {
        cpu_sflags[i] = tcg_global_mem_new_i32(tcg_env,
                                               offsetof(CPUAVR32State, sflags[i]),
                                               avr32_cpu_sr_flag_names[i]);
    }
}

// Decode helper required only if insn wide is variable
static uint32_t decode_insn_load_bytes(DisasContext *ctx, uint32_t insn,
                                       int i, int n)
{
    for (; i < n; i += 2) {
        insn |= translator_lduw(ctx->env, &ctx->base,
                                ctx->base.pc_next + i) << (16 - i * 8);
    }
    return insn;
}

static void gen_goto_tb(DisasContext* ctx, int n, target_ulong dest){
    if (translator_use_goto_tb(&ctx->base, dest)) {
        tcg_gen_goto_tb(n);
        tcg_gen_movi_i32(cpu_r[PC_REG], dest);
        tcg_gen_exit_tb(ctx->base.tb, n);
    } else {

        tcg_gen_movi_i32(cpu_r[PC_REG], dest);
        tcg_gen_lookup_and_goto_ptr();
    }
    ctx->base.is_jmp = DISAS_CHAIN;
}


static void gen_exception(DisasContext *ctx, int excp)
{
    gen_helper_raise_exception(tcg_env, tcg_constant_i32(excp));
    ctx->base.is_jmp = DISAS_NORETURN;
}

/* Privileged instructions trap in the application mode */
static bool gen_priv_check(DisasContext *ctx)
{
    if (!ctx->priv) {
        gen_exception(ctx, EXCP_AVR32_PRIVILEGE);
        return false;
    }
    return true;
}

static void gen_set_sr_bit(DisasContext *ctx, int bit, int val)
{
    if (bit < 16) {
        tcg_gen_movi_i32(cpu_sflags[bit], val);
        return;
    }
    /* the upper half may change the mode and thus the register banks */
    if (gen_priv_check(ctx)) {
        TCGv sr = tcg_temp_new_i32();

        gen_helper_mfsr(sr, tcg_env, tcg_constant_i32(SYSREG_SR));
        if (val) {
            tcg_gen_ori_i32(sr, sr, 1u << bit);
        } else {
            tcg_gen_andi_i32(sr, sr, ~(1u << bit));
        }
        gen_helper_mtsr(tcg_env, tcg_constant_i32(SYSREG_SR), sr);
        ctx->base.is_jmp = DISAS_UPDATE;
    }
}

static uint32_t decode_insn_load(DisasContext *ctx);
static bool decode_insn(DisasContext *ctx, uint32_t insn);
#include "decode-insn.c.inc"

static int sign_extend_8(int number){
    if(((number >> 7) & 1) == 1){
        number |= 0xFFFFFF00;
    }
    return number;
}

static bool trans_ABS(DisasContext *ctx, arg_ABS *a){
    TCGv reg = cpu_r[a->rd];
    tcg_gen_abs_i32(reg, reg);
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], reg, 0); /* Zf = R == 0 */
    ctx->base.pc_next += 2;
    return true;
}

//TODO: add tests
static bool trans_ACALL(DisasContext *ctx, arg_ACALL *a)
{
    TCGv addr = tcg_temp_new_i32();

    tcg_gen_ld_i32(addr, tcg_env, offsetof(CPUAVR32State, sysr[SYSREG_ACBA]));
    tcg_gen_addi_i32(addr, addr, a->disp << 2);
    tcg_gen_movi_i32(cpu_r[AVR32_LR_REG], ctx->base.pc_next + 2);
    tcg_gen_qemu_ld_i32(cpu_r[AVR32_PC_REG], addr, ctx->mem_idx, MO_BEUL);
    ctx->base.pc_next += 2;
    ctx->base.is_jmp = DISAS_JUMP;
    return true;
}

static bool trans_ACR(DisasContext *ctx, arg_ACR *a){
    TCGv rd = tcg_temp_new_i32();
    TCGv res = tcg_temp_new_i32();
    TCGv cond = tcg_temp_new_i32();

    tcg_gen_mov_i32(rd, cpu_r[a->rd]);

    //Add carry to reg
    tcg_gen_add_i32(res, rd, cpu_sflags[sflagC]);
    tcg_gen_mov_i32(cpu_r[a->rd], res);

    //modify status flags

    // Z-Flag
    tcg_gen_setcondi_tl(TCG_COND_EQ, cond, res, 0); /* Zf = R == 0 */
    tcg_gen_and_i32(cpu_sflags[sflagZ], cond, cpu_sflags[sflagZ]);

    // V-Flag
    tcg_gen_shri_i32(res, res, 31);
    tcg_gen_shri_i32(rd, rd, 31);
    tcg_gen_andc_i32(cpu_sflags[sflagV], res, rd);

    // N -Flag
    tcg_gen_mov_i32(cpu_sflags[sflagN], res);

    // C-Flag
    tcg_gen_andc_i32(cpu_sflags[sflagC], rd, res);


    ctx->base.pc_next += 2;
    return true;
}

static bool trans_ADC(DisasContext *ctx, arg_ADC *a){
    TCGv res = tcg_temp_new_i32();
    TCGv rx = tcg_temp_new_i32();
    TCGv ry = tcg_temp_new_i32();
    TCGv temp = tcg_temp_new_i32();

    tcg_gen_mov_i32(rx, cpu_r[a->rx]);
    tcg_gen_mov_i32(ry, cpu_r[a->ry]);

    tcg_gen_add_i32(res, rx,ry);
    tcg_gen_add_i32(res, res,cpu_sflags[sflagC]);
    tcg_gen_mov_i32( cpu_r[a->rd], res);

    // Z-flag
    tcg_gen_setcondi_i32(TCG_COND_EQ, temp, res, 0);
    tcg_gen_and_i32(cpu_sflags[sflagZ], cpu_sflags[sflagZ], temp);

    //move bit 31 to position 0
    tcg_gen_shri_i32(res, res, 31);
    tcg_gen_shri_i32(rx, rx, 31);
    tcg_gen_shri_i32(ry, ry, 31);

    // N-flag
    tcg_gen_mov_i32(cpu_sflags[sflagN], res);

    // V-flag
    set_v_flag_add(rx, ry, res, cpu_sflags);

    // C-flag
    set_c_flag_add(rx, ry, res, cpu_sflags);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_ADD_f1(DisasContext *ctx, arg_ADD_f1 *a){
    TCGv res = tcg_temp_new_i32();
    TCGv Rd = tcg_temp_new_i32();
    TCGv Rs = tcg_temp_new_i32();
    tcg_gen_mov_i32(Rd,  cpu_r[a->rd]);
    tcg_gen_mov_i32(Rs, cpu_r[a->rs]);


    tcg_gen_add_i32(res,  cpu_r[a->rd], cpu_r[a->rs]);
    tcg_gen_add_i32(cpu_r[a->rd],  cpu_r[a->rd], cpu_r[a->rs]);

    // set N flag: N ← RES[31]
    tcg_gen_shri_i32(cpu_sflags[sflagN], res, 31);

    // set Z flag: Z ← (RES[31:0] == 0)
    tcg_gen_setcondi_tl(TCG_COND_EQ, cpu_sflags[sflagZ], res, 0); /* Zf = R == 0 */

    tcg_gen_shri_i32(Rd, Rd, 31);
    tcg_gen_shri_i32(Rs, Rs, 31);
    tcg_gen_shri_i32(res, res, 31);

    // V-flag
    set_v_flag_add(Rd, Rs, res, cpu_sflags);

    // C-flag
    set_c_flag_add(Rd, Rs, res, cpu_sflags);


    if(a->rd == PC_REG){
        ctx->base.is_jmp = DISAS_JUMP;
    }

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_ADD_f2(DisasContext *ctx, arg_ADD_f2 *a){
    TCGv res = tcg_temp_new_i32();

    TCGv Rx = tcg_temp_new_i32();
    TCGv Ry = tcg_temp_new_i32();

    tcg_gen_mov_i32(Rx, cpu_r[a->rx]);
    tcg_gen_shli_i32(Ry, cpu_r[a->ry], a->sa);

    tcg_gen_add_i32(res, Rx, Ry);
    tcg_gen_add_i32(cpu_r[a->rd], Rx, Ry);

    // set N flag: N ← RES[31]
    tcg_gen_shri_i32(cpu_sflags[sflagN], res, 31);

    // set Z flag: Z ← (RES[31:0] == 0)
    tcg_gen_setcondi_tl(TCG_COND_EQ, cpu_sflags[sflagZ], res, 0); /* Zf = R == 0 */


    tcg_gen_shri_i32(Rx, Rx, 31);
    tcg_gen_shri_i32(Ry, Ry, 31);
    tcg_gen_shri_i32(res, res, 31);

    // V-flag
    set_v_flag_add(Rx, Ry, res, cpu_sflags);

    // C-flag
    set_c_flag_add(Rx, Ry, res, cpu_sflags);


    if(a->rd == PC_REG){
        ctx->base.is_jmp = DISAS_JUMP;
    }

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_ADD_cond(DisasContext *ctx, arg_ADD_cond *a){

    TCGLabel *no_add = gen_new_label();

    TCGv reg = tcg_temp_new_i32();
    int val = checkCondition(a->cond, reg, cpu_r, cpu_sflags);

    tcg_gen_brcondi_i32(TCG_COND_NE, reg, val, no_add);
    tcg_gen_add_i32(cpu_r[a->rd], cpu_r[a->rx], cpu_r[a->ry]);

    gen_set_label(no_add);


    if(a->rd == PC_REG){
        ctx->base.is_jmp = DISAS_JUMP;
    }
    ctx->base.pc_next += 4;
    return true;
}

//TODO: Add tests
static bool trans_ADDABS(DisasContext *ctx, arg_ADDABS *a){
    TCGv temp = tcg_temp_new_i32();
    tcg_gen_abs_i32(temp, cpu_r[a->ry]);
    tcg_gen_add_i32(cpu_r[a->rd], cpu_r[a->rx], temp);

    // Zf = result == 0
    tcg_gen_setcondi_tl(TCG_COND_EQ, cpu_sflags[sflagZ], cpu_r[a->rd], 0);

    ctx->base.pc_next += 4;
    return true;
}

//TODO: add tests
static bool trans_ADDHHW(DisasContext *ctx, arg_ADDHHW *a){
    TCGv op1 = tcg_temp_new_i32();
    TCGv op2 = tcg_temp_new_i32();
    TCGv res = tcg_temp_new_i32();

    if(a->x == 1){
        tcg_gen_shri_i32(op1, cpu_r[a->rx], 0x10);
    }
    else{
        tcg_gen_andi_i32(op1, cpu_r[a->rx], 0xFFFF);
    }
    tcg_gen_ext16s_i32(op1, op1);

    if(a->y == 1){
        tcg_gen_shri_i32(op2, cpu_r[a->ry], 0x10);
    }
    else{
        tcg_gen_andi_i32(op2, cpu_r[a->ry], 0xFFFF);
    }
    tcg_gen_ext16s_i32(op2, op2);
    tcg_gen_add_i32(cpu_r[a->rd], op1, op2);

    tcg_gen_mov_i32(res, cpu_r[a->rd]);

    // set N flag: N ← RES[31]
    tcg_gen_shri_i32(cpu_sflags[sflagN], res, 31);

    // set Z flag: Z ← (RES[31:0] == 0)
    tcg_gen_setcondi_tl(TCG_COND_EQ, cpu_sflags[sflagZ], res, 0); /* Zf = R == 0 */

    tcg_gen_shri_i32(op1, op1, 31);
    tcg_gen_shri_i32(op2, op2, 31);
    tcg_gen_shri_i32(res, res, 31);

    // V-flag
    set_v_flag_add(op1, op2, res, cpu_sflags);

    // C-flag
    set_c_flag_add(op1, op2, res, cpu_sflags);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_AND_f1(DisasContext *ctx, arg_AND_f1 *a){

    tcg_gen_and_i32(cpu_r[a->rd], cpu_r[a->rd], cpu_r[a->rs]);
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], cpu_r[a->rd], 0);
    tcg_gen_shri_i32(cpu_sflags[sflagN], cpu_r[a->rd], 31);
    ctx->base.pc_next += 2;
    return true;
}

static bool trans_AND_f2(DisasContext *ctx, arg_AND_f2 *a){
    TCGv temp = tcg_temp_new_i32();
    tcg_gen_shli_i32(temp, cpu_r[a->ry], a->sa5);
    tcg_gen_and_i32(cpu_r[a->rd], cpu_r[a->rx], temp);
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], cpu_r[a->rd], 0);
    tcg_gen_shri_i32(cpu_sflags[sflagN], cpu_r[a->rd], 31);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_AND_f3(DisasContext *ctx, arg_AND_f3 *a){
    TCGv temp = tcg_temp_new_i32();
    tcg_gen_shri_i32(temp, cpu_r[a->ry], a->sa5);
    tcg_gen_and_i32(cpu_r[a->rd], cpu_r[a->rx], temp);
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], cpu_r[a->rd], 0);
    tcg_gen_shri_i32(cpu_sflags[sflagN], cpu_r[a->rd], 31);

    ctx->base.pc_next += 4;
    return true;
}

//TODO: add tests
static bool trans_AND_cond(DisasContext *ctx, arg_AND_cond *a){
    TCGv conVal = tcg_temp_new_i32();
    int val = checkCondition(a->cond, conVal, cpu_r, cpu_sflags);
    TCGLabel *noAction = gen_new_label();
    tcg_gen_brcondi_i32(TCG_COND_NE, conVal, val, noAction);
    tcg_gen_and_i32(cpu_r[a->rd], cpu_r[a->rx], cpu_r[a->ry]);

    gen_set_label(noAction);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_ANDH(DisasContext *ctx, arg_ANDH *a){
    TCGv imm = tcg_temp_new_i32();
    TCGv rd = cpu_r[a->rd];

    tcg_gen_movi_i32(imm, a->imm);

    tcg_gen_shli_i32(imm, imm, 16);
    tcg_gen_ori_i32(imm, imm, 0x0000FFFF);
    tcg_gen_and_i32(rd, rd, imm);
    if(a->coh){
        tcg_gen_andi_i32(rd, rd, 0xFFFF0000);
    }
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], rd, 0);
    tcg_gen_shri_i32(cpu_sflags[sflagN], rd, 31);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_ANDL(DisasContext *ctx, arg_ANDL *a){
    TCGv imm = tcg_temp_new_i32();
    TCGv rd = cpu_r[a->rd];

    tcg_gen_movi_i32(imm, a->imm);
    tcg_gen_ori_i32(imm, imm, 0xFFFF0000);

    tcg_gen_and_i32(rd, rd, imm);
    if(a->coh){
        tcg_gen_andi_i32(rd, rd, 0x0000FFFF);
    }

    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], rd, 0);
    tcg_gen_shri_i32(cpu_sflags[sflagN], rd, 31);


    ctx->base.pc_next += 4;
    return true;
}

//TODO: add tests
static bool trans_ANDN(DisasContext *ctx, arg_ANDN *a){
    tcg_gen_andc_i32(cpu_r[a->rd], cpu_r[a->rd], cpu_r[a->rs]);
    ctx->base.pc_next += 2;
    return true;
}

static bool trans_ASR_f1(DisasContext *ctx, arg_ASR_f1 *a){
    TCGv shift = tcg_temp_new_i32();
    TCGv res = tcg_temp_new_i32();
    TCGv op = tcg_temp_new_i32();
    tcg_gen_andi_i32(shift, cpu_r[a->ry], 0x1F);

    tcg_gen_mov_i32(op, cpu_r[a->rx]);
    tcg_gen_sar_i32(res, cpu_r[a->rx], shift);
    tcg_gen_mov_i32(cpu_r[a->rd], res);
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], res, 0);
    tcg_gen_shri_i32(cpu_sflags[sflagN], res, 31);

    TCGLabel *exit = gen_new_label();
    TCGLabel *setCtoZero = gen_new_label();
    tcg_gen_brcondi_i32(TCG_COND_EQ, shift, 0, setCtoZero);

    tcg_gen_subi_i32(shift, shift, 0x1);
    tcg_gen_shr_i32(op, op, shift);
    tcg_gen_andi_i32(cpu_sflags[sflagC], op, 0x00000001);
    tcg_gen_br(exit);

    gen_set_label(setCtoZero);
    tcg_gen_movi_i32(cpu_sflags[sflagC], 0);

    gen_set_label(exit);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_ASR_f2(DisasContext *ctx, arg_ASR_f2 *a){
    int sa = a->bp4 << 1;
    sa += a->bp1;
    TCGv shift = tcg_temp_new_i32();
    TCGv res = tcg_temp_new_i32();
    TCGv op = tcg_temp_new_i32();
    tcg_gen_movi_i32(shift, sa);
    tcg_gen_mov_i32(op, cpu_r[a->rd]);

    tcg_gen_sar_i32(res, cpu_r[a->rd], shift);
    tcg_gen_mov_i32(cpu_r[a->rd], res);

    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], res, 0);
    tcg_gen_shri_i32(cpu_sflags[sflagN], res, 31);
    if(sa != 0){
        tcg_gen_subi_i32(shift, shift, 0x1);
        tcg_gen_shr_i32(op, op, shift);
        tcg_gen_andi_i32(cpu_sflags[sflagC], op, 0x00000001);
    }
    else{
        tcg_gen_movi_i32(cpu_sflags[sflagC], 0);
    }

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_ASR_f3(DisasContext *ctx, arg_ASR_f3 *a){
    TCGv res = tcg_temp_new_i32();
    TCGv op = tcg_temp_new_i32();
    tcg_gen_mov_i32(op, cpu_r[a->rs]);
    tcg_gen_sari_i32(res, cpu_r[a->rs], a->sa5);
    tcg_gen_mov_i32(cpu_r[a->rd], res);

    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], res, 0);
    tcg_gen_shri_i32(cpu_sflags[sflagN], res, 31);

    if(a->sa5 == 0){
        tcg_gen_movi_i32(cpu_sflags[sflagC], 0x0);
    }
    else{
        tcg_gen_shri_i32(cpu_sflags[sflagC], op, a->sa5-1);
        tcg_gen_andi_i32(cpu_sflags[sflagC], cpu_sflags[sflagC], 0x1);
    }

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_BFEXTS(DisasContext *ctx, arg_BFEXTS *a){
    TCGv rd = tcg_temp_new_i32();
    TCGv rs = tcg_temp_new_i32();
    TCGv temp = tcg_temp_new_i32();
    TCGLabel *end = gen_new_label();
    tcg_gen_mov_i32(rd, cpu_r[a->rd]);
    tcg_gen_mov_i32(rs, cpu_r[a->rs]);

    tcg_gen_shri_i32(rd, rs, a->bp5);
    tcg_gen_movi_i32(temp, 0xFFFFFFFF);
    tcg_gen_shri_i32(temp, temp, 32-a->w5);
    tcg_gen_and_i32(rd, rd, temp);
    tcg_gen_shri_i32(temp, rd, a->w5-1);
    tcg_gen_brcondi_i32(TCG_COND_EQ, temp, 0x0, end);

    tcg_gen_movi_i32(temp, 0xFFFFFFFF);
    tcg_gen_shli_i32(temp, temp, a->w5);
    tcg_gen_or_i32(rd, rd, temp);

    gen_set_label(end);
    tcg_gen_mov_i32(temp, rd);

    tcg_gen_mov_i32(cpu_r[a->rd], rd);

    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], rd, 0);
    tcg_gen_shri_i32(temp, rd, 31);
    tcg_gen_mov_i32(cpu_sflags[sflagC], temp);
    tcg_gen_mov_i32(cpu_sflags[sflagN], temp);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_BFEXTU(DisasContext *ctx, arg_BFEXTU *a){

    TCGv rd =tcg_temp_new_i32();
    TCGv rs = tcg_temp_new_i32();
    TCGv res = tcg_temp_new_i32();
    TCGv temp = tcg_temp_new_i32();

    tcg_gen_mov_i32(rd, cpu_r[a->rd]);
    tcg_gen_mov_i32(rs, cpu_r[a->rs]);

    tcg_gen_shri_i32(res, rs, a->bp5);
    tcg_gen_movi_i32(temp, 0xFFFFFFFF);
    tcg_gen_shri_i32(temp, temp, 32-a->w5);
    tcg_gen_and_i32(res, res, temp);
    tcg_gen_mov_i32(cpu_r[a->rd], res);

    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], res, 0);
    tcg_gen_shri_i32(temp, res, 31);
    tcg_gen_mov_i32(cpu_sflags[sflagC], temp);
    tcg_gen_mov_i32(cpu_sflags[sflagN], temp);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_BFINS(DisasContext *ctx, arg_BFINS *a){
    TCGv temp = tcg_temp_new_i32();
    TCGv mask = tcg_temp_new_i32();
    TCGv revMask = tcg_temp_new_i32();
    int bp5 = a->bp5;
    int w5 = a->w5-1;

    int maskI = 0x0;
    for(int i = 0; i<= w5; i++){
        maskI |= (1 << i);
    }
    int revMaskI = ~maskI;
    tcg_gen_movi_i32(mask, maskI);
    tcg_gen_movi_i32(revMask, revMaskI);

    tcg_gen_and_i32(temp, cpu_r[a->rs], mask);

    tcg_gen_rotli_i32(mask, mask, bp5);
    tcg_gen_rotli_i32(revMask, revMask, bp5);
    tcg_gen_shli_i32(temp, temp, bp5);

    tcg_gen_and_i32(cpu_r[a->rd], cpu_r[a->rd], revMask);
    tcg_gen_or_i32(cpu_r[a->rd], cpu_r[a->rd], temp);

    tcg_gen_shri_i32(cpu_sflags[sflagN], cpu_r[a->rd], 31);
    tcg_gen_shri_i32(cpu_sflags[sflagC], cpu_r[a->rd], 31);
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], cpu_r[a->rd], 0);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_BLD(DisasContext *ctx, arg_BLD *a){
    TCGv bit = tcg_temp_new_i32();
    tcg_gen_shri_i32(bit, cpu_r[a->rd], a->bp5);
    tcg_gen_andi_i32(bit, bit, 0x00000001);
    tcg_gen_mov_i32(cpu_sflags[sflagC], bit);
    tcg_gen_mov_i32(cpu_sflags[sflagZ], bit);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_BR_f1(DisasContext *ctx, arg_BR_f1 *a){
    int disp = (a->disp);

    //check if bit 8 is set => sign extend disp
    disp = sign_extend_8(disp);
    disp = disp << 1;

    TCGLabel *no_branch = gen_new_label();
    TCGv reg = tcg_temp_new_i32();
    int val = checkCondition(a->rd, reg, cpu_r, cpu_sflags);

    tcg_gen_brcondi_i32(TCG_COND_NE, reg, val, no_branch);
    gen_goto_tb(ctx, 0, ctx->base.pc_next+disp);

    gen_set_label(no_branch);

    ctx->base.pc_next += 2;
    ctx->base.is_jmp = DISAS_CHAIN;
    return true;
}

static bool trans_BR_f2(DisasContext *ctx, arg_BR_f2 *a){
    int disp = (a->disp2 << 17);
    disp |= (a->disp1 << 16);
    disp |= (a->disp0);

    if(disp >> 20 == 1){
        disp |= 0xFFE00000;
    }
    disp = disp << 1;

    TCGLabel *no_branch = gen_new_label();

    TCGv reg = tcg_temp_new_i32();
    int val = checkCondition(a->cond, reg, cpu_r, cpu_sflags);

    tcg_gen_brcondi_i32(TCG_COND_NE, reg, val, no_branch);
    gen_goto_tb(ctx, 0, ctx->base.pc_next+disp);

    gen_set_label(no_branch);

    ctx->base.pc_next += 4;
    ctx->base.is_jmp = DISAS_CHAIN;
    return true;
}

//TODO: Implement according to manual
static bool trans_BREAKPOINT(DisasContext *ctx, arg_BREAKPOINT *a)
{
    ctx->base.pc_next += 2;
    gen_exception(ctx, EXCP_AVR32_BREAKPOINT);
    return true;
}

static bool trans_BREV_r(DisasContext *ctx, arg_BREV_r *a){
    TCGv temp = tcg_temp_new_i32();
    TCGv new_val = tcg_temp_new_i32();
    tcg_gen_movi_i32(temp, 0);

    for(int i = 0; i<32; i++){
        tcg_gen_shri_i32(new_val, cpu_r[a->rd], i);
        tcg_gen_andi_i32(new_val, new_val, 0x00000001);
        tcg_gen_shli_i32(new_val, new_val, 31-i);
        tcg_gen_add_i32(temp, temp, new_val);
    }
    tcg_gen_mov_i32(cpu_r[a->rd], temp);
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], temp, 0);

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_BST(DisasContext *ctx, arg_BST *a){
    TCGv temp = tcg_temp_new_i32();
    tcg_gen_shli_i32(temp, cpu_sflags[sflagC], a->bp5);
    tcg_gen_or_i32(cpu_r[a->rd], cpu_r[a->rd], temp);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_CACHE(DisasContext *ctx, arg_CACHE *a)
{
    /* no caches are modelled */
    ctx->base.pc_next += 4;
    gen_priv_check(ctx);
    return true;
}

static bool trans_CASTSB(DisasContext *ctx, arg_CASTSB *a){
    tcg_gen_ext8s_i32(cpu_r[a->rd], cpu_r[a->rd]);

    tcg_gen_shri_i32(cpu_sflags[sflagN], cpu_r[a->rd], 31);
    tcg_gen_shri_i32(cpu_sflags[sflagC], cpu_r[a->rd], 31);
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], cpu_r[a->rd], 0);
    ctx->base.pc_next += 2; //verified
    return true;
}

static bool trans_CASTSH(DisasContext *ctx, arg_CASTSH *a){
    tcg_gen_ext16s_i32(cpu_r[a->rd], cpu_r[a->rd]);

    tcg_gen_shri_i32(cpu_sflags[sflagN], cpu_r[a->rd], 31);
    tcg_gen_shri_i32(cpu_sflags[sflagC], cpu_r[a->rd], 31);
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], cpu_r[a->rd], 0);

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_CASTUB(DisasContext *ctx, arg_CASTUB *a){
    tcg_gen_andi_i32(cpu_r[a->rd], cpu_r[a->rd], 0x000000FF);
    tcg_gen_shri_i32(cpu_sflags[sflagN], cpu_r[a->rd], 31);
    tcg_gen_shri_i32(cpu_sflags[sflagC], cpu_r[a->rd], 31);
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], cpu_r[a->rd], 0);
    ctx->base.pc_next += 2;
    return true;
}

static bool trans_CASTUH(DisasContext *ctx, arg_CASTUH *a){
    tcg_gen_andi_i32(cpu_r[a->rd], cpu_r[a->rd], 0x0000FFFF);
    tcg_gen_shri_i32(cpu_sflags[sflagN], cpu_r[a->rd], 31);
    tcg_gen_shri_i32(cpu_sflags[sflagC], cpu_r[a->rd], 31);
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], cpu_r[a->rd], 0);
    ctx->base.pc_next += 2;
    return true;
}

static bool trans_CBR(DisasContext *ctx, arg_CBR *a){
    int bp = 0;
    bp = a->bp4 << 1;
    bp += a->bp1;

    TCGv mask = tcg_temp_new_i32();
    tcg_gen_movi_i32(mask, 0xFFFFFFFE);
    tcg_gen_rotli_i32(mask, mask, bp);
    tcg_gen_and_i32(cpu_r[a->rd], cpu_r[a->rd], mask);

    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], cpu_r[a->rd], 0);

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_CLZ(DisasContext *ctx, arg_CLZ *a){
    TCGLabel *head = gen_new_label();
    TCGLabel *end = gen_new_label();
    TCGLabel *ifT = gen_new_label();

    TCGv temp = tcg_temp_new_i32();
    tcg_gen_movi_i32(temp, 32);
    TCGv i = tcg_temp_new_i32();
    tcg_gen_movi_i32(i, 31);

    TCGv Rs = tcg_temp_new_i32();

    gen_set_label(head);
    tcg_gen_brcondi_i32(TCG_COND_EQ, i, -1, end);
    tcg_gen_mov_i32(Rs, cpu_r[a->rs]);
    tcg_gen_shr_i32(Rs, Rs, i);
    tcg_gen_brcondi_i32(TCG_COND_EQ, Rs, 1, ifT);
    tcg_gen_subi_i32(i, i, 1);
    tcg_gen_br(head);

    gen_set_label(ifT);
    tcg_gen_movi_i32(temp, 31);
    tcg_gen_sub_i32(temp, temp, i);

    gen_set_label(end);
    tcg_gen_mov_i32(cpu_r[a->rd], temp);

    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], temp, 0);
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagC], temp, 0x20);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_COM(DisasContext *ctx, arg_COM *a){
    tcg_gen_not_i32(cpu_r[a->rd], cpu_r[a->rd]);
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], cpu_r[a->rd], 0);
    ctx->base.pc_next += 2;
    return true;
}

static bool trans_COP(DisasContext *ctx, arg_COP *a){
    //This instruction is processor specific!
    ctx->base.pc_next += 4;
    return true;
}

static bool trans_CPB(DisasContext *ctx, arg_CPB *a){
    TCGv res = tcg_temp_new_i32();
    TCGv rd = tcg_temp_new_i32();
    TCGv rs = tcg_temp_new_i32();

    tcg_gen_andi_i32(rd, cpu_r[a->rd], 0x000000FF);
    tcg_gen_andi_i32(rs, cpu_r[a->rs], 0x000000FF);
    tcg_gen_sub_i32(res, rd, rs);

    //Z-flag
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], res, 0);

    //N-Flag
    tcg_gen_shri_i32(res, res, 7);
    tcg_gen_andi_i32(res, res, 0x00000001);
    tcg_gen_mov_i32(cpu_sflags[sflagN], res);

    tcg_gen_shri_i32(rd, rd, 7);
    tcg_gen_andi_i32(rd, rd, 0x00000001);
    tcg_gen_shri_i32(rs, rs, 7);
    tcg_gen_andi_i32(rs, rs, 0x00000001);

    //V-flag
    set_v_flag_cp(rd, rs, res, cpu_sflags);

    //C-flag
    set_c_flag_cp(rd, rs, res, cpu_sflags);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_CPH(DisasContext *ctx, arg_CPH *a){
    TCGv res = tcg_temp_new_i32();
    TCGv rd = tcg_temp_new_i32();
    TCGv rs = tcg_temp_new_i32();

    tcg_gen_andi_i32(rd, cpu_r[a->rd], 0x0000FFFF);
    tcg_gen_andi_i32(rs, cpu_r[a->rs], 0x0000FFFF);
    tcg_gen_sub_i32(res, rd, rs);
    tcg_gen_andi_i32(res, res, 0x0000FFFF);

    //Z-flag
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], res, 0);

    //N-Flag
    tcg_gen_shri_i32(res, res, 15);
    tcg_gen_andi_i32(res, res, 0x00000001);
    tcg_gen_mov_i32(cpu_sflags[sflagN], res);

    tcg_gen_shri_i32(rd, rd, 15);
    tcg_gen_andi_i32(rd, rd, 0x00000001);
    tcg_gen_shri_i32(rs, rs, 15);
    tcg_gen_andi_i32(rs, rs, 0x00000001);

    //V-flag
    set_v_flag_cp(rd, rs, res, cpu_sflags);

    //C-flag
    set_c_flag_cp(rd, rs, res, cpu_sflags);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_CPW_f1(DisasContext *ctx, arg_CPW_f1 *a){
    TCGv Rd = tcg_temp_new_i32();
    TCGv Rs = tcg_temp_new_i32();

    tcg_gen_mov_i32(Rd, cpu_r[a->rd]);
    tcg_gen_mov_i32(Rs, cpu_r[a->rs]);

    cpw_instruction(Rd, Rs, cpu_sflags);

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_CPW_f2(DisasContext *ctx, arg_CPW_f2 *a){
    TCGv Rdt = cpu_r[a->rd];

    TCGv Rd = tcg_temp_new_i32();
    TCGv Rs = tcg_temp_new_i32();

    tcg_gen_mov_i32(Rd, Rdt);
    int imm = a->imm6;
    if(a->imm6 >> 5 == 1){
        imm |= 0xFFFFFFC0;
    }
    tcg_gen_movi_i32(Rs, imm);

    cpw_instruction(Rd, Rs, cpu_sflags);

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_CPW_f3(DisasContext *ctx, arg_CPW_f3 *a){
    int mmI = ( a->immu) <<17;
    mmI |= (a->immm) << 16;
    mmI |= (a->imml);

    //Sign extend
    if(mmI >> 20){
        mmI |= 0xFFE00000;
    }

    TCGv Rdt = cpu_r[a->rd];

    TCGv Rd = tcg_temp_new_i32();
    TCGv Rs = tcg_temp_new_i32();

    tcg_gen_mov_i32(Rd, Rdt);
    tcg_gen_movi_i32(Rs, mmI);

    cpw_instruction(Rd, Rs, cpu_sflags);

    ctx->base.pc_next += 4;
    return true;
}

//TODO: add more tests
static bool trans_CPC_f1(DisasContext *ctx, arg_CPC_f1 *a){
    TCGv res = tcg_temp_new_i32();
    TCGv rd = tcg_temp_new_i32();
    TCGv rs = tcg_temp_new_i32();

    tcg_gen_mov_i32(rd, cpu_r[a->rd]);
    tcg_gen_mov_i32(rs, cpu_r[a->rs]);
    tcg_gen_sub_i32(res, cpu_r[a->rd], cpu_r[a->rs]);
    tcg_gen_sub_i32(res, res, cpu_sflags[sflagC]);

    set_flags_cpc(rd, rs, res, cpu_sflags);

    ctx->base.pc_next += 4;
    return true;
}

//TODO: add more tests
static bool trans_CPC_f2(DisasContext *ctx, arg_CPC_f2 *a){
    TCGv res = tcg_temp_new_i32();
    TCGv rd = tcg_temp_new_i32();
    TCGv rs = tcg_temp_new_i32();

    tcg_gen_mov_i32(rd, cpu_r[a->rd]);
    tcg_gen_movi_i32(rs, 0);
    tcg_gen_sub_i32(res, rd, cpu_sflags[sflagC]);

    set_flags_cpc(rd, rs, res, cpu_sflags);

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_CSRF(DisasContext *ctx, arg_CSRF *a)
{
    ctx->base.pc_next += 2;
    gen_set_sr_bit(ctx, a->bp5, 0);
    return true;
}

//TODO: add tests
static bool trans_CSRFCZ(DisasContext *ctx, arg_CSRFCZ *a){
    tcg_gen_mov_i32(cpu_sflags[sflagC], cpu_sflags[a->bp5]);
    tcg_gen_mov_i32(cpu_sflags[sflagZ], cpu_sflags[a->bp5]);

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_DIVS(DisasContext *ctx, arg_DIVS *a){
    TCGv rx = tcg_temp_new_i32();
    TCGv ry = tcg_temp_new_i32();

    tcg_gen_mov_i32(rx, cpu_r[a->rx]);
    tcg_gen_mov_i32(ry, cpu_r[a->ry]);

    tcg_gen_div_i32(cpu_r[a->rd], rx, ry);
    tcg_gen_rem_i32(cpu_r[a->rd+1], rx, ry);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_DIVU(DisasContext *ctx, arg_DIVU *a){
    TCGv rx = tcg_temp_new_i32();
    TCGv ry = tcg_temp_new_i32();

    tcg_gen_mov_i32(rx, cpu_r[a->rx]);
    tcg_gen_mov_i32(ry, cpu_r[a->ry]);

    tcg_gen_divu_i32(cpu_r[a->rd], rx, ry);
    tcg_gen_remu_i32(cpu_r[a->rd+1], rx,ry);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_EOR_f1(DisasContext *ctx, arg_EOR_f1 *a){
    tcg_gen_xor_i32(cpu_r[a->rd], cpu_r[a->rd], cpu_r[a->rs]);
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], cpu_r[a->rd], 0);
    tcg_gen_shri_i32(cpu_sflags[sflagN], cpu_r[a->rd], 31);

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_EOR_f2(DisasContext *ctx, arg_EOR_f2 *a){
    TCGv temp = tcg_temp_new_i32();
    tcg_gen_shli_i32(temp, cpu_r[a->ry], a->sa5);
    tcg_gen_xor_i32(cpu_r[a->rd], cpu_r[a->rx], temp);

    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], cpu_r[a->rd], 0);
    tcg_gen_shri_i32(cpu_sflags[sflagN], cpu_r[a->rd], 31);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_EOR_f3(DisasContext *ctx, arg_EOR_f3 *a){
    TCGv temp = tcg_temp_new_i32();
    tcg_gen_shri_i32(temp, cpu_r[a->ry], a->sa5);
    tcg_gen_xor_i32(cpu_r[a->rd], cpu_r[a->rx], temp);

    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], cpu_r[a->rd], 0);
    tcg_gen_shri_i32(cpu_sflags[sflagN], cpu_r[a->rd], 31);

    ctx->base.pc_next += 4;
    return true;
}

//TODO: add tests
static bool trans_EOR_cond(DisasContext *ctx, arg_EOR_cond *a){
    TCGv reg = tcg_temp_new_i32();
    TCGLabel *exit = gen_new_label();
    int val = checkCondition(a->cond, reg, cpu_r, cpu_sflags);
    tcg_gen_brcondi_i32(TCG_COND_NE, reg, val, exit);

    tcg_gen_xor_i32(cpu_r[a->rd], cpu_r[a->rx], cpu_r[a->ry]);

    gen_set_label(exit);
    ctx->base.pc_next += 4;
    return true;
}

static bool trans_EORH(DisasContext *ctx, arg_EORH *a){
    TCGv imm = tcg_temp_new_i32();
    tcg_gen_movi_i32(imm, a->imm16);

    tcg_gen_shli_i32(imm, imm, 0x10);
    tcg_gen_xor_i32(cpu_r[a->rd], cpu_r[a->rd], imm);
    tcg_gen_shri_i32(cpu_sflags[sflagN], cpu_r[a->rd], 31);
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], cpu_r[a->rd], 0);


    ctx->base.pc_next += 4;
    return true;
}

static bool trans_EORL(DisasContext *ctx, arg_EORH *a){
    TCGv imm = tcg_temp_new_i32();
    tcg_gen_movi_i32(imm, a->imm16);

    tcg_gen_xor_i32(cpu_r[a->rd], cpu_r[a->rd], imm);
    tcg_gen_shri_i32(cpu_sflags[sflagN], cpu_r[a->rd], 31);
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], cpu_r[a->rd], 0);
    ctx->base.pc_next += 4;
    return true;
}

static bool trans_FRS(DisasContext *ctx, arg_FRS *a)
{
    /* no return stack is modelled */
    ctx->base.pc_next += 2;
    return true;
}

static bool trans_ICALL(DisasContext *ctx, arg_ICALL *a){
    TCGv dest = tcg_temp_new_i32();

    /* icall lr: read the target before LR is overwritten */
    tcg_gen_mov_i32(dest, cpu_r[a->rd]);
    tcg_gen_addi_i32(cpu_r[LR_REG], cpu_r[PC_REG], 2);
    tcg_gen_mov_i32(cpu_r[PC_REG], dest);

    ctx->base.is_jmp = DISAS_JUMP;
    ctx->base.pc_next += 2;
    return true;
}

static bool trans_LDD_f1(DisasContext *ctx, arg_LDD_f1 *a){
    TCGv ptr = cpu_r[a->rp];
    TCGv rd = cpu_r[a->rd*2];
    TCGv rdp = cpu_r[a->rd*2+1];

    tcg_gen_qemu_ld_i32(rdp, ptr, ctx->mem_idx, MO_BEUL);
    tcg_gen_addi_i32(ptr, ptr, 4);
    tcg_gen_qemu_ld_i32(rd, ptr, ctx->mem_idx, MO_BEUL);
    tcg_gen_addi_i32(ptr, ptr, 4);

    ctx->base.pc_next += 2;

    return true;
}

static bool trans_LDD_f2(DisasContext *ctx, arg_LDD_f2 *a){
    TCGv ptr = tcg_temp_new_i32();
    tcg_gen_mov_i32(ptr, cpu_r[a->rp]);
    TCGv rd = cpu_r[a->rd*2];
    TCGv rdp = cpu_r[a->rd*2+1];

    tcg_gen_subi_i32(ptr, ptr, 8);
    tcg_gen_qemu_ld_i32(rdp, ptr, ctx->mem_idx, MO_BEUL);
    tcg_gen_addi_i32(ptr, ptr, 4);

    tcg_gen_qemu_ld_i32(rd, ptr, ctx->mem_idx, MO_BEUL);
    tcg_gen_subi_i32(ptr, ptr, 4);

    tcg_gen_mov_i32(cpu_r[a->rp], ptr);

    ctx->base.pc_next += 2;

    return true;
}

static bool trans_LDD_f3(DisasContext *ctx, arg_LDD_f3 *a){
    TCGv ptr = tcg_temp_new_i32();
    tcg_gen_mov_i32(ptr, cpu_r[a->rp]);
    TCGv rd = cpu_r[a->rd*2];
    TCGv rdp = cpu_r[a->rd*2+1];

    tcg_gen_qemu_ld_i32(rdp, ptr, ctx->mem_idx, MO_BEUL);
    tcg_gen_addi_i32(ptr, ptr, 4);
    tcg_gen_qemu_ld_i32(rd, ptr, ctx->mem_idx, MO_BEUL);

    ctx->base.pc_next += 2;

    return true;
}

static bool trans_LDD_f4(DisasContext *ctx, arg_LDD_f4 *a){
    TCGv ptr = tcg_temp_new_i32();
    tcg_gen_mov_i32(ptr, cpu_r[a->rp]);
    TCGv disp = tcg_temp_new_i32();

    int dispI = a->disp16;
    if(dispI >> 15){
        dispI |= 0xFFFF0000;
    }
    tcg_gen_movi_i32(disp, dispI);
    tcg_gen_add_i32(ptr, ptr, disp);

    TCGv rd = cpu_r[a->rs*2];
    TCGv rdp = cpu_r[a->rs*2+1];

    tcg_gen_qemu_ld_i32(rdp, ptr, ctx->mem_idx, MO_BEUL);
    tcg_gen_addi_i32(ptr, ptr, 4);
    tcg_gen_qemu_ld_i32(rd, ptr, ctx->mem_idx, MO_BEUL);
    tcg_gen_addi_i32(ptr, ptr, 4);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_LDD_f5(DisasContext *ctx, arg_LDD_f5 *a){
    TCGv ptr = tcg_temp_new_i32();

    tcg_gen_shli_i32(ptr, cpu_r[a->ry], a->sa);
    tcg_gen_add_i32(ptr, ptr, cpu_r[a->rx]);


    TCGv rd = cpu_r[a->rd];
    TCGv rdp = cpu_r[a->rd+1];

    tcg_gen_qemu_ld_i32(rdp, ptr, ctx->mem_idx, MO_BEUL);
    tcg_gen_addi_i32(ptr, ptr, 4);
    tcg_gen_qemu_ld_i32(rd, ptr, ctx->mem_idx, MO_BEUL);
    tcg_gen_addi_i32(ptr, ptr, 4);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_LDsb_f1(DisasContext *ctx, arg_LDsb_f1 *a){
    int dispI = a->disp16;
    if(dispI >> 15){
        dispI |= 0xFFFF0000;
    }

    TCGv ptr = tcg_temp_new_i32();
    tcg_gen_movi_i32(ptr, dispI);
    tcg_gen_add_i32(ptr, ptr, cpu_r[a->rp]);

    tcg_gen_qemu_ld_i32(cpu_r[a->rd], ptr, ctx->mem_idx, MO_SB);

    ctx->base.pc_next += 4;
    return true;
}

//TODO: add tests
static bool trans_LDsb_f2(DisasContext *ctx, arg_LDsb_f2 *a){
    TCGv ptr = tcg_temp_new_i32();
    tcg_gen_mov_i32(ptr, cpu_r[a->ry]);
    tcg_gen_shli_i32(ptr, ptr, a->sa);
    tcg_gen_add_i32(ptr, ptr, cpu_r[a->rx]);

    tcg_gen_qemu_ld_i32(cpu_r[a->rd], ptr, ctx->mem_idx, MO_SB);

    ctx->base.pc_next += 4;
    return true;
}

//TODO: add tests
static bool trans_LDsb_cond(DisasContext *ctx, arg_LDsb_cond *a){
    TCGv reg = tcg_temp_new_i32();
    TCGLabel *exit = gen_new_label();
    int val = checkCondition(a->cond4, reg, cpu_r, cpu_sflags);
    tcg_gen_brcondi_i32(TCG_COND_NE, reg, val, exit);

    TCGv ptr = tcg_temp_new_i32();
    tcg_gen_addi_i32(ptr, cpu_r[a->rp], a->disp9);

    tcg_gen_qemu_ld_i32(cpu_r[a->rd], ptr, ctx->mem_idx, MO_SB);


    gen_set_label(exit);
    ctx->base.pc_next += 4;
    return true;
}

static bool trans_LDub_f1(DisasContext *ctx, arg_LDub_f1 *a){
    tcg_gen_qemu_ld_tl(cpu_r[a->rd], cpu_r[a->rp], ctx->mem_idx, MO_UB);
    tcg_gen_addi_i32(cpu_r[a->rp], cpu_r[a->rp], 0x1);

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_LDub_f2(DisasContext *ctx, arg_LDub_f2 *a){
    tcg_gen_qemu_ld_tl(cpu_r[a->rd], cpu_r[a->rp], ctx->mem_idx, MO_UB);
    tcg_gen_subi_i32(cpu_r[a->rp], cpu_r[a->rp], 0x1);

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_LDub_f3(DisasContext *ctx, arg_LDub_f3 *a){

    TCGv ptr = tcg_temp_new_i32();
    tcg_gen_addi_i32(ptr, cpu_r[a->rp], a->disp3);

    tcg_gen_qemu_ld_tl(cpu_r[a->rd], ptr, ctx->mem_idx, MO_UB);

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_LDUB_f4(DisasContext *ctx, arg_LDUB_f4 *a){
    TCGv ptr = tcg_temp_new_i32();
    int disp = a->disp16;
    if(disp >> 15){
        disp |= 0xFFFF0000;
    }
    tcg_gen_movi_i32(ptr, disp);
    tcg_gen_add_i32(ptr, ptr, cpu_r[a->rp]);

    tcg_gen_qemu_ld_tl(cpu_r[a->rd], ptr, ctx->mem_idx, MO_UB);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_LDub_f5(DisasContext *ctx, arg_LDub_f5 *a){

    TCGv ptr = tcg_temp_new_i32();
    tcg_gen_shli_i32(ptr, cpu_r[a->ry], a->sa);
    tcg_gen_add_i32(ptr, ptr, cpu_r[a->rx]);

    tcg_gen_qemu_ld_tl(cpu_r[a->rd], ptr, ctx->mem_idx, MO_UB);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_LDUB_cond(DisasContext *ctx, arg_LDUB_cond *a){


    TCGLabel *exit = gen_new_label();

    TCGv reg = tcg_temp_new_i32();
    int val = checkCondition(a->cond4, reg, cpu_r, cpu_sflags);

    tcg_gen_brcondi_i32(TCG_COND_NE, reg, val, exit);

    TCGv ptr = tcg_temp_new_i32();
    tcg_gen_mov_i32(ptr, cpu_r[a->rp]);
    tcg_gen_addi_i32(ptr, ptr, a->disp9);

    tcg_gen_qemu_ld_tl(cpu_r[a->rd], ptr, ctx->mem_idx, MO_UB);

    gen_set_label(exit);
    ctx->base.pc_next += 4;
    return true;
}

//TODO: add tests
static bool trans_LDSH_f1(DisasContext *ctx, arg_LDSH_f1 *a){
    tcg_gen_qemu_ld_tl(cpu_r[a->rd], cpu_r[a->rp], ctx->mem_idx, MO_BESW);
    tcg_gen_addi_i32(cpu_r[a->rp], cpu_r[a->rp], 0x2);

    ctx->base.pc_next += 2;
    return true;
}

//TODO: add tests
static bool trans_LDSH_f2(DisasContext *ctx, arg_LDSH_f2 *a){
    tcg_gen_subi_i32(cpu_r[a->rp], cpu_r[a->rp], 0x2);
    tcg_gen_qemu_ld_tl(cpu_r[a->rd], cpu_r[a->rp], ctx->mem_idx, MO_BESW);
    ctx->base.pc_next += 2;
    return true;
}

static bool trans_LDSH_f3(DisasContext *ctx, arg_LDSH_f3 *a){
    TCGv addr = tcg_temp_new_i32();
    tcg_gen_movi_i32(addr, a->disp3 << 1);
    tcg_gen_add_i32(addr, addr, cpu_r[a->rp]);
    tcg_gen_qemu_ld_tl(cpu_r[a->rd], addr, ctx->mem_idx, MO_BESW);

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_LDSH_f4(DisasContext *ctx, arg_LDSH_f4 *a){
    TCGv addr = tcg_temp_new_i32();
    int disp = a->disp16;
    if(disp>> 15 == 1){
        disp |= 0xFFFF0000;
    }

    tcg_gen_addi_i32(addr, cpu_r[a->rp], disp);
    tcg_gen_qemu_ld_tl(cpu_r[a->rd], addr, ctx->mem_idx, MO_BESW);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_LDSH_f5(DisasContext *ctx, arg_LDSH_f5 *a){
    TCGv addr = tcg_temp_new_i32();

    tcg_gen_shli_i32(addr, cpu_r[a->ry], a->sa);
    tcg_gen_add_i32(addr, addr, cpu_r[a->rx]);
    tcg_gen_qemu_ld_tl(cpu_r[a->rd], addr, ctx->mem_idx, MO_BESW);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_LDSH_cond(DisasContext *ctx, arg_LDSH_cond *a){
    TCGLabel *no_load = gen_new_label();

    TCGv reg = tcg_temp_new_i32();
    int val = checkCondition(a->cond4, reg, cpu_r, cpu_sflags);

    tcg_gen_brcondi_i32(TCG_COND_NE, reg, val, no_load);

    TCGv addr = tcg_temp_new_i32();
    tcg_gen_movi_i32(addr, a->disp9);
    tcg_gen_shli_i32(addr, addr, 1);
    tcg_gen_add_i32(addr, addr, cpu_r[a->rp]);
    tcg_gen_qemu_ld_tl(cpu_r[a->rd], addr, ctx->mem_idx, MO_BESW);

    gen_set_label(no_load);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_LDUH_f1(DisasContext *ctx, arg_LDUH_f1 *a){
    tcg_gen_qemu_ld_tl(cpu_r[a->rd], cpu_r[a->rp], ctx->mem_idx, MO_BEUW);
    tcg_gen_addi_i32(cpu_r[a->rp], cpu_r[a->rp], 2);

    ctx->base.pc_next += 2;
    return true;
}

//TODO: add tests
static bool trans_LDUH_f2(DisasContext *ctx, arg_LDUH_f2 *a){
    tcg_gen_subi_i32(cpu_r[a->rp], cpu_r[a->rp], 2);
    tcg_gen_qemu_ld_tl(cpu_r[a->rd], cpu_r[a->rp], ctx->mem_idx, MO_BEUW);

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_LDUH_f3(DisasContext *ctx, arg_LDUH_f3 *a){
    TCGv addr = tcg_temp_new_i32();
    tcg_gen_addi_i32(addr, cpu_r[a->rp], a->disp3<<1);
    tcg_gen_qemu_ld_tl(cpu_r[a->rd], addr, ctx->mem_idx, MO_BEUW);

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_LDUH_f4(DisasContext *ctx, arg_LDUH_f4 *a){
    int disp = a->disp16;
    if(disp>> 16 == 1){
        disp |= 0xFFFF0000;
    }

    TCGv addr = tcg_temp_new_i32();
    tcg_gen_addi_i32(addr, cpu_r[a->rp], disp);
    tcg_gen_qemu_ld_tl(cpu_r[a->rd], addr, ctx->mem_idx, MO_BEUW);

    ctx->base.pc_next += 4;
    return true;
}

//TODO: add tests
static bool trans_LDUH_f5(DisasContext *ctx, arg_LDUH_f5 *a){
    TCGv ptr = tcg_temp_new_i32();
    tcg_gen_mov_i32(ptr, cpu_r[a->ry]);
    tcg_gen_shli_i32(ptr, ptr, a->sa);
    tcg_gen_add_i32(ptr, ptr, cpu_r[a->rx]);

    tcg_gen_qemu_ld_i32(cpu_r[a->rd], ptr, ctx->mem_idx, MO_BEUW);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_LDUH_cond(DisasContext *ctx, arg_LDUH_cond *a){
    TCGLabel *no_load = gen_new_label();

    TCGv reg = tcg_temp_new_i32();
    int val = checkCondition(a->cond4, reg, cpu_r, cpu_sflags);

    tcg_gen_brcondi_i32(TCG_COND_NE, reg, val, no_load);

    TCGv addr = tcg_temp_new_i32();
    tcg_gen_movi_i32(addr, a->disp9);
    tcg_gen_shli_i32(addr, addr, 1);
    tcg_gen_add_i32(addr, addr, cpu_r[a->rp]);
    tcg_gen_qemu_ld_tl(cpu_r[a->rd], addr, ctx->mem_idx, MO_BEUW);

    gen_set_label(no_load);

    ctx->base.pc_next += 4;
    return true;
}


static bool trans_LDW_f1(DisasContext *ctx, arg_LDW_f1 *a){
    tcg_gen_qemu_ld_i32(cpu_r[a->rd], cpu_r[a->rp], ctx->mem_idx, MO_BEUL);
    tcg_gen_addi_i32(cpu_r[a->rp], cpu_r[a->rp], 0x4);

    if(a->rd == PC_REG){
        ctx->base.is_jmp = DISAS_JUMP;
    }

    ctx->base.pc_next += 2;
    return true;
}

//TODO: add tests
static bool trans_LDW_f2(DisasContext *ctx, arg_LDW_f2 *a){
    tcg_gen_subi_i32(cpu_r[a->rp], cpu_r[a->rp], 0x4);
    tcg_gen_qemu_ld_i32(cpu_r[a->rd], cpu_r[a->rp], ctx->mem_idx, MO_BEUL);

    if(a->rd == PC_REG){
        ctx->base.is_jmp = DISAS_JUMP;
    }

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_LDW_f3(DisasContext *ctx, arg_LDW_f3 *a){
    int dispI = a->disp5 <<2;

    TCGv ptr = tcg_temp_new_i32();
    tcg_gen_addi_i32(ptr, cpu_r[a->rp], dispI);
    tcg_gen_qemu_ld_i32(cpu_r[a->rd], ptr, ctx->mem_idx, MO_BEUL);

    if(a->rd == PC_REG){
        ctx->base.is_jmp = DISAS_JUMP;
    }

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_LDW_f4(DisasContext *ctx, arg_LDW_f4 *a){
    int disp = a->disp16;

    //Sign extend
    if(disp >> 15){
        disp |= 0xFFFF0000;
    }
    TCGv ptr = tcg_temp_new_i32();
    tcg_gen_addi_i32(ptr, cpu_r[a->rp], disp);

    tcg_gen_qemu_ld_i32(cpu_r[a->rd], ptr, ctx->mem_idx, MO_BEUL);

    if(a->rd == PC_REG){
        ctx->base.is_jmp = DISAS_JUMP;
    }

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_LDW_f5(DisasContext *ctx, arg_LDW_f5 *a){
    TCGv ptr = tcg_temp_new_i32();
    tcg_gen_shli_i32(ptr, cpu_r[a->ry], a->sa);
    tcg_gen_add_i32(ptr, ptr, cpu_r[a->rx]);

    tcg_gen_qemu_ld_i32(cpu_r[a->rd], ptr, ctx->mem_idx, MO_BEUL);

    if(a->rd == PC_REG){
        ctx->base.is_jmp = DISAS_JUMP;
    }

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_LDW_f6(DisasContext *ctx, arg_LDW_f6 *a){
    int part = (a->x << 1) + a->y;
    TCGv ptr = tcg_temp_new_i32();
    TCGv temp = tcg_temp_new_i32();
    tcg_gen_mov_i32(ptr, cpu_r[a->rx]);

    switch (part) {
        case 0x0:
            tcg_gen_andi_i32(temp, cpu_r[a->ry], 0x000000FF);
            break;
        case 0x1:
            tcg_gen_andi_i32(temp, cpu_r[a->ry], 0x0000FF00);
            tcg_gen_shri_i32(temp, temp, 8);
            break;
        case 0x2:
            tcg_gen_andi_i32(temp, cpu_r[a->ry], 0x00FF0000);
            tcg_gen_shri_i32(temp, temp, 16);
            break;
        case 0x3:
            tcg_gen_andi_i32(temp, cpu_r[a->ry], 0xFF000000);
            tcg_gen_shri_i32(temp, temp, 24);
            break;
        default:
            printf("[LDW_f6] ERROR: undefined condition %d\n", part);
            g_assert_not_reached();
            return false;
    }
    tcg_gen_shli_i32(temp, temp, 2);
    tcg_gen_add_i32(ptr, ptr, temp);

    tcg_gen_qemu_ld_i32(cpu_r[a->rd], ptr, ctx->mem_idx, MO_BEUL);

    if(a->rd == PC_REG){
        ctx->base.is_jmp = DISAS_JUMP;
    }

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_LDW_cond(DisasContext *ctx, arg_LDW_cond *a){
    int disp = a->disp9 << 2;
    TCGLabel *no_ld = gen_new_label();

    TCGv reg = tcg_temp_new_i32();
    int val = checkCondition(a->cond4, reg, cpu_r, cpu_sflags);

    tcg_gen_brcondi_i32(TCG_COND_NE, reg, val, no_ld);
    TCGv ptr = tcg_temp_new_i32();
    tcg_gen_movi_i32(ptr, disp);
    tcg_gen_add_i32(ptr, ptr, cpu_r[a->rp]);
    tcg_gen_qemu_ld_i32(cpu_r[a->rd], ptr, ctx->mem_idx, MO_BEUL);
    gen_set_label(no_ld);
    if(a->rd == PC_REG){
        ctx->base.is_jmp = DISAS_JUMP;
    }
    ctx->base.pc_next += 4;
    return true;
}

// LDC, processor depending instruction

static bool trans_LDDPC(DisasContext *ctx, arg_LDDPC *a){
    TCGv addr = tcg_temp_new_i32();
    TCGv Rd = cpu_r[a->rd];
    TCGv PC = cpu_r[PC_REG];

    tcg_gen_andi_tl(addr, PC, 0xFFFFFFFC);
    tcg_gen_addi_tl(addr, addr, a->disp << 2);

    tcg_gen_qemu_ld_tl(Rd, addr, ctx->mem_idx, MO_BEUL);

    if(a->rd == PC_REG){
        ctx->base.is_jmp = DISAS_JUMP;
    }
    ctx->base.pc_next += 2;
    return true;
}

static bool trans_LDDSP(DisasContext *ctx, arg_LDDSP *a){
    TCGv addr = tcg_temp_new_i32();
    tcg_gen_andi_i32(addr, cpu_r[SP_REG], 0xFFFFFFFC);
    tcg_gen_addi_i32(addr, addr, a->disp << 2);
    tcg_gen_qemu_ld_tl(cpu_r[a->rd], addr, ctx->mem_idx, MO_BEUL);

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_LDINSB(DisasContext *ctx, arg_LDINSB *a){
    TCGv ptr = tcg_temp_new_i32();
    TCGv temp = tcg_temp_new_i32();
    TCGv mask = tcg_temp_new_i32();
    int disp = a->disp12;
    //Sign extend
    if(disp >> 12){
        disp |= 0xFFFFF000;
    }
    tcg_gen_addi_i32(ptr, cpu_r[a->rp], disp);
    tcg_gen_movi_i32(mask, 0xFFFFFF00);
    tcg_gen_rotli_i32(mask, mask, a->part * 8);
    tcg_gen_and_i32(cpu_r[a->rd], cpu_r[a->rd], mask);

    tcg_gen_qemu_ld_i32(temp, ptr, ctx->mem_idx, MO_UB);
    tcg_gen_shli_i32(temp, temp, a->part * 8);
    tcg_gen_or_i32(cpu_r[a->rd], cpu_r[a->rd], temp);
    tcg_gen_movi_i32(cpu_r[10], a->part);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_LDINSH(DisasContext *ctx, arg_LDINSH *a){
    TCGv ptr = tcg_temp_new_i32();
    TCGv temp = tcg_temp_new_i32();
    TCGv mask = tcg_temp_new_i32();
    int disp = a->disp12;
    //Sign extend
    if(disp >> 12){
        disp |= 0xFFFFF000;
    }
    disp = disp << 1;
    tcg_gen_addi_i32(ptr, cpu_r[a->rp], disp);
    tcg_gen_movi_i32(mask, 0xFFFF0000);
    tcg_gen_rotli_i32(mask, mask, a->part * 16);
    tcg_gen_and_i32(cpu_r[a->rd], cpu_r[a->rd], mask);

    tcg_gen_qemu_ld_i32(temp, ptr, ctx->mem_idx, MO_UW);
    tcg_gen_shli_i32(temp, temp, a->part * 16);
    tcg_gen_or_i32(cpu_r[a->rd], cpu_r[a->rd], temp);
    tcg_gen_movi_i32(cpu_r[10], a->part);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_LDM(DisasContext *ctx, arg_LDM *a){
    int reglist = a->list;
    TCGv loadaddress = tcg_temp_new_i32();
    tcg_gen_mov_i32(loadaddress, cpu_r[a->rp]);
    bool setFlags = false;

    if(((reglist >> 15) &1) == 1){
        if(a->rp == PC_REG){
            tcg_gen_mov_i32(loadaddress, cpu_r[SP_REG]);
        }
        tcg_gen_qemu_ld_tl(cpu_r[PC_REG], loadaddress, ctx->mem_idx, MO_BEUL);
        tcg_gen_addi_i32(loadaddress, loadaddress, 4);

        ctx->base.is_jmp = DISAS_JUMP;
        if(a->rp == PC_REG){
            if(((reglist >> 14) & 1) == 0 && ((reglist >> 12) &1) == 0){
                tcg_gen_movi_i32(cpu_r[12], 0);
            }
            else if((reglist >> 14) == 0 && (reglist >> 12) == 1){
                tcg_gen_movi_i32(cpu_r[12], 1);
            }
            else{
                tcg_gen_movi_i32(cpu_r[12], -1);
            }
            setFlags = true;
        }
        else{
            if(((reglist >> 14) &1) ==1){
                tcg_gen_qemu_ld_tl(cpu_r[LR_REG], loadaddress, ctx->mem_idx, MO_BEUL);
                tcg_gen_addi_i32(loadaddress, loadaddress, 4);
            }
            if(((reglist >> 13) &1) ==1){
                tcg_gen_qemu_ld_tl(cpu_r[SP_REG], loadaddress, ctx->mem_idx, MO_BEUL);
                tcg_gen_addi_i32(loadaddress, loadaddress, 4);

            }
            if(((reglist >> 12) &1) ==1){
                tcg_gen_qemu_ld_tl(cpu_r[12], loadaddress, ctx->mem_idx, MO_BEUL);
                tcg_gen_addi_i32(loadaddress, loadaddress, 4);
            }
            setFlags = true;
        }
    }
    else{
        if(((reglist >> 14) &1) ==1){
            tcg_gen_qemu_ld_tl(cpu_r[LR_REG], loadaddress, ctx->mem_idx, MO_BEUL);
            tcg_gen_addi_i32(loadaddress, loadaddress, 4);
        }
        if(((reglist >> 13) &1) ==1){
            tcg_gen_qemu_ld_tl(cpu_r[SP_REG], loadaddress, ctx->mem_idx, MO_BEUL);
            tcg_gen_addi_i32(loadaddress, loadaddress, 4);

        }
        if(((reglist >> 12) &1)==1){
            tcg_gen_qemu_ld_tl(cpu_r[12], loadaddress, ctx->mem_idx, MO_BEUL);
            tcg_gen_addi_i32(loadaddress, loadaddress, 4);
        }
    }

    for(int i = 11; i>= 0; i--){
        if(((reglist >> i) & 1)  == 1){
            tcg_gen_qemu_ld_tl(cpu_r[i], loadaddress, ctx->mem_idx, MO_BEUL);
            tcg_gen_addi_i32(loadaddress, loadaddress, 4);
        }
    }
    if(a->op == 1){
        if(a->rp == PC_REG){
            tcg_gen_mov_i32(cpu_r[SP_REG], loadaddress);
        }
        else{
            tcg_gen_mov_i32(cpu_r[a->rp], loadaddress);
        }
    }

    if(setFlags){
        tcg_gen_movi_i32(cpu_sflags[sflagV], 0);
        tcg_gen_movi_i32(cpu_sflags[sflagC], 0);
        TCGv res = tcg_temp_new_i32();
        tcg_gen_mov_i32(res, cpu_r[12]);
        tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], res, 0);
        tcg_gen_shri_i32(cpu_sflags[sflagN], res, 31);
    }


    ctx->base.pc_next += 4;
    return true;
}

//TODO: add tests
/* ldmts/stmts work on the application context: SP is SP_APP */
static void gen_app_reg_ld(DisasContext *ctx, int reg, TCGv addr)
{
    if (reg == AVR32_SP_REG && ctx->priv) {
        TCGv t = tcg_temp_new_i32();

        tcg_gen_qemu_ld_i32(t, addr, ctx->mem_idx, MO_BEUL);
        tcg_gen_st_i32(t, tcg_env, offsetof(CPUAVR32State, sp_app));
    } else {
        tcg_gen_qemu_ld_i32(cpu_r[reg], addr, ctx->mem_idx, MO_BEUL);
    }
}

static void gen_app_reg_st(DisasContext *ctx, int reg, TCGv addr)
{
    if (reg == AVR32_SP_REG && ctx->priv) {
        TCGv t = tcg_temp_new_i32();

        tcg_gen_ld_i32(t, tcg_env, offsetof(CPUAVR32State, sp_app));
        tcg_gen_qemu_st_i32(t, addr, ctx->mem_idx, MO_BEUL);
    } else {
        tcg_gen_qemu_st_i32(cpu_r[reg], addr, ctx->mem_idx, MO_BEUL);
    }
}

static bool trans_LDMTS(DisasContext *ctx, arg_LDMTS *a)
{
    TCGv addr = tcg_temp_new_i32();

    tcg_gen_mov_i32(addr, cpu_r[a->rp]);
    for (int i = 15; i >= 0; i--) {
        if ((a->list >> i) & 1) {
            gen_app_reg_ld(ctx, i, addr);
            tcg_gen_addi_i32(addr, addr, 4);
        }
    }
    if (a->op) {
        tcg_gen_mov_i32(cpu_r[a->rp], addr);
    }
    ctx->base.pc_next += 4;
    return true;
}

static bool trans_STMTS(DisasContext *ctx, arg_STMTS *a)
{
    TCGv addr = tcg_temp_new_i32();

    tcg_gen_mov_i32(addr, cpu_r[a->rp]);
    if (a->op) {
        for (int i = 0; i <= 15; i++) {
            if ((a->list >> i) & 1) {
                tcg_gen_subi_i32(addr, addr, 4);
                gen_app_reg_st(ctx, i, addr);
            }
        }
        tcg_gen_mov_i32(cpu_r[a->rp], addr);
    } else {
        for (int i = 15; i >= 0; i--) {
            if ((a->list >> i) & 1) {
                gen_app_reg_st(ctx, i, addr);
                tcg_gen_addi_i32(addr, addr, 4);
            }
        }
    }
    ctx->base.pc_next += 4;
    return true;
}

//TODO: add more tests
static bool trans_LDSWPSH(DisasContext *ctx, arg_LDSWPSH *a){
    TCGv temp = tcg_temp_new_i32();
    TCGv upper = tcg_temp_new_i32();
    TCGv lower = tcg_temp_new_i32();
    TCGv addr = tcg_temp_new_i32();

    int disp = a->disp12;
    //Sign extend
    if(disp >> 12){
        disp |= 0xFFFFF000;
    }
    disp = disp << 1;
    tcg_gen_addi_i32(addr, cpu_r[a->rp], disp);

    tcg_gen_qemu_ld_tl(temp, addr, ctx->mem_idx, MO_BEUW);
    tcg_gen_andi_i32(lower, temp, 0x000000FF);
    tcg_gen_shli_i32(lower, lower, 0x8);
    tcg_gen_shri_i32(upper, temp, 0x8);
    tcg_gen_movi_i32(temp, 0);
    tcg_gen_or_i32(temp, temp, lower);
    tcg_gen_or_i32(temp, temp, upper);

    tcg_gen_ext16s_i32(cpu_r[a->rd], temp);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_LDSWPUH(DisasContext *ctx, arg_LDSWPUH *a){
    TCGv temp = tcg_temp_new_i32();
    TCGv upper = tcg_temp_new_i32();
    TCGv lower = tcg_temp_new_i32();
    TCGv addr = tcg_temp_new_i32();

    int disp = a->disp12;
    //Sign extend
    if(disp >> 12){
        disp |= 0xFFFFF000;
    }
    disp = disp << 1;
    tcg_gen_addi_i32(addr, cpu_r[a->rp], disp);

    tcg_gen_qemu_ld_tl(temp, addr, ctx->mem_idx, MO_BEUW);
    tcg_gen_andi_i32(lower, temp, 0x000000FF);
    tcg_gen_shli_i32(lower, lower, 0x8);
    tcg_gen_shri_i32(upper, temp, 0x8);
    tcg_gen_movi_i32(temp, 0);
    tcg_gen_or_i32(temp, temp, lower);
    tcg_gen_or_i32(temp, temp, upper);

    tcg_gen_mov_i32(cpu_r[a->rd], temp);

    ctx->base.pc_next += 4;
    return true;
}


static bool trans_LDSWPW(DisasContext *ctx, arg_LDSWPW *a){
    TCGv temp = tcg_temp_new_i32();
    TCGv top = tcg_temp_new_i32();
    TCGv high = tcg_temp_new_i32();
    TCGv upper = tcg_temp_new_i32();
    TCGv lower = tcg_temp_new_i32();
    TCGv addr = tcg_temp_new_i32();

    int disp = a->disp12;
    //Sign extend
    if(disp >> 12){
        disp |= 0xFFFFF000;
    }
    disp = disp << 2;
    tcg_gen_addi_i32(addr, cpu_r[a->rp], disp);

    tcg_gen_qemu_ld_tl(temp, addr, ctx->mem_idx, MO_BEUL);
    tcg_gen_andi_i32(lower, temp, 0x000000FF);
    tcg_gen_andi_i32(upper, temp, 0x0000FF00);
    tcg_gen_andi_i32(high, temp, 0x00FF0000);
    tcg_gen_andi_i32(top, temp, 0xFF000000);

    tcg_gen_shli_i32(lower, lower, 24);
    tcg_gen_shli_i32(upper, upper, 8);
    tcg_gen_shri_i32(high, high, 8);
    tcg_gen_shri_i32(top, top, 24);

    tcg_gen_movi_i32(temp, 0);
    tcg_gen_or_i32(temp, temp, lower);
    tcg_gen_or_i32(temp, temp, upper);
    tcg_gen_or_i32(temp, temp, high);
    tcg_gen_or_i32(temp, temp, top);

    tcg_gen_mov_i32(cpu_r[a->rd], temp);

    ctx->base.pc_next += 4;
    return true;
}


static void lslr_set_flags(TCGv shamt, TCGv res, TCGv op, bool isLSR){
    // Z-Flag
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], res, 0);

    /// N-Flag
    tcg_gen_shri_i32(res, res, 31);
    tcg_gen_mov_i32(cpu_sflags[sflagN], res);

    // C-Flag
    TCGLabel *setZero = gen_new_label();
    TCGLabel *end = gen_new_label();
    TCGv temp = tcg_temp_new_i32();

    tcg_gen_brcondi_i32(TCG_COND_EQ, shamt, 0, setZero);
    tcg_gen_movi_i32(temp, 32);
    if(isLSR){
        tcg_gen_subi_i32(temp, shamt, 1);

    }else{
        tcg_gen_sub_i32(temp, temp, shamt);
    }
    tcg_gen_shr_i32(temp, op, temp);
    tcg_gen_andi_i32(cpu_sflags[sflagC], temp, 0x00000001);
    tcg_gen_br(end);

    gen_set_label(setZero);
    tcg_gen_movi_i32(cpu_sflags[sflagC], 0);

    gen_set_label(end);

}

static bool trans_LSL_f1(DisasContext *ctx, arg_LSL_f1 *a){
    TCGv rx = tcg_temp_new_i32();
    tcg_gen_mov_i32(rx, cpu_r[a->rx]);
    TCGv Ry = tcg_temp_new_i32();
    TCGv res = tcg_temp_new_i32();
    tcg_gen_andi_i32(Ry, cpu_r[a->ry], 0x0000001F);
    tcg_gen_shl_i32(cpu_r[a->rd], rx, Ry);

    tcg_gen_mov_i32(res, cpu_r[a->rd]);

    lslr_set_flags(Ry, res, rx, false);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_LSL_f2(DisasContext *ctx, arg_LSL_f2 *a){
    int amount = a->bp4 << 1;
    amount |= a->bp1;

    TCGv res = tcg_temp_new_i32();
    TCGv Rd = tcg_temp_new_i32();
    TCGv sa = tcg_temp_new_i32();

    tcg_gen_movi_i32(sa, amount);
    tcg_gen_mov_i32(Rd, cpu_r[a->rd]);
    tcg_gen_shli_i32(cpu_r[a->rd], Rd, amount);
    tcg_gen_mov_i32(res, cpu_r[a->rd]);

    lslr_set_flags(sa, res, Rd, false);

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_LSL_f3(DisasContext *ctx, arg_LSL_f3 *a){
    TCGv res = tcg_temp_new_i32();
    TCGv Rs = tcg_temp_new_i32();
    TCGv sa = tcg_temp_new_i32();

    tcg_gen_movi_i32(sa, a->sa5);
    tcg_gen_mov_i32(Rs, cpu_r[a->rs]);
    tcg_gen_shl_i32(cpu_r[a->rd], Rs, sa);
    tcg_gen_mov_i32(res, cpu_r[a->rd]);

    lslr_set_flags(sa, res, Rs, false);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_LSR_f1(DisasContext *ctx, arg_LSR_f1 *a){
    TCGv rx = tcg_temp_new_i32();
    tcg_gen_mov_i32(rx, cpu_r[a->rx]);
    TCGv Ry = tcg_temp_new_i32();
    TCGv res = tcg_temp_new_i32();
    TCGv op = tcg_temp_new_i32();
    tcg_gen_andi_i32(Ry, cpu_r[a->ry], 0x0000001F);
    tcg_gen_shr_i32(cpu_r[a->rd], rx, Ry);

    tcg_gen_mov_i32(res, cpu_r[a->rd]);
    tcg_gen_mov_i32(op, cpu_r[a->rx]);

    lslr_set_flags(Ry, res, op, true);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_LSR_f2(DisasContext *ctx, arg_LSR_f2 *a){
    int amount = a->bp4 << 1;
    amount |= a->bp1;

    TCGv res = tcg_temp_new_i32();
    TCGv Rd = tcg_temp_new_i32();
    TCGv sa = tcg_temp_new_i32();

    tcg_gen_movi_i32(sa, amount);
    tcg_gen_mov_i32(Rd, cpu_r[a->rd]);
    tcg_gen_shri_i32(cpu_r[a->rd], cpu_r[a->rd], amount);
    tcg_gen_mov_i32(res, cpu_r[a->rd]);

    lslr_set_flags(sa, res, Rd, true);

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_LSR_f3(DisasContext *ctx, arg_LSR_f3 *a){
    TCGv res = tcg_temp_new_i32();
    TCGv Rs = tcg_temp_new_i32();
    TCGv sa = tcg_temp_new_i32();

    tcg_gen_movi_i32(sa, a->sa5);
    tcg_gen_mov_i32(Rs, cpu_r[a->rs]);
    tcg_gen_shr_i32(cpu_r[a->rd], Rs, sa);
    tcg_gen_mov_i32(res, cpu_r[a->rd]);

    lslr_set_flags(sa, res, Rs, true);

    ctx->base.pc_next += 4;
    return true;
}


static bool trans_MAC(DisasContext *ctx, arg_MAC *a){
    TCGv temp = tcg_temp_new_i32();
    tcg_gen_mul_i32(temp, cpu_r[a->rx],cpu_r[a->ry]);
    tcg_gen_add_i32(cpu_r[a->rd], temp, cpu_r[a->rd]);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_MACHHD(DisasContext *ctx, arg_MACHHD *a){
    TCGv_i64 operand1 = tcg_temp_new_i64();
    TCGv_i64 operand2 = tcg_temp_new_i64();
    TCGv_i64 rdp = tcg_temp_new_i64();
    TCGv_i64 rd = tcg_temp_new_i64();
    TCGv_i64 rx = tcg_temp_new_i64();
    TCGv_i64 ry = tcg_temp_new_i64();
    TCGv_i64 res = tcg_temp_new_i64();
    TCGv_i64 dwr = tcg_temp_new_i64();
    tcg_gen_extu_i32_i64(rdp, cpu_r[a->rd+1]);
    tcg_gen_extu_i32_i64(rd, cpu_r[a->rd]);
    tcg_gen_extu_i32_i64(rx, cpu_r[a->rx]);
    tcg_gen_extu_i32_i64(ry, cpu_r[a->ry]);

    if(a->x == 1){
        tcg_gen_shri_i64(operand1, rx, 16);
        tcg_gen_ext16s_i64(operand1, operand1);
    }
    else{
        tcg_gen_andi_i64(operand1, rx, 0x0000FFFF);
        tcg_gen_ext16s_i64(operand1, operand1);
    }
    if(a->y == 1){
        tcg_gen_shri_i64(operand2, ry, 16);
        tcg_gen_ext16s_i64(operand2, operand2);
    }
    else{
        tcg_gen_andi_i64(operand2, ry, 0x0000FFFF);
        tcg_gen_ext16s_i64(operand2, operand2);
    }

    tcg_gen_mul_i64(res, operand1, operand2);

    tcg_gen_mov_i64(dwr, rdp);
    tcg_gen_shli_i64(dwr, dwr, 32);
    tcg_gen_or_i64(dwr, dwr, rd);
    tcg_gen_shri_i64(dwr, dwr, 16);
    tcg_gen_add_i64(dwr, dwr, res);
    tcg_gen_shli_i64(dwr, dwr, 16);
    tcg_gen_extr_i64_i32(cpu_r[a->rd], cpu_r[a->rd+1], dwr);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_MACHHW(DisasContext *ctx, arg_MACHHW *a){
    TCGv operand1 = tcg_temp_new_i32();
    TCGv operand2 = tcg_temp_new_i32();
    TCGv rd = tcg_temp_new_i32();

    if(a->x == 1){
        tcg_gen_shri_i32(operand1, cpu_r[a->rx], 16);
        tcg_gen_ext16s_i32(operand1, operand1);
    }
    else{
        tcg_gen_andi_i32(operand1, cpu_r[a->rx], 0x0000FFFF);
        tcg_gen_ext16s_i32(operand1, operand1);
    }
    if(a->y == 1){
        tcg_gen_shri_i32(operand2, cpu_r[a->ry], 16);
        tcg_gen_ext16s_i32(operand2, operand2);
    }
    else{
        tcg_gen_andi_i32(operand2, cpu_r[a->ry], 0x0000FFFF);
        tcg_gen_ext16s_i32(operand2, operand2);
    }

    tcg_gen_mov_i32(rd, cpu_r[a->rd]);
    tcg_gen_mul_i32(cpu_r[a->rd], operand1, operand2);
    tcg_gen_add_i32(cpu_r[a->rd], cpu_r[a->rd], rd);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_MACSD(DisasContext *ctx, arg_MACSD*a){
    TCGv_i64 rdp = tcg_temp_new_i64();
    TCGv_i64 rd = tcg_temp_new_i64();
    TCGv_i64 acc = tcg_temp_new_i64();
    TCGv_i64 prod64 = tcg_temp_new_i64();
    TCGv_i64 res = tcg_temp_new_i64();
    TCGv_i64 rx = tcg_temp_new_i64();
    TCGv_i64 ry = tcg_temp_new_i64();
    tcg_gen_ext_i32_i64(rdp, cpu_r[a->rd+1]);
    tcg_gen_ext_i32_i64(rd, cpu_r[a->rd]);
    tcg_gen_ext_i32_i64(rx, cpu_r[a->rx]);
    tcg_gen_ext_i32_i64(ry, cpu_r[a->ry]);
    tcg_gen_shli_i64(acc, rdp, 32);
    tcg_gen_add_i64(acc, acc, rd);

    tcg_gen_mul_i64(prod64, rx, ry);

    tcg_gen_add_i64(res, prod64, acc);
    tcg_gen_extr_i64_i32(cpu_r[a->rd], cpu_r[a->rd+1], res);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_MACSATHHW(DisasContext *ctx, arg_MACSATHHW *a) {
    TCGv operand1 = tcg_temp_new_i32();
    TCGv operand2 = tcg_temp_new_i32();

    //TODO: Move segment to helper function. Also in other translation functions.
    if(a->x == 1){
        tcg_gen_shri_i32(operand1, cpu_r[a->rx], 16);
        tcg_gen_ext16s_i32(operand1, operand1);
    }
    else{
        tcg_gen_andi_i32(operand1, cpu_r[a->rx], 0x0000FFFF);
        tcg_gen_ext16s_i32(operand1, operand1);
    }
    if(a->y == 1){
        tcg_gen_shri_i32(operand2, cpu_r[a->ry], 16);
        tcg_gen_ext16s_i32(operand2, operand2);
    }
    else{
        tcg_gen_andi_i32(operand2, cpu_r[a->ry], 0x0000FFFF);
        tcg_gen_ext16s_i32(operand2, operand2);
    }

    gen_helper_macsathhw(tcg_env, tcg_constant_i32(a->rd), operand1, operand2);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_MACUD(DisasContext *ctx, arg_MACUD *a){
    TCGv_i64 rdp = tcg_temp_new_i64();
    TCGv_i64 rd = tcg_temp_new_i64();
    TCGv_i64 acc = tcg_temp_new_i64();
    TCGv_i64 prod64 = tcg_temp_new_i64();
    TCGv_i64 res = tcg_temp_new_i64();
    TCGv_i64 rx = tcg_temp_new_i64();
    TCGv_i64 ry = tcg_temp_new_i64();
    tcg_gen_extu_i32_i64(rdp, cpu_r[a->rd+1]);
    tcg_gen_extu_i32_i64(rd, cpu_r[a->rd]);
    tcg_gen_extu_i32_i64(rx, cpu_r[a->rx]);
    tcg_gen_extu_i32_i64(ry, cpu_r[a->ry]);
    tcg_gen_shli_i64(acc, rdp, 32);
    tcg_gen_add_i64(acc, acc, rd);

    tcg_gen_mul_i64(prod64, rx, ry);

    tcg_gen_add_i64(res, prod64, acc);
    tcg_gen_extr_i64_i32(cpu_r[a->rd], cpu_r[a->rd+1], res);

    ctx->base.pc_next += 4;
    return true;
}

//TODO: implement MACWH.D insn

static bool trans_MAX(DisasContext *ctx, arg_MAX *a){
    TCGLabel *if_1 = gen_new_label();
    TCGLabel *else_1 = gen_new_label();
    TCGLabel *exit = gen_new_label();

    tcg_gen_brcond_i32(TCG_COND_GT, cpu_r[a->rx], cpu_r[a->ry], if_1);
    tcg_gen_br(else_1);

    gen_set_label(if_1);
    tcg_gen_mov_i32(cpu_r[a->rd], cpu_r[a->rx]);
    tcg_gen_br(exit);

    gen_set_label(else_1);
    tcg_gen_mov_i32(cpu_r[a->rd], cpu_r[a->ry]);

    gen_set_label(exit);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_MCALL(DisasContext *ctx, arg_MCALL *a){
    TCGv PC = cpu_r[PC_REG];

    TCGv Rp = tcg_temp_new_i32();
    tcg_gen_mov_i32(Rp, cpu_r[a->rp]);
    tcg_gen_addi_i32(cpu_r[LR_REG], cpu_r[PC_REG], 0x4);
    tcg_gen_andi_i32(Rp, Rp, 0xFFFFFFFC);

    TCGv disp = tcg_temp_new_i32();
    int dispI = a->disp;
    if(dispI >> 15 == 1){
        dispI |= 0xFFFF0000;
    }

    tcg_gen_movi_i32(disp, dispI);
    tcg_gen_shli_i32(disp, disp, 2);

    tcg_gen_add_i32(Rp, Rp, disp);
    tcg_gen_qemu_ld_tl(PC, Rp, ctx->mem_idx, MO_BEUL);


    ctx->base.is_jmp = DISAS_JUMP;
    ctx->base.pc_next += 4;
    return true;
}

//TODO: add tests
/* not implemented on the AP7000 (doc32003, 6.2.7) */
static bool trans_MEMC(DisasContext *ctx, arg_MEMC *a)
{
    ctx->base.pc_next += 4;
    gen_exception(ctx, EXCP_AVR32_UNIMPLEMENTED);
    return true;
}

/* not implemented on the AP7000 (doc32003, 6.2.7) */
static bool trans_MEMS_bp5_imm15(DisasContext *ctx, arg_MEMS_bp5_imm15 *a)
{
    ctx->base.pc_next += 4;
    gen_exception(ctx, EXCP_AVR32_UNIMPLEMENTED);
    return true;
}

/* not implemented on the AP7000 (doc32003, 6.2.7) */
static bool trans_MEMT_bp5_imm15(DisasContext *ctx, arg_MEMT_bp5_imm15 *a)
{
    ctx->base.pc_next += 4;
    gen_exception(ctx, EXCP_AVR32_UNIMPLEMENTED);
    return true;
}

static bool trans_MFSR(DisasContext *ctx, arg_MFSR *a)
{
    ctx->base.pc_next += 4;
    if (a->sr != SYSREG_JECR && a->sr != SYSREG_JOSP && !gen_priv_check(ctx)) {
        return true;
    }
    gen_helper_mfsr(cpu_r[a->rd], tcg_env, tcg_constant_i32(a->sr));
    return true;
}

static bool trans_MIN(DisasContext *ctx, arg_MIN *a){
    TCGLabel *if_1 = gen_new_label();
    TCGLabel *else_1 = gen_new_label();
    TCGLabel *exit = gen_new_label();

    tcg_gen_brcond_i32(TCG_COND_LT, cpu_r[a->rx], cpu_r[a->ry], if_1);
    tcg_gen_br(else_1);

    gen_set_label(if_1);
    tcg_gen_mov_i32(cpu_r[a->rd], cpu_r[a->rx]);
    tcg_gen_br(exit);

    gen_set_label(else_1);
    tcg_gen_mov_i32(cpu_r[a->rd], cpu_r[a->ry]);

    gen_set_label(exit);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_MOV_f1(DisasContext *ctx, arg_MOV_f1 *a){
    int mmI = a->imm8;
    if(mmI >> 7){
        mmI |= 0xFFFFFF00;
    }

    tcg_gen_movi_tl(cpu_r[a->rd], mmI);

    if(a->rd == PC_REG){
        ctx->base.is_jmp = DISAS_JUMP;
    }

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_MOV_f2(DisasContext *ctx, arg_MOV_f2 *a){
    int mmI = ( a->immu) <<17;
    mmI |= (a->immm) << 16;
    mmI |= (a->imml);

    //Sign extend
    if(mmI >> 20){
        mmI |= 0xFFE00000;
    }
    tcg_gen_movi_i32(cpu_r[a->rd], mmI);

    if(a->rd == PC_REG){
        ctx->base.is_jmp = DISAS_JUMP;
    }

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_MOV_f3(DisasContext *ctx, arg_MOV_f3 *a){
    tcg_gen_mov_tl(cpu_r[a->rd], cpu_r[a->rs]);

    if(a->rd == PC_REG){
        ctx->base.is_jmp = DISAS_JUMP;
    }
    ctx->base.pc_next += 2;
    return true;
}

static bool trans_MOVc_f1(DisasContext *ctx, arg_MOVc_f1 *a){
    TCGLabel *no_move = gen_new_label();
    TCGv reg = tcg_temp_new_i32();
    int val = checkCondition(a->cond4, reg, cpu_r, cpu_sflags);

    tcg_gen_brcondi_i32(TCG_COND_NE, reg, val, no_move);
    tcg_gen_mov_i32(cpu_r[a->rd], cpu_r[a->rs]);

    gen_set_label(no_move);
    if(a->rd == PC_REG){
        ctx->base.is_jmp = DISAS_JUMP;
    }
    ctx->base.pc_next += 4;
    return true;
}

static bool trans_MOVc_f2(DisasContext *ctx, arg_MOVc_f2 *a){
    int imm = sign_extend_8(a->imm8);


    TCGLabel *no_move = gen_new_label();
    TCGv reg = tcg_temp_new_i32();
    int val = checkCondition(a->cond4, reg, cpu_r, cpu_sflags);

    tcg_gen_brcondi_i32(TCG_COND_NE, reg, val, no_move);
    tcg_gen_movi_i32(cpu_r[a->rd], imm);

    gen_set_label(no_move);

    if(a->rd == PC_REG){
        ctx->base.is_jmp = DISAS_JUMP;
    }
    ctx->base.pc_next += 4;
    return true;
}

static bool trans_MOVH(DisasContext *ctx, arg_MOVH *a){
    tcg_gen_movi_tl(cpu_r[a->rd], a->imm16);
    tcg_gen_shli_i32(cpu_r[a->rd], cpu_r[a->rd], 0x10);

    if(a->rd == PC_REG){
        ctx->base.is_jmp = DISAS_JUMP;
    }
    ctx->base.pc_next += 4;
    return true;
}

static bool trans_MTDR(DisasContext *ctx, arg_MTDR *a)
{
    ctx->base.pc_next += 4;
    gen_priv_check(ctx);
    return true;
}

static bool trans_MTSR(DisasContext *ctx, arg_MTSR *a)
{
    ctx->base.pc_next += 4;
    if (a->sr != SYSREG_JECR && a->sr != SYSREG_JOSP && !gen_priv_check(ctx)) {
        return true;
    }
    gen_helper_mtsr(tcg_env, tcg_constant_i32(a->sr), cpu_r[a->rs]);
    ctx->base.is_jmp = DISAS_UPDATE;
    return true;
}

static bool trans_MUL_f1(DisasContext *ctx, arg_MUL_f1 *a){
    tcg_gen_mul_i32(cpu_r[a->rd], cpu_r[a->rd], cpu_r[a->rs]);

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_MUL_f2(DisasContext *ctx, arg_MUL_f2 *a){
    tcg_gen_mul_i32(cpu_r[a->rd], cpu_r[a->rx], cpu_r[a->ry]);
    ctx->base.pc_next +=4;
    return true;
}

static bool trans_MUL_f3(DisasContext *ctx, arg_MUL_f3 *a){
    int imm = sign_extend_8(a->imm8);
    tcg_gen_muli_i32(cpu_r[a->rd], cpu_r[a->rs], imm);
    ctx->base.pc_next +=4;
    return true;
}

static bool trans_MULHHW(DisasContext *ctx, arg_MULHHW *a){
    TCGv op1 = tcg_temp_new_i32();
    TCGv op2 = tcg_temp_new_i32();
    if(a->x == 1){
        tcg_gen_shri_i32(op1, cpu_r[a->rx], 0x10);
    }
    else{
        tcg_gen_andi_i32(op1, cpu_r[a->rx], 0x0000FFFF);
    }
    tcg_gen_ext16s_i32(op1, op1);

    if(a->y == 1){
        tcg_gen_shri_i32(op2, cpu_r[a->ry], 0x10);
    }
    else{
        tcg_gen_andi_i32(op2, cpu_r[a->ry], 0x0000FFFF);
    }
    tcg_gen_ext16s_i32(op2, op2);

    tcg_gen_mul_i32(cpu_r[a->rd], op1, op2);

    ctx->base.pc_next +=4;
    return true;
}

static bool trans_MULUD(DisasContext *ctx, arg_MULUD *a){
    TCGv Rd = cpu_r[a->rd];
    TCGv Rdp = cpu_r[a->rd+1];

    tcg_gen_mulu2_i32(Rd, Rdp, cpu_r[a->rx], cpu_r[a->ry]);

    ctx->base.pc_next +=4;
    return true;
}

static bool trans_MUSFR(DisasContext *ctx, arg_MUSFR *a){
    tcg_gen_mov_i32(cpu_sflags[sflagC], cpu_r[a->rs]);
    tcg_gen_andi_i32(cpu_sflags[sflagC], cpu_sflags[sflagC], 0x1);

    tcg_gen_mov_i32(cpu_sflags[sflagZ], cpu_r[a->rs]);
    tcg_gen_shri_i32(cpu_sflags[sflagZ], cpu_sflags[sflagZ], 0x1);
    tcg_gen_andi_i32(cpu_sflags[sflagZ], cpu_sflags[sflagZ], 0x1);

    tcg_gen_mov_i32(cpu_sflags[sflagN], cpu_r[a->rs]);
    tcg_gen_shri_i32(cpu_sflags[sflagN], cpu_sflags[sflagN], 0x2);
    tcg_gen_andi_i32(cpu_sflags[sflagN], cpu_sflags[sflagN], 0x1);

    tcg_gen_mov_i32(cpu_sflags[sflagV], cpu_r[a->rs]);
    tcg_gen_shri_i32(cpu_sflags[sflagV], cpu_sflags[sflagV], 0x3);
    tcg_gen_andi_i32(cpu_sflags[sflagV], cpu_sflags[sflagV], 0x1);

    ctx->base.pc_next +=2;
    return true;
}

static bool trans_MUSTR(DisasContext *ctx, arg_MUSTR *a){
    TCGv temp = tcg_temp_new_i32();

    tcg_gen_mov_i32(temp, cpu_sflags[sflagV]);
    tcg_gen_shli_i32(temp, temp, 3);
    tcg_gen_mov_i32(cpu_r[a->rd], temp);

    tcg_gen_mov_i32(temp, cpu_sflags[sflagN]);
    tcg_gen_shli_i32(temp, temp, 2);
    tcg_gen_or_i32(cpu_r[a->rd], cpu_r[a->rd],temp);

    tcg_gen_mov_i32(temp, cpu_sflags[sflagZ]);
    tcg_gen_shli_i32(temp, temp, 1);
    tcg_gen_or_i32(cpu_r[a->rd], cpu_r[a->rd],temp);

    tcg_gen_mov_i32(temp, cpu_sflags[sflagC]);
    tcg_gen_or_i32(cpu_r[a->rd], cpu_r[a->rd],temp);

    ctx->base.pc_next +=2;
    return true;
}

static bool trans_NEG(DisasContext *ctx, arg_NEG *a){
    TCGv zero = tcg_temp_new_i32();
    TCGv rd = tcg_temp_new_i32();
    TCGv res = tcg_temp_new_i32();

    tcg_gen_movi_i32(zero, 0);
    tcg_gen_mov_i32(rd, cpu_r[a->rd]);
    tcg_gen_sub_i32(res, zero, rd);
    tcg_gen_mov_i32(cpu_r[a->rd], res);

    tcg_gen_shri_i32(cpu_sflags[sflagN], res, 31);
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], res, 0);
    tcg_gen_shri_i32(res, res, 31);
    tcg_gen_shri_i32(rd, rd, 31);

    tcg_gen_and_i32(cpu_sflags[sflagV], rd, res);
    tcg_gen_or_i32(cpu_sflags[sflagC], rd, res);

    ctx->base.pc_next += 2;
    return true;
}
static bool trans_NOP(DisasContext *ctx, arg_NOP *a){

    ctx->base.pc_next += 2;
    return true;
}


static bool trans_OR_f1(DisasContext *ctx, arg_OR_f1 *a){
    tcg_gen_or_i32(cpu_r[a->rd], cpu_r[a->rd], cpu_r[a->rs]);
    TCGv res = tcg_temp_new_i32();
    tcg_gen_mov_i32(res, cpu_r[a->rd]);
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], res, 0);
    tcg_gen_shri_i32(cpu_sflags[sflagN], res, 31);

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_OR_f2(DisasContext *ctx, arg_OR_f2 *a){
    TCGv ry = tcg_temp_new_i32();
    tcg_gen_mov_i32(ry, cpu_r[a->ry]);
    tcg_gen_shli_i32(ry, ry, a->sa5);
    tcg_gen_or_i32(cpu_r[a->rd], cpu_r[a->rx], ry);

    TCGv res = tcg_temp_new_i32();
    tcg_gen_mov_i32(res, cpu_r[a->rd]);
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], res, 0);
    tcg_gen_shri_i32(cpu_sflags[sflagN], res, 31);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_OR_f3(DisasContext *ctx, arg_OR_f2 *a){
    TCGv ry = tcg_temp_new_i32();
    tcg_gen_mov_i32(ry, cpu_r[a->ry]);
    tcg_gen_shri_i32(ry, ry, a->sa5);
    tcg_gen_or_i32(cpu_r[a->rd], cpu_r[a->rx], ry);

    TCGv res = tcg_temp_new_i32();
    tcg_gen_mov_i32(res, cpu_r[a->rd]);
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], res, 0);
    tcg_gen_shri_i32(cpu_sflags[sflagN], res, 31);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_OR_cond(DisasContext *ctx, arg_OR_cond *a){
    TCGLabel *no_op = gen_new_label();

    TCGv reg = tcg_temp_new_i32();
    int val = checkCondition(a->cond4, reg, cpu_r, cpu_sflags);

    tcg_gen_brcondi_i32(TCG_COND_NE, reg, val, no_op);

    tcg_gen_or_i32(cpu_r[a->rd], cpu_r[a->rx], cpu_r[a->ry]);

    gen_set_label(no_op);
    ctx->base.pc_next += 4;
    return true;
}

static bool trans_ORH(DisasContext *ctx, arg_ORH *a){
    TCGv imm = tcg_temp_new_i32();
    tcg_gen_movi_i32(imm, a->imm16);
    tcg_gen_shli_i32(imm, imm, 16);
    tcg_gen_or_i32(cpu_r[a->rd], cpu_r[a->rd], imm);

    tcg_gen_shri_i32(cpu_sflags[sflagN], cpu_r[a->rd], 31);
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], cpu_r[a->rd], 0);


    ctx->base.pc_next += 4;
    return true;
}

static bool trans_ORL(DisasContext *ctx, arg_ORH *a){
    TCGv imm = tcg_temp_new_i32();
    tcg_gen_movi_i32(imm, a->imm16);
    tcg_gen_or_i32(cpu_r[a->rd], cpu_r[a->rd], imm);

    tcg_gen_shri_i32(cpu_sflags[sflagN], cpu_r[a->rd], 31);
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], cpu_r[a->rd], 0);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_POPM(DisasContext *ctx, arg_POPM *a){
    bool setFlags = false;

    if (((a->list >> 8) & 1) == 1 && (((a->list >> 0) & 1) == 1)){
        tcg_gen_qemu_ld_tl(cpu_r[PC_REG], cpu_r[SP_REG], ctx->mem_idx, MO_BEUL);
        tcg_gen_addi_i32(cpu_r[SP_REG], cpu_r[SP_REG], 0x4);
        ctx->base.is_jmp = DISAS_JUMP;

        if (((a->list >> 6) & 1) == 0 && ((a->list >> 7) & 1) == 0){
            tcg_gen_movi_i32(cpu_r[12], 0);
        }
        else if (((a->list >> 6) & 1) == 1 && ((a->list >> 7) & 1) == 0){
            tcg_gen_movi_i32(cpu_r[12], 1);
        }
        else{
            tcg_gen_movi_i32(cpu_r[12], -1);
        }
        setFlags = true;
    }
    else{
        if(((a->list >> 8) & 1) == 1){
            tcg_gen_qemu_ld_tl(cpu_r[PC_REG], cpu_r[SP_REG], ctx->mem_idx, MO_BEUL);
            tcg_gen_addi_i32(cpu_r[SP_REG], cpu_r[SP_REG], 0x4);
            ctx->base.is_jmp = DISAS_JUMP;
        }
        if(((a->list >> 7) & 1) == 1){
            tcg_gen_qemu_ld_tl(cpu_r[LR_REG], cpu_r[SP_REG], ctx->mem_idx, MO_BEUL);
            tcg_gen_addi_i32(cpu_r[SP_REG], cpu_r[SP_REG], 0x4);

        }
        if(((a->list >> 6) & 1) == 1){
            tcg_gen_qemu_ld_tl(cpu_r[12], cpu_r[SP_REG], ctx->mem_idx, MO_BEUL);
            tcg_gen_addi_i32(cpu_r[SP_REG], cpu_r[SP_REG], 0x4);

        }
        if(((a->list >> 8) & 1) == 1){
            setFlags = true;
        }
    }

    if(((a->list >> 5) & 1) == 1){
        tcg_gen_qemu_ld_tl(cpu_r[11], cpu_r[SP_REG], ctx->mem_idx, MO_BEUL);
        tcg_gen_addi_i32(cpu_r[SP_REG], cpu_r[SP_REG], 0x4);

    }
    if(((a->list >> 4) & 1) == 1){
        tcg_gen_qemu_ld_tl(cpu_r[10], cpu_r[SP_REG], ctx->mem_idx, MO_BEUL);
        tcg_gen_addi_i32(cpu_r[SP_REG], cpu_r[SP_REG], 0x4);
    }

    if(((a->list >> 3) & 1) == 1){
        tcg_gen_qemu_ld_tl(cpu_r[9], cpu_r[SP_REG], ctx->mem_idx, MO_BEUL);
        tcg_gen_addi_i32(cpu_r[SP_REG], cpu_r[SP_REG], 0x4);
        tcg_gen_qemu_ld_tl(cpu_r[8], cpu_r[SP_REG], ctx->mem_idx, MO_BEUL);
        tcg_gen_addi_i32(cpu_r[SP_REG], cpu_r[SP_REG], 0x4);
    }

    if(((a->list >> 2) & 1) == 1){
        tcg_gen_qemu_ld_tl(cpu_r[7], cpu_r[SP_REG], ctx->mem_idx, MO_BEUL);
        tcg_gen_addi_i32(cpu_r[SP_REG], cpu_r[SP_REG], 0x4);
        tcg_gen_qemu_ld_tl(cpu_r[6], cpu_r[SP_REG], ctx->mem_idx, MO_BEUL);
        tcg_gen_addi_i32(cpu_r[SP_REG], cpu_r[SP_REG], 0x4);
        tcg_gen_qemu_ld_tl(cpu_r[5], cpu_r[SP_REG], ctx->mem_idx, MO_BEUL);
        tcg_gen_addi_i32(cpu_r[SP_REG], cpu_r[SP_REG], 0x4);
        tcg_gen_qemu_ld_tl(cpu_r[4], cpu_r[SP_REG], ctx->mem_idx, MO_BEUL);
        tcg_gen_addi_i32(cpu_r[SP_REG], cpu_r[SP_REG], 0x4);
    }
    if(((a->list >> 1) & 1) == 1){
        tcg_gen_qemu_ld_tl(cpu_r[3], cpu_r[SP_REG], ctx->mem_idx, MO_BEUL);
        tcg_gen_addi_i32(cpu_r[SP_REG], cpu_r[SP_REG], 0x4);
        tcg_gen_qemu_ld_tl(cpu_r[2], cpu_r[SP_REG], ctx->mem_idx, MO_BEUL);
        tcg_gen_addi_i32(cpu_r[SP_REG], cpu_r[SP_REG], 0x4);
        tcg_gen_qemu_ld_tl(cpu_r[1], cpu_r[SP_REG], ctx->mem_idx, MO_BEUL);
        tcg_gen_addi_i32(cpu_r[SP_REG], cpu_r[SP_REG], 0x4);
        tcg_gen_qemu_ld_tl(cpu_r[0], cpu_r[SP_REG], ctx->mem_idx, MO_BEUL);
        tcg_gen_addi_i32(cpu_r[SP_REG], cpu_r[SP_REG], 0x4);
    }

    if(setFlags){
        tcg_gen_movi_i32(cpu_sflags[sflagV], 0);
        tcg_gen_movi_i32(cpu_sflags[sflagC], 0);
        TCGv res = tcg_temp_new_i32();
        tcg_gen_mov_i32(res, cpu_r[12]);
        tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], res, 0);
        tcg_gen_shri_i32(cpu_sflags[sflagN], res, 31);
    }

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_PUSHM(DisasContext *ctx, arg_PUSHM *a){
    TCGv sp = cpu_r[SP_REG];
    if (((a->list >> 0) & 1) == 1){
        tcg_gen_subi_i32(sp, sp, 0x4);
        tcg_gen_qemu_st_tl(cpu_r[0], sp, ctx->mem_idx, MO_BEUL);
        tcg_gen_subi_i32(sp, sp, 0x4);
        tcg_gen_qemu_st_tl(cpu_r[1], sp, ctx->mem_idx, MO_BEUL);
        tcg_gen_subi_i32(sp, sp, 0x4);
        tcg_gen_qemu_st_tl(cpu_r[2], sp, ctx->mem_idx, MO_BEUL);
        tcg_gen_subi_i32(sp, sp, 0x4);
        tcg_gen_qemu_st_tl(cpu_r[3], sp, ctx->mem_idx, MO_BEUL);
    }
    if (((a->list >> 1) & 1) == 1){
        tcg_gen_subi_i32(sp, sp, 0x4);
        tcg_gen_qemu_st_tl(cpu_r[4], sp, ctx->mem_idx, MO_BEUL);
        tcg_gen_subi_i32(sp, sp, 0x4);
        tcg_gen_qemu_st_tl(cpu_r[5], sp, ctx->mem_idx, MO_BEUL);
        tcg_gen_subi_i32(sp, sp, 0x4);
        tcg_gen_qemu_st_tl(cpu_r[6], sp, ctx->mem_idx, MO_BEUL);
        tcg_gen_subi_i32(sp, sp, 0x4);
        tcg_gen_qemu_st_tl(cpu_r[7], sp, ctx->mem_idx, MO_BEUL);
    }
    if (((a->list >> 2) & 1) == 1) {
        tcg_gen_subi_i32(sp, sp, 0x4);
        tcg_gen_qemu_st_tl(cpu_r[8], sp, ctx->mem_idx, MO_BEUL);
        tcg_gen_subi_i32(sp, sp, 0x4);
        tcg_gen_qemu_st_tl(cpu_r[9], sp, ctx->mem_idx, MO_BEUL);
    }
    if (((a->list >> 3) & 1) == 1) {
        tcg_gen_subi_i32(sp, sp, 0x4);
        tcg_gen_qemu_st_tl(cpu_r[10], sp, ctx->mem_idx, MO_BEUL);
    }
    if (((a->list >> 4) & 1) == 1) {
        tcg_gen_subi_i32(sp, sp, 0x4);
        tcg_gen_qemu_st_tl(cpu_r[11], sp, ctx->mem_idx, MO_BEUL);
    }
    if (((a->list >> 5) & 1) == 1) {
        tcg_gen_subi_i32(sp, sp, 0x4);
        tcg_gen_qemu_st_tl(cpu_r[12], sp, ctx->mem_idx, MO_BEUL);
    }
    if (((a->list >> 6) & 1) == 1) {
        tcg_gen_subi_i32(sp, sp, 0x4);
        tcg_gen_qemu_st_tl(cpu_r[LR_REG], sp, ctx->mem_idx, MO_BEUL);
    }
    if (((a->list >> 7) & 1) == 1) {
        tcg_gen_subi_i32(sp, sp, 0x4);
        tcg_gen_qemu_st_tl(cpu_r[PC_REG], sp, ctx->mem_idx, MO_BEUL);
    }

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_RCALL_f1(DisasContext *ctx, arg_RCALL_f1 *a){
    uint32_t disp = a->disp2 << 8;
    disp |= a->disp8;
    if(disp >> 9 == 1){
        disp |= 0xFFFFFC00;
    }
    disp = disp << 1;

    tcg_gen_addi_i32(cpu_r[AVR32_LR_REG], cpu_r[AVR32_PC_REG], 2);
    tcg_gen_addi_i32(cpu_r[AVR32_PC_REG], cpu_r[AVR32_PC_REG], disp);

    ctx->base.is_jmp = DISAS_JUMP;
    ctx->base.pc_next += 2;
    return true;
}

static bool trans_RCALL_f2(DisasContext *ctx, arg_RCALL_f2 *a){
    uint32_t disp = a->immu << 17;
    disp |= a->immm << 16;
    disp |= a->imml;
    if(disp >> 20 == 1){
        disp |= 0xFFE00000;
    }
    disp = disp << 1;
    tcg_gen_addi_i32(cpu_r[AVR32_LR_REG], cpu_r[AVR32_PC_REG], 4);
    tcg_gen_addi_i32(cpu_r[AVR32_PC_REG], cpu_r[AVR32_PC_REG], disp);

    ctx->base.is_jmp = DISAS_JUMP;
    ctx->base.pc_next += 4;
    return true;
}

static bool trans_RET(DisasContext *ctx, arg_RET *a){
    TCGLabel *no_return = gen_new_label();

    TCGv reg = tcg_temp_new_i32();
    int val = checkCondition(a->cond4, reg, cpu_r, cpu_sflags);

    tcg_gen_brcondi_i32(TCG_COND_NE, reg, val, no_return);

    TCGLabel *leave = gen_new_label();
    if(a->rd != LR_REG && a->rd != SP_REG && a->rd != PC_REG){
        tcg_gen_mov_i32(cpu_r[12], cpu_r[a->rd]);
    }
    else if(a->rd == LR_REG){
        tcg_gen_movi_i32(cpu_r[12], -1);
    }
    else if(a->rd == SP_REG){
        tcg_gen_movi_i32(cpu_r[12], 0);
    }
    else{
        tcg_gen_movi_i32(cpu_r[12], 1);
    }

    TCGv r12 = tcg_temp_new_i32();
    tcg_gen_mov_i32(r12, cpu_r[12]);
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], r12, 0);
    tcg_gen_shri_i32(r12, r12, 31);
    tcg_gen_mov_i32(cpu_sflags[sflagN], r12);
    tcg_gen_movi_i32(cpu_sflags[sflagC], 0);
    tcg_gen_movi_i32(cpu_sflags[sflagV], 0);

    tcg_gen_mov_i32(cpu_r[PC_REG], cpu_r[LR_REG]);
    tcg_gen_br(leave);

    gen_set_label(no_return);
    tcg_gen_addi_i32(cpu_r[PC_REG], cpu_r[PC_REG], 2);

    gen_set_label(leave);
    ctx->base.is_jmp = DISAS_JUMP;
    ctx->base.pc_next += 2;
    return true;
}

static bool trans_RETE(DisasContext *ctx, arg_RETE *a)
{
    ctx->base.pc_next += 2;
    gen_helper_rete(tcg_env);
    ctx->base.is_jmp = DISAS_EXIT;
    return true;
}

static bool trans_RETS(DisasContext *ctx, arg_RETS *a)
{
    ctx->base.pc_next += 2;
    gen_helper_rets(tcg_env);
    ctx->base.is_jmp = DISAS_EXIT;
    return true;
}

static bool trans_RJMP(DisasContext *ctx, arg_RJMP *a){
    int disp = a->disp8;
    disp |= (a->disp2 << 8);
    if(disp >> 9){
        disp |= 0xFFFFFC00;
    }
    disp = disp << 1;

    tcg_gen_addi_i32(cpu_r[PC_REG], cpu_r[PC_REG], disp);

    ctx->base.is_jmp = DISAS_JUMP;
    ctx->base.pc_next += 2;
    return true;
}

static bool trans_ROL(DisasContext *ctx, arg_ROL *a){
    TCGv tempC = tcg_temp_new_i32();
    TCGv res = tcg_temp_new_i32();

    tcg_gen_shri_i32(tempC, cpu_r[a->rd], 31);
    tcg_gen_shli_i32(res, cpu_r[a->rd], 1);
    tcg_gen_add_i32(res, res, cpu_sflags[sflagC]);
    tcg_gen_mov_i32(cpu_sflags[sflagC], tempC);

    tcg_gen_shri_i32(cpu_sflags[sflagN], res, 31);
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], res, 0);
    tcg_gen_mov_i32(cpu_r[a->rd], res);

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_ROR(DisasContext *ctx, arg_ROR *a){
    TCGv tempC = tcg_temp_new_i32();
    tcg_gen_andi_i32(tempC, cpu_r[a->rd], 0x00000001);
    tcg_gen_shri_i32(cpu_r[a->rd], cpu_r[a->rd], 1);
    tcg_gen_shli_i32(cpu_sflags[sflagC], cpu_sflags[sflagC], 31);
    tcg_gen_or_i32(cpu_r[a->rd], cpu_r[a->rd], cpu_sflags[sflagC]);
    tcg_gen_mov_i32(cpu_sflags[sflagC], tempC);

    tcg_gen_shri_i32(cpu_sflags[sflagN], cpu_r[a->rd], 31);
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], cpu_r[a->rd], 0);

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_RSUB_f1(DisasContext *ctx, arg_RSUB_f1 *a){
    TCGv rd = cpu_r[a->rd];
    TCGv rs = cpu_r[a->rs];

    TCGv temp = tcg_temp_new_i32();
    TCGv left = tcg_temp_new_i32();
    TCGv right = tcg_temp_new_i32();
    TCGv res = tcg_temp_new_i32();
    TCGv op1 = tcg_temp_new_i32();
    TCGv op2 = tcg_temp_new_i32();

    tcg_gen_mov_i32(op1, rs);
    tcg_gen_mov_i32(op2, rd);

    tcg_gen_sub_i32(rd, rs, rd);
    tcg_gen_mov_i32(res, rd);

    // Z flag
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], res, 0);

    //N-flag
    tcg_gen_shri_i32(res, res, 31);
    tcg_gen_mov_i32(cpu_sflags[sflagN], res);

    //V-Flag
    tcg_gen_shri_i32(op1, op1, 31);
    tcg_gen_shri_i32(op2, op2, 31);

    tcg_gen_andc_i32(left, op1, op2);
    tcg_gen_andc_i32(left, left, res);

    tcg_gen_andc_i32(right, op2, op1);
    tcg_gen_and_i32(right, right, res);
    tcg_gen_or_i32(cpu_sflags[sflagV], left, right);

    // C-Flag
    tcg_gen_andc_i32(left, op2, op1);
    tcg_gen_and_i32(temp, op2, res);
    tcg_gen_andc_i32(right, res, op1);
    tcg_gen_or_i32(temp, temp, left);
    tcg_gen_or_i32(cpu_sflags[sflagC], temp, right);

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_RSUB_f2(DisasContext *ctx, arg_RSUB_f2 *a){
    int imm8 = a->imm8;
    imm8 = sign_extend_8(imm8);

    TCGv rd = cpu_r[a->rd];
    TCGv rs = cpu_r[a->rs];
    TCGv imm = tcg_temp_new_i32();
    TCGv temp = tcg_temp_new_i32();
    TCGv left = tcg_temp_new_i32();
    TCGv right = tcg_temp_new_i32();
    TCGv res = tcg_temp_new_i32();
    TCGv op1 = tcg_temp_new_i32();
    TCGv op2 = tcg_temp_new_i32();

    tcg_gen_movi_i32(imm, imm8);
    tcg_gen_mov_i32(op1, imm);
    tcg_gen_mov_i32(op2, rs);

    tcg_gen_sub_i32(rd, imm, rs);
    tcg_gen_mov_i32(res, rd);

    // Z flag
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], res, 0);

    //N-flag
    tcg_gen_shri_i32(res, res, 31);
    tcg_gen_mov_i32(cpu_sflags[sflagN], res);

    //V-Flag
    tcg_gen_shri_i32(op1, op1, 31);
    tcg_gen_shri_i32(op2, op2, 31);

    tcg_gen_andc_i32(left, op1, op2);
    tcg_gen_andc_i32(left, left, res);

    tcg_gen_andc_i32(right, op2, op1);
    tcg_gen_and_i32(right, right, res);
    tcg_gen_or_i32(cpu_sflags[sflagV], left, right);

    // C-Flag
    tcg_gen_andc_i32(left, op2, op1);
    tcg_gen_and_i32(temp, op2, res);
    tcg_gen_andc_i32(right, res, op1);
    tcg_gen_or_i32(temp, temp, left);
    tcg_gen_or_i32(cpu_sflags[sflagC], temp, right);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_RSUBc(DisasContext *ctx, arg_RSUBc *a){
    TCGLabel *end = gen_new_label();

    TCGv reg = tcg_temp_new_i32();
    int val = checkCondition(a->cond4, reg, cpu_r, cpu_sflags);

    tcg_gen_brcondi_i32(TCG_COND_NE, reg, val, end);

    int imm = sign_extend_8(a->imm8);
    TCGv imm8 = tcg_temp_new_i32();
    tcg_gen_movi_i32(imm8, imm);
    tcg_gen_sub_i32(cpu_r[a->rd], imm8, cpu_r[a->rd]);


    gen_set_label(end);
    if(a->rd == PC_REG){
        ctx->base.is_jmp = DISAS_JUMP;
    }

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_SATU(DisasContext *ctx, arg_SATU *a){
    TCGLabel *if1 = gen_new_label();
    TCGLabel *if1_else = gen_new_label();
    TCGLabel *exit = gen_new_label();

    TCGLabel *if2 = gen_new_label();
    TCGLabel *if2_else = gen_new_label();

    TCGv temp = tcg_temp_new_i32();
    TCGv mask = tcg_temp_new_i32();
    TCGv subTemp = tcg_temp_new_i32();
    TCGv bp = tcg_temp_new_i32();

    tcg_gen_movi_i32(bp, a->bp5);

    tcg_gen_movi_i32(mask, 0xFFFFFFFF);
    tcg_gen_shri_i32(mask, mask, 31- a->bp5-1);

    tcg_gen_shri_i32(temp, cpu_r[a->rd], a->sa5);
    tcg_gen_and_i32(subTemp, temp, mask);

    tcg_gen_brcond_i32(TCG_COND_EQ, subTemp, temp, if1);
    tcg_gen_brcondi_i32(TCG_COND_EQ, bp, 0, if1);
    tcg_gen_br(if1_else);

    // if
    gen_set_label(if1);
    tcg_gen_mov_i32(cpu_r[a->rd], temp);
    tcg_gen_br(exit);

    // else
    gen_set_label(if1_else);
    tcg_gen_movi_i32(cpu_sflags[sflagQ], 1);
    tcg_gen_shri_i32(temp, temp, 31);
    tcg_gen_brcondi_i32(TCG_COND_EQ, temp, 1, if2);
    tcg_gen_br(if2_else);

    // if2
    gen_set_label(if2);
    tcg_gen_movi_i32(cpu_r[a->rd], 0);
    tcg_gen_br(exit);

    // if2_else
    gen_set_label(if2_else);
    tcg_gen_movi_i32(cpu_r[a->rd], 2);
    tcg_gen_shli_i32(cpu_r[a->rd], cpu_r[a->rd], a->bp5);
    tcg_gen_subi_i32(cpu_r[a->rd], cpu_r[a->rd], 1);

    // exit
    gen_set_label(exit);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_SBC(DisasContext *ctx, arg_SBC *a){
    TCGv res = tcg_temp_new_i32();
    TCGv rx = tcg_temp_new_i32();
    TCGv ry = tcg_temp_new_i32();
    TCGv cond = tcg_temp_new_i32();
    TCGv left = tcg_temp_new_i32();
    TCGv right = tcg_temp_new_i32();

    tcg_gen_mov_i32(rx, cpu_r[a->rx]);
    tcg_gen_mov_i32(ry, cpu_r[a->ry]);

    tcg_gen_sub_i32(res, cpu_r[a->rx], cpu_r[a->ry]);
    tcg_gen_sub_i32(res, res, cpu_sflags[sflagC]);
    tcg_gen_mov_i32(cpu_r[a->rd], res);

    // z-flag
    tcg_gen_setcondi_i32(TCG_COND_EQ, cond, res, 0);
    tcg_gen_and_i32(cpu_sflags[sflagZ], cond, cpu_sflags[sflagZ]);

    // n-flag
    tcg_gen_shri_i32(res, res, 31);
    tcg_gen_mov_i32(cpu_sflags[sflagN], res);

    //V-flag
    tcg_gen_shri_i32(rx, rx, 31);
    tcg_gen_shri_i32(ry, ry, 31);

    tcg_gen_andc_i32(left, rx, ry);
    tcg_gen_andc_i32(left, left, res);
    tcg_gen_andc_i32(right, ry, rx);
    tcg_gen_and_i32(right, right, res);
    tcg_gen_or_i32(cpu_sflags[sflagV], left, right);

    //C-flag
    tcg_gen_andc_i32(left, ry, rx);
    tcg_gen_and_i32(right, ry, res);
    tcg_gen_or_i32(left, left, right);

    tcg_gen_andc_i32(right, res, rx);
    tcg_gen_or_i32(cpu_sflags[sflagC], left, right);


    ctx->base.pc_next += 4;
    return true;
}

static bool trans_SBR(DisasContext *ctx, arg_SBR *a){
    TCGv bp = tcg_temp_new_i32();
    tcg_gen_movi_i32(bp, a->bp4);
    tcg_gen_shli_i32(bp, bp, 1);
    tcg_gen_addi_i32(bp, bp, a->bp1);

    TCGv val = tcg_temp_new_i32();
    tcg_gen_movi_i32(val, 1);
    tcg_gen_shl_i32(val, val, bp);
    tcg_gen_or_i32(cpu_r[a->rd], cpu_r[a->rd], val);

    tcg_gen_movi_i32(cpu_sflags[sflagZ], 0);

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_SCALL(DisasContext *ctx, arg_SCALL *a)
{
    gen_helper_scall(tcg_env, tcg_constant_i32(ctx->base.pc_next + 2));
    ctx->base.pc_next += 2;
    ctx->base.is_jmp = DISAS_EXIT;
    return true;
}


static bool trans_SCR(DisasContext *ctx, arg_SCR *a){
    TCGv res = tcg_temp_new_i32();
    TCGv res31 = tcg_temp_new_i32();
    TCGv rd31 = tcg_temp_new_i32();

    tcg_gen_sub_i32(res, cpu_r[a->rd], cpu_sflags[sflagC]);
    tcg_gen_shri_i32(res31, res, 31);
    tcg_gen_shri_i32(rd31, cpu_r[a->rd], 31);
    tcg_gen_mov_i32(cpu_r[a->rd], res);

    //V-Flag
    tcg_gen_andc_i32(cpu_sflags[sflagV], rd31, res31);

    //N-Flag
    tcg_gen_mov_i32(cpu_sflags[sflagN], res31);

    //Z-Flag
    TCGv temp = tcg_temp_new_i32();
    tcg_gen_setcondi_i32(TCG_COND_EQ, temp, res31, 0);
    tcg_gen_and_i32(cpu_sflags[sflagZ], cpu_sflags[sflagZ], temp);

    //C-Flag
    tcg_gen_andc_i32(cpu_sflags[sflagC],  res31, rd31);

    ctx->base.pc_next += 2;
    return true;
}

//TODO: implement
static bool trans_SLEEP(DisasContext *ctx, arg_SLEEP *a)
{
    ctx->base.pc_next += 4;
    if (gen_priv_check(ctx)) {
        gen_helper_sleep(tcg_env, tcg_constant_i32(a->op8),
                         tcg_constant_i32(ctx->base.pc_next));
        ctx->base.is_jmp = DISAS_NORETURN;
    }
    return true;
}

static bool trans_SR(DisasContext *ctx, arg_SR *a){
    TCGv reg = tcg_temp_new_i32();
    int val = checkCondition(a->cond4, reg, cpu_r, cpu_sflags);

    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_r[a->rd], reg, val);

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_SSRF(DisasContext *ctx, arg_SSRF *a)
{
    ctx->base.pc_next += 2;
    gen_set_sr_bit(ctx, a->bp5, 1);
    return true;
}

static bool trans_STB_f1(DisasContext *ctx, arg_STB_f1 *a){
    TCGv ptr = cpu_r[a->rp];
    TCGv rs = cpu_r[a->rs];
    tcg_gen_qemu_st_tl(rs, ptr, ctx->mem_idx, MO_UB);
    tcg_gen_addi_i32(cpu_r[a->rp], cpu_r[a->rp], 1);

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_STB_f2(DisasContext *ctx, arg_STB_f2 *a){
    TCGv ptr = cpu_r[a->rp];
    TCGv rs = cpu_r[a->rs];
    tcg_gen_subi_i32(ptr, ptr, 0x1);
    tcg_gen_qemu_st_tl(rs, ptr, ctx->mem_idx, MO_UB);

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_STB_f3(DisasContext *ctx, arg_STB_f3 *a){

    TCGv ptr = tcg_temp_new_i32();
    tcg_gen_addi_i32(ptr, cpu_r[a->rp], a->disp3);
    TCGv rs = cpu_r[a->rd];

    tcg_gen_qemu_st_tl(rs, ptr, ctx->mem_idx, MO_UB);

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_STB_f4(DisasContext *ctx, arg_STB_f4 *a){
    int disp = a->imm16;
    if(disp >> 15 == 1){
        disp |= 0xFFFF0000;
    }
    TCGv ptr = tcg_temp_new_i32();
    tcg_gen_addi_i32(ptr, cpu_r[a->rp], disp);
    TCGv rs = cpu_r[a->rs];

    tcg_gen_qemu_st_tl(rs, ptr, ctx->mem_idx, MO_UB);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_STB_f5(DisasContext *ctx, arg_STB_f5 *a){
    TCGv ptr = tcg_temp_new_i32();
    tcg_gen_shli_i32(ptr, cpu_r[a->ry], a->sa);
    tcg_gen_add_i32(ptr, ptr, cpu_r[a->rx]);

    tcg_gen_qemu_st_tl(cpu_r[a->rd], ptr, ctx->mem_idx, MO_UB);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_STBc(DisasContext *ctx, arg_STBc *a){
    TCGLabel *exit = gen_new_label();

    TCGv reg = tcg_temp_new_i32();
    int val = checkCondition(a->cond4, reg, cpu_r, cpu_sflags);
    tcg_gen_brcondi_i32(TCG_COND_NE, reg, val, exit);

    TCGv ptr = tcg_temp_new_i32();
    tcg_gen_addi_i32(ptr, cpu_r[a->rp], a->disp9);
    tcg_gen_qemu_st_tl(cpu_r[a->rd], ptr, ctx->mem_idx, MO_UB);

    gen_set_label(exit);
    ctx->base.pc_next += 4;
    return true;
}

static bool trans_STD_f1(DisasContext *ctx, arg_STD_f1 *a){
    TCGv ptr = cpu_r[a->rp];
    TCGv rs = cpu_r[a->rs*2];
    TCGv rsp = cpu_r[a->rs*2 + 1];

    tcg_gen_qemu_st_i32(rsp, ptr, ctx->mem_idx, MO_BEUL);
    tcg_gen_addi_i32(ptr, ptr, 4);
    tcg_gen_qemu_st_i32(rs, ptr, ctx->mem_idx, MO_BEUL);
    tcg_gen_addi_i32(ptr, ptr, 4);

    ctx->base.pc_next += 2;
    return true;
}


static bool trans_STD_f2(DisasContext *ctx, arg_STD_f2 *a){
    TCGv ptr = cpu_r[a->rp];
    TCGv rs = cpu_r[a->rs*2];
    TCGv rsp = cpu_r[a->rs*2 + 1];

    tcg_gen_subi_i32(ptr, ptr, 0x4);
    tcg_gen_qemu_st_i32(rs, ptr, ctx->mem_idx, MO_BEUL);
    tcg_gen_subi_i32(ptr, ptr, 0x4);
    tcg_gen_qemu_st_i32(rsp, ptr, ctx->mem_idx, MO_BEUL);

    ctx->base.pc_next += 2;
    return true;
}

//TODO: add tests
static bool trans_STD_f3(DisasContext *ctx, arg_STD_f3 *a){
    TCGv ptr = tcg_temp_new_i32();
    TCGv rs = cpu_r[a->rs*2];
    TCGv rsp = cpu_r[a->rs*2 + 1];

    tcg_gen_mov_i32(ptr, cpu_r[a->rp]);

    tcg_gen_qemu_st_tl(rsp, ptr, ctx->mem_idx, MO_BEUL);
    tcg_gen_addi_tl(ptr, ptr, 4);
    tcg_gen_qemu_st_i32(rs, ptr, ctx->mem_idx, MO_BEUL);
    tcg_gen_addi_i32(ptr, ptr, 4);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_STD_f4(DisasContext *ctx, arg_STD_f4 *a){
    TCGv ptr = tcg_temp_new_i32();
    TCGv disp = tcg_temp_new_i32();
    TCGv rs = cpu_r[a->rs*2];
    TCGv rsp = cpu_r[a->rs*2 + 1];

    int dispI = a->disp16;

    //Sign extend
    if(dispI >> 15){
        dispI |= 0xFFFF0000;
    }

    tcg_gen_movi_i32(disp, dispI);
    tcg_gen_add_i32(ptr, cpu_r[a->rp], disp);

    tcg_gen_qemu_st_tl(rsp, ptr, ctx->mem_idx, MO_BEUL);
    tcg_gen_addi_tl(ptr, ptr, 4);
    tcg_gen_qemu_st_i32(rs, ptr, ctx->mem_idx, MO_BEUL);
    tcg_gen_addi_i32(ptr, ptr, 4);

    ctx->base.pc_next += 4;
    return true;
}

//TODO: Add STD format 5.

static bool trans_STH_f1(DisasContext *ctx, arg_STH_f1 *a){
    tcg_gen_qemu_st_tl(cpu_r[a->rs], cpu_r[a->rp], ctx->mem_idx, MO_BEUW);
    tcg_gen_addi_i32(cpu_r[a->rp], cpu_r[a->rp], 2);

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_STH_f2(DisasContext *ctx, arg_STH_f2 *a){
    tcg_gen_subi_i32(cpu_r[a->rp], cpu_r[a->rp], 2);
    tcg_gen_qemu_st_tl(cpu_r[a->rs], cpu_r[a->rp], ctx->mem_idx, MO_BEUW);

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_STH_f3(DisasContext *ctx, arg_STH_f3 *a){
    TCGv addr = tcg_temp_new_i32();
    tcg_gen_addi_i32(addr, cpu_r[a->rp], a->disp3 << 1);

    tcg_gen_qemu_st_tl(cpu_r[a->rd], addr, ctx->mem_idx, MO_BEUW);

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_STH_f4(DisasContext *ctx, arg_STH_f4 *a){
    int disp = a->imm16;
    if(disp >> 15 == 1){
        disp |= 0xFFFF0000;
    }
    TCGv addr = tcg_temp_new_i32();
    tcg_gen_addi_i32(addr, cpu_r[a->rp], disp);
    TCGv rs = cpu_r[a->rs];

    tcg_gen_qemu_st_tl(rs, addr, ctx->mem_idx, MO_BEUW);


    ctx->base.pc_next += 4;
    return true;
}

static bool trans_STH_f5(DisasContext *ctx, arg_STH_f5 *a){
    TCGv addr = tcg_temp_new_i32();
    tcg_gen_shli_i32(addr, cpu_r[a->ry], a->sa);
    tcg_gen_add_i32(addr, addr, cpu_r[a->rx]);
    TCGv rs = cpu_r[a->rd];

    tcg_gen_qemu_st_tl(rs, addr, ctx->mem_idx, MO_BEUW);


    ctx->base.pc_next += 4;
    return true;
}

static bool trans_STHc(DisasContext *ctx, arg_STHc *a){
    TCGLabel *exit = gen_new_label();

    TCGv reg = tcg_temp_new_i32();
    int val = checkCondition(a->cond4, reg, cpu_r, cpu_sflags);
    tcg_gen_brcondi_i32(TCG_COND_NE, reg, val, exit);

    TCGv ptr = tcg_temp_new_i32();
    tcg_gen_addi_i32(ptr, cpu_r[a->rp], a->disp9 << 1);
    tcg_gen_qemu_st_tl(cpu_r[a->rd], ptr, ctx->mem_idx, MO_BEUW);

    gen_set_label(exit);
    ctx->base.pc_next += 4;
    return true;
}

//TODO: Add STCOND

static bool trans_STDSP(DisasContext *ctx, arg_STDSP *a){
    TCGv ptr = tcg_temp_new_i32();
    tcg_gen_mov_i32(ptr, cpu_r[SP_REG]);
    tcg_gen_andi_i32(ptr, ptr, 0xFFFFFFFC);

    TCGv disp = tcg_temp_new_i32();
    tcg_gen_movi_i32(disp, a->disp<<2);

    tcg_gen_add_i32(ptr, ptr, disp);

    tcg_gen_qemu_st_i32(cpu_r[a->rd], ptr, ctx->mem_idx, MO_BEUL);

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_STM(DisasContext *ctx, arg_STM *a){
    int regFlag = 0;
    TCGv addr = tcg_temp_new_i32();
    tcg_gen_mov_i32(addr, cpu_r[a->rp]);
    if(a->op == 1){
        for (int i = 0; i <= 15; i++){
            regFlag = a->list >> i & 1;
            if(regFlag == 1){
                tcg_gen_subi_i32(addr, addr, 0x4);
                tcg_gen_qemu_st_tl(cpu_r[i], addr, ctx->mem_idx, MO_BEUL);
            }
        }
        tcg_gen_mov_i32(cpu_r[a->rp], addr);
    }
    else{
        for (int i = 15; i >= 0; i--){
            regFlag = a->list >> i & 1;
            if(regFlag == 1){
                tcg_gen_qemu_st_tl(cpu_r[i], addr, ctx->mem_idx, MO_BEUL);
                tcg_gen_addi_i32(addr, addr, 0x4);
            }
        }
    }

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_STW_f1(DisasContext *ctx, arg_STW_f1 *a){
    tcg_gen_qemu_st_tl(cpu_r[a->rs], cpu_r[a->rp], ctx->mem_idx, MO_BEUL);
    tcg_gen_addi_i32(cpu_r[a->rp], cpu_r[a->rp], 4);

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_STW_f2(DisasContext *ctx, arg_STW_f2 *a){
    tcg_gen_subi_i32(cpu_r[a->rp], cpu_r[a->rp], 4);
    tcg_gen_qemu_st_tl(cpu_r[a->rs], cpu_r[a->rp], ctx->mem_idx, MO_BEUL);

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_STW_f3(DisasContext *ctx, arg_STW_f3 *a){
    TCGv addr = tcg_temp_new_i32();
    tcg_gen_addi_i32(addr, cpu_r[a->rp], a->disp4<<2);
    tcg_gen_qemu_st_tl(cpu_r[a->rs], addr, ctx->mem_idx, MO_BEUL);

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_STW_f4(DisasContext *ctx, arg_STW_f4 *a){
    int disp = a->imm16;

    //Sign extend
    if(disp >> 15){
        disp |= 0xFFFF0000;
    }

    TCGv addr = tcg_temp_new_i32();
    tcg_gen_addi_i32(addr, cpu_r[a->rp], disp);

    tcg_gen_qemu_st_tl(cpu_r[a->rs], addr, ctx->mem_idx, MO_BEUL);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_STW_f5(DisasContext *ctx, arg_STW_f5 *a){
    TCGv addr = tcg_temp_new_i32();
    tcg_gen_shli_i32(addr, cpu_r[a->ry], a->sa);
    tcg_gen_add_i32(addr, addr, cpu_r[a->rx]);

    tcg_gen_qemu_st_tl(cpu_r[a->rd], addr, ctx->mem_idx, MO_BEUL);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_STWcond(DisasContext *ctx, arg_STWcond *a){
    TCGLabel *leave = gen_new_label();

    TCGv reg = tcg_temp_new_i32();
    int val = checkCondition(a->cond4, reg, cpu_r, cpu_sflags);

    tcg_gen_brcondi_i32(TCG_COND_NE, reg, val, leave);
    TCGv addr = tcg_temp_new_i32();
    tcg_gen_addi_i32(addr, cpu_r[a->rp], a->disp9 <<2 );
    tcg_gen_qemu_st_tl(cpu_r[a->rd], addr, ctx->mem_idx, MO_BEUL);

    gen_set_label(leave);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_SUB_f1(DisasContext *ctx, arg_SUB_f1 *a){
    TCGv op1 = tcg_temp_new_i32();
    TCGv op2 = tcg_temp_new_i32();
    tcg_gen_mov_i32(op1, cpu_r[a->rd]);
    tcg_gen_mov_i32(op2, cpu_r[a->rs]);

    TCGv left = tcg_temp_new_i32();
    TCGv middel = tcg_temp_new_i32();
    TCGv right = tcg_temp_new_i32();
    TCGv res = tcg_temp_new_i32();
    tcg_gen_sub_i32(res, op1, op2);
    tcg_gen_mov_i32(cpu_r[a->rd], res);

    // Z-Flag
    tcg_gen_setcondi_tl(TCG_COND_EQ, cpu_sflags[sflagZ], res, 0); // Zf = Res == 0

    tcg_gen_shri_i32(res, res, 31);
    tcg_gen_shri_i32(op1, op1, 31);
    tcg_gen_shri_i32(op2, op2, 31);

    //V-Flag
    tcg_gen_andc_i32(left, op1, op2);
    tcg_gen_andc_i32(left, left, res);
    tcg_gen_and_i32(right, op2, res);
    tcg_gen_andc_i32(right, right, op1);
    tcg_gen_or_i32(cpu_sflags[sflagV], left, right);

    //N-Flag
    tcg_gen_mov_i32(cpu_sflags[sflagN], res);

    //C-Flag
    tcg_gen_andc_i32(left, op2, op1);
    tcg_gen_and_i32(middel, op2, res);
    tcg_gen_andc_i32(right, res, op1);

    tcg_gen_or_i32(left, left, middel);
    tcg_gen_or_i32(cpu_sflags[sflagC], left, right);


    ctx->base.pc_next += 2;
    return true;
}

static bool trans_SUB_f2(DisasContext *ctx, arg_SUB_f2 *a){
    //Format 2
    TCGv op1 = tcg_temp_new_i32();
    TCGv op2 = tcg_temp_new_i32();
    tcg_gen_shli_i32(op2, cpu_r[a->ry], a->sa);

    tcg_gen_mov_i32(op1, cpu_r[a->rx]);

    TCGv left = tcg_temp_new_i32();
    TCGv middel = tcg_temp_new_i32();
    TCGv right = tcg_temp_new_i32();
    TCGv res = tcg_temp_new_i32();
    tcg_gen_sub_i32(res, op1, op2);
    tcg_gen_sub_i32(cpu_r[a->rd], op1, op2);

    tcg_gen_setcondi_tl(TCG_COND_EQ, cpu_sflags[sflagZ], res, 0); // Zf = Res == 0

    tcg_gen_shri_i32(res, res, 31);
    tcg_gen_shri_i32(op1, op1, 31);
    tcg_gen_shri_i32(op2, op2, 31);

    //V-Flag
    tcg_gen_andc_i32(left, op1, op2);
    tcg_gen_andc_i32(left, left, res);
    tcg_gen_and_i32(right, op2, res);
    tcg_gen_andc_i32(right, right, op1);
    tcg_gen_or_i32(cpu_sflags[sflagV], left, right);

    //N-Flag
    tcg_gen_mov_i32(cpu_sflags[sflagN], res);

    //C-Flag
    tcg_gen_andc_i32(left, op2, op1);
    tcg_gen_and_i32(middel, op2, res);
    tcg_gen_andc_i32(right, res, op1);

    tcg_gen_or_i32(left, left, middel);
    tcg_gen_or_i32(cpu_sflags[sflagC], left, right);

    ctx->base.pc_next += 4;
    return true;
}

static bool trans_SUB_f3(DisasContext *ctx, arg_SUB_f3 *a){
    TCGv op1 = tcg_temp_new_i32();
    TCGv op2 = tcg_temp_new_i32();
    tcg_gen_mov_i32(op1, cpu_r[a->rd]);

    if(a->rd == SP_REG){
        int imm = a->imm8 << 2;
        if((imm >> 9) == 1){
            imm |= 0xFFFFFC00;
        }
        tcg_gen_movi_i32(op2, imm);
    }
    else{
        int imm = sign_extend_8(a->imm8);
        tcg_gen_movi_i32(op2, imm);
    }

    TCGv left = tcg_temp_new_i32();
    TCGv middel = tcg_temp_new_i32();
    TCGv right = tcg_temp_new_i32();
    TCGv res = tcg_temp_new_i32();
    tcg_gen_sub_i32(res, op1, op2);
    tcg_gen_mov_i32(cpu_r[a->rd], res);
    tcg_gen_setcondi_tl(TCG_COND_EQ, cpu_sflags[sflagZ], res, 0); // Zf = Res == 0


    tcg_gen_shri_i32(res, res, 31);
    tcg_gen_shri_i32(op1, op1, 31);
    tcg_gen_shri_i32(op2, op2, 31);

    //V-Flag
    tcg_gen_andc_i32(left, op1, op2);
    tcg_gen_andc_i32(left, left, res);
    tcg_gen_and_i32(right, op2, res);
    tcg_gen_andc_i32(right, right, op1);
    tcg_gen_or_i32(cpu_sflags[sflagV], left, right);

    //N-Flag
    tcg_gen_mov_i32(cpu_sflags[sflagN], res);

    //C-Flag
    tcg_gen_andc_i32(left, op2, op1);
    tcg_gen_and_i32(middel, op2, res);
    tcg_gen_andc_i32(right, res, op1);

    tcg_gen_or_i32(left, left, middel);
    tcg_gen_or_i32(cpu_sflags[sflagC], left, right);

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_SUB_f4(DisasContext *ctx, arg_SUB_f4 *a){
    int imm = a->imml;
    imm |= (a->immm << 16);
    imm |= (a->immu << 17);

    //Sign extend
    if(imm >> 20){
        imm |= 0xFFF00000;
    }

    TCGv op1 = tcg_temp_new_i32();
    TCGv op2 = tcg_temp_new_i32();
    tcg_gen_mov_i32(op1, cpu_r[a->rd]);
    tcg_gen_movi_i32(op2, imm);

    TCGv left = tcg_temp_new_i32();
    TCGv middel = tcg_temp_new_i32();
    TCGv right = tcg_temp_new_i32();
    TCGv res = tcg_temp_new_i32();
    tcg_gen_sub_i32(res, op1, op2);
    tcg_gen_sub_i32(cpu_r[a->rd], op1, op2);
    tcg_gen_setcondi_tl(TCG_COND_EQ, cpu_sflags[sflagZ], res, 0); // Zf = Res == 0

    tcg_gen_shri_i32(res, res, 31);
    tcg_gen_shri_i32(op1, op1, 31);
    tcg_gen_shri_i32(op2, op2, 31);

    //V-Flag
    tcg_gen_andc_i32(left, op1, op2);
    tcg_gen_andc_i32(left, left, res);
    tcg_gen_and_i32(right, op2, res);
    tcg_gen_andc_i32(right, right, op1);
    tcg_gen_or_i32(cpu_sflags[sflagV], left, right);

    //N-Flag
    tcg_gen_mov_i32(cpu_sflags[sflagN], res);

    //C-Flag
    tcg_gen_andc_i32(left, op2, op1);
    tcg_gen_and_i32(middel, op2, res);
    tcg_gen_andc_i32(right, res, op1);

    tcg_gen_or_i32(left, left, middel);
    tcg_gen_or_i32(cpu_sflags[sflagC], left, right);


    ctx->base.pc_next += 4;
    return true;
}

static bool trans_SUB_f5(DisasContext *ctx, arg_SUB_f5 *a){
    int imm = a->imm16;

    //Sign extend
    if(imm >> 15){
        imm = imm|0xFFFF0000;
    }

    TCGv op1 = tcg_temp_new_i32();
    TCGv op2 = tcg_temp_new_i32();
    tcg_gen_mov_i32(op1, cpu_r[a->rs]);
    tcg_gen_movi_i32(op2, imm);

    TCGv left = tcg_temp_new_i32();
    TCGv middel = tcg_temp_new_i32();
    TCGv right = tcg_temp_new_i32();
    TCGv res = tcg_temp_new_i32();
    tcg_gen_sub_i32(res, op1, op2);
    tcg_gen_sub_i32(cpu_r[a->rd], op1, op2);
    tcg_gen_setcondi_tl(TCG_COND_EQ, cpu_sflags[sflagZ], res, 0); // Zf = Res == 0

    tcg_gen_shri_i32(res, res, 31);
    tcg_gen_shri_i32(op1, op1, 31);
    tcg_gen_shri_i32(op2, op2, 31);

    //V-Flag
    tcg_gen_andc_i32(left, op1, op2);
    tcg_gen_andc_i32(left, left, res);
    tcg_gen_and_i32(right, op2, res);
    tcg_gen_andc_i32(right, right, op1);
    tcg_gen_or_i32(cpu_sflags[sflagV], left, right);

    //N-Flag
    tcg_gen_mov_i32(cpu_sflags[sflagN], res);

    //C-Flag
    tcg_gen_andc_i32(left, op2, op1);
    tcg_gen_and_i32(middel, op2, res);
    tcg_gen_andc_i32(right, res, op1);

    tcg_gen_or_i32(left, left, middel);
    tcg_gen_or_i32(cpu_sflags[sflagC], left, right);

    ctx->base.pc_next += 4;
    return true;
}

//TODO: check if f needs to be set or not, as manual has contradictory statements
static bool trans_SUBc_f1(DisasContext *ctx, arg_SUBc_f1 *a){
    TCGLabel *if1 = gen_new_label();
    TCGLabel *exit = gen_new_label();

    TCGv rd = tcg_temp_new_i32();
    TCGv res = tcg_temp_new_i32();
    TCGv left = tcg_temp_new_i32();
    TCGv right = tcg_temp_new_i32();
    TCGv k = tcg_temp_new_i32();

    TCGv reg = tcg_temp_new_i32();
    int val = checkCondition(a->cond4, reg, cpu_r, cpu_sflags);
    tcg_gen_brcondi_i32(TCG_COND_NE, reg, val, exit);

    // if
    gen_set_label(if1);
    // See 32-bit Atmel Architecture Manual chapter 9.4 SUB{cond4} (page.358)
    // https://ww1.microchip.com/downloads/en/devicedoc/doc32000.pdf
    tcg_gen_subi_i32(res, cpu_r[a->rd], sign_extend_8(a->imm8));
    tcg_gen_mov_i32(rd, cpu_r[a->rd]);
    tcg_gen_movi_i32(k, sign_extend_8(a->imm8));
    tcg_gen_mov_i32(cpu_r[a->rd], res);

    if(a->f == 1){
        //z-flag
        tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], res, 0);
        //n-flag
        tcg_gen_shri_i32(res, res, 31);
        tcg_gen_mov_i32(cpu_sflags[sflagN], res);

        //v-flag
        tcg_gen_shri_i32(rd, rd, 31);
        tcg_gen_shri_i32(k, k, 31);

        tcg_gen_andc_i32(left, rd, k);
        tcg_gen_andc_i32(left, left, res);
        tcg_gen_andc_i32(right, k, rd);
        tcg_gen_and_i32(right, right, res);
        tcg_gen_or_i32(cpu_sflags[sflagV], left, right);

        //c-flag
        tcg_gen_andc_i32(left, k, rd);
        tcg_gen_and_i32(right, k, res);
        tcg_gen_or_i32(left, left, right);
        tcg_gen_andc_i32(right, res, rd);
        tcg_gen_or_i32(cpu_sflags[sflagC], left, right);
    }

    gen_set_label(exit);
    ctx->base.pc_next += 4;
    return true;
}

static bool trans_SUBc_f2(DisasContext *ctx, arg_SUBc_f2 *a){
    TCGLabel *if1 = gen_new_label();
    TCGLabel *exit = gen_new_label();

    TCGv reg = tcg_temp_new_i32();
    int val = checkCondition(a->cond4, reg, cpu_r, cpu_sflags);
    tcg_gen_brcondi_i32(TCG_COND_NE, reg, val, exit);

    // if
    gen_set_label(if1);
    tcg_gen_sub_i32(cpu_r[a->rd], cpu_r[a->rx], cpu_r[a->ry]);

    gen_set_label(exit);
    ctx->base.pc_next += 4;
    return true;
}

static bool trans_TNBZ(DisasContext *ctx, arg_TNBZ *a){
    TCGv rdl = tcg_temp_new_i32();
    TCGv rdml = tcg_temp_new_i32();
    TCGv rdmr = tcg_temp_new_i32();
    TCGv rdr = tcg_temp_new_i32();

    TCGv res = tcg_temp_new_i32();
    tcg_gen_movi_i32(res, 0);

    tcg_gen_mov_i32(rdl, cpu_r[a->rd]);
    tcg_gen_andi_i32(rdl, rdl, 0xFF000000);
    tcg_gen_setcondi_i32(TCG_COND_EQ, rdl, rdl, 0);

    tcg_gen_mov_i32(rdml, cpu_r[a->rd]);
    tcg_gen_andi_i32(rdml, rdml, 0x00FF0000);
    tcg_gen_setcondi_i32(TCG_COND_EQ, rdml, rdml, 0);

    tcg_gen_mov_i32(rdmr, cpu_r[a->rd]);
    tcg_gen_andi_i32(rdmr, rdmr, 0x0000FF00);
    tcg_gen_setcondi_i32(TCG_COND_EQ, rdmr, rdmr, 0);

    tcg_gen_mov_i32(rdr, cpu_r[a->rd]);
    tcg_gen_andi_i32(rdr, rdr, 0x000000FF);
    tcg_gen_setcondi_i32(TCG_COND_EQ, rdr, rdr, 0);

    tcg_gen_add_i32(res, res, rdl);
    tcg_gen_add_i32(res, res, rdml);
    tcg_gen_add_i32(res, res, rdmr);
    tcg_gen_add_i32(res, res, rdr);

    tcg_gen_setcondi_i32(TCG_COND_NE, cpu_sflags[sflagZ], res, 0);

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_TST(DisasContext *ctx, arg_TST *a){
    TCGv res = tcg_temp_new_i32();
    tcg_gen_and_i32(res, cpu_r[a->rd], cpu_r[a->rs]);

    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[sflagZ], res, 0);
    tcg_gen_shri_i32(cpu_sflags[sflagN], res, 31);

    ctx->base.pc_next += 2;
    return true;
}

static bool trans_RETD(DisasContext *ctx, arg_RETD *a)
{
    ctx->base.pc_next += 2;
    if (gen_priv_check(ctx)) {
        gen_helper_retd(tcg_env);
        ctx->base.is_jmp = DISAS_EXIT;
    }
    return true;
}

static bool trans_TLBR(DisasContext *ctx, arg_TLBR *a)
{
    ctx->base.pc_next += 2;
    if (gen_priv_check(ctx)) {
        gen_helper_tlbr(tcg_env);
    }
    return true;
}

static bool trans_TLBS(DisasContext *ctx, arg_TLBS *a)
{
    ctx->base.pc_next += 2;
    if (gen_priv_check(ctx)) {
        gen_helper_tlbs(tcg_env);
    }
    return true;
}

static bool trans_TLBW(DisasContext *ctx, arg_TLBW *a)
{
    ctx->base.pc_next += 2;
    if (gen_priv_check(ctx)) {
        gen_helper_tlbw(tcg_env);
    }
    return true;
}

static bool trans_MFDR(DisasContext *ctx, arg_MFDR *a)
{
    ctx->base.pc_next += 4;
    if (gen_priv_check(ctx)) {
        gen_helper_mfdr(cpu_r[a->rd], tcg_env, tcg_constant_i32(a->sr));
    }
    return true;
}

/* no write buffer and no caches are modelled */
static bool trans_SYNC(DisasContext *ctx, arg_SYNC *a)
{
    ctx->base.pc_next += 4;
    return true;
}

static bool trans_PREF(DisasContext *ctx, arg_PREF *a)
{
    ctx->base.pc_next += 4;
    return true;
}

static bool trans_STCOND(DisasContext *ctx, arg_STCOND *a)
{
    TCGLabel *skip = gen_new_label();
    TCGv addr = tcg_temp_new_i32();

    tcg_gen_mov_i32(cpu_sflags[SR_Z], cpu_sflags[SR_L]);
    tcg_gen_brcondi_i32(TCG_COND_EQ, cpu_sflags[SR_L], 0, skip);
    tcg_gen_addi_i32(addr, cpu_r[a->rp], (int16_t)a->disp16);
    tcg_gen_qemu_st_i32(cpu_r[a->rs], addr, ctx->mem_idx, MO_BEUL);
    gen_set_label(skip);
    ctx->base.pc_next += 4;
    return true;
}

static bool trans_XCHG(DisasContext *ctx, arg_XCHG *a)
{
    TCGv t = tcg_temp_new_i32();

    tcg_gen_qemu_ld_i32(t, cpu_r[a->rx], ctx->mem_idx, MO_BEUL);
    tcg_gen_qemu_st_i32(cpu_r[a->ry], cpu_r[a->rx], ctx->mem_idx, MO_BEUL);
    tcg_gen_mov_i32(cpu_r[a->rd], t);
    ctx->base.pc_next += 4;
    return true;
}

static bool trans_SWAPB(DisasContext *ctx, arg_SWAPB *a)
{
    tcg_gen_bswap32_i32(cpu_r[a->rd], cpu_r[a->rd]);
    ctx->base.pc_next += 2;
    return true;
}

static bool trans_SWAPBH(DisasContext *ctx, arg_SWAPBH *a)
{
    TCGv t = tcg_temp_new_i32();

    /* swap the bytes within each halfword */
    tcg_gen_shri_i32(t, cpu_r[a->rd], 8);
    tcg_gen_andi_i32(t, t, 0x00ff00ff);
    tcg_gen_shli_i32(cpu_r[a->rd], cpu_r[a->rd], 8);
    tcg_gen_andi_i32(cpu_r[a->rd], cpu_r[a->rd], 0xff00ff00);
    tcg_gen_or_i32(cpu_r[a->rd], cpu_r[a->rd], t);
    ctx->base.pc_next += 2;
    return true;
}

static bool trans_SWAPH(DisasContext *ctx, arg_SWAPH *a)
{
    tcg_gen_rotli_i32(cpu_r[a->rd], cpu_r[a->rd], 16);
    ctx->base.pc_next += 2;
    return true;
}

static void gen_sthh(DisasContext *ctx, TCGv addr, int rx, int x, int ry, int y)
{
    TCGv hi = tcg_temp_new_i32();
    TCGv lo = tcg_temp_new_i32();

    if (x) {
        tcg_gen_andi_i32(hi, cpu_r[rx], 0xffff0000);
    } else {
        tcg_gen_shli_i32(hi, cpu_r[rx], 16);
    }
    if (y) {
        tcg_gen_shri_i32(lo, cpu_r[ry], 16);
    } else {
        tcg_gen_ext16u_i32(lo, cpu_r[ry]);
    }
    tcg_gen_or_i32(hi, hi, lo);
    tcg_gen_qemu_st_i32(hi, addr, ctx->mem_idx, MO_BEUL);
}

static bool trans_STHHW_disp(DisasContext *ctx, arg_STHHW_disp *a)
{
    TCGv addr = tcg_temp_new_i32();

    tcg_gen_addi_i32(addr, cpu_r[a->rp], a->disp8 << 2);
    gen_sthh(ctx, addr, a->rx, a->x, a->ry, a->y);
    ctx->base.pc_next += 4;
    return true;
}

static bool trans_STHHW_idx(DisasContext *ctx, arg_STHHW_idx *a)
{
    TCGv addr = tcg_temp_new_i32();

    tcg_gen_shli_i32(addr, cpu_r[a->ri], a->sa);
    tcg_gen_add_i32(addr, addr, cpu_r[a->rp]);
    gen_sthh(ctx, addr, a->rx, a->x, a->ry, a->y);
    ctx->base.pc_next += 4;
    return true;
}

static bool trans_STSWPH(DisasContext *ctx, arg_STSWPH *a)
{
    TCGv addr = tcg_temp_new_i32();

    tcg_gen_addi_i32(addr, cpu_r[a->rp], sextract32(a->disp12, 0, 12) << 1);
    tcg_gen_qemu_st_i32(cpu_r[a->rs], addr, ctx->mem_idx, MO_LEUW);
    ctx->base.pc_next += 4;
    return true;
}

static bool trans_STSWPW(DisasContext *ctx, arg_STSWPW *a)
{
    TCGv addr = tcg_temp_new_i32();

    tcg_gen_addi_i32(addr, cpu_r[a->rp], sextract32(a->disp12, 0, 12) << 2);
    tcg_gen_qemu_st_i32(cpu_r[a->rs], addr, ctx->mem_idx, MO_LEUL);
    ctx->base.pc_next += 4;
    return true;
}

/* 32-bit saturating subtraction; flags as for sub, Q on saturation */
static void gen_satsub_w(TCGv rd, TCGv op1, TCGv op2)
{
    TCGv res = tcg_temp_new_i32();
    TCGv ovf = tcg_temp_new_i32();
    TCGv t = tcg_temp_new_i32();
    TCGv sat = tcg_temp_new_i32();

    tcg_gen_sub_i32(res, op1, op2);
    /* overflow: operands of different sign and result sign != op1 sign */
    tcg_gen_xor_i32(ovf, op1, op2);
    tcg_gen_xor_i32(t, op1, res);
    tcg_gen_and_i32(ovf, ovf, t);
    tcg_gen_shri_i32(ovf, ovf, 31);
    /* saturate towards the sign of op1 */
    tcg_gen_sari_i32(sat, op1, 31);
    tcg_gen_xori_i32(sat, sat, 0x7fffffff);
    tcg_gen_movcond_i32(TCG_COND_NE, res, ovf, tcg_constant_i32(0), sat, res);
    tcg_gen_or_i32(cpu_sflags[SR_Q], cpu_sflags[SR_Q], ovf);
    tcg_gen_mov_i32(cpu_sflags[SR_V], ovf);
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_sflags[SR_Z], res, 0);
    tcg_gen_shri_i32(cpu_sflags[SR_N], res, 31);
    tcg_gen_setcond_i32(TCG_COND_LTU, cpu_sflags[SR_C], op1, op2);
    tcg_gen_mov_i32(rd, res);
}

static bool trans_SATSUBW_rr(DisasContext *ctx, arg_SATSUBW_rr *a)
{
    gen_satsub_w(cpu_r[a->rd], cpu_r[a->rx], cpu_r[a->ry]);
    ctx->base.pc_next += 4;
    return true;
}

static bool trans_SATSUBW_imm(DisasContext *ctx, arg_SATSUBW_imm *a)
{
    gen_satsub_w(cpu_r[a->rd], cpu_r[a->rs],
                 tcg_constant_i32((int16_t)a->imm16));
    ctx->base.pc_next += 4;
    return true;
}

static bool trans_MULSD(DisasContext *ctx, arg_MULSD *a)
{
    tcg_gen_muls2_i32(cpu_r[a->rd], cpu_r[a->rd + 1], cpu_r[a->rx],
                      cpu_r[a->ry]);
    ctx->base.pc_next += 4;
    return true;
}

/* st.d Rb[Ri << sa], Rs: the odd register goes to the lower address */
static bool trans_STD_idx(DisasContext *ctx, arg_STD_idx *a)
{
    TCGv addr = tcg_temp_new_i32();

    tcg_gen_shli_i32(addr, cpu_r[a->ri], a->sa);
    tcg_gen_add_i32(addr, addr, cpu_r[a->rb]);
    tcg_gen_qemu_st_i32(cpu_r[a->rs + 1], addr, ctx->mem_idx, MO_BEUL);
    tcg_gen_addi_i32(addr, addr, 4);
    tcg_gen_qemu_st_i32(cpu_r[a->rs], addr, ctx->mem_idx, MO_BEUL);
    ctx->base.pc_next += 4;
    return true;
}

static void avr32_tr_init_disas_context(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *ctx = container_of(dcbase, DisasContext, base);

    ctx->env = cpu_env(cs);
    ctx->cs = cs;
    ctx->pc = ctx->base.pc_first;
    ctx->priv = ctx->base.tb->flags & AVR32_TBFLAG_PRIV;
    ctx->mem_idx = ctx->priv ? MMU_KERNEL_IDX : MMU_USER_IDX;
}

static void avr32_tr_tb_start(DisasContextBase *db, CPUState *cs)
{
}

static void avr32_tr_insn_start(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *ctx = container_of(dcbase, DisasContext, base);

    tcg_gen_insn_start(ctx->base.pc_next, 0, 0);
}

static void avr32_tr_translate_insn(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *ctx = container_of(dcbase, DisasContext, base);
    uint32_t insn;

    tcg_gen_movi_i32(cpu_r[AVR32_PC_REG], ctx->base.pc_next);

    insn = decode_insn_load(ctx);
    if (!decode_insn(ctx, insn)) {
        qemu_log_mask(LOG_GUEST_ERROR, "avr32: illegal instruction 0x%08x "
                      "at 0x%08" VADDR_PRIx "\n", insn, ctx->base.pc_next);
        gen_exception(ctx, EXCP_AVR32_ILLEGAL);
        ctx->base.pc_next += 2;
    }
}

static void avr32_tr_tb_stop(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *ctx = container_of(dcbase, DisasContext, base);

    switch (ctx->base.is_jmp) {
    case DISAS_NEXT:
    case DISAS_TOO_MANY:
        gen_goto_tb(ctx, 1, ctx->base.pc_next);
        break;
    case DISAS_NORETURN:
        break;
    case DISAS_JUMP:
        tcg_gen_lookup_and_goto_ptr();
        break;
    case DISAS_CHAIN:
        gen_goto_tb(ctx, 1, ctx->base.pc_next);
        tcg_gen_movi_i32(cpu_r[AVR32_PC_REG], ctx->base.pc_next);
        tcg_gen_exit_tb(NULL, 0);
        break;
    case DISAS_UPDATE:
        tcg_gen_movi_i32(cpu_r[AVR32_PC_REG], ctx->base.pc_next);
        /* fall through */
    case DISAS_EXIT:
        tcg_gen_exit_tb(NULL, 0);
        break;
    default:
        g_assert_not_reached();
    }
}

static const TranslatorOps avr32_tr_ops = {
    .init_disas_context = avr32_tr_init_disas_context,
    .tb_start           = avr32_tr_tb_start,
    .insn_start         = avr32_tr_insn_start,
    .translate_insn     = avr32_tr_translate_insn,
    .tb_stop            = avr32_tr_tb_stop,
};

void avr32_translate_code(CPUState *cs, TranslationBlock *tb,
                          int *max_insns, vaddr pc, void *host_pc)
{
    DisasContext dc = { };

    translator_loop(cs, tb, max_insns, pc, host_pc, &avr32_tr_ops, &dc.base,
                    TCG_TYPE_I32);
}
