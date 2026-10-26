/*
 * IA-64 translation: A-unit and I-unit integer instructions
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

/* Field shorthands of the common formats */
#define R1(b)   ((int)field(b, 6, 7))
#define R2(b)   ((int)field(b, 13, 7))
#define R3(b)   ((int)field(b, 20, 7))
#define P1(b)   ((int)field(b, 6, 6))
#define P2(b)   ((int)field(b, 27, 6))

/* addp4/shladdp4: 32-bit result with the region bits from r3{31:30}. */
static void gen_p4(TCGv_i64 d, TCGv_i64 sum, TCGv_i64 r3)
{
    TCGv_i64 t = tcg_temp_new_i64();

    tcg_gen_extract_i64(t, r3, 30, 2);
    tcg_gen_shli_i64(t, t, 61);
    tcg_gen_ext32u_i64(d, sum);
    tcg_gen_or_i64(d, d, t);
}

/* Write r1 = v with nat; checks were done by the caller. */
static void finish_gr(DisasContext *dc, int r1, TCGv_i64 v, TCGv_i32 nat)
{
    store_gr(dc, r1, v, nat);
}

static void trans_a_alu(DisasContext *dc, const struct asm_inst *in)
{
    uint64_t b = in->i_bits;
    int r1 = R1(b), r2 = R2(b), r3 = R3(b);
    bool imm = in->i_format == ASM_FMT_A3;
    TCGv_i64 s2, s3 = load_gr(dc, r3);
    TCGv_i32 nat;
    TCGv_i64 d = tcg_temp_new_i64();

    if (!check_gr_dst(dc, r1) || !check_gr(dc, r3) ||
        (!imm && !check_gr(dc, r2))) {
        return;
    }
    if (imm) {
        s2 = tcg_constant_i64(in->i_oper[2].o_value);
        nat = load_nat(dc, r3);
    } else {
        s2 = load_gr(dc, r2);
        nat = nat_or(load_nat(dc, r2), load_nat(dc, r3));
    }

    switch (in->i_iop) {
    case ASM_OP_ADD:
        tcg_gen_add_i64(d, s2, s3);
        if (field(b, 27, 2) == 1) {
            tcg_gen_addi_i64(d, d, 1);
        }
        break;
    case ASM_OP_SUB:
        tcg_gen_sub_i64(d, s2, s3);
        if (!imm && field(b, 27, 2) == 0) {
            tcg_gen_subi_i64(d, d, 1);
        }
        break;
    case ASM_OP_ADDP4:
        tcg_gen_add_i64(d, s2, s3);
        gen_p4(d, d, s3);
        break;
    case ASM_OP_AND:
        tcg_gen_and_i64(d, s2, s3);
        break;
    case ASM_OP_ANDCM:
        tcg_gen_andc_i64(d, s2, s3);
        break;
    case ASM_OP_OR:
        tcg_gen_or_i64(d, s2, s3);
        break;
    case ASM_OP_XOR:
        tcg_gen_xor_i64(d, s2, s3);
        break;
    default:
        gen_illegal(dc);
        return;
    }
    finish_gr(dc, r1, d, nat);
}

static void trans_a_shladd(DisasContext *dc, const struct asm_inst *in)
{
    uint64_t b = in->i_bits;
    int r1 = R1(b), r2 = R2(b), r3 = R3(b);
    int count = field(b, 27, 2) + 1;
    TCGv_i64 d = tcg_temp_new_i64();

    if (!check_gr_dst(dc, r1) || !check_gr(dc, r2) || !check_gr(dc, r3)) {
        return;
    }
    tcg_gen_shli_i64(d, load_gr(dc, r2), count);
    tcg_gen_add_i64(d, d, load_gr(dc, r3));
    if (in->i_iop == ASM_OP_SHLADDP4) {
        gen_p4(d, d, load_gr(dc, r3));
    }
    finish_gr(dc, r1, d, nat_or(load_nat(dc, r2), load_nat(dc, r3)));
}

static void trans_a_addimm(DisasContext *dc, const struct asm_inst *in)
{
    uint64_t b = in->i_bits;
    int r1 = R1(b);
    int r3 = in->i_format == ASM_FMT_A5 ? field(b, 20, 2) : R3(b);
    TCGv_i64 d = tcg_temp_new_i64();

    if (!check_gr_dst(dc, r1) || !check_gr(dc, r3)) {
        return;
    }
    tcg_gen_addi_i64(d, load_gr(dc, r3), in->i_oper[2].o_value);
    if (in->i_iop == ASM_OP_ADDP4) {
        gen_p4(d, d, load_gr(dc, r3));
    }
    finish_gr(dc, r1, d, load_nat(dc, r3));
}

/*
 * cmp and cmp4.  Decodes relation, ctype and width from the internal
 * opcode name.
 */
typedef struct CmpOp {
    int iop;
    TCGCond cond;
    bool w32;
    int ctype;
} CmpOp;

#define CMPOPS(W, w32) \
    { ASM_OP_##W##_EQ, TCG_COND_EQ, w32, CT_NONE }, \
    { ASM_OP_##W##_EQ_UNC, TCG_COND_EQ, w32, CT_UNC }, \
    { ASM_OP_##W##_EQ_AND, TCG_COND_EQ, w32, CT_AND }, \
    { ASM_OP_##W##_EQ_OR, TCG_COND_EQ, w32, CT_OR }, \
    { ASM_OP_##W##_EQ_OR_ANDCM, TCG_COND_EQ, w32, CT_ORANDCM }, \
    { ASM_OP_##W##_NE_AND, TCG_COND_NE, w32, CT_AND }, \
    { ASM_OP_##W##_NE_OR, TCG_COND_NE, w32, CT_OR }, \
    { ASM_OP_##W##_NE_OR_ANDCM, TCG_COND_NE, w32, CT_ORANDCM }, \
    { ASM_OP_##W##_LT, TCG_COND_LT, w32, CT_NONE }, \
    { ASM_OP_##W##_LT_UNC, TCG_COND_LT, w32, CT_UNC }, \
    { ASM_OP_##W##_LTU, TCG_COND_LTU, w32, CT_NONE }, \
    { ASM_OP_##W##_LTU_UNC, TCG_COND_LTU, w32, CT_UNC }, \
    { ASM_OP_##W##_GT_AND, TCG_COND_GT, w32, CT_AND }, \
    { ASM_OP_##W##_GT_OR, TCG_COND_GT, w32, CT_OR }, \
    { ASM_OP_##W##_GT_OR_ANDCM, TCG_COND_GT, w32, CT_ORANDCM }, \
    { ASM_OP_##W##_LE_AND, TCG_COND_LE, w32, CT_AND }, \
    { ASM_OP_##W##_LE_OR, TCG_COND_LE, w32, CT_OR }, \
    { ASM_OP_##W##_LE_OR_ANDCM, TCG_COND_LE, w32, CT_ORANDCM }, \
    { ASM_OP_##W##_GE_AND, TCG_COND_GE, w32, CT_AND }, \
    { ASM_OP_##W##_GE_OR, TCG_COND_GE, w32, CT_OR }, \
    { ASM_OP_##W##_GE_OR_ANDCM, TCG_COND_GE, w32, CT_ORANDCM }, \
    { ASM_OP_##W##_LT_AND, TCG_COND_LT, w32, CT_AND }, \
    { ASM_OP_##W##_LT_OR, TCG_COND_LT, w32, CT_OR }, \
    { ASM_OP_##W##_LT_OR_ANDCM, TCG_COND_LT, w32, CT_ORANDCM }

static const CmpOp cmp_ops[] = {
    CMPOPS(CMP, false),
    CMPOPS(CMP4, true),
};

/*
 * For the unc type the instruction executes with qp false too and then
 * clears both targets: model that as a NaT source.
 */
static TCGv_i32 unc_nat(DisasContext *dc, int ctype, TCGv_i32 nat)
{
    int qp = field(dc->bits, 0, 6);
    TCGv_i32 t;

    if (ctype != CT_UNC || qp == 0) {
        return nat;
    }
    t = tcg_temp_new_i32();
    tcg_gen_xori_i32(t, cpu_pr[pr_idx(dc, qp)], 1);
    return nat ? nat_or(nat, t) : t;
}

static void trans_a_cmp(DisasContext *dc, const struct asm_inst *in)
{
    uint64_t b = in->i_bits;
    int p1 = P1(b), p2 = P2(b), r2 = R2(b), r3 = R3(b);
    const CmpOp *op = NULL;
    TCGv_i64 a, c;
    TCGv_i32 res = tcg_temp_new_i32();
    TCGv_i32 nat;

    for (size_t i = 0; i < ARRAY_SIZE(cmp_ops); i++) {
        if (cmp_ops[i].iop == in->i_iop) {
            op = &cmp_ops[i];
        }
    }
    if (!op || !check_gr(dc, r3) ||
        (in->i_format != ASM_FMT_A8 && !check_gr(dc, r2))) {
        if (!op) {
            gen_illegal(dc);
        }
        return;
    }
    if (op->ctype == CT_UNC && p1 == p2) {
        gen_illegal(dc);
        return;
    }

    a = tcg_temp_new_i64();
    c = tcg_temp_new_i64();
    if (in->i_format == ASM_FMT_A8) {
        tcg_gen_movi_i64(a, in->i_oper[3].o_value);
        nat = load_nat(dc, r3);
    } else {
        tcg_gen_mov_i64(a, load_gr(dc, r2));
        nat = nat_or(load_nat(dc, r2), load_nat(dc, r3));
    }
    tcg_gen_mov_i64(c, load_gr(dc, r3));
    if (op->w32) {
        if (op->cond == TCG_COND_LTU) {
            tcg_gen_ext32u_i64(a, a);
            tcg_gen_ext32u_i64(c, c);
        } else {
            tcg_gen_ext32s_i64(a, a);
            tcg_gen_ext32s_i64(c, c);
        }
    }
    tcg_gen_setcond_i64(op->cond, a, a, c);
    tcg_gen_extrl_i64_i32(res, a);
    gen_write_cmp(dc, op->ctype, p1, p2, res, unc_nat(dc, op->ctype, nat));
}

/* tbit, tnat, tf */
static void trans_i_test(DisasContext *dc, const struct asm_inst *in)
{
    uint64_t b = in->i_bits;
    int p1 = P1(b), p2 = P2(b), r3 = R3(b);
    int iop = in->i_iop;
    TCGv_i32 res = tcg_temp_new_i32();
    TCGv_i32 nat = NULL;
    int ctype;
    bool z;

    if (in->i_format != ASM_FMT_I30 && !check_gr(dc, r3)) {
        return;
    }

    switch (iop) {
    case ASM_OP_TBIT_Z: case ASM_OP_TNAT_Z: case ASM_OP_TF_Z:
        ctype = CT_NONE; z = true; break;
    case ASM_OP_TBIT_Z_UNC: case ASM_OP_TNAT_Z_UNC: case ASM_OP_TF_Z_UNC:
        ctype = CT_UNC; z = true; break;
    case ASM_OP_TBIT_Z_AND: case ASM_OP_TNAT_Z_AND: case ASM_OP_TF_Z_AND:
        ctype = CT_AND; z = true; break;
    case ASM_OP_TBIT_NZ_AND: case ASM_OP_TNAT_NZ_AND: case ASM_OP_TF_NZ_AND:
        ctype = CT_AND; z = false; break;
    case ASM_OP_TBIT_Z_OR: case ASM_OP_TNAT_Z_OR: case ASM_OP_TF_Z_OR:
        ctype = CT_OR; z = true; break;
    case ASM_OP_TBIT_NZ_OR: case ASM_OP_TNAT_NZ_OR: case ASM_OP_TF_NZ_OR:
        ctype = CT_OR; z = false; break;
    case ASM_OP_TBIT_Z_OR_ANDCM: case ASM_OP_TNAT_Z_OR_ANDCM:
    case ASM_OP_TF_Z_OR_ANDCM:
        ctype = CT_ORANDCM; z = true; break;
    case ASM_OP_TBIT_NZ_OR_ANDCM: case ASM_OP_TNAT_NZ_OR_ANDCM:
    case ASM_OP_TF_NZ_OR_ANDCM:
        ctype = CT_ORANDCM; z = false; break;
    default:
        gen_illegal(dc);
        return;
    }
    if (ctype == CT_UNC && p1 == p2) {
        gen_illegal(dc);
        return;
    }

    if (in->i_format == ASM_FMT_I16) {
        TCGv_i64 t = tcg_temp_new_i64();

        tcg_gen_extract_i64(t, load_gr(dc, r3), field(b, 14, 6), 1);
        tcg_gen_extrl_i64_i32(res, t);
        nat = load_nat(dc, r3);
    } else if (in->i_format == ASM_FMT_I17) {
        tcg_gen_mov_i32(res, load_nat(dc, r3));
    } else {
        /* No processor features are implemented. */
        tcg_gen_movi_i32(res, 0);
    }
    if (z) {
        tcg_gen_xori_i32(res, res, 1);
    }
    gen_write_cmp(dc, ctype, p1, p2, res, unc_nat(dc, ctype, nat));
}

static void trans_i_shift(DisasContext *dc, const struct asm_inst *in)
{
    uint64_t b = in->i_bits;
    int r1 = R1(b), r2 = R2(b), r3 = R3(b);
    TCGv_i64 d = tcg_temp_new_i64();
    TCGv_i64 cnt = tcg_temp_new_i64();
    TCGv_i64 big = tcg_temp_new_i64();
    TCGv_i64 src, n;

    if (!check_gr_dst(dc, r1) || !check_gr(dc, r2) || !check_gr(dc, r3)) {
        return;
    }
    if (in->i_iop == ASM_OP_SHL) {
        /* shl r1 = r2, r3: count in r3 */
        src = load_gr(dc, r2);
        n = load_gr(dc, r3);
        tcg_gen_andi_i64(cnt, n, 63);
        tcg_gen_shl_i64(d, src, cnt);
        tcg_gen_movcond_i64(TCG_COND_GTU, d, n, tcg_constant_i64(63),
                            tcg_constant_i64(0), d);
    } else {
        /* shr r1 = r3, r2: count in r2 */
        src = load_gr(dc, r3);
        n = load_gr(dc, r2);
        tcg_gen_andi_i64(cnt, n, 63);
        if (in->i_iop == ASM_OP_SHR_U) {
            tcg_gen_shr_i64(d, src, cnt);
            tcg_gen_movi_i64(big, 0);
        } else {
            tcg_gen_sar_i64(d, src, cnt);
            tcg_gen_sari_i64(big, src, 63);
        }
        tcg_gen_movcond_i64(TCG_COND_GTU, d, n, tcg_constant_i64(63),
                            big, d);
    }
    finish_gr(dc, r1, d, nat_or(load_nat(dc, r2), load_nat(dc, r3)));
}

static void trans_i_bitfield(DisasContext *dc, const struct asm_inst *in)
{
    uint64_t b = in->i_bits;
    int r1 = R1(b), r2 = R2(b), r3 = R3(b);
    TCGv_i64 d = tcg_temp_new_i64();
    TCGv_i32 nat;
    int pos, len;

    if (!check_gr_dst(dc, r1)) {
        return;
    }

    switch (in->i_format) {
    case ASM_FMT_I10: {     /* shrp r1 = r2, r3, count6 */
        int cnt = field(b, 27, 6);

        if (!check_gr(dc, r2) || !check_gr(dc, r3)) {
            return;
        }
        if (cnt == 0) {
            tcg_gen_mov_i64(d, load_gr(dc, r3));
        } else {
            tcg_gen_extract2_i64(d, load_gr(dc, r3), load_gr(dc, r2), cnt);
        }
        nat = nat_or(load_nat(dc, r2), load_nat(dc, r3));
        break;
    }
    case ASM_FMT_I11:       /* extr[.u] r1 = r3, pos6, len6 */
        if (!check_gr(dc, r3)) {
            return;
        }
        pos = field(b, 14, 6);
        len = field(b, 27, 6) + 1;
        if (pos + len > 64) {
            len = 64 - pos;
        }
        if (field(b, 13, 1)) {
            tcg_gen_sextract_i64(d, load_gr(dc, r3), pos, len);
        } else {
            tcg_gen_extract_i64(d, load_gr(dc, r3), pos, len);
        }
        nat = load_nat(dc, r3);
        break;
    case ASM_FMT_I12:       /* dep.z r1 = r2, pos6, len6 */
    case ASM_FMT_I13:       /* dep.z r1 = imm8, pos6, len6 */
        pos = 63 - field(b, 20, 6);
        len = field(b, 27, 6) + 1;
        if (pos + len > 64) {
            len = 64 - pos;
        }
        if (in->i_format == ASM_FMT_I12) {
            if (!check_gr(dc, r2)) {
                return;
            }
            tcg_gen_deposit_z_i64(d, load_gr(dc, r2), pos, len);
            nat = load_nat(dc, r2);
        } else {
            tcg_gen_deposit_z_i64(d, tcg_constant_i64(in->i_oper[2].o_value),
                                  pos, len);
            nat = NULL;
        }
        break;
    case ASM_FMT_I14:       /* dep r1 = imm1, r3, pos6, len6 */
        if (!check_gr(dc, r3)) {
            return;
        }
        pos = 63 - field(b, 14, 6);
        len = field(b, 27, 6) + 1;
        if (pos + len > 64) {
            len = 64 - pos;
        }
        tcg_gen_deposit_i64(d, load_gr(dc, r3),
                            tcg_constant_i64(field(b, 36, 1) ? -1 : 0),
                            pos, len);
        nat = load_nat(dc, r3);
        break;
    case ASM_FMT_I15:       /* dep r1 = r2, r3, pos6, len4 */
        if (!check_gr(dc, r2) || !check_gr(dc, r3)) {
            return;
        }
        pos = 63 - field(b, 31, 6);
        len = field(b, 27, 4) + 1;
        if (pos + len > 64) {
            len = 64 - pos;
        }
        tcg_gen_deposit_i64(d, load_gr(dc, r3), load_gr(dc, r2), pos, len);
        nat = nat_or(load_nat(dc, r2), load_nat(dc, r3));
        break;
    default:
        gen_illegal(dc);
        return;
    }
    finish_gr(dc, r1, d, nat);
}

/* zxt, sxt, czx, popcnt */
static void trans_i_unary(DisasContext *dc, const struct asm_inst *in)
{
    uint64_t b = in->i_bits;
    int r1 = R1(b), r3 = R3(b);
    TCGv_i64 d = tcg_temp_new_i64();
    TCGv_i64 s;

    if (!check_gr_dst(dc, r1) || !check_gr(dc, r3)) {
        return;
    }
    s = load_gr(dc, r3);
    switch (in->i_iop) {
    case ASM_OP_ZXT1: tcg_gen_ext8u_i64(d, s); break;
    case ASM_OP_ZXT2: tcg_gen_ext16u_i64(d, s); break;
    case ASM_OP_ZXT4: tcg_gen_ext32u_i64(d, s); break;
    case ASM_OP_SXT1: tcg_gen_ext8s_i64(d, s); break;
    case ASM_OP_SXT2: tcg_gen_ext16s_i64(d, s); break;
    case ASM_OP_SXT4: tcg_gen_ext32s_i64(d, s); break;
    case ASM_OP_POPCNT: tcg_gen_ctpop_i64(d, s); break;
    case ASM_OP_CZX1_L: case ASM_OP_CZX1_R:
    case ASM_OP_CZX2_L: case ASM_OP_CZX2_R:
        gen_helper_czx(d, s, tcg_constant_i32(in->i_iop == ASM_OP_CZX2_L ||
                                              in->i_iop == ASM_OP_CZX2_R),
                       tcg_constant_i32(in->i_iop == ASM_OP_CZX1_L ||
                                        in->i_iop == ASM_OP_CZX2_L));
        break;
    default:
        gen_illegal(dc);
        return;
    }
    finish_gr(dc, r1, d, load_nat(dc, r3));
}

/* Multimedia: everything goes through one helper keyed by the opcode. */
static void trans_mm(DisasContext *dc, const struct asm_inst *in)
{
    uint64_t b = in->i_bits;
    int r1 = R1(b), r2 = R2(b), r3 = R3(b);
    TCGv_i64 d = tcg_temp_new_i64();
    TCGv_i64 a, c;
    TCGv_i32 nat;
    int imm = 0;

    if (!check_gr_dst(dc, r1)) {
        return;
    }
    switch (in->i_format) {
    case ASM_FMT_I3:        /* mux1 r1 = r2, mbtype4 */
    case ASM_FMT_I4:        /* mux2 r1 = r2, mhtype8 */
        if (!check_gr(dc, r2)) {
            return;
        }
        a = load_gr(dc, r2);
        c = tcg_constant_i64(0);
        nat = load_nat(dc, r2);
        imm = in->i_format == ASM_FMT_I3 ? field(b, 20, 4) : field(b, 20, 8);
        break;
    case ASM_FMT_I5:        /* pshr r1 = r3, r2 */
        if (!check_gr(dc, r2) || !check_gr(dc, r3)) {
            return;
        }
        a = load_gr(dc, r3);
        c = load_gr(dc, r2);
        nat = nat_or(load_nat(dc, r2), load_nat(dc, r3));
        imm = -1;
        break;
    case ASM_FMT_I6:        /* pshr r1 = r3, count5 */
        if (!check_gr(dc, r3)) {
            return;
        }
        a = load_gr(dc, r3);
        c = tcg_constant_i64(field(b, 14, 5));
        nat = load_nat(dc, r3);
        imm = -1;
        break;
    case ASM_FMT_I7:        /* pshl r1 = r2, r3 */
        if (!check_gr(dc, r2) || !check_gr(dc, r3)) {
            return;
        }
        a = load_gr(dc, r2);
        c = load_gr(dc, r3);
        nat = nat_or(load_nat(dc, r2), load_nat(dc, r3));
        imm = -1;
        break;
    case ASM_FMT_I8:        /* pshl r1 = r2, count5 */
        if (!check_gr(dc, r2)) {
            return;
        }
        a = load_gr(dc, r2);
        c = tcg_constant_i64(31 - field(b, 20, 5));
        nat = load_nat(dc, r2);
        imm = -1;
        break;
    case ASM_FMT_I1:        /* pmpyshr2 r1 = r2, r3, count2 */
        imm = (int[]){ 0, 7, 15, 16 }[field(b, 30, 2)];
        /* fall through */
    case ASM_FMT_I2:
    case ASM_FMT_A9:
    case ASM_FMT_A10:
        if (!check_gr(dc, r2) || !check_gr(dc, r3)) {
            return;
        }
        a = load_gr(dc, r2);
        c = load_gr(dc, r3);
        nat = nat_or(load_nat(dc, r2), load_nat(dc, r3));
        if (in->i_format == ASM_FMT_A10) {
            imm = field(b, 27, 2) + 1;
        }
        break;
    default:
        gen_illegal(dc);
        return;
    }
    gen_helper_mm(d, tcg_env, tcg_constant_i32(in->i_iop), a, c,
                  tcg_constant_i32(imm));
    finish_gr(dc, r1, d, nat);
}

static void trans_a(DisasContext *dc, const struct asm_inst *in)
{
    switch (in->i_format) {
    case ASM_FMT_A1:
    case ASM_FMT_A3:
        trans_a_alu(dc, in);
        break;
    case ASM_FMT_A2:
        trans_a_shladd(dc, in);
        break;
    case ASM_FMT_A4:
    case ASM_FMT_A5:
        trans_a_addimm(dc, in);
        break;
    case ASM_FMT_A6:
    case ASM_FMT_A7:
    case ASM_FMT_A8:
        trans_a_cmp(dc, in);
        break;
    case ASM_FMT_A9:
    case ASM_FMT_A10:
        trans_mm(dc, in);
        break;
    default:
        gen_illegal(dc);
        break;
    }
}

/* mov pr = r2, mask17 / mov pr.rot = imm44: physical predicate order. */
static void trans_i_movpr(DisasContext *dc, const struct asm_inst *in)
{
    uint64_t b = in->i_bits;
    uint64_t mask;
    TCGv_i64 src;

    if (in->i_format == ASM_FMT_I23) {
        int r2 = R2(b);

        if (!check_gr(dc, r2)) {
            return;
        }
        mask = (field(b, 6, 7) << 1) | (field(b, 24, 8) << 8);
        if (field(b, 36, 1)) {
            mask |= ~0ull << 16;
        }
        gen_nat_check(dc, load_nat(dc, r2), 0);
        src = load_gr(dc, r2);
    } else {
        uint64_t imm = sextract64(field(b, 6, 27) | (field(b, 36, 1) << 27),
                                  0, 28) << 16;

        mask = ~0ull << 16;
        src = tcg_constant_i64(imm);
    }
    gen_helper_mov_to_pr(tcg_env, src, tcg_constant_i64(mask));
}

static void trans_i_movfrom(DisasContext *dc, const struct asm_inst *in)
{
    uint64_t b = in->i_bits;
    int r1 = R1(b);
    TCGv_i64 d = tcg_temp_new_i64();

    if (!check_gr_dst(dc, r1)) {
        return;
    }
    switch (in->i_iop) {
    case ASM_OP_MOV_IP:
        tcg_gen_movi_i64(d, dc->ip);
        break;
    case ASM_OP_MOV_PR:
        gen_helper_mov_from_pr(d, tcg_env);
        break;
    case ASM_OP_MOV_:       /* I22: mov r1 = b2 */
        tcg_gen_mov_i64(d, cpu_br[field(b, 13, 3)]);
        break;
    default:
        gen_illegal(dc);
        return;
    }
    finish_gr(dc, r1, d, NULL);
}

/* mov b1 = r2 (I21) */
static void trans_i_movbr(DisasContext *dc, const struct asm_inst *in)
{
    uint64_t b = in->i_bits;
    int r2 = R2(b);

    if (!check_gr(dc, r2)) {
        return;
    }
    gen_nat_check(dc, load_nat(dc, r2), 0);
    tcg_gen_mov_i64(cpu_br[field(b, 6, 3)], load_gr(dc, r2));
}

static void gen_mov_to_ar(DisasContext *dc, int ar, TCGv_i64 v);
static void gen_mov_from_ar(DisasContext *dc, int r1, int ar);

static void trans_i_movar(DisasContext *dc, const struct asm_inst *in)
{
    uint64_t b = in->i_bits;
    int ar = field(b, 20, 7);

    if (ar < 64) {
        /* M-unit registers through the I unit */
        gen_illegal(dc);
        return;
    }
    if (in->i_format == ASM_FMT_I28) {
        gen_mov_from_ar(dc, R1(b), ar);
    } else if (in->i_format == ASM_FMT_I26) {
        int r2 = R2(b);

        if (!check_gr(dc, r2)) {
            return;
        }
        gen_nat_check(dc, load_nat(dc, r2), 0);
        gen_mov_to_ar(dc, ar, load_gr(dc, r2));
    } else {
        gen_mov_to_ar(dc, ar, tcg_constant_i64(in->i_oper[2].o_value));
    }
}

static void gen_break(DisasContext *dc, uint64_t imm)
{
    gen_save_ip(dc);
    gen_helper_break(tcg_env, tcg_constant_i32(imm));
    /* only a firmware call returns: ip/ri are set by the helper */
    dc->base.is_jmp = DISAS_EXIT;
}

/* chk.s r2 / chk.s f2: branch to recovery code when the source is NaT. */
static void gen_chk(DisasContext *dc, TCGv_i32 nat, int64_t disp)
{
    TCGLabel *ok = gen_new_label();

    tcg_gen_brcondi_i32(TCG_COND_EQ, nat, 0, ok);
    tcg_gen_movi_i64(cpu_ip, dc->ip + disp);
    tcg_gen_movi_i32(cpu_ri, 0);
    tcg_gen_lookup_and_goto_ptr();
    gen_set_label(ok);
}

static void trans_i(DisasContext *dc, const struct asm_inst *in)
{
    switch (in->i_format) {
    case ASM_FMT_I1: case ASM_FMT_I2: case ASM_FMT_I3: case ASM_FMT_I4:
    case ASM_FMT_I6: case ASM_FMT_I8:
        trans_mm(dc, in);
        break;
    case ASM_FMT_I5:
        if (in->i_iop == ASM_OP_SHR_ || in->i_iop == ASM_OP_SHR_U) {
            trans_i_shift(dc, in);
        } else {
            trans_mm(dc, in);
        }
        break;
    case ASM_FMT_I7:
        if (in->i_iop == ASM_OP_SHL) {
            trans_i_shift(dc, in);
        } else {
            trans_mm(dc, in);
        }
        break;
    case ASM_FMT_I9:
    case ASM_FMT_I29:
        trans_i_unary(dc, in);
        break;
    case ASM_FMT_I10: case ASM_FMT_I11: case ASM_FMT_I12: case ASM_FMT_I13:
    case ASM_FMT_I14: case ASM_FMT_I15:
        trans_i_bitfield(dc, in);
        break;
    case ASM_FMT_I16: case ASM_FMT_I17: case ASM_FMT_I30:
        trans_i_test(dc, in);
        break;
    case ASM_FMT_I18:       /* nop.i, hint.i */
        break;
    case ASM_FMT_I19:
        gen_break(dc, field(in->i_bits, 6, 20) |
                      (field(in->i_bits, 36, 1) << 20));
        break;
    case ASM_FMT_I20:
        if (check_gr(dc, R2(in->i_bits))) {
            gen_chk(dc, load_nat(dc, R2(in->i_bits)), in->i_oper[2].o_value);
        }
        break;
    case ASM_FMT_I21:
        trans_i_movbr(dc, in);
        break;
    case ASM_FMT_I22:
    case ASM_FMT_I25:
        trans_i_movfrom(dc, in);
        break;
    case ASM_FMT_I23:
    case ASM_FMT_I24:
        trans_i_movpr(dc, in);
        break;
    case ASM_FMT_I26: case ASM_FMT_I27: case ASM_FMT_I28:
        trans_i_movar(dc, in);
        break;
    default:
        gen_illegal(dc);
        break;
    }
}
