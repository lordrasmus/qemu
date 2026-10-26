/*
 * IA-64 translation: F unit and floating-point memory access
 *
 * All arithmetic is in fpu_helper.c; the translator resolves register
 * rotation and checks psr.dfl/dfh.
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

/* Must match fpu_helper.c */
enum { FK_S, FK_D, FK_E, FK_8, FK_FILL, FK_EXP, FK_SIG };
enum {
    FM_MERGE_S, FM_MERGE_NS, FM_MERGE_SE,
    FM_MIX_L, FM_MIX_R, FM_MIX_LR,
    FM_SXT_L, FM_SXT_R,
    FM_PACK, FM_SWAP, FM_SWAP_NL, FM_SWAP_NR,
    FM_AND, FM_ANDCM, FM_OR, FM_XOR,
    FM_PMERGE_S, FM_PMERGE_NS, FM_PMERGE_SE,
    FM_SELECT,
};

#define F1(b)   ((int)field(b, 6, 7))
#define F2(b)   ((int)field(b, 13, 7))
#define F3(b)   ((int)field(b, 20, 7))
#define F4(b)   ((int)field(b, 27, 7))
#define SF(b)   ((int)field(b, 34, 2))

/*
 * Disabled FP register check for the logical registers used; f0/f1 are
 * never disabled.  Returns false after raising the fault.
 */
static bool fp_access(DisasContext *dc, int n, const int *regs)
{
    int code = 0;

    for (int i = 0; i < n; i++) {
        if (regs[i] >= 2 && regs[i] < 32 &&
            FIELD_EX32(dc->tbflags, TBFLAGS, DFL)) {
            code |= 1;
        }
        if (regs[i] >= 32 && FIELD_EX32(dc->tbflags, TBFLAGS, DFH)) {
            code |= 2;
        }
    }
    if (code) {
        gen_fault(dc, IVT_DISABLED_FP, code);
        return false;
    }
    return true;
}

static bool fp_dest(DisasContext *dc, int f)
{
    if (f < 2) {
        gen_illegal(dc);
        return false;
    }
    return true;
}

/* Pack physical register numbers for the helpers. */
static uint32_t fregs(DisasContext *dc, int f1, int f2, int f3, int f4)
{
    return fr_idx(dc, f1) | (fr_idx(dc, f2) << 8) |
           (fr_idx(dc, f3) << 16) | (fr_idx(dc, f4) << 24);
}

static TCGv_i32 fr_natval(DisasContext *dc, int f)
{
    TCGv_i32 t = tcg_temp_new_i32();

    gen_helper_fr_is_natval(t, tcg_env, tcg_constant_i32(fr_idx(dc, f)));
    return t;
}

static void gen_chk_f(DisasContext *dc, int f2, int64_t disp)
{
    int r[1] = { f2 };

    if (fp_access(dc, 1, r)) {
        gen_chk(dc, fr_natval(dc, f2), disp);
    }
}

static void trans_f_arith(DisasContext *dc, const struct asm_inst *in)
{
    uint64_t b = in->i_bits;
    int f1 = F1(b), f2 = F2(b), f3 = F3(b), f4 = F4(b);
    int r[4] = { f1, f2, f3, f4 };
    int kind, comp = 0;
    bool par = false;
    uint32_t regs = fregs(dc, f1, f2, f3, f4);

    if (!fp_access(dc, 4, r) || !fp_dest(dc, f1)) {
        return;
    }
    switch (in->i_iop) {
    case ASM_OP_FMA_: kind = 0; break;
    case ASM_OP_FMA_S: kind = 0; comp = 1; break;
    case ASM_OP_FMA_D: kind = 0; comp = 2; break;
    case ASM_OP_FMS_: kind = 1; break;
    case ASM_OP_FMS_S: kind = 1; comp = 1; break;
    case ASM_OP_FMS_D: kind = 1; comp = 2; break;
    case ASM_OP_FNMA_: kind = 2; break;
    case ASM_OP_FNMA_S: kind = 2; comp = 1; break;
    case ASM_OP_FNMA_D: kind = 2; comp = 2; break;
    case ASM_OP_FPMA: kind = 0; par = true; break;
    case ASM_OP_FPMS: kind = 1; par = true; break;
    case ASM_OP_FPNMA: kind = 2; par = true; break;
    default:
        gen_illegal(dc);
        return;
    }
    if (par) {
        gen_helper_fpma(tcg_env, tcg_constant_i32(regs),
                        tcg_constant_i32(kind | SF(b) << 8));
    } else {
        gen_helper_fma(tcg_env, tcg_constant_i32(regs),
                       tcg_constant_i32(kind | comp << 4 | SF(b) << 8));
    }
}

static void trans_f_cmp(DisasContext *dc, const struct asm_inst *in)
{
    uint64_t b = in->i_bits;
    int p1 = P1(b), p2 = P2(b), f2 = F2(b), f3 = F3(b);
    int r[2] = { f2, f3 };
    int ctype = field(b, 12, 1) ? CT_UNC : CT_NONE;
    TCGv_i32 v = tcg_temp_new_i32();
    TCGv_i32 res = tcg_temp_new_i32();
    TCGv_i32 nat = tcg_temp_new_i32();

    if (!fp_access(dc, 2, r)) {
        return;
    }
    if (ctype == CT_UNC && p1 == p2) {
        gen_illegal(dc);
        return;
    }
    if (in->i_format == ASM_FMT_F4) {
        int rel = field(b, 33, 1) << 1 | field(b, 36, 1);

        /* ra rb: 00 eq, 01 lt, 10 le, 11 unord */
        gen_helper_fcmp(v, tcg_env, tcg_constant_i32(fregs(dc, 0, f2, f3, 0)),
                        tcg_constant_i32(rel | SF(b) << 8));
    } else {
        gen_helper_fclass(v, tcg_env, tcg_constant_i32(fr_idx(dc, f2)),
                          tcg_constant_i32(in->i_oper[4].o_value));
    }
    tcg_gen_andi_i32(res, v, 1);
    tcg_gen_shri_i32(nat, v, 1);
    gen_write_cmp(dc, ctype, p1, p2, res, unc_nat(dc, ctype, nat));
}

/* frcpa, frsqrta and the parallel forms: p2 cleared when qp is false */
static void trans_f_approx(DisasContext *dc, const struct asm_inst *in)
{
    uint64_t b = in->i_bits;
    int qp = field(b, 0, 6);
    int f1 = F1(b), f2 = F2(b), f3 = F3(b), p2 = P2(b);
    int r[3] = { f1, f2, f3 };
    TCGv_i32 v = tcg_temp_new_i32();
    TCGLabel *skip = NULL, *done = gen_new_label();
    uint32_t regs = fregs(dc, f1, f2, f3, 0);
    bool rsqrt = in->i_iop == ASM_OP_FRSQRTA || in->i_iop == ASM_OP_FPRSQRTA;

    if (qp) {
        skip = gen_new_label();
        tcg_gen_brcondi_i32(TCG_COND_EQ, cpu_pr[pr_idx(dc, qp)], 0, skip);
    }
    if (!fp_access(dc, 3, r) || !fp_dest(dc, f1)) {
        goto out;
    }
    switch (in->i_iop) {
    case ASM_OP_FRCPA:
        gen_helper_frcpa(v, tcg_env, tcg_constant_i32(regs),
                         tcg_constant_i32(SF(b)));
        break;
    case ASM_OP_FRSQRTA:
        gen_helper_frsqrta(v, tcg_env, tcg_constant_i32(regs),
                           tcg_constant_i32(SF(b)));
        break;
    default:
        gen_helper_fprcpa(v, tcg_env, tcg_constant_i32(regs),
                          tcg_constant_i32(rsqrt | SF(b) << 8));
        break;
    }
    store_pr(dc, p2, v);
out:
    tcg_gen_br(done);
    if (skip) {
        gen_set_label(skip);
        store_pr_imm(dc, p2, 0);
    }
    gen_set_label(done);
}

static void trans_f_minmax(DisasContext *dc, const struct asm_inst *in)
{
    uint64_t b = in->i_bits;
    int f1 = F1(b), f2 = F2(b), f3 = F3(b);
    int r[3] = { f1, f2, f3 };
    uint32_t regs = fregs(dc, f1, f2, f3, 0);
    int op, rel;

    if (!fp_access(dc, 3, r) || !fp_dest(dc, f1)) {
        return;
    }
    switch (in->i_iop) {
    case ASM_OP_FMIN: op = 0; goto scalar;
    case ASM_OP_FMAX: op = 1; goto scalar;
    case ASM_OP_FAMIN: op = 2; goto scalar;
    case ASM_OP_FAMAX: op = 3;
    scalar:
        gen_helper_fminmax(tcg_env, tcg_constant_i32(regs),
                           tcg_constant_i32(op | SF(b) << 8));
        return;
    case ASM_OP_FPMIN: op = 0; goto par;
    case ASM_OP_FPMAX: op = 1; goto par;
    case ASM_OP_FPAMIN: op = 2; goto par;
    case ASM_OP_FPAMAX: op = 3;
    par:
        gen_helper_fpminmax(tcg_env, tcg_constant_i32(regs),
                            tcg_constant_i32(op | SF(b) << 8));
        return;
    case ASM_OP_FPCMP_EQ: rel = 0; break;
    case ASM_OP_FPCMP_LT: rel = 1; break;
    case ASM_OP_FPCMP_LE: rel = 2; break;
    case ASM_OP_FPCMP_UNORD: rel = 3; break;
    case ASM_OP_FPCMP_NEQ: rel = 4; break;
    case ASM_OP_FPCMP_NLT: rel = 5; break;
    case ASM_OP_FPCMP_NLE: rel = 6; break;
    case ASM_OP_FPCMP_ORD: rel = 7; break;
    default:
        gen_illegal(dc);
        return;
    }
    gen_helper_fpcmp(tcg_env, tcg_constant_i32(regs),
                     tcg_constant_i32(rel | SF(b) << 8));
}

static void trans_f_misc(DisasContext *dc, const struct asm_inst *in)
{
    uint64_t b = in->i_bits;
    int f1 = F1(b), f2 = F2(b), f3 = F3(b);
    int r[3] = { f1, f2, f3 };
    int op;

    if (!fp_access(dc, 3, r) || !fp_dest(dc, f1)) {
        return;
    }
    switch (in->i_iop) {
    case ASM_OP_FMERGE_S: op = FM_MERGE_S; break;
    case ASM_OP_FMERGE_NS: op = FM_MERGE_NS; break;
    case ASM_OP_FMERGE_SE: op = FM_MERGE_SE; break;
    case ASM_OP_FMIX_L: op = FM_MIX_L; break;
    case ASM_OP_FMIX_R: op = FM_MIX_R; break;
    case ASM_OP_FMIX_LR: op = FM_MIX_LR; break;
    case ASM_OP_FSXT_L: op = FM_SXT_L; break;
    case ASM_OP_FSXT_R: op = FM_SXT_R; break;
    case ASM_OP_FPACK: op = FM_PACK; break;
    case ASM_OP_FSWAP_: op = FM_SWAP; break;
    case ASM_OP_FSWAP_NL: op = FM_SWAP_NL; break;
    case ASM_OP_FSWAP_NR: op = FM_SWAP_NR; break;
    case ASM_OP_FAND: op = FM_AND; break;
    case ASM_OP_FANDCM: op = FM_ANDCM; break;
    case ASM_OP_FOR: op = FM_OR; break;
    case ASM_OP_FXOR: op = FM_XOR; break;
    case ASM_OP_FPMERGE_S: op = FM_PMERGE_S; break;
    case ASM_OP_FPMERGE_NS: op = FM_PMERGE_NS; break;
    case ASM_OP_FPMERGE_SE: op = FM_PMERGE_SE; break;
    default:
        gen_illegal(dc);
        return;
    }
    gen_helper_fmisc(tcg_env, tcg_constant_i32(fregs(dc, f1, f2, f3, 0)),
                     tcg_constant_i32(op));
}

static void trans_f_cvt(DisasContext *dc, const struct asm_inst *in)
{
    uint64_t b = in->i_bits;
    int f1 = F1(b), f2 = F2(b);
    int r[2] = { f1, f2 };
    uint32_t regs = fregs(dc, f1, f2, 0, 0);
    int ctl;

    if (!fp_access(dc, 2, r) || !fp_dest(dc, f1)) {
        return;
    }
    switch (in->i_iop) {
    case ASM_OP_FCVT_XF:
        gen_helper_fcvt_xf(tcg_env, tcg_constant_i32(regs));
        return;
    case ASM_OP_FCVT_FX: case ASM_OP_FPCVT_FX: ctl = 0; break;
    case ASM_OP_FCVT_FXU: case ASM_OP_FPCVT_FXU: ctl = 1; break;
    case ASM_OP_FCVT_FX_TRUNC: case ASM_OP_FPCVT_FX_TRUNC: ctl = 2; break;
    default: ctl = 3; break;
    }
    ctl |= SF(b) << 8;
    if (in->i_iop >= ASM_OP_FPCVT_FX && in->i_iop <= ASM_OP_FPCVT_FXU_TRUNC) {
        gen_helper_fpcvt(tcg_env, tcg_constant_i32(regs),
                         tcg_constant_i32(ctl));
    } else {
        gen_helper_fcvt_fx(tcg_env, tcg_constant_i32(regs),
                           tcg_constant_i32(ctl));
    }
}

static void trans_f(DisasContext *dc, const struct asm_inst *in)
{
    uint64_t b = in->i_bits;

    switch (in->i_format) {
    case ASM_FMT_F1:
        trans_f_arith(dc, in);
        break;
    case ASM_FMT_F2: {
        int r[4] = { F1(b), F2(b), F3(b), F4(b) };
        int kind = in->i_iop == ASM_OP_XMA_L ? 0 :
                   in->i_iop == ASM_OP_XMA_H ? 1 : 2;

        if (fp_access(dc, 4, r) && fp_dest(dc, F1(b))) {
            gen_helper_xma(tcg_env,
                           tcg_constant_i32(fregs(dc, F1(b), F2(b), F3(b),
                                                  F4(b))),
                           tcg_constant_i32(kind));
        }
        break;
    }
    case ASM_FMT_F3: {
        int r[4] = { F1(b), F2(b), F3(b), F4(b) };

        if (fp_access(dc, 4, r) && fp_dest(dc, F1(b))) {
            gen_helper_fselect(tcg_env,
                               tcg_constant_i32(fregs(dc, F1(b), F2(b),
                                                      F3(b), F4(b))));
        }
        break;
    }
    case ASM_FMT_F4:
    case ASM_FMT_F5:
        trans_f_cmp(dc, in);
        break;
    case ASM_FMT_F6:
    case ASM_FMT_F7:
        trans_f_approx(dc, in);
        break;
    case ASM_FMT_F8:
        trans_f_minmax(dc, in);
        break;
    case ASM_FMT_F9:
        trans_f_misc(dc, in);
        break;
    case ASM_FMT_F10:
    case ASM_FMT_F11:
        trans_f_cvt(dc, in);
        break;
    case ASM_FMT_F12:
        gen_helper_fsetc(tcg_env, tcg_constant_i32(SF(b)),
                         tcg_constant_i32(field(b, 13, 7) |
                                          field(b, 20, 7) << 8));
        break;
    case ASM_FMT_F13:
        gen_helper_fclrf(tcg_env, tcg_constant_i32(SF(b)));
        break;
    case ASM_FMT_F14: {
        TCGv_i32 v = tcg_temp_new_i32();

        gen_helper_fchkf(v, tcg_env, tcg_constant_i32(SF(b)));
        gen_chk(dc, v, in->i_oper[1].o_value);
        break;
    }
    case ASM_FMT_F15:
        gen_break(dc, field(b, 6, 20) | (field(b, 36, 1) << 20));
        break;
    case ASM_FMT_F16:       /* nop.f, hint.f */
        break;
    default:
        gen_illegal(dc);
        break;
    }
}

/*
 * Floating-point loads, stores, setf, getf
 */

static int ldf_kind(int iop, bool *spec)
{
    *spec = false;
    switch (iop) {
    case ASM_OP_LDFS_S: case ASM_OP_LDFS_SA: case ASM_OP_LDFPS_S:
    case ASM_OP_LDFPS_SA:
        *spec = true;
        /* fall through */
    case ASM_OP_LDFS_ ... ASM_OP_LDFS_C_NC:
    case ASM_OP_LDFPS_ ... ASM_OP_LDFPS_C_NC:
        return FK_S;
    case ASM_OP_LDFD_S: case ASM_OP_LDFD_SA: case ASM_OP_LDFPD_S:
    case ASM_OP_LDFPD_SA:
        *spec = true;
        /* fall through */
    case ASM_OP_LDFD_ ... ASM_OP_LDFD_C_NC:
    case ASM_OP_LDFPD_ ... ASM_OP_LDFPD_C_NC:
        return FK_D;
    case ASM_OP_LDFE_S: case ASM_OP_LDFE_SA:
        *spec = true;
        /* fall through */
    case ASM_OP_LDFE_ ... ASM_OP_LDFE_C_NC:
        return FK_E;
    case ASM_OP_LDF8_S: case ASM_OP_LDF8_SA: case ASM_OP_LDFP8_S:
    case ASM_OP_LDFP8_SA:
        *spec = true;
        /* fall through */
    case ASM_OP_LDF8_ ... ASM_OP_LDF8_C_NC:
    case ASM_OP_LDFP8_ ... ASM_OP_LDFP8_C_NC:
        return FK_8;
    case ASM_OP_LDF_FILL:
        return FK_FILL;
    }
    return -1;
}

static void trans_m_fp(DisasContext *dc, const struct asm_inst *in)
{
    uint64_t b = in->i_bits;
    int r2 = R2(b), r3 = R3(b);
    TCGv_i64 addr = tcg_temp_new_i64();
    TCGv_i64 inc = NULL;
    TCGv_i32 inc_nat = NULL;
    bool spec;
    int kind;

    switch (in->i_format) {
    case ASM_FMT_M6: case ASM_FMT_M7: case ASM_FMT_M8: {
        int f1 = F1(b);
        int r[1] = { f1 };
        bool upd = in->i_format != ASM_FMT_M6;

        kind = ldf_kind(in->i_iop, &spec);
        if (kind < 0 || !fp_access(dc, 1, r) || !fp_dest(dc, f1) ||
            !check_gr(dc, r3)) {
            if (kind < 0) {
                gen_illegal(dc);
            }
            return;
        }
        if (upd && r3 == 0) {
            gen_illegal(dc);
            return;
        }
        if (!get_post_inc(dc, in->i_format == ASM_FMT_M7, r2,
                          in->i_format == ASM_FMT_M8 ?
                          in->i_oper[3].o_value : 0, upd, &inc, &inc_nat)) {
            return;
        }
        tcg_gen_mov_i64(addr, load_gr(dc, r3));
        gen_save_ip(dc);
        if (spec) {
            TCGLabel *ok = gen_new_label(), *done = gen_new_label();

            tcg_gen_brcondi_i32(TCG_COND_EQ, load_nat(dc, r3), 0, ok);
            gen_helper_setf(tcg_env, tcg_constant_i32(fr_idx(dc, f1)),
                            tcg_constant_i64(0), tcg_constant_i32(16));
            tcg_gen_br(done);
            gen_set_label(ok);
            gen_helper_ldf(tcg_env, tcg_constant_i32(fr_idx(dc, f1)), addr,
                           tcg_constant_i32(kind | 16),
                           tcg_constant_i32(dc->mem_idx));
            gen_set_label(done);
        } else {
            gen_nat_check(dc, load_nat(dc, r3), ISR_R);
            gen_helper_ldf(tcg_env, tcg_constant_i32(fr_idx(dc, f1)), addr,
                           tcg_constant_i32(kind),
                           tcg_constant_i32(dc->mem_idx));
        }
        if (upd) {
            gen_post_inc(dc, r3, addr, inc, inc_nat);
        }
        break;
    }
    case ASM_FMT_M11: case ASM_FMT_M12: {
        int f1 = F1(b), f2 = F2(b);
        int r[2] = { f1, f2 };
        bool upd = in->i_format == ASM_FMT_M12;

        kind = ldf_kind(in->i_iop, &spec);
        if (kind < 0 || !fp_access(dc, 2, r) || !fp_dest(dc, f1) ||
            !fp_dest(dc, f2) || !check_gr(dc, r3)) {
            if (kind < 0) {
                gen_illegal(dc);
            }
            return;
        }
        if (f1 == f2 || (upd && r3 == 0)) {
            gen_illegal(dc);
            return;
        }
        tcg_gen_mov_i64(addr, load_gr(dc, r3));
        gen_save_ip(dc);
        if (!spec) {
            gen_nat_check(dc, load_nat(dc, r3), ISR_R);
        }
        gen_helper_ldfp(tcg_env,
                        tcg_constant_i32(fregs(dc, f1, f2, 0, 0)), addr,
                        tcg_constant_i32(kind | (spec ? 16 : 0)),
                        tcg_constant_i32(dc->mem_idx));
        if (upd) {
            gen_post_inc(dc, r3, addr,
                         tcg_constant_i64(in->i_oper[4].o_value), NULL);
        }
        break;
    }
    case ASM_FMT_M9: case ASM_FMT_M10: {
        int f2 = F2(b);
        int r[1] = { f2 };
        bool upd = in->i_format == ASM_FMT_M10;

        switch (in->i_iop) {
        case ASM_OP_STFS: kind = FK_S; break;
        case ASM_OP_STFD: kind = FK_D; break;
        case ASM_OP_STFE: kind = FK_E; break;
        case ASM_OP_STF8: kind = FK_8; break;
        default: kind = FK_FILL; break;
        }
        if (!fp_access(dc, 1, r) || !check_gr(dc, r3)) {
            return;
        }
        if (upd && r3 == 0) {
            gen_illegal(dc);
            return;
        }
        tcg_gen_mov_i64(addr, load_gr(dc, r3));
        gen_nat_check(dc, load_nat(dc, r3), ISR_W);
        gen_save_ip(dc);
        gen_helper_stf(tcg_env, tcg_constant_i32(fr_idx(dc, f2)), addr,
                       tcg_constant_i32(kind), tcg_constant_i32(dc->mem_idx));
        if (upd) {
            gen_post_inc(dc, r3, addr,
                         tcg_constant_i64(in->i_oper[3].o_value), NULL);
        }
        break;
    }
    case ASM_FMT_M18: {     /* setf f1 = r2 */
        int f1 = F1(b);
        int r[1] = { f1 };
        TCGv_i32 ctl = tcg_temp_new_i32();

        switch (in->i_iop) {
        case ASM_OP_SETF_S: kind = FK_S; break;
        case ASM_OP_SETF_D: kind = FK_D; break;
        case ASM_OP_SETF_EXP: kind = FK_EXP; break;
        default: kind = FK_SIG; break;
        }
        if (!fp_access(dc, 1, r) || !fp_dest(dc, f1) || !check_gr(dc, r2)) {
            return;
        }
        tcg_gen_shli_i32(ctl, load_nat(dc, r2), 4);
        tcg_gen_ori_i32(ctl, ctl, kind);
        gen_helper_setf(tcg_env, tcg_constant_i32(fr_idx(dc, f1)),
                        load_gr(dc, r2), ctl);
        break;
    }
    case ASM_FMT_M19: {     /* getf r1 = f2 */
        int r1 = R1(b), f2 = F2(b);
        int r[1] = { f2 };
        TCGv_i64 d = tcg_temp_new_i64();

        switch (in->i_iop) {
        case ASM_OP_GETF_S: kind = FK_S; break;
        case ASM_OP_GETF_D: kind = FK_D; break;
        case ASM_OP_GETF_EXP: kind = FK_EXP; break;
        default: kind = FK_SIG; break;
        }
        if (!fp_access(dc, 1, r) || !check_gr_dst(dc, r1)) {
            return;
        }
        gen_helper_getf(d, tcg_env, tcg_constant_i32(fr_idx(dc, f2)),
                        tcg_constant_i32(kind));
        store_gr(dc, r1, d, fr_natval(dc, f2));
        break;
    }
    default:
        gen_illegal(dc);
        break;
    }
}
