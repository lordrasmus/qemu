/*
 * C674x floating point instructions (the C67x+ part), included by
 * translate.c.  Latencies and read stages from SPRUFE8B.
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

/*
 * A double in a register pair: the even register holds the low word.
 * khi is the cycle offset at which the high word is read.
 */
static TCGv_i64 dp_own(DisasContext *dc, int r, int khi)
{
    return gpair(dc, dc->s, r, khi);
}

static TCGv_i64 dp_x(DisasContext *dc, int r, int khi)
{
    return gpair(dc, dc->s ^ F_X(dc->insn), r, khi);
}

/* DP operand of the unary ops: src1 names the even, src2 the odd register */
static TCGv_i64 dp_unary(DisasContext *dc)
{
    TCGv_i64 v = tmp64();

    tcg_gen_concat_i32_i64(v, gr(dc, F_SRC1(dc->insn)),
                           gr(dc, F_SRC2(dc->insn)));
    return v;
}

typedef void GenF32(TCGv_i32, TCGv_env, TCGv_i32, TCGv_i32);
typedef void GenF64(TCGv_i64, TCGv_env, TCGv_i64, TCGv_i64);

static void fp_sp(DisasContext *dc, GenF32 *fn, TCGv_i32 a, TCGv_i32 b,
                  int delay)
{
    op2_sat(dc, fn, a, b, delay);
}

static void fp_dp(DisasContext *dc, GenF64 *fn, TCGv_i64 a, TCGv_i64 b,
                  int dlo, int dhi)
{
    TCGv_i64 r = tmp64();
    TCGLabel *l = guard(dc);

    fn(r, tcg_env, a, b);
    unguard(l);
    wr_pair64(dc, dc->s, F_DST(dc->insn), r, dlo, dhi);
}

static void fp_dp_cmp(DisasContext *dc, int op, TCGv_i64 a, TCGv_i64 b)
{
    TCGv_i32 r = tmp();
    TCGLabel *l = guard(dc);

    gen_helper_cmpdp(r, tcg_env, a, b, cst(op));
    unguard(l);
    wr(dc, F_DST(dc->insn), r, 1);
}

static void fp_sp_cmp(DisasContext *dc, int op, TCGv_i32 a, TCGv_i32 b)
{
    TCGv_i32 r = tmp();
    TCGLabel *l = guard(dc);

    gen_helper_cmpsp(r, tcg_env, a, b, cst(op));
    unguard(l);
    wr(dc, F_DST(dc->insn), r, 0);
}

typedef void GenCvt32(TCGv_i32, TCGv_env, TCGv_i32);
typedef void GenCvtD2S(TCGv_i32, TCGv_env, TCGv_i64);
typedef void GenCvtS2D(TCGv_i64, TCGv_env, TCGv_i32);
typedef void GenCvtD2D(TCGv_i64, TCGv_env, TCGv_i64);

static void fp_cvt32(DisasContext *dc, GenCvt32 *fn, TCGv_i32 a, int delay)
{
    TCGv_i32 r = tmp();
    TCGLabel *l = guard(dc);

    fn(r, tcg_env, a);
    unguard(l);
    wr(dc, F_DST(dc->insn), r, delay);
}

static void fp_d2s(DisasContext *dc, GenCvtD2S *fn, TCGv_i64 a, int delay)
{
    TCGv_i32 r = tmp();
    TCGLabel *l = guard(dc);

    fn(r, tcg_env, a);
    unguard(l);
    wr(dc, F_DST(dc->insn), r, delay);
}

static void fp_s2d(DisasContext *dc, GenCvtS2D *fn, TCGv_i32 a, int dlo,
                   int dhi)
{
    TCGv_i64 r = tmp64();
    TCGLabel *l = guard(dc);

    fn(r, tcg_env, a);
    unguard(l);
    wr_pair64(dc, dc->s, F_DST(dc->insn), r, dlo, dhi);
}

static void fp_d2d(DisasContext *dc, GenCvtD2D *fn, TCGv_i64 a, int dlo,
                   int dhi)
{
    TCGv_i64 r = tmp64();
    TCGLabel *l = guard(dc);

    fn(r, tcg_env, a);
    unguard(l);
    wr_pair64(dc, dc->s, F_DST(dc->insn), r, dlo, dhi);
}

/* .L unit format, both .L and (ops 0x70-0x77) .S */
static bool trans_fp_l(DisasContext *dc, int op)
{
    int src2 = F_SRC2(dc->insn), src1 = F_SRC1(dc->insn);

    switch (op) {
    case 0x01: fp_d2s(dc, gen_helper_dptrunc, dp_unary(dc), 3); return true;
    case 0x08: fp_d2s(dc, gen_helper_dpint, dp_unary(dc), 3); return true;
    case 0x09: fp_d2s(dc, gen_helper_dpsp, dp_unary(dc), 3); return true;
    case 0x0a: fp_cvt32(dc, gen_helper_spint, XR(src2), 3); return true;
    case 0x0b: fp_cvt32(dc, gen_helper_sptrunc, XR(src2), 3); return true;
    case 0x10:
    case 0x70:
        fp_sp(dc, gen_helper_addsp, RR(src1), XR(src2), 3);
        return true;
    case 0x11:
    case 0x71:
        fp_sp(dc, gen_helper_subsp, RR(src1), XR(src2), 3);
        return true;
    case 0x15:
        fp_sp(dc, gen_helper_subsp, XR(src1), RR(src2), 3);
        return true;
    case 0x75:
        fp_sp(dc, gen_helper_subsp, XR(src2), RR(src1), 3);
        return true;
    case 0x18:
    case 0x72:
        fp_dp(dc, gen_helper_adddp, dp_own(dc, src1, 1), dp_x(dc, src2, 1),
              5, 6);
        return true;
    case 0x19:
    case 0x73:
        fp_dp(dc, gen_helper_subdp, dp_own(dc, src1, 1), dp_x(dc, src2, 1),
              5, 6);
        return true;
    case 0x1d:
        fp_dp(dc, gen_helper_subdp, dp_x(dc, src1, 1), dp_own(dc, src2, 1),
              5, 6);
        return true;
    case 0x77:
        fp_dp(dc, gen_helper_subdp, dp_x(dc, src2, 1), dp_own(dc, src1, 1),
              5, 6);
        return true;
    case 0x39: fp_s2d(dc, gen_helper_intdp, XR(src2), 3, 4); return true;
    case 0x3b: fp_s2d(dc, gen_helper_intdpu, XR(src2), 3, 4); return true;
    case 0x49: fp_cvt32(dc, gen_helper_intspu, XR(src2), 3); return true;
    case 0x4a: fp_cvt32(dc, gen_helper_intsp, XR(src2), 3); return true;
    }
    return false;
}

/* .S unit, 1 or 2 sources format */
static bool trans_fp_s(DisasContext *dc, int op)
{
    int src2 = F_SRC2(dc->insn), src1 = F_SRC1(dc->insn);

    switch (op) {
    case 0x02: fp_s2d(dc, gen_helper_spdp, XR(src2), 0, 1); return true;
    case 0x28: fp_dp_cmp(dc, 0, dp_own(dc, src1, 1), dp_x(dc, src2, 1));
        return true;
    case 0x29: fp_dp_cmp(dc, 1, dp_own(dc, src1, 1), dp_x(dc, src2, 1));
        return true;
    case 0x2a: fp_dp_cmp(dc, 2, dp_own(dc, src1, 1), dp_x(dc, src2, 1));
        return true;
    case 0x2c: fp_d2d(dc, gen_helper_absdp, dp_unary(dc), 0, 1); return true;
    case 0x2d: fp_d2d(dc, gen_helper_rcpdp, dp_unary(dc), 0, 1); return true;
    case 0x2e: fp_d2d(dc, gen_helper_rsqrdp, dp_unary(dc), 0, 1); return true;
    case 0x38: fp_sp_cmp(dc, 0, RR(src1), XR(src2)); return true;
    case 0x39: fp_sp_cmp(dc, 1, RR(src1), XR(src2)); return true;
    case 0x3a: fp_sp_cmp(dc, 2, RR(src1), XR(src2)); return true;
    case 0x3d: fp_cvt32(dc, gen_helper_rcpsp, XR(src2), 0); return true;
    case 0x3e: fp_cvt32(dc, gen_helper_rsqrsp, XR(src2), 0); return true;
    }
    return false;
}

/* .M unit, multiply format */
static bool trans_fp_m(DisasContext *dc, int op)
{
    int dst = F_DST(dc->insn), src2 = F_SRC2(dc->insn), src1 = F_SRC1(dc->insn);
    TCGLabel *l;

    switch (op) {
    case 0x04:
    case 0x06: {    /* MPYI */
        TCGv_i32 r = tmp();
        tcg_gen_mul_i32(r, op == 0x06 ? scst5(src1) : RR(src1), XR(src2));
        wr(dc, dst, r, 8);
        return true;
    }
    case 0x08:
    case 0x0c: {    /* MPYID */
        TCGv_i32 lo = tmp(), hi = tmp();
        tcg_gen_muls2_i32(lo, hi, op == 0x0c ? scst5(src1) : RR(src1),
                          XR(src2));
        wr_pair(dc, dc->s, dst, lo, hi, 8, 9);
        return true;
    }
    case 0x0e:      /* MPYDP */
        fp_dp(dc, gen_helper_mpydp, dp_own(dc, src1, 2), dp_x(dc, src2, 1),
              8, 9);
        return true;
    case 0x1c: {    /* MPYSP */
        TCGv_i32 r = tmp();
        l = guard(dc);
        gen_helper_mpysp(r, tcg_env, RR(src1), XR(src2));
        unguard(l);
        wr(dc, dst, r, 3);
        return true;
    }
    }
    return false;
}

/* .M unit, compound format */
static bool trans_fp_m_compound(DisasContext *dc, int op)
{
    int src2 = F_SRC2(dc->insn), src1 = F_SRC1(dc->insn);
    TCGv_i64 r = tmp64();
    TCGLabel *l = guard(dc);

    if (op == 0x16) {   /* MPYSPDP */
        gen_helper_mpyspdp(r, tcg_env, RR(src1), dp_x(dc, src2, 1));
        unguard(l);
        wr_pair64(dc, dc->s, F_DST(dc->insn), r, 5, 6);
    } else {            /* MPYSP2DP */
        gen_helper_mpysp2dp(r, tcg_env, RR(src1), XR(src2));
        unguard(l);
        wr_pair64(dc, dc->s, F_DST(dc->insn), r, 3, 4);
    }
    return true;
}
