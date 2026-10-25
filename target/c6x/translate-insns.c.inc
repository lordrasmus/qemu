/*
 * TI C64x+ / C674x instructions, included by translate.c
 *
 * Field names follow the opcode maps of SPRU732J: dst 27:23, src2 22:18,
 * src1 17:13, x 12 (cross path), s 1 (side).  Which field an operand of
 * a given opfield comes from, and which of them the cross path applies to,
 * follows the "Opcode map field used ... For operand type" tables.
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

static bool trans_fp_l(DisasContext *dc, int op);
static bool trans_fp_s(DisasContext *dc, int op);
static bool trans_fp_m(DisasContext *dc, int op);
static bool trans_fp_m_compound(DisasContext *dc, int op);
static bool trans_m_noncond(DisasContext *dc, uint32_t insn);
static bool trans_atomic(DisasContext *dc, uint32_t insn, int op);
static bool trans_sploop(DisasContext *dc, uint32_t insn, int op);

/* Guard code with side effects by the predicate */
static TCGLabel *guard(DisasContext *dc)
{
    TCGLabel *l;

    if (!dc->cond) {
        return NULL;
    }
    l = gen_new_label();
    tcg_gen_brcondi_i32(TCG_COND_EQ, dc->cond, 0, l);
    return l;
}

static void unguard(TCGLabel *l)
{
    if (l) {
        gen_set_label(l);
    }
}

/*
 * base + offset for address arithmetic.  A4-A7 and B4-B7 may be in
 * circular mode, which the TB flags tell.
 */
static void gen_adda(DisasContext *dc, TCGv_i32 d, TCGv_i32 base,
                     TCGv_i32 off, int reg)
{
    int r = reg & 31;

    if (dc->circ && r >= 4 && r <= 7) {
        gen_helper_adda(d, tcg_env, base, off, cst(reg));
    } else {
        tcg_gen_add_i32(d, base, off);
    }
}

/* Cross path operand from field f */
#define XR(f)   gx(dc, f)
/* Own side operand from field f */
#define RR(f)   gr(dc, f)

static TCGv_i32 scst5(uint32_t f)
{
    return cst(sextract32(f, 0, 5));
}

/* Simple one-cycle results */
typedef void GenOp2(TCGv_i32, TCGv_i32, TCGv_i32);

static void op2(DisasContext *dc, GenOp2 *fn, TCGv_i32 a, TCGv_i32 b,
                int delay)
{
    TCGv_i32 t = tmp();

    fn(t, a, b);
    wr(dc, F_DST(dc->insn), t, delay);
}

static void gen_andn(TCGv_i32 d, TCGv_i32 a, TCGv_i32 b)
{
    tcg_gen_andc_i32(d, b, a);
}

static void gen_cmpeq(TCGv_i32 d, TCGv_i32 a, TCGv_i32 b)
{
    tcg_gen_setcond_i32(TCG_COND_EQ, d, a, b);
}

static void gen_cmpgt(TCGv_i32 d, TCGv_i32 a, TCGv_i32 b)
{
    tcg_gen_setcond_i32(TCG_COND_GT, d, a, b);
}

static void gen_cmplt(TCGv_i32 d, TCGv_i32 a, TCGv_i32 b)
{
    tcg_gen_setcond_i32(TCG_COND_LT, d, a, b);
}

static void gen_cmpgtu(TCGv_i32 d, TCGv_i32 a, TCGv_i32 b)
{
    tcg_gen_setcond_i32(TCG_COND_GTU, d, a, b);
}

static void gen_cmpltu(TCGv_i32 d, TCGv_i32 a, TCGv_i32 b)
{
    tcg_gen_setcond_i32(TCG_COND_LTU, d, a, b);
}

/* Comparisons with a 40-bit operand */
static void cmp_long(DisasContext *dc, TCGCond c, TCGv_i64 a, TCGv_i64 b)
{
    TCGv_i64 t = tmp64();
    TCGv_i32 r = tmp();

    tcg_gen_setcond_i64(c, t, a, b);
    tcg_gen_extrl_i64_i32(r, t);
    wr(dc, F_DST(dc->insn), r, 0);
}

static TCGv_i64 ext64(TCGv_i32 v, bool sign)
{
    TCGv_i64 t = tmp64();

    if (sign) {
        tcg_gen_ext_i32_i64(t, v);
    } else {
        tcg_gen_extu_i32_i64(t, v);
    }
    return t;
}

static TCGv_i64 cst64(int64_t v)
{
    return tcg_constant_i64(v);
}

/* Write a 40-bit add/sub result */
static void long_result(DisasContext *dc, TCGv_i64 v)
{
    wr_long(dc, F_DST(dc->insn), v, 0);
}

/* Bit field operations: csta/cstb as in CLR, SET, EXT, EXTU */
static void gen_field(DisasContext *dc, int op, TCGv_i32 src2,
                      TCGv_i32 csta, TCGv_i32 cstb)
{
    TCGv_i32 d = tmp();

    switch (op) {
    case 0:
        gen_helper_extu(d, src2, csta, cstb);
        break;
    case 1:
        gen_helper_ext(d, src2, csta, cstb);
        break;
    case 2:
        gen_helper_set(d, src2, csta, cstb);
        break;
    default:
        gen_helper_clr(d, src2, csta, cstb);
        break;
    }
    wr(dc, F_DST(dc->insn), d, 0);
}

static void gen_field_reg(DisasContext *dc, int op, TCGv_i32 src2,
                          TCGv_i32 src1)
{
    TCGv_i32 a = tmp(), b = tmp();

    tcg_gen_extract_i32(a, src1, 5, 5);
    tcg_gen_extract_i32(b, src1, 0, 5);
    gen_field(dc, op, src2, a, b);
}

/* Shifts.  Register amounts use 6 bits, values over 39 shift by 40. */
static TCGv_i32 shift_amount(TCGv_i32 r)
{
    TCGv_i32 a = tmp();

    tcg_gen_andi_i32(a, r, 63);
    tcg_gen_umin_i32(a, a, cst(40));
    return a;
}

static void gen_shl40(DisasContext *dc, TCGv_i64 v, TCGv_i32 amt, bool lng)
{
    TCGv_i64 a = tmp64(), r = tmp64();

    tcg_gen_extu_i32_i64(a, amt);
    tcg_gen_extract_i64(r, v, 0, 40);
    tcg_gen_shl_i64(r, r, a);
    if (lng) {
        wr_long(dc, F_DST(dc->insn), r, 0);
    } else {
        TCGv_i32 t = tmp();

        tcg_gen_extrl_i64_i32(t, r);
        wr(dc, F_DST(dc->insn), t, 0);
    }
}

static void gen_shr40(DisasContext *dc, TCGv_i64 v, TCGv_i32 amt, bool sign,
                      bool lng)
{
    TCGv_i64 a = tmp64(), r = tmp64();

    tcg_gen_extu_i32_i64(a, amt);
    if (sign) {
        tcg_gen_sextract_i64(r, v, 0, 40);
        tcg_gen_sar_i64(r, r, a);
    } else {
        tcg_gen_extract_i64(r, v, 0, 40);
        tcg_gen_shr_i64(r, r, a);
    }
    if (lng) {
        wr_long(dc, F_DST(dc->insn), r, 0);
    } else {
        TCGv_i32 t = tmp();

        tcg_gen_extrl_i64_i32(t, r);
        wr(dc, F_DST(dc->insn), t, 0);
    }
}

/* Helpers that need env for the SAT bit run only if the predicate holds */
typedef void GenSat(TCGv_i32, TCGv_env, TCGv_i32, TCGv_i32);

static void op2_sat(DisasContext *dc, GenSat *fn, TCGv_i32 a, TCGv_i32 b,
                    int delay)
{
    TCGv_i32 t = tmp();
    TCGLabel *l = guard(dc);

    fn(t, tcg_env, a, b);
    unguard(l);
    wr(dc, F_DST(dc->insn), t, delay);
}

/* .L unit, and the .S unit floating point ops sharing its format */

static bool trans_l(DisasContext *dc, uint32_t insn)
{
    int op = extract32(insn, 5, 7);
    int dst = F_DST(insn), src2 = F_SRC2(insn), src1 = F_SRC1(insn);
    bool noncond = F_CREG(insn) == 0 && F_Z(insn) == 1;
    TCGv_i32 t;

    if (noncond) {
        TCGv_i32 a = RR(src1), b = XR(src2);
        TCGv_i32 hi = tmp(), lo = tmp();

        dc->cond = NULL;
        switch (op) {
        case 0x0c:  /* ADDSUB */
            tcg_gen_add_i32(hi, a, b);
            tcg_gen_sub_i32(lo, a, b);
            break;
        case 0x0d:  /* ADDSUB2 */
            gen_helper_add2(hi, a, b);
            gen_helper_sub2(lo, a, b);
            break;
        case 0x0e:  /* SADDSUB */
            gen_helper_sadd(hi, tcg_env, a, b);
            gen_helper_ssub(lo, tcg_env, a, b);
            break;
        case 0x0f:  /* SADDSUB2 */
            gen_helper_sadd2(hi, tcg_env, a, b);
            gen_helper_ssub2(lo, tcg_env, a, b);
            break;
        case 0x33:  /* DPACKX2: two PACKLH2 */
            gen_helper_packlh2(lo, a, b);
            gen_helper_packlh2(hi, b, a);
            break;
        case 0x34:  /* DPACK2: PACK2 and PACKH2 */
            gen_helper_pack2(lo, a, b);
            gen_helper_packh2(hi, a, b);
            break;
        case 0x36:  /* SHFL3 */
            gen_helper_shfl3_lo(lo, a, b);
            gen_helper_shfl3_hi(hi, a, b);
            break;
        default:
            return false;
        }
        wr_pair(dc, dc->s, dst, lo, hi, 0, 0);
        return true;
    }

    if (op >= 0x70 && op <= 0x77) {
        return trans_fp_l(dc, op);
    }

    switch (op) {
    case 0x1a: /* unary */
        t = tmp();
        switch (src1) {
        case 0x00:  /* ABS */
            tcg_gen_abs_i32(t, XR(src2));
            /* ABS of 8000 0000h saturates */
            tcg_gen_movcond_i32(TCG_COND_LT, t, t, cst(0), cst(0x7fffffff), t);
            break;
        case 0x01:  /* SWAP4 */
            gen_helper_swap4(t, XR(src2));
            break;
        case 0x02:  /* UNPKLU4 */
            gen_helper_unpklu4(t, XR(src2));
            break;
        case 0x03:  /* UNPKHU4 */
            gen_helper_unpkhu4(t, XR(src2));
            break;
        case 0x04:  /* ABS2 */
            gen_helper_abs2(t, XR(src2));
            break;
        case 0x05:  /* MVK, 5-bit form */
            if (F_X(insn)) {
                return false;
            }
            tcg_gen_movi_i32(t, sextract32(src2, 0, 5));
            break;
        default:
            return false;
        }
        wr(dc, dst, t, 0);
        return true;

    case 0x00: op2(dc, gen_helper_pack2, RR(src1), XR(src2), 0); return true;
    case 0x02: op2(dc, tcg_gen_add_i32, scst5(src1), XR(src2), 0); return true;
    case 0x03: op2(dc, tcg_gen_add_i32, RR(src1), XR(src2), 0); return true;
    case 0x04: op2(dc, gen_helper_sub2, RR(src1), XR(src2), 0); return true;
    case 0x05: op2(dc, gen_helper_add2, RR(src1), XR(src2), 0); return true;
    case 0x06: op2(dc, tcg_gen_sub_i32, scst5(src1), XR(src2), 0); return true;
    case 0x07: op2(dc, tcg_gen_sub_i32, RR(src1), XR(src2), 0); return true;
    case 0x0e: op2_sat(dc, gen_helper_ssub, scst5(src1), XR(src2), 0);
        return true;
    case 0x0f: op2_sat(dc, gen_helper_ssub, RR(src1), XR(src2), 0);
        return true;
    case 0x12: op2_sat(dc, gen_helper_sadd, scst5(src1), XR(src2), 0);
        return true;
    case 0x13: op2_sat(dc, gen_helper_sadd, RR(src1), XR(src2), 0);
        return true;
    case 0x17: op2(dc, tcg_gen_sub_i32, XR(src1), RR(src2), 0); return true;
    case 0x1b: op2(dc, gen_helper_packlh2, RR(src1), XR(src2), 0); return true;
    case 0x1c: op2(dc, gen_helper_packhl2, RR(src1), XR(src2), 0); return true;
    case 0x1e: op2(dc, gen_helper_packh2, RR(src1), XR(src2), 0); return true;
    case 0x1f: op2_sat(dc, gen_helper_ssub, XR(src1), RR(src2), 0);
        return true;

    case 0x20:  /* ADD scst5, slong */
        if (F_X(insn)) {
            return false;
        }
        {
            TCGv_i64 r = tmp64();
            tcg_gen_add_i64(r, gl(dc, src2, true), cst64(sextract32(src1, 0, 5)));
            long_result(dc, r);
        }
        return true;
    case 0x21: {    /* ADD xsint, slong */
        TCGv_i64 r = tmp64();
        tcg_gen_add_i64(r, ext64(XR(src1), true), gl(dc, src2, true));
        long_result(dc, r);
        return true;
    }
    case 0x23: {    /* ADD sint, xsint -> slong */
        TCGv_i64 r = tmp64();
        tcg_gen_add_i64(r, ext64(RR(src1), true), ext64(XR(src2), true));
        long_result(dc, r);
        return true;
    }
    case 0x24: {    /* SUB scst5, slong */
        TCGv_i64 r = tmp64();
        tcg_gen_sub_i64(r, cst64(sextract32(src1, 0, 5)), gl(dc, src2, true));
        long_result(dc, r);
        return true;
    }
    case 0x27: {    /* SUB sint, xsint -> slong */
        TCGv_i64 r = tmp64();
        tcg_gen_sub_i64(r, ext64(RR(src1), true), ext64(XR(src2), true));
        long_result(dc, r);
        return true;
    }
    case 0x29: {    /* ADDU xuint, ulong */
        TCGv_i64 r = tmp64();
        tcg_gen_add_i64(r, ext64(XR(src1), false), gl(dc, src2, false));
        long_result(dc, r);
        return true;
    }
    case 0x2b: {    /* ADDU uint, xuint -> ulong */
        TCGv_i64 r = tmp64();
        tcg_gen_add_i64(r, ext64(RR(src1), false), ext64(XR(src2), false));
        long_result(dc, r);
        return true;
    }
    case 0x2c: {    /* SSUB scst5, slong */
        TCGv_i64 r = tmp64();
        TCGLabel *l = guard(dc);
        gen_helper_ssub40(r, tcg_env, cst64(sextract32(src1, 0, 5)),
                          gl(dc, src2, true));
        unguard(l);
        long_result(dc, r);
        return true;
    }
    case 0x2f: {    /* SUBU uint, xuint -> ulong */
        TCGv_i64 r = tmp64();
        tcg_gen_sub_i64(r, ext64(RR(src1), false), ext64(XR(src2), false));
        long_result(dc, r);
        return true;
    }
    case 0x30: {    /* SADD scst5, slong */
        TCGv_i64 r = tmp64();
        TCGLabel *l = guard(dc);
        gen_helper_sadd40(r, tcg_env, cst64(sextract32(src1, 0, 5)),
                          gl(dc, src2, true));
        unguard(l);
        long_result(dc, r);
        return true;
    }
    case 0x31: {    /* SADD xsint, slong */
        TCGv_i64 r = tmp64();
        TCGLabel *l = guard(dc);
        gen_helper_sadd40(r, tcg_env, ext64(XR(src1), true),
                          gl(dc, src2, true));
        unguard(l);
        long_result(dc, r);
        return true;
    }
    case 0x37: {    /* SUB xsint, sint -> slong */
        TCGv_i64 r = tmp64();
        tcg_gen_sub_i64(r, ext64(XR(src1), true), ext64(RR(src2), true));
        long_result(dc, r);
        return true;
    }
    case 0x38: {    /* ABS slong */
        TCGv_i64 r = tmp64();
        TCGLabel *l = guard(dc);
        gen_helper_abs40(r, tcg_env, gl(dc, src2, true));
        unguard(l);
        long_result(dc, r);
        return true;
    }
    case 0x3f: {    /* SUBU xuint, uint -> ulong */
        TCGv_i64 r = tmp64();
        tcg_gen_sub_i64(r, ext64(XR(src1), false), ext64(RR(src2), false));
        long_result(dc, r);
        return true;
    }
    case 0x40: {    /* SAT slong */
        TCGv_i32 r = tmp();
        TCGLabel *l = guard(dc);
        gen_helper_sat40(r, tcg_env, gl(dc, src2, true));
        unguard(l);
        wr(dc, dst, r, 0);
        return true;
    }
    case 0x41: op2(dc, gen_helper_min2, RR(src1), XR(src2), 0); return true;
    case 0x42: op2(dc, gen_helper_max2, RR(src1), XR(src2), 0); return true;
    case 0x43: op2(dc, gen_helper_maxu4, RR(src1), XR(src2), 0); return true;
    case 0x48: op2(dc, gen_helper_minu4, RR(src1), XR(src2), 0); return true;

    /* compares: field order per the opfield tables */
    case 0x44:  /* CMPGT scst5, slong */
        cmp_long(dc, TCG_COND_GT, cst64(sextract32(src1, 0, 5)),
                 gl(dc, src2, true));
        return true;
    case 0x45:  /* CMPGT xsint, slong */
        cmp_long(dc, TCG_COND_GT, ext64(XR(src1), true), gl(dc, src2, true));
        return true;
    case 0x46: op2(dc, gen_cmpgt, scst5(src1), XR(src2), 0); return true;
    case 0x47: op2(dc, gen_cmpgt, RR(src1), XR(src2), 0); return true;
    case 0x4b: op2(dc, gen_helper_subc, RR(src1), XR(src2), 0); return true;
    case 0x4c:  /* CMPGTU ucst5, ulong */
        cmp_long(dc, TCG_COND_GTU, cst64(src1), gl(dc, src2, false));
        return true;
    case 0x4d:
        cmp_long(dc, TCG_COND_GTU, ext64(XR(src1), false),
                 gl(dc, src2, false));
        return true;
    case 0x4e: op2(dc, gen_cmpgtu, cst(src1), XR(src2), 0); return true;
    case 0x4f: op2(dc, gen_cmpgtu, RR(src1), XR(src2), 0); return true;
    case 0x50:
        cmp_long(dc, TCG_COND_EQ, cst64(sextract32(src1, 0, 5)),
                 gl(dc, src2, true));
        return true;
    case 0x51:
        cmp_long(dc, TCG_COND_EQ, ext64(XR(src1), true), gl(dc, src2, true));
        return true;
    case 0x52: op2(dc, gen_cmpeq, scst5(src1), XR(src2), 0); return true;
    case 0x53: op2(dc, gen_cmpeq, RR(src1), XR(src2), 0); return true;
    case 0x54:  /* CMPLT scst5, slong */
        cmp_long(dc, TCG_COND_LT, cst64(sextract32(src1, 0, 5)),
                 gl(dc, src2, true));
        return true;
    case 0x55:
        cmp_long(dc, TCG_COND_LT, ext64(XR(src1), true), gl(dc, src2, true));
        return true;
    case 0x56: op2(dc, gen_cmplt, scst5(src1), XR(src2), 0); return true;
    case 0x57: op2(dc, gen_cmplt, RR(src1), XR(src2), 0); return true;
    case 0x5a: op2(dc, gen_helper_subabs4, RR(src1), XR(src2), 0); return true;
    case 0x5c:
        cmp_long(dc, TCG_COND_LTU, cst64(src1), gl(dc, src2, false));
        return true;
    case 0x5d:
        cmp_long(dc, TCG_COND_LTU, ext64(XR(src1), false),
                 gl(dc, src2, false));
        return true;
    case 0x5e: op2(dc, gen_cmpltu, cst(src1), XR(src2), 0); return true;
    case 0x5f: op2(dc, gen_cmpltu, RR(src1), XR(src2), 0); return true;

    case 0x60: {    /* NORM slong */
        TCGv_i64 v = gl(dc, src2, true), r = tmp64();
        TCGv_i32 r32 = tmp();
        tcg_gen_shli_i64(r, v, 24);
        tcg_gen_clrsb_i64(r, r);
        tcg_gen_extrl_i64_i32(r32, r);
        tcg_gen_umin_i32(r32, r32, cst(39));
        wr(dc, dst, r32, 0);
        return true;
    }
    case 0x61: op2(dc, gen_helper_shlmb, RR(src1), XR(src2), 0); return true;
    case 0x62: op2(dc, gen_helper_shrmb, RR(src1), XR(src2), 0); return true;
    case 0x63:      /* NORM xsint */
        t = tmp();
        tcg_gen_clrsb_i32(t, XR(src2));
        wr(dc, dst, t, 0);
        return true;
    case 0x64: op2_sat(dc, gen_helper_ssub2, RR(src1), XR(src2), 0);
        return true;
    case 0x65: op2(dc, gen_helper_add4, RR(src1), XR(src2), 0); return true;
    case 0x66: op2(dc, gen_helper_sub4, RR(src1), XR(src2), 0); return true;
    case 0x68: op2(dc, gen_helper_packl4, RR(src1), XR(src2), 0); return true;
    case 0x69: op2(dc, gen_helper_packh4, RR(src1), XR(src2), 0); return true;
    case 0x6a: op2(dc, gen_helper_lmbd, scst5(src1), XR(src2), 0); return true;
    case 0x6b: op2(dc, gen_helper_lmbd, RR(src1), XR(src2), 0); return true;
    case 0x6e: op2(dc, tcg_gen_xor_i32, scst5(src1), XR(src2), 0); return true;
    case 0x6f: op2(dc, tcg_gen_xor_i32, RR(src1), XR(src2), 0); return true;
    case 0x7a: op2(dc, tcg_gen_and_i32, scst5(src1), XR(src2), 0); return true;
    case 0x7b: op2(dc, tcg_gen_and_i32, RR(src1), XR(src2), 0); return true;
    case 0x7c: op2(dc, gen_andn, XR(src2), RR(src1), 0); return true;
    case 0x7e: op2(dc, tcg_gen_or_i32, scst5(src1), XR(src2), 0); return true;
    case 0x7f: op2(dc, tcg_gen_or_i32, RR(src1), XR(src2), 0); return true;
    }
    return trans_fp_l(dc, op);
}

/* .S unit */

static bool trans_mvc(DisasContext *dc, uint32_t insn, bool to_cr);

static bool trans_s(DisasContext *dc, uint32_t insn)
{
    int op = extract32(insn, 6, 6);
    int dst = F_DST(insn), src2 = F_SRC2(insn), src1 = F_SRC1(insn);
    TCGv_i32 t;

    switch (op) {
    case 0x00: {    /* BPOS / BDEC */
        int32_t off = sextract32(insn, 13, 10) << 2;
        TCGv_i32 r = gr(dc, dst);
        TCGv_i32 c = tmp();
        TCGv_i32 saved = dc->cond;

        tcg_gen_setcondi_i32(TCG_COND_GE, c, r, 0);
        if (saved) {
            tcg_gen_and_i32(c, c, saved);
        }
        dc->cond = c;
        gen_branch(dc, NULL, dc->pce1 + off);
        if (extract32(insn, 12, 1)) {   /* BDEC */
            t = tmp();
            tcg_gen_subi_i32(t, r, 1);
            wr(dc, dst, t, 0);
        }
        dc->cond = saved;
        return true;
    }
    case 0x03:      /* B IRP, B NRP */
        if (dc->user) {
            gen_exception(dc, EXCP_INTERNAL, IERR_OPX | IERR_PRX);
            return true;
        }
        if (src2 == CR_IRP) {
            TCGLabel *l = guard(dc);
            t = tmp();
            gen_helper_b_irp(t, tcg_env);
            unguard(l);
            gen_branch(dc, t, 0);
        } else if (src2 == CR_NRP) {
            TCGLabel *l = guard(dc);
            t = tmp();
            gen_helper_b_nrp(t, tcg_env);
            unguard(l);
            gen_branch(dc, t, 0);
        } else {
            return false;
        }
        dc->exit_tb = true;
        return true;
    case 0x04: {    /* BNOP displacement */
        int32_t off = sextract32(insn, 16, 12) << 2;

        if (extract32(insn, 12, 1)) {
            return false;
        }
        gen_branch(dc, NULL, dc->pce1 + off);
        dc->nops = MAX(dc->nops, extract32(insn, 13, 3));
        return true;
    }
    case 0x05: {    /* ADDKPC */
        int32_t off = sextract32(insn, 16, 7) << 2;

        if (extract32(insn, 12, 1)) {
            return false;
        }
        wr(dc, dst, cst(dc->pce1 + off), 0);
        dc->nops = MAX(dc->nops, extract32(insn, 13, 3));
        return true;
    }
    case 0x0d:      /* B reg, BNOP reg */
        t = tmp();
        tcg_gen_andi_i32(t, XR(src2), ~3);
        gen_branch(dc, t, 0);
        if (extract32(insn, 23, 5) == 1) {
            dc->nops = MAX(dc->nops, extract32(insn, 13, 3));
        } else if (dst != 0) {
            return false;
        }
        return true;
    case 0x0e:
        return trans_mvc(dc, insn, true);
    case 0x0f:
        return trans_mvc(dc, insn, false);

    case 0x3c:      /* unary */
        t = tmp();
        switch (src1) {
        case 0x00: {    /* ABSSP */
            TCGLabel *l = guard(dc);
            gen_helper_abssp(t, tcg_env, XR(src2));
            unguard(l);
            break;
        }
        case 0x02:
            gen_helper_unpklu4(t, XR(src2));
            break;
        case 0x03:
            gen_helper_unpkhu4(t, XR(src2));
            break;
        default:
            return false;
        }
        wr(dc, dst, t, 0);
        return true;

    case 0x01: op2(dc, gen_helper_add2, RR(src1), XR(src2), 0); return true;
    case 0x06: op2(dc, tcg_gen_add_i32, scst5(src1), XR(src2), 0); return true;
    case 0x07: op2(dc, tcg_gen_add_i32, RR(src1), XR(src2), 0); return true;
    case 0x08: op2(dc, gen_helper_packhl2, RR(src1), XR(src2), 0); return true;
    case 0x09: op2(dc, gen_helper_packh2, RR(src1), XR(src2), 0); return true;
    case 0x0a: op2(dc, tcg_gen_xor_i32, scst5(src1), XR(src2), 0); return true;
    case 0x0b: op2(dc, tcg_gen_xor_i32, RR(src1), XR(src2), 0); return true;
    case 0x10: op2(dc, gen_helper_packlh2, RR(src1), XR(src2), 0); return true;
    case 0x11: op2(dc, gen_helper_sub2, RR(src1), XR(src2), 0); return true;
    case 0x12: {    /* SHL xsint, ucst5 -> slong */
        gen_shl40(dc, ext64(XR(src2), true), cst(src1), true);
        return true;
    }
    case 0x13:
        gen_shl40(dc, ext64(XR(src2), true), shift_amount(RR(src1)), true);
        return true;
    case 0x14: op2(dc, gen_helper_cmpgt2, RR(src1), XR(src2), 0); return true;
    case 0x15: op2(dc, gen_helper_cmpgtu4, RR(src1), XR(src2), 0); return true;
    case 0x16: op2(dc, tcg_gen_sub_i32, scst5(src1), XR(src2), 0); return true;
    case 0x17: op2(dc, tcg_gen_sub_i32, RR(src1), XR(src2), 0); return true;
    case 0x18: op2(dc, gen_helper_shr2, cst(src1), XR(src2), 0); return true;
    case 0x19: op2(dc, gen_helper_shru2, cst(src1), XR(src2), 0); return true;
    case 0x1a: op2(dc, tcg_gen_or_i32, scst5(src1), XR(src2), 0); return true;
    case 0x1b: op2(dc, tcg_gen_or_i32, RR(src1), XR(src2), 0); return true;
    case 0x1c: op2(dc, gen_helper_cmpeq4, RR(src1), XR(src2), 0); return true;
    case 0x1d: op2(dc, gen_helper_cmpeq2, RR(src1), XR(src2), 0); return true;
    case 0x1e: op2(dc, tcg_gen_and_i32, scst5(src1), XR(src2), 0); return true;
    case 0x1f: op2(dc, tcg_gen_and_i32, RR(src1), XR(src2), 0); return true;
    case 0x20: op2_sat(dc, gen_helper_sadd, RR(src1), XR(src2), 0);
        return true;
    case 0x22: op2_sat(dc, gen_helper_sshl, XR(src2), cst(src1), 0);
        return true;
    case 0x23: {
        TCGv_i32 a = tmp();
        tcg_gen_andi_i32(a, RR(src1), 63);
        op2_sat(dc, gen_helper_sshl, XR(src2), a, 0);
        return true;
    }
    case 0x24:      /* SHRU ulong, ucst5 */
        if (F_X(insn)) {
            return false;
        }
        gen_shr40(dc, gl(dc, src2, false), cst(src1), false, true);
        return true;
    case 0x25:
        if (F_X(insn)) {
            return false;
        }
        gen_shr40(dc, gl(dc, src2, false), shift_amount(RR(src1)), false, true);
        return true;
    case 0x26: {
        t = tmp();
        tcg_gen_shri_i32(t, XR(src2), src1);
        wr(dc, dst, t, 0);
        return true;
    }
    case 0x27:
        gen_shr40(dc, ext64(XR(src2), false), shift_amount(RR(src1)), false,
                  false);
        return true;
    case 0x2b: gen_field_reg(dc, 0, XR(src2), RR(src1)); return true;
    case 0x2f: gen_field_reg(dc, 1, XR(src2), RR(src1)); return true;
    case 0x30:      /* SHL slong, ucst5 */
        if (F_X(insn)) {
            return false;
        }
        gen_shl40(dc, gl(dc, src2, true), cst(src1), true);
        return true;
    case 0x31:
        if (F_X(insn)) {
            return false;
        }
        gen_shl40(dc, gl(dc, src2, true), shift_amount(RR(src1)), true);
        return true;
    case 0x32:
        t = tmp();
        tcg_gen_shli_i32(t, XR(src2), src1);
        wr(dc, dst, t, 0);
        return true;
    case 0x33:
        gen_shl40(dc, ext64(XR(src2), true), shift_amount(RR(src1)), false);
        return true;
    case 0x34:
        if (F_X(insn)) {
            return false;
        }
        gen_shr40(dc, gl(dc, src2, true), cst(src1), true, true);
        return true;
    case 0x35:
        if (F_X(insn)) {
            return false;
        }
        gen_shr40(dc, gl(dc, src2, true), shift_amount(RR(src1)), true, true);
        return true;
    case 0x36:
        t = tmp();
        tcg_gen_sari_i32(t, XR(src2), src1);
        wr(dc, dst, t, 0);
        return true;
    case 0x37:
        gen_shr40(dc, ext64(XR(src2), true), shift_amount(RR(src1)), true,
                  false);
        return true;
    case 0x3b: gen_field_reg(dc, 2, XR(src2), RR(src1)); return true;
    case 0x3f: gen_field_reg(dc, 3, XR(src2), RR(src1)); return true;
    }
    return trans_fp_s(dc, op);
}

/* .S unit, extended format: op in bits 9:6 */
static bool trans_s_ext(DisasContext *dc, uint32_t insn)
{
    int op = extract32(insn, 6, 4);
    int dst = F_DST(insn), src2 = F_SRC2(insn), src1 = F_SRC1(insn);
    bool noncond = F_CREG(insn) == 0 && F_Z(insn) == 1;

    if (noncond) {
        dc->cond = NULL;
        if (op == 0x0b) {   /* RPACK2 */
            op2_sat(dc, gen_helper_rpack2, RR(src1), XR(src2), 0);
            return true;
        }
        return false;
    }

    switch (op) {
    case 0x0: op2_sat(dc, gen_helper_sadd2, RR(src1), XR(src2), 0); return true;
    case 0x1:   /* SADDUS2 */
        op2_sat(dc, gen_helper_saddus2, RR(src1), XR(src2), 0);
        return true;
    case 0x2: op2_sat(dc, gen_helper_spack2, RR(src1), XR(src2), 0);
        return true;
    case 0x3: op2_sat(dc, gen_helper_saddu4, RR(src1), XR(src2), 0);
        return true;
    case 0x4: op2_sat(dc, gen_helper_spacku4, RR(src1), XR(src2), 0);
        return true;
    case 0x5: op2(dc, tcg_gen_sub_i32, XR(src2), RR(src1), 0); return true;
    case 0x6: op2(dc, gen_andn, XR(src2), RR(src1), 0); return true;
    case 0x7: op2(dc, gen_helper_shr2, RR(src1), XR(src2), 0); return true;
    case 0x8: op2(dc, gen_helper_shru2, RR(src1), XR(src2), 0); return true;
    case 0x9: op2(dc, gen_helper_shlmb, RR(src1), XR(src2), 0); return true;
    case 0xa: op2(dc, gen_helper_shrmb, RR(src1), XR(src2), 0); return true;
    case 0xb:       /* DMV */
        wr_pair(dc, dc->s, dst, XR(src2), RR(src1), 0, 0);
        return true;
    case 0xc: op2(dc, gen_helper_min2, RR(src1), XR(src2), 0); return true;
    case 0xd: op2(dc, gen_helper_max2, RR(src1), XR(src2), 0); return true;
    case 0xf: op2(dc, gen_helper_pack2, RR(src1), XR(src2), 0); return true;
    }
    return false;
}

/* MVK, MVKH, MVKLH: .S unit, 16-bit constant */
static bool trans_mvk(DisasContext *dc, uint32_t insn)
{
    int dst = F_DST(insn);
    uint32_t c = extract32(insn, 7, 16);
    TCGv_i32 t = tmp();

    if (extract32(insn, 6, 1)) {
        /* MVKH/MVKLH: replace the upper half */
        tcg_gen_deposit_i32(t, gr(dc, dst), cst(c), 16, 16);
    } else {
        tcg_gen_movi_i32(t, (int16_t)c);
    }
    wr(dc, dst, t, 0);
    return true;
}

/* EXTU, EXT, SET, CLR with constant fields */
static bool trans_field(DisasContext *dc, uint32_t insn)
{
    gen_field(dc, extract32(insn, 6, 2), gr(dc, F_SRC2(insn)),
              cst(extract32(insn, 13, 5)), cst(extract32(insn, 8, 5)));
    return true;
}

/* ADDK */
static bool trans_addk(DisasContext *dc, uint32_t insn)
{
    int dst = F_DST(insn);
    TCGv_i32 t = tmp();

    tcg_gen_addi_i32(t, gr(dc, dst), sextract32(insn, 7, 16));
    wr(dc, dst, t, 0);
    return true;
}

/* B displacement, CALLP */
static bool trans_bimm(DisasContext *dc, uint32_t insn)
{
    int32_t off = sextract32(insn, 7, 21) << 2;

    if (F_CREG(insn) == 0 && F_Z(insn) == 1) {
        /* CALLP: return address into A3/B3, NOP 5 */
        dc->cond = NULL;
        wr(dc, 3, cst(dc->next_ep), 0);
        gen_branch(dc, NULL, dc->pce1 + off);
        dc->nops = MAX(dc->nops, 5);
        return true;
    }
    gen_branch(dc, NULL, dc->pce1 + off);
    return true;
}

/* .M unit, 16x16 multiply format: op in bits 11:7 */
static bool trans_m_mpy(DisasContext *dc, uint32_t insn)
{
    int op = extract32(insn, 7, 5);
    int dst = F_DST(insn), src2 = F_SRC2(insn), src1 = F_SRC1(insn);
    TCGv_i32 a = RR(src1), b = XR(src2);
    TCGv_i32 t = tmp();
    /* (src1 half, signed, src2 half, signed); half 1 = high */
    static const int8_t mpy[32][4] = {
        [0x19] = { 0, 1, 0, 1 },    /* MPY */
        [0x18] = { 0, 1, 0, 1 },    /* MPY scst5 */
        [0x01] = { 1, 1, 1, 1 },    /* MPYH */
        [0x09] = { 1, 1, 0, 1 },    /* MPYHL */
        [0x0f] = { 1, 0, 0, 0 },    /* MPYHLU */
        [0x0b] = { 1, 1, 0, 0 },    /* MPYHSLU */
        [0x03] = { 1, 1, 1, 0 },    /* MPYHSU */
        [0x07] = { 1, 0, 1, 0 },    /* MPYHU */
        [0x0d] = { 1, 0, 0, 1 },    /* MPYHULS */
        [0x05] = { 1, 0, 1, 1 },    /* MPYHUS */
        [0x11] = { 0, 1, 1, 1 },    /* MPYLH */
        [0x17] = { 0, 0, 1, 0 },    /* MPYLHU */
        [0x13] = { 0, 1, 1, 0 },    /* MPYLSHU */
        [0x15] = { 0, 0, 1, 1 },    /* MPYLUHS */
        [0x1b] = { 0, 1, 0, 0 },    /* MPYSU */
        [0x1e] = { 0, 1, 0, 0 },    /* MPYSU scst5 */
        [0x1f] = { 0, 0, 0, 0 },    /* MPYU */
        [0x1d] = { 0, 0, 0, 1 },    /* MPYUS */
    };

    switch (op) {
    case 0x18:
    case 0x1e:
        a = scst5(src1);
        /* fall through */
    case 0x19: case 0x01: case 0x09: case 0x0f: case 0x0b: case 0x03:
    case 0x07: case 0x0d: case 0x05: case 0x11: case 0x17: case 0x13:
    case 0x15: case 0x1b: case 0x1f: case 0x1d: {
        TCGv_i32 x = tmp(), y = tmp();
        const int8_t *m = mpy[op];

        if (m[1]) {
            tcg_gen_sextract_i32(x, a, m[0] * 16, 16);
        } else {
            tcg_gen_extract_i32(x, a, m[0] * 16, 16);
        }
        if (m[3]) {
            tcg_gen_sextract_i32(y, b, m[2] * 16, 16);
        } else {
            tcg_gen_extract_i32(y, b, m[2] * 16, 16);
        }
        tcg_gen_mul_i32(t, x, y);
        wr(dc, dst, t, 1);
        return true;
    }
    case 0x1a: op2_sat(dc, gen_helper_smpy, a, b, 1); return true;
    case 0x02: op2_sat(dc, gen_helper_smpyh, a, b, 1); return true;
    case 0x0a: op2_sat(dc, gen_helper_smpyhl, a, b, 1); return true;
    case 0x12: op2_sat(dc, gen_helper_smpylh, a, b, 1); return true;

    case 0x10:      /* MPY32 -> 32 */
        tcg_gen_mul_i32(t, a, b);
        wr(dc, dst, t, 3);
        return true;
    case 0x14: {    /* MPY32 -> 64 */
        TCGv_i32 lo = tmp(), hi = tmp();
        tcg_gen_muls2_i32(lo, hi, a, b);
        wr_pair(dc, dc->s, dst, lo, hi, 3, 3);
        return true;
    }
    case 0x16: {    /* MPY32SU: signed src1, unsigned src2 */
        TCGv_i32 lo = tmp(), hi = tmp();
        tcg_gen_mulsu2_i32(lo, hi, a, b);
        wr_pair(dc, dc->s, dst, lo, hi, 3, 3);
        return true;
    }
    }
    return trans_fp_m(dc, op);
}

/* .M unit, compound format: op in bits 10:6 */
static bool trans_m_compound(DisasContext *dc, uint32_t insn)
{
    int op = extract32(insn, 6, 5);
    int dst = F_DST(insn), src2 = F_SRC2(insn), src1 = F_SRC1(insn);
    bool noncond = F_CREG(insn) == 0 && F_Z(insn) == 1;
    TCGv_i32 a = RR(src1), b = XR(src2);
    TCGv_i32 t = tmp();
    TCGv_i64 r = tmp64();

    if (noncond) {
        dc->cond = NULL;
        return trans_m_noncond(dc, insn);
    }

    switch (op) {
    case 0x00: gen_helper_mpy2(r, a, b); wr_pair64(dc, dc->s, dst, r, 3, 3);
        return true;
    case 0x01: {
        TCGLabel *l = guard(dc);
        gen_helper_smpy2(r, tcg_env, a, b);
        unguard(l);
        wr_pair64(dc, dc->s, dst, r, 3, 3);
        return true;
    }
    case 0x02: op2(dc, gen_helper_dotpsu4, a, b, 3); return true;
    case 0x04: gen_helper_mpyu4(r, a, b); wr_pair64(dc, dc->s, dst, r, 3, 3);
        return true;
    case 0x05: gen_helper_mpysu4(r, a, b); wr_pair64(dc, dc->s, dst, r, 3, 3);
        return true;
    case 0x06: op2(dc, gen_helper_dotpu4, a, b, 3); return true;
    case 0x07: op2(dc, gen_helper_dotpnrsu2, a, b, 3); return true;
    case 0x09: op2(dc, gen_helper_dotpn2, a, b, 3); return true;
    case 0x0b: gen_helper_dotp2_64(r, a, b);
        wr_pair64(dc, dc->s, dst, r, 3, 3);
        return true;
    case 0x0c: op2(dc, gen_helper_dotp2, a, b, 3); return true;
    case 0x0d: op2(dc, gen_helper_dotprsu2, a, b, 3); return true;
    case 0x0e: op2(dc, gen_helper_mpylir, a, b, 3); return true;
    case 0x10: op2(dc, gen_helper_mpyhir, a, b, 3); return true;
    case 0x11: op2(dc, gen_helper_gmpy4, a, b, 3); return true;
    case 0x12: op2(dc, gen_helper_avgu4, a, b, 1); return true;
    case 0x13: op2(dc, gen_helper_avg2, a, b, 1); return true;
    case 0x14: gen_helper_mpyhi(r, a, b); wr_pair64(dc, dc->s, dst, r, 3, 3);
        return true;
    case 0x15: gen_helper_mpyli(r, a, b); wr_pair64(dc, dc->s, dst, r, 3, 3);
        return true;
    case 0x16:
    case 0x17:
        return trans_fp_m_compound(dc, op);
    case 0x18: {    /* MPY32U */
        TCGv_i32 lo = tmp(), hi = tmp();
        tcg_gen_mulu2_i32(lo, hi, a, b);
        wr_pair(dc, dc->s, dst, lo, hi, 3, 3);
        return true;
    }
    case 0x19: {    /* MPY32US: unsigned src1, signed src2 */
        TCGv_i32 lo = tmp(), hi = tmp();
        tcg_gen_mulsu2_i32(lo, hi, b, a);
        wr_pair(dc, dc->s, dst, lo, hi, 3, 3);
        return true;
    }
    case 0x1a: op2_sat(dc, gen_helper_sshvr, b, a, 1); return true;
    case 0x1c: op2_sat(dc, gen_helper_sshvl, b, a, 1); return true;
    case 0x1d: {    /* ROTL xuint, uint */
        TCGv_i32 n = tmp();
        tcg_gen_andi_i32(n, a, 31);
        tcg_gen_rotl_i32(t, b, n);
        wr(dc, dst, t, 1);
        return true;
    }
    case 0x1e:      /* ROTL xuint, ucst5 */
        tcg_gen_rotli_i32(t, b, src1);
        wr(dc, dst, t, 1);
        return true;
    }
    return false;
}

/* .M unit: the noncond (C64x+) forms of the compound format */
static bool trans_m_noncond(DisasContext *dc, uint32_t insn)
{
    int op = extract32(insn, 6, 5);
    int dst = F_DST(insn), src2 = F_SRC2(insn), src1 = F_SRC1(insn);
    TCGv_i32 a = RR(src1), b = XR(src2);
    TCGv_i64 r = tmp64();
    TCGv_i32 t = tmp();

    switch (op) {
    case 0x0a: {    /* CMPY */
        gen_helper_cmpy(r, tcg_env, a, b);
        wr_pair64(dc, dc->s, dst, r, 3, 3);
        return true;
    }
    case 0x0b: op2_sat(dc, gen_helper_cmpyr, a, b, 3); return true;
    case 0x0c: op2_sat(dc, gen_helper_cmpyr1, a, b, 3); return true;
    case 0x0f: {    /* MPY2IR */
        gen_helper_mpy2ir(r, tcg_env, a, b);
        wr_pair64(dc, dc->s, dst, r, 3, 3);
        return true;
    }
    case 0x14: {    /* DDOTPL2R: src1 is a pair */
        TCGv_i64 p = gpair(dc, dc->s, src1, 0);
        gen_helper_ddotpl2r(t, tcg_env, p, b);
        wr(dc, dst, t, 3);
        return true;
    }
    case 0x15: {    /* DDOTPH2R */
        TCGv_i64 p = gpair(dc, dc->s, src1, 0);
        gen_helper_ddotph2r(t, tcg_env, p, b);
        wr(dc, dst, t, 3);
        return true;
    }
    case 0x16: {    /* DDOTPL2 */
        TCGv_i64 p = gpair(dc, dc->s, src1, 0);
        gen_helper_ddotpl2(r, p, b);
        wr_pair64(dc, dc->s, dst, r, 3, 3);
        return true;
    }
    case 0x17: {    /* DDOTPH2 */
        TCGv_i64 p = gpair(dc, dc->s, src1, 0);
        gen_helper_ddotph2(r, p, b);
        wr_pair64(dc, dc->s, dst, r, 3, 3);
        return true;
    }
    case 0x18:      /* DDOTP4 */
        gen_helper_ddotp4(r, a, b);
        wr_pair64(dc, dc->s, dst, r, 3, 3);
        return true;
    case 0x19: op2_sat(dc, gen_helper_smpy32, a, b, 3); return true;
    case 0x1b: op2(dc, gen_helper_xormpy, a, b, 3); return true;
    case 0x1f: {    /* GMPY, no cross path */
        TCGv_i32 poly = tmp();
        tcg_gen_ld_i32(poly, tcg_env, dc->s ? offsetof(CPUC6xState, gplyb) :
                       offsetof(CPUC6xState, gplya));
        gen_helper_gmpy(t, a, b, poly);
        wr(dc, dst, t, 3);
        return true;
    }
    }
    return false;
}

/* .M unit, unary format */
static bool trans_m_unary(DisasContext *dc, uint32_t insn)
{
    int dst = F_DST(insn), src2 = F_SRC2(insn);
    TCGv_i32 b = XR(src2), t = tmp();

    switch (F_SRC1(insn)) {
    case 0x18: gen_helper_xpnd4(t, b); break;
    case 0x19: gen_helper_xpnd2(t, b); break;
    case 0x1a:      /* MVD */
        wr(dc, dst, b, 3);
        return true;
    case 0x1c: gen_helper_shfl(t, b); break;
    case 0x1d: gen_helper_deal(t, b); break;
    case 0x1e: gen_helper_bitc4(t, b); break;
    case 0x1f: gen_helper_bitr(t, b); break;
    default:
        return false;
    }
    wr(dc, dst, t, 1);
    return true;
}

/* .D unit, 1 or 2 sources: op in bits 12:7, no cross path */
static bool trans_d(DisasContext *dc, uint32_t insn)
{
    int op = extract32(insn, 7, 6);
    int dst = F_DST(insn), src2 = F_SRC2(insn), src1 = F_SRC1(insn);
    TCGv_i32 b = RR(src2), t = tmp();
    int sh;

    switch (op) {
    case 0x00:      /* MVK scst5 */
        if (src2) {
            return false;
        }
        wr(dc, dst, scst5(src1), 0);
        return true;
    case 0x0c: case 0x0d: case 0x0e:
        return trans_atomic(dc, insn, op);
    case 0x10: op2(dc, tcg_gen_add_i32, b, RR(src1), 0); return true;
    case 0x11: op2(dc, tcg_gen_sub_i32, b, RR(src1), 0); return true;
    case 0x12: op2(dc, tcg_gen_add_i32, b, cst(src1), 0); return true;
    case 0x13: op2(dc, tcg_gen_sub_i32, b, cst(src1), 0); return true;
    case 0x30: case 0x32: case 0x34: case 0x36: case 0x38: case 0x3a:
    case 0x3c: case 0x3d:
        /* ADDAB/H/W/D */
        sh = op >= 0x3c ? 3 : (op - 0x30) >> 2;
        if ((op & 2) || op == 0x3d) {
            tcg_gen_movi_i32(t, src1 << sh);
        } else {
            tcg_gen_shli_i32(t, RR(src1), sh);
        }
        gen_adda(dc, t, b, t, rnum(dc->s, src2));
        wr(dc, dst, t, 0);
        return true;
    case 0x31: case 0x33: case 0x35: case 0x37: case 0x39: case 0x3b:
        /* SUBAB/H/W */
        sh = (op - 0x31) >> 2;
        if (op & 2) {
            tcg_gen_movi_i32(t, src1 << sh);
        } else {
            tcg_gen_shli_i32(t, RR(src1), sh);
        }
        tcg_gen_neg_i32(t, t);
        gen_adda(dc, t, b, t, rnum(dc->s, src2));
        wr(dc, dst, t, 0);
        return true;
    }
    return false;
}

/* .D unit, extended format: op in bits 9:6, cross path */
static bool trans_d_ext(DisasContext *dc, uint32_t insn)
{
    int op = extract32(insn, 6, 4);
    int src2 = F_SRC2(insn), src1 = F_SRC1(insn);

    switch (op) {
    case 0x0: op2(dc, gen_andn, XR(src2), RR(src1), 0); return true;
    case 0x2: op2(dc, tcg_gen_or_i32, RR(src1), XR(src2), 0); return true;
    case 0x3: op2(dc, tcg_gen_or_i32, scst5(src1), XR(src2), 0); return true;
    case 0x4: op2(dc, gen_helper_add2, RR(src1), XR(src2), 0); return true;
    case 0x5: op2(dc, gen_helper_sub2, RR(src1), XR(src2), 0); return true;
    case 0x6: op2(dc, tcg_gen_and_i32, RR(src1), XR(src2), 0); return true;
    case 0x7: op2(dc, tcg_gen_and_i32, scst5(src1), XR(src2), 0); return true;
    case 0xa: op2(dc, tcg_gen_add_i32, RR(src1), XR(src2), 0); return true;
    case 0xb: op2(dc, tcg_gen_add_i32, XR(src2), scst5(src1), 0); return true;
    case 0xc: op2(dc, tcg_gen_sub_i32, RR(src1), XR(src2), 0); return true;
    case 0xe: op2(dc, tcg_gen_xor_i32, RR(src1), XR(src2), 0); return true;
    case 0xf: op2(dc, tcg_gen_xor_i32, scst5(src1), XR(src2), 0); return true;
    }
    return false;
}

/* Loads and stores */

static void gen_load(DisasContext *dc, int side, int reg, TCGv_i32 addr,
                     int size, bool sign)
{
    TCGv_i32 v = tmp();
    MemOp mo = MO_LE | (size == 1 ? MO_8 : size == 2 ? MO_16 : MO_32);
    TCGLabel *l = guard(dc);
    TCGv_i32 a = tmp();

    if (sign) {
        mo |= MO_SIGN;
    }
    tcg_gen_andi_i32(a, addr, ~(size - 1));
    tcg_gen_qemu_ld_i32(v, a, dc->user, mo);
    unguard(l);
    wr_side(dc, side, reg, v, 4);
}

static void gen_store(DisasContext *dc, TCGv_i32 val, TCGv_i32 addr,
                      int size, bool aligned)
{
    MemOp mo = MO_LE | (size == 1 ? MO_8 : size == 2 ? MO_16 : MO_32);
    TCGLabel *l = guard(dc);
    TCGv_i32 a = tmp();

    if (aligned) {
        tcg_gen_andi_i32(a, addr, ~(size - 1));
    } else {
        tcg_gen_mov_i32(a, addr);
    }
    tcg_gen_qemu_st_i32(val, a, dc->user, mo);
    unguard(l);
}

static void gen_load64(DisasContext *dc, int side, int reg, TCGv_i32 addr,
                       bool aligned)
{
    TCGv_i64 v = tmp64();
    TCGLabel *l = guard(dc);
    TCGv_i32 a = tmp();

    if (aligned) {
        tcg_gen_andi_i32(a, addr, ~7);
    } else {
        tcg_gen_mov_i32(a, addr);
    }
    tcg_gen_qemu_ld_i64(v, a, dc->user, MO_LE | MO_64 | MO_UNALN);
    unguard(l);
    wr_pair64(dc, side, reg, v, 4, 4);
}

static void gen_store64(DisasContext *dc, int side, int reg, TCGv_i32 addr,
                        bool aligned)
{
    TCGv_i64 v = tmp64();
    TCGLabel *l = guard(dc);
    TCGv_i32 a = tmp();

    tcg_gen_concat_i32_i64(v, cpu_R[rnum(side, reg & ~1)],
                           cpu_R[rnum(side, reg | 1)]);
    if (aligned) {
        tcg_gen_andi_i32(a, addr, ~7);
    } else {
        tcg_gen_mov_i32(a, addr);
    }
    tcg_gen_qemu_st_i64(v, a, dc->user, MO_LE | MO_64 | MO_UNALN);
    unguard(l);
}

/*
 * Address generation.  Returns the address to access and queues the
 * base register update for the modifying modes.  scale is log2 of the
 * offset unit.
 */
static TCGv_i32 gen_addr(DisasContext *dc, int bside, int baseR, int mode,
                         int offR, int scale)
{
    TCGv_i32 base = cpu_R[rnum(bside, baseR)];
    TCGv_i32 off = tmp(), nb = tmp(), addr;

    if (mode & 4) {
        tcg_gen_shli_i32(off, cpu_R[rnum(bside, offR)], scale);
    } else {
        tcg_gen_movi_i32(off, offR << scale);
    }
    if (!(mode & 1)) {
        tcg_gen_neg_i32(off, off);
    }
    gen_adda(dc, nb, base, off, rnum(bside, baseR));

    if (!(mode & 8)) {
        return nb;
    }
    if (mode & 2) {
        addr = tmp();
        tcg_gen_mov_i32(addr, base);
    } else {
        addr = nb;
    }
    wr_side(dc, bside, baseR, nb, 0);
    return addr;
}

static bool trans_ldst(DisasContext *dc, uint32_t insn)
{
    int op = extract32(insn, 4, 3);
    int y = extract32(insn, 7, 1);
    int r = extract32(insn, 8, 1);
    int mode = extract32(insn, 9, 4);
    int offR = F_SRC1(insn), baseR = F_SRC2(insn), sd = F_DST(insn);
    int side = dc->s;   /* data register side */
    TCGv_i32 addr;

    if ((mode & 0xa) == 0x2 || (mode & 0xe) == 0x6) {
        return false;   /* reserved modes */
    }

    if (r) {
        switch (op) {
        case 2: case 6: {   /* LDDW: op 6, LDNDW: op 2 */
            bool nonal = op == 2;
            int scale = nonal ? (extract32(insn, 23, 1) ? 3 : 0) : 3;
            int dreg = nonal ? extract32(insn, 24, 4) << 1 : sd;

            addr = gen_addr(dc, y, baseR, mode, offR, scale);
            gen_load64(dc, side, dreg, addr, !nonal);
            return true;
        }
        case 3:             /* LDNW */
            addr = gen_addr(dc, y, baseR, mode, offR, 2);
            {
                TCGv_i32 v = tmp();
                TCGLabel *l = guard(dc);
                tcg_gen_qemu_ld_i32(v, addr, dc->user, MO_LE | MO_32 | MO_UNALN);
                unguard(l);
                wr_side(dc, side, sd, v, 4);
            }
            return true;
        case 4:             /* STDW */
            addr = gen_addr(dc, y, baseR, mode, offR, 3);
            gen_store64(dc, side, sd, addr, true);
            return true;
        case 5:             /* STNW */
            addr = gen_addr(dc, y, baseR, mode, offR, 2);
            gen_store(dc, cpu_R[rnum(side, sd)], addr, 4, false);
            return true;
        case 7: {           /* STNDW */
            int scale = extract32(insn, 23, 1) ? 3 : 0;

            addr = gen_addr(dc, y, baseR, mode, offR, scale);
            gen_store64(dc, side, extract32(insn, 24, 4) << 1, addr, false);
            return true;
        }
        }
        return false;
    }

    switch (op) {
    case 0: /* LDHU */
        addr = gen_addr(dc, y, baseR, mode, offR, 1);
        gen_load(dc, side, sd, addr, 2, false);
        return true;
    case 1: /* LDBU */
        addr = gen_addr(dc, y, baseR, mode, offR, 0);
        gen_load(dc, side, sd, addr, 1, false);
        return true;
    case 2: /* LDB */
        addr = gen_addr(dc, y, baseR, mode, offR, 0);
        gen_load(dc, side, sd, addr, 1, true);
        return true;
    case 3: /* STB */
        addr = gen_addr(dc, y, baseR, mode, offR, 0);
        gen_store(dc, cpu_R[rnum(side, sd)], addr, 1, true);
        return true;
    case 4: /* LDH */
        addr = gen_addr(dc, y, baseR, mode, offR, 1);
        gen_load(dc, side, sd, addr, 2, true);
        return true;
    case 5: /* STH */
        addr = gen_addr(dc, y, baseR, mode, offR, 1);
        gen_store(dc, cpu_R[rnum(side, sd)], addr, 2, true);
        return true;
    case 6: /* LDW */
        addr = gen_addr(dc, y, baseR, mode, offR, 2);
        gen_load(dc, side, sd, addr, 4, false);
        return true;
    case 7: /* STW */
        addr = gen_addr(dc, y, baseR, mode, offR, 2);
        gen_store(dc, cpu_R[rnum(side, sd)], addr, 4, true);
        return true;
    }
    return false;
}

/* 15-bit offsets from B14/B15, and the C64x+ long ADDA forms */
static bool trans_ldst_long(DisasContext *dc, uint32_t insn)
{
    int op = extract32(insn, 4, 3);
    int y = extract32(insn, 7, 1);
    uint32_t off = extract32(insn, 8, 15);
    int sd = F_DST(insn);
    TCGv_i32 base = cpu_R[REG_B(14 + y)];
    TCGv_i32 addr = tmp();
    static const int8_t scale[8] = { 1, 0, 0, 0, 1, 1, 2, 2 };

    if (F_CREG(insn) == 0 && F_Z(insn) == 1) {
        /* ADDAB/ADDAH/ADDAW with a 15-bit constant */
        dc->cond = NULL;
        switch (op) {
        case 3:
        case 5:
        case 7:
            tcg_gen_addi_i32(addr, base, off << ((op - 3) >> 1));
            wr(dc, sd, addr, 0);
            return true;
        }
        return false;
    }

    tcg_gen_addi_i32(addr, base, off << scale[op]);
    switch (op) {
    case 0: gen_load(dc, dc->s, sd, addr, 2, false); return true;
    case 1: gen_load(dc, dc->s, sd, addr, 1, false); return true;
    case 2: gen_load(dc, dc->s, sd, addr, 1, true); return true;
    case 3: gen_store(dc, cpu_R[rnum(dc->s, sd)], addr, 1, true); return true;
    case 4: gen_load(dc, dc->s, sd, addr, 2, true); return true;
    case 5: gen_store(dc, cpu_R[rnum(dc->s, sd)], addr, 2, true); return true;
    case 6: gen_load(dc, dc->s, sd, addr, 4, false); return true;
    case 7: gen_store(dc, cpu_R[rnum(dc->s, sd)], addr, 4, true); return true;
    }
    return false;
}

/* LL, SL, CMTL */
static bool trans_atomic(DisasContext *dc, uint32_t insn, int op)
{
    int sd = F_DST(insn);
    TCGv_i32 addr = cpu_R[rnum(dc->s, F_SRC2(insn))];
    TCGv_i32 v = tmp();
    TCGLabel *l;

    if (!dc->s || F_SRC1(insn)) {
        return false;
    }
    l = guard(dc);
    switch (op) {
    case 0x0c:  /* LL */
        gen_helper_ll(v, tcg_env, addr);
        break;
    case 0x0d:  /* SL */
        gen_helper_sl(tcg_env, addr, cpu_R[rnum(dc->s, sd)]);
        break;
    case 0x0e:  /* CMTL */
        gen_helper_cmtl(v, tcg_env, addr);
        break;
    }
    unguard(l);
    if (op != 0x0d) {
        wr(dc, sd, v, 4);
    }
    return true;
}

/* MVC */
static bool trans_mvc(DisasContext *dc, uint32_t insn, bool to_cr)
{
    int crlo = to_cr ? F_DST(insn) : F_SRC2(insn);
    TCGLabel *l;

    if (!dc->s) {
        return false;
    }
    if (to_cr) {
        TCGv_i32 v = XR(F_SRC2(insn));

        switch (crlo) {
        case CR_ILC:
        case CR_RILC: {
            /* the loop buffer sees the new value four cycles later */
            TCGv_i32 t = tmp();
            tcg_gen_mov_i32(t, v);
            queue(dc, crlo == CR_ILC ? DQ_DEST_ILC : DQ_DEST_RILC, t, 3);
            return true;
        }
        }
        l = guard(dc);
        gen_helper_mvc_to(tcg_env, cst(crlo), v);
        unguard(l);
        switch (crlo) {
        case CR_AMR:
        case CR_CSR:
        case CR_ISR_IFR:
        case CR_ICR:
        case CR_IER:
        case CR_TSR:
            dc->exit_tb = true;
            break;
        }
        return true;
    } else {
        TCGv_i32 t = tmp();

        if (F_X(insn)) {
            return false;
        }
        if (crlo == CR_PCE1) {
            tcg_gen_movi_i32(t, dc->pce1);
        } else {
            l = guard(dc);
            gen_helper_mvc_from(t, tcg_env, cst(crlo));
            unguard(l);
        }
        wr(dc, F_DST(insn), t, 0);
        return true;
    }
}

/* Instructions without a functional unit */
static bool trans_nfu(DisasContext *dc, uint32_t insn)
{
    int op = extract32(insn, 13, 4);
    bool noncond = F_CREG(insn) == 0 && F_Z(insn) == 1;

    if (noncond) {
        dc->cond = NULL;
        if (extract32(insn, 17, 11)) {
            return false;
        }
        switch (op) {
        case 0:     /* SWE */
        case 1:     /* SWENR */
            dc->swe = true;
            if (op == 1) {
                qemu_log_mask(LOG_UNIMP, "c6x: SWENR at %08x\n", dc->ep);
            }
            return true;
        case 2:     /* DINT */
            gen_helper_dint(tcg_env);
            return true;
        case 3:     /* RINT */
            gen_helper_rint(tcg_env);
            dc->exit_tb = true;
            return true;
        }
        return false;
    }

    if (!extract32(insn, 17, 1)) {
        if (F_CREG(insn) || F_Z(insn) || extract32(insn, 18, 10)) {
            return false;
        }
        if (op == 0xf) {    /* IDLE */
            dc->idle = true;
            return true;
        }
        dc->nops = MAX(dc->nops, op);   /* NOP op + 1 */
        return true;
    }
    return trans_sploop(dc, insn, op);
}

static bool decode(DisasContext *dc, uint32_t insn)
{
    switch (extract32(insn, 2, 2)) {
    case 1:
        return trans_ldst(dc, insn);
    case 3:
        return trans_ldst_long(dc, insn);
    case 2:
        if (insn & 0x10) {
            return trans_l(dc, insn);
        }
        if (insn & 0x20) {
            return trans_mvk(dc, insn);
        }
        return trans_field(dc, insn);
    }

    switch (extract32(insn, 4, 2)) {
    case 0:
        if (insn & 0x40) {
            return trans_d(dc, insn);
        }
        if ((insn & 0x1ffe) == 0) {
            return trans_nfu(dc, insn);
        }
        return trans_m_mpy(dc, insn);
    case 1:
        if (insn & 0x40) {
            return trans_addk(dc, insn);
        }
        return trans_bimm(dc, insn);
    case 2:
        return trans_s(dc, insn);
    default:
        switch (extract32(insn, 10, 2)) {
        case 2:
            return trans_d_ext(dc, insn);
        case 3:
            return trans_s_ext(dc, insn);
        default:
            if ((insn & 0xffc) == 0xf0) {
                return trans_m_unary(dc, insn);
            }
            return trans_m_compound(dc, insn);
        }
    }
}
