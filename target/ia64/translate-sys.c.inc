/*
 * IA-64 translation: M-unit system instructions and the dispatch of the
 * M unit
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

static TCGv_i64 ar_global(int ar)
{
    switch (ar) {
    case AR_CCV: return cpu_ar_ccv;
    case AR_UNAT: return cpu_ar_unat;
    case AR_CSD: return cpu_ar_csd;
    case AR_SSD: return cpu_ar_ssd;
    case AR_PFS: return cpu_ar_pfs;
    case AR_LC: return cpu_ar_lc;
    case AR_EC: return cpu_ar_ec;
    }
    return NULL;
}

static void gen_mov_to_ar(DisasContext *dc, int ar, TCGv_i64 v)
{
    TCGv_i64 g = ar_global(ar);

    if (ar == AR_EC) {
        tcg_gen_andi_i64(cpu_ar_ec, v, 0x3f);
        return;
    }
    if (g) {
        tcg_gen_mov_i64(g, v);
        return;
    }
    gen_save_ip(dc);
    gen_helper_mov_to_ar(tcg_env, tcg_constant_i32(ar), v);
    /* rsc, bspstore, fpsr, itc: state outside the TB key, but continue */
    if (ar == AR_RSC || ar == AR_FPSR) {
        dc->base.is_jmp = DISAS_EXIT_NEXT;
    }
}

static void gen_mov_from_ar(DisasContext *dc, int r1, int ar)
{
    TCGv_i64 g = ar_global(ar);
    TCGv_i64 d = tcg_temp_new_i64();

    if (!check_gr_dst(dc, r1)) {
        return;
    }
    if (g) {
        tcg_gen_mov_i64(d, g);
    } else {
        gen_save_ip(dc);
        gen_helper_mov_from_ar(d, tcg_env, tcg_constant_i32(ar));
    }
    store_gr(dc, r1, d, NULL);
}

static void trans_m_movar(DisasContext *dc, const struct asm_inst *in)
{
    uint64_t b = in->i_bits;
    int ar = field(b, 20, 7);

    if (ar >= 64 && ar < 112) {
        /* I-unit registers through the M unit */
        gen_illegal(dc);
        return;
    }
    if (in->i_format == ASM_FMT_M31) {
        gen_mov_from_ar(dc, R1(b), ar);
    } else if (in->i_format == ASM_FMT_M29) {
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

static void trans_m_movcr(DisasContext *dc, const struct asm_inst *in)
{
    uint64_t b = in->i_bits;
    int cr = field(b, 20, 7);

    if (!check_priv(dc)) {
        return;
    }
    if (in->i_format == ASM_FMT_M33) {
        int r1 = R1(b);
        TCGv_i64 d = tcg_temp_new_i64();

        if (!check_gr_dst(dc, r1)) {
            return;
        }
        gen_save_ip(dc);
        gen_helper_mov_from_cr(d, tcg_env, tcg_constant_i32(cr));
        store_gr(dc, r1, d, NULL);
        if (cr == CR_IVR) {
            dc->base.is_jmp = DISAS_EXIT_NEXT;
        }
    } else {
        int r2 = R2(b);

        if (!check_gr(dc, r2)) {
            return;
        }
        gen_nat_check(dc, load_nat(dc, r2), 0);
        gen_save_ip(dc);
        gen_helper_mov_to_cr(tcg_env, tcg_constant_i32(cr), load_gr(dc, r2));
        dc->base.is_jmp = DISAS_EXIT_NEXT;
    }
}

static void trans_m_alloc(DisasContext *dc, const struct asm_inst *in)
{
    uint64_t b = in->i_bits;
    int r1 = R1(b);
    int sof = field(b, 13, 7);
    int sol = field(b, 20, 7);
    int sor = field(b, 27, 4);
    int idx, rot;
    TCGv_i64 pfs = tcg_temp_new_i64();

    if (field(b, 0, 6) != 0 || sol > sof || sof > 96 || sor * 8 > sof) {
        gen_illegal(dc);
        dc->base.is_jmp = DISAS_NORETURN;
        return;
    }
    /* r1 names a register of the new frame */
    if (r1 < 32 || r1 - 32 >= sof) {
        if (r1 == 0 || r1 >= 32) {
            gen_illegal(dc);
            dc->base.is_jmp = DISAS_NORETURN;
            return;
        }
        idx = r1;
    } else {
        idx = r1 - 32;
        rot = sor * 8;
        if (idx < rot) {
            idx = (idx + dc->rrb_gr) % rot;
        }
        idx += 32;
    }
    tcg_gen_mov_i64(pfs, cpu_ar_pfs);
    gen_save_ip(dc);
    gen_helper_alloc(tcg_env, tcg_constant_i32(sof), tcg_constant_i32(sol),
                     tcg_constant_i32(sor));
    tcg_gen_mov_i64(cpu_gr[idx], pfs);
    tcg_gen_movi_i32(cpu_nat[idx], 0);
    dc->base.is_jmp = DISAS_EXIT_NEXT;
}

static void trans_m_psr(DisasContext *dc, const struct asm_inst *in)
{
    uint64_t b = in->i_bits;
    uint64_t imm24 = field(b, 6, 21) | (field(b, 31, 2) << 21) |
                     (field(b, 36, 1) << 23);

    switch (in->i_iop) {
    case ASM_OP_SUM:
        tcg_gen_ori_i64(cpu_psr, cpu_psr, imm24 & PSR_UM_MASK);
        dc->base.is_jmp = DISAS_EXIT_NEXT;
        return;
    case ASM_OP_RUM:
        tcg_gen_andi_i64(cpu_psr, cpu_psr, ~(imm24 & PSR_UM_MASK));
        dc->base.is_jmp = DISAS_EXIT_NEXT;
        return;
    case ASM_OP_SSM:
    case ASM_OP_RSM:
        if (!check_priv(dc)) {
            return;
        }
        gen_save_ip(dc);
        gen_helper_ssm_rsm(tcg_env, tcg_constant_i64(imm24),
                           tcg_constant_i32(in->i_iop == ASM_OP_SSM));
        dc->base.is_jmp = DISAS_EXIT_NEXT;
        return;
    case ASM_OP_MOV_PSR_L:
    case ASM_OP_MOV_PSR_UM: {     /* M35: mov psr.l/um = r2 */
        int r2 = R2(b);

        if (in->i_format == ASM_FMT_M36) {
            break;
        }
        if (in->i_iop == ASM_OP_MOV_PSR_L && !check_priv(dc)) {
            return;
        }
        if (!check_gr(dc, r2)) {
            return;
        }
        gen_nat_check(dc, load_nat(dc, r2), 0);
        gen_save_ip(dc);
        gen_helper_mov_to_psr(tcg_env, load_gr(dc, r2),
                              tcg_constant_i32(in->i_iop == ASM_OP_MOV_PSR_L));
        dc->base.is_jmp = DISAS_EXIT_NEXT;
        return;
    }
    default:
        break;
    }

    /* M36: mov r1 = psr / psr.um */
    {
        int r1 = R1(b);
        TCGv_i64 d = tcg_temp_new_i64();

        if (in->i_iop == ASM_OP_MOV_PSR && !check_priv(dc)) {
            return;
        }
        if (!check_gr_dst(dc, r1)) {
            return;
        }
        if (in->i_iop == ASM_OP_MOV_PSR) {
            /* bits 31:0 and 36:35 (mc, it) */
            tcg_gen_andi_i64(d, cpu_psr, (PSR_DEFINED_MASK & 0xffffffffull) |
                             PSR_MC | PSR_IT);
        } else {
            tcg_gen_andi_i64(d, cpu_psr, PSR_UM_MASK);
        }
        store_gr(dc, r1, d, NULL);
    }
}

/* mov indirect register files and itr */
static void trans_m_indirect(DisasContext *dc, const struct asm_inst *in)
{
    uint64_t b = in->i_bits;
    int r3 = R3(b);
    int which;

    switch (in->i_iop) {
    case ASM_OP_MOV_RR: which = 0; break;
    case ASM_OP_MOV_PKR: which = 1; break;
    case ASM_OP_MOV_IBR: which = 2; break;
    case ASM_OP_MOV_DBR: which = 3; break;
    case ASM_OP_MOV_PMC: which = 4; break;
    case ASM_OP_MOV_PMD: which = 5; break;
    case ASM_OP_MOV_CPUID: which = 6; break;
    case ASM_OP_MOV_MSR: which = 7; break;
    case ASM_OP_ITR_I: which = 8; break;
    case ASM_OP_ITR_D: which = 9; break;
    default:
        gen_illegal(dc);
        return;
    }
    if (which != 6 && which != 5 && !check_priv(dc)) {
        return;
    }
    if (!check_gr(dc, r3)) {
        return;
    }
    gen_nat_check(dc, load_nat(dc, r3), 0);
    gen_save_ip(dc);
    if (in->i_format == ASM_FMT_M43) {
        int r1 = R1(b);
        TCGv_i64 d = tcg_temp_new_i64();

        if (!check_gr_dst(dc, r1)) {
            return;
        }
        gen_helper_mov_from_ind(d, tcg_env, tcg_constant_i32(which),
                                load_gr(dc, r3));
        store_gr(dc, r1, d, NULL);
    } else {
        int r2 = R2(b);

        if (!check_gr(dc, r2)) {
            return;
        }
        gen_nat_check(dc, load_nat(dc, r2), 0);
        gen_helper_mov_to_ind(tcg_env, tcg_constant_i32(which),
                              load_gr(dc, r3), load_gr(dc, r2));
        dc->base.is_jmp = DISAS_EXIT_NEXT;
    }
}

static void trans_m_tlb(DisasContext *dc, const struct asm_inst *in)
{
    uint64_t b = in->i_bits;
    int r2 = R2(b), r3 = R3(b);

    if (!check_priv(dc)) {
        return;
    }
    gen_save_ip(dc);
    switch (in->i_iop) {
    case ASM_OP_ITC_I:
    case ASM_OP_ITC_D:
        if (!check_gr(dc, r2)) {
            return;
        }
        gen_nat_check(dc, load_nat(dc, r2), 0);
        gen_helper_itc(tcg_env, tcg_constant_i32(in->i_iop == ASM_OP_ITC_I),
                       load_gr(dc, r2));
        break;
    case ASM_OP_PTC_L:
    case ASM_OP_PTC_G:
    case ASM_OP_PTC_GA:
        if (!check_gr(dc, r2) || !check_gr(dc, r3)) {
            return;
        }
        gen_helper_ptc(tcg_env, load_gr(dc, r3), load_gr(dc, r2));
        break;
    case ASM_OP_PTR_I:
    case ASM_OP_PTR_D:
        if (!check_gr(dc, r2) || !check_gr(dc, r3)) {
            return;
        }
        gen_helper_ptr(tcg_env, tcg_constant_i32(in->i_iop == ASM_OP_PTR_I),
                       load_gr(dc, r3), load_gr(dc, r2));
        break;
    case ASM_OP_PTC_E:
        gen_helper_ptc_e(tcg_env);
        break;
    }
    dc->base.is_jmp = DISAS_EXIT_NEXT;
}

/* thash, ttag, tpa, tak */
static void trans_m_xlate(DisasContext *dc, const struct asm_inst *in)
{
    uint64_t b = in->i_bits;
    int r1 = R1(b), r3 = R3(b);
    TCGv_i64 d = tcg_temp_new_i64();
    TCGv_i32 nat = load_nat(dc, r3);

    if (!check_gr_dst(dc, r1) || !check_gr(dc, r3)) {
        return;
    }
    gen_save_ip(dc);
    switch (in->i_iop) {
    case ASM_OP_THASH:
        gen_helper_thash(d, tcg_env, load_gr(dc, r3));
        break;
    case ASM_OP_TTAG:
        gen_helper_ttag(d, tcg_env, load_gr(dc, r3));
        break;
    case ASM_OP_TPA:
        if (!check_priv(dc)) {
            return;
        }
        gen_nat_check(dc, nat, ISR_NA);
        gen_helper_tpa(d, tcg_env, load_gr(dc, r3));
        nat = NULL;
        break;
    case ASM_OP_TAK:
        if (!check_priv(dc)) {
            return;
        }
        gen_nat_check(dc, nat, ISR_NA);
        gen_helper_tak(d, tcg_env, load_gr(dc, r3));
        nat = NULL;
        break;
    default:
        gen_illegal(dc);
        return;
    }
    store_gr(dc, r1, d, nat);
}

static void trans_m_probe(DisasContext *dc, const struct asm_inst *in)
{
    uint64_t b = in->i_bits;
    int r1 = R1(b), r2 = R2(b), r3 = R3(b);
    int kind;
    TCGv_i64 pl;
    TCGv_i64 d = tcg_temp_new_i64();

    if (!check_gr(dc, r3)) {
        return;
    }
    switch (in->i_iop) {
    case ASM_OP_PROBE_R: kind = 0; break;
    case ASM_OP_PROBE_W: kind = 1; break;
    case ASM_OP_PROBE_R_FAULT: kind = 2; break;
    case ASM_OP_PROBE_W_FAULT: kind = 3; break;
    default: kind = 4; break;           /* probe.rw.fault */
    }
    if (in->i_format == ASM_FMT_M38) {
        if (!check_gr(dc, r2)) {
            return;
        }
        pl = load_gr(dc, r2);
    } else {
        pl = tcg_constant_i64(field(b, 13, 2));
    }
    if (kind < 2 && !check_gr_dst(dc, r1)) {
        return;
    }
    gen_nat_check(dc, load_nat(dc, r3), kind < 2 ? 0 : ISR_NA);
    gen_save_ip(dc);
    gen_helper_probe(d, tcg_env, tcg_constant_i32(kind), load_gr(dc, r3), pl);
    if (kind < 2) {
        store_gr(dc, r1, d, NULL);
    }
}

static void trans_m(DisasContext *dc, const struct asm_inst *in)
{
    uint64_t b = in->i_bits;

    switch (in->i_format) {
    case ASM_FMT_M1: case ASM_FMT_M2: case ASM_FMT_M3:
        trans_m_load(dc, in);
        break;
    case ASM_FMT_M4: case ASM_FMT_M5:
        trans_m_store(dc, in);
        break;
    case ASM_FMT_M6: case ASM_FMT_M7: case ASM_FMT_M8:
    case ASM_FMT_M9: case ASM_FMT_M10:
    case ASM_FMT_M11: case ASM_FMT_M12:
    case ASM_FMT_M18: case ASM_FMT_M19:
        trans_m_fp(dc, in);
        break;
    case ASM_FMT_M13: case ASM_FMT_M14: case ASM_FMT_M15:
        trans_m_lfetch(dc, in);
        break;
    case ASM_FMT_M16: case ASM_FMT_M17:
        trans_m_sem(dc, in);
        break;
    case ASM_FMT_M20:       /* chk.s.m r2 */
        if (check_gr(dc, R2(b))) {
            gen_chk(dc, load_nat(dc, R2(b)), in->i_oper[2].o_value);
        }
        break;
    case ASM_FMT_M21:       /* chk.s f2 */
        gen_chk_f(dc, field(b, 13, 7), in->i_oper[2].o_value);
        break;
    case ASM_FMT_M22:
    case ASM_FMT_M23:       /* chk.a: the ALAT never holds an entry */
        gen_chk(dc, tcg_constant_i32(1), in->i_oper[2].o_value);
        break;
    case ASM_FMT_M24:
        switch (in->i_iop) {
        case ASM_OP_MF_:
        case ASM_OP_MF_A:
            tcg_gen_mb(TCG_MO_ALL | TCG_BAR_SC);
            break;
        case ASM_OP_SRLZ_I:
        case ASM_OP_SYNC_I:
            /* re-fetch: code may have been changed */
            dc->base.is_jmp = DISAS_EXIT_NEXT;
            break;
        default:            /* invala, fwb, srlz.d */
            break;
        }
        break;
    case ASM_FMT_M25:
        if (field(b, 0, 6) != 0) {
            gen_illegal(dc);
            break;
        }
        gen_save_ip(dc);
        if (in->i_iop == ASM_OP_FLUSHRS) {
            gen_helper_flushrs(tcg_env);
        } else {
            gen_helper_loadrs(tcg_env);
        }
        dc->base.is_jmp = DISAS_EXIT_NEXT;
        break;
    case ASM_FMT_M26: case ASM_FMT_M27:     /* invala.e */
        break;
    case ASM_FMT_M28:       /* fc, fc.i */
        if (check_gr(dc, R3(b))) {
            gen_nat_check(dc, load_nat(dc, R3(b)), ISR_NA);
        }
        break;
    case ASM_FMT_M29: case ASM_FMT_M30: case ASM_FMT_M31:
        trans_m_movar(dc, in);
        break;
    case ASM_FMT_M32: case ASM_FMT_M33:
        trans_m_movcr(dc, in);
        break;
    case ASM_FMT_M34:
        trans_m_alloc(dc, in);
        break;
    case ASM_FMT_M35: case ASM_FMT_M36: case ASM_FMT_M44:
        trans_m_psr(dc, in);
        break;
    case ASM_FMT_M37:
        gen_break(dc, field(b, 6, 20) | (field(b, 36, 1) << 20));
        break;
    case ASM_FMT_M38: case ASM_FMT_M39: case ASM_FMT_M40:
        trans_m_probe(dc, in);
        break;
    case ASM_FMT_M41: case ASM_FMT_M45: case ASM_FMT_M47:
        trans_m_tlb(dc, in);
        break;
    case ASM_FMT_M42: case ASM_FMT_M43:
        trans_m_indirect(dc, in);
        break;
    case ASM_FMT_M46:
        trans_m_xlate(dc, in);
        break;
    case ASM_FMT_M48:       /* nop.m, hint.m */
        break;
    default:
        gen_illegal(dc);
        break;
    }
}
