/*
 * IA-64 translation: branches (B unit, brl)
 *
 * Every branch reads its qualifying predicate itself.  Branches that
 * change the frame marker leave through lookup_and_goto_ptr, since the
 * frame marker is part of the TB key (cs_base).
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

static TCGv_i32 branch_cond(DisasContext *dc)
{
    return load_pr(dc, field(dc->bits, 0, 6));
}

/* Fall through to the next instruction when cond is false. */
static TCGLabel *gen_branch_skip(DisasContext *dc, TCGv_i32 cond)
{
    TCGLabel *nt = gen_new_label();

    tcg_gen_brcondi_i32(TCG_COND_EQ, cond, 0, nt);
    return nt;
}

static void gen_branch_fallthrough(DisasContext *dc, TCGLabel *nt)
{
    gen_set_label(nt);
    gen_goto_next(dc, 1);
    dc->base.is_jmp = DISAS_NORETURN;
}

/* br.call / brl.call: b1 = return bundle, push the frame. */
static void gen_call(DisasContext *dc, int b1, TCGv_i64 target)
{
    TCGv_i64 t = tcg_temp_new_i64();
    int ri;

    tcg_gen_mov_i64(t, target);
    tcg_gen_andi_i64(t, t, ~0xfull);
    tcg_gen_movi_i64(cpu_br[b1], next_ip(dc, &ri) + (ri ? 16 : 0));
    gen_save_ip(dc);
    gen_helper_br_call(tcg_env);
    tcg_gen_mov_i64(cpu_ip, t);
    tcg_gen_movi_i32(cpu_ri, 0);
    tcg_gen_lookup_and_goto_ptr();
}

static void trans_b_ipcall(DisasContext *dc, int b1, uint64_t target)
{
    TCGLabel *nt = gen_branch_skip(dc, branch_cond(dc));

    gen_call(dc, b1, tcg_constant_i64(target));
    gen_branch_fallthrough(dc, nt);
}

static void trans_b_loop(DisasContext *dc, int iop, uint64_t target)
{
    int qp = field(dc->bits, 0, 6);
    int kind;
    TCGv_i32 taken = tcg_temp_new_i32();
    TCGLabel *nt = gen_new_label();
    int ri;
    uint64_t nip = next_ip(dc, &ri);

    if (dc->slot != 2 ||
        ((iop == ASM_OP_BR_CLOOP || iop == ASM_OP_BR_CTOP ||
          iop == ASM_OP_BR_CEXIT) && qp != 0)) {
        gen_illegal(dc);
        dc->base.is_jmp = DISAS_NORETURN;
        return;
    }
    if (iop == ASM_OP_BR_CLOOP) {
        /* Frame marker unchanged: chain directly. */
        TCGLabel *zero = gen_new_label();

        tcg_gen_brcondi_i64(TCG_COND_EQ, cpu_ar_lc, 0, zero);
        tcg_gen_subi_i64(cpu_ar_lc, cpu_ar_lc, 1);
        gen_goto(dc, 0, target, 0);
        gen_set_label(zero);
        gen_goto(dc, 1, nip, ri);
        dc->base.is_jmp = DISAS_NORETURN;
        return;
    }

    switch (iop) {
    case ASM_OP_BR_CTOP: kind = 0; break;
    case ASM_OP_BR_CEXIT: kind = 1; break;
    case ASM_OP_BR_WTOP: kind = 2; break;
    default: kind = 3; break;
    }
    gen_helper_br_loop(taken, tcg_env, tcg_constant_i32(kind),
                       tcg_constant_i32(qp ? pr_idx(dc, qp) : 0));
    tcg_gen_brcondi_i32(TCG_COND_EQ, taken, 0, nt);
    tcg_gen_movi_i64(cpu_ip, target);
    tcg_gen_movi_i32(cpu_ri, 0);
    tcg_gen_lookup_and_goto_ptr();
    gen_set_label(nt);
    tcg_gen_movi_i64(cpu_ip, nip);
    tcg_gen_movi_i32(cpu_ri, ri);
    tcg_gen_lookup_and_goto_ptr();
    dc->base.is_jmp = DISAS_NORETURN;
}

static void trans_b_indirect(DisasContext *dc, const struct asm_inst *in)
{
    uint64_t b = in->i_bits;
    int b2 = field(b, 13, 3);
    TCGLabel *nt;
    TCGv_i64 t = tcg_temp_new_i64();

    switch (in->i_iop) {
    case ASM_OP_BR_COND:
        nt = gen_branch_skip(dc, branch_cond(dc));
        tcg_gen_andi_i64(cpu_ip, cpu_br[b2], ~0xfull);
        tcg_gen_movi_i32(cpu_ri, 0);
        tcg_gen_lookup_and_goto_ptr();
        gen_branch_fallthrough(dc, nt);
        break;
    case ASM_OP_BR_RET:
        nt = gen_branch_skip(dc, branch_cond(dc));
        tcg_gen_andi_i64(t, cpu_br[b2], ~0xfull);
        gen_save_ip(dc);
        gen_helper_br_ret(tcg_env);
        tcg_gen_mov_i64(cpu_ip, t);
        tcg_gen_movi_i32(cpu_ri, 0);
        /* br.ret may lower the privilege level: back to the main loop. */
        tcg_gen_exit_tb(NULL, 0);
        gen_branch_fallthrough(dc, nt);
        break;
    case ASM_OP_BR_CALL:    /* B5 */
        nt = gen_branch_skip(dc, branch_cond(dc));
        gen_call(dc, field(b, 6, 3), cpu_br[b2]);
        gen_branch_fallthrough(dc, nt);
        break;
    default:                /* br.ia: no IA-32 */
        gen_fault(dc, IVT_GENERAL, GE_DISABLED_ISA);
        dc->base.is_jmp = DISAS_NORETURN;
        break;
    }
}

static void trans_b_misc(DisasContext *dc, const struct asm_inst *in)
{
    switch (in->i_iop) {
    case ASM_OP_COVER:
        gen_save_ip(dc);
        gen_helper_cover(tcg_env);
        dc->base.is_jmp = DISAS_EXIT_NEXT;
        break;
    case ASM_OP_CLRRRB_:
        tcg_gen_andi_i64(cpu_cfm, cpu_cfm,
                         ~(R_CFM_RRB_GR_MASK | R_CFM_RRB_FR_MASK |
                           R_CFM_RRB_PR_MASK));
        dc->base.is_jmp = DISAS_EXIT_NEXT;
        break;
    case ASM_OP_CLRRRB_PR:
        tcg_gen_andi_i64(cpu_cfm, cpu_cfm, ~R_CFM_RRB_PR_MASK);
        dc->base.is_jmp = DISAS_EXIT_NEXT;
        break;
    case ASM_OP_RFI:
        if (!check_priv(dc)) {
            dc->base.is_jmp = DISAS_NORETURN;
            break;
        }
        gen_save_ip(dc);
        gen_helper_rfi(tcg_env);
        dc->base.is_jmp = DISAS_EXIT;
        break;
    case ASM_OP_BSW_0:
    case ASM_OP_BSW_1:
        if (!check_priv(dc)) {
            dc->base.is_jmp = DISAS_NORETURN;
            break;
        }
        gen_helper_bsw(tcg_env, tcg_constant_i32(in->i_iop == ASM_OP_BSW_1));
        dc->base.is_jmp = DISAS_EXIT_NEXT;
        break;
    case ASM_OP_EPC:
        gen_save_ip(dc);
        gen_helper_epc(tcg_env);
        dc->base.is_jmp = DISAS_EXIT_NEXT;
        break;
    default:                /* vmsw: no virtualization support */
        gen_illegal(dc);
        dc->base.is_jmp = DISAS_NORETURN;
        break;
    }
}

static void trans_b(DisasContext *dc, const struct asm_inst *in)
{
    uint64_t b = in->i_bits;

    switch (in->i_format) {
    case ASM_FMT_B1:
    case ASM_FMT_B2: {
        uint64_t target = dc->ip + in->i_oper[1].o_value;

        if (in->i_iop == ASM_OP_BR_COND) {
            gen_branch_cond(dc, branch_cond(dc), target);
        } else {
            trans_b_loop(dc, in->i_iop, target);
        }
        break;
    }
    case ASM_FMT_B3:
        trans_b_ipcall(dc, field(b, 6, 3), dc->ip + in->i_oper[2].o_value);
        break;
    case ASM_FMT_B4:
    case ASM_FMT_B5:
        trans_b_indirect(dc, in);
        break;
    case ASM_FMT_B6:
    case ASM_FMT_B7:        /* brp: hint only */
        break;
    case ASM_FMT_B8:
        trans_b_misc(dc, in);
        break;
    case ASM_FMT_B9:
        if (in->i_iop == ASM_OP_BREAK_B) {
            /* break.b reports a zero immediate */
            gen_break(dc, 0);
        }
        break;
    default:
        gen_illegal(dc);
        dc->base.is_jmp = DISAS_NORETURN;
        break;
    }
}

static void trans_x(DisasContext *dc, const struct asm_inst *in)
{
    uint64_t b = in->i_bits;

    switch (in->i_iop) {
    case ASM_OP_MOVL: {
        int r1 = R1(b);

        if (check_gr_dst(dc, r1)) {
            store_gr(dc, r1, tcg_constant_i64(in->i_oper[2].o_value), NULL);
        }
        break;
    }
    case ASM_OP_BRL_COND:
        gen_branch_cond(dc, branch_cond(dc), dc->ip + in->i_oper[1].o_value);
        break;
    case ASM_OP_BRL_CALL:
        trans_b_ipcall(dc, field(b, 6, 3), dc->ip + in->i_oper[2].o_value);
        break;
    case ASM_OP_BREAK_X:
        gen_break(dc, in->i_oper[1].o_value & 0x1fffff);
        break;
    case ASM_OP_NOP_X:
    case ASM_OP_HINT_X:
        break;
    default:
        gen_illegal(dc);
        break;
    }
}
