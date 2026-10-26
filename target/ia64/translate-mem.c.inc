/*
 * IA-64 translation: integer loads, stores and semaphores
 *
 * The ALAT is not modelled: advanced loads are ordinary loads, check
 * loads always reload and chk.a always branches to the recovery code,
 * which the architecture allows (an ALAT may lose entries at any time).
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

/* ld kinds */
enum { LDK_NORMAL, LDK_SPEC, LDK_ACQ, LDK_FILL, LDK_CHECK };

static int ld_size(int iop)
{
    switch (iop) {
    case ASM_OP_LD1_ ... ASM_OP_LD1_SA: return 1;
    case ASM_OP_LD16_ ... ASM_OP_LD16_ACQ: return 16;
    case ASM_OP_LD2_ ... ASM_OP_LD2_SA: return 2;
    case ASM_OP_LD4_ ... ASM_OP_LD4_SA: return 4;
    case ASM_OP_LD8_ ... ASM_OP_LD8_SA: return 8;
    }
    return 0;
}

static int ld_kind(int iop)
{
    switch (iop) {
    case ASM_OP_LD1_S: case ASM_OP_LD1_SA: case ASM_OP_LD2_S:
    case ASM_OP_LD2_SA: case ASM_OP_LD4_S: case ASM_OP_LD4_SA:
    case ASM_OP_LD8_S: case ASM_OP_LD8_SA:
        return LDK_SPEC;
    case ASM_OP_LD1_ACQ: case ASM_OP_LD2_ACQ: case ASM_OP_LD4_ACQ:
    case ASM_OP_LD8_ACQ: case ASM_OP_LD16_ACQ:
    case ASM_OP_LD1_C_CLR_ACQ: case ASM_OP_LD2_C_CLR_ACQ:
    case ASM_OP_LD4_C_CLR_ACQ: case ASM_OP_LD8_C_CLR_ACQ:
        return LDK_ACQ;
    case ASM_OP_LD8_FILL:
        return LDK_FILL;
    }
    return LDK_NORMAL;
}

/* Post-increment of the base register for M2/M3, M5, M7/M8 etc. */
static void gen_post_inc(DisasContext *dc, int r3, TCGv_i64 addr,
                         TCGv_i64 inc, TCGv_i32 inc_nat)
{
    int i = gr_idx(dc, r3);

    tcg_gen_add_i64(cpu_gr[i], addr, inc);
    if (inc_nat) {
        tcg_gen_or_i32(cpu_nat[i], cpu_nat[i], inc_nat);
    }
}

/*
 * Base update operand: the register for M2-style, the immediate for
 * M3-style, or none.  Returns false if the instruction faulted already.
 */
static bool get_post_inc(DisasContext *dc, bool reg, int r2, int64_t imm,
                         bool has, TCGv_i64 *inc, TCGv_i32 *inc_nat)
{
    *inc = NULL;
    *inc_nat = NULL;
    if (!has) {
        return true;
    }
    if (reg) {
        if (!check_gr(dc, r2)) {
            return false;
        }
        *inc = tcg_temp_new_i64();
        tcg_gen_mov_i64(*inc, load_gr(dc, r2));
        *inc_nat = load_nat(dc, r2);
    } else {
        *inc = tcg_constant_i64(imm);
    }
    return true;
}

static void trans_m_load(DisasContext *dc, const struct asm_inst *in)
{
    uint64_t b = in->i_bits;
    int r1 = R1(b), r2 = R2(b), r3 = R3(b);
    int size = ld_size(in->i_iop);
    int kind = ld_kind(in->i_iop);
    bool upd = in->i_format != ASM_FMT_M1;
    TCGv_i64 addr = tcg_temp_new_i64();
    TCGv_i64 inc;
    TCGv_i32 inc_nat;

    if (!size || !check_gr_dst(dc, r1) || !check_gr(dc, r3) ||
        (upd && r3 == 0)) {
        if (size && upd && r3 == 0) {
            gen_illegal(dc);
        }
        return;
    }
    if (upd && r1 == r3) {
        gen_illegal(dc);
        return;
    }
    if (!get_post_inc(dc, in->i_format == ASM_FMT_M2, r2,
                      in->i_format == ASM_FMT_M3 ? in->i_oper[3].o_value : 0,
                      upd, &inc, &inc_nat)) {
        return;
    }
    tcg_gen_mov_i64(addr, load_gr(dc, r3));

    if (kind == LDK_SPEC) {
        gen_save_ip(dc);
        gen_helper_ld_spec(tcg_env, tcg_constant_i32(gr_idx(dc, r1)), addr,
                           load_nat(dc, r3), tcg_constant_i32(size),
                           tcg_constant_i32(dc->mem_idx));
    } else {
        TCGv_i64 v = tcg_temp_new_i64();
        TCGv_i32 nat = NULL;

        gen_nat_check(dc, load_nat(dc, r3), ISR_R);
        if (size == 16) {
            TCGv_i128 t = tcg_temp_new_i128();

            tcg_gen_qemu_ld_i128(t, addr, dc->mem_idx,
                                 MO_128 | MO_ALIGN | dc->be |
                                 MO_ATOM_IFALIGN_PAIR);
            if (dc->be == MO_BE) {
                tcg_gen_extr_i128_i64(cpu_ar_csd, v, t);
            } else {
                tcg_gen_extr_i128_i64(v, cpu_ar_csd, t);
            }
        } else {
            tcg_gen_qemu_ld_i64(v, addr, dc->mem_idx, data_mop(dc, size));
        }
        if (kind == LDK_ACQ) {
            tcg_gen_mb(TCG_MO_ALL | TCG_BAR_LDAQ);
        }
        if (kind == LDK_FILL) {
            TCGv_i64 t = tcg_temp_new_i64();

            nat = tcg_temp_new_i32();
            tcg_gen_extract_i64(t, addr, 3, 6);
            tcg_gen_shr_i64(t, cpu_ar_unat, t);
            tcg_gen_andi_i64(t, t, 1);
            tcg_gen_extrl_i64_i32(nat, t);
        }
        store_gr(dc, r1, v, nat);
    }
    if (upd) {
        gen_post_inc(dc, r3, addr, inc, inc_nat);
    }
}

static void trans_m_store(DisasContext *dc, const struct asm_inst *in)
{
    uint64_t b = in->i_bits;
    int r2 = R2(b), r3 = R3(b);
    int iop = in->i_iop;
    int size;
    bool rel = false, spill = false;
    bool upd = in->i_format == ASM_FMT_M5;
    TCGv_i64 addr = tcg_temp_new_i64();

    switch (iop) {
    case ASM_OP_ST1_REL: rel = true; /* fall through */
    case ASM_OP_ST1_: size = 1; break;
    case ASM_OP_ST2_REL: rel = true; /* fall through */
    case ASM_OP_ST2_: size = 2; break;
    case ASM_OP_ST4_REL: rel = true; /* fall through */
    case ASM_OP_ST4_: size = 4; break;
    case ASM_OP_ST8_REL: rel = true; /* fall through */
    case ASM_OP_ST8_: size = 8; break;
    case ASM_OP_ST8_SPILL: size = 8; spill = true; break;
    case ASM_OP_ST16_REL: rel = true; /* fall through */
    case ASM_OP_ST16_: size = 16; break;
    default:
        gen_illegal(dc);
        return;
    }
    if (!check_gr(dc, r2) || !check_gr(dc, r3)) {
        return;
    }
    if (upd && r3 == 0) {
        gen_illegal(dc);
        return;
    }
    tcg_gen_mov_i64(addr, load_gr(dc, r3));
    gen_nat_check(dc, load_nat(dc, r3), ISR_W);
    if (!spill) {
        gen_nat_check(dc, load_nat(dc, r2), ISR_W);
    }
    if (rel) {
        tcg_gen_mb(TCG_MO_ALL | TCG_BAR_STRL);
    }
    if (size == 16) {
        TCGv_i128 t = tcg_temp_new_i128();

        if (dc->be == MO_BE) {
            tcg_gen_concat_i64_i128(t, cpu_ar_csd, load_gr(dc, r2));
        } else {
            tcg_gen_concat_i64_i128(t, load_gr(dc, r2), cpu_ar_csd);
        }
        tcg_gen_qemu_st_i128(t, addr, dc->mem_idx,
                             MO_128 | MO_ALIGN | dc->be |
                             MO_ATOM_IFALIGN_PAIR);
    } else {
        tcg_gen_qemu_st_i64(load_gr(dc, r2), addr, dc->mem_idx,
                            data_mop(dc, size));
    }
    if (spill) {
        TCGv_i64 sh = tcg_temp_new_i64();
        TCGv_i64 nat = tcg_temp_new_i64();

        tcg_gen_extract_i64(sh, addr, 3, 6);
        tcg_gen_extu_i32_i64(nat, load_nat(dc, r2));
        tcg_gen_shl_i64(nat, nat, sh);
        tcg_gen_shl_i64(sh, tcg_constant_i64(1), sh);
        tcg_gen_andc_i64(cpu_ar_unat, cpu_ar_unat, sh);
        tcg_gen_or_i64(cpu_ar_unat, cpu_ar_unat, nat);
    }
    if (upd) {
        gen_post_inc(dc, r3, addr, tcg_constant_i64(in->i_oper[3].o_value),
                     NULL);
    }
}

/* xchg, cmpxchg, cmp8xchg16, fetchadd */
static void trans_m_sem(DisasContext *dc, const struct asm_inst *in)
{
    uint64_t b = in->i_bits;
    int r1 = R1(b), r2 = R2(b), r3 = R3(b);
    int iop = in->i_iop;
    int size;
    TCGv_i64 addr = tcg_temp_new_i64();
    TCGv_i64 d = tcg_temp_new_i64();
    MemOp mop;

    switch (iop) {
    case ASM_OP_XCHG1: case ASM_OP_CMPXCHG1_ACQ: case ASM_OP_CMPXCHG1_REL:
        size = 1; break;
    case ASM_OP_XCHG2: case ASM_OP_CMPXCHG2_ACQ: case ASM_OP_CMPXCHG2_REL:
        size = 2; break;
    case ASM_OP_XCHG4: case ASM_OP_CMPXCHG4_ACQ: case ASM_OP_CMPXCHG4_REL:
    case ASM_OP_FETCHADD4_ACQ: case ASM_OP_FETCHADD4_REL:
        size = 4; break;
    default:
        size = 8; break;
    }
    if (!check_gr_dst(dc, r1) || !check_gr(dc, r3) ||
        (in->i_format == ASM_FMT_M16 && !check_gr(dc, r2))) {
        return;
    }
    tcg_gen_mov_i64(addr, load_gr(dc, r3));
    gen_nat_check(dc, load_nat(dc, r3), ISR_R | ISR_W);
    mop = data_mop(dc, size);

    switch (iop) {
    case ASM_OP_XCHG1: case ASM_OP_XCHG2: case ASM_OP_XCHG4: case ASM_OP_XCHG8:
        gen_nat_check(dc, load_nat(dc, r2), ISR_R | ISR_W);
        tcg_gen_atomic_xchg_i64(d, addr, load_gr(dc, r2), dc->mem_idx, mop);
        break;
    case ASM_OP_CMPXCHG1_ACQ ... ASM_OP_CMPXCHG8_REL: {
        TCGv_i64 cmp = tcg_temp_new_i64();

        gen_nat_check(dc, load_nat(dc, r2), ISR_R | ISR_W);
        tcg_gen_extract_i64(cmp, cpu_ar_ccv, 0, size * 8);
        tcg_gen_atomic_cmpxchg_i64(d, addr, cmp, load_gr(dc, r2),
                                   dc->mem_idx, mop);
        break;
    }
    case ASM_OP_CMP8XCHG16_ACQ:
    case ASM_OP_CMP8XCHG16_REL:
        gen_nat_check(dc, load_nat(dc, r2), ISR_R | ISR_W);
        gen_save_ip(dc);
        gen_helper_cmp8xchg16(d, tcg_env, addr, load_gr(dc, r2),
                              tcg_constant_i32(dc->mem_idx));
        break;
    case ASM_OP_FETCHADD4_ACQ ... ASM_OP_FETCHADD8_REL:
        tcg_gen_atomic_fetch_add_i64(d, addr,
                                     tcg_constant_i64(in->i_oper[3].o_value),
                                     dc->mem_idx, mop);
        break;
    default:
        gen_illegal(dc);
        return;
    }
    store_gr(dc, r1, d, NULL);
}

/* lfetch: only the base update is visible. */
static void trans_m_lfetch(DisasContext *dc, const struct asm_inst *in)
{
    uint64_t b = in->i_bits;
    int r2 = R2(b), r3 = R3(b);

    if (!check_gr(dc, r3)) {
        return;
    }
    if (in->i_format == ASM_FMT_M14) {
        if (!check_gr(dc, r2) || r3 == 0) {
            if (r3 == 0) {
                gen_illegal(dc);
            }
            return;
        }
        gen_post_inc(dc, r3, load_gr(dc, r3), load_gr(dc, r2),
                     load_nat(dc, r2));
    } else if (in->i_format == ASM_FMT_M15) {
        if (r3 == 0) {
            gen_illegal(dc);
            return;
        }
        gen_post_inc(dc, r3, load_gr(dc, r3),
                     tcg_constant_i64(in->i_oper[2].o_value), NULL);
    }
}
