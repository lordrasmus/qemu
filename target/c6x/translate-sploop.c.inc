/*
 * SPLOOP, included by translate.c
 *
 * The loop body is one scheduled iteration of dynlen cycles; iteration k
 * starts ii cycles after iteration k-1, and each runs the whole body.
 * Body cycle l belongs to stage l / ii.  The translation is a run time
 * loop over periods of ii cycles in which cycle j executes body cycles
 * s * ii + j for every stage s whose iteration is active; a bit mask in
 * env->spl_active says which are.  Iterations start at stage boundaries
 * while ILC allows (SPRU732J 7.9), helper_sploop_boundary decides.
 *
 * Results that cross a period boundary go through env->dq, like those
 * crossing TB boundaries.  The loop leaves when post-loop code may
 * start; if the epilog is not finished by then (SPKERNEL fstg/fcyc
 * shorter than the epilog), the rest of it is merged into the following
 * execute packets, which is what the hardware does too.
 *
 * Not supported: SPLOOPW, reloading (conditional SPLOOP), interrupting
 * a loop.  Interrupts are simply not taken until the loop is done.
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

static bool trans_sploop(DisasContext *dc, uint32_t insn, int op)
{
    switch (op) {
    case 0x8:       /* SPMASK */
    case 0x9:       /* SPMASKR */
        if (extract32(insn, 26, 6) || extract32(insn, 2, 11)) {
            return false;
        }
        dc->ep_spmask = extract32(insn, 18, 8);
        return true;
    case 0xa:       /* SPKERNEL outside a loop body */
    case 0xb:
        return true;
    case 0xc:       /* SPLOOP */
    case 0xd:       /* SPLOOPD */
    case 0xf:       /* SPLOOPW */
        if (extract32(insn, 18, 5) || extract32(insn, 2, 11)) {
            return false;
        }
        if (op == 0xf) {
            qemu_log_mask(LOG_UNIMP, "c6x: SPLOOPW at %08x\n", dc->ep);
            return false;
        }
        if (dc->cond) {
            qemu_log_mask(LOG_UNIMP, "c6x: reloading SPLOOP at %08x, "
                          "treated as unconditional\n", dc->ep);
        }
        dc->spl_kind = op;
        dc->spl.ii = extract32(insn, 23, 5) + 1;
        return true;
    }
    return false;
}

/* Functional unit of an instruction as an SPMASK bit, or -1 */
static int insn_unit(uint32_t insn)
{
    int side = F_S(insn);
    int unit;   /* 0 L, 1 S, 2 D, 3 M */

    switch (extract32(insn, 2, 2)) {
    case 1:
        unit = 2;
        side = extract32(insn, 7, 1);
        break;
    case 3:
        unit = 2;
        side = 1;
        break;
    case 2:
        if (insn & 0x10) {
            int op = extract32(insn, 5, 7);
            unit = (op >= 0x70 && op <= 0x77) ? 1 : 0;
        } else {
            unit = 1;
        }
        break;
    default:
        switch (extract32(insn, 4, 2)) {
        case 0:
            if (insn & 0x40) {
                unit = 2;
            } else if ((insn & 0x1ffe) == 0) {
                return -1;
            } else {
                unit = 3;
            }
            break;
        case 1:
        case 2:
            unit = 1;
            break;
        default:
            switch (extract32(insn, 10, 2)) {
            case 2:
                unit = 2;
                break;
            case 3:
                unit = 1;
                break;
            default:
                unit = 3;
                break;
            }
            break;
        }
        break;
    }
    return unit * 2 + side;
}

static bool is_nfu(uint32_t insn)
{
    return extract32(insn, 2, 4) == 0 && (insn & 0x1ffe) == 0;
}

/* Read the loop body following the SPLOOP execute packet */
static bool spl_read_body(DisasContext *dc, CPUC6xState *env)
{
    SplBody *b = &dc->spl;
    uint32_t addr = dc->next_ep;
    int l = 0;

    memset(b->n, 0, sizeof(b->n));
    for (;;) {
        uint32_t insns[MAX_EP];
        int n = 0, nops = 0;
        uint32_t mask = 0;
        bool kernel = false;

        for (;;) {
            if (header_fp(env, dc, addr) || n == MAX_EP) {
                return false;
            }
            insns[n] = ld_insn(env, dc, addr);
            addr += 4;
            if (!F_P(insns[n++])) {
                break;
            }
        }
        for (int i = 0; i < n; i++) {
            uint32_t insn = insns[i];

            if (!is_nfu(insn)) {
                continue;
            }
            if (!extract32(insn, 17, 1)) {
                if (F_CREG(insn) == 0 && F_Z(insn) == 1) {
                    return false;   /* SWE, DINT, ... */
                }
                nops = MAX(nops, extract32(insn, 13, 4));
                continue;
            }
            switch (extract32(insn, 13, 4)) {
            case 0x8:
            case 0x9:
                mask = extract32(insn, 18, 8);
                break;
            case 0xa:
                kernel = true;
                b->fstgfcyc = extract32(insn, 22, 6);
                break;
            default:
                return false;
            }
        }
        if (l >= SPL_MAX_CYCLES) {
            return false;
        }
        for (int i = 0; i < n; i++) {
            uint32_t insn = insns[i];
            int u;

            if (is_nfu(insn)) {
                continue;
            }
            u = insn_unit(insn);
            b->insn[l][b->n[l]] = insn;
            b->pce1[l][b->n[l]] = (addr - 4 * (n - i)) & ~31;
            b->masked[l][b->n[l]] = u >= 0 && (mask & (1 << u));
            b->n[l]++;
        }
        l += 1 + nops;
        if (kernel) {
            break;
        }
    }

    b->dynlen = l;
    b->post = addr;
    b->stages = DIV_ROUND_UP(l, b->ii);
    return b->stages <= 32 && b->dynlen > b->ii;
}

/* Delay given by SPKERNEL fstg/fcyc, see Table 3-24 */
static int spl_delay(SplBody *b)
{
    int cbits = b->ii == 1 ? 0 : b->ii == 2 ? 1 : b->ii <= 4 ? 2 :
                b->ii <= 8 ? 3 : 4;
    int sbits = 6 - cbits;
    int stage = 0;

    for (int i = 0; i < sbits; i++) {
        stage |= extract32(b->fstgfcyc, 5 - i, 1) << i;
    }
    return stage * b->ii + (cbits ? extract32(b->fstgfcyc, 0, cbits) : 0);
}

/*
 * Translate the instructions of body cycle l, guarded by guard (in
 * addition to their own predicates).  Instructions on units in spmask
 * are left out.
 */
static bool spl_group(DisasContext *dc, int l, TCGv_i32 guard_v,
                      TCGv_i32 guard_masked, uint32_t spmask)
{
    SplBody *b = &dc->spl;
    TCGv_i32 conds[MAX_EP];
    static const int8_t cregs[8] = {
        -1, REG_B(0), REG_B(1), REG_B(2), REG_A(1), REG_A(2), REG_A(0), -1
    };

    for (int i = 0; i < b->n[l]; i++) {
        uint32_t insn = b->insn[l][i];
        int creg = F_CREG(insn), z = F_Z(insn);
        TCGv_i32 g = b->masked[l][i] ? guard_masked : guard_v;

        conds[i] = tmp();
        if (cregs[creg] >= 0) {
            tcg_gen_setcondi_i32(z ? TCG_COND_EQ : TCG_COND_NE, conds[i],
                                 cpu_R[cregs[creg]], 0);
            tcg_gen_and_i32(conds[i], conds[i], g);
        } else {
            tcg_gen_mov_i32(conds[i], g);
        }
    }
    bool branched = dc->branched;
    bool ok = true;

    dc->branched = false;
    for (int i = 0; i < b->n[l] && ok; i++) {
        uint32_t insn = b->insn[l][i];
        int u = insn_unit(insn);

        if (u >= 0 && (spmask & (1 << u))) {
            continue;
        }
        if (F_CREG(insn) == 7) {
            ok = false;
            break;
        }
        dc->insn = insn;
        dc->pce1 = b->pce1[l][i];
        dc->cond = conds[i];
        dc->s = F_S(insn);
        ok = decode(dc, insn) && !dc->branched &&
             dc->base.is_jmp != DISAS_NORETURN;
    }
    dc->branched = branched;
    return ok;
}

static void spl_commit_due(DisasContext *dc, int c)
{
    for (int i = 0; i < dc->npend; i++) {
        Pend *p = &dc->pend[i];

        if (p->due == c) {
            gen_commit(p);
            drop_pend(dc, i--);
        }
    }
}

static void spl_dq_tick(void)
{
    TCGv_i32 live = tmp();
    TCGLabel *l = gen_new_label();

    tcg_gen_ld8u_i32(live, tcg_env, offsetof(CPUC6xState, dq_live));
    tcg_gen_brcondi_i32(TCG_COND_EQ, live, 0, l);
    gen_helper_dq_tick(tcg_env);
    gen_set_label(l);
}

static TCGv_i32 ld_env32(int off)
{
    TCGv_i32 t = tmp();

    tcg_gen_ld_i32(t, tcg_env, off);
    return t;
}

/* The loop itself, translated after the SPLOOP execute packet retired */
static bool gen_sploop(DisasContext *dc, CPUC6xState *env)
{
    SplBody *b = &dc->spl;
    TCGLabel *top, *merge;
    int base;

    if (!spl_read_body(dc, env)) {
        qemu_log_mask(LOG_UNIMP, "c6x: unsupported SPLOOP body at %08x\n",
                      dc->ep);
        return false;
    }
    b->delay = spl_delay(b);

    gen_flush(dc, dc->cycle - 1, NULL);
    dc->npend = 0;
    gen_helper_sploop_start(tcg_env);

    top = gen_new_label();
    merge = gen_new_label();
    gen_set_label(top);
    base = dc->cycle;

    for (int j = 0; j < b->ii; j++) {
        TCGv_i32 act, m, t, P;
        TCGLabel *cont;

        dc->cycle = base + j;
        if (j == 0) {
            gen_helper_sploop_boundary(tcg_env, cst(dc->spl_kind),
                                       cst(b->ii), cst(b->stages),
                                       cst(b->dynlen), cst(b->delay));
            /* post-loop code may start right at this boundary */
            t = ld_env32(offsetof(CPUC6xState, spl_t));
            P = ld_env32(offsetof(CPUC6xState, spl_P));
            tcg_gen_brcond_i32(TCG_COND_EQ, t, P, merge);
        }
        act = ld_env32(offsetof(CPUC6xState, spl_active));
        m = ld_env32(offsetof(CPUC6xState, spl_m));

        for (int s = 0; s < b->stages; s++) {
            int l = s * b->ii + j;
            TCGv_i32 g, gm;

            if (l >= b->dynlen || !b->n[l]) {
                continue;
            }
            g = tmp();
            gm = tmp();
            tcg_gen_extract_i32(g, act, s, 1);
            tcg_gen_setcondi_i32(TCG_COND_EQ, gm, m, s);
            if (!spl_group(dc, l, g, gm, 0)) {
                qemu_log_mask(LOG_UNIMP, "c6x: unsupported instruction in "
                              "SPLOOP body at %08x\n", dc->ep);
                return false;
            }
        }

        /* end of the cycle */
        spl_commit_due(dc, base + j);
        spl_dq_tick();
        t = ld_env32(offsetof(CPUC6xState, spl_t));
        tcg_gen_addi_i32(t, t, 1);
        tcg_gen_st_i32(t, tcg_env, offsetof(CPUC6xState, spl_t));
        P = ld_env32(offsetof(CPUC6xState, spl_P));
        cont = gen_new_label();
        tcg_gen_brcond_i32(TCG_COND_NE, t, P, cont);
        gen_flush(dc, base + j, NULL);
        tcg_gen_br(merge);
        gen_set_label(cont);
    }
    gen_flush(dc, base + b->ii - 1, NULL);
    dc->npend = 0;
    tcg_gen_br(top);

    gen_set_label(merge);
    dc->cycle = base + b->ii;
    dc->dq_cycles = dc->cycle + DQ_SLOTS;

    /* the rest of the epilog runs in parallel with what follows */
    dc->drain_d = b->delay;
    dc->drain = b->delay < b->dynlen - b->ii;
    dc->next_ep = b->post;
    return true;
}

/*
 * One cycle of the merged epilog: in normal termination iteration
 * N - 1 - i executes body cycle (i + 1) * ii + d.
 */
static bool spl_drain_cycle(DisasContext *dc, uint32_t spmask)
{
    SplBody *b = &dc->spl;
    TCGv_i32 g = ld_env32(offsetof(CPUC6xState, spl_drain));
    TCGv_i32 saved_cond = dc->cond;
    uint32_t saved_insn = dc->insn, saved_pce1 = dc->pce1;
    int saved_s = dc->s;
    bool ok = true;

    for (int i = 0; i + 1 < b->stages; i++) {
        int l = (i + 1) * b->ii + dc->drain_d;

        if (l < b->dynlen && b->n[l]) {
            ok = ok && spl_group(dc, l, g, cst(0), spmask);
        }
    }
    dc->insn = saved_insn;
    dc->pce1 = saved_pce1;
    dc->cond = saved_cond;
    dc->s = saved_s;

    dc->drain_d++;
    if (dc->drain_d >= b->dynlen - b->ii) {
        dc->drain = false;
    }
    return ok;
}
