/*
 * Intel Itanium (IA-64) instruction translation
 *
 * Bundles are decoded with the FreeBSD ia64 disassembler (ia64-decode.c),
 * each slot is one translator instruction.  The translator follows the
 * instruction descriptions of the Intel Itanium Architecture SDM, vol. 3.
 *
 * Instruction groups are executed in program order.  For code without
 * dependency violations this is indistinguishable from the architected
 * "all reads before all writes" semantics.
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include "qemu/osdep.h"
#include "cpu.h"
#include "tcg/tcg-op.h"
#include "exec/helper-proto.h"
#include "exec/helper-gen.h"
#include "exec/log.h"
#include "exec/translator.h"
#include "exec/translation-block.h"
#include "qemu/qemu-print.h"
#include "ia64-disasm-int.h"
#include "ia64-disasm.h"

#define HELPER_H "helper.h"
#include "exec/helper-info.c.inc"
#undef  HELPER_H

/* Leave the TB, cpu_ip/cpu_ri are set by the code before. */
#define DISAS_EXIT      DISAS_TARGET_0
/* Continue at the next slot but return to the main loop first. */
#define DISAS_EXIT_NEXT DISAS_TARGET_1
/* Indirect jump: cpu_ip/cpu_ri hold the target, state is unchanged. */
#define DISAS_JUMP      DISAS_TARGET_2

typedef struct DisasContext {
    DisasContextBase base;
    uint64_t ip;            /* bundle address */
    int slot;
    struct asm_bundle bundle;
    uint64_t bundle_ip;     /* ip of the decoded bundle, or -1 */
    bool bundle_ok;
    bool mlx;
    const struct asm_inst *insn;
    uint64_t bits;

    uint64_t cfm;
    int sof, sol, sor, rrb_gr, rrb_fr, rrb_pr;
    int cpl;
    int mem_idx;
    uint32_t tbflags;
    MemOp be;

    TCGLabel *skip;         /* qp false */
} DisasContext;

static TCGv_i64 cpu_gr[128];
static TCGv_i32 cpu_nat[128];
static TCGv_i32 cpu_pr[64];
static TCGv_i64 cpu_br[8];
static TCGv_i64 cpu_ip;
static TCGv_i32 cpu_ri;
static TCGv_i64 cpu_psr;
static TCGv_i64 cpu_cfm;
static TCGv_i64 cpu_ar_lc, cpu_ar_ec, cpu_ar_pfs, cpu_ar_ccv, cpu_ar_unat;
static TCGv_i64 cpu_ar_csd, cpu_ar_ssd;

static inline uint64_t field(uint64_t bits, int pos, int len)
{
    return extract64(bits, pos, len);
}

/*
 * Register naming
 */

/* Index into env->gr for logical GR r, or -1 outside the frame. */
static int gr_idx(DisasContext *dc, int r)
{
    int i, rot;

    if (r < 32) {
        return r;
    }
    i = r - 32;
    if (i >= dc->sof) {
        return -1;
    }
    rot = dc->sor * 8;
    if (i < rot) {
        i = (i + dc->rrb_gr) % rot;
    }
    return 32 + i;
}

static int fr_idx(DisasContext *dc, int f)
{
    if (f < 32) {
        return f;
    }
    return 32 + (f - 32 + dc->rrb_fr) % 96;
}

static int pr_idx(DisasContext *dc, int p)
{
    if (p < 16) {
        return p;
    }
    return 16 + (p - 16 + dc->rrb_pr) % 48;
}

/*
 * Exceptions
 */

static void gen_save_ip(DisasContext *dc)
{
    tcg_gen_movi_i64(cpu_ip, dc->ip);
    tcg_gen_movi_i32(cpu_ri, dc->slot);
}

static void gen_fault(DisasContext *dc, int vec, uint64_t isr)
{
    gen_save_ip(dc);
    gen_helper_fault(tcg_env, tcg_constant_i32(vec), tcg_constant_i64(isr));
}

static void gen_illegal(DisasContext *dc)
{
    gen_fault(dc, IVT_GENERAL, GE_ILLEGAL_OP);
}

static void gen_priv_op(DisasContext *dc)
{
    gen_fault(dc, IVT_GENERAL, GE_PRIV_OP);
}

static bool check_priv(DisasContext *dc)
{
    if (dc->cpl != 0) {
        gen_priv_op(dc);
        return false;
    }
    return true;
}

/* NaT consumption fault when nat is set. */
static void gen_nat_check(DisasContext *dc, TCGv_i32 nat, uint64_t isr)
{
    TCGLabel *ok = gen_new_label();

    tcg_gen_brcondi_i32(TCG_COND_EQ, nat, 0, ok);
    gen_fault(dc, IVT_NAT_CONSUMPTION, isr);
    gen_set_label(ok);
}

/*
 * General registers
 */

static TCGv_i64 load_gr(DisasContext *dc, int r)
{
    int i = gr_idx(dc, r);

    if (r == 0) {
        return tcg_constant_i64(0);
    }
    if (i < 0) {
        /* Caller emits the fault through check_gr. */
        return tcg_constant_i64(0);
    }
    return cpu_gr[i];
}

static TCGv_i32 load_nat(DisasContext *dc, int r)
{
    int i = gr_idx(dc, r);

    if (r == 0 || i < 0) {
        return tcg_constant_i32(0);
    }
    return cpu_nat[i];
}

/* Out-of-frame register references raise Illegal Operation. */
static bool check_gr(DisasContext *dc, int r)
{
    if (gr_idx(dc, r) < 0) {
        gen_illegal(dc);
        return false;
    }
    return true;
}

static bool check_gr_dst(DisasContext *dc, int r)
{
    if (r == 0 || gr_idx(dc, r) < 0) {
        gen_illegal(dc);
        return false;
    }
    return true;
}

static void store_gr(DisasContext *dc, int r, TCGv_i64 v, TCGv_i32 nat)
{
    int i = gr_idx(dc, r);

    tcg_debug_assert(r != 0 && i >= 0);
    tcg_gen_mov_i64(cpu_gr[i], v);
    if (nat) {
        tcg_gen_mov_i32(cpu_nat[i], nat);
    } else {
        tcg_gen_movi_i32(cpu_nat[i], 0);
    }
}

static TCGv_i32 nat_or(TCGv_i32 a, TCGv_i32 b)
{
    TCGv_i32 t = tcg_temp_new_i32();

    tcg_gen_or_i32(t, a, b);
    return t;
}

/*
 * Predicates
 */

static void store_pr(DisasContext *dc, int p, TCGv_i32 v)
{
    if (p != 0) {
        tcg_gen_mov_i32(cpu_pr[pr_idx(dc, p)], v);
    }
}

static void store_pr_imm(DisasContext *dc, int p, int v)
{
    if (p != 0) {
        tcg_gen_movi_i32(cpu_pr[pr_idx(dc, p)], v);
    }
}

static TCGv_i32 load_pr(DisasContext *dc, int p)
{
    if (p == 0) {
        return tcg_constant_i32(1);
    }
    return cpu_pr[pr_idx(dc, p)];
}

/*
 * Compare result writing, shared by cmp, tbit, tnat, tf and fcmp.
 * ctype: 0 none, 1 unc, 2 and, 3 or, 4 or.andcm.
 * res is the relation, nat set means the sources were NaT.
 */
enum { CT_NONE, CT_UNC, CT_AND, CT_OR, CT_ORANDCM };

static void gen_write_cmp(DisasContext *dc, int ctype, int p1, int p2,
                          TCGv_i32 res, TCGv_i32 nat)
{
    TCGv_i32 t = tcg_temp_new_i32();
    TCGv_i32 f = tcg_temp_new_i32();
    TCGv_i32 notnat = tcg_temp_new_i32();

    if (!nat) {
        nat = tcg_constant_i32(0);
    }
    tcg_gen_xori_i32(notnat, nat, 1);
    switch (ctype) {
    case CT_NONE:
    case CT_UNC:
        /* With NaT sources both targets are cleared. */
        tcg_gen_and_i32(t, res, notnat);
        tcg_gen_xori_i32(f, res, 1);
        tcg_gen_and_i32(f, f, notnat);
        if (p1 == p2 && p1 != 0) {
            /* Undefined; write the true result. */
            store_pr(dc, p1, t);
            break;
        }
        store_pr(dc, p1, t);
        store_pr(dc, p2, f);
        break;
    case CT_AND:
        /* false or NaT clears both */
        tcg_gen_and_i32(t, res, notnat);
        if (p1) {
            tcg_gen_and_i32(cpu_pr[pr_idx(dc, p1)], cpu_pr[pr_idx(dc, p1)], t);
        }
        if (p2) {
            tcg_gen_and_i32(cpu_pr[pr_idx(dc, p2)], cpu_pr[pr_idx(dc, p2)], t);
        }
        break;
    case CT_OR:
        tcg_gen_and_i32(t, res, notnat);
        if (p1) {
            tcg_gen_or_i32(cpu_pr[pr_idx(dc, p1)], cpu_pr[pr_idx(dc, p1)], t);
        }
        if (p2) {
            tcg_gen_or_i32(cpu_pr[pr_idx(dc, p2)], cpu_pr[pr_idx(dc, p2)], t);
        }
        break;
    case CT_ORANDCM:
        tcg_gen_and_i32(t, res, notnat);
        if (p1) {
            tcg_gen_or_i32(cpu_pr[pr_idx(dc, p1)], cpu_pr[pr_idx(dc, p1)], t);
        }
        if (p2) {
            tcg_gen_xori_i32(f, t, 1);
            tcg_gen_and_i32(cpu_pr[pr_idx(dc, p2)], cpu_pr[pr_idx(dc, p2)], f);
        }
        break;
    }
}

/*
 * Control flow
 */

static uint64_t next_ip(DisasContext *dc, int *ri)
{
    if (dc->slot == 2 || (dc->mlx && dc->slot == 1)) {
        *ri = 0;
        return dc->ip + 16;
    }
    *ri = dc->slot + 1;
    return dc->ip;
}

static void gen_goto(DisasContext *dc, int n, uint64_t ip, int ri)
{
    if (translator_use_goto_tb(&dc->base, ip | ri)) {
        tcg_gen_goto_tb(n);
        tcg_gen_movi_i64(cpu_ip, ip);
        tcg_gen_movi_i32(cpu_ri, ri);
        tcg_gen_exit_tb(dc->base.tb, n);
    } else {
        tcg_gen_movi_i64(cpu_ip, ip);
        tcg_gen_movi_i32(cpu_ri, ri);
        tcg_gen_lookup_and_goto_ptr();
    }
}

static void gen_goto_next(DisasContext *dc, int n)
{
    int ri;
    uint64_t ip = next_ip(dc, &ri);

    gen_goto(dc, n, ip, ri);
}

/* Set ip/ri to the next instruction (used before leaving the TB). */
static void gen_set_next(DisasContext *dc)
{
    int ri;
    uint64_t ip = next_ip(dc, &ri);

    tcg_gen_movi_i64(cpu_ip, ip);
    tcg_gen_movi_i32(cpu_ri, ri);
}

/*
 * Conditional IP-relative branch: taken if cond != 0.  The frame marker
 * is unchanged, so direct chaining is fine.
 */
static void gen_branch_cond(DisasContext *dc, TCGv_i32 cond, uint64_t target)
{
    TCGLabel *nt = gen_new_label();

    tcg_gen_brcondi_i32(TCG_COND_EQ, cond, 0, nt);
    gen_goto(dc, 0, target, 0);
    gen_set_label(nt);
    gen_goto_next(dc, 1);
    dc->base.is_jmp = DISAS_NORETURN;
}

/*
 * Memory
 */

static MemOp size_mop(int bytes)
{
    switch (bytes) {
    case 1: return MO_8;
    case 2: return MO_16;
    case 4: return MO_32;
    case 8: return MO_64;
    }
    g_assert_not_reached();
}

static MemOp data_mop(DisasContext *dc, int bytes)
{
    MemOp mop = size_mop(bytes);

    if (bytes > 1) {
        mop |= dc->be | MO_ALIGN;
    }
    return mop;
}

static void trans_m_fp(DisasContext *dc, const struct asm_inst *in);
static void gen_chk_f(DisasContext *dc, int f2, int64_t disp);

#include "translate-int.c.inc"
#include "translate-mem.c.inc"
#include "translate-branch.c.inc"
#include "translate-sys.c.inc"
#include "translate-fp.c.inc"

/*
 * Dispatch
 */

static void translate_one(DisasContext *dc, const struct asm_inst *in)
{
    int qp = field(in->i_bits, 0, 6);
    bool unpredicated = false;

    dc->insn = in;
    dc->bits = in->i_bits;
    dc->skip = NULL;

    switch (in->i_iop) {
    /* These always execute or interpret qp themselves. */
    case ASM_OP_CMP_EQ_UNC: case ASM_OP_CMP_LT_UNC: case ASM_OP_CMP_LTU_UNC:
    case ASM_OP_CMP4_EQ_UNC: case ASM_OP_CMP4_LT_UNC: case ASM_OP_CMP4_LTU_UNC:
    case ASM_OP_TBIT_Z_UNC: case ASM_OP_TNAT_Z_UNC: case ASM_OP_TF_Z_UNC:
    /* Branches evaluate qp themselves. */
    case ASM_OP_BR_COND: case ASM_OP_BR_CALL: case ASM_OP_BR_RET:
    case ASM_OP_BR_IA:
    case ASM_OP_BR_CLOOP: case ASM_OP_BR_CTOP: case ASM_OP_BR_CEXIT:
    case ASM_OP_BR_WTOP: case ASM_OP_BR_WEXIT:
    case ASM_OP_BRL_COND: case ASM_OP_BRL_CALL:
        unpredicated = true;
        break;
    case ASM_OP_FRCPA: case ASM_OP_FPRCPA:
    case ASM_OP_FRSQRTA: case ASM_OP_FPRSQRTA:
        unpredicated = true;
        break;
    case ASM_OP_FCMP:
    case ASM_OP_FCLASS_M:
        /* fcmp.unc / fclass.unc clear the targets with qp false */
        unpredicated = field(in->i_bits, 12, 1);
        break;
    default:
        break;
    }

    if (qp != 0 && !unpredicated) {
        dc->skip = gen_new_label();
        tcg_gen_brcondi_i32(TCG_COND_EQ, cpu_pr[pr_idx(dc, qp)], 0, dc->skip);
    }

    switch (in->i_format & 0xff00) {
    case ASM_FMT_A:
        trans_a(dc, in);
        break;
    case ASM_FMT_I:
        trans_i(dc, in);
        break;
    case ASM_FMT_M:
        trans_m(dc, in);
        break;
    case ASM_FMT_B:
        trans_b(dc, in);
        break;
    case ASM_FMT_F:
        trans_f(dc, in);
        break;
    case ASM_FMT_X:
        trans_x(dc, in);
        break;
    default:
        gen_illegal(dc);
        break;
    }

    /* Close exits here, so that the qp-false path can continue. */
    switch (dc->base.is_jmp) {
    case DISAS_EXIT:
        tcg_gen_exit_tb(NULL, 0);
        dc->base.is_jmp = DISAS_NORETURN;
        break;
    case DISAS_EXIT_NEXT:
        gen_set_next(dc);
        tcg_gen_exit_tb(NULL, 0);
        dc->base.is_jmp = DISAS_NORETURN;
        break;
    case DISAS_JUMP:
        tcg_gen_lookup_and_goto_ptr();
        dc->base.is_jmp = DISAS_NORETURN;
        break;
    default:
        break;
    }

    if (dc->skip) {
        gen_set_label(dc->skip);
        /*
         * A conditional instruction that ends the TB falls through to the
         * next one when its predicate is false.
         */
        if (dc->base.is_jmp == DISAS_NORETURN) {
            gen_goto_next(dc, 1);
        }
    }
}

/*
 * Translator glue
 */

static void ia64_tr_init_disas_context(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *dc = container_of(dcbase, DisasContext, base);
    uint32_t flags = dc->base.tb->flags;
    uint64_t cfm = dc->base.tb->cs_base;

    dc->tbflags = flags;
    dc->cfm = cfm;
    dc->sof = FIELD_EX64(cfm, CFM, SOF);
    dc->sol = FIELD_EX64(cfm, CFM, SOL);
    dc->sor = FIELD_EX64(cfm, CFM, SOR);
    dc->rrb_gr = FIELD_EX64(cfm, CFM, RRB_GR);
    dc->rrb_fr = FIELD_EX64(cfm, CFM, RRB_FR);
    dc->rrb_pr = FIELD_EX64(cfm, CFM, RRB_PR);
    dc->cpl = FIELD_EX32(flags, TBFLAGS, CPL);
    dc->mem_idx = dc->cpl | (FIELD_EX32(flags, TBFLAGS, DT) ? MMU_IDX_VIRT : 0);
    dc->be = FIELD_EX32(flags, TBFLAGS, BE) ? MO_BE : MO_LE;
    dc->bundle_ip = -1;
}

static void ia64_tr_tb_start(DisasContextBase *db, CPUState *cs)
{
}

static void ia64_tr_insn_start(DisasContextBase *dcbase, CPUState *cs)
{
    tcg_gen_insn_start(dcbase->pc_next, 0, 0);
}

static void ia64_tr_translate_insn(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *dc = container_of(dcbase, DisasContext, base);
    CPUIA64State *env = cpu_env(cs);
    uint64_t pc = dc->base.pc_next;
    int ri;

    dc->ip = pc & ~0xfull;
    dc->slot = pc & 3;

    if (dc->bundle_ip != dc->ip) {
        uint8_t bytes[16];
        uint64_t lo = translator_ldq(env, &dc->base, dc->ip);
        uint64_t hi = translator_ldq(env, &dc->base, dc->ip + 8);

        stq_le_p(bytes, lo);
        stq_le_p(bytes + 8, hi);
        dc->bundle_ok = asm_decode((uint64_t)(uintptr_t)bytes, &dc->bundle);
        dc->bundle_ip = dc->ip;
        dc->mlx = dc->bundle_ok && dc->bundle.b_templ[1] == 'L';
    }

    if (!dc->bundle_ok || dc->slot == 3 || (dc->mlx && dc->slot == 2)) {
        gen_illegal(dc);
        dc->base.is_jmp = DISAS_NORETURN;
    } else {
        const struct asm_inst *in;

        in = &dc->bundle.b_inst[dc->mlx && dc->slot == 1 ? 2 : dc->slot];
        if (in->i_op == ASM_OP_NONE) {
            gen_illegal(dc);
            dc->base.is_jmp = DISAS_NORETURN;
        } else {
            translate_one(dc, in);
        }
    }

    next_ip(dc, &ri);
    dc->base.pc_next = (ri == 0) ? dc->ip + 16 : dc->ip + ri;
    /* Stay inside the page of the TB's first bundle. */
    if (dc->base.is_jmp == DISAS_NEXT && ri == 0 &&
        ((dc->ip + 16) & ~TARGET_PAGE_MASK) == 0) {
        dc->base.is_jmp = DISAS_TOO_MANY;
    }
}

static void ia64_tr_tb_stop(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *dc = container_of(dcbase, DisasContext, base);
    uint64_t pc = dc->base.pc_next;

    switch (dc->base.is_jmp) {
    case DISAS_NEXT:
    case DISAS_TOO_MANY:
        gen_goto(dc, 0, pc & ~0xfull, pc & 3);
        break;
    case DISAS_JUMP:
        tcg_gen_lookup_and_goto_ptr();
        break;
    case DISAS_EXIT_NEXT:
        tcg_gen_movi_i64(cpu_ip, pc & ~0xfull);
        tcg_gen_movi_i32(cpu_ri, pc & 3);
        tcg_gen_exit_tb(NULL, 0);
        break;
    case DISAS_EXIT:
        tcg_gen_exit_tb(NULL, 0);
        break;
    case DISAS_NORETURN:
        break;
    default:
        g_assert_not_reached();
    }
}

static const TranslatorOps ia64_tr_ops = {
    .init_disas_context = ia64_tr_init_disas_context,
    .tb_start           = ia64_tr_tb_start,
    .insn_start         = ia64_tr_insn_start,
    .translate_insn     = ia64_tr_translate_insn,
    .tb_stop            = ia64_tr_tb_stop,
};

void ia64_translate_code(CPUState *cs, TranslationBlock *tb,
                         int *max_insns, vaddr pc, void *host_pc)
{
    DisasContext dc;

    translator_loop(cs, tb, max_insns, pc, host_pc, &ia64_tr_ops, &dc.base,
                    TCG_TYPE_VA);
}

void ia64_cpu_dump_state(CPUState *cs, FILE *f, int flags)
{
    CPUIA64State *env = cpu_env(cs);
    int sof = FIELD_EX64(env->cfm, CFM, SOF);

    qemu_fprintf(f, "IP=%016" PRIx64 " ri=%d PSR=%016" PRIx64
                 " CFM=%010" PRIx64 " cpl=%d\n",
                 env->ip, env->ri, env->psr, env->cfm, ia64_cpl(env));
    qemu_fprintf(f, "IIP=%016" PRIx64 " IPSR=%016" PRIx64 " ISR=%016" PRIx64
                 " IFA=%016" PRIx64 " IFS=%016" PRIx64 "\n",
                 env->cr[CR_IIP], env->cr[CR_IPSR], env->cr[CR_ISR],
                 env->cr[CR_IFA], env->cr[CR_IFS]);
    qemu_fprintf(f, "BSP=%016" PRIx64 " BSPSTORE=%016" PRIx64
                 " ndirty=%d PFS=%016" PRIx64 " RSC=%" PRIx64 "\n",
                 ia64_bsp(env), env->ar[AR_BSPSTORE], env->rse_ndirty,
                 env->ar[AR_PFS], env->ar[AR_RSC]);
    for (int i = 0; i < 32 + sof; i++) {
        qemu_fprintf(f, "r%-3d=%016" PRIx64 "%c%c", i, env->gr[i],
                     env->nat[i] ? '*' : ' ', (i % 4) == 3 ? '\n' : ' ');
    }
    qemu_fprintf(f, "\n");
    for (int i = 0; i < 8; i++) {
        qemu_fprintf(f, "b%d=%016" PRIx64 "%c", i, env->br[i],
                     (i % 4) == 3 ? '\n' : ' ');
    }
    qemu_fprintf(f, "pr=");
    for (int i = 63; i >= 0; i--) {
        qemu_fprintf(f, "%d", env->pr[i]);
    }
    qemu_fprintf(f, "\n");
    if (flags & CPU_DUMP_FPU) {
        for (int i = 0; i < 128; i++) {
            qemu_fprintf(f, "f%-3d=%05" PRIx64 ":%016" PRIx64 "%c", i,
                         env->fr[i].se, env->fr[i].mant,
                         (i % 3) == 2 ? '\n' : ' ');
        }
        qemu_fprintf(f, "\n");
    }
}

void ia64_tcg_init(void)
{
    static char gr_names[128][8];
    static char nat_names[128][8];
    static char pr_names[64][8];
    static char br_names[8][4];

    for (int i = 0; i < 128; i++) {
        snprintf(gr_names[i], sizeof(gr_names[i]), "r%d", i);
        snprintf(nat_names[i], sizeof(nat_names[i]), "nat%d", i);
        cpu_gr[i] = tcg_global_mem_new_i64(tcg_env,
                                           offsetof(CPUIA64State, gr[i]),
                                           gr_names[i]);
        cpu_nat[i] = tcg_global_mem_new_i32(tcg_env,
                                            offsetof(CPUIA64State, nat[i]),
                                            nat_names[i]);
    }
    for (int i = 0; i < 64; i++) {
        snprintf(pr_names[i], sizeof(pr_names[i]), "p%d", i);
        cpu_pr[i] = tcg_global_mem_new_i32(tcg_env,
                                           offsetof(CPUIA64State, pr[i]),
                                           pr_names[i]);
    }
    for (int i = 0; i < 8; i++) {
        snprintf(br_names[i], sizeof(br_names[i]), "b%d", i);
        cpu_br[i] = tcg_global_mem_new_i64(tcg_env,
                                           offsetof(CPUIA64State, br[i]),
                                           br_names[i]);
    }
    cpu_ip = tcg_global_mem_new_i64(tcg_env, offsetof(CPUIA64State, ip), "ip");
    cpu_ri = tcg_global_mem_new_i32(tcg_env, offsetof(CPUIA64State, ri), "ri");
    cpu_psr = tcg_global_mem_new_i64(tcg_env, offsetof(CPUIA64State, psr),
                                     "psr");
    cpu_cfm = tcg_global_mem_new_i64(tcg_env, offsetof(CPUIA64State, cfm),
                                     "cfm");
#define AR_GLOBAL(var, n, name) \
    var = tcg_global_mem_new_i64(tcg_env, offsetof(CPUIA64State, ar[n]), name)
    AR_GLOBAL(cpu_ar_lc, AR_LC, "ar.lc");
    AR_GLOBAL(cpu_ar_ec, AR_EC, "ar.ec");
    AR_GLOBAL(cpu_ar_pfs, AR_PFS, "ar.pfs");
    AR_GLOBAL(cpu_ar_ccv, AR_CCV, "ar.ccv");
    AR_GLOBAL(cpu_ar_unat, AR_UNAT, "ar.unat");
    AR_GLOBAL(cpu_ar_csd, AR_CSD, "ar.csd");
    AR_GLOBAL(cpu_ar_ssd, AR_SSD, "ar.ssd");
#undef AR_GLOBAL
}
