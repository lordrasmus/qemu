/*
 * TI TMS320C64x+ / C674x instruction translation
 *
 * One qemu "instruction" is one execute packet (EP).  All instructions of
 * an EP read their operands before any of them writes, and each result
 * appears a fixed number of cycles later.  The translator models that
 * exactly: results go into a queue of pending writes that is retired at
 * the end of each cycle.  Programs rely on it (reading the old value of
 * a register in the delay slots of a load that targets it), and so does
 * the Linux kernel's own entry code.
 *
 * Writes that are still in flight when a TB ends move to env->dq and are
 * retired by the next TB; the TB flags tell it for how many cycles.
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

#define HELPER_H "helper.h"
#include "exec/helper-info.c.inc"
#undef  HELPER_H

#define MAX_PEND    256
#define MAX_EP      8
#define SPL_MAX_CYCLES  48

/* A decoded SPLOOP body, see translate-sploop.c.inc */
typedef struct SplBody {
    int ii;
    int dynlen;
    int stages;
    int delay;
    uint32_t fstgfcyc;
    uint32_t post;          /* first EP after the SPKERNEL EP */
    uint8_t n[SPL_MAX_CYCLES];
    uint32_t insn[SPL_MAX_CYCLES][MAX_EP];
    uint32_t pce1[SPL_MAX_CYCLES][MAX_EP];
    bool masked[SPL_MAX_CYCLES][MAX_EP];
} SplBody;

typedef struct Pend {
    int due;            /* cycle at whose end the write happens */
    int dest;           /* gpr index or DQ_DEST_* */
    TCGv_i32 val;       /* NULL for a static branch target */
    TCGv_i32 cond;      /* NULL: unconditional */
    uint32_t target;    /* static branch target */
} Pend;

typedef struct DisasContext {
    DisasContextBase base;
    uint32_t ep;        /* address of the execute packet */
    uint32_t next_ep;
    int cycle;          /* cycle the current EP issues in, from TB start */
    bool user;
    bool circ;          /* AMR selects circular addressing somewhere */

    Pend pend[MAX_PEND];
    int npend;

    int dq_cycles;      /* cycles < dq_cycles retire env->dq */
    int dyn_br;         /* cycle at whose end env's branch lands, or -1 */
    int clean;          /* cycles without a branch in flight, max 5 */
    bool need_poll;
    int goto_tb_n;

    /* the instruction being translated */
    uint32_t insn;
    uint32_t pce1;      /* fetch packet address of this instruction */
    TCGv_i32 cond;
    int s;              /* unit side, 0 = A */

    /* the execute packet being translated */
    int nops;           /* extra cycles requested by multicycle NOPs */
    bool branched;
    bool swe;
    bool idle;
    bool exit_tb;       /* state the TB flags depend on changed */
    uint32_t ep_spmask; /* SPMASK units of this EP */
    int spl_kind;       /* SPLOOP(D) in this EP: its opfield */

    /* code read so far, from code_base up to code_end */
    uint32_t code_base;
    uint32_t code_end;
    uint32_t code[2 * TARGET_PAGE_SIZE / 4 + 32];

    /* SPLOOP */
    SplBody spl;
    bool drain;         /* epilog still running in parallel */
    int drain_d;        /* epilog cycle */
} DisasContext;

static TCGv_i32 cpu_R[NUM_GPR];
static TCGv_i32 cpu_pc;

static const char * const regnames[NUM_GPR] = {
    "a0", "a1", "a2", "a3", "a4", "a5", "a6", "a7",
    "a8", "a9", "a10", "a11", "a12", "a13", "a14", "a15",
    "a16", "a17", "a18", "a19", "a20", "a21", "a22", "a23",
    "a24", "a25", "a26", "a27", "a28", "a29", "a30", "a31",
    "b0", "b1", "b2", "b3", "b4", "b5", "b6", "b7",
    "b8", "b9", "b10", "b11", "b12", "b13", "b14", "b15",
    "b16", "b17", "b18", "b19", "b20", "b21", "b22", "b23",
    "b24", "b25", "b26", "b27", "b28", "b29", "b30", "b31",
};

/* Instruction fields */
#define F_P(i)      ((i) & 1)
#define F_S(i)      extract32(i, 1, 1)
#define F_DST(i)    extract32(i, 23, 5)
#define F_SRC2(i)   extract32(i, 18, 5)
#define F_SRC1(i)   extract32(i, 13, 5)
#define F_X(i)      extract32(i, 12, 1)
#define F_CREG(i)   extract32(i, 29, 3)
#define F_Z(i)      extract32(i, 28, 1)

/* Temporaries */

static TCGv_i32 tmp(void)
{
    return tcg_temp_new_i32();
}

static TCGv_i64 tmp64(void)
{
    return tcg_temp_new_i64();
}

/* The queue of pending writes */

static void queue(DisasContext *dc, int dest, TCGv_i32 val, int delay)
{
    Pend *p;

    /* at most ten cycles of results can be in flight */
    g_assert(dc->npend < MAX_PEND);
    p = &dc->pend[dc->npend++];
    p->due = dc->cycle + delay;
    p->dest = dest;
    p->val = val;
    p->cond = dc->cond;
    p->target = 0;
}

/* Register numbers: side 0 = A, 1 = B */
static inline int rnum(int side, int r)
{
    return side * 32 + r;
}

/* Write a result to a register of the unit's side */
static void wr(DisasContext *dc, int r, TCGv_i32 v, int delay)
{
    TCGv_i32 t = tmp();

    tcg_gen_mov_i32(t, v);
    queue(dc, rnum(dc->s, r), t, delay);
}

static void wr_side(DisasContext *dc, int side, int r, TCGv_i32 v, int delay)
{
    TCGv_i32 t = tmp();

    tcg_gen_mov_i32(t, v);
    queue(dc, rnum(side, r), t, delay);
}

/* A register pair: r is the even register, which takes the low word */
static void wr_pair(DisasContext *dc, int side, int r, TCGv_i32 lo,
                    TCGv_i32 hi, int dlo, int dhi)
{
    wr_side(dc, side, r & ~1, lo, dlo);
    wr_side(dc, side, r | 1, hi, dhi);
}

static void wr_pair64(DisasContext *dc, int side, int r, TCGv_i64 v,
                      int dlo, int dhi)
{
    TCGv_i32 lo = tmp(), hi = tmp();

    tcg_gen_extr_i64_i32(lo, hi, v);
    wr_pair(dc, side, r, lo, hi, dlo, dhi);
}

/* A 40-bit long: low 32 bits in the even register, 8 in the odd one */
static void wr_long(DisasContext *dc, int r, TCGv_i64 v, int delay)
{
    TCGv_i32 lo = tmp(), hi = tmp();

    tcg_gen_extr_i64_i32(lo, hi, v);
    tcg_gen_andi_i32(hi, hi, 0xff);
    wr_pair(dc, dc->s, r, lo, hi, delay, delay);
}

/*
 * The value a register will have at the start of cycle dc->cycle + k,
 * as far as the writes queued in this TB tell.  For instructions that
 * read parts of their operands in later pipeline stages.
 */
static TCGv_i32 rd_at(DisasContext *dc, int reg, int k)
{
    TCGv_i32 v = cpu_R[reg];

    for (int i = 0; i < dc->npend; i++) {
        Pend *p = &dc->pend[i];

        if (p->dest == reg && p->val && p->due < dc->cycle + k) {
            TCGv_i32 n = tmp();

            if (p->cond) {
                tcg_gen_movcond_i32(TCG_COND_NE, n, p->cond,
                                    tcg_constant_i32(0), p->val, v);
            } else {
                tcg_gen_mov_i32(n, p->val);
            }
            v = n;
        }
    }
    return v;
}

/* Operand reads */
static TCGv_i32 gr(DisasContext *dc, int r)
{
    return cpu_R[rnum(dc->s, r)];
}

static TCGv_i32 gx(DisasContext *dc, int r)
{
    return cpu_R[rnum(dc->s ^ F_X(dc->insn), r)];
}

/* Signed or unsigned 40-bit long from a register pair of the unit side */
static TCGv_i64 gl(DisasContext *dc, int r, bool sign)
{
    TCGv_i64 v = tmp64();

    tcg_gen_concat_i32_i64(v, gr(dc, r & ~1), gr(dc, r | 1));
    if (sign) {
        tcg_gen_sextract_i64(v, v, 0, 40);
    } else {
        tcg_gen_extract_i64(v, v, 0, 40);
    }
    return v;
}

/* 64-bit register pair, low word in the even register */
static TCGv_i64 gpair(DisasContext *dc, int side, int r, int khi)
{
    TCGv_i64 v = tmp64();

    tcg_gen_concat_i32_i64(v, cpu_R[rnum(side, r & ~1)],
                           rd_at(dc, rnum(side, r | 1), khi));
    return v;
}

static TCGv_i32 cst(int32_t v)
{
    return tcg_constant_i32(v);
}

/* Exceptions and control flow */

static void gen_flush(DisasContext *dc, int done, Pend *skip);

static void gen_exception(DisasContext *dc, int excp, uint32_t ierr)
{
    gen_flush(dc, dc->cycle - 1, NULL);
    tcg_gen_movi_i32(cpu_pc, dc->ep);
    gen_helper_raise(tcg_env, cst(excp), cst(ierr));
    dc->base.is_jmp = DISAS_NORETURN;
    /* a TB must not be empty */
    dc->base.pc_next = MAX(dc->base.pc_next, dc->ep + 4);
}

static void gen_illegal(DisasContext *dc)
{
    qemu_log_mask(LOG_GUEST_ERROR | LOG_UNIMP,
                  "c6x: illegal or unimplemented opcode %08x at %08x\n",
                  dc->insn, dc->ep);
    gen_exception(dc, EXCP_INTERNAL, IERR_OPX);
}

static void gen_goto(DisasContext *dc, uint32_t dest)
{
    if (dc->goto_tb_n < 2 && translator_use_goto_tb(&dc->base, dest)) {
        tcg_gen_goto_tb(dc->goto_tb_n);
        tcg_gen_movi_i32(cpu_pc, dest);
        tcg_gen_exit_tb(dc->base.tb, dc->goto_tb_n);
        dc->goto_tb_n++;
    } else {
        tcg_gen_movi_i32(cpu_pc, dest);
        tcg_gen_lookup_and_goto_ptr();
    }
}

static void gen_store_clean(int n)
{
    tcg_gen_st_i32(cst(n), tcg_env, offsetof(CPUC6xState, clean_eps));
}

/*
 * Move the writes still pending after the end of cycle "done" to env->dq,
 * for the TB that follows.
 */
static void gen_flush(DisasContext *dc, int done, Pend *skip)
{
    for (int i = 0; i < dc->npend; i++) {
        Pend *p = &dc->pend[i];
        TCGv_i32 v;

        if (p == skip || p->due <= done) {
            continue;
        }
        v = p->val ? p->val : cst(p->target);
        gen_helper_dq_push(tcg_env, cst(p->due - done - 1), cst(p->dest), v,
                           p->cond ? p->cond : cst(1));
    }
}

static void gen_commit(Pend *p)
{
    TCGv_i32 dst;

    switch (p->dest) {
    case DQ_DEST_ILC:
    case DQ_DEST_RILC:
        dst = tmp();
        tcg_gen_ld_i32(dst, tcg_env, p->dest == DQ_DEST_ILC ?
                       offsetof(CPUC6xState, ilc) :
                       offsetof(CPUC6xState, rilc));
        break;
    default:
        dst = cpu_R[p->dest];
        break;
    }
    if (p->cond) {
        tcg_gen_movcond_i32(TCG_COND_NE, dst, p->cond, cst(0), p->val, dst);
    } else {
        tcg_gen_mov_i32(dst, p->val);
    }
    if (p->dest == DQ_DEST_ILC || p->dest == DQ_DEST_RILC) {
        tcg_gen_st_i32(dst, tcg_env, p->dest == DQ_DEST_ILC ?
                       offsetof(CPUC6xState, ilc) :
                       offsetof(CPUC6xState, rilc));
    }
}

static void drop_pend(DisasContext *dc, int i)
{
    memmove(&dc->pend[i], &dc->pend[i + 1],
            (dc->npend - i - 1) * sizeof(Pend));
    dc->npend--;
}

/*
 * End of cycle c: retire what is due.  Returns false if the TB ended
 * because a branch landed unconditionally.
 */
static bool end_cycle(DisasContext *dc, int c)
{
    for (int i = 0; i < dc->npend; i++) {
        Pend *p = &dc->pend[i];

        if (p->due == c && p->dest != DQ_DEST_PC) {
            gen_commit(p);
            drop_pend(dc, i--);
        }
    }

    if (c < dc->dq_cycles) {
        gen_helper_dq_tick(tcg_env);
    }

    if (c == dc->dyn_br) {
        /* the tick put the target into env->pc */
        gen_flush(dc, c, NULL);
        gen_store_clean(0);
        tcg_gen_lookup_and_goto_ptr();
        dc->base.is_jmp = DISAS_NORETURN;
        return false;
    }

    for (int i = 0; i < dc->npend; i++) {
        Pend *p = &dc->pend[i];
        TCGLabel *skip = NULL;

        if (p->due != c || p->dest != DQ_DEST_PC) {
            continue;
        }
        if (p->cond) {
            skip = gen_new_label();
            tcg_gen_brcondi_i32(TCG_COND_EQ, p->cond, 0, skip);
        }
        gen_flush(dc, c, p);
        gen_store_clean(0);
        if (p->val) {
            tcg_gen_mov_i32(cpu_pc, p->val);
            tcg_gen_lookup_and_goto_ptr();
        } else {
            gen_goto(dc, p->target);
        }
        if (!skip) {
            dc->base.is_jmp = DISAS_NORETURN;
            return false;
        }
        gen_set_label(skip);
        drop_pend(dc, i--);
    }
    return true;
}

static bool branch_pending_at(DisasContext *dc, int c)
{
    if (dc->dyn_br >= c) {
        return true;
    }
    for (int i = 0; i < dc->npend; i++) {
        if (dc->pend[i].dest == DQ_DEST_PC && dc->pend[i].due >= c) {
            return true;
        }
    }
    return false;
}

static bool branch_pending(DisasContext *dc)
{
    if (dc->dyn_br >= dc->cycle) {
        return true;
    }
    for (int i = 0; i < dc->npend; i++) {
        if (dc->pend[i].dest == DQ_DEST_PC) {
            return true;
        }
    }
    return false;
}

static void gen_branch(DisasContext *dc, TCGv_i32 reg, uint32_t target)
{
    Pend *p;

    queue(dc, DQ_DEST_PC, NULL, 5);
    p = &dc->pend[dc->npend - 1];
    if (reg) {
        p->val = tmp();
        tcg_gen_mov_i32(p->val, reg);
    } else {
        p->target = target;
    }
    dc->branched = true;
}

/* Leave the TB at the next EP boundary, with everything in env */
static void gen_exit_tb(DisasContext *dc, int done, uint32_t next)
{
    gen_flush(dc, done, NULL);
    gen_store_clean(dc->clean);
    tcg_gen_movi_i32(cpu_pc, next);
}

/*
 * Instruction words.  The translator wants code read in ascending order
 * without gaps, but the header word of a fetch packet comes after the
 * instructions it describes; so read ahead in order and keep the words.
 */
static uint32_t ld_insn(CPUC6xState *env, DisasContext *dc, uint32_t addr)
{
    uint32_t i;

    g_assert(addr >= dc->code_base);
    while (dc->code_end <= addr) {
        i = (dc->code_end - dc->code_base) / 4;
        g_assert(i < ARRAY_SIZE(dc->code));
        dc->code[i] = translator_ldl(env, &dc->base, dc->code_end);
        dc->code_end += 4;
    }
    return dc->code[(addr - dc->code_base) / 4];
}

/* Is the fetch packet of addr header based? */
static bool header_fp(CPUC6xState *env, DisasContext *dc, uint32_t addr)
{
    uint32_t h = ld_insn(env, dc, (addr & ~31) + 28);

    return (h >> 28) == 0xe;
}

#include "translate-insns.c.inc"
#include "translate-fp.c.inc"
#include "translate-sploop.c.inc"

/* The execute packet */

static void gen_poll(DisasContext *dc)
{
    TCGv_i32 req = tmp();
    TCGLabel *no = gen_new_label();

    tcg_gen_ld_i32(req, tcg_env,
                   offsetof(C6xCPU, parent_obj.interrupt_request) -
                   offsetof(C6xCPU, env));
    tcg_gen_andi_i32(req, req, CPU_INTERRUPT_HARD);
    tcg_gen_brcondi_i32(TCG_COND_EQ, req, 0, no);
    gen_exit_tb(dc, dc->cycle - 1, dc->ep);
    tcg_gen_exit_tb(NULL, 0);
    gen_set_label(no);
}

static void c6x_tr_init_disas_context(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *dc = container_of(dcbase, DisasContext, base);
    uint32_t flags = dc->base.tb->flags;
    int br = FIELD_EX32(flags, TBFLAGS, BR);

    dc->user = FIELD_EX32(flags, TBFLAGS, USER);
    dc->circ = FIELD_EX32(flags, TBFLAGS, CIRC);
    dc->dq_cycles = FIELD_EX32(flags, TBFLAGS, DQ);
    dc->dyn_br = br ? br - 1 : -1;
    dc->clean = FIELD_EX32(flags, TBFLAGS, CLEAN);
    dc->cycle = 0;
    dc->npend = 0;
    dc->need_poll = true;
    dc->goto_tb_n = 0;
    dc->drain = false;
    dc->code_base = dc->base.pc_first & ~3u;
    dc->code_end = dc->code_base;
}

static void c6x_tr_tb_start(DisasContextBase *db, CPUState *cs)
{
}

static void c6x_tr_insn_start(DisasContextBase *dcbase, CPUState *cs)
{
    tcg_gen_insn_start(dcbase->pc_next, 0, 0);
#ifndef CONFIG_USER_ONLY
    /*
     * Let every execute packet do I/O.  qemu would otherwise restart the
     * TB at a packet that touches MMIO, and the results in flight from the
     * packets before it exist only in this TB's temporaries.  Without
     * icount nothing else depends on can_do_io.
     */
    tcg_gen_st8_i32(tcg_constant_i32(1), tcg_env,
                    offsetof(CPUState, neg.can_do_io) - sizeof(CPUState));
#endif
}

static void c6x_tr_translate_insn(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *dc = container_of(dcbase, DisasContext, base);
    CPUC6xState *env = cpu_env(cs);
    uint32_t insns[MAX_EP], pce1[MAX_EP];
    uint32_t addr = dc->base.pc_next;
    int n = 0, cycles;

    dc->ep = addr;

    /* gather the execute packet */
    for (;;) {
        if (header_fp(env, dc, addr)) {
            qemu_log_mask(LOG_UNIMP, "c6x: compact fetch packet at %08x\n",
                          addr & ~31);
            dc->insn = 0;
            gen_exception(dc, EXCP_INTERNAL, IERR_FPX);
            return;
        }
        insns[n] = ld_insn(env, dc, addr);
        pce1[n] = addr & ~31;
        addr += 4;
        if (!F_P(insns[n++])) {
            break;
        }
        if (n == MAX_EP) {
            dc->insn = insns[0];
            gen_exception(dc, EXCP_INTERNAL, IERR_EPX);
            return;
        }
    }
    dc->next_ep = addr;
    dc->base.pc_next = addr;

    if (dc->clean >= 5 && dc->need_poll && !branch_pending(dc)) {
        gen_poll(dc);
        dc->need_poll = false;
    }

    dc->nops = 0;
    dc->branched = false;
    dc->swe = false;
    dc->idle = false;
    dc->exit_tb = false;
    dc->ep_spmask = 0;
    dc->spl_kind = 0;

    /* every instruction reads its predicate before any writes */
    TCGv_i32 conds[MAX_EP];
    for (int i = 0; i < n; i++) {
        uint32_t insn = insns[i];
        int creg = F_CREG(insn), z = F_Z(insn);
        static const int8_t cregs[8] = {
            -1, REG_B(0), REG_B(1), REG_B(2), REG_A(1), REG_A(2), REG_A(0), -1
        };

        if (cregs[creg] < 0) {
            conds[i] = NULL;
        } else {
            conds[i] = tmp();
            tcg_gen_setcondi_i32(z ? TCG_COND_EQ : TCG_COND_NE, conds[i],
                                 cpu_R[cregs[creg]], 0);
        }
    }

    for (int i = 0; i < n; i++) {
        dc->insn = insns[i];
        dc->pce1 = pce1[i];
        dc->cond = conds[i];
        dc->s = F_S(insns[i]);
        if (F_CREG(insns[i]) == 7) {
            gen_illegal(dc);
            return;
        }
        if (!decode(dc, insns[i])) {
            gen_illegal(dc);
            return;
        }
        if (dc->base.is_jmp == DISAS_NORETURN) {
            return;
        }
    }

    /* retire the cycles of this EP */
    cycles = 1 + dc->nops;
    for (int c = 0; c < cycles; c++) {
        if (dc->drain) {
            dc->cycle += c;
            if (!spl_drain_cycle(dc, c ? 0 : dc->ep_spmask)) {
                dc->cycle -= c;
                gen_illegal(dc);
                return;
            }
            dc->cycle -= c;
        }
        /*
         * Interrupts wait until five cycles passed without a branch in
         * flight; the cycles of a multicycle NOP count singly.
         */
        if (branch_pending_at(dc, dc->cycle + c)) {
            dc->clean = 0;
        } else {
            dc->clean = MIN(dc->clean + 1, 5);
        }
        if (!end_cycle(dc, dc->cycle + c)) {
            return;
        }
    }
    dc->cycle += cycles;

    if (dc->spl_kind) {
        dc->branched = false;
        if (!gen_sploop(dc, env)) {
            gen_illegal(dc);
            return;
        }
        dc->clean = 0;
    }
    dc->base.pc_next = dc->next_ep;

    if (dc->swe) {
        gen_flush(dc, dc->cycle - 1, NULL);
        gen_store_clean(dc->clean);
        tcg_gen_movi_i32(cpu_pc, dc->next_ep);
        gen_helper_raise(tcg_env, cst(EXCP_SWE), cst(0));
        dc->base.is_jmp = DISAS_NORETURN;
        return;
    }
    if (dc->idle) {
        gen_exit_tb(dc, dc->cycle - 1, dc->next_ep);
        gen_helper_idle(tcg_env);
        dc->base.is_jmp = DISAS_NORETURN;
        return;
    }
    if (dc->exit_tb && !branch_pending(dc)) {
        dc->base.is_jmp = DISAS_TOO_MANY;
        dc->need_poll = true;
        return;
    }
    if (dc->exit_tb) {
        dc->need_poll = true;
    }

    if (dc->drain) {
        /* the epilog must finish inside this TB */
        dc->base.max_insns = MAX(dc->base.max_insns, dc->base.num_insns + 2);
        return;
    }

    /* stay within the TB's first page */
    if ((dc->next_ep ^ dc->base.pc_first) & TARGET_PAGE_MASK) {
        dc->base.is_jmp = DISAS_TOO_MANY;
    }
}

static void c6x_tr_tb_stop(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *dc = container_of(dcbase, DisasContext, base);

    switch (dc->base.is_jmp) {
    case DISAS_NEXT:
    case DISAS_TOO_MANY:
        gen_flush(dc, dc->cycle - 1, NULL);
        gen_store_clean(dc->clean);
        if (dc->exit_tb) {
            tcg_gen_movi_i32(cpu_pc, dc->base.pc_next);
            tcg_gen_exit_tb(NULL, 0);
        } else {
            gen_goto(dc, dc->base.pc_next);
        }
        break;
    case DISAS_NORETURN:
        break;
    default:
        g_assert_not_reached();
    }
}

static const TranslatorOps c6x_tr_ops = {
    .init_disas_context = c6x_tr_init_disas_context,
    .tb_start           = c6x_tr_tb_start,
    .insn_start         = c6x_tr_insn_start,
    .translate_insn     = c6x_tr_translate_insn,
    .tb_stop            = c6x_tr_tb_stop,
};

void c6x_translate_code(CPUState *cs, TranslationBlock *tb,
                        int *max_insns, vaddr pc, void *host_pc)
{
    DisasContext dc;

    translator_loop(cs, tb, max_insns, pc, host_pc, &c6x_tr_ops, &dc.base,
                    TCG_TYPE_VA);
}

void c6x_cpu_dump_state(CPUState *cs, FILE *f, int flags)
{
    CPUC6xState *env = cpu_env(cs);

    qemu_fprintf(f, "PC=%08x TSR=%08x CSR=%08x IER=%08x IFR=%08x "
                 "IRP=%08x NRP=%08x\n", env->pc, env->tsr, c6x_csr_read(env),
                 env->ier, env->ifr, env->irp, env->nrp);
    for (int i = 0; i < NUM_GPR; i++) {
        qemu_fprintf(f, "%4s=%08x%c", regnames[i], env->gpr[i],
                     (i % 8) == 7 ? '\n' : ' ');
    }
}

void c6x_tcg_init(void)
{
    for (int i = 0; i < NUM_GPR; i++) {
        cpu_R[i] = tcg_global_mem_new_i32(tcg_env,
                                          offsetof(CPUC6xState, gpr[i]),
                                          regnames[i]);
    }
    cpu_pc = tcg_global_mem_new_i32(tcg_env, offsetof(CPUC6xState, pc), "pc");
}
