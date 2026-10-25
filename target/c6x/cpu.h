/*
 * TI TMS320C64x+ CPU
 *
 * Written from the TMS320C64x/C64x+ DSP CPU and Instruction Set Reference
 * Guide (SPRU732J) and the Linux arch/c6x sources.
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#ifndef C6X_CPU_H
#define C6X_CPU_H

#include "cpu-qom.h"
#include "exec/cpu-common.h"
#include "exec/cpu-interrupt.h"
#include "exec/page-protection.h"
#include "exec/target_page.h"
#include "exec/cputlb.h"
#include "hw/core/registerfields.h"
#include "fpu/softfloat-types.h"

typedef struct CPUArchState CPUC6xState;

struct C6xCPUClass {
    CPUClass parent_class;
    DeviceRealize parent_realize;
    ResettablePhases parent_phases;
};

/* A0-A31 are gpr[0..31], B0-B31 are gpr[32..63]. */
#define REG_A(n)    (n)
#define REG_B(n)    (32 + (n))
#define NUM_GPR     64

/* Control registers, indexed by the crlo field of MVC. */
enum {
    CR_AMR = 0x00,
    CR_CSR = 0x01,
    CR_ISR_IFR = 0x02,      /* write: ISR, read: IFR */
    CR_ICR = 0x03,
    CR_IER = 0x04,
    CR_ISTP = 0x05,
    CR_IRP = 0x06,
    CR_NRP = 0x07,
    CR_TSCL = 0x0a,
    CR_TSCH = 0x0b,
    CR_ILC = 0x0d,
    CR_RILC = 0x0e,
    CR_REP = 0x0f,
    CR_PCE1 = 0x10,
    CR_DNUM = 0x11,
    CR_FADCR = 0x12,
    CR_FAUCR = 0x13,
    CR_FMCR = 0x14,
    CR_SSR = 0x15,
    CR_GPLYA = 0x16,
    CR_GPLYB = 0x17,
    CR_GFPGFR = 0x18,
    CR_DIER = 0x19,
    CR_TSR = 0x1a,
    CR_ITSR = 0x1b,
    CR_NTSR = 0x1c,
    CR_ECR_EFR = 0x1d,      /* write: ECR, read: EFR */
    CR_IERR = 0x1f,
};

FIELD(CSR, GIE, 0, 1)
FIELD(CSR, PGIE, 1, 1)
FIELD(CSR, SAT, 9, 1)
FIELD(CSR, EN, 8, 1)
FIELD(CSR, PWRD, 10, 6)

FIELD(TSR, GIE, 0, 1)
FIELD(TSR, SGIE, 1, 1)
FIELD(TSR, GEE, 2, 1)
FIELD(TSR, XEN, 3, 1)
FIELD(TSR, DBGM, 4, 1)
FIELD(TSR, CXM, 6, 2)
FIELD(TSR, INT, 9, 1)
FIELD(TSR, EXC, 10, 1)
FIELD(TSR, SPLX, 14, 1)
FIELD(TSR, IB, 15, 1)
/* NTSR only */
FIELD(TSR, HWE, 16, 1)

#define IER_NMIE        (1u << 1)
#define IFR_NMIF        (1u << 1)
#define INT_MASKABLE    0xfff0u

#define EFR_SXF         (1u << 0)
#define EFR_IXF         (1u << 1)
#define EFR_EXF         (1u << 30)
#define EFR_NXF         (1u << 31)

#define IERR_IFX        (1u << 0)
#define IERR_FPX        (1u << 1)
#define IERR_EPX        (1u << 2)
#define IERR_OPX        (1u << 3)
#define IERR_RCX        (1u << 4)
#define IERR_RAX        (1u << 5)
#define IERR_PRX        (1u << 6)
#define IERR_LBX        (1u << 7)

/* cs->exception_index values */
enum {
    EXCP_IRQ = 1,       /* maskable interrupt */
    EXCP_SWE,           /* SWE: software exception, returns to the next EP */
    EXCP_SWENR,         /* SWENR: software exception to REP */
    EXCP_INTERNAL,      /* internal exception, cause in IERR */
};

/*
 * Results are written a fixed number of cycles after their instruction
 * issued.  The translator keeps what is in flight inside a TB to itself;
 * what is still outstanding when the TB ends goes here.  Slot n holds the
 * writes that complete at the end of the n-th cycle from now.
 */
#define DQ_SLOTS        10
#define DQ_ENTRIES      24

/* Destinations other than general purpose registers */
enum {
    DQ_DEST_PC = NUM_GPR,       /* a branch lands */
    DQ_DEST_ILC,
    DQ_DEST_RILC,
};

typedef struct C6xDelayed {
    uint32_t val;
    uint8_t dest;
} C6xDelayed;

struct CPUArchState {
    uint32_t gpr[NUM_GPR];
    uint32_t pc;                /* address of the next execute packet */

    uint32_t amr;
    uint32_t csr;               /* GIE and PGIE live in tsr and itsr */
    uint32_t ier;
    uint32_t ifr;
    uint32_t istp;
    uint32_t irp;
    uint32_t nrp;
    uint32_t ilc;
    uint32_t rilc;
    uint32_t rep;
    uint32_t ssr;
    uint32_t gplya;
    uint32_t gplyb;
    uint32_t gfpgfr;
    uint32_t dier;
    uint32_t tsr;
    uint32_t itsr;
    uint32_t ntsr;
    uint32_t efr;
    uint32_t ierr;
    uint32_t dnum;

    /* Time stamp counter: cycles = (virtual ns * freq) - tsc_base */
    uint32_t cpu_freq;
    bool tsc_running;
    int64_t tsc_base;
    uint32_t tsch;

    /* Writes in flight across TB boundaries, see DQ_SLOTS. */
    uint8_t dq_n[DQ_SLOTS];
    uint8_t dq_live;            /* highest used slot + 1 */
    C6xDelayed dq[DQ_SLOTS][DQ_ENTRIES];

    /* SPLOOP state, see translate-sploop.c.inc */
    uint32_t spl_active;        /* bit s: the iteration in stage s runs */
    int32_t spl_m;              /* stage boundaries passed */
    uint32_t spl_t;             /* cycles since the loop started */
    uint32_t spl_P;             /* cycle at which post-loop code starts */
    uint32_t spl_term;
    uint32_t spl_drain;         /* the epilog continues after spl_P */

    /* LL/SL/CMTL */
    uint32_t ll_addr;
    uint32_t ll_valid;
    uint32_t sl_val;
    uint32_t sl_valid;

    /* C67x+ floating point configuration and status */
    float_status fp_status;
    uint32_t fadcr;
    uint32_t faucr;
    uint32_t fmcr;

    /*
     * Cycles since a branch was last in flight (issued or in its delay
     * slots); interrupts need five.
     */
    uint32_t clean_eps;

    /* Level of the INT4-INT15 inputs and of NMI */
    uint32_t irq_lines;

    /* Set together with exception_index */
    uint32_t excp_ierr;
};

struct ArchCPU {
    CPUState parent_obj;
    CPUC6xState env;
};

static inline bool c6x_user_mode(CPUC6xState *env)
{
    return FIELD_EX32(env->tsr, TSR, CXM) == 1;
}

void c6x_tcg_init(void);
void c6x_cpu_do_interrupt(CPUState *cs);
bool c6x_cpu_exec_interrupt(CPUState *cs, int interrupt_request);
void c6x_cpu_dump_state(CPUState *cpu, FILE *f, int flags);
void c6x_update_irq(CPUC6xState *env);
void c6x_dq_drain(CPUC6xState *env);
uint32_t c6x_csr_read(CPUC6xState *env);
uint64_t c6x_tsc(CPUC6xState *env);

#define CPU_RESOLVING_TYPE TYPE_C6X_CPU

#ifndef CONFIG_USER_ONLY
hwaddr c6x_cpu_get_phys_page_debug(CPUState *cpu, vaddr addr);
bool c6x_cpu_tlb_fill(CPUState *cs, vaddr address, int size,
                      MMUAccessType access_type, int mmu_idx,
                      bool probe, uintptr_t retaddr);
#endif

/*
 * TB flags.  DQ: the first DQ cycles of the TB must retire env->dq.
 * BR: a branch issued before the TB lands at the end of its BR-th cycle.
 * CLEAN: env->clean_eps, capped at 5.  CIRC: AMR is not zero.
 */
FIELD(TBFLAGS, USER, 0, 1)
FIELD(TBFLAGS, DQ, 1, 4)
FIELD(TBFLAGS, BR, 5, 3)
FIELD(TBFLAGS, CLEAN, 8, 3)
FIELD(TBFLAGS, CIRC, 11, 1)

void c6x_translate_code(CPUState *cs, TranslationBlock *tb,
                        int *max_insns, vaddr pc, void *host_pc);

#endif /* C6X_CPU_H */
