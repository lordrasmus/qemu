/*
 * Fujitsu FR-V CPU
 *
 * Written from the instruction set as binutils' CGEN description
 * (cpu/frv.cpu) describes it and from the Linux arch/frv sources.  The
 * modelled core is an FR401 (MB93401A), the integer and media core of the
 * FR400 series that Linux runs on without an MMU.
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#ifndef FRV_CPU_H
#define FRV_CPU_H

#include "cpu-qom.h"
#include "exec/cpu-common.h"
#include "exec/cpu-interrupt.h"
#include "exec/page-protection.h"
#include "exec/target_page.h"
#include "exec/cputlb.h"
#include "hw/core/registerfields.h"
#include "fpu/softfloat-types.h"

typedef struct CPUArchState CPUFRVState;

struct FRVCPUClass {
    CPUClass parent_class;
    DeviceRealize parent_realize;
    ResettablePhases parent_phases;
};

#define NUM_GR      64
#define NUM_FR      64
#define NUM_SPR     4096

/* Special purpose register numbers */
enum {
    SPR_PSR     = 0,
    SPR_PCSR    = 1,
    SPR_BPCSR   = 2,
    SPR_TBR     = 3,
    SPR_BPSR    = 4,
    SPR_HSR0    = 16,
    SPR_CCR     = 256,
    SPR_CCCR    = 263,
    SPR_LR      = 272,
    SPR_LCR     = 273,
    SPR_IACC0H  = 280,
    SPR_IACC0L  = 281,
    SPR_ISR     = 288,
    SPR_EPCR0   = 512,
    SPR_ESR0    = 576,
    SPR_ESR14   = 590,
    SPR_ESR15   = 591,
    SPR_ESFR0   = 672,
    SPR_ESFR1   = 673,
    SPR_SR0     = 768,
    SPR_MSR0    = 1280,
    SPR_MSR1    = 1281,
    SPR_ACC0    = 1408,
    SPR_ACCG0   = 1472,
    SPR_EAR0    = 1536,
    SPR_EAR15   = 1551,
    SPR_DCR     = 2048,
    SPR_BRR     = 2049,
};

FIELD(PSR, ET, 0, 1)
FIELD(PSR, PS, 1, 1)
FIELD(PSR, S, 2, 1)
FIELD(PSR, PIL, 3, 4)
FIELD(PSR, EM, 7, 1)
FIELD(PSR, EF, 8, 1)
FIELD(PSR, ESR, 9, 1)
FIELD(PSR, BE, 12, 1)
FIELD(PSR, CM, 13, 1)
FIELD(PSR, NEM, 14, 1)
FIELD(PSR, VER, 24, 4)
FIELD(PSR, IMPLE, 28, 4)

FIELD(TBR, TT, 4, 8)
FIELD(TBR, TBA, 12, 20)

#define BPSR_BET    0x00000001
#define BPSR_BS     0x00001000
#define BRR_SB      0x00000002

#define ISR_EMAM    0x00000001
#define ISR_AEXC    0x00000004
#define ISR_DTT_DIVBYZERO 0x00000008
#define ISR_DTT_OVERFLOW  0x00000010
#define ISR_EDE     0x00000020

#define ESR_VALID   0x00000001
#define ESR_EAV     0x00000800
#define EPCR_V      0x00000001
#define ESFR1_ESR0  0x00000001
#define ESFR1_ESR15 0x00008000

/* Trap types, the TT field of TBR */
enum {
    TT_PRIV_INSN = 0x06,
    TT_ILLEGAL_INSN = 0x07,
    TT_FP_DISABLED = 0x0a,
    TT_MP_DISABLED = 0x0b,
    TT_DATA_ACCESS_ERROR = 0x11,
    TT_DIVISION = 0x17,
    TT_INTERRUPT = 0x20,        /* + level 1..15 */
    TT_TRAP = 0x80,             /* + trap number 0..127 */
    TT_BREAK = 0xff,
};

/* cs->exception_index: the trap type, or one of these */
enum {
    EXCP_IRQ = 0x100,
};

struct CPUArchState {
    uint32_t gr[NUM_GR];
    uint32_t fr[NUM_FR];
    uint32_t pc;                /* address of the next VLIW packet */

    /* Condition codes, kept apart from CCR and CCCR */
    uint32_t icc[4];            /* N Z V C */
    uint32_t fcc[4];            /* E L G U */
    uint32_t cc[8];             /* 0/1 undefined, 2 false, 3 true */

    /* Special purpose registers other than CCR and CCCR */
    uint32_t spr[NUM_SPR];

    bool debug_mode;            /* in the break handler, rett #1 leaves */

    /* Set together with exception_index */
    uint32_t excp_next;         /* address of the packet after the trap */
    uint32_t excp_addr;         /* faulting data address */
    uint32_t excp_dtt;          /* division exception type */

    /* Interrupt level 1-15 requested by the interrupt controller */
    uint32_t irq_level;

    float_status fp_status;
};

struct ArchCPU {
    CPUState parent_obj;
    CPUFRVState env;
};

static inline bool frv_user_mode(CPUFRVState *env)
{
    return !FIELD_EX32(env->spr[SPR_PSR], PSR, S);
}

void frv_tcg_init(void);
void frv_cpu_do_interrupt(CPUState *cs);
bool frv_cpu_exec_interrupt(CPUState *cs, int interrupt_request);
void frv_cpu_dump_state(CPUState *cpu, FILE *f, int flags);
void frv_update_irq(CPUFRVState *env);
uint32_t frv_spr_read(CPUFRVState *env, int spr);
void frv_spr_write(CPUFRVState *env, int spr, uint32_t val);
G_NORETURN void frv_raise(CPUFRVState *env, int tt, uintptr_t ra);

#define CPU_RESOLVING_TYPE TYPE_FRV_CPU

#ifndef CONFIG_USER_ONLY
hwaddr frv_cpu_get_phys_page_debug(CPUState *cpu, vaddr addr);
bool frv_cpu_tlb_fill(CPUState *cs, vaddr address, int size,
                      MMUAccessType access_type, int mmu_idx,
                      bool probe, uintptr_t retaddr);
G_NORETURN void frv_cpu_do_unaligned_access(CPUState *cs, vaddr addr,
                                 MMUAccessType access_type, int mmu_idx,
                                 uintptr_t retaddr);
#endif

/* TB flags */
FIELD(TBFLAGS, USER, 0, 1)
FIELD(TBFLAGS, DEBUG, 1, 1)

void frv_translate_code(CPUState *cs, TranslationBlock *tb,
                        int *max_insns, vaddr pc, void *host_pc);

#endif /* FRV_CPU_H */
