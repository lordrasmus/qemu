/*
 * Blackfin CPU
 *
 * Architecture state after the Blackfin Processor Programming Reference,
 * Rev. 2.2 (chapters 2 to 4).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef BFIN_CPU_H
#define BFIN_CPU_H

#include "cpu-qom.h"
#include "exec/cpu-common.h"
#include "exec/cpu-defs.h"
#include "exec/cpu-interrupt.h"
#include "qemu/cpu-float.h"

/*
 * Registers by their (group << 3 | number) encoding, as used by REGMV,
 * PUSH/POP and the allreg operand of the instruction set.
 */
enum {
    BFIN_R0 = 0,                    /* group 0: R0..R7 */
    BFIN_P0 = 8,                    /* group 1: P0..P5, SP, FP */
    BFIN_SP = 14,
    BFIN_FP = 15,
    BFIN_I0 = 16,                   /* group 2: I0..I3, M0..M3 */
    BFIN_M0 = 20,
    BFIN_B0 = 24,                   /* group 3: B0..B3, L0..L3 */
    BFIN_L0 = 28,
    BFIN_A0X = 32,                  /* group 4 */
    BFIN_A0W = 33,
    BFIN_A1X = 34,
    BFIN_A1W = 35,
    BFIN_ASTAT = 38,
    BFIN_RETS = 39,
    BFIN_LC0 = 48,                  /* group 6 */
    BFIN_LT0 = 49,
    BFIN_LB0 = 50,
    BFIN_LC1 = 51,
    BFIN_LT1 = 52,
    BFIN_LB1 = 53,
    BFIN_CYCLES = 54,
    BFIN_CYCLES2 = 55,
    BFIN_USP = 56,                  /* group 7 */
    BFIN_SEQSTAT = 57,
    BFIN_SYSCFG = 58,
    BFIN_RETI = 59,
    BFIN_RETX = 60,
    BFIN_RETN = 61,
    BFIN_RETE = 62,
    BFIN_EMUDAT = 63,
    BFIN_NREGS = 64,
};

/* ASTAT bit positions (def_LPBlackfin.h, Figure 2-9 of the PGR) */
enum {
    ASTAT_AZ = 0,
    ASTAT_AN = 1,
    ASTAT_AC0_COPY = 2,
    ASTAT_V_COPY = 3,
    ASTAT_CC = 5,
    ASTAT_AQ = 6,
    ASTAT_RND_MOD = 8,
    ASTAT_AC0 = 12,
    ASTAT_AC1 = 13,
    ASTAT_AV0 = 16,
    ASTAT_AV0S = 17,
    ASTAT_AV1 = 18,
    ASTAT_AV1S = 19,
    ASTAT_V = 24,
    ASTAT_VS = 25,
};

typedef struct CPUArchState {
    uint32_t reg[BFIN_NREGS];
    /*
     * one 0/1 word per ASTAT bit.  Every bit is stored, the reserved ones
     * too; AC0_COPY and V_COPY follow AC0 and V when an instruction
     * produces them, but ASTAT writes and CC moves set each bit alone,
     * as ADI's hardware test vectors (random_*, b0) show
     */
    uint32_t astat[32];
    uint32_t pc;

    /*
     * Core event controller state.  IPEND bit 4 (global interrupt
     * disable) is kept in ipend like the others.
     */
    uint32_t evt[16];
    uint32_t imask;
    uint32_t ipend;
    uint32_t ilat;

    uint32_t irq_level;             /* IVG lines held asserted by the SIC */
    uint32_t ksp;                   /* supervisor SP while in user mode */
    int irq_ivg;                    /* event being taken by do_interrupt */

    /* multi-issue: D-register results of the DSP slot, see dsp.c */
    uint32_t dsp_defer;
    uint32_t dsp_wb_mask;
    uint32_t dsp_wb[8];

    uint64_t cycles_base;           /* virtual ns at CYCLES = 0 */

    uint32_t cycles2_shadow;     /* CYCLES2 as of the last CYCLES read */
    uint32_t clock_freq;

    /* DCPLB_FAULT_ADDR and ICPLB_FAULT_ADDR, set by misaligned accesses */
    uint32_t dcplb_fault_addr;
    uint32_t icplb_fault_addr;
} CPUBfinState;

struct ArchCPU {
    CPUState parent_obj;

    CPUBfinState env;
};

struct BfinCPUClass {
    CPUClass parent_class;

    DeviceRealize parent_realize;
    ResettablePhases parent_phases;
};

#define BFIN_CPU_TYPE_SUFFIX "-" TYPE_BFIN_CPU
#define BFIN_CPU_TYPE_NAME(name) (name BFIN_CPU_TYPE_SUFFIX)
#define CPU_RESOLVING_TYPE TYPE_BFIN_CPU

/*
 * Supervisor while some event is being serviced.  IPEND[4] is the global
 * interrupt disable, not an event of its own.
 */
static inline bool bfin_supervisor(CPUBfinState *env)
{
    return (env->ipend & ~(1u << 4)) != 0;
}

#define EXCP_BFIN_EXCEPTION 1
#define EXCP_BFIN_IRQ       2

#define BFIN_TBFLAG_SUPER 1

#define MMU_USER_IDX 0
#define MMU_SUPER_IDX 1

uint32_t bfin_get_astat(CPUBfinState *env);
void bfin_set_astat(CPUBfinState *env, uint32_t val);

void bfin_update_irq(CPUBfinState *env);
void bfin_cpu_do_interrupt(CPUState *cs);
bool bfin_cpu_exec_interrupt(CPUState *cs, int interrupt_request);
bool bfin_cpu_tlb_fill(CPUState *cs, vaddr address, int size,
                       MMUAccessType access_type, int mmu_idx,
                       bool probe, uintptr_t retaddr);
G_NORETURN void bfin_cpu_do_unaligned_access(CPUState *cs, vaddr addr,
                                             MMUAccessType access_type,
                                             int mmu_idx, uintptr_t retaddr);
hwaddr bfin_cpu_get_phys_page_debug(CPUState *cs, vaddr addr);

void bfin_tcg_init(void);
void bfin_translate_code(CPUState *cs, TranslationBlock *tb,
                         int *max_insns, vaddr pc, void *host_pc);

#endif
