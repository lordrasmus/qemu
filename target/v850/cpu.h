/*
 * NEC V850E1 CPU
 *
 * Written from the V850E1 Architecture User's Manual (U14559EJ).
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#ifndef V850_CPU_H
#define V850_CPU_H

#include "cpu-qom.h"
#include "exec/cpu-common.h"
#include "exec/cpu-interrupt.h"
#include "exec/page-protection.h"
#include "exec/target_page.h"
#include "exec/cputlb.h"

typedef struct CPUArchState CPUV850State;

struct V850CPUClass {
    CPUClass parent_class;
    DeviceRealize parent_realize;
    ResettablePhases parent_phases;
};

#define NUM_GPR     32
#define R_SP        3
#define R_EP        30

/* System registers, by LDSR/STSR regID */
enum {
    SR_EIPC = 0,
    SR_EIPSW = 1,
    SR_FEPC = 2,
    SR_FEPSW = 3,
    SR_ECR = 4,
    SR_PSW = 5,
    SR_CTPC = 16,
    SR_CTPSW = 17,
    SR_DBPC = 18,
    SR_DBPSW = 19,
    SR_CTBP = 20,
    NUM_SR = 32,
};

#define PSW_Z       (1u << 0)
#define PSW_S       (1u << 1)
#define PSW_OV      (1u << 2)
#define PSW_CY      (1u << 3)
#define PSW_SAT     (1u << 4)
#define PSW_ID      (1u << 5)
#define PSW_EP      (1u << 6)
#define PSW_NP      (1u << 7)
#define PSW_FLAGS   0x1fu
#define PSW_MASK    0xffu

/* cs->exception_index values */
enum {
    EXCP_IRQ = 1,       /* maskable interrupt */
    EXCP_TRAP,          /* TRAP insn, vector in excp_code */
    EXCP_ILGOP,         /* illegal opcode, to 0x60 */
    EXCP_DBTRAP,        /* DBTRAP insn, to 0x60 */
};

#define V850_NUM_IRQS   32

/* gdb-sim style "trap 31" system calls, hooked in by the board */
typedef void (*V850SimSyscall)(CPUV850State *env);

struct CPUArchState {
    uint32_t gpr[NUM_GPR];
    uint32_t pc;

    /*
     * PSW condition flags, kept apart: Z is set when zf == 0, S is
     * bit 31 of sf, OV bit 31 of ovf, CY and SAT are 0 or 1.
     */
    uint32_t zf;
    uint32_t sf;
    uint32_t ovf;
    uint32_t cyf;
    uint32_t sat;
    uint32_t psw;               /* the other PSW bits */

    uint32_t sr[NUM_SR];        /* system registers other than PSW */

    /* Interrupt requests latched on a rising edge, bit n = vector 0x80+n*16 */
    uint32_t irq_pending;
    uint32_t irq_lines;

    /* Set together with exception_index */
    uint32_t excp_code;

    V850SimSyscall sim_syscall;
};

struct ArchCPU {
    CPUState parent_obj;
    CPUV850State env;
};

static inline uint32_t v850_psw_read(CPUV850State *env)
{
    return env->psw
        | (env->zf == 0 ? PSW_Z : 0)
        | (env->sf >> 31 ? PSW_S : 0)
        | (env->ovf >> 31 ? PSW_OV : 0)
        | (env->cyf ? PSW_CY : 0)
        | (env->sat ? PSW_SAT : 0);
}

static inline void v850_psw_write(CPUV850State *env, uint32_t val)
{
    env->zf = !(val & PSW_Z);
    env->sf = val & PSW_S ? 0x80000000u : 0;
    env->ovf = val & PSW_OV ? 0x80000000u : 0;
    env->cyf = !!(val & PSW_CY);
    env->sat = !!(val & PSW_SAT);
    env->psw = val & PSW_MASK & ~PSW_FLAGS;
}

void v850_tcg_init(void);
void v850_cpu_do_interrupt(CPUState *cs);
bool v850_cpu_exec_interrupt(CPUState *cs, int interrupt_request);
void v850_cpu_dump_state(CPUState *cpu, FILE *f, int flags);
void v850_update_irq(CPUV850State *env);

#define CPU_RESOLVING_TYPE TYPE_V850_CPU

#ifndef CONFIG_USER_ONLY
hwaddr v850_cpu_get_phys_page_debug(CPUState *cpu, vaddr addr);
bool v850_cpu_tlb_fill(CPUState *cs, vaddr address, int size,
                       MMUAccessType access_type, int mmu_idx,
                       bool probe, uintptr_t retaddr);
#endif

void v850_translate_code(CPUState *cs, TranslationBlock *tb,
                         int *max_insns, vaddr pc, void *host_pc);

#endif /* V850_CPU_H */
