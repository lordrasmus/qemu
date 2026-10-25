/*
 * Andes NDS32 (AndeStar V3, N13 class) CPU
 *
 * Written from the Linux arch/nds32 sources and the binutils opcode
 * tables; there is no public ISA manual for V3.
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#ifndef NDS32_CPU_H
#define NDS32_CPU_H

#include "cpu-qom.h"
#include "exec/cpu-common.h"
#include "exec/cpu-interrupt.h"
#include "exec/page-protection.h"
#include "exec/target_page.h"
#include "exec/cputlb.h"
#include "hw/core/registerfields.h"

typedef struct CPUArchState CPUNDS32State;

struct NDS32CPUClass {
    CPUClass parent_class;
    DeviceRealize parent_realize;
    ResettablePhases parent_phases;
};

/* General registers with a fixed role. */
#define R_TA    15
#define R_P0    26
#define R_P1    27
#define R_FP    28
#define R_GP    29
#define R_LP    30
#define R_SP    31

/* System register index: major << 7 | minor << 3 | ext (as in binutils). */
#define SRIDX(ma, mi, ex)   (((ma) << 7) | ((mi) << 3) | (ex))

#define SR_CPU_VER      SRIDX(0, 0, 0)
#define SR_CORE_ID      SRIDX(0, 0, 1)
#define SR_ICM_CFG      SRIDX(0, 1, 0)
#define SR_DCM_CFG      SRIDX(0, 2, 0)
#define SR_MMU_CFG      SRIDX(0, 3, 0)
#define SR_MSC_CFG      SRIDX(0, 4, 0)
#define SR_MSC_CFG2     SRIDX(0, 4, 1)
#define SR_FUCOP_EXIST  SRIDX(0, 5, 0)
#define SR_PSW          SRIDX(1, 0, 0)
#define SR_IPSW         SRIDX(1, 0, 1)
#define SR_P_IPSW       SRIDX(1, 0, 2)
#define SR_IVB          SRIDX(1, 1, 1)
#define SR_INT_CTRL     SRIDX(1, 1, 2)
#define SR_EVA          SRIDX(1, 2, 1)
#define SR_P_EVA        SRIDX(1, 2, 2)
#define SR_ITYPE        SRIDX(1, 3, 1)
#define SR_P_ITYPE      SRIDX(1, 3, 2)
#define SR_MERR         SRIDX(1, 4, 1)
#define SR_IPC          SRIDX(1, 5, 1)
#define SR_P_IPC        SRIDX(1, 5, 2)
#define SR_OIPC         SRIDX(1, 5, 3)
#define SR_P_P0         SRIDX(1, 6, 2)
#define SR_P_P1         SRIDX(1, 7, 2)
#define SR_INT_MASK     SRIDX(1, 8, 0)
#define SR_INT_MASK2    SRIDX(1, 8, 1)
#define SR_INT_PEND     SRIDX(1, 9, 0)
#define SR_INT_PEND2    SRIDX(1, 9, 1)
#define SR_INT_TRIGGER  SRIDX(1, 9, 4)
#define SR_SP_USR       SRIDX(1, 10, 0)
#define SR_SP_PRIV      SRIDX(1, 10, 1)
#define SR_INT_PRI      SRIDX(1, 11, 0)
#define SR_INT_PRI2     SRIDX(1, 11, 1)
#define SR_MMU_CTL      SRIDX(2, 0, 0)
#define SR_L1_PPTB      SRIDX(2, 1, 0)
#define SR_TLB_VPN      SRIDX(2, 2, 0)
#define SR_TLB_DATA     SRIDX(2, 3, 0)
#define SR_TLB_MISC     SRIDX(2, 4, 0)
#define SR_VLPT_IDX     SRIDX(2, 5, 0)
#define SR_ILMB         SRIDX(2, 6, 0)
#define SR_DLMB         SRIDX(2, 7, 0)
#define SR_CACHE_CTL    SRIDX(2, 8, 0)
#define SR_SDZ_CTL      SRIDX(2, 15, 0)
#define SR_MISC_CTL     SRIDX(2, 15, 1)
#define SR_EDM_CFG      SRIDX(3, 5, 0)
#define SR_EDM_CTL      SRIDX(3, 7, 0)
#define SR_FUCOP_CTL    SRIDX(4, 5, 0)

#define NUM_SR          1024

FIELD(PSW, GIE, 0, 1)
FIELD(PSW, INTL, 1, 2)
FIELD(PSW, POM, 3, 2)
FIELD(PSW, BE, 5, 1)
FIELD(PSW, IT, 6, 1)
FIELD(PSW, DT, 7, 1)
FIELD(PSW, IME, 8, 1)
FIELD(PSW, DME, 9, 1)
FIELD(PSW, DEX, 10, 1)
FIELD(PSW, CPL, 16, 3)

FIELD(ITYPE, ETYPE, 0, 4)
FIELD(ITYPE, INST, 4, 1)
FIELD(ITYPE, VECTOR, 5, 7)
FIELD(ITYPE, SWID, 16, 15)

FIELD(MMU_CTL, NTC0, 1, 2)
FIELD(MMU_CTL, MPZIU, 10, 1)
FIELD(MMU_CTL, NTM0, 11, 2)
FIELD(MMU_CTL, UNA, 23, 1)

FIELD(TLB_MISC, ACC_PSZ, 0, 4)
FIELD(TLB_MISC, CID, 4, 9)

/* PTE / TLB_DATA layout */
#define PTE_V       (1u << 0)
#define PTE_M_SHIFT 1
#define PTE_M_MASK  (7u << 1)
#define PTE_D       (1u << 4)
#define PTE_X       (1u << 5)
#define PTE_A       (1u << 6)
#define PTE_G       (1u << 7)

#define MISC_CTL_SP_SHADOW_EN   (1u << 4)
#define INT_MASK_IDIVZE         (1u << 30)

/* Interruption vectors */
enum {
    VEC_RESET = 0,
    VEC_TLB_FILL = 1,
    VEC_PTE_NOT_PRESENT = 2,
    VEC_TLB_MISC = 3,
    VEC_TLB_VLPT_MISS = 4,
    VEC_MACHINE_ERROR = 5,
    VEC_DEBUG = 6,
    VEC_GENERAL = 7,
    VEC_SYSCALL = 8,
    VEC_INTERRUPT = 9,
};

/* General exception types */
enum {
    ETYPE_ALIGN = 0,
    ETYPE_RESERVED_INSN = 1,
    ETYPE_TRAP = 2,
    ETYPE_ARITH = 3,
    ETYPE_PRECISE_BUS = 4,
    ETYPE_COPROC = 6,
};

/* TLB misc exception types */
enum {
    TLBMISC_RD_PROT = 0,
    TLBMISC_WRT_PROT = 1,
    TLBMISC_NOEXEC = 2,
    TLBMISC_PAGE_MODIFY = 3,
};

/*
 * cs->exception_index carries the vector in the low byte; the details
 * (ETYPE, SWID, INST) are put into env->pending_itype by whoever raises it.
 */
#define EXCP_NDS32(vec)   (vec)
#define EXCP_IRQ          EXCP_NDS32(VEC_INTERRUPT)

#define TLB_SETS    512
#define TLB_WAYS    8

typedef struct NDS32TLBEntry {
    uint32_t vpn;       /* VA >> 12 */
    uint32_t data;      /* PTE format */
    uint16_t cid;
    bool valid;
    bool locked;
} NDS32TLBEntry;

struct CPUArchState {
    uint32_t gpr[32];
    uint32_t pc;
    uint32_t d[4];      /* d0.lo, d0.hi, d1.lo, d1.hi */

    uint32_t sr[NUM_SR];

    /* Value latched by llw, for scw. */
    uint32_t lock_addr;
    uint32_t lock_val;

    /* Set together with exception_index. */
    uint32_t pending_itype;
    uint32_t pending_eva;
    bool pending_eva_valid;

    /* Level of the 32 IVIC inputs. */
    uint32_t irq_lines;

    NDS32TLBEntry tlb[TLB_SETS][TLB_WAYS];
    uint32_t tlb_rr;
};

struct ArchCPU {
    CPUState parent_obj;
    CPUNDS32State env;
};

#define MMU_IDX_USER    1
#define MMU_IDX_XLATE   2
#define NB_MMU_MODES_USED 4

static inline bool nds32_user_mode(CPUNDS32State *env)
{
    return FIELD_EX32(env->sr[SR_PSW], PSW, POM) == 0;
}

void nds32_tcg_init(void);
void nds32_cpu_do_interrupt(CPUState *cs);
void nds32_cpu_dump_state(CPUState *cpu, FILE *f, int flags);
G_NORETURN void nds32_raise(CPUNDS32State *env, int vec, uint32_t itype,
                            uint32_t eva, bool eva_valid, uintptr_t retaddr);
void nds32_update_irq(CPUNDS32State *env);
void nds32_tlb_flush_all(CPUNDS32State *env);

#define CPU_RESOLVING_TYPE TYPE_NDS32_CPU

#ifndef CONFIG_USER_ONLY
hwaddr nds32_cpu_get_phys_page_debug(CPUState *cpu, vaddr addr);
bool nds32_cpu_tlb_fill(CPUState *cs, vaddr address, int size,
                        MMUAccessType access_type, int mmu_idx,
                        bool probe, uintptr_t retaddr);
G_NORETURN void nds32_cpu_do_unaligned_access(CPUState *cpu, vaddr addr,
                                              MMUAccessType access_type,
                                              int mmu_idx, uintptr_t retaddr);
#endif

/* TB flags: the parts of PSW and MMU_CTL the translator depends on. */
FIELD(TBFLAGS, USER, 0, 1)
FIELD(TBFLAGS, IT, 1, 1)
FIELD(TBFLAGS, DT, 2, 1)
FIELD(TBFLAGS, UNA, 3, 1)
FIELD(TBFLAGS, BE, 4, 1)

void nds32_translate_code(CPUState *cs, TranslationBlock *tb,
                          int *max_insns, vaddr pc, void *host_pc);

#endif /* NDS32_CPU_H */
