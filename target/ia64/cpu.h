/*
 * Intel Itanium (IA-64) CPU
 *
 * Written from the Intel Itanium Architecture Software Developer's
 * Manual, revision 2.3.  IA-32 execution is not supported.
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#ifndef IA64_CPU_H
#define IA64_CPU_H

#include "cpu-qom.h"
#include "exec/cpu-common.h"
#include "exec/cpu-interrupt.h"
#include "exec/page-protection.h"
#include "exec/target_page.h"
#include "exec/cputlb.h"
#include "hw/core/registerfields.h"
#include "fpu/softfloat-types.h"

typedef struct CPUArchState CPUIA64State;

struct IA64CPUClass {
    CPUClass parent_class;
    DeviceRealize parent_realize;
    ResettablePhases parent_phases;
};

/* Implementation parameters reported through PAL_VM_SUMMARY. */
#define IA64_IMPL_VA_MSB    50
#define IA64_IMPL_PA_MSB    49
#define IA64_RID_BITS       24
#define IA64_NUM_STACKED    96      /* physical stacked registers */

/* Application registers */
enum {
    AR_K0 = 0, AR_K7 = 7,
    AR_RSC = 16, AR_BSP = 17, AR_BSPSTORE = 18, AR_RNAT = 19,
    AR_FCR = 21, AR_EFLAG = 24, AR_CSD = 25, AR_SSD = 26, AR_CFLG = 27,
    AR_FSR = 28, AR_FIR = 29, AR_FDR = 30,
    AR_CCV = 32, AR_UNAT = 36, AR_FPSR = 40, AR_ITC = 44, AR_RUC = 45,
    AR_PFS = 64, AR_LC = 65, AR_EC = 66,
};

/* Control registers */
enum {
    CR_DCR = 0, CR_ITM = 1, CR_IVA = 2, CR_PTA = 8,
    CR_IPSR = 16, CR_ISR = 17, CR_IIP = 19, CR_IFA = 20, CR_ITIR = 21,
    CR_IIPA = 22, CR_IFS = 23, CR_IIM = 24, CR_IHA = 25,
    CR_IIB0 = 26, CR_IIB1 = 27,
    CR_LID = 64, CR_IVR = 65, CR_TPR = 66, CR_EOI = 67,
    CR_IRR0 = 68, CR_IRR3 = 71, CR_ITV = 72, CR_PMV = 73, CR_CMCV = 74,
    CR_LRR0 = 80, CR_LRR1 = 81,
};

/* Processor status register */
#define PSR_BE      (1ull << 1)
#define PSR_UP      (1ull << 2)
#define PSR_AC      (1ull << 3)
#define PSR_MFL     (1ull << 4)
#define PSR_MFH     (1ull << 5)
#define PSR_IC      (1ull << 13)
#define PSR_I       (1ull << 14)
#define PSR_PK      (1ull << 15)
#define PSR_DT      (1ull << 17)
#define PSR_DFL     (1ull << 18)
#define PSR_DFH     (1ull << 19)
#define PSR_SP      (1ull << 20)
#define PSR_PP      (1ull << 21)
#define PSR_DI      (1ull << 22)
#define PSR_SI      (1ull << 23)
#define PSR_DB      (1ull << 24)
#define PSR_LP      (1ull << 25)
#define PSR_TB      (1ull << 26)
#define PSR_RT      (1ull << 27)
#define PSR_CPL_SHIFT 32
#define PSR_CPL     (3ull << 32)
#define PSR_IS      (1ull << 34)
#define PSR_MC      (1ull << 35)
#define PSR_IT      (1ull << 36)
#define PSR_ID      (1ull << 37)
#define PSR_DA      (1ull << 38)
#define PSR_DD      (1ull << 39)
#define PSR_SS      (1ull << 40)
#define PSR_RI_SHIFT 41
#define PSR_RI      (3ull << 41)
#define PSR_ED      (1ull << 43)
#define PSR_BN      (1ull << 44)
#define PSR_IA      (1ull << 45)
#define PSR_VM      (1ull << 46)

/* Bits user mode may touch with sum/rum/mov psr.um. */
#define PSR_UM_MASK (PSR_BE | PSR_UP | PSR_AC | PSR_MFL | PSR_MFH)
/* Bits defined in the architecture. */
#define PSR_DEFINED (0x3fULL | (0x7fffULL << 13) | (0x3fffULL << 32))
#define PSR_DEFINED_MASK  (PSR_BE | PSR_UP | PSR_AC | PSR_MFL | PSR_MFH | \
                           (0x7fffull << 13) | (0x7fffull << 32))

/* Current frame marker */
FIELD(CFM, SOF, 0, 7)
FIELD(CFM, SOL, 7, 7)
FIELD(CFM, SOR, 14, 4)
FIELD(CFM, RRB_GR, 18, 7)
FIELD(CFM, RRB_FR, 25, 7)
FIELD(CFM, RRB_PR, 32, 6)
#define CFM_MASK    ((1ull << 38) - 1)

FIELD(PFS, PFM, 0, 38)
FIELD(PFS, PEC, 52, 6)
FIELD(PFS, PPL, 62, 2)

FIELD(RSC, MODE, 0, 2)
FIELD(RSC, PL, 2, 2)
FIELD(RSC, BE, 4, 1)
FIELD(RSC, LOADRS, 16, 14)

#define IFS_V       (1ull << 63)

/* Interruption status register */
FIELD(ISR, CODE, 0, 16)
FIELD(ISR, VECTOR, 16, 8)
#define ISR_X       (1ull << 32)
#define ISR_W       (1ull << 33)
#define ISR_R       (1ull << 34)
#define ISR_NA      (1ull << 35)
#define ISR_SP      (1ull << 36)
#define ISR_RS      (1ull << 37)
#define ISR_IR      (1ull << 38)
#define ISR_NI      (1ull << 39)
#define ISR_SO      (1ull << 40)
#define ISR_EI_SHIFT 41
#define ISR_ED      (1ull << 43)

/* Region register, PTA, ITIR, PTE */
FIELD(RR, VE, 0, 1)
FIELD(RR, PS, 2, 6)
FIELD(RR, RID, 8, 24)
FIELD(PTA, VE, 0, 1)
FIELD(PTA, SIZE, 2, 6)
FIELD(PTA, VF, 8, 1)
FIELD(ITIR, PS, 2, 6)
FIELD(ITIR, KEY, 8, 24)
#define PTE_P       (1ull << 0)
FIELD(PTE, MA, 2, 3)
#define PTE_A       (1ull << 5)
#define PTE_D       (1ull << 6)
FIELD(PTE, PL, 7, 2)
FIELD(PTE, AR, 9, 3)
FIELD(PTE, PPN, 12, 38)
#define PTE_ED      (1ull << 52)

FIELD(DCR, PP, 0, 1)
FIELD(DCR, BE, 1, 1)
FIELD(DCR, LC, 2, 1)

/* FPSR status field layout (13 bits, sf0..sf3 at 6 + 13 * n) */
#define FPSR_TRAPS_MASK 0x3full
#define SF_FTZ      (1u << 0)
#define SF_WRE      (1u << 1)
#define SF_PC_SHIFT 2
#define SF_RC_SHIFT 4
#define SF_TD       (1u << 6)
#define SF_FLAG_V   (1u << 7)
#define SF_FLAG_D   (1u << 8)
#define SF_FLAG_Z   (1u << 9)
#define SF_FLAG_O   (1u << 10)
#define SF_FLAG_U   (1u << 11)
#define SF_FLAG_I   (1u << 12)

/* Interruption vector offsets */
enum {
    IVT_VHPT = 0x0000,
    IVT_ITLB = 0x0400,
    IVT_DTLB = 0x0800,
    IVT_ALT_ITLB = 0x0c00,
    IVT_ALT_DTLB = 0x1000,
    IVT_NESTED_DTLB = 0x1400,
    IVT_IKEY_MISS = 0x1800,
    IVT_DKEY_MISS = 0x1c00,
    IVT_DIRTY = 0x2000,
    IVT_IACCESS_BIT = 0x2400,
    IVT_DACCESS_BIT = 0x2800,
    IVT_BREAK = 0x2c00,
    IVT_EXTINT = 0x3000,
    IVT_PAGE_NOT_PRESENT = 0x5000,
    IVT_KEY_PERMISSION = 0x5100,
    IVT_IACCESS_RIGHTS = 0x5200,
    IVT_DACCESS_RIGHTS = 0x5300,
    IVT_GENERAL = 0x5400,
    IVT_DISABLED_FP = 0x5500,
    IVT_NAT_CONSUMPTION = 0x5600,
    IVT_SPECULATION = 0x5700,
    IVT_DEBUG = 0x5900,
    IVT_UNALIGNED = 0x5a00,
    IVT_UNSUPPORTED_DATA = 0x5b00,
    IVT_FP_FAULT = 0x5c00,
    IVT_FP_TRAP = 0x5d00,
    IVT_LOWER_PRIV = 0x5e00,
    IVT_TAKEN_BRANCH = 0x5f00,
    IVT_SINGLE_STEP = 0x6000,
};

/* ISR.code values of the General Exception vector */
#define GE_ILLEGAL_OP       0x00
#define GE_PRIV_OP          0x10
#define GE_PRIV_REG         0x20
#define GE_RESERVED_REG     0x30
#define GE_DISABLED_ISA     0x40
#define GE_ILLEGAL_DEP      0x80

/*
 * cs->exception_index is the IVT offset; the ISR and the other
 * interruption resources are prepared in env->pend_* by the raiser.
 */
#define EXCP_IA64(vec)      (vec)
#define EXCP_FIRMWARE       0x10000     /* break into the board firmware */
#define EXCP_HALT           0x10001

/* Interruption resources written by an interruption. */
#define PEND_IFA    (1u << 0)
#define PEND_ITIR   (1u << 1)
#define PEND_IHA    (1u << 2)
#define PEND_IIM    (1u << 3)
#define PEND_TRAP   (1u << 4)   /* IIP/ri already point past the insn */

typedef struct IA64TLBEntry {
    uint64_t vpn;       /* va & ~(page size - 1), region bits included */
    uint64_t pte;       /* insertion format */
    uint64_t itir;
    uint32_t rid;
    uint8_t ps;
    bool valid;
} IA64TLBEntry;

#define IA64_NUM_TR     8
#define IA64_NUM_TC     256

/* Register-format floating point value: 64-bit significand, 17-bit exp. */
typedef struct IA64FReg {
    uint64_t mant;
    uint64_t se;        /* bit 17 sign, bits 0..16 biased exponent */
} IA64FReg;

#define FP_EXP_BIAS     0xffff
#define FP_EXP_INTEGER  0x1003e     /* exponent of setf.sig values */
#define FP_EXP_NATVAL   0x1fffe
#define FP_EXP_MAX      0x1ffff

struct CPUArchState {
    /*
     * r0..r127.  r16..r31 are the active bank; r32 and up are the
     * current register frame, unrotated (the translator applies rrb.gr).
     */
    uint64_t gr[128];
    uint32_t nat[128];
    uint32_t pr[64];
    uint64_t br[8];
    IA64FReg fr[128];

    uint64_t ip;        /* bundle address */
    uint32_t ri;        /* slot within the bundle (psr.ri) */
    uint64_t psr;       /* psr.ri is kept in ri, not here */
    uint64_t cfm;
    uint64_t ar[128];
    uint64_t cr[128];
    uint64_t rr[8];
    uint64_t pkr[16];
    uint64_t ibr[16];
    uint64_t dbr[16];

    /* The bank of r16..r31 not selected by psr.bn. */
    uint64_t bank[16];
    uint32_t bank_nat[16];

    /*
     * Register stack: the dirty partition, registers of the parent frames
     * not yet written to the backing store.  rse[0] is the oldest and goes
     * to ar.bspstore first.
     */
    uint64_t rse[IA64_NUM_STACKED];
    uint32_t rse_nat[IA64_NUM_STACKED];
    int32_t rse_ndirty;

    /* Translation registers and cache */
    IA64TLBEntry itr[IA64_NUM_TR];
    IA64TLBEntry dtr[IA64_NUM_TR];
    IA64TLBEntry itc[IA64_NUM_TC];
    IA64TLBEntry dtc[IA64_NUM_TC];
    uint32_t itc_next;
    uint32_t dtc_next;

    /* Local SAPIC */
    uint64_t irr[4];
    uint64_t insvc[4];  /* vectors in service */

    /* Interruption resources pending with exception_index */
    uint64_t pend_isr;
    uint64_t pend_ifa;
    uint64_t pend_itir;
    uint64_t pend_iha;
    uint64_t pend_iim;
    uint32_t pend_flags;

    /* Firmware call: break immediate */
    uint32_t fw_imm;

    float_status fp_status;

    /* ITC = (virtual ns * itc_freq / 1e9) + itc_offset */
    int64_t itc_offset;
    uint64_t itc_freq;
    struct QEMUTimer *itm_timer;

    /* Board hook for break immediates reserved for the firmware. */
    void (*firmware_call)(CPUIA64State *env, uint32_t imm);
};

struct ArchCPU {
    CPUState parent_obj;
    CPUIA64State env;
};

/*
 * mmu_idx: privilege level (0..3) plus 4 when translation is on.
 * Instruction and data translation are enabled separately.
 */
#define MMU_IDX_VIRT    4
#define NB_MMU_MODES_USED 8

static inline int ia64_cpl(CPUIA64State *env)
{
    return (env->psr >> PSR_CPL_SHIFT) & 3;
}

static inline int ia64_ri(CPUIA64State *env)
{
    return env->ri;
}

/* Index into the dirty/clean partitions is in 8-byte slots. */
static inline bool ia64_is_rnat_slot(uint64_t addr)
{
    return ((addr >> 3) & 0x3f) == 0x3f;
}

void ia64_tcg_init(void);
void ia64_cpu_do_interrupt(CPUState *cs);
void ia64_cpu_dump_state(CPUState *cpu, FILE *f, int flags);
G_NORETURN void ia64_raise(CPUIA64State *env, int vec, uint64_t isr,
                           uintptr_t retaddr);
G_NORETURN void ia64_raise_fault(CPUIA64State *env, int vec, uint64_t isr,
                                 uint32_t flags, uint64_t ifa, uintptr_t ra);
void ia64_update_irq(CPUIA64State *env);
void ia64_set_irq_vector(CPUIA64State *env, int vector, bool level);
uint64_t ia64_read_itc(CPUIA64State *env);
void ia64_write_itc(CPUIA64State *env, uint64_t val);
void ia64_rearm_itm(CPUIA64State *env);
void ia64_set_psr(CPUIA64State *env, uint64_t psr);
void ia64_switch_bank(CPUIA64State *env, bool bn);
uint64_t ia64_bsp(CPUIA64State *env);
void ia64_rse_frame_load(CPUIA64State *env, int n, uintptr_t ra);
void ia64_tlb_flush_all(CPUIA64State *env);
uint64_t ia64_thash(CPUIA64State *env, uint64_t va);
uint64_t ia64_ttag(CPUIA64State *env, uint64_t va);
void ia64_fpu_update_status(CPUIA64State *env);
void ia64_set_gr(CPUIA64State *env, int r, uint64_t val, bool nat);
uint64_t ia64_get_gr(CPUIA64State *env, int r, bool *nat);

/* Board side of the local SAPIC: a vector written to the interrupt block. */
void ia64_cpu_deliver_vector(ArchCPU *cpu, int vector);

#define CPU_RESOLVING_TYPE TYPE_IA64_CPU

#ifndef CONFIG_USER_ONLY
hwaddr ia64_cpu_get_phys_page_debug(CPUState *cpu, vaddr addr);
bool ia64_cpu_tlb_fill(CPUState *cs, vaddr address, int size,
                       MMUAccessType access_type, int mmu_idx,
                       bool probe, uintptr_t retaddr);
G_NORETURN void ia64_cpu_do_unaligned_access(CPUState *cpu, vaddr addr,
                                             MMUAccessType access_type,
                                             int mmu_idx, uintptr_t retaddr);
#endif

/*
 * TB flags: the PSR bits the translator depends on.  The frame marker
 * travels in cs_base, so register rotation is resolved at translation.
 */
FIELD(TBFLAGS, CPL, 0, 2)
FIELD(TBFLAGS, DT, 2, 1)
FIELD(TBFLAGS, BE, 3, 1)
FIELD(TBFLAGS, DFL, 4, 1)
FIELD(TBFLAGS, DFH, 5, 1)
FIELD(TBFLAGS, IC, 6, 1)
FIELD(TBFLAGS, SP, 7, 1)
FIELD(TBFLAGS, SS, 8, 1)
FIELD(TBFLAGS, TB, 9, 1)
FIELD(TBFLAGS, LP, 10, 1)
FIELD(TBFLAGS, SI, 11, 1)
FIELD(TBFLAGS, BN, 12, 1)

void ia64_translate_code(CPUState *cs, TranslationBlock *tb,
                         int *max_insns, vaddr pc, void *host_pc);

#endif /* IA64_CPU_H */
