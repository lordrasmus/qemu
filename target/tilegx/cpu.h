/*
 *  TILE-Gx virtual CPU header
 *
 *  Copyright (c) 2015 Chen Gang
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#ifndef TILEGX_CPU_H
#define TILEGX_CPU_H

#include "cpu-qom.h"
#include "exec/cpu-common.h"
#include "exec/cpu-interrupt.h"
#include "exec/page-protection.h"
#include "exec/target_page.h"

/* TILE-Gx common register alias */
#define TILEGX_R_RE    0   /*  0 register, for function/syscall return value */
#define TILEGX_R_ERR   1   /*  1 register, for syscall errno flag */
#define TILEGX_R_NR    10  /* 10 register, for syscall number */
#define TILEGX_R_BP    52  /* 52 register, optional frame pointer */
#define TILEGX_R_TP    53  /* TP register, thread local storage data */
#define TILEGX_R_SP    54  /* SP register, stack pointer */
#define TILEGX_R_LR    55  /* LR register, may save pc, but it is not pc */
#define TILEGX_R_COUNT 56  /* Only 56 registers are really useful */
#define TILEGX_R_SN    56  /* SN register, obsoleted, it likes zero register */
#define TILEGX_R_IDN0  57  /* IDN0 register, cause IDN_ACCESS exception */
#define TILEGX_R_IDN1  58  /* IDN1 register, cause IDN_ACCESS exception */
#define TILEGX_R_UDN0  59  /* UDN0 register, cause UDN_ACCESS exception */
#define TILEGX_R_UDN1  60  /* UDN1 register, cause UDN_ACCESS exception */
#define TILEGX_R_UDN2  61  /* UDN2 register, cause UDN_ACCESS exception */
#define TILEGX_R_UDN3  62  /* UDN3 register, cause UDN_ACCESS exception */
#define TILEGX_R_ZERO  63  /* Zero register, always zero */
#define TILEGX_R_NOREG 255 /* Invalid register value */

/*
 * Interrupts and exceptions (the hardware numbering).  The vector of
 * interrupt N at protection level PL is INTERRUPT_VECTOR_BASE_PL + N * 256.
 */
enum {
    INT_MEM_ERROR = 0,
    INT_SINGLE_STEP_3 = 1,
    INT_SINGLE_STEP_2 = 2,
    INT_SINGLE_STEP_1 = 3,
    INT_SINGLE_STEP_0 = 4,
    INT_IDN_COMPLETE = 5,
    INT_UDN_COMPLETE = 6,
    INT_ITLB_MISS = 7,
    INT_ILL = 8,
    INT_GPV = 9,
    INT_IDN_ACCESS = 10,
    INT_UDN_ACCESS = 11,
    INT_SWINT_3 = 12,
    INT_SWINT_2 = 13,
    INT_SWINT_1 = 14,
    INT_SWINT_0 = 15,
    INT_ILL_TRANS = 16,
    INT_UNALIGN_DATA = 17,
    INT_DTLB_MISS = 18,
    INT_DTLB_ACCESS = 19,
    INT_IDN_FIREWALL = 20,
    INT_UDN_FIREWALL = 21,
    INT_TILE_TIMER = 22,
    INT_AUX_TILE_TIMER = 23,
    INT_IDN_TIMER = 24,
    INT_UDN_TIMER = 25,
    INT_IDN_AVAIL = 26,
    INT_UDN_AVAIL = 27,
    INT_IPI_3 = 28,
    INT_IPI_2 = 29,
    INT_IPI_1 = 30,
    INT_IPI_0 = 31,
    INT_PERF_COUNT = 32,
    INT_AUX_PERF_COUNT = 33,
    INT_INTCTRL_3 = 34,
    INT_INTCTRL_2 = 35,
    INT_INTCTRL_1 = 36,
    INT_INTCTRL_0 = 37,
    INT_BOOT_ACCESS = 38,
    INT_WORLD_ACCESS = 39,
    INT_I_ASID = 40,
    INT_D_ASID = 41,
    INT_DOUBLE_FAULT = 42,
    TILEGX_NUM_INTERRUPTS = 43,
};

/* Interrupts that the INTERRUPT_MASK and the critical section hold off. */
#define TILEGX_MASKABLE_INTERRUPTS \
    ((1ULL << INT_MEM_ERROR) | (0xfULL << INT_SINGLE_STEP_3) | \
     (1ULL << INT_IDN_COMPLETE) | (1ULL << INT_UDN_COMPLETE) | \
     (0xffffULL << INT_IDN_FIREWALL) | (0xfULL << INT_INTCTRL_3))

/* QEMU-internal exception numbers, above the hardware interrupts. */
enum {
    TILEGX_EXCP_HV_CALL = 0x100,    /* jump into the hypervisor glue page */
    TILEGX_EXCP_HALT,               /* hv_halt, hv_power_off */
};

/* MMU indexes: user, and the supervisor with or without a context. */
#define MMU_USER_IDX    0
#define MMU_KERNEL_IDX  1

/* Number of SPR protection domains (the SPR number's high byte). */
#define TILEGX_SPR_DOMAINS 0x28

typedef struct CPUArchState {
    uint64_t regs[TILEGX_R_COUNT];
    uint64_t pc;

    uint32_t cpl;                   /* current protection level */
    uint32_t ics;                   /* interrupt critical section */

    /* Per protection level state */
    uint64_t ex_context[4][2];      /* saved PC; saved PL and ICS */
    uint64_t system_save[4][4];
    uint64_t int_mask[4];           /* set bit: interrupt masked */
    uint64_t vector_base[4];
    uint64_t ipi_mask[4];
    uint64_t ipi_event[4];
    uint64_t single_step_control[4];

    uint8_t mpl[TILEGX_SPR_DOMAINS]; /* minimum protection levels */

    uint64_t cmpexch_value;
    uint64_t world_spr[256];        /* other SPRs of the world domain */
    uint64_t tile_timer_control;
    uint64_t aux_tile_timer_control;
    uint64_t timer_deadline;        /* cycle at which the count is zero */
    uint64_t pending;               /* raised queued interrupts */

    /* Faulting address of the last memory exception, for the handler */
    uint64_t fault_addr;
    uint64_t fault_info;

    /*
     * The client context the hypervisor runs: page table, access PTE,
     * ASID and flags from hv_install_context().  Until one is
     * installed, the PT_LOAD segments of the kernel image are mapped.
     */
    uint64_t ctx_page_table;
    uint64_t ctx_access;
    uint32_t ctx_asid;
    uint32_t ctx_flags;
    bool ctx_valid;
    uint32_t log2_page_small;
    uint32_t log2_page_large;
    uint32_t pte_super_shift[3];

    /* Fields up to this point are cleared by a CPU reset */
    struct {} end_reset_fields;

    /* Boot mappings: virtual start, physical start, length */
    uint64_t boot_map[4][3];
    int boot_map_count;
    uint64_t hv_glue_va;            /* VA of the hypervisor glue page */
    uint64_t cpu_speed;             /* cycles per second */
    int64_t cycle_base_ns;          /* clock origin for SPR_CYCLE */
    void *timer;                    /* QEMUTimer for the tile timer */
} CPUTLGState;

struct ArchCPU {
    CPUState parent_obj;

    CPUTLGState env;
};

struct TileGXCPUClass {
    CPUClass parent_class;

    DeviceRealize parent_realize;
    ResettablePhases parent_phases;
};

#define CPU_RESOLVING_TYPE TYPE_TILEGX_CPU

void tilegx_translate_init(void);
void tilegx_translate_code(CPUState *cs, TranslationBlock *tb,
                           int *max_insns, vaddr pc, void *host_pc);
void tilegx_cpu_dump_state(CPUState *cs, FILE *f, int flags);

#ifndef CONFIG_USER_ONLY
bool tilegx_cpu_tlb_fill(CPUState *cs, vaddr address, int size,
                         MMUAccessType access_type, int mmu_idx,
                         bool probe, uintptr_t retaddr);
hwaddr tilegx_cpu_get_phys_addr_debug(CPUState *cs, vaddr addr);
void tilegx_cpu_do_interrupt(CPUState *cs);
bool tilegx_cpu_exec_interrupt(CPUState *cs, int interrupt_request);
G_NORETURN void tilegx_raise_exception(CPUTLGState *env, int excp,
                                       uintptr_t retaddr);
G_NORETURN void tilegx_cpu_do_unaligned_access(CPUState *cs, vaddr addr,
                                               MMUAccessType access_type,
                                               int mmu_idx, uintptr_t retaddr);
void tilegx_update_irq(CPUTLGState *env);
void tilegx_timer_cb(void *opaque);
uint64_t tilegx_cycle(CPUTLGState *env);
void tilegx_hv_call(CPUTLGState *env, int index);
void tilegx_flush_tlb(CPUTLGState *env);
#endif

#endif
