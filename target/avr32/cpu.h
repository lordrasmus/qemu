/*
 * QEMU AVR32 CPU
 *
 * Copyright (c) 2022-2023 Florian Göhler, Johannes Willbold
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see
 * <http://www.gnu.org/licenses/lgpl-2.1.html>
 */
#ifndef QEMU_AVR32_CPU_H
#define QEMU_AVR32_CPU_H

#include "exec/cpu-common.h"
#include "exec/cpu-defs.h"
#include "hw/core/cpu.h"
#include "qom/object.h"

#define TYPE_AVR32_CPU "avr32-cpu"
#define AVR32_CPU_TYPE_SUFFIX "-" TYPE_AVR32_CPU
#define AVR32_CPU_TYPE_NAME(name) (name AVR32_CPU_TYPE_SUFFIX)
#define CPU_RESOLVING_TYPE TYPE_AVR32_CPU

OBJECT_DECLARE_CPU_TYPE(AVR32CPU, AVR32CPUClass, AVR32_CPU)

/* Register file: r0-r12, SP, LR, PC */
#define AVR32_NREGS     16
#define AVR32_SP_REG    13
#define AVR32_LR_REG    14
#define AVR32_PC_REG    15
#define AVR32_NSYSREGS  256

/* Status register bits */
#define SR_C    0
#define SR_Z    1
#define SR_N    2
#define SR_V    3
#define SR_Q    4
#define SR_L    5
#define SR_T    14
#define SR_R    15
#define SR_GM   16
#define SR_I0M  17
#define SR_I1M  18
#define SR_I2M  19
#define SR_I3M  20
#define SR_EM   21
#define SR_M0   22
#define SR_M1   23
#define SR_M2   24
#define SR_D    26
#define SR_DM   27
#define SR_J    28
#define SR_H    29
#define SR_SS   31

/* Execution modes, SR[M2:M0] */
enum {
    AVR32_MODE_APP = 0,
    AVR32_MODE_SUP = 1,
    AVR32_MODE_INT0 = 2,
    AVR32_MODE_INT1 = 3,
    AVR32_MODE_INT2 = 4,
    AVR32_MODE_INT3 = 5,
    AVR32_MODE_EX = 6,
    AVR32_MODE_NMI = 7,
};

/* System registers, by number (the address is the number times four) */
#define SYSREG_SR       0
#define SYSREG_EVBA     1
#define SYSREG_ACBA     2
#define SYSREG_CPUCR    3
#define SYSREG_ECR      4
#define SYSREG_RSR(m)   (4 + (m))       /* m = AVR32_MODE_SUP .. NMI */
#define SYSREG_RSR_DBG  12
#define SYSREG_RAR(m)   (12 + (m))
#define SYSREG_RAR_DBG  20
#define SYSREG_JECR     21
#define SYSREG_JOSP     22
#define SYSREG_CONFIG0  64
#define SYSREG_CONFIG1  65
#define SYSREG_COUNT    66
#define SYSREG_COMPARE  67
#define SYSREG_TLBEHI   68
#define SYSREG_TLBELO   69
#define SYSREG_PTBR     70
#define SYSREG_TLBEAR   71
#define SYSREG_MMUCR    72
#define SYSREG_TLBARLO  73
#define SYSREG_TLBARHI  74
#define SYSREG_BEAR     79

/* TLBEHI / TLBELO / MMUCR fields */
#define TLBEHI_V        (1u << 9)
#define TLBEHI_I        (1u << 8)
#define TLBEHI_ASID     0xffu
#define TLBELO_C        (1u << 9)
#define TLBELO_G        (1u << 8)
#define TLBELO_B        (1u << 7)
#define TLBELO_AP_SHIFT 4
#define TLBELO_SZ_SHIFT 2
#define TLBELO_D        (1u << 1)
#define TLBELO_W        (1u << 0)
#define MMUCR_E         (1u << 0)
#define MMUCR_M         (1u << 1)
#define MMUCR_I         (1u << 2)
#define MMUCR_N         (1u << 3)
#define MMUCR_S         (1u << 4)
#define MMUCR_DRP_SHIFT 14

#define AVR32_TLB_SIZE  32

/*
 * Exceptions, by their offset from EVBA. Interrupts use the autovector
 * from the interrupt controller instead.
 */
enum {
    EXCP_AVR32_UNRECOVERABLE = 0x00,
    EXCP_AVR32_TLB_MULTIPLE = 0x04,
    EXCP_AVR32_BUS_ERROR_DATA = 0x08,
    EXCP_AVR32_BUS_ERROR_INSN = 0x0c,
    EXCP_AVR32_NMI = 0x10,
    EXCP_AVR32_INSN_ADDR = 0x14,
    EXCP_AVR32_ITLB_PROT = 0x18,
    EXCP_AVR32_BREAKPOINT = 0x1c,
    EXCP_AVR32_ILLEGAL = 0x20,
    EXCP_AVR32_UNIMPLEMENTED = 0x24,
    EXCP_AVR32_PRIVILEGE = 0x28,
    EXCP_AVR32_FPE = 0x2c,
    EXCP_AVR32_COPROC = 0x30,
    EXCP_AVR32_DATA_ADDR_R = 0x34,
    EXCP_AVR32_DATA_ADDR_W = 0x38,
    EXCP_AVR32_DTLB_PROT_R = 0x3c,
    EXCP_AVR32_DTLB_PROT_W = 0x40,
    EXCP_AVR32_DTLB_MODIFIED = 0x44,
    EXCP_AVR32_ITLB_MISS = 0x50,
    EXCP_AVR32_DTLB_MISS_R = 0x60,
    EXCP_AVR32_DTLB_MISS_W = 0x70,
    EXCP_AVR32_SCALL = 0x100,
    EXCP_AVR32_IRQ = 0x200,             /* not an offset */
};

/* MMU indexes: application mode, privileged modes */
#define MMU_USER_IDX    0
#define MMU_KERNEL_IDX  1

typedef struct AVR32TLBEntry {
    uint32_t hi;                        /* TLBEHI format */
    uint32_t lo;                        /* TLBELO format */
} AVR32TLBEntry;

typedef struct CPUArchState {
    uint32_t r[AVR32_NREGS];            /* current view of the register file */
    uint32_t sflags[32];                /* SR, one bit per word */
    uint32_t sysr[AVR32_NSYSREGS];

    uint32_t sp_app;                    /* the stack pointer not in use */
    uint32_t sp_sys;
    uint32_t int3_bank[6];              /* r8-r12, lr of the other context */

    AVR32TLBEntry tlb[AVR32_TLB_SIZE];

    /* cycle counter: COUNT = (now - count_base) * freq */
    int64_t count_base;
    uint32_t count_freq;
    bool compare_pending;

    /* interrupt request from the interrupt controller */
    uint32_t irq_pending;               /* bit n: INTn requested */
    uint32_t irq_offset[4];             /* autovector per level */
    bool nmi_pending;

    /* not migrated */
    QEMUTimer *compare_timer;
    qemu_irq compare_irq;
} CPUAVR32State;

struct ArchCPU {
    CPUState parent_obj;

    CPUAVR32State env;
};

struct AVR32CPUClass {
    CPUClass parent_class;

    DeviceRealize parent_realize;
    ResettablePhases parent_phases;
};

static inline int avr32_mode(CPUAVR32State *env)
{
    return env->sflags[SR_M0] | env->sflags[SR_M1] << 1 |
           env->sflags[SR_M2] << 2;
}

uint32_t avr32_get_sr(CPUAVR32State *env);
void avr32_set_sr(CPUAVR32State *env, uint32_t sr);

/* TB flags */
#define AVR32_TBFLAG_PRIV   1

void avr32_tcg_init(void);
void avr32_translate_code(CPUState *cs, TranslationBlock *tb,
                          int *max_insns, vaddr pc, void *host_pc);
bool avr32_cpu_tlb_fill(CPUState *cs, vaddr address, int size,
                        MMUAccessType access_type, int mmu_idx,
                        bool probe, uintptr_t retaddr);
G_NORETURN void avr32_cpu_do_unaligned_access(CPUState *cs, vaddr addr,
                                              MMUAccessType access_type,
                                              int mmu_idx, uintptr_t retaddr);
void avr32_cpu_do_interrupt(CPUState *cs);
bool avr32_cpu_exec_interrupt(CPUState *cs, int interrupt_request);
hwaddr avr32_cpu_get_phys_page_debug(CPUState *cs, vaddr addr);
void avr32_cpu_set_irq(AVR32CPU *cpu, int level, bool raise, uint32_t offset);
void avr32_cpu_set_nmi(AVR32CPU *cpu, bool raise);
void avr32_compare_timer_cb(void *opaque);

int avr32_cpu_gdb_read_register(CPUState *cpu, GByteArray *buf, int reg);
int avr32_cpu_gdb_write_register(CPUState *cpu, uint8_t *buf, int reg);

#endif /* QEMU_AVR32_CPU_H */
