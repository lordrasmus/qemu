/*
 * QEMU base architecture bit definitions
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef QEMU_BASE_ARCH_DEFS_H
#define QEMU_BASE_ARCH_DEFS_H

#include "qapi/qapi-types-machine.h"

/*
 * One bit per SysEmuTarget.  Macros rather than an enum: the mask is 64 bits
 * wide, and the values must stay constant expressions for the option tables.
 */
#define QEMU_ARCH_ALPHA       (1ULL << SYS_EMU_TARGET_ALPHA)
#define QEMU_ARCH_ARM         ((1ULL << SYS_EMU_TARGET_ARM) | \
                               (1ULL << SYS_EMU_TARGET_AARCH64))
#define QEMU_ARCH_I386        ((1ULL << SYS_EMU_TARGET_I386) | \
                               (1ULL << SYS_EMU_TARGET_X86_64))
#define QEMU_ARCH_M68K        (1ULL << SYS_EMU_TARGET_M68K)
#define QEMU_ARCH_MICROBLAZE  (1ULL << SYS_EMU_TARGET_MICROBLAZE)
#define QEMU_ARCH_MIPS        ((1ULL << SYS_EMU_TARGET_MIPS) | \
                               (1ULL << SYS_EMU_TARGET_MIPSEL) | \
                               (1ULL << SYS_EMU_TARGET_MIPS64) | \
                               (1ULL << SYS_EMU_TARGET_MIPS64EL))
#define QEMU_ARCH_PPC         ((1ULL << SYS_EMU_TARGET_PPC) | \
                               (1ULL << SYS_EMU_TARGET_PPC64))
#define QEMU_ARCH_S390X       (1ULL << SYS_EMU_TARGET_S390X)
#define QEMU_ARCH_SH4         ((1ULL << SYS_EMU_TARGET_SH4) | \
                               (1ULL << SYS_EMU_TARGET_SH4EB))
#define QEMU_ARCH_SPARC       ((1ULL << SYS_EMU_TARGET_SPARC) | \
                               (1ULL << SYS_EMU_TARGET_SPARC64))
#define QEMU_ARCH_XTENSA      ((1ULL << SYS_EMU_TARGET_XTENSA) | \
                               (1ULL << SYS_EMU_TARGET_XTENSAEB))
#define QEMU_ARCH_OR1K        (1ULL << SYS_EMU_TARGET_OR1K)
#define QEMU_ARCH_TRICORE     (1ULL << SYS_EMU_TARGET_TRICORE)
#define QEMU_ARCH_HPPA        (1ULL << SYS_EMU_TARGET_HPPA)
#define QEMU_ARCH_RISCV       ((1ULL << SYS_EMU_TARGET_RISCV32) | \
                               (1ULL << SYS_EMU_TARGET_RISCV64))
#define QEMU_ARCH_RX          (1ULL << SYS_EMU_TARGET_RX)
#define QEMU_ARCH_AVR         (1ULL << SYS_EMU_TARGET_AVR)
#define QEMU_ARCH_HEXAGON     (1ULL << SYS_EMU_TARGET_HEXAGON)
#define QEMU_ARCH_LOONGARCH   (1ULL << SYS_EMU_TARGET_LOONGARCH64)
#define QEMU_ARCH_H8300       (1ULL << SYS_EMU_TARGET_H8300)
#define QEMU_ARCH_ARC         (1ULL << SYS_EMU_TARGET_ARC)
#define QEMU_ARCH_AVR32       (1ULL << SYS_EMU_TARGET_AVR32)
#define QEMU_ARCH_BFIN        (1ULL << SYS_EMU_TARGET_BFIN)
#define QEMU_ARCH_KVX         (1ULL << SYS_EMU_TARGET_KVX)
#define QEMU_ARCH_NIOS2       (1ULL << SYS_EMU_TARGET_NIOS2)
#define QEMU_ARCH_NDS32       (1ULL << SYS_EMU_TARGET_NDS32)
#define QEMU_ARCH_C6X         (1ULL << SYS_EMU_TARGET_C6X)
#define QEMU_ARCH_CRIS        (1ULL << SYS_EMU_TARGET_CRIS)
#define QEMU_ARCH_CSKY        (1ULL << SYS_EMU_TARGET_CSKYV2)
#define QEMU_ARCH_TILEGX      (1ULL << SYS_EMU_TARGET_TILEGX)
#define QEMU_ARCH_LM32        (1ULL << SYS_EMU_TARGET_LM32)
#define QEMU_ARCH_IA64        (1ULL << SYS_EMU_TARGET_IA64)
#define QEMU_ARCH_META        (1ULL << SYS_EMU_TARGET_META)
#define QEMU_ARCH_ALL         UINT64_MAX

QEMU_BUILD_BUG_ON(SYS_EMU_TARGET__MAX > 64);

/**
 * qemu_arch_available:
 * @arch_bitmask: bitmask of QEMU_ARCH_* constants
 *
 * Return whether the current target architecture is contained in @arch_bitmask
 */
bool qemu_arch_available(uint64_t arch_bitmask);

#endif
