/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"
#include "qemu.h"
#include "loader.h"
#include "target_elf.h"

/* binutils include/elf/arc.h */
#define EF_ARC_MACH_MSK         0x000000ff
#define E_ARC_MACH_ARC600       0x00000002
#define E_ARC_MACH_ARC700       0x00000003
#define E_ARC_MACH_ARC601       0x00000004
#define EF_ARC_CPU_ARCV2EM      0x00000005

const char *get_elf_cpu_model(uint32_t eflags)
{
    switch (eflags & EF_ARC_MACH_MSK) {
    case E_ARC_MACH_ARC600:
    case E_ARC_MACH_ARC601:
        return "arc600";
    case E_ARC_MACH_ARC700:
        return "arc700";
    case EF_ARC_CPU_ARCV2EM:
        return "arcem";
    default:
        return "archs";
    }
}
