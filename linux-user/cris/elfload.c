/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"
#include "qemu.h"
#include "loader.h"
#include "target_elf.h"

#define EF_CRIS_VARIANT_MASK    0x0000000e
#define EF_CRIS_VARIANT_V32     0x00000002

const char *get_elf_cpu_model(uint32_t eflags)
{
    if ((eflags & EF_CRIS_VARIANT_MASK) == EF_CRIS_VARIANT_V32) {
        return "crisv32";
    }
    return "crisv10";
}
