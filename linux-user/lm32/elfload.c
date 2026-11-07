/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"
#include "qemu.h"
#include "loader.h"
#include "target_elf.h"

const char *get_elf_cpu_model(uint32_t eflags)
{
    /* Milkymist SoC: multiplier, divider, barrel shifter, sign extension */
    return "lm32-full";
}
