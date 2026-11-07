/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"
#include "qemu.h"
#include "loader.h"
#include "target_elf.h"

/* arch/kvx/include/asm/elf.h: kv3-1 binaries carry 0x1308, kv3-2 0x2308 */
#define EF_KVX_CORE_MASK        0x0000f000
#define EF_KVX_CORE_KV3_2       0x00002000

const char *get_elf_cpu_model(uint32_t eflags)
{
    if ((eflags & EF_KVX_CORE_MASK) == EF_KVX_CORE_KV3_2) {
        return "kv3-2";
    }
    return "kv3-1";
}
