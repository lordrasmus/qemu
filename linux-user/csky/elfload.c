/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"
#include "qemu.h"
#include "loader.h"
#include "target_elf.h"

/* e_flags bits 0-27 hold the CPU (binutils include/elf/csky.h) */
#define EF_CSKY_ARCH_MASK   0x0fffffff
#define CSKY_ARCH_860       0xb

const char *get_elf_cpu_model(uint32_t eflags)
{
    if ((eflags & EF_CSKY_ARCH_MASK) == CSKY_ARCH_860) {
        return "ck860f";
    }
    return "ck810f";
}

void elf_core_copy_regs(target_elf_gregset_t *r, const CPUCSKYState *env)
{
    r->pt.tls = tswapal(env->regs[31]);
    r->pt.lr = tswapal(env->regs[15]);
    r->pt.pc = tswapal(env->pc);
    r->pt.sr = tswapal((env->cp0.psr & ~1) | env->psr_c);
    r->pt.usp = tswapal(env->regs[14]);
    r->pt.orig_a0 = tswapal(env->regs[0]);
    r->pt.a0 = tswapal(env->regs[0]);
    r->pt.a1 = tswapal(env->regs[1]);
    r->pt.a2 = tswapal(env->regs[2]);
    r->pt.a3 = tswapal(env->regs[3]);
    for (int i = 0; i < 10; i++) {
        r->pt.regs[i] = tswapal(env->regs[4 + i]);
    }
    for (int i = 0; i < 15; i++) {
        r->pt.exregs[i] = tswapal(env->regs[16 + i]);
    }
    r->pt.rhi = tswapal(env->hi);
    r->pt.rlo = tswapal(env->lo);
    r->pt.dcsr = tswapal(env->dcsr_v);
}
