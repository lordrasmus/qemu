/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"
#include "exec/page-protection.h"
#include "qemu.h"
#include "loader.h"
#include "target_elf.h"


const char *get_elf_cpu_model(uint32_t eflags)
{
    return "any";
}

/* Same order as ELF_CORE_COPY_REGS in arch/nios2/include/asm/elf.h */
void elf_core_copy_regs(target_elf_gregset_t *r, const CPUNios2State *env)
{
    int i;

    memset(r, 0, sizeof(*r));
    for (i = 0; i < 8; i++) {           /* r8-r15 */
        r->regs[i] = tswapal(env->regs[i + 8]);
    }
    for (i = 8; i < 15; i++) {          /* r1-r7 */
        r->regs[i] = tswapal(env->regs[i - 7]);
    }
    r->regs[15] = tswapal(-1);          /* orig_r2 */
    r->regs[16] = tswapal(env->regs[R_RA]);
    r->regs[17] = tswapal(env->regs[R_FP]);
    r->regs[18] = tswapal(env->regs[R_SP]);
    r->regs[19] = tswapal(env->regs[R_GP]);
    r->regs[21] = tswapal(env->pc);     /* ea */
    for (i = 23; i < 31; i++) {         /* r16-r23 */
        r->regs[i] = tswapal(env->regs[i - 7]);
    }
    r->regs[31] = tswapal(env->regs[R_FP]);
    r->regs[32] = tswapal(env->regs[R_GP]);
    r->regs[33] = tswapal(env->regs[R_RA]);
}

bool init_guest_commpage(void)
{
    static const uint8_t kuser_page[4 + 2 * 64] = {
        /* __kuser_helper_version */
        [0x00] = 0x02, 0x00, 0x00, 0x00,

        /* __kuser_cmpxchg */
        [0x04] = 0x3a, 0x6c, 0x3b, 0x00,  /* trap 16 */
                 0x3a, 0x28, 0x00, 0xf8,  /* ret */

        /* __kuser_sigtramp */
        [0x44] = 0xc4, 0x22, 0x80, 0x00,  /* movi r2, __NR_rt_sigreturn */
                 0x3a, 0x68, 0x3b, 0x00,  /* trap 0 */
    };

    int host_page_size = qemu_real_host_page_size();
    void *want, *addr;

    want = g2h_untagged(COMMPAGE & -host_page_size);
    addr = mmap(want, host_page_size, PROT_READ | PROT_WRITE,
                MAP_ANONYMOUS | MAP_PRIVATE |
                (reserved_va ? MAP_FIXED : MAP_FIXED_NOREPLACE),
                -1, 0);
    if (addr == MAP_FAILED) {
        perror("Allocating guest commpage");
        exit(EXIT_FAILURE);
    }
    if (addr != want) {
        return false;
    }

    memcpy(g2h_untagged(COMMPAGE), kuser_page, sizeof(kuser_page));

    if (mprotect(addr, host_page_size, PROT_READ)) {
        perror("Protecting guest commpage");
        exit(EXIT_FAILURE);
    }

    page_set_flags(COMMPAGE, COMMPAGE | ~TARGET_PAGE_MASK,
                   PAGE_READ | PAGE_EXEC | PAGE_VALID, PAGE_VALID);
    return true;
}
