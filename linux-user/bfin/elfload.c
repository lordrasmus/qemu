/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"
#include "qemu.h"
#include "loader.h"
#include "target_elf.h"
#include "target_cpu.h"
#include "user-internals.h"
#include "exec/page-protection.h"

const char *get_elf_cpu_model(uint32_t eflags)
{
    return "bf537";
}

/*
 * Page zero carries the kernel's fixed code (arch/blackfin/kernel/
 * fixed_code.S).  The sigreturn stub is copied as is.  Everything else
 * is EXCPT 15: the atomic sequences are done by cpu_loop with host
 * atomics, any other address there faults like a NULL jump.
 */
bool init_guest_commpage(void)
{
    static const uint16_t stub[] = {
        0xe128, TARGET_NR_rt_sigreturn,     /* P0 = __NR_rt_sigreturn; */
        0x00a0,                             /* EXCPT 0; */
        0x2000,                             /* JUMP.S 0; */
    };
    void *want = g2h_untagged(COMMPAGE);
    uint16_t *p;

    if (!reserved_va) {
        void *addr = mmap(want, TARGET_PAGE_SIZE, PROT_READ | PROT_WRITE,
                          MAP_ANONYMOUS | MAP_PRIVATE | MAP_FIXED_NOREPLACE,
                          -1, 0);
        if (addr == MAP_FAILED) {
            perror("Allocating guest commpage");
            exit(EXIT_FAILURE);
        }
        if (addr != want) {
            return false;
        }
    } else if (mprotect(want, TARGET_PAGE_SIZE, PROT_READ | PROT_WRITE)) {
        perror("Protecting guest commpage");
        exit(EXIT_FAILURE);
    }

    p = want;
    for (int i = 0; i < TARGET_PAGE_SIZE / 2; i++) {
        p[i] = cpu_to_le16(0x00af);         /* EXCPT 15; */
    }
    for (int i = 0; i < ARRAY_SIZE(stub); i++) {
        p[BFIN_SIGRETURN_STUB / 2 + i] = cpu_to_le16(stub[i]);
    }

    if (mprotect(want, TARGET_PAGE_SIZE, PROT_READ)) {
        perror("Protecting guest commpage");
        exit(EXIT_FAILURE);
    }
    page_set_flags(COMMPAGE, COMMPAGE | ~TARGET_PAGE_MASK,
                   PAGE_READ | PAGE_EXEC | PAGE_VALID, PAGE_VALID);
    return true;
}
