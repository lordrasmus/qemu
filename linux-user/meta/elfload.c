/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"
#include "qemu.h"
#include "loader.h"
#include "user-internals.h"
#include "target_elf.h"

const char *get_elf_cpu_model(uint32_t eflags)
{
    /* the META 2.1 core of the TZ1090 (Comet) */
    return "harrier";
}

/*
 * The gateway page holds kernel helpers that user code calls directly
 * (uclibc-ng: ___metag_load_tp and the 32-bit cmpxchg).  Each entry is
 * a SWITCH that cpu_loop() recognises by its address.
 */
bool init_guest_commpage(void)
{
    int host_page_size = qemu_real_host_page_size();
    abi_ptr commpage = COMMPAGE & -host_page_size;
    void *want = g2h_untagged(commpage);
    void *addr;

    addr = mmap(want, host_page_size, PROT_READ | PROT_WRITE,
                MAP_ANONYMOUS | MAP_PRIVATE |
                (commpage < reserved_va ? MAP_FIXED : MAP_FIXED_NOREPLACE),
                -1, 0);
    if (addr == MAP_FAILED) {
        perror("Allocating guest commpage");
        exit(EXIT_FAILURE);
    }
    if (addr != want) {
        return false;
    }

    __put_user(META_GATEWAY_SWITCH, (uint32_t *)g2h_untagged(META_GATEWAY_GET_TLS));
    __put_user(META_GATEWAY_SWITCH, (uint32_t *)g2h_untagged(META_GATEWAY_CMPXCHG));

    if (mprotect(addr, host_page_size, PROT_READ)) {
        perror("Protecting guest commpage");
        exit(EXIT_FAILURE);
    }

    page_set_flags(commpage, commpage | (host_page_size - 1),
                   PAGE_READ | PAGE_EXEC | PAGE_VALID, PAGE_VALID);
    return true;
}
