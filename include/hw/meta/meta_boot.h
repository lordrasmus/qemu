/*
 * META direct Linux boot
 *
 * Copyright (c) 2010 Imagination Technologies
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#ifndef HW_META_META_BOOT_H
#define HW_META_META_BOOT_H

#include "core.h"

struct MetaBootInfo {
    /* The kernel ELF, loaded at its virtual addresses through the boot MMU */
    const char *kernel_filename;
    /* Cmdline is put after the kernel and passed in D0Ar4 */
    const char *kernel_cmdline;
    /*
     * With an initrd, a device tree goes after the kernel instead, carrying
     * the cmdline and the initrd in /chosen.
     */
    const char *initrd_filename;

    /* RAM information */
    hwaddr ram_phys;
    hwaddr ram_size;

    /* Callback to setup up SoC hardware in the event of a direct kernel boot */
    void (*setup_direct_boot)(CPUArchState *env, void *opaque);
    void *setup_direct_boot_opaque;

    /* The thread to boot the kernel on */
    int boot_thread;

    /* Some boot options */
    uint32_t flags;
#define META_BOOT_MINIM     (1 << 0)
#define META_BOOT_MMU_HTP   (1 << 1)

    /* where load_elf put the kernel, filled in by meta_setup_boot */
    vaddr entry;
    vaddr kernel_end;
    void *fdt;
    int fdt_size;
};

/* Load a Linux kernel and set up the core to boot it on every reset */
void meta_setup_boot(MetaCore *core, MetaBootInfo *boot);

#endif
