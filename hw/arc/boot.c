/*
 * QEMU ARC CPU
 *
 * Copyright (c) 2020 Synppsys Inc.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see
 * http://www.gnu.org/licenses/lgpl-2.1.html
 */

#include "qemu/osdep.h"
#include "system/physmem.h"
#include "boot.h"
#include "target/arc/arc-common.h"
#include "elf.h"
#include "hw/core/loader.h"
#include "qemu/error-report.h"
#include "qemu/units.h"

void arc_cpu_reset(void *opaque)
{
    ARCCPU *cpu = opaque;
    CPUARCState *env = &cpu->env;
    const struct arc_boot_info *info = env->boot_info;

    cpu_reset(CPU(cpu));

    /*
     * Right before start CPU gets reset wiping out everything
     * but PC which we set on Elf load.
     *
     * And if we still want to pass something like U-Boot data
     * via CPU registers we have to do it here.
     */

    if (info->kernel_cmdline && strlen(info->kernel_cmdline)) {
        /* Load "cmdline" far enough from the kernel image. */
        hwaddr cmdline_offset, cmdline_addr;
        const hwaddr max_page_size = 64 * KiB;

        /*
         * During early boot only first 1 GiB is mapped by kernel.
         * So do not place cmdline after that point.
         */
        cmdline_offset = MIN(1 * GiB, info->ram_size) -
            strlen(info->kernel_cmdline);
        cmdline_addr = info->ram_start +
            QEMU_ALIGN_DOWN(cmdline_offset, max_page_size);

        physical_memory_write(cmdline_addr, info->kernel_cmdline,
                                  strlen(info->kernel_cmdline));

        /* We're passing "cmdline" */
        cpu->env.r[0] = ARC_UBOOT_CMDLINE;
        cpu->env.r[2] = cmdline_addr;
    }
}


/*
 * The kernel takes the initrd from "initrd=<start>,<size>" on the command
 * line (arch/arc/mm/init.c in 4.19, init/do_mounts_initrd.c later). Put it
 * at the top of the first GiB, which is all the kernel maps early, below
 * the command line that arc_cpu_reset() writes there.
 */
static void arc_load_initrd(struct arc_boot_info *info, uint64_t kernel_end)
{
    hwaddr top = info->ram_start + MIN(1 * GiB, info->ram_size) - 128 * KiB;
    ssize_t size = get_image_size(info->initrd_filename, NULL);
    hwaddr start;

    if (size < 0) {
        error_report("could not load initrd '%s'", info->initrd_filename);
        exit(EXIT_FAILURE);
    }
    start = QEMU_ALIGN_DOWN(top - size, 64 * KiB);
    if (start < kernel_end ||
        load_image_targphys(info->initrd_filename, start, top - start,
                            NULL) != size) {
        error_report("could not load initrd '%s'", info->initrd_filename);
        exit(EXIT_FAILURE);
    }
    info->kernel_cmdline = g_strdup_printf("%s%sinitrd=0x%" HWADDR_PRIx
                                           ",0x%zx",
                                           info->kernel_cmdline ?: "",
                                           info->kernel_cmdline ? " " : "",
                                           start, size);
}

void arc_load_kernel(ARCCPU *cpu, struct arc_boot_info *info)
{
    hwaddr entry;
    uint64_t high = 0;
    int elf_machine, kernel_size;

    if (!info->kernel_filename) {
        error_report("missing kernel file");
        exit(EXIT_FAILURE);
    }

    /*
     * cpu->family is a bitmask, not an ordinal: ARC700 is 1 << 2, so the
     * original "> 2" test asked for an ARCv2 ELF on ARC700 kernels.
     */
    elf_machine = (cpu->family & ARC_OPCODE_ARCV1) != 0 ?
                  EM_ARC_COMPACT : EM_ARC_COMPACT2;
    elf_machine = (cpu->family & ARC_OPCODE_V3_ALL) != 0 ? EM_ARC_COMPACT3_64 : elf_machine;
    kernel_size = load_elf(info->kernel_filename, NULL, NULL, NULL,
                           &entry, NULL, &high, NULL,
                           false, /* little endian */
                           elf_machine, 1, 0);

    if (kernel_size < 0) {
        int is_linux;

        kernel_size = load_uimage(info->kernel_filename, &entry, NULL,
                                  &is_linux, NULL, NULL);
        if (!is_linux) {
            error_report("Wrong U-Boot image, only Linux kernel is supported");
            exit(EXIT_FAILURE);
        }
    }

    if (kernel_size < 0) {
        error_report("No kernel image found");
        exit(EXIT_FAILURE);
    }

    if (info->initrd_filename) {
        arc_load_initrd(info, high);
    }

    cpu->env.boot_info = info;

    /* Set CPU's PC to point to the entry-point */
    cpu->env.pc = entry;
}


/*-*-indent-tabs-mode:nil;tab-width:4;indent-line-function:'insert-tab'-*-*/
/* vim: set ts=4 sw=4 et: */
