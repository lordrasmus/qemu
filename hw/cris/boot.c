/*
 * CRIS image loading.
 *
 * Copyright (c) 2010 Edgar E. Iglesias, Axis Communications AB.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "target/cris/cpu.h"
#include "hw/core/loader.h"
#include "elf.h"
#include "boot.h"
#include "qemu/cutils.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "system/reset.h"

static void main_cpu_reset(void *opaque)
{
    CRISCPU *cpu = opaque;
    CPUCRISState *env = &cpu->env;
    struct cris_load_info *li;

    li = env->load_info;

    cpu_reset(CPU(cpu));

    if (!li) {
        /* nothing more to do.  */
        return;
    }

    env->pc = li->entry;

    if (li->image_filename) {
        env->regs[8] = 0x56902387; /* RAM boot magic.  */
        env->regs[9] = 0x40004000 + li->image_size;
    }

    if (li->cmdline) {
        /* Let the kernel know we are modifying the cmdline.  */
        env->regs[10] = 0x87109563;
        env->regs[11] = li->cmdline_addr;
    }
}

static uint64_t translate_kernel_address(void *opaque, uint64_t addr)
{
    return addr - 0x80000000LL;
}

void cris_load_image(CRISCPU *cpu, struct cris_load_info *li)
{
    CPUCRISState *env = &cpu->env;
    uint64_t entry;
    int kcmdline_len;
    int image_size;

    env->load_info = li;
    /* Boots a kernel elf binary, os/linux-2.6/vmlinux from the axis 
       devboard SDK.  */
    image_size = load_elf(li->image_filename, NULL,
                          translate_kernel_address, NULL,
                          &entry, NULL, NULL, NULL, 0, EM_CRIS, 0, 0);
    li->entry = entry;
    if (image_size < 0) {
        /* Takes a kimage from the axis devboard SDK.  */
        image_size = load_image_targphys(li->image_filename, 0x40004000,
                                         li->ram_size, NULL);
        li->entry = 0x40004000;
    }

    if (image_size < 0) {
        fprintf(stderr, "qemu: could not load kernel '%s'\n",
                li->image_filename);
        exit(1);
    }

    /*
     * The command line goes to the last 4K of DRAM, clear of the kernel,
     * whose first segment starts at 0x40000000.  The initrd goes page
     * aligned below it and is announced as "initrd=<start>,<size>" on the
     * command line, which Linux on cris only understands with the initrd
     * patch of the qemu-imgs repo.
     */
    li->cmdline_addr = 0x40000000 + li->ram_size - 4 * KiB;
    if (li->initrd_filename) {
        int64_t size = get_image_size(li->initrd_filename, NULL);
        hwaddr top = li->cmdline_addr;
        hwaddr start;
        char *cmdline;

        if (size <= 0) {
            error_report("could not load initrd '%s'", li->initrd_filename);
            exit(1);
        }
        start = (top - size) & ~(hwaddr)(8 * KiB - 1);
        if (start < 0x40000000 + li->ram_size / 2 ||
            load_image_targphys(li->initrd_filename, start, size,
                                NULL) != size) {
            error_report("could not load initrd '%s'", li->initrd_filename);
            exit(1);
        }
        cmdline = g_strdup_printf("%s%sinitrd=0x%" HWADDR_PRIx ",%" PRId64,
                                  li->cmdline ? li->cmdline : "",
                                  li->cmdline && li->cmdline[0] ? " " : "",
                                  start, size);
        li->cmdline = cmdline;
    }

    if (li->cmdline && (kcmdline_len = strlen(li->cmdline))) {
        if (kcmdline_len > 256) {
            fprintf(stderr, "Too long CRIS kernel cmdline (max 256)\n");
            exit(1);
        }
        pstrcpy_targphys("cmdline", li->cmdline_addr, 256, li->cmdline);
    }
    qemu_register_reset(main_cpu_reset, cpu);
}
