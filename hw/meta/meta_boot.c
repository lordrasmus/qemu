/*
 * META direct Linux boot
 *
 * Instead of a boot ROM and an LDR bootloader, set up what the bootloader
 * leaves behind for Linux: MMU tables mapping RAM at 0x40000000 (local heap)
 * plus a page of global heap at 0xc0000000 for the TBI structures, the kernel
 * at its virtual addresses, and the entry registers.
 *
 * Copyright (c) 2010 Imagination Technologies
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "qemu/cutils.h"
#include "cpu.h"
#include "elf.h"
#include "hw/core/loader.h"
#include "system/reset.h"
#include "system/address-spaces.h"
#include "hw/meta/meta_boot.h"
#include "hw/meta/meta_boot_mmu.h"
#include "hw/meta/meta_boot_tbi.h"
#include "system/device_tree.h"
#include <libfdt.h>

#define META_LHEAP_ADDR 0x40000000
#define META_GHEAP_ADDR 0xc0000000
#define META_GHEAP_SIZE 0x1000

static uint64_t meta_boot_translate(void *opaque, uint64_t addr)
{
    MetaBootInfo *boot = opaque;

    /* the local heap maps RAM from its start */
    return addr - META_LHEAP_ADDR + boot->ram_phys;
}

/* RAM the boot MMU leaves to Linux: all of it but the tables and TBI */
static hwaddr meta_boot_lheap_end(MetaBootInfo *boot)
{
    hwaddr mmutable_size = (boot->flags & META_BOOT_MMU_HTP) ? 0x8000 : 0x80000;

    return boot->ram_phys + boot->ram_size - mmutable_size - META_GHEAP_SIZE;
}

static void meta_boot_reset(void *opaque)
{
    CPUArchState *env = opaque;
    MetaCore *core = META_THREAD2CORE(env);
    MetaBootInfo *boot = env->boot_info;
    struct meta_mmu_seg local_segs[2], global_segs[2];
    vaddr gheap_brk = META_GHEAP_ADDR;
    vaddr lheap_size;
    hwaddr gheap_phys, gheap_v2p;
    target_ulong tbi, segs, args;
    target_ulong brk;
    size_t len;

    cpu_meta_core_reset(core);

    /*
     * Allow the SoC to set up hardware that would normally be done by the
     * bootloader, such as DDR and clock registers.
     */
    if (boot->setup_direct_boot) {
        boot->setup_direct_boot(env, boot->setup_direct_boot_opaque);
    }

    /*
     * Local heap maps all of RAM except a bit at the end which is used for
     * page tables and global heap.
     */
    lheap_size = meta_boot_lheap_end(boot) - boot->ram_phys;
    gheap_phys = boot->ram_phys + lheap_size;
    gheap_v2p = gheap_phys - META_GHEAP_ADDR;
    meta_mmu_init_seg(&local_segs[0], META_LHEAP_ADDR,
                      META_LHEAP_ADDR + lheap_size, boot->ram_phys,
                      meta_tbi_seg_id(env->thread_num, SEG_SCOPE_LOCAL,
                                      SEG_TYPE_HEAP));
    meta_mmu_null_seg(&local_segs[1]);
    /* Global heap (for TBI structures) */
    meta_mmu_init_seg(&global_segs[0], META_GHEAP_ADDR,
                      META_GHEAP_ADDR + META_GHEAP_SIZE, gheap_phys,
                      meta_tbi_seg_id(env->thread_num, SEG_SCOPE_GLOBAL,
                                      SEG_TYPE_HEAP));
    meta_mmu_null_seg(&global_segs[1]);

    /* Set up MMU tables */
    meta_mmu_setup(env, boot->flags & META_BOOT_MMU_HTP,
                   boot->ram_phys + boot->ram_size, local_segs, global_segs);

    /* Set up TBI structures */
    brk = gheap_brk;
    tbi = meta_tbi_setup(env, &brk, gheap_v2p);
    segs = meta_tbi_seg_setup(env, &brk, gheap_v2p, local_segs, global_segs);

    /*
     * After the kernel goes the device tree (with an initrd) or the command
     * line; the kernel tries D0Ar4 as a device tree first.
     */
    args = boot->kernel_end;
    len = boot->kernel_cmdline ? strnlen(boot->kernel_cmdline, 255) : 0;
    if (boot->fdt) {
        address_space_write(&address_space_memory,
                            meta_boot_translate(boot, args),
                            MEMTXATTRS_UNSPECIFIED, boot->fdt, boot->fdt_size);
    } else {
        char buf[256] = { 0 };

        memcpy(buf, boot->kernel_cmdline ? boot->kernel_cmdline : "", len);
        address_space_write(&address_space_memory,
                            meta_boot_translate(boot, args),
                            MEMTXATTRS_UNSPECIFIED, buf, len + 1);
    }

    /* Set up kernel entry point */
    env->pc[META_PC] = boot->entry; /* PC */
    env->dregs[1][3] = tbi;         /* D1Ar1: TBI (ISTAT) */
    env->dregs[0][3] = 0;           /* D0Ar2: TBI */
    env->dregs[1][2] = segs;        /* D1Ar3: TBI Segments */
    env->dregs[0][2] = args;        /* D0Ar4: Arguments or DTB */
    env->dregs[1][4] = 0;           /* D1RtP: Return pointer */
    meta_core_intreg_write(env, META_UNIT_CT, META_TXENABLE, 1);
}


/*
 * A TZ1090 device tree with just the interrupt controller, for passing the
 * initrd.  linux,initrd-* are physical; arch/metag needs
 * kernel-metag-dt-initrd.patch to take them.
 */
static void meta_boot_initrd_fdt(MetaBootInfo *boot)
{
    static const char *meta_compatible[] = { "toumaz,tz1090", "img,meta" };
    hwaddr lheap_end = meta_boot_lheap_end(boot);
    hwaddr initrd_base;
    ssize_t initrd_size;
    uint32_t intc;
    void *fdt;

    initrd_size = get_image_size(boot->initrd_filename, NULL);
    if (initrd_size < 0) {
        error_report("could not load initrd '%s'", boot->initrd_filename);
        exit(1);
    }
    initrd_base = (lheap_end - initrd_size) & TARGET_PAGE_MASK;
    if (meta_boot_translate(boot, boot->kernel_end) + 0x10000 > initrd_base) {
        error_report("initrd '%s' does not fit in RAM", boot->initrd_filename);
        exit(1);
    }
    initrd_size = load_image_targphys(boot->initrd_filename, initrd_base,
                                      lheap_end - initrd_base, &error_fatal);

    fdt = create_device_tree(&boot->fdt_size);
    qemu_fdt_setprop_string(fdt, "/", "model", "Comet METAmorph (01SP)");
    qemu_fdt_setprop_string_array(fdt, "/", "compatible",
                                  (char **)meta_compatible,
                                  ARRAY_SIZE(meta_compatible));
    qemu_fdt_setprop_cell(fdt, "/", "#address-cells", 1);
    qemu_fdt_setprop_cell(fdt, "/", "#size-cells", 1);

    intc = qemu_fdt_alloc_phandle(fdt);
    qemu_fdt_add_subnode(fdt, "/interrupt-controller");
    qemu_fdt_setprop_string(fdt, "/interrupt-controller", "compatible",
                            "img,meta-intc");
    qemu_fdt_setprop(fdt, "/interrupt-controller", "interrupt-controller",
                     NULL, 0);
    qemu_fdt_setprop_cell(fdt, "/interrupt-controller", "#interrupt-cells", 2);
    qemu_fdt_setprop_cell(fdt, "/interrupt-controller", "num-banks", 2);
    qemu_fdt_setprop_cell(fdt, "/interrupt-controller", "phandle", intc);
    qemu_fdt_setprop_cell(fdt, "/", "interrupt-parent", intc);

    /* the memory comes from the TBI heap segment, not from here */
    qemu_fdt_add_subnode(fdt, "/memory");
    qemu_fdt_setprop_string(fdt, "/memory", "device_type", "memory");
    qemu_fdt_setprop_cells(fdt, "/memory", "reg", 0, 0);

    qemu_fdt_add_subnode(fdt, "/chosen");
    if (boot->kernel_cmdline) {
        qemu_fdt_setprop_string(fdt, "/chosen", "bootargs",
                                boot->kernel_cmdline);
    }
    qemu_fdt_setprop_cell(fdt, "/chosen", "linux,initrd-start", initrd_base);
    qemu_fdt_setprop_cell(fdt, "/chosen", "linux,initrd-end",
                          initrd_base + initrd_size);

    fdt_pack(fdt);
    boot->fdt_size = fdt_totalsize(fdt);
    boot->fdt = fdt;
}

void meta_setup_boot(MetaCore *core, MetaBootInfo *boot)
{
    CPUArchState *env;
    uint64_t entry, high;

    if (boot->boot_thread < 0 || boot->boot_thread >= core->num_threads) {
        error_report("Invalid boot thread %d", boot->boot_thread);
        exit(1);
    }
    env = &core->threads[boot->boot_thread].env;

    if (!boot->kernel_filename) {
        error_report("META: no boot ROM, -kernel is required");
        exit(1);
    }
    if (load_elf(boot->kernel_filename, NULL, meta_boot_translate, boot,
                 &entry, NULL, &high, NULL, ELFDATA2LSB, EM_METAG, 0, 0) < 0) {
        error_report("could not load kernel '%s'", boot->kernel_filename);
        exit(1);
    }
    boot->entry = entry;
    /* high is the translated (physical) end */
    boot->kernel_end = ROUND_UP(high - boot->ram_phys + META_LHEAP_ADDR, 8);

    if (boot->initrd_filename) {
        meta_boot_initrd_fdt(boot);
    }

    env->boot_info = boot;
    qemu_register_reset(meta_boot_reset, env);
}
