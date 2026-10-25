/*
 * Andes AE3XX-like board for NDS32 Linux
 *
 * Addresses and compatible strings follow arch/nds32/boot/dts/ae3xx.dts,
 * so a mainline kernel boots with the device tree generated here.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/error-report.h"
#include "qemu/guest-random.h"
#include "qapi/error.h"
#include "cpu.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "hw/char/serial-mm.h"
#include "system/system.h"
#include "system/reset.h"
#include "system/device_tree.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "elf.h"

#include <libfdt.h>

#ifndef EM_NDS32
#define EM_NDS32 167
#endif

#define AE3XX_UART_BASE     0xf0300000
#define AE3XX_UART_IRQ      8
#define AE3XX_TIMER_BASE    0xf0400000
#define AE3XX_TIMER_IRQ     2
#define AE3XX_PCLK          30000000

static struct {
    uint32_t entry;
    uint32_t fdt;
} boot_info;

static void ae3xx_cpu_reset(void *opaque)
{
    NDS32CPU *cpu = opaque;
    CPUNDS32State *env = &cpu->env;

    cpu_reset(CPU(cpu));
    env->pc = boot_info.entry;
    env->gpr[2] = boot_info.fdt;
}

static void *ae3xx_create_fdt(MachineState *ms, int *size,
                              hwaddr initrd_start, hwaddr initrd_end)
{
    void *fdt = create_device_tree(size);
    uint32_t intc, clk;
    uint8_t rng_seed[32];
    char *name;

    qemu_fdt_setprop_string(fdt, "/", "compatible", "andestech,ae3xx");
    qemu_fdt_setprop_cell(fdt, "/", "#address-cells", 1);
    qemu_fdt_setprop_cell(fdt, "/", "#size-cells", 1);

    qemu_fdt_add_subnode(fdt, "/cpus");
    qemu_fdt_setprop_cell(fdt, "/cpus", "#address-cells", 1);
    qemu_fdt_setprop_cell(fdt, "/cpus", "#size-cells", 0);
    qemu_fdt_add_subnode(fdt, "/cpus/cpu@0");
    qemu_fdt_setprop_string(fdt, "/cpus/cpu@0", "device_type", "cpu");
    qemu_fdt_setprop(fdt, "/cpus/cpu@0", "compatible",
                     "andestech,n13\0andestech,nds32v3",
                     sizeof("andestech,n13\0andestech,nds32v3"));
    qemu_fdt_setprop_cell(fdt, "/cpus/cpu@0", "reg", 0);
    qemu_fdt_setprop_cell(fdt, "/cpus/cpu@0", "clock-frequency", 60000000);

    name = g_strdup_printf("/memory@%x", 0);
    qemu_fdt_add_subnode(fdt, name);
    qemu_fdt_setprop_string(fdt, name, "device_type", "memory");
    qemu_fdt_setprop_cells(fdt, name, "reg", 0, ms->ram_size);
    g_free(name);

    intc = qemu_fdt_alloc_phandle(fdt);
    qemu_fdt_add_subnode(fdt, "/interrupt-controller");
    qemu_fdt_setprop_string(fdt, "/interrupt-controller", "compatible",
                            "andestech,ativic32");
    qemu_fdt_setprop_cell(fdt, "/interrupt-controller", "#interrupt-cells", 1);
    qemu_fdt_setprop(fdt, "/interrupt-controller", "interrupt-controller",
                     NULL, 0);
    qemu_fdt_setprop_cell(fdt, "/interrupt-controller", "phandle", intc);
    qemu_fdt_setprop_cell(fdt, "/", "interrupt-parent", intc);

    clk = qemu_fdt_alloc_phandle(fdt);
    qemu_fdt_add_subnode(fdt, "/clk");
    qemu_fdt_setprop_string(fdt, "/clk", "compatible", "fixed-clock");
    qemu_fdt_setprop_cell(fdt, "/clk", "#clock-cells", 0);
    qemu_fdt_setprop_cell(fdt, "/clk", "clock-frequency", AE3XX_PCLK);
    qemu_fdt_setprop_cell(fdt, "/clk", "phandle", clk);

    qemu_fdt_add_subnode(fdt, "/apb");
    qemu_fdt_setprop_string(fdt, "/apb", "compatible", "simple-bus");
    qemu_fdt_setprop_cell(fdt, "/apb", "#address-cells", 1);
    qemu_fdt_setprop_cell(fdt, "/apb", "#size-cells", 1);
    qemu_fdt_setprop(fdt, "/apb", "ranges", NULL, 0);

    name = g_strdup_printf("/apb/serial@%x", AE3XX_UART_BASE);
    qemu_fdt_add_subnode(fdt, name);
    qemu_fdt_setprop(fdt, name, "compatible",
                     "andestech,uart16550\0ns16550a",
                     sizeof("andestech,uart16550\0ns16550a"));
    qemu_fdt_setprop_cells(fdt, name, "reg", AE3XX_UART_BASE, 0x1000);
    qemu_fdt_setprop_cell(fdt, name, "interrupts", AE3XX_UART_IRQ);
    qemu_fdt_setprop_cell(fdt, name, "clock-frequency", 14745600);
    qemu_fdt_setprop_cell(fdt, name, "reg-shift", 2);
    qemu_fdt_setprop_cell(fdt, name, "reg-offset", 32);
    qemu_fdt_setprop_cell(fdt, name, "no-loopback-test", 1);
    qemu_fdt_add_subnode(fdt, "/chosen");
    qemu_fdt_setprop_string(fdt, "/chosen", "stdout-path", name);
    g_free(name);

    name = g_strdup_printf("/apb/timer@%x", AE3XX_TIMER_BASE);
    qemu_fdt_add_subnode(fdt, name);
    qemu_fdt_setprop_string(fdt, name, "compatible", "andestech,atcpit100");
    qemu_fdt_setprop_cells(fdt, name, "reg", AE3XX_TIMER_BASE, 0x1000);
    qemu_fdt_setprop_cell(fdt, name, "interrupts", AE3XX_TIMER_IRQ);
    qemu_fdt_setprop_cell(fdt, name, "clocks", clk);
    qemu_fdt_setprop_string(fdt, name, "clock-names", "PCLK");
    g_free(name);

    if (ms->kernel_cmdline && *ms->kernel_cmdline) {
        qemu_fdt_setprop_string(fdt, "/chosen", "bootargs",
                                ms->kernel_cmdline);
    }
    if (initrd_start) {
        qemu_fdt_setprop_cell(fdt, "/chosen", "linux,initrd-start",
                              initrd_start);
        qemu_fdt_setprop_cell(fdt, "/chosen", "linux,initrd-end",
                              initrd_end);
    }
    qemu_guest_getrandom_nofail(rng_seed, sizeof(rng_seed));
    qemu_fdt_setprop(fdt, "/chosen", "rng-seed", rng_seed, sizeof(rng_seed));

    return fdt;
}

static void ae3xx_init(MachineState *ms)
{
    MemoryRegion *sysmem = get_system_memory();
    NDS32CPU *cpu;
    DeviceState *dev;
    qemu_irq irq[32];
    hwaddr high, initrd_start = 0, initrd_end = 0;
    void *fdt;
    int fdt_size;

    cpu = NDS32_CPU(cpu_create(ms->cpu_type));
    for (int i = 0; i < 32; i++) {
        irq[i] = qdev_get_gpio_in_named(DEVICE(cpu), "IRQ", i);
    }

    memory_region_add_subregion(sysmem, 0, ms->ram);

    serial_mm_init(sysmem, AE3XX_UART_BASE + 32, 2, irq[AE3XX_UART_IRQ],
                   115200, serial_hd(0), DEVICE_LITTLE_ENDIAN);

    dev = qdev_new("atcpit100");
    qdev_prop_set_uint32(dev, "clock-frequency", AE3XX_PCLK);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, AE3XX_TIMER_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, irq[AE3XX_TIMER_IRQ]);

    if (!ms->kernel_filename) {
        error_report("ae3xx: -kernel is required");
        exit(EXIT_FAILURE);
    }

    {
        uint64_t entry, hi;
        /* vmlinux carries physical load addresses and entry. */
        ssize_t size = load_elf(ms->kernel_filename, NULL,
                                NULL, NULL, &entry, NULL,
                                &hi, NULL, ELFDATA2LSB, EM_NDS32, 0, 0);
        if (size < 0) {
            /* Raw Image, linked at PAGE_OFFSET + 0xc000. */
            size = load_image_targphys(ms->kernel_filename, 0xc000,
                                       ms->ram_size - 0xc000, NULL);
            if (size < 0) {
                error_report("ae3xx: could not load kernel '%s'",
                             ms->kernel_filename);
                exit(EXIT_FAILURE);
            }
            entry = 0xc000;
            hi = 0xc000 + size;
        }
        boot_info.entry = entry;
        high = ROUND_UP(hi, 1 * MiB);
    }

    /*
     * The device tree goes right behind the kernel: head.S maps only as
     * much as its TLB covers (16 MB here), and the kernel parses the
     * tree before paging_init.  The initrd follows 1 MB later.
     */
    boot_info.fdt = high;
    high += 1 * MiB;

    if (ms->initrd_filename) {
        ssize_t size = load_image_targphys(ms->initrd_filename, high,
                                           ms->ram_size - high, NULL);
        if (size < 0) {
            error_report("ae3xx: could not load initrd '%s'",
                         ms->initrd_filename);
            exit(EXIT_FAILURE);
        }
        initrd_start = high;
        initrd_end = high + size;
        high = ROUND_UP(initrd_end, 4 * KiB);
    }

    if (ms->dtb) {
        fdt = load_device_tree(ms->dtb, &fdt_size);
        if (!fdt) {
            exit(EXIT_FAILURE);
        }
        if (ms->kernel_cmdline && *ms->kernel_cmdline) {
            qemu_fdt_setprop_string(fdt, "/chosen", "bootargs",
                                    ms->kernel_cmdline);
        }
        if (initrd_start) {
            qemu_fdt_setprop_cell(fdt, "/chosen", "linux,initrd-start",
                                  initrd_start);
            qemu_fdt_setprop_cell(fdt, "/chosen", "linux,initrd-end",
                                  initrd_end);
        }
    } else {
        fdt = ae3xx_create_fdt(ms, &fdt_size, initrd_start, initrd_end);
    }
    if (initrd_start) {
        /*
         * arch/nds32 never calls reserve_initrd_mem(), but it honours
         * /memreserve/; without this the initrd is handed to the page
         * allocator before it is unpacked.
         */
        fdt_add_mem_rsv(fdt, initrd_start, initrd_end - initrd_start);
    }
    fdt_pack(fdt);
    fdt_size = fdt_totalsize(fdt);
    rom_add_blob_fixed("fdt", fdt, fdt_size, boot_info.fdt);
    ms->fdt = fdt;

    qemu_register_reset(ae3xx_cpu_reset, cpu);
}

static void ae3xx_machine_init(MachineClass *mc)
{
    mc->desc = "Andes AE3XX (NDS32 N13) for Linux";
    mc->init = ae3xx_init;
    mc->default_cpu_type = TYPE_NDS32_CPU;
    mc->default_ram_size = 256 * MiB;
    mc->default_ram_id = "ae3xx.ram";
    mc->is_default = true;
}

DEFINE_MACHINE("ae3xx", ae3xx_machine_init)
