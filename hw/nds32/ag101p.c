/*
 * Andes AG101P-like board for the Andes Linux 3.4 BSP
 *
 * Built for a kernel configured with PLAT_AG101P and IVIC (no interrupt
 * controller, devices wired to the CPU's IVIC inputs, platforms/nointc.c).
 * Addresses and IRQ numbers are those of arch/nds32/include/asm/spec*.h:
 *
 *   SDRAM     0x00000000
 *   UART0     0x99600000  IRQ 7   (8250, reg-shift 2)
 *   UART1     0x98300000  IRQ 11
 *   FTTMR010  0x98400000  IRQ 19, 14, 15 (timer 1, 2, 3)
 *
 * The kernel is entered at its physical entry point with the MMU off and
 * finds its parameters as ATAGs at 0x400 (BOOT_PARAMETER_PA_BASE).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "cpu.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "hw/char/serial-mm.h"
#include "hw/misc/unimp.h"
#include "system/system.h"
#include "system/reset.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "elf.h"

#ifndef EM_NDS32
#define EM_NDS32 167
#endif

#define AG101P_UART0_BASE   0x99600000
#define AG101P_UART0_IRQ    7
#define AG101P_UART1_BASE   0x98300000
#define AG101P_UART1_IRQ    11
#define AG101P_TIMER_BASE   0x98400000
#define AG101P_TIMER_IRQ0   19
#define AG101P_TIMER_IRQ1   14
#define AG101P_TIMER_IRQ2   15
/* APB clock = CONFIG_SYS_CLK / 2 of the kernel (40 MHz there). */
#define AG101P_APB_CLK      20000000
#define AG101P_UART_CLK     14745600

#define AG101P_ATAG_BASE    0x400
#define AG101P_INITRD_BASE  (16 * MiB)

#define ATAG_NONE       0x00000000
#define ATAG_CORE       0x54410001
#define ATAG_MEM        0x54410002
#define ATAG_INITRD2    0x54420005
#define ATAG_CMDLINE    0x54410009

static struct {
    uint32_t entry;
} boot_info;

static void ag101p_cpu_reset(void *opaque)
{
    NDS32CPU *cpu = opaque;
    CPUNDS32State *env = &cpu->env;

    cpu_reset(CPU(cpu));
    env->pc = boot_info.entry;
}

static void ag101p_write_atags(MachineState *ms, hwaddr initrd_start,
                               hwaddr initrd_size)
{
    GByteArray *a = g_byte_array_new();
    const char *cmdline = ms->kernel_cmdline ? ms->kernel_cmdline : "";
    size_t clen = strlen(cmdline);

#define PUT(v) do { uint32_t _v = cpu_to_le32(v); \
                    g_byte_array_append(a, (uint8_t *)&_v, 4); } while (0)

    /* size in words, tag, flags, pagesize, rootdev */
    PUT(5); PUT(ATAG_CORE); PUT(1); PUT(4096); PUT(0);
    PUT(4); PUT(ATAG_MEM); PUT(ms->ram_size); PUT(0);
    if (initrd_size) {
        PUT(4); PUT(ATAG_INITRD2); PUT(initrd_start); PUT(initrd_size);
    }
    if (clen) {
        uint32_t words = (clen + 4) / 4;
        uint8_t *buf = g_malloc0(words * 4);

        memcpy(buf, cmdline, clen);
        PUT(2 + words); PUT(ATAG_CMDLINE);
        g_byte_array_append(a, buf, words * 4);
        g_free(buf);
    }
    PUT(0); PUT(ATAG_NONE);
#undef PUT

    if (a->len > 0x2000 - AG101P_ATAG_BASE) {
        error_report("ag101p: kernel command line too long");
        exit(EXIT_FAILURE);
    }
    rom_add_blob_fixed("atags", a->data, a->len, AG101P_ATAG_BASE);
    g_byte_array_free(a, true);
}

static void ag101p_init(MachineState *ms)
{
    MemoryRegion *sysmem = get_system_memory();
    NDS32CPU *cpu;
    DeviceState *dev;
    qemu_irq irq[32];
    hwaddr initrd_size = 0;
    uint64_t entry, hi;
    ssize_t size;

    cpu = NDS32_CPU(cpu_create(ms->cpu_type));
    for (int i = 0; i < 32; i++) {
        irq[i] = qdev_get_gpio_in_named(DEVICE(cpu), "IRQ", i);
    }

    memory_region_add_subregion(sysmem, 0, ms->ram);

    /* The rest of the AHB/APB peripherals the BSP maps but we lack. */
    create_unimplemented_device("ag101p.ahb", 0x90000000, 0x01000000);
    create_unimplemented_device("ag101p.apb", 0x98000000, 0x02000000);

    serial_mm_init(sysmem, AG101P_UART0_BASE, 2, irq[AG101P_UART0_IRQ],
                   AG101P_UART_CLK / 16, serial_hd(0), DEVICE_LITTLE_ENDIAN);
    serial_mm_init(sysmem, AG101P_UART1_BASE, 2, irq[AG101P_UART1_IRQ],
                   AG101P_UART_CLK / 16, serial_hd(1), DEVICE_LITTLE_ENDIAN);

    dev = qdev_new("fttmr010");
    qdev_prop_set_uint32(dev, "clock-frequency", AG101P_APB_CLK);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, AG101P_TIMER_BASE, 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, irq[AG101P_TIMER_IRQ0]);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 1, irq[AG101P_TIMER_IRQ1]);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 2, irq[AG101P_TIMER_IRQ2]);

    if (!ms->kernel_filename) {
        error_report("ag101p: -kernel is required");
        exit(EXIT_FAILURE);
    }

    /* vmlinux carries physical load addresses and entry. */
    size = load_elf(ms->kernel_filename, NULL, NULL, NULL, &entry, NULL,
                    &hi, NULL, ELFDATA2LSB, EM_NDS32, 0, 0);
    if (size < 0) {
        /* Raw Image, linked at PAGE_OFFSET + 0xc000. */
        size = load_image_targphys(ms->kernel_filename, 0xc000,
                                   ms->ram_size - 0xc000, NULL);
        if (size < 0) {
            error_report("ag101p: could not load kernel '%s'",
                         ms->kernel_filename);
            exit(EXIT_FAILURE);
        }
        entry = 0xc000;
        hi = 0xc000 + size;
    }
    boot_info.entry = entry;

    if (ms->initrd_filename) {
        if (hi > AG101P_INITRD_BASE) {
            error_report("ag101p: kernel overlaps the initrd at 0x%lx",
                         (unsigned long)AG101P_INITRD_BASE);
            exit(EXIT_FAILURE);
        }
        size = load_image_targphys(ms->initrd_filename, AG101P_INITRD_BASE,
                                   ms->ram_size - AG101P_INITRD_BASE, NULL);
        if (size < 0) {
            error_report("ag101p: could not load initrd '%s'",
                         ms->initrd_filename);
            exit(EXIT_FAILURE);
        }
        initrd_size = size;
    }

    ag101p_write_atags(ms, AG101P_INITRD_BASE, initrd_size);
    qemu_register_reset(ag101p_cpu_reset, cpu);
}

static void ag101p_machine_init(MachineClass *mc)
{
    mc->desc = "Andes AG101P (NDS32 N13, IVIC) for the Andes Linux 3.4 BSP";
    mc->init = ag101p_init;
    mc->default_cpu_type = TYPE_NDS32_CPU;
    mc->default_ram_size = 256 * MiB;
    mc->default_ram_id = "ag101p.ram";
}

DEFINE_MACHINE("ag101p", ag101p_machine_init)
