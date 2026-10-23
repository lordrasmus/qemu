/*
 * Atmel NGW100 network gateway board (AT32AP7000)
 *
 * SDRAM, the internal SRAM, the interrupt controller, the power manager
 * and USART1 (the console). All other peripherals are unimplemented and
 * read as zero.
 *
 * -kernel takes a Linux vmlinux; it is started the way U-Boot does:
 * r12 = ATAG_MAGIC, r11 = the tag list (core, memory, command line and,
 * with -initrd, the RAM disk).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "elf.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/misc/unimp.h"
#include "system/address-spaces.h"
#include "system/reset.h"
#include "system/system.h"
#include "hw/avr32/ap7000.h"
#include "target/avr32/cpu.h"

#define NGW100_SDRAM_BASE   0x10000000
#define NGW100_SDRAM_MAX    (256 * MiB)
#define AP7000_SRAM_BASE    0x24000000
#define AP7000_SRAM_SIZE    (32 * KiB)
#define AP7000_PERIPH_BASE  0xff000000
#define AP7000_PM_BASE      0xfff00000
#define AP7000_INTC_BASE    0xfff00400
#define AP7000_USART1_BASE  0xffe01000
#define AP7000_USART1_GROUP 7

#define ATAG_MAGIC          0xa2a25441
#define ATAG_NONE           0x00000000
#define ATAG_CORE           0x54410001
#define ATAG_MEM            0x54410002
#define ATAG_CMDLINE        0x54410003
#define ATAG_RDIMG          0x54410004

#define NGW100_TAGS_SIZE    (4 * KiB)

typedef struct NGW100Boot {
    AVR32CPU *cpu;
    uint32_t entry;
    uint32_t tags;                      /* physical */
} NGW100Boot;

static uint64_t ngw100_translate(void *opaque, uint64_t addr)
{
    return addr & 0x1fffffff;           /* P1/P2 to physical */
}

static void ngw100_reset(void *opaque)
{
    NGW100Boot *boot = opaque;
    CPUAVR32State *env = &boot->cpu->env;

    cpu_reset(CPU(boot->cpu));
    env->r[AVR32_PC_REG] = boot->entry;
    env->r[12] = ATAG_MAGIC;
    env->r[11] = 0x80000000 | boot->tags;
}

static uint32_t *tag_put(uint32_t *p, uint32_t tag, const uint32_t *data,
                         int words)
{
    *p++ = cpu_to_be32(2 + words);
    *p++ = cpu_to_be32(tag);
    for (int i = 0; i < words; i++) {
        *p++ = cpu_to_be32(data[i]);
    }
    return p;
}

static void ngw100_load(MachineState *ms, NGW100Boot *boot)
{
    uint64_t entry, high, ram_end = NGW100_SDRAM_BASE + ms->ram_size;
    g_autofree uint32_t *tags = g_malloc0(NGW100_TAGS_SIZE);
    const char *cmdline = ms->kernel_cmdline ? ms->kernel_cmdline : "";
    uint32_t *p = tags;
    uint32_t data[3];
    hwaddr initrd = 0;
    ssize_t initrd_size = 0;
    size_t len;

    if (load_elf(ms->kernel_filename, NULL, ngw100_translate, NULL, &entry,
                 NULL, &high, NULL, ELFDATA2MSB, EM_AVR32, 0, 0) < 0) {
        error_report("could not load kernel '%s'", ms->kernel_filename);
        exit(1);
    }
    boot->entry = entry;
    boot->tags = ram_end - NGW100_TAGS_SIZE;

    if (ms->initrd_filename) {
        hwaddr low = ROUND_UP(ngw100_translate(NULL, high), 4 * KiB);

        initrd_size = get_image_size(ms->initrd_filename, NULL);
        if (initrd_size < 0) {
            error_report("could not load initrd '%s'", ms->initrd_filename);
            exit(1);
        }
        initrd = QEMU_ALIGN_DOWN(boot->tags - initrd_size, 4 * KiB);
        if (initrd < low ||
            load_image_targphys(ms->initrd_filename, initrd,
                                boot->tags - initrd, NULL) < 0) {
            error_report("could not load initrd '%s'", ms->initrd_filename);
            exit(1);
        }
    }

    data[0] = 1;                        /* read-only root */
    data[1] = 4096;
    data[2] = 0;
    p = tag_put(p, ATAG_CORE, data, 3);

    data[0] = NGW100_SDRAM_BASE;
    data[1] = ms->ram_size;
    data[2] = 0;
    p = tag_put(p, ATAG_MEM, data, 3);

    len = strlen(cmdline) + 1;
    if (len > NGW100_TAGS_SIZE / 2) {
        error_report("kernel command line too long");
        exit(1);
    }
    *p++ = cpu_to_be32(2 + DIV_ROUND_UP(len, 4));
    *p++ = cpu_to_be32(ATAG_CMDLINE);
    memcpy(p, cmdline, len);
    p += DIV_ROUND_UP(len, 4);

    if (initrd_size) {
        data[0] = initrd;
        data[1] = initrd_size;
        data[2] = 0;
        p = tag_put(p, ATAG_RDIMG, data, 3);
    }

    p = tag_put(p, ATAG_NONE, NULL, 0);
    p[-2] = 0;                          /* size 0 ends the list */
    rom_add_blob_fixed("atags", tags, (uint8_t *)p - (uint8_t *)tags,
                       boot->tags);
}

static void ngw100_init(MachineState *ms)
{
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *sram = g_new(MemoryRegion, 1);
    NGW100Boot *boot = g_new0(NGW100Boot, 1);
    DeviceState *intc, *dev;
    AVR32CPU *cpu;

    if (ms->ram_size > NGW100_SDRAM_MAX) {
        error_report("at most %" PRIu64 " MiB of SDRAM",
                     (uint64_t)NGW100_SDRAM_MAX / MiB);
        exit(1);
    }

    cpu = AVR32_CPU(cpu_create(ms->cpu_type));
    boot->cpu = cpu;

    memory_region_add_subregion(sysmem, NGW100_SDRAM_BASE, ms->ram);
    memory_region_init_ram(sram, NULL, "ap7000.sram", AP7000_SRAM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, AP7000_SRAM_BASE, sram);

    create_unimplemented_device("ap7000-peripherals", AP7000_PERIPH_BASE,
                                16 * MiB);

    intc = qdev_new(TYPE_AP7000_INTC);
    object_property_set_link(OBJECT(intc), "cpu", OBJECT(cpu), &error_fatal);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(intc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(intc), 0, AP7000_INTC_BASE);

    /* COUNT/COMPARE is IRQ 0, group 0 */
    qdev_connect_gpio_out_named(DEVICE(cpu), "compare", 0,
                                qdev_get_gpio_in(intc, 0));

    sysbus_create_simple(TYPE_AP7000_PM, AP7000_PM_BASE, NULL);

    dev = qdev_new(TYPE_ATMEL_USART);
    qdev_prop_set_chr(dev, "chardev", serial_hd(0));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, AP7000_USART1_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                       qdev_get_gpio_in(intc, AP7000_USART1_GROUP * 32));

    if (ms->kernel_filename) {
        ngw100_load(ms, boot);
    } else {
        boot->entry = 0xa0000000;
    }
    qemu_register_reset(ngw100_reset, boot);
}

static void ngw100_machine_init(MachineClass *mc)
{
    mc->desc = "Atmel NGW100 (AT32AP7000)";
    mc->init = ngw100_init;
    mc->default_cpu_type = AVR32_CPU_TYPE_NAME("ap7000");
    mc->default_ram_size = 32 * MiB;
    mc->default_ram_id = "ngw100.sdram";
}

DEFINE_MACHINE("atngw100", ngw100_machine_init)
