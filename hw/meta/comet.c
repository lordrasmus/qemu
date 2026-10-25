/*
 * Toumaz Xenif (Comet, TZ1090) SoC and the Comet METAmorph board (01SP)
 *
 * Only what Linux needs to boot to userspace: the Meta "harrier" core with
 * its code and data RAMs, the on-chip and UCC SRAMs, SDRAM, and the
 * peripheral registers the kernel reads for the clock setup.  The console is
 * the debug adapter's channel 1 (-device da).  The other blocks (SCB, SPI,
 * UARTs, SD, USB, display, PDC, DMA) are unimplemented-device regions.
 *
 * Copyright (c) 2011 Imagination Technologies
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "cpu.h"
#include "core.h"
#include "hw/core/boards.h"
#include "hw/misc/unimp.h"
#include "system/address-spaces.h"
#include "system/reset.h"
#include "hw/meta/meta_boot.h"

#define COMET_DEFAULT_RESET_CFG      0xf0000390
#define COMET_IRQ_MAX                64

#define COMET_ROM_BASE               0xe0000000
#define COMET_ROM_SIZE               0xc000

#define COMET_CORECODE_BASE          0x80000000
#define COMET_CORECODE_SLAVE_BASE    0x02040000
#define COMET_CORECODE_SIZE          0x10000
#define COMET_CORECODE_END           0x82000000

#define COMET_COREDATA_BASE          0x82000000
#define COMET_COREDATA_SLAVE_BASE    0x02030000
#define COMET_COREDATA_SIZE          0x10000
#define COMET_COREDATA_END           0x84000000

#define COMET_INTRAM_BASE            0xe0200000
#define COMET_INTRAM_SIZE            0x60000

#define COMET_UCC0_MTXSRAM_BASE      0xe0260000
#define COMET_UCC0_MTXSRAM_SIZE      0x50000
#define COMET_UCC0_MCPSRAM16_BASE    0xe02b0000
#define COMET_UCC0_MCPSRAM16_SIZE    0x8e00
#define COMET_UCC0_MCPSRAM24_BASE    0xe02c0000
#define COMET_UCC0_MCPSRAM24_SIZE    0x2b800

#define COMET_UCC1_MTXSRAM_BASE      0xe0300000
#define COMET_UCC1_MTXSRAM_SIZE      0x50000

#define COMET_SDRAM_BASE             0xb0000000
#define COMET_SDRAM_SIZE             0x10000000

#define COMET_PERIPHERALS_BASE       0x02004000
#define COMET_PERIPHERALS_SIZE       0x400

/* peripheral registers */
#define CR_PERIP_RESET_CFG           0x0004
#define CR_COMET_CORE_REV            0x02d0

#define CORECODE_ALIASES ((COMET_CORECODE_END - COMET_CORECODE_BASE) / \
                          COMET_CORECODE_SIZE)
#define COREDATA_ALIASES ((COMET_COREDATA_END - COMET_COREDATA_BASE) / \
                          COMET_COREDATA_SIZE)

typedef struct CometState {
    MetaCore *core;
    MetaBootInfo boot;
    uint32_t reset_cfg;

    MemoryRegion rom;
    MemoryRegion core_code[CORECODE_ALIASES];
    MemoryRegion core_code_slave;
    MemoryRegion core_data[COREDATA_ALIASES];
    MemoryRegion core_data_slave;
    MemoryRegion internal;
    MemoryRegion ucc0_mtxsram;
    MemoryRegion ucc0_mcpsram16;
    MemoryRegion ucc0_mcpsram24;
    MemoryRegion ucc1_mtxsram;
    MemoryRegion peripherals;
} CometState;

static uint64_t comet_peripheral_io_read(void *opaque, hwaddr addr,
                                         unsigned size)
{
    CometState *s = opaque;

    switch (addr) {
    case CR_PERIP_RESET_CFG:
        return s->reset_cfg;
    case CR_COMET_CORE_REV:
        /* PS1 */
        return 0x00010000;
    default:
        qemu_log_mask(LOG_UNIMP, "comet: unhandled perip read 0x%08"
                      HWADDR_PRIx "\n", COMET_PERIPHERALS_BASE + addr);
        return 0;
    }
}

static void comet_peripheral_io_write(void *opaque, hwaddr addr,
                                      uint64_t val, unsigned size)
{
    qemu_log_mask(LOG_UNIMP, "comet: unhandled perip write 0x%08"
                  HWADDR_PRIx " = 0x%08" PRIx64 "\n",
                  COMET_PERIPHERALS_BASE + addr, val);
}

static const MemoryRegionOps comet_peripherals_io_ops = {
    .read = comet_peripheral_io_read,
    .write = comet_peripheral_io_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void comet_setup_direct_boot(CPUArchState *env, void *opaque)
{
    CometState *s = opaque;
    MetaCore *core = META_THREAD2CORE(env);

    /*
     * Report an XTAL1 frequency of exactly 24MHz, and set the timer divider to
     * divide it down to exactly 1MHz (the timer frequency that QEMU provides).
     */
    s->reset_cfg = (COMET_DEFAULT_RESET_CFG & ~0x00000f00) | 2 << 8;
    core->expand.timer_div = 23;
}

static void comet_ram(MemoryRegion *mr, const char *name, hwaddr base,
                      uint64_t size)
{
    memory_region_init_ram(mr, NULL, name, size, &error_fatal);
    memory_region_add_subregion(get_system_memory(), base, mr);
}

static void zero1sp_init(MachineState *machine)
{
    MemoryRegion *sysmem = get_system_memory();
    CometState *s = g_new0(CometState, 1);
    hwaddr addr;
    int i;

    if (machine->ram_size > COMET_SDRAM_SIZE) {
        error_report("Comet has at most %" PRIu64 " MiB of SDRAM",
                     (uint64_t)COMET_SDRAM_SIZE / MiB);
        exit(1);
    }

    s->reset_cfg = COMET_DEFAULT_RESET_CFG;
    s->core = cpu_meta_core_init("harrier", COMET_IRQ_MAX);
    if (!s->core) {
        error_report("Comet SoC failed to init");
        exit(1);
    }

    memory_region_init_io(&s->peripherals, NULL, &comet_peripherals_io_ops, s,
                          "comet.peripherals", COMET_PERIPHERALS_SIZE);
    memory_region_add_subregion(sysmem, COMET_PERIPHERALS_BASE,
                                &s->peripherals);

    create_unimplemented_device("comet.scb", 0x02004400, 0x700);
    create_unimplemented_device("comet.uart0", 0x02004b00, 0x100);
    create_unimplemented_device("comet.uart1", 0x02004c00, 0x100);
    create_unimplemented_device("comet.spim1", 0x02004e00, 0x100);
    create_unimplemented_device("comet.i2s", 0x02004f00, 0x200);
    create_unimplemented_device("comet.sdio", 0x02005400, 0x200);
    create_unimplemented_device("comet.gpio", 0x02005800, 0x200);
    create_unimplemented_device("comet.pdc", 0x02006000, 0x1000);
    create_unimplemented_device("comet.pdp", 0x02008000, 0x800);
    create_unimplemented_device("comet.pdi", 0x02008800, 0x100);
    create_unimplemented_device("comet.hep", 0x02008c00, 0x400);
    create_unimplemented_device("comet.sys_inf", 0x02009000, 0x100);
    create_unimplemented_device("comet.mdc", 0x0200c000, 0x1000);
    create_unimplemented_device("comet.ucc0", 0x02010000, 0x3000);
    create_unimplemented_device("comet.ucc1", 0x02014000, 0x3000);
    create_unimplemented_device("comet.usb", 0x02020000, 0x10000);
    create_unimplemented_device("comet.sys_event", 0x04000000, 0x400000);

    comet_ram(&s->rom, "meta_core_rom.ram", COMET_ROM_BASE, COMET_ROM_SIZE);

    /* core code and data RAM, repeated up to the end of their window */
    comet_ram(&s->core_code[0], "meta_code.ram", COMET_CORECODE_BASE,
              COMET_CORECODE_SIZE);
    for (addr = COMET_CORECODE_BASE + COMET_CORECODE_SIZE, i = 1;
         addr < COMET_CORECODE_END; addr += COMET_CORECODE_SIZE, i++) {
        memory_region_init_alias(&s->core_code[i], NULL, "meta_code.ram",
                                 &s->core_code[0], 0, COMET_CORECODE_SIZE);
        memory_region_add_subregion(sysmem, addr, &s->core_code[i]);
    }
    memory_region_init_alias(&s->core_code_slave, NULL, "meta_code_slave",
                             &s->core_code[0], 0, COMET_CORECODE_SIZE);
    memory_region_add_subregion(sysmem, COMET_CORECODE_SLAVE_BASE,
                                &s->core_code_slave);

    comet_ram(&s->core_data[0], "meta_data.ram", COMET_COREDATA_BASE,
              COMET_COREDATA_SIZE);
    for (addr = COMET_COREDATA_BASE + COMET_COREDATA_SIZE, i = 1;
         addr < COMET_COREDATA_END; addr += COMET_COREDATA_SIZE, i++) {
        memory_region_init_alias(&s->core_data[i], NULL, "meta_data.ram",
                                 &s->core_data[0], 0, COMET_COREDATA_SIZE);
        memory_region_add_subregion(sysmem, addr, &s->core_data[i]);
    }
    memory_region_init_alias(&s->core_data_slave, NULL, "meta_data_slave",
                             &s->core_data[0], 0, COMET_COREDATA_SIZE);
    memory_region_add_subregion(sysmem, COMET_COREDATA_SLAVE_BASE,
                                &s->core_data_slave);

    /* register core memories with core */
    cpu_meta_core_mem_init(s->core, META_CORE_MEM_CODE,
                           COMET_CORECODE_BASE, COMET_CORECODE_SIZE);
    cpu_meta_core_mem_init(s->core, META_CORE_MEM_DATA,
                           COMET_COREDATA_BASE, COMET_COREDATA_SIZE);

    comet_ram(&s->internal, "meta_internal.ram", COMET_INTRAM_BASE,
              COMET_INTRAM_SIZE);
    comet_ram(&s->ucc0_mtxsram, "meta_ucc0_mtx.ram", COMET_UCC0_MTXSRAM_BASE,
              COMET_UCC0_MTXSRAM_SIZE);
    comet_ram(&s->ucc0_mcpsram16, "meta_ucc0_mcp16.ram",
              COMET_UCC0_MCPSRAM16_BASE, COMET_UCC0_MCPSRAM16_SIZE);
    comet_ram(&s->ucc0_mcpsram24, "meta_ucc0_mcp24.ram",
              COMET_UCC0_MCPSRAM24_BASE, COMET_UCC0_MCPSRAM24_SIZE);
    comet_ram(&s->ucc1_mtxsram, "meta_ucc1_mtx.ram", COMET_UCC1_MTXSRAM_BASE,
              COMET_UCC1_MTXSRAM_SIZE);

    memory_region_add_subregion(sysmem, COMET_SDRAM_BASE, machine->ram);

    /* boot Linux directly */
    s->boot.kernel_filename = machine->kernel_filename;
    s->boot.kernel_cmdline = machine->kernel_cmdline;
    s->boot.initrd_filename = machine->initrd_filename;
    s->boot.ram_phys = COMET_SDRAM_BASE;
    s->boot.ram_size = machine->ram_size;
    s->boot.flags |= META_BOOT_MMU_HTP;
    s->boot.setup_direct_boot = comet_setup_direct_boot;
    s->boot.setup_direct_boot_opaque = s;
    meta_setup_boot(s->core, &s->boot);
}

static void zero1sp_machine_init(MachineClass *mc)
{
    mc->desc = "Comet METAmorph development platform (TZ1090)";
    mc->alias = "01sp";
    mc->init = zero1sp_init;
    mc->is_default = true;
    mc->default_ram_size = 256 * MiB;
    mc->default_ram_id = "comet.sdram";
    mc->max_cpus = META_MAX_THREADS;
}

DEFINE_MACHINE("comet", zero1sp_machine_init)
