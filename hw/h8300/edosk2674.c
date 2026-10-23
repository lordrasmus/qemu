/*
 * edosk2674 emulation
 *
 * Copyright (c) 2019 Yoshinori Sato
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "cpu.h"
#include "hw/core/sysbus.h"
#include "hw/core/loader.h"
#include "hw/block/flash.h"
#include "hw/net/smc91c111.h"
#include "hw/h8300/h8s2674.h"
#include "system/system.h"
#include "system/qtest.h"
#include "system/device_tree.h"
#include "hw/core/boards.h"

#define DRAM_BASE 0x00400000
#define DTB_ADDR (DRAM_BASE + machine->ram_size - dtb_size)

static void setup_vector(unsigned int base, PFlashCFI01 *fl)
{
    uint32_t rom_vec[128];
    int i;

    for (i = 0; i < ARRAY_SIZE(rom_vec); i++) {
        rom_vec[i] = cpu_to_be32(base + i * 4);
    }

    /*
     * The vector table lives at address 0, which is where the boot flash
     * is mapped.  Write it into the flash storage directly: going through
     * the memory bus would hand the words to the CFI command decoder, and
     * the chip would come up in some undefined state instead of read-array
     * mode -- with a -pflash image that breaks the CFI probe outright.
     */
    if (fl) {
        memcpy(memory_region_get_ram_ptr(pflash_cfi01_get_memory(fl)),
               rom_vec, sizeof(rom_vec));
    } else {
        rom_add_blob_fixed("vector", rom_vec, sizeof(rom_vec), 0x000000);
    }
}


static void edosk2674_init(MachineState *machine)
{
    H8S2674State *s = g_new(H8S2674State, 1);
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *sdram = g_new(MemoryRegion, 1);
    const char *kernel_filename = machine->kernel_filename;
    const char *dtb_filename = machine->dtb;
    void *dtb = NULL;
    int dtb_size;
    DriveInfo *dinfo;
    PFlashCFI01 *flash;

    /* Allocate memory space */
    memory_region_init_ram(sdram, NULL, "sdram", machine->ram_size,
                           &error_fatal);
    memory_region_add_subregion(sysmem, DRAM_BASE, sdram);
    dinfo = drive_get(IF_PFLASH, 0, 0);
    flash = pflash_cfi01_register(0x0, "edosk2674.flash", 4 * MiB,
                          dinfo ? blk_by_legacy_dinfo(dinfo) : NULL,
                          128 * KiB, 2, 0x0089, 0x0016, 0x0000, 0x0000,
                          0);

    if (!kernel_filename && machine->firmware) {
        rom_add_file_fixed(machine->firmware, 0, 0);
    }

    /* Initialize MCU */
    object_initialize_child(OBJECT(machine), "mcu", s, TYPE_H8S2674);
    object_property_set_link(OBJECT(s), "memory", OBJECT(sysmem),
                             &error_abort);
    object_property_set_uint(OBJECT(s), "clock-freq", 33333333,
                             &error_abort);
    object_property_set_uint(OBJECT(s), "console", 2, &error_abort);
    sysbus_realize(SYS_BUS_DEVICE(s), &error_abort);

    smc91c96_init(0xf80000, s->irq[16]);

    /* Load kernel and dtb */
    if (kernel_filename) {
        h8300_load_image(H8300_CPU(first_cpu), kernel_filename,
                      DRAM_BASE, machine->ram_size);
        setup_vector(0xffc000 - 0x200, flash);
        if (dtb_filename) {
            dtb = load_device_tree(dtb_filename, &dtb_size);
            if (dtb == NULL) {
                fprintf(stderr, "Couldn't open dtb file %s\n", dtb_filename);
                exit(1);
            }
            if (machine->kernel_cmdline &&
                qemu_fdt_setprop_string(dtb, "/chosen", "bootargs",
                                        machine->kernel_cmdline) < 0) {
                fprintf(stderr, "couldn't set /chosen/bootargs\n");
                exit(1);
            }
	    rom_add_blob_fixed("dtb", dtb, dtb_size, DTB_ADDR);
	    H8300_CPU(first_cpu)->env.regs[0] = DTB_ADDR;
        }
    }
}

static void edosk2674_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "EDOSK2674";
    mc->init = edosk2674_init;
    mc->is_default = 0;
    mc->default_cpu_type = TYPE_H8S2674_CPU;
    mc->default_ram_size = 8 * MiB;
}

static const TypeInfo edosk2674_type = {
    .name = MACHINE_TYPE_NAME("edosk2674"),
    .parent = TYPE_MACHINE,
    .class_init = edosk2674_class_init,
};

static void edosk2674_machine_init(void)
{
    type_register_static(&edosk2674_type);
}

type_init(edosk2674_machine_init)
