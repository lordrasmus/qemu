/*
 *  QEMU model for the Milkymist board.
 *
 *  Copyright (c) 2010 Michael Walle <michael@walle.cc>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/error-report.h"
#include "cpu.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "system/system.h"
#include "system/qtest.h"
#include "system/reset.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "elf.h"
#include "milkymist-hw.h"
#include "hw/misc/unimp.h"
#include "lm32.h"
#include "system/address-spaces.h"
#include "qemu/cutils.h"


typedef struct {
    LM32CPU *cpu;
    hwaddr bootstrap_pc;
    hwaddr flash_base;
    hwaddr initrd_base;
    size_t initrd_size;
    hwaddr cmdline_base;
} ResetInfo;

static void cpu_irq_handler(void *opaque, int irq, int level)
{
    LM32CPU *cpu = opaque;
    CPUState *cs = CPU(cpu);

    if (level) {
        cpu_interrupt(cs, CPU_INTERRUPT_HARD);
    } else {
        cpu_reset_interrupt(cs, CPU_INTERRUPT_HARD);
    }
}

static void main_cpu_reset(void *opaque)
{
    ResetInfo *reset_info = opaque;
    CPULM32State *env = &reset_info->cpu->env;

    cpu_reset(CPU(reset_info->cpu));

    /* init defaults */
    env->pc = reset_info->bootstrap_pc;
    env->regs[R_R1] = reset_info->cmdline_base;
    env->regs[R_R2] = reset_info->initrd_base;
    env->regs[R_R3] = reset_info->initrd_base + reset_info->initrd_size;
    env->eba = reset_info->flash_base;
    env->deba = reset_info->flash_base;
}

static void
milkymist_init(MachineState *machine)
{
    MachineClass *mc = MACHINE_GET_CLASS(machine);
    const char *kernel_filename = machine->kernel_filename;
    const char *kernel_cmdline = machine->kernel_cmdline;
    const char *initrd_filename = machine->initrd_filename;
    LM32CPU *cpu;
    CPULM32State *env;
    int kernel_size;
    MemoryRegion *address_space_mem = get_system_memory();
    qemu_irq irq[32];
    int i;
    ResetInfo *reset_info;

    if (machine->ram_size != mc->default_ram_size) {
        char *sz = size_to_str(mc->default_ram_size);
        error_report("Invalid RAM size, should be %s", sz);
        g_free(sz);
        exit(EXIT_FAILURE);
    }

    /* memory map */
    hwaddr sdram_base   = 0x40000000;

    hwaddr initrd_base  = sdram_base + 0x1002000;
    hwaddr cmdline_base = sdram_base + 0x1000000;
    size_t initrd_max = machine->ram_size - 0x1002000;

    reset_info = g_malloc0(sizeof(ResetInfo));

    cpu = LM32_CPU(cpu_create(machine->cpu_type));

    env = &cpu->env;
    reset_info->cpu = cpu;

    cpu_lm32_set_phys_msb_ignore(env, 1);

    memory_region_add_subregion(address_space_mem, sdram_base, machine->ram);

    /* create irq lines */
    env->pic_state = lm32_pic_init(qemu_allocate_irq(cpu_irq_handler, cpu, 0));
    for (i = 0; i < 32; i++) {
        irq[i] = qdev_get_gpio_in(env->pic_state, i);
    }

    if (!kernel_filename && !qtest_enabled()) {
        error_report("milkymist: no BIOS, -kernel is required");
        exit(1);
    }

    milkymist_uart_create(0x60000000, irq[0], serial_hd(0));
    milkymist_sysctl_create(0x60001000, irq[1], irq[2], irq[3],
            80000000, 0x10014d31, 0x0000041f, 0x00000001);
    milkymist_hpdmc_create(0x60002000);
    /* the rest of the CSR bus; Linux leaves it alone */
    create_unimplemented_device("milkymist.vgafb", 0x60003000, 0x1000);
    create_unimplemented_device("milkymist.memcard", 0x60004000, 0x1000);
    create_unimplemented_device("milkymist.ac97", 0x60005000, 0x1000);
    create_unimplemented_device("milkymist.pfpu", 0x60006000, 0x1000);
    create_unimplemented_device("milkymist.tmu2", 0x60007000, 0x1000);
    create_unimplemented_device("milkymist.minimac2", 0x60008000, 0x1000);
    create_unimplemented_device("milkymist.softusb", 0x6000f000, 0x1000);

    /* make sure juart isn't the first chardev */
    env->juart_state = lm32_juart_init(serial_hd(1));

    if (kernel_filename) {
        uint64_t entry;

        /* Boots a kernel elf binary.  */
        kernel_size = load_elf(kernel_filename, NULL, NULL, NULL,
                               &entry, NULL, NULL, NULL,
                               ELFDATA2MSB, EM_LATTICEMICO32, 0, 0);
        reset_info->bootstrap_pc = entry;

        if (kernel_size < 0) {
            kernel_size = load_image_targphys(kernel_filename, sdram_base,
                                              machine->ram_size, NULL);
            reset_info->bootstrap_pc = sdram_base;
        }

        if (kernel_size < 0) {
            error_report("could not load kernel '%s'", kernel_filename);
            exit(1);
        }
    }

    if (kernel_cmdline && strlen(kernel_cmdline)) {
        pstrcpy_targphys("cmdline", cmdline_base, TARGET_PAGE_SIZE,
                kernel_cmdline);
        reset_info->cmdline_base = (uint32_t)cmdline_base;
    }

    if (initrd_filename) {
        size_t initrd_size;
        initrd_size = load_image_targphys(initrd_filename, initrd_base,
                initrd_max, &error_fatal);
        reset_info->initrd_base = (uint32_t)initrd_base;
        reset_info->initrd_size = (uint32_t)initrd_size;
    }

    qemu_register_reset(main_cpu_reset, reset_info);
}

static void milkymist_machine_init(MachineClass *mc)
{
    mc->desc = "Milkymist One";
    mc->init = milkymist_init;
    mc->default_cpu_type = LM32_CPU_TYPE_NAME("lm32-full");
    mc->default_ram_size = 128 * MiB;
    mc->default_ram_id = "milkymist.sdram";
}

DEFINE_MACHINE("milkymist", milkymist_machine_init)
