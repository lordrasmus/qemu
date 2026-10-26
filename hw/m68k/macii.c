/*
 * QEMU Macintosh II / IIx System Emulator
 *
 * The Mac II (68020 + 68851 PMMU + 68881) and the Mac IIx (68030 + 68882)
 * share one logic board design: two real VIAs, a Z8530 SCC, an NCR 5380
 * SCSI controller, the Apple Sound Chip and six NuBus slots. This board
 * models what Linux needs from it ("macintosh_config" entries MAC_MODEL_II
 * and MAC_MODEL_IIX in arch/m68k/mac/config.c); -cpu m68020 gives a Mac II,
 * -cpu m68030 a Mac IIx.
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
#include "qemu/units.h"
#include "qemu/guest-random.h"
#include "exec/target_page.h"
#include "system/system.h"
#include "target/m68k/cpu.h"
#include "hw/core/boards.h"
#include "hw/core/or-irq.h"
#include "hw/core/irq.h"
#include "elf.h"
#include "hw/core/loader.h"
#include "ui/console.h"
#include "hw/char/escc.h"
#include "hw/core/sysbus.h"
#include "hw/scsi/ncr5380.h"
#include "standard-headers/asm-m68k/bootinfo.h"
#include "standard-headers/asm-m68k/bootinfo-mac.h"
#include "bootinfo.h"
#include "hw/m68k/q800-glue.h"
#include "hw/misc/mac_via.h"
#include "hw/input/adb.h"
#include "hw/audio/asc.h"
#include "hw/nubus/mac-nubus-bridge.h"
#include "hw/display/macfb.h"
#include "hw/block/swim.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "system/reset.h"

#define MACROM_ADDR     0x40800000

/* I/O space, decoded in 128 KiB and repeated up to 0x50000000 + 32 MiB */
#define IO_BASE               0x50000000
#define IO_SLICE              0x00020000
#define IO_SLICE_MASK         (IO_SLICE - 1)
#define IO_SIZE               0x02000000

#define VIA_BASE              (IO_BASE + 0x00000)
#define SCC_BASE              (IO_BASE + 0x04000)
#define SCSI_DMA_BASE         (IO_BASE + 0x06000)
#define SCSI_BASE             (IO_BASE + 0x10000)
#define ASC_BASE              (IO_BASE + 0x14000)
#define SWIM_BASE             (IO_BASE + 0x16000)

/* the framebuffer card in NuBus slot 9 */
#define VIDEO_BASE            0xf9000000

#define MAC_CLOCK  3686418

/* Size of the RAM decode area */
#define RAM_SIZE              0x40000000

#define MACII_NUBUS_SLOTS_AVAILABLE   (BIT(0x9) | BIT(0xa) | BIT(0xb) | \
                                       BIT(0xc) | BIT(0xd) | BIT(0xe))

#define TYPE_MACII_MACHINE MACHINE_TYPE_NAME("macii")
OBJECT_DECLARE_SIMPLE_TYPE(MacIIMachineState, MACII_MACHINE)

struct MacIIMachineState {
    MachineState parent_obj;

    M68kCPU cpu;
    MemoryRegion rom;
    GLUEState glue;
    MOS6522Q800VIA1State via1;
    MOS6522Q800VIA2State via2;
    ESCCState escc;
    OrIRQState escc_orgate;
    NCR5380State ncr5380;
    Swim swim;
    MacNubusBridge mac_nubus_bridge;
    MacfbNubusState macfb;
    ASCState asc;
    MemoryRegion ramio;
    MemoryRegion macio;
    MemoryRegion macio_alias;
};

static void main_cpu_reset(void *opaque)
{
    M68kCPU *cpu = opaque;
    CPUState *cs = CPU(cpu);

    cpu_reset(cs);
    cpu->env.aregs[7] = ldl_phys(cs->as, 0);
    cpu->env.pc = ldl_phys(cs->as, 4);
}

static void rerandomize_rng_seed(void *opaque)
{
    struct bi_record *rng_seed = opaque;
    qemu_guest_getrandom_nofail((void *)rng_seed->data + 2,
                                be16_to_cpu(*(uint16_t *)rng_seed->data));
}

/* mac_reset() jumps to ROM offset 0xa; power off through VIA2 like a ROM */
static uint8_t fake_mac_rom[] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,

    /* via2[vDirB] |= VIA2B_vPower */
    0x20, 0x7C, 0x50, 0xF0, 0x24, 0x00, /* moveal VIA2_BASE+vDirB,%a0 */
    0x10, 0x10,                         /* moveb %a0@,%d0 */
    0x00, 0x00, 0x00, 0x04,             /* orib #4,%d0 */
    0x10, 0x80,                         /* moveb %d0,%a0@ */

    /* via2[vBufB] &= ~VIA2B_vPower */
    0x20, 0x7C, 0x50, 0xF0, 0x20, 0x00, /* moveal VIA2_BASE+vBufB,%a0 */
    0x10, 0x10,                         /* moveb %a0@,%d0 */
    0x02, 0x00, 0xFF, 0xFB,             /* andib #-5,%d0 */
    0x10, 0x80,                         /* moveb %d0,%a0@ */

    /* while (true) ; */
    0x60, 0xFE                          /* bras [self] */
};

static MemTxResult macio_alias_read(void *opaque, hwaddr addr, uint64_t *data,
                                    unsigned size, MemTxAttrs attrs)
{
    MemTxResult r;
    uint32_t val;

    addr &= IO_SLICE_MASK;
    addr |= IO_BASE;

    switch (size) {
    case 4:
        val = address_space_ldl_be(&address_space_memory, addr, attrs, &r);
        break;
    case 2:
        val = address_space_lduw_be(&address_space_memory, addr, attrs, &r);
        break;
    case 1:
        val = address_space_ldub(&address_space_memory, addr, attrs, &r);
        break;
    default:
        g_assert_not_reached();
    }

    *data = val;
    return r;
}

static MemTxResult macio_alias_write(void *opaque, hwaddr addr, uint64_t value,
                                     unsigned size, MemTxAttrs attrs)
{
    MemTxResult r;

    addr &= IO_SLICE_MASK;
    addr |= IO_BASE;

    switch (size) {
    case 4:
        address_space_stl_be(&address_space_memory, addr, value, attrs, &r);
        break;
    case 2:
        address_space_stw_be(&address_space_memory, addr, value, attrs, &r);
        break;
    case 1:
        address_space_stb(&address_space_memory, addr, value, attrs, &r);
        break;
    default:
        g_assert_not_reached();
    }

    return r;
}

static const MemoryRegionOps macio_alias_ops = {
    .read_with_attrs = macio_alias_read,
    .write_with_attrs = macio_alias_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static uint64_t ramio_read(void *opaque, hwaddr addr, unsigned size)
{
    return 0x0;
}

static void ramio_write(void *opaque, hwaddr addr, uint64_t val,
                        unsigned size)
{
}

static const MemoryRegionOps ramio_ops = {
    .read = ramio_read,
    .write = ramio_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void macii_machine_init(MachineState *machine)
{
    MacIIMachineState *m = MACII_MACHINE(machine);
    M68kCPUClass *cc;
    uint64_t elf_entry;
    int32_t kernel_size;
    ram_addr_t initrd_base;
    int32_t initrd_size;
    const MacFbMode *macfb_mode;
    ram_addr_t ram_size = machine->ram_size;
    const char *kernel_filename = machine->kernel_filename;
    const char *initrd_filename = machine->initrd_filename;
    const char *kernel_cmdline = machine->kernel_cmdline;
    hwaddr parameters_base;
    CPUState *cs;
    DeviceState *dev;
    SysBusDevice *sysbus;
    BusState *adb_bus;
    NubusBus *nubus;
    DriveInfo *dinfo;
    uint8_t rng_seed[32];
    bool is_030;
    int i;

    if (!kernel_filename) {
        error_report("macii needs a Linux kernel (-kernel), there is no ROM");
        exit(1);
    }

    if (ram_size > 1 * GiB) {
        error_report("Too much memory for this machine: %" PRId64 " MiB, "
                     "maximum 1024 MiB", ram_size / MiB);
        exit(1);
    }

    /* init CPU */
    object_initialize_child(OBJECT(machine), "cpu", &m->cpu, machine->cpu_type);
    qdev_realize(DEVICE(&m->cpu), NULL, &error_fatal);
    qemu_register_reset(main_cpu_reset, &m->cpu);
    cc = M68K_CPU_GET_CLASS(&m->cpu);
    is_030 = !strcmp(object_class_get_name(OBJECT_CLASS(cc)),
                     M68K_CPU_TYPE_NAME("m68030"));

    /* RAM */
    memory_region_init_io(&m->ramio, OBJECT(machine), &ramio_ops, &m->ramio,
                          "ram", RAM_SIZE);
    memory_region_add_subregion(get_system_memory(), 0x0, &m->ramio);
    memory_region_add_subregion(&m->ramio, 0, machine->ram);

    /* I/O devices, repeated every IO_SLICE */
    memory_region_init(&m->macio, OBJECT(machine), "mac-io", IO_SLICE);
    memory_region_add_subregion(get_system_memory(), IO_BASE, &m->macio);

    memory_region_init_io(&m->macio_alias, OBJECT(machine), &macio_alias_ops,
                          &m->macio, "mac-io.alias", IO_SIZE - IO_SLICE);
    memory_region_add_subregion(get_system_memory(), IO_BASE + IO_SLICE,
                                &m->macio_alias);

    /* IRQ routing: VIA1 level 1, VIA2 level 2, SCC level 4, NMI level 7 */
    object_initialize_child(OBJECT(machine), "glue", &m->glue, TYPE_GLUE);
    object_property_set_link(OBJECT(&m->glue), "cpu", OBJECT(&m->cpu),
                             &error_abort);
    object_property_set_bool(OBJECT(&m->glue), "classic-only", true,
                             &error_abort);
    sysbus_realize(SYS_BUS_DEVICE(&m->glue), &error_fatal);

    /* VIA 1: ADB, RTC/PRAM, 60 Hz and one second ticks */
    object_initialize_child(OBJECT(machine), "via1", &m->via1,
                            TYPE_MOS6522_Q800_VIA1);
    dinfo = drive_get(IF_MTD, 0, 0);
    if (dinfo) {
        qdev_prop_set_drive(DEVICE(&m->via1), "drive",
                            blk_by_legacy_dinfo(dinfo));
    }
    sysbus = SYS_BUS_DEVICE(&m->via1);
    sysbus_realize(sysbus, &error_fatal);
    memory_region_add_subregion(&m->macio, VIA_BASE - IO_BASE,
                                sysbus_mmio_get_region(sysbus, 1));
    sysbus_connect_irq(sysbus, 0,
                       qdev_get_gpio_in(DEVICE(&m->glue), GLUE_IRQ_IN_VIA1));

    adb_bus = qdev_get_child_bus(DEVICE(&m->via1), "adb.0");
    dev = qdev_new(TYPE_ADB_KEYBOARD);
    qdev_realize_and_unref(dev, adb_bus, &error_fatal);
    dev = qdev_new(TYPE_ADB_MOUSE);
    qdev_realize_and_unref(dev, adb_bus, &error_fatal);

    /* VIA 2: NuBus and SCSI interrupts, soft power */
    object_initialize_child(OBJECT(machine), "via2", &m->via2,
                            TYPE_MOS6522_Q800_VIA2);
    sysbus = SYS_BUS_DEVICE(&m->via2);
    sysbus_realize(sysbus, &error_fatal);
    memory_region_add_subregion(&m->macio, VIA_BASE - IO_BASE + VIA_SIZE,
                                sysbus_mmio_get_region(sysbus, 1));
    sysbus_connect_irq(sysbus, 0,
                       qdev_get_gpio_in(DEVICE(&m->glue), GLUE_IRQ_IN_VIA2));

    /* SCC */
    object_initialize_child(OBJECT(machine), "escc", &m->escc, TYPE_ESCC);
    dev = DEVICE(&m->escc);
    qdev_prop_set_uint32(dev, "disabled", 0);
    qdev_prop_set_uint32(dev, "frequency", MAC_CLOCK);
    qdev_prop_set_uint32(dev, "it_shift", 1);
    qdev_prop_set_bit(dev, "bit_swap", true);
    qdev_prop_set_chr(dev, "chrA", serial_hd(0));
    qdev_prop_set_chr(dev, "chrB", serial_hd(1));
    qdev_prop_set_uint32(dev, "chnBtype", 0);
    qdev_prop_set_uint32(dev, "chnAtype", 0);
    sysbus = SYS_BUS_DEVICE(dev);
    sysbus_realize(sysbus, &error_fatal);

    object_initialize_child(OBJECT(machine), "escc_orgate", &m->escc_orgate,
                            TYPE_OR_IRQ);
    object_property_set_int(OBJECT(&m->escc_orgate), "num-lines", 2,
                            &error_fatal);
    dev = DEVICE(&m->escc_orgate);
    qdev_realize(dev, NULL, &error_fatal);
    sysbus_connect_irq(sysbus, 0, qdev_get_gpio_in(dev, 0));
    sysbus_connect_irq(sysbus, 1, qdev_get_gpio_in(dev, 1));
    qdev_connect_gpio_out(dev, 0,
                          qdev_get_gpio_in(DEVICE(&m->glue),
                                           GLUE_IRQ_IN_ESCC));
    memory_region_add_subregion(&m->macio, SCC_BASE - IO_BASE,
                                sysbus_mmio_get_region(sysbus, 0));

    /* SCSI: NCR 5380, IRQ and DRQ are negative edge triggered on VIA2 */
    object_initialize_child(OBJECT(machine), "ncr5380", &m->ncr5380,
                            TYPE_NCR5380);
    sysbus = SYS_BUS_DEVICE(&m->ncr5380);
    sysbus_realize(sysbus, &error_fatal);
    sysbus_connect_irq(sysbus, 0,
                       qemu_irq_invert(qdev_get_gpio_in(DEVICE(&m->via2),
                                                        VIA2_IRQ_SCSI_BIT)));
    sysbus_connect_irq(sysbus, 1,
                       qemu_irq_invert(qdev_get_gpio_in(DEVICE(&m->via2),
                                                     VIA2_IRQ_SCSI_DATA_BIT)));
    memory_region_add_subregion(&m->macio, SCSI_BASE - IO_BASE,
                                sysbus_mmio_get_region(sysbus, 0));
    memory_region_add_subregion(&m->macio, SCSI_DMA_BASE - IO_BASE,
                                sysbus_mmio_get_region(sysbus, 1));
    scsi_bus_legacy_handle_cmdline(&m->ncr5380.bus);

    /* Apple Sound Chip */
    object_initialize_child(OBJECT(machine), "asc", &m->asc, TYPE_ASC);
    qdev_prop_set_uint8(DEVICE(&m->asc), "asctype", ASC_TYPE_ASC);
    if (machine->audiodev) {
        qdev_prop_set_string(DEVICE(&m->asc), "audiodev", machine->audiodev);
    }
    sysbus = SYS_BUS_DEVICE(&m->asc);
    sysbus_realize_and_unref(sysbus, &error_fatal);
    memory_region_add_subregion(&m->macio, ASC_BASE - IO_BASE,
                                sysbus_mmio_get_region(sysbus, 0));
    sysbus_connect_irq(sysbus, 0, qdev_get_gpio_in(DEVICE(&m->glue),
                                                   GLUE_IRQ_IN_ASC));
    qdev_connect_gpio_out(DEVICE(&m->glue), GLUE_IRQ_ASC,
                          qdev_get_gpio_in(DEVICE(&m->via2),
                                           VIA2_IRQ_ASC_BIT));

    /* floppy: IWM on the Mac II, SWIM on the IIx; Linux only knows SWIM */
    object_initialize_child(OBJECT(machine), "swim", &m->swim, TYPE_SWIM);
    sysbus = SYS_BUS_DEVICE(&m->swim);
    sysbus_realize(sysbus, &error_fatal);
    memory_region_add_subregion(&m->macio, SWIM_BASE - IO_BASE,
                                sysbus_mmio_get_region(sysbus, 0));

    /* NuBus, slots 9 to E */
    object_initialize_child(OBJECT(machine), "mac-nubus-bridge",
                            &m->mac_nubus_bridge, TYPE_MAC_NUBUS_BRIDGE);
    sysbus = SYS_BUS_DEVICE(&m->mac_nubus_bridge);
    dev = DEVICE(&m->mac_nubus_bridge);
    qdev_prop_set_uint32(dev, "slot-available-mask",
                         MACII_NUBUS_SLOTS_AVAILABLE);
    sysbus_realize(sysbus, &error_fatal);
    memory_region_add_subregion(get_system_memory(),
                                MAC_NUBUS_FIRST_SLOT * NUBUS_SUPER_SLOT_SIZE,
                                sysbus_mmio_get_region(sysbus, 0));
    memory_region_add_subregion(get_system_memory(),
                                NUBUS_SLOT_BASE +
                                MAC_NUBUS_FIRST_SLOT * NUBUS_SLOT_SIZE,
                                sysbus_mmio_get_region(sysbus, 1));
    for (i = 0; i < VIA2_NUBUS_IRQ_INTVIDEO; i++) {
        qdev_connect_gpio_out(dev, 9 + i,
                              qdev_get_gpio_in_named(DEVICE(&m->via2),
                                                     "nubus-irq",
                                                     VIA2_NUBUS_IRQ_9 + i));
    }
    nubus = NUBUS_BUS(qdev_get_child_bus(dev, "nubus-bus.0"));

    /* video card in slot 9 */
    object_initialize_child(OBJECT(machine), "macfb", &m->macfb,
                            TYPE_NUBUS_MACFB);
    dev = DEVICE(&m->macfb);
    qdev_prop_set_uint32(dev, "slot", 9);
    qdev_prop_set_uint32(dev, "width", graphic_width ?: 640);
    qdev_prop_set_uint32(dev, "height", graphic_height ?: 480);
    qdev_prop_set_uint8(dev, "depth", graphic_depth ?: 8);
    qdev_realize(dev, BUS(nubus), &error_fatal);
    macfb_mode = (NUBUS_MACFB(dev)->macfb).mode;

    /* Linux direct boot */
    cs = CPU(&m->cpu);
    {
        uint64_t high;
        void *param_blob, *param_ptr, *param_rng_seed;

        param_blob = g_malloc((kernel_cmdline ? strlen(kernel_cmdline) : 0) +
                              1024);

        kernel_size = load_elf(kernel_filename, NULL, NULL, NULL,
                               &elf_entry, NULL, &high, NULL, ELFDATA2MSB,
                               EM_68K, 0, 0);
        if (kernel_size < 0) {
            error_report("could not load kernel '%s'", kernel_filename);
            exit(1);
        }
        stl_phys(cs->as, 4, elf_entry); /* reset initial PC */
        parameters_base = (high + 1) & ~1;
        param_ptr = param_blob;

        BOOTINFO1(param_ptr, BI_MACHTYPE, MACH_MAC);
        if (is_030) {
            BOOTINFO1(param_ptr, BI_FPUTYPE, FPU_68882);
            BOOTINFO1(param_ptr, BI_MMUTYPE, MMU_68030);
            BOOTINFO1(param_ptr, BI_CPUTYPE, CPU_68030);
            BOOTINFO1(param_ptr, BI_MAC_CPUID, CPUB_68030);
            BOOTINFO1(param_ptr, BI_MAC_MODEL, MAC_MODEL_IIX);
        } else {
            BOOTINFO1(param_ptr, BI_FPUTYPE, FPU_68881);
            BOOTINFO1(param_ptr, BI_MMUTYPE, MMU_68851);
            BOOTINFO1(param_ptr, BI_CPUTYPE, CPU_68020);
            BOOTINFO1(param_ptr, BI_MAC_CPUID, CPUB_68020);
            BOOTINFO1(param_ptr, BI_MAC_MODEL, MAC_MODEL_II);
        }
        BOOTINFO1(param_ptr, BI_MAC_MEMSIZE, ram_size >> 20); /* in MB */
        BOOTINFO2(param_ptr, BI_MEMCHUNK, 0, ram_size);
        BOOTINFO1(param_ptr, BI_MAC_VADDR, VIDEO_BASE + macfb_mode->offset);
        BOOTINFO1(param_ptr, BI_MAC_VDEPTH, macfb_mode->depth);
        BOOTINFO1(param_ptr, BI_MAC_VDIM,
                  (macfb_mode->height << 16) | macfb_mode->width);
        BOOTINFO1(param_ptr, BI_MAC_VROW, macfb_mode->stride);
        BOOTINFO1(param_ptr, BI_MAC_SCCBASE, SCC_BASE);

        memory_region_init_ram_ptr(&m->rom, NULL, "m68k_fake_mac.rom",
                                   sizeof(fake_mac_rom), fake_mac_rom);
        memory_region_set_readonly(&m->rom, true);
        memory_region_add_subregion(get_system_memory(), MACROM_ADDR, &m->rom);

        if (kernel_cmdline) {
            BOOTINFOSTR(param_ptr, BI_COMMAND_LINE, kernel_cmdline);
        }

        param_rng_seed = param_ptr;
        qemu_guest_getrandom_nofail(rng_seed, sizeof(rng_seed));
        BOOTINFODATA(param_ptr, BI_RNG_SEED, rng_seed, sizeof(rng_seed));

        if (initrd_filename) {
            initrd_size = get_image_size(initrd_filename, NULL);
            if (initrd_size < 0) {
                error_report("could not load initial ram disk '%s'",
                             initrd_filename);
                exit(1);
            }
            initrd_base = (ram_size - initrd_size) & TARGET_PAGE_MASK;
            load_image_targphys(initrd_filename, initrd_base,
                                ram_size - initrd_base, &error_fatal);
            BOOTINFO2(param_ptr, BI_RAMDISK, initrd_base, initrd_size);
        }
        BOOTINFO0(param_ptr, BI_LAST);
        rom_add_blob_fixed_as("bootinfo", param_blob, param_ptr - param_blob,
                              parameters_base, cs->as);
        qemu_register_reset_nosnapshotload(rerandomize_rng_seed,
                            rom_ptr_for_as(cs->as, parameters_base,
                                           param_ptr - param_blob) +
                            (param_rng_seed - param_blob));
        g_free(param_blob);
    }
}

static void macii_machine_class_init(ObjectClass *oc, const void *data)
{
    static const char * const valid_cpu_types[] = {
        M68K_CPU_TYPE_NAME("m68020"),
        M68K_CPU_TYPE_NAME("m68030"),
        NULL
    };
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Macintosh II (-cpu m68020) / IIx (-cpu m68030)";
    mc->init = macii_machine_init;
    mc->default_cpu_type = M68K_CPU_TYPE_NAME("m68030");
    mc->valid_cpu_types = valid_cpu_types;
    mc->max_cpus = 1;
    mc->block_default_type = IF_SCSI;
    mc->default_ram_id = "m68k_mac.ram";
    mc->default_ram_size = 128 * MiB;
    machine_add_audiodev_property(mc);
}

static const TypeInfo macii_machine_typeinfo = {
    .name       = TYPE_MACII_MACHINE,
    .parent     = TYPE_MACHINE,
    .instance_size = sizeof(MacIIMachineState),
    .class_init = macii_machine_class_init,
};

static void macii_machine_register_types(void)
{
    type_register_static(&macii_machine_typeinfo);
}

type_init(macii_machine_register_types)
