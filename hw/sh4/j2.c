/*
 * J-Core J2 SoC (SH-2 compatible, no MMU)
 *
 * The board Linux knows as "jcore,j2-soc": SDRAM at 0x10000000, the AIC1
 * interrupt controller with its PIT timer and a Xilinx UARTlite. The device
 * tree is generated here.
 *
 * -kernel takes a Linux vmlinux. r4 points at the device tree; the command
 * line and -initrd go into the boot parameters in the kernel's zero page,
 * which is the only place arch/sh looks for an initrd.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/error-report.h"
#include "qemu/cutils.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "elf.h"
#include "hw/core/boards.h"
#include "hw/core/irq.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/char/xilinx_uartlite.h"
#include "hw/misc/unimp.h"
#include "system/address-spaces.h"
#include "system/device_tree.h"
#include "system/reset.h"
#include "system/system.h"
#include "exec/cpu-interrupt.h"
#include "target/sh4/cpu.h"
#include <libfdt.h>

#define J2_SDRAM_BASE       0x10000000
#define J2_SDRAM_MAX        (512 * MiB)
#define J2_SOC_BASE         0xabcd0000
#define J2_SOC_SIZE         0x100000
#define J2_AIC_OFFSET       0x200
#define J2_UART_OFFSET      0x100
#define J2_UART_VECTOR      0x12
#define J2_PIT_VECTOR       0x48
#define J2_CPU_FREQ         50000000
#define J2_BUS_PERIOD_NS    (NANOSECONDS_PER_SECOND / J2_CPU_FREQ)

#define J2_ZERO_PAGE        0x1000      /* CONFIG_ZERO_PAGE_OFFSET */
#define J2_DTB_SIZE         (64 * KiB)

/*
 * AIC1 and the PIT share one register block. The interrupt number is the
 * CPU's vector number; the controller has no masking of its own. Inputs
 * are edge-triggered and the CPU taking the interrupt clears it.
 */
#define TYPE_JCORE_AIC "jcore-aic1"
OBJECT_DECLARE_SIMPLE_TYPE(JCoreAICState, JCORE_AIC)

#define JCORE_AIC_NVEC      128

#define REG_PITEN           0x00
#define REG_INTPRI          0x08
#define REG_THROT           0x10
#define REG_COUNT           0x14
#define REG_BUSPD           0x18
#define REG_SECHI           0x20
#define REG_SECLO           0x24
#define REG_NSEC            0x28

#define PITEN_ENABLE        (1u << 26)

struct JCoreAICState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    SuperHCPU *cpu;
    QEMUTimer *timer;

    uint32_t piten;
    uint32_t intpri;
    uint32_t throt;
    bool pit_pending;
    uint32_t line[JCORE_AIC_NVEC / 32];
    uint32_t pending[JCORE_AIC_NVEC / 32];
};

static int jcore_aic_prio(JCoreAICState *s, int vec)
{
    return (s->intpri >> (28 - 4 * ((vec - 16) & 7))) & 0xf;
}

static void jcore_aic_update(JCoreAICState *s)
{
    bool pending = s->pit_pending;

    for (int i = 0; i < ARRAY_SIZE(s->pending); i++) {
        pending |= s->pending[i] != 0;
    }
    if (pending) {
        cpu_interrupt(CPU(s->cpu), CPU_INTERRUPT_HARD);
    } else {
        cpu_reset_interrupt(CPU(s->cpu), CPU_INTERRUPT_HARD);
    }
}

static int jcore_aic_ack(void *opaque, int imask, int *level)
{
    JCoreAICState *s = opaque;
    int best = -1, best_prio = imask;

    if (s->pit_pending) {
        int prio = (s->piten >> 20) & 0xf;

        if (prio > best_prio) {
            best = (s->piten >> 12) & 0xff;
            best_prio = prio;
        }
    }
    for (int vec = 0; vec < JCORE_AIC_NVEC; vec++) {
        if ((s->pending[vec / 32] & (1u << (vec % 32))) &&
            jcore_aic_prio(s, vec) > best_prio) {
            best = vec;
            best_prio = jcore_aic_prio(s, vec);
        }
    }
    if (best < 0) {
        return -1;
    }
    if (s->pit_pending && best == ((s->piten >> 12) & 0xff)) {
        s->pit_pending = false;
    } else {
        s->pending[best / 32] &= ~(1u << (best % 32));
    }
    jcore_aic_update(s);
    *level = best_prio;
    return best;
}

static void jcore_aic_set_irq(void *opaque, int vec, int level)
{
    JCoreAICState *s = opaque;

    uint32_t bit = 1u << (vec % 32);

    if (level && !(s->line[vec / 32] & bit)) {
        s->pending[vec / 32] |= bit;
    }
    if (level) {
        s->line[vec / 32] |= bit;
    } else {
        s->line[vec / 32] &= ~bit;
    }
    jcore_aic_update(s);
}

static void jcore_pit_arm(JCoreAICState *s)
{
    timer_del(s->timer);
    if ((s->piten & PITEN_ENABLE) && s->throt) {
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                            (int64_t)s->throt * J2_BUS_PERIOD_NS);
    }
}

static void jcore_pit_tick(void *opaque)
{
    JCoreAICState *s = opaque;

    s->pit_pending = true;
    jcore_aic_update(s);
    jcore_pit_arm(s);                   /* periodic until disabled */
}

static uint64_t jcore_aic_read(void *opaque, hwaddr addr, unsigned size)
{
    JCoreAICState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    switch (addr) {
    case REG_PITEN:
        return s->piten;
    case REG_INTPRI:
        return s->intpri;
    case REG_THROT:
        return s->throt;
    case REG_BUSPD:
        return J2_BUS_PERIOD_NS;
    case REG_SECHI:
        return (uint64_t)(now / NANOSECONDS_PER_SECOND) >> 32;
    case REG_SECLO:
        return now / NANOSECONDS_PER_SECOND;
    case REG_NSEC:
        return now % NANOSECONDS_PER_SECOND;
    default:
        return 0;
    }
}

static void jcore_aic_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned size)
{
    JCoreAICState *s = opaque;

    switch (addr) {
    case REG_PITEN:
        s->piten = val;
        jcore_pit_arm(s);
        break;
    case REG_INTPRI:
        s->intpri = val;
        jcore_aic_update(s);
        break;
    case REG_THROT:
        s->throt = val;
        break;
    }
}

static const MemoryRegionOps jcore_aic_ops = {
    .read = jcore_aic_read,
    .write = jcore_aic_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void jcore_aic_reset(DeviceState *dev)
{
    JCoreAICState *s = JCORE_AIC(dev);

    timer_del(s->timer);
    s->piten = s->intpri = s->throt = 0;
    s->pit_pending = false;
    memset(s->line, 0, sizeof(s->line));
    memset(s->pending, 0, sizeof(s->pending));
}

static void jcore_aic_realize(DeviceState *dev, Error **errp)
{
    JCoreAICState *s = JCORE_AIC(dev);
    CPUSH4State *env;

    if (!s->cpu) {
        error_setg(errp, TYPE_JCORE_AIC ": 'cpu' link not set");
        return;
    }
    env = &s->cpu->env;
    env->sh2_irq_ack = jcore_aic_ack;
    env->sh2_irq_opaque = s;

    memory_region_init_io(&s->mmio, OBJECT(s), &jcore_aic_ops, s,
                          TYPE_JCORE_AIC, 0x30);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->mmio);
    qdev_init_gpio_in(dev, jcore_aic_set_irq, JCORE_AIC_NVEC);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, jcore_pit_tick, s);
}

static const Property jcore_aic_properties[] = {
    DEFINE_PROP_LINK("cpu", JCoreAICState, cpu, TYPE_SUPERH_CPU, SuperHCPU *),
};

static void jcore_aic_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = jcore_aic_realize;
    device_class_set_legacy_reset(dc, jcore_aic_reset);
    device_class_set_props(dc, jcore_aic_properties);
}

static const TypeInfo jcore_aic_info = {
    .name = TYPE_JCORE_AIC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(JCoreAICState),
    .class_init = jcore_aic_class_init,
};

typedef struct J2Boot {
    SuperHCPU *cpu;
    uint32_t entry;
    uint32_t dtb;
} J2Boot;

static void j2_reset(void *opaque)
{
    J2Boot *boot = opaque;
    CPUSH4State *env = &boot->cpu->env;

    cpu_reset(CPU(boot->cpu));
    env->pc = boot->entry;
    env->gregs[4] = boot->dtb;
}

static void *j2_build_fdt(MachineState *ms, int *size)
{
    void *fdt = create_device_tree(size);
    const char *soc = "/soc@abcd0000";
    const char *aic = "/soc@abcd0000/interrupt-controller@200";
    const char *uart = "/soc@abcd0000/serial@100";
    uint32_t aic_phandle;

    qemu_fdt_setprop_string(fdt, "/", "compatible", "jcore,j2-soc");
    qemu_fdt_setprop_string(fdt, "/", "model", "J2 SoC (qemu)");
    qemu_fdt_setprop_cell(fdt, "/", "#address-cells", 1);
    qemu_fdt_setprop_cell(fdt, "/", "#size-cells", 1);

    qemu_fdt_add_subnode(fdt, "/cpus");
    qemu_fdt_setprop_cell(fdt, "/cpus", "#address-cells", 1);
    qemu_fdt_setprop_cell(fdt, "/cpus", "#size-cells", 0);
    qemu_fdt_add_subnode(fdt, "/cpus/cpu@0");
    qemu_fdt_setprop_string(fdt, "/cpus/cpu@0", "device_type", "cpu");
    qemu_fdt_setprop_string(fdt, "/cpus/cpu@0", "compatible", "jcore,j2");
    qemu_fdt_setprop_cell(fdt, "/cpus/cpu@0", "reg", 0);
    qemu_fdt_setprop_cell(fdt, "/cpus/cpu@0", "clock-frequency", J2_CPU_FREQ);

    qemu_fdt_add_subnode(fdt, "/memory@10000000");
    qemu_fdt_setprop_string(fdt, "/memory@10000000", "device_type", "memory");
    qemu_fdt_setprop_cells(fdt, "/memory@10000000", "reg", J2_SDRAM_BASE,
                           ms->ram_size);

    qemu_fdt_add_subnode(fdt, "/aliases");
    qemu_fdt_setprop_string(fdt, "/aliases", "serial0", uart);
    qemu_fdt_add_subnode(fdt, "/chosen");
    qemu_fdt_setprop_string(fdt, "/chosen", "stdout-path", "serial0");

    qemu_fdt_add_subnode(fdt, soc);
    qemu_fdt_setprop_string(fdt, soc, "compatible", "simple-bus");
    qemu_fdt_setprop_cells(fdt, soc, "ranges", 0, J2_SOC_BASE, J2_SOC_SIZE);
    qemu_fdt_setprop_cell(fdt, soc, "#address-cells", 1);
    qemu_fdt_setprop_cell(fdt, soc, "#size-cells", 1);

    aic_phandle = qemu_fdt_alloc_phandle(fdt);
    qemu_fdt_add_subnode(fdt, aic);
    qemu_fdt_setprop_string(fdt, aic, "compatible", "jcore,aic1");
    qemu_fdt_setprop_cells(fdt, aic, "reg", J2_AIC_OFFSET, 0x10);
    qemu_fdt_setprop(fdt, aic, "interrupt-controller", NULL, 0);
    qemu_fdt_setprop_cell(fdt, aic, "#interrupt-cells", 1);
    qemu_fdt_setprop_cell(fdt, aic, "phandle", aic_phandle);
    qemu_fdt_setprop_cell(fdt, "/", "interrupt-parent", aic_phandle);

    qemu_fdt_add_subnode(fdt, "/soc@abcd0000/cache-controller@c0");
    qemu_fdt_setprop_string(fdt, "/soc@abcd0000/cache-controller@c0",
                            "compatible", "jcore,cache");
    qemu_fdt_setprop_cells(fdt, "/soc@abcd0000/cache-controller@c0", "reg",
                           0xc0, 4);

    qemu_fdt_add_subnode(fdt, "/soc@abcd0000/timer@200");
    qemu_fdt_setprop_string(fdt, "/soc@abcd0000/timer@200", "compatible",
                            "jcore,pit");
    qemu_fdt_setprop_cells(fdt, "/soc@abcd0000/timer@200", "reg",
                           J2_AIC_OFFSET, 0x30);
    qemu_fdt_setprop_cell(fdt, "/soc@abcd0000/timer@200", "interrupts",
                          J2_PIT_VECTOR);

    qemu_fdt_add_subnode(fdt, uart);
    qemu_fdt_setprop_string(fdt, uart, "compatible",
                            "xlnx,xps-uartlite-1.00.a");
    qemu_fdt_setprop_string(fdt, uart, "device_type", "serial");
    qemu_fdt_setprop_cells(fdt, uart, "reg", J2_UART_OFFSET, 0x10);
    qemu_fdt_setprop_cell(fdt, uart, "interrupts", J2_UART_VECTOR);
    qemu_fdt_setprop_cell(fdt, uart, "current-speed", 115200);
    qemu_fdt_setprop_cell(fdt, uart, "clock-frequency", J2_CPU_FREQ);

    fdt_pack(fdt);
    *size = fdt_totalsize(fdt);
    return fdt;
}

/* arch/sh/include/asm/setup.h */
static void j2_boot_params(MachineState *ms, hwaddr initrd, ssize_t size)
{
    uint8_t *zp = rom_ptr(J2_SDRAM_BASE + J2_ZERO_PAGE, 0x1000);
    const char *cmdline = ms->kernel_cmdline ? ms->kernel_cmdline : "";

    if (!zp) {
        if (size || *cmdline) {
            error_report("kernel has no zero page at 0x%x; "
                         "-initrd and -append are ignored",
                         J2_SDRAM_BASE + J2_ZERO_PAGE);
        }
        return;
    }
    if (size) {
        stl_be_p(zp + 0x00c, 1);                        /* LOADER_TYPE */
        stl_be_p(zp + 0x010, initrd - J2_SDRAM_BASE);   /* INITRD_START */
        stl_be_p(zp + 0x014, size);                     /* INITRD_SIZE */
    }
    pstrcpy((char *)zp + 0x100, 0x1000 - 0x100, cmdline);
}

static void j2_load(MachineState *ms, J2Boot *boot)
{
    uint64_t entry, high, ram_end = J2_SDRAM_BASE + ms->ram_size;
    hwaddr initrd = 0;
    ssize_t initrd_size = 0;
    void *fdt;
    int fdt_size;

    if (load_elf(ms->kernel_filename, NULL, NULL, NULL, &entry, NULL, &high,
                 NULL, ELFDATA2MSB, EM_SH, 0, 0) < 0) {
        error_report("could not load kernel '%s'", ms->kernel_filename);
        exit(1);
    }
    boot->entry = entry;
    boot->dtb = ram_end - J2_DTB_SIZE;

    if (ms->initrd_filename) {
        initrd_size = get_image_size(ms->initrd_filename, NULL);
        if (initrd_size < 0) {
            error_report("could not load initrd '%s'", ms->initrd_filename);
            exit(1);
        }
        initrd = QEMU_ALIGN_DOWN(boot->dtb - initrd_size, 4 * KiB);
        if (initrd < ROUND_UP(high, 4 * KiB) ||
            load_image_targphys(ms->initrd_filename, initrd,
                                boot->dtb - initrd, NULL) < 0) {
            error_report("could not load initrd '%s'", ms->initrd_filename);
            exit(1);
        }
    }
    j2_boot_params(ms, initrd, initrd_size);

    fdt = j2_build_fdt(ms, &fdt_size);
    if (fdt_size > J2_DTB_SIZE) {
        error_report("device tree too large");
        exit(1);
    }
    rom_add_blob_fixed("dtb", fdt, fdt_size, boot->dtb);
    ms->fdt = fdt;
}

static void j2_init(MachineState *ms)
{
    MemoryRegion *sysmem = get_system_memory();
    J2Boot *boot = g_new0(J2Boot, 1);
    DeviceState *aic, *uart;
    SuperHCPU *cpu;

    if (!target_big_endian()) {
        error_report("the J2 is big-endian; use qemu-system-sh4eb");
        exit(1);
    }
    if (ms->ram_size > J2_SDRAM_MAX) {
        error_report("at most %" PRIu64 " MiB of SDRAM",
                     (uint64_t)J2_SDRAM_MAX / MiB);
        exit(1);
    }

    cpu = SUPERH_CPU(cpu_create(ms->cpu_type));
    boot->cpu = cpu;

    memory_region_add_subregion(sysmem, J2_SDRAM_BASE, ms->ram);
    create_unimplemented_device("j2-soc", J2_SOC_BASE, J2_SOC_SIZE);

    aic = qdev_new(TYPE_JCORE_AIC);
    object_property_set_link(OBJECT(aic), "cpu", OBJECT(cpu), &error_fatal);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(aic), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(aic), 0, J2_SOC_BASE + J2_AIC_OFFSET);

    uart = qdev_new(TYPE_XILINX_UARTLITE);
    qdev_prop_set_enum(uart, "endianness", ENDIAN_MODE_BIG);
    qdev_prop_set_bit(uart, "pulse-irq", true);
    qdev_prop_set_chr(uart, "chardev", serial_hd(0));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(uart), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(uart), 0, J2_SOC_BASE + J2_UART_OFFSET);
    sysbus_connect_irq(SYS_BUS_DEVICE(uart), 0,
                       qdev_get_gpio_in(aic, J2_UART_VECTOR));

    if (ms->kernel_filename) {
        j2_load(ms, boot);
    }
    qemu_register_reset(j2_reset, boot);
}

static void j2_machine_init(MachineClass *mc)
{
    mc->desc = "J-Core J2 SoC (SH-2, no MMU)";
    mc->init = j2_init;
    mc->default_cpu_type = TYPE_J2_CPU;
    mc->default_ram_size = 64 * MiB;
    mc->default_ram_id = "j2.sdram";
}

DEFINE_MACHINE("j2", j2_machine_init)

static void j2_register_types(void)
{
    type_register_static(&jcore_aic_info);
}

type_init(j2_register_types)
