/*
 * Spectrum Digital DSK6455 (TI TMS320C6455) for C6x Linux
 *
 * Addresses and compatible strings follow arch/c6x/boot/dts/dsk6455.dts
 * of Linux 5.10, so a kernel built from dsk6455_defconfig finds its
 * devices in the device tree generated here.  The board has no UART the
 * kernel knows about (TI's console went through the emulator), so the
 * tree adds an NS16550A at an unused address; enable SERIAL_8250_OF.
 *
 * The CPU is a C674x: C64x+ with the C67x+ floating point instructions,
 * which the uClibc-ng toolchain for c6x emits.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/error-report.h"
#include "qemu/guest-random.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "cpu.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "hw/char/serial-mm.h"
#include "system/system.h"
#include "system/reset.h"
#include "system/runstate.h"
#include "system/device_tree.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "elf.h"

#ifndef EM_TI_C6000
#define EM_TI_C6000 140
#endif

#include <libfdt.h>

#define DSK_DDR_BASE        0xe0000000
#define DSK_L2_BASE         0x00800000
#define DSK_L2_SIZE         (2 * MiB)
#define DSK_MEGAMOD_BASE    0x01800000
#define DSK_CACHE_BASE      0x01840000
#define DSK_TIMER1_BASE     0x02980000
#define DSK_TIMER1_EVT      69
#define DSK_PLL_BASE        0x029a0000
#define DSK_DSCR_BASE       0x02a80000
#define DSK_UART_BASE       0x02b80000
#define DSK_UART_EVT        100
#define DSK_EMIFA_BASE      0x70000000
#define DSK_EXIT_BASE       0x02bc0000

#define DSK_CLKIN           50000000
#define DSK_PLLM            20
#define DSK_CPU_FREQ        (DSK_CLKIN * DSK_PLLM)

static struct {
    uint32_t entry;
    uint32_t fdt;
} boot_info;

static void dsk_cpu_reset(void *opaque)
{
    C6xCPU *cpu = opaque;
    CPUC6xState *env = &cpu->env;

    cpu_reset(CPU(cpu));
    env->pc = boot_info.entry;
    /* the kernel takes the device tree in B4 if A4 holds its magic */
    env->gpr[REG_A(4)] = 0xd00dfeed;
    env->gpr[REG_B(4)] = boot_info.fdt;
}

/* PLL1 controller: multiplier 20 from the 50 MHz input, as on the DSK */

typedef struct DskRegs {
    uint32_t pll[0x200 / 4];
    uint32_t dscr_lo[0x1000 / 4];
    uint32_t dscr_hi[0x1000 / 4];   /* 0x40000 */
} DskRegs;

static DskRegs dsk_regs;

static uint64_t pll_read(void *opaque, hwaddr addr, unsigned size)
{
    return dsk_regs.pll[addr / 4];
}

static void pll_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    if (addr == 0x138) {        /* PLLCMD: transitions complete at once */
        return;
    }
    dsk_regs.pll[addr / 4] = val;
}

static const MemoryRegionOps pll_ops = {
    .read = pll_read,
    .write = pll_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

/*
 * Device state control: PERCFG0 at 0x40008 enables peripherals two bits
 * each, PERSTAT0/1 at 0x40014/0x40018 report the state three bits each,
 * see the ti,dscr-devstate-* properties.
 */
static uint32_t dscr_stat(int first, int n)
{
    uint32_t ctl = dsk_regs.dscr_hi[0x8 / 4], v = 0;

    for (int i = 0; i < n; i++) {
        if (extract32(ctl, 2 * (first + i), 2) == 1) {
            v |= 1u << (3 * i);
        }
    }
    return v;
}

static uint64_t dscr_read(void *opaque, hwaddr addr, unsigned size)
{
    if (addr == 0x40014) {
        return dscr_stat(0, 10);
    }
    if (addr == 0x40018) {
        return dscr_stat(10, 2);
    }
    if (addr < 0x1000) {
        return dsk_regs.dscr_lo[addr / 4];
    }
    if (addr >= 0x40000 && addr < 0x41000) {
        return dsk_regs.dscr_hi[(addr - 0x40000) / 4];
    }
    return 0;
}

static void dscr_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    if (addr < 0x1000) {
        dsk_regs.dscr_lo[addr / 4] = val;
    } else if (addr >= 0x40000 && addr < 0x41000) {
        dsk_regs.dscr_hi[(addr - 0x40000) / 4] = val;
    }
}

static const MemoryRegionOps dscr_ops = {
    .read = dscr_read,
    .write = dscr_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

/*
 * Cache control: there are no caches, so every writeback or invalidate
 * operation is complete by the time the kernel polls for it.
 */
static uint64_t cache_read(void *opaque, hwaddr addr, unsigned size)
{
    return 0;
}

static void cache_write(void *opaque, hwaddr addr, uint64_t val,
                        unsigned size)
{
}

static const MemoryRegionOps cache_ops = {
    .read = cache_read,
    .write = cache_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

/*
 * Not on the real board: writing a value here ends qemu with that exit
 * code.  The device tree hands it to the kernel as a syscon-poweroff, so
 * powering off ends qemu; programs without an operating system can use
 * it directly.
 */
/* reads as all ones, so the regmap update in syscon-poweroff writes */
static uint64_t exit_read(void *opaque, hwaddr addr, unsigned size)
{
    return 0xffffffff;
}

static void exit_write(void *opaque, hwaddr addr, uint64_t val,
                       unsigned size)
{
    qemu_system_shutdown_request_with_code(SHUTDOWN_CAUSE_GUEST_SHUTDOWN,
                                           val);
}

static const MemoryRegionOps exit_ops = {
    .read = exit_read,
    .write = exit_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void dsk_regs_reset(void *opaque)
{
    memset(&dsk_regs, 0, sizeof(dsk_regs));
    dsk_regs.pll[0x100 / 4] = 1;                    /* PLLCTL: PLLEN */
    dsk_regs.pll[0x110 / 4] = DSK_PLLM - 1;         /* PLLM */
    dsk_regs.pll[0x160 / 4] = 0x8000 | 7;           /* PLLDIV4 */
    dsk_regs.pll[0x164 / 4] = 0x8000 | 7;           /* PLLDIV5 */
    /* silicon revision 2.1 */
    dsk_regs.dscr_lo[0x8 / 4] = 2 << 28;
}

static void fdt_node(void *fdt, const char *name, const char *compat,
                     size_t compat_len)
{
    qemu_fdt_add_subnode(fdt, name);
    if (compat) {
        qemu_fdt_setprop(fdt, name, "compatible", compat, compat_len);
    }
}

#define COMPAT(s)   s, sizeof(s)

static void *dsk_create_fdt(MachineState *ms, int *size,
                            hwaddr initrd_start, hwaddr initrd_end)
{
    void *fdt = create_device_tree(size);
    uint32_t core_pic, megamod;
    uint8_t rng_seed[32];
    char *name, *uart;

    qemu_fdt_setprop_string(fdt, "/", "model", "Spectrum Digital DSK6455");
    qemu_fdt_setprop_string(fdt, "/", "compatible", "spectrum-digital,dsk6455");
    qemu_fdt_setprop_cell(fdt, "/", "#address-cells", 1);
    qemu_fdt_setprop_cell(fdt, "/", "#size-cells", 1);

    qemu_fdt_add_subnode(fdt, "/cpus");
    qemu_fdt_setprop_cell(fdt, "/cpus", "#address-cells", 1);
    qemu_fdt_setprop_cell(fdt, "/cpus", "#size-cells", 0);
    qemu_fdt_add_subnode(fdt, "/cpus/cpu@0");
    qemu_fdt_setprop_string(fdt, "/cpus/cpu@0", "device_type", "cpu");
    qemu_fdt_setprop_string(fdt, "/cpus/cpu@0", "model", "ti,c64x+");
    qemu_fdt_setprop_cell(fdt, "/cpus/cpu@0", "reg", 0);

    name = g_strdup_printf("/memory@%x", DSK_DDR_BASE);
    qemu_fdt_add_subnode(fdt, name);
    qemu_fdt_setprop_string(fdt, name, "device_type", "memory");
    qemu_fdt_setprop_cells(fdt, name, "reg", DSK_DDR_BASE, ms->ram_size);
    g_free(name);

    fdt_node(fdt, "/soc", COMPAT("simple-bus"));
    qemu_fdt_setprop_string(fdt, "/soc", "model", "tms320c6455");
    qemu_fdt_setprop_cell(fdt, "/soc", "#address-cells", 1);
    qemu_fdt_setprop_cell(fdt, "/soc", "#size-cells", 1);
    qemu_fdt_setprop(fdt, "/soc", "ranges", NULL, 0);

    core_pic = qemu_fdt_alloc_phandle(fdt);
    fdt_node(fdt, "/soc/interrupt-controller",
             COMPAT("ti,c64x+core-pic"));
    qemu_fdt_setprop(fdt, "/soc/interrupt-controller", "interrupt-controller",
                     NULL, 0);
    qemu_fdt_setprop_cell(fdt, "/soc/interrupt-controller",
                          "#interrupt-cells", 1);
    qemu_fdt_setprop_cell(fdt, "/soc/interrupt-controller", "phandle",
                          core_pic);

    megamod = qemu_fdt_alloc_phandle(fdt);
    name = g_strdup_printf("/soc/interrupt-controller@%x", DSK_MEGAMOD_BASE);
    fdt_node(fdt, name, COMPAT("ti,c64x+megamod-pic"));
    qemu_fdt_setprop(fdt, name, "interrupt-controller", NULL, 0);
    qemu_fdt_setprop_cell(fdt, name, "#interrupt-cells", 1);
    qemu_fdt_setprop_cells(fdt, name, "reg", DSK_MEGAMOD_BASE, 0x1000);
    qemu_fdt_setprop_cell(fdt, name, "interrupt-parent", core_pic);
    qemu_fdt_setprop_cells(fdt, name, "interrupts", 12, 13, 14, 15);
    qemu_fdt_setprop_cell(fdt, name, "phandle", megamod);
    g_free(name);

    name = g_strdup_printf("/soc/cache-controller@%x", DSK_CACHE_BASE);
    fdt_node(fdt, name, COMPAT("ti,c64x+cache"));
    qemu_fdt_setprop_cells(fdt, name, "reg", DSK_CACHE_BASE, 0x8400);
    g_free(name);

    name = g_strdup_printf("/soc/timer@%x", DSK_TIMER1_BASE);
    fdt_node(fdt, name, COMPAT("ti,c64x+timer64"));
    qemu_fdt_setprop_cells(fdt, name, "reg", DSK_TIMER1_BASE, 0x40);
    qemu_fdt_setprop_cell(fdt, name, "ti,dscr-dev-enable", 4);
    qemu_fdt_setprop_cell(fdt, name, "interrupt-parent", megamod);
    qemu_fdt_setprop_cell(fdt, name, "interrupts", DSK_TIMER1_EVT);
    g_free(name);

    name = g_strdup_printf("/soc/clock-controller@%x", DSK_PLL_BASE);
    fdt_node(fdt, name, COMPAT("ti,c6455-pll\0ti,c64x+pll"));
    qemu_fdt_setprop_cells(fdt, name, "reg", DSK_PLL_BASE, 0x200);
    qemu_fdt_setprop_cell(fdt, name, "ti,c64x+pll-bypass-delay", 1440);
    qemu_fdt_setprop_cell(fdt, name, "ti,c64x+pll-reset-delay", 15360);
    qemu_fdt_setprop_cell(fdt, name, "ti,c64x+pll-lock-delay", 24000);
    qemu_fdt_setprop_cell(fdt, name, "clock-frequency", DSK_CLKIN);
    g_free(name);

    name = g_strdup_printf("/soc/device-state-config-regs@%x", DSK_DSCR_BASE);
    fdt_node(fdt, name, COMPAT("ti,c64x+dscr"));
    qemu_fdt_setprop_cells(fdt, name, "reg", DSK_DSCR_BASE, 0x41000);
    qemu_fdt_setprop_cell(fdt, name, "ti,dscr-devstat", 0);
    qemu_fdt_setprop_cells(fdt, name, "ti,dscr-silicon-rev", 8, 28, 0xf);
    qemu_fdt_setprop_cells(fdt, name, "ti,dscr-rmii-resets",
                           0, 0x40020, 0x00040000);
    qemu_fdt_setprop_cells(fdt, name, "ti,dscr-locked-regs",
                           0x40008, 0x40004, 0x0f0a0b00);
    qemu_fdt_setprop_cells(fdt, name, "ti,dscr-devstate-ctl-regs",
                           0, 12, 0x40008, 1, 0, 0, 2,
                           12, 1, 0x40008, 3, 0, 30, 2,
                           13, 2, 0x4002c, 1, 0xffffffff, 0, 1);
    qemu_fdt_setprop_cells(fdt, name, "ti,dscr-devstate-stat-regs",
                           0, 10, 0x40014, 1, 0, 0, 3,
                           10, 2, 0x40018, 1, 0, 0, 3);
    g_free(name);

    uart = g_strdup_printf("/soc/serial@%x", DSK_UART_BASE);
    fdt_node(fdt, uart, COMPAT("ns16550a"));
    qemu_fdt_setprop_cells(fdt, uart, "reg", DSK_UART_BASE, 0x100);
    qemu_fdt_setprop_cell(fdt, uart, "reg-shift", 2);
    qemu_fdt_setprop_cell(fdt, uart, "reg-io-width", 4);
    qemu_fdt_setprop_cell(fdt, uart, "clock-frequency", 1843200);
    qemu_fdt_setprop_cell(fdt, uart, "interrupt-parent", megamod);
    qemu_fdt_setprop_cell(fdt, uart, "interrupts", DSK_UART_EVT);

    {
        uint32_t syscon = qemu_fdt_alloc_phandle(fdt);

        name = g_strdup_printf("/soc/syscon@%x", DSK_EXIT_BASE);
        fdt_node(fdt, name, COMPAT("syscon"));
        qemu_fdt_setprop_cells(fdt, name, "reg", DSK_EXIT_BASE, 4);
        qemu_fdt_setprop_cell(fdt, name, "phandle", syscon);
        g_free(name);
        fdt_node(fdt, "/poweroff", COMPAT("syscon-poweroff"));
        qemu_fdt_setprop_cell(fdt, "/poweroff", "regmap", syscon);
        qemu_fdt_setprop_cell(fdt, "/poweroff", "offset", 0);
        qemu_fdt_setprop_cell(fdt, "/poweroff", "value", 0);
    }

    qemu_fdt_add_subnode(fdt, "/chosen");
    qemu_fdt_setprop_string(fdt, "/chosen", "stdout-path", uart);
    g_free(uart);
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

static void dsk6455_init(MachineState *ms)
{
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *l2 = g_new(MemoryRegion, 1);
    MemoryRegion *emifa = g_new(MemoryRegion, 1);
    MemoryRegion *io = g_new(MemoryRegion, 4);
    C6xCPU *cpu;
    DeviceState *dev, *megamod;
    hwaddr high, initrd_start = 0, initrd_end = 0;
    uint64_t entry, hi;
    ssize_t size;
    void *fdt;
    int fdt_size;

    cpu = C6X_CPU(object_new(ms->cpu_type));
    qdev_prop_set_uint32(DEVICE(cpu), "clock-frequency", DSK_CPU_FREQ);
    qdev_realize(DEVICE(cpu), NULL, &error_fatal);

    memory_region_add_subregion(sysmem, DSK_DDR_BASE, ms->ram);
    memory_region_init_ram(l2, NULL, "dsk6455.l2", DSK_L2_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, DSK_L2_BASE, l2);
    memory_region_init_ram(emifa, NULL, "dsk6455.emifa", 0x100, &error_fatal);
    memory_region_add_subregion(sysmem, DSK_EMIFA_BASE, emifa);

    memory_region_init_io(&io[0], NULL, &pll_ops, NULL, "dsk6455.pll", 0x200);
    memory_region_add_subregion(sysmem, DSK_PLL_BASE, &io[0]);
    memory_region_init_io(&io[1], NULL, &dscr_ops, NULL, "dsk6455.dscr",
                          0x41000);
    memory_region_add_subregion(sysmem, DSK_DSCR_BASE, &io[1]);
    memory_region_init_io(&io[2], NULL, &cache_ops, NULL, "dsk6455.cache",
                          0x8400);
    memory_region_add_subregion(sysmem, DSK_CACHE_BASE, &io[2]);
    memory_region_init_io(&io[3], NULL, &exit_ops, NULL, "dsk6455.exit", 4);
    memory_region_add_subregion(sysmem, DSK_EXIT_BASE, &io[3]);
    qemu_register_reset(dsk_regs_reset, NULL);
    dsk_regs_reset(NULL);

    megamod = qdev_new("c64x-megamod");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(megamod), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(megamod), 0, DSK_MEGAMOD_BASE);
    for (int i = 0; i < 12; i++) {
        qdev_connect_gpio_out(megamod, i,
                              qdev_get_gpio_in_named(DEVICE(cpu), "IRQ",
                                                     i + 4));
    }

    dev = qdev_new("c64x-timer64");
    qdev_prop_set_uint32(dev, "cpu-frequency", DSK_CPU_FREQ);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, DSK_TIMER1_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                       qdev_get_gpio_in(megamod, DSK_TIMER1_EVT));

    serial_mm_init(sysmem, DSK_UART_BASE, 2,
                   qdev_get_gpio_in(megamod, DSK_UART_EVT), 115200,
                   serial_hd(0), DEVICE_LITTLE_ENDIAN);

    if (!ms->kernel_filename) {
        error_report("dsk6455: -kernel is required");
        exit(EXIT_FAILURE);
    }
    size = load_elf(ms->kernel_filename, NULL, NULL, NULL, &entry, NULL,
                    &hi, NULL, ELFDATA2LSB, EM_TI_C6000, 0, 0);
    if (size < 0) {
        error_report("dsk6455: could not load kernel '%s'",
                     ms->kernel_filename);
        exit(EXIT_FAILURE);
    }
    boot_info.entry = entry;
    high = ROUND_UP(hi, 1 * MiB);

    if (ms->initrd_filename) {
        size = load_image_targphys(ms->initrd_filename, high,
                                   DSK_DDR_BASE + ms->ram_size - high - MiB,
                                   NULL);
        if (size < 0) {
            error_report("dsk6455: could not load initrd '%s'",
                         ms->initrd_filename);
            exit(EXIT_FAILURE);
        }
        initrd_start = high;
        initrd_end = high + size;
        high = ROUND_UP(initrd_end, 64 * KiB);
    }
    boot_info.fdt = high;

    if (ms->dtb) {
        fdt = load_device_tree(ms->dtb, &fdt_size);
        if (!fdt) {
            exit(EXIT_FAILURE);
        }
    } else {
        fdt = dsk_create_fdt(ms, &fdt_size, initrd_start, initrd_end);
    }
    fdt_add_mem_rsv(fdt, boot_info.fdt, 64 * KiB);
    fdt_pack(fdt);
    fdt_size = fdt_totalsize(fdt);
    rom_add_blob_fixed("fdt", fdt, fdt_size, boot_info.fdt);
    ms->fdt = fdt;

    qemu_register_reset(dsk_cpu_reset, cpu);
}

static void dsk6455_machine_init(MachineClass *mc)
{
    mc->desc = "Spectrum Digital DSK6455 (TI C6455, C674x CPU) for Linux";
    mc->init = dsk6455_init;
    mc->default_cpu_type = TYPE_C6X_CPU;
    mc->default_ram_size = 128 * MiB;
    mc->default_ram_id = "dsk6455.ddr";
    mc->is_default = true;
}

DEFINE_MACHINE("dsk6455", dsk6455_machine_init)
