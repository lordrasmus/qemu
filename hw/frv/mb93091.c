/*
 * Fujitsu MB93091 VDK CPU board with an FR401 (MB93401A) for Linux
 *
 * The on-chip peripherals at 0xfe000000-0xfeffffff as Linux arch/frv
 * (v4.9) programs them: the bus and SDRAM controller, the interrupt
 * controller, timer 0 of the 8254-like timer block, and the two 16550
 * UARTs.  The board's FPGA and the MB93090 motherboard read as zero,
 * except for the clock switch that says the input clock is 33.3 MHz.
 *
 * The kernel is loaded as ELF and entered the way RedBoot starts it:
 * GR8 = 0xdead1eaf and GR9 = the command line.  An initrd goes to the top
 * of RAM and is announced as "initrd=<start>,<size>" on the command line.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "cpu.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/char/serial-mm.h"
#include "hw/misc/unimp.h"
#include "system/system.h"
#include "system/reset.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "elf.h"

#ifndef EM_CYGNUS_FRV
#define EM_CYGNUS_FRV 0x5441
#endif

#define MB_RAM_BASE     0xc0000000      /* CONFIG_UCPAGE_OFFSET_C0000000 */
#define MB_BUSCTL_BASE  0xfe000000
#define MB_TIMER_BASE   0xfeff9400
#define MB_IRC_BASE     0xfeff9800
#define MB_CLKC_BASE    0xfeff9a00
#define MB_UART0_BASE   0xfeff9c00
#define MB_UART1_BASE   0xfeff9c40
#define MB_CLKSW_ADDR   0xe130000c

/*
 * CLKSW 64 makes CLKIN 33.3 MHz; with CLKC.CMODE 8 the core runs at
 * twice that and the resource bus, which clocks the timers and the
 * UARTs, at CLKIN.
 */
#define MB_CLKSW        64
#define MB_CLKC         (8u << 24)
#define MB_RES_BUS_HZ   (MB_CLKSW * 125u * 100000u / 24u)

/* Interrupt sources, numbered for the IRR fields that route them */
enum {
    SRC_XIRQ0 = 0,      /* IRR3 bits 19:16, ... XIRQ7 in IRR2 31:28 */
    SRC_DMA0 = 8,       /* IRR4 */
    SRC_TIMER0 = 12,    /* IRR5 19:16, TIMER1 23:20, TIMER2 27:24 */
    SRC_UART0 = 16,     /* IRR6 19:16, UART1 23:20 */
    SRC_DMA4 = 24,      /* IRR7 */
    NUM_SRC = 32,
};

typedef struct MBState {
    FRVCPU *cpu;
    MemoryRegion busctl;
    MemoryRegion irc_io;
    MemoryRegion timer_io;
    MemoryRegion misc_io;

    /* interrupt controller */
    uint32_t irr[8];
    uint32_t iitmr[2];
    uint32_t tm1;
    uint32_t rs;        /* latched edge requests, bit 16 + level */
    uint32_t mask;      /* bit 16 + level: masked */
    uint32_t src_level; /* current input level per source */
    qemu_irq src_irq[NUM_SRC];

    /* timer 0 */
    QEMUTimer *timer;
    uint32_t tprv;
    uint32_t cksl[3];
    uint32_t tctr;
    uint32_t count[3];
    uint8_t low_written[3];
    bool running;

    uint64_t initrd_start, initrd_size;
    uint32_t entry, cmdline;
} MBState;

/* Interrupt controller */

/* Level 1-15 a source is routed to, 0: none */
static int src_route(MBState *s, int src)
{
    int reg, shift;

    if (src < 8) {
        reg = src < 4 ? 3 : 2;
        shift = 16 + 4 * (src & 3);
    } else {
        reg = 4 + (src - 8) / 4;
        shift = 16 + 4 * ((src - 8) & 3);
    }
    return (s->irr[reg] >> shift) & 0xf;
}

static uint32_t irc_requests(MBState *s)
{
    uint32_t req = s->rs;

    for (int src = 0; src < NUM_SRC; src++) {
        int level = src_route(s, src);

        if (level && (s->src_level & (1u << src))) {
            req |= 1u << (16 + level);
        }
    }
    return req;
}

static int irc_irl(MBState *s)
{
    uint32_t pend = irc_requests(s) & ~s->mask;

    for (int level = 15; level > 0; level--) {
        if (pend & (1u << (16 + level))) {
            return level;
        }
    }
    return 0;
}

static void irc_update(MBState *s)
{
    FRVCPU *cpu = s->cpu;

    cpu->env.irq_level = irc_irl(s);
    frv_update_irq(&cpu->env);
}

/*
 * The timers are edge triggered (IITMR0 0x003f0000 in Linux), so their
 * requests latch until cleared through RC; everything else follows its
 * input level.
 */
static bool src_edge(MBState *s, int src)
{
    return src >= SRC_TIMER0 && src < SRC_TIMER0 + 3;
}

static void irc_set_irq(void *opaque, int src, int level)
{
    MBState *s = opaque;
    uint32_t bit = 1u << src;
    bool was = s->src_level & bit;

    s->src_level = level ? s->src_level | bit : s->src_level & ~bit;
    if (level && !was && src_edge(s, src)) {
        int route = src_route(s, src);

        if (route) {
            s->rs |= 1u << (16 + route);
        }
        s->src_level &= ~bit;
    }
    irc_update(s);
}

static uint64_t irc_read(void *opaque, hwaddr addr, unsigned size)
{
    MBState *s = opaque;

    switch (addr) {
    case 0x00:
        return 0;
    case 0x08:
        return s->tm1;
    case 0x10:
        return irc_requests(s);
    case 0x20:
        return s->mask;
    case 0x28:
        return irc_irl(s) << 16;
    case 0x40 ... 0x7f:
        return s->irr[(addr - 0x40) / 8];
    case 0x80 ... 0x8f:
        return s->iitmr[(addr - 0x80) / 8];
    default:
        return 0;
    }
}

static void irc_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    MBState *s = opaque;

    switch (addr) {
    case 0x08:
        s->tm1 = val;
        break;
    case 0x18: /* RC: clear the latched requests whose bits are set */
        s->rs &= ~val;
        break;
    case 0x20:
        s->mask = val & 0xfffe0000;
        break;
    case 0x28: /* clearing IRL: nothing latched here */
        break;
    case 0x40 ... 0x7f:
        s->irr[(addr - 0x40) / 8] = val;
        break;
    case 0x80 ... 0x8f:
        s->iitmr[(addr - 0x80) / 8] = val;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "mb93091: irc write %02x = %08x\n",
                      (int)addr, (uint32_t)val);
        break;
    }
    irc_update(s);
}

static const MemoryRegionOps irc_ops = {
    .read = irc_read,
    .write = irc_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

/*
 * Timer 0, the only one Linux uses: mode 2 (rate generator), counting
 * resource bus clock / TPRV / 2^select.
 */

static uint64_t timer_period_ns(MBState *s)
{
    uint64_t div = (uint64_t)MAX(s->tprv, 1) << ((s->cksl[0] >> 24) & 0xf);
    uint64_t count = s->count[0] ? s->count[0] : 0x10000;

    return muldiv64(count * div, NANOSECONDS_PER_SECOND, MB_RES_BUS_HZ);
}

static void timer_tick(void *opaque)
{
    MBState *s = opaque;

    irc_set_irq(s, SRC_TIMER0, 1);
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              timer_period_ns(s));
}

static void timer_restart(MBState *s)
{
    if (s->running) {
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  timer_period_ns(s));
    } else {
        timer_del(s->timer);
    }
}

static uint64_t timer_read(void *opaque, hwaddr addr, unsigned size)
{
    MBState *s = opaque;

    switch (addr) {
    case 0x00:
    case 0x08:
    case 0x10:
        return (s->count[addr / 8] & 0xff) << 24;
    case 0x18:
        return s->tctr;
    case 0x20:
        return s->tprv << 24;
    case 0x30:
    case 0x38:
    case 0x40:
        return s->cksl[(addr - 0x30) / 8];
    default:
        return 0;
    }
}

static void timer_write(void *opaque, hwaddr addr, uint64_t val,
                        unsigned size)
{
    MBState *s = opaque;
    int t;

    switch (addr) {
    case 0x00:
    case 0x08:
    case 0x10:
        /* the low byte, then the high byte of the count */
        t = addr / 8;
        if (!s->low_written[t]) {
            s->count[t] = (s->count[t] & 0xff00) | (val >> 24);
            s->low_written[t] = 1;
        } else {
            s->count[t] = (s->count[t] & 0x00ff) | ((val >> 24) << 8);
            s->low_written[t] = 0;
            if (t == 0) {
                s->running = true;
                timer_restart(s);
            }
        }
        break;
    case 0x18:
        s->tctr = val;
        if (((val >> 30) & 3) != 3) {
            t = (val >> 30) & 3;
            s->low_written[t] = 0;
            if (t == 0) {
                s->running = false;
                timer_restart(s);
            }
        }
        break;
    case 0x20:
        s->tprv = (val >> 24) & 0xff;
        break;
    case 0x30:
    case 0x38:
    case 0x40:
        s->cksl[(addr - 0x30) / 8] = val;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps timer_ops = {
    .read = timer_read,
    .write = timer_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

/* CLKC and the FPGA clock switch */

static uint64_t misc_read(void *opaque, hwaddr addr, unsigned size)
{
    return addr == 0 ? MB_CLKC : 0;
}

static void misc_write(void *opaque, hwaddr addr, uint64_t val,
                       unsigned size)
{
}

static const MemoryRegionOps misc_ops = {
    .read = misc_read,
    .write = misc_write,
    .endianness = DEVICE_BIG_ENDIAN,
};

static uint64_t clksw_read(void *opaque, hwaddr addr, unsigned size)
{
    return MB_CLKSW;
}

static const MemoryRegionOps clksw_ops = {
    .read = clksw_read,
    .write = misc_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

/*
 * The UARTs: 16550 registers eight bytes apart, each in bits 31:24 of
 * its word.  Linux reads them with byte loads as well as with word loads
 * shifted right by 24, so word accesses move the byte to the top.
 */
static uint64_t uart_read(void *opaque, hwaddr addr, unsigned size)
{
    MemoryRegion *mr = opaque;
    uint64_t val = 0;

    memory_region_dispatch_read(mr, addr & ~7, &val, MO_8,
                                MEMTXATTRS_UNSPECIFIED);
    return size == 4 ? val << 24 : (addr & 3) ? 0 : val;
}

static void uart_write(void *opaque, hwaddr addr, uint64_t val,
                       unsigned size)
{
    MemoryRegion *mr = opaque;

    if (size == 4) {
        val >>= 24;
    } else if (addr & 3) {
        return;
    }
    memory_region_dispatch_write(mr, addr & ~7, val & 0xff, MO_8,
                                 MEMTXATTRS_UNSPECIFIED);
}

static const MemoryRegionOps uart_ops = {
    .read = uart_read,
    .write = uart_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void mb_uart(hwaddr base, qemu_irq irq, Chardev *chr)
{
    DeviceState *dev = qdev_new(TYPE_SERIAL_MM);
    MemoryRegion *io = g_new(MemoryRegion, 1);

    qdev_prop_set_uint8(dev, "regshift", 3);
    qdev_prop_set_uint32(dev, "baudbase", 115200);
    qdev_prop_set_chr(dev, "chardev", chr);
    qdev_prop_set_uint8(dev, "endianness", DEVICE_BIG_ENDIAN);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, irq);

    memory_region_init_io(io, NULL, &uart_ops,
                          sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 0),
                          "mb93091.uart", 0x40);
    memory_region_add_subregion_overlap(get_system_memory(), base, io, 1);
}

/*
 * Bus and SDRAM controller registers.  Linux reads where SDRAM is from
 * DBR0/DAM0 and moves it to PAGE_OFFSET; it already is there.
 */
static void busctl_reset(MBState *s, uint64_t ram_size)
{
    uint8_t *p = memory_region_get_ram_ptr(&s->busctl);

    memset(p, 0, memory_region_size(&s->busctl));
    stl_be_p(p + 0xe00, MB_RAM_BASE);           /* DBR0 */
    stl_be_p(p + 0xe08, 0xfe000000);            /* DBR1: unused */
    stl_be_p(p + 0xe10, 0xfe000000);
    stl_be_p(p + 0xe18, 0xfe000000);
    stl_be_p(p + 0xf00, ram_size - 1);          /* DAM0 */
}

static MBState mb;

static void mb_reset(void *opaque)
{
    MBState *s = opaque;
    CPUFRVState *env = &s->cpu->env;
    MachineState *ms = MACHINE(qdev_get_machine());

    cpu_reset(CPU(s->cpu));
    busctl_reset(s, ms->ram_size);
    memset(s->irr, 0, sizeof(s->irr));
    s->mask = 0xfffe0000;
    s->rs = 0;
    s->running = false;
    timer_del(s->timer);
    irc_update(s);

    env->pc = s->entry;
    env->gr[8] = 0xdead1eaf;
    env->gr[9] = s->cmdline;
}

static void mb93091_init(MachineState *ms)
{
    MemoryRegion *sysmem = get_system_memory();
    MBState *s = &mb;
    MemoryRegion *clksw = g_new(MemoryRegion, 1);
    uint64_t entry, hi;
    hwaddr top = MB_RAM_BASE + ms->ram_size;
    g_autofree char *cmdline = NULL;
    ssize_t size;

    /* SDRAM must end below the I/O space at 0xe0000000 */
    if (ms->ram_size > 512 * MiB) {
        error_report("mb93091: at most 512 MiB of RAM");
        exit(EXIT_FAILURE);
    }

    s->cpu = FRV_CPU(object_new(ms->cpu_type));
    qdev_realize(DEVICE(s->cpu), NULL, &error_fatal);

    memory_region_add_subregion(sysmem, MB_RAM_BASE, ms->ram);

    /* everything at 0xe0000000 and up that is not modelled reads zero */
    create_unimplemented_device("mb93091.io", 0xe0000000, 0x20000000);

    memory_region_init_ram(&s->busctl, NULL, "mb93091.busctl", 0x10000,
                           &error_fatal);
    memory_region_add_subregion_overlap(sysmem, MB_BUSCTL_BASE, &s->busctl,
                                        1);
    memory_region_init_io(&s->irc_io, NULL, &irc_ops, s, "mb93091.irc",
                          0x100);
    memory_region_add_subregion_overlap(sysmem, MB_IRC_BASE, &s->irc_io, 1);
    memory_region_init_io(&s->timer_io, NULL, &timer_ops, s,
                          "mb93091.timer", 0x48);
    memory_region_add_subregion_overlap(sysmem, MB_TIMER_BASE, &s->timer_io,
                                        1);
    memory_region_init_io(&s->misc_io, NULL, &misc_ops, s, "mb93091.clkc",
                          0x10);
    memory_region_add_subregion_overlap(sysmem, MB_CLKC_BASE, &s->misc_io,
                                        1);
    memory_region_init_io(clksw, NULL, &clksw_ops, s, "mb93091.clksw", 4);
    memory_region_add_subregion_overlap(sysmem, MB_CLKSW_ADDR, clksw, 1);

    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, timer_tick, s);
    for (int i = 0; i < NUM_SRC; i++) {
        s->src_irq[i] = qemu_allocate_irq(irc_set_irq, s, i);
    }

    /*
     * Linux registers both ports as line 0, so UART1 ends up as ttyS0;
     * the first serial port goes there.
     */
    mb_uart(MB_UART1_BASE, s->src_irq[SRC_UART0 + 1], serial_hd(0));
    mb_uart(MB_UART0_BASE, s->src_irq[SRC_UART0], serial_hd(1));

    if (!ms->kernel_filename) {
        error_report("mb93091: -kernel is required");
        exit(EXIT_FAILURE);
    }
    size = load_elf(ms->kernel_filename, NULL, NULL, NULL, &entry, NULL,
                    &hi, NULL, ELFDATA2MSB, EM_CYGNUS_FRV, 0, 0);
    if (size < 0) {
        error_report("mb93091: could not load kernel '%s'",
                     ms->kernel_filename);
        exit(EXIT_FAILURE);
    }
    s->entry = entry;

    /* the command line in the last 4 KiB, the initrd below it */
    top -= 4 * KiB;
    s->cmdline = top;
    cmdline = g_strdup(ms->kernel_cmdline ? ms->kernel_cmdline : "");
    if (ms->initrd_filename) {
        size = get_image_size(ms->initrd_filename, NULL);
        if (size < 0) {
            error_report("mb93091: could not load initrd '%s'",
                         ms->initrd_filename);
            exit(EXIT_FAILURE);
        }
        s->initrd_start = (top - size) & ~(hwaddr)(64 * KiB - 1);
        if (s->initrd_start < ROUND_UP(hi, 64 * KiB) + 4 * MiB) {
            error_report("mb93091: the initrd does not fit into RAM");
            exit(EXIT_FAILURE);
        }
        load_image_targphys(ms->initrd_filename, s->initrd_start, size,
                            &error_fatal);
        s->initrd_size = size;
        g_free(cmdline);
        cmdline = g_strdup_printf("%s%sinitrd=0x%" PRIx64 ",%" PRIu64,
                                  ms->kernel_cmdline ? ms->kernel_cmdline
                                                     : "",
                                  ms->kernel_cmdline && *ms->kernel_cmdline
                                  ? " " : "",
                                  s->initrd_start, s->initrd_size);
    }
    if (strlen(cmdline) >= 512) {
        error_report("mb93091: the command line is longer than 511 bytes");
        exit(EXIT_FAILURE);
    }
    rom_add_blob_fixed("cmdline", cmdline, strlen(cmdline) + 1, s->cmdline);

    qemu_register_reset(mb_reset, s);
}

static void mb93091_machine_init(MachineClass *mc)
{
    mc->desc = "Fujitsu MB93091 VDK with an FR401 CPU, for uClinux";
    mc->init = mb93091_init;
    mc->default_cpu_type = TYPE_FRV_CPU;
    mc->default_ram_size = 256 * MiB;
    mc->default_ram_id = "mb93091.sdram";
    mc->is_default = true;
}

DEFINE_MACHINE("mb93091", mb93091_machine_init)
