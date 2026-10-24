/*
 * Axis ETRAX 100LX (CRISv10) system on chip
 *
 * DRAM at 0x40000000 (0xc0000000 is the uncached alias Linux runs at) and
 * the I/O registers at 0xb0000000; the bus does not see address bit 31.
 * Modelled are the interrupt controller, timer 0 and 1, serial port 0 and
 * the MMU registers, which are forwarded to the CPU. Everything else in
 * the register space is plain storage.
 *
 * -kernel takes a Linux vmlinux and starts it the way the Axis boot
 * loader does: r8 says the DRAM is set up, r10/r11 pass -append. -initrd
 * is passed on that command line as "initrd=<start>,<size>", which the
 * 4.9 kernel only understands with kernel-cris-initrd.patch.
 *
 * Register layout: arch/cris/include/uapi/arch-v10/arch/sv_addr.agh.
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
#include "chardev/char-fe.h"
#include "hw/core/boards.h"
#include "hw/core/irq.h"
#include "hw/core/loader.h"
#include "system/address-spaces.h"
#include "system/reset.h"
#include "system/system.h"
#include "target/cris/cpu.h"

#define ETRAX_DRAM_BASE     0x40000000
#define ETRAX_IO_BASE       0x30000000      /* 0xb0000000 without bit 31 */
#define ETRAX_IO_SIZE       0x1000

#define RAM_INIT_MAGIC      0x56902387
#define COMMAND_LINE_MAGIC  0x87109563

/* register offsets */
#define R_TIMER_CTRL        0x20
#define R_WATCHDOG          0x24
#define R_SERIAL0           0x60
#define R_IRQ_MASK0         0xc0
#define R_IRQ_MASK1         0xc8
#define R_IRQ_MASK2         0xd0
#define R_VECT_MASK         0xd8
#define R_TIM_PRESC_STATUS  0xf0
#define R_TIMER_PRESCALE    0xf4
#define R_DMA_CMD           0x1d0
#define R_MMU               0x240
#define R_MMU_END           0x260

/* R_IRQ_MASK0 and R_VECT_MASK bits */
#define IRQ0_TIMER0         (1u << 0)
#define IRQ0_TIMER1         (1u << 1)
#define VECT_TIMER0         2
#define VECT_TIMER1         3
#define VECT_SERIAL         8
/* R_IRQ_MASK1: serial port 0 data available and transmitter ready */
#define IRQ1_SER0_DATA      (1u << 8)
#define IRQ1_SER0_READY     (1u << 9)
#define IRQ1_SERIAL         0xff00

/* R_TIMER_CTRL */
#define TIMER_I0_CLR        (1u << 6)
#define TIMER_I1_CLR        (1u << 14)

/* R_SERIAL0_STATUS / bits 15-8 of R_SERIAL0_READ */
#define SER_DATA_AVAIL      (1u << 0)
#define SER_TR_READY        (1u << 5)
#define SER_REC_ENABLE      (1u << 6)       /* in R_SERIAL0_REC_CTRL */

typedef struct EtraxState {
    CRISCPU *cpu;
    MemoryRegion io;
    CharFrontend chr;

    uint32_t irq_mask[3];
    uint32_t vect_mask;
    uint32_t vector;                        /* raised to the CPU, or 0 */

    uint32_t timer_ctrl;
    uint32_t prescale;
    bool timer_irq[2];
    int64_t timer_base[2];
    QEMUTimer *timer[2];

    uint8_t rx_fifo[16];
    unsigned rx_len;

    uint8_t regs[ETRAX_IO_SIZE];            /* everything not modelled */

    uint32_t entry;
    uint32_t cmdline;                       /* physical, 0 without -append */
} EtraxState;

static const unsigned timer_hz[16] = {
    300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200,
    230400, 460800, 921600, 1843200, 6250000, 0,
};

/* the active sources of a group; R_IRQ_MASKx_RD only those enabled */
static uint32_t etrax_irq_read(EtraxState *s, int group, bool masked)
{
    uint32_t v = 0;

    switch (group) {
    case 0:
        v = (s->timer_irq[0] ? IRQ0_TIMER0 : 0) |
            (s->timer_irq[1] ? IRQ0_TIMER1 : 0);
        break;
    case 1:
        v = IRQ1_SER0_READY | (s->rx_len ? IRQ1_SER0_DATA : 0);
        break;
    }
    return masked ? v & s->irq_mask[group] : v;
}

/* likewise R_VECT_READ and R_VECT_MASK_RD */
static uint32_t etrax_vect_read(EtraxState *s, bool masked)
{
    uint32_t irq0 = etrax_irq_read(s, 0, true), v = 0;

    if (irq0 & IRQ0_TIMER0) {
        v |= 1u << VECT_TIMER0;
    }
    if (irq0 & IRQ0_TIMER1) {
        v |= 1u << VECT_TIMER1;
    }
    if (etrax_irq_read(s, 1, true) & IRQ1_SERIAL) {
        v |= 1u << VECT_SERIAL;
    }
    return masked ? v & s->vect_mask : v;
}

/* The CPU gets the vector 0x20 + n of the lowest active source */
static void etrax_update_irq(EtraxState *s)
{
    uint32_t pend = etrax_vect_read(s, true);
    uint32_t vector = pend ? 0x20 + ctz32(pend) : 0;

    if (vector != s->vector) {
        s->vector = vector;
        qemu_set_irq(qdev_get_gpio_in(DEVICE(s->cpu), CRIS_CPU_IRQ), vector);
    }
}

/* timer n counts down from its 8-bit divider; 0 means 256 */
static unsigned etrax_timer_div(EtraxState *s, int n)
{
    unsigned div = (s->timer_ctrl >> (n ? 24 : 16)) & 0xff;

    return div ? div : 256;
}

static int64_t etrax_timer_tick_ns(EtraxState *s, int n)
{
    unsigned sel = (s->timer_ctrl >> (n ? 8 : 0)) & 0xf;
    uint64_t hz;

    if (sel == 15) {
        if (n) {
            return 0;               /* cascade: not modelled */
        }
        hz = 25000000 / (s->prescale ? s->prescale : 65536);
    } else {
        hz = timer_hz[sel];
    }
    return hz ? NANOSECONDS_PER_SECOND / hz : 0;
}

static bool etrax_timer_running(EtraxState *s, int n)
{
    return ((s->timer_ctrl >> (n ? 12 : 4)) & 3) == 2;
}

static void etrax_timer_arm(EtraxState *s, int n)
{
    int64_t tick = etrax_timer_tick_ns(s, n);

    timer_del(s->timer[n]);
    if (etrax_timer_running(s, n) && tick) {
        timer_mod(s->timer[n],
                  s->timer_base[n] + etrax_timer_div(s, n) * tick);
    }
}

static void etrax_timer_cb(EtraxState *s, int n)
{
    s->timer_irq[n] = true;
    s->timer_base[n] += etrax_timer_div(s, n) * etrax_timer_tick_ns(s, n);
    etrax_timer_arm(s, n);
    etrax_update_irq(s);
}

static void etrax_timer0_cb(void *opaque)
{
    etrax_timer_cb(opaque, 0);
}

static void etrax_timer1_cb(void *opaque)
{
    etrax_timer_cb(opaque, 1);
}

static uint32_t etrax_timer_count(EtraxState *s, int n)
{
    int64_t tick = etrax_timer_tick_ns(s, n);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    unsigned div = etrax_timer_div(s, n);

    if (!etrax_timer_running(s, n) || !tick) {
        return div & 0xff;
    }
    return (div - ((now - s->timer_base[n]) / tick) % div) & 0xff;
}

static void etrax_timer_ctrl_write(EtraxState *s, uint32_t val)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint32_t old = s->timer_ctrl;

    if (val & TIMER_I0_CLR) {
        s->timer_irq[0] = false;
    }
    if (val & TIMER_I1_CLR) {
        s->timer_irq[1] = false;
    }
    s->timer_ctrl = val & ~(TIMER_I0_CLR | TIMER_I1_CLR);
    for (int n = 0; n < 2; n++) {
        bool was = ((old >> (n ? 12 : 4)) & 3) == 2;

        if (!was && etrax_timer_running(s, n)) {
            s->timer_base[n] = now;
        }
        etrax_timer_arm(s, n);
    }
    etrax_update_irq(s);
}

static uint32_t etrax_serial_status(EtraxState *s)
{
    return SER_TR_READY | (s->rx_len ? SER_DATA_AVAIL : 0);
}

static uint32_t etrax_serial_pop(EtraxState *s)
{
    uint32_t c = 0;

    if (s->rx_len) {
        c = s->rx_fifo[0];
        memmove(s->rx_fifo, s->rx_fifo + 1, --s->rx_len);
        qemu_chr_fe_accept_input(&s->chr);
        etrax_update_irq(s);
    }
    return c;
}

static int etrax_serial_can_rx(void *opaque)
{
    EtraxState *s = opaque;

    return sizeof(s->rx_fifo) - s->rx_len;
}

static void etrax_serial_rx(void *opaque, const uint8_t *buf, int size)
{
    EtraxState *s = opaque;

    while (size-- && s->rx_len < sizeof(s->rx_fifo)) {
        s->rx_fifo[s->rx_len++] = *buf++;
    }
    etrax_update_irq(s);
}

static uint64_t etrax_io_read(void *opaque, hwaddr addr, unsigned size)
{
    EtraxState *s = opaque;
    CPUCRISState *env = &s->cpu->env;
    uint32_t v;

    if (addr >= R_MMU && addr < R_MMU_END) {
        v = crisv10_mmu_read(env, (addr - R_MMU) & ~3);
        return extract32(v, (addr & 3) * 8, size * 8);
    }

    switch (addr) {
    case R_TIMER_CTRL:              /* R_TIMER_DATA */
        if (size == 4) {
            return etrax_timer_count(s, 1) << 24 |
                   etrax_timer_count(s, 0) << 16;
        }
        break;
    case R_TIMER_CTRL + 2:          /* R_TIMER0_DATA */
        return etrax_timer_count(s, 0);
    case R_TIMER_CTRL + 3:          /* R_TIMER1_DATA */
        return etrax_timer_count(s, 1);
    case R_SERIAL0:                 /* R_SERIAL0_READ / R_SERIAL0_REC_DATA */
        v = etrax_serial_status(s) << 8;
        return size == 1 ? etrax_serial_pop(s)
                         : v | etrax_serial_pop(s);
    case R_SERIAL0 + 1:             /* R_SERIAL0_STATUS */
        return etrax_serial_status(s);
    case R_IRQ_MASK0:               /* R_IRQ_MASKx_RD */
    case R_IRQ_MASK1:
    case R_IRQ_MASK2:
        return etrax_irq_read(s, (addr - R_IRQ_MASK0) / 8, true);
    case R_IRQ_MASK0 + 4:           /* R_IRQ_READx */
    case R_IRQ_MASK1 + 4:
    case R_IRQ_MASK2 + 4:
        return etrax_irq_read(s, (addr - R_IRQ_MASK0) / 8, false);
    case R_VECT_MASK:               /* R_VECT_MASK_RD */
        return etrax_vect_read(s, true);
    case R_VECT_MASK + 4:           /* R_VECT_READ */
        return etrax_vect_read(s, false);
    case R_TIM_PRESC_STATUS:
        return 0;
    }
    if (addr >= R_DMA_CMD && addr < R_DMA_CMD + 40 && !(addr & 3)) {
        return 0;                   /* R_DMA_CHn_CMD: commands complete */
    }

    v = 0;
    for (unsigned i = 0; i < size; i++) {
        v |= s->regs[(addr + i) % ETRAX_IO_SIZE] << (8 * i);
    }
    return v;
}

static void etrax_io_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    EtraxState *s = opaque;
    CPUCRISState *env = &s->cpu->env;
    uint8_t ch;


    if (addr >= R_MMU && addr < R_MMU_END) {
        crisv10_mmu_write(env, addr - R_MMU, val, size);
        return;
    }

    switch (addr) {
    case R_TIMER_CTRL:
        if (size == 4) {
            etrax_timer_ctrl_write(s, val);
            return;
        }
        break;
    case R_SERIAL0:
        if (size == 1) {            /* R_SERIAL0_TR_DATA */
            ch = val;
            qemu_chr_fe_write_all(&s->chr, &ch, 1);
            return;
        }
        break;                      /* R_SERIAL0_CTRL */
    case R_IRQ_MASK0:               /* R_IRQ_MASKx_CLR */
    case R_IRQ_MASK1:
    case R_IRQ_MASK2:
        s->irq_mask[(addr - R_IRQ_MASK0) / 8] &= ~val;
        etrax_update_irq(s);
        return;
    case R_IRQ_MASK0 + 4:           /* R_IRQ_MASKx_SET */
    case R_IRQ_MASK1 + 4:
    case R_IRQ_MASK2 + 4:
        s->irq_mask[(addr - R_IRQ_MASK0) / 8] |= val;
        etrax_update_irq(s);
        return;
    case R_VECT_MASK:               /* R_VECT_MASK_CLR */
        s->vect_mask &= ~val;
        etrax_update_irq(s);
        return;
    case R_VECT_MASK + 4:           /* R_VECT_MASK_SET */
        s->vect_mask |= val;
        etrax_update_irq(s);
        return;
    case R_TIMER_PRESCALE:
        s->prescale = val & 0xffff;
        etrax_timer_arm(s, 0);
        return;
    }

    for (unsigned i = 0; i < size; i++) {
        s->regs[(addr + i) % ETRAX_IO_SIZE] = val >> (8 * i);
    }
}

static const MemoryRegionOps etrax_io_ops = {
    .read = etrax_io_read,
    .write = etrax_io_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static uint64_t etrax_translate(void *opaque, uint64_t addr)
{
    return addr & 0x7fffffff;           /* 0xc... runs at 0x4... */
}

static void etrax_reset(void *opaque)
{
    EtraxState *s = opaque;
    CPUCRISState *env = &s->cpu->env;

    cpu_reset(CPU(s->cpu));
    memset(s->irq_mask, 0, sizeof(s->irq_mask));
    s->vect_mask = 0;
    s->timer_ctrl = 0;
    s->timer_irq[0] = s->timer_irq[1] = false;
    timer_del(s->timer[0]);
    timer_del(s->timer[1]);
    s->rx_len = 0;
    s->vector = 0;
    qemu_set_irq(qdev_get_gpio_in(DEVICE(s->cpu), CRIS_CPU_IRQ), 0);

    if (s->entry) {
        env->pc = s->entry;
        env->regs[8] = RAM_INIT_MAGIC;
        if (s->cmdline) {
            env->regs[10] = COMMAND_LINE_MAGIC;
            env->regs[11] = s->cmdline;
        }
    }
}

/*
 * The command line goes to the last 4K of DRAM, the initrd page aligned
 * below it.  Linux 4.9 needs kernel-cris-initrd.patch to take the initrd,
 * announced as "initrd=<start>,<size>" on the command line.
 */
static void etrax_load_boot_data(MachineState *ms, EtraxState *s)
{
    hwaddr top = ETRAX_DRAM_BASE + ms->ram_size - 4 * KiB;
    char buf[256] = { 0 };          /* the kernel copies 256 bytes */

    pstrcpy(buf, sizeof(buf), ms->kernel_cmdline ? ms->kernel_cmdline : "");
    if (ms->initrd_filename) {
        int64_t size = get_image_size(ms->initrd_filename, NULL);
        hwaddr start;

        if (size <= 0) {
            error_report("could not load initrd '%s'", ms->initrd_filename);
            exit(1);
        }
        start = (top - size) & ~(hwaddr)(8 * KiB - 1);
        if (start < ETRAX_DRAM_BASE + ms->ram_size / 2 ||
            load_image_targphys(ms->initrd_filename, start, size,
                                NULL) != size) {
            error_report("could not load initrd '%s'", ms->initrd_filename);
            exit(1);
        }
        snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf),
                 "%sinitrd=0x%" HWADDR_PRIx ",%" PRId64,
                 buf[0] ? " " : "", start, size);
    }
    if (buf[0]) {
        s->cmdline = top;
        rom_add_blob_fixed("cmdline", buf, sizeof(buf), s->cmdline);
    }
}

static void etrax_init(MachineState *ms)
{
    MemoryRegion *sysmem = get_system_memory();
    EtraxState *s = g_new0(EtraxState, 1);
    uint64_t entry;

    s->cpu = CRIS_CPU(cpu_create(ms->cpu_type));
    memory_region_add_subregion(sysmem, ETRAX_DRAM_BASE, ms->ram);
    memory_region_init_io(&s->io, NULL, &etrax_io_ops, s, "etrax100lx.io",
                          ETRAX_IO_SIZE);
    memory_region_add_subregion(sysmem, ETRAX_IO_BASE, &s->io);

    s->timer[0] = timer_new_ns(QEMU_CLOCK_VIRTUAL, etrax_timer0_cb, s);
    s->timer[1] = timer_new_ns(QEMU_CLOCK_VIRTUAL, etrax_timer1_cb, s);
    s->prescale = 1000;
    qemu_chr_fe_init(&s->chr, serial_hd(0), &error_fatal);
    qemu_chr_fe_set_handlers(&s->chr, etrax_serial_can_rx, etrax_serial_rx,
                             NULL, NULL, s, NULL, true);

    if (ms->kernel_filename) {
        if (load_elf(ms->kernel_filename, NULL, etrax_translate, NULL,
                     &entry, NULL, NULL, NULL, ELFDATA2LSB, EM_CRIS,
                     0, 0) < 0) {
            error_report("could not load kernel '%s'", ms->kernel_filename);
            exit(1);
        }
        s->entry = entry;
        etrax_load_boot_data(ms, s);
    }
    qemu_register_reset(etrax_reset, s);
}

static void etrax_machine_init(MachineClass *mc)
{
    mc->desc = "Axis ETRAX 100LX (CRISv10)";
    mc->init = etrax_init;
    mc->default_cpu_type = CRIS_CPU_TYPE_NAME("crisv10");
    mc->default_ram_size = 64 * MiB;
    mc->default_ram_id = "etrax100lx.dram";
}

DEFINE_MACHINE("etrax100lx", etrax_machine_init)
