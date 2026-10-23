/*
 * A 68000 system built around the MC68328 (DragonBall) peripherals
 *
 * Linux runs the plain 68000 only on the 68328 family; it needs the
 * 68328 interrupt controller and timer 1. Those are modelled here, with
 * RAM from address 0 and a 16550 UART as console (Linux 6.18 no longer
 * has a 68328 serial driver; the kernel side is CONFIG_M68000_QEMU).
 *
 * -kernel takes a vmlinux. -append and -initrd are handed over the way
 * U-Boot does it on ColdFire (CONFIG_UBOOT, arch/m68k/kernel/uboot.c).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/error-report.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "cpu.h"
#include "elf.h"
#include "hw/core/boards.h"
#include "hw/core/irq.h"
#include "hw/core/loader.h"
#include "hw/char/serial-mm.h"
#include "system/address-spaces.h"
#include "system/reset.h"
#include "system/system.h"

#define M68328_REGS_BASE    0xfffff000
#define M68328_UART_BASE    0xffffe000
#define M68328_UART_IRQ     2           /* the 68328 UART line */
/*
 * Linux programs timer 1 with the 68VZ328 numbering (arch/m68k/68000/
 * timers.c includes MC68VZ328.h), where it is interrupt 1.
 */
#define M68328_TMR1_IRQ     1
#define M68328_SYSCLK       16580608    /* 32768 Hz * 506 */
#define M68328_ARGS_SIZE    (4 * KiB)

/* interrupt controller */
#define REG_IVR             0x300
#define REG_ICR             0x302
#define REG_IMR             0x304
#define REG_IWR             0x308
#define REG_ISR             0x30c
#define REG_IPR             0x310
/* timer 1 */
#define REG_TCTL1           0x600
#define REG_TPRER1          0x602
#define REG_TCMP1           0x604
#define REG_TCR1            0x606
#define REG_TCN1            0x608
#define REG_TSTAT1          0x60a
#define REG_RTCTIME         0xb00

#define TCTL_TEN            0x0001
#define TCTL_CLK_MASK       0x000e
#define TCTL_CLK_SYSCLK     0x0002
#define TCTL_CLK_SYSCLK16   0x0004
#define TCTL_CLK_32KHZ      0x0008
#define TCTL_IRQEN          0x0010
#define TCTL_FRR            0x0100
#define TSTAT_COMP          0x0001

typedef struct M68328State {
    M68kCPU *cpu;
    MemoryRegion regs;
    QEMUTimer *timer;

    uint8_t ivr;
    uint32_t imr;
    uint32_t ipr;                       /* raw request lines */
    uint16_t tctl, tprer, tcmp, tstat;
    int64_t tcn_base;                   /* time TCN was zero */
    uint8_t other[0x1000];              /* everything else: plain storage */

    uint32_t entry;
    hwaddr args;
} M68328State;

/* the 68328 assigns each source a fixed level */
static int m68328_level(int irq)
{
    switch (irq) {
    case M68328_TMR1_IRQ:
        return 6;
    case 16:                            /* IRQ1 .. IRQ3, IRQ6 */
        return 1;
    case 17:
        return 2;
    case 18:
        return 3;
    case 19:
        return 6;
    case 23:                            /* IRQ7 */
        return 7;
    default:
        return 4;
    }
}

static void m68328_update(M68328State *s)
{
    uint32_t pend = s->ipr & ~s->imr;
    int level = 0;

    for (int i = 0; i < 32; i++) {
        if (pend & (1u << i)) {
            level = MAX(level, m68328_level(i));
        }
    }
    m68k_set_irq_level(s->cpu, level, level ? (s->ivr & 0xf8) | level : 0);
}

static void m68328_set_irq(void *opaque, int irq, int level)
{
    M68328State *s = opaque;

    if (level) {
        s->ipr |= 1u << irq;
    } else {
        s->ipr &= ~(1u << irq);
    }
    m68328_update(s);
}

static int64_t m68328_tick_ns(M68328State *s)
{
    int64_t hz;

    switch (s->tctl & TCTL_CLK_MASK) {
    case TCTL_CLK_SYSCLK:
        hz = M68328_SYSCLK;
        break;
    case TCTL_CLK_SYSCLK16:
        hz = M68328_SYSCLK / 16;
        break;
    default:
        hz = 32768;
        break;
    }
    return NANOSECONDS_PER_SECOND * (s->tprer + 1) / hz;
}

static uint32_t m68328_tcn(M68328State *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (!(s->tctl & TCTL_TEN)) {
        return 0;
    }
    return ((now - s->tcn_base) / m68328_tick_ns(s)) & 0xffff;
}

static void m68328_timer_arm(M68328State *s)
{
    timer_del(s->timer);
    if ((s->tctl & TCTL_TEN) && s->tcmp) {
        timer_mod(s->timer, s->tcn_base + s->tcmp * m68328_tick_ns(s));
    }
}

static void m68328_timer_irq(M68328State *s)
{
    m68328_set_irq(s, M68328_TMR1_IRQ,
                   (s->tstat & TSTAT_COMP) && (s->tctl & TCTL_IRQEN));
}

static void m68328_timer_cb(void *opaque)
{
    M68328State *s = opaque;

    s->tstat |= TSTAT_COMP;
    m68328_timer_irq(s);
    /* restart mode counts from zero again; free-run mode wraps at 0x10000 */
    s->tcn_base += (s->tctl & TCTL_FRR ? 0x10000 : s->tcmp) *
                   m68328_tick_ns(s);
    m68328_timer_arm(s);
}

static uint32_t m68328_reg_read(M68328State *s, hwaddr addr, unsigned size)
{
    uint32_t v = 0;

    for (unsigned i = 0; i < size; i++) {
        v = v << 8 | s->other[(addr + i) & 0xfff];
    }
    return v;
}

static void m68328_reg_write(M68328State *s, hwaddr addr, uint32_t val,
                             unsigned size)
{
    for (unsigned i = 0; i < size; i++) {
        s->other[(addr + i) & 0xfff] = val >> (8 * (size - 1 - i));
    }
}

static uint64_t m68328_read(void *opaque, hwaddr addr, unsigned size)
{
    M68328State *s = opaque;
    int64_t now;
    uint64_t v;

    switch (addr) {
    case REG_IVR:
        return s->ivr;
    case REG_IMR:
        return size == 4 ? s->imr : s->imr >> 16;
    case REG_IMR + 2:
        return s->imr & 0xffff;
    case REG_ISR:
        v = s->ipr & ~s->imr;
        return size == 4 ? v : v >> 16;
    case REG_ISR + 2:
        return (s->ipr & ~s->imr) & 0xffff;
    case REG_IPR:
        return size == 4 ? s->ipr : s->ipr >> 16;
    case REG_IPR + 2:
        return s->ipr & 0xffff;
    case REG_TCTL1:
        return s->tctl;
    case REG_TPRER1:
        return s->tprer;
    case REG_TCMP1:
        return s->tcmp;
    case REG_TCN1:
        return m68328_tcn(s);
    case REG_TSTAT1:
        return s->tstat;
    case REG_RTCTIME:
        now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / NANOSECONDS_PER_SECOND;
        v = (now / 3600 % 24) << 24 | (now / 60 % 60) << 16 | now % 60;
        return size == 4 ? v : v >> 16;
    default:
        return m68328_reg_read(s, addr, size);
    }
}

static void m68328_write(void *opaque, hwaddr addr, uint64_t val,
                         unsigned size)
{
    M68328State *s = opaque;

    switch (addr) {
    case REG_IVR:
        s->ivr = val;
        m68328_update(s);
        return;
    case REG_IMR:
        if (size == 4) {
            s->imr = val;
        } else {
            s->imr = (s->imr & 0xffff) | (uint32_t)val << 16;
        }
        m68328_update(s);
        return;
    case REG_IMR + 2:
        s->imr = (s->imr & 0xffff0000) | (val & 0xffff);
        m68328_update(s);
        return;
    case REG_TCTL1:
        if ((val & TCTL_TEN) && !(s->tctl & TCTL_TEN)) {
            s->tcn_base = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        }
        s->tctl = val;
        m68328_timer_arm(s);
        m68328_timer_irq(s);
        return;
    case REG_TPRER1:
        s->tprer = val;
        m68328_timer_arm(s);
        return;
    case REG_TCMP1:
        s->tcmp = val;
        m68328_timer_arm(s);
        return;
    case REG_TSTAT1:
        /* a status bit is cleared by writing 0 after it was read set */
        s->tstat &= val;
        m68328_timer_irq(s);
        return;
    default:
        m68328_reg_write(s, addr, val, size);
        return;
    }
}

static const MemoryRegionOps m68328_ops = {
    .read = m68328_read,
    .write = m68328_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/* see hw/m68k/mcf5208.c: sp[2..5] are initrd start/end, cmdline start/end */
static void m68328_load(MachineState *ms, M68328State *s)
{
    const char *cmdline = ms->kernel_cmdline;
    hwaddr cmd = s->args + 256, initrd_start = 0, initrd_end = 0;
    hwaddr cmd_start = 0, cmd_end = 0;
    uint64_t entry, high;
    uint32_t sp[6];

    if (load_elf(ms->kernel_filename, NULL, NULL, NULL, &entry, NULL, &high,
                 NULL, ELFDATA2MSB, EM_68K, 0, 0) < 0) {
        error_report("could not load kernel '%s'", ms->kernel_filename);
        exit(1);
    }
    s->entry = entry;

    if (cmdline && *cmdline) {
        size_t len = MIN(strlen(cmdline) + 1, M68328_ARGS_SIZE - 256);

        rom_add_blob_fixed("cmdline", cmdline, len, cmd);
        cmd_start = cmd;
        cmd_end = cmd + len - 1;
    }
    if (ms->initrd_filename) {
        ssize_t size = get_image_size(ms->initrd_filename, NULL);

        if (size < 0) {
            error_report("could not load initrd '%s'", ms->initrd_filename);
            exit(1);
        }
        initrd_start = QEMU_ALIGN_DOWN(s->args - size, 4 * KiB);
        if (initrd_start < high ||
            load_image_targphys(ms->initrd_filename, initrd_start,
                                s->args - initrd_start, NULL) != size) {
            error_report("could not load initrd '%s'", ms->initrd_filename);
            exit(1);
        }
        initrd_end = initrd_start + size;
    }

    sp[0] = 0;                          /* return address */
    sp[1] = 0;                          /* bd_info */
    sp[2] = cpu_to_be32(initrd_start);
    sp[3] = cpu_to_be32(initrd_end);
    sp[4] = cpu_to_be32(cmd_start);
    sp[5] = cpu_to_be32(cmd_end);
    rom_add_blob_fixed("uboot-args", sp, sizeof(sp), s->args);
}

static void m68328_reset(void *opaque)
{
    M68328State *s = opaque;
    CPUM68KState *env = &s->cpu->env;

    cpu_reset(CPU(s->cpu));
    s->ivr = 0;
    s->imr = 0x00ffffff;
    s->tctl = s->tprer = s->tcmp = s->tstat = 0;
    timer_del(s->timer);
    m68328_update(s);
    if (s->entry) {
        env->pc = s->entry;
        env->aregs[7] = s->args;
    }
}

static void m68328_init(MachineState *ms)
{
    MemoryRegion *sysmem = get_system_memory();
    M68328State *s = g_new0(M68328State, 1);
    qemu_irq *irqs;

    s->cpu = M68K_CPU(cpu_create(ms->cpu_type));
    memory_region_add_subregion(sysmem, 0, ms->ram);

    memory_region_init_io(&s->regs, NULL, &m68328_ops, s, "m68328",
                          0x1000);
    memory_region_add_subregion(sysmem, M68328_REGS_BASE, &s->regs);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, m68328_timer_cb, s);
    irqs = qemu_allocate_irqs(m68328_set_irq, s, 32);

    serial_mm_init(sysmem, M68328_UART_BASE, 0, irqs[M68328_UART_IRQ],
                   115200, serial_hd(0), DEVICE_BIG_ENDIAN);

    s->args = ms->ram_size - M68328_ARGS_SIZE;
    if (ms->kernel_filename) {
        m68328_load(ms, s);
    }
    qemu_register_reset(m68328_reset, s);
}

static void m68328_machine_init(MachineClass *mc)
{
    mc->desc = "68000 with MC68328 peripherals";
    mc->init = m68328_init;
    mc->default_cpu_type = M68K_CPU_TYPE_NAME("m68000");
    mc->default_ram_size = 128 * MiB;
    mc->default_ram_id = "m68328.ram";
}

DEFINE_MACHINE("m68328", m68328_machine_init)
