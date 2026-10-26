/*
 * The "GDB" V850E platform of Linux (arch/v850, CONFIG_V850E_SIM)
 *
 * The kernel ran on the v850 simulator of gdb and talks to it only
 * through "trap 31" system calls: console via read/write/poll, time
 * via gettimeofday, the root filesystem as the host file rootfs.image.
 * Interrupt 0 is a periodic tick.  Here -initrd stands in for
 * rootfs.image and the first serial for the console.
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/error-report.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "qemu/fifo8.h"
#include "qapi/error.h"
#include "cpu.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "chardev/char-fe.h"
#include "system/system.h"
#include "system/reset.h"
#include "system/runstate.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "hw/core/irq.h"
#include "elf.h"

#define SIM_LOW_RAM_SIZE    (1 * MiB)       /* the vectors at 0 */
#define SIM_RAM_BASE        0x8f000000
#define SIM_R0_RAM_BASE     0xfffff000      /* reached by [r0] - n */
#define SIM_TICK_HZ         24              /* HZ of the sim kernel */

#define SIM_ROOT_FD         3

enum {
    SIMSYS_exit = 1,
    SIMSYS_read = 3,
    SIMSYS_write = 4,
    SIMSYS_open = 5,
    SIMSYS_close = 6,
    SIMSYS_lseek = 19,
    SIMSYS_time = 23,
    SIMSYS_poll = 24,
    SIMSYS_stat = 38,
    SIMSYS_gettimeofday = 116,
    SIMSYS_make_raw = 1024,
};

typedef struct V850Sim {
    V850CPU *cpu;
    CharFrontend chr;
    Fifo8 rx;
    QEMUTimer *tick;
    uint8_t *root;
    gsize root_size;
    uint32_t root_pos;
    bool root_open;
} V850Sim;

static V850Sim sim;

static int sim_can_receive(void *opaque)
{
    return fifo8_num_free(&sim.rx);
}

static void sim_receive(void *opaque, const uint8_t *buf, int size)
{
    fifo8_push_all(&sim.rx, buf, size);
}

static void sim_tick(void *opaque)
{
    qemu_irq irq = qdev_get_gpio_in_named(DEVICE(sim.cpu), "IRQ", 0);

    qemu_irq_pulse(irq);
    timer_mod(sim.tick, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / SIM_TICK_HZ);
}

static void sim_mem_read(uint32_t addr, void *buf, uint32_t len)
{
    address_space_read(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                       buf, len);
}

static void sim_mem_write(uint32_t addr, const void *buf, uint32_t len)
{
    address_space_write(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                        buf, len);
}

static void sim_put32(uint32_t addr, uint32_t val)
{
    uint8_t b[4];

    stl_le_p(b, val);
    sim_mem_write(addr, b, 4);
}

static bool sim_is_root(uint32_t name)
{
    static const char root[] = "rootfs.image";
    char buf[sizeof(root)];

    sim_mem_read(name, buf, sizeof(buf));
    return sim.root && !memcmp(buf, root, sizeof(root));
}

static int32_t sim_read(uint32_t fd, uint32_t buf, uint32_t len)
{
    uint32_t n = 0;

    if (fd == 0) {
        while (n < len && !fifo8_is_empty(&sim.rx)) {
            uint8_t c = fifo8_pop(&sim.rx);

            sim_mem_write(buf + n++, &c, 1);
        }
        qemu_chr_fe_accept_input(&sim.chr);
        return n;
    }
    if (fd == SIM_ROOT_FD && sim.root_open) {
        n = MIN(len, sim.root_size - sim.root_pos);
        sim_mem_write(buf, sim.root + sim.root_pos, n);
        sim.root_pos += n;
        return n;
    }
    return -1;
}

static int32_t sim_write(uint32_t fd, uint32_t buf, uint32_t len)
{
    uint8_t tmp[256];
    uint32_t done = 0;

    if (fd != 1 && fd != 2) {
        return -1;
    }
    while (done < len) {
        uint32_t n = MIN(len - done, sizeof(tmp));

        sim_mem_read(buf + done, tmp, n);
        qemu_chr_fe_write_all(&sim.chr, tmp, n);
        done += n;
    }
    return len;
}

/* struct pollfd { int fd; short events, revents; } */
static int32_t sim_poll(uint32_t pfd, uint32_t nfds)
{
    int32_t ready = 0;

    for (uint32_t i = 0; i < nfds; i++) {
        uint8_t p[8];
        uint16_t revents = 0;

        sim_mem_read(pfd + i * 8, p, 8);
        if (ldl_le_p(p) == 0 && (lduw_le_p(p + 4) & 1) &&
            !fifo8_is_empty(&sim.rx)) {
            revents = 1;        /* POLLIN */
            ready++;
        }
        stw_le_p(p + 6, revents);
        sim_mem_write(pfd + i * 8 + 6, p + 6, 2);
    }
    return ready;
}

static void sim_syscall(CPUV850State *env)
{
    uint32_t a0 = env->gpr[7], a1 = env->gpr[8], a2 = env->gpr[9];
    int32_t ret = 0;
    struct timeval tv;

    switch (env->gpr[6]) {
    case SIMSYS_exit:
        qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
        break;
    case SIMSYS_read:
        ret = sim_read(a0, a1, a2);
        break;
    case SIMSYS_write:
        ret = sim_write(a0, a1, a2);
        break;
    case SIMSYS_open:
        if (sim_is_root(a0)) {
            sim.root_open = true;
            sim.root_pos = 0;
            ret = SIM_ROOT_FD;
        } else {
            ret = -1;
        }
        break;
    case SIMSYS_close:
        if (a0 == SIM_ROOT_FD) {
            sim.root_open = false;
        }
        break;
    case SIMSYS_lseek:
        if (a0 != SIM_ROOT_FD || !sim.root_open) {
            ret = -1;
            break;
        }
        switch (a2) {
        case 0:
            sim.root_pos = a1;
            break;
        case 1:
            sim.root_pos += a1;
            break;
        case 2:
            sim.root_pos = sim.root_size + a1;
            break;
        }
        sim.root_pos = MIN(sim.root_pos, sim.root_size);
        ret = sim.root_pos;
        break;
    case SIMSYS_time:
        gettimeofday(&tv, NULL);
        ret = tv.tv_sec;
        if (a0) {
            sim_put32(a0, ret);
        }
        break;
    case SIMSYS_poll:
        ret = sim_poll(a0, a1);
        break;
    case SIMSYS_stat:
        /*
         * newlib's v850 struct stat has st_size at offset 16; Linux
         * passes only 40 bytes, so write no further.
         */
        if (sim_is_root(a0)) {
            uint8_t zero[40] = { 0 };

            sim_mem_write(a1, zero, sizeof(zero));
            sim_put32(a1 + 16, sim.root_size);
        } else {
            ret = -1;
        }
        break;
    case SIMSYS_gettimeofday:
        gettimeofday(&tv, NULL);
        if (a0) {
            sim_put32(a0, tv.tv_sec);
            sim_put32(a0 + 4, tv.tv_usec);
        }
        if (a1) {
            sim_put32(a1, 0);
            sim_put32(a1 + 4, 0);
        }
        break;
    case SIMSYS_make_raw:
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "v850-sim: system call %u\n", env->gpr[6]);
        ret = -1;
        break;
    }
    env->gpr[10] = ret;
}

static void sim_cpu_reset(void *opaque)
{
    cpu_reset(CPU(opaque));
}

static void v850_sim_init(MachineState *ms)
{
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *low = g_new(MemoryRegion, 1);
    MemoryRegion *r0ram = g_new(MemoryRegion, 1);
    uint64_t entry;
    int64_t size;

    sim.cpu = V850_CPU(cpu_create(ms->cpu_type));
    sim.cpu->env.sim_syscall = sim_syscall;

    memory_region_add_subregion(sysmem, SIM_RAM_BASE, ms->ram);
    memory_region_init_ram(low, NULL, "v850-sim.low", SIM_LOW_RAM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, 0, low);
    memory_region_init_ram(r0ram, NULL, "v850-sim.r0ram", 4 * KiB,
                           &error_fatal);
    memory_region_add_subregion(sysmem, SIM_R0_RAM_BASE, r0ram);

    fifo8_create(&sim.rx, 256);
    qemu_chr_fe_init(&sim.chr, serial_hd(0), &error_abort);
    qemu_chr_fe_set_handlers(&sim.chr, sim_can_receive, sim_receive, NULL,
                             NULL, &sim, NULL, true);

    if (!ms->kernel_filename) {
        error_report("v850-sim: -kernel is required");
        exit(EXIT_FAILURE);
    }
    size = load_elf(ms->kernel_filename, NULL, NULL, NULL, &entry, NULL,
                    NULL, NULL, ELFDATA2LSB, EM_NONE, 0, 0);
    if (size < 0) {
        error_report("v850-sim: could not load kernel '%s'",
                     ms->kernel_filename);
        exit(EXIT_FAILURE);
    }

    if (ms->initrd_filename) {
        GError *err = NULL;

        if (!g_file_get_contents(ms->initrd_filename, (gchar **)&sim.root,
                                 &sim.root_size, &err)) {
            error_report("v850-sim: could not read '%s': %s",
                         ms->initrd_filename, err->message);
            exit(EXIT_FAILURE);
        }
    }

    sim.tick = timer_new_ns(QEMU_CLOCK_VIRTUAL, sim_tick, NULL);
    timer_mod(sim.tick, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / SIM_TICK_HZ);

    /* the reset vector at 0 jumps to the kernel */
    qemu_register_reset(sim_cpu_reset, sim.cpu);
}

static void v850_sim_machine_init(MachineClass *mc)
{
    mc->desc = "Linux's GDB V850E simulator platform (V850E1 CPU)";
    mc->init = v850_sim_init;
    mc->default_cpu_type = TYPE_V850_CPU;
    mc->default_ram_size = 48 * MiB;
    mc->default_ram_id = "v850-sim.ram";
    mc->is_default = true;
}

DEFINE_MACHINE("v850-sim", v850_sim_machine_init)
