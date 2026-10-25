/*
 * TILE-Gx machine: one tile and the hypervisor's client interface
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * A TILE-Gx Linux kernel never touches the chip's devices; it is a client
 * of the Tilera hypervisor and reaches the console, memory layout, page
 * tables and boot files through hypervisor calls (arch/tile/include/hv/
 * hypervisor.h).  The kernel calls them by jumping to slot N of the glue
 * page (HV_GLUE_START_CPA + N * 32), which the translator turns into
 * TILEGX_EXCP_HV_CALL; tilegx_hv_call() below serves the call and returns
 * to lr.  Arguments are in r0..r9, results in r0..r9 as for a C function.
 *
 * One tile at coordinate (0,0), RAM from PA 0, -initrd as the hvfs file
 * "initramfs", -append as the command line.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/error-report.h"
#include "qemu/cutils.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "elf.h"
#include "chardev/char-fe.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "system/address-spaces.h"
#include "system/reset.h"
#include "system/runstate.h"
#include "system/system.h"
#include "system/rtc.h"
#include "target/tilegx/cpu.h"

/* Where Linux links itself (arch/tile/include/asm/page.h, tilegx) */
#define TILEGX_MEM_SV_START     0xfffffff700000000ULL
#define TILEGX_PAGE_OFFSET      0xfffffe0000000000ULL
#define TILEGX_HV_GLUE_CPA      0x10000ULL

/* IPI trigger page: a store to word N raises IPI event N */
#define TILEGX_IPI_BASE         0x8000000000ULL
#define TILEGX_IPI_SIZE         (64 * KiB)

#define HV_EINVAL       -801
#define HV_ENODEV       -802
#define HV_ENOENT       -803
#define HV_EBADF        -804
#define HV_EFAULT       -805

enum {
    HV_INIT = 1, HV_INSTALL_CONTEXT, HV_SYSCONF, HV_GET_RTC, HV_SET_RTC,
    HV_FLUSH_ASID, HV_FLUSH_PAGE, HV_FLUSH_PAGES, HV_RESTART, HV_HALT,
    HV_POWER_OFF, HV_INQUIRE_PHYSICAL, HV_INQUIRE_MEMORY_CONTROLLER,
    HV_INQUIRE_VIRTUAL, HV_INQUIRE_ASID, HV_NANOSLEEP,
    HV_CONSOLE_READ_IF_READY, HV_CONSOLE_WRITE, HV_DOWNCALL_DISPATCH,
    HV_INQUIRE_TOPOLOGY, HV_FS_FINDFILE, HV_FS_FSTAT, HV_FS_PREAD,
    HV_PHYSADDR_READ64, HV_PHYSADDR_WRITE64, HV_GET_COMMAND_LINE,
    HV_SET_CACHING, HV_BZERO_PAGE, HV_REGISTER_MESSAGE_STATE,
    HV_SEND_MESSAGE, HV_RECEIVE_MESSAGE, HV_INQUIRE_CONTEXT,
    HV_START_ALL_TILES, HV_DEV_OPEN, HV_DEV_CLOSE, HV_DEV_PREAD,
    HV_DEV_PWRITE, HV_DEV_POLL, HV_DEV_POLL_CANCEL, HV_DEV_PREADA,
    HV_DEV_PWRITEA, HV_FLUSH_REMOTE, HV_CONSOLE_PUTC, HV_INQUIRE_TILES,
    HV_CONFSTR, HV_REEXEC, HV_SET_COMMAND_LINE, HV_CLEAR_INTR,
    HV_ENABLE_INTR, HV_DISABLE_INTR, HV_RAISE_INTR, HV_TRIGGER_IPI,
    HV_STORE_MAPPING, HV_INQUIRE_REALPA, HV_FLUSH_ALL, HV_GET_IPI_PTE,
    HV_SET_PTE_SUPER_SHIFT,
    HV_CONSOLE_SET_IPI = 63,
    HV_SEND_NMI = 65,
};

#define HV_SYSCONF_GLUE_SIZE        1
#define HV_SYSCONF_PAGE_SIZE_SMALL  2
#define HV_SYSCONF_PAGE_SIZE_LARGE  3
#define HV_SYSCONF_CPU_SPEED        4
#define HV_SYSCONF_CPU_TEMP         5
#define HV_SYSCONF_BOARD_TEMP       6
#define HV_SYSCONF_VALID_PAGE_SIZES 7
#define HV_SYSCONF_PAGE_SIZE_JUMBO  8

#define HV_CONFSTR_CHIP_MODEL       6
#define HV_CONFSTR_BOARD_DESC       7
#define HV_CONFSTR_HV_SW_VER        5

#define HV_CTX_PG_SM_4K     0x10
#define HV_CTX_PG_SM_16K    0x20
#define HV_CTX_PG_SM_64K    0x40

#define HV_PTE_PRESENT      (1ULL << 0)
#define HV_PTE_MODE_MMIO    (5ULL << 16)
#define HV_PTE_READABLE     (1ULL << 32)
#define HV_PTE_WRITABLE     (1ULL << 33)
#define HV_PTE_PTFN_SHIFT   35

typedef struct TileGXState {
    TileGXCPU *cpu;
    MachineState *ms;
    CharFrontend chr;
    MemoryRegion ipi;
    uint64_t entry;

    /* console input and the IPI it raises */
    uint8_t rx[256];
    int rx_head, rx_count;
    int console_ipi_pl, console_ipi_event;

    /* hvfs: the initramfs */
    uint8_t *initrd;
    int64_t initrd_size;
} TileGXState;

static TileGXState *tilegx_state;

/* ------------------------------------------------------------------ */
/* Console                                                             */

static void console_raise(TileGXState *s)
{
    CPUTLGState *env = &s->cpu->env;

    if (s->console_ipi_event >= 0 && s->rx_count) {
        env->ipi_event[s->console_ipi_pl] |= 1ULL << s->console_ipi_event;
        tilegx_update_irq(env);
    }
}

static int tilegx_chr_can_rx(void *opaque)
{
    TileGXState *s = opaque;

    return sizeof(s->rx) - s->rx_count;
}

static void tilegx_chr_rx(void *opaque, const uint8_t *buf, int size)
{
    TileGXState *s = opaque;
    int i;

    for (i = 0; i < size && s->rx_count < sizeof(s->rx); i++) {
        s->rx[(s->rx_head + s->rx_count++) % sizeof(s->rx)] = buf[i];
    }
    console_raise(s);
}

static int console_getc(TileGXState *s)
{
    int c;

    if (!s->rx_count) {
        return -1;
    }
    c = s->rx[s->rx_head];
    s->rx_head = (s->rx_head + 1) % sizeof(s->rx);
    s->rx_count--;
    qemu_chr_fe_accept_input(&s->chr);
    return c;
}

/* ------------------------------------------------------------------ */
/* IPI trigger page                                                    */

static uint64_t tilegx_ipi_read(void *opaque, hwaddr addr, unsigned size)
{
    return 0;
}

static void tilegx_ipi_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned size)
{
    TileGXState *s = opaque;
    CPUTLGState *env = &s->cpu->env;

    /* The kernel's IPIs go to its own protection level. */
    env->ipi_event[2] |= 1ULL << ((addr / 8) & 63);
    tilegx_update_irq(env);
}

static const MemoryRegionOps tilegx_ipi_ops = {
    .read = tilegx_ipi_read,
    .write = tilegx_ipi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
    .impl = { .min_access_size = 8, .max_access_size = 8 },
};

/* ------------------------------------------------------------------ */
/* Hypervisor calls                                                    */

static int guest_rw(CPUTLGState *env, uint64_t va, void *buf, int len,
                    bool is_write)
{
    return cpu_memory_rw_debug(env_cpu(env), va, buf, len, is_write);
}

static int guest_strlen_copy(CPUTLGState *env, uint64_t va, char *buf,
                             int size)
{
    int i;

    for (i = 0; i < size - 1; i++) {
        if (guest_rw(env, va + i, &buf[i], 1, false)) {
            return -1;
        }
        if (!buf[i]) {
            return i;
        }
    }
    buf[i] = 0;
    return i;
}

/* Copy a string out, return its length with the NUL like hv_confstr. */
static int put_string(CPUTLGState *env, uint64_t va, int len, const char *str)
{
    int n = strlen(str) + 1;

    if (len > 0 && va) {
        guest_rw(env, va, (void *)str, MIN(n, len), true);
    }
    return n;
}

void tilegx_hv_call(CPUTLGState *env, int index)
{
    TileGXState *s = tilegx_state;
    MachineState *ms = s->ms;
    uint64_t *r = env->regs;
    uint64_t a0 = r[0], a1 = r[1], a2 = r[2], a3 = r[3];
    uint64_t ret = 0;
    char name[128];

    switch (index) {
    case HV_INIT:
        /* (version, chip_num, chip_rev, client_pl) */
        break;
    case HV_INSTALL_CONTEXT:
        if (a3 & HV_CTX_PG_SM_4K) {
            env->log2_page_small = 12;
        } else if (a3 & HV_CTX_PG_SM_16K) {
            env->log2_page_small = 14;
        } else if (a3 & HV_CTX_PG_SM_64K) {
            env->log2_page_small = 16;
        }
        env->ctx_page_table = a0;
        env->ctx_access = a1;
        env->ctx_asid = a2;
        env->ctx_flags = a3;
        env->ctx_valid = true;
        tilegx_flush_tlb(env);
        break;
    case HV_INQUIRE_CONTEXT:
        r[1] = env->ctx_access;
        r[2] = env->ctx_asid | ((uint64_t)env->ctx_flags << 32);
        ret = env->ctx_page_table;
        break;
    case HV_SYSCONF:
        switch (a0) {
        case HV_SYSCONF_GLUE_SIZE:
            ret = 0x10000;
            break;
        case HV_SYSCONF_PAGE_SIZE_SMALL:
            ret = 1ULL << env->log2_page_small;
            break;
        case HV_SYSCONF_PAGE_SIZE_LARGE:
            ret = 1ULL << env->log2_page_large;
            break;
        case HV_SYSCONF_PAGE_SIZE_JUMBO:
            ret = 1ULL << 32;
            break;
        case HV_SYSCONF_CPU_SPEED:
            ret = env->cpu_speed;
            break;
        case HV_SYSCONF_CPU_TEMP:
        case HV_SYSCONF_BOARD_TEMP:
            ret = 40 + 273;
            break;
        case HV_SYSCONF_VALID_PAGE_SIZES:
            ret = HV_CTX_PG_SM_4K | HV_CTX_PG_SM_16K | HV_CTX_PG_SM_64K;
            break;
        default:
            ret = -1;
            break;
        }
        break;
    case HV_CONFSTR:
        switch (a0) {
        case HV_CONFSTR_CHIP_MODEL:
            ret = put_string(env, a1, a2, "TILE-Gx36");
            break;
        case HV_CONFSTR_BOARD_DESC:
            ret = put_string(env, a1, a2, "QEMU");
            break;
        case HV_CONFSTR_HV_SW_VER:
            ret = put_string(env, a1, a2, "qemu");
            break;
        default:
            ret = HV_EINVAL;
            break;
        }
        break;
    case HV_GET_RTC: {
        struct tm tm;

        qemu_get_timedate(&tm, 0);
        ret = tm.tm_sec;
        r[1] = tm.tm_min;
        r[2] = tm.tm_hour;
        r[3] = tm.tm_mday;
        r[4] = tm.tm_mon;
        r[5] = tm.tm_year;
        r[6] = 0;
        break;
    }
    case HV_SET_RTC:
        break;
    case HV_FLUSH_ASID:
    case HV_FLUSH_PAGE:
    case HV_FLUSH_PAGES:
    case HV_FLUSH_ALL:
    case HV_FLUSH_REMOTE:
        tilegx_flush_tlb(env);
        break;
    case HV_RESTART:
    case HV_REEXEC:
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        break;
    case HV_HALT:
    case HV_POWER_OFF:
        qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
        break;
    case HV_INQUIRE_PHYSICAL:
        /* {start, size, controller} */
        if (a0 == 0) {
            ret = 0;
            r[1] = ms->ram_size;
        } else {
            ret = 0;
            r[1] = 0;
        }
        r[2] = 0;
        break;
    case HV_INQUIRE_MEMORY_CONTROLLER:
        /* {coord, speed} */
        ret = 0;
        r[1] = 10ULL * 1000 * 1000 * 1000;
        break;
    case HV_INQUIRE_VIRTUAL:
        /* {start, size}: the user half and the kernel half */
        if (a0 == 0) {
            ret = 0;
            r[1] = 1ULL << 41;
        } else if (a0 == 1) {
            ret = -(1ULL << 41);
            r[1] = (1ULL << 41) - 0x100000000ULL;
        } else {
            ret = 0;
            r[1] = 0;
        }
        break;
    case HV_INQUIRE_ASID:
        /* {start, size} */
        if (a0 == 0) {
            ret = 1 | (255ULL << 32);
        } else {
            ret = 0;
        }
        break;
    case HV_INQUIRE_TOPOLOGY:
        /* {coord {x, y}, width, height} */
        ret = 0;
        r[1] = 1 | (1ULL << 32);
        break;
    case HV_INQUIRE_TILES: {
        /* (set, cpumask, length): tile 0 only */
        uint8_t one = 1, zero = 0;
        int i;

        ret = 0;
        for (i = 0; i < (int)a2; i++) {
            if (guest_rw(env, a1 + i, i ? &zero : &one, 1, true)) {
                ret = HV_EFAULT;
            }
        }
        break;
    }
    case HV_NANOSLEEP:
        break;
    case HV_CONSOLE_READ_IF_READY:
        ret = console_getc(s);
        break;
    case HV_CONSOLE_PUTC: {
        uint8_t c = a0;

        qemu_chr_fe_write_all(&s->chr, &c, 1);
        break;
    }
    case HV_CONSOLE_WRITE: {
        int len = a1, done = 0;
        uint8_t buf[256];

        while (done < len) {
            int n = MIN(len - done, (int)sizeof(buf));

            if (guest_rw(env, a0 + done, buf, n, false)) {
                break;
            }
            qemu_chr_fe_write_all(&s->chr, buf, n);
            done += n;
        }
        ret = done;
        break;
    }
    case HV_CONSOLE_SET_IPI:
        /* (ipi, event, coord) */
        s->console_ipi_pl = a0 & 3;
        s->console_ipi_event = (int32_t)a1;
        console_raise(s);
        break;
    case HV_GET_COMMAND_LINE: {
        const char *cmd = ms->kernel_cmdline ? ms->kernel_cmdline : "";
        int n = strlen(cmd);

        if (n >= (int)a1) {
            n = a1 - 1;
        }
        guest_rw(env, a0, (void *)cmd, n, true);
        guest_rw(env, a0 + n, (void *)"", 1, true);
        ret = n;
        break;
    }
    case HV_FS_FINDFILE:
        if (guest_strlen_copy(env, a0, name, sizeof(name)) < 0) {
            ret = HV_EFAULT;
        } else if (s->initrd && (!strcmp(name, "initramfs") ||
                                 !strcmp(name, "initramfs.cpio.gz"))) {
            ret = 1;
        } else {
            ret = HV_ENOENT;
        }
        break;
    case HV_FS_FSTAT:
        /* {size, flags} */
        if (a0 == 1 && s->initrd) {
            ret = (uint32_t)s->initrd_size;
        } else {
            ret = (uint32_t)HV_EBADF;
        }
        break;
    case HV_FS_PREAD: {
        /* (inode, buf, length, offset) */
        int64_t len = (int32_t)a2, off = (int32_t)a3;

        if (a0 != 1 || !s->initrd) {
            ret = HV_EBADF;
            break;
        }
        if (off >= s->initrd_size) {
            ret = 0;
            break;
        }
        len = MIN(len, s->initrd_size - off);
        ret = guest_rw(env, a1, s->initrd + off, len, true) ? HV_EFAULT : len;
        break;
    }
    case HV_PHYSADDR_READ64:
        ret = address_space_ldq_le(&address_space_memory, a0,
                                   MEMTXATTRS_UNSPECIFIED, NULL);
        break;
    case HV_PHYSADDR_WRITE64:
        address_space_stq_le(&address_space_memory, a0, a1,
                             MEMTXATTRS_UNSPECIFIED, NULL);
        break;
    case HV_BZERO_PAGE: {
        static const uint8_t zero[256];
        uint64_t i;

        for (i = 0; i < (uint32_t)a1; i += sizeof(zero)) {
            guest_rw(env, a0 + i, (void *)zero, sizeof(zero), true);
        }
        break;
    }
    case HV_GET_IPI_PTE: {
        /* (tile, pl, &pte) */
        uint64_t pte = HV_PTE_PRESENT | HV_PTE_MODE_MMIO | HV_PTE_READABLE |
                       HV_PTE_WRITABLE |
                       ((TILEGX_IPI_BASE >> 11) << HV_PTE_PTFN_SHIFT);

        ret = guest_rw(env, a2, &pte, 8, true) ? HV_EFAULT : 0;
        break;
    }
    case HV_SET_PTE_SUPER_SHIFT:
        if (a0 < 3) {
            env->pte_super_shift[a0] = a1;
        }
        break;
    case HV_INQUIRE_REALPA:
        ret = a0;
        break;
    case HV_SET_CACHING:
    case HV_REGISTER_MESSAGE_STATE:
    case HV_START_ALL_TILES:
    case HV_STORE_MAPPING:
    case HV_SET_COMMAND_LINE:
    case HV_CLEAR_INTR:
    case HV_ENABLE_INTR:
    case HV_DISABLE_INTR:
    case HV_RAISE_INTR:
    case HV_DOWNCALL_DISPATCH:
        break;
    case HV_TRIGGER_IPI:
        env->ipi_event[2] |= 1ULL << (a1 & 63);
        tilegx_update_irq(env);
        break;
    case HV_DEV_OPEN:
        ret = HV_ENODEV;
        break;
    case HV_DEV_CLOSE:
    case HV_DEV_PREAD:
    case HV_DEV_PWRITE:
    case HV_DEV_POLL:
    case HV_DEV_POLL_CANCEL:
    case HV_DEV_PREADA:
    case HV_DEV_PWRITEA:
        ret = HV_EBADF;
        break;
    case HV_SEND_MESSAGE:
    case HV_RECEIVE_MESSAGE:
    case HV_SEND_NMI:
    default:
        qemu_log_mask(LOG_UNIMP, "tilegx: hypervisor call %d\n", index);
        ret = HV_EINVAL;
        break;
    }

    qemu_log_mask(CPU_LOG_INT, "tilegx: hv call %d (%" PRIx64 ", %" PRIx64
                  ", %" PRIx64 ", %" PRIx64 ") = %" PRIx64 "\n",
                  index, a0, a1, a2, a3, ret);
    r[0] = ret;
    env->pc = r[TILEGX_R_LR];
}

/* ------------------------------------------------------------------ */
/* Machine                                                             */

static void tilegx_reset(void *opaque)
{
    TileGXState *s = opaque;
    CPUTLGState *env = &s->cpu->env;
    uint64_t ram = s->ms->ram_size;

    cpu_reset(CPU(s->cpu));
    env->pc = s->entry;
    env->cycle_base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    env->hv_glue_va = TILEGX_MEM_SV_START + TILEGX_HV_GLUE_CPA;

    /*
     * The hypervisor maps the image's text and data, and memory 1:1,
     * until the client installs its own page table.
     */
    env->boot_map[0][0] = TILEGX_MEM_SV_START;
    env->boot_map[0][1] = 0;
    env->boot_map[0][2] = MIN(ram, 256 * MiB);
    env->boot_map[1][0] = TILEGX_PAGE_OFFSET;
    env->boot_map[1][1] = 0;
    env->boot_map[1][2] = ram;
    env->boot_map[2][0] = 0;            /* head_64.S reads by PA */
    env->boot_map[2][1] = 0;
    env->boot_map[2][2] = ram;
    env->boot_map_count = 3;

    s->rx_head = s->rx_count = 0;
    s->console_ipi_event = -1;
}

static void tilegx_init(MachineState *ms)
{
    MemoryRegion *sysmem = get_system_memory();
    TileGXState *s = g_new0(TileGXState, 1);
    Elf64_Ehdr ehdr;
    uint64_t entry;

    if (ms->ram_size & (16 * MiB - 1)) {
        error_report("RAM size must be a multiple of 16 MiB");
        exit(1);
    }

    tilegx_state = s;
    s->ms = ms;
    s->cpu = TILEGX_CPU(cpu_create(ms->cpu_type));
    memory_region_add_subregion(sysmem, 0, ms->ram);
    memory_region_init_io(&s->ipi, NULL, &tilegx_ipi_ops, s, "tilegx.ipi",
                          TILEGX_IPI_SIZE);
    memory_region_add_subregion(sysmem, TILEGX_IPI_BASE, &s->ipi);

    qemu_chr_fe_init(&s->chr, serial_hd(0), &error_fatal);
    qemu_chr_fe_set_handlers(&s->chr, tilegx_chr_can_rx, tilegx_chr_rx,
                             NULL, NULL, s, NULL, true);

    if (!ms->kernel_filename) {
        error_report("tilegx: -kernel is required");
        exit(1);
    }
    if (load_elf(ms->kernel_filename, NULL, NULL, NULL, &entry, NULL, NULL,
                 NULL, ELFDATA2LSB, EM_TILEGX, 0, 0) < 0 ||
        !load_elf_hdr(ms->kernel_filename, &ehdr, NULL, NULL)) {
        error_report("could not load kernel '%s'", ms->kernel_filename);
        exit(1);
    }
    /* load_elf returns the entry as a load address; we start paged. */
    s->entry = ehdr.e_entry;

    if (ms->initrd_filename) {
        gsize len;

        if (!g_file_get_contents(ms->initrd_filename, (gchar **)&s->initrd,
                                 &len, NULL)) {
            error_report("could not load initrd '%s'", ms->initrd_filename);
            exit(1);
        }
        s->initrd_size = len;
    }

    qemu_register_reset(tilegx_reset, s);
}

static void tilegx_machine_init(MachineClass *mc)
{
    mc->desc = "TILE-Gx tile under the Tilera hypervisor interface";
    mc->init = tilegx_init;
    mc->default_cpu_type = TYPE_TILEGX_CPU;
    mc->default_ram_size = 256 * MiB;
    mc->default_ram_id = "tilegx.ram";
}

DEFINE_MACHINE("tilegx", tilegx_machine_init)
