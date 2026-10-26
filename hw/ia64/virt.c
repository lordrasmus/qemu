/*
 * IA-64 virtual machine for Linux
 *
 * A DIG-like platform: RAM at 0, an I/O SAPIC, a 16550 on the legacy I/O
 * port space (reached through the sparse port window the EFI memory map
 * announces), and a minimal firmware written by qemu itself:
 *
 * - the EFI system table with runtime services, the memory map and the
 *   ia64_boot_param block that elilo would hand to the kernel,
 * - the SAL system table with the PAL and SAL entry points,
 * - ACPI 2.0 tables: MADT (one LSAPIC, the I/O SAPIC), a hardware-reduced
 *   FADT and a DSDT that describes COM1,
 * - PAL, SAL and EFI procedures as stubs that execute a break into qemu
 *   (see firmware_call below).
 *
 * The kernel is a vmlinux, entered in physical mode with r28 pointing to
 * the boot parameters.
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "cpu.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "hw/isa/isa.h"
#include "hw/char/serial-isa.h"
#include "hw/ia64/iosapic.h"
#include "hw/virtio/virtio-mmio.h"
#include "system/system.h"
#include "system/reset.h"
#include "system/runstate.h"
#include "system/rtc.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "accel/tcg/cpu-loop.h"
#include "elf.h"

/* Physical layout */
/*
 * The firmware lives outside of RAM: arch/ia64/mm/contig.c registers all
 * of [0, top of RAM) and would hand out firmware pages inside it.
 */
#define FW_BASE         0x000000fe00000000ull   /* firmware, 1 MB */
#define FW_PAL          (FW_BASE + 0x00000)     /* EFI_PAL_CODE, 64 KB */
#define FW_CODE         (FW_BASE + 0x10000)     /* runtime code */
#define FW_FDESC        (FW_BASE + 0x18000)     /* function descriptors */
#define FW_EFI          (FW_BASE + 0x20000)     /* runtime data */
#define FW_SAL          (FW_BASE + 0x21000)
#define FW_ACPI         (FW_BASE + 0x22000)
#define FW_DATA_END     (FW_BASE + 0x30000)
#define FW_BOOTPARAM    (FW_BASE + 0x30000)     /* loader data */
#define FW_CMDLINE      (FW_BASE + 0x31000)
#define FW_MEMMAP       (FW_BASE + 0x32000)
#define FW_END          (FW_BASE + 0x100000)

#define IOPORT_BASE     0x000000fffc000000ull   /* 64 MB sparse window */
#define IOPORT_SIZE     (64 * MiB)
#define IOSAPIC_BASE    0xfec00000ull
#define PIB_BASE        0xfee00000ull           /* processor interrupt block */
#define VIRTIO_BASE     0xfed00000ull           /* virtio-mmio transports */
#define VIRTIO_SIZE     0x200
#define VIRTIO_NUM      8
#define VIRTIO_GSI      16                      /* first I/O SAPIC pin */

#define PAGE_OFFSET     0xe000000000000000ull

#define PLATFORM_FREQ   100000000ull

/* Firmware call numbers: break.i 0x1c0000 + n */
enum {
    FW_CALL_PAL = 0,
    FW_CALL_SAL = 1,
    FW_CALL_EFI = 2,    /* + runtime service index */
};

enum {
    EFI_GET_TIME, EFI_SET_TIME, EFI_GET_WAKEUP_TIME, EFI_SET_WAKEUP_TIME,
    EFI_SET_VIRTUAL_ADDRESS_MAP, EFI_CONVERT_POINTER, EFI_GET_VARIABLE,
    EFI_GET_NEXT_VARIABLE, EFI_SET_VARIABLE, EFI_GET_NEXT_HIGH_MONO_COUNT,
    EFI_RESET_SYSTEM, EFI_UPDATE_CAPSULE, EFI_QUERY_CAPSULE_CAPS,
    EFI_QUERY_VARIABLE_INFO,
    EFI_NUM_SERVICES
};

#define EFI_SUCCESS         0
#define EFI_UNSUPPORTED     0x8000000000000003ull

/* EFI memory types and attributes */
enum {
    EFI_LOADER_DATA = 2, EFI_RUNTIME_CODE = 5, EFI_RUNTIME_DATA = 6,
    EFI_CONVENTIONAL = 7, EFI_ACPI_RECLAIM = 9, EFI_MMIO = 11,
    EFI_MMIO_PORT = 12, EFI_PAL_CODE = 13,
};
#define EFI_MEM_UC          (1ull << 0)
#define EFI_MEM_WB          (1ull << 3)
#define EFI_MEM_RUNTIME     (1ull << 63)

typedef struct IA64VirtState {
    IA64CPU *cpu;
    DeviceState *iosapic;
    MemoryRegion ioport_window;
    MemoryRegion isa_io;
    MemoryRegion pib;
    MemoryRegion fw_ram;
    AddressSpace isa_as;
    uint64_t entry;
    uint8_t *fw;            /* image of [FW_BASE, FW_END) */
} IA64VirtState;

static IA64VirtState virt;

/*
 * Legacy I/O ports: Linux maps port p to
 * io_base | ((p >> 2) << 12) | (p & 0xfff).
 */
static hwaddr ioport_of(hwaddr off)
{
    return ((off >> 12) << 2) | (off & 3);
}

static uint64_t ioport_read(void *opaque, hwaddr off, unsigned size)
{
    uint64_t val = 0;

    address_space_read(&virt.isa_as, ioport_of(off), MEMTXATTRS_UNSPECIFIED,
                       &val, size);
    return le64_to_cpu(val);
}

static void ioport_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    val = cpu_to_le64(val);
    address_space_write(&virt.isa_as, ioport_of(off), MEMTXATTRS_UNSPECIFIED,
                        &val, size);
}

static const MemoryRegionOps ioport_ops = {
    .read = ioport_read,
    .write = ioport_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

/* Processor interrupt block: an 8-byte write at +0x0 0000 sends an IPI. */
static uint64_t pib_read(void *opaque, hwaddr off, unsigned size)
{
    return 0;
}

static void pib_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    if (off < 0x100000 && !(val & 0x700)) {
        /* fixed delivery to ourselves (uniprocessor) */
        ia64_cpu_deliver_vector(virt.cpu, val & 0xff);
    }
}

static const MemoryRegionOps pib_ops = {
    .read = pib_read,
    .write = pib_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
};

/*
 * Firmware image construction
 */

static void put8(uint64_t pa, uint8_t v)
{
    virt.fw[pa - FW_BASE] = v;
}

static void put16(uint64_t pa, uint16_t v)
{
    stw_le_p(virt.fw + pa - FW_BASE, v);
}

static void put32(uint64_t pa, uint32_t v)
{
    stl_le_p(virt.fw + pa - FW_BASE, v);
}

static void put64(uint64_t pa, uint64_t v)
{
    stq_le_p(virt.fw + pa - FW_BASE, v);
}

static void putmem(uint64_t pa, const void *src, size_t len)
{
    memcpy(virt.fw + pa - FW_BASE, src, len);
}

/* { .mib nop.m 0; break.i 0x1c0000 + n; br.ret.sptk.many b0 } */
static void put_stub(uint64_t pa, unsigned n)
{
    put64(pa, 0x0000000100000011ull | ((uint64_t)n << 52));
    put64(pa + 8, 0x00840008800400c0ull);
}

static void acpi_checksum(uint64_t pa, uint32_t len, uint32_t off)
{
    uint8_t sum = 0;

    put8(pa + off, 0);
    for (uint32_t i = 0; i < len; i++) {
        sum += virt.fw[pa - FW_BASE + i];
    }
    put8(pa + off, -sum);
}

static void acpi_header(uint64_t pa, const char *sig, uint32_t len,
                        uint8_t rev)
{
    putmem(pa, sig, 4);
    put32(pa + 4, len);
    put8(pa + 8, rev);
    putmem(pa + 10, "QEMU  ", 6);
    putmem(pa + 16, "IA64VIRT", 8);
    put32(pa + 24, 1);
    putmem(pa + 28, "QEMU", 4);
    put32(pa + 32, 1);
}

static void acpi_finish(uint64_t pa)
{
    acpi_checksum(pa, ldl_le_p(virt.fw + pa - FW_BASE + 4), 9);
}

/* AML with one- or two-byte package lengths (each package < 4 KB) */
typedef struct Aml {
    uint8_t b[4096];
    int n;
} Aml;

static void aml_byte(Aml *a, uint8_t v)
{
    a->b[a->n++] = v;
}

static void aml_bytes(Aml *a, const void *p, int len)
{
    memcpy(a->b + a->n, p, len);
    a->n += len;
}

/* Wrap body as a package: op bytes, PkgLength, body. */
static void aml_pkg(Aml *out, const uint8_t *op, int oplen, const Aml *body)
{
    aml_bytes(out, op, oplen);
    if (body->n + 1 < 64) {
        aml_byte(out, body->n + 1);
    } else {
        int len = body->n + 2;

        assert(len < 4096);
        aml_byte(out, 0x40 | (len & 0xf));
        aml_byte(out, len >> 4);
    }
    aml_bytes(out, body->b, body->n);
}

static void aml_name_crs(Aml *dev, const uint8_t *crs, int len)
{
    static const uint8_t buffer_op[] = { 0x11 };
    Aml buf = { .n = 0 };

    aml_byte(&buf, 0x0a);                       /* BytePrefix: size */
    aml_byte(&buf, len);
    aml_bytes(&buf, crs, len);
    aml_byte(dev, 0x08);                        /* Name(_CRS, Buffer) */
    aml_bytes(dev, "_CRS", 4);
    aml_pkg(dev, buffer_op, 1, &buf);
}

/* virtio-mmio transport i: LNRO0005, its registers and a level GSI */
static void aml_virtio(Aml *sb, int i)
{
    static const uint8_t device_op[] = { 0x5b, 0x82 };
    uint8_t crs[] = {
        0x86, 0x09, 0x00, 0x01, 0, 0, 0, 0, 0, 0, 0, 0, /* Memory32Fixed RW */
        0x89, 0x06, 0x00, 0x01, 0x01, 0, 0, 0, 0,       /* Interrupt, level */
        0x79, 0x00,
    };
    Aml dev = { .n = 0 };
    char name[5];

    stl_le_p(crs + 4, VIRTIO_BASE + i * VIRTIO_SIZE);
    stl_le_p(crs + 8, VIRTIO_SIZE);
    stl_le_p(crs + 17, VIRTIO_GSI + i);

    snprintf(name, sizeof(name), "VR%02d", i);
    aml_bytes(&dev, name, 4);
    aml_byte(&dev, 0x08);                       /* Name(_HID, "LNRO0005") */
    aml_bytes(&dev, "_HID", 4);
    aml_byte(&dev, 0x0d);
    aml_bytes(&dev, "LNRO0005", 9);
    aml_byte(&dev, 0x08);                       /* Name(_UID, i) */
    aml_bytes(&dev, "_UID", 4);
    aml_byte(&dev, 0x0a);
    aml_byte(&dev, i);
    aml_name_crs(&dev, crs, sizeof(crs));
    aml_pkg(sb, device_op, 2, &dev);
}

static uint64_t build_dsdt(uint64_t pa)
{
    static const uint8_t crs_data[] = {
        0x47, 0x01, 0xf8, 0x03, 0xf8, 0x03, 0x01, 0x08, /* IO 0x3f8, 8 */
        0x22, 0x10, 0x00,                               /* IRQNoFlags {4} */
        0x79, 0x00,                                     /* end tag */
    };
    static const uint8_t scope_op[] = { 0x10 };
    static const uint8_t device_op[] = { 0x5b, 0x82 };
    static const uint8_t buffer_op[] = { 0x11 };
    Aml buf = { .n = 0 }, dev = { .n = 0 }, sb = { .n = 0 }, all = { .n = 0 };

    /* Buffer (0x0d) { crs } */
    aml_byte(&buf, 0x0a);
    aml_byte(&buf, sizeof(crs_data));
    aml_bytes(&buf, crs_data, sizeof(crs_data));

    aml_bytes(&dev, "COM1", 4);
    aml_byte(&dev, 0x08);                       /* Name(_HID, EisaId) */
    aml_bytes(&dev, "_HID", 4);
    aml_byte(&dev, 0x0c);
    aml_bytes(&dev, "\x41\xd0\x05\x01", 4);     /* PNP0501 */
    aml_byte(&dev, 0x08);                       /* Name(_UID, 0) */
    aml_bytes(&dev, "_UID", 4);
    aml_byte(&dev, 0x00);
    aml_byte(&dev, 0x08);                       /* Name(_CRS, Buffer) */
    aml_bytes(&dev, "_CRS", 4);
    aml_pkg(&dev, buffer_op, 1, &buf);

    aml_bytes(&sb, "\\_SB_", 5);
    aml_pkg(&sb, device_op, 2, &dev);
    for (int i = 0; i < VIRTIO_NUM; i++) {
        aml_virtio(&sb, i);
    }

    aml_pkg(&all, scope_op, 1, &sb);

    acpi_header(pa, "DSDT", 36 + all.n, 2);
    putmem(pa + 36, all.b, all.n);
    acpi_finish(pa);
    return 36 + all.n;
}

/* RSDP, XSDT, FADT, MADT, DSDT; returns the RSDP address. */
static uint64_t build_acpi(void)
{
    uint64_t rsdp = FW_ACPI;
    uint64_t xsdt = FW_ACPI + 0x40;
    uint64_t fadt = FW_ACPI + 0x100;
    uint64_t madt = FW_ACPI + 0x300;
    uint64_t dsdt = FW_ACPI + 0x400;
    uint32_t len;

    build_dsdt(dsdt);

    /* FADT, ACPI 5.0, hardware reduced */
    acpi_header(fadt, "FACP", 268, 5);
    put32(fadt + 112, 1u << 20);                /* HW_REDUCED_ACPI */
    put8(fadt + 131, 0);                        /* minor version */
    put64(fadt + 140, dsdt);                    /* X_DSDT */
    acpi_finish(fadt);

    /* MADT */
    len = 44;
    acpi_header(madt, "APIC", 0, 1);
    put32(madt + 36, PIB_BASE);
    put32(madt + 40, 0);
    /* local SAPIC: processor 0, id 0, eid 0, enabled */
    put8(madt + len, 7);
    put8(madt + len + 1, 17);
    put8(madt + len + 2, 0);
    put8(madt + len + 3, 0);
    put8(madt + len + 4, 0);
    put32(madt + len + 8, 1);
    put32(madt + len + 12, 0);
    put8(madt + len + 16, 0);
    len += 17;
    /* I/O SAPIC: id 0, GSI base 0 */
    put8(madt + len, 6);
    put8(madt + len + 1, 16);
    put8(madt + len + 2, 0);
    put32(madt + len + 4, 0);
    put64(madt + len + 8, IOSAPIC_BASE);
    len += 16;
    /* local SAPIC address override */
    put8(madt + len, 5);
    put8(madt + len + 1, 12);
    put64(madt + len + 4, PIB_BASE);
    len += 12;
    put32(madt + 4, len);
    acpi_finish(madt);

    /* XSDT */
    acpi_header(xsdt, "XSDT", 36 + 16, 1);
    put64(xsdt + 36, fadt);
    put64(xsdt + 44, madt);
    acpi_finish(xsdt);

    /* RSDP 2.0 */
    putmem(rsdp, "RSD PTR ", 8);
    putmem(rsdp + 9, "QEMU  ", 6);
    put8(rsdp + 15, 2);
    put32(rsdp + 16, 0);
    put32(rsdp + 20, 36);
    put64(rsdp + 24, xsdt);
    acpi_checksum(rsdp, 20, 8);
    acpi_checksum(rsdp, 36, 32);
    return rsdp;
}

static void put_guid(uint64_t pa, uint32_t a, uint16_t b, uint16_t c,
                     const uint8_t d[8])
{
    put32(pa, a);
    put16(pa + 4, b);
    put16(pa + 6, c);
    putmem(pa + 8, d, 8);
}

/* SAL system table with the entry point descriptor */
static void build_sal(void)
{
    uint64_t t = FW_SAL;
    uint64_t e = t + 96;
    uint8_t sum = 0;

    putmem(t, "SST_", 4);
    put32(t + 4, 96 + 48);
    put8(t + 8, 0x20);                          /* SAL 3.20 */
    put8(t + 9, 0x03);
    put16(t + 10, 1);                           /* entries */
    put8(t + 16, 0x00);                         /* SAL A rev */
    put8(t + 17, 0x01);
    put8(t + 18, 0x00);                         /* SAL B rev */
    put8(t + 19, 0x01);
    putmem(t + 20, "QEMU", 4);
    putmem(t + 52, "IA-64 virt", 10);

    put8(e, 0);                                 /* entry point */
    put64(e + 8, FW_PAL);
    put64(e + 16, FW_CODE);                     /* SAL proc */
    put64(e + 24, FW_FDESC);                    /* gp: anything sane */

    for (int i = 0; i < 96 + 48; i++) {
        sum += virt.fw[t - FW_BASE + i];
    }
    put8(t + 12, -sum);
}

typedef struct MemDesc {
    uint32_t type;
    uint64_t start, end, attr;
} MemDesc;

static void build_efi(MachineState *ms, uint64_t rsdp, uint64_t initrd,
                      uint64_t initrd_size)
{
    static const uint8_t acpi20[8] = { 0xbc, 0x22, 0x00, 0x80, 0xc7, 0x3c,
                                       0x88, 0x81 };
    static const uint8_t sal[8] = { 0x9a, 0x16, 0x00, 0x90, 0x27, 0x3f,
                                    0xc1, 0x4d };
    uint64_t st = FW_EFI;
    uint64_t vendor = FW_EFI + 0x100;
    uint64_t rt = FW_EFI + 0x200;
    uint64_t cfg = FW_EFI + 0x300;
    uint64_t ram = ms->ram_size;
    MemDesc md[12];
    int n = 0;
    const char *cmd = ms->kernel_cmdline ? ms->kernel_cmdline : "";

    /* system table */
    put64(st, 0x5453595320494249ull);           /* "IBI SYST" */
    put32(st + 8, (1 << 16) | 10);              /* EFI 1.10 */
    put32(st + 12, 120);
    put64(st + 24, vendor);
    put32(st + 32, 0x00010000);
    put64(st + 88, rt);
    put64(st + 96, 0);                          /* no boot services */
    put64(st + 104, 2);                         /* nr_tables */
    put64(st + 112, cfg);
    for (int i = 0; "QEMU"[i]; i++) {
        put16(vendor + 2 * i, "QEMU"[i]);
    }

    put_guid(cfg, 0x8868e871, 0xe4f1, 0x11d3, acpi20);
    put64(cfg + 16, rsdp);
    put_guid(cfg + 24, 0xeb9d2d32, 0x2d88, 0x11d3, sal);
    put64(cfg + 40, FW_SAL);

    /* runtime services: header, then function descriptor pointers */
    put64(rt, 0x56524553544e5552ull);           /* "RUNTSERV" */
    put32(rt + 8, (1 << 16) | 10);
    put32(rt + 12, 24 + 8 * EFI_NUM_SERVICES);
    for (int i = 0; i < EFI_NUM_SERVICES; i++) {
        uint64_t fd = FW_FDESC + 16 * (i + 1);

        put_stub(FW_CODE + 16 * (i + 1), FW_CALL_EFI + i);
        put64(fd, FW_CODE + 16 * (i + 1));
        put64(fd + 8, FW_FDESC);
        put64(rt + 24 + 8 * i, fd);
    }
    /* SAL and PAL entry */
    put_stub(FW_CODE, FW_CALL_SAL);
    put_stub(FW_PAL, FW_CALL_PAL);

    /* memory map */
    md[n++] = (MemDesc){ EFI_CONVENTIONAL, 0, ram, EFI_MEM_WB };
    md[n++] = (MemDesc){ EFI_PAL_CODE, FW_PAL, FW_PAL + 0x10000,
                         EFI_MEM_WB | EFI_MEM_RUNTIME };
    md[n++] = (MemDesc){ EFI_RUNTIME_CODE, FW_CODE, FW_EFI,
                         EFI_MEM_WB | EFI_MEM_RUNTIME };
    md[n++] = (MemDesc){ EFI_RUNTIME_DATA, FW_EFI, FW_DATA_END,
                         EFI_MEM_WB | EFI_MEM_RUNTIME };
    /* boot parameters: not memory the kernel may allocate from */
    md[n++] = (MemDesc){ EFI_RUNTIME_DATA, FW_DATA_END, FW_END,
                         EFI_MEM_WB | EFI_MEM_RUNTIME };
    md[n++] = (MemDesc){ EFI_MMIO, IOSAPIC_BASE, IOSAPIC_BASE + 0x100000,
                         EFI_MEM_UC | EFI_MEM_RUNTIME };
    md[n++] = (MemDesc){ EFI_MMIO, VIRTIO_BASE, VIRTIO_BASE + 0x1000,
                         EFI_MEM_UC | EFI_MEM_RUNTIME };
    md[n++] = (MemDesc){ EFI_MMIO, PIB_BASE, PIB_BASE + 0x200000,
                         EFI_MEM_UC | EFI_MEM_RUNTIME };
    md[n++] = (MemDesc){ EFI_MMIO_PORT, IOPORT_BASE,
                         IOPORT_BASE + IOPORT_SIZE,
                         EFI_MEM_UC | EFI_MEM_RUNTIME };
    for (int i = 0; i < n; i++) {
        uint64_t d = FW_MEMMAP + 40 * i;

        put32(d, md[i].type);
        put64(d + 8, md[i].start);
        put64(d + 16, 0);
        put64(d + 24, (md[i].end - md[i].start) >> 12);
        put64(d + 32, md[i].attr);
    }

    /* command line */
    putmem(FW_CMDLINE, cmd, MIN(strlen(cmd), 4095));

    /* struct ia64_boot_param */
    put64(FW_BOOTPARAM, FW_CMDLINE);
    put64(FW_BOOTPARAM + 8, st);
    put64(FW_BOOTPARAM + 16, FW_MEMMAP);
    put64(FW_BOOTPARAM + 24, 40 * n);
    put64(FW_BOOTPARAM + 32, 40);
    put32(FW_BOOTPARAM + 40, 1);
    put16(FW_BOOTPARAM + 44, 80);
    put16(FW_BOOTPARAM + 46, 25);
    put64(FW_BOOTPARAM + 56, 0);                /* no FPSWA */
    put64(FW_BOOTPARAM + 64, initrd);
    put64(FW_BOOTPARAM + 72, initrd_size);
}

/*
 * Firmware calls
 */

/* Guest pointer to physical: identity regions 6/7, else the TLB. */
static hwaddr fw_pa(CPUIA64State *env, uint64_t va)
{
    if (!(env->psr & PSR_DT)) {
        return va & ((1ull << 50) - 1);
    }
    if ((va >> 61) >= 6) {
        return va & ((1ull << 50) - 1);
    }
    return ia64_cpu_get_phys_page_debug(env_cpu(env), va);
}

static void fw_write(CPUIA64State *env, uint64_t va, const void *buf, int len)
{
    address_space_write(&address_space_memory, fw_pa(env, va), MEMTXATTRS_UNSPECIFIED, buf, len);
}

static void fw_ret(CPUIA64State *env, uint64_t r8, uint64_t r9, uint64_t r10,
                   uint64_t r11)
{
    env->gr[8] = r8;
    env->gr[9] = r9;
    env->gr[10] = r10;
    env->gr[11] = r11;
    env->nat[8] = env->nat[9] = env->nat[10] = env->nat[11] = 0;
}

static uint64_t in(CPUIA64State *env, int i)
{
    return env->gr[32 + i];
}

static void pal_call(CPUIA64State *env)
{
    uint64_t idx = env->gr[28];
    int64_t st = 0;
    uint64_t v0 = 0, v1 = 0, v2 = 0;

    switch (idx) {
    case 1:                 /* PAL_CACHE_FLUSH */
    case 21: case 22: case 23: case 27:     /* MC_* */
    case 18:                /* PROC_SET_FEATURES */
        break;
    case 6:                 /* PAL_PTCE_INFO */
        v0 = 0;
        v1 = (1ull << 32) | 1;
        v2 = 0;
        break;
    case 8:                 /* PAL_VM_SUMMARY */
        v0 = ((uint64_t)(IA64_IMPL_PA_MSB + 1) << 1) | (24ull << 8) |
             (15ull << 16) | (7ull << 32) | (7ull << 40) | (1ull << 56);
        v1 = IA64_IMPL_VA_MSB | ((uint64_t)IA64_RID_BITS << 8) | (1ull << 16);
        break;
    case 11:                /* PAL_DEBUG_INFO */
        v0 = 4;
        v1 = 4;
        break;
    case 13:                /* PAL_FREQ_BASE */
        v0 = PLATFORM_FREQ;
        break;
    case 14:                /* PAL_FREQ_RATIOS: num << 32 | den */
        v0 = (1ull << 32) | 1;
        v1 = (1ull << 32) | 1;
        v2 = (1ull << 32) | 1;
        break;
    case 17:                /* PAL_PROC_GET_FEATURES */
        break;
    case 19:                /* PAL_RSE_INFO */
        v0 = IA64_NUM_STACKED;
        break;
    case 20:                /* PAL_VERSION */
        v0 = 0x0001000000000000ull;
        v1 = v0;
        break;
    case 29:                /* PAL_HALT_LIGHT */
        fw_ret(env, 0, 0, 0, 0);
        env->ip = env->br[0] & ~0xfull;
        env->ri = 0;
        if (!cpu_test_interrupt(env_cpu(env), CPU_INTERRUPT_HARD)) {
            CPUState *cs = env_cpu(env);

            cs->halted = 1;
            cs->exception_index = EXCP_HLT;
            cpu_loop_exit(cs);
        }
        return;
    case 34:                /* PAL_VM_PAGE_SIZE */
        v0 = 0x115557000ull;
        v1 = 0x115557000ull;
        break;
    default:
        st = -1;            /* unimplemented */
        break;
    }
    fw_ret(env, st, v0, v1, v2);
    if (idx < 256) {
        /* static calling convention: return through b0 */
        env->ip = env->br[0] & ~0xfull;
        env->ri = 0;
    }
}

static void sal_call(CPUIA64State *env)
{
    uint64_t fn = in(env, 0);
    int64_t st = 0;
    uint64_t v0 = 0, v1 = 0, v2 = 0;

    switch (fn) {
    case 0x01000012:        /* SAL_FREQ_BASE */
        v0 = PLATFORM_FREQ;
        v1 = 0;
        break;
    case 0x01000002:        /* SAL_GET_STATE_INFO_SIZE */
        v0 = 0;
        break;
    case 0x01000010:        /* SAL_PCI_CONFIG_READ */
        v0 = ~0ull;
        break;
    case 0x01000000:        /* SET_VECTORS */
    case 0x01000003:        /* CLEAR_STATE_INFO */
    case 0x01000005:        /* MC_SET_PARAMS */
    case 0x01000006:
    case 0x01000008:        /* CACHE_FLUSH */
    case 0x01000009:
    case 0x01000011:
        break;
    case 0x01000001:        /* GET_STATE_INFO: nothing logged */
        st = -5;
        break;
    default:
        st = -1;
        break;
    }
    fw_ret(env, st, v0, v1, v2);
}

static void efi_call(CPUIA64State *env, int svc)
{
    uint64_t st = EFI_SUCCESS;

    switch (svc) {
    case EFI_GET_TIME: {
        struct tm tm;
        uint8_t t[16] = { 0 };

        qemu_get_timedate(&tm, 0);
        stw_le_p(t, tm.tm_year + 1900);
        t[2] = tm.tm_mon + 1;
        t[3] = tm.tm_mday;
        t[4] = tm.tm_hour;
        t[5] = tm.tm_min;
        t[6] = tm.tm_sec;
        stw_le_p(t + 12, 0x7ff);                /* unspecified timezone */
        fw_write(env, in(env, 0), t, 16);
        if (in(env, 1)) {
            uint8_t cap[16] = { 0 };

            stl_le_p(cap, 1);                   /* resolution 1 Hz */
            fw_write(env, in(env, 1), cap, 16);
        }
        break;
    }
    case EFI_SET_VIRTUAL_ADDRESS_MAP:
        /* the runtime function descriptors now hold virtual addresses */
        for (int i = 0; i < EFI_NUM_SERVICES; i++) {
            hwaddr fd = FW_FDESC + 16 * (i + 1);
            uint64_t v[2];

            address_space_read(&address_space_memory, fd, MEMTXATTRS_UNSPECIFIED, v, 16);
            v[0] = cpu_to_le64(le64_to_cpu(v[0]) | PAGE_OFFSET);
            v[1] = cpu_to_le64(le64_to_cpu(v[1]) | PAGE_OFFSET);
            address_space_write(&address_space_memory, fd, MEMTXATTRS_UNSPECIFIED, v, 16);
        }
        break;
    case EFI_RESET_SYSTEM:
        if (in(env, 0) == 2) {                  /* EfiResetShutdown */
            qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
        } else {
            qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        }
        break;
    default:
        st = EFI_UNSUPPORTED;
        break;
    }
    fw_ret(env, st, 0, 0, 0);
}

static void virt_firmware_call(CPUIA64State *env, uint32_t n)
{
    /*
     * Firmware on a real machine runs through the TLB and evicts translation
     * cache entries, which the architecture allows at any time.  SMP Linux
     * depends on it: per_cpu_init() moves the boot CPU's per-CPU area and
     * repoints ar.k3, but the old PERCPU_ADDR translation is never purged.
     * ia64_tlb_init() would write the ptce parameters through it into the old
     * area, and every later flush_tlb_all() would read zero loop counts and
     * purge nothing.  PAL_CACHE_SUMMARY runs in between, so drop the caches
     * on every firmware call except the one the idle loop makes.
     */
    if (n != FW_CALL_PAL || env->gr[28] != 29) {
        ia64_tlb_flush_all(env);
    }
    if (n == FW_CALL_PAL) {
        pal_call(env);
    } else if (n == FW_CALL_SAL) {
        sal_call(env);
    } else if (n >= FW_CALL_EFI && n < FW_CALL_EFI + EFI_NUM_SERVICES) {
        efi_call(env, n - FW_CALL_EFI);
    } else {
        fw_ret(env, -1, 0, 0, 0);
    }
}

static void virt_cpu_reset(void *opaque)
{
    IA64CPU *cpu = opaque;
    CPUIA64State *env = &cpu->env;

    cpu_reset(CPU(cpu));
    env->ip = virt.entry & ~0xfull;
    env->ri = 0;
    env->gr[28] = FW_BOOTPARAM;
    env->firmware_call = virt_firmware_call;
}

static void virt_init(MachineState *ms)
{
    MemoryRegion *sysmem = get_system_memory();
    qemu_irq isa_irqs[16];
    ISABus *isa_bus;
    uint64_t hi, initrd = 0, initrd_size = 0, rsdp;
    ssize_t size;

    virt.cpu = IA64_CPU(cpu_create(ms->cpu_type));
    memory_region_add_subregion(sysmem, 0, ms->ram);

    /* I/O SAPIC */
    virt.iosapic = qdev_new(TYPE_IOSAPIC);
    IOSAPIC(virt.iosapic)->cpu = virt.cpu;
    sysbus_realize_and_unref(SYS_BUS_DEVICE(virt.iosapic), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(virt.iosapic), 0, IOSAPIC_BASE);

    memory_region_init_io(&virt.pib, NULL, &pib_ops, NULL, "pib", 2 * MiB);
    memory_region_add_subregion(sysmem, PIB_BASE, &virt.pib);

    /* legacy I/O ports behind the sparse window */
    memory_region_init(&virt.isa_io, NULL, "isa-io", 64 * KiB);
    address_space_init(&virt.isa_as, &virt.isa_io, "isa-io");
    memory_region_init_io(&virt.ioport_window, NULL, &ioport_ops, NULL,
                          "ioport-window", IOPORT_SIZE);
    memory_region_add_subregion(sysmem, IOPORT_BASE, &virt.ioport_window);

    isa_bus = isa_bus_new(NULL, sysmem, &virt.isa_io, &error_abort);
    for (int i = 0; i < 16; i++) {
        isa_irqs[i] = qdev_get_gpio_in(virt.iosapic, i);
    }
    isa_bus_register_input_irqs(isa_bus, isa_irqs);
    serial_hds_isa_init(isa_bus, 0, 1);

    /*
     * virtio-mmio transports, created from the top down so that the first
     * -device lands on the lowest address, as on arm/virt.
     */
    for (int i = VIRTIO_NUM - 1; i >= 0; i--) {
        sysbus_create_simple(TYPE_VIRTIO_MMIO, VIRTIO_BASE + i * VIRTIO_SIZE,
                             qdev_get_gpio_in(virt.iosapic, VIRTIO_GSI + i));
    }

    if (!ms->kernel_filename) {
        error_report("ia64 virt: -kernel is required");
        exit(EXIT_FAILURE);
    }
    size = load_elf(ms->kernel_filename, NULL, NULL, NULL, &virt.entry, NULL,
                    &hi, NULL, ELFDATA2LSB, EM_IA_64, 0, 0);
    if (size < 0) {
        error_report("ia64 virt: could not load kernel '%s'",
                     ms->kernel_filename);
        exit(EXIT_FAILURE);
    }
    if (hi >= ms->ram_size) {
        error_report("ia64 virt: kernel does not fit into RAM");
        exit(EXIT_FAILURE);
    }

    if (ms->initrd_filename) {
        size = get_image_size(ms->initrd_filename, NULL);
        if (size < 0) {
            error_report("ia64 virt: could not load initrd '%s'",
                         ms->initrd_filename);
            exit(EXIT_FAILURE);
        }
        /* at the top of RAM, granule aligned */
        initrd = (ms->ram_size - size) & ~(16 * MiB - 1);
        if (initrd <= ROUND_UP(hi, 16 * MiB)) {
            error_report("ia64 virt: initrd does not fit into RAM");
            exit(EXIT_FAILURE);
        }
        initrd_size = load_image_targphys(ms->initrd_filename, initrd,
                                          ms->ram_size - initrd, NULL);
    }

    memory_region_init_ram(&virt.fw_ram, NULL, "ia64.firmware",
                           FW_END - FW_BASE, &error_fatal);
    memory_region_add_subregion(sysmem, FW_BASE, &virt.fw_ram);
    virt.fw = g_malloc0(FW_END - FW_BASE);
    rsdp = build_acpi();
    build_sal();
    build_efi(ms, rsdp, initrd, initrd_size);
    rom_add_blob_fixed("ia64-firmware", virt.fw, FW_END - FW_BASE, FW_BASE);

    qemu_register_reset(virt_cpu_reset, virt.cpu);
}

static void virt_machine_init(MachineClass *mc)
{
    mc->desc = "IA-64 virtual machine (EFI/SAL/PAL/ACPI emulated by qemu)";
    mc->init = virt_init;
    mc->default_cpu_type = TYPE_IA64_CPU;
    mc->default_ram_size = 512 * MiB;
    mc->default_ram_id = "ia64.ram";
    mc->is_default = true;
}

DEFINE_MACHINE("virt", virt_machine_init)
