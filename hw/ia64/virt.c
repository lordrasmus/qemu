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
#include "qemu/main-loop.h"
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
#include "hw/acpi/acpi.h"
#include "hw/acpi/aml-build.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/pci_host.h"
#include "hw/pci-host/gpex.h"
#include "system/system.h"
#include "system/reset.h"
#include "system/runstate.h"
#include "system/rtc.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "accel/tcg/cpu-loop.h"
#include "elf.h"
#include "virt.h"

#define IOPORT_BASE     0x000000fffc000000ull   /* 64 MB sparse window */
#define IOPORT_SIZE     (64 * MiB)
#define IOSAPIC_BASE    0xfec00000ull
#define PIB_BASE        0xfee00000ull           /* processor interrupt block */
#define VIRTIO_BASE     0xfed00000ull           /* virtio-mmio transports */
#define VIRTIO_SIZE     0x200
#define VIRTIO_NUM      8
#define VIRTIO_GSI      16                      /* first I/O SAPIC pin */

/* PCI: the generic host bridge, its INTA..INTD on four I/O SAPIC pins */
#define PCI_MMIO_BASE   0xc0000000ull
#define PCI_MMIO_SIZE   (FW_ACPI - PCI_MMIO_BASE)
#define PCI_PORT_BASE     0x1000
#define PCI_GSI         24

#define PLATFORM_FREQ   100000000ull

/* ACPI fixed hardware on the legacy I/O ports */
#define ACPI_PM_BASE    0x600                   /* PM1a evt, cnt, timer */
#define ACPI_GPE0_BASE  0x620
#define ACPI_GPE0_LEN   4
#define ACPI_SCI_GSI    9

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
    uint8_t *acpi_tables;   /* image of [FW_ACPI, + FW_ACPI_SIZE) */
    MemoryRegion acpi_ram;
    ACPIREGS acpi;
    MemoryRegion acpi_pm, acpi_gpe;
    MemoryRegion pci_mmio;
    MemoryRegion pci_vga;
    PCIBus *pci_bus;
} IA64VirtState;

static IA64VirtState virt;

/* Debug output switched on by the environment, looked up once */
static bool env_flag(const char *name, int *cache)
{
    if (*cache < 0) {
        *cache = getenv(name) != NULL;
    }
    return *cache;
}

static int io_debug = -1, efi_debug = -1, dbgprint = -1;

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
    if (env_flag("IA64_IO_DEBUG", &io_debug) && ioport_of(off) != 0x3fd &&
        ioport_of(off) != 0x3f8) {
        fprintf(stderr, "io: in%u 0x%" HWADDR_PRIx " (off 0x%" HWADDR_PRIx
                ") -> 0x%" PRIx64 "\n", size, ioport_of(off), off,
                le64_to_cpu(val));
    }
    return le64_to_cpu(val);
}

static void ioport_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    if (env_flag("IA64_IO_DEBUG", &io_debug) && ioport_of(off) != 0x3f8) {
        fprintf(stderr, "io: out%u 0x%" HWADDR_PRIx " (off 0x%" HWADDR_PRIx
                ") <- 0x%" PRIx64 "\n", size, ioport_of(off), off, val);
    }
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
 * COM1 for the firmware console: set up like a firmware would (8N1, FIFOs
 * on, so that a terminal's escape sequences arrive in one piece), and polled.
 */
void virt_com1_init(void)
{
    static const uint8_t regs[][2] = {
        { 3, 0x03 }, { 2, 0x07 }, { 4, 0x0b }, { 1, 0x00 },
    };

    for (int i = 0; i < ARRAY_SIZE(regs); i++) {
        address_space_write(&virt.isa_as, 0x3f8 + regs[i][0],
                            MEMTXATTRS_UNSPECIFIED, &regs[i][1], 1);
    }
}

int virt_com1_getc(void)
{
    uint8_t lsr = 0, c = 0;

    address_space_read(&virt.isa_as, 0x3fd, MEMTXATTRS_UNSPECIFIED, &lsr, 1);
    if (!(lsr & 1)) {
        return -1;
    }
    address_space_read(&virt.isa_as, 0x3f8, MEMTXATTRS_UNSPECIFIED, &c, 1);
    return c;
}

/*
 * ACPI fixed hardware: PM1a event and control blocks, the PM timer and a
 * GPE block, with the SCI on an I/O SAPIC pin.  SCI_EN is set from reset,
 * there is no SMI command port.
 */
static void virt_acpi_update_sci(ACPIREGS *ar)
{
    acpi_update_sci(ar, qdev_get_gpio_in(virt.iosapic, ACPI_SCI_GSI));
}

static uint64_t acpi_gpe_read(void *opaque, hwaddr addr, unsigned size)
{
    return acpi_gpe_ioport_readb(&virt.acpi, addr);
}

static void acpi_gpe_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    acpi_gpe_ioport_writeb(&virt.acpi, addr, val);
    virt_acpi_update_sci(&virt.acpi);
}

static const MemoryRegionOps acpi_gpe_ops = {
    .read = acpi_gpe_read,
    .write = acpi_gpe_write,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 1 },
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void virt_acpi_reset(void *opaque)
{
    acpi_pm1_evt_reset(&virt.acpi);
    acpi_pm1_cnt_reset(&virt.acpi);
    acpi_pm_tmr_reset(&virt.acpi);
    acpi_gpe_reset(&virt.acpi);
    virt_acpi_update_sci(&virt.acpi);
}

static void virt_acpi_init(void)
{
    memory_region_init(&virt.acpi_pm, NULL, "acpi-pm", 64);
    memory_region_add_subregion(&virt.isa_io, ACPI_PM_BASE, &virt.acpi_pm);
    acpi_pm_tmr_init(&virt.acpi, virt_acpi_update_sci, &virt.acpi_pm);
    acpi_pm1_evt_init(&virt.acpi, virt_acpi_update_sci, &virt.acpi_pm);
    acpi_pm1_cnt_init(&virt.acpi, &virt.acpi_pm, true, true, 2, true);
    acpi_gpe_init(&virt.acpi, ACPI_GPE0_LEN);
    memory_region_init_io(&virt.acpi_gpe, NULL, &acpi_gpe_ops, NULL,
                          "acpi-gpe0", ACPI_GPE0_LEN);
    memory_region_add_subregion(&virt.isa_io, ACPI_GPE0_BASE, &virt.acpi_gpe);
    qemu_register_reset(virt_acpi_reset, NULL);
}

/*
 * PCI: the generic host bridge.  Its I/O space sits below the legacy ports
 * in the sparse window, its memory window is mapped one to one, and the
 * OS reaches configuration space through SAL_PCI_CONFIG_READ/WRITE.
 */
static void virt_pci_init(void)
{
    DeviceState *dev = qdev_new(TYPE_GPEX_HOST);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    sysbus_realize_and_unref(sbd, &error_fatal);
    memory_region_init_alias(&virt.pci_mmio, OBJECT(dev), "pci-mmio",
                             sysbus_mmio_get_region(sbd, 1),
                             PCI_MMIO_BASE, PCI_MMIO_SIZE);
    memory_region_add_subregion(get_system_memory(), PCI_MMIO_BASE,
                                &virt.pci_mmio);
    /* the legacy VGA window lies over RAM, as on a PC */
    memory_region_init_alias(&virt.pci_vga, OBJECT(dev), "pci-vga",
                             sysbus_mmio_get_region(sbd, 1),
                             VGA_WINDOW_BASE, VGA_WINDOW_SIZE);
    memory_region_add_subregion_overlap(get_system_memory(), VGA_WINDOW_BASE,
                                        &virt.pci_vga, 1);
    memory_region_add_subregion_overlap(&virt.isa_io, 0,
                                        sysbus_mmio_get_region(sbd, 2), -1);
    for (int i = 0; i < PCI_NUM_PINS; i++) {
        sysbus_connect_irq(sbd, i,
                           qdev_get_gpio_in(virt.iosapic, PCI_GSI + i));
        gpex_set_irq_num(GPEX_HOST(dev), i, PCI_GSI + i);
    }
    virt.pci_bus = PCI_HOST_BRIDGE(dev)->bus;
}

/* SAL's PCI address: compatible or, with type 1, extended */
static PCIDevice *sal_pci_dev(uint64_t addr, uint64_t type, uint32_t *reg)
{
    int bus, devfn;

    if (type == 1) {
        *reg = addr & 0xfff;
        devfn = (addr >> 12) & 0xff;
        bus = (addr >> 20) & 0xff;
        if (addr >> 28) {
            return NULL;
        }
    } else {
        *reg = addr & 0xff;
        devfn = (addr >> 8) & 0xff;
        bus = (addr >> 16) & 0xff;
        if (addr >> 24) {
            return NULL;
        }
    }
    return virt.pci_bus ? pci_find_device(virt.pci_bus, bus, devfn) : NULL;
}

/*
 * PC firmware runs the option ROM of the VGA card at boot, the IA-64 ones
 * do so in an x86 emulator.  Windows' HAL has one of its own for int 10h:
 * it copies the first 2 KB of physical memory (interrupt vectors, BIOS
 * data area; its stack lives there too) and reaches C0000-FFFFF through
 * physical memory.  Put the ROM there and leave its initialization to
 * that emulator: int 10h points at a stub behind the ROM that calls the
 * ROM's init entry, which takes the vector over, and then passes the call
 * on.
 */
static void phys_write(hwaddr pa, const void *buf, hwaddr len)
{
    address_space_write(&address_space_memory, pa, MEMTXATTRS_UNSPECIFIED,
                        buf, len);
}

static void virt_vga_post(PCIDevice *d)
{
    /*
     * The int 10h handler.  The HAL's emulator runs it on a stack in its
     * 2 KB copy of low memory, a few hundred bytes above the BIOS data
     * area, too little for a ROM written in C: switch to one of our own.
     * The first call runs the ROM's init entry and takes over the vector
     * it installs; every call is passed on to that.
     */
    static const uint8_t stub[] = {
        0x2e, 0x8c, 0x16, 0x66, 0x00,           /* mov cs:[old_ss], ss */
        0x2e, 0x89, 0x26, 0x68, 0x00,           /* mov cs:[old_sp], sp */
        0x0e, 0x17,                             /* push cs; pop ss */
        0xbc, 0x00, 0x0f,                       /* mov sp, 0xf00 */
        0x2e, 0x80, 0x3e, 0x6e, 0x00, 0x00,     /* cmp byte cs:[done], 0 */
        0x75, 0x3c,                             /* jne pass */
        0x50, 0x53, 0x51, 0x52,                 /* push ax, bx, cx, dx */
        0x56, 0x57, 0x55, 0x1e, 0x06,           /* push si, di, bp, ds, es */
        0xb8, 0xff, 0xff,                       /* mov ax, 0xffff: no bdf */
        0x9a, 0x03, 0x00, 0x00, 0xc0,           /* call far c000:0003 */
        0x31, 0xc0, 0x8e, 0xd8,                 /* xor ax, ax; mov ds, ax */
        0xa1, 0x40, 0x00,                       /* mov ax, [0x40] */
        0x2e, 0xa3, 0x6a, 0x00,                 /* mov cs:[vec], ax */
        0xa1, 0x42, 0x00,                       /* mov ax, [0x42] */
        0x2e, 0xa3, 0x6c, 0x00,                 /* mov cs:[vec + 2], ax */
        0xc7, 0x06, 0x40, 0x00, 0x00, 0x00,     /* mov word [0x40], 0 */
        0x8c, 0x0e, 0x42, 0x00,                 /* mov [0x42], cs */
        0x2e, 0xc6, 0x06, 0x6e, 0x00, 0x01,     /* mov byte cs:[done], 1 */
        0x07, 0x1f, 0x5d, 0x5f, 0x5e,           /* pop es, ds, bp, di, si */
        0x5a, 0x59, 0x5b, 0x58,                 /* pop dx, cx, bx, ax */
        0x9c,                                   /* pass: pushf */
        0x2e, 0xff, 0x1e, 0x6a, 0x00,           /* call far cs:[vec] */
        0x2e, 0x8e, 0x16, 0x66, 0x00,           /* mov ss, cs:[old_ss] */
        0x2e, 0x8b, 0x26, 0x68, 0x00,           /* mov sp, cs:[old_sp] */
        0xca, 0x02, 0x00,                       /* retf 2: keep the flags */
        0, 0, 0, 0,                             /* old_ss, old_sp */
        0, 0, 0, 0,                             /* vec */
        0,                                      /* done */
    };
    static const uint8_t iret = 0xcf;
    uint8_t ivt[1024];
    const uint8_t *rom = memory_region_get_ram_ptr(&d->rom);
    uint64_t size = rom[2] * 512;               /* the image, not the BAR */

    if (lduw_le_p(rom) != 0xaa55 || size > VGA_STUB - VGA_ROM_BASE) {
        warn_report("ia64 virt: no usable VGA ROM");
        return;
    }
    phys_write(VGA_ROM_BASE, rom, size);
    phys_write(VGA_STUB, stub, sizeof(stub));
    phys_write(VGA_STUB + sizeof(stub), &iret, 1);
    for (int i = 0; i < 256; i++) {
        uint16_t off = i == 0x10 ? 0 : sizeof(stub);

        stw_le_p(ivt + 4 * i, off);
        stw_le_p(ivt + 4 * i + 2, VGA_STUB >> 4);
    }
    phys_write(0, ivt, sizeof(ivt));
    memset(ivt, 0, 256);
    phys_write(0x400, ivt, 256);                    /* BIOS data area */
}

/*
 * Firmware's job, done before the boot loader runs: give every BAR on bus
 * 0 its place in the I/O and memory windows, enable decoding, and note the
 * GSI in the interrupt line register.
 */
void virt_pci_assign(void)
{
    uint64_t io = PCI_PORT_BASE, mem = PCI_MMIO_BASE;

    for (int devfn = 0; virt.pci_bus && devfn < 256; devfn++) {
        PCIDevice *d = pci_find_device(virt.pci_bus, 0, devfn);
        uint32_t cs, cmd = PCI_COMMAND_MASTER;
        uint8_t pin;

        if (!d) {
            continue;
        }
        cs = pci_config_size(d);
        for (int i = 0; i < PCI_ROM_SLOT; i++) {
            PCIIORegion *r = &d->io_regions[i];
            uint32_t bar = PCI_BASE_ADDRESS_0 + 4 * i;
            uint64_t size = r->size, *next;

            if (!size) {
                continue;
            }
            if (r->type & PCI_BASE_ADDRESS_SPACE_IO) {
                next = &io;
                cmd |= PCI_COMMAND_IO;
            } else {
                next = &mem;
                cmd |= PCI_COMMAND_MEMORY;
            }
            *next = ROUND_UP(*next, size);
            pci_host_config_write_common(d, bar, cs, *next, 4);
            if (r->type & PCI_BASE_ADDRESS_MEM_TYPE_64) {
                pci_host_config_write_common(d, bar + 4, cs, *next >> 32, 4);
                i++;
            }
            *next += size;
        }
        pin = pci_get_byte(d->config + PCI_INTERRUPT_PIN);
        if (pin) {
            pci_host_config_write_common(d, PCI_INTERRUPT_LINE, cs,
                PCI_GSI + (PCI_SLOT(devfn) + pin - 1) % 4, 1);
        }
        if (pci_get_word(d->config + PCI_CLASS_DEVICE) ==
            PCI_CLASS_DISPLAY_VGA) {
            cmd |= PCI_COMMAND_IO | PCI_COMMAND_MEMORY;
            if (d->has_rom) {
                virt_vga_post(d);
            }
        }
        pci_host_config_write_common(d, PCI_COMMAND, cs, cmd, 2);
    }
    if (io > 0x10000 || mem > PCI_MMIO_BASE + PCI_MMIO_SIZE) {
        warn_report("ia64 virt: the PCI windows are too small");
    }
}

/*
 * Firmware image construction
 */

/* The image that holds pa: the firmware or the ACPI tables */
static uint8_t *fw_ptr(uint64_t pa)
{
    if (pa >= FW_ACPI && pa < FW_ACPI + FW_ACPI_SIZE) {
        return virt.acpi_tables + pa - FW_ACPI;
    }
    assert(pa >= FW_BASE && pa < FW_END);
    return virt.fw + pa - FW_BASE;
}

static void put8(uint64_t pa, uint8_t v)
{
    *fw_ptr(pa) = v;
}

static void put16(uint64_t pa, uint16_t v)
{
    stw_le_p(fw_ptr(pa), v);
}

static void put32(uint64_t pa, uint32_t v)
{
    stl_le_p(fw_ptr(pa), v);
}

static void put64(uint64_t pa, uint64_t v)
{
    stq_le_p(fw_ptr(pa), v);
}

static void putmem(uint64_t pa, const void *src, size_t len)
{
    memcpy(fw_ptr(pa), src, len);
}

/* { .mib nop.m 0; break.i 0x1c0000 + n; br.ret.sptk.many b0 } */
static void put_stub(uint64_t pa, unsigned n)
{
    put64(pa, 0x0000000100000011ull | ((uint64_t)n << 52));
    put64(pa + 8, 0x00840008800400c0ull);
}

static void fw_acpi_checksum(uint64_t pa, uint32_t len, uint32_t off)
{
    uint8_t sum = 0;

    put8(pa + off, 0);
    for (uint32_t i = 0; i < len; i++) {
        sum += *fw_ptr(pa + i);
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
    fw_acpi_checksum(pa, ldl_le_p(fw_ptr(pa + 4)), 9);
}

static void dsdt_com1(Aml *scope)
{
    Aml *dev = aml_device("COM1");
    Aml *crs = aml_resource_template();

    aml_append(dev, aml_name_decl("_HID", aml_eisaid("PNP0501")));
    aml_append(dev, aml_name_decl("_UID", aml_int(0)));
    aml_append(crs, aml_io(AML_DECODE16, 0x3f8, 0x3f8, 1, 8));
    aml_append(crs, aml_irq_no_flags(4));
    aml_append(dev, aml_name_decl("_CRS", crs));
    aml_append(scope, dev);
}

/* _CRS of virtio-mmio transport i: its registers and a level GSI */
static void dsdt_virtio(Aml *sb, int i)
{
    Aml *dev = aml_device("VR%02d", i);
    Aml *crs = aml_resource_template();
    uint32_t gsi = VIRTIO_GSI + i;

    aml_append(dev, aml_name_decl("_HID", aml_string("LNRO0005")));
    aml_append(dev, aml_name_decl("_UID", aml_int(i)));
    aml_append(crs, aml_memory32_fixed(VIRTIO_BASE + i * VIRTIO_SIZE,
                                       VIRTIO_SIZE, AML_READ_WRITE));
    aml_append(crs, aml_interrupt(AML_CONSUMER, AML_LEVEL, AML_ACTIVE_HIGH,
                                  AML_EXCLUSIVE, &gsi, 1));
    aml_append(dev, aml_name_decl("_CRS", crs));
    aml_append(sb, dev);
}

/*
 * The PCI root: bus 0, I/O ports from 0x1000 (the legacy ones belong to
 * COM1), the memory window, and INTx routed like the generic host bridge
 * swizzles them onto four I/O SAPIC pins.  The other devices sit below it,
 * as behind an ISA bridge: Windows' ACPI interrupt arbiter needs the PCI
 * routing interface for any device it gives an interrupt.
 */
static void dsdt_pci(Aml *sb)
{
    Aml *dev = aml_device("PCI0");
    Aml *crs = aml_resource_template();
    Aml *prt = aml_package(128);

    aml_append(dev, aml_name_decl("_HID", aml_eisaid("PNP0A03")));
    aml_append(dev, aml_name_decl("_UID", aml_int(0)));
    aml_append(dev, aml_name_decl("_BBN", aml_int(0)));
    aml_append(crs, aml_word_bus_number(AML_MIN_FIXED, AML_MAX_FIXED,
                                        AML_POS_DECODE, 0, 0, 255, 0, 256));
    aml_append(crs, aml_word_io(AML_MIN_FIXED, AML_MAX_FIXED, AML_POS_DECODE,
                                AML_ENTIRE_RANGE, 0, PCI_PORT_BASE, 0xffff, 0,
                                0x10000 - PCI_PORT_BASE));
    /* the legacy VGA registers and window */
    aml_append(crs, aml_word_io(AML_MIN_FIXED, AML_MAX_FIXED, AML_POS_DECODE,
                                AML_ENTIRE_RANGE, 0, 0x3b0, 0x3df, 0, 0x30));
    aml_append(crs, aml_dword_memory(AML_POS_DECODE, AML_MIN_FIXED,
                                     AML_MAX_FIXED, AML_NON_CACHEABLE,
                                     AML_READ_WRITE, 0, VGA_WINDOW_BASE,
                                     VGA_WINDOW_BASE + VGA_WINDOW_SIZE - 1,
                                     0, VGA_WINDOW_SIZE));
    aml_append(crs, aml_dword_memory(AML_POS_DECODE, AML_MIN_FIXED,
                                     AML_MAX_FIXED, AML_NON_CACHEABLE,
                                     AML_READ_WRITE, 0, PCI_MMIO_BASE,
                                     PCI_MMIO_BASE + PCI_MMIO_SIZE - 1, 0,
                                     PCI_MMIO_SIZE));
    aml_append(dev, aml_name_decl("_CRS", crs));

    for (int slot = 0; slot < 32; slot++) {
        for (int pin = 0; pin < 4; pin++) {
            Aml *e = aml_package(4);

            aml_append(e, aml_int((slot << 16) | 0xffff));
            aml_append(e, aml_int(pin));
            aml_append(e, aml_int(0));
            aml_append(e, aml_int(PCI_GSI + (slot + pin) % 4));
            aml_append(prt, e);
        }
    }
    aml_append(dev, aml_name_decl("_PRT", prt));
    dsdt_com1(dev);
    for (int i = 0; i < VIRTIO_NUM; i++) {
        dsdt_virtio(dev, i);
    }
    aml_append(sb, dev);
}

static uint64_t build_dsdt(uint64_t pa)
{
    Aml *all = init_aml_allocator();
    Aml *sb = aml_scope("\\_SB");
    Aml *pr = aml_scope("\\_PR");
    Aml *s5 = aml_package(4);
    uint32_t len;

    dsdt_pci(sb);
    aml_append(all, sb);

    /* the MADT's LSAPIC */
    aml_append(pr, aml_processor(0, 0, 0, "CPU0"));
    aml_append(all, pr);

    /* SLP_TYP 0 powers off */
    for (int i = 0; i < 4; i++) {
        aml_append(s5, aml_int(0));
    }
    aml_append(all, aml_name_decl("\\_S5", s5));

    len = all->buf->len;
    assert(pa + 36 + len <= FW_ACPI + FW_ACPI_SIZE);
    acpi_header(pa, "DSDT", 36 + len, 2);
    putmem(pa + 36, all->buf->data, len);
    acpi_finish(pa);
    free_aml_allocator();
    return 36 + len;
}

/*
 * A generic address structure for an I/O port block, given as the memory
 * address the sparse port window puts it at: the Windows HAL maps register
 * blocks in memory space only and ignores those in I/O space.  A block of
 * up to four ports aligned to four is contiguous there.
 */
static void acpi_gas_port(uint64_t pa, uint8_t bits, uint16_t port)
{
    put8(pa, 0);
    put8(pa + 1, bits);
    put8(pa + 2, 0);
    put8(pa + 3, 0);
    put64(pa + 4, IOPORT_BASE + (((uint64_t)port >> 2) << 12) + port);
}

/* RSDP, XSDT, FACS, FADT, MADT, DSDT; returns the RSDP address. */
static uint64_t build_acpi(void)
{
    uint64_t rsdp = FW_ACPI;
    uint64_t xsdt = FW_ACPI + 0x40;
    uint64_t rsdt = FW_ACPI + 0x80;
    uint64_t facs = FW_ACPI + 0xc0;
    uint64_t fadt = FW_ACPI + 0x100;
    uint64_t madt = FW_ACPI + 0x200;
    uint64_t spcr = FW_ACPI + 0x300;
    uint64_t dsdt = FW_ACPI + 0x400;
    uint32_t len;

    build_dsdt(dsdt);

    /* FACS */
    putmem(facs, "FACS", 4);
    put32(facs + 4, 64);
    put8(facs + 32, 1);                         /* version */

    /* FADT, ACPI 2.0, with the fixed hardware the PM block provides */
    acpi_header(fadt, "FACP", 244, 3);
    put32(fadt + 36, facs);
    put32(fadt + 40, dsdt);
    put16(fadt + 46, ACPI_SCI_GSI);
    put32(fadt + 56, ACPI_PM_BASE);             /* PM1a_EVT_BLK */
    put32(fadt + 64, ACPI_PM_BASE + 4);         /* PM1a_CNT_BLK */
    put32(fadt + 76, ACPI_PM_BASE + 8);         /* PM_TMR_BLK */
    put32(fadt + 80, ACPI_GPE0_BASE);
    put8(fadt + 88, 4);
    put8(fadt + 89, 2);
    put8(fadt + 91, 4);
    put8(fadt + 92, ACPI_GPE0_LEN);
    put16(fadt + 96, 101);                      /* no C2 */
    put16(fadt + 98, 1001);                     /* no C3 */
    /* WBINVD, C1, power and sleep buttons are control methods, no RTC */
    put32(fadt + 112, (1 << 0) | (1 << 2) | (1 << 4) | (1 << 5) | (1 << 6));
    put64(fadt + 132, facs);
    put64(fadt + 140, dsdt);
    acpi_gas_port(fadt + 148, 32, ACPI_PM_BASE);
    acpi_gas_port(fadt + 172, 16, ACPI_PM_BASE + 4);
    acpi_gas_port(fadt + 208, 32, ACPI_PM_BASE + 8);
    acpi_gas_port(fadt + 220, ACPI_GPE0_LEN * 8, ACPI_GPE0_BASE);
    acpi_finish(fadt);

    /* MADT */
    len = 44;
    acpi_header(madt, "APIC", 0, 1);
    put32(madt + 36, PIB_BASE);
    put32(madt + 40, 0);
    /*
     * local SAPIC: processor 0, id 0, eid 0, enabled; in the 12 byte form
     * of ACPI 2.0, the only one the Windows HAL accepts
     */
    put8(madt + len, 7);
    put8(madt + len + 1, 12);
    put8(madt + len + 2, 0);
    put8(madt + len + 3, 0);
    put8(madt + len + 4, 0);
    put32(madt + len + 8, 1);
    len += 12;
    /* I/O SAPIC: id 0, GSI base 0 */
    put8(madt + len, 6);
    put8(madt + len + 1, 16);
    put8(madt + len + 2, 0);
    put32(madt + len + 4, 0);
    put64(madt + len + 8, IOSAPIC_BASE);
    len += 16;
    /* the SCI: level, active high */
    put8(madt + len, 2);
    put8(madt + len + 1, 10);
    put8(madt + len + 2, 0);
    put8(madt + len + 3, ACPI_SCI_GSI);
    put32(madt + len + 4, ACPI_SCI_GSI);
    put16(madt + len + 8, 0x000d);
    len += 10;
    /* local SAPIC address override */
    put8(madt + len, 5);
    put8(madt + len + 1, 12);
    put64(madt + len + 4, PIB_BASE);
    len += 12;
    put32(madt + 4, len);
    acpi_finish(madt);

    /*
     * SPCR: COM1 is the console, which Windows' loader and setup redirect
     * to (headless operation) when they find this table.
     */
    acpi_header(spcr, "SPCR", 80, 1);
    put8(spcr + 36, 0);                         /* 16550 */
    put8(spcr + 40, 1);                         /* system I/O */
    put8(spcr + 41, 8);
    put64(spcr + 44, 0x3f8);
    put8(spcr + 52, 1 << 3);                    /* I/O SAPIC interrupt */
    put32(spcr + 54, 4);
    put8(spcr + 58, 7);                         /* 115200 */
    put8(spcr + 59, 0);                         /* no parity */
    put8(spcr + 60, 1);                         /* one stop bit */
    put8(spcr + 61, 0);                         /* no flow control */
    put8(spcr + 62, 0);                         /* VT100 */
    put16(spcr + 64, 0xffff);                   /* not a PCI device */
    put16(spcr + 66, 0xffff);
    acpi_finish(spcr);

    /* XSDT and RSDT */
    acpi_header(xsdt, "XSDT", 36 + 24, 1);
    put64(xsdt + 36, fadt);
    put64(xsdt + 44, madt);
    put64(xsdt + 52, spcr);
    acpi_finish(xsdt);
    acpi_header(rsdt, "RSDT", 36 + 12, 1);
    put32(rsdt + 36, fadt);
    put32(rsdt + 40, madt);
    put32(rsdt + 44, spcr);
    acpi_finish(rsdt);

    /* RSDP 2.0 */
    putmem(rsdp, "RSD PTR ", 8);
    putmem(rsdp + 9, "QEMU  ", 6);
    put8(rsdp + 15, 2);
    put32(rsdp + 16, rsdt);
    put32(rsdp + 20, 36);
    put64(rsdp + 24, xsdt);
    fw_acpi_checksum(rsdp, 20, 8);
    fw_acpi_checksum(rsdp, 36, 32);
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

/*
 * SAL system table: the entry point descriptor, then memory descriptors in
 * the SAL 2.x form for the firmware's parts.  The Windows HAL wants them
 * (memory type, usage): PAL code (0, 1), SAL code and data (0, 4; it
 * maps this one and finds the SAL entry and gp in it), the rest of the
 * firmware (0, 5) and the firmware address space (4, 0);
 * without all four it treats SAL and PAL as missing.  Linux skips them.
 */
static void build_sal_memory(uint64_t e, uint8_t type, uint8_t usage,
                             uint64_t start, uint64_t end)
{
    put8(e, 1);
    put8(e + 1, 1);                             /* used by SAL */
    put8(e + 2, 0);                             /* write back */
    put8(e + 3, 7);                             /* RWX */
    put8(e + 4, 0x09);                          /* WB, UC supported */
    put8(e + 6, type);
    put8(e + 7, usage);
    put64(e + 8, start);
    put32(e + 16, (end - start) >> 12);
}

static void build_sal(void)
{
    uint64_t t = FW_SAL;
    uint64_t e = t + 96;
    uint32_t len = 96 + 48 + 4 * 32;
    uint8_t sum = 0;

    putmem(t, "SST_", 4);
    put32(t + 4, len);
    put8(t + 8, 0x20);                          /* SAL 3.20 */
    put8(t + 9, 0x03);
    put16(t + 10, 5);                           /* entries */
    put8(t + 20, 0x00);                         /* SAL A rev */
    put8(t + 21, 0x01);
    put8(t + 22, 0x00);                         /* SAL B rev */
    put8(t + 23, 0x01);
    putmem(t + 24, "QEMU", 4);
    putmem(t + 56, "IA-64 virt", 10);

    put8(e, 0);                                 /* entry point */
    put64(e + 8, FW_PAL);
    put64(e + 16, FW_STUB(FW_CALL_SAL));        /* SAL proc */
    put64(e + 24, FW_FDESC);                    /* gp: anything sane */
    e += 48;

    build_sal_memory(e, 0, 1, FW_PAL, FW_CODE);
    build_sal_memory(e + 32, 0, 4, FW_CODE, FW_DATA_END);
    build_sal_memory(e + 64, 0, 5, FW_DATA_END, FW_END);
    build_sal_memory(e + 96, 4, 0, FW_BASE, FW_END);

    for (int i = 0; i < len; i++) {
        sum += *fw_ptr(t + i);
    }
    put8(t + 12, -sum);
}

/* The descriptors of everything but RAM, in ascending order */
int virt_fw_memmap(MemDesc *md)
{
    int n = 0;

    md[n++] = (MemDesc){ EFI_ACPI_RECLAIM, FW_ACPI, FW_ACPI + FW_ACPI_SIZE,
                         EFI_MEM_WB };
    md[n++] = (MemDesc){ EFI_MMIO, IOSAPIC_BASE, IOSAPIC_BASE + 0x100000,
                         EFI_MEM_UC | EFI_MEM_RUNTIME };
    md[n++] = (MemDesc){ EFI_MMIO, VIRTIO_BASE, VIRTIO_BASE + 0x1000,
                         EFI_MEM_UC | EFI_MEM_RUNTIME };
    md[n++] = (MemDesc){ EFI_MMIO, PIB_BASE, PIB_BASE + 0x200000,
                         EFI_MEM_UC | EFI_MEM_RUNTIME };
    md[n++] = (MemDesc){ EFI_PAL_CODE, FW_PAL, FW_PAL + 0x10000,
                         EFI_MEM_WB | EFI_MEM_RUNTIME };
    md[n++] = (MemDesc){ EFI_RUNTIME_CODE, FW_CODE, FW_EFI,
                         EFI_MEM_WB | EFI_MEM_RUNTIME };
    md[n++] = (MemDesc){ EFI_RUNTIME_DATA, FW_EFI, FW_DATA_END,
                         EFI_MEM_WB | EFI_MEM_RUNTIME };
    /* boot parameters: not memory the kernel may allocate from */
    md[n++] = (MemDesc){ EFI_RUNTIME_DATA, FW_DATA_END, FW_END,
                         EFI_MEM_WB | EFI_MEM_RUNTIME };
    md[n++] = (MemDesc){ EFI_MMIO_PORT, IOPORT_BASE,
                         IOPORT_BASE + IOPORT_SIZE,
                         EFI_MEM_UC | EFI_MEM_RUNTIME };
    return n;
}

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
    MemDesc md[16];
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
        put64(rt + 24 + 8 * i, FW_FD(FW_CALL_EFI + i));
    }
    /* stubs and descriptors of every firmware call; gp is anything sane */
    for (int i = FW_CALL_EFI; i < 256; i++) {
        put_stub(FW_STUB(i), i);
        put64(FW_FD(i), FW_STUB(i));
        put64(FW_FD(i) + 8, FW_FDESC);
    }
    /* the IVT: 20 vectors of 64 bundles, then 48 of 16 */
    for (int i = 0; i < 68; i++) {
        put_stub(FW_IVT + (i < 20 ? i * 0x400 : 0x5000 + (i - 20) * 0x100),
                 FW_CALL_IVT);
    }
    /* SAL and PAL entry */
    put_stub(FW_STUB(FW_CALL_SAL), FW_CALL_SAL);
    put_stub(FW_PAL, FW_CALL_PAL);

    /* memory map */
    md[n++] = (MemDesc){ EFI_CONVENTIONAL, 0, ram, EFI_MEM_WB };
    n += virt_fw_memmap(md + n);
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

/*
 * Guest pointer to physical: as is with data translation off, else through
 * the caller's TLB, faulting into its handlers on a miss.  A call keeps the
 * pages it has translated: the OS may evict a TLB entry between the probe
 * of an output and the write, and a fault after the first write would
 * repeat a call that has already had effects.
 */
static struct {
    uint64_t page;
    hwaddr pa;
    bool store;
} fw_pages[16];
static int fw_npages;

hwaddr fw_pa(CPUIA64State *env, uint64_t va, MMUAccessType type)
{
    uint64_t page = va & TARGET_PAGE_MASK;
    hwaddr pa;
    int i;

    if (!(env->psr & PSR_DT)) {
        return va & ((1ull << 50) - 1);
    }
    for (i = 0; i < fw_npages; i++) {
        if (fw_pages[i].page == page &&
            (fw_pages[i].store || type != MMU_DATA_STORE)) {
            return fw_pages[i].pa | (va & ~TARGET_PAGE_MASK);
        }
    }
    if (getenv("IA64_FW_NOFAULT")) {    /* TEMP: the old guesswork */
        hwaddr pa = ia64_fw_translate(env, va);

        if (pa != -1) {
            return pa;
        }
        if (va >= 0xe000000080000000ull && va < 0xe000000180000000ull) {
            return va - 0xe000000080000000ull;
        }
        return va & ((1ull << 50) - 1);
    }
    pa = ia64_fw_access(env, va, type);
    i = fw_npages < ARRAY_SIZE(fw_pages) ? fw_npages++ : page % fw_npages;
    fw_pages[i].page = page;
    fw_pages[i].pa = pa & TARGET_PAGE_MASK;
    fw_pages[i].store = type == MMU_DATA_STORE;
    return pa;
}

static void fw_write(CPUIA64State *env, uint64_t va, const void *buf, int len)
{
    address_space_write(&address_space_memory, fw_pa(env, va, MMU_DATA_STORE),
                        MEMTXATTRS_UNSPECIFIED, buf, len);
}

void fw_ret(CPUIA64State *env, uint64_t r8, uint64_t r9, uint64_t r10,
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

/*
 * McKinley's caches.  A unified level has no instruction cache: PAL
 * answers type 1 there with -2, and Linux sizes its cache_info array
 * by unique_caches, so a success would write past it.
 */
static const struct {
    uint8_t level, type, unified, wb, assoc, line;
    uint32_t size;
} pal_caches[] = {
    { 0, 1, 0, 0,  4, 6,   16 * KiB },      /* L1I */
    { 0, 2, 0, 0,  4, 6,   16 * KiB },      /* L1D, write-through */
    { 1, 2, 1, 1,  8, 7,  256 * KiB },      /* L2 */
    { 2, 2, 1, 1, 12, 7,    3 * MiB },      /* L3 */
};

static int64_t pal_cache_info(uint64_t level, uint64_t type,
                              uint64_t *info1, uint64_t *info2)
{
    for (int i = 0; i < ARRAY_SIZE(pal_caches); i++) {
        const typeof(pal_caches[0]) *c = &pal_caches[i];

        if (c->level == level && c->type == type) {
            *info1 = c->unified | (c->wb << 1) | (c->assoc << 8) |
                     (c->line << 16) | ((uint64_t)c->line << 24) |
                     (1ull << 32) | (1ull << 40);   /* latencies */
            *info2 = c->size | (12ull << 32) |      /* alias boundary */
                     ((uint64_t)ctz32(c->size / c->assoc) << 40) |
                     ((uint64_t)IA64_IMPL_PA_MSB << 48);
            return 0;
        }
    }
    return -2;
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
    case 2:                 /* PAL_CACHE_INFO: level in r29, type in r30 */
        st = pal_cache_info(env->gr[29], env->gr[30], &v0, &v1);
        break;
    case 4:                 /* PAL_CACHE_SUMMARY */
        v0 = ARRAY_SIZE(pal_caches) - 1;    /* levels: L1 is split */
        v1 = ARRAY_SIZE(pal_caches);        /* unique caches */
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
    if (env_flag("IA64_EFI_DEBUG", &efi_debug)) {
        fprintf(stderr, "pal: %" PRId64 " (0x%" PRIx64 ", 0x%" PRIx64 ", 0x%"
                PRIx64 ") -> %" PRId64 "\n", idx, env->gr[29], env->gr[30],
                env->gr[31], st);
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
        /* Windows allocates a buffer this large and fails on zero */
        v0 = 16 * KiB;
        break;
    case 0x01000010: {      /* SAL_PCI_CONFIG_READ: address, size, type */
        uint32_t reg;
        PCIDevice *d = sal_pci_dev(in(env, 1), in(env, 3), &reg);
        uint64_t size = in(env, 2);

        v0 = size == 4 ? 0xffffffffu : (1u << (8 * size)) - 1;
        if (size != 1 && size != 2 && size != 4) {
            st = -2;
        } else if (d && reg + size <= pci_config_size(d)) {
            v0 = pci_host_config_read_common(d, reg, pci_config_size(d),
                                             size);
        }
        break;
    }
    case 0x01000011: {      /* SAL_PCI_CONFIG_WRITE: address, size, value, type */
        uint32_t reg;
        PCIDevice *d = sal_pci_dev(in(env, 1), in(env, 4), &reg);
        uint64_t size = in(env, 2);

        if (size != 1 && size != 2 && size != 4) {
            st = -2;
        } else if (d && reg + size <= pci_config_size(d)) {
            pci_host_config_write_common(d, reg, pci_config_size(d),
                                         in(env, 3), size);
        }
        break;
    }
    case 0x01000000:        /* SET_VECTORS */
    case 0x01000003:        /* CLEAR_STATE_INFO */
    case 0x01000005:        /* MC_SET_PARAMS */
    case 0x01000006:
    case 0x01000008:        /* CACHE_FLUSH */
    case 0x01000009:
        break;
    case 0x01000001:        /* GET_STATE_INFO: nothing logged */
        st = -5;
        break;
    default:
        st = -1;
        break;
    }
    if (env_flag("IA64_EFI_DEBUG", &efi_debug)) {
        fprintf(stderr, "sal: 0x%" PRIx64 " (0x%" PRIx64 ", 0x%" PRIx64 ", 0x%"
                PRIx64 ") -> %" PRId64 "\n", fn, in(env, 1), in(env, 2),
                in(env, 3), st);
    }
    fw_ret(env, st, v0, v1, v2);
}

/*
 * The runtime function descriptors follow the mapping the OS chose for the
 * descriptor that holds them: virtual = physical - start + virtual start.
 */
static void efi_set_virtual_address_map(CPUIA64State *env)
{
    uint64_t size = in(env, 0), dsize = in(env, 1), map = in(env, 3);
    uint64_t virt_start = 0, phys_start = 0;
    bool found = false;

    for (uint64_t off = 0; dsize >= 40 && off + dsize <= size; off += dsize) {
        hwaddr d = fw_pa(env, map + off, MMU_DATA_LOAD);
        uint64_t start = address_space_ldq_le(&address_space_memory, d + 8,
                                              MEMTXATTRS_UNSPECIFIED, NULL);
        uint64_t pages = address_space_ldq_le(&address_space_memory, d + 24,
                                              MEMTXATTRS_UNSPECIFIED, NULL);

        if (env_flag("IA64_EFI_DEBUG", &efi_debug)) {
            fprintf(stderr, "efi: SetVirtualAddressMap: %" PRIx64 " pages at %"
                    PRIx64 " -> %" PRIx64 "\n", pages, start,
                    address_space_ldq_le(&address_space_memory, d + 16,
                                         MEMTXATTRS_UNSPECIFIED, NULL));
        }
        if (start <= FW_FDESC && FW_FDESC < start + (pages << 12)) {
            phys_start = start;
            virt_start = address_space_ldq_le(&address_space_memory, d + 16,
                                              MEMTXATTRS_UNSPECIFIED, NULL);
            found = true;
            break;
        }
    }
    if (!found) {
        return;
    }
    efi_virtual_address_change(phys_start, virt_start);
    for (int i = FW_CALL_EFI; i < 256; i++) {
        uint64_t v[2] = {
            cpu_to_le64(FW_STUB(i) - phys_start + virt_start),
            cpu_to_le64(FW_FDESC - phys_start + virt_start),
        };

        address_space_write(&address_space_memory, FW_FD(i),
                            MEMTXATTRS_UNSPECIFIED, v, 16);
    }
}

/* SetVirtualAddressMap after the notification functions */
void virt_svam_complete(CPUIA64State *env)
{
    efi_set_virtual_address_map(env);
    fw_ret(env, EFI_SUCCESS, 0, 0, 0);
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
        if (efi_svam_begin(env, in(env, 3), in(env, 0), in(env, 1))) {
            return;     /* notification functions first */
        }
        efi_set_virtual_address_map(env);
        break;
    case EFI_CONVERT_POINTER:
        st = efi_convert_pointer(env);
        break;
    case EFI_GET_VARIABLE:
        st = efi_get_variable(env);
        break;
    case EFI_GET_NEXT_VARIABLE:
        st = efi_get_next_variable(env);
        break;
    case EFI_SET_VARIABLE:
        st = efi_set_variable(env);
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
    if (getenv("IA64_EFI_DEBUG")) {     /* TEMP */
        fprintf(stderr, "efi: runtime %d -> %" PRIx64 "\n", svc, st);
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
     * on every PAL, SAL and runtime call except the one the idle loop makes;
     * not on boot services or in the firmware's TLB miss handlers.  Nor
     * when a call repeats after a fault on its arguments: the translations
     * the OS has just inserted for it must survive.
     */
    if (n < FW_CALL_BOOT && (n != FW_CALL_PAL || env->gr[28] != 29) &&
        !env->fw_repeat) {
        ia64_tlb_flush_all(env);
    }
    env->fw_repeat = false;
    fw_npages = 0;
    if (n == FW_CALL_PAL) {
        pal_call(env);
    } else if (n == FW_CALL_SAL) {
        /* PCI configuration writes remap BARs */
        BQL_LOCK_GUARD();
        sal_call(env);
    } else if (n >= FW_CALL_EFI && n < FW_CALL_EFI + EFI_NUM_SERVICES) {
        efi_call(env, n - FW_CALL_EFI);
    } else if (n >= FW_CALL_BOOT) {
        efi_boot_call(env, n);
    } else {
        fw_ret(env, -1, 0, 0, 0);
    }
}

/*
 * Windows' DbgPrint is break 0x80017 with r32 pointing to a counted
 * string; without a kernel debugger nobody sees it, so show it on stderr.
 */
static void virt_break_trace(CPUIA64State *env, uint32_t imm)
{
    uint8_t str[16];
    char buf[512];
    uint16_t len;

    if (imm != 0x80017 || !env_flag("IA64_DBGPRINT", &dbgprint)) {
        return;
    }
    if (cpu_memory_rw_debug(env_cpu(env), env->gr[32], str, 16, false)) {
        return;
    }
    len = MIN(lduw_le_p(str), sizeof(buf) - 1);
    if (len && !cpu_memory_rw_debug(env_cpu(env), ldq_le_p(str + 8), buf,
                                    len, false)) {
        /* XP: r32 points to a counted string */
    } else {
        /* Server 2003: r32 is the text, r33 its length */
        len = MIN(env->gr[33], sizeof(buf) - 1);
        if (cpu_memory_rw_debug(env_cpu(env), env->gr[32], buf, len, false)) {
            return;
        }
    }
    buf[len] = 0;
    fprintf(stderr, "%s", buf);
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
    env->break_trace = virt_break_trace;
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
    virt_acpi_init();
    if (ms->ram_size > PCI_MMIO_BASE) {
        error_report("ia64 virt: at most %llu MB of RAM",
                     PCI_MMIO_BASE / MiB);
        exit(EXIT_FAILURE);
    }
    virt_pci_init();

    /*
     * virtio-mmio transports, created from the top down so that the first
     * -device lands on the lowest address, as on arm/virt.
     */
    for (int i = VIRTIO_NUM - 1; i >= 0; i--) {
        sysbus_create_simple(TYPE_VIRTIO_MMIO, VIRTIO_BASE + i * VIRTIO_SIZE,
                             qdev_get_gpio_in(virt.iosapic, VIRTIO_GSI + i));
    }

    if (!ms->kernel_filename) {
        /* no kernel: the EFI boot manager starts from a disk */
        virt.entry = FW_STUB(FW_CALL_BOOT);
        goto firmware;
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

firmware:
    memory_region_init_ram(&virt.fw_ram, NULL, "ia64.firmware",
                           FW_END - FW_BASE, &error_fatal);
    memory_region_add_subregion(sysmem, FW_BASE, &virt.fw_ram);
    virt.fw = g_malloc0(FW_END - FW_BASE);
    memory_region_init_ram(&virt.acpi_ram, NULL, "ia64.acpi", FW_ACPI_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, FW_ACPI, &virt.acpi_ram);
    virt.acpi_tables = g_malloc0(FW_ACPI_SIZE);
    rsdp = build_acpi();
    build_sal();
    build_efi(ms, rsdp, initrd, initrd_size);
    rom_add_blob_fixed("ia64-firmware", virt.fw, FW_END - FW_BASE, FW_BASE);
    rom_add_blob_fixed("ia64-acpi", virt.acpi_tables, FW_ACPI_SIZE, FW_ACPI);

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
