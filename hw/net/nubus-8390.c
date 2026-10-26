/*
 * NuBus Ethernet card with a DP8390 and shared memory
 *
 * Modelled on Apple's 8390 based NuBus cards as the Linux mac8390 driver
 * sees them ("apple" type): the declaration ROM names the card's MAC
 * address and where its 32 KiB of buffer memory lies, the DP8390 registers
 * follow 64 KiB after the memory, one per long word in reverse order
 * (register 0 at +60, register 15 at +0). The 8390 core is the one of the
 * NE2000; its packet memory is the card's shared memory.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/nubus/nubus.h"
#include "net/net.h"
#include "ne2000.h"
#include "qom/object.h"

#define TYPE_NUBUS_8390 "nubus-8390"
OBJECT_DECLARE_SIMPLE_TYPE(Nubus8390State, NUBUS_8390)

#define CARD_MEM_SIZE       0x8000
#define CARD_REG_OFFSET     0x10000
#define CARD_REG_SIZE       0x40
#define DECL_ROM_SIZE       0x1000      /* one page, the ROM at its end */

struct Nubus8390State {
    NubusDevice parent_obj;

    NE2000State ne2000;
    MemoryRegion mem;
    MemoryRegion regs;
    MemoryRegion rom;
};

/* sResource IDs and values, see Apple's "Designing Cards and Drivers" */
#define RESID_TYPE          0x01
#define RESID_NAME          0x02
#define RESID_MINOR_BASEOS  0x0a
#define RESID_MINOR_LENGTH  0x0b
#define RESID_BOARDID       0x20
#define RESID_MAC_ADDRESS   0x80
#define RESID_END           0xff

#define ROM_TEST_PATTERN    0x5a932bc7

typedef struct RomBuilder {
    uint8_t *p;
    int pos;
} RomBuilder;

static void rom_put8(RomBuilder *b, uint8_t v)
{
    b->p[b->pos++] = v;
}

static void rom_put16(RomBuilder *b, uint16_t v)
{
    rom_put8(b, v >> 8);
    rom_put8(b, v);
}

static void rom_put32(RomBuilder *b, uint32_t v)
{
    rom_put16(b, v >> 16);
    rom_put16(b, v);
}

/* a directory entry: ID and a 24-bit value or offset from the entry */
static void rom_entry(uint8_t *p, int pos, uint8_t id, uint32_t data)
{
    p[pos] = id;
    p[pos + 1] = data >> 16;
    p[pos + 2] = data >> 8;
    p[pos + 3] = data;
}

static void rom_entry_off(uint8_t *p, int pos, uint8_t id, int target)
{
    rom_entry(p, pos, id, (target - pos) & 0xffffff);
}

/*
 * Build the declaration ROM into the last 256 bytes of @rom, all four byte
 * lanes in use:
 *
 *   0x00  root directory: board sResource 0x01, Ethernet sResource 0x80
 *   0x10  board directory: type, name, board ID
 *   0x30  Ethernet directory: type, name, memory offset/length, MAC address
 *   0x60  the data blocks
 *   0xec  format block (20 bytes, ends at the top of the slot space)
 */
static void nubus_8390_build_rom(uint8_t *rom, const uint8_t *mac,
                                 uint32_t mem_offset)
{
    uint8_t *p = rom + DECL_ROM_SIZE - 0x100;
    RomBuilder b = { .p = p, .pos = 0x60 };
    int board_type, board_name, eth_type, eth_name, base, len, macaddr;
    int fblock = 0x100 - 20;

    board_type = b.pos;
    rom_put16(&b, 0x0001);      /* category board */
    rom_put16(&b, 0x0000);
    rom_put16(&b, 0x0000);
    rom_put16(&b, 0x0000);

    eth_type = b.pos;
    rom_put16(&b, 0x0004);      /* category network */
    rom_put16(&b, 0x0001);      /* type Ethernet */
    rom_put16(&b, 0x0001);      /* DrSW Apple */
    rom_put16(&b, 0x0001);      /* DrHW */

    base = b.pos;
    rom_put32(&b, mem_offset);
    len = b.pos;
    rom_put32(&b, CARD_MEM_SIZE);

    macaddr = b.pos;
    for (int i = 0; i < 6; i++) {
        rom_put8(&b, mac[i]);
    }

    board_name = b.pos;
    for (const char *s = "QEMU NuBus Ethernet"; *s; s++) {
        rom_put8(&b, *s);
    }
    rom_put8(&b, 0);

    eth_name = b.pos;
    for (const char *s = "Network_Ethernet_8390"; *s; s++) {
        rom_put8(&b, *s);
    }
    rom_put8(&b, 0);
    assert(b.pos <= fblock);

    rom_entry_off(p, 0x00, 0x01, 0x10);
    rom_entry_off(p, 0x04, 0x80, 0x30);
    rom_entry(p, 0x08, RESID_END, 0);

    rom_entry_off(p, 0x10, RESID_TYPE, board_type);
    rom_entry_off(p, 0x14, RESID_NAME, board_name);
    rom_entry(p, 0x18, RESID_BOARDID, 0x0123);
    rom_entry(p, 0x1c, RESID_END, 0);

    rom_entry_off(p, 0x30, RESID_TYPE, eth_type);
    rom_entry_off(p, 0x34, RESID_NAME, eth_name);
    rom_entry_off(p, 0x38, RESID_MINOR_BASEOS, base);
    rom_entry_off(p, 0x3c, RESID_MINOR_LENGTH, len);
    rom_entry_off(p, 0x40, RESID_MAC_ADDRESS, macaddr);
    rom_entry(p, 0x44, RESID_END, 0);

    b.pos = fblock;
    rom_put32(&b, (0 - fblock) & 0xffffff);     /* directory offset */
    rom_put32(&b, 0x100);                       /* length */
    rom_put32(&b, 0);                           /* CRC, unchecked */
    rom_put8(&b, 1);                            /* revision */
    rom_put8(&b, 1);                            /* format: Apple */
    rom_put32(&b, ROM_TEST_PATTERN);
    rom_put8(&b, 0);
    rom_put8(&b, 0x0f);                         /* byte lanes: all four */
}

/* register n sits at offset 60 - 4 * n */
static uint64_t nubus_8390_reg_read(void *opaque, hwaddr addr, unsigned size)
{
    Nubus8390State *s = opaque;
    uint64_t val = 0;

    if ((addr & 3) == 0) {
        memory_region_dispatch_read(&s->ne2000.io, 15 - (addr >> 2), &val,
                                    MO_8, MEMTXATTRS_UNSPECIFIED);
    }
    return val;
}

static void nubus_8390_reg_write(void *opaque, hwaddr addr, uint64_t val,
                                 unsigned size)
{
    Nubus8390State *s = opaque;

    if ((addr & 3) == 0) {
        memory_region_dispatch_write(&s->ne2000.io, 15 - (addr >> 2), val,
                                     MO_8, MEMTXATTRS_UNSPECIFIED);
    }
}

static const MemoryRegionOps nubus_8390_reg_ops = {
    .read = nubus_8390_reg_read,
    .write = nubus_8390_reg_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

static void nubus_8390_set_irq(void *opaque, int n, int level)
{
    nubus_set_irq(NUBUS_DEVICE(opaque), level);
}

static NetClientInfo nubus_8390_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .receive = ne2000_receive,
};

static DeviceRealize nubus_8390_parent_realize;

static void nubus_8390_realize(DeviceState *dev, Error **errp)
{
    ERRP_GUARD();
    Nubus8390State *s = NUBUS_8390(dev);
    NubusDevice *nd = NUBUS_DEVICE(dev);
    NE2000State *ne = &s->ne2000;
    uint32_t mem_offset;

    if (nd->slot < 9 || nd->slot > 0xe) {
        error_setg(errp, "nubus-8390 needs a slot from 9 to 14");
        return;
    }

    nubus_8390_parent_realize(dev, errp);
    if (*errp) {
        return;
    }

    /* the card decodes from the slot base plus slot << 20, like Apple's */
    mem_offset = 0;

    qemu_macaddr_default_if_unset(&ne->c.macaddr);
    ne2000_setup_io(ne, dev, 0x20);
    /* only reached through the register window below, which is guarded */
    ne->io.disable_reentrancy_guard = true;
    /* the packet memory is shared, there is no PROM copy in it */
    ne->match_par = true;
    ne->irq = qemu_allocate_irq(nubus_8390_set_irq, s, 0);
    ne->nic = qemu_new_nic(&nubus_8390_net_info, &ne->c,
                           object_get_typename(OBJECT(dev)), dev->id,
                           &dev->mem_reentrancy_guard, ne);
    qemu_format_nic_info_str(qemu_get_queue(ne->nic), ne->c.macaddr.a);

    memory_region_init_ram_ptr(&s->mem, OBJECT(dev), "nubus-8390-mem",
                               CARD_MEM_SIZE, ne->mem);
    memory_region_add_subregion(&nd->slot_mem,
                                ((hwaddr)nd->slot << 20) + mem_offset,
                                &s->mem);

    memory_region_init_io(&s->regs, OBJECT(dev), &nubus_8390_reg_ops, s,
                          "nubus-8390-regs", CARD_REG_SIZE);
    memory_region_add_subregion(&nd->slot_mem,
                                ((hwaddr)nd->slot << 20) + mem_offset +
                                CARD_REG_OFFSET, &s->regs);

    memory_region_init_rom(&s->rom, OBJECT(dev), "nubus-8390-declrom",
                           DECL_ROM_SIZE, &error_abort);
    nubus_8390_build_rom(memory_region_get_ram_ptr(&s->rom),
                         ne->c.macaddr.a, mem_offset);
    memory_region_add_subregion(&nd->slot_mem, NUBUS_SLOT_SIZE - DECL_ROM_SIZE,
                                &s->rom);
}

static void nubus_8390_reset(DeviceState *dev)
{
    Nubus8390State *s = NUBUS_8390(dev);

    ne2000_reset(&s->ne2000);
}

static const Property nubus_8390_properties[] = {
    DEFINE_NIC_PROPERTIES(Nubus8390State, ne2000.c),
};

static void nubus_8390_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_parent_realize(dc, nubus_8390_realize,
                                    &nubus_8390_parent_realize);
    device_class_set_legacy_reset(dc, nubus_8390_reset);
    device_class_set_props(dc, nubus_8390_properties);
    dc->desc = "NuBus DP8390 Ethernet card";
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo nubus_8390_types[] = {
    {
        .name = TYPE_NUBUS_8390,
        .parent = TYPE_NUBUS_DEVICE,
        .instance_size = sizeof(Nubus8390State),
        .class_init = nubus_8390_class_init,
    },
};

DEFINE_TYPES(nubus_8390_types)
