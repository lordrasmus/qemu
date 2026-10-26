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
#include "hw/nubus/nubus-declrom.h"
#include "net/net.h"
#include "ne2000.h"
#include "qom/object.h"

#define TYPE_NUBUS_8390 "nubus-8390"
OBJECT_DECLARE_SIMPLE_TYPE(Nubus8390State, NUBUS_8390)

#define CARD_MEM_SIZE       0x8000
#define CARD_REG_OFFSET     0x10000
#define CARD_REG_SIZE       0x40
#define DECL_ROM_SIZE       0x1000      /* one page, the ROM at its end */
#define DECL_ROM_MAX        0x100

struct Nubus8390State {
    NubusDevice parent_obj;

    NE2000State ne2000;
    MemoryRegion mem;
    MemoryRegion regs;
    MemoryRegion rom;
};

/* sResource IDs, see Apple's "Designing Cards and Drivers" */
#define RESID_MAC_ADDRESS   0x80

/*
 * The declaration ROM, all four byte lanes in use:
 *
 *   0x00  root directory: board sResource 0x01, Ethernet sResource 0x80
 *   0x10  board directory: type, name, board ID
 *   0x30  Ethernet directory: type, name, memory offset/length, MAC address
 *   0x60  the data blocks, then the format block
 */
static int nubus_8390_build_rom(uint8_t *buf, int size, const uint8_t *mac,
                                uint32_t mem_offset)
{
    NubusDeclRom r;
    int board_type, board_name, eth_type, eth_name, base, len, macaddr;

    nubus_declrom_init(&r, buf, size);
    r.pos = 0x60;

    board_type = r.pos;
    nubus_declrom_put16(&r, 0x0001);    /* category board */
    nubus_declrom_put16(&r, 0x0000);
    nubus_declrom_put16(&r, 0x0000);
    nubus_declrom_put16(&r, 0x0000);

    eth_type = r.pos;
    nubus_declrom_put16(&r, 0x0004);    /* category network */
    nubus_declrom_put16(&r, 0x0001);    /* type Ethernet */
    nubus_declrom_put16(&r, 0x0001);    /* DrSW Apple */
    nubus_declrom_put16(&r, 0x0001);    /* DrHW */

    base = r.pos;
    nubus_declrom_put32(&r, mem_offset);
    len = r.pos;
    nubus_declrom_put32(&r, CARD_MEM_SIZE);

    macaddr = r.pos;
    nubus_declrom_put_bytes(&r, mac, 6);

    board_name = r.pos;
    nubus_declrom_puts(&r, "QEMU NuBus Ethernet");
    eth_name = r.pos;
    nubus_declrom_puts(&r, "Network_Ethernet_8390");

    nubus_declrom_entry_off(&r, 0x00, 0x01, 0x10);
    nubus_declrom_entry_off(&r, 0x04, 0x80, 0x30);
    nubus_declrom_entry(&r, 0x08, NUBUS_RESID_END, 0);

    nubus_declrom_entry_off(&r, 0x10, NUBUS_RESID_TYPE, board_type);
    nubus_declrom_entry_off(&r, 0x14, NUBUS_RESID_NAME, board_name);
    nubus_declrom_entry(&r, 0x18, NUBUS_RESID_BOARDID, 0x0123);
    nubus_declrom_entry(&r, 0x1c, NUBUS_RESID_END, 0);

    nubus_declrom_entry_off(&r, 0x30, NUBUS_RESID_TYPE, eth_type);
    nubus_declrom_entry_off(&r, 0x34, NUBUS_RESID_NAME, eth_name);
    nubus_declrom_entry_off(&r, 0x38, NUBUS_RESID_MINOR_BASEOS, base);
    nubus_declrom_entry_off(&r, 0x3c, NUBUS_RESID_MINOR_LENGTH, len);
    nubus_declrom_entry_off(&r, 0x40, RESID_MAC_ADDRESS, macaddr);
    nubus_declrom_entry(&r, 0x44, NUBUS_RESID_END, 0);

    return nubus_declrom_finish(&r);
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
    uint8_t rom[DECL_ROM_MAX];
    uint32_t mem_offset;
    int rom_len;

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
    rom_len = nubus_8390_build_rom(rom, sizeof(rom), ne->c.macaddr.a,
                                   mem_offset);
    memcpy((uint8_t *)memory_region_get_ram_ptr(&s->rom) + DECL_ROM_SIZE -
           rom_len, rom, rom_len);
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
