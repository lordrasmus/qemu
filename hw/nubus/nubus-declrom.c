/*
 * Building NuBus declaration ROMs
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "hw/nubus/nubus-declrom.h"

#define FBLOCK_SIZE         20
#define FBLOCK_CRC          8
#define ROM_TEST_PATTERN    0x5a932bc7

void nubus_declrom_init(NubusDeclRom *r, uint8_t *buf, int size)
{
    r->p = buf;
    r->size = size;
    r->pos = 0;
    memset(buf, 0, size);
}

void nubus_declrom_put8(NubusDeclRom *r, uint8_t v)
{
    assert(r->pos < r->size);
    r->p[r->pos++] = v;
}

void nubus_declrom_put16(NubusDeclRom *r, uint16_t v)
{
    nubus_declrom_put8(r, v >> 8);
    nubus_declrom_put8(r, v);
}

void nubus_declrom_put32(NubusDeclRom *r, uint32_t v)
{
    nubus_declrom_put16(r, v >> 16);
    nubus_declrom_put16(r, v);
}

void nubus_declrom_puts(NubusDeclRom *r, const char *s)
{
    nubus_declrom_put_bytes(r, s, strlen(s) + 1);
}

void nubus_declrom_put_bytes(NubusDeclRom *r, const void *data, int len)
{
    assert(r->pos + len <= r->size);
    memcpy(r->p + r->pos, data, len);
    r->pos += len;
}

void nubus_declrom_align(NubusDeclRom *r, int align)
{
    while (r->pos % align) {
        nubus_declrom_put8(r, 0);
    }
}

void nubus_declrom_entry(NubusDeclRom *r, int pos, uint8_t id, uint32_t data)
{
    assert(pos + 4 <= r->size);
    r->p[pos] = id;
    r->p[pos + 1] = data >> 16;
    r->p[pos + 2] = data >> 8;
    r->p[pos + 3] = data;
}

void nubus_declrom_entry_off(NubusDeclRom *r, int pos, uint8_t id,
                             int target)
{
    nubus_declrom_entry(r, pos, id, (target - pos) & 0xffffff);
}

/*
 * The Slot Manager checks the CRC: over the whole ROM, the sum is rotated
 * left by one bit before each byte is added, the CRC field counts as 0.
 */
static uint32_t nubus_declrom_crc(const uint8_t *p, int len, int crc_pos)
{
    uint32_t sum = 0;

    for (int i = 0; i < len; i++) {
        sum = (sum << 1) | (sum >> 31);
        if (i < crc_pos || i >= crc_pos + 4) {
            sum += p[i];
        }
    }
    return sum;
}

int nubus_declrom_finish(NubusDeclRom *r)
{
    int fblock = r->pos;
    int len = fblock + FBLOCK_SIZE;

    nubus_declrom_put32(r, (0 - fblock) & 0xffffff);   /* directory offset */
    nubus_declrom_put32(r, len);                        /* length */
    nubus_declrom_put32(r, 0);                          /* CRC */
    nubus_declrom_put8(r, 1);                           /* revision */
    nubus_declrom_put8(r, 1);                           /* format: Apple */
    nubus_declrom_put32(r, ROM_TEST_PATTERN);
    nubus_declrom_put8(r, 0);
    nubus_declrom_put8(r, 0x0f);                        /* byte lanes: all */

    stl_be_p(r->p + fblock + FBLOCK_CRC,
             nubus_declrom_crc(r->p, len, fblock + FBLOCK_CRC));
    return len;
}
