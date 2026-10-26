/*
 * Building NuBus declaration ROMs
 *
 * A ROM is built from offset 0 upwards with all four byte lanes in use and
 * gets its format block appended by nubus_declrom_finish(); the result
 * belongs at the top of the slot space. Directory entries hold 24-bit
 * offsets relative to themselves, so the ROM can be moved as a whole.
 * See Apple's "Designing Cards and Drivers for the Macintosh Family".
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_NUBUS_DECLROM_H
#define HW_NUBUS_DECLROM_H

/* sResource IDs */
#define NUBUS_RESID_TYPE            0x01
#define NUBUS_RESID_NAME            0x02
#define NUBUS_RESID_DRVR_DIR        0x04
#define NUBUS_RESID_FLAGS           0x07
#define NUBUS_RESID_HWDEVID         0x08
#define NUBUS_RESID_MINOR_BASEOS    0x0a
#define NUBUS_RESID_MINOR_LENGTH    0x0b
#define NUBUS_RESID_BOARDID         0x20
#define NUBUS_RESID_END             0xff

/* driver directory */
#define NUBUS_RESID_MACOS68020      0x02

typedef struct NubusDeclRom {
    uint8_t *p;
    int size;
    int pos;
} NubusDeclRom;

void nubus_declrom_init(NubusDeclRom *r, uint8_t *buf, int size);
void nubus_declrom_put8(NubusDeclRom *r, uint8_t v);
void nubus_declrom_put16(NubusDeclRom *r, uint16_t v);
void nubus_declrom_put32(NubusDeclRom *r, uint32_t v);
void nubus_declrom_puts(NubusDeclRom *r, const char *s);
void nubus_declrom_put_bytes(NubusDeclRom *r, const void *data, int len);
void nubus_declrom_align(NubusDeclRom *r, int align);

/* directory entry at @pos: an ID and a 24-bit value */
void nubus_declrom_entry(NubusDeclRom *r, int pos, uint8_t id, uint32_t data);

/* directory entry at @pos: an ID and the offset of @target from @pos */
void nubus_declrom_entry_off(NubusDeclRom *r, int pos, uint8_t id,
                             int target);

/*
 * Append the format block, with the root directory at offset 0, and fill
 * in the CRC. Returns the length of the ROM.
 */
int nubus_declrom_finish(NubusDeclRom *r);

#endif
