/*
 * QEMU Motorola 680x0 Macintosh Video Card Emulation
 *                 Copyright (c) 2012-2018 Laurent Vivier
 *
 * some parts from QEMU G364 framebuffer Emulator.
 *                 Copyright (c) 2007-2011 Herve Poussineau
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 *
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "hw/core/sysbus.h"
#include "ui/console.h"
#include "ui/pixel_ops.h"
#include "hw/nubus/nubus.h"
#include "hw/nubus/nubus-declrom.h"
#include "hw/display/macfb.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "trace.h"
#include "macfb-drvr.h"

#define VIDEO_BASE 0x0
#define DAFB_BASE  0x00800000

#define MACFB_PAGE_SIZE 4096
#define MACFB_VRAM_SIZE (4 * MiB)

#define DAFB_MODE_VADDR1    0x0
#define DAFB_MODE_VADDR2    0x4
#define DAFB_MODE_CTRL1     0x8
#define DAFB_MODE_CTRL2     0xc
#define DAFB_MODE_SENSE     0x1c
#define DAFB_INTR_MASK      0x104
#define DAFB_INTR_STAT      0x108
#define DAFB_INTR_CLEAR     0x10c
#define DAFB_LUT_INDEX      0x200
#define DAFB_LUT            0x210

#define DAFB_INTR_VBL   0x4

/* Vertical Blank period (60.15Hz) */
#define DAFB_INTR_VBL_PERIOD_NS 16625800

/*
 * Quadra sense codes taken from Apple Technical Note HW26:
 * "Macintosh Quadra Built-In Video". The sense codes and
 * extended sense codes have different meanings:
 *
 * Sense:
 *    bit 2: SENSE2 (pin 10)
 *    bit 1: SENSE1 (pin 7)
 *    bit 0: SENSE0 (pin 4)
 *
 * 0 = pin tied to ground
 * 1 = pin unconnected
 *
 * Extended Sense:
 *    bit 2: pins 4-10
 *    bit 1: pins 10-7
 *    bit 0: pins 7-4
 *
 * 0 = pins tied together
 * 1 = pins unconnected
 *
 * Reads from the sense register appear to be active low, i.e. a 1 indicates
 * that the pin is tied to ground, a 0 indicates the pin is disconnected.
 *
 * Writes to the sense register appear to activate pulldowns i.e. a 1 enables
 * a pulldown on a particular pin.
 *
 * The MacOS toolbox appears to use a series of reads and writes to first
 * determine if extended sense is to be used, and then check which pins are
 * tied together in order to determine the display type.
 */

typedef struct MacFbSense {
    uint8_t type;
    uint8_t sense;
    uint8_t ext_sense;
} MacFbSense;

static const MacFbSense macfb_sense_table[] = {
    { MACFB_DISPLAY_APPLE_21_COLOR, 0x0, 0 },
    { MACFB_DISPLAY_APPLE_PORTRAIT, 0x1, 0 },
    { MACFB_DISPLAY_APPLE_12_RGB, 0x2, 0 },
    { MACFB_DISPLAY_APPLE_2PAGE_MONO, 0x3, 0 },
    { MACFB_DISPLAY_NTSC_UNDERSCAN, 0x4, 0 },
    { MACFB_DISPLAY_NTSC_OVERSCAN, 0x4, 0 },
    { MACFB_DISPLAY_APPLE_12_MONO, 0x6, 0 },
    { MACFB_DISPLAY_APPLE_13_RGB, 0x6, 0 },
    { MACFB_DISPLAY_16_COLOR, 0x7, 0x3 },
    { MACFB_DISPLAY_PAL1_UNDERSCAN, 0x7, 0x0 },
    { MACFB_DISPLAY_PAL1_OVERSCAN, 0x7, 0x0 },
    { MACFB_DISPLAY_PAL2_UNDERSCAN, 0x7, 0x6 },
    { MACFB_DISPLAY_PAL2_OVERSCAN, 0x7, 0x6 },
    { MACFB_DISPLAY_VGA, 0x7, 0x5 },
    { MACFB_DISPLAY_SVGA, 0x7, 0x5 },
};

static const MacFbMode macfb_mode_table[] = {
    { MACFB_DISPLAY_VGA, 1, 0x100, 0x71e, 640, 480, 0x400, 0x1000 },
    { MACFB_DISPLAY_VGA, 2, 0x100, 0x70e, 640, 480, 0x400, 0x1000 },
    { MACFB_DISPLAY_VGA, 4, 0x100, 0x706, 640, 480, 0x400, 0x1000 },
    { MACFB_DISPLAY_VGA, 8, 0x100, 0x702, 640, 480, 0x400, 0x1000 },
    { MACFB_DISPLAY_VGA, 24, 0x100, 0x7ff, 640, 480, 0x1000, 0x1000 },
    { MACFB_DISPLAY_VGA, 1, 0xd0 , 0x70e, 800, 600, 0x340, 0xe00 },
    { MACFB_DISPLAY_VGA, 2, 0xd0 , 0x706, 800, 600, 0x340, 0xe00 },
    { MACFB_DISPLAY_VGA, 4, 0xd0 , 0x702, 800, 600, 0x340, 0xe00 },
    { MACFB_DISPLAY_VGA, 8, 0xd0,  0x700, 800, 600, 0x340, 0xe00 },
    { MACFB_DISPLAY_VGA, 24, 0x340, 0x100, 800, 600, 0xd00, 0xe00 },
    { MACFB_DISPLAY_APPLE_21_COLOR, 1, 0x90, 0x506, 1152, 870, 0x240, 0x80 },
    { MACFB_DISPLAY_APPLE_21_COLOR, 2, 0x90, 0x502, 1152, 870, 0x240, 0x80 },
    { MACFB_DISPLAY_APPLE_21_COLOR, 4, 0x90, 0x500, 1152, 870, 0x240, 0x80 },
    { MACFB_DISPLAY_APPLE_21_COLOR, 8, 0x120, 0x5ff, 1152, 870, 0x480, 0x80 },
};

typedef void macfb_draw_line_func(MacfbState *s, uint8_t *d, uint32_t addr,
                                  int width);

static inline uint8_t macfb_read_byte(MacfbState *s, uint32_t addr)
{
    return s->vram[addr & s->vram_bit_mask];
}

/* 1-bit color */
static void macfb_draw_line1(MacfbState *s, uint8_t *d, uint32_t addr,
                             int width)
{
    uint8_t r, g, b;
    int x;

    for (x = 0; x < width; x++) {
        int bit = x & 7;
        int idx = (macfb_read_byte(s, addr) >> (7 - bit)) & 1;
        r = s->color_palette[idx * 3];
        g = s->color_palette[idx * 3 + 1];
        b = s->color_palette[idx * 3 + 2];
        addr += (bit == 7);

        *(uint32_t *)d = rgb_to_pixel32(r, g, b);
        d += 4;
    }
}

/* 2-bit color */
static void macfb_draw_line2(MacfbState *s, uint8_t *d, uint32_t addr,
                             int width)
{
    uint8_t r, g, b;
    int x;

    for (x = 0; x < width; x++) {
        int bit = (x & 3);
        int idx = (macfb_read_byte(s, addr) >> ((3 - bit) << 1)) & 3;
        r = s->color_palette[idx * 3];
        g = s->color_palette[idx * 3 + 1];
        b = s->color_palette[idx * 3 + 2];
        addr += (bit == 3);

        *(uint32_t *)d = rgb_to_pixel32(r, g, b);
        d += 4;
    }
}

/* 4-bit color */
static void macfb_draw_line4(MacfbState *s, uint8_t *d, uint32_t addr,
                             int width)
{
    uint8_t r, g, b;
    int x;

    for (x = 0; x < width; x++) {
        int bit = x & 1;
        int idx = (macfb_read_byte(s, addr) >> ((1 - bit) << 2)) & 15;
        r = s->color_palette[idx * 3];
        g = s->color_palette[idx * 3 + 1];
        b = s->color_palette[idx * 3 + 2];
        addr += (bit == 1);

        *(uint32_t *)d = rgb_to_pixel32(r, g, b);
        d += 4;
    }
}

/* 8-bit color */
static void macfb_draw_line8(MacfbState *s, uint8_t *d, uint32_t addr,
                             int width)
{
    uint8_t r, g, b;
    int x;

    for (x = 0; x < width; x++) {
        r = s->color_palette[macfb_read_byte(s, addr) * 3];
        g = s->color_palette[macfb_read_byte(s, addr) * 3 + 1];
        b = s->color_palette[macfb_read_byte(s, addr) * 3 + 2];
        addr++;

        *(uint32_t *)d = rgb_to_pixel32(r, g, b);
        d += 4;
    }
}

/* 16-bit color */
static void macfb_draw_line16(MacfbState *s, uint8_t *d, uint32_t addr,
                              int width)
{
    uint8_t r, g, b;
    int x;

    for (x = 0; x < width; x++) {
        uint16_t pixel;
        pixel = (macfb_read_byte(s, addr) << 8) | macfb_read_byte(s, addr + 1);
        r = ((pixel >> 10) & 0x1f) << 3;
        g = ((pixel >> 5) & 0x1f) << 3;
        b = (pixel & 0x1f) << 3;
        addr += 2;

        *(uint32_t *)d = rgb_to_pixel32(r, g, b);
        d += 4;
    }
}

/* 24-bit color */
static void macfb_draw_line24(MacfbState *s, uint8_t *d, uint32_t addr,
                              int width)
{
    uint8_t r, g, b;
    int x;

    for (x = 0; x < width; x++) {
        r = macfb_read_byte(s, addr + 1);
        g = macfb_read_byte(s, addr + 2);
        b = macfb_read_byte(s, addr + 3);
        addr += 4;

        *(uint32_t *)d = rgb_to_pixel32(r, g, b);
        d += 4;
    }
}


enum {
    MACFB_DRAW_LINE1,
    MACFB_DRAW_LINE2,
    MACFB_DRAW_LINE4,
    MACFB_DRAW_LINE8,
    MACFB_DRAW_LINE16,
    MACFB_DRAW_LINE24,
    MACFB_DRAW_LINE_NB,
};

static macfb_draw_line_func * const
                              macfb_draw_line_table[MACFB_DRAW_LINE_NB] = {
    macfb_draw_line1,
    macfb_draw_line2,
    macfb_draw_line4,
    macfb_draw_line8,
    macfb_draw_line16,
    macfb_draw_line24,
};

static int macfb_check_dirty(MacfbState *s, DirtyBitmapSnapshot *snap,
                             ram_addr_t addr, int len)
{
    return memory_region_snapshot_get_dirty(&s->mem_vram, snap, addr, len);
}

static void macfb_draw_graphic(MacfbState *s)
{
    DisplaySurface *surface = qemu_console_surface(s->con);
    DirtyBitmapSnapshot *snap = NULL;
    ram_addr_t page;
    uint32_t v = 0;
    int y, ymin;
    int macfb_stride = s->mode->stride;
    macfb_draw_line_func *macfb_draw_line;

    switch (s->depth) {
    case 1:
        v = MACFB_DRAW_LINE1;
        break;
    case 2:
        v = MACFB_DRAW_LINE2;
        break;
    case 4:
        v = MACFB_DRAW_LINE4;
        break;
    case 8:
        v = MACFB_DRAW_LINE8;
        break;
    case 16:
        v = MACFB_DRAW_LINE16;
        break;
    case 24:
        v = MACFB_DRAW_LINE24;
        break;
    }

    macfb_draw_line = macfb_draw_line_table[v];
    assert(macfb_draw_line != NULL);

    snap = memory_region_snapshot_and_clear_dirty(&s->mem_vram, 0x0,
                                             memory_region_size(&s->mem_vram),
                                             DIRTY_MEMORY_VGA);

    ymin = -1;
    page = s->mode->offset;
    for (y = 0; y < s->height; y++, page += macfb_stride) {
        if (macfb_check_dirty(s, snap, page, macfb_stride)) {
            uint8_t *data_display;

            data_display = surface_data(surface) + y * surface_stride(surface);
            macfb_draw_line(s, data_display, page, s->width);

            if (ymin < 0) {
                ymin = y;
            }
        } else {
            if (ymin >= 0) {
                qemu_console_update(s->con, 0, ymin, s->width, y - ymin);
                ymin = -1;
            }
        }
    }

    if (ymin >= 0) {
        qemu_console_update(s->con, 0, ymin, s->width, y - ymin);
    }

    g_free(snap);
}

static void macfb_invalidate_display(void *opaque)
{
    MacfbState *s = opaque;

    memory_region_set_dirty(&s->mem_vram, 0, MACFB_VRAM_SIZE);
}

static uint32_t macfb_sense_read(MacfbState *s)
{
    const MacFbSense *macfb_sense;
    uint8_t sense;

    assert(s->type < ARRAY_SIZE(macfb_sense_table));
    macfb_sense = &macfb_sense_table[s->type];
    if (macfb_sense->sense == 0x7) {
        /* Extended sense */
        sense = 0;
        if (!(macfb_sense->ext_sense & 1)) {
            /* Pins 7-4 together */
            if (~s->regs[DAFB_MODE_SENSE >> 2] & 3) {
                sense = (~s->regs[DAFB_MODE_SENSE >> 2] & 7) | 3;
            }
        }
        if (!(macfb_sense->ext_sense & 2)) {
            /* Pins 10-7 together */
            if (~s->regs[DAFB_MODE_SENSE >> 2] & 6) {
                sense = (~s->regs[DAFB_MODE_SENSE >> 2] & 7) | 6;
            }
        }
        if (!(macfb_sense->ext_sense & 4)) {
            /* Pins 4-10 together */
            if (~s->regs[DAFB_MODE_SENSE >> 2] & 5) {
                sense = (~s->regs[DAFB_MODE_SENSE >> 2] & 7) | 5;
            }
        }
    } else {
        /* Normal sense */
        sense = (~macfb_sense->sense & 7) |
                (~s->regs[DAFB_MODE_SENSE >> 2] & 7);
    }

    trace_macfb_sense_read(sense);
    return sense;
}

static void macfb_sense_write(MacfbState *s, uint32_t val)
{
    s->regs[DAFB_MODE_SENSE >> 2] = val;

    trace_macfb_sense_write(val);
}

static void macfb_update_mode(MacfbState *s)
{
    s->width = s->mode->width;
    s->height = s->mode->height;
    s->depth = s->mode->depth;

    trace_macfb_update_mode(s->width, s->height, s->depth);
    macfb_invalidate_display(s);
}

static void macfb_mode_write(MacfbState *s)
{
    const MacFbMode *macfb_mode;
    int i;

    for (i = 0; i < ARRAY_SIZE(macfb_mode_table); i++) {
        macfb_mode = &macfb_mode_table[i];

        if (s->type != macfb_mode->type) {
            continue;
        }

        if ((s->regs[DAFB_MODE_CTRL1 >> 2] & 0xff) ==
             (macfb_mode->mode_ctrl1 & 0xff) &&
            (s->regs[DAFB_MODE_CTRL2 >> 2] & 0xff) ==
             (macfb_mode->mode_ctrl2 & 0xff)) {
            s->mode = macfb_mode;
            macfb_update_mode(s);
            break;
        }
    }
}

static const MacFbMode *macfb_find_mode(MacfbDisplayType display_type,
                                  uint16_t width, uint16_t height,
                                  uint8_t depth)
{
    const MacFbMode *macfb_mode;
    int i;

    for (i = 0; i < ARRAY_SIZE(macfb_mode_table); i++) {
        macfb_mode = &macfb_mode_table[i];

        if (display_type == macfb_mode->type && width == macfb_mode->width &&
                height == macfb_mode->height && depth == macfb_mode->depth) {
            return macfb_mode;
        }
    }

    return NULL;
}

static gchar *macfb_mode_list(void)
{
    GString *list = g_string_new("");
    const MacFbMode *macfb_mode;
    int i;

    for (i = 0; i < ARRAY_SIZE(macfb_mode_table); i++) {
        macfb_mode = &macfb_mode_table[i];

        g_string_append_printf(list, "    %dx%dx%d\n", macfb_mode->width,
                               macfb_mode->height, macfb_mode->depth);
    }

    return g_string_free(list, FALSE);
}


static bool macfb_update_display(void *opaque)
{
    MacfbState *s = opaque;
    DisplaySurface *surface = qemu_console_surface(s->con);

    qemu_flush_coalesced_mmio_buffer();

    if (s->width == 0 || s->height == 0) {
        return true;
    }

    if (s->width != surface_width(surface) ||
        s->height != surface_height(surface)) {
        qemu_console_resize(s->con, s->width, s->height);
    }

    macfb_draw_graphic(s);

    return true;
}

static void macfb_update_irq(MacfbState *s)
{
    uint32_t irq_state = s->regs[DAFB_INTR_STAT >> 2] &
                         s->regs[DAFB_INTR_MASK >> 2];

    if (irq_state) {
        qemu_irq_raise(s->irq);
    } else {
        qemu_irq_lower(s->irq);
    }
}

static int64_t macfb_next_vbl(void)
{
    return (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + DAFB_INTR_VBL_PERIOD_NS) /
            DAFB_INTR_VBL_PERIOD_NS * DAFB_INTR_VBL_PERIOD_NS;
}

static void macfb_vbl_timer(void *opaque)
{
    MacfbState *s = opaque;
    int64_t next_vbl;

    s->regs[DAFB_INTR_STAT >> 2] |= DAFB_INTR_VBL;
    macfb_update_irq(s);

    /* 60 Hz irq */
    next_vbl = macfb_next_vbl();
    timer_mod(s->vbl_timer, next_vbl);
}

/* the Linux console colours, for when nothing will program the CLUT */
static const uint8_t macfb_vga_palette[16 * 3] = {
    0x00, 0x00, 0x00,  0xaa, 0x00, 0x00,  0x00, 0xaa, 0x00,  0xaa, 0x55, 0x00,
    0x00, 0x00, 0xaa,  0xaa, 0x00, 0xaa,  0x00, 0xaa, 0xaa,  0xaa, 0xaa, 0xaa,
    0x55, 0x55, 0x55,  0xff, 0x55, 0x55,  0x55, 0xff, 0x55,  0xff, 0xff, 0x55,
    0x55, 0x55, 0xff,  0xff, 0x55, 0xff,  0x55, 0xff, 0xff,  0xff, 0xff, 0xff,
};

static void macfb_reset(MacfbState *s)
{
    int i;

    s->palette_current = 0;
    for (i = 0; i < 256; i++) {
        s->color_palette[i * 3] = 255 - i;
        s->color_palette[i * 3 + 1] = 255 - i;
        s->color_palette[i * 3 + 2] = 255 - i;
    }
    if (s->vga_palette) {
        memcpy(s->color_palette, macfb_vga_palette, sizeof(macfb_vga_palette));
    }
    memset(s->vram, 0, MACFB_VRAM_SIZE);
    macfb_invalidate_display(s);
}

static uint64_t macfb_ctrl_read(void *opaque,
                                hwaddr addr,
                                unsigned int size)
{
    MacfbState *s = opaque;
    uint64_t val = 0;

    switch (addr) {
    case DAFB_MODE_VADDR1:
    case DAFB_MODE_VADDR2:
    case DAFB_MODE_CTRL1:
    case DAFB_MODE_CTRL2:
    case DAFB_INTR_STAT:
        val = s->regs[addr >> 2];
        break;
    case DAFB_MODE_SENSE:
        val = macfb_sense_read(s);
        break;
    case DAFB_LUT ... DAFB_LUT + 3:
        val = s->color_palette[s->palette_current];
        s->palette_current = (s->palette_current + 1) %
                             ARRAY_SIZE(s->color_palette);
        break;
    default:
        if (addr < MACFB_CTRL_TOPADDR) {
            val = s->regs[addr >> 2];
        }
    }

    trace_macfb_ctrl_read(addr, val, size);
    return val;
}

static void macfb_ctrl_write(void *opaque,
                             hwaddr addr,
                             uint64_t val,
                             unsigned int size)
{
    MacfbState *s = opaque;
    int64_t next_vbl;

    switch (addr) {
    case DAFB_MODE_VADDR1:
    case DAFB_MODE_VADDR2:
        s->regs[addr >> 2] = val;
        break;
    case DAFB_MODE_CTRL1 ... DAFB_MODE_CTRL1 + 3:
    case DAFB_MODE_CTRL2 ... DAFB_MODE_CTRL2 + 3:
        s->regs[addr >> 2] = val;
        if (val) {
            macfb_mode_write(s);
        }
        break;
    case DAFB_MODE_SENSE:
        macfb_sense_write(s, val);
        break;
    case DAFB_INTR_MASK:
        s->regs[addr >> 2] = val;
        if (val & DAFB_INTR_VBL) {
            next_vbl = macfb_next_vbl();
            timer_mod(s->vbl_timer, next_vbl);
        } else {
            timer_del(s->vbl_timer);
        }
        break;
    case DAFB_INTR_CLEAR:
        s->regs[DAFB_INTR_STAT >> 2] &= ~DAFB_INTR_VBL;
        macfb_update_irq(s);
        break;
    case DAFB_LUT_INDEX:
        s->palette_current = (val & 0xff) * 3;
        break;
    case DAFB_LUT ... DAFB_LUT + 3:
        s->color_palette[s->palette_current] = val & 0xff;
        s->palette_current = (s->palette_current + 1) %
                             ARRAY_SIZE(s->color_palette);
        if (s->palette_current % 3) {
            macfb_invalidate_display(s);
        }
        break;
    default:
        if (addr < MACFB_CTRL_TOPADDR) {
            s->regs[addr >> 2] = val;
        }
    }

    trace_macfb_ctrl_write(addr, val, size);
}

static const MemoryRegionOps macfb_ctrl_ops = {
    .read = macfb_ctrl_read,
    .write = macfb_ctrl_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static int macfb_post_load(void *opaque, int version_id)
{
    macfb_mode_write(opaque);
    return 0;
}

static const VMStateDescription vmstate_macfb = {
    .name = "macfb",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = macfb_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8(type, MacfbState),
        VMSTATE_UINT8_ARRAY(color_palette, MacfbState, 256 * 3),
        VMSTATE_UINT32(palette_current, MacfbState),
        VMSTATE_UINT32_ARRAY(regs, MacfbState, MACFB_NUM_REGS),
        VMSTATE_TIMER_PTR(vbl_timer, MacfbState),
        VMSTATE_END_OF_LIST()
    }
};

static const GraphicHwOps macfb_ops = {
    .invalidate = macfb_invalidate_display,
    .gfx_update = macfb_update_display,
};

static bool macfb_common_realize(DeviceState *dev, MacfbState *s, Error **errp)
{
    DisplaySurface *surface;

    if (s->width == 1152 && s->height == 870) {
        s->type = MACFB_DISPLAY_APPLE_21_COLOR;
    }

    s->mode = macfb_find_mode(s->type, s->width, s->height, s->depth);
    if (!s->mode) {
        gchar *list;
        error_setg(errp, "unknown display mode: width %d, height %d, depth %d",
                   s->width, s->height, s->depth);
        list = macfb_mode_list();
        error_append_hint(errp, "Available modes:\n%s", list);
        g_free(list);

        return false;
    }

    /*
     * Set mode control registers to match the mode found above so that
     * macfb_mode_write() does the right thing if no MacOS toolbox ROM
     * is present to initialise them
     */
    s->regs[DAFB_MODE_CTRL1 >> 2] = s->mode->mode_ctrl1;
    s->regs[DAFB_MODE_CTRL2 >> 2] = s->mode->mode_ctrl2;

    s->con = qemu_graphic_console_create(dev, 0, &macfb_ops, s);
    surface = qemu_console_surface(s->con);

    if (surface_bits_per_pixel(surface) != 32) {
        error_setg(errp, "unknown host depth %d",
                   surface_bits_per_pixel(surface));
        return false;
    }

    memory_region_init_io(&s->mem_ctrl, OBJECT(dev), &macfb_ctrl_ops, s,
                          "macfb-ctrl", 0x1000);

    memory_region_init_ram(&s->mem_vram, OBJECT(dev), "macfb-vram",
                           MACFB_VRAM_SIZE, &error_abort);
    memory_region_set_log(&s->mem_vram, true, DIRTY_MEMORY_VGA);
    s->vram = memory_region_get_ram_ptr(&s->mem_vram);
    s->vram_bit_mask = MACFB_VRAM_SIZE - 1;

    s->vbl_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, macfb_vbl_timer, s);
    macfb_update_mode(s);
    return true;
}

static void macfb_sysbus_realize(DeviceState *dev, Error **errp)
{
    MacfbSysBusState *s = MACFB(dev);
    MacfbState *ms = &s->macfb;

    if (!macfb_common_realize(dev, ms, errp)) {
        return;
    }

    sysbus_init_mmio(SYS_BUS_DEVICE(s), &ms->mem_ctrl);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &ms->mem_vram);

    qdev_init_gpio_out(dev, &ms->irq, 1);
}

static void macfb_nubus_set_irq(void *opaque, int n, int level)
{
    MacfbNubusState *s = NUBUS_MACFB(opaque);
    NubusDevice *nd = NUBUS_DEVICE(s);

    nubus_set_irq(nd, level);
}

/*
 * Declaration ROM of the NuBus card for the Mac OS Slot Manager: a board
 * sResource and a video sResource (catDisplay, typVideo, drSwApple) with
 * the indexed modes 0x80 to 0x83 (1, 2, 4 and 8 bits) and the driver
 * from macfb-drvr.S. The base address is in the minor slot space, so the
 * frame buffer is also reachable in 24-bit mode, where the Mac II maps
 * $s00000 to $Fss00000.
 */
#define MACFB_DECL_ROM_SIZE     0x1000
#define MACFB_ROM_MODES         4
#define MACFB_ROM_FIRST_MODE    0x80

#define RESID_MODE_PARAMS       0x01
#define RESID_MODE_PAGE_CNT     0x03
#define RESID_MODE_DEV_TYPE     0x04

static bool macfb_nubus_build_rom(MacfbState *s, uint8_t *buf, int size,
                                  int *rom_len, Error **errp)
{
    static const uint8_t depths[MACFB_ROM_MODES] = { 1, 2, 4, 8 };
    const MacFbMode *modes[MACFB_ROM_MODES];
    uint8_t drvr[sizeof(macfb_drvr)];
    NubusDeclRom r;
    int board_dir, video_dir, mode_dir[MACFB_ROM_MODES], drvr_dir;
    int board_type, board_name, video_type, video_name, minor_base;
    int minor_len, vpblock[MACFB_ROM_MODES], drvr_block, pos, i;

    for (i = 0; i < MACFB_ROM_MODES; i++) {
        modes[i] = macfb_find_mode(s->type, s->mode->width, s->mode->height,
                                   depths[i]);
        if (!modes[i]) {
            error_setg(errp, "no %d bit mode for %dx%d, needed by the "
                       "declaration ROM", depths[i], s->mode->width,
                       s->mode->height);
            return false;
        }
    }

    /* the driver's mode table */
    memcpy(drvr, macfb_drvr, sizeof(drvr));
    pos = MACFB_DRVR_PARAMS;
    stw_be_p(drvr + pos, s->mode->height);
    pos += 4;
    for (i = 0; i < MACFB_ROM_MODES; i++, pos += 12) {
        stw_be_p(drvr + pos, modes[i]->mode_ctrl1);
        stw_be_p(drvr + pos + 2, modes[i]->mode_ctrl2);
        stl_be_p(drvr + pos + 4, modes[i]->offset);
        stw_be_p(drvr + pos + 8, modes[i]->stride);
        stw_be_p(drvr + pos + 10, modes[i]->depth);
    }

    nubus_declrom_init(&r, buf, size);

    /* directories first, their entries are filled in below */
    r.pos = 0x10;
    board_dir = r.pos;
    r.pos += 4 * 4;
    video_dir = r.pos;
    r.pos += (7 + MACFB_ROM_MODES + 1) * 4;
    for (i = 0; i < MACFB_ROM_MODES; i++) {
        mode_dir[i] = r.pos;
        r.pos += 4 * 4;
    }
    drvr_dir = r.pos;
    r.pos += 2 * 4;

    board_type = r.pos;
    nubus_declrom_put16(&r, 0x0001);    /* catBoard */
    nubus_declrom_put16(&r, 0x0000);
    nubus_declrom_put16(&r, 0x0000);
    nubus_declrom_put16(&r, 0x0000);

    video_type = r.pos;
    nubus_declrom_put16(&r, 0x0003);    /* catDisplay */
    nubus_declrom_put16(&r, 0x0001);    /* typVideo */
    nubus_declrom_put16(&r, 0x0001);    /* drSwApple */
    nubus_declrom_put16(&r, 0x5146);    /* DrHW, ours */

    minor_base = r.pos;
    nubus_declrom_put32(&r, 0);
    minor_len = r.pos;
    nubus_declrom_put32(&r, 1 * MiB);

    board_name = r.pos;
    nubus_declrom_puts(&r, "QEMU NuBus Video");
    video_name = r.pos;
    nubus_declrom_puts(&r, "Display_Video_QEMU_macfb");
    nubus_declrom_align(&r, 4);

    /* VPBlock */
    for (i = 0; i < MACFB_ROM_MODES; i++) {
        vpblock[i] = r.pos;
        nubus_declrom_put32(&r, 4 + 42);            /* sBlock length */
        nubus_declrom_put32(&r, modes[i]->offset);  /* vpBaseOffset */
        nubus_declrom_put16(&r, modes[i]->stride);  /* vpRowBytes */
        nubus_declrom_put16(&r, 0);                 /* vpBounds */
        nubus_declrom_put16(&r, 0);
        nubus_declrom_put16(&r, modes[i]->height);
        nubus_declrom_put16(&r, modes[i]->width);
        nubus_declrom_put16(&r, 0);                 /* vpVersion */
        nubus_declrom_put16(&r, 0);                 /* vpPackType */
        nubus_declrom_put32(&r, 0);                 /* vpPackSize */
        nubus_declrom_put32(&r, 72 << 16);          /* vpHRes */
        nubus_declrom_put32(&r, 72 << 16);          /* vpVRes */
        nubus_declrom_put16(&r, 0);                 /* vpPixelType: chunky */
        nubus_declrom_put16(&r, modes[i]->depth);   /* vpPixelSize */
        nubus_declrom_put16(&r, 1);                 /* vpCmpCount */
        nubus_declrom_put16(&r, modes[i]->depth);   /* vpCmpSize */
        nubus_declrom_put32(&r, 0);                 /* vpPlaneBytes */
    }

    drvr_block = r.pos;
    nubus_declrom_put32(&r, 4 + sizeof(drvr));
    nubus_declrom_put_bytes(&r, drvr, sizeof(drvr));
    nubus_declrom_align(&r, 4);

    /* root */
    nubus_declrom_entry_off(&r, 0x00, 0x01, board_dir);
    nubus_declrom_entry_off(&r, 0x04, 0x80, video_dir);
    nubus_declrom_entry(&r, 0x08, NUBUS_RESID_END, 0);

    pos = board_dir;
    nubus_declrom_entry_off(&r, pos, NUBUS_RESID_TYPE, board_type);
    nubus_declrom_entry_off(&r, pos + 4, NUBUS_RESID_NAME, board_name);
    nubus_declrom_entry(&r, pos + 8, NUBUS_RESID_BOARDID, 0x5146);
    nubus_declrom_entry(&r, pos + 12, NUBUS_RESID_END, 0);

    pos = video_dir;
    nubus_declrom_entry_off(&r, pos, NUBUS_RESID_TYPE, video_type);
    nubus_declrom_entry_off(&r, pos + 4, NUBUS_RESID_NAME, video_name);
    nubus_declrom_entry_off(&r, pos + 8, NUBUS_RESID_DRVR_DIR, drvr_dir);
    nubus_declrom_entry(&r, pos + 12, NUBUS_RESID_FLAGS, 0x0002);
    nubus_declrom_entry(&r, pos + 16, NUBUS_RESID_HWDEVID, 1);
    nubus_declrom_entry_off(&r, pos + 20, NUBUS_RESID_MINOR_BASEOS,
                            minor_base);
    nubus_declrom_entry_off(&r, pos + 24, NUBUS_RESID_MINOR_LENGTH,
                            minor_len);
    for (i = 0; i < MACFB_ROM_MODES; i++) {
        nubus_declrom_entry_off(&r, pos + 28 + i * 4,
                                MACFB_ROM_FIRST_MODE + i, mode_dir[i]);
    }
    nubus_declrom_entry(&r, pos + 28 + i * 4, NUBUS_RESID_END, 0);

    for (i = 0; i < MACFB_ROM_MODES; i++) {
        pos = mode_dir[i];
        nubus_declrom_entry_off(&r, pos, RESID_MODE_PARAMS, vpblock[i]);
        nubus_declrom_entry(&r, pos + 4, RESID_MODE_PAGE_CNT, 1);
        nubus_declrom_entry(&r, pos + 8, RESID_MODE_DEV_TYPE, 0); /* CLUT */
        nubus_declrom_entry(&r, pos + 12, NUBUS_RESID_END, 0);
    }

    nubus_declrom_entry_off(&r, drvr_dir, NUBUS_RESID_MACOS68020,
                            drvr_block);
    nubus_declrom_entry(&r, drvr_dir + 4, NUBUS_RESID_END, 0);

    *rom_len = nubus_declrom_finish(&r);
    return true;
}

static void macfb_nubus_realize(DeviceState *dev, Error **errp)
{
    ERRP_GUARD();
    NubusDevice *nd = NUBUS_DEVICE(dev);
    MacfbNubusState *s = NUBUS_MACFB(dev);
    MacfbNubusDeviceClass *ndc = NUBUS_MACFB_GET_CLASS(dev);
    MacfbState *ms = &s->macfb;

    ndc->parent_realize(dev, errp);
    if (*errp) {
        return;
    }

    if (!macfb_common_realize(dev, ms, errp)) {
        return;
    }

    memory_region_add_subregion(&nd->slot_mem, DAFB_BASE, &ms->mem_ctrl);
    memory_region_add_subregion(&nd->slot_mem, VIDEO_BASE, &ms->mem_vram);

    if (!nd->romfile) {
        uint8_t rom[MACFB_DECL_ROM_SIZE];
        int rom_len;

        if (!macfb_nubus_build_rom(ms, rom, sizeof(rom), &rom_len, errp)) {
            return;
        }
        memory_region_init_rom(&s->rom, OBJECT(dev), "macfb-declrom",
                               MACFB_DECL_ROM_SIZE, &error_abort);
        memcpy((uint8_t *)memory_region_get_ram_ptr(&s->rom) +
               MACFB_DECL_ROM_SIZE - rom_len, rom, rom_len);
        memory_region_add_subregion(&nd->slot_mem,
                                    NUBUS_SLOT_SIZE - MACFB_DECL_ROM_SIZE,
                                    &s->rom);

        /* what 24-bit mode sees of the slot */
        memory_region_init_alias(&s->vram_24bit, OBJECT(dev),
                                 "macfb-vram-24bit", &ms->mem_vram, 0, 1 * MiB);
        memory_region_add_subregion(&nd->slot_mem, (hwaddr)nd->slot << 20,
                                    &s->vram_24bit);
    }

    ms->irq = qemu_allocate_irq(macfb_nubus_set_irq, s, 0);
}

static void macfb_nubus_unrealize(DeviceState *dev)
{
    MacfbNubusState *s = NUBUS_MACFB(dev);
    MacfbNubusDeviceClass *ndc = NUBUS_MACFB_GET_CLASS(dev);
    MacfbState *ms = &s->macfb;

    ndc->parent_unrealize(dev);

    qemu_free_irq(ms->irq);
}

static void macfb_sysbus_reset(DeviceState *d)
{
    MacfbSysBusState *s = MACFB(d);
    macfb_reset(&s->macfb);
}

static void macfb_nubus_reset(DeviceState *d)
{
    MacfbNubusState *s = NUBUS_MACFB(d);
    macfb_reset(&s->macfb);
}

static const Property macfb_sysbus_properties[] = {
    DEFINE_PROP_UINT32("width", MacfbSysBusState, macfb.width, 640),
    DEFINE_PROP_UINT32("height", MacfbSysBusState, macfb.height, 480),
    DEFINE_PROP_UINT8("depth", MacfbSysBusState, macfb.depth, 8),
    DEFINE_PROP_UINT8("display", MacfbSysBusState, macfb.type,
                      MACFB_DISPLAY_VGA),
};

static const VMStateDescription vmstate_macfb_sysbus = {
    .name = "macfb-sysbus",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT(macfb, MacfbSysBusState, 1, vmstate_macfb, MacfbState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property macfb_nubus_properties[] = {
    DEFINE_PROP_UINT32("width", MacfbNubusState, macfb.width, 640),
    DEFINE_PROP_UINT32("height", MacfbNubusState, macfb.height, 480),
    DEFINE_PROP_UINT8("depth", MacfbNubusState, macfb.depth, 8),
    DEFINE_PROP_UINT8("display", MacfbNubusState, macfb.type,
                      MACFB_DISPLAY_VGA),
    DEFINE_PROP_BOOL("vga-palette", MacfbNubusState, macfb.vga_palette, false),
};

static const VMStateDescription vmstate_macfb_nubus = {
    .name = "macfb-nubus",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT(macfb, MacfbNubusState, 1, vmstate_macfb, MacfbState),
        VMSTATE_END_OF_LIST()
    }
};

static void macfb_sysbus_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = macfb_sysbus_realize;
    dc->desc = "SysBus Macintosh framebuffer";
    device_class_set_legacy_reset(dc, macfb_sysbus_reset);
    dc->vmsd = &vmstate_macfb_sysbus;
    device_class_set_props(dc, macfb_sysbus_properties);
}

static void macfb_nubus_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    MacfbNubusDeviceClass *ndc = NUBUS_MACFB_CLASS(klass);

    device_class_set_parent_realize(dc, macfb_nubus_realize,
                                    &ndc->parent_realize);
    device_class_set_parent_unrealize(dc, macfb_nubus_unrealize,
                                      &ndc->parent_unrealize);
    dc->desc = "Nubus Macintosh framebuffer";
    device_class_set_legacy_reset(dc, macfb_nubus_reset);
    dc->vmsd = &vmstate_macfb_nubus;
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
    device_class_set_props(dc, macfb_nubus_properties);
}

static const TypeInfo macfb_sysbus_info = {
    .name          = TYPE_MACFB,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MacfbSysBusState),
    .class_init    = macfb_sysbus_class_init,
};

static const TypeInfo macfb_nubus_info = {
    .name          = TYPE_NUBUS_MACFB,
    .parent        = TYPE_NUBUS_DEVICE,
    .instance_size = sizeof(MacfbNubusState),
    .class_init    = macfb_nubus_class_init,
    .class_size    = sizeof(MacfbNubusDeviceClass),
};

static void macfb_register_types(void)
{
    type_register_static(&macfb_sysbus_info);
    type_register_static(&macfb_nubus_info);
}

type_init(macfb_register_types)
