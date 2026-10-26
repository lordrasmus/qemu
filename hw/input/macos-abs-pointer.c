/*
 * Absolute pointer for classic Mac OS
 *
 * The Mac's mice are relative and Mac OS accelerates what they report, so
 * its cursor drifts away from the host pointer and an ungrabbed pointer
 * leaves the window. Like Basilisk II and SheepShaver, this device writes
 * the host position straight into the low memory globals instead: MTemp,
 * RawMouse and Mouse, then CrsrNew from CrsrCouple, which the cursor VBL
 * task picks up. Buttons stay with the machine's mouse. Nothing is written
 * until Mac OS has set ScrnBase, and since any other guest would have its
 * memory written to, the device is only for guests running Mac OS 7 to 9.
 *
 * Where no VBL task draws the cursor, as with Mac OS 9 on the NewWorld
 * machines, steer=on leaves the globals alone and moves the relative mouse
 * instead: it reads where Mac OS has the cursor and sends half the way
 * there, at most STEER_MAX_STEP and the rest once within STEER_EXACT,
 * until it arrives, whatever Mac OS
 * accelerates. A step only follows once Mac OS has moved the cursor for the
 * previous one, or after STEER_WAIT_NS, so that steps do not pile up and
 * overshoot while the guest has yet to poll the mouse.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/qdev.h"
#include "hw/input/macos-abs-pointer.h"
#include "hw/core/cpu.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "qapi/error.h"
#include "ui/console.h"
#include "ui/input.h"
#include "qemu/timer.h"
#include "hw/core/qdev-properties.h"
#include "qom/object.h"

OBJECT_DECLARE_SIMPLE_TYPE(MacOSAbsPointerState, MACOS_ABS_POINTER)

#define LM_SCRNBASE     0x824
#define LM_MTEMP        0x828
#define LM_RAWMOUSE     0x82c
#define LM_MOUSE        0x830
#define LM_CRSRNEW      0x8ce
#define LM_CRSRCOUPLE   0x8cf

struct MacOSAbsPointerState {
    DeviceState parent_obj;

    QemuInputHandlerState *hs;
    Notifier mode_change;
    QEMUTimer *steer_timer;
    bool steer;
    bool step_pending;
    int step_x, step_y;         /* where the cursor was when it was sent */
    int64_t step_ns;
    hwaddr lm_phys;
    int x, y;
};

#define STEER_PERIOD_NS     (5 * SCALE_MS)
#define STEER_WAIT_NS       (100 * SCALE_MS)
#define STEER_MAX_STEP      32
#define STEER_EXACT         8

static void macos_abs_pointer_event(DeviceState *dev, QemuConsole *src,
                                    QemuInputEvent *evt)
{
    MacOSAbsPointerState *s = MACOS_ABS_POINTER(dev);
    QemuConsole *con = src ?: qemu_console_lookup_by_index(0);
    InputMoveEvent *move = &evt->abs;
    int width = 640, height = 480;

    if (con) {
        width = qemu_console_get_width(con, width);
        height = qemu_console_get_height(con, height);
    }

    if (move->axis == INPUT_AXIS_X) {
        s->x = qemu_input_scale_axis(move->value, INPUT_EVENT_ABS_MIN,
                                     INPUT_EVENT_ABS_MAX, 0, width - 1);
    } else if (move->axis == INPUT_AXIS_Y) {
        s->y = qemu_input_scale_axis(move->value, INPUT_EVENT_ABS_MIN,
                                     INPUT_EVENT_ABS_MAX, 0, height - 1);
    }
}

/*
 * The globals are at logical addresses. On the 68k Macs low memory is
 * mapped one to one, PowerPC Mac OS puts logical page 0 elsewhere. Asking
 * the MMU only helps while the CPU translates, not in the nanokernel's real
 * mode, so remember the page once a translation has shown it.
 */
static bool lm_base(MacOSAbsPointerState *s, hwaddr *base)
{
    TranslateForDebugResult t;

    if (!first_cpu) {
        return false;
    }
    if (cpu_translate_for_debug(first_cpu, 0, &t) && t.physaddr) {
        s->lm_phys = t.physaddr;
    }
    if (s->lm_phys) {
        *base = s->lm_phys;
        return true;
    }
    /* steering needs the real page, the 68k Macs use page 0 itself */
    *base = 0;
    return !s->steer;
}

static uint32_t lm_ld(hwaddr addr, int len)
{
    uint8_t buf[4] = { 0 };

    address_space_read(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                       buf, len);
    return len == 4 ? ldl_be_p(buf) : len == 2 ? lduw_be_p(buf) : buf[0];
}

static void lm_st(hwaddr addr, uint32_t val, int len)
{
    uint8_t buf[4];

    if (len == 2) {
        stw_be_p(buf, val);
    } else {
        buf[0] = val;
    }
    address_space_write(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                        buf, len);
}

/* close by Mac OS does not accelerate, further away go half the way */
static int steer_step(int d)
{
    if (abs(d) <= STEER_EXACT) {
        return d;
    }
    return MAX(-STEER_MAX_STEP, MIN(STEER_MAX_STEP, d / 2));
}

/* one step of the relative mouse towards the host position */
static void macos_abs_pointer_steer(void *opaque)
{
    MacOSAbsPointerState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    hwaddr lm;
    int mx, my;

    if (!lm_base(s, &lm) || !lm_ld(lm + LM_SCRNBASE, 4)) {
        return;
    }
    mx = (int16_t)lm_ld(lm + LM_MOUSE + 2, 2);
    my = (int16_t)lm_ld(lm + LM_MOUSE, 2);
    if (s->step_pending && mx == s->step_x && my == s->step_y &&
        now - s->step_ns < STEER_WAIT_NS) {
        /* the last step has not arrived yet */
        timer_mod(s->steer_timer, now + STEER_PERIOD_NS);
        return;
    }
    s->step_pending = false;
    if (mx == s->x && my == s->y) {
        return;
    }
    qemu_input_queue_rel(NULL, INPUT_AXIS_X, steer_step(s->x - mx));
    qemu_input_queue_rel(NULL, INPUT_AXIS_Y, steer_step(s->y - my));
    qemu_input_event_sync();
    s->step_pending = true;
    s->step_x = mx;
    s->step_y = my;
    s->step_ns = now;
    timer_mod(s->steer_timer, now + STEER_PERIOD_NS);
}

static void macos_abs_pointer_sync(DeviceState *dev)
{
    MacOSAbsPointerState *s = MACOS_ABS_POINTER(dev);
    hwaddr lm;

    if (s->steer) {
        /* not from here: the relative events would come back to us */
        timer_mod(s->steer_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        return;
    }
    if (!lm_base(s, &lm) || !lm_ld(lm + LM_SCRNBASE, 4)) {
        return;
    }
    lm_st(lm + LM_MTEMP, s->y, 2);
    lm_st(lm + LM_MTEMP + 2, s->x, 2);
    lm_st(lm + LM_RAWMOUSE, s->y, 2);
    lm_st(lm + LM_RAWMOUSE + 2, s->x, 2);
    lm_st(lm + LM_MOUSE, s->y, 2);
    lm_st(lm + LM_MOUSE + 2, s->x, 2);
    lm_st(lm + LM_CRSRNEW, lm_ld(lm + LM_CRSRCOUPLE, 1), 1);
}

static const QemuInputHandler macos_abs_pointer_handler = {
    .name  = "Mac OS absolute pointer",
    .mask  = INPUT_EVENT_MASK_ABS,
    .event = macos_abs_pointer_event,
    .sync  = macos_abs_pointer_sync,
};

/*
 * A mouse that the guest starts using, like USB HID, moves itself to the
 * front, and the UI goes back to relative events. Take the front again.
 */
static void macos_abs_pointer_mode_change(Notifier *n, void *data)
{
    MacOSAbsPointerState *s = container_of(n, MacOSAbsPointerState,
                                           mode_change);

    if (!qemu_input_is_absolute(NULL)) {
        qemu_input_handler_activate(s->hs);
    }
}

static void macos_abs_pointer_realize(DeviceState *dev, Error **errp)
{
    MacOSAbsPointerState *s = MACOS_ABS_POINTER(dev);

    s->steer_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, macos_abs_pointer_steer,
                                  s);
    s->hs = qemu_input_handler_register(dev, &macos_abs_pointer_handler);
    /* ahead of the relative mice, so that the UI sends absolute positions */
    qemu_input_handler_activate(s->hs);
    s->mode_change.notify = macos_abs_pointer_mode_change;
    qemu_add_mouse_mode_change_notifier(&s->mode_change);
}

static void macos_abs_pointer_unrealize(DeviceState *dev)
{
    MacOSAbsPointerState *s = MACOS_ABS_POINTER(dev);

    qemu_remove_mouse_mode_change_notifier(&s->mode_change);
    qemu_input_handler_unregister(s->hs);
    timer_free(s->steer_timer);
}

static const Property macos_abs_pointer_properties[] = {
    DEFINE_PROP_BOOL("steer", MacOSAbsPointerState, steer, false),
};

static void macos_abs_pointer_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = macos_abs_pointer_realize;
    dc->unrealize = macos_abs_pointer_unrealize;
    dc->desc = "absolute pointer for Mac OS 7 to 9 (writes its low memory)";
    device_class_set_props(dc, macos_abs_pointer_properties);
    set_bit(DEVICE_CATEGORY_INPUT, dc->categories);
}

static const TypeInfo macos_abs_pointer_types[] = {
    {
        .name = TYPE_MACOS_ABS_POINTER,
        .parent = TYPE_DEVICE,
        .instance_size = sizeof(MacOSAbsPointerState),
        .class_init = macos_abs_pointer_class_init,
    },
};

DEFINE_TYPES(macos_abs_pointer_types)
