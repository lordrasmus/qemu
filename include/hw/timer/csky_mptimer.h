#ifndef CSKY_MPTIMER_H
#define CSKY_MPTIMER_H

#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "qemu/timer.h"
#include "system/system.h"
#include "qemu/cutils.h"
#include "qemu/log.h"
#include "hw/core/ptimer.h"
#include "cpu.h"
#include "hw/csky/cskydev.h"

#define CSKY_MPTIMER_CTLR_ENABLE         (1 << 0)
#define CSKY_MPTIMER_CTLR_DIVISION       (0xff << 8)
#define CSKY_MPTIMER_PTIM_CTLR_ENABLE    (1 << 0)
#define CSKY_MPTIMER_PTIM_CTLR_IE_MASK        (1 << 1)

#define CSKY_MPTIMER_MAX_CPUS 8
#define TYPE_CSKY_MPTIMER "csky_mptimer"

#define CSKY_MPTIMER(obj) \
    OBJECT_CHECK(csky_mptimer_state, (obj), TYPE_CSKY_MPTIMER)


#define QEMU_VIRT_CLOCK_FREQ 1000000000ll
/* State of a single timer. Fields in this struct
 * accessed by coprocessor */
typedef struct csky_mptimer_unit {
    uint32_t ctlr;
    uint32_t isr;
    uint32_t int_level;
    uint64_t cmpr;
    struct ptimer_state *timer;
    qemu_irq irq;
} csky_mptimer_unit;

typedef struct csky_mptimer_state {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/
    uint32_t ctlr;
    uint64_t icvr;
    uint32_t freq;
    MemoryRegion iomem;
    csky_mptimer_unit mptimer_unit[CSKY_MPTIMER_MAX_CPUS];
} csky_mptimer_state;
void csky_mptimer_set_freq(uint32_t freq);
uint32_t csky_mptimer_get(csky_mptimer_state *s, int index, int pos);
void csky_mptimer_set(csky_mptimer_state *s, int index, int pos,
                      uint32_t value);
#endif
