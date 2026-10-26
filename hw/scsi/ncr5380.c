/*
 * NCR 5380 SCSI controller
 *
 * The 5380 is a bus level chip: software drives the SCSI signals through
 * the Initiator Command Register and watches the bus in the Current SCSI
 * Bus Status and Bus and Status registers. This model therefore also plays
 * the targets' side of the bus: it answers selection, runs the information
 * transfer phases with REQ/ACK and passes the commands to QEMU's SCSI
 * devices. Disconnection and reselection are not modelled, a target stays
 * connected until COMMAND COMPLETE.
 *
 * In DMA mode the chip does the REQ/ACK handshake itself. The Macintosh
 * uses "pseudo DMA": the CPU reads or writes a separate address range that
 * asserts DACK, and the access ends in a bus error when no byte is ready.
 *
 * Register behaviour follows the NCR 5380 data sheet and what the Linux
 * driver (drivers/scsi/NCR5380.c, mac_scsi.c) relies on.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/scsi/ncr5380.h"
#include "scsi/constants.h"
#include "scsi/utils.h"

/* registers */
#define REG_DATA        0       /* r: current SCSI data, w: output data */
#define REG_ICR         1
#define REG_MR          2
#define REG_TCR         3
#define REG_STATUS      4       /* r: bus status, w: select enable */
#define REG_BASR        5       /* r: bus and status, w: start DMA send */
#define REG_IDR         6       /* r: input data, w: start DMA target recv */
#define REG_RPI         7       /* r: reset parity/irq, w: start DMA init recv */

#define ICR_RST         0x80
#define ICR_AIP         0x40
#define ICR_LA          0x20
#define ICR_ACK         0x10
#define ICR_BSY         0x08
#define ICR_SEL         0x04
#define ICR_ATN         0x02
#define ICR_DATA        0x01
#define ICR_RW_MASK     (ICR_RST | ICR_ACK | ICR_BSY | ICR_SEL | ICR_ATN | \
                         ICR_DATA)

#define MR_TARGET       0x40
#define MR_MONITOR_BSY  0x04
#define MR_DMA          0x02
#define MR_ARBITRATE    0x01

#define SR_RST          0x80
#define SR_BSY          0x40
#define SR_REQ          0x20
#define SR_MSG          0x10
#define SR_CD           0x08
#define SR_IO           0x04
#define SR_SEL          0x02

#define BASR_DRQ        0x40
#define BASR_IRQ        0x10
#define BASR_PHASE      0x08
#define BASR_BUSY_ERR   0x04
#define BASR_ATN        0x02
#define BASR_ACK        0x01

#define PHASE_DATAOUT   0
#define PHASE_DATAIN    SR_IO
#define PHASE_CMD       SR_CD
#define PHASE_STATUS    (SR_CD | SR_IO)
#define PHASE_MSGOUT    (SR_MSG | SR_CD)
#define PHASE_MSGIN     (SR_MSG | SR_CD | SR_IO)

/* target states */
enum {
    T_FREE,
    T_SELECTED,         /* BSY answered, waiting for SEL to go away */
    T_MSGOUT,
    T_CMD,
    T_WAIT,             /* the SCSI layer has the request, no REQ */
    T_DATAIN,
    T_DATAOUT,
    T_STATUS,
    T_MSGIN,
};

static bool ncr_target_drives_data(NCR5380State *s)
{
    return s->target >= 0 && (s->phase & SR_IO) &&
           (s->tstate == T_DATAIN || s->tstate == T_STATUS ||
            s->tstate == T_MSGIN);
}

static uint8_t ncr_bus_data(NCR5380State *s)
{
    if (ncr_target_drives_data(s)) {
        return s->data;
    }
    if ((s->icr & ICR_DATA) || (s->mr & MR_ARBITRATE)) {
        return s->odr;
    }
    return 0;
}

static bool ncr_phase_match(NCR5380State *s)
{
    uint8_t phase = s->target >= 0 ? s->phase : 0;

    return (s->tcr & 7) == (phase >> 2);
}

static void ncr_bus_free(NCR5380State *s)
{
    if (s->sreq) {
        SCSIRequest *req = s->sreq;

        s->sreq = NULL;
        scsi_req_cancel(req);
    }
    if (s->target >= 0 && (s->mr & MR_MONITOR_BSY)) {
        s->busy_error = true;
        s->irq_flag = true;
        s->mr &= ~MR_DMA;
        s->dma_send = s->dma_recv = false;
    }
    s->target = -1;
    s->tstate = T_FREE;
    s->phase = 0;
    s->req = false;
}

static void ncr_start_command(NCR5380State *s)
{
    SCSIDevice *dev = scsi_device_find(&s->bus, 0, s->target, s->lun);
    int32_t len;

    s->tstate = T_WAIT;
    if (!dev) {
        s->status = CHECK_CONDITION;
        s->tstate = T_STATUS;
        s->phase = PHASE_STATUS;
        s->data = s->status;
        s->req = true;
        return;
    }

    s->sreq = scsi_req_new(dev, 0, s->lun, s->cdb, s->cdb_len, s);
    len = scsi_req_enqueue(s->sreq);
    if (len) {
        s->data_in = len > 0;
        if (s->tstate == T_WAIT) {
            s->phase = s->data_in ? PHASE_DATAIN : PHASE_DATAOUT;
        }
        if (s->sreq) {
            scsi_req_continue(s->sreq);
        }
    }
}

/* the initiator raised ACK: the target takes or lets go of the byte */
static void ncr_ack_rise(NCR5380State *s)
{
    if (!s->req) {
        return;
    }
    switch (s->tstate) {
    case T_MSGOUT:
        s->data = ncr_bus_data(s);
        break;
    case T_CMD:
        if (s->cdb_pos < sizeof(s->cdb)) {
            s->cdb[s->cdb_pos] = ncr_bus_data(s);
        }
        s->cdb_pos++;
        break;
    case T_DATAOUT:
        s->buf[s->buf_pos++] = ncr_bus_data(s);
        break;
    case T_DATAIN:
        s->buf_pos++;
        break;
    default:
        break;
    }
    s->req = false;
}

/* the initiator dropped ACK: the target moves on */
static void ncr_ack_fall(NCR5380State *s)
{
    bool atn = s->icr & ICR_ATN;

    switch (s->tstate) {
    case T_MSGOUT:
        if (s->data & 0x80) {           /* IDENTIFY */
            s->lun = s->data & 7;
        } else if (s->data == 0x06 || s->data == 0x0c) {
            /* ABORT, BUS DEVICE RESET */
            ncr_bus_free(s);
            return;
        }
        if (atn) {
            s->req = true;
        } else {
            s->tstate = T_CMD;
            s->phase = PHASE_CMD;
            s->cdb_pos = 0;
            s->cdb_len = 0;
            s->req = true;
        }
        break;
    case T_CMD:
        if (s->cdb_pos == 1) {
            s->cdb_len = scsi_cdb_length(s->cdb);
            if (s->cdb_len <= 0 || s->cdb_len > sizeof(s->cdb)) {
                s->cdb_len = 6;
            }
        }
        if (s->cdb_pos < s->cdb_len) {
            s->req = true;
        } else {
            ncr_start_command(s);
        }
        break;
    case T_DATAIN:
        if (s->buf_pos < s->buf_len) {
            s->data = s->buf[s->buf_pos];
            s->req = true;
        } else {
            s->tstate = T_WAIT;
            scsi_req_continue(s->sreq);
        }
        break;
    case T_DATAOUT:
        if (s->buf_pos < s->buf_len) {
            s->req = true;
        } else {
            /*
             * Show the coming STATUS phase right away, so that a DMA send
             * ends with a phase mismatch rather than a stalled DATA OUT.
             */
            s->tstate = T_WAIT;
            s->phase = PHASE_STATUS;
            scsi_req_continue(s->sreq);
        }
        break;
    case T_STATUS:
        s->tstate = T_MSGIN;
        s->phase = PHASE_MSGIN;
        s->data = 0x00;                 /* COMMAND COMPLETE */
        s->req = true;
        break;
    case T_MSGIN:
        if (atn) {
            /* the initiator rejects the message: take it */
            s->tstate = T_MSGOUT;
            s->phase = PHASE_MSGOUT;
            s->req = true;
        } else {
            ncr_bus_free(s);
        }
        break;
    default:
        break;
    }
}

/* run the handshakes until the bus is stable, then drive the outputs */
static void ncr_sync(NCR5380State *s)
{
    bool changed;

    do {
        bool ack = (s->icr & ICR_ACK) || s->dma_ack;

        changed = false;
        if (ack != s->bus_ack) {
            s->bus_ack = ack;
            if (ack) {
                ncr_ack_rise(s);
            } else {
                ncr_ack_fall(s);
            }
            changed = true;
        }

        if (s->mr & MR_DMA) {
            if (s->req && !ncr_phase_match(s)) {
                s->irq_flag = true;
            }
            if (s->dma_recv && s->req && ncr_phase_match(s) && !s->dma_ack) {
                /* latch the byte and acknowledge it */
                s->idr = ncr_bus_data(s);
                s->dma_ack = true;
                changed = true;
            }
        } else if (s->dma_ack) {
            s->dma_ack = false;
            changed = true;
        }
    } while (changed);

    if (s->mr & MR_DMA) {
        if (s->dma_recv) {
            s->drq_flag = s->dma_ack;
        } else if (s->dma_send) {
            s->drq_flag = s->req && ncr_phase_match(s) && !s->dma_ack;
        } else {
            s->drq_flag = false;
        }
    } else {
        s->drq_flag = false;
    }

    qemu_set_irq(s->irq, s->irq_flag);
    qemu_set_irq(s->drq, s->drq_flag);
}

static void ncr_check_selection(NCR5380State *s)
{
    uint8_t ids;
    int id;

    if (s->target >= 0 || !(s->icr & ICR_SEL) || (s->icr & ICR_BSY) ||
        !(s->icr & ICR_DATA)) {
        return;
    }
    ids = s->odr & ~s->arb_id;
    if (!ids || (ids & (ids - 1))) {
        return;
    }
    id = ctz32(ids);
    if (!scsi_device_find(&s->bus, 0, id, 0)) {
        return;
    }
    s->target = id;
    s->lun = 0;
    s->tstate = T_SELECTED;
    s->phase = 0;
    s->req = false;
}

static void ncr_bus_reset(NCR5380State *s)
{
    s->busy_error = false;
    s->mr &= ~MR_MONITOR_BSY;
    ncr_bus_free(s);
    s->mr = 0;
    s->tcr = 0;
    s->aip = false;
    s->dma_send = s->dma_recv = s->dma_ack = false;
    s->irq_flag = true;
    bus_cold_reset(BUS(&s->bus));
}

static uint64_t ncr_reg_read(void *opaque, hwaddr addr, unsigned size)
{
    NCR5380State *s = opaque;
    int reg = (addr >> s->it_shift) & 7;
    uint8_t val = 0;

    switch (reg) {
    case REG_DATA:
        val = ncr_bus_data(s);
        break;
    case REG_ICR:
        val = (s->icr & ICR_RW_MASK) | (s->aip ? ICR_AIP : 0);
        break;
    case REG_MR:
        val = s->mr;
        break;
    case REG_TCR:
        val = s->tcr;
        break;
    case REG_STATUS:
        if (s->icr & ICR_RST) {
            val |= SR_RST;
        }
        if ((s->icr & ICR_BSY) || s->target >= 0) {
            val |= SR_BSY;
        }
        if (s->req) {
            val |= SR_REQ;
        }
        if (s->target >= 0) {
            val |= s->phase;
        }
        if (s->icr & ICR_SEL) {
            val |= SR_SEL;
        }
        break;
    case REG_BASR:
        if (s->drq_flag) {
            val |= BASR_DRQ;
        }
        if (s->irq_flag) {
            val |= BASR_IRQ;
        }
        if (ncr_phase_match(s)) {
            val |= BASR_PHASE;
        }
        if (s->busy_error) {
            val |= BASR_BUSY_ERR;
        }
        if (s->icr & ICR_ATN) {
            val |= BASR_ATN;
        }
        if (s->bus_ack) {
            val |= BASR_ACK;
        }
        break;
    case REG_IDR:
        val = s->idr;
        break;
    case REG_RPI:
        s->irq_flag = false;
        s->busy_error = false;
        ncr_sync(s);
        break;
    }
    return val;
}

static void ncr_reg_write(void *opaque, hwaddr addr, uint64_t val,
                          unsigned size)
{
    NCR5380State *s = opaque;
    int reg = (addr >> s->it_shift) & 7;
    uint8_t old;

    switch (reg) {
    case REG_DATA:
        s->odr = val;
        ncr_check_selection(s);
        break;
    case REG_ICR:
        old = s->icr;
        s->icr = val & ICR_RW_MASK;
        if ((s->icr & ICR_RST) && !(old & ICR_RST)) {
            ncr_bus_reset(s);
            break;
        }
        ncr_check_selection(s);
        if (s->tstate == T_SELECTED && !(s->icr & ICR_SEL)) {
            if (s->icr & ICR_ATN) {
                s->tstate = T_MSGOUT;
                s->phase = PHASE_MSGOUT;
            } else {
                s->tstate = T_CMD;
                s->phase = PHASE_CMD;
                s->cdb_pos = 0;
                s->cdb_len = 0;
            }
            s->req = true;
        }
        break;
    case REG_MR:
        old = s->mr;
        s->mr = val;
        if (!(s->mr & MR_ARBITRATE)) {
            s->aip = false;
        } else if (!(old & MR_ARBITRATE)) {
            s->arb_id = s->odr;
            s->aip = s->target < 0;
        }
        if (!(s->mr & MR_DMA) && (old & MR_DMA)) {
            /* ending DMA mode clears end of DMA and phase mismatch */
            s->dma_send = s->dma_recv = false;
            s->irq_flag = false;
        }
        break;
    case REG_TCR:
        s->tcr = val & 0x0f;
        break;
    case REG_STATUS:
        s->ser = val;
        break;
    case REG_BASR:
        if (s->mr & MR_DMA) {
            s->dma_send = true;
            s->dma_recv = false;
        }
        break;
    case REG_IDR:
        /* target mode DMA is not modelled */
        break;
    case REG_RPI:
        if (s->mr & MR_DMA) {
            s->dma_recv = true;
            s->dma_send = false;
        }
        break;
    }
    ncr_sync(s);
}

static const MemoryRegionOps ncr_reg_ops = {
    .read = ncr_reg_read,
    .write = ncr_reg_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

/* pseudo DMA: each byte is one DACK cycle, a missing byte is a bus error */
static MemTxResult ncr_pdma_read(void *opaque, hwaddr addr, uint64_t *data,
                                 unsigned size, MemTxAttrs attrs)
{
    NCR5380State *s = opaque;
    uint64_t val = 0;
    unsigned i;

    for (i = 0; i < size; i++) {
        if (!s->dma_recv || !s->drq_flag) {
            return MEMTX_ERROR;
        }
        val = (val << 8) | s->idr;
        s->dma_ack = false;
        ncr_sync(s);
    }
    *data = val;
    return MEMTX_OK;
}

static MemTxResult ncr_pdma_write(void *opaque, hwaddr addr, uint64_t val,
                                  unsigned size, MemTxAttrs attrs)
{
    NCR5380State *s = opaque;
    unsigned i;

    for (i = 0; i < size; i++) {
        if (!s->dma_send || !s->drq_flag) {
            return MEMTX_ERROR;
        }
        s->odr = val >> (8 * (size - 1 - i));
        s->dma_ack = true;
        ncr_sync(s);
        s->dma_ack = false;
        ncr_sync(s);
    }
    return MEMTX_OK;
}

static const MemoryRegionOps ncr_pdma_ops = {
    .read_with_attrs = ncr_pdma_read,
    .write_with_attrs = ncr_pdma_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void ncr_transfer_data(SCSIRequest *req, uint32_t len)
{
    NCR5380State *s = req->hba_private;

    s->buf = scsi_req_get_buf(req);
    s->buf_len = len;
    s->buf_pos = 0;
    if (s->data_in) {
        s->tstate = T_DATAIN;
        s->phase = PHASE_DATAIN;
        s->data = s->buf[0];
    } else {
        s->tstate = T_DATAOUT;
        s->phase = PHASE_DATAOUT;
    }
    s->req = len > 0;
    ncr_sync(s);
}

static void ncr_command_complete(SCSIRequest *req, size_t resid)
{
    NCR5380State *s = req->hba_private;

    if (req != s->sreq) {
        return;
    }
    s->status = req->status;
    s->sreq = NULL;
    scsi_req_unref(req);
    s->tstate = T_STATUS;
    s->phase = PHASE_STATUS;
    s->data = s->status;
    s->req = true;
    ncr_sync(s);
}

static void ncr_request_cancelled(SCSIRequest *req)
{
    NCR5380State *s = req->hba_private;

    if (req == s->sreq) {
        s->sreq = NULL;
    }
    scsi_req_unref(req);
}

static const struct SCSIBusInfo ncr_scsi_info = {
    .tcq = false,
    .max_target = 7,
    .max_lun = 7,

    .transfer_data = ncr_transfer_data,
    .complete = ncr_command_complete,
    .cancel = ncr_request_cancelled,
};

static void ncr5380_reset_hold(Object *obj, ResetType type)
{
    NCR5380State *s = NCR5380(obj);

    ncr_bus_free(s);
    s->odr = s->idr = s->icr = s->mr = s->tcr = s->ser = 0;
    s->aip = false;
    s->irq_flag = s->busy_error = false;
    s->dma_send = s->dma_recv = s->dma_ack = s->drq_flag = false;
    s->bus_ack = false;
}

static void ncr5380_realize(DeviceState *dev, Error **errp)
{
    NCR5380State *s = NCR5380(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->regs, OBJECT(s), &ncr_reg_ops, s,
                          "ncr5380-regs", 8 << s->it_shift);
    sysbus_init_mmio(sbd, &s->regs);
    memory_region_init_io(&s->pdma, OBJECT(s), &ncr_pdma_ops, s,
                          "ncr5380-pdma", 0x2000);
    sysbus_init_mmio(sbd, &s->pdma);
    sysbus_init_irq(sbd, &s->irq);
    sysbus_init_irq(sbd, &s->drq);

    s->target = -1;
    scsi_bus_init(&s->bus, sizeof(s->bus), dev, &ncr_scsi_info);
}

static const Property ncr5380_properties[] = {
    DEFINE_PROP_UINT32("it_shift", NCR5380State, it_shift, 4),
};

static void ncr5380_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = ncr5380_realize;
    rc->phases.hold = ncr5380_reset_hold;
    device_class_set_props(dc, ncr5380_properties);
    set_bit(DEVICE_CATEGORY_STORAGE, dc->categories);
    dc->user_creatable = false;
}

static const TypeInfo ncr5380_types[] = {
    {
        .name = TYPE_NCR5380,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(NCR5380State),
        .class_init = ncr5380_class_init,
    },
};

DEFINE_TYPES(ncr5380_types)
