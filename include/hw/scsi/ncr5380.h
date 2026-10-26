/*
 * NCR 5380 SCSI controller
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_SCSI_NCR5380_H
#define HW_SCSI_NCR5380_H

#include "hw/core/sysbus.h"
#include "hw/scsi/scsi.h"
#include "qom/object.h"

#define TYPE_NCR5380 "ncr5380"
OBJECT_DECLARE_SIMPLE_TYPE(NCR5380State, NCR5380)

struct NCR5380State {
    SysBusDevice parent_obj;

    MemoryRegion regs;          /* the eight registers, reg << it_shift */
    MemoryRegion pdma;          /* pseudo DMA: every access is DACK */
    qemu_irq irq;
    qemu_irq drq;
    uint32_t it_shift;

    SCSIBus bus;

    /* chip registers */
    uint8_t odr;                /* output data */
    uint8_t idr;                /* input data, latched in DMA receive */
    uint8_t icr;
    uint8_t mr;
    uint8_t tcr;
    uint8_t ser;                /* select enable */
    bool aip;                   /* arbitration in progress */
    uint8_t arb_id;             /* our ID, from the data bus at arbitration */
    bool irq_flag;
    bool busy_error;
    bool dma_send;
    bool dma_recv;
    bool dma_ack;               /* ACK asserted by the DMA logic */
    bool drq_flag;
    bool bus_ack;               /* ACK as the target sees it */

    /* the selected target */
    int target;                 /* -1: bus free */
    int lun;
    bool req;
    uint8_t phase;              /* SR_MSG | SR_CD | SR_IO lines */
    uint8_t data;               /* byte the target drives in an IN phase */
    int tstate;
    uint8_t cdb[16];
    int cdb_len;
    int cdb_pos;
    SCSIRequest *sreq;
    uint8_t *buf;
    uint32_t buf_len;
    uint32_t buf_pos;
    bool data_in;
    uint8_t status;
};

#endif
