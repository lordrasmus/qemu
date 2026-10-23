/*
 * Atmel USART with peripheral DMA controller (PDC), as on the AT32AP7000
 *
 * Only what the Linux atmel_serial driver needs: polled and PDC transfers
 * in both directions and the receive timeout that flushes a partly filled
 * PDC buffer. Line settings are accepted and ignored.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "hw/avr32/ap7000.h"

#define US_CR       0x00
#define US_MR       0x04
#define US_IER      0x08
#define US_IDR      0x0c
#define US_IMR      0x10
#define US_CSR      0x14
#define US_RHR      0x18
#define US_THR      0x1c
#define US_BRGR     0x20
#define US_RTOR     0x24
#define US_TTGR     0x28
#define US_VERSION  0xfc
#define PDC_RPR     0x100
#define PDC_RCR     0x104
#define PDC_TPR     0x108
#define PDC_TCR     0x10c
#define PDC_RNPR    0x110
#define PDC_RNCR    0x114
#define PDC_TNPR    0x118
#define PDC_TNCR    0x11c
#define PDC_PTCR    0x120
#define PDC_PTSR    0x124

#define CR_RSTRX    (1 << 2)
#define CR_RSTTX    (1 << 3)
#define CR_RXEN     (1 << 4)
#define CR_RXDIS    (1 << 5)
#define CR_TXEN     (1 << 6)
#define CR_TXDIS    (1 << 7)
#define CR_RSTSTA   (1 << 8)
#define CR_STTTO    (1 << 11)
#define CR_RETTO    (1 << 15)

#define CSR_RXRDY   (1 << 0)
#define CSR_TXRDY   (1 << 1)
#define CSR_ENDRX   (1 << 3)
#define CSR_ENDTX   (1 << 4)
#define CSR_OVRE    (1 << 5)
#define CSR_TIMEOUT (1 << 8)
#define CSR_TXEMPTY (1 << 9)
#define CSR_TXBUFE  (1 << 11)
#define CSR_RXBUFF  (1 << 12)

#define PTCR_RXTEN  (1 << 0)
#define PTCR_RXTDIS (1 << 1)
#define PTCR_TXTEN  (1 << 8)
#define PTCR_TXTDIS (1 << 9)

/* time without new characters before TIMEOUT is flagged */
#define RX_TIMEOUT_NS   (1 * SCALE_MS)

static uint32_t usart_status(AtmelUsartState *s)
{
    uint32_t csr = s->csr | CSR_TXRDY | CSR_TXEMPTY;

    /* the PDC end flags follow the counters */
    if (!s->rcr) {
        csr |= CSR_ENDRX;
        if (!s->rncr) {
            csr |= CSR_RXBUFF;
        }
    }
    if (!s->tcr) {
        csr |= CSR_ENDTX;
        if (!s->tncr) {
            csr |= CSR_TXBUFE;
        }
    }
    return csr;
}

static void usart_update_irq(AtmelUsartState *s)
{
    qemu_set_irq(s->irq, !!(usart_status(s) & s->imr));
}

/* PDC transmit: the whole buffer goes out at once */
static void usart_pdc_tx(AtmelUsartState *s)
{
    uint8_t buf[256];

    if (!s->tx_enabled || !(s->ptsr & PTCR_TXTEN)) {
        return;
    }
    while (s->tcr) {
        uint32_t n = MIN(s->tcr, sizeof(buf));

        dma_memory_read(&address_space_memory, s->tpr, buf, n,
                        MEMTXATTRS_UNSPECIFIED);
        qemu_chr_fe_write_all(&s->chr, buf, n);
        s->tpr += n;
        s->tcr -= n;
        if (!s->tcr && s->tncr) {
            s->tpr = s->tnpr;
            s->tcr = s->tncr;
            s->tncr = 0;
        }
    }
    usart_update_irq(s);
}

static bool usart_pdc_rx(AtmelUsartState *s)
{
    return s->ptsr & PTCR_RXTEN;
}

static int usart_can_receive(void *opaque)
{
    AtmelUsartState *s = opaque;

    if (!s->rx_enabled) {
        return 0;
    }
    if (usart_pdc_rx(s)) {
        return s->rcr ? s->rcr : s->rncr;
    }
    return !(s->csr & CSR_RXRDY);
}

static void usart_receive(void *opaque, const uint8_t *buf, int size)
{
    AtmelUsartState *s = opaque;

    if (!usart_pdc_rx(s)) {
        if (s->csr & CSR_RXRDY) {
            s->csr |= CSR_OVRE;
        }
        s->rhr = buf[0];
        s->csr |= CSR_RXRDY;
        usart_update_irq(s);
        return;
    }

    while (size > 0) {
        uint32_t n;

        if (!s->rcr) {
            if (!s->rncr) {
                s->csr |= CSR_OVRE;
                break;
            }
            s->rpr = s->rnpr;
            s->rcr = s->rncr;
            s->rncr = 0;
        }
        n = MIN(s->rcr, size);
        dma_memory_write(&address_space_memory, s->rpr, buf, n,
                         MEMTXATTRS_UNSPECIFIED);
        s->rpr += n;
        s->rcr -= n;
        buf += n;
        size -= n;
    }
    if (s->timeout_armed && s->rtor) {
        timer_mod(s->rx_timeout,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + RX_TIMEOUT_NS);
    }
    usart_update_irq(s);
}

static void usart_rx_timeout(void *opaque)
{
    AtmelUsartState *s = opaque;

    s->timeout_armed = false;
    s->csr |= CSR_TIMEOUT;
    usart_update_irq(s);
}

static void usart_cr(AtmelUsartState *s, uint32_t cr)
{
    if (cr & CR_RSTRX) {
        s->rx_enabled = false;
        s->csr &= ~CSR_RXRDY;
    }
    if (cr & CR_RSTTX) {
        s->tx_enabled = false;
    }
    if (cr & CR_RXEN) {
        s->rx_enabled = true;
    }
    if (cr & CR_RXDIS) {
        s->rx_enabled = false;
    }
    if (cr & CR_TXEN) {
        s->tx_enabled = true;
    }
    if (cr & CR_TXDIS) {
        s->tx_enabled = false;
    }
    if (cr & CR_RSTSTA) {
        s->csr &= ~CSR_OVRE;
    }
    if (cr & CR_STTTO) {
        /* the timeout starts with the next received character */
        s->csr &= ~CSR_TIMEOUT;
        s->timeout_armed = true;
        timer_del(s->rx_timeout);
    }
    if ((cr & CR_RETTO) && s->rtor) {
        s->csr &= ~CSR_TIMEOUT;
        s->timeout_armed = true;
        timer_mod(s->rx_timeout,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + RX_TIMEOUT_NS);
    }
    if (s->rx_enabled) {
        qemu_chr_fe_accept_input(&s->chr);
    }
    usart_pdc_tx(s);
}

static uint64_t usart_read(void *opaque, hwaddr offset, unsigned size)
{
    AtmelUsartState *s = opaque;
    uint32_t val;

    switch (offset) {
    case US_MR:
        return s->mr;
    case US_IMR:
        return s->imr;
    case US_CSR:
        return usart_status(s);
    case US_RHR:
        val = s->rhr;
        s->csr &= ~CSR_RXRDY;
        usart_update_irq(s);
        qemu_chr_fe_accept_input(&s->chr);
        return val;
    case US_BRGR:
        return s->brgr;
    case US_RTOR:
        return s->rtor;
    case US_TTGR:
        return s->ttgr;
    case US_VERSION:
        return 0x302;                   /* a USART, not a DBGU */
    case PDC_RPR:
        return s->rpr;
    case PDC_RCR:
        return s->rcr;
    case PDC_TPR:
        return s->tpr;
    case PDC_TCR:
        return s->tcr;
    case PDC_RNPR:
        return s->rnpr;
    case PDC_RNCR:
        return s->rncr;
    case PDC_TNPR:
        return s->tnpr;
    case PDC_TNCR:
        return s->tncr;
    case PDC_PTSR:
        return s->ptsr;
    default:
        return 0;
    }
}

static void usart_write(void *opaque, hwaddr offset, uint64_t value,
                        unsigned size)
{
    AtmelUsartState *s = opaque;
    uint8_t ch = value;

    switch (offset) {
    case US_CR:
        usart_cr(s, value);
        break;
    case US_MR:
        s->mr = value;
        break;
    case US_IER:
        s->imr |= value;
        break;
    case US_IDR:
        s->imr &= ~value;
        break;
    case US_THR:
        if (s->tx_enabled) {
            qemu_chr_fe_write_all(&s->chr, &ch, 1);
        }
        break;
    case US_BRGR:
        s->brgr = value;
        break;
    case US_RTOR:
        s->rtor = value;
        break;
    case US_TTGR:
        s->ttgr = value;
        break;
    case PDC_RPR:
        s->rpr = value;
        break;
    case PDC_RCR:
        s->rcr = value & 0xffff;
        qemu_chr_fe_accept_input(&s->chr);
        break;
    case PDC_TPR:
        s->tpr = value;
        break;
    case PDC_TCR:
        s->tcr = value & 0xffff;
        usart_pdc_tx(s);
        break;
    case PDC_RNPR:
        s->rnpr = value;
        break;
    case PDC_RNCR:
        s->rncr = value & 0xffff;
        qemu_chr_fe_accept_input(&s->chr);
        break;
    case PDC_TNPR:
        s->tnpr = value;
        break;
    case PDC_TNCR:
        s->tncr = value & 0xffff;
        usart_pdc_tx(s);
        break;
    case PDC_PTCR:
        if (value & PTCR_RXTEN) {
            s->ptsr |= PTCR_RXTEN;
        }
        if (value & PTCR_RXTDIS) {
            s->ptsr &= ~PTCR_RXTEN;
        }
        if (value & PTCR_TXTEN) {
            s->ptsr |= PTCR_TXTEN;
        }
        if (value & PTCR_TXTDIS) {
            s->ptsr &= ~PTCR_TXTEN;
        }
        qemu_chr_fe_accept_input(&s->chr);
        usart_pdc_tx(s);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "atmel_usart: write 0x%" PRIx64 " at 0x%"
                      HWADDR_PRIx "\n", value, offset);
        break;
    }
    usart_update_irq(s);
}

static const MemoryRegionOps usart_ops = {
    .read = usart_read,
    .write = usart_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void usart_reset(DeviceState *dev)
{
    AtmelUsartState *s = ATMEL_USART(dev);

    s->mr = s->imr = s->csr = s->rhr = s->brgr = s->rtor = s->ttgr = 0;
    s->rx_enabled = s->tx_enabled = s->timeout_armed = false;
    s->rpr = s->rcr = s->rnpr = s->rncr = 0;
    s->tpr = s->tcr = s->tnpr = s->tncr = 0;
    s->ptsr = 0;
    timer_del(s->rx_timeout);
}

static void usart_realize(DeviceState *dev, Error **errp)
{
    AtmelUsartState *s = ATMEL_USART(dev);

    memory_region_init_io(&s->mmio, OBJECT(s), &usart_ops, s, "atmel-usart",
                          0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->mmio);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    s->rx_timeout = timer_new_ns(QEMU_CLOCK_VIRTUAL, usart_rx_timeout, s);
    qemu_chr_fe_set_handlers(&s->chr, usart_can_receive, usart_receive,
                             NULL, NULL, s, NULL, true);
}

static const VMStateDescription vmstate_usart = {
    .name = "atmel-usart",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(mr, AtmelUsartState),
        VMSTATE_UINT32(imr, AtmelUsartState),
        VMSTATE_UINT32(csr, AtmelUsartState),
        VMSTATE_UINT32(rhr, AtmelUsartState),
        VMSTATE_UINT32(brgr, AtmelUsartState),
        VMSTATE_UINT32(rtor, AtmelUsartState),
        VMSTATE_UINT32(ttgr, AtmelUsartState),
        VMSTATE_BOOL(rx_enabled, AtmelUsartState),
        VMSTATE_BOOL(tx_enabled, AtmelUsartState),
        VMSTATE_BOOL(timeout_armed, AtmelUsartState),
        VMSTATE_UINT32(rpr, AtmelUsartState),
        VMSTATE_UINT32(rcr, AtmelUsartState),
        VMSTATE_UINT32(rnpr, AtmelUsartState),
        VMSTATE_UINT32(rncr, AtmelUsartState),
        VMSTATE_UINT32(tpr, AtmelUsartState),
        VMSTATE_UINT32(tcr, AtmelUsartState),
        VMSTATE_UINT32(tnpr, AtmelUsartState),
        VMSTATE_UINT32(tncr, AtmelUsartState),
        VMSTATE_UINT32(ptsr, AtmelUsartState),
        VMSTATE_TIMER_PTR(rx_timeout, AtmelUsartState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property usart_properties[] = {
    DEFINE_PROP_CHR("chardev", AtmelUsartState, chr),
};

static void usart_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = usart_realize;
    device_class_set_legacy_reset(dc, usart_reset);
    dc->vmsd = &vmstate_usart;
    device_class_set_props(dc, usart_properties);
}

static const TypeInfo usart_info = {
    .name = TYPE_ATMEL_USART,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AtmelUsartState),
    .class_init = usart_class_init,
};

static void usart_register_types(void)
{
    type_register_static(&usart_info);
}

type_init(usart_register_types)
