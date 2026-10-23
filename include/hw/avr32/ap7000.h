/*
 * AT32AP7000 on-chip peripherals
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_AVR32_AP7000_H
#define HW_AVR32_AP7000_H

#include "hw/core/sysbus.h"
#include "chardev/char-fe.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define AP7000_INTC_GROUPS  64

#define TYPE_AP7000_INTC "ap7000-intc"
OBJECT_DECLARE_SIMPLE_TYPE(AP7000IntcState, AP7000_INTC)

struct AP7000IntcState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    CPUState *cpu;

    uint32_t ipr[AP7000_INTC_GROUPS];
    uint32_t irr[AP7000_INTC_GROUPS];
    uint32_t icr[4];
};

#define TYPE_AP7000_PM "ap7000-pm"
OBJECT_DECLARE_SIMPLE_TYPE(AP7000PMState, AP7000_PM)

struct AP7000PMState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    uint32_t regs[0x100 / 4];
};

#define TYPE_ATMEL_USART "atmel-usart"
OBJECT_DECLARE_SIMPLE_TYPE(AtmelUsartState, ATMEL_USART)

struct AtmelUsartState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    CharFrontend chr;
    qemu_irq irq;
    QEMUTimer *rx_timeout;

    uint32_t mr;
    uint32_t imr;
    uint32_t csr;                       /* latched status bits */
    uint32_t rhr;
    uint32_t brgr;
    uint32_t rtor;
    uint32_t ttgr;
    bool rx_enabled;
    bool tx_enabled;
    bool timeout_armed;

    /* peripheral DMA controller */
    uint32_t rpr, rcr, rnpr, rncr;
    uint32_t tpr, tcr, tnpr, tncr;
    uint32_t ptsr;
};

#endif /* HW_AVR32_AP7000_H */
