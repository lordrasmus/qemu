/*
 * ADSP-BF537 STAMP board
 *
 * The parts of the BF537 a Linux kernel needs to boot, after the
 * ADSP-BF537 Hardware Reference, Rev. 3.4, and the Blackfin Processor
 * Programming Reference, Rev. 2.2:
 *
 *  - 64 MiB SDRAM at 0, the L1 SRAM banks
 *  - core MMRs: event controller (EVT, IMASK, IPEND, ILAT), core timer,
 *    DSPID; everything else there stores what is written
 *  - system MMRs: PLL, CHIPID, system interrupt controller, UART0 and
 *    its DMA channels, memory DMA; everything else stores what is written
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "cpu.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "hw/core/irq.h"
#include "qemu/error-report.h"
#include "hw/core/qdev-properties.h"
#include "chardev/char-fe.h"
#include "system/system.h"
#include "system/address-spaces.h"
#include "system/reset.h"
#include "elf.h"
#include "system/dma.h"

#define SDRAM_SIZE      (64 * MiB)
#define CCLK_HZ         500000000u      /* CLKIN 25 MHz x 20 */
#define SCLK_HZ         100000000u      /* CCLK / 5 */

#define CORE_MMR_BASE   0xffe00000u
#define SYS_MMR_BASE    0xffc00000u

/* system interrupt controller peripheral IDs (HWR table 4-2) */
#define PID_UART0_RX    11
#define PID_UART0_TX    12

typedef struct BfinBoard {
    BfinCPU *cpu;
    MemoryRegion core_mmr;
    MemoryRegion sys_mmr;
    GHashTable *store;          /* MMRs that only keep their value */

    /* core timer */
    QEMUTimer *tmr;
    uint32_t tcntl, tperiod, tscale, tcount;
    int64_t tmr_start;          /* ns at which tcount was loaded */

    /* system interrupt controller */
    uint32_t sic_imask, sic_iwr, sic_isr;
    uint32_t sic_iar[4];
    qemu_irq ivg[16];

    /* kernel entry and command line, applied after every CPU reset */
    uint32_t entry, cmdline;
    uint32_t sdbctl;            /* EBIU_SDBCTL for the RAM size */

    /* UART0 */
    CharFrontend chr;
    uint8_t rbr, ier, lcr, mcr, lsr, scr, gctl, dll, dlh;

    /* peripheral DMA channels 8 and 9, UART0 RX and TX */
    struct {
        uint32_t addr, x, y, status;
    } pdma[2];
} BfinBoard;

/* ----------------------------------------------------------- SIC */

/*
 * A peripheral request that is set in SIC_IMASK drives the IVG its
 * SIC_IARx nibble selects (IVG7 + value) as a level.
 */
static void sic_update(BfinBoard *b)
{
    uint32_t act = b->sic_isr & b->sic_imask;
    uint32_t lines = 0;

    for (int pid = 0; pid < 32; pid++) {
        if (act & (1u << pid)) {
            int ivg = 7 + ((b->sic_iar[pid / 8] >> (4 * (pid % 8))) & 0xf);

            if (ivg < 16) {
                lines |= 1u << ivg;
            }
        }
    }
    for (int ivg = 7; ivg < 16; ivg++) {
        qemu_set_irq(b->ivg[ivg], (lines >> ivg) & 1);
    }
}

static void sic_set(BfinBoard *b, int pid, bool level)
{
    if (level) {
        b->sic_isr |= 1u << pid;
    } else {
        b->sic_isr &= ~(1u << pid);
    }
    sic_update(b);
}

/* ---------------------------------------------------------- UART0 */

#define LSR_DR      0x01
#define LSR_THRE    0x20
#define LSR_TEMT    0x40
#define IER_ERBFI   0x01
#define IER_ETBEI   0x02
#define LCR_DLAB    0x80

static bool pdma_enabled(BfinBoard *b, int n);
static bool pdma_irq(BfinBoard *b, int n);
static bool pdma_running(BfinBoard *b, int n);
static void pdma_rx(BfinBoard *b, uint8_t ch);

/*
 * RX while data is ready, TX while the holding register is empty; with
 * its DMA channel enabled the line is the channel's interrupt instead
 */
static void uart_update(BfinBoard *b)
{
    sic_set(b, PID_UART0_RX, pdma_enabled(b, 0) ? pdma_irq(b, 0) :
            (b->lsr & LSR_DR) && (b->ier & IER_ERBFI));
    sic_set(b, PID_UART0_TX, pdma_enabled(b, 1) ? pdma_irq(b, 1) :
            (b->lsr & LSR_THRE) && (b->ier & IER_ETBEI));
}

static int uart_can_receive(void *opaque)
{
    BfinBoard *b = opaque;

    if (pdma_enabled(b, 0)) {
        return pdma_running(b, 0);
    }
    return !(b->lsr & LSR_DR);
}

static void uart_receive(void *opaque, const uint8_t *buf, int size)
{
    BfinBoard *b = opaque;

    if (pdma_enabled(b, 0)) {
        for (int i = 0; i < size && pdma_running(b, 0); i++) {
            pdma_rx(b, buf[i]);
        }
        return;
    }
    b->rbr = buf[0];
    b->lsr |= LSR_DR;
    uart_update(b);
}

static uint32_t uart_read(BfinBoard *b, hwaddr off)
{
    uint32_t v = 0;

    switch (off) {
    case 0x00:
        if (b->lcr & LCR_DLAB) {
            return b->dll;
        }
        v = b->rbr;
        b->lsr &= ~LSR_DR;
        uart_update(b);
        qemu_chr_fe_accept_input(&b->chr);
        return v;
    case 0x04:
        return b->lcr & LCR_DLAB ? b->dlh : b->ier;
    case 0x08:                                  /* IIR, highest first */
        if ((b->lsr & LSR_DR) && (b->ier & IER_ERBFI)) {
            return 0x04;
        }
        if ((b->lsr & LSR_THRE) && (b->ier & IER_ETBEI)) {
            return 0x02;
        }
        return 0x01;
    case 0x0c:
        return b->lcr;
    case 0x10:
        return b->mcr;
    case 0x14:
        return b->lsr;
    case 0x1c:
        return b->scr;
    case 0x24:
        return b->gctl;
    }
    return 0;
}

static void uart_write(BfinBoard *b, hwaddr off, uint32_t v)
{
    switch (off) {
    case 0x00:
        if (b->lcr & LCR_DLAB) {
            b->dll = v;
        } else {
            uint8_t ch = v;

            /* the character leaves at once, THRE stays set */
            qemu_chr_fe_write_all(&b->chr, &ch, 1);
        }
        break;
    case 0x04:
        if (b->lcr & LCR_DLAB) {
            b->dlh = v;
        } else {
            b->ier = v & 0x0f;
        }
        break;
    case 0x0c:
        b->lcr = v;
        break;
    case 0x10:
        b->mcr = v;
        break;
    case 0x1c:
        b->scr = v;
        break;
    case 0x24:
        b->gctl = v;
        break;
    }
    uart_update(b);
}

/* ----------------------------------------------------- core timer */

#define TCNTL_TMPWR     0x1
#define TCNTL_TMREN     0x2
#define TCNTL_TAUTORLD  0x4
#define TCNTL_TINT      0x8

static uint64_t tmr_ns_per_tick(BfinBoard *b)
{
    return muldiv64((uint64_t)b->tscale + 1, NANOSECONDS_PER_SECOND, CCLK_HZ);
}

static bool tmr_running(BfinBoard *b)
{
    return (b->tcntl & (TCNTL_TMPWR | TCNTL_TMREN)) ==
           (TCNTL_TMPWR | TCNTL_TMREN);
}

static uint32_t tmr_count(BfinBoard *b)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t ticks;

    if (!tmr_running(b)) {
        return b->tcount;
    }
    ticks = (now - b->tmr_start) / MAX(tmr_ns_per_tick(b), 1);
    return ticks >= b->tcount ? 0 : b->tcount - ticks;
}

static void tmr_arm(BfinBoard *b)
{
    timer_del(b->tmr);
    if (tmr_running(b) && b->tcount) {
        b->tmr_start = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        timer_mod(b->tmr, b->tmr_start + b->tcount * tmr_ns_per_tick(b));
    }
}

/* TCOUNT reached 0: TINT, IVTMR, reload or stop */
static void tmr_expire(void *opaque)
{
    BfinBoard *b = opaque;
    CPUBfinState *env = &b->cpu->env;

    b->tcntl |= TCNTL_TINT;
    env->ilat |= 1u << 6;
    bfin_update_irq(env);
    if (b->tcntl & TCNTL_TAUTORLD) {
        b->tcount = b->tperiod;
        tmr_arm(b);
    } else {
        /* the count stops at 0; TMREN stays as written (ADI's timer tests) */
        b->tcount = 0;
        timer_del(b->tmr);
    }
}

/*
 * The expiry callback runs from the main loop, possibly after the guest
 * has polled; catch up on a read so a short count is seen as expired
 */
static void tmr_sync(BfinBoard *b)
{
    if (tmr_running(b) && b->tcount &&
        qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) >=
        b->tmr_start + (int64_t)(b->tcount * tmr_ns_per_tick(b))) {
        tmr_expire(b);
    }
}

/* ------------------------------------------------------ memory DMA */

/*
 * Memory DMA streams 0 and 1 in register based stop mode (HWR chapter 5):
 * MDMA_Dn at 0xffc00f00 + 0x80 * n, its source MDMA_Sn 0x40 above.
 * The transfer runs completely when the destination channel is enabled.
 */
#define MDMA_BASE       0xffc00f00u
#define DMA_CONFIG      0x08
#define DMA_START       0x04
#define DMA_XCOUNT      0x10
#define DMA_XMODIFY     0x14
#define DMA_YCOUNT      0x18
#define DMA_YMODIFY     0x1c
#define DMA_IRQ_STATUS  0x28
#define DMAEN           0x01
#define DMA2D           0x10
#define DI_EN           0x80
#define DMA_DONE        0x01
#define DMA_ERR         0x02
#define PID_MDMA0       29

static uint32_t store_get(BfinBoard *b, hwaddr addr);
static void store_set(BfinBoard *b, hwaddr addr, uint32_t v);

static void mdma_run(BfinBoard *b, int n)
{
    hwaddr d = MDMA_BASE + 0x80 * n, s = d + 0x40;
    uint32_t dcfg = store_get(b, d + DMA_CONFIG);
    uint32_t scfg = store_get(b, s + DMA_CONFIG);
    int dw = 1 << ((dcfg >> 2) & 3), sw = 1 << ((scfg >> 2) & 3);
    uint32_t da = store_get(b, d + DMA_START), sa = store_get(b, s + DMA_START);
    uint32_t dx = store_get(b, d + DMA_XCOUNT) & 0xffff;
    uint32_t dy = dcfg & DMA2D ? store_get(b, d + DMA_YCOUNT) & 0xffff : 1;
    uint32_t sx = store_get(b, s + DMA_XCOUNT) & 0xffff;
    uint32_t sy = scfg & DMA2D ? store_get(b, s + DMA_YCOUNT) & 0xffff : 1;
    int32_t dxm = (int16_t)store_get(b, d + DMA_XMODIFY);
    int32_t dym = (int16_t)store_get(b, d + DMA_YMODIFY);
    int32_t sxm = (int16_t)store_get(b, s + DMA_XMODIFY);
    int32_t sym = (int16_t)store_get(b, s + DMA_YMODIFY);
    uint64_t total;
    uint32_t si = 0, sj = 0, di = 0, dj = 0;

    if (!(dcfg & DMAEN) || !(scfg & DMAEN)) {
        return;
    }
    /* a count of 0 means 0x10000 */
    dx = dx ?: 0x10000;
    sx = sx ?: 0x10000;
    dy = dy ?: 0x10000;
    sy = sy ?: 0x10000;
    total = (uint64_t)dx * dy * dw;
    if (dw != sw || total != (uint64_t)sx * sy * sw) {
        qemu_log_mask(LOG_UNIMP, "bfin mdma%d: unsupported size mix\n", n);
    }
    for (uint64_t done = 0; done < total; done += dw) {
        uint32_t buf;

        dma_memory_read(&address_space_memory, sa, &buf, sw,
                        MEMTXATTRS_UNSPECIFIED);
        dma_memory_write(&address_space_memory, da, &buf, dw,
                         MEMTXATTRS_UNSPECIFIED);
        if (++si < sx) {
            sa += sxm;
        } else {
            si = 0;
            sj++;
            sa += scfg & DMA2D ? sym : sxm;
        }
        if (++di < dx) {
            da += dxm;
        } else {
            di = 0;
            dj++;
            da += dcfg & DMA2D ? dym : dxm;
        }
    }
    store_set(b, d + DMA_IRQ_STATUS, store_get(b, d + DMA_IRQ_STATUS) | DMA_DONE);
    store_set(b, s + DMA_IRQ_STATUS, store_get(b, s + DMA_IRQ_STATUS) | DMA_DONE);
    if (dcfg & DI_EN) {
        sic_set(b, PID_MDMA0 + n, true);
    }
}

/* -------------------------------------------------- peripheral DMA */

/*
 * The UART0 channels in their default mapping (HWR table 5-7): channel 8
 * receives, channel 9 transmits, register blocks at 0xffc00c00 + 0x40 * n.
 * Linux's bfin_uart takes them unless built with SERIAL_BFIN_PIO.  Stop
 * and autobuffer flow, 1D and 2D, 8-bit words; no descriptors.  A
 * transmit buffer leaves at once, as a character written to THR does.
 */
#define PDMA_BASE       0xffc00c00u
#define PDMA_UART0_RX   8
#define DMA_CURR_ADDR   0x24
#define DMA_PMAP        0x2c
#define DMA_CURR_X      0x30
#define DMA_CURR_Y      0x38
#define WNR             0x02
#define DI_SEL          0x40
#define DMAFLOW         0x7000
#define DMAFLOW_AUTO    0x1000
#define DMA_RUN         0x08

static hwaddr pdma_reg(int n, hwaddr off)
{
    return PDMA_BASE + 0x40 * (PDMA_UART0_RX + n) + off;
}

static uint32_t pdma_cfg(BfinBoard *b, int n)
{
    return store_get(b, pdma_reg(n, DMA_CONFIG));
}

static bool pdma_enabled(BfinBoard *b, int n)
{
    return pdma_cfg(b, n) & DMAEN;
}

static bool pdma_running(BfinBoard *b, int n)
{
    return b->pdma[n].status & DMA_RUN;
}

static bool pdma_irq(BfinBoard *b, int n)
{
    return (b->pdma[n].status & DMA_DONE) && (pdma_cfg(b, n) & DI_EN);
}

static uint32_t pdma_count(BfinBoard *b, int n, hwaddr off)
{
    return (store_get(b, pdma_reg(n, off)) & 0xffff) ?: 0x10000;
}

/* load the work unit's start values */
static void pdma_load(BfinBoard *b, int n)
{
    b->pdma[n].addr = store_get(b, pdma_reg(n, DMA_START));
    b->pdma[n].x = pdma_count(b, n, DMA_XCOUNT);
    b->pdma[n].y = pdma_cfg(b, n) & DMA2D ? pdma_count(b, n, DMA_YCOUNT) : 1;
}

/*
 * Advance past one element: XMODIFY within a row, YMODIFY after its last
 * element.  An interrupt comes after every row with DI_SEL, otherwise
 * after the work unit, which then restarts in autobuffer mode and stops
 * the channel in stop mode.
 */
static void pdma_step(BfinBoard *b, int n)
{
    uint32_t cfg = pdma_cfg(b, n);
    bool row_end = --b->pdma[n].x == 0;

    if (!row_end) {
        b->pdma[n].addr += (int16_t)store_get(b, pdma_reg(n, DMA_XMODIFY));
        return;
    }
    if (b->pdma[n].y > 1) {
        b->pdma[n].addr += (int16_t)store_get(b, pdma_reg(n, DMA_YMODIFY));
        b->pdma[n].x = pdma_count(b, n, DMA_XCOUNT);
        b->pdma[n].y--;
        if (cfg & DI_SEL) {
            b->pdma[n].status |= DMA_DONE;
        }
    } else {
        b->pdma[n].status |= DMA_DONE;
        if ((cfg & DMAFLOW) == DMAFLOW_AUTO) {
            pdma_load(b, n);
        } else {
            b->pdma[n].status &= ~DMA_RUN;
        }
    }
    uart_update(b);
}

static void pdma_rx(BfinBoard *b, uint8_t ch)
{
    dma_memory_write(&address_space_memory, b->pdma[0].addr, &ch, 1,
                     MEMTXATTRS_UNSPECIFIED);
    pdma_step(b, 0);
}

static void pdma_tx(BfinBoard *b)
{
    if ((pdma_cfg(b, 1) & DMAFLOW) != 0) {
        qemu_log_mask(LOG_UNIMP, "bfin uart0 tx dma: only stop mode\n");
        return;
    }
    while (pdma_running(b, 1)) {
        uint8_t ch;

        dma_memory_read(&address_space_memory, b->pdma[1].addr, &ch, 1,
                        MEMTXATTRS_UNSPECIFIED);
        qemu_chr_fe_write_all(&b->chr, &ch, 1);
        pdma_step(b, 1);
    }
}

static uint32_t pdma_read(BfinBoard *b, hwaddr addr)
{
    int ch = (addr - PDMA_BASE) / 0x40, n = ch - PDMA_UART0_RX;
    hwaddr off = addr & 0x3f;

    if (off == DMA_PMAP) {
        return store_get(b, addr) ?: ch << 12;
    }
    if (n != 0 && n != 1) {
        return store_get(b, addr);
    }
    switch (off) {
    case DMA_CURR_ADDR:
        return b->pdma[n].addr;
    case DMA_IRQ_STATUS:
        return b->pdma[n].status;
    case DMA_CURR_X:
        return b->pdma[n].x & 0xffff;
    case DMA_CURR_Y:
        return b->pdma[n].y & 0xffff;
    }
    return store_get(b, addr);
}

static void pdma_write(BfinBoard *b, hwaddr addr, uint32_t v)
{
    int n = (addr - PDMA_BASE) / 0x40 - PDMA_UART0_RX;
    hwaddr off = addr & 0x3f;
    bool was_on;

    if (n != 0 && n != 1) {
        store_set(b, addr, v);
        return;
    }
    if (off == DMA_IRQ_STATUS) {
        /* DONE and ERR are write-1-to-clear */
        b->pdma[n].status &= ~(v & (DMA_DONE | DMA_ERR));
        uart_update(b);
        return;
    }
    was_on = pdma_enabled(b, n);
    store_set(b, addr, v);
    if (off != DMA_CONFIG) {
        return;
    }
    if (!(v & DMAEN)) {
        b->pdma[n].status &= ~DMA_RUN;
    } else if (!was_on) {
        if (((v & WNR) != 0) != (n == 0) || ((v >> 2) & 3) != 0) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "bfin uart0 dma%d: config 0x%x\n", 8 + n, v);
        }
        if ((v & DMAFLOW) > DMAFLOW_AUTO) {
            qemu_log_mask(LOG_UNIMP,
                          "bfin uart0 dma%d: no descriptor flow\n", 8 + n);
        }
        pdma_load(b, n);
        b->pdma[n].status |= DMA_RUN;
        if (n == 1) {
            pdma_tx(b);
        } else {
            qemu_chr_fe_accept_input(&b->chr);
        }
    }
    uart_update(b);
}

/* ------------------------------------------------------- MMR space */

static uint32_t store_get(BfinBoard *b, hwaddr addr)
{
    return GPOINTER_TO_UINT(g_hash_table_lookup(b->store,
                                                GUINT_TO_POINTER(addr)));
}

static void store_set(BfinBoard *b, hwaddr addr, uint32_t v)
{
    g_hash_table_insert(b->store, GUINT_TO_POINTER(addr), GUINT_TO_POINTER(v));
}

static uint64_t core_mmr_read(void *opaque, hwaddr off, unsigned size)
{
    BfinBoard *b = opaque;
    CPUBfinState *env = &b->cpu->env;
    hwaddr addr = CORE_MMR_BASE + off;

    switch (addr) {
    case 0xffe02000 ... 0xffe0203c:             /* EVT0..15 */
        return env->evt[(addr - 0xffe02000) / 4];
    case 0xffe02104:
        return env->imask;
    case 0xffe02108:
        return env->ipend;
    case 0xffe0210c:
        return env->ilat;
    case 0xffe03000:
        tmr_sync(b);
        return b->tcntl;
    case 0xffe03004:
        return b->tperiod;
    case 0xffe03008:
        return b->tscale;
    case 0xffe0300c:
        tmr_sync(b);
        return tmr_count(b);
    case 0xffe00008:                            /* DCPLB_FAULT_ADDR */
        return env->dcplb_fault_addr;
    case 0xffe01008:                            /* ICPLB_FAULT_ADDR */
        return env->icplb_fault_addr;
    case 0xffe05000:                            /* DSPID: ADI, BF5xx */
        return 0xe5040003;
    }
    return store_get(b, addr);
}

static void core_mmr_write(void *opaque, hwaddr off, uint64_t v,
                           unsigned size)
{
    BfinBoard *b = opaque;
    CPUBfinState *env = &b->cpu->env;
    hwaddr addr = CORE_MMR_BASE + off;

    switch (addr) {
    case 0xffe02000 ... 0xffe0203c:
        env->evt[(addr - 0xffe02000) / 4] = v;
        return;
    case 0xffe02104:
        env->imask = (v & 0xffff) | 0x1f;
        bfin_update_irq(env);
        return;
    case 0xffe02108:                            /* IPEND is read only */
        return;
    case 0xffe0210c:                            /* writing 1 cancels */
        env->ilat &= ~(v & ~env->imask);
        bfin_update_irq(env);
        return;
    case 0xffe03000:
        b->tcount = tmr_count(b);
        b->tcntl = v & 0xf;
        tmr_arm(b);
        return;
    case 0xffe03004:
        if (!tmr_running(b)) {
            b->tperiod = v;
            b->tcount = v;
        }
        return;
    case 0xffe03008:
        b->tscale = v & 0xff;
        return;
    case 0xffe0300c:
        if (!tmr_running(b)) {
            b->tcount = v;
        }
        return;
    }
    store_set(b, addr, v);
}

static uint64_t sys_mmr_read(void *opaque, hwaddr off, unsigned size)
{
    BfinBoard *b = opaque;
    hwaddr addr = SYS_MMR_BASE + off;

    switch (addr) {
    case 0xffc00000:                            /* PLL_CTL: MSEL 20 */
        return store_get(b, addr) ?: 20 << 9;
    case 0xffc00004:                            /* PLL_DIV: SSEL 5 */
        return store_get(b, addr) ?: 5;
    case 0xffc0000c:                            /* PLL_STAT: locked */
        return 0x20;
    case 0xffc00014:                            /* CHIPID: BF537 rev 0.2 */
        return 0x227c80cb;
    case 0xffc00a14:
        /*
         * EBIU_SDBCTL as the boot loader leaves it: EBCAW_10 | EBSZ | EBE;
         * Linux sizes its memory from EBSZ
         */
        return store_get(b, addr) ?: b->sdbctl;
    case 0xffc0010c:
        return b->sic_imask;
    case 0xffc00110 ... 0xffc0011c:
        return b->sic_iar[(addr - 0xffc00110) / 4];
    case 0xffc00120:
        return b->sic_isr;
    case 0xffc00124:
        return b->sic_iwr;
    case 0xffc00400 ... 0xffc0042f:
        return uart_read(b, addr - 0xffc00400);
    case 0xffc00c00 ... 0xffc00eff:
        return pdma_read(b, addr);
    }
    return store_get(b, addr);
}

static void sys_mmr_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    BfinBoard *b = opaque;
    hwaddr addr = SYS_MMR_BASE + off;

    switch (addr) {
    case 0xffc0010c:
        b->sic_imask = v;
        sic_update(b);
        return;
    case 0xffc00110 ... 0xffc0011c:
        b->sic_iar[(addr - 0xffc00110) / 4] = v;
        sic_update(b);
        return;
    case 0xffc00120:                            /* SIC_ISR is read only */
        return;
    case 0xffc00124:
        b->sic_iwr = v;
        return;
    case 0xffc00400 ... 0xffc0042f:
        uart_write(b, addr - 0xffc00400, v);
        return;
    case 0xffc00c00 ... 0xffc00eff:
        pdma_write(b, addr, v);
        return;
    case 0xffc00f00 ... 0xffc00fff:
        if ((addr & 0x3f) == DMA_IRQ_STATUS) {
            /* DONE and ERR are write-1-to-clear */
            store_set(b, addr, store_get(b, addr) & ~(v & (DMA_DONE | DMA_ERR)));
            if ((addr & 0x40) == 0 && !(store_get(b, addr) & DMA_DONE)) {
                sic_set(b, PID_MDMA0 + ((addr >> 7) & 1), false);
            }
            return;
        }
        store_set(b, addr, v);
        if ((addr & 0x7f) == DMA_CONFIG && (v & DMAEN)) {
            mdma_run(b, (addr >> 7) & 1);
        }
        return;
    }
    store_set(b, addr, v);
}

static const MemoryRegionOps core_mmr_ops = {
    .read = core_mmr_read,
    .write = core_mmr_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static const MemoryRegionOps sys_mmr_ops = {
    .read = sys_mmr_read,
    .write = sys_mmr_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

/* ----------------------------------------------------------- board */

static void bf537_reset(void *opaque)
{
    BfinBoard *b = opaque;
    CPUBfinState *env = &b->cpu->env;

    env->clock_freq = CCLK_HZ;
    if (b->entry) {
        env->evt[1] = b->entry;
        env->pc = b->entry;
        env->reg[BFIN_R0] = b->cmdline;
    }
}

static void bf537_add_ram(MemoryRegion *sysmem, const char *name,
                          hwaddr base, uint64_t size)
{
    MemoryRegion *mr = g_new(MemoryRegion, 1);

    memory_region_init_ram(mr, NULL, name, size, &error_fatal);
    memory_region_add_subregion(sysmem, base, mr);
}

static void bf537_stamp_init(MachineState *machine)
{
    MemoryRegion *sysmem = get_system_memory();
    BfinBoard *b = g_new0(BfinBoard, 1);
    g_autofree char *initrd_arg = NULL;
    uint64_t entry;
    uint32_t ebsz;

    /*
     * EBSZ of EBIU_SDBCTL (HWR table 6-14 lists 16 to 128 MiB for the
     * BF537; 256 and 512 MiB use the encodings of the larger parts, so a
     * test rootfs that does not fit the STAMP's 64 MiB still runs)
     */
    switch (machine->ram_size) {
    case 16 * MiB:
        ebsz = 0x0;
        break;
    case 32 * MiB:
        ebsz = 0x2;
        break;
    case 64 * MiB:
        ebsz = 0x4;
        break;
    case 128 * MiB:
        ebsz = 0x6;
        break;
    case 256 * MiB:
        ebsz = 0x8;
        break;
    case 512 * MiB:
        ebsz = 0xa;
        break;
    default:
        error_report("RAM size must be 16, 32, 64, 128, 256 or 512 MiB");
        exit(1);
    }
    b->sdbctl = 0x20 | ebsz | 1;            /* EBCAW_10 | EBSZ | EBE */

    b->cpu = BFIN_CPU(cpu_create(machine->cpu_type));
    for (int i = 0; i < 16; i++) {
        b->ivg[i] = qdev_get_gpio_in(DEVICE(b->cpu), i);
    }

    memory_region_add_subregion(sysmem, 0, machine->ram);
    bf537_add_ram(sysmem, "l1-data-a", 0xff800000, 32 * KiB);
    bf537_add_ram(sysmem, "l1-data-b", 0xff900000, 32 * KiB);
    bf537_add_ram(sysmem, "l1-code", 0xffa00000, 64 * KiB);
    bf537_add_ram(sysmem, "l1-scratch", 0xffb00000, 4 * KiB);

    b->store = g_hash_table_new(NULL, NULL);
    memory_region_init_io(&b->sys_mmr, NULL, &sys_mmr_ops, b, "sys-mmr",
                          2 * MiB);
    memory_region_add_subregion(sysmem, SYS_MMR_BASE, &b->sys_mmr);
    memory_region_init_io(&b->core_mmr, NULL, &core_mmr_ops, b, "core-mmr",
                          2 * MiB);
    memory_region_add_subregion(sysmem, CORE_MMR_BASE, &b->core_mmr);

    b->tmr = timer_new_ns(QEMU_CLOCK_VIRTUAL, tmr_expire, b);
    /* default peripheral to IVG assignment (HWR table 4-2) */
    b->sic_iar[0] = 0x22211000;
    b->sic_iar[1] = 0x43333332;
    b->sic_iar[2] = 0x55555444;
    b->sic_iar[3] = 0x66655555;
    b->lsr = LSR_THRE | LSR_TEMT;

    qemu_chr_fe_init(&b->chr, serial_hd(0), &error_abort);
    qemu_chr_fe_set_handlers(&b->chr, uart_can_receive, uart_receive,
                             NULL, NULL, b, NULL, true);

    if (machine->kernel_filename) {
        if (load_elf(machine->kernel_filename, NULL, NULL, NULL, &entry,
                     NULL, NULL, NULL, ELFDATA2LSB, EM_BLACKFIN, 0, 0) < 0) {
            error_report("could not load kernel '%s'",
                         machine->kernel_filename);
            exit(1);
        }
        b->entry = entry;
    }
    if (machine->initrd_filename) {
        /*
         * below the 1 MiB the kernel keeps for uncached DMA at the top of
         * memory; Linux learns of it through "initrd=start,size"
         */
        int64_t size = get_image_size(machine->initrd_filename, NULL);
        uint32_t start;

        if (size <= 0 || size > machine->ram_size / 2) {
            error_report("could not load initrd '%s'",
                         machine->initrd_filename);
            exit(1);
        }
        start = (machine->ram_size - 2 * MiB - size) & ~(uint32_t)0xfff;
        load_image_targphys(machine->initrd_filename, start, size, NULL);
        initrd_arg = g_strdup_printf("initrd=0x%x,0x%" PRIx64, start, size);
    }
    if (machine->kernel_filename) {
        /* the kernel takes its command line from R0 (head.S) */
        g_autofree char *cmd = g_strjoin(" ",
                                 machine->kernel_cmdline ?: "",
                                 initrd_arg ?: "", NULL);

        g_strstrip(cmd);
        if (*cmd) {
            b->cmdline = machine->ram_size - 4 * KiB;
            rom_add_blob_fixed("cmdline", cmd, strlen(cmd) + 1, b->cmdline);
        }
    }
    qemu_register_reset(bf537_reset, b);
}

static void bf537_stamp_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "ADSP-BF537 STAMP";
    mc->init = bf537_stamp_init;
    mc->default_cpu_type = BFIN_CPU_TYPE_NAME("bf537");
    mc->default_ram_size = SDRAM_SIZE;
    mc->default_ram_id = "sdram";
}

static const TypeInfo bf537_stamp_type = {
    .name = MACHINE_TYPE_NAME("bf537-stamp"),
    .parent = TYPE_MACHINE,
    .class_init = bf537_stamp_class_init,
};

static void bf537_stamp_register_types(void)
{
    type_register_static(&bf537_stamp_type);
}

type_init(bf537_stamp_register_types)
