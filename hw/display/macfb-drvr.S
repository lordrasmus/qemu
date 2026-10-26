/*
 * Mac OS video driver (DRVR) for the NuBus macfb card
 *
 * The Slot Manager copies this driver out of the card's declaration ROM
 * and opens it. It drives the DAFB style registers of macfb: mode control,
 * CLUT and the VBL interrupt. They lie 8 MiB into the slot, outside the
 * 1 MiB a slot has in 24-bit mode, so every register access runs in 32-bit
 * mode (_SwapMMUMode). The frame buffer is in the first 1 MiB and works in
 * either mode.
 *
 * macfb fills in the mode table at "params" when it builds the ROM.
 *
 * Rebuild macfb-drvr.h with m68k binutils:
 *   m68k-linux-gnu-as -m68020 --register-prefix-optional -o macfb-drvr.o macfb-drvr.S
 *   m68k-linux-gnu-objcopy -O binary macfb-drvr.o macfb-drvr.bin
 *   scripts/macfb-drvr-header.sh macfb-drvr.o macfb-drvr.bin > macfb-drvr.h
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

/* parameter blocks */
        .set    ioTrap, 6
        .set    csCode, 26
        .set    csParam, 28
        .set    noQueueBit, 9

/* device control entry */
        .set    dCtlStorage, 20
        .set    dCtlSlot, 40
        .set    dCtlDevBase, 42

        .set    JIODone, 0x8fc

        .set    noErr, 0
        .set    controlErr, -17
        .set    statusErr, -18
        .set    openErr, -23

/* macfb registers, from the slot base */
        .set    REGS, 0x00800000
        .set    MODE_CTRL1, 0x8
        .set    MODE_CTRL2, 0xc
        .set    INTR_MASK, 0x104
        .set    INTR_CLEAR, 0x10c
        .set    LUT_INDEX, 0x200
        .set    LUT, 0x210
        .set    INTR_VBL, 0x4

/* private storage: slot interrupt queue element, then the driver state */
        .set    sqLink, 0
        .set    sqType, 4
        .set    sqPrio, 6
        .set    sqAddr, 8
        .set    sqParm, 12
        .set    regbase, 16             /* 32-bit address of the registers */
        .set    devbase, 20             /* dCtlDevBase */
        .set    slot, 24
        .set    mode, 26                /* current spID, 0x80 to 0x83 */
        .set    gray, 28                /* luminance mapping on */
        .set    irqoff, 30              /* VBL interrupts off */
        .set    storsize, 32

        .set    sIQType, 6

/* one mode table entry */
        .set    m_ctrl1, 0
        .set    m_ctrl2, 2
        .set    m_offset, 4
        .set    m_rowbytes, 8
        .set    m_depth, 10
        .set    m_size, 12

        .set    FIRSTMODE, 0x80
        .set    NMODES, 4

        .macro  _SwapMMUMode
        .short  0xa05d
        .endm
        .macro  _StripAddress
        .short  0xa055
        .endm
        .macro  _NewHandleSysClear
        .short  0xa722
        .endm
        .macro  _HLock
        .short  0xa029
        .endm
        .macro  _DisposHandle
        .short  0xa023
        .endm
        .macro  _SIntInstall
        .short  0xa075
        .endm
        .macro  _SIntRemove
        .short  0xa076
        .endm
        .macro  _DoVBLTask
        .short  0xa072
        .endm

        .text
drvr:
        .short  0x4c00                  /* dNeedLock, dStatEnable, dCtlEnable */
        .short  0, 0, 0                 /* delay, event mask, menu */
        .short  do_open - drvr
        .short  do_prime - drvr
        .short  do_control - drvr
        .short  do_status - drvr
        .short  do_close - drvr
name:
        .byte   name_end - name - 1
        .ascii  ".Display_Video_QEMU"
name_end:
        .balign 2

/*
 * The mode table, filled in by macfb: the visible height, then for each
 * spID from 0x80 the MODE_CTRL1/2 values, the offset of the first pixel,
 * the row bytes and the depth.
 */
        .globl  params
params:
height:
        .short  0
        .short  0
modetab:
        .space  NMODES * m_size

/* a linear gamma table: version, type, formula size, channels, entries, bits */
gamma:
        .short  0, 0, 0, 1, 256, 8
        .set    i, 0
        .rept   256
        .byte   i
        .set    i, i + 1
        .endr

/*
 * Open: A0 parameter block, A1 DCE.
 */
do_open:
        movem.l d1-d7/a0-a6, -(%sp)
        move.l  %a1, %a3
        moveq   #storsize, %d0
        _NewHandleSysClear
        tst.w   %d0
        bne     open_fail
        move.l  %a0, dCtlStorage(%a3)
        _HLock
        move.l  (%a0), %d0
        _StripAddress
        move.l  %d0, %a2

        moveq   #0, %d0
        move.b  dCtlSlot(%a3), %d0
        move.w  %d0, slot(%a2)
        ror.l   #8, %d0
        or.l    #0xf0000000 + REGS, %d0
        move.l  %d0, regbase(%a2)
        move.l  dCtlDevBase(%a3), devbase(%a2)

        move.w  #FIRSTMODE, %d0
        bsr     set_mode

        move.w  #sIQType, sqType(%a2)
        lea     vbl_int(%pc), %a0
        move.l  %a0, sqAddr(%a2)
        move.l  %a2, sqParm(%a2)
        move.l  %a2, %a0
        move.w  slot(%a2), %d0
        _SIntInstall
        tst.w   %d0
        bne     open_fail

        moveq   #0, %d0
        bsr     set_irq
        moveq   #noErr, %d0
        movem.l (%sp)+, d1-d7/a0-a6
        rts

open_fail:
        move.w  #openErr, %d0
        movem.l (%sp)+, d1-d7/a0-a6
        rts

/*
 * Close: no more interrupts, give back the storage.
 */
do_close:
        movem.l d1-d7/a0-a6, -(%sp)
        move.l  %a1, %a3
        move.l  dCtlStorage(%a3), %a4
        move.l  (%a4), %d0
        _StripAddress
        move.l  %d0, %a2
        moveq   #1, %d0
        bsr     set_irq
        move.l  %a2, %a0
        move.w  slot(%a2), %d0
        _SIntRemove
        move.l  %a4, %a0
        _DisposHandle
        clr.l   dCtlStorage(%a3)
        moveq   #noErr, %d0
        movem.l (%sp)+, d1-d7/a0-a6
        rts

do_prime:
        moveq   #noErr, %d0
        bra     done

/*
 * Control: csCode selects the call, csParam holds a pointer to its record.
 */
do_control:
        movem.l d1-d7/a0-a6, -(%sp)
        move.l  dCtlStorage(%a1), %a2
        move.l  (%a2), %d0
        _StripAddress
        move.l  %d0, %a2
        move.l  csParam(%a0), %a3
        move.w  csCode(%a0), %d0
        cmp.w   #9, %d0
        bhi     ctl_bad
        add.w   %d0, %d0
        move.w  ctl_tab(%pc, %d0.w), %d0
        jmp     ctl_tab(%pc, %d0.w)
ctl_tab:
        .short  ctl_reset - ctl_tab
        .short  ctl_ok - ctl_tab                /* KillIO */
        .short  ctl_setmode - ctl_tab
        .short  ctl_setentries - ctl_tab
        .short  ctl_ok - ctl_tab                /* SetGamma */
        .short  ctl_graypage - ctl_tab
        .short  ctl_setgray - ctl_tab
        .short  ctl_setinterrupt - ctl_tab
        .short  ctl_bad - ctl_tab               /* DirectSetEntries */
        .short  ctl_ok - ctl_tab                /* SetDefaultMode */

/* VDPageInfo: csMode.w, csData.l, csPage.w, csBaseAddr.l */
ctl_reset:
        move.w  #FIRSTMODE, %d0
        bsr     set_mode
        bsr     gray_page
        move.w  #FIRSTMODE, (%a3)
        clr.w   6(%a3)
        bsr     base_addr
        move.l  %d0, 8(%a3)
        bra     ctl_ok

ctl_setmode:
        tst.w   6(%a3)
        bne     ctl_bad
        move.w  (%a3), %d0
        sub.w   #FIRSTMODE, %d0
        cmp.w   #NMODES, %d0
        bcc     ctl_bad
        move.w  (%a3), %d0
        bsr     set_mode
        bsr     base_addr
        move.l  %d0, 8(%a3)
        bra     ctl_ok

/* VDSetEntryRecord: csTable.l, csStart.w, csCount.w (entries - 1) */
ctl_setentries:
        move.l  (%a3), %d0
        _StripAddress
        move.l  %d0, %a4                /* ColorSpec: value.w, r.w, g.w, b.w */
        move.w  4(%a3), %d4             /* index, -1: take it from value */
        move.w  6(%a3), %d5
        bsr     to32
        move.l  regbase(%a2), %a5
1:
        move.w  %d4, %d0
        cmp.w   #-1, %d4
        bne     2f
        move.w  (%a4), %d0
2:
        and.l   #0xff, %d0
        move.l  %d0, LUT_INDEX(%a5)
        move.b  2(%a4), %d1             /* red */
        move.b  4(%a4), %d2             /* green */
        move.b  6(%a4), %d3             /* blue */
        tst.w   gray(%a2)
        beq     3f
        bsr     luminance
3:
        move.b  %d1, LUT(%a5)
        move.b  %d2, LUT(%a5)
        move.b  %d3, LUT(%a5)
        addq.l  #8, %a4
        cmp.w   #-1, %d4
        beq     4f
        addq.w  #1, %d4
4:
        dbra    %d5, 1b
        bsr     from32
        bra     ctl_ok

ctl_graypage:
        tst.w   6(%a3)
        bne     ctl_bad
        bsr     gray_page
        bra     ctl_ok

/* VDGrayRecord, VDFlagRecord: csMode.b */
ctl_setgray:
        moveq   #0, %d0
        move.b  (%a3), %d0
        move.w  %d0, gray(%a2)
        bra     ctl_ok

ctl_setinterrupt:
        moveq   #0, %d0
        move.b  (%a3), %d0
        bsr     set_irq
        bra     ctl_ok

ctl_ok:
        moveq   #noErr, %d0
        bra     ctl_done
ctl_bad:
        moveq   #controlErr, %d0
ctl_done:
        movem.l (%sp)+, d1-d7/a0-a6
        bra     done

/*
 * Status
 */
do_status:
        movem.l d1-d7/a0-a6, -(%sp)
        move.l  dCtlStorage(%a1), %a2
        move.l  (%a2), %d0
        _StripAddress
        move.l  %d0, %a2
        move.l  csParam(%a0), %a3
        move.w  csCode(%a0), %d0
        cmp.w   #9, %d0
        bhi     st_bad
        add.w   %d0, %d0
        move.w  st_tab(%pc, %d0.w), %d0
        jmp     st_tab(%pc, %d0.w)
st_tab:
        .short  st_bad - st_tab
        .short  st_bad - st_tab
        .short  st_getmode - st_tab
        .short  st_getentries - st_tab
        .short  st_getpages - st_tab
        .short  st_getbaseaddr - st_tab
        .short  st_getgray - st_tab
        .short  st_getinterrupt - st_tab
        .short  st_getgamma - st_tab
        .short  st_getdefaultmode - st_tab

st_getmode:
        move.w  mode(%a2), (%a3)
        clr.w   6(%a3)
        bsr     base_addr
        move.l  %d0, 8(%a3)
        bra     st_ok

st_getentries:
        move.l  (%a3), %d0
        _StripAddress
        move.l  %d0, %a4
        move.w  4(%a3), %d4
        move.w  6(%a3), %d5
        bsr     to32
        move.l  regbase(%a2), %a5
1:
        move.w  %d4, %d0
        cmp.w   #-1, %d4
        bne     2f
        move.w  (%a4), %d0
2:
        and.l   #0xff, %d0
        move.l  %d0, LUT_INDEX(%a5)
        move.w  %d0, (%a4)
        moveq   #2, %d3
3:
        move.b  LUT(%a5), %d1
        move.b  %d1, %d2
        lsl.w   #8, %d2
        move.b  %d1, %d2
        move.w  %d2, 2(%a4)
        addq.l  #2, %a4
        dbra    %d3, 3b
        addq.l  #2, %a4
        cmp.w   #-1, %d4
        beq     4f
        addq.w  #1, %d4
4:
        dbra    %d5, 1b
        bsr     from32
        bra     st_ok

st_getpages:
        move.w  #1, 6(%a3)
        bra     st_ok

st_getbaseaddr:
        tst.w   6(%a3)
        bne     st_bad
        bsr     base_addr
        move.l  %d0, 8(%a3)
        bra     st_ok

st_getgray:
        move.w  gray(%a2), %d0
        move.b  %d0, (%a3)
        bra     st_ok

st_getinterrupt:
        move.w  irqoff(%a2), %d0
        move.b  %d0, (%a3)
        bra     st_ok

/* VDGammaRecord: csGTable.l */
st_getgamma:
        lea     gamma(%pc), %a0
        move.l  %a0, (%a3)
        bra     st_ok

/* VDDefModeRec: csID.b */
st_getdefaultmode:
        move.b  #FIRSTMODE, (%a3)
        bra     st_ok

st_ok:
        moveq   #noErr, %d0
        bra     st_done
st_bad:
        moveq   #statusErr, %d0
st_done:
        movem.l (%sp)+, d1-d7/a0-a6
        /* fall through */

/* immediate calls return, queued ones finish through IODone */
done:
        btst    #noQueueBit - 8, ioTrap(%a0)
        bne     1f
        move.l  JIODone, -(%sp)
1:
        rts

/*
 * Helpers, all with A2 pointing to the storage.
 */

/* D0: the mode table entry of the current mode into A6 */
mode_entry:
        move.w  mode(%a2), %d0
        sub.w   #FIRSTMODE, %d0
        mulu.w  #m_size, %d0
        lea     modetab(%pc), %a6
        add.l   %d0, %a6
        rts

/* D0: base address of page 0 in the current mode */
base_addr:
        bsr     mode_entry
        move.l  devbase(%a2), %d0
        add.l   m_offset(%a6), %d0
        rts

/* D0.w: spID; program MODE_CTRL1/2 */
set_mode:
        move.w  %d0, mode(%a2)
        bsr     mode_entry
        bsr     to32
        move.l  regbase(%a2), %a5
        moveq   #0, %d0
        move.w  m_ctrl1(%a6), %d0
        move.l  %d0, MODE_CTRL1(%a5)
        move.w  m_ctrl2(%a6), %d0
        move.l  %d0, MODE_CTRL2(%a5)
        bra     from32

/* D0.b: 0 enables the VBL interrupt, anything else disables it */
set_irq:
        and.w   #0xff, %d0
        move.w  %d0, irqoff(%a2)
        bsr     to32
        move.l  regbase(%a2), %a5
        moveq   #INTR_VBL, %d0
        tst.w   irqoff(%a2)
        beq     1f
        moveq   #0, %d0
1:
        move.l  %d0, INTR_MASK(%a5)
        bra     from32

/* fill page 0 with a 50% dither for the current depth */
gray_page:
        bsr     mode_entry
        move.l  devbase(%a2), %a4
        add.l   m_offset(%a6), %a4
        move.w  m_depth(%a6), %d0
        move.l  #0xaaaaaaaa, %d3
        cmp.w   #1, %d0
        beq     1f
        move.l  #0xcccccccc, %d3
        cmp.w   #2, %d0
        beq     1f
        move.l  #0xf0f0f0f0, %d3
        cmp.w   #4, %d0
        beq     1f
        move.l  #0xff00ff00, %d3
1:
        move.w  height(%pc), %d1
        subq.w  #1, %d1
2:
        move.l  %a4, %a5
        move.w  m_rowbytes(%a6), %d2
        lsr.w   #2, %d2
        subq.w  #1, %d2
3:
        move.l  %d3, (%a5)+
        dbra    %d2, 3b
        not.l   %d3
        add.w   m_rowbytes(%a6), %a4
        dbra    %d1, 2b
        rts

/* D1-D3: red, green, blue; all three become (77 r + 151 g + 28 b) / 256 */
luminance:
        and.w   #0xff, %d1
        and.w   #0xff, %d2
        and.w   #0xff, %d3
        mulu.w  #77, %d1
        mulu.w  #151, %d2
        mulu.w  #28, %d3
        add.l   %d2, %d1
        add.l   %d3, %d1
        lsr.l   #8, %d1
        move.b  %d1, %d2
        move.b  %d1, %d3
        rts

/* switch to 32-bit addressing, the previous mode in D7 */
to32:
        moveq   #1, %d0
        _SwapMMUMode
        move.b  %d0, %d7
        rts

from32:
        move.b  %d7, %d0
        _SwapMMUMode
        rts

/*
 * VBL interrupt from the slot: A1 is sqParm, the storage. A0-A3 and D0-D3
 * may be used. Acknowledge, run the slot's VBL tasks, report it handled.
 */
vbl_int:
        move.l  %a1, %a2
        moveq   #1, %d0
        _SwapMMUMode
        move.b  %d0, %d3
        move.l  regbase(%a2), %a0
        move.l  #INTR_VBL, INTR_CLEAR(%a0)
        move.b  %d3, %d0
        _SwapMMUMode
        move.w  slot(%a2), %d0
        _DoVBLTask
        moveq   #1, %d0
        rts
