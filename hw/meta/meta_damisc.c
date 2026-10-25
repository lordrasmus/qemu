/*
 * ImgTec Debug Adapter (DA) misc op driver
 *
 * Copyright (C) 2013 Imagination Technologies Ltd.
 *
 * Authors:
 *  James Hogan <james.hogan@imgtec.com>
 *  Paul Burton <paul.burton@imgtec.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "cpu.h"
#include "chardev/char.h"
#include "chardev/char-fe.h"
#include "hw/core/qdev-properties.h"
#include "system/runstate.h"
#include "system/system.h"
#include "accel/tcg/cpu-ldst.h"
#include "exec/helper-proto.h"
#include "hw/meta/meta_switch.h"
#include "hw/meta/meta_damisc.h"

#define CHAN_BUFSZ 65536

typedef struct MetaDAChannel {
    CharFrontend chr;

    /* circular buffer to hold input data */
    uint8_t buf_in[CHAN_BUFSZ];
    uint32_t buf_in_start;
    uint32_t buf_in_count;
} MetaDAChannel;

OBJECT_DECLARE_SIMPLE_TYPE(MetaDAMiscState, META_DAMISC)

struct MetaDAMiscState {
    MetaSwitchDevice dev;

    int32_t exit_threads;
    uint32_t exit_count;
    uint32_t exit_code;

    uint32_t nchannels;
    MetaDAChannel *chans;
};

/* SWITCH subgroup 3 operations */
enum {
    OP_EXIT           = 0x0,
    OP_CONT           = 0x1,
    OP_DEBUGRUNSCRIPT = 0x2,
    OP_CHANBIOS       = 0x3,
    OP_HOSTCCB        = 0x4,
};

/* CHANBIOS operations */
enum {
    CHANBIOS_INCHR    = 0x1,
    CHANBIOS_OUTCHR   = 0x2,
    CHANBIOS_RDBUF    = 0x3,
    CHANBIOS_WRBUF    = 0x4,
    CHANBIOS_RDSTAT   = 0x5,
};

/* channel error codes */
enum {
    CHAN_AOK = 0x0,
    CHAN_ERR = 0x1,
    CHAN_BAD = 0x2,
    CHAN_PRM = 0x3,
    CHAN_ADR = 0x4,
    CHAN_CNT = 0x5,
    CHAN_CBF = 0x6,
    CHAN_CBE = 0x7,
    CHAN_BSY = 0x8,
};

static bool chan_open(MetaDAChannel *chan)
{
    return qemu_chr_fe_backend_connected(&chan->chr);
}

static int meta_damisc_op_exit(MetaDAMiscState *dm, CPUArchState *env,
                               const uint32_t *args)
{
    uint32_t ret_code = args[0];

    qemu_log_mask(CPU_LOG_INT, "T%d Exit, ret %d\n", env->thread_num,
                  ret_code);
    helper_thread_stop(env);

    /* count the number of threads exited */
    dm->exit_count++;

    /* check whether this is the new overall return code */
    if (ret_code > dm->exit_code) {
        dm->exit_code = ret_code;
    }

    /* check if qemu should exit */
    if (dm->exit_threads == dm->exit_count) {
        qemu_system_shutdown_request_with_code(SHUTDOWN_CAUSE_GUEST_SHUTDOWN,
                                               dm->exit_code);
    }

    return 0;
}

int meta_damisc_chan_errno(int err)
{
    switch (err) {
    case CHAN_AOK: return 0;
    case CHAN_ERR: return EIO;
    case CHAN_BAD: return EBADF;
    case CHAN_PRM: return EINVAL;
    case CHAN_ADR: return EFAULT;
    case CHAN_CNT: return EINVAL;
    case CHAN_CBF: return ENOSPC;
    case CHAN_CBE: return ESPIPE;
    case CHAN_BSY: return EBUSY;
    default: return EINVAL;
    }
}

static void chan_advance_buf_in(MetaDAChannel *chan, size_t sz)
{
    chan->buf_in_start += sz;
    chan->buf_in_start %= sizeof(chan->buf_in);
}

ssize_t meta_damisc_chan_read(MetaSwitchDevice *dev, CPUArchState *env,
                              int chan_idx, uint8_t *buf, size_t sz)
{
    MetaDAMiscState *dm = META_DAMISC(dev);
    MetaDAChannel *chan;
    uint8_t *buf_curr = buf;
    size_t sz_curr, rem;

    if (chan_idx >= dm->nchannels || !chan_open(&dm->chans[chan_idx])) {
        return -CHAN_BAD;
    }
    chan = &dm->chans[chan_idx];

    /* calculate actual size of read */
    rem = MIN(sz, chan->buf_in_count);
    if (!rem) {
        return -CHAN_CBE;
    }

    while (rem) {
        /* read at most to the end of the buffer */
        sz_curr = MIN(sizeof(chan->buf_in) - chan->buf_in_start, rem);

        /* copy the data */
        memcpy(buf_curr, &chan->buf_in[chan->buf_in_start], sz_curr);

        /* advance */
        chan_advance_buf_in(chan, sz_curr);
        chan->buf_in_count -= sz_curr;
        buf_curr += sz_curr;
        rem -= sz_curr;
    }
    qemu_chr_fe_accept_input(&chan->chr);

    return buf_curr - buf;
}

ssize_t meta_damisc_chan_write(MetaSwitchDevice *dev, CPUArchState *env,
                               int chan_idx, const uint8_t *buf, size_t sz)
{
    MetaDAMiscState *dm = META_DAMISC(dev);

    if (chan_idx >= dm->nchannels || !chan_open(&dm->chans[chan_idx])) {
        return -CHAN_BAD;
    }

    return qemu_chr_fe_write_all(&dm->chans[chan_idx].chr, buf, sz);
}

static int chan_canread_handler(void *opaque)
{
    MetaDAChannel *chan = opaque;
    return sizeof(chan->buf_in) - chan->buf_in_count;
}

static void chan_read_handler(void *opaque, const uint8_t *buf, int size)
{
    MetaDAChannel *chan = opaque;
    const uint8_t *curr = buf;
    int sz_curr, rem = size;
    size_t off_curr;

    while (rem) {
        /* find base of the current chunk */
        off_curr = chan->buf_in_start + chan->buf_in_count;
        off_curr %= sizeof(chan->buf_in);

        /* copy at most to the end of the buffer */
        sz_curr = MIN(sizeof(chan->buf_in) - off_curr, rem);
        memcpy(&chan->buf_in[off_curr], curr, sz_curr);

        /* advance */
        curr += sz_curr;
        rem -= sz_curr;
        chan->buf_in_count += sz_curr;
    }
}

static int meta_damisc_op_chanbios_inchr(MetaDAMiscState *dm,
                                         CPUArchState *env,
                                         const uint32_t *args,
                                         uint32_t *ret)
{
    uint32_t chan_idx = args[0];
    MetaDAChannel *chan;
    int err = CHAN_ERR;
    uint8_t chr = 0;

    /* check for a valid channel */
    if (chan_idx >= dm->nchannels) {
        err = CHAN_BAD;
        goto out;
    }
    chan = &dm->chans[chan_idx];

    /* trivial no-op case */
    if (!chan_open(chan)) {
        err = CHAN_AOK;
        goto out;
    }

    /* check there's a character to read */
    if (!chan->buf_in_count) {
        err = CHAN_CBE;
        goto out;
    }

    /* read character */
    chr = chan->buf_in[chan->buf_in_start];
    chan_advance_buf_in(chan, 1);
    chan->buf_in_count--;
    qemu_chr_fe_accept_input(&chan->chr);

    err = CHAN_AOK;
out:
    ret[0] = (err << 16) | chr;
    ret[1] = 0;
    return 0;
}

static int meta_damisc_op_chanbios_outchr(MetaDAMiscState *dm,
                                          CPUArchState *env,
                                          const uint32_t *args,
                                          uint32_t *ret)
{
    uint32_t chan_idx = args[0];
    MetaDAChannel *chan;
    uint8_t chr = args[1];
    int err = CHAN_ERR;

    /* check for a valid channel */
    if (chan_idx >= dm->nchannels) {
        err = CHAN_BAD;
        goto out;
    }
    chan = &dm->chans[chan_idx];

    /* trivial no-op case */
    if (!chan_open(chan)) {
        err = CHAN_AOK;
        goto out;
    }

    /* perform the write */
    qemu_chr_fe_write_all(&chan->chr, &chr, 1);
    err = CHAN_AOK;
out:
    ret[0] = err;
    ret[1] = 0;
    return 0;
}

static int meta_damisc_op_chanbios_rdbuf(MetaDAMiscState *dm,
                                         CPUArchState *env,
                                         const uint32_t *args,
                                         uint32_t *ret)
{
    uint32_t chan_idx = args[0];
    MetaDAChannel *chan;
    uint32_t size = args[1];
    vaddr pbuf = args[2];
    vaddr pread = args[3];
    vaddr pcurr = pbuf;
    size_t nrem, ncurr;
    uint8_t *pstart;
    int err = CHAN_ERR;

    /* check for a valid channel */
    if (chan_idx >= dm->nchannels) {
        err = CHAN_BAD;
        goto out;
    }
    chan = &dm->chans[chan_idx];

    /* check size */
    if (!size) {
        err = CHAN_PRM;
        goto out;
    } else if (size > CHAN_BUFSZ) {
        err = CHAN_CNT;
        goto out;
    }

    /* trivial no-op case */
    if (!chan_open(chan)) {
        err = CHAN_AOK;
        goto out;
    }

    /* calculate actual size of read */
    nrem = MIN(size, chan->buf_in_count);
    if (!nrem) {
        err = CHAN_CBE;
        goto out;
    }

    /* read to memory */
    while (nrem) {
        /* read at most to the end of the buffer */
        ncurr = sizeof(chan->buf_in) - chan->buf_in_start;
        ncurr = MIN(ncurr, nrem);

        /* do the read */
        pstart = &chan->buf_in[chan->buf_in_start];
        if (cpu_memory_rw_debug(env_cpu(env), pcurr, pstart, ncurr, true) < 0) {
            err = CHAN_ADR;
            goto out;
        }

        /* advance */
        pcurr += ncurr;
        nrem -= ncurr;
        chan_advance_buf_in(chan, ncurr);
        chan->buf_in_count -= ncurr;
    }
    qemu_chr_fe_accept_input(&chan->chr);

    err = CHAN_AOK;
out:
    if (pread) {
        /* *pread = pcurr - pbuf */
        cpu_stl_le_mmuidx_ra(env, pread, pcurr - pbuf, meta_mmu_index(env), 0);
    }
    ret[0] = err;
    ret[1] = 0;
    return 0;
}

static int meta_damisc_op_chanbios_wrbuf(MetaDAMiscState *dm,
                                         CPUArchState *env,
                                         const uint32_t *args,
                                         uint32_t *ret)
{
    uint32_t chan_idx = args[0];
    MetaDAChannel *chan;
    uint32_t size = args[1];
    vaddr pbuf = args[2];
    vaddr pwrote = args[3];
    int nwrote = 0, err = CHAN_ERR;
    uint8_t *buf;

    /* check for a valid channel */
    if (chan_idx >= dm->nchannels) {
        err = CHAN_BAD;
        goto out;
    }
    chan = &dm->chans[chan_idx];

    /* check size */
    if (!size) {
        err = CHAN_PRM;
        goto out;
    } else if (size > CHAN_BUFSZ) {
        err = CHAN_CNT;
        goto out;
    }

    /* trivial no-op case */
    if (!chan_open(chan)) {
        /* pretend to have written all the bytes */
        nwrote = size;
        err = CHAN_AOK;
        goto out;
    }

    /* allocate buf & read into it */
    buf = g_malloc(size);
    if (cpu_memory_rw_debug(env_cpu(env), pbuf, buf, size, false) < 0) {
        err = CHAN_ADR;
        goto out_free;
    }

    /* perform the write */
    nwrote = qemu_chr_fe_write_all(&chan->chr, buf, size);

    err = CHAN_AOK;
out_free:
    g_free(buf);
out:
    if (pwrote) {
        /* *pwrote = nwrote */
        cpu_stl_le_mmuidx_ra(env, pwrote, nwrote, meta_mmu_index(env), 0);
    }
    ret[0] = err;
    ret[1] = 0;
    return 0;
}

static int meta_damisc_op_chanbios_rdstat(MetaDAMiscState *dm,
                                          CPUArchState *env,
                                          const uint32_t *args,
                                          uint32_t *ret)
{
    uint32_t chan_idx = args[0];
    vaddr pstatus = args[3];
    int mmu_idx = meta_mmu_index(env);
    int err = CHAN_ERR;

    /* check for a valid channel */
    if (chan_idx >= dm->nchannels) {
        err = CHAN_BAD;
        goto out;
    }

    /* write status struct to memory */
    cpu_stl_le_mmuidx_ra(env, pstatus, dm->chans[chan_idx].buf_in_count,
                         mmu_idx, 0);
    cpu_stl_le_mmuidx_ra(env, pstatus + 4, CHAN_BUFSZ, mmu_idx, 0);
    err = CHAN_AOK;
out:
    ret[0] = err;
    ret[1] = 0;
    return 0;
}

static int meta_damisc_op_chanbios(MetaDAMiscState *dm, CPUArchState *env,
                                   const uint32_t *args, uint32_t *ret)
{
    uint32_t fn = args[0];

    switch (fn) {
    case CHANBIOS_INCHR:
        return meta_damisc_op_chanbios_inchr(dm, env, args + 1, ret);

    case CHANBIOS_OUTCHR:
        return meta_damisc_op_chanbios_outchr(dm, env, args + 1, ret);

    case CHANBIOS_RDBUF:
        return meta_damisc_op_chanbios_rdbuf(dm, env, args + 1, ret);

    case CHANBIOS_WRBUF:
        return meta_damisc_op_chanbios_wrbuf(dm, env, args + 1, ret);

    case CHANBIOS_RDSTAT:
        return meta_damisc_op_chanbios_rdstat(dm, env, args + 1, ret);

    default:
        qemu_log_mask(LOG_UNIMP, "Unhandled CHANBIOS function %d\n", fn);
        ret[0] = CHAN_BAD;
        ret[1] = 0;
        return 0;
    }
}

static int meta_damisc_handler(MetaSwitchDevice *dev, CPUArchState *env,
                               const uint32_t *args, int nargs,
                               uint32_t *ret, int nret)
{
    MetaDAMiscState *dm = META_DAMISC(dev);

    if (nargs != 6) {
        return -1;
    }

    switch (args[5]) {
    case OP_EXIT:
        return meta_damisc_op_exit(dm, env, args);

    case OP_CHANBIOS:
        if (nret != 2) {
            return -1;
        }
        return meta_damisc_op_chanbios(dm, env, args, ret);

    default:
        qemu_log_mask(LOG_UNIMP, "DAMisc unhandled op %d\n", args[5]);
        return -1;
    }
}

/*
 * Channel n is the chardev "chan<n>".  Without one, channel 1 (the Linux
 * console, ttyDA1) falls back to the first serial port, so -nographic and
 * -serial work as usual.
 */
static int meta_damisc_initfn(MetaSwitchDevice *dev)
{
    MetaDAMiscState *dm = META_DAMISC(dev);
    Chardev *chr;
    char label[32];
    int i;

    dev->handler = meta_damisc_handler;

    dm->chans = g_new0(MetaDAChannel, dm->nchannels);

    for (i = 0; i < dm->nchannels; i++) {
        snprintf(label, sizeof(label), "chan%d", i);
        chr = qemu_chr_find(label);
        if (!chr && i == 1) {
            chr = serial_hd(0);
        }
        if (!chr) {
            continue;
        }
        if (!qemu_chr_fe_init(&dm->chans[i].chr, chr, NULL)) {
            continue;
        }

        /* setup input */
        qemu_chr_fe_set_handlers(&dm->chans[i].chr,
                                 chan_canread_handler,
                                 chan_read_handler,
                                 NULL, NULL,
                                 &dm->chans[i], NULL, true);
        qemu_chr_fe_accept_input(&dm->chans[i].chr);
    }

    return 0;
}

static const Property meta_damisc_properties[] = {
    DEFINE_PROP_INT32("exit_threads", MetaDAMiscState, exit_threads, -1),
    DEFINE_PROP_UINT32("nchannels", MetaDAMiscState, nchannels, 6),
};

static void meta_damisc_class_initfn(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    MetaSwitchDeviceClass *sc = META_SWITCH_DEVICE_CLASS(klass);

    sc->init = meta_damisc_initfn;
    sc->subgroup = META_SWITCH_MISC;
    device_class_set_props(dc, meta_damisc_properties);
    dc->user_creatable = false;
}

static const TypeInfo meta_damisc_info = {
    .name          = TYPE_META_DAMISC,
    .parent        = TYPE_META_SWITCH_DEVICE,
    .instance_size = sizeof(MetaDAMiscState),
    .class_init    = meta_damisc_class_initfn,
};

static void meta_damisc_register(void)
{
    type_register_static(&meta_damisc_info);
}

type_init(meta_damisc_register)
