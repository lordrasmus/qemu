/*
 * ImgTec Debug Adapter (DA) misc op driver: channels for the other DA devices
 *
 * Copyright (C) 2013 Imagination Technologies Ltd.
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 */

#ifndef HW_META_META_DAMISC_H
#define HW_META_META_DAMISC_H

#include "hw/meta/meta_switch.h"

#define TYPE_META_DAMISC "damisc"

ssize_t meta_damisc_chan_read(MetaSwitchDevice *dev, CPUArchState *env,
                              int chan_idx, uint8_t *buf, size_t sz);
ssize_t meta_damisc_chan_write(MetaSwitchDevice *dev, CPUArchState *env,
                               int chan_idx, const uint8_t *buf, size_t sz);

int meta_damisc_chan_errno(int err);

#endif
