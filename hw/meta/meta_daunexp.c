/*
 * ImgTec Debug Adapter (DA) UnExpXXX handler: a SWITCH that trapped into the
 * guest's unexpected-exception handler is handled here and the context stepped
 * past it.
 *
 * Copyright (C) 2013 Imagination Technologies Ltd.
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "cpu.h"
#include "accel/tcg/cpu-ldst.h"
#include "hw/meta/meta_switch.h"

#define TYPE_META_DAUNEXP "daunexp"

static int meta_daunexp_handler(MetaSwitchDevice *dev, CPUArchState *env,
                                const uint32_t *args, int nargs,
                                uint32_t *ret, int nret)
{
    const vaddr ctx_off_pc = 4;
    const vaddr ctx_off_stp =
        8 +       /* Flags, SaveMask, CurrPC */
        (8 * 8) + /* DX[8] */
        16;       /* CurrRPT, CurrBPOBITS, CurrMODE, CurrDIVTIME */

    vaddr ctx_stp, pctx = args[0];
    uint32_t signum = args[2];
    uint32_t nested_switch = args[4];
    int mmu_idx = meta_mmu_index(env);
    int retcode;

    if (signum > 3 /* TBID_SIGNUM_SW3 */) {
        /* this UnExpXXX wasn't caused by a SWITCH */
        return -1;
    }

    ctx_stp = cpu_ldl_le_mmuidx_ra(env, pctx + ctx_off_stp, mmu_idx, 0);
    ret[0] = args[0];
    ret[1] = args[1];

    retcode = meta_switch_handle(env, nested_switch, ctx_stp);
    if (!retcode) {
        /* advance context CurrPC */
        uint32_t pc = cpu_ldl_le_mmuidx_ra(env, pctx + ctx_off_pc, mmu_idx, 0);
        MetaISA isa = meta_current_isa_pc(env, pc);
        if (isa == MINIM) {
            uint16_t insn_core = cpu_lduw_le_mmuidx_ra(env, pc, mmu_idx, 0);
            size_t minim_words = meta_minim_decode_size(insn_core);
            pc += 4 * minim_words;
        } else {
            pc += 4;
        }
        cpu_stl_le_mmuidx_ra(env, pctx + ctx_off_pc, pc, mmu_idx, 0);
    }
    return retcode;
}

static int meta_daunexp_initfn(MetaSwitchDevice *dev)
{
    dev->handler = meta_daunexp_handler;
    return 0;
}

static void meta_daunexp_class_initfn(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    MetaSwitchDeviceClass *sc = META_SWITCH_DEVICE_CLASS(klass);

    sc->init = meta_daunexp_initfn;
    sc->subgroup = META_SWITCH_UNEXP;
    dc->user_creatable = false;
}

static const TypeInfo meta_daunexp_info = {
    .name          = TYPE_META_DAUNEXP,
    .parent        = TYPE_META_SWITCH_DEVICE,
    .instance_size = sizeof(MetaSwitchDevice),
    .class_init    = meta_daunexp_class_initfn,
};

static void meta_daunexp_register(void)
{
    type_register_static(&meta_daunexp_info);
}

type_init(meta_daunexp_register)
