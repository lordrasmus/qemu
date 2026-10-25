#include "qemu/osdep.h"
#include "qemu/log.h"
#include "instruction.h"

/* Operand 2 Replace (O2R) mapping */
static MetaUnit meta_o2r_map[4] = {
    META_UNIT_A1,
    META_UNIT_D0,
    META_UNIT_RA,
    META_UNIT_A0,
};

/* 2 bit base unit (BU) mapping */
static MetaUnit meta_bu_map[4] = {
    [0x1] = META_UNIT_D0,
    [0x2] = META_UNIT_D1,
    [0x3] = META_UNIT_A0,
    [0x0] = META_UNIT_A1,
};

static inline uint32_t bits(uint32_t val, int start, int sz)
{
    return (val >> start) & ((1 << sz) - 1);
}

static bool is_d_unit(MetaUnit u)
{
    return (u == META_UNIT_D0) || (u == META_UNIT_D1);
}

static MetaUnit meta_decode_o2r(MetaUnit us1, int *rs2 /* in out */)
{
    MetaUnit u;
    int us = (*rs2 >> 3) & 0x3;

    *rs2 &= 0x7;
    u = meta_o2r_map[us];
    if (u == us1) {
        u = META_UNIT_D1;
    }
    if (u == META_UNIT_RA) {
        *rs2 |= 0x10;
    }
    return u;
}

static int meta_decode_fpu_du_mov(MetaInstruction *inst)
{
    MetaInstructionUnitToUnit *utu = &inst->unittounit;

    inst->op = OP_UNITTOUNIT;
    inst->fx = true;
    utu->swap = utu->kick = utu->defr = false;
    utu->width = WIDTH_32B + bits(inst->raw, 4, 1);

    inst->dst.type = DST_REG;
    inst->src1.type = SRC_REG;
    inst->src1.u = inst->dst.u = META_UNIT_D0 + bits(inst->raw, 24, 1);
    inst->dst.i = bits(inst->raw, 19, 5);
    inst->src1.i = bits(inst->raw, 9, 5);

    if (bits(inst->raw, 7, 2) == 0x2) {
        /* F MOV Dx.n, Fx.n */
        inst->src1.u = META_UNIT_FX;
        inst->src1.i &= 0xf;

        if (utu->width == WIDTH_64B) {
            inst->src2 = inst->src1;
            inst->src2.i++;
        }
    } else {
        /* F MOV Fx.n, Dx.n */
        inst->dst.u = META_UNIT_FX;
        inst->dst.i &= 0xf;

        if (utu->width == WIDTH_64B) {
            inst->src2 = inst->src1;
            inst->src2.u = meta_unit_partner(inst->src1.u);
        }
    }

    return 0;
}

static int meta_decode_duaddsub(MetaInstruction *inst)
{
    MetaInstructionDUAddSub *as = &inst->duaddsub;
    uint32_t major = bits(inst->raw, 28, 4);
    uint32_t variant = bits(inst->raw, 25, 2);
    bool o2r, ac, ar;

    inst->op = OP_DUADDSUB;

    if (((variant == 0) && bits(inst->raw, 8, 1)) ||
        ((variant == 1) && (bits(inst->raw, 0, 2) == 0x3)) ||
        ((variant == 2) && (bits(inst->raw, 5, 4) == 0xf))) {
        if (major ||
            (bits(inst->raw, 25, 3) != 0x4) ||
            !bits(inst->raw, 6, 1)) {
            /* DSP */
            inst->dsp = true;
        } else {
            /* FP, handled below */
        }
    } else {
        /* GP */
        inst->dsp = false;
    }

    if (bits(inst->raw, 6, 1) &&
        bits(inst->raw, 27, 1) &&
        bits(inst->raw, 8, 1) &&
        !variant && !major) {
        /* F MOV */
        return meta_decode_fpu_du_mov(inst);
    }

    as->d.mod = false;
    as->d.pshift = false;
    as->d.split8 = inst->dsp && (major > 0x1);
    as->d.split8_src1 = as->d.split8 && bits(inst->raw, 7, 1);
    as->d.split8_upper = as->d.split8 && bits(inst->raw, 2, 1);

    o2r = (variant == 0 || variant == 2) &&
         bits(inst->raw, 0, 1);

    ac = inst->dsp && (variant == 0 || variant == 3) &&
         bits(inst->raw, 6, 1);

    ar = inst->dsp && (variant == 0 || variant == 3) &&
         !as->d.split8 && bits(inst->raw, 7, 1);

    as->sub = !!(major & 0x1);
    as->setflags = bits(inst->raw, 27, 1);
    as->top = false;

    inst->src1.u = inst->src2.u = inst->dst.u =
        META_UNIT_D0 + bits(inst->raw, 24, 1);
    inst->dst.i = bits(inst->raw, 19, 5);
    inst->src1.i = bits(inst->raw, 14, 5);
    inst->src2.i = bits(inst->raw, 9, 5);
    inst->src1.type = inst->src2.type = SRC_REG;
    inst->dst.type = DST_REG;

    if (o2r) {
        inst->src2.u = meta_decode_o2r(inst->src1.u, &inst->src2.i);
    }

    if (ac) {
        inst->dst.type = DST_ACCUM;
        inst->src1.type = inst->src2.type = SRC_ACCUM;
        inst->src2.u = meta_unit_partner(inst->src1.u);
        inst->dst.i = inst->src1.i = inst->src2.i = inst->dst.i & 0x3;
    } else if (variant == 0) {
        bool o1z = bits(inst->raw, 2, 1) && !as->d.split8;

        if (o1z) {
            inst->src1.type = SRC_IMM;
            inst->src1.i = 0;
        }

        as->d.pshift = inst->dsp && bits(inst->raw, 5, 1);
        inst->dual = inst->dsp && bits(inst->raw, 4, 1);
        as->d.mod = inst->dsp && bits(inst->raw, 3, 1) && !as->d.split8;
    } else if (variant == 1) {
        bool o1z = bits(inst->raw, 2, 1) && !inst->dsp;
        bool se = bits(inst->raw, 1, 1);
        as->top = bits(inst->raw, 0, 1) && !se;

        if (o1z) {
            inst->src1.type = SRC_IMM;
            inst->src1.i = 0;
        } else {
            inst->src1.i = inst->dst.i;
        }

        inst->src2.type = SRC_IMM;
        inst->src2.i = bits(inst->raw, 3, 16);

        if (se) {
            inst->src2.i = ((int32_t)inst->src2.i << 16) >> 16;
        }
        if (as->top) {
            inst->src2.i <<= 16;
        }

        inst->dual = inst->dsp && bits(inst->raw, 2, 1);
    } else if (variant == 2) {
        inst->cc = bits(inst->raw, 1, 4);

        if (!inst->dsp) {
            inst->dst.u = bits(inst->raw, 5, 4);
        }
    } else if (variant == 3) {
        bool ca = bits(inst->raw, 5, 1);

        inst->src2.type = SRC_IMM;
        inst->src2.i = bits(inst->raw, 6, 8);

        if (ca) {
            inst->dst.u = bits(inst->raw, 1, 4);
        } else {
            inst->cc = bits(inst->raw, 1, 4);
        }
    }

    if (inst->dsp) {
        if ((inst->dst.type == DST_REG) &&
            is_d_unit(inst->dst.u) &&
            (inst->dst.i & 0x10) &&
            !bits(inst->raw, 26, 1)) {
            /* non-conditional may target DSP RAM */
            inst->dst.type = DST_DSPRAM;
        }

        if ((inst->src1.type == SRC_REG) &&
            is_d_unit(inst->src1.u) &&
            (inst->src1.i & 0x10) &&
            !as->d.split8_src1) {
            inst->src1.type = SRC_DSPRAM;
        }

        if (ar) {
            /* source an accumulator register */
            inst->src2.type = SRC_ACCUM;
            inst->src2.i &= 0x3;
        } else if (o2r && inst->dual) {
            /* dual unit with O2R flag sources the read pipeline */
            inst->src2.type = SRC_REG;
            inst->src2.u = META_UNIT_RA;
            inst->src2.i = META_RA;
        }

        if ((inst->src2.type == SRC_REG) &&
            is_d_unit(inst->src2.u) &&
            (inst->src2.i & 0x10) &&
            !as->d.split8) {
            inst->src2.type = SRC_DSPRAM;
        }
    }

    if ((variant == 0) && bits(inst->raw, 1, 1)) {
        return -1;
    }

    return 0;
}

static int meta_decode_mul(MetaInstruction *inst)
{
    MetaInstructionMul *mul = &inst->mul;
    uint32_t major = bits(inst->raw, 28, 4);
    bool conditional = bits(inst->raw, 26, 1);
    bool immediate = bits(inst->raw, 25, 1);
    bool o2r;

    inst->op = OP_MUL;

    if ((!conditional && !immediate && bits(inst->raw, 8, 1)) ||
        (!conditional && immediate && (bits(inst->raw, 0, 2) == 0x3))) {
        /* DSP */
        inst->dsp = true;
    } else {
        /* GP */
        inst->dsp = false;
    }

    inst->src1.type = inst->src2.type = SRC_REG;
    inst->dst.type = DST_REG;
    inst->src1.u = inst->src2.u = inst->dst.u =
        META_UNIT_D0 + bits(inst->raw, 24, 1);
    inst->dst.i = bits(inst->raw, 19, 5);
    inst->src1.i = bits(inst->raw, 14, 5);
    inst->src2.i = bits(inst->raw, 9, 5);

    mul->sign = false;
    mul->l2 = false;
    mul->top = false;

    mul->d.split8 = inst->dsp && (major == 0x4);
    mul->d.split8_src1 = mul->d.split8 && bits(inst->raw, 7, 1);
    mul->d.split8_upper = mul->d.split8 && bits(inst->raw, 2, 1);

    mul->d.mod = false;
    mul->d.acc = 0;

    o2r = !immediate && bits(inst->raw, 0, 1);

    if (o2r) {
        inst->src2.u = meta_decode_o2r(inst->src1.u, &inst->src2.i);
    }

    if (conditional) {
        /* conditional */
        bool ca = bits(inst->raw, 5, 1);

        if (ca) {
            inst->dst.u = bits(inst->raw, 1, 4);
        } else {
            inst->cc = bits(inst->raw, 1, 4);
        }
    }

    if (conditional && immediate) {
        inst->src2.type = SRC_IMM;
        inst->src2.i = bits(inst->raw, 6, 8);

        mul->l2 = bits(inst->raw, 0, 1);
    } else if (conditional) {
        mul->l2 = bits(inst->raw, 6, 1);
    } else if (immediate) {
        bool se = bits(inst->raw, 1, 1) || inst->dsp;
        mul->top = bits(inst->raw, 0, 1) && !inst->dsp;

        mul->l2 = bits(inst->raw, 2, 1) && !inst->dsp;
        inst->dual = bits(inst->raw, 2, 1) && inst->dsp;

        inst->src1.i = inst->dst.i;

        inst->src2.type = SRC_IMM;
        inst->src2.i = bits(inst->raw, 3, 16);

        if (se) {
            inst->src2.i = ((int32_t)inst->src2.i << 16) >> 16;
        }
        if (mul->top) {
            inst->src2.i <<= 16;
        }

        mul->sign = inst->dsp;
    } else {
        if (!inst->dsp) {
            /* not in the TRM, but in the simulator :s */
            inst->cc = bits(inst->raw, 1, 4);
        }

        inst->dual = bits(inst->raw, 4, 1) && inst->dsp;
        mul->l2 = bits(inst->raw, 6, 1) && !inst->dsp;
        mul->sign = !bits(inst->raw, 6, 1) && inst->dsp;

        if (inst->dsp && !mul->d.split8) {
            mul->d.mod = bits(inst->raw, 3, 1);
            mul->d.acc = (bits(inst->raw, 7, 1) << 1) | /* A2 */
                          bits(inst->raw, 2, 1);        /* A1 */
        }
    }

    if ((inst->dst.u != META_UNIT_D0) &&
        (inst->dst.u != META_UNIT_D1) &&
        (inst->dst.u != META_UNIT_A0) &&
        (inst->dst.u != META_UNIT_A1)) {
        /* 32-bit multiply only allowed when targetting [AD][01] */
        mul->l2 = false;
    }

    if (inst->dsp) {
        if (mul->d.acc) {
            inst->dst.type = DST_ACCUM;
            inst->dst.i &= 0x3;
        } else if ((inst->dst.type == DST_REG) &&
                   (inst->dst.i & 0x10)) {
            /* may target DSP RAM */
            inst->dst.type = DST_DSPRAM;
        }

        if ((inst->src1.type == SRC_REG) &&
            is_d_unit(inst->src1.u) &&
            (inst->src1.i & 0x10)) {
            /* may source DSP RAM */
            inst->src1.type = SRC_DSPRAM;
        }

        if (o2r && inst->dual) {
            /* dual unit with O2R flag sources the read pipeline */
            inst->src2.type = SRC_REG;
            inst->src2.u = META_UNIT_RA;
            inst->src2.i = META_RA;
        }

        if ((inst->src2.type == SRC_REG) &&
            is_d_unit(inst->src2.u) &&
            (inst->src2.i & 0x10) &&
            !immediate &&
            !mul->d.split8) {
            /* non-immediate non-split8 may source DSP RAM */
            inst->src2.type = SRC_DSPRAM;
        }
    }

    if (!conditional && !immediate && bits(inst->raw, 1, 1)) {
        return -1;
    }

    return 0;
}

static int meta_decode_andorxor(MetaInstruction *inst)
{
    MetaInstructionAndOrXor *aox = &inst->andorxor;
    uint32_t major = bits(inst->raw, 28, 4);
    uint32_t variant = bits(inst->raw, 25, 2);
    bool o2r;

    inst->op = OP_ANDORXOR;

    if (((variant == 0) && bits(inst->raw, 8, 1)) ||
        ((variant == 1) && (bits(inst->raw, 0, 2) == 0x3)) ||
        ((variant == 2) && (bits(inst->raw, 6, 3) == 0x7))) {
        /* DSP */
        inst->dsp = true;
    } else {
        /* GP */
        inst->dsp = false;
    }

    if ((variant == 0) &&
        bits(inst->raw, 8, 1) &&
        bits(inst->raw, 3, 1)) {
        /* really split-8 arithmetic */
        if (major == 4) {
            return meta_decode_mul(inst);
        } else {
            return meta_decode_duaddsub(inst);
        }
    }

    switch (major) {
    case 0x2:
        aox->op = OP_AND;
        break;

    case 0x3:
        aox->op = OP_OR;
        break;

    case 0x4:
        aox->op = OP_XOR;
        break;

    default:
        return -1;
    }

    aox->setflags = bits(inst->raw, 27, 1);
    aox->mask = false;
    aox->top = false;

    inst->src1.type = inst->src2.type = SRC_REG;
    inst->dst.type = DST_REG;
    inst->src1.u = inst->src2.u = inst->dst.u =
        META_UNIT_D0 + bits(inst->raw, 24, 1);
    inst->dst.i = bits(inst->raw, 19, 5);
    inst->src1.i = bits(inst->raw, 14, 5);
    inst->src2.i = bits(inst->raw, 9, 5);

    o2r = (variant == 0 || variant == 2) &&
         bits(inst->raw, 0, 1);

    if (o2r) {
        inst->src2.u = meta_decode_o2r(inst->src1.u, &inst->src2.i);
    }

    if (variant == 0) {
        inst->dual = inst->dsp && bits(inst->raw, 4, 1);
    } else if (variant == 1) {
        aox->mask = bits(inst->raw, 2, 1) && !inst->dsp;
        bool se = bits(inst->raw, 1, 1);
        aox->top = bits(inst->raw, 0, 1) && !se;

        inst->src1.i = inst->dst.i;

        inst->src2.type = SRC_IMM;
        inst->src2.i = bits(inst->raw, 3, 16);

        if (se) {
            inst->src2.i = ((int32_t)inst->src2.i << 16) >> 16;
        }
        if (aox->top) {
            inst->src2.i <<= 16;
        }
        if (aox->mask) {
            inst->src2.i |= aox->top ? 0xffff : 0xffff0000;
        }

        inst->dual = inst->dsp && bits(inst->raw, 2, 1);
    } else if (variant == 2) {
        inst->cc = bits(inst->raw, 1, 4);

        if (!inst->dsp) {
            inst->dst.u = bits(inst->raw, 5, 4);
        }
    } else if (variant == 3) {
        bool ca = bits(inst->raw, 5, 1);

        inst->src2.type = SRC_IMM;
        inst->src2.i = bits(inst->raw, 6, 8);

        if (ca) {
            inst->dst.u = bits(inst->raw, 1, 4);
        } else {
            inst->cc = bits(inst->raw, 1, 4);
        }
    }

    if (inst->dsp) {
        if ((inst->dst.u == inst->src1.u) &&
            (inst->dst.i & 0x10) &&
            !bits(inst->raw, 26, 1)) {
            /* non-conditional may target DSP RAM within execution unit */
            inst->dst.type = DST_DSPRAM;
        }

        if ((inst->src1.type == SRC_REG) &&
            is_d_unit(inst->src1.u) &&
            (inst->src1.i & 0x10)) {
            /* non-immediate may source DSP RAM */
            inst->src1.type = SRC_DSPRAM;
        }

        if (o2r && inst->dual) {
            /* dual unit with O2R flag sources the read pipeline */
            inst->src2.type = SRC_REG;
            inst->src2.u = META_UNIT_RA;
            inst->src2.i = META_RA;
        }

        if ((inst->src2.type == SRC_REG) &&
            is_d_unit(inst->src2.u) &&
            (inst->src2.i & 0x10) &&
            !bits(inst->raw, 25, 1) &&
            !bits(inst->raw, 0, 1)) {
            /* non-immediate may source DSP RAM */
            inst->src2.type = SRC_DSPRAM;
        }
    }

    if ((variant == 0) && bits(inst->raw, 1, 1)) {
        return -1;
    }

    return 0;
}

static int meta_decode_shift(MetaInstruction *inst)
{
    MetaInstructionShift *sh = &inst->shift;
    bool conditional = bits(inst->raw, 26, 1);
    bool immediate = bits(inst->raw, 25, 1);

    inst->op = OP_SHIFT;
    inst->dsp = bits(inst->raw, 8, 1);

    sh->arithmetic = bits(inst->raw, 7, 1);
    sh->right = bits(inst->raw, 6, 1);
    sh->setflags = bits(inst->raw, 27, 1);
    sh->rspp = RSPP_NONE;

    inst->dst.type = DST_REG;
    inst->src1.type = inst->src2.type = SRC_REG;
    inst->dst.u = inst->src1.u = inst->src2.u =
        META_UNIT_D0 + bits(inst->raw, 24, 1);
    inst->dst.i = bits(inst->raw, 19, 5);
    inst->src1.i = bits(inst->raw, 14, 5);
    inst->src2.i = bits(inst->raw, 9, 5);

    if (conditional) {
        bool ca = bits(inst->raw, 5, 1);

        if (ca) {
            inst->dst.u = bits(inst->raw, 1, 4);
        } else {
            inst->cc = bits(inst->raw, 1, 4);
        }
    } else {
        inst->dual = inst->dsp && bits(inst->raw, 4, 1);
        sh->rspp = bits(inst->raw, 2, 2);
    }

    if (immediate) {
        inst->src2.type = SRC_IMM;
        inst->src2.i = bits(inst->raw, 9, 5);
    }

    if (inst->dsp) {
        if ((inst->dst.type == DST_REG) &&
            is_d_unit(inst->dst.u) &&
            (inst->dst.i & 0x10) &&
            !bits(inst->raw, 26, 1)) {
            /* non-conditional may target DSP RAM */
            inst->dst.type = DST_DSPRAM;
        }

        if (bits(inst->raw, 0, 1)) {
            inst->src1.type = SRC_ACCUM;
            inst->src1.i &= 0x3;
        } else if (inst->src1.i & 0x10) {
            inst->src1.type = SRC_DSPRAM;
        }

        if ((inst->src2.type == SRC_REG) &&
            (inst->src2.i & 0x10)) {
            inst->src2.type = SRC_DSPRAM;
        }
    }

    if (!conditional && !immediate && bits(inst->raw, 1, 1)) {
        return -1;
    }

    return 0;
}

static int meta_decode_cmptst(MetaInstruction *inst)
{
    MetaInstructionCmpTst *ct = &inst->cmptst;
    bool conditional = bits(inst->raw, 26, 1);
    bool immediate = bits(inst->raw, 25, 1);
    bool o2r;

    inst->op = OP_CMPTST;
    inst->src1.type = inst->src2.type = SRC_REG;
    inst->dst.u = inst->src1.u = inst->src2.u =
        META_UNIT_D0 + bits(inst->raw, 24, 1);
    inst->dst.i = bits(inst->raw, 19, 5);
    inst->src1.i = bits(inst->raw, 14, 5);
    inst->src2.i = bits(inst->raw, 9, 5);

    if (immediate && !conditional) {
        inst->src1.i = inst->dst.i;
    }

    inst->dsp =
        (!conditional && !immediate && bits(inst->raw, 8, 1)) ||
        (!conditional && immediate && (bits(inst->raw, 0, 2) == 0x3));

    inst->dual = inst->dsp && bits(inst->raw, immediate ? 2 : 4, 1);

    ct->tst = bits(inst->raw, 27, 1);
    ct->ext_op = bits(inst->raw, 2, 4) & 0xb;
    ct->mask = false;
    ct->top = false;
    ct->src2_maxneg = false;

    o2r = !immediate && bits(inst->raw, 0, 1);
    if (o2r) {
        inst->src2.u = meta_decode_o2r(inst->src1.u, &inst->src2.i);
    }

    if (conditional) {
        inst->cc = bits(inst->raw, 1, 4);
    }

    if (conditional || immediate) {
        ct->ext_op = OP_NONE;
    }

    switch (ct->ext_op) {
    case OP_ABS:
    case OP_FFB:
    case OP_NORM:
        inst->src2.type = SRC_NONE;
        inst->dst.type = DST_REG;
        break;

    case OP_MAX:
    case OP_MIN:
    case OP_NMIN:
        inst->dst.type = DST_REG;
        break;

    default:
        break;
    }

    if (conditional && immediate) {
        inst->src2.type = SRC_IMM;
        inst->src2.i = bits(inst->raw, 6, 8);
    } else if (immediate) {
        bool se = bits(inst->raw, 1, 1) || inst->dsp;
        ct->mask = bits(inst->raw, 2, 1) && !inst->dsp;
        ct->top = bits(inst->raw, 0, 1) && !se;

        inst->src2.type = SRC_IMM;
        inst->src2.i = bits(inst->raw, 3, 16);

        if (se) {
            inst->src2.i = ((int32_t)inst->src2.i << 16) >> 16;
        }
        if (ct->top) {
            inst->src2.i <<= 16;
        }
        if (ct->mask) {
            inst->src2.i |= ct->top ? 0x0000ffff : 0xffff0000;
        }

        inst->dual = inst->dsp && bits(inst->raw, 2, 1);
    }

    /* keep a separate src2 for the compare since we may need the
       original later for something like NMIN */
    if (!conditional && !immediate) {
        switch (ct->ext_op) {
        case OP_FFB:
        case OP_NORM:
        case OP_NMIN:
            /* compare against zero */
            ct->src2_cmp.type = SRC_IMM;
            ct->src2_cmp.i = 0;
            ct->tst = false;
            break;

        case OP_ABS:
            /* compare against max negative - we can't decide what
               that is here since it requires knowledge of the current
               arithmetic mode (for split-16) */
            ct->src2_cmp.type = SRC_IMM;
            ct->src2_cmp.i = 0xdeadbeef;
            ct->src2_maxneg = true;
            ct->tst = false;
            break;

        default:
            ct->src2_cmp = inst->src2;
        }
    } else {
        ct->src2_cmp = inst->src2;
    }

    if (inst->dsp) {
        if ((inst->dst.type == DST_REG) &&
            is_d_unit(inst->dst.u) &&
            (inst->dst.i & 0x10)) {
            /* may target DSP RAM */
            inst->dst.type = DST_DSPRAM;
        }

        if ((inst->src1.type == SRC_REG) &&
            is_d_unit(inst->src1.u) &&
            (inst->src1.i & 0x10)) {
            /* may source DSP RAM */
            inst->src1.type = SRC_DSPRAM;
        }

        if (o2r && inst->dual) {
            /* dual unit with O2R flag sources the read pipeline */
            inst->src2.type = SRC_REG;
            inst->src2.u = META_UNIT_RA;
            inst->src2.i = META_RA;
        }

        if ((inst->src2.type == SRC_REG) &&
            is_d_unit(inst->src2.u) &&
            (inst->src2.i & 0x10) &&
            !immediate) {
            /* non-immediate may source DSP RAM */
            inst->src2.type = SRC_DSPRAM;
        }
    }

    if (!conditional && !immediate && bits(inst->raw, 1, 1)) {
        return -1;
    }

    return 0;
}

static int meta_decode_aaddsub(MetaInstruction *inst)
{
    MetaInstructionAUAddSub *as = &inst->auaddsub;
    bool conditional = bits(inst->raw, 26, 1);
    bool immediate = bits(inst->raw, 25, 1);
    bool o2r = !immediate && bits(inst->raw, 0, 1);

    inst->op = OP_AUADDSUB;
    as->sub = bits(inst->raw, 27, 1);
    as->top = false;

    inst->dst.type = DST_REG;
    inst->src1.type = inst->src2.type = SRC_REG;
    inst->dst.u = inst->src1.u = inst->src2.u =
        META_UNIT_A0 + bits(inst->raw, 24, 1);
    inst->dst.i = bits(inst->raw, 19, 5);
    as->pc_src1 = bits(inst->raw, 18, 1);
    inst->src1.i = bits(inst->raw, 14, 4);
    as->pc_src2 = bits(inst->raw, 13, 1);
    inst->src2.i = bits(inst->raw, 9, 5);

    if (o2r) {
        as->pc_src2 = false;
        inst->src2.u = meta_decode_o2r(inst->src1.u, &inst->src2.i);
    } else {
        inst->src2.i &= 0xf;
    }

    if (conditional && immediate) {
        bool ca = bits(inst->raw, 5, 1);

        as->pc_src2 = false;
        inst->src2.type = SRC_IMM;
        inst->src2.i = bits(inst->raw, 6, 8);

        if (ca) {
            inst->dst.u = bits(inst->raw, 1, 4);
        } else {
            inst->cc = bits(inst->raw, 1, 4);
        }
    } else if (conditional) {
        inst->cc = bits(inst->raw, 1, 4);
        inst->dst.u = bits(inst->raw, 5, 4);
    } else if (immediate) {
        bool o1z = bits(inst->raw, 2, 1);
        bool se = bits(inst->raw, 1, 1);
        as->top = bits(inst->raw, 0, 1);

        as->pc_src1 = bits(inst->raw, 23, 1);
        as->pc_src2 = false;
        inst->src1.i = inst->dst.i;

        if (o1z) {
            inst->src1.type = SRC_IMM;
            inst->src1.i = 0;
        }

        inst->src2.type = SRC_IMM;
        inst->src2.i = bits(inst->raw, 3, 16);

        if (se) {
            inst->src2.i = ((int32_t)inst->src2.i << 16) >> 16;
        }
        if (as->top) {
            inst->src2.i <<= 16;
        }
    } else {
        bool o1z = bits(inst->raw, 2, 1);

        if (o1z) {
            as->pc_src1 = false;
            inst->src1.type = SRC_IMM;
            inst->src1.i = 0;
        }
    }

    if ((inst->dst.u == META_UNIT_A0) ||
        (inst->dst.u == META_UNIT_A1)) {
        inst->dst.i &= 0xf;
    }

    return 0;
}

static int meta_decode_dspreg_init(MetaInstruction *inst)
{
    inst->op = OP_DSPREG_INIT;
    inst->dsp = true;
    inst->dual = bits(inst->raw, 2, 1);

    inst->dst.type = DST_DSPREG;
    inst->src1.type = SRC_REG;
    inst->dst.u = inst->src1.u =
        META_UNIT_D0 + bits(inst->raw, 0, 1);
    inst->dst.i = bits(inst->raw, 19, 5);
    inst->src1.i = bits(inst->raw, 14, 5);

    if (bits(inst->raw, 1, 1)) {
        inst->src1.type = SRC_IMM;
        inst->src1.i = bits(inst->raw, 3, 16);

        /* sign extend */
        inst->src1.i = ((int32_t)inst->src1.i << 16) >> 16;
    }

    return 0;
}

static int meta_decode_dspreg_copy(MetaInstruction *inst)
{
    inst->op = OP_DSPREG_COPY;
    inst->dsp = true;
    inst->dual = bits(inst->raw, 2, 1);

    inst->dst.type = DST_REG;
    inst->src1.type = SRC_DSPREG;
    inst->dst.u = inst->src1.u =
        META_UNIT_D0 + bits(inst->raw, 0, 1);
    inst->dst.i = bits(inst->raw, 19, 5);
    inst->src1.i = bits(inst->raw, 14, 5);

    return 0;
}

static int meta_decode_dsp_getset(MetaInstruction *inst)
{
    MetaInstructionDspGetSet *gs = &inst->dspgetset;

    if (bits(inst->raw, 3, 5) || bits(inst->raw, 1, 1)) {
        return -1;
    }

    inst->dsp = true;

    gs->l1 = bits(inst->raw, 2, 1);

    if (bits(inst->raw, 8, 1)) {
        /* load */
        inst->op = OP_DSPREG_GET;
        inst->dst.type = DST_DSPREG;
        inst->dst.u = META_UNIT_D0 + bits(inst->raw, 0, 1);
        inst->dst.i = bits(inst->raw, 19, 5);
    } else {
        /* store */
        inst->op = OP_DSPREG_SET;
        inst->src1.type = SRC_DSPREG;
        inst->src1.u = META_UNIT_D0 + bits(inst->raw, 0, 1);
        inst->src1.i = bits(inst->raw, 19, 5);
    }

    gs->a_base.u = gs->a_off.u =
        META_UNIT_A0 + bits(inst->raw, 18, 1);

    gs->a_base.type = SRC_REG;
    gs->a_base.i = bits(inst->raw, 14, 4);

    if (bits(inst->raw, 13, 1)) {
        if (bits(inst->raw, 11, 2)) {
            return -1;
        }
        gs->a_off.type = SRC_IMM;
        gs->a_off.i = ((int32_t)bits(inst->raw, 9, 2) << 30) >> 30;
    } else {
        gs->a_off.type = SRC_REG;
        gs->a_off.i = bits(inst->raw, 9, 4);
    }

    return 0;
}

static int meta_decode_dsp(MetaInstruction *inst)
{
    uint32_t minor = bits(inst->raw, 24, 4);

    switch (minor) {
    case 0x1: /* D MOV DspReg, Reg */
        return meta_decode_dspreg_init(inst);

    case 0x2: /* D MOV Reg, DspReg */
        return meta_decode_dspreg_copy(inst);

    case 0x4:
        return meta_decode_dsp_getset(inst);

    default:
        return -1;
    }
}

static int meta_decode_branch(MetaInstruction *inst)
{
    MetaInstructionBranch *br = &inst->branch;

    inst->op = OP_BRANCH;
    inst->cc = bits(inst->raw, 1, 4);

    br->offset = ((int32_t)bits(inst->raw, 5, 19) << 13) >> 11;
    br->repeat = bits(inst->raw, 0, 1);

    return 0;
}

static int meta_decode_mov_porttounit(MetaInstruction *inst)
{
    MetaInstructionPortToUnit *ptu = &inst->porttounit;

    inst->op = OP_PORTTOUNIT;
    inst->cc = bits(inst->raw, 1, 4);

    ptu->dst2.type = DST_NONE;
    ptu->width =
        (bits(inst->raw, 9, 1) << 1) |
        bits(inst->raw, 0, 1);

    if (ptu->width == WIDTH_64B) {
        /* this encoding doesn't allow 64b */
        ptu->width = WIDTH_32B;
    }

    inst->dst.type = DST_REG;
    inst->dst.u = bits(inst->raw, 5, 4);
    inst->dst.i = bits(inst->raw, 14, 5);

    inst->src1.i = bits(inst->raw, 19, 5);

    if (inst->src1.i == META_RA) {
        inst->src1.type = SRC_REG;
        inst->src1.u = META_UNIT_RA;
    } else {
        inst->src1.type = SRC_COPROC;
    }

    return 0;
}

static int meta_decode_mov_ttrec_64b(MetaInstruction *inst)
{
    MetaInstructionUnitToUnit *utu = &inst->unittounit;

    if (bits(inst->raw, 5, 2) ||
        (bits(inst->raw, 9, 5) != 0x10)) {
        return -1;
    }

    inst->op = OP_UNITTOUNIT;
    inst->cc = bits(inst->raw, 1, 4);

    inst->src1.type = inst->src2.type = SRC_REG;
    inst->src1.u = inst->src2.u = meta_bu_map[bits(inst->raw, 7, 2)];
    inst->src1.i = bits(inst->raw, 19, 5);
    inst->src2.i = bits(inst->raw, 14, 5);

    inst->dst.type = DST_REG;
    inst->dst.u = META_UNIT_TT;
    inst->dst.i = META_TTREC;

    utu->swap = utu->kick = utu->defr = false;
    utu->width = WIDTH_64B;

    return 0;
}

static int meta_decode_mov_porttounit_64b(MetaInstruction *inst)
{
    MetaInstructionPortToUnit *ptu = &inst->porttounit;

    if (bits(inst->raw, 0, 1)) {
        return meta_decode_mov_ttrec_64b(inst);
    }

    if (inst->raw & 0x00000180) {
        return -1;
    }

    inst->op = OP_PORTTOUNIT;
    inst->cc = bits(inst->raw, 1, 4);
    ptu->width = WIDTH_64B;

    inst->dst.type = DST_REG;
    inst->dst.u = meta_bu_map[bits(inst->raw, 5, 2)];
    inst->dst.i = bits(inst->raw, 14, 5);

    ptu->dst2.type = DST_REG;
    ptu->dst2.u = meta_unit_partner(inst->dst.u);
    ptu->dst2.i = bits(inst->raw, 9, 5);

    inst->src1.i = bits(inst->raw, 19, 5);

    if (inst->src1.i == META_RA) {
        inst->src1.type = SRC_REG;
        inst->src1.u = META_UNIT_RA;
    } else {
        inst->src1.type = SRC_COPROC;
    }

    return 0;
}

static int meta_decode_unittounit(MetaInstruction *inst)
{
    MetaInstructionUnitToUnit *utu = &inst->unittounit;
    const uint32_t rti = 0xa3ffffff;
    const uint32_t rth = 0xa37fffff;

    inst->op = OP_UNITTOUNIT;
    inst->cc = bits(inst->raw, 1, 4);

    inst->src1.type = SRC_REG;
    inst->src1.u = bits(inst->raw, 10, 4);
    inst->src1.i = bits(inst->raw, 19, 5);

    inst->dst.type = DST_REG;
    inst->dst.u = bits(inst->raw, 5, 4);
    inst->dst.i = bits(inst->raw, 14, 5);

    utu->swap = bits(inst->raw, 9, 1);
    utu->kick = utu->defr = false;
    utu->width = WIDTH_32B;

    if (bits(inst->raw, 0, 1)) {
        if ((inst->raw != rti) &&
            (inst->raw != rth) &&
            bits(inst->raw, 9, 1)) {
            /* MOVTT */
            utu->tt = true;
            utu->swap = false;
        } else if (bits(inst->raw, 9, 5) == 0x1f) {
            /* RTI or RTH */
            inst->op = bits(inst->raw, 23, 1) ? OP_RTI : OP_RTH;
        } else if (!bits(inst->raw, 9, 1)) {
            /* KICK or DEFR */
            inst->src1.u = META_UNIT_TR;
            if (bits(inst->raw, 9, 5) == 0x10) {
                utu->defr = true;
            } else {
                utu->kick = true;
            }
        }
    }

    return 0;
}

static int meta_decode_set_conditional(MetaInstruction *inst)
{
    MetaInstructionGet *get = &inst->get;

    inst->op = OP_SET;
    inst->cc = bits(inst->raw, 1, 4);

    get->width =
        (bits(inst->raw, 9, 1) << 1) |
        bits(inst->raw, 0, 1);
    get->linked = bits(inst->raw, 7, 1);
    get->update_addr = false;
    get->read = READ_NONE;
    get->mx = false;

    get->base.type = SRC_REG;
    get->base.u = meta_bu_map[bits(inst->raw, 5, 2)];
    get->base.i = bits(inst->raw, 14, 5);

    get->off.type = SRC_NONE;

    inst->src1.type = SRC_REG;
    inst->src1.u = bits(inst->raw, 10, 4);
    inst->src1.i = bits(inst->raw, 19, 5);

    return 0;
}

static int meta_decode_getset_extimm(MetaInstruction *inst)
{
    MetaInstructionGet *get = &inst->get;
    uint32_t minor = bits(inst->raw, 24, 4);

    if (minor == 0x5) {
        inst->op = OP_SET;
        inst->src1.type = SRC_REG;
        inst->src1.u = meta_bu_map[bits(inst->raw, 3, 2)];
        inst->src1.i = bits(inst->raw, 19, 5);
    } else {
        inst->op = OP_GET;
        inst->dst.type = DST_REG;
        inst->dst.u = meta_bu_map[bits(inst->raw, 3, 2)];
        inst->dst.i = bits(inst->raw, 19, 5);
    }

    get->linked = false;
    get->read = READ_NONE;
    get->mx = false;
    get->update_addr = false;
    get->post = false;
    get->width = bits(inst->raw, 1, 2);

    get->base.type = SRC_REG;
    get->base.u = meta_bu_map[bits(inst->raw, 5, 2)];
    get->base.i = bits(inst->raw, 0, 1);

    get->off.type = SRC_IMM;
    get->off.u = get->base.u;
    get->off.i = ((int32_t)bits(inst->raw, 7, 12) << 20) >> (20 - get->width);

    return 0;
}

static int meta_decode_issue_extimm(MetaInstruction *inst)
{
    MetaInstructionGet *get = &inst->get;

    if (bits(inst->raw, 3, 2)) {
        return -1;
    }

    inst->op = OP_GET;
    inst->dst.type = DST_REG;
    inst->dst.u = META_UNIT_RA;
    inst->dst.i = bits(inst->raw, 19, 5);

    get->width = bits(inst->raw, 1, 2);
    get->read = READ_NONE;
    get->mx = false;
    get->update_addr = false;
    get->post = false;
    get->linked = false;

    get->base.type = SRC_REG;
    get->base.u = meta_bu_map[bits(inst->raw, 5, 2)];
    get->base.i = bits(inst->raw, 0, 1);

    get->off.type = SRC_IMM;
    get->off.i = ((int32_t)bits(inst->raw, 7, 12) << 20) >> (20 - get->width);

    return 0;
}

static int meta_decode_lock(MetaInstruction *inst)
{
    MetaInstructionLock *lock = &inst->lock;
    uint32_t lbits = bits(inst->raw, 0, 2);

    if (bits(inst->raw, 2, 22)) {
        return -1;
    }

    inst->op = OP_LOCK;

    switch (lbits) {
    case 0x0:
    case 0x1:
        lock->n = lbits;
        break;

    case 0x3:
        lock->n = 2;
        break;

    default:
        return -1;
    }

    return 0;
}

static int meta_decode_movct(MetaInstruction *inst)
{
    MetaInstructionUnitToUnit *utu = &inst->unittounit;
    bool se = bits(inst->raw, 1, 1);
    bool h = bits(inst->raw, 0, 1);

    inst->op = OP_UNITTOUNIT;
    utu->swap = utu->kick = utu->defr = false;
    utu->width = WIDTH_32B;

    inst->dst.type = DST_REG;
    inst->dst.u = bits(inst->raw, 2, 1) ? META_UNIT_TT : META_UNIT_CT;
    inst->dst.i = bits(inst->raw, 19, 5);

    inst->src1.type = SRC_IMM;
    inst->src1.i = bits(inst->raw, 3, 16);

    if (se) {
        inst->src1.i = ((int32_t)inst->src1.i << 16) >> 16;
    }
    if (h) {
        inst->src1.i <<= 16;
    }

    return 0;
}

static int meta_decode_misc_duop(MetaInstruction *inst)
{
    if (inst->raw & 0x00003ee0) {
        return -1;
    }

    inst->dsp = bits(inst->raw, 8, 1);
    inst->dual = inst->dsp && bits(inst->raw, 4, 1);

    switch (bits(inst->raw, 1, 2)) {
    case 0x0: /* XSDB */
    case 0x1: /* XSDW */
        inst->op = OP_XSD;
        inst->xsd.width = bits(inst->raw, 1, 1);
        inst->xsd.setflags = bits(inst->raw, 3, 1);
        break;

    case 0x2: /* BEX */
        inst->op = OP_BEX;
        inst->bex.setflags = bits(inst->raw, 3, 1);
        break;

    case 0x3: /* RTDW */
        inst->op = OP_RTD;
        inst->rtd.setflags = bits(inst->raw, 3, 1);
        break;
    }

    inst->dst.type = DST_REG;
    inst->src1.type = SRC_REG;
    inst->dst.u = inst->src1.u =
        META_UNIT_D0 + bits(inst->raw, 0, 1);
    inst->dst.i = bits(inst->raw, 19, 5);
    inst->src1.i = bits(inst->raw, 14, 5);

    if (inst->dsp) {
        if (inst->dst.i & 0x10) {
            inst->dst.type = DST_DSPRAM;
        }

        if (inst->src1.i & 0x10) {
            inst->src1.type = SRC_DSPRAM;
        }
    }

    return 0;
}

static int meta_decode_callr(MetaInstruction *inst)
{
    MetaInstructionJump *jump = &inst->jump;

    inst->op = OP_JUMP;

    inst->src1.type = SRC_REG;
    inst->src1.u = meta_bu_map[bits(inst->raw, 3, 2)];
    inst->src1.i = bits(inst->raw, 0, 3);

    jump->offset = ((int32_t)bits(inst->raw, 5, 19) << 13) >> 11;
    jump->call = jump->rel = true;

    return 0;
}

static int meta_decode_jump(MetaInstruction *inst)
{
    MetaInstructionJump *jump = &inst->jump;

    inst->op = OP_JUMP;

    inst->src1.type = SRC_REG;
    inst->src1.u = meta_bu_map[bits(inst->raw, 0, 2)];
    inst->src1.i = bits(inst->raw, 19, 5);

    jump->offset = bits(inst->raw, 3, 16);
    jump->call = bits(inst->raw, 2, 1);
    jump->rel = false;

    return 0;
}

static int meta_decode_dcache(MetaInstruction *inst)
{
    MetaInstructionSource base;

    base.type = SRC_REG;
    base.u = meta_bu_map[bits(inst->raw, 5, 2)];
    base.i = bits(inst->raw, 14, 5);

    if (bits(inst->raw, 7, 1)) {
        /* read */
        inst->dst.type = DST_REG;
        inst->dst.u = meta_bu_map[bits(inst->raw, 3, 2)];
        inst->dst.i = bits(inst->raw, 19, 5);

        if (bits(inst->raw, 0, 1)) {
            /* CACHER[DL] */
            MetaInstructionCache *cache = &inst->cache;
            inst->op = OP_CACHE;
            cache->op = CACHE_R;
            cache->width = WIDTH_32B + bits(inst->raw, 1, 1);
        } else {
            /* LNKGET */
            MetaInstructionGet *get = &inst->get;

            inst->op = OP_GET;

            get->width = bits(inst->raw, 1, 2);
            get->read = READ_NONE;
            get->update_addr = false;
            get->linked = true;
            get->mx = false;

            get->base = base;

            get->off.type = SRC_IMM;
            get->off.i = 0;
        }
    } else {
        /* write */
        MetaInstructionCache *cache = &inst->cache;
        inst->op = OP_CACHE;
        inst->src1.type = SRC_REG;
        inst->src1.u = meta_bu_map[bits(inst->raw, 3, 2)];
        inst->src1.i = bits(inst->raw, 19, 5);

        if (bits(inst->raw, 0, 1)) {
            /* CACHEW[DL] */
            cache->op = CACHE_W;
            cache->width = WIDTH_32B + bits(inst->raw, 1, 1);
        } else {
            /* DCACHE */
            cache->op = DCACHE;
            cache->width = WIDTH_64B;
        }
    }

    if (inst->op == OP_CACHE) {
        MetaInstructionCache *cache = &inst->cache;
        cache->base = base;
        cache->off.type = SRC_IMM;
        cache->off.i = ((int32_t)bits(inst->raw, 8, 6) << 26) >>
            (26 - (cache->op == DCACHE) ? 6 : cache->width);
    }

    return 0;
}

static int meta_decode_icache(MetaInstruction *inst)
{
    MetaInstructionCache *cache = &inst->cache;

    if (bits(inst->raw, 5, 4)) {
        return -1;
    }

    inst->op = OP_CACHE;
    cache->op = ICACHE;
    cache->lines = bits(inst->raw, 1, 4);
    cache->off.type = SRC_IMM;
    cache->off.i = ((int32_t)bits(inst->raw, 9, 15) << 17) >> 17;
    cache->priority = bits(inst->raw, 0, 1);

    return 0;
}

static int meta_decode_switch(MetaInstruction *inst)
{
    inst->op = OP_SWITCH;
    inst->swtch.code = bits(inst->raw, 0, 24);
    return 0;
}

static int meta_decode_misc(MetaInstruction *inst)
{
    uint32_t minor = bits(inst->raw, 24, 4);

    switch (minor) {
    case 0x0:
        return meta_decode_branch(inst);

    case 0x1:
        return meta_decode_mov_porttounit(inst);

    case 0x2:
        return meta_decode_mov_porttounit_64b(inst);

    case 0x3:
        return meta_decode_unittounit(inst);

    case 0x4:
        return meta_decode_set_conditional(inst);

    case 0x5:
    case 0x7:
        return meta_decode_getset_extimm(inst);

    case 0x6:
        return meta_decode_issue_extimm(inst);

    case 0x8:
        return meta_decode_lock(inst);

    case 0x9:
        return meta_decode_movct(inst);

    case 0xa:
        return meta_decode_misc_duop(inst);

    case 0xb:
        return meta_decode_callr(inst);

    case 0xc:
        return meta_decode_jump(inst);

    case 0xd:
        return meta_decode_dcache(inst);

    case 0xe:
        return meta_decode_icache(inst);

    case 0xf:
        return meta_decode_switch(inst);

    default:
        return -1;
    }
}

static int meta_decode_fpu_unittounit(MetaInstruction *inst)
{
    inst->op = OP_MMOV;
    inst->multistep = MULTISTEP_RMASK;
    inst->step = (bits(inst->raw, 7, 7) << 1) | 0x1;
    inst->multimov.width = WIDTH_32B + bits(inst->raw, 24, 1);
    inst->dst.type = DST_REG;
    inst->src1.type = SRC_REG;
    inst->fx = true;

    if (bits(inst->raw, 28, 4) == 0xc) {
        inst->dst.u = META_UNIT_FX;
        inst->dst.i = bits(inst->raw, 14, 4);
        inst->src1.u = META_UNIT_D0 + bits(inst->raw, 0, 1);
        inst->src1.i = bits(inst->raw, 19, 5);
    } else {
        inst->dst.u = META_UNIT_D0 + bits(inst->raw, 0, 1);
        inst->dst.i = bits(inst->raw, 19, 5);
        inst->src1.u = META_UNIT_FX;
        inst->src1.i = bits(inst->raw, 14, 4);
    }

    return 0;
}

static int meta_decode_getset(MetaInstruction *inst)
{
    MetaInstructionGet *get = &inst->get;
    uint32_t major = bits(inst->raw, 28, 4);

    if ((inst->raw & 0x0e000006) == 0x0e000002) {
        return meta_decode_fpu_unittounit(inst);
    }

    inst->op = (major == 0xb) ? OP_SET : OP_GET;
    inst->multistep = bits(inst->raw, 27, 1) ? MULTISTEP_RMASK : MULTISTEP_NONE;

    get->linked = false;
    get->update_addr = false;
    get->post = false;
    get->mx = false;
    get->read = READ_NONE;

    get->base.type = get->off.type = SRC_REG;
    get->base.u = get->off.u = meta_bu_map[bits(inst->raw, 5, 2)];
    get->base.i = bits(inst->raw, 14, 5);
    get->off.i = bits(inst->raw, 9, 5);

    if (inst->multistep) {
        /* multistep */
        bool l1 = bits(inst->raw, 24, 1);

        get->mx = bits(inst->raw, 0, 1) && bits(inst->raw, 25, 2);
        if (get->mx) {
            inst->dsp = true;
        }

        inst->step = (bits(inst->raw, 7, 7) << 1) | 0x1;

        get->width = WIDTH_32B + l1;
        get->update_addr = true;
        get->post = true;

        get->off.type = SRC_IMM;
        get->off.i = (0x4 >> get->mx) << l1;

        inst->dst.type = DST_REG;

        if (bits(inst->raw, 25, 2) == 0x3) {
            /* F MGET */
            inst->dst.u = META_UNIT_FX;
            inst->dst.i = bits(inst->raw, 19, 4);
        } else {
            inst->dst.u = meta_bu_map[bits(inst->raw, 3, 2)];
            inst->dst.i = bits(inst->raw, 19, 5);
        }

        if (inst->op == OP_GET) {
            switch (bits(inst->raw, 25, 2)) {
            case 0x2:
                if (bits(inst->raw, 1, 1)) {
                    get->read = READ_FLUSH;
                } else {
                    get->read = READ_DRAIN;
                }
                get->update_addr = false;
                break;

            case 0x1:
                get->read = READ_PRIME;
                inst->dst.u = META_UNIT_RA;
                /* can't decide port here - M8 is unknown */
                break;

            default:
                break;
            }
        }
    } else {
        /* not multistep */
        get->post = bits(inst->raw, 0, 1);
        get->update_addr = bits(inst->raw, 7, 1);
        get->width =
            (bits(inst->raw, 26, 1) << 1) |
            bits(inst->raw, 24, 1);

        inst->dst.type = DST_REG;
        inst->dst.u = bits(inst->raw, 1, 4);
        inst->dst.i = bits(inst->raw, 19, 5);

        if (bits(inst->raw, 25, 1)) {
            get->off.type = SRC_IMM;
            get->off.i = ((int32_t)bits(inst->raw, 8, 6) << 26)
                             >> (26 - get->width);
        }
    }

    if (inst->op == OP_SET) {
        /* we actually want to source data */
        inst->dst.type = DST_NONE;
        inst->src1.type = SRC_REG;
        inst->src1.u = inst->dst.u;
        inst->src1.i = inst->dst.i;
    }

    return 0;
}

static int meta_decode_xfr(MetaInstruction *inst)
{
    MetaInstructionXfr *xfr = &inst->xfr;

    inst->op = OP_XFR;
    inst->multistep = MULTISTEP_RMASK;
    inst->step = 0x3;

    xfr->update_src = bits(inst->raw, 27, 1);
    xfr->update_dst = bits(inst->raw, 26, 1);
    xfr->l1 = bits(inst->raw, 25, 1);
    xfr->post = bits(inst->raw, 24, 1);

    xfr->base_src.type = xfr->off_src.type = SRC_REG;
    xfr->base_dst.type = xfr->off_dst.type = DST_REG;
    xfr->base_src.i = bits(inst->raw, 19, 5);
    xfr->off_src.i = bits(inst->raw, 14, 5);
    xfr->base_dst.i = bits(inst->raw, 9, 5);
    xfr->off_dst.i = bits(inst->raw, 4, 5);
    xfr->base_src.u = xfr->off_src.u = meta_bu_map[bits(inst->raw, 2, 2)];
    xfr->base_dst.u = xfr->off_dst.u = meta_bu_map[bits(inst->raw, 0, 2)];

    if (xfr->base_dst.u == xfr->base_src.u) {
        inst->op = OP_CPRR;
    }

    return 0;
}

static int meta_decode_coproc(MetaInstruction *inst)
{
    return -1;
}

static int meta_decode_fpu_mov(MetaInstruction *inst)
{
    if (bits(inst->raw, 0, 1)) {
        inst->op = OP_FPU_MOV_IMM;
        inst->fpu.paired = bits(inst->raw, 2, 1);
        inst->fpu.dbl = bits(inst->raw, 1, 1);

        inst->dst.type = DST_REG;
        inst->dst.u = META_UNIT_FX;
        inst->dst.i = bits(inst->raw, 19, 4);

        inst->src1.type = SRC_IMM;
        inst->src1.i = bits(inst->raw, 3, 16);

        return 0;
    }

    inst->cc = bits(inst->raw, 1, 4);
    inst->fpu.paired = bits(inst->raw, 6, 1);
    inst->fpu.dbl = bits(inst->raw, 5, 1);

    inst->dst.type = DST_REG;
    inst->dst.u = META_UNIT_FX;
    inst->dst.i = bits(inst->raw, 19, 4);

    inst->src1.type = SRC_REG;
    inst->src1.u = META_UNIT_FX;
    inst->src1.i = bits(inst->raw, 14, 4);

    switch (bits(inst->raw, 7, 2)) {
    case 0x0:
        inst->op = OP_FPU_MOV_REG;
        return 0;

    case 0x1:
        inst->op = OP_FPU_ABS;
        return 0;

    case 0x2:
        inst->op = OP_FPU_NEG;
        return 0;

    case 0x3:
        if (inst->fpu.paired) {
            inst->op = OP_FPU_SWAP;

            if (inst->raw & 0x008c203f) {
                return -1;
            }
        } else {
            inst->op = OP_FPU_PACK;

            inst->src2.type = SRC_REG;
            inst->src2.u = META_UNIT_FX;
            inst->src2.i = bits(inst->raw, 9, 4);

            if (inst->raw & 0x008c207f) {
                return -1;
            }
        }
        return 0;

    default:
        return -1;
    }
}

static int meta_decode_fpu_arith(MetaInstruction *inst)
{
    MetaInstructionFpuArith *a = &inst->fpu.arith;

    a->reduction = false;
    a->invert = bits(inst->raw, 7, 1);
    inst->fpu.paired = bits(inst->raw, 6, 1);
    inst->fpu.dbl = bits(inst->raw, 5, 1);

    inst->dst.type = DST_REG;
    inst->dst.u = META_UNIT_FX;
    inst->dst.i = inst->fpu.info.args.rd = bits(inst->raw, 19, 4);

    inst->src1.type = inst->src2.type = SRC_REG;
    inst->src1.u = inst->src2.u = META_UNIT_FX;
    inst->src1.i = inst->fpu.info.args.rs1 = bits(inst->raw, 14, 4);
    inst->src2.i = inst->fpu.info.args.rs2 = bits(inst->raw, 9, 4);

    if (bits(inst->raw, 0, 1)) {
        if (bits(inst->raw, 8, 1)) {
            inst->op = OP_FPU_SUB;
            inst->fpu.info.op.enc = META_FXOPENC_SUB;
        } else {
            inst->op = OP_FPU_ADD;
            inst->fpu.info.op.enc = META_FXOPENC_ADD;
        }

        /* use rop3 for src2 */
        inst->fpu.info.args.rs3 = inst->fpu.info.args.rs2;
        inst->fpu.info.args.rs2 = 0;
    } else {
        inst->op = OP_FPU_MUL;
        inst->fpu.info.op.enc = META_FXOPENC_MUL;
    }

    if (inst->raw & 0x00842000) {
        return -1;
    }

    return 0;
}

static int meta_decode_fpu_convert(MetaInstruction *inst)
{
    MetaInstructionFpuConvert *conv = &inst->fpu.convert;

    inst->op = OP_FPU_CONVERT;
    inst->cc = bits(inst->raw, 1, 4);
    inst->fpu.paired = bits(inst->raw, 6, 1);
    inst->fpu.dbl = bits(inst->raw, 5, 1);

    inst->dst.type = DST_REG;
    inst->dst.u = META_UNIT_FX;
    inst->dst.i = inst->fpu.info.args.rd = bits(inst->raw, 19, 4);

    inst->src1.type = SRC_REG;
    inst->src1.u = META_UNIT_FX;
    inst->src1.i = inst->fpu.info.args.rs3 = bits(inst->raw, 14, 4);

    conv->z_rounding = bits(inst->raw, 12, 1);

    if (bits(inst->raw, 0, 1)) {
        conv->dst_type = inst->fpu.dbl ? FPU_D : FPU_F;

        if (bits(inst->raw, 7, 1)) {
            conv->src_type = FPU_XL;
            conv->dst_type = FPU_D;
            inst->fpu.info.op.enc = META_FXOPENC_XLTOD;
        } else {
            if (bits(inst->raw, 8, 1)) {
                if (bits(inst->raw, 13, 1)) {
                    if (bits(inst->raw, 9, 1)) {
                        conv->src_type = FPU_L;
                        conv->dst_type = FPU_D;
                        inst->fpu.info.op.enc = META_FXOPENC_LTOD;
                    } else {
                        conv->src_type = FPU_I;
                        inst->fpu.info.op.enc = META_FXOPENC_ITOF;
                    }
                } else {
                    if (bits(inst->raw, 9, 1)) {
                        conv->src_type = FPU_H;
                        inst->fpu.info.op.enc = META_FXOPENC_HTOF;
                        inst->fpu.info.args.rs2 = inst->fpu.info.args.rs3;
                        inst->fpu.info.args.rs3 = 0;
                    } else {
                        if (inst->fpu.dbl) {
                            conv->src_type = FPU_D;
                            conv->dst_type = FPU_F;
                            inst->fpu.info.op.enc = META_FXOPENC_DTOF;
                        } else {
                            conv->src_type = FPU_F;
                            conv->dst_type = FPU_D;
                            inst->fpu.info.op.enc = META_FXOPENC_FTOD;
                        }
                    }
                }
            } else {
                conv->src_type = FPU_X;
                inst->fpu.info.op.enc = META_FXOPENC_XTOF;
            }
        }
    } else {
        conv->src_type = inst->fpu.dbl ? FPU_D : FPU_F;

        if (bits(inst->raw, 7, 1)) {
            conv->src_type = FPU_D;
            conv->dst_type = FPU_XL;
            inst->fpu.info.op.enc = META_FXOPENC_DTOXL;
        } else {
            if (bits(inst->raw, 8, 1)) {
                if (bits(inst->raw, 9, 1)) {
                    if (bits(inst->raw, 13, 1)) {
                        conv->src_type = FPU_D;
                        conv->dst_type = FPU_L;
                        inst->fpu.info.op.enc = META_FXOPENC_DTOL;
                    } else {
                        conv->dst_type = FPU_H;
                        inst->fpu.info.op.enc = META_FXOPENC_FTOH;
                        inst->fpu.info.args.rs2 = inst->fpu.info.args.rs3;
                        inst->fpu.info.args.rs3 = 0;
                    }
                } else {
                    conv->dst_type = FPU_I;
                    inst->fpu.info.op.enc = META_FXOPENC_FTOI;
                }
            } else {
                conv->dst_type = FPU_X;
                inst->fpu.info.op.enc = META_FXOPENC_FTOX;
            }
        }
    }

    if (conv->src_type == FPU_X || conv->dst_type == FPU_X) {
        inst->src2.type = SRC_IMM;
        inst->src2.i = bits(inst->raw, 9, 5);
    }

    if (conv->src_type == FPU_XL || conv->dst_type == FPU_XL) {
        inst->src2.type = SRC_IMM;
        inst->src2.i = bits(inst->raw, 8, 6);
    }

    return 0;
}

static int meta_decode_fpu_cmp(MetaInstruction *inst)
{
    MetaInstructionFpuCmp *cmp = &inst->fpu.cmp;

    inst->cc = bits(inst->raw, 1, 4);
    inst->fpu.paired = bits(inst->raw, 6, 1);
    inst->fpu.dbl = bits(inst->raw, 5, 1);

    inst->src1.type = inst->src2.type = SRC_REG;
    inst->src1.u = inst->src2.u = META_UNIT_FX;
    inst->src1.i = inst->fpu.info.args.rs1 = bits(inst->raw, 14, 4);
    inst->src2.i = inst->fpu.info.args.rs2 = bits(inst->raw, 9, 4);

    if (bits(inst->raw, 0, 1)) {
        inst->op = bits(inst->raw, 7, 1) ? OP_FPU_MAX : OP_FPU_MIN;
        inst->dst.type = DST_REG;
        inst->dst.u = META_UNIT_FX;
        inst->dst.i = inst->fpu.info.args.rd = bits(inst->raw, 19, 4);

        if (inst->op == OP_FPU_MAX) {
            inst->fpu.info.op.flags |= META_FXINST_MAX;
        }

        if (inst->raw & 0x00842100) {
            return -1;
        }
    } else {
        inst->op = OP_FPU_CMP;
        cmp->abs = bits(inst->raw, 19, 1);
        cmp->quiet = bits(inst->raw, 7, 1);
        inst->fpu.info.op.flags |= cmp->abs ? META_FXINST_A : 0;
        inst->fpu.info.op.flags |= cmp->quiet ? META_FXINST_Q : 0;

        if (bits(inst->raw, 8, 1)) {
            inst->src2.type = SRC_IMM;
            inst->src2.i = 0;
            inst->fpu.info.op.flags |= META_FXINST_Z;
        }

        if (inst->raw & 0x00f42001) {
            return -1;
        }
    }

    return 0;
}

static int meta_decode_fpu_rearith(MetaInstruction *inst)
{
    MetaInstructionFpuArith *a = &inst->fpu.arith;

    a->reduction = true;
    a->invert = bits(inst->raw, 7, 1);
    inst->fpu.paired = true;

    inst->dst.type = DST_REG;
    inst->dst.u = META_UNIT_FX;
    inst->dst.i = inst->fpu.info.args.rd = bits(inst->raw, 19, 4);

    inst->src1.type = inst->src2.type = SRC_REG;
    inst->src1.u = inst->src2.u = META_UNIT_FX;
    inst->src1.i = inst->fpu.info.args.rs1 = bits(inst->raw, 14, 4);
    inst->src2.i = inst->fpu.info.args.rs2 = bits(inst->raw, 9, 4);

    if (!bits(inst->raw, 0, 1)) {
        if (bits(inst->raw, 8, 1)) {
            inst->op = OP_FPU_SUB;
            inst->fpu.info.op.enc = META_FXOPENC_SUBRE;
        } else {
            inst->op = OP_FPU_ADD;
            inst->fpu.info.op.enc = META_FXOPENC_ADDRE;
        }

        /* use rop3 for src2 */
        inst->fpu.info.args.rs3 = inst->fpu.info.args.rs2;
        inst->fpu.info.args.rs2 = 0;
    } else {
        inst->op = OP_FPU_MUL;
        inst->fpu.info.op.enc = META_FXOPENC_MULRE;
    }

    if (inst->raw & 0x008c627e) {
        return -1;
    }

    return 0;
}

static int meta_decode_fpu_muz(MetaInstruction *inst)
{
    MetaInstructionFpuMuz *muz = &inst->fpu.muz;

    inst->op = OP_FPU_MUZ;
    inst->fpu.info.op.enc = META_FXOPENC_MUZ;
    muz->sub = bits(inst->raw, 8, 1);
    muz->invert = bits(inst->raw, 7, 1);
    muz->quiet = bits(inst->raw, 1, 1);
    muz->o3o = bits(inst->raw, 0, 1);
    inst->fpu.info.op.flags |= muz->sub ? META_FXINST_PM : 0;
    inst->fpu.info.op.flags |= muz->invert ? META_FXINST_N : 0;
    inst->fpu.info.op.flags |= muz->quiet ? META_FXINST_Q : 0;
    inst->fpu.info.op.flags |= muz->o3o ? META_FXINST_O3O : 0;

    inst->fpu.paired = bits(inst->raw, 6, 1);
    inst->fpu.dbl = bits(inst->raw, 5, 1);

    inst->dst.type = DST_REG;
    inst->dst.u = META_UNIT_FX;
    inst->dst.i = inst->fpu.info.args.rd = bits(inst->raw, 19, 4);

    inst->src1.type = SRC_REG;
    inst->src1.u = META_UNIT_FX;
    inst->src1.i = inst->fpu.info.args.rs1 = bits(inst->raw, 14, 4);

    inst->src2.type = SRC_REG;
    inst->src2.u = META_UNIT_FX;
    inst->src2.i = inst->fpu.info.args.rs2 = bits(inst->raw, 9, 4);

    if (inst->raw & 0x0084200c) {
        return -1;
    }

    return 0;
}

static int meta_decode_fpu_reciprocal(MetaInstruction *inst)
{
    MetaInstructionFpuRcp *rcp = &inst->fpu.rcp;

    inst->op = bits(inst->raw, 8, 1) ? OP_FPU_RSQ : OP_FPU_RCP;

    rcp->zero_denorm = bits(inst->raw, 10, 1);
    rcp->quiet = bits(inst->raw, 9, 1);
    rcp->invert = bits(inst->raw, 7, 1);
    inst->fpu.info.op.flags |= rcp->zero_denorm ? META_FXINST_Z : 0;
    inst->fpu.info.op.flags |= rcp->quiet ? META_FXINST_Q : 0;
    inst->fpu.info.op.flags |= rcp->invert ? META_FXINST_N : 0;

    inst->fpu.paired = bits(inst->raw, 6, 1);
    inst->fpu.dbl = bits(inst->raw, 5, 1);

    inst->dst.type = DST_REG;
    inst->dst.u = META_UNIT_FX;
    inst->dst.i = inst->fpu.info.args.rd = bits(inst->raw, 19, 4);

    inst->src1.type = SRC_REG;
    inst->src1.u = META_UNIT_FX;
    inst->src1.i = inst->fpu.info.args.rs1 = bits(inst->raw, 14, 4);

    return 0;
}

static int meta_decode_fpu(MetaInstruction *inst)
{
    uint32_t minor = bits(inst->raw, 24, 4);
    int ret;

    inst->fx = true;
    inst->fpu.paired = false;
    inst->fpu.dbl = false;
    inst->fpu.info.op.raw = 0;
    inst->fpu.info.args.raw = 0;

    switch (minor) {
    case 0x0:
        ret = meta_decode_fpu_mov(inst);
        break;

    case 0x1:
        ret = meta_decode_fpu_arith(inst);
        break;

    case 0x2:
        ret = meta_decode_fpu_convert(inst);
        break;

    case 0x3:
        ret = meta_decode_fpu_cmp(inst);
        break;

    case 0x4:
        ret = meta_decode_fpu_rearith(inst);
        break;

    case 0x6:
        ret = meta_decode_fpu_muz(inst);
        break;

    case 0x7:
        ret = meta_decode_fpu_reciprocal(inst);
        break;

    default:
        ret = -1;
    }

    inst->fpu.info.op.flags |= inst->fpu.paired ? META_FXINST_P : 0;
    inst->fpu.info.op.flags |= inst->fpu.dbl ? META_FXINST_D : 0;

    return ret;
}

int meta_decode(MetaInstruction *inst, uint32_t raw)
{
    inst->raw = raw;
    inst->op = OP_BAD;
    inst->cc = META_CC_A;
    inst->dst.type = DST_NONE;
    inst->src1.type = SRC_NONE;
    inst->src2.type = SRC_NONE;
    inst->dsp = false;
    inst->dual = false;
    inst->fx = false;
    inst->multistep = MULTISTEP_NONE;

    switch (raw >> 28) {
    case 0x0:
    case 0x1:
        return meta_decode_duaddsub(inst);

    case 0x2:
    case 0x3:
    case 0x4:
        return meta_decode_andorxor(inst);

    case 0x5:
        return meta_decode_shift(inst);

    case 0x6:
        return meta_decode_mul(inst);

    case 0x7:
        return meta_decode_cmptst(inst);

    case 0x8:
        return meta_decode_aaddsub(inst);

    case 0x9:
        return meta_decode_dsp(inst);

    case 0xa:
        return meta_decode_misc(inst);

    case 0xb:
    case 0xc:
        return meta_decode_getset(inst);

    case 0xd:
        return meta_decode_xfr(inst);

    case 0xe:
        return meta_decode_coproc(inst);

    case 0xf:
        return meta_decode_fpu(inst);
    }

    /* keep foolish gcc happy */
    return -1;
}
