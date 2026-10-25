#include "qemu/osdep.h"
#include "qemu/log.h"
#include "instruction.h"
#include "log.h"

/* see http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSet64 */
#define NBITSSET(v) \
    (((v) * 0x200040008001ULL & 0x111111111111111ULL) % 0xf)

static const char *meta_creg_names[META_CT_MAX] = {
    "TXENABLE",     "TXMODE",       "TXSTATUS",     "TXRPT",
    "TXTIMER",      "TXL1START",    "TXL1END",      "TXL1COUNT",
    "TXL2START",    "TXL2END",      "TXL2COUNT",    "TXBPOBITS",
    "TXMRSIZE",     "TXTIMERI",     "TXDRCTRL",     "TXDRSIZE",
    "TXCATCH0",     "TXCATCH1",     "TXCATCH2",     "TXCATCH3",
    "TXDEFR",       "CT.21",        "TXCLKCTRL",    "TXINTERN0",
    "TXAMAREG0",    "TXAMAREG1",    "TXAMAREG2",    "TXAMAREG3",
    "TXDIVTIME",    "TXPRIVEXT",    "TXTACTCYC",    "TXIDLECYC",
};

static const char *meta_dreg_names[2][32] = {
    { "D0Re0", "D0Ar6", "D0Ar4", "D0Ar2", "D0FrT", "D0.5",  "D0.6",  "D0.7",
      "D0.8",  "D0.9",  "D0.10", "D0.11", "D0.12", "D0.13", "D0.14", "D0.15",
      "D0.16", "D0.17", "D0.18", "D0.19", "D0.20", "D0.21", "D0.22", "D0.23",
      "D0.24", "D0.25", "D0.26", "D0.27", "D0.28", "D0.29", "D0.30", "D0.31", },
    { "D1Re0", "D1Ar5", "D1Ar3", "D1Ar1", "D1RtP", "D1.5",  "D1.6",  "D1.7",
      "D1.8",  "D1.9",  "D1.10", "D1.11", "D1.12", "D1.13", "D1.14", "D1.15",
      "D1.16", "D1.17", "D1.18", "D1.19", "D1.20", "D1.21", "D1.22", "D1.23",
      "D1.24", "D1.25", "D1.26", "D1.27", "D1.28", "D1.29", "D1.30", "D1.31", },
};

static const char *meta_areg_names[2][16] = {
    { "A0StP", "A0FrP", "A0.2",  "A0.3",  "A0.4",  "A0.5",  "A0.6",  "A0.7",
      "A0.8",  "A0.9",  "A0.10", "A0.11", "A0.12", "A0.13", "A0.14", "A0.15", },
    { "A1GbP", "A1LbP", "A1.2",  "A1.3",  "A1.4",  "A1.5",  "A1.6",  "A1.7",
      "A1.8",  "A1.9",  "A1.10", "A1.11", "A1.12", "A1.13", "A1.14", "A1.15", },
};

static const char *meta_pcreg_names[META_PC_MAX] = {
    "PC",   "PCX",
};

static const char *meta_port_names[META_RA_MAX] = {
    [META_RA]       = "RA",
    [META_RAPF]     = "RAPF",
    [META_RAM8X32]  = "RAM8X32",
    [META_RAM8X]    = "RAM8X",
    [META_RABZ]     = "RABZ",
    [META_RAWZ]     = "RAWZ",
    [META_RADZ]     = "RADZ",
    [META_RABX]     = "RABX",
    [META_RAWX]     = "RAWX",
    [META_RADX]     = "RADX",
    [META_RAM16X]   = "RAM16X",
};

static const char *meta_treg_names[META_TR_MAX] = {
    "TXSTAT",       "TXMASK",       "TXSTATI",      "TXMASKI",
    "TXPOLL",       "TXGPIOI",      "TXPOLLI",      "TXGPIOO",
};

static const char *meta_ttreg_names[META_TT_MAX] = {
    "TTEXEC",       "TTCTRL",       "TTMARK",       "TTREC",
    "GTEXEC",
};

static const char *meta_fxreg_names[16] = {
    "FX.0",  "FX.1",  "FX.2",  "FX.3",  "FX.4",  "FX.5",  "FX.6",  "FX.7",
    "FX.8",  "FX.9",  "FX.10", "FX.11", "FX.12", "FX.13", "FX.14", "FX.15",
};

static const char **meta_reg_names[] = {
    [META_UNIT_CT]    = meta_creg_names,
    [META_UNIT_D0]    = meta_dreg_names[0],
    [META_UNIT_D1]    = meta_dreg_names[1],
    [META_UNIT_A0]    = meta_areg_names[0],
    [META_UNIT_A1]    = meta_areg_names[1],
    [META_UNIT_PC]    = meta_pcreg_names,
    [META_UNIT_RA]    = meta_port_names,
    [META_UNIT_TR]    = meta_treg_names,
    [META_UNIT_TT]    = meta_ttreg_names,
    [META_UNIT_FX]    = meta_fxreg_names,
};

static const char *meta_cc_names[16] = {
    "A",  "EQ", "NE", "CS",
    "CC", "N",  "PL", "VS",
    "VC", "HI", "LS", "GE",
    "LT", "GT", "LE", "NV",
};

static const char *width_names[] = {
    "B", "W", "D", "L",
};

const char *cmp_ext_op_names[] = {
    [OP_FFB]  = "FFB",
    [OP_NORM] = "NORM",
    [OP_MORT] = "MORT",
    [OP_MIN]  = "MIN",
    [OP_MAX]  = "MAX",
    [OP_ABS]  = "ABS",
    [OP_NMIN] = "NMIN",
};

typedef enum {
    LOG_DECIMAL = (1 << 0),
    LOG_TOP     = (1 << 1),
} SrcLogFlags;

static inline void meta_log_source(const MetaInstructionSource *src,
                                   SrcLogFlags flags, const char *post)
{
    int du = src->u - META_UNIT_D0;

    switch (src->type) {
    case SRC_IMM: {
            int32_t val = src->i;

            if (flags & LOG_TOP) {
                val = (uint32_t)val >> 16;
            }

            if (flags & LOG_DECIMAL) {
                qemu_log("#%d%s", val, post);
            } else if ((val < 0) && !(flags & LOG_TOP)) {
                qemu_log("#-0x%x%s", -val, post);
            } else {
                qemu_log("#0x%x%s", val, post);
            }
            break;
        }

    case SRC_DSPRAM: {
            int bank = (src->i >> 3) & 0x1;
            int rptr = (src->i >> 2) & 0x1;
            int incr = (src->i >> 1) & 0x1;
            int inci = (src->i >> 0) & 0x1;

            const char *str_idx[2] = { "0", "1" };
            const char *str_bank[2] = { "A", "B" };

            qemu_log("[D%d%sR.%d%s%s%s%s%s%s%s]%s",
                     du, str_bank[bank], rptr,
                     (inci && !incr) ? "++" : ((incr || inci) ? "+" : ""),
                     incr ? "D" : "",
                     incr ? str_idx[du] : "",
                     incr ? str_bank[bank] : "",
                     incr ? "RI." : "",
                     incr ? str_idx[inci] : "",
                     incr ? "++" : "",
                     post);
            break;
        }

    case SRC_ACCUM: {
            qemu_log("AC%d.%d%s",
                     du, src->i,
                     post);
            break;
        }

    case SRC_REG:
            if (src->u == META_UNIT_RA) {
                if (src->i == META_RD) {
                    qemu_log("RD%s", post);
                } else {
                    qemu_log("??RA.%d??%s", src->i, post);
                }
            } else {
                qemu_log("%s%s",
                         meta_reg_names[src->u][src->i],
                         post);
            }
        break;

    case SRC_DSPREG: {
            if (src->i & 0x10) {
                qemu_log("AC%d.%d%s", du, src->i & 0x3, post);
            } else {
                bool inc = (src->i >> 3) & 0x1;
                int bank = (src->i >> 2) & 0x1;
                bool write = (src->i >> 1) & 0x1;
                int idx = src->i & 0x1;

                qemu_log("D%d%s%s%s.%d%s",
                         du,
                         bank ? "B" : "A",
                         write ? "W" : "R",
                         inc ? "I" : "",
                         idx,
                         post);
            }
            break;
        }

    default:
        assert(false);
    }
}

static inline void meta_log_dest(const MetaInstructionDest *dst,
                                 const char *post)
{
    int du = dst->u - META_UNIT_D0;

    switch (dst->type) {
    case DST_DSPRAM: {
            int bank = (dst->i >> 3) & 0x1;
            int rptr = (dst->i >> 2) & 0x1;
            int incr = (dst->i >> 1) & 0x1;
            int inci = (dst->i >> 0) & 0x1;

            const char *str_idx[2] = { "0", "1" };
            const char *str_bank[2] = { "A", "B" };

            qemu_log("[D%d%sW.%d%s%s%s%s%s%s%s]%s",
                     du, str_bank[bank], rptr,
                     (inci && !incr) ? "++" : ((incr || inci) ? "+" : ""),
                     incr ? "D" : "",
                     incr ? str_idx[du] : "",
                     incr ? str_bank[bank] : "",
                     incr ? "WI." : "",
                     incr ? str_idx[inci] : "",
                     incr ? "++" : "",
                     post);
            break;
        }

    case DST_ACCUM: {
            qemu_log("AC%d.%d%s",
                     du, dst->i,
                     post);
            break;
        }

    case DST_REG:
        qemu_log("%s%s",
                 meta_reg_names[dst->u][dst->i],
                 post);
        break;

    case DST_DSPREG: {
            if (dst->i & 0x10) {
                qemu_log("AC%d.%d%s", du, dst->i & 0x3, post);
            } else {
                bool inc = (dst->i >> 3) & 0x1;
                int bank = (dst->i >> 2) & 0x1;
                bool write = (dst->i >> 1) & 0x1;
                int idx = dst->i & 0x1;

                qemu_log("D%d%s%s%s.%d%s",
                         du,
                         bank ? "B" : "A",
                         write ? "W" : "R",
                         inc ? "I" : "",
                         idx,
                         post);
            }
            break;
        }

    default:
        assert(false);
    }
}

static void meta_log_source_rmask(MetaOpWidth width, const char *pre,
                                  const MetaInstructionSource *src,
                                  uint16_t rmask, const char *post)
{
    int i = 0;

    qemu_log("%s", pre);

    while (rmask) {
        if (rmask & 0x1) {
            MetaInstructionSource s = *src;
            bool last = !(rmask & ~0x1);
            s.i += i * meta_lsm_reg_inc(s.u, width);
            meta_log_source(&s, 0, last ? post : ",");
        }

        rmask >>= 1;
        i++;
    }
}

static void meta_log_dest_rmask(MetaOpWidth width, const char *pre,
                                const MetaInstructionDest *dst,
                                uint16_t rmask, const char *post)
{
    int i = 0;

    qemu_log("%s", pre);

    while (rmask) {
        if (rmask & 0x1) {
            MetaInstructionDest d = *dst;
            bool last = !(rmask & ~0x1);
            d.i += i * meta_lsm_reg_inc(d.u, width);
            meta_log_dest(&d, last ? post : ",");
        }

        rmask >>= 1;
        i++;
    }
}

static void log_duaddsub(uint32_t pc, const MetaInstruction *inst)
{
    const MetaInstructionDUAddSub *as = &inst->duaddsub;
    bool movneg = (inst->src1.type == SRC_IMM) && !inst->src1.i;

    if (inst->dsp) {
        qemu_log("%s%s%s",
                 inst->dual ? "L" : "",
                 as->d.mod ? "M" : "",
                 as->d.pshift ? "O" : "");
    }
    qemu_log("\t%s%s%s%s%s\t",
             movneg ? (as->sub ? "NEG" : "MOV") : (as->sub ? "SUB" : "ADD"),
             as->setflags ? "S" : "",
             as->d.split8 ? (as->d.split8_upper ? "T" : "B") :
                            (as->top ? "T" : ""),
             as->d.split8 ? "8" : "",
             inst->cc ? meta_cc_names[inst->cc] : "");
    meta_log_dest(&inst->dst, ",");
    if (!movneg) {
        meta_log_source(&inst->src1, 0, ",");
    }
    meta_log_source(&inst->src2, as->top ? LOG_TOP : 0, "\n");
}

static void log_andorxor(uint32_t pc, const MetaInstruction *inst)
{
    const MetaInstructionAndOrXor *aox = &inst->andorxor;

    if (inst->dsp) {
        qemu_log("%s",
                 inst->dual ? "L" : "");
    }
    qemu_log("\t%s%s%s%s%s\t",
             (aox->op == OP_AND) ? "AND" : ((aox->op == OP_OR) ? "OR" : "XOR"),
             aox->setflags ? "S" : "",
             aox->mask ? "M" : "",
             aox->top ? "T" : "",
             inst->cc ? meta_cc_names[inst->cc] : "");
    meta_log_dest(&inst->dst, ",");
    meta_log_source(&inst->src1, 0, ",");
    meta_log_source(&inst->src2, aox->top ? LOG_TOP : 0, "\n");
}

static void log_shift(uint32_t pc, const MetaInstruction *inst)
{
    const MetaInstructionShift *sh = &inst->shift;

    if (inst->dsp) {
        qemu_log("%s%s",
                 inst->dual ? "L" : "",
                 sh->rspp == RSPP_ROUND ? "R" :
                     (sh->rspp == RSPP_SATS9 ? "G" :
                         (sh->rspp == RSPP_SATU8 ? "B" : "")));
    }
    qemu_log("\t%sS%s%s%s\t",
             sh->arithmetic ? "A" : "L",
             sh->right ? "R" : "L",
             sh->setflags ? "S" : "",
             inst->cc ? meta_cc_names[inst->cc] : "");
    meta_log_dest(&inst->dst, ",");
    meta_log_source(&inst->src1, 0, ",");
    meta_log_source(&inst->src2, 0, "\n");
}

static void log_mul(uint32_t pc, const MetaInstruction *inst)
{
    const MetaInstructionMul *mul = &inst->mul;

    if (inst->dsp) {
        const char *acc_mods[4] = { "", "Z", "P", "N" };

        qemu_log("%s%s%s%s",
                 inst->dual ? "L" : "",
                 mul->sign ? "" : "U",
                 mul->d.mod ? "M" : "",
                 acc_mods[mul->d.acc]);
    }
    qemu_log("\tMUL%s%s%s%s\t",
             inst->dsp ? "" : (mul->l2 ? "D" : "W"),
             mul->d.split8 ? (mul->d.split8_upper ? "T" : "B") : "",
             mul->d.split8 ? "8" : "",
             inst->cc ? meta_cc_names[inst->cc] : "");
    meta_log_dest(&inst->dst, ",");
    meta_log_source(&inst->src1, 0, ",");
    meta_log_source(&inst->src2, 0, "\n");
}

static void log_cmptst(uint32_t pc, const MetaInstruction *inst)
{
    const MetaInstructionCmpTst *ct = &inst->cmptst;

    if (inst->dsp) {
        qemu_log(inst->dual ? "L" : "");
    }
    qemu_log("\t%s%s%s%s\t",
             (ct->ext_op != OP_NONE) ? cmp_ext_op_names[ct->ext_op] :
                 (ct->tst ? "TST" : "CMP"),
             ct->mask ? "M" : "",
             ct->top ? "T" : "",
             inst->cc ? meta_cc_names[inst->cc] : "");
    if (inst->dst.type != DST_NONE) {
        meta_log_dest(&inst->dst, ",");
    }
    if (inst->src2.type != SRC_NONE) {
        meta_log_source(&inst->src1, 0, ",");
        meta_log_source(&inst->src2,
                        ct->top ? LOG_TOP : 0,
                        "\n");
    } else {
        meta_log_source(&inst->src1, 0, "\n");
    }
}

static void log_dspreg_init(uint32_t pc, const MetaInstruction *inst)
{
    if (inst->dsp) {
        qemu_log(inst->dual ? "L" : "");
    }
    qemu_log("\tMOV\t");
    meta_log_dest(&inst->dst, ",");
    meta_log_source(&inst->src1, 0, "\n");
}

static void log_dspreg_copy(uint32_t pc, const MetaInstruction *inst)
{
    if (inst->dsp) {
        qemu_log(inst->dual ? "L" : "");
    }
    qemu_log("\tMOV\t");
    meta_log_dest(&inst->dst, ",");
    meta_log_source(&inst->src1, 0, "\n");
}

static void log_dspreg_get(uint32_t pc, const MetaInstruction *inst)
{
    const MetaInstructionDspGetSet *gs = &inst->dspgetset;

    if (inst->dsp) {
        bool top =
            (inst->dst.type == DST_DSPREG) &&
            ((inst->dst.i & 0x18) == 0x18);

        qemu_log("%s",
                 top ? "H" : "");
    }
    qemu_log("\tGET%s\t",
             gs->l1 ? "L" : "D");
    meta_log_dest(&inst->dst, ",[");
    meta_log_source(&gs->a_base, 0, "");
    if (gs->a_off.type == SRC_IMM) {
        if (gs->a_off.i == 1) {
            qemu_log("++");
        } else if (gs->a_off.i == -1) {
            qemu_log("--");
        } else if (gs->a_off.i) {
            qemu_log("+#%d++", gs->a_off.i);
        }
    } else {
        qemu_log("+");
        meta_log_source(&gs->a_off, 0, "++");
    }
    qemu_log("]\n");
}

static void log_dspreg_set(uint32_t pc, const MetaInstruction *inst)
{
    const MetaInstructionDspGetSet *gs = &inst->dspgetset;

    if (inst->dsp) {
        bool top =
            (inst->src1.type == SRC_DSPREG) &&
            ((inst->src1.i & 0x18) == 0x18);

        qemu_log(top ? "H" : "");
    }
    qemu_log("\tSET%s\t[",
             gs->l1 ? "L" : "D");
    meta_log_source(&gs->a_base, 0, "");
    if (gs->a_off.type == SRC_IMM) {
        if (gs->a_off.i == 1) {
            qemu_log("++");
        } else if (gs->a_off.i == -1) {
            qemu_log("--");
        } else if (gs->a_off.i) {
            qemu_log("+#%d++", gs->a_off.i);
        }
    } else {
        qemu_log("+");
        meta_log_source(&gs->a_off, 0, "++");
    }
    qemu_log("],");
    meta_log_source(&inst->src1, 0, "\n");
}

static void log_xsd(uint32_t pc, const MetaInstruction *inst)
{
    const MetaInstructionXsd *xsd = &inst->xsd;

    if (inst->dsp) {
        qemu_log(inst->dual ? "L" : "");
    }
    qemu_log("\tXSD%s%s\t",
             xsd->setflags ? "S" : "",
             xsd->width ? "W" : "B");
    meta_log_dest(&inst->dst, ",");
    meta_log_source(&inst->src1, 0, "\n");
}

static void log_bex(uint32_t pc, const MetaInstruction *inst)
{
    const MetaInstructionBex *bex = &inst->bex;

    qemu_log("\tBEX%s%s\t",
             bex->setflags ? "S" : "",
             inst->dual ? "L" : "");
    meta_log_dest(&inst->dst, ",");
    meta_log_source(&inst->src1, 0, "\n");
}

static void log_rtd(uint32_t pc, const MetaInstruction *inst)
{
    const MetaInstructionRtd *rtd = &inst->rtd;

    if (inst->dsp) {
        qemu_log(inst->dual ? "L" : "");
    }
    qemu_log("\tRTD%sW\t",
             rtd->setflags ? "S" : "");
    meta_log_dest(&inst->dst, ",");
    meta_log_source(&inst->src1, 0, "\n");
}

static void log_porttounit(uint32_t pc, const MetaInstruction *inst)
{
    const MetaInstructionPortToUnit *ptu = &inst->porttounit;

    qemu_log("\tMOV%s%s\t",
             width_names[ptu->width],
             inst->cc ? meta_cc_names[inst->cc] : "");
    meta_log_dest(&inst->dst, ",");
    if (ptu->width == WIDTH_64B) {
        meta_log_dest(&ptu->dst2, ",");
    }
    meta_log_source(&inst->src1, 0, "\n");
}

static void log_branch(uint32_t pc, const MetaInstruction *inst)
{
    const MetaInstructionBranch *br = &inst->branch;

    if ((inst->cc == META_CC_NV) &&
        !br->repeat &&
        (br->offset == -4)) {
        qemu_log("\tNOP\n");
    } else {
        qemu_log("\tB%s%s\t0x%08x\n",
                 inst->cc ? meta_cc_names[inst->cc] : "",
                 br->repeat ? "R" : "",
                 pc + br->offset);
    }
}

static void log_auaddsub(uint32_t pc, const MetaInstruction *inst)
{
    const MetaInstructionAUAddSub *as = &inst->auaddsub;

    qemu_log("\t%s%s%s\t",
             ((inst->src1.type == SRC_IMM) && !inst->src1.i) ?
                 (as->sub ? "NEG" : "MOV") : (as->sub ? "SUB" : "ADD"),
             as->top ? "T" : "",
             inst->cc ? meta_cc_names[inst->cc] : "");
    meta_log_dest(&inst->dst, ",");
    if (as->pc_src1) {
        qemu_log("CPC%d,", inst->src1.u - META_UNIT_A0);
    } else if ((inst->src1.type != SRC_IMM) || inst->src1.i) {
        meta_log_source(&inst->src1, 0, ",");
    }
    if (as->pc_src2) {
        qemu_log("CPC%d\n", inst->src2.u - META_UNIT_A0);
    } else {
        meta_log_source(&inst->src2,
                        as->top ? LOG_TOP : 0,
                        "\n");
    }
}

static void log_xfr(uint32_t pc, const MetaInstruction *inst)
{
    const MetaInstructionXfr *xfr = &inst->xfr;

    qemu_log("\tXFR%s\t[",
             xfr->l1 ? "L" : "D");
    meta_log_source(&xfr->base_dst, 0,
                    (xfr->update_dst && !xfr->post) ? "++" : "+");
    meta_log_source(&xfr->off_dst, 0,
                    (xfr->update_dst && xfr->post) ? "++],[" : "],[");
    meta_log_source(&xfr->base_src, 0,
                    (xfr->update_src && !xfr->post) ? "++" : "+");
    meta_log_source(&xfr->off_src, 0,
                    (xfr->update_src && xfr->post) ? "++]\n" : "]\n");
}

static void log_getset(uint32_t pc, const MetaInstruction *inst)
{
    const MetaInstructionGet *get = &inst->get;
    uint32_t inc_one = 1 << get->width;

    if (get->read == READ_FLUSH) {
        qemu_log("\tMDRD\t#%d\n",
                 (int)NBITSSET(inst->step));
        return;
    }

    qemu_log("%s\t%s%s%s%s%s\t",
             get->mx ? "W" : "",
             inst->multistep ? "M" : "",
             get->linked ? "LNK" : "",
             ((get->read == READ_DRAIN) ||
              (inst->dst.u == META_UNIT_RA)) ?
                 "MOV" :
                 ((inst->op == OP_GET) ? "GET" : "SET"),
             width_names[get->width],
             inst->cc ? meta_cc_names[inst->cc] : "");

    if (inst->op == OP_GET) {
        /* log destination(s) */
        if (inst->multistep) {
            meta_log_dest_rmask(get->width, "", &inst->dst, inst->step, ",");
        } else {
            meta_log_dest(&inst->dst, ",");
            if (get->width == WIDTH_64B) {
                MetaInstructionDest dst_hi = meta_hi_dest(inst->dst);
                meta_log_dest(&dst_hi, ",");
            }
        }
    }

    if (get->read == READ_DRAIN) {
        qemu_log("RD");
    } else {
        qemu_log("[");
        if ((get->off.type == SRC_IMM) &&
            get->update_addr &&
            !get->post) {
            if (get->off.i == inc_one) {
                qemu_log("++");
            } else if (get->off.i == -inc_one) {
                qemu_log("--");
            }
        }
        meta_log_source(&get->base, 0, "");
        if (get->off.type == SRC_IMM) {
            if (!get->off.i) {
                /* no increment */
            } else if (!get->update_addr) {
                qemu_log("+#%d", get->off.i);
            } else if (get->post) {
                if (get->off.i == inc_one ||
                    get->read == READ_PRIME) {
                    qemu_log("++");
                } else if (get->off.i == -inc_one) {
                    qemu_log("--");
                } else {
                    qemu_log("+#%d++", get->off.i);
                }
            } else if ((get->off.i != inc_one) &&
                       (get->off.i != -inc_one)) {
                qemu_log("++#%d", get->off.i);
            }
        } else if (get->off.type != SRC_NONE) {
            qemu_log((get->update_addr && !get->post) ? "++" : "+");
            meta_log_source(&get->off, 0,
                            (get->update_addr && get->post) ? "++" : "");
        }
        qemu_log("]");
    }

    if (inst->op == OP_SET) {
        /* log source(s) */
        if (inst->multistep) {
            meta_log_source_rmask(get->width, ",",
                                  &inst->src1, inst->step, "");
        } else {
            qemu_log(",");
            meta_log_source(&inst->src1, 0, "");
            if (get->width == WIDTH_64B) {
                MetaInstructionSource src_hi = meta_hi_source(inst->src1);
                qemu_log(",");
                meta_log_source(&src_hi, 0, "");
            }
        }
    }

    qemu_log("\n");
}

static void log_unittounit(uint32_t pc, const MetaInstruction *inst)
{
    const MetaInstructionUnitToUnit *utu = &inst->unittounit;

    qemu_log("\t%s%s%s%s\t",
             utu->defr ? "DEFR" :
                 (utu->kick ? "KICK" :
                      (utu->swap ? "SWAP" : "MOV")),
             (utu->width == WIDTH_64B) ? "L" : "",
             utu->tt ? "TT" : "",
             inst->cc ? meta_cc_names[inst->cc] : "");
    meta_log_dest(&inst->dst, ",");
    meta_log_source(&inst->src1, 0, "");
    if (utu->width == WIDTH_64B) {
        qemu_log(",");
        meta_log_source(&inst->src2, 0, "");
    }
    qemu_log("\n");
}

static void log_rti(uint32_t pc, const MetaInstruction *inst)
{
    qemu_log("\tRTI\n");
}

static void log_rth(uint32_t pc, const MetaInstruction *inst)
{
    qemu_log("\tRTH\n");
}

static void log_lock(uint32_t pc, const MetaInstruction *inst)
{
    qemu_log("\tLOCK%d\n",
             inst->lock.n);
}

static void log_jump(uint32_t pc, const MetaInstruction *inst)
{
    qemu_log("\t%s%s\t",
             inst->jump.call ? "CALL" : "JUMP",
             inst->jump.rel ? "R" : "");
    meta_log_source(&inst->src1, 0, ",");
    qemu_log("#0x%x\n",
             inst->jump.offset + (inst->jump.rel ? pc : 0));
}

static void log_switch(uint32_t pc, const MetaInstruction *inst)
{
    qemu_log("\tSWITCH\t#0x%06x\n",
             inst->swtch.code);
}

static void log_cache(uint32_t pc, const MetaInstruction *inst)
{
    const MetaInstructionCache *cache = &inst->cache;

    if (cache->op == ICACHE) {
        qemu_log("\tICACHE%s\t#%+d,#0x%x\n",
                 cache->priority ? "R" : "",
                 cache->off.i,
                 cache->lines);
        return;
    }

    if (cache->op == CACHE_R) {
        qemu_log("\tCACHER%s\t",
                 width_names[cache->width]);
        meta_log_dest(&inst->dst, ",");
        if (cache->width == WIDTH_64B) {
            MetaInstructionDest dst_hi = meta_hi_dest(inst->dst);
            meta_log_dest(&dst_hi, ",");
        }
    } else {
        if (cache->op == CACHE_W) {
            qemu_log("\tCACHEW%s\t",
                     width_names[cache->width]);
        } else {
            qemu_log("\tDCACHE\t");
        }
    }

    qemu_log("[");
    meta_log_source(&cache->base, 0, "");
    if (cache->off.i) {
        qemu_log("+#0x%x", cache->off.i);
    }

    if (cache->op == CACHE_R) {
        qemu_log("]\n");
    } else {
        qemu_log("],");
        meta_log_source(&inst->src1, 0, "");
        if (cache->width == WIDTH_64B) {
            MetaInstructionSource src_hi = meta_hi_source(inst->src1);
            qemu_log(",");
            meta_log_source(&src_hi, 0, "");
        }
        qemu_log("\n");
    }
}

static void log_multimov(uint32_t pc, const MetaInstruction *inst)
{
    uint16_t rmask_src = inst->step;
    uint16_t rmask_dst = inst->step;

    if (inst->dst.u == META_UNIT_FX) {
        rmask_dst = (1 << NBITSSET(rmask_dst)) - 1;
    } else {
        rmask_src = (1 << NBITSSET(rmask_src)) - 1;
    }

    qemu_log("\tMMOV%s\t",
             inst->multimov.width == WIDTH_64B ? "L" : "");

    meta_log_dest_rmask(inst->multimov.width, "",
                        &inst->dst, rmask_dst, ",");
    meta_log_source_rmask(inst->multimov.width, "",
                          &inst->src1, rmask_src, "\n");
}

static void log_fpu_mov_imm(uint32_t pc, const MetaInstruction *inst)
{
    qemu_log("%s%s\tMOV\tFX.%d,#0x%x\n",
             inst->fpu.dbl ? "D" : "",
             inst->fpu.paired ? "L" : "",
             inst->dst.i,
             inst->src1.i);
}

static void log_fpu_absmovnegswap(uint32_t pc, const MetaInstruction *inst)
{
    qemu_log("%s%s\t%s%s\tFX.%d,FX.%d\n",
             inst->fpu.dbl ? "D" : "",
             inst->fpu.paired ? "L" : "",
             (inst->op == OP_FPU_MOV_REG) ? "MOV" :
                ((inst->op == OP_FPU_ABS) ? "ABS" :
                    ((inst->op == OP_FPU_NEG) ? "NEG" : "SWAP")),
             inst->cc ? meta_cc_names[inst->cc] : "",
             inst->dst.i,
             inst->src1.i);
}

static void log_fpu_pack(uint32_t pc, const MetaInstruction *inst)
{
    qemu_log("\tPACK\tFX.%d,FX.%d,FX.%d\n",
             inst->dst.i,
             inst->src1.i,
             inst->src2.i);
}

static void log_fpu_arith(uint32_t pc, const MetaInstruction *inst)
{
    qemu_log("%s%s%s\t%s%s%s\tFX.%d,FX.%d,FX.%d\n",
             inst->fpu.dbl ? "D" : "",
             inst->fpu.paired ? "L" : "",
             inst->fpu.arith.invert ? "I" : "",
             inst->op == OP_FPU_ADD ? "ADD" :
                (inst->op == OP_FPU_SUB ? "SUB" : "MUL"),
             inst->fpu.arith.reduction ? "RE" : "",
             inst->cc ? meta_cc_names[inst->cc] : "",
             inst->dst.i,
             inst->src1.i,
             inst->src2.i);
}

static void log_fpu_cmp(uint32_t pc, const MetaInstruction *inst)
{
    qemu_log("%s%s%s%s\tCMP%s\tFX.%d,",
             inst->fpu.dbl ? "D" : "",
             inst->fpu.paired ? "L" : "",
             inst->fpu.cmp.abs ? "A" : "",
             inst->fpu.cmp.quiet ? "Q" : "",
             inst->cc ? meta_cc_names[inst->cc] : "",
             inst->src1.i);
    if (inst->src2.type == SRC_IMM) {
        qemu_log("#%d\n", inst->src2.i);
    } else {
        qemu_log("FX.%d\n", inst->src2.i);
    }
}

static void log_fpu_convert(uint32_t pc, const MetaInstruction *inst)
{
    const MetaInstructionFpuConvert *conv = &inst->fpu.convert;
    static const char *type_names[16] = {
        [0 ... 15] = "ERR",
        [FPU_H]  = "H",
        [FPU_F]  = "F",
        [FPU_I]  = "I",
        [FPU_X]  = "X",
        [FPU_D]  = "D",
        [FPU_L]  = "L",
        [FPU_XL] = "XL",
    };

    qemu_log("%s%s\t%sTO%s%s\tFX.%d,FX.%d",
             inst->fpu.dbl ? "D" : "",
             inst->fpu.paired ? "L" : "",
             type_names[conv->src_type],
             type_names[conv->dst_type],
             inst->cc ? meta_cc_names[inst->cc] : "",
             inst->dst.i,
             inst->src1.i);
    if (inst->src2.type == SRC_IMM) {
        qemu_log(",#%d", inst->src2.i);
    }
    qemu_log("\n");
}

static void log_fpu_minmax(uint32_t pc, const MetaInstruction *inst)
{
    qemu_log("%s%s\t%s%s\tFX.%d,FX.%d,FX.%d\n",
             inst->fpu.dbl ? "D" : "",
             inst->fpu.paired ? "L" : "",
             inst->op == OP_FPU_MIN ? "MIN" : "MAX",
             inst->cc ? meta_cc_names[inst->cc] : "",
             inst->dst.i,
             inst->src1.i,
             inst->src2.i);
}

static void log_fpu_reciprocal(uint32_t pc, const MetaInstruction *inst)
{
    const MetaInstructionFpuRcp *rcp = &inst->fpu.rcp;
    qemu_log("%s%s%s%s\t%s\tFX.%d,FX.%d\n",
             inst->fpu.dbl ? "D" : "",
             inst->fpu.paired ? "L" : "",
             rcp->quiet ? "Q" : "",
             rcp->zero_denorm ? "Z" : "",
             inst->op == OP_FPU_RSQ ? "RSQ" : "RCP",
             inst->dst.i,
             inst->src1.i);
}

static void log_fpu_muz(uint32_t pc, const MetaInstruction *inst)
{
    const MetaInstructionFpuMuz *muz = &inst->fpu.muz;
    qemu_log("%s%s%s%s\tMUZ%s%s\tFX.%d,FX.%d,FX.%d\n",
             inst->fpu.dbl ? "D" : "",
             inst->fpu.paired ? "L" : "",
             muz->invert ? "I" : "",
             muz->quiet ? "Q" : "",
             muz->sub ? "S" : "",
             muz->o3o ? "1" : "",
             inst->dst.i,
             inst->src1.i,
             inst->src2.i);
}

static void log_bad(uint32_t pc, const MetaInstruction *inst)
{
    qemu_log("\tDWORD\t0x%08x\n", inst->raw);
}

typedef void (*log_fn_p)(uint32_t pc, const MetaInstruction *inst);

static log_fn_p log_functions[OP_COUNT] = {
    [0 ... OP_COUNT-1] = log_bad,

    [OP_ANDORXOR]      = log_andorxor,
    [OP_AUADDSUB]      = log_auaddsub,
    [OP_BEX]           = log_bex,
    [OP_BRANCH]        = log_branch,
    [OP_CACHE]         = log_cache,
    [OP_CMPTST]        = log_cmptst,
    [OP_DSPREG_COPY]   = log_dspreg_copy,
    [OP_DSPREG_GET]    = log_dspreg_get,
    [OP_DSPREG_INIT]   = log_dspreg_init,
    [OP_DSPREG_SET]    = log_dspreg_set,
    [OP_DUADDSUB]      = log_duaddsub,
    [OP_FPU_ABS]       = log_fpu_absmovnegswap,
    [OP_FPU_ADD]       = log_fpu_arith,
    [OP_FPU_CMP]       = log_fpu_cmp,
    [OP_FPU_CONVERT]   = log_fpu_convert,
    [OP_FPU_MAX]       = log_fpu_minmax,
    [OP_FPU_MIN]       = log_fpu_minmax,
    [OP_FPU_MOV_IMM]   = log_fpu_mov_imm,
    [OP_FPU_MOV_REG]   = log_fpu_absmovnegswap,
    [OP_FPU_MUL]       = log_fpu_arith,
    [OP_FPU_MUZ]       = log_fpu_muz,
    [OP_FPU_NEG]       = log_fpu_absmovnegswap,
    [OP_FPU_PACK]      = log_fpu_pack,
    [OP_FPU_RCP]       = log_fpu_reciprocal,
    [OP_FPU_RSQ]       = log_fpu_reciprocal,
    [OP_FPU_SUB]       = log_fpu_arith,
    [OP_FPU_SWAP]      = log_fpu_absmovnegswap,
    [OP_GET]           = log_getset,
    [OP_JUMP]          = log_jump,
    [OP_LOCK]          = log_lock,
    [OP_MMOV]          = log_multimov,
    [OP_MUL]           = log_mul,
    [OP_PORTTOUNIT]    = log_porttounit,
    [OP_RTD]           = log_rtd,
    [OP_RTH]           = log_rth,
    [OP_RTI]           = log_rti,
    [OP_SET]           = log_getset,
    [OP_SHIFT]         = log_shift,
    [OP_SWITCH]        = log_switch,
    [OP_UNITTOUNIT]    = log_unittounit,
    [OP_XFR]           = log_xfr,
    [OP_XSD]           = log_xsd,
};

void meta_log_instruction(int mask, uint32_t pc, const MetaInstruction *inst)
{
    if (!qemu_loglevel_mask(mask)) {
        return;
    }

    qemu_log("%s%s",
             inst->dsp ? "D" : "",
             inst->fx ? "F" : "");

    log_functions[inst->op](pc, inst);
}
