#ifndef META_INSTRUCTION_H
#define META_INSTRUCTION_H

#include "cpu.h"

typedef enum {
    OP_BAD = -1,

    OP_ANDORXOR,
    OP_AUADDSUB,
    OP_BEX,
    OP_BRANCH,
    OP_CACHE,
    OP_CMPTST,
    OP_CPRR,
    OP_DSPREG_COPY,
    OP_DSPREG_GET,
    OP_DSPREG_INIT,
    OP_DSPREG_SET,
    OP_DUADDSUB,
    OP_FPU_ABS,
    OP_FPU_ADD,
    OP_FPU_CMP,
    OP_FPU_CONVERT,
    OP_FPU_MAX,
    OP_FPU_MIN,
    OP_FPU_MOV_IMM,
    OP_FPU_MOV_REG,
    OP_FPU_MUL,
    OP_FPU_MUZ,
    OP_FPU_NEG,
    OP_FPU_PACK,
    OP_FPU_RCP,
    OP_FPU_RSQ,
    OP_FPU_SUB,
    OP_FPU_SWAP,
    OP_GET,
    OP_JUMP,
    OP_LOCK,
    OP_MMOV,
    OP_MUL,
    OP_PORTTOUNIT,
    OP_RTD,
    OP_RTH,
    OP_RTI,
    OP_SET,
    OP_SHIFT,
    OP_SWITCH,
    OP_UNITTOUNIT,
    OP_XFR,
    OP_XSD,

    OP_COUNT,
} MetaOp;

typedef enum {
    WIDTH_8B  = 0x0,
    WIDTH_16B = 0x1,
    WIDTH_32B = 0x2,
    WIDTH_64B = 0x3,
} MetaOpWidth;

typedef enum {
    META_CC_A,  /* 0x0 */
    META_CC_EQ,
    META_CC_NE,
    META_CC_CS,
    META_CC_CC, /* 0x4 */
    META_CC_N,
    META_CC_PL,
    META_CC_VS,
    META_CC_VC, /* 0x8 */
    META_CC_HI,
    META_CC_LS,
    META_CC_GE,
    META_CC_LT, /* 0xC */
    META_CC_GT,
    META_CC_LE,
    META_CC_NV,
} MetaCc;

typedef enum {
    META_SCC_A,  /* 0x0 */
    META_SCC_LEQ,
    META_SCC_LNE,
    META_SCC_LCS,
    META_SCC_LCC, /* 0x4 */
    META_SCC_HEQ,
    META_SCC_HNE,
    META_SCC_HCS,
    META_SCC_HCC, /* 0x8 */
    META_SCC_LHI,
    META_SCC_LLS,
    META_SCC_HHI,
    META_SCC_HLS, /* 0xC */
    META_SCC_EEQ,
    META_SCC_ECS,
    META_SCC_NV,
} MetaScc;

typedef enum {
    META_FXCC_A,   /* 0x0 */
    META_FXCC_FEQ,
    META_FXCC_UNE,
    META_FXCC_FLT,
    META_FXCC_UGE, /* 0x4 */
    META_FXCC_FMI,
    META_FXCC_UPL,
    META_FXCC_UVS,
    META_FXCC_FVC, /* 0x8 */
    META_FXCC_UGT,
    META_FXCC_FLE,
    META_FXCC_FGE,
    META_FXCC_ULT, /* 0xC */
    META_FXCC_FGT,
    META_FXCC_ULE,
    META_FXCC_NV,
} MetaFxCc;

typedef enum {
    META_FXPCC_A,   /* 0x0 */
    META_FXPCC_LEQ,
    META_FXPCC_LNE,
    META_FXPCC_LLO,
    META_FXPCC_LHS, /* 0x4 */
    META_FXPCC_HEQ,
    META_FXPCC_HNE,
    META_FXPCC_HLO,
    META_FXPCC_HHS, /* 0x8 */
    META_FXPCC_LGR,
    META_FXPCC_LLE,
    META_FXPCC_HGR,
    META_FXPCC_HLE, /* 0xC */
    META_FXPCC_EEQ,
    META_FXPCC_ELO,
    META_FXPCC_NV,
} MetaFxPairedCc;

typedef enum {
    DST_NONE,
    DST_REG,
    DST_DSPREG,
    DST_DSPRAM,
    DST_ACCUM,
    DST_COPROC,
} MetaInstructionDestType;

typedef struct {
    MetaInstructionDestType type;
    MetaUnit u;
    int i;
} MetaInstructionDest;

typedef enum {
    SRC_NONE,
    SRC_REG,
    SRC_IMM,
    SRC_DSPREG,
    SRC_DSPRAM,
    SRC_ACCUM,
    SRC_COPROC,
} MetaInstructionSourceType;

typedef struct {
    MetaInstructionSourceType type;
    MetaUnit u;
    int i;
} MetaInstructionSource;

static inline MetaInstructionSource meta_hi_source(
                                             const MetaInstructionSource lo)
{
    MetaInstructionSource hi = lo;

    if (lo.u == META_UNIT_FX) {
        hi.i = lo.i + 1;
    } else {
        hi.u = meta_unit_partner(lo.u);
    }

    return hi;
}

static inline MetaInstructionDest meta_hi_dest(const MetaInstructionDest lo)
{
    MetaInstructionDest hi = lo;

    if (lo.u == META_UNIT_FX) {
        hi.i = lo.i + 1;
    } else {
        hi.u = meta_unit_partner(lo.u);
    }

    return hi;
}

typedef struct {
    bool sub;
    bool setflags;
    bool top;

    struct {
        bool mod;
        bool pshift;
        bool split8;
        bool split8_src1;
        bool split8_upper;
    } d;
} MetaInstructionDUAddSub;

typedef struct {
    bool sub;
    bool top;

    bool pc_src1;
    bool pc_src2;
} MetaInstructionAUAddSub;

typedef struct {
    bool right;
    bool arithmetic;
    bool setflags;

    enum {
        RSPP_NONE  = 0,
        RSPP_ROUND = 1,
        RSPP_SATS9 = 2,
        RSPP_SATU8 = 3,
    } rspp;
} MetaInstructionShift;

typedef struct {
    enum {
        OP_AND,
        OP_OR,
        OP_XOR,
    } op;
    bool setflags;
    bool mask;
    bool top;
} MetaInstructionAndOrXor;

typedef struct {
    bool sign;
    bool l2;
    bool top;

    struct {
        bool mod;
        enum {
            ACC_NONE = 0,
            ACC_Z    = 1,
            ACC_P    = 2,
            ACC_N    = 3,
        } acc;

        bool split8;
        bool split8_src1;
        bool split8_upper;
    } d;
} MetaInstructionMul;

typedef struct {
    bool tst;
    bool mask;
    bool top;

    enum {
        OP_NONE = 0x0,
        OP_FFB  = 0x1,
        OP_NORM = 0x2,
        OP_MORT = 0x3,
        OP_MIN  = 0x8,
        OP_MAX  = 0x9,
        OP_ABS  = 0xa,
        OP_NMIN = 0xb,
    } ext_op;

    MetaInstructionSource src2_cmp;
    bool src2_maxneg;
} MetaInstructionCmpTst;

typedef struct {
    bool l1;

    MetaInstructionSource a_base;
    MetaInstructionSource a_off;
} MetaInstructionDspGetSet;

typedef struct {
    bool setflags;

    enum {
        XSDB = 0x0,
        XSDW = 0x1,
    } width;
} MetaInstructionXsd;

typedef struct {
    bool setflags;
} MetaInstructionBex;

typedef struct {
    bool setflags;
} MetaInstructionRtd;

typedef struct {
    MetaOpWidth width;
    MetaInstructionDest dst2;
} MetaInstructionPortToUnit;

typedef struct {
    int32_t offset;
    bool repeat;
} MetaInstructionBranch;

typedef struct {
    bool update_src;
    bool update_dst;
    bool post;
    bool l1;

    MetaInstructionSource base_src;
    MetaInstructionSource base_dst;
    MetaInstructionSource off_src;
    MetaInstructionSource off_dst;
} MetaInstructionXfr;

typedef struct {
    MetaOpWidth width;

    MetaInstructionSource base;
    MetaInstructionSource off;
    bool update_addr;
    bool post;
    bool linked;

    enum {
        READ_NONE = 0,
        READ_PRIME,
        READ_DRAIN,
        READ_FLUSH,
    } read;

    bool mx;
} MetaInstructionGet;

typedef struct {
    bool swap;
    bool kick;
    bool defr;
    bool tt;
    MetaOpWidth width;
} MetaInstructionUnitToUnit;

typedef struct {
    int n;
} MetaInstructionLock;

typedef struct {
    int32_t offset;
    bool call;
    bool rel;
} MetaInstructionJump;

typedef struct {
    uint32_t code;
} MetaInstructionSwitch;

typedef struct {
    enum {
        CACHE_R,
        CACHE_W,
        DCACHE,
        ICACHE,
    } op;
    MetaOpWidth width;
    MetaInstructionSource base;
    MetaInstructionSource off;
    uint8_t lines;
    bool priority;
} MetaInstructionCache;

typedef struct {
    MetaOpWidth width;
} MetaInstructionMultiMov;

typedef struct {
    bool invert;
    bool reduction;
} MetaInstructionFpuArith;

typedef struct {
    bool abs;
    bool quiet;
} MetaInstructionFpuCmp;

typedef enum {
    FPU_16B = 0x0,
    FPU_32B = 0x4,
    FPU_64B = 0x8,

    FPU_H   = FPU_16B | 0x0,

    FPU_F   = FPU_32B | 0x0,
    FPU_I   = FPU_32B | 0x1,
    FPU_X   = FPU_32B | 0x2,

    FPU_D   = FPU_64B | 0x0,
    FPU_L   = FPU_64B | 0x1,
    FPU_XL  = FPU_64B | 0x2,
} MetaFpuDataType;

typedef struct {
    MetaFpuDataType src_type;
    MetaFpuDataType dst_type;
    bool z_rounding;
} MetaInstructionFpuConvert;

typedef struct {
    bool sub;
    bool invert;
    bool quiet;
    bool o3o;
} MetaInstructionFpuMuz;

typedef struct {
    bool invert;
    bool quiet;
    bool zero_denorm;
} MetaInstructionFpuRcp;

typedef struct {
    uint32_t raw;

    MetaOp op;
    MetaCc cc;
    MetaInstructionDest dst;
    MetaInstructionSource src1;
    MetaInstructionSource src2;

    bool dsp;
    bool dual;
    bool fx;

    enum {
        MULTISTEP_NONE = 0,
        MULTISTEP_RMASK,
        MULTISTEP_COUNT,
    } multistep;
    uint16_t step;

    union {
        MetaInstructionAndOrXor andorxor;
        MetaInstructionAUAddSub auaddsub;
        MetaInstructionBex bex;
        MetaInstructionBranch branch;
        MetaInstructionCache cache;
        MetaInstructionCmpTst cmptst;
        MetaInstructionDspGetSet dspgetset;
        MetaInstructionDUAddSub duaddsub;
        MetaInstructionGet get;
        MetaInstructionJump jump;
        MetaInstructionLock lock;
        MetaInstructionMul mul;
        MetaInstructionMultiMov multimov;
        MetaInstructionPortToUnit porttounit;
        MetaInstructionRtd rtd;
        MetaInstructionShift shift;
        MetaInstructionSwitch swtch;
        MetaInstructionUnitToUnit unittounit;
        MetaInstructionXfr xfr;
        MetaInstructionXsd xsd;
    };

    struct {
        union {
            MetaInstructionFpuArith arith;
            MetaInstructionFpuCmp cmp;
            MetaInstructionFpuConvert convert;
            MetaInstructionFpuMuz muz;
            MetaInstructionFpuRcp rcp;
        };

        bool paired;
        bool dbl;

        MetaFxInstInfo info;
    } fpu;
} MetaInstruction;

int meta_decode(MetaInstruction *inst, uint32_t raw);

#endif /* META_INSTRUCTION_H */
