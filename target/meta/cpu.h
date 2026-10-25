/*
 *  META virtual CPU header
 *
 *  Copyright (c) 2010 Imagination Technologies
 *  Written by Graham Whaley
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */
#ifndef CPU_META_H
#define CPU_META_H

#include "cpu-qom.h"
#include "exec/cpu-common.h"
#include "exec/cpu-interrupt.h"
#include "exec/cpu-defs.h"
#include "exec/page-protection.h"
#include "exec/target_page.h"
#include "exec/cputlb.h"
#include "fpu/softfloat-types.h"
#include "core_config.h"
#include "units.h"

typedef struct CPUArchState CPUMETAState;

#define TARGET_HAS_ICE 1
#define TARGET_STACK_GROWS_UP 1

#define META_MAX_THREADS 4

#define META_COREMAJOR(ENV) ((ENV)->global->core_rev >> 24)

#define META_READPORT_DEPTH 6

#if !defined(CONFIG_USER_ONLY)
/* the simulator magic instructions (-sim-magic in the original) are off */
#define sim_magic 0
#endif

typedef struct MetaCore MetaCore;
typedef struct MetaBootInfo MetaBootInfo;

typedef enum MetaISA {
    META1,
    META2,
    MINIM,
} MetaISA;

typedef struct MetaGlobalState {
    /* META revision number */
#define META_COREREV META_COREREV4
#define META_COREREV4(MAJ, MIN, REV, MAINT) \
                (((MAJ)<<24) | ((MIN)<<16) | ((REV)<<8) | (MAINT))
#define META_COREREV3(MAJ, MIN, REV)    META_COREREV(MAJ, MIN, REV, 0)
#define META_COREREV2(MAJ, MIN)         META_COREREV(MAJ, MIN, 0, 0)
#define META_COREREV1(MAJ)              META_COREREV(MAJ, 0, 0, 0)
    uint32_t core_rev;

    /* Core configuration registers (see core_config.h) */
    uint32_t core_id;
    uint32_t core_config2;

    /* Global control unit regs */
    uint32_t cregs[1];
    /* Global address unit regs */
    uint32_t aregs[2][8];
    /* Global data unit regs */
    uint32_t dregs[2][16];
    /* Global trace unit regs */
    uint32_t ttregs[1];

#if !defined(CONFIG_USER_ONLY)
    /* Lock state
     * 3 chunks of META_MAX_THREADS bits:
     * msb [LOCKW][LOCK1][LOCK2] lsb
     * LOCK2 indicates which thread if any has exclusive lock
     * LOCK1 indicates which thread if any has voluntary lock
     * LOCKW indicates which threads are waiting for LOCK1
     */
#define META_LOCK2_THR(T) (1 << (T))
#define META_LOCK2_ALL    ((1 << META_MAX_THREADS) - 1)
#define META_LOCK2_OTH(T) (META_LOCK2_ALL & ~META_LOCK2_THR(T))
#define META_LOCK1_THR(T) ((1 << META_MAX_THREADS) << (T))
#define META_LOCK1_ALL    (META_LOCK2_ALL << META_MAX_THREADS)
#define META_LOCK1_OTH(T) (META_LOCK1_ALL & ~META_LOCK1_THR(T))
#define META_LOCKW_THR(T) ((1 << (META_MAX_THREADS*2)) << (T))
#define META_LOCKW_ALL    (META_LOCK1_ALL << META_MAX_THREADS)
#define META_LOCKW_OTH(T) (META_LOCKW_ALL & ~META_LOCKW_THR(T))
    /* LOCK1 or LOCK2 */
#define META_LOCK_THR(T)  (META_LOCK2_THR(T) | META_LOCK1_THR(T))
    /* LOCK1, LOCK2, or LOCKW */
#define META_LOCKA_THR(T) (META_LOCK_THR(T) | META_LOCKW_THR(T))
    /* LOCKW or another has LOCK2 */
#define META_LOCKH_THR(T) (META_LOCKW_THR(T) | META_LOCK2_OTH(T))
    uint32_t lock;

    /* MMU idx */
    uint8_t mmu;
    /* MMU mode flags */
#define META_MMU_HTP       0x1 /* HTP enhanced mode (rather than ATP compat) */
#define META_MMU_EB_DCACHE 0x2
#define META_MMU_EB_ICACHE 0x4
    uint8_t mmu_flags;
    /* MMU root page table address (ATP) */
    uint32_t mmu_ptroot;
#endif

    /* Code breakpoint counters (CODEBxCTRL.IBKxCount) */
    uint32_t codebcount[4];

    /* Accumulators */
    uint64_t accregs[2][3];

    /* DSP RAM address size */
    uint8_t dspram_addr_bits;

    /* DSP RAM [du][bank][off] */
    uint32_t dspram[2][2][1 << 16];
} MetaGlobalState;

/* Block states */
typedef enum {
    META_BLOCK_NONE = 0,
    META_BLOCK_OFF,
    META_BLOCK_TXSTAT,
    META_BLOCK_TXSTATI,
    META_BLOCK_HALT,
} MetaBlock;

/* Thread capabilities */
typedef enum {
    /* DSP types */
    META_TCAPS_DSP      = 0x0,  /* Full DSP */
    META_TCAPS_LDSP     = 0x8,  /* Lite DSP, see CORE_ID, CORE_CONFIG2 */
    META_TCAPS_NODSP    = 0xc,  /* No DSP */
    /* FPU types */
    META_TCAPS_FPU      = 0x0,  /* Full FPU */
    META_TCAPS_LFPU     = 0x2,  /* Lite FPU */
    META_TCAPS_FPU16    = 0x0,  /* 16 FPU registers */
    META_TCAPS_FPU8     = 0x1,  /* 8 FPU registers */

    /* Combinations */
    META_TCAPS_GP       = META_TCAPS_NODSP,
} MetaTCaps;

/* avoid circular #include */
struct ptimer_state;

#define META_THREAD2CORE(T) ((struct MetaCore *)(T)->global)

struct CPUArchState {
    MetaGlobalState *global;
    unsigned int thread_num;
    MetaTCaps tcaps;

    /* Control unit regs */
    uint32_t cregs[31];
    /* Address unit regs */
    uint32_t aregs[2][8];
    /* Data unit regs */
    uint32_t dregs[2][16];
    /* The two PCs */
    uint32_t pc[2];
    /* Trigger unit regs */
    uint32_t tregs[8];
    /* Trace unit regs */
    uint32_t ttregs[4];
    /* Floating point unit regs */
    uint32_t fxregs[16];

    /* condition flags (bottom bits of TXSTATUS) */
    uint32_t cf[5];
    /* load/store multiple step (TXSTATUS.LSM_STEP) */
    uint32_t lsmstep;

    /* lnkget address (0 if none) */
    uint32_t lnkaddr;

    /* repeated cycles on top of instructions executed */
    uint32_t repcyc;

    /* catch replay should happen immediately (1) or is in progress (2) */
    int catch_replay;

    /* Accumulators */
    uint64_t accregs[2][1];

    /* DSP RAM pointers ([DU][Ram][PtrIdx]) */
    uint32_t dspram_rp[2][2][2];
    uint32_t dspram_rpi[2][2][2];
    uint32_t dspram_wp[2][2][2];
    uint32_t dspram_wpi[2][2][2];

    /* DSP RAM prefetched data ([DU][Ram][PtrIdx]) */
    uint32_t dspram_fetched[2][2][2];

    /* TXDRCTRL fields */
    uint32_t dspram_off_and[2];
    uint32_t dspram_off_or[2];

    /* AU modulo addressing */
    uint32_t aumod_mask;

#if !defined(CONFIG_USER_ONLY)
    /* timer data */
    struct ptimer_state *txtimer, *txtimeri;
    uint32_t timer_period; /* ns */
#endif

    /*
     * Per thread page tables (HTP) [local(0)/global(2) | 0/1].
     * see META_MMCUTBLPHYS{0,1}_* below
     */
    uint32_t mmu_tblphys[4];

    /* trigger mask we're blocked on (see MetaBlock) */
    uint32_t block;


    /* kick count */
    uint32_t kicks, ikicks;

    /* switch code */
    uint32_t switch_code;
    uint32_t switch_nextpc;

    target_ulong tls_value; /* for User Mode Emulation */

    /* FPU state */
    struct {
        float_status status;
    } fx;

    /* read ports */
    uint64_t readport_data[META_READPORT_DEPTH];
    int readport_idx_w;
    int readport_idx_r;
    int readport_count;

    /* simulator output */
    bool sim_active;

    /* where a memory fault resumes, see meta_restore_state_to_opc() */
    uint32_t fault_next_pc;
    uint32_t fault_lsmstep;
    uint32_t fault_catch0;
    uint32_t fault_catch1;

    /* Fields up to this point are cleared by a CPU reset */
    struct {} end_reset_fields;

    struct MetaBootInfo *boot_info;
};

struct METACPUClass {
    CPUClass parent_class;

    DeviceRealize parent_realize;
    ResettablePhases parent_phases;
};

struct ArchCPU {
    CPUState parent_obj;

    CPUMETAState env;
};

/* Get whether a thread has at least reduced DSP support */
static inline int meta_dsp_supported(CPUArchState *env)
{
    return (env->tcaps & META_TCAPS_NODSP) != META_TCAPS_NODSP;
}

/* Get whether a thread has at least single precision FPU support */
static inline int meta_fpu_supported(CPUArchState *env)
{
    return !!(env->global->core_id & META_CORECFG_FPU_TYPE_M);
}

static inline int meta_lsm_reg_inc(MetaUnit unit, int width_log2)
{
    if (unit == META_UNIT_RA) {
        return 0;
    }
    if (unit == META_UNIT_FX) {
        return (width_log2 == 3) ? 2 : 1;
    }
    return 1;
}

/* register enums */
typedef enum {
    META_TXENABLE,      /* 0x00 */
    META_TXMODE,
    META_TXSTATUS,
    META_TXRPT,
    META_TXTIMER,       /* 0x04 */
    META_TXL1START,
    META_TXL1END,
    META_TXL1COUNT,
    META_TXL2START,     /* 0x08 */
    META_TXL2END,
    META_TXL2COUNT,
    META_TXBPOBITS,
    META_TXMRSIZE,      /* 0x0C */
    META_TXTIMERI,
    META_TXDRCTRL,
    META_TXDRSIZE,
    META_TXCATCH0,      /* 0x10 */
    META_TXCATCH1,
    META_TXCATCH2,
    META_TXCATCH3,
    META_TXDEFR,        /* 0x14 */  /* HTP */
    META_CT21,
    META_TXCLKCTRL,                 /* HTP */
    META_TXSTATE,
    META_TXINTERN0 = META_TXSTATE,  /* HTP */
    META_TXAMAREG0,     /* 0x18 */
    META_TXAMAREG1,
    META_TXAMAREG2,
    META_TXAMAREG3,
    META_TXDIVTIME,     /* 0x1C */
    META_TXPRIVEXT,
    META_TXTACTCYC,
    META_TXIDLECYC,
    META_CT_MAX,
} MetaControlReg;

typedef enum {
    META_PC,            /* 0x00 */
    META_PCX,
    META_PC_MAX,
} MetaPcReg;

typedef enum {
    /* output */
    META_RA         = 0x10,
    META_RAPF       = 0x11,
    META_RAM8X32    = 0x16,
    META_RAM8X      = 0x17,
    META_RABZ       = 0x18,
    META_RAWZ       = 0x19,
    META_RADZ       = 0x1a,
    META_RABX       = 0x1c,
    META_RAWX       = 0x1d,
    META_RADX       = 0x1e,
    META_RAM16X     = 0x1f,

    /* input */
    META_RD = META_RA,  /* 0x00 */

    META_RA_MAX     = 32,
} MetaPortReg;

typedef enum {
    META_TXSTAT,        /* 0x00 */
    META_TXMASK,
    META_TXSTATI,
    META_TXMASKI,
    META_TXPOLL,        /* 0x04 */
    META_TXGPIOI,
    META_TXPOLLI,
    META_TXGPIOO,
    META_TR_MAX,
} MetaTriggerReg;

typedef enum {
    META_TTEXEC,        /* 0x00 */
    META_TTCTRL,
    META_TTMARK,
    META_TTREC,
    META_GTEXEC,        /* 0x04 */
    META_TT_MAX,
} MetaTraceReg;

/* Condition flags */
typedef enum {
    META_CF_C, /* Carry */
    META_CF_V, /* oVerflow */
    META_CF_N, /* Negative */
    META_CF_Z, /* Zero */

    META_CF_SCC, /* split condition code */

    /* floating point */
    META_FF_C = META_CF_C,
    META_FF_V = META_CF_V,
    META_FF_N = META_CF_N,
    META_FF_Z = META_CF_Z,

    /* split floating point */
    META_SFF_LN = META_CF_C,
    META_SFF_HN = META_CF_V,
    META_SFF_HZ = META_CF_N,
    META_SFF_LZ = META_CF_Z,

    /* split-16 DSP */
    META_SCF_LC = META_CF_C,
    META_SCF_HC = META_CF_V,
    META_SCF_HZ = META_CF_N,
    META_SCF_LZ = META_CF_Z,
} MetaCf;

/* FPU instruction flags */
typedef enum {
    /* TXCATCH0 FPUFlags */
    META_FXINST_O3O  = (1 << 0),
    META_FXINST_N    = (1 << 1),
    META_FXINST_Z    = (1 << 2),
    META_FXINST_Q    = (1 << 3),
    META_FXINST_A    = (1 << 4),

    /* TXCATCH0 FPUWidth */
    META_FXINST_D    = (1 << 5),
    META_FXINST_P    = (1 << 6),

    /* Additional flags used by helpers */
    META_FXINST_PM   = (1 << 7),
    META_FXINST_HIGH = (1 << 8),
    META_FXINST_MAX  = (1 << 9),
} MetaFxInstFlags;

/* FPU instruction info, for exception helper */
typedef struct {
    union {
        struct {
            unsigned int enc: 6;
            unsigned int flags: 10;
        } __attribute__((packed));
        uint16_t raw;
    } op;

    union {
        struct {
            unsigned int rd: 4;
            unsigned int rs1: 4;
            unsigned int rs2: 4;
            unsigned int rs3: 4;
            unsigned int imm: 16;
        } __attribute__((packed));
        uint32_t raw;
    } args;
} MetaFxInstInfo;

#define META_FX_INST_INFO_EMPTY { { { 0, 0 } }, { { 0, 0, 0, 0, 0 } } }

/* FPUOpEnc values */
typedef enum {
    META_FXOPENC_ADD = 0,
    META_FXOPENC_SUB = 1,
    META_FXOPENC_MUL = 2,
    META_FXOPENC_FTOI = 3,
    META_FXOPENC_FTOX = 4,
    META_FXOPENC_ITOF = 5,
    META_FXOPENC_XTOF = 6,
    META_FXOPENC_FTOH = 7,
    META_FXOPENC_HTOF = 8,
    META_FXOPENC_DTOF = 9,
    META_FXOPENC_FTOD = 10,
    META_FXOPENC_DTOL = 11,
    META_FXOPENC_LTOD = 12,
    META_FXOPENC_DTOXL = 13,
    META_FXOPENC_XLTOD = 14,
    META_FXOPENC_CMP = 15,
    META_FXOPENC_MIN = 16,
    META_FXOPENC_MAX = 17,
    META_FXOPENC_ADDRE = 18,
    META_FXOPENC_SUBRE = 19,
    META_FXOPENC_MULRE = 20,

    META_FXOPENC_MUZ = 25,
    META_FXOPENC_MUZS = 26,
    META_FXOPENC_RCP = 27,
    META_FXOPENC_RSQ = 28,
} MetaFxOpEnc;

/* QUICKRoT instruction info, for helper */
typedef union {
    struct {
        bool set_flags: 1;
        bool update_ctrl: 1;
    };
    uint32_t _raw;
} MetaQRInstInfo;

/* Control register fields */

#define META_TXENABLE_TCAPS_MASK    0x0000f000
#define META_TXENABLE_TCAPS_SHIFT   12
#define META_TXENABLE_TSTOPPED_MASK (1 << 2)
#define META_TXENABLE_TOFF_MASK     (1 << 1)
#define META_TXENABLE_THREADEN_MASK (1 << 0)
/* The only values that can be stored in cregs[META_TXENABLE] are: */
typedef enum {
    META_TXENABLE_ENABLED = META_TXENABLE_THREADEN_MASK,
    META_TXENABLE_OFF     = META_TXENABLE_TOFF_MASK,
    META_TXENABLE_STOPPED = META_TXENABLE_OFF |
                            META_TXENABLE_TSTOPPED_MASK,
} MetaTxEnable;

#define META_TXMODE_DSPRRADIX_MASK      (1 << 29)
#define META_TXMODE_FPURMODEGUARD_MASK  (1 << 18)
#define META_TXMODE_FPURMODE_MASK       0x3
#define META_TXMODE_FPURMODE_SHIFT      16
#define META_TXMODE_AU1ADDRMODE_MASK    0x7
#define META_TXMODE_AU1ADDRMODE_SHIFT   12
#define META_TXMODE_AU0ADDRMODE_MASK    0x7
#define META_TXMODE_AU0ADDRMODE_SHIFT   8
#define META_TXMODE_M8_MASK             (1 << 6)
#define META_TXMODE_DUACCSAT_MASK       (1 << 5)
#define META_TXMODE_DUSATURATION_MASK   (1 << 4)
#define META_TXMODE_DUROUNDING_MASK     (1 << 3)
#define META_TXMODE_DUPRODSHIFT_MASK    (1 << 2)
#define META_TXMODE_DUARITHMODE_MASK    0x3
#define META_TXMODE_DUARITHMODE_SHIFT   0

typedef enum {
    META_AUADDR_LINEAR   = 0x0,
    META_AUADDR_MODULO   = 0x3,
    META_AUADDR_BITREV8  = 0x4,
    META_AUADDR_BITREV16 = 0x5,
    META_AUADDR_BITREV32 = 0x6,
    META_AUADDR_BITREV64 = 0x7,
} MetaAUAddrMode;

static MetaAUAddrMode meta_auaddrmode(CPUArchState *env, int au)
{
    int shift = au ? META_TXMODE_AU1ADDRMODE_SHIFT
                   : META_TXMODE_AU0ADDRMODE_SHIFT;
    return (env->cregs[META_TXMODE] >> shift) &
               META_TXMODE_AU1ADDRMODE_MASK;
}

/* All DU mode bits */
#define META_TXMODE_DUMODE_MASK         0x7f

typedef enum {
    META_DUARITH_16X16   = 0,
    META_DUARITH_SPLIT16 = 1,
    META_DUARITH_32X32H  = 2,
    META_DUARITH_32X32L  = 3,
} MetaDUArithMode;

static inline MetaDUArithMode meta_duarithmode(CPUArchState *env)
{
    return env->cregs[META_TXMODE] & META_TXMODE_DUARITHMODE_MASK;
}

#define META_TXSTATUS_FPACTIVE_MASK (1 << 24)
#define META_TXSTATUS_CBIMARKER_MASK (1 << 23)
#define META_TXSTATUS_CBMARKER_MASK (1 << 22)
#define META_TXSTATUS_FREASON_MASK  0x00300000
#define META_TXSTATUS_FREASON_SHIFT 20
#define META_TXSTATUS_HREASON_MASK  0x000c0000
#define META_TXSTATUS_HREASON_SHIFT 18
#define META_TXSTATUS_PSTAT_MASK    0x00020000
#define META_TXSTATUS_PSTAT_SHIFT   17
#define META_TXSTATUS_ISTAT_MASK    0x00010000
#define META_TXSTATUS_LSMSTEP_MASK  0x00000700
#define META_TXSTATUS_LSMSTEP_SHIFT 8
/*
 * This is the mask of what should not be self-written to cregs[TXSTATUS]
 * PSTAT/ISTAT are effectively read-only unless thread disabled
 * LSM step is stored separately
 * Condition flags are stored in cf
 */
#define META_TXSTATUS_ROMASK        0x0003ffff
/* When thread is stopped and other thread has priv */
#define META_TXSTATUS_STOPPEDROMASK 0x0000ffff

/* Reason for a memory fault (TXSTATUS.FReason) */
typedef enum {
    META_FREASON_NOERROR = 0,   /* No error */
    META_FREASON_GENERAL = 1,   /* General violation */
    META_FREASON_PAGE    = 2,   /* Page fault */
    META_FREASON_PROTECT = 3,   /* Protection violation */
} MetaFReason;

extern const char *meta_freason_names[4];

/* Reason for a halt (TXSTATUS.HReason) */
typedef enum {
    META_HREASON_SWITCH  = 0,   /* SWITCH instruction */
    META_HREASON_UNKNOWN = 1,   /* Unknown instruction */
    META_HREASON_PRIV    = 2,   /* Privilege violation */
    META_HREASON_FAULT   = 3,   /* MemFault */
} MetaHReason;

extern const char *meta_hreason_names[4];

/* Combined FReason, HReason signal numbers */
#define META_SIGNUM(FR, HR) (((FR)<<2) | (HR))
#define _META_SIGNUM(FR, HR) META_SIGNUM(META_FREASON_##FR, META_HREASON_##HR)
#define META_SIGNUM_FR(SIGNUM) ((SIGNUM) >> 2)
#define META_SIGNUM_HR(SIGNUM) ((SIGNUM) & 3)
typedef enum {
    META_SIGNUM_IIF = _META_SIGNUM(NOERROR, UNKNOWN), /* IIF=0x1 illegal */
    META_SIGNUM_PFG = _META_SIGNUM(NOERROR, PRIV),    /* PFG=0x2 priv */
    META_SIGNUM_DHF = _META_SIGNUM(NOERROR, FAULT),   /* DHF=0x3 watchpoint/unaligned */
    META_SIGNUM_IGF = _META_SIGNUM(GENERAL, UNKNOWN), /* IGF=0x5 general instruction fault */
    META_SIGNUM_DGF = _META_SIGNUM(GENERAL, FAULT),   /* DGF=0x7 general data fault */
    META_SIGNUM_IPF = _META_SIGNUM(PAGE,    UNKNOWN), /* IPF=0x9 page instruction fault */
    META_SIGNUM_DPF = _META_SIGNUM(PAGE,    FAULT),   /* DPF=0xB page instruction fault */
    META_SIGNUM_IHF = _META_SIGNUM(PROTECT, UNKNOWN), /* IHF=0xD breakpoint*/
    META_SIGNUM_DWF = _META_SIGNUM(PROTECT, FAULT),   /* DWF=0xF read-only */
} MetaSigNum;

#define META_TXDRCTRL_DSPRAMSIZE_MASK   0x001f0000
#define META_TXDRCTRL_DSPRAMSIZE_SHIFT  16
#define META_TXDRCTRL_D1OFFSAND_MASK    0x0000f000
#define META_TXDRCTRL_D1OFFSAND_SHIFT   12
#define META_TXDRCTRL_D1OFFSOR_MASK     0x00000f00
#define META_TXDRCTRL_D1OFFSOR_SHIFT    8
#define META_TXDRCTRL_D0OFFSAND_MASK    0x000000f0
#define META_TXDRCTRL_D0OFFSAND_SHIFT   4
#define META_TXDRCTRL_D0OFFSOR_MASK     0x0000000f
#define META_TXDRCTRL_D0OFFSOR_SHIFT    0

#define META_TXDRSIZE_RAMBRADIXSCALE_MASK  0x70000000
#define META_TXDRSIZE_RAMBRADIXSCALE_SHIFT 28
#define META_TXDRSIZE_RAMBMODSIZE_MASK     0x0fff0000
#define META_TXDRSIZE_RAMBMODSIZE_SHIFT    16
#define META_TXDRSIZE_RAMARADIXSCALE_MASK  0x00007000
#define META_TXDRSIZE_RAMARADIXSCALE_SHIFT 12
#define META_TXDRSIZE_RAMAMODSIZE_MASK     0x00000fff
#define META_TXDRSIZE_RAMAMODSIZE_SHIFT    0

/* Catch state (derived from tbx/metac_2_1.inc) */

/* Contents of TXCATCH0 register */
#define META_TXCATCH0_LDRXX_MASK     0xf8000000  /* Load destination reg 0-31 */
#define META_TXCATCH0_LDRXX_SHIFT    27
#define META_TXCATCH0_LDDST_MASK     0x07ff0000  /* Load destination bits */
#define META_TXCATCH0_LDDST_SHIFT    16
#define META_TXCATCH0_LDDST_TR_SHIFT 23
#define META_TXCATCH0_LDDST_TR_MASK  (1 << META_TXCATCH0_LDDST_TR_SHIFT)
#define META_TXCATCH0_LDDST_FX_SHIFT 22
#define META_TXCATCH0_LDDST_FX_MASK  (1 << META_TXCATCH0_LDDST_FX_SHIFT)
#define META_TXCATCH0_LDDST_PC_SHIFT 21
#define META_TXCATCH0_LDDST_PC_MASK  (1 << META_TXCATCH0_LDDST_PC_SHIFT)
#define META_TXCATCH0_LDDST_A1_SHIFT 20
#define META_TXCATCH0_LDDST_A1_MASK  (1 << META_TXCATCH0_LDDST_A1_SHIFT)
#define META_TXCATCH0_LDDST_A0_SHIFT 19
#define META_TXCATCH0_LDDST_A0_MASK  (1 << META_TXCATCH0_LDDST_A0_SHIFT)
#define META_TXCATCH0_LDDST_A_MASK   (META_TXCATCH0_LDDST_A0_MASK | \
                                      META_TXCATCH0_LDDST_A1_MASK)
#define META_TXCATCH0_LDDST_D1_SHIFT 18
#define META_TXCATCH0_LDDST_D1_MASK  (1 << META_TXCATCH0_LDDST_D1_SHIFT)
#define META_TXCATCH0_LDDST_D0_SHIFT 17
#define META_TXCATCH0_LDDST_D0_MASK  (1 << META_TXCATCH0_LDDST_D0_SHIFT)
#define META_TXCATCH0_LDDST_D_MASK   (META_TXCATCH0_LDDST_D0_MASK | \
                                      META_TXCATCH0_LDDST_D1_MASK)
#define META_TXCATCH0_LDDST_CT_SHIFT 16
#define META_TXCATCH0_LDDST_CT_MASK  (1 << META_TXCATCH0_LDDST_CT_SHIFT)
#define META_TXCATCH0_WATCHSTOP_MASK 0x00004000  /* Set if Data Watch set fault */
#define META_TXCATCH0_WATCHS_MASK    0x00004000  /* Set if Data Watch set fault */
#define META_TXCATCH0_WATCH1_MASK    0x00002000  /* Set if Data Watch 1 matches */
#define META_TXCATCH0_WATCH0_MASK    0x00001000  /* Set if Data Watch 0 matches */
#define META_TXCATCH0_FAULT_MASK     0x00000C00  /* See META_FREASON_* */
#define META_TXCATCH0_FAULT_SHIFT    10
#define META_TXCATCH0_PRIV_MASK      0x00000200  /* Privilege of transaction */
#define META_TXCATCH0_READ_MASK      0x00000100  /* Set for Read or Load cases */
#define META_TXCATCH0_LNKGET_MASK    0x00000008  /* LNKGET marker bit */
#define META_TXCATCH0_PREPROC_MASK   0x00000004  /* reverse destination registers */
/* Loads are indicated by one of the LDDST bits being set */
#define META_TXCATCH0_LDM16_MASK     0x00000004  /* Load M16 flag */
#define META_TXCATCH0_LDL2L1_MASK    0x00000003  /* Load data size L2,L1 */
#define META_TXCATCH0_LDL2L1_SHIFT   0
/* Reads are indicated by the READ bit being set without LDDST bits */
#define META_TXCATCH0_RAXX_MASK      0x0000001f  /* RAXX issue port for read */
#define META_TXCATCH0_RAXX_SHIFT     0
/* Write operations are all that remain if READ bit is not set */
#define META_TXCATCH0_WMASK_MASK     0x000000ff  /* Write byte lane mask (0xff for lnkset) */
#define META_TXCATCH0_WMASK_SHIFT    0

/* FPU exception fields */
#define META_TXCATCH0_FPURDREG_MASK    0x0000001f
#define META_TXCATCH0_FPURDREG_SHIFT   27
#define META_TXCATCH0_FPUSELROP1_MASK  0x0000001f
#define META_TXCATCH0_FPUSELROP1_SHIFT 22
#define META_TXCATCH0_FPUSPECIAL_MASK  0x0000000f
#define META_TXCATCH0_FPUSPECIAL_SHIFT 16
#define META_TXCATCH0_FPUFLAGS_MASK    0x0000001f
#define META_TXCATCH0_FPUFLAGS_SHIFT   8
#define META_TXCATCH0_FPUWIDTH_MASK    0x00000003
#define META_TXCATCH0_FPUWIDTH_SHIFT   6
#define META_TXCATCH0_FPUOPENC_MASK    0x0000003f
#define META_TXCATCH0_FPUOPENC_SHIFT   0

#define META_TXCATCH1_FPUSELROP2_MASK  0x0000001f
#define META_TXCATCH1_FPUSELROP2_SHIFT 27
#define META_TXCATCH1_FPUSELROP3_MASK  0x0000001f
#define META_TXCATCH1_FPUSELROP3_SHIFT 22
#define META_TXCATCH1_FPUIMM_MASK      0x0000ffff
#define META_TXCATCH1_FPUIMM_SHIFT     0

typedef enum {
    META_TXCATCH0_LDDST_CT,
    META_TXCATCH0_LDDST_D0,
    META_TXCATCH0_LDDST_D1,
    META_TXCATCH0_LDDST_A0,
    META_TXCATCH0_LDDST_A1,
    META_TXCATCH0_LDDST_PC,
    META_TXCATCH0_LDDST_FPU,
    META_TXCATCH0_LDDST_TR,
    META_TXCATCH0_LDDST_TMPLT,
    META_TXCATCH0_LDDST_D0DSP,
    META_TXCATCH0_LDDST_D1DSP,
} MetaTxCatch0Lddst;

typedef enum {
    META_DEFR_BUSERR        = 7,    /* bus response received */
    META_DEFR_FPU_DENORMAL  = 5,
    META_DEFR_FPU_INVALID   = 4,
    META_DEFR_FPU_DIVZERO   = 3,
    META_DEFR_FPU_OVERFLOW  = 2,
    META_DEFR_FPU_UNDERFLOW = 1,
    META_DEFR_FPU_INEXACT   = 0,
} MetaDefrTrigger;

#define META_DEFR_BUSERR_MASK        (1 << META_DEFR_BUSERR)
#define META_DEFR_FPU_DENORMAL_MASK  (1 << META_DEFR_FPU_DENORMAL)
#define META_DEFR_FPU_INVALID_MASK   (1 << META_DEFR_FPU_INVALID)
#define META_DEFR_FPU_DIVZERO_MASK   (1 << META_DEFR_FPU_DIVZERO)
#define META_DEFR_FPU_OVERFLOW_MASK  (1 << META_DEFR_FPU_OVERFLOW)
#define META_DEFR_FPU_UNDERFLOW_MASK (1 << META_DEFR_FPU_UNDERFLOW)
#define META_DEFR_FPU_INEXACT_MASK   (1 << META_DEFR_FPU_INEXACT)
#define META_DEFR_FPU_MASK           0x3f
#define META_DEFR_MASK               0xff

#define META_TXDEFR_BUSERR_MASK     0x80000000  /* 1: error, 0: warning */
#define META_TXDEFR_BUSSRC_MASK     0x40000000  /* 0: data, 1: instruction */
#define META_TXDEFR_BUSSTATE_MASK   0x3f000000
#define META_TXDEFR_BUSSTATE_SHIFT  24
#define META_TXDEFR_BUSSTATE_ERR             0x1 /* 1: error, 0: warning */
#define META_TXDEFR_BUSSTATE_LINKSET_SUCCESS 0x2 /* linked set succeeded */
#define META_TXDEFR_BUSSTATE_LINKSET_FAIL    0x4 /* linked set failed */
#define META_TXDEFR_BUSERR_ALL_MASK 0xff000000
#define META_TXDEFR_TRIGSTAT_MASK   0x00ff0000  /* see META_DEFR_* */
#define META_TXDEFR_TRIGSTAT_SHIFT  16
#define META_TXDEFR_TRIGICTRL_MASK  0x000000ff  /* 1's: int, 0's: bg */
#define META_TXDEFR_ROMASK          0x8040ff40

#define META_TXDEFR_LNKSET_SUCCESS  (META_TXDEFR_BUSSTATE_LINKSET_SUCCESS \
                                            << META_TXDEFR_BUSSTATE_SHIFT)
#define META_TXDEFR_LNKSET_FAIL     (META_TXDEFR_BUSSTATE_LINKSET_FAIL \
                                            << META_TXDEFR_BUSSTATE_SHIFT)

#define META_TXDIVTIME_RPDIRTY_MASK 0x80000000
#define META_TXDIVTIME_IRQENC_MASK  0x0f000000
#define META_TXDIVTIME_IRQENV_SHIFT 24
#define META_TXDIVTIME_RPMASK_MASK  0x003f0000
#define META_TXDIVTIME_RPMASK_SHIFT 16
#define META_TXDIVTIME_RFCTRL_MASK  0x000000ff
#define META_TXDIVTIME_ROMASK       0x7fffff00

typedef enum {
    META_TXPRIVEXT_PTOGGLE      = 0,
    META_TXPRIVEXT_MEMCHECK     = 1,
    META_TXPRIVEXT_STEP         = 2,
    META_TXPRIVEXT_KEEPPRIO     = 3,
    META_TXPRIVEXT_UNALIGNFAULT = 4,
    META_TXPRIVEXT_MINIMENABLE  = 7,
    META_TXPRIVEXT_TENW         = 8,
    META_TXPRIVEXT_TSTATUS      = 9,
    META_TXPRIVEXT_ITIMER       = 10,
    META_TXPRIVEXT_AMA          = 11,
    META_TXPRIVEXT_TCB          = 12,
    META_TXPRIVEXT_TICICYC      = 13,
    META_TXPRIVEXT_ILOCK        = 14,
    META_TXPRIVEXT_GCR          = 16,
    META_TXPRIVEXT_TRIG         = 17,
    META_TXPRIVEXT_CP0          = 24,
    META_TXPRIVEXT_CP1          = 25,
    META_TXPRIVEXT_CP2          = 26,
    META_TXPRIVEXT_CP3          = 27,
    META_TXPRIVEXT_CP4          = 28,
    META_TXPRIVEXT_CP5          = 29,
    META_TXPRIVEXT_CP6          = 30,
    META_TXPRIVEXT_CP7          = 31,
} MetaTxPrivext;

#define META_TXPRIVEXT_PTOGGLE_MASK      (1 << META_TXPRIVEXT_PTOGGLE)
#define META_TXPRIVEXT_MEMCHECK_MASK     (1 << META_TXPRIVEXT_MEMCHECK)
#define META_TXPRIVEXT_STEP_MASK         (1 << META_TXPRIVEXT_STEP)
#define META_TXPRIVEXT_KEEPPRIO_MASK     (1 << META_TXPRIVEXT_KEEPPRIO)
#define META_TXPRIVEXT_UNALIGNFAULT_MASK (1 << META_TXPRIVEXT_UNALIGNFAULT)
#define META_TXPRIVEXT_MINIMENABLE_MASK  (1 << META_TXPRIVEXT_MINIMENABLE)
#define META_TXPRIVEXT_TENW_MASK         (1 << META_TXPRIVEXT_TENW)
#define META_TXPRIVEXT_TSTATUS_MASK      (1 << META_TXPRIVEXT_TSTATUS)
#define META_TXPRIVEXT_ITIMER_MASK       (1 << META_TXPRIVEXT_ITIMER)
#define META_TXPRIVEXT_AMA_MASK          (1 << META_TXPRIVEXT_AMA)
#define META_TXPRIVEXT_TCB_MASK          (1 << META_TXPRIVEXT_TCB)
#define META_TXPRIVEXT_TICICYC_MASK      (1 << META_TXPRIVEXT_TICICYC)
#define META_TXPRIVEXT_ILOCK_MASK        (1 << META_TXPRIVEXT_ILOCK)
#define META_TXPRIVEXT_GCR_MASK          (1 << META_TXPRIVEXT_GCR)
#define META_TXPRIVEXT_TRIG_MASK         (1 << META_TXPRIVEXT_TRIG)
#define META_TXPRIVEXT_CP0_MASK          (1 << META_TXPRIVEXT_CP0)
#define META_TXPRIVEXT_CP1_MASK          (1 << META_TXPRIVEXT_CP1)
#define META_TXPRIVEXT_CP2_MASK          (1 << META_TXPRIVEXT_CP2)
#define META_TXPRIVEXT_CP3_MASK          (1 << META_TXPRIVEXT_CP3)
#define META_TXPRIVEXT_CP4_MASK          (1 << META_TXPRIVEXT_CP4)
#define META_TXPRIVEXT_CP5_MASK          (1 << META_TXPRIVEXT_CP5)
#define META_TXPRIVEXT_CP6_MASK          (1 << META_TXPRIVEXT_CP6)
#define META_TXPRIVEXT_CP7_MASK          (1 << META_TXPRIVEXT_CP7)
#define META_TXPRIVEXT_META1ROMASK        0x00fc80f0
#define META_TXPRIVEXT_META2ROMASK        0x00fc8040    /* self register view */
#define META_TXPRIVEXT_META2RUNNINGROMASK 0x00fc8090    /* MMIO running */
#define META_TXPRIVEXT_META2STOPPEDROMASK 0x00fc8040    /* MMIO stopped */
#define META_TXPRIVEXT_ROMASK(ENV)        ((META_COREMAJOR(ENV) < 2) \
                                          ? META_TXPRIVEXT_META1ROMASK \
                                          : META_TXPRIVEXT_META2ROMASK)
#define META_TXPRIVEXT_RUNNINGROMASK(ENV) ((META_COREMAJOR(ENV) < 2) \
                                          ? META_TXPRIVEXT_META1ROMASK \
                                          : META_TXPRIVEXT_META2RUNNINGROMASK)
#define META_TXPRIVEXT_STOPPEDROMASK(ENV) ((META_COREMAJOR(ENV) < 2) \
                                          ? META_TXPRIVEXT_META1ROMASK \
                                          : META_TXPRIVEXT_META2STOPPEDROMASK)

typedef enum {
    META_TRIGGER_TIMER  = 0,
    META_TRIGGER_KICK   = 1,
    META_TRIGGER_HALT   = 2,
    META_TRIGGER_DEFR   = 3,
    META_TRIGGER_HW0    = 4,
    META_TRIGGER_HWMAX  = 15,
} MetaTrigger;

#define META_TRIGGER_TIMER_MASK     (1 << META_TRIGGER_TIMER)
#define META_TRIGGER_KICK_MASK      (1 << META_TRIGGER_KICK)
#define META_TRIGGER_HALT_MASK      (1 << META_TRIGGER_HALT)
#define META_TRIGGER_DEFR_MASK      (1 << META_TRIGGER_DEFR)
#define META_TRIGGER_GE_DEFR_MASK   0xfffb  /* priority >= DEFR */
#define META_TXMASK_ROMASK          0xffff0000

#define META_TTCTRL_TT_MASK         (1 << 15)
#define META_TTCTRL_ALL_MASK        (1 << 13)
#define META_TTCTRL_ALLTAG_MASK     (1 << 10)
#define META_TTCTRL_TAG_MASK        (1 << 9)
#define META_TTCTRL_TTPC_MASK       (1 << 7)
#define META_TTCTRL_MARKPC_MASK     (1 << 5)
#define META_TTCTRL_ENABLE_MASK     (1 << 3)
#define META_TTCTRL_ENABLEI_MASK    (1 << 2)
#define META_TTCTRL_PC_MASK         (1 << 0)

/* HTP MMU table phys */

#define META_MMCUTBLPHYS0_BASE_MASK     0xffc00000
#define META_MMCUTBLPHYS0_RANGE_MASK    0x00000f00
#define META_MMCUTBLPHYS0_RANGE_SHIFT   8
#define META_MMCUTBLPHYS0_WIN_MASK      0x000000c0
#define META_MMCUTBLPHYS0_WIN_SHIFT     6
#define META_MMCUTBLPHYS0_SINGLE_MASK   (1 << 5)
#define META_MMCUTBLPHYS0_SYSCOH_MASK   (1 << 4)
#define META_MMCUTBLPHYS0_WRCOMB_MASK   (1 << 3)
#define META_MMCUTBLPHYS0_PRIV_MASK     (1 << 2)
#define META_MMCUTBLPHYS0_WRITE_MASK    (1 << 1)
#define META_MMCUTBLPHYS0_VALID_MASK    (1 << 0)
#define META_MMCUTBLPHYS0_MASK          0xffc00fff

#define META_MMCUTBLPHYS1_MASK          0xfffffffc
#define META_MMCUTBLPHYS1_MMPTMASK      0xfff80000

#if !defined(CONFIG_USER_ONLY)

/* MMU modes definitions
 * We need an MMU mode for each thread in priv/unpriv, plus a bypassed MMU mode.
 * Encode thread number in lowest bits, followed by pstat and bypass bits.
 * When bypass bit set, pstat and tx are 0, so for 4 threads that's 9 mmu modes.
 */
#define MMU_MODE0_SUFFIX _mmut0
#define MMU_MODE1_SUFFIX _mmut1
#define MMU_MODE2_SUFFIX _mmut2
#define MMU_MODE3_SUFFIX _mmut3
#define MMU_MODE4_SUFFIX _mmut0p
#define MMU_MODE5_SUFFIX _mmut1p
#define MMU_MODE6_SUFFIX _mmut2p
#define MMU_MODE7_SUFFIX _mmut3p
#define MMU_MODE8_SUFFIX _nommu
#define MMU_IDX_TX        (META_MAX_THREADS - 1)
#define MMU_IDX_PSTAT     (META_MAX_THREADS << 0)
#define MMU_IDX_BYPASS    (META_MAX_THREADS << 1)
#define MMU_MMUTX_IDX(X)  (X)
#define MMU_MMUTXP_IDX(X) (MMU_IDX_PSTAT | (X))
#define MMU_NOMMU_IDX     MMU_IDX_BYPASS
/* See NB_MMU_MODES further up the file.  */

static inline int meta_mmu_index(CPUArchState *env)
{
    if (!env->global->mmu) {
        /* MMU bypassed */
        return MMU_NOMMU_IDX;
    }
    if (env->cregs[META_TXSTATUS] & META_TXSTATUS_PSTAT_MASK) {
        /* Priv'd thread */
        return MMU_MMUTXP_IDX(env->thread_num);
    } else {
        /* Unpriv'd thread */
        return MMU_MMUTX_IDX(env->thread_num);
    }
}

#endif

void cpu_state_reset(CPUMETAState *env);
/* cpu_interrupt()/cpu_reset_interrupt() taking the BQL when not held */
void meta_interrupt(CPUMETAState *env, int mask);
void meta_reset_interrupt(CPUMETAState *env, int mask);
void meta_cpu_do_interrupt(CPUState *cs);
bool meta_cpu_exec_interrupt(CPUState *cs, int interrupt_request);
/* unblock a thread */
void do_unblock(CPUArchState *env);
/* for background only triggers (e.g. TXTIMER) */
int do_bgtrigger(CPUArchState *env, uint32_t trigger_mask);
/* for interrupt only triggers (e.g. TXTIMERI) */
int do_itrigger(CPUArchState *env, uint32_t trigger_mask);
/* for general (background+interrupt) triggers */
int do_trigger(CPUArchState *env, uint32_t trigger_mask);
/* any of the three, run on the vCPU (for timers and devices) */
#define META_TRIGGER_BG   0
#define META_TRIGGER_INT  1
#define META_TRIGGER_BOTH 2
void meta_trigger_async(CPUArchState *env, uint32_t mask, int kind);
uint32_t meta_core_intreg_read(CPUArchState *env, MetaUnit unit, int id);
void meta_core_intreg_write(CPUArchState *end, MetaUnit unit, int id, uint32_t val);
void meta_update_txcatch_signum(MetaSigNum signum, uint32_t *catch0);

uint32_t meta_get_pte1(MetaCore *core, int t, uint32_t segment);
void meta_set_pte1(MetaCore *core, int t, uint32_t segment, uint32_t pte);
uint32_t meta_get_pte2(MetaCore *core, int t, uint32_t page);
bool meta_cpu_tlb_fill(CPUState *cs, vaddr address, int size,
                       MMUAccessType access_type, int mmu_idx,
                       bool probe, uintptr_t retaddr);
hwaddr meta_cpu_get_phys_page_debug(CPUState *cs, vaddr addr);
G_NORETURN void meta_cpu_do_unaligned_access(CPUState *cs, vaddr addr,
                                             MMUAccessType access_type,
                                             int mmu_idx, uintptr_t retaddr);
void meta_restore_state_to_opc(CPUState *cs, const TranslationBlock *tb,
                               const uint64_t *data);
void meta_restore_fault_state(CPUArchState *env);

static inline unsigned int meta_supported_isas(CPUArchState *env)
{
    uint8_t maj = META_COREMAJOR(env);

    if (maj < 2) {
        return 1 << META1;
    }

    return (1 << META2) | (1 << MINIM);
}

#define META_ISAFLAGS_IGNORE_MINIMENABLE     (1 << 0)
static inline MetaISA meta_current_isa_pc_flags(CPUArchState *env, target_ulong pc,
                                                unsigned int flags)
{
    uint8_t maj = META_COREMAJOR(env);

    if (maj < 2) {
        return META1;
    }

    /* Determine whether the current PC is in MiniM code */

    /* TXPRIVEXT.MiniMEnable must be set */
    if (!(flags & META_ISAFLAGS_IGNORE_MINIMENABLE) &&
            !(env->cregs[META_TXPRIVEXT] & META_TXPRIVEXT_MINIMENABLE_MASK)) {
        return META2;
    }
    /* PC must be in lower half of 32MB region */
    if (pc & 0x00800000) {
        return META2;
    }

    return MINIM;
}

static inline MetaISA meta_current_isa_pc(CPUArchState *env, target_ulong pc)
{
    return meta_current_isa_pc_flags(env, pc, 0);
}

static inline MetaISA meta_current_isa(CPUArchState *env)
{
    return meta_current_isa_pc_flags(env, env->pc[META_PC], 0);
}

static inline target_ulong meta_pc_to_virt(CPUArchState *env, target_ulong pc)
{
    if (meta_current_isa_pc(env, pc) == MINIM) {
        return (pc & 0xff000000) | ((pc & 0x007ffffc) >> 1);
    }

    return pc;
}

static inline target_ulong meta_virt_to_pc(CPUArchState *env, target_ulong pc)
{
    if (meta_current_isa_pc(env, pc) == MINIM) {
        return (pc & 0xff000000) | ((pc & 0x003fffff) << 1);
    }

    return pc;
}

static inline size_t meta_minim_decode_size(uint16_t insn_core)
{
    if ((insn_core & 0xf000) == 0xb000) {
        return 2; /* long instruction */
    }

    if ((insn_core & 0xc000) == 0xc000) {
        return 2; /* extended instruction */
    }

    /* core instruction */
    return 1;
}

/* internal interrupt to wake thread from TXSTAT[I] block */
#define CPU_INTERRUPT_WAKE CPU_INTERRUPT_TGT_INT_0

#if defined(CONFIG_USER_ONLY)
/* Exceptions */
enum {
    EXCP_NONE           = -1,
    EXCP_FAULT_BASE,
    EXCP_GATEWAY_TRAP,
    EXCP_FAULT_END      = EXCP_FAULT_BASE + 15,
    EXCP_FPU,

    EXCP_LAST = EXCP_FPU,
};
#endif /* CONFIG_USER_ONLY */

#if !defined(CONFIG_USER_ONLY)
#define META_TBFLAG_TX      0x00000003  /* Thread number */
#define META_TBFLAG_MMU     0x00000004  /* MMU on */
#define META_TBFLAG_PSTAT   0x00000008  /* Privilege state */
#define META_TBFLAG_ISTAT   0x00000010  /* Interrupt state */
#else
#define META_TBFLAG_TX      0
#define META_TBFLAG_MMU     0
#define META_TBFLAG_PSTAT   0
#define META_TBFLAG_ISTAT   0
#endif
#define META_TBFLAG_LSM     0x000000e0  /* Load store multiple */
#define META_TBFLAG_LSM_SHIFT 5
#if !defined(CONFIG_USER_ONLY)
#define META_TBFLAG_STEP    0x00000100  /* Single step */
#define META_TBFLAG_REPLAY  0x00000200  /* Replay catch state */
#define META_TBFLAG_MINIM   0x00000400  /* MiniM enabled and MiniM code */
#define META_TBFLAG_TREN    0x00000800  /* Trace enabled */
#define META_TBFLAG_TRALL   0x00001000  /* Trace all instruction issues */
#define META_TBFLAG_TRPC    0x00002000  /* Trace PC for all events */
#define META_TBFLAG_TRTTPC  0x00004000  /* Trace TT PC */
#define META_TBFLAG_TRTAG   0x00008000  /* Trace tag data */
#define META_TBFLAG_TRALLTAG 0x00010000 /* Trace tag data when all & tt event */
#define META_TBFLAG_DMTR    0x00020000  /* Data memory tracing */
#define META_TBFLAG_IMTR    0x00040000  /* Instruction memory tracing */
#define META_TBFLAG_OTHERTR 0x00080000  /* Other less frequent trace events */
#else
#define META_TBFLAG_STEP    0
#define META_TBFLAG_REPLAY  0
#define META_TBFLAG_MINIM   0
#define META_TBFLAG_TREN    0
#define META_TBFLAG_TRALL   0
#define META_TBFLAG_TRPC    0
#define META_TBFLAG_TRTTPC  0
#define META_TBFLAG_TRTAG   0
#define META_TBFLAG_TRALLTAG 0
#define META_TBFLAG_DMTR    0
#define META_TBFLAG_IMTR    0
#define META_TBFLAG_OTHERTR 0
#endif
#define META_TBFLAG_DUMODE        0x07f00000 /* Bottom 7 bits of TXMODE */
#define  META_TBFLAG_DUARITHMODE  0x00300000
#define  META_TBFLAG_DUPRODSHIFT  0x00400000
#define  META_TBFLAG_DUROUNDING   0x00800000
#define  META_TBFLAG_DUSATURATION 0x01000000
#define  META_TBFLAG_DUACCSAT     0x02000000
#define  META_TBFLAG_M8           0x04000000
#define META_TBFLAG_DUMODE_SHIFT  20
#define META_TBFLAG_DSPRAMODULO   0x08000000 /* non-zero RAMAModSize */
#define META_TBFLAG_DSPRAMODULO_SHIFT 27
#define META_TBFLAG_DSPRBMODULO   0x10000000 /* non-zero RAMBModSize */
#define META_TBFLAG_DSPRBMODULO_SHIFT 28
#define META_TBFLAG_DSPRRADIX     0x20000000 /* TXMODE:29 */
#define META_TBFLAG_DSPRRADIX_SHIFT   29
#define META_TBFLAG_TXL1COUNTNZ   0x000100000000llu
#define META_TBFLAG_TXL1COUNTNZ_SHIFT 32
#define META_TBFLAG_TXL2COUNTNZ   0x000200000000llu
#define META_TBFLAG_TXL2COUNTNZ_SHIFT 33
#define META_TBFLAG_AU0NLINEAR    0x000400000000llu
#define META_TBFLAG_AU0NLINEAR_SHIFT  34
#define META_TBFLAG_AU1NLINEAR    0x000800000000llu
#define META_TBFLAG_AU1NLINEAR_SHIFT  35
#define META_TBFLAG_RPDIRTY       0x001000000000llu
#define META_TBFLAG_RPDIRTY_SHIFT     36
#define META_TBFLAG_SCC           0x002000000000llu
#define META_TBFLAG_SCC_SHIFT         37
#define META_TBFLAG_SIMACTIVE     0x004000000000llu
#define META_TBFLAG_SIMACTIVE_SHIFT   38
#define META_TBFLAG_ALIGNFAULT    0x008000000000llu

static inline uint64_t meta_tb_flags(CPUArchState *env, target_ulong *pc)
{
    uint64_t f, *flags = &f;
    uint32_t txstatus = env->cregs[META_TXSTATUS];

    *pc = meta_pc_to_virt(env, env->pc[META_PC]);
    *flags = env->thread_num |
             (env->lsmstep << META_TBFLAG_LSM_SHIFT);
#if !defined(CONFIG_USER_ONLY)
    if (env->global->mmu) {
        *flags |= META_TBFLAG_MMU;
    }
    if (txstatus & META_TXSTATUS_PSTAT_MASK) {
        *flags |= META_TBFLAG_PSTAT;
    }
    if (txstatus & META_TXSTATUS_ISTAT_MASK) {
        *flags |= META_TBFLAG_ISTAT;
    }
#endif
    if (env->cregs[META_TXPRIVEXT] & META_TXPRIVEXT_STEP_MASK) {
        *flags |= META_TBFLAG_STEP;
    }
    if (env->catch_replay) {
        if ((txstatus & (META_TXSTATUS_ISTAT_MASK | META_TXSTATUS_CBIMARKER_MASK))
                    == (META_TXSTATUS_ISTAT_MASK | META_TXSTATUS_CBIMARKER_MASK)) {
            *flags |= META_TBFLAG_REPLAY;
        }
        if ((txstatus & (META_TXSTATUS_ISTAT_MASK | META_TXSTATUS_CBMARKER_MASK))
                    == META_TXSTATUS_CBMARKER_MASK) {
            *flags |= META_TBFLAG_REPLAY;
        }
    }
    if (meta_current_isa(env) == MINIM) {
        *flags |= META_TBFLAG_MINIM;
    }
    *flags |= !!((env->cregs[META_TXDRSIZE] & META_TXDRSIZE_RAMAMODSIZE_MASK)
                  >> META_TXDRSIZE_RAMAMODSIZE_SHIFT) << META_TBFLAG_DSPRAMODULO_SHIFT;
    *flags |= !!((env->cregs[META_TXDRSIZE] & META_TXDRSIZE_RAMBMODSIZE_MASK)
                  >> META_TXDRSIZE_RAMBMODSIZE_SHIFT) << META_TBFLAG_DSPRBMODULO_SHIFT;
#if !defined(CONFIG_USER_ONLY)
    if (!(txstatus & META_TXSTATUS_ISTAT_MASK)) {
#endif
        *flags |= (env->cregs[META_TXMODE] &
                   (META_TBFLAG_DUMODE >> META_TBFLAG_DUMODE_SHIFT))
                   << META_TBFLAG_DUMODE_SHIFT;
        *flags |= (uint64_t)!!(env->cregs[META_TXMODE] & META_TXMODE_DSPRRADIX_MASK)
                      << META_TBFLAG_DSPRRADIX_SHIFT;

        *flags |= (uint64_t)!!meta_auaddrmode(env, 0)
                      << META_TBFLAG_AU0NLINEAR_SHIFT;
        *flags |= (uint64_t)!!meta_auaddrmode(env, 1)
                      << META_TBFLAG_AU1NLINEAR_SHIFT;

        if (env->cregs[META_TXL1COUNT]) {
            *flags |= META_TBFLAG_TXL1COUNTNZ;
        }
        if (env->cregs[META_TXL2COUNT]) {
            *flags |= META_TBFLAG_TXL2COUNTNZ;
        }
#if !defined(CONFIG_USER_ONLY)
    }
#endif
    if (env->cregs[META_TXDIVTIME] & META_TXDIVTIME_RPDIRTY_MASK) {
        *flags |= META_TBFLAG_RPDIRTY;
    }
    if (env->cf[META_CF_SCC]) {
        *flags |= META_TBFLAG_SCC;
    }
    if (env->sim_active) {
        *flags |= META_TBFLAG_SIMACTIVE;
    }
    if ((meta_supported_isas(env) & (1 << META2)) &&
        (env->cregs[META_TXPRIVEXT] & META_TXPRIVEXT_UNALIGNFAULT_MASK)) {
        *flags |= META_TBFLAG_ALIGNFAULT;
    }
    return f;
}

#if !defined(CONFIG_USER_ONLY)
static inline int tb_mmu_index(int flags)
{
    if (!(flags & META_TBFLAG_MMU)) {
        /* MMU bypassed */
        return MMU_NOMMU_IDX;
    }
    if (flags & META_TBFLAG_PSTAT) {
        /* Priv'd thread */
        return MMU_MMUTXP_IDX(flags & META_TBFLAG_TX);
    } else {
        /* Unpriv'd thread */
        return MMU_MMUTX_IDX(flags & META_TBFLAG_TX);
    }
}
#endif

void meta_tcg_init(void);
void meta_translate_code(CPUState *cs, TranslationBlock *tb,
                         int *max_insns, vaddr pc, void *host_pc);
void meta_cpu_dump_state(CPUState *cs, FILE *f, int flags);

#define CPU_RESOLVING_TYPE TYPE_META_CPU

#endif
