/*
 * CSKY CPU
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "exec/gdbstub.h"
#include "qapi/error.h"
#include "cpu.h"
#include "migration/vmstate.h"
#include "exec/cputlb.h"
#include "exec/translation-block.h"
#include "accel/tcg/cpu-ops.h"
#include "tcg/debug-assert.h"
#include "gdbstub/helpers.h"
#include "qemu/option.h"
#include "qemu/config-file.h"
#include "qemu/error-report.h"

static void csky_cpu_set_pc(CPUState *cs, vaddr value)
{
    CSKYCPU *cpu = CSKY_CPU(cs);

    cpu->env.pc = value;
}

static int csky_cpu_memory_rw_debug(CPUState *cs, vaddr addr,
                        uint8_t *buf, size_t len, bool is_write)
{
    int ret;
    ret  = cpu_memory_rw_debug(cs, addr, buf, len, is_write);
#ifndef CONFIG_USER_ONLY
    if (ret == -1) {
        CSKYCPU *cpu = CSKY_CPU(cs);
        if (cpu->env.hbreak) {
            if (is_write == false) {
                memset(buf, 0, len);
                ret = 0;
            }
        }
    }
#endif
    return ret;
}
static void csky_cpu_exec_enter(CPUState *cs)
{
    CSKYCPU *cpu = CSKY_CPU(cs);

    if (csky_has_feature(&(cpu->env), CPU_C860) && (cpu->env.in_reset)) {
        cs->exception_index = EXCP_CSKY_IN_RESET;
    }
}

static bool csky_cpu_has_work(CPUState *cs)
{
    return cpu_test_interrupt(cs, CPU_INTERRUPT_HARD);
}

static vaddr csky_cpu_get_pc(CPUState *cs)
{
    return cpu_env(cs)->pc;
}

static TCGTBCPUState csky_get_tb_cpu_state(CPUState *cs)
{
    CPUCSKYState *env = cpu_env(cs);
    uint32_t mask, flags;

    if (env->features & CPU_C860) {
        mask = CSKY_MP_ASID_MASK;
    } else {
        mask = CSKY_ASID_MASK;
    }
#if defined(TARGET_CSKYV2)
    flags = (env->psr_s << CSKY_TBFLAG_PSR_S_SHIFT)
        | (env->psr_bm << CSKY_TBFLAG_PSR_BM_SHIFT)
        | (env->sce_condexec_bits << CSKY_TBFLAG_SCE_CONDEXEC_SHIFT)
        | ((env->mmu.meh & mask) << CSKY_TBFLAG_ASID_SHIFT)
        | (env->psr_tm << CSKY_TBFLAG_PSR_TM_SHIFT)
        | (env->psr_t << CSKY_TBFLAG_PSR_T_SHIFT)
        | (env->idly4_counter << CSKY_TBFLAG_IDLY4_SHIFT);
#else
    flags = (PSR_CPID(env->cp0.psr) << CSKY_TBFLAG_CPID_SHIFT)
        | (env->psr_s << CSKY_TBFLAG_PSR_S_SHIFT)
        | ((env->mmu.meh & mask) << CSKY_TBFLAG_ASID_SHIFT)
        | (env->psr_tm << CSKY_TBFLAG_PSR_TM_SHIFT)
        | (env->idly4_counter << CSKY_TBFLAG_IDLY4_SHIFT);
#endif

    return (TCGTBCPUState){ .pc = env->pc, .flags = flags };
}

static void csky_cpu_synchronize_from_tb(CPUState *cs,
                                         const TranslationBlock *tb)
{
    tcg_debug_assert(!tcg_cflags_has(cs, CF_PCREL));
    cpu_env(cs)->pc = tb->pc;
}

static void csky_restore_state_to_opc(CPUState *cs,
                                      const TranslationBlock *tb,
                                      const uint64_t *data)
{
    cpu_env(cs)->pc = data[0];
}

static int csky_cpu_mmu_index(CPUState *cs, bool ifetch)
{
    return csky_mmu_index(cpu_env(cs));
}

static inline void csky_set_feature(CPUCSKYState *env, uint64_t feature)
{
    env->features |= feature;
}

static inline void csky_clear_feature(CPUCSKYState *env, uint64_t feature)
{
    env->features &= ~feature;
}

/*
 * The C-SKY fork registered "cpu-prop" and "csky-extend" option groups in
 * vl.c.  They are not carried over; without them every option keeps its
 * default.
 */
static QemuOpts *csky_find_opts(const char *group)
{
    QemuOptsList *list = qemu_find_opts_err(group, NULL);

    return list ? qemu_opts_find(list, NULL) : NULL;
}

static void csky_cpu_handle_opts(CPUCSKYState *env)
{
    QemuOpts *opts;
    bool b;
    char *str;
    uint32_t n;

    env->hbreak = true;

#ifndef CONFIG_USER_ONLY
    opts = csky_find_opts("cpu-prop");
    if (opts) {

        b = qemu_opt_get_bool(opts, "pctrace", false);
        if (b) {
            env->pctrace = 1;
        }

        /* ck807/ck810 elrw off by default,
         * ck801/ck802/ck803/ck860 default elrw on by default. */
        if (csky_has_feature(env, CPU_C807 | CPU_C810)) {
            b = qemu_opt_get_bool(opts, "elrw", false);
        } else {
            b = qemu_opt_get_bool(opts, "elrw", true);
        }
        if (b) {
            csky_set_feature(env, ABIV2_ELRW);
        } else {
            csky_clear_feature(env, ABIV2_ELRW);
        }

        str = qemu_opt_get_del(opts, "mem_prot");
        if (str != NULL) {
            if (!strcmp(str, "mmu")) {
                env->features |= CSKY_MMU;
                env->features &= ~CSKY_MGU;
            } else if (!strcmp(str, "mgu")) {
                env->features |= CSKY_MGU;
                env->features &= ~CSKY_MMU;
            } else if (!strcmp(str, "no")) {
                env->features &= ~CSKY_MGU;
                env->features &= ~CSKY_MMU;
            } else {
                error_report("mem_prot= only allow mmu/mgu/no");
                exit(1);
            }
        }

        b = qemu_opt_get_bool(opts, "full_mmu", false);
        if (b) {
            env->full_mmu = 1;
        }

        b = qemu_opt_get_bool(opts, "unaligned_access", false);
        if (b) {
            csky_set_feature(env, UNALIGNED_ACCESS);
        }
    }
#endif

    opts = csky_find_opts("csky-extend");
    if (opts) {
        n = qemu_opt_get_number(opts, "vdsp", 0);
        if (n != 0) {
            if (!csky_has_feature(env, CPU_C810)) {
                error_report("only 810 support vdsp");
                exit(1);
            }

            if (n == 64) {
                csky_set_feature(env, ABIV2_VDSP64);
            } else if (n == 128) {
                csky_set_feature(env, ABIV2_VDSP128);
            } else {
                error_report("vdsp= only allow 64 or 128");
                exit(1);
            }
        }

        n = qemu_opt_get_size(opts, "cpu_freq", 0);
        env->cpu_freq = n;

        str = qemu_opt_get_del(opts, "exit_addr");
        if (str != NULL) {
            env->exit_addr = strtoul(str, NULL, 0);
        } else {
            env->exit_addr = 0;
        }

        str = qemu_opt_get_del(opts, "jcount_start");
        if (str != NULL) {
            env->jcount_start = strtoul(str, NULL, 0);
        } else {
            env->jcount_start = 0;
        }

        str = qemu_opt_get_del(opts, "jcount_end");
        if (str != NULL) {
            env->jcount_end = strtoul(str, NULL, 0);
        } else {
            env->jcount_end = 0;
        }

        b = qemu_opt_get_bool(opts, "mmu_default", false);
        if (b) {
            env->mmu_default = 1;
        }

        b = qemu_opt_get_bool(opts, "tb_trace", false);
        if (b) {
            env->tb_trace = 1;
        }

        b = qemu_opt_get_bool(opts, "denormal", false);
        if (b) {
            csky_set_feature(env, DENORMALIZE);
            env->vfp.fp_status.flush_to_zero = 0;
            env->vfp.fp_status.flush_inputs_to_zero = 0;
        }

        b = qemu_opt_get_bool(opts, "hbreak", true);
        if (!b) {
            env->hbreak = false;
        }
    }
}

struct csky_trace_info tb_trace_info[TB_TRACE_NUM];

static void csky_cpu_reset_hold(Object *obj, ResetType type)
{
    CPUState *s = CPU(obj);
    CSKYCPU *cpu = CSKY_CPU(s);
    CSKYCPUClass *mcc = CSKY_CPU_GET_CLASS(cpu);
    CPUCSKYState *env = &cpu->env;
    uint32_t cpidr;

    if (mcc->parent_phases.hold) {
        mcc->parent_phases.hold(obj, type);
    }

    /* backup data before memset */
    cpidr = env->cp0.cpidr[0];

    memset(env, 0, offsetof(CPUCSKYState, features));

    env->cp0.cpidr[0] = cpidr;
    env->cp0.cpidr[1] = 0x10000000;
    env->cp0.cpidr[2] = 0x20000000;
    env->cp0.cpidr[3] = 0x30000000;
    env->cp0.cpidr[4] = 0x40000000;
    env->cp0.cpidr[5] = 0x50000000;
    env->cp0.cpidr[6] = 0x60000000;
    env->cp0.cpidr[7] = 0x70000000;

#if defined(TARGET_CSKYV1)
    env->cp1.fsr = 0x0;
#endif

#if defined(CONFIG_USER_ONLY)
    env->cp0.psr = 0x140;
#if defined(TARGET_CSKYV2)
    env->sce_condexec_bits = 1;
    env->sce_condexec_bits_bk = 1;
#endif

#else
    if (csky_has_feature(env, ABIV2_TEE)) {
        env->tee.nt_psr = 0x80000000;
        env->tee.t_psr = 0xc0000000;
        env->cp0.psr = env->tee.t_psr;
        env->psr_t = PSR_T(env->cp0.psr);
        env->mmu = env->t_mmu;
    } else {
        env->cp0.psr = 0x80000000;
        env->mmu = env->nt_mmu;
    }
    if (csky_has_feature(env, CPU_C860)) {
        env->in_reset = true;
    }
    env->psr_s = PSR_S(env->cp0.psr);
#if defined(TARGET_CSKYV2)
    env->psr_bm = PSR_BM(env->cp0.psr);
    env->sce_condexec_bits = 1;
    env->sce_condexec_bits_bk = 1;
    env->mmu.msa0 = 0x1e;
    env->mmu.msa1 = 0x16;
#endif

#if TARGET_BIG_ENDIAN
    env->cp0.ccr = 0x80;
#endif

    csky_nommu_init(env);
#endif

    /*
     * softfloat has no fallback NaN rules any more.  The C-SKY fork
     * grouped csky with ARM in pickNaN() and pickNaNMulAdd(), so use the
     * rules target/arm sets.
     */
    set_float_2nan_prop_rule(float_2nan_prop_s_ab, &env->vfp.fp_status);
    set_float_3nan_prop_rule(float_3nan_prop_s_cab, &env->vfp.fp_status);
    set_float_infzeronan_rule(float_infzeronan_dnan_if_qnan,
                              &env->vfp.fp_status);
    set_float_default_nan_pattern(0b01000000, &env->vfp.fp_status);

    if (csky_has_feature(env, DENORMALIZE)) {
        env->vfp.fp_status.flush_to_zero = 0;
        env->vfp.fp_status.flush_inputs_to_zero = 0;
    } else {
        env->vfp.fp_status.flush_to_zero = 1;
        env->vfp.fp_status.flush_inputs_to_zero = 1;
    }

    s->exception_index = -1;
    tlb_flush(s);

    env->trace_info = g_malloc0(sizeof(struct csky_trace_info) * TB_TRACE_NUM);
    env->trace_index = 0;
    csky_cpu_handle_opts(env);
    csky_trace_handle_opts(s, env->cpuid);
}

static void csky_cpu_disas_set_info(const CPUState *s, disassemble_info *info)
{
    info->endian = BFD_ENDIAN_LITTLE;
#if defined(TARGET_CSKYV1)
    info->print_insn = print_insn_csky_v1;
#else
    info->print_insn = print_insn_csky_v2;
#endif
}

/* CPU models */
static ObjectClass *csky_cpu_class_by_name(const char *cpu_model)
{
    ObjectClass *oc;
    char *typename;

    if (cpu_model == NULL) {
        return NULL;
    }

    typename = g_strdup_printf("%s-" TYPE_CSKY_CPU, cpu_model);
    oc = object_class_by_name(typename);
    g_free(typename);
    if (oc != NULL && (object_class_dynamic_cast(oc, TYPE_CSKY_CPU) == NULL ||
                       object_class_is_abstract(oc))) {
        return NULL;
    }
    return oc;
}

static void ck510_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV1);
    csky_set_feature(env, CPU_510);
    csky_set_feature(env, CSKY_MGU);
    env->cpuid = CSKY_CPUID_CK510;
    env->cp0.cpidr[0] = env->cpuid;
}

static void ck520_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV1);
    csky_set_feature(env, CPU_520);
    csky_set_feature(env, CSKY_MGU);
    env->cpuid = CSKY_CPUID_CK520;
    env->cp0.cpidr[0] = env->cpuid;
}

static void ck610_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV1);
    csky_set_feature(env, CPU_610);
    csky_set_feature(env, CSKY_MMU);
    env->cpuid = CSKY_CPUID_CK610;
    env->cp0.cpidr[0] = env->cpuid;
}

static void ck610e_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV1);
    csky_set_feature(env, CPU_610);
    csky_set_feature(env, ABIV1_DSP);
    csky_set_feature(env, CSKY_MMU);
    env->cpuid = CSKY_CPUID_CK610;
    env->cp0.cpidr[0] = env->cpuid;
}

static void ck610f_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV1);
    csky_set_feature(env, CPU_610);
    csky_set_feature(env, ABIV1_FPU);
    csky_set_feature(env, CSKY_MMU);
    env->cpuid = CSKY_CPUID_CK610;
    env->cp0.cpidr[0] = env->cpuid;
}

static void ck610ef_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV1);
    csky_set_feature(env, CPU_610);
    csky_set_feature(env, ABIV1_DSP);
    csky_set_feature(env, ABIV1_FPU);
    csky_set_feature(env, CSKY_MMU);
    env->cpuid = CSKY_CPUID_CK610;
    env->cp0.cpidr[0] = env->cpuid;
}

static void e801_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_E801);
    csky_set_feature(env, ABIV2_ELRW);
    csky_set_feature(env, CSKY_MGU);
    env->cpuid = CSKY_CPUID_E801;
    env->cp0.cpidr[0] = env->cpuid;
}

static void e801t_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_E801);
    csky_set_feature(env, ABIV2_TEE);
    csky_set_feature(env, ABIV2_ELRW);
    csky_set_feature(env, CSKY_MGU);
    env->cpuid = CSKY_CPUID_E801;
    env->cp0.cpidr[0] = env->cpuid;
}

static void e802_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_E802);
    csky_set_feature(env, ABIV2_ELRW);
    csky_set_feature(env, CSKY_MGU);
    env->cpuid = CSKY_CPUID_E802;
    env->cp0.cpidr[0] = env->cpuid;
}

static void e802j_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_E802);
    csky_set_feature(env, ABIV2_JAVA);
    csky_set_feature(env, ABIV2_ELRW);
    csky_set_feature(env, CSKY_MGU);
    env->cpuid = CSKY_CPUID_E802;
    env->cp0.cpidr[0] = env->cpuid;
}

static void e802t_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_E802);
    csky_set_feature(env, ABIV2_TEE);
    csky_set_feature(env, ABIV2_ELRW);
    csky_set_feature(env, CSKY_MGU);
    env->cpuid = CSKY_CPUID_E802;
    env->cp0.cpidr[0] = env->cpuid;
}

static void e803_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_E803);
    csky_set_feature(env, ABIV2_ELRW);
    csky_set_feature(env, CSKY_MGU);
    env->cpuid = CSKY_CPUID_E803;
    env->cp0.cpidr[0] = env->cpuid;
}

static void e803t_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_E803);
    csky_set_feature(env, ABIV2_ELRW);
    csky_set_feature(env, ABIV2_TEE);
    csky_set_feature(env, CSKY_MGU);
    env->cpuid = CSKY_CPUID_E803;
    env->cp0.cpidr[0] = env->cpuid;
}

static void e804_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_E804);
    csky_set_feature(env, ABIV2_ELRW);
    csky_set_feature(env, CSKY_MGU);
    env->cpuid = CSKY_CPUID_E804;
    env->cp0.cpidr[0] = env->cpuid;
}

static void e804f_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_E804);
    csky_set_feature(env, ABIV2_ELRW);
    csky_set_feature(env, ABIV2_FPU_SINGLE);
    csky_set_feature(env, CSKY_MGU);
    env->cpuid = CSKY_CPUID_E804;
    env->cp0.cpidr[0] = env->cpuid;
}

static void e804ft_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_E804);
    csky_set_feature(env, ABIV2_ELRW);
    csky_set_feature(env, ABIV2_FPU_SINGLE);
    csky_set_feature(env, ABIV2_TEE);
    csky_set_feature(env, CSKY_MGU);
    env->cpuid = CSKY_CPUID_E804;
    env->cp0.cpidr[0] = env->cpuid;
}

static void e804t_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_E804);
    csky_set_feature(env, ABIV2_ELRW);
    csky_set_feature(env, ABIV2_TEE);
    csky_set_feature(env, CSKY_MGU);
    env->cpuid = CSKY_CPUID_E804;
    env->cp0.cpidr[0] = env->cpuid;
}

static void e804d_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_E804);
    csky_set_feature(env, ABIV2_DSP2);
    csky_set_feature(env, ABIV2_ELRW);
    csky_set_feature(env, CSKY_MGU);
    env->cpuid = CSKY_CPUID_E804;
    env->cp0.cpidr[0] = env->cpuid;
}

static void e804df_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_E804);
    csky_set_feature(env, ABIV2_ELRW);
    csky_set_feature(env, ABIV2_DSP2);
    csky_set_feature(env, ABIV2_FPU_SINGLE);
    csky_set_feature(env, CSKY_MGU);
    env->cpuid = CSKY_CPUID_E804;
    env->cp0.cpidr[0] = env->cpuid;
}

static void e804dft_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_E804);
    csky_set_feature(env, ABIV2_DSP2);
    csky_set_feature(env, ABIV2_ELRW);
    csky_set_feature(env, ABIV2_FPU_SINGLE);
    csky_set_feature(env, ABIV2_TEE);
    csky_set_feature(env, CSKY_MGU);
    env->cpuid = CSKY_CPUID_E804;
    env->cp0.cpidr[0] = env->cpuid;
}

static void e804dt_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_E804);
    csky_set_feature(env, ABIV2_DSP2);
    csky_set_feature(env, ABIV2_ELRW);
    csky_set_feature(env, ABIV2_TEE);
    csky_set_feature(env, CSKY_MGU);
    env->cpuid = CSKY_CPUID_E804;
    env->cp0.cpidr[0] = env->cpuid;
}

static void i805_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_I805);
    csky_set_feature(env, ABIV2_ELRW);
    csky_set_feature(env, CSKY_MGU);
    env->cpuid = CSKY_CPUID_I805;
    env->cp0.cpidr[0] = env->cpuid;
}

static void i805f_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_I805);
    csky_set_feature(env, ABIV2_FPU_SINGLE);
    csky_set_feature(env, ABIV2_ELRW);
    csky_set_feature(env, CSKY_MGU);
    env->cpuid = CSKY_CPUID_I805;
    env->cp0.cpidr[0] = env->cpuid;
}

static void i805dft_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_I805);
    csky_set_feature(env, ABIV2_ELRW);
    csky_set_feature(env, ABIV2_DSP2);
    csky_set_feature(env, ABIV2_FPU_SINGLE);
    csky_set_feature(env, ABIV2_TEE);
    csky_set_feature(env, CSKY_MGU);
    env->cpuid = CSKY_CPUID_I805;
    env->cp0.cpidr[0] = env->cpuid;
}

static void c807_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_C807);
    csky_set_feature(env, CSKY_MMU);
    env->cpuid = CSKY_CPUID_C807;
    env->cp0.cpidr[0] = env->cpuid;
}

static void c807f_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_C807);
    csky_set_feature(env, ABIV2_FPU);
    csky_set_feature(env, CSKY_MMU);
    env->cpuid = CSKY_CPUID_C807;
    env->cp0.cpidr[0] = env->cpuid;
}

static void c807fv_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_C807);
    csky_set_feature(env, ABIV2_FPU);
    csky_set_feature(env, CSKY_MMU);
    env->cpuid = CSKY_CPUID_C807;
    env->cp0.cpidr[0] = env->cpuid;
}

static void c810_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_C810);
    csky_set_feature(env, CSKY_MMU);
    env->cpuid = CSKY_CPUID_C810;
    env->cp0.cpidr[0] = env->cpuid;
}

static void c810v_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_C810);
    csky_set_feature(env, CSKY_MMU);
    env->cpuid = CSKY_CPUID_C810;
    env->cp0.cpidr[0] = env->cpuid;
}

static void c810f_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_C810);
    csky_set_feature(env, ABIV2_FPU);
    csky_set_feature(env, CSKY_MMU);
    env->cpuid = CSKY_CPUID_C810;
    env->cp0.cpidr[0] = env->cpuid;
}

static void c810t_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_C810);
    csky_set_feature(env, ABIV2_TEE);
    csky_set_feature(env, CSKY_MMU);
    env->cpuid = CSKY_CPUID_C810;
    env->cp0.cpidr[0] = env->cpuid;
}

static void c810fv_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_C810);
    csky_set_feature(env, ABIV2_FPU);
    csky_set_feature(env, CSKY_MMU);
    env->cpuid = CSKY_CPUID_C810;
    env->cp0.cpidr[0] = env->cpuid;
}

static void c810tv_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_C810);
    csky_set_feature(env, ABIV2_TEE);
    csky_set_feature(env, CSKY_MMU);
    env->cpuid = CSKY_CPUID_C810;
    env->cp0.cpidr[0] = env->cpuid;
}

static void c810ft_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_C810);
    csky_set_feature(env, ABIV2_TEE);
    csky_set_feature(env, CSKY_MMU);
    env->cpuid = CSKY_CPUID_C810;
    env->cp0.cpidr[0] = env->cpuid;
}

static void c810ftv_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_C810);
    csky_set_feature(env, ABIV2_DSP);
    csky_set_feature(env, ABIV2_TEE);
    csky_set_feature(env, CSKY_MMU);
    env->cpuid = CSKY_CPUID_C810;
    env->cp0.cpidr[0] = env->cpuid;
}

static void c860_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_C860);
    csky_set_feature(env, ABIV2_ELRW);
    csky_set_feature(env, CSKY_MMU);
    env->cpuid = CSKY_CPUID_C860;
    env->cp0.cpidr[0] = env->cpuid;
}

static void c860v_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_C860);
    csky_set_feature(env, ABIV2_VDSP2);
    csky_set_feature(env, ABIV2_ELRW);
    csky_set_feature(env, CSKY_MMU);
    env->cpuid = CSKY_CPUID_C860;
    env->cp0.cpidr[0] = env->cpuid;
}

static void c860f_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_C860);
    csky_set_feature(env, ABIV2_FPU3);
    csky_set_feature(env, ABIV2_ELRW);
    csky_set_feature(env, CSKY_MMU);
    env->cpuid = CSKY_CPUID_C860;
    env->cp0.cpidr[0] = env->cpuid;
}

static void c860fv_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_C860);
    csky_set_feature(env, ABIV2_FPU3);
    csky_set_feature(env, ABIV2_VDSP2);
    csky_set_feature(env, ABIV2_ELRW);
    csky_set_feature(env, CSKY_MMU);
    env->cpuid = CSKY_CPUID_C860;
    env->cp0.cpidr[0] = env->cpuid;
}

static void s802_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_E802);
    csky_set_feature(env, ABIV2_ELRW);
    csky_set_feature(env, CSKY_MGU);
    env->cpuid = CSKY_CPUID_S802;
    env->cp0.cpidr[0] = env->cpuid;
}

static void s802t_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_E802);
    csky_set_feature(env, ABIV2_TEE);
    csky_set_feature(env, ABIV2_ELRW);
    csky_set_feature(env, CSKY_MGU);
    env->cpuid = CSKY_CPUID_S802;
    env->cp0.cpidr[0] = env->cpuid;
}

static void s803_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_E803);
    csky_set_feature(env, ABIV2_ELRW);
    csky_set_feature(env, CSKY_MGU);
    env->cpuid = CSKY_CPUID_S803;
    env->cp0.cpidr[0] = env->cpuid;
}

static void s803t_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_E803);
    csky_set_feature(env, ABIV2_TEE);
    csky_set_feature(env, ABIV2_ELRW);
    csky_set_feature(env, CSKY_MGU);
    env->cpuid = CSKY_CPUID_S803;
    env->cp0.cpidr[0] = env->cpuid;
}

static void r807_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_C807);
    csky_set_feature(env, CSKY_MGU);
    env->cpuid = CSKY_CPUID_R807;
    env->cp0.cpidr[0] = env->cpuid;
}

static void r807f_cpu_initfn(Object *obj)
{
    CSKYCPU *cpu = CSKY_CPU(obj);
    CPUCSKYState *env = &cpu->env;

    csky_set_feature(env, CPU_ABIV2);
    csky_set_feature(env, CPU_C807);
    csky_set_feature(env, ABIV2_FPU);
    csky_set_feature(env, CSKY_MGU);
    env->cpuid = CSKY_CPUID_R807;
    env->cp0.cpidr[0] = env->cpuid;
}

typedef struct CSKYCPUInfo {
    const char *name;
    void (*instance_init)(Object *obj);
} CSKYCPUInfo;

/*
 * postfix is alphabetical order: d(e), f, (h), j, (m), t, v
 * Edsp, Float, sHield, Java, Memory, Trust, Vdsp
 */
static const CSKYCPUInfo csky_cpus[] = {
    { .name = "ck510",       .instance_init = ck510_cpu_initfn },
    { .name = "ck520",       .instance_init = ck520_cpu_initfn },
    { .name = "ck610",       .instance_init = ck610_cpu_initfn },
    { .name = "ck610e",      .instance_init = ck610e_cpu_initfn },
    { .name = "ck610f",      .instance_init = ck610f_cpu_initfn },
    { .name = "ck610ef",     .instance_init = ck610ef_cpu_initfn },
    { .name = "ck801",       .instance_init = e801_cpu_initfn },
    { .name = "ck801t",      .instance_init = e801t_cpu_initfn },
    { .name = "ck802",       .instance_init = e802_cpu_initfn },
    { .name = "ck802h",      .instance_init = e802_cpu_initfn },
    { .name = "ck802j",      .instance_init = e802j_cpu_initfn },
    { .name = "ck802t",      .instance_init = e802t_cpu_initfn },
    { .name = "ck802ht",     .instance_init = e802t_cpu_initfn },
    { .name = "ck803",       .instance_init = e804_cpu_initfn },
    { .name = "ck803h",      .instance_init = e804_cpu_initfn },
    { .name = "ck803t",      .instance_init = e804t_cpu_initfn },
    { .name = "ck803ht",     .instance_init = e804t_cpu_initfn },
    { .name = "ck803f",      .instance_init = e804f_cpu_initfn },
    { .name = "ck803fh",     .instance_init = e804f_cpu_initfn },
    { .name = "ck803e",      .instance_init = e804d_cpu_initfn },
    { .name = "ck803eh",     .instance_init = e804d_cpu_initfn },
    { .name = "ck803et",     .instance_init = e804dt_cpu_initfn },
    { .name = "ck803eht",    .instance_init = e804dt_cpu_initfn },
    { .name = "ck803ef",     .instance_init = e804df_cpu_initfn },
    { .name = "ck803efh",    .instance_init = e804df_cpu_initfn },
    { .name = "ck803ft",     .instance_init = e804ft_cpu_initfn },
    { .name = "ck803eft",    .instance_init = e804dft_cpu_initfn },
    { .name = "ck803efht",   .instance_init = e804dft_cpu_initfn },
    { .name = "ck803r1",     .instance_init = e804_cpu_initfn },
    { .name = "ck803hr1",    .instance_init = e804_cpu_initfn },
    { .name = "ck803tr1",    .instance_init = e804t_cpu_initfn },
    { .name = "ck803htr1",   .instance_init = e804t_cpu_initfn },
    { .name = "ck803fr1",    .instance_init = e804f_cpu_initfn },
    { .name = "ck803fhr1",   .instance_init = e804f_cpu_initfn },
    { .name = "ck803er1",    .instance_init = e804d_cpu_initfn },
    { .name = "ck803ehr1",   .instance_init = e804d_cpu_initfn },
    { .name = "ck803etr1",   .instance_init = e804dt_cpu_initfn },
    { .name = "ck803ehtr1",  .instance_init = e804dt_cpu_initfn },
    { .name = "ck803efr1",   .instance_init = e804df_cpu_initfn },
    { .name = "ck803efhr1",  .instance_init = e804df_cpu_initfn },
    { .name = "ck803ftr1",   .instance_init = e804ft_cpu_initfn },
    { .name = "ck803fhtr1",  .instance_init = e804ft_cpu_initfn },
    { .name = "ck803eftr1",  .instance_init = e804dft_cpu_initfn },
    { .name = "ck803efhtr1", .instance_init = e804dft_cpu_initfn },
    { .name = "ck803s",      .instance_init = e804_cpu_initfn },
    { .name = "ck803sf",     .instance_init = e804f_cpu_initfn },
    { .name = "ck803sef",    .instance_init = e804df_cpu_initfn },
    { .name = "ck803st",     .instance_init = e804t_cpu_initfn },
    { .name = "ck803r2",     .instance_init = e804_cpu_initfn },
    { .name = "ck803hr2",    .instance_init = e804_cpu_initfn },
    { .name = "ck803tr2",    .instance_init = e804t_cpu_initfn },
    { .name = "ck803htr2",   .instance_init = e804t_cpu_initfn },
    { .name = "ck803fr2",    .instance_init = e804f_cpu_initfn },
    { .name = "ck803fhr2",   .instance_init = e804f_cpu_initfn },
    { .name = "ck803er2",    .instance_init = e804d_cpu_initfn },
    { .name = "ck803ehr2",   .instance_init = e804d_cpu_initfn },
    { .name = "ck803etr2",   .instance_init = e804df_cpu_initfn },
    { .name = "ck803ehtr2",  .instance_init = e804dft_cpu_initfn },
    { .name = "ck803efr2",   .instance_init = e804df_cpu_initfn },
    { .name = "ck803efhr2",  .instance_init = e804df_cpu_initfn },
    { .name = "ck803ftr2",   .instance_init = e804ft_cpu_initfn },
    { .name = "ck803fhtr2",  .instance_init = e804ft_cpu_initfn },
    { .name = "ck803eftr2",  .instance_init = e804dft_cpu_initfn },
    { .name = "ck803efhtr2", .instance_init = e804dft_cpu_initfn },
    { .name = "ck803r3",     .instance_init = e804_cpu_initfn },
    { .name = "ck803hr3",    .instance_init = e804_cpu_initfn },
    { .name = "ck803tr3",    .instance_init = e804t_cpu_initfn },
    { .name = "ck803htr3",   .instance_init = e804t_cpu_initfn },
    { .name = "ck803fr3",    .instance_init = e804f_cpu_initfn },
    { .name = "ck803fvr3",   .instance_init = i805f_cpu_initfn },
    { .name = "ck803fhr3",   .instance_init = e804f_cpu_initfn },
    { .name = "ck803er3",    .instance_init = e804d_cpu_initfn },
    { .name = "ck803ehr3",   .instance_init = e804d_cpu_initfn },
    { .name = "ck803etr3",   .instance_init = e804dt_cpu_initfn },
    { .name = "ck803ehtr3",  .instance_init = e804dt_cpu_initfn },
    { .name = "ck803efr3",   .instance_init = e804df_cpu_initfn },
    { .name = "ck803efvr3",  .instance_init = i805dft_cpu_initfn },
    { .name = "ck803efhr3",  .instance_init = e804df_cpu_initfn },
    { .name = "ck803ftr3",   .instance_init = e804ft_cpu_initfn },
    { .name = "ck803fhtr3",  .instance_init = e804ft_cpu_initfn },
    { .name = "ck803eftr3",  .instance_init = e804dft_cpu_initfn },
    { .name = "ck803efhtr3", .instance_init = e804dft_cpu_initfn },
    { .name = "ck804",       .instance_init = e804_cpu_initfn },
    { .name = "ck804e",      .instance_init = e804_cpu_initfn },
    { .name = "ck804ef",     .instance_init = e804f_cpu_initfn },
    { .name = "ck804efh",    .instance_init = e804f_cpu_initfn },
    { .name = "ck804efht",   .instance_init = e804ft_cpu_initfn },
    { .name = "ck804eft",    .instance_init = e804ft_cpu_initfn },
    { .name = "ck804eh",     .instance_init = e804_cpu_initfn },
    { .name = "ck804eht",    .instance_init = e804t_cpu_initfn },
    { .name = "ck804et",     .instance_init = e804t_cpu_initfn },
    { .name = "ck804f",      .instance_init = e804f_cpu_initfn },
    { .name = "ck804fh",     .instance_init = e804f_cpu_initfn },
    { .name = "ck804ft",     .instance_init = e804ft_cpu_initfn },
    { .name = "ck804h",      .instance_init = e804_cpu_initfn },
    { .name = "ck804ht",     .instance_init = e804t_cpu_initfn },
    { .name = "ck804t",      .instance_init = e804t_cpu_initfn },
    { .name = "ck805",       .instance_init = i805_cpu_initfn },
    { .name = "ck805e",      .instance_init = i805dft_cpu_initfn },
    { .name = "ck805f",      .instance_init = i805f_cpu_initfn },
    { .name = "ck805t",      .instance_init = i805dft_cpu_initfn },
    { .name = "ck805ef",     .instance_init = i805dft_cpu_initfn },
    { .name = "ck805et",     .instance_init = i805dft_cpu_initfn },
    { .name = "ck805ft",     .instance_init = i805dft_cpu_initfn },
    { .name = "ck805eft",    .instance_init = i805dft_cpu_initfn },
    { .name = "ck807",       .instance_init = c807_cpu_initfn },
    { .name = "ck807e",      .instance_init = c807_cpu_initfn },
    { .name = "ck807f",      .instance_init = c807f_cpu_initfn },
    { .name = "ck807ef",     .instance_init = c807f_cpu_initfn },
    { .name = "ck810",       .instance_init = c810_cpu_initfn },
    { .name = "ck810v",      .instance_init = c810v_cpu_initfn },
    { .name = "ck810f",      .instance_init = c810f_cpu_initfn },
    { .name = "ck810t",      .instance_init = c810t_cpu_initfn },
    { .name = "ck810fv",     .instance_init = c810fv_cpu_initfn },
    { .name = "ck810tv",     .instance_init = c810tv_cpu_initfn },
    { .name = "ck810ft",     .instance_init = c810ft_cpu_initfn },
    { .name = "ck810ftv",    .instance_init = c810ftv_cpu_initfn },
    { .name = "ck810e",      .instance_init = c810_cpu_initfn },
    { .name = "ck810et",     .instance_init = c810t_cpu_initfn },
    { .name = "ck810ef",     .instance_init = c810f_cpu_initfn },
    { .name = "ck810efm",    .instance_init = c810f_cpu_initfn },
    { .name = "ck810eft",    .instance_init = c810ft_cpu_initfn },
    { .name = "ck860",       .instance_init = c860_cpu_initfn },
    { .name = "ck860v",      .instance_init = c860v_cpu_initfn },
    { .name = "ck860f",      .instance_init = c860f_cpu_initfn },
    { .name = "ck860fv",     .instance_init = c860fv_cpu_initfn },
    { .name = "e801",        .instance_init = e801_cpu_initfn },
    { .name = "e802",        .instance_init = e802_cpu_initfn },
    { .name = "e802t",       .instance_init = e802t_cpu_initfn },
    { .name = "e803",        .instance_init = e803_cpu_initfn },
    { .name = "e803t",       .instance_init = e803t_cpu_initfn },
    { .name = "e804d",       .instance_init = e804d_cpu_initfn },
    { .name = "e804dt",      .instance_init = e804dt_cpu_initfn },
    { .name = "e804f",       .instance_init = e804f_cpu_initfn },
    { .name = "e804ft",      .instance_init = e804ft_cpu_initfn },
    { .name = "e804df",      .instance_init = e804df_cpu_initfn },
    { .name = "e804dft",     .instance_init = e804dft_cpu_initfn },
    { .name = "c807",        .instance_init = c807_cpu_initfn },
    { .name = "c807f",       .instance_init = c807f_cpu_initfn },
    { .name = "c807fv",      .instance_init = c807fv_cpu_initfn },
    { .name = "c810",        .instance_init = c810_cpu_initfn },
    { .name = "c810v",       .instance_init = c810v_cpu_initfn },
    { .name = "c810t",       .instance_init = c810t_cpu_initfn },
    { .name = "c810tv",      .instance_init = c810tv_cpu_initfn },
    { .name = "c860",        .instance_init = c860_cpu_initfn },
    { .name = "c860v",       .instance_init = c860v_cpu_initfn },
    { .name = "s802",        .instance_init = s802_cpu_initfn },
    { .name = "s802t",       .instance_init = s802t_cpu_initfn },
    { .name = "s803",        .instance_init = s803_cpu_initfn },
    { .name = "s803t",       .instance_init = s803t_cpu_initfn },
    { .name = "r807",        .instance_init = r807_cpu_initfn },
    { .name = "r807f",       .instance_init = r807f_cpu_initfn },
    { .name = "i805",        .instance_init = i805_cpu_initfn },
    { .name = "i805f",       .instance_init = i805f_cpu_initfn },
    { .name = NULL }
};

static int csky_fpu3_gdb_get_reg(CPUState *cs, GByteArray *buf, int reg)
{
    CPUCSKYState *env = cpu_env(cs);
    int nregs = 16, ret = 0;

    if (reg < nregs) {
        ret = gdb_get_reg64(buf, env->vfp.reg[reg + 16].fpu[0]);
    }
    return ret;
}

static int csky_fpu3_gdb_set_reg(CPUState *cs, uint8_t *buf, int reg)
{
    CPUCSKYState *env = cpu_env(cs);
    int nregs = 16, ret = 0;

    if (reg < nregs) {
#if TARGET_BIG_ENDIAN
        env->vfp.reg[reg + 16].fpu[0] = ldq_be_p(buf);
#else
        env->vfp.reg[reg + 16].fpu[0] = ldq_le_p(buf);
#endif
        ret = 8;
    }
    return ret;
}

static void csky_cpu_register_gdb_regs_for_features(CSKYCPU *cpu)
{
    CPUState *cs = CPU(cpu);
    CPUCSKYState *env = &cpu->env;

    if (csky_has_feature(env, ABIV2_FPU3)) {
        cs->gdb_num_regs = 1172;
        gdb_register_coprocessor(cs, csky_fpu3_gdb_get_reg,
                                 csky_fpu3_gdb_set_reg,
                                 gdb_find_static_feature("csky-fpu3.xml"));
    }
}

static void csky_cpu_realizefn(DeviceState *dev, Error **errp)
{
    CPUState *cs = CPU(dev);
    CSKYCPU *cpu = CSKY_CPU(dev);
    CSKYCPUClass *cc = CSKY_CPU_GET_CLASS(dev);
    Error *local_err = NULL;

    cpu_common_realize(cs, &local_err);
    if (local_err != NULL) {
        error_propagate(errp, local_err);
        return;
    }
    csky_cpu_register_gdb_regs_for_features(cpu);
    qemu_init_vcpu(cs);
    cpu_reset(cs);

    cc->parent_realize(dev, errp);
}

static void csky_cpu_initfn(Object *obj)
{
}

static const VMStateDescription vmstate_csky_cpu = {
    .name = "cpu",
    .unmigratable = 1,
};

static const char *csky_gdb_arch_name(CPUState *cs)
{
    return "csky";
}

#include "hw/core/sysemu-cpu-ops.h"

static const struct SysemuCPUOps csky_sysemu_ops = {
    .has_work = csky_cpu_has_work,
    .get_phys_addr_debug = csky_cpu_get_phys_page_debug,
    .legacy_vmsd = &vmstate_csky_cpu,
};

static const TCGCPUOps csky_tcg_ops = {
    .guest_default_memory_order = 0,
    .mttcg_supported = true,

    .initialize = csky_translate_init,
    .translate_code = csky_translate_code,
    .get_tb_cpu_state = csky_get_tb_cpu_state,
    .synchronize_from_tb = csky_cpu_synchronize_from_tb,
    .restore_state_to_opc = csky_restore_state_to_opc,
    .mmu_index = csky_cpu_mmu_index,
    .tlb_fill = csky_cpu_tlb_fill,
    .pointer_wrap = cpu_pointer_wrap_uint32,
    .cpu_exec_enter = csky_cpu_exec_enter,
    .cpu_exec_interrupt = csky_cpu_exec_interrupt,
    .cpu_exec_halt = csky_cpu_has_work,
    .cpu_exec_reset = cpu_reset,
    .do_interrupt = csky_cpu_do_interrupt,
    .do_unaligned_access = csky_cpu_do_unaligned_access,
};

static void csky_cpu_class_init(ObjectClass *c, const void *data)
{
    CSKYCPUClass *mcc = CSKY_CPU_CLASS(c);
    CPUClass *cc = CPU_CLASS(c);
    DeviceClass *dc = DEVICE_CLASS(c);
    ResettableClass *rc = RESETTABLE_CLASS(c);

    mcc->rmr = 0x1;
    device_class_set_parent_realize(dc, csky_cpu_realizefn,
                                    &mcc->parent_realize);
    resettable_class_set_parent_phases(rc, NULL, csky_cpu_reset_hold, NULL,
                                       &mcc->parent_phases);

    cc->class_by_name = csky_cpu_class_by_name;
    cc->dump_state = csky_cpu_dump_state;
    cc->set_pc = csky_cpu_set_pc;
    cc->get_pc = csky_cpu_get_pc;
    cc->gdb_read_register = csky_cpu_gdb_read_register;
    cc->gdb_write_register = csky_cpu_gdb_write_register;
    cc->sysemu_ops = &csky_sysemu_ops;
    cc->tcg_ops = &csky_tcg_ops;
    cc->disas_set_info = csky_cpu_disas_set_info;
#if defined(TARGET_CSKYV1)

#if defined(CONFIG_USER_ONLY)
    cc->gdb_core_xml_file = "csky-abiv1-linux-user-core.xml";
#else
    cc->gdb_core_xml_file = "csky-abiv1-softmmu-core.xml";
#endif

#else

#if defined(CONFIG_USER_ONLY)
    cc->gdb_core_xml_file = "csky-abiv2-linux-user-core.xml";
#else
    cc->gdb_core_xml_file = "csky-abiv2-softmmu-core.xml";
#endif

#endif
    cc->gdb_arch_name = csky_gdb_arch_name;
    cc->memory_rw_debug = csky_cpu_memory_rw_debug;
}

static void register_cpu_type(const CSKYCPUInfo *info)
{
    TypeInfo type_info = {
        .parent = TYPE_CSKY_CPU,
        .instance_init = info->instance_init,
    };

    type_info.name = g_strdup_printf("%s-" TYPE_CSKY_CPU, info->name);
    type_register_static(&type_info);
    g_free((void *)type_info.name);
}

static const TypeInfo csky_cpu_type_info = {
    .name = TYPE_CSKY_CPU,
    .parent = TYPE_CPU,
    .instance_size = sizeof(CSKYCPU),
    .instance_align = __alignof(CSKYCPU),
    .instance_init = csky_cpu_initfn,
    .abstract = true,
    .class_size = sizeof(CSKYCPUClass),
    .class_init = csky_cpu_class_init,
};

static void csky_cpu_register_types(void)
{
    int i;

    type_register_static(&csky_cpu_type_info);
    for (i = 0; csky_cpus[i].name; i++) {
        register_cpu_type(&csky_cpus[i]);
    }
}

type_init(csky_cpu_register_types)
