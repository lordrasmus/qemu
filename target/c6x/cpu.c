/*
 * QEMU TI C64x+ / C674x CPU
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "cpu.h"
#include "exec/translation-block.h"
#include "exec/cputlb.h"
#include "accel/tcg/cpu-ops.h"
#include "exec/log.h"
#include "fpu/softfloat-helpers.h"
#include "gdbstub/helpers.h"
#include "hw/core/qdev-properties.h"

static void c6x_cpu_set_pc(CPUState *cs, vaddr value)
{
    cpu_env(cs)->pc = value;
}

static vaddr c6x_cpu_get_pc(CPUState *cs)
{
    return cpu_env(cs)->pc;
}

static void c6x_restore_state_to_opc(CPUState *cs,
                                     const TranslationBlock *tb,
                                     const uint64_t *data)
{
    cpu_env(cs)->pc = data[0];
}

static bool c6x_cpu_has_work(CPUState *cs)
{
    return cpu_test_interrupt(cs, CPU_INTERRUPT_HARD);
}

static TCGTBCPUState c6x_get_tb_cpu_state(CPUState *cs)
{
    CPUC6xState *env = cpu_env(cs);
    uint32_t flags = 0;
    int br = 0;

    for (int i = 0; i < env->dq_live && !br; i++) {
        for (int j = 0; j < env->dq_n[i]; j++) {
            if (env->dq[i][j].dest == DQ_DEST_PC) {
                br = i + 1;
                break;
            }
        }
    }
    flags = FIELD_DP32(flags, TBFLAGS, USER, c6x_user_mode(env));
    flags = FIELD_DP32(flags, TBFLAGS, DQ, env->dq_live);
    flags = FIELD_DP32(flags, TBFLAGS, BR, br);
    flags = FIELD_DP32(flags, TBFLAGS, CLEAN, MIN(env->clean_eps, 5));
    flags = FIELD_DP32(flags, TBFLAGS, CIRC, env->amr != 0);
    return (TCGTBCPUState){ .pc = env->pc, .flags = flags };
}

static int c6x_cpu_mmu_index(CPUState *cs, bool ifetch)
{
    return c6x_user_mode(cpu_env(cs));
}

static void c6x_cpu_reset_hold(Object *obj, ResetType type)
{
    CPUState *cs = CPU(obj);
    C6xCPUClass *ccc = C6X_CPU_GET_CLASS(obj);
    CPUC6xState *env = cpu_env(cs);
    uint32_t freq = env->cpu_freq;

    if (ccc->parent_phases.hold) {
        ccc->parent_phases.hold(obj, type);
    }

    memset(env, 0, offsetof(CPUC6xState, cpu_freq));
    env->cpu_freq = freq;
    env->tsc_running = false;
    env->tsc_base = 0;
    env->tsch = 0;
    memset(env->dq_n, 0, sizeof(env->dq_n));
    env->dq_live = 0;
    env->ll_valid = 0;
    env->sl_valid = 0;
    env->fadcr = env->faucr = env->fmcr = 0;

    /* C64x+ CPU ID, revision 0, little endian */
    env->csr = (0x10 << 24) | R_CSR_EN_MASK;
    env->ier = 1;
    env->gfpgfr = 0x0700001d;
    env->clean_eps = 5;

    /* the C67x+ units flush denormals and round to nearest */
    set_float_rounding_mode(float_round_nearest_even, &env->fp_status);
    set_flush_to_zero(true, &env->fp_status);
    set_flush_inputs_to_zero(true, &env->fp_status);
    set_default_nan_mode(true, &env->fp_status);
    /* NaN results are 0x7fffffff / 0x7fffffffffffffff */
    set_float_default_nan_pattern(0b01111111, &env->fp_status);
    set_float_2nan_prop_rule(float_2nan_prop_s_ab, &env->fp_status);
    set_float_ftz_detection(float_ftz_before_rounding, &env->fp_status);

    cs->exception_index = -1;
}

#ifndef CONFIG_USER_ONLY
/* INT4-INT15: a rising edge sets the interrupt flag */
static void c6x_cpu_set_irq(void *opaque, int irq, int level)
{
    C6xCPU *cpu = opaque;
    CPUC6xState *env = &cpu->env;
    uint32_t bit = 1u << irq;

    if (level && !(env->irq_lines & bit)) {
        env->ifr |= bit;
    }
    env->irq_lines = level ? env->irq_lines | bit : env->irq_lines & ~bit;
    c6x_update_irq(env);
}
#endif

static ObjectClass *c6x_cpu_class_by_name(const char *cpu_model)
{
    return object_class_by_name(TYPE_C6X_CPU);
}

static void c6x_cpu_realizefn(DeviceState *dev, Error **errp)
{
    CPUState *cs = CPU(dev);
    C6xCPUClass *ccc = C6X_CPU_GET_CLASS(dev);
    Error *local_err = NULL;

    cpu_common_realize(cs, &local_err);
    if (local_err != NULL) {
        error_propagate(errp, local_err);
        return;
    }

    qemu_init_vcpu(cs);
    cpu_reset(cs);

#ifndef CONFIG_USER_ONLY
    qdev_init_gpio_in_named(DEVICE(cs), c6x_cpu_set_irq, "IRQ", 16);
#endif

    ccc->parent_realize(dev, errp);
}

static void c6x_cpu_disas_set_info(const CPUState *cpu,
                                   disassemble_info *info)
{
    info->endian = BFD_ENDIAN_LITTLE;
}

static int c6x_cpu_gdb_read_register(CPUState *cs, GByteArray *mem_buf, int n)
{
    CPUC6xState *env = cpu_env(cs);

    if (n < NUM_GPR) {
        return gdb_get_reg32(mem_buf, env->gpr[n]);
    } else if (n == NUM_GPR) {
        return gdb_get_reg32(mem_buf, env->pc);
    }
    return 0;
}

static int c6x_cpu_gdb_write_register(CPUState *cs, uint8_t *mem_buf, int n)
{
    CPUC6xState *env = cpu_env(cs);
    uint32_t val = ldl_p(mem_buf);

    if (n < NUM_GPR) {
        env->gpr[n] = val;
    } else if (n == NUM_GPR) {
        env->pc = val;
    } else {
        return 0;
    }
    return 4;
}

static const Property c6x_cpu_properties[] = {
    DEFINE_PROP_UINT32("clock-frequency", C6xCPU, env.cpu_freq, 1000000000),
};

#ifndef CONFIG_USER_ONLY
#include "hw/core/sysemu-cpu-ops.h"

static const struct SysemuCPUOps c6x_sysemu_ops = {
    .has_work = c6x_cpu_has_work,
    .get_phys_addr_debug = c6x_cpu_get_phys_page_debug,
};
#endif

static const TCGCPUOps c6x_tcg_ops = {
    .guest_default_memory_order = TCG_MO_ALL,
    .mttcg_supported = false,

    .initialize = c6x_tcg_init,
    .translate_code = c6x_translate_code,
    .get_tb_cpu_state = c6x_get_tb_cpu_state,
    .restore_state_to_opc = c6x_restore_state_to_opc,
    .mmu_index = c6x_cpu_mmu_index,
    .pointer_wrap = cpu_pointer_wrap_uint32,

#ifndef CONFIG_USER_ONLY
    .tlb_fill = c6x_cpu_tlb_fill,
    .cpu_exec_interrupt = c6x_cpu_exec_interrupt,
    .cpu_exec_halt = c6x_cpu_has_work,
    .cpu_exec_reset = cpu_reset,
    .do_interrupt = c6x_cpu_do_interrupt,
#endif
};

static void c6x_cpu_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    CPUClass *cc = CPU_CLASS(oc);
    C6xCPUClass *ccc = C6X_CPU_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    device_class_set_parent_realize(dc, c6x_cpu_realizefn,
                                    &ccc->parent_realize);
    resettable_class_set_parent_phases(rc, NULL, c6x_cpu_reset_hold, NULL,
                                       &ccc->parent_phases);
    device_class_set_props(dc, c6x_cpu_properties);

    cc->class_by_name = c6x_cpu_class_by_name;
    cc->dump_state = c6x_cpu_dump_state;
    cc->set_pc = c6x_cpu_set_pc;
    cc->get_pc = c6x_cpu_get_pc;
    cc->disas_set_info = c6x_cpu_disas_set_info;
#ifndef CONFIG_USER_ONLY
    cc->sysemu_ops = &c6x_sysemu_ops;
#endif
    cc->gdb_read_register = c6x_cpu_gdb_read_register;
    cc->gdb_write_register = c6x_cpu_gdb_write_register;
    cc->gdb_num_core_regs = NUM_GPR + 1;
    cc->tcg_ops = &c6x_tcg_ops;
}

static const TypeInfo c6x_cpu_type_info = {
    .name = TYPE_C6X_CPU,
    .parent = TYPE_CPU,
    .instance_size = sizeof(C6xCPU),
    .instance_align = __alignof(C6xCPU),
    .class_size = sizeof(C6xCPUClass),
    .class_init = c6x_cpu_class_init,
};

static void c6x_cpu_register_types(void)
{
    type_register_static(&c6x_cpu_type_info);
}

type_init(c6x_cpu_register_types)
