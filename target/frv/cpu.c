/*
 * QEMU Fujitsu FR-V CPU
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

static void frv_cpu_set_pc(CPUState *cs, vaddr value)
{
    cpu_env(cs)->pc = value;
}

static vaddr frv_cpu_get_pc(CPUState *cs)
{
    return cpu_env(cs)->pc;
}

static void frv_restore_state_to_opc(CPUState *cs,
                                     const TranslationBlock *tb,
                                     const uint64_t *data)
{
    CPUFRVState *env = cpu_env(cs);

    env->pc = data[0];
    env->excp_next = data[1];
}

static bool frv_cpu_has_work(CPUState *cs)
{
    return cpu_test_interrupt(cs, CPU_INTERRUPT_HARD);
}

static TCGTBCPUState frv_get_tb_cpu_state(CPUState *cs)
{
    CPUFRVState *env = cpu_env(cs);
    uint32_t flags = 0;

    flags = FIELD_DP32(flags, TBFLAGS, USER, frv_user_mode(env));
    flags = FIELD_DP32(flags, TBFLAGS, DEBUG, env->debug_mode);
    return (TCGTBCPUState){ .pc = env->pc, .flags = flags };
}

static int frv_cpu_mmu_index(CPUState *cs, bool ifetch)
{
    return frv_user_mode(cpu_env(cs));
}

static void frv_cpu_reset_hold(Object *obj, ResetType type)
{
    CPUState *cs = CPU(obj);
    FRVCPUClass *fcc = FRV_CPU_GET_CLASS(obj);
    CPUFRVState *env = cpu_env(cs);

    if (fcc->parent_phases.hold) {
        fcc->parent_phases.hold(obj, type);
    }

    memset(env, 0, offsetof(CPUFRVState, fp_status));

    /* FR401 core, MB93401A silicon, supervisor mode, big endian */
    env->spr[SPR_PSR] = (2u << R_PSR_IMPLE_SHIFT) | (1u << R_PSR_VER_SHIFT) |
                        R_PSR_S_MASK | R_PSR_BE_MASK;
    env->pc = 0xff000000;

    set_float_rounding_mode(float_round_nearest_even, &env->fp_status);
    set_default_nan_mode(true, &env->fp_status);

    cs->exception_index = -1;
}

#ifndef CONFIG_USER_ONLY
/* The interrupt controller presents one level, 1-15, or 0 for none */
static void frv_cpu_set_irq(void *opaque, int irq, int level)
{
    FRVCPU *cpu = opaque;

    cpu->env.irq_level = level;
    frv_update_irq(&cpu->env);
}
#endif

static ObjectClass *frv_cpu_class_by_name(const char *cpu_model)
{
    return object_class_by_name(TYPE_FRV_CPU);
}

static void frv_cpu_realizefn(DeviceState *dev, Error **errp)
{
    CPUState *cs = CPU(dev);
    FRVCPUClass *fcc = FRV_CPU_GET_CLASS(dev);
    Error *local_err = NULL;

    cpu_common_realize(cs, &local_err);
    if (local_err != NULL) {
        error_propagate(errp, local_err);
        return;
    }

    qemu_init_vcpu(cs);
    cpu_reset(cs);

#ifndef CONFIG_USER_ONLY
    qdev_init_gpio_in_named(DEVICE(cs), frv_cpu_set_irq, "IRQ", 1);
#endif

    fcc->parent_realize(dev, errp);
}

static void frv_cpu_disas_set_info(const CPUState *cpu,
                                   disassemble_info *info)
{
    info->endian = BFD_ENDIAN_BIG;
}

static int frv_cpu_gdb_read_register(CPUState *cs, GByteArray *mem_buf, int n)
{
    CPUFRVState *env = cpu_env(cs);

    if (n < NUM_GR) {
        return gdb_get_reg32(mem_buf, env->gr[n]);
    } else if (n < NUM_GR + NUM_FR) {
        return gdb_get_reg32(mem_buf, env->fr[n - NUM_GR]);
    } else if (n == NUM_GR + NUM_FR) {
        return gdb_get_reg32(mem_buf, env->pc);
    }
    return 0;
}

static int frv_cpu_gdb_write_register(CPUState *cs, uint8_t *mem_buf, int n)
{
    CPUFRVState *env = cpu_env(cs);
    uint32_t val = ldl_p(mem_buf);

    if (n == 0) {
        return 4;
    } else if (n < NUM_GR) {
        env->gr[n] = val;
    } else if (n < NUM_GR + NUM_FR) {
        env->fr[n - NUM_GR] = val;
    } else if (n == NUM_GR + NUM_FR) {
        env->pc = val;
    } else {
        return 0;
    }
    return 4;
}

#ifndef CONFIG_USER_ONLY
#include "hw/core/sysemu-cpu-ops.h"

static const struct SysemuCPUOps frv_sysemu_ops = {
    .has_work = frv_cpu_has_work,
    .get_phys_addr_debug = frv_cpu_get_phys_page_debug,
};
#endif

static const TCGCPUOps frv_tcg_ops = {
    .guest_default_memory_order = TCG_MO_ALL,
    .mttcg_supported = false,

    .initialize = frv_tcg_init,
    .translate_code = frv_translate_code,
    .get_tb_cpu_state = frv_get_tb_cpu_state,
    .restore_state_to_opc = frv_restore_state_to_opc,
    .mmu_index = frv_cpu_mmu_index,
    .pointer_wrap = cpu_pointer_wrap_uint32,

#ifndef CONFIG_USER_ONLY
    .tlb_fill = frv_cpu_tlb_fill,
    .do_unaligned_access = frv_cpu_do_unaligned_access,
    .cpu_exec_interrupt = frv_cpu_exec_interrupt,
    .cpu_exec_halt = frv_cpu_has_work,
    .cpu_exec_reset = cpu_reset,
    .do_interrupt = frv_cpu_do_interrupt,
#endif
};

static void frv_cpu_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    CPUClass *cc = CPU_CLASS(oc);
    FRVCPUClass *fcc = FRV_CPU_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    device_class_set_parent_realize(dc, frv_cpu_realizefn,
                                    &fcc->parent_realize);
    resettable_class_set_parent_phases(rc, NULL, frv_cpu_reset_hold, NULL,
                                       &fcc->parent_phases);

    cc->class_by_name = frv_cpu_class_by_name;
    cc->dump_state = frv_cpu_dump_state;
    cc->set_pc = frv_cpu_set_pc;
    cc->get_pc = frv_cpu_get_pc;
    cc->disas_set_info = frv_cpu_disas_set_info;
#ifndef CONFIG_USER_ONLY
    cc->sysemu_ops = &frv_sysemu_ops;
#endif
    cc->gdb_read_register = frv_cpu_gdb_read_register;
    cc->gdb_write_register = frv_cpu_gdb_write_register;
    cc->gdb_num_core_regs = NUM_GR + NUM_FR + 1;
    cc->tcg_ops = &frv_tcg_ops;
}

static const TypeInfo frv_cpu_type_info = {
    .name = TYPE_FRV_CPU,
    .parent = TYPE_CPU,
    .instance_size = sizeof(FRVCPU),
    .instance_align = __alignof(FRVCPU),
    .class_size = sizeof(FRVCPUClass),
    .class_init = frv_cpu_class_init,
};

static void frv_cpu_register_types(void)
{
    type_register_static(&frv_cpu_type_info);
}

type_init(frv_cpu_register_types)
