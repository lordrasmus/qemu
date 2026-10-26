/*
 * QEMU NEC V850E1 CPU
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
#include "gdbstub/helpers.h"

static void v850_cpu_set_pc(CPUState *cs, vaddr value)
{
    cpu_env(cs)->pc = value & ~1;
}

static vaddr v850_cpu_get_pc(CPUState *cs)
{
    return cpu_env(cs)->pc;
}

static void v850_restore_state_to_opc(CPUState *cs,
                                      const TranslationBlock *tb,
                                      const uint64_t *data)
{
    cpu_env(cs)->pc = data[0];
}

static bool v850_cpu_has_work(CPUState *cs)
{
    return cpu_test_interrupt(cs, CPU_INTERRUPT_HARD);
}

static TCGTBCPUState v850_get_tb_cpu_state(CPUState *cs)
{
    return (TCGTBCPUState){ .pc = cpu_env(cs)->pc };
}

static int v850_cpu_mmu_index(CPUState *cs, bool ifetch)
{
    return 0;
}

static void v850_cpu_reset_hold(Object *obj, ResetType type)
{
    CPUState *cs = CPU(obj);
    V850CPUClass *vcc = V850_CPU_GET_CLASS(obj);
    CPUV850State *env = cpu_env(cs);

    if (vcc->parent_phases.hold) {
        vcc->parent_phases.hold(obj, type);
    }

    memset(env->gpr, 0, sizeof(env->gpr));
    memset(env->sr, 0, sizeof(env->sr));
    env->pc = 0;
    v850_psw_write(env, PSW_ID);
    env->irq_pending = 0;
    cs->exception_index = -1;
}

#ifndef CONFIG_USER_ONLY
/* A rising edge latches the request until the CPU takes it. */
static void v850_cpu_set_irq(void *opaque, int irq, int level)
{
    V850CPU *cpu = opaque;
    CPUV850State *env = &cpu->env;
    uint32_t bit = 1u << irq;

    if (level && !(env->irq_lines & bit)) {
        env->irq_pending |= bit;
    }
    env->irq_lines = level ? env->irq_lines | bit : env->irq_lines & ~bit;
    v850_update_irq(env);
}
#endif

static ObjectClass *v850_cpu_class_by_name(const char *cpu_model)
{
    return object_class_by_name(TYPE_V850_CPU);
}

static void v850_cpu_realizefn(DeviceState *dev, Error **errp)
{
    CPUState *cs = CPU(dev);
    V850CPUClass *vcc = V850_CPU_GET_CLASS(dev);
    Error *local_err = NULL;

    cpu_common_realize(cs, &local_err);
    if (local_err != NULL) {
        error_propagate(errp, local_err);
        return;
    }

    qemu_init_vcpu(cs);
    cpu_reset(cs);

#ifndef CONFIG_USER_ONLY
    qdev_init_gpio_in_named(DEVICE(cs), v850_cpu_set_irq, "IRQ",
                            V850_NUM_IRQS);
#endif

    vcc->parent_realize(dev, errp);
}

static void v850_cpu_disas_set_info(const CPUState *cpu,
                                    disassemble_info *info)
{
    info->endian = BFD_ENDIAN_LITTLE;
}

/* gdb's numbering: r0-r31, the 32 system registers, then pc */
static int v850_cpu_gdb_read_register(CPUState *cs, GByteArray *mem_buf,
                                      int n)
{
    CPUV850State *env = cpu_env(cs);

    if (n < NUM_GPR) {
        return gdb_get_reg32(mem_buf, env->gpr[n]);
    } else if (n == NUM_GPR + SR_PSW) {
        return gdb_get_reg32(mem_buf, v850_psw_read(env));
    } else if (n < NUM_GPR + NUM_SR) {
        return gdb_get_reg32(mem_buf, env->sr[n - NUM_GPR]);
    } else if (n == NUM_GPR + NUM_SR) {
        return gdb_get_reg32(mem_buf, env->pc);
    }
    return 0;
}

static int v850_cpu_gdb_write_register(CPUState *cs, uint8_t *mem_buf, int n)
{
    CPUV850State *env = cpu_env(cs);
    uint32_t val = ldl_p(mem_buf);

    if (n == 0) {
        /* r0 stays zero */
    } else if (n < NUM_GPR) {
        env->gpr[n] = val;
    } else if (n == NUM_GPR + SR_PSW) {
        v850_psw_write(env, val);
    } else if (n < NUM_GPR + NUM_SR) {
        env->sr[n - NUM_GPR] = val;
    } else if (n == NUM_GPR + NUM_SR) {
        env->pc = val & ~1;
    } else {
        return 0;
    }
    return 4;
}

#ifndef CONFIG_USER_ONLY
#include "hw/core/sysemu-cpu-ops.h"

static const struct SysemuCPUOps v850_sysemu_ops = {
    .has_work = v850_cpu_has_work,
    .get_phys_addr_debug = v850_cpu_get_phys_page_debug,
};
#endif

static const TCGCPUOps v850_tcg_ops = {
    .guest_default_memory_order = TCG_MO_ALL,
    .mttcg_supported = false,

    .initialize = v850_tcg_init,
    .translate_code = v850_translate_code,
    .get_tb_cpu_state = v850_get_tb_cpu_state,
    .restore_state_to_opc = v850_restore_state_to_opc,
    .mmu_index = v850_cpu_mmu_index,
    .pointer_wrap = cpu_pointer_wrap_uint32,

#ifndef CONFIG_USER_ONLY
    .tlb_fill = v850_cpu_tlb_fill,
    .cpu_exec_interrupt = v850_cpu_exec_interrupt,
    .cpu_exec_halt = v850_cpu_has_work,
    .cpu_exec_reset = cpu_reset,
    .do_interrupt = v850_cpu_do_interrupt,
#endif
};

static void v850_cpu_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    CPUClass *cc = CPU_CLASS(oc);
    V850CPUClass *vcc = V850_CPU_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    device_class_set_parent_realize(dc, v850_cpu_realizefn,
                                    &vcc->parent_realize);
    resettable_class_set_parent_phases(rc, NULL, v850_cpu_reset_hold, NULL,
                                       &vcc->parent_phases);

    cc->class_by_name = v850_cpu_class_by_name;
    cc->dump_state = v850_cpu_dump_state;
    cc->set_pc = v850_cpu_set_pc;
    cc->get_pc = v850_cpu_get_pc;
    cc->disas_set_info = v850_cpu_disas_set_info;
#ifndef CONFIG_USER_ONLY
    cc->sysemu_ops = &v850_sysemu_ops;
#endif
    cc->gdb_read_register = v850_cpu_gdb_read_register;
    cc->gdb_write_register = v850_cpu_gdb_write_register;
    cc->gdb_num_core_regs = NUM_GPR + NUM_SR + 1;
    cc->tcg_ops = &v850_tcg_ops;
}

static const TypeInfo v850_cpu_type_info = {
    .name = TYPE_V850_CPU,
    .parent = TYPE_CPU,
    .instance_size = sizeof(V850CPU),
    .instance_align = __alignof(V850CPU),
    .class_size = sizeof(V850CPUClass),
    .class_init = v850_cpu_class_init,
};

static void v850_cpu_register_types(void)
{
    type_register_static(&v850_cpu_type_info);
}

type_init(v850_cpu_register_types)
