/*
 * Blackfin CPU
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/qemu-print.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "cpu.h"
#include "exec/cpu-interrupt.h"
#include "exec/translation-block.h"
#include "accel/tcg/cpu-ops.h"
#include "disas/dis-asm.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysemu-cpu-ops.h"

static void bfin_cpu_set_pc(CPUState *cs, vaddr value)
{
    cpu_env(cs)->pc = value;
}

static vaddr bfin_cpu_get_pc(CPUState *cs)
{
    return cpu_env(cs)->pc;
}

static void bfin_restore_state_to_opc(CPUState *cs,
                                      const TranslationBlock *tb,
                                      const uint64_t *data)
{
    cpu_env(cs)->pc = data[0];
}

/* the loop bottoms are part of the key, see gen_hwloop() */
static TCGTBCPUState bfin_get_tb_cpu_state(CPUState *cs)
{
    CPUBfinState *env = cpu_env(cs);

    return (TCGTBCPUState){
        .pc = env->pc,
        .flags = bfin_supervisor(env) ? BFIN_TBFLAG_SUPER : 0,
        .cs_base = env->reg[BFIN_LB0] |
                   ((uint64_t)env->reg[BFIN_LB1] << 32),
    };
}

static int bfin_cpu_mmu_index(CPUState *cs, bool ifetch)
{
    return bfin_supervisor(cpu_env(cs)) ? MMU_SUPER_IDX : MMU_USER_IDX;
}

static bool bfin_cpu_has_work(CPUState *cs)
{
    return cpu_test_interrupt(cs, CPU_INTERRUPT_HARD);
}

static void bfin_cpu_disas_set_info(const CPUState *cpu,
                                    disassemble_info *info)
{
    info->mach = bfd_arch_bfin;
    info->endian = BFD_ENDIAN_LITTLE;
}

static void bfin_cpu_reset_hold(Object *obj, ResetType type)
{
    CPUState *cs = CPU(obj);
    BfinCPUClass *bcc = BFIN_CPU_GET_CLASS(obj);
    CPUBfinState *env = cpu_env(cs);

    if (bcc->parent_phases.hold) {
        bcc->parent_phases.hold(obj, type);
    }

    memset(env->reg, 0, sizeof(env->reg));
    memset(env->astat, 0, sizeof(env->astat));
    memset(env->evt, 0, sizeof(env->evt));
    env->imask = 0x1f;
    env->ilat = 0;
    env->irq_level = 0;
    env->ksp = 0;
    /* coming out of reset the core services the reset event */
    env->ipend = 1u << 1;
    env->cycles_base = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    env->cycles2_shadow = 0;
    env->pc = env->evt[1];
}

/* the 16 IVG inputs; the SIC drives IVG7..15 as levels */
static void bfin_cpu_set_irq(void *opaque, int irq, int level)
{
    BfinCPU *cpu = opaque;
    CPUBfinState *env = &cpu->env;

    if (level) {
        env->irq_level |= 1u << irq;
    } else {
        env->irq_level &= ~(1u << irq);
    }
    bfin_update_irq(env);
}

static void bfin_cpu_initfn(Object *obj)
{
    qdev_init_gpio_in(DEVICE(obj), bfin_cpu_set_irq, 16);
}

static void bfin_cpu_realizefn(DeviceState *dev, Error **errp)
{
    CPUState *cs = CPU(dev);
    BfinCPUClass *bcc = BFIN_CPU_GET_CLASS(dev);
    Error *local_err = NULL;

    cpu_common_realize(cs, &local_err);
    if (local_err != NULL) {
        error_propagate(errp, local_err);
        return;
    }
    qemu_init_vcpu(cs);
    cpu_reset(cs);
    bcc->parent_realize(dev, errp);
}

static const Property bfin_cpu_properties[] = {
    DEFINE_PROP_UINT32("clock-frequency", BfinCPU, env.clock_freq,
                       500000000),
};

static void bfin_cpu_dump_state(CPUState *cs, FILE *f, int flags)
{
    CPUBfinState *env = cpu_env(cs);
    static const char *const names[16] = {
        "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7",
        "p0", "p1", "p2", "p3", "p4", "p5", "sp", "fp",
    };

    qemu_fprintf(f, "pc=%08x astat=%08x ipend=%04x imask=%04x "
                 "seqstat=%08x\n", env->pc, bfin_get_astat(env),
                 env->ipend, env->imask, env->reg[BFIN_SEQSTAT]);
    for (int i = 0; i < 16; i++) {
        qemu_fprintf(f, "%-2s=%08x%c", names[i], env->reg[i],
                     (i & 7) == 7 ? '\n' : ' ');
    }
    qemu_fprintf(f, "a0=%02x%08x a1=%02x%08x rets=%08x usp=%08x\n",
                 env->reg[BFIN_A0X] & 0xff, env->reg[BFIN_A0W],
                 env->reg[BFIN_A1X] & 0xff, env->reg[BFIN_A1W],
                 env->reg[BFIN_RETS], env->reg[BFIN_USP]);
}

static ObjectClass *bfin_cpu_class_by_name(const char *cpu_model)
{
    g_autofree char *typename = g_strdup_printf(BFIN_CPU_TYPE_NAME("%s"),
                                                cpu_model);

    return object_class_by_name(typename);
}

static const struct SysemuCPUOps bfin_sysemu_ops = {
    .has_work = bfin_cpu_has_work,
    .get_phys_addr_debug = bfin_cpu_get_phys_page_debug,
};

static const TCGCPUOps bfin_tcg_ops = {
    .guest_default_memory_order = TCG_MO_ALL,
    .mttcg_supported = false,

    .initialize = bfin_tcg_init,
    .translate_code = bfin_translate_code,
    .get_tb_cpu_state = bfin_get_tb_cpu_state,
    .restore_state_to_opc = bfin_restore_state_to_opc,
    .mmu_index = bfin_cpu_mmu_index,
    .pointer_wrap = cpu_pointer_wrap_uint32,

    .tlb_fill = bfin_cpu_tlb_fill,
    .do_unaligned_access = bfin_cpu_do_unaligned_access,
    .cpu_exec_interrupt = bfin_cpu_exec_interrupt,
    .cpu_exec_halt = bfin_cpu_has_work,
    .cpu_exec_reset = cpu_reset,
    .do_interrupt = bfin_cpu_do_interrupt,
};

static void bfin_cpu_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    CPUClass *cc = CPU_CLASS(oc);
    BfinCPUClass *bcc = BFIN_CPU_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    device_class_set_parent_realize(dc, bfin_cpu_realizefn,
                                    &bcc->parent_realize);
    device_class_set_props(dc, bfin_cpu_properties);
    resettable_class_set_parent_phases(rc, NULL, bfin_cpu_reset_hold, NULL,
                                       &bcc->parent_phases);

    cc->class_by_name = bfin_cpu_class_by_name;
    cc->dump_state = bfin_cpu_dump_state;
    cc->set_pc = bfin_cpu_set_pc;
    cc->get_pc = bfin_cpu_get_pc;
    cc->disas_set_info = bfin_cpu_disas_set_info;
    cc->sysemu_ops = &bfin_sysemu_ops;
    cc->tcg_ops = &bfin_tcg_ops;
}

static const TypeInfo bfin_cpu_type_infos[] = {
    {
        .name = TYPE_BFIN_CPU,
        .parent = TYPE_CPU,
        .instance_size = sizeof(BfinCPU),
        .instance_align = __alignof(BfinCPU),
        .instance_init = bfin_cpu_initfn,
        .abstract = true,
        .class_size = sizeof(BfinCPUClass),
        .class_init = bfin_cpu_class_init,
    },
    {
        .name = BFIN_CPU_TYPE_NAME("bf537"),
        .parent = TYPE_BFIN_CPU,
    },
};

DEFINE_TYPES(bfin_cpu_type_infos)
