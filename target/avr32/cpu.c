/*
 * QEMU AVR32 CPU
 *
 * Copyright (c) 2022-2023 Florian Göhler, Johannes Willbold
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see
 * <http://www.gnu.org/licenses/lgpl-2.1.html>
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

/*
 * AP7000: AVR32B architecture revision 1, shared 32-entry TLB, Java,
 * performance counters, OCD, SIMD and DSP; 16 KiB 4-way caches with
 * 32-byte lines.
 */
#define AP7000_CONFIG0  ((1u << 24) | (1u << 13) | (1u << 10) | (2u << 7) | \
                         (1u << 5) | (1u << 4) | (1u << 3) | (1u << 2) | \
                         (1u << 1))
#define AP7000_CONFIG1  (((AVR32_TLB_SIZE - 1u) << 20) | \
                         (7u << 16) | (4u << 13) | (2u << 10) | \
                         (7u << 6) | (4u << 3) | 2u)

static void avr32_cpu_set_pc(CPUState *cs, vaddr value)
{
    cpu_env(cs)->r[AVR32_PC_REG] = value;
}

static vaddr avr32_cpu_get_pc(CPUState *cs)
{
    return cpu_env(cs)->r[AVR32_PC_REG];
}

static void avr32_restore_state_to_opc(CPUState *cs,
                                       const TranslationBlock *tb,
                                       const uint64_t *data)
{
    cpu_env(cs)->r[AVR32_PC_REG] = data[0];
}

static TCGTBCPUState avr32_get_tb_cpu_state(CPUState *cs)
{
    CPUAVR32State *env = cpu_env(cs);

    return (TCGTBCPUState){
        .pc = env->r[AVR32_PC_REG],
        .flags = avr32_mode(env) != AVR32_MODE_APP ? AVR32_TBFLAG_PRIV : 0,
    };
}

static int avr32_cpu_mmu_index(CPUState *cs, bool ifetch)
{
    return avr32_mode(cpu_env(cs)) != AVR32_MODE_APP ? MMU_KERNEL_IDX
                                                     : MMU_USER_IDX;
}

static bool avr32_cpu_has_work(CPUState *cs)
{
    return cpu_test_interrupt(cs, CPU_INTERRUPT_HARD);
}

static void avr32_cpu_disas_set_info(const CPUState *cpu,
                                     disassemble_info *info)
{
    info->mach = bfd_arch_avr32;
    info->endian = BFD_ENDIAN_BIG;
}

static void avr32_cpu_reset_hold(Object *obj, ResetType type)
{
    CPUState *cs = CPU(obj);
    AVR32CPU *cpu = AVR32_CPU(cs);
    AVR32CPUClass *acc = AVR32_CPU_GET_CLASS(obj);
    CPUAVR32State *env = &cpu->env;

    if (acc->parent_phases.hold) {
        acc->parent_phases.hold(obj, type);
    }

    memset(env->r, 0, sizeof(env->r));
    memset(env->sflags, 0, sizeof(env->sflags));
    memset(env->sysr, 0, sizeof(env->sysr));
    memset(env->int3_bank, 0, sizeof(env->int3_bank));
    memset(env->tlb, 0, sizeof(env->tlb));
    env->sp_app = env->sp_sys = 0;

    /* Supervisor mode, exceptions and interrupts masked */
    env->sflags[SR_GM] = 1;
    env->sflags[SR_EM] = 1;
    env->sflags[SR_M0] = 1;

    env->sysr[SYSREG_CONFIG0] = AP7000_CONFIG0;
    env->sysr[SYSREG_CONFIG1] = AP7000_CONFIG1;
    env->sysr[SYSREG_MMUCR] = MMUCR_S;
    env->sysr[SYSREG_TLBARLO] = 0xffffffff;

    env->count_base = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    env->compare_pending = false;
    if (env->compare_timer) {
        timer_del(env->compare_timer);
    }
    env->irq_pending = 0;
    env->nmi_pending = false;

    env->r[AVR32_PC_REG] = 0xa0000000;
}

static void avr32_cpu_realizefn(DeviceState *dev, Error **errp)
{
    CPUState *cs = CPU(dev);
    AVR32CPU *cpu = AVR32_CPU(cs);
    AVR32CPUClass *acc = AVR32_CPU_GET_CLASS(dev);
    Error *local_err = NULL;

    cpu_common_realize(cs, &local_err);
    if (local_err != NULL) {
        error_propagate(errp, local_err);
        return;
    }

    cpu->env.compare_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                          avr32_compare_timer_cb, cpu);
    qdev_init_gpio_out_named(dev, &cpu->env.compare_irq, "compare", 1);

    qemu_init_vcpu(cs);
    cpu_reset(cs);

    acc->parent_realize(dev, errp);
}

static const Property avr32_cpu_properties[] = {
    DEFINE_PROP_UINT32("clock-frequency", AVR32CPU, env.count_freq,
                       140000000),
};

static void avr32_cpu_dump_state(CPUState *cs, FILE *f, int flags)
{
    CPUAVR32State *env = cpu_env(cs);
    static const char *const names[AVR32_NREGS] = {
        "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7",
        "r8", "r9", "r10", "r11", "r12", "sp", "lr", "pc",
    };

    for (int i = 0; i < AVR32_NREGS; i++) {
        qemu_fprintf(f, "%-3s=%08x%c", names[i], env->r[i],
                     (i & 3) == 3 ? '\n' : ' ');
    }
    qemu_fprintf(f, "sr =%08x evba=%08x ecr=%08x\n", avr32_get_sr(env),
                 env->sysr[SYSREG_EVBA], env->sysr[SYSREG_ECR]);
}

static ObjectClass *avr32_cpu_class_by_name(const char *cpu_model)
{
    g_autofree char *typename = g_strdup_printf(AVR32_CPU_TYPE_NAME("%s"),
                                                cpu_model);

    return object_class_by_name(typename);
}

static const struct SysemuCPUOps avr32_sysemu_ops = {
    .has_work = avr32_cpu_has_work,
    .get_phys_addr_debug = avr32_cpu_get_phys_page_debug,
};

static const TCGCPUOps avr32_tcg_ops = {
    /* MTTCG not yet supported: require strict ordering */
    .guest_default_memory_order = TCG_MO_ALL,
    .mttcg_supported = false,

    .initialize = avr32_tcg_init,
    .translate_code = avr32_translate_code,
    .get_tb_cpu_state = avr32_get_tb_cpu_state,
    .restore_state_to_opc = avr32_restore_state_to_opc,
    .mmu_index = avr32_cpu_mmu_index,
    .pointer_wrap = cpu_pointer_wrap_uint32,

    .tlb_fill = avr32_cpu_tlb_fill,
    .cpu_exec_interrupt = avr32_cpu_exec_interrupt,
    .cpu_exec_halt = avr32_cpu_has_work,
    .cpu_exec_reset = cpu_reset,
    .do_interrupt = avr32_cpu_do_interrupt,
    .do_unaligned_access = avr32_cpu_do_unaligned_access,
};

static void avr32_cpu_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    CPUClass *cc = CPU_CLASS(oc);
    AVR32CPUClass *acc = AVR32_CPU_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    device_class_set_parent_realize(dc, avr32_cpu_realizefn,
                                    &acc->parent_realize);
    device_class_set_props(dc, avr32_cpu_properties);
    resettable_class_set_parent_phases(rc, NULL, avr32_cpu_reset_hold, NULL,
                                       &acc->parent_phases);

    cc->class_by_name = avr32_cpu_class_by_name;
    cc->dump_state = avr32_cpu_dump_state;
    cc->set_pc = avr32_cpu_set_pc;
    cc->get_pc = avr32_cpu_get_pc;
    cc->disas_set_info = avr32_cpu_disas_set_info;
    cc->sysemu_ops = &avr32_sysemu_ops;
    cc->gdb_read_register = avr32_cpu_gdb_read_register;
    cc->gdb_write_register = avr32_cpu_gdb_write_register;
    cc->gdb_num_core_regs = 17;
    cc->tcg_ops = &avr32_tcg_ops;
}

static const TypeInfo avr32_cpu_type_infos[] = {
    {
        .name = TYPE_AVR32_CPU,
        .parent = TYPE_CPU,
        .instance_size = sizeof(AVR32CPU),
        .instance_align = __alignof(AVR32CPU),
        .abstract = true,
        .class_size = sizeof(AVR32CPUClass),
        .class_init = avr32_cpu_class_init,
    },
    {
        .name = AVR32_CPU_TYPE_NAME("ap7000"),
        .parent = TYPE_AVR32_CPU,
    },
};

DEFINE_TYPES(avr32_cpu_type_infos)
