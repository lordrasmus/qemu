/*
 * QEMU TILE-Gx CPU
 *
 *  Copyright (c) 2015 Chen Gang
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
#include "qapi/error.h"
#include "qemu/qemu-print.h"
#include "qemu/timer.h"
#include "cpu.h"
#include "exec/translation-block.h"
#include "accel/tcg/cpu-ops.h"
#include "migration/vmstate.h"

static void tilegx_cpu_set_pc(CPUState *cs, vaddr value)
{
    cpu_env(cs)->pc = value;
}

static vaddr tilegx_cpu_get_pc(CPUState *cs)
{
    return cpu_env(cs)->pc;
}

static void tilegx_restore_state_to_opc(CPUState *cs,
                                        const TranslationBlock *tb,
                                        const uint64_t *data)
{
    cpu_env(cs)->pc = data[0];
}

static TCGTBCPUState tilegx_get_tb_cpu_state(CPUState *cs)
{
    CPUTLGState *env = cpu_env(cs);

    return (TCGTBCPUState){
        .pc = env->pc,
        .flags = env->cpl,
    };
}

static int tilegx_cpu_mmu_index(CPUState *cs, bool ifetch)
{
    return cpu_env(cs)->cpl ? MMU_KERNEL_IDX : MMU_USER_IDX;
}

static bool tilegx_cpu_has_work(CPUState *cs)
{
    return cpu_test_interrupt(cs, CPU_INTERRUPT_HARD);
}

static void tilegx_cpu_reset_hold(Object *obj, ResetType type)
{
    CPUState *cs = CPU(obj);
    TileGXCPUClass *tcc = TILEGX_CPU_GET_CLASS(obj);
    CPUTLGState *env = cpu_env(cs);
    int i;

    if (tcc->parent_phases.hold) {
        tcc->parent_phases.hold(obj, type);
    }

    memset(env, 0, offsetof(CPUTLGState, end_reset_fields));

    /*
     * The hypervisor starts the client at the kernel's protection level
     * with every interrupt masked, and owns the ones it does not hand
     * down; the client sets its own masks and MPLs.
     */
    env->cpl = 2;
    env->ics = 0;
    for (i = 0; i < TILEGX_SPR_DOMAINS; i++) {
        env->mpl[i] = 2;
    }
    env->mpl[INT_SWINT_3] = 3;
    env->mpl[INT_SWINT_2] = 3;
    env->mpl[INT_INTCTRL_3] = 3;
    for (i = 0; i < 4; i++) {
        env->int_mask[i] = ~0ULL;
    }
    env->log2_page_small = 16;
    env->log2_page_large = 24;
}

static ObjectClass *tilegx_cpu_class_by_name(const char *cpu_model)
{
    return object_class_by_name(TYPE_TILEGX_CPU);
}

static void tilegx_cpu_realizefn(DeviceState *dev, Error **errp)
{
    CPUState *cs = CPU(dev);
    TileGXCPUClass *tcc = TILEGX_CPU_GET_CLASS(dev);
    Error *local_err = NULL;

    cpu_common_realize(cs, &local_err);
    if (local_err != NULL) {
        error_propagate(errp, local_err);
        return;
    }

#ifndef CONFIG_USER_ONLY
    cpu_env(cs)->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, tilegx_timer_cb,
                                      TILEGX_CPU(cs));
#endif
    cpu_reset(cs);
    qemu_init_vcpu(cs);

    tcc->parent_realize(dev, errp);
}

static void tilegx_cpu_initfn(Object *obj)
{
    CPUTLGState *env = cpu_env(CPU(obj));

    env->cpu_speed = 1000000000;
}

void tilegx_cpu_dump_state(CPUState *cs, FILE *f, int flags)
{
    static const char * const reg_names[TILEGX_R_COUNT] = {
         "r0",  "r1",  "r2",  "r3",  "r4",  "r5",  "r6",  "r7",
         "r8",  "r9", "r10", "r11", "r12", "r13", "r14", "r15",
        "r16", "r17", "r18", "r19", "r20", "r21", "r22", "r23",
        "r24", "r25", "r26", "r27", "r28", "r29", "r30", "r31",
        "r32", "r33", "r34", "r35", "r36", "r37", "r38", "r39",
        "r40", "r41", "r42", "r43", "r44", "r45", "r46", "r47",
        "r48", "r49", "r50", "r51",  "bp",  "tp",  "sp",  "lr"
    };
    CPUTLGState *env = cpu_env(cs);
    int i;

    for (i = 0; i < TILEGX_R_COUNT; i++) {
        qemu_fprintf(f, "%-3s %016" PRIx64 "%s", reg_names[i], env->regs[i],
                     (i & 3) == 3 ? "\n" : " ");
    }
    qemu_fprintf(f, "pc  %016" PRIx64 " pl %u ics %u pending %" PRIx64 "\n",
                 env->pc, env->cpl, env->ics, env->pending);
    for (i = 0; i < 4; i++) {
        qemu_fprintf(f, "pl%d ex %016" PRIx64 " %" PRIx64 " mask %016" PRIx64
                     " base %016" PRIx64 "\n", i, env->ex_context[i][0],
                     env->ex_context[i][1], env->int_mask[i],
                     env->vector_base[i]);
    }
}

#ifndef CONFIG_USER_ONLY
#include "hw/core/sysemu-cpu-ops.h"

static const VMStateDescription vmstate_tilegx_cpu = {
    .name = "cpu",
    .unmigratable = 1,
};

static const struct SysemuCPUOps tilegx_sysemu_ops = {
    .has_work = tilegx_cpu_has_work,
    .get_phys_addr_debug = tilegx_cpu_get_phys_addr_debug,
};
#endif

static const TCGCPUOps tilegx_tcg_ops = {
    /* MTTCG not yet supported: require strict ordering */
    .guest_default_memory_order = TCG_MO_ALL,
    .mttcg_supported = false,

    .initialize = tilegx_translate_init,
    .translate_code = tilegx_translate_code,
    .get_tb_cpu_state = tilegx_get_tb_cpu_state,
    .restore_state_to_opc = tilegx_restore_state_to_opc,
    .mmu_index = tilegx_cpu_mmu_index,
    .pointer_wrap = cpu_pointer_wrap_notreached,

#ifndef CONFIG_USER_ONLY
    .tlb_fill = tilegx_cpu_tlb_fill,
    .cpu_exec_interrupt = tilegx_cpu_exec_interrupt,
    .cpu_exec_halt = tilegx_cpu_has_work,
    .cpu_exec_reset = cpu_reset,
    .do_interrupt = tilegx_cpu_do_interrupt,
    .do_unaligned_access = tilegx_cpu_do_unaligned_access,
#endif
};

static void tilegx_cpu_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    CPUClass *cc = CPU_CLASS(oc);
    TileGXCPUClass *tcc = TILEGX_CPU_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    device_class_set_parent_realize(dc, tilegx_cpu_realizefn,
                                    &tcc->parent_realize);
    resettable_class_set_parent_phases(rc, NULL, tilegx_cpu_reset_hold, NULL,
                                       &tcc->parent_phases);

    cc->class_by_name = tilegx_cpu_class_by_name;
    cc->dump_state = tilegx_cpu_dump_state;
    cc->set_pc = tilegx_cpu_set_pc;
    cc->get_pc = tilegx_cpu_get_pc;
#ifndef CONFIG_USER_ONLY
    dc->vmsd = &vmstate_tilegx_cpu;
    cc->sysemu_ops = &tilegx_sysemu_ops;
#endif
    cc->tcg_ops = &tilegx_tcg_ops;
}

static const TypeInfo tilegx_cpu_type_info = {
    .name = TYPE_TILEGX_CPU,
    .parent = TYPE_CPU,
    .instance_size = sizeof(TileGXCPU),
    .instance_align = __alignof(TileGXCPU),
    .instance_init = tilegx_cpu_initfn,
    .class_size = sizeof(TileGXCPUClass),
    .class_init = tilegx_cpu_class_init,
};

static void tilegx_cpu_register_types(void)
{
    type_register_static(&tilegx_cpu_type_info);
}

type_init(tilegx_cpu_register_types)
