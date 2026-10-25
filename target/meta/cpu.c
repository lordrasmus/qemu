/*
 *  META virtual CPU
 *
 *  Copyright (c) 2011 Imagination Technologies
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

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/main-loop.h"
#include "cpu.h"
#include "exec/translation-block.h"
#include "accel/tcg/cpu-ops.h"
#include "hw/core/sysemu-cpu-ops.h"

const char *meta_freason_names[4] = {
    [META_FREASON_NOERROR] = "no error",
    [META_FREASON_GENERAL] = "general violation",
    [META_FREASON_PAGE]    = "page fault",
    [META_FREASON_PROTECT] = "protection violation",
};

const char *meta_hreason_names[4] = {
    [META_HREASON_SWITCH]  = "SWITCH",
    [META_HREASON_UNKNOWN] = "unknown instruction",
    [META_HREASON_PRIV]    = "privilege violation",
    [META_HREASON_FAULT]   = "memory fault",
};

void meta_interrupt(CPUMETAState *env, int mask)
{
    if (bql_locked()) {
        cpu_interrupt(env_cpu(env), mask);
    } else {
        bql_lock();
        cpu_interrupt(env_cpu(env), mask);
        bql_unlock();
    }
}

void meta_reset_interrupt(CPUMETAState *env, int mask)
{
    if (bql_locked()) {
        cpu_reset_interrupt(env_cpu(env), mask);
    } else {
        bql_lock();
        cpu_reset_interrupt(env_cpu(env), mask);
        bql_unlock();
    }
}

static void meta_cpu_set_pc(CPUState *cs, vaddr value)
{
    CPUMETAState *env = cpu_env(cs);

    env->pc[META_PC] = meta_virt_to_pc(env, value);
}

static vaddr meta_cpu_get_pc(CPUState *cs)
{
    CPUMETAState *env = cpu_env(cs);

    return meta_pc_to_virt(env, env->pc[META_PC]);
}

static bool meta_cpu_has_work(CPUState *cs)
{
    return cpu_test_interrupt(cs, CPU_INTERRUPT_HARD | CPU_INTERRUPT_WAKE);
}

/* The flags are 64 bits; the upper half travels in cs_base. */
static TCGTBCPUState meta_get_tb_cpu_state(CPUState *cs)
{
    target_ulong pc;
    uint64_t flags = meta_tb_flags(cpu_env(cs), &pc);

    return (TCGTBCPUState){
        .pc = pc,
        .flags = (uint32_t)flags,
        .cs_base = flags >> 32,
    };
}

static int meta_cpu_mmu_index(CPUState *cs, bool ifetch)
{
    return meta_mmu_index(cpu_env(cs));
}

static void meta_cpu_reset_hold(Object *obj, ResetType type)
{
    METACPUClass *mcc = META_CPU_GET_CLASS(obj);

    if (mcc->parent_phases.hold) {
        mcc->parent_phases.hold(obj, type);
    }
    cpu_state_reset(&META_CPU(obj)->env);
}

static void meta_cpu_realizefn(DeviceState *dev, Error **errp)
{
    CPUState *cs = CPU(dev);
    METACPUClass *mcc = META_CPU_GET_CLASS(dev);
    Error *local_err = NULL;

    cpu_common_realize(cs, &local_err);
    if (local_err != NULL) {
        error_propagate(errp, local_err);
        return;
    }

    qemu_init_vcpu(cs);
    cpu_reset(cs);

    mcc->parent_realize(dev, errp);
}

static bool meta_cpu_exec_halt(CPUState *cs)
{
    return meta_cpu_has_work(cs);
}

static const struct SysemuCPUOps meta_sysemu_ops = {
    .has_work = meta_cpu_has_work,
    .get_phys_addr_debug = meta_cpu_get_phys_page_debug,
};

static const TCGCPUOps meta_tcg_ops = {
    .guest_default_memory_order = TCG_MO_ALL,
    .mttcg_supported = false,

    .initialize = meta_tcg_init,
    .translate_code = meta_translate_code,
    .get_tb_cpu_state = meta_get_tb_cpu_state,
    .restore_state_to_opc = meta_restore_state_to_opc,
    .mmu_index = meta_cpu_mmu_index,
    .pointer_wrap = cpu_pointer_wrap_uint32,
    .tlb_fill = meta_cpu_tlb_fill,
    .cpu_exec_interrupt = meta_cpu_exec_interrupt,
    .cpu_exec_halt = meta_cpu_exec_halt,
    .cpu_exec_reset = cpu_reset,
    .do_interrupt = meta_cpu_do_interrupt,
    .do_unaligned_access = meta_cpu_do_unaligned_access,
};

static void meta_cpu_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    CPUClass *cc = CPU_CLASS(oc);
    METACPUClass *mcc = META_CPU_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    device_class_set_parent_realize(dc, meta_cpu_realizefn,
                                    &mcc->parent_realize);
    resettable_class_set_parent_phases(rc, NULL, meta_cpu_reset_hold, NULL,
                                       &mcc->parent_phases);
    /* a core's threads are created by the SoC, not by -cpu */
    dc->user_creatable = false;

    cc->dump_state = meta_cpu_dump_state;
    cc->set_pc = meta_cpu_set_pc;
    cc->get_pc = meta_cpu_get_pc;
    cc->sysemu_ops = &meta_sysemu_ops;
    cc->tcg_ops = &meta_tcg_ops;
}

static const TypeInfo meta_cpu_type_info = {
    .name = TYPE_META_CPU,
    .parent = TYPE_CPU,
    .instance_size = sizeof(METACPU),
    .instance_align = __alignof(METACPU),
    .class_size = sizeof(METACPUClass),
    .class_init = meta_cpu_class_init,
};

static void meta_cpu_register_types(void)
{
    type_register_static(&meta_cpu_type_info);
}

type_init(meta_cpu_register_types)
