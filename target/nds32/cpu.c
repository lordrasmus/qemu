/*
 * QEMU NDS32 CPU
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
#include "hw/core/qdev-properties.h"

static void nds32_cpu_set_pc(CPUState *cs, vaddr value)
{
    cpu_env(cs)->pc = value;
}

static vaddr nds32_cpu_get_pc(CPUState *cs)
{
    return cpu_env(cs)->pc;
}

static void nds32_restore_state_to_opc(CPUState *cs,
                                       const TranslationBlock *tb,
                                       const uint64_t *data)
{
    cpu_env(cs)->pc = data[0];
}

static bool nds32_cpu_has_work(CPUState *cs)
{
    return cpu_test_interrupt(cs, CPU_INTERRUPT_HARD);
}

static TCGTBCPUState nds32_get_tb_cpu_state(CPUState *cs)
{
    CPUNDS32State *env = cpu_env(cs);
    uint32_t psw = env->sr[SR_PSW];
    uint32_t flags = 0;

    flags = FIELD_DP32(flags, TBFLAGS, USER, nds32_user_mode(env));
    flags = FIELD_DP32(flags, TBFLAGS, IT, FIELD_EX32(psw, PSW, IT));
    flags = FIELD_DP32(flags, TBFLAGS, DT, FIELD_EX32(psw, PSW, DT));
    flags = FIELD_DP32(flags, TBFLAGS, UNA,
                       FIELD_EX32(env->sr[SR_MMU_CTL], MMU_CTL, UNA));
    flags = FIELD_DP32(flags, TBFLAGS, BE, FIELD_EX32(psw, PSW, BE));
    return (TCGTBCPUState){ .pc = env->pc, .flags = flags };
}

static int nds32_cpu_mmu_index(CPUState *cs, bool ifetch)
{
    CPUNDS32State *env = cpu_env(cs);
    uint32_t psw = env->sr[SR_PSW];
    int idx = nds32_user_mode(env) ? MMU_IDX_USER : 0;

    if (ifetch ? FIELD_EX32(psw, PSW, IT) : FIELD_EX32(psw, PSW, DT)) {
        idx |= MMU_IDX_XLATE;
    }
    return idx;
}

static void nds32_cpu_reset_hold(Object *obj, ResetType type)
{
    CPUState *cs = CPU(obj);
    NDS32CPU *cpu = NDS32_CPU(cs);
    NDS32CPUClass *ncc = NDS32_CPU_GET_CLASS(obj);
    CPUNDS32State *env = &cpu->env;

    if (ncc->parent_phases.hold) {
        ncc->parent_phases.hold(obj, type);
    }

    memset(env->gpr, 0, sizeof(env->gpr));
    memset(env->d, 0, sizeof(env->d));
    memset(env->sr, 0, sizeof(env->sr));
    memset(env->tlb, 0, sizeof(env->tlb));
    env->tlb_rr = 0;
    env->lock_addr = -1;
    env->pc = 0;

    /* N13, rev 12, with the performance extension and 16-bit insns. */
    env->sr[SR_CPU_VER] = 0x0d0c0003;
    /* 256 sets, 4 ways, 32-byte lines for both caches. */
    env->sr[SR_ICM_CFG] = (2 << 0) | (3 << 3) | (3 << 6);
    env->sr[SR_DCM_CFG] = (2 << 0) | (3 << 3) | (3 << 6);
    /*
     * MMU, 8 ways x 512 sets (enough for head.S to map a 16 MB kernel),
     * TLB locking, hardware page table walker, little endian,
     * non-translated VA->PA mapping.
     */
    env->sr[SR_MMU_CFG] = (2 << 0) | (7 << 8) | (7 << 11) | (1 << 24) |
                          (1 << 25) | (1 << 30);
    /* Baseline V3. */
    env->sr[SR_MSC_CFG] = 2 << 13;
    /* PSW: superuser, interruption level 0, translation off. */
    env->sr[SR_PSW] = FIELD_DP32(0, PSW, POM, 1);
    /* IVIC version 1 with 32 inputs. */
    env->sr[SR_IVB] = (5 << 1) | (1 << 11);
    /* NTM: identity mapping of the four partitions. */
    env->sr[SR_MMU_CTL] = (0 << 11) | (1 << 13) | (2 << 15) | (3 << 17);

    cs->exception_index = -1;
}

#ifndef CONFIG_USER_ONLY
static void nds32_cpu_set_irq(void *opaque, int irq, int level)
{
    NDS32CPU *cpu = opaque;
    CPUNDS32State *env = &cpu->env;

    env->irq_lines = deposit32(env->irq_lines, irq, 1, !!level);
    nds32_update_irq(env);
}
#endif

static ObjectClass *nds32_cpu_class_by_name(const char *cpu_model)
{
    return object_class_by_name(TYPE_NDS32_CPU);
}

static void nds32_cpu_realizefn(DeviceState *dev, Error **errp)
{
    CPUState *cs = CPU(dev);
    NDS32CPUClass *ncc = NDS32_CPU_GET_CLASS(dev);
    Error *local_err = NULL;

    cpu_common_realize(cs, &local_err);
    if (local_err != NULL) {
        error_propagate(errp, local_err);
        return;
    }

    qemu_init_vcpu(cs);
    cpu_reset(cs);

#ifndef CONFIG_USER_ONLY
    qdev_init_gpio_in_named(DEVICE(cs), nds32_cpu_set_irq, "IRQ", 32);
#endif

    ncc->parent_realize(dev, errp);
}

#ifndef CONFIG_USER_ONLY
static bool nds32_cpu_exec_interrupt(CPUState *cs, int interrupt_request)
{
    CPUNDS32State *env = cpu_env(cs);

    if ((interrupt_request & CPU_INTERRUPT_HARD) &&
        FIELD_EX32(env->sr[SR_PSW], PSW, GIE) &&
        (env->irq_lines & env->sr[SR_INT_MASK2])) {
        cs->exception_index = EXCP_IRQ;
        nds32_cpu_do_interrupt(cs);
        return true;
    }
    return false;
}
#endif

static void nds32_cpu_disas_set_info(const CPUState *cpu,
                                     disassemble_info *info)
{
}

static int nds32_cpu_gdb_read_register(CPUState *cs, GByteArray *mem_buf,
                                       int n)
{
    CPUNDS32State *env = cpu_env(cs);

    if (n < 32) {
        return gdb_get_reg32(mem_buf, env->gpr[n]);
    } else if (n == 32) {
        return gdb_get_reg32(mem_buf, env->pc);
    }
    return 0;
}

static int nds32_cpu_gdb_write_register(CPUState *cs, uint8_t *mem_buf, int n)
{
    CPUNDS32State *env = cpu_env(cs);
    uint32_t val = ldl_p(mem_buf);

    if (n < 32) {
        env->gpr[n] = val;
    } else if (n == 32) {
        env->pc = val;
    } else {
        return 0;
    }
    return 4;
}

#ifndef CONFIG_USER_ONLY
#include "hw/core/sysemu-cpu-ops.h"

static const struct SysemuCPUOps nds32_sysemu_ops = {
    .has_work = nds32_cpu_has_work,
    .get_phys_addr_debug = nds32_cpu_get_phys_page_debug,
};
#endif

static const TCGCPUOps nds32_tcg_ops = {
    .guest_default_memory_order = TCG_MO_ALL,
    .mttcg_supported = false,

    .initialize = nds32_tcg_init,
    .translate_code = nds32_translate_code,
    .get_tb_cpu_state = nds32_get_tb_cpu_state,
    .restore_state_to_opc = nds32_restore_state_to_opc,
    .mmu_index = nds32_cpu_mmu_index,
    .pointer_wrap = cpu_pointer_wrap_uint32,

#ifndef CONFIG_USER_ONLY
    .tlb_fill = nds32_cpu_tlb_fill,
    .cpu_exec_interrupt = nds32_cpu_exec_interrupt,
    .cpu_exec_halt = nds32_cpu_has_work,
    .cpu_exec_reset = cpu_reset,
    .do_interrupt = nds32_cpu_do_interrupt,
    .do_unaligned_access = nds32_cpu_do_unaligned_access,
#endif
};

static void nds32_cpu_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    CPUClass *cc = CPU_CLASS(oc);
    NDS32CPUClass *ncc = NDS32_CPU_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    device_class_set_parent_realize(dc, nds32_cpu_realizefn,
                                    &ncc->parent_realize);
    resettable_class_set_parent_phases(rc, NULL, nds32_cpu_reset_hold, NULL,
                                       &ncc->parent_phases);

    cc->class_by_name = nds32_cpu_class_by_name;
    cc->dump_state = nds32_cpu_dump_state;
    cc->set_pc = nds32_cpu_set_pc;
    cc->get_pc = nds32_cpu_get_pc;
    cc->disas_set_info = nds32_cpu_disas_set_info;
#ifndef CONFIG_USER_ONLY
    cc->sysemu_ops = &nds32_sysemu_ops;
#endif
    cc->gdb_read_register = nds32_cpu_gdb_read_register;
    cc->gdb_write_register = nds32_cpu_gdb_write_register;
    cc->gdb_num_core_regs = 33;
    cc->tcg_ops = &nds32_tcg_ops;
}

static const TypeInfo nds32_cpu_type_info = {
    .name = TYPE_NDS32_CPU,
    .parent = TYPE_CPU,
    .instance_size = sizeof(NDS32CPU),
    .instance_align = __alignof(NDS32CPU),
    .class_size = sizeof(NDS32CPUClass),
    .class_init = nds32_cpu_class_init,
};

static void nds32_cpu_register_types(void)
{
    type_register_static(&nds32_cpu_type_info);
}

type_init(nds32_cpu_register_types)
