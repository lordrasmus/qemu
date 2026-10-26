/*
 * QEMU IA-64 CPU
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "cpu.h"
#include "exec/translation-block.h"
#include "exec/cputlb.h"
#include "accel/tcg/cpu-ops.h"
#include "exec/log.h"
#include "gdbstub/helpers.h"
#include "disas/dis-asm.h"
#include "ia64-disasm-int.h"
#include "ia64-disasm.h"

/* Program counters are bundle | slot. */
static void ia64_cpu_set_pc(CPUState *cs, vaddr value)
{
    CPUIA64State *env = cpu_env(cs);

    env->ip = value & ~0xfull;
    env->ri = value & 3;
}

static vaddr ia64_cpu_get_pc(CPUState *cs)
{
    CPUIA64State *env = cpu_env(cs);

    return env->ip | env->ri;
}

static void ia64_restore_state_to_opc(CPUState *cs,
                                      const TranslationBlock *tb,
                                      const uint64_t *data)
{
    CPUIA64State *env = cpu_env(cs);

    env->ip = data[0] & ~0xfull;
    env->ri = data[0] & 3;
}

static bool ia64_cpu_has_work(CPUState *cs)
{
    return cpu_test_interrupt(cs, CPU_INTERRUPT_HARD);
}

static TCGTBCPUState ia64_get_tb_cpu_state(CPUState *cs)
{
    CPUIA64State *env = cpu_env(cs);
    uint64_t psr = env->psr;
    uint32_t flags = 0;

    flags = FIELD_DP32(flags, TBFLAGS, CPL, ia64_cpl(env));
    flags = FIELD_DP32(flags, TBFLAGS, DT, !!(psr & PSR_DT));
    flags = FIELD_DP32(flags, TBFLAGS, BE, !!(psr & PSR_BE));
    flags = FIELD_DP32(flags, TBFLAGS, DFL, !!(psr & PSR_DFL));
    flags = FIELD_DP32(flags, TBFLAGS, DFH, !!(psr & PSR_DFH));
    flags = FIELD_DP32(flags, TBFLAGS, IC, !!(psr & PSR_IC));
    flags = FIELD_DP32(flags, TBFLAGS, BN, !!(psr & PSR_BN));
    return (TCGTBCPUState){
        .pc = env->ip | env->ri,
        .cs_base = env->cfm,
        .flags = flags,
    };
}

static int ia64_cpu_mmu_index(CPUState *cs, bool ifetch)
{
    CPUIA64State *env = cpu_env(cs);
    uint64_t psr = env->psr;
    int idx = ia64_cpl(env);

    if (ifetch ? (psr & PSR_IT) : (psr & PSR_DT)) {
        idx |= MMU_IDX_VIRT;
    }
    return idx;
}

static void ia64_cpu_reset_hold(Object *obj, ResetType type)
{
    CPUState *cs = CPU(obj);
    IA64CPU *cpu = IA64_CPU(cs);
    IA64CPUClass *icc = IA64_CPU_GET_CLASS(obj);
    CPUIA64State *env = &cpu->env;

    if (icc->parent_phases.hold) {
        icc->parent_phases.hold(obj, type);
    }

    memset(env, 0, offsetof(CPUIA64State, itc_freq));

    /* f1 = +1.0 */
    env->fr[1].mant = 1ull << 63;
    env->fr[1].se = FP_EXP_BIAS;
    env->pr[0] = 1;
    /* Firmware hands over in physical mode, collection off, bank 1. */
    env->psr = PSR_BN;
    /* FPSR as Linux sets it: all traps disabled, sf0 extended */
    env->ar[AR_FPSR] = 0x0009804c0270033full;
    env->rr[0] = 0x30;
    for (int i = 1; i < 8; i++) {
        env->rr[i] = 0x30;
    }
    env->cr[CR_ITV] = 1 << 16;
    env->cr[CR_PMV] = 1 << 16;
    env->cr[CR_CMCV] = 1 << 16;
    env->cr[CR_LRR0] = 1 << 16;
    env->cr[CR_LRR1] = 1 << 16;
    env->cr[CR_TPR] = 1 << 16;
    if (env->itm_timer) {
        timer_del(env->itm_timer);
    }
    cs->exception_index = -1;
}

#ifndef CONFIG_USER_ONLY
static void ia64_itm_cb(void *opaque)
{
    IA64CPU *cpu = opaque;
    CPUIA64State *env = &cpu->env;
    uint64_t itv = env->cr[CR_ITV];

    if (!(itv & (1 << 16))) {
        ia64_set_irq_vector(env, itv & 0xff, true);
    }
}
#endif

static ObjectClass *ia64_cpu_class_by_name(const char *cpu_model)
{
    return object_class_by_name(TYPE_IA64_CPU);
}

static void ia64_cpu_realizefn(DeviceState *dev, Error **errp)
{
    CPUState *cs = CPU(dev);
    IA64CPU *cpu = IA64_CPU(dev);
    IA64CPUClass *icc = IA64_CPU_GET_CLASS(dev);
    Error *local_err = NULL;

    cpu_common_realize(cs, &local_err);
    if (local_err != NULL) {
        error_propagate(errp, local_err);
        return;
    }

    cpu->env.itc_freq = 100 * 1000 * 1000;
#ifndef CONFIG_USER_ONLY
    cpu->env.itm_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, ia64_itm_cb, cpu);
#endif
    qemu_init_vcpu(cs);
    cpu_reset(cs);

    icc->parent_realize(dev, errp);
}

#ifndef CONFIG_USER_ONLY
static bool ia64_cpu_exec_interrupt(CPUState *cs, int interrupt_request)
{
    CPUIA64State *env = cpu_env(cs);

    if ((interrupt_request & CPU_INTERRUPT_HARD) && (env->psr & PSR_I)) {
        cs->exception_index = EXCP_IA64(IVT_EXTINT);
        env->pend_isr = 0;
        env->pend_flags = 0;
        ia64_cpu_do_interrupt(cs);
        return true;
    }
    return false;
}
#endif

/*
 * Disassembly for -d in_asm.  "Addresses" are bundle | slot, so each
 * instruction advances by one, except the last slot of a bundle.
 */
static int ia64_print_insn(bfd_vma pc, disassemble_info *info)
{
    uint8_t bytes[16];
    struct asm_bundle b;
    char buf[160];
    int slot = pc & 3;
    uint64_t ip = pc & ~0xfull;
    bool mlx;

    if (info->read_memory_func(ip, bytes, 16, info)) {
        return -1;
    }
    if (!asm_decode((uint64_t)(uintptr_t)bytes, &b)) {
        info->fprintf_func(info->stream, "<reserved template>");
        return 16 - slot;
    }
    mlx = b.b_templ[1] == 'L';
    if (mlx && slot == 1) {
        asm_format_inst(&b, 2, ip, buf, sizeof(buf));
    } else {
        asm_format_inst(&b, slot, ip, buf, sizeof(buf));
    }
    info->fprintf_func(info->stream, "%s", buf);
    return (slot == 2 || (mlx && slot == 1)) ? 16 - slot : 1;
}

static void ia64_cpu_disas_set_info(const CPUState *cpu,
                                    disassemble_info *info)
{
    info->endian = BFD_ENDIAN_LITTLE;
    info->print_insn = ia64_print_insn;
}

/*
 * gdb's ia64 numbering (gdb/ia64-tdep.h): r0-r127, f0-f127 (16 bytes in
 * the 82-bit register format), p0-p63, b0-b7, vfp, vrap, pr, ip, psr
 * (with ri), cfm, ar0-ar127.  The stacked registers are those of the
 * current frame, unrotated.
 */
enum {
    GDB_FR0 = 128, GDB_PR0 = 256, GDB_BR0 = 320, GDB_VFP = 328,
    GDB_VRAP = 329, GDB_PR = 330, GDB_IP = 331, GDB_PSR = 332, GDB_CFM = 333,
    GDB_AR0 = 334, GDB_NUM_REGS = 462,
};

static uint64_t ia64_gdb_pr(CPUIA64State *env)
{
    uint64_t pr = 0;

    for (int i = 0; i < 64; i++) {
        pr |= (uint64_t)(env->pr[i] & 1) << i;
    }
    return pr;
}

/*
 * gdb reads the stacked registers from the backing store, where neither
 * the dirty partition nor the current frame have been written yet.  Put
 * them there when it asks for BSP; the RSE would store the same values.
 */
static void ia64_gdb_sync_rse(CPUState *cs)
{
    CPUIA64State *env = cpu_env(cs);
    uint64_t addr = env->ar[AR_BSPSTORE];
    int sof = FIELD_EX64(env->cfm, CFM, SOF);

    for (int i = 0; i < env->rse_ndirty + sof; i++) {
        uint64_t v = i < env->rse_ndirty ? env->rse[i]
                                         : env->gr[32 + i - env->rse_ndirty];
        uint8_t buf[8];

        if (ia64_is_rnat_slot(addr)) {
            addr += 8;
        }
        stq_le_p(buf, v);
        cpu_memory_rw_debug(cs, addr, buf, 8, true);
        addr += 8;
    }
}

static int ia64_cpu_gdb_read_register(CPUState *cs, GByteArray *mem_buf,
                                      int n)
{
    CPUIA64State *env = cpu_env(cs);

    if (n < 128) {
        return gdb_get_reg64(mem_buf, env->gr[n]);
    } else if (n < GDB_PR0) {
        return gdb_get_reg128(mem_buf, env->fr[n - GDB_FR0].se,
                              env->fr[n - GDB_FR0].mant);
    } else if (n < GDB_BR0) {
        return gdb_get_reg64(mem_buf, env->pr[n - GDB_PR0] & 1);
    } else if (n < GDB_VFP) {
        return gdb_get_reg64(mem_buf, env->br[n - GDB_BR0]);
    }
    switch (n) {
    case GDB_VFP:
    case GDB_VRAP:
        return gdb_get_reg64(mem_buf, 0);
    case GDB_PR:
        return gdb_get_reg64(mem_buf, ia64_gdb_pr(env));
    case GDB_IP:
        return gdb_get_reg64(mem_buf, env->ip);
    case GDB_PSR:
        return gdb_get_reg64(mem_buf, env->psr |
                             ((uint64_t)env->ri << PSR_RI_SHIFT));
    case GDB_CFM:
        return gdb_get_reg64(mem_buf, env->cfm);
    }
    if (n < GDB_NUM_REGS) {
        int ar = n - GDB_AR0;

        if (ar == AR_BSP) {
            ia64_gdb_sync_rse(cs);
        }
        return gdb_get_reg64(mem_buf, ar == AR_BSP ? ia64_bsp(env)
                                                   : env->ar[ar]);
    }
    return 0;
}

static int ia64_cpu_gdb_write_register(CPUState *cs, uint8_t *mem_buf, int n)
{
    CPUIA64State *env = cpu_env(cs);
    uint64_t val = ldq_p(mem_buf);

    if (n < 128) {
        env->gr[n] = val;
    } else if (n < GDB_PR0) {
        env->fr[n - GDB_FR0].mant = val;
        env->fr[n - GDB_FR0].se = ldq_p(mem_buf + 8) & 0x3ffff;
        return 16;
    } else if (n < GDB_BR0) {
        env->pr[n - GDB_PR0] = val & 1;
    } else if (n < GDB_VFP) {
        env->br[n - GDB_BR0] = val;
    } else if (n == GDB_PR) {
        for (int i = 1; i < 64; i++) {
            env->pr[i] = (val >> i) & 1;
        }
    } else if (n == GDB_IP) {
        env->ip = val & ~0xfull;
    } else if (n == GDB_PSR) {
        env->ri = (val >> PSR_RI_SHIFT) & 3;
    } else if (n >= GDB_AR0 && n < GDB_NUM_REGS && n - GDB_AR0 != AR_BSP) {
        env->ar[n - GDB_AR0] = val;
    }
    return n < GDB_NUM_REGS ? 8 : 0;
}

#ifndef CONFIG_USER_ONLY
#include "hw/core/sysemu-cpu-ops.h"

static const struct SysemuCPUOps ia64_sysemu_ops = {
    .has_work = ia64_cpu_has_work,
    .get_phys_addr_debug = ia64_cpu_get_phys_page_debug,
};
#endif

static const TCGCPUOps ia64_tcg_ops = {
    .guest_default_memory_order = TCG_MO_ALL,
    .mttcg_supported = false,

    .initialize = ia64_tcg_init,
    .translate_code = ia64_translate_code,
    .get_tb_cpu_state = ia64_get_tb_cpu_state,
    .restore_state_to_opc = ia64_restore_state_to_opc,
    .mmu_index = ia64_cpu_mmu_index,
    .pointer_wrap = cpu_pointer_wrap_notreached,

#ifndef CONFIG_USER_ONLY
    .tlb_fill = ia64_cpu_tlb_fill,
    .cpu_exec_interrupt = ia64_cpu_exec_interrupt,
    .cpu_exec_reset = cpu_reset,
    .do_interrupt = ia64_cpu_do_interrupt,
    .do_unaligned_access = ia64_cpu_do_unaligned_access,
#endif
};

static void ia64_cpu_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    CPUClass *cc = CPU_CLASS(oc);
    IA64CPUClass *icc = IA64_CPU_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    device_class_set_parent_realize(dc, ia64_cpu_realizefn,
                                    &icc->parent_realize);
    resettable_class_set_parent_phases(rc, NULL, ia64_cpu_reset_hold, NULL,
                                       &icc->parent_phases);

    cc->class_by_name = ia64_cpu_class_by_name;
    cc->dump_state = ia64_cpu_dump_state;
    cc->set_pc = ia64_cpu_set_pc;
    cc->get_pc = ia64_cpu_get_pc;
    cc->disas_set_info = ia64_cpu_disas_set_info;
#ifndef CONFIG_USER_ONLY
    cc->sysemu_ops = &ia64_sysemu_ops;
#endif
    cc->gdb_read_register = ia64_cpu_gdb_read_register;
    cc->gdb_write_register = ia64_cpu_gdb_write_register;
    cc->gdb_num_core_regs = GDB_NUM_REGS;
    cc->tcg_ops = &ia64_tcg_ops;
}

static const TypeInfo ia64_cpu_type_info = {
    .name = TYPE_IA64_CPU,
    .parent = TYPE_CPU,
    .instance_size = sizeof(IA64CPU),
    .instance_align = __alignof(IA64CPU),
    .class_size = sizeof(IA64CPUClass),
    .class_init = ia64_cpu_class_init,
};

static void ia64_cpu_register_types(void)
{
    type_register_static(&ia64_cpu_type_info);
}

type_init(ia64_cpu_register_types)
