/*
 * QEMU AVR32 CPU: exceptions, interrupts, MMU and system registers
 *
 * The event model, the MMU and the system registers follow the AVR32
 * Architecture Document (Atmel doc32000) for AVR32B as implemented by the
 * AT32AP7000.
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
#include "qemu/log.h"
#include "qemu/timer.h"
#include "cpu.h"
#include "exec/cpu-interrupt.h"
#include "exec/target_page.h"
#include "exec/cputlb.h"
#include "exec/page-protection.h"
#include "exec/helper-proto.h"
#include "accel/tcg/cpu-loop.h"
#include "hw/core/irq.h"

/* OCD device ID: AT32AP700x, Atmel, revision C */
#define AP7000_DID      0x21e8203f

/*
 * Status register and register banks
 */

uint32_t avr32_get_sr(CPUAVR32State *env)
{
    uint32_t sr = 0;

    for (int i = 0; i < 32; i++) {
        sr |= (env->sflags[i] & 1) << i;
    }
    return sr;
}

/*
 * The application mode has its own stack pointer, and on the AP7000 the
 * INT3 context has its own r8-r12 and lr.
 */
static void avr32_switch_bank(CPUAVR32State *env, int old_mode, int new_mode)
{
    bool old_app = old_mode == AVR32_MODE_APP;
    bool new_app = new_mode == AVR32_MODE_APP;
    bool old_int3 = old_mode == AVR32_MODE_INT3;
    bool new_int3 = new_mode == AVR32_MODE_INT3;

    if (old_app != new_app) {
        if (old_app) {
            env->sp_app = env->r[AVR32_SP_REG];
            env->r[AVR32_SP_REG] = env->sp_sys;
        } else {
            env->sp_sys = env->r[AVR32_SP_REG];
            env->r[AVR32_SP_REG] = env->sp_app;
        }
    }
    if (old_int3 != new_int3) {
        for (int i = 0; i < 6; i++) {
            int reg = i < 5 ? 8 + i : AVR32_LR_REG;
            uint32_t tmp = env->r[reg];

            env->r[reg] = env->int3_bank[i];
            env->int3_bank[i] = tmp;
        }
    }
}

void avr32_set_sr(CPUAVR32State *env, uint32_t sr)
{
    int old_mode = avr32_mode(env);

    for (int i = 0; i < 32; i++) {
        env->sflags[i] = (sr >> i) & 1;
    }
    avr32_switch_bank(env, old_mode, avr32_mode(env));
}

/* Enter an event context: save SR and the return address, switch modes */
static void avr32_enter(CPUAVR32State *env, int mode, uint32_t ret_pc,
                        uint32_t set_bits, uint32_t handler)
{
    uint32_t sr = avr32_get_sr(env);
    uint32_t nsr = sr;

    env->sysr[SYSREG_RSR(mode)] = sr;
    env->sysr[SYSREG_RAR(mode)] = ret_pc;
    nsr &= ~((7u << SR_M0) | (1u << SR_R) | (1u << SR_J));
    nsr |= (mode << SR_M0) | set_bits;
    avr32_set_sr(env, nsr);
    env->r[AVR32_PC_REG] = handler;
}

/*
 * Exceptions
 */

void avr32_cpu_do_interrupt(CPUState *cs)
{
    CPUAVR32State *env = cpu_env(cs);
    int idx = cs->exception_index;
    uint32_t pc = env->r[AVR32_PC_REG];
    uint32_t evba = env->sysr[SYSREG_EVBA];

    qemu_log_mask(CPU_LOG_INT, "avr32: exception 0x%x at pc 0x%08x\n",
                  idx, pc);

    if (idx == EXCP_AVR32_BREAKPOINT) {
        uint32_t sr = avr32_get_sr(env);

        env->sysr[SYSREG_RSR_DBG] = sr;
        env->sysr[SYSREG_RAR_DBG] = pc;
        sr &= ~((7u << SR_M0) | (1u << SR_R) | (1u << SR_J));
        sr |= (AVR32_MODE_EX << SR_M0) | (1u << SR_D) | (1u << SR_DM) |
              (1u << SR_EM) | (1u << SR_GM);
        avr32_set_sr(env, sr);
        env->r[AVR32_PC_REG] = evba + EXCP_AVR32_BREAKPOINT;
        return;
    }

    if (idx != EXCP_AVR32_NMI && env->sflags[SR_EM] &&
        idx != EXCP_AVR32_TLB_MULTIPLE) {
        idx = EXCP_AVR32_UNRECOVERABLE;
    }
    env->sysr[SYSREG_ECR] = (idx & 0x1ff) >> 2;
    avr32_enter(env, idx == EXCP_AVR32_NMI ? AVR32_MODE_NMI : AVR32_MODE_EX,
                pc, (1u << SR_EM) | (1u << SR_GM), evba + idx);
}

static void avr32_take_irq(CPUAVR32State *env, int level)
{
    uint32_t masks = ((2u << level) - 1) << SR_I0M;
    uint32_t offset = env->irq_offset[level];

    env->sysr[SYSREG_ECR] = (offset & 0x1ff) >> 2;
    avr32_enter(env, AVR32_MODE_INT0 + level, env->r[AVR32_PC_REG], masks,
                env->sysr[SYSREG_EVBA] + offset);
}

bool avr32_cpu_exec_interrupt(CPUState *cs, int interrupt_request)
{
    CPUAVR32State *env = cpu_env(cs);

    if (!(interrupt_request & CPU_INTERRUPT_HARD)) {
        return false;
    }
    if (env->nmi_pending && avr32_mode(env) != AVR32_MODE_NMI) {
        env->sysr[SYSREG_ECR] = EXCP_AVR32_NMI >> 2;
        avr32_enter(env, AVR32_MODE_NMI, env->r[AVR32_PC_REG],
                    (1u << SR_EM) | (1u << SR_GM),
                    env->sysr[SYSREG_EVBA] + EXCP_AVR32_NMI);
        return true;
    }
    if (env->sflags[SR_GM]) {
        return false;
    }
    for (int level = 3; level >= 0; level--) {
        if ((env->irq_pending & (1u << level)) &&
            !env->sflags[SR_I0M + level]) {
            avr32_take_irq(env, level);
            return true;
        }
    }
    return false;
}

static void avr32_update_irq(AVR32CPU *cpu)
{
    CPUState *cs = CPU(cpu);

    if (cpu->env.irq_pending || cpu->env.nmi_pending) {
        cpu_interrupt(cs, CPU_INTERRUPT_HARD);
    } else {
        cpu_reset_interrupt(cs, CPU_INTERRUPT_HARD);
    }
}

/* Called by the interrupt controller with the autovector of a level */
void avr32_cpu_set_irq(AVR32CPU *cpu, int level, bool raise, uint32_t offset)
{
    if (raise) {
        cpu->env.irq_pending |= 1u << level;
        cpu->env.irq_offset[level] = offset;
    } else {
        cpu->env.irq_pending &= ~(1u << level);
    }
    avr32_update_irq(cpu);
}

void avr32_cpu_set_nmi(AVR32CPU *cpu, bool raise)
{
    cpu->env.nmi_pending = raise;
    avr32_update_irq(cpu);
}

G_NORETURN static void avr32_raise(CPUAVR32State *env, int excp,
                                   uintptr_t retaddr)
{
    CPUState *cs = env_cpu(env);

    cs->exception_index = excp;
    cpu_loop_exit_restore(cs, retaddr);
}

void helper_raise_exception(CPUAVR32State *env, uint32_t excp)
{
    avr32_raise(env, excp, 0);
}

void helper_scall(CPUAVR32State *env, uint32_t next_pc)
{
    int mode = avr32_mode(env);
    uint32_t handler = env->sysr[SYSREG_EVBA] + EXCP_AVR32_SCALL;

    if (mode == AVR32_MODE_APP || mode == AVR32_MODE_SUP) {
        avr32_enter(env, AVR32_MODE_SUP, next_pc, 0, handler);
    } else {
        env->r[AVR32_LR_REG] = next_pc;
        env->r[AVR32_PC_REG] = handler;
    }
}

void helper_rets(CPUAVR32State *env)
{
    int mode = avr32_mode(env);

    if (mode == AVR32_MODE_APP) {
        avr32_raise(env, EXCP_AVR32_PRIVILEGE, 0);
    }
    if (mode == AVR32_MODE_SUP) {
        uint32_t pc = env->sysr[SYSREG_RAR(AVR32_MODE_SUP)];

        avr32_set_sr(env, env->sysr[SYSREG_RSR(AVR32_MODE_SUP)]);
        env->r[AVR32_PC_REG] = pc;
    } else {
        env->r[AVR32_PC_REG] = env->r[AVR32_LR_REG];
    }
}

void helper_rete(CPUAVR32State *env)
{
    int mode = avr32_mode(env);
    uint32_t pc;

    if (mode == AVR32_MODE_APP || mode == AVR32_MODE_SUP) {
        avr32_raise(env, EXCP_AVR32_PRIVILEGE, 0);
    }
    pc = env->sysr[SYSREG_RAR(mode)];
    avr32_set_sr(env, env->sysr[SYSREG_RSR(mode)]);
    env->sflags[SR_L] = 0;
    env->r[AVR32_PC_REG] = pc;
}

void helper_retd(CPUAVR32State *env)
{
    uint32_t pc = env->sysr[SYSREG_RAR_DBG];

    avr32_set_sr(env, env->sysr[SYSREG_RSR_DBG]);
    env->r[AVR32_PC_REG] = pc;
}

void helper_sleep(CPUAVR32State *env, uint32_t op8, uint32_t next_pc)
{
    CPUState *cs = env_cpu(env);

    if (op8 & 0x80) {
        env->sflags[SR_GM] = 0;
    }
    env->r[AVR32_PC_REG] = next_pc;
    cs->halted = 1;
    cs->exception_index = EXCP_HLT;
    cpu_loop_exit(cs);
}

/*
 * Cycle counter and compare interrupt
 */

static uint32_t avr32_count(CPUAVR32State *env)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    return muldiv64(now - env->count_base, env->count_freq,
                    NANOSECONDS_PER_SECOND);
}

static void avr32_compare_schedule(CPUAVR32State *env)
{
    uint32_t compare = env->sysr[SYSREG_COMPARE];
    uint64_t ticks;

    if (!compare) {
        timer_del(env->compare_timer);
        return;
    }
    /* COUNT wraps without a match if COMPARE is already behind it */
    ticks = compare;
    if (compare <= avr32_count(env)) {
        ticks += 1ull << 32;
    }
    timer_mod(env->compare_timer, env->count_base +
              muldiv64(ticks, NANOSECONDS_PER_SECOND, env->count_freq));
}

void avr32_compare_timer_cb(void *opaque)
{
    AVR32CPU *cpu = opaque;
    CPUAVR32State *env = &cpu->env;

    /*
     * Clear-on-compare came with architecture document rev. B (2007); the
     * AP7000 core keeps COUNT running, and Linux uses it as a continuous
     * clocksource while programming COMPARE = COUNT + delta.
     */
    env->compare_pending = true;
    qemu_irq_raise(env->compare_irq);
    avr32_compare_schedule(env);
}

/*
 * System registers
 */

uint32_t helper_mfsr(CPUAVR32State *env, uint32_t n)
{
    switch (n) {
    case SYSREG_SR:
        return avr32_get_sr(env);
    case SYSREG_COUNT:
        return avr32_count(env);
    default:
        return env->sysr[n & 0xff];
    }
}

void helper_mtsr(CPUAVR32State *env, uint32_t n, uint32_t val)
{
    CPUState *cs = env_cpu(env);

    switch (n) {
    case SYSREG_SR:
        avr32_set_sr(env, val);
        break;
    case SYSREG_CONFIG0:
    case SYSREG_CONFIG1:
    case SYSREG_BEAR:
        break;
    case SYSREG_COUNT:
        env->count_base = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) -
                          muldiv64(val, NANOSECONDS_PER_SECOND,
                                   env->count_freq);
        avr32_compare_schedule(env);
        break;
    case SYSREG_COMPARE:
        env->sysr[n] = val;
        env->compare_pending = false;
        qemu_irq_lower(env->compare_irq);
        avr32_compare_schedule(env);
        break;
    case SYSREG_TLBEHI:
        /* QEMU's TLB does not know about ASIDs */
        if ((val ^ env->sysr[n]) & TLBEHI_ASID) {
            tlb_flush(cs);
        }
        env->sysr[n] = val;
        break;
    case SYSREG_MMUCR:
        if (val & MMUCR_I) {
            for (int i = 0; i < AVR32_TLB_SIZE; i++) {
                env->tlb[i].hi &= ~TLBEHI_V;
            }
            val &= ~MMUCR_I;
        }
        env->sysr[n] = val;
        tlb_flush(cs);
        break;
    default:
        env->sysr[n & 0xff] = val;
        break;
    }
}

/* OCD registers: only the device ID is of interest */
uint32_t helper_mfdr(CPUAVR32State *env, uint32_t n)
{
    return n == 0 ? AP7000_DID : 0;
}

/*
 * MMU
 */

typedef enum {
    MMU_OK,
    MMU_MISS,
    MMU_PROT,
    MMU_MODIFIED,
} MMUResult;

static const int tlb_page_shift[4] = { 10, 12, 16, 20 };

static int avr32_tlb_match(CPUAVR32State *env, uint32_t va, uint32_t asid)
{
    bool shared = env->sysr[SYSREG_MMUCR] & MMUCR_M;

    for (int i = 0; i < AVR32_TLB_SIZE; i++) {
        AVR32TLBEntry *e = &env->tlb[i];
        int shift = tlb_page_shift[(e->lo >> TLBELO_SZ_SHIFT) & 3];

        if (!(e->hi & TLBEHI_V) || ((va ^ e->hi) >> shift)) {
            continue;
        }
        if (shared || (e->lo & TLBELO_G) || (e->hi & TLBEHI_ASID) == asid) {
            return i;
        }
    }
    return -1;
}

static MMUResult avr32_translate(CPUAVR32State *env, uint32_t va,
                                 MMUAccessType access, bool user,
                                 hwaddr *pa, int *prot, int *page_size)
{
    uint32_t mmucr = env->sysr[SYSREG_MMUCR];
    bool paging = mmucr & MMUCR_E;
    bool segmented = !paging || (mmucr & MMUCR_S);
    AVR32TLBEntry *e;
    int i, shift, ap;

    *prot = PAGE_READ | PAGE_WRITE | PAGE_EXEC;
    *page_size = TARGET_PAGE_SIZE;

    if (segmented && va >= 0x80000000) {
        if (user) {
            return MMU_PROT;
        }
        if (va >= 0xe0000000) {
            *pa = va;                           /* P4 */
            return MMU_OK;
        }
        if (va < 0xc0000000 || !paging) {
            *pa = va & 0x1fffffff;              /* P1, P2, unpaged P3 */
            return MMU_OK;
        }
    } else if (!paging) {
        *pa = va;
        return MMU_OK;
    }

    i = avr32_tlb_match(env, va, env->sysr[SYSREG_TLBEHI] & TLBEHI_ASID);
    if (i < 0) {
        return MMU_MISS;
    }
    e = &env->tlb[i];
    env->sysr[SYSREG_TLBARLO] &= ~(1u << (31 - i));    /* accessed */

    ap = (e->lo >> TLBELO_AP_SHIFT) & 7;
    if (user && !(ap & 4)) {
        return MMU_PROT;
    }
    if (access == MMU_INST_FETCH && !(ap & 1)) {
        return MMU_PROT;
    }
    if (access == MMU_DATA_STORE) {
        if (!(ap & 2)) {
            return MMU_PROT;
        }
        if (!(e->lo & TLBELO_D)) {
            return MMU_MODIFIED;
        }
    }

    shift = tlb_page_shift[(e->lo >> TLBELO_SZ_SHIFT) & 3];
    *pa = (e->lo & ~((1u << shift) - 1)) | (va & ((1u << shift) - 1));
    *prot = PAGE_READ;
    if (ap & 1) {
        *prot |= PAGE_EXEC;
    }
    if ((ap & 2) && (e->lo & TLBELO_D)) {
        *prot |= PAGE_WRITE;
    }
    *page_size = MAX(1 << shift, TARGET_PAGE_SIZE);
    return MMU_OK;
}

static void avr32_mmu_fault_regs(CPUAVR32State *env, uint32_t va, bool insn)
{
    env->sysr[SYSREG_TLBEAR] = va;
    env->sysr[SYSREG_TLBEHI] = (va & 0xfffffc00) | TLBEHI_V |
                               (insn ? TLBEHI_I : 0) |
                               (env->sysr[SYSREG_TLBEHI] & TLBEHI_ASID);
}

bool avr32_cpu_tlb_fill(CPUState *cs, vaddr address, int size,
                        MMUAccessType access_type, int mmu_idx,
                        bool probe, uintptr_t retaddr)
{
    CPUAVR32State *env = cpu_env(cs);
    hwaddr pa;
    int prot, page_size, excp;
    MMUResult res;

    res = avr32_translate(env, address, access_type, mmu_idx == MMU_USER_IDX,
                          &pa, &prot, &page_size);
    if (res == MMU_OK) {
        tlb_set_page(cs, address & TARGET_PAGE_MASK, pa & TARGET_PAGE_MASK,
                     prot, mmu_idx, page_size);
        return true;
    }
    if (probe) {
        return false;
    }

    switch (res) {
    case MMU_MISS:
        excp = access_type == MMU_INST_FETCH ? EXCP_AVR32_ITLB_MISS :
               access_type == MMU_DATA_STORE ? EXCP_AVR32_DTLB_MISS_W :
                                               EXCP_AVR32_DTLB_MISS_R;
        break;
    case MMU_PROT:
        excp = access_type == MMU_INST_FETCH ? EXCP_AVR32_ITLB_PROT :
               access_type == MMU_DATA_STORE ? EXCP_AVR32_DTLB_PROT_W :
                                               EXCP_AVR32_DTLB_PROT_R;
        break;
    default:
        excp = EXCP_AVR32_DTLB_MODIFIED;
        break;
    }
    avr32_mmu_fault_regs(env, address, access_type == MMU_INST_FETCH);
    avr32_raise(env, excp, retaddr);
}

void avr32_cpu_do_unaligned_access(CPUState *cs, vaddr addr,
                                   MMUAccessType access_type, int mmu_idx,
                                   uintptr_t retaddr)
{
    CPUAVR32State *env = cpu_env(cs);
    int excp = access_type == MMU_INST_FETCH ? EXCP_AVR32_INSN_ADDR :
               access_type == MMU_DATA_STORE ? EXCP_AVR32_DATA_ADDR_W :
                                               EXCP_AVR32_DATA_ADDR_R;

    avr32_mmu_fault_regs(env, addr, access_type == MMU_INST_FETCH);
    avr32_raise(env, excp, retaddr);
}

hwaddr avr32_cpu_get_phys_page_debug(CPUState *cs, vaddr addr)
{
    CPUAVR32State *env = cpu_env(cs);
    hwaddr pa;
    int prot, page_size;
    uint32_t tlbarlo = env->sysr[SYSREG_TLBARLO];

    if (avr32_translate(env, addr, MMU_DATA_LOAD, false, &pa, &prot,
                        &page_size) != MMU_OK) {
        pa = -1;
    }
    env->sysr[SYSREG_TLBARLO] = tlbarlo;
    return pa;
}

/* The AP7000 has one shared TLB: TLBEHI[I] and MMUCR[IRP] are not used */
static int avr32_tlb_index(CPUAVR32State *env)
{
    return (env->sysr[SYSREG_MMUCR] >> MMUCR_DRP_SHIFT) % AVR32_TLB_SIZE;
}

static void avr32_tlb_flush_entry(CPUState *cs, AVR32TLBEntry *e)
{
    int shift = tlb_page_shift[(e->lo >> TLBELO_SZ_SHIFT) & 3];
    uint32_t size = MAX(1u << shift, TARGET_PAGE_SIZE);

    if (e->hi & TLBEHI_V) {
        tlb_flush_range_by_mmuidx(cs, e->hi & ~(size - 1), size,
                                  (1 << MMU_USER_IDX) | (1 << MMU_KERNEL_IDX),
                                  TARGET_LONG_BITS);
    }
}

void helper_tlbw(CPUAVR32State *env)
{
    int i = avr32_tlb_index(env);
    AVR32TLBEntry *e = &env->tlb[i];

    avr32_tlb_flush_entry(env_cpu(env), e);
    e->hi = env->sysr[SYSREG_TLBEHI] & ~TLBEHI_I;
    e->lo = env->sysr[SYSREG_TLBELO];
    env->sysr[SYSREG_TLBARLO] |= 1u << (31 - i);        /* not accessed */
}

void helper_tlbr(CPUAVR32State *env)
{
    AVR32TLBEntry *e = &env->tlb[avr32_tlb_index(env)];

    env->sysr[SYSREG_TLBEHI] = e->hi;
    env->sysr[SYSREG_TLBELO] = e->lo;
}

void helper_tlbs(CPUAVR32State *env)
{
    uint32_t ehi = env->sysr[SYSREG_TLBEHI];
    uint32_t mmucr = env->sysr[SYSREG_MMUCR] | MMUCR_N;
    int i = avr32_tlb_match(env, ehi, ehi & TLBEHI_ASID);

    if (i >= 0) {
        mmucr &= ~(MMUCR_N | (0x3fu << MMUCR_DRP_SHIFT));
        mmucr |= i << MMUCR_DRP_SHIFT;
    }
    env->sysr[SYSREG_MMUCR] = mmucr;
}

/* MACSATHH.W: saturating fractional multiply-accumulate of halfwords */
void helper_macsathhw(CPUAVR32State *env, uint32_t rd, uint32_t op1,
                      uint32_t op2)
{
    int32_t a = (int16_t)op1, b = (int16_t)op2;
    int64_t prod, res;

    if (a == -0x8000 && b == -0x8000) {
        prod = 0x7fffffff;
        env->sflags[SR_Q] = 1;
    } else {
        prod = (int64_t)(a * b) << 1;
    }
    res = prod + (int32_t)env->r[rd];
    if (res > INT32_MAX) {
        res = INT32_MAX;
        env->sflags[SR_Q] = 1;
    } else if (res < INT32_MIN) {
        res = INT32_MIN;
        env->sflags[SR_Q] = 1;
    }
    env->r[rd] = res;
}
