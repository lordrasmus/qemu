/*
 * TILE-Gx system emulation: protection levels, interrupts, SPRs, MMU
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 *
 * Linux runs as a client of the Tilera hypervisor.  The hypervisor is not
 * emulated as code: its calls are served by tilegx_hv_call(), and its
 * TLB refill is the page table walk in tilegx_cpu_tlb_fill().  A walk
 * that finds no valid PTE is handed to the client as the hypervisor
 * would, as a downcall to its ITLB_MISS/DTLB_MISS/DTLB_ACCESS vector
 * with the address in SYSTEM_SAVE_K_3 and the access info in
 * SYSTEM_SAVE_K_2.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "cpu.h"
#include "exec/helper-proto.h"
#include "exec/cputlb.h"
#include "exec/page-protection.h"
#include "exec/target_page.h"
#include "accel/tcg/cpu-ldst.h"
#include "accel/tcg/probe.h"
#include "accel/tcg/cpu-loop.h"
#include "system/memory.h"
#include "hw/core/cpu.h"
#include "qemu/main-loop.h"

/* HV_PTE bits (hypervisor.h) */
#define PTE_PRESENT     (1ULL << 0)
#define PTE_PAGE        (1ULL << 8)
#define PTE_GLOBAL      (1ULL << 9)
#define PTE_USER        (1ULL << 10)
#define PTE_ACCESSED    (1ULL << 11)
#define PTE_DIRTY       (1ULL << 12)
#define PTE_SUPER       (1ULL << 15)
#define PTE_READABLE    (1ULL << 32)
#define PTE_WRITABLE    (1ULL << 33)
#define PTE_EXECUTABLE  (1ULL << 34)
#define PTE_PTFN_SHIFT  35
#define PT_ALIGN_SHIFT  11      /* HV_LOG2_PAGE_TABLE_ALIGN */
#define L1_SPAN_SHIFT   32      /* HV_LOG2_L1_SPAN */
#define VA_WIDTH        42

/* ------------------------------------------------------------------ */
/* Interrupts                                                          */

static int target_pl(CPUTLGState *env, int intno)
{
    return MAX(env->mpl[intno], env->cpl);
}

static void ipi_update(CPUTLGState *env)
{
    int pl;

    for (pl = 0; pl < 4; pl++) {
        if (env->ipi_event[pl] & ~env->ipi_mask[pl]) {
            env->pending |= 1ULL << (INT_IPI_0 - pl);
        } else {
            env->pending &= ~(1ULL << (INT_IPI_0 - pl));
        }
    }
}

/* The highest-priority queued interrupt that may be taken now, or -1. */
static int deliverable_irq(CPUTLGState *env)
{
    uint64_t p = env->pending;
    int i;

    for (i = 0; p && i < TILEGX_NUM_INTERRUPTS; i++) {
        int pl;

        if (!(p & (1ULL << i))) {
            continue;
        }
        pl = target_pl(env, i);
        if (env->int_mask[pl] & (1ULL << i)) {
            continue;
        }
        if (pl == env->cpl && env->ics) {
            continue;
        }
        return i;
    }
    return -1;
}

void tilegx_update_irq(CPUTLGState *env)
{
    CPUState *cs = env_cpu(env);
    BQL_LOCK_GUARD();

    ipi_update(env);
    if (deliverable_irq(env) >= 0) {
        cpu_interrupt(cs, CPU_INTERRUPT_HARD);
    } else {
        cpu_reset_interrupt(cs, CPU_INTERRUPT_HARD);
    }
}

/*
 * swint2 is the hypervisor's "fast" system call (hv/syscall_public.h):
 * the number is in r10.  The only one Linux makes is fence_incoherent,
 * which is nothing to do here.
 */
#define HV_SYS_FAST_MASK            (1 << 14)
#define HV_SYS_fence_incoherent     (51 | HV_SYS_FAST_MASK | (1 << 13))

static void hv_fast_call(CPUTLGState *env)
{
    switch (env->regs[10]) {
    case HV_SYS_fence_incoherent:
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "tilegx: hypervisor fast call %#" PRIx64
                      " at pc %016" PRIx64 "\n", env->regs[10], env->pc);
        break;
    }
    env->pc += 8;
}

static void take_interrupt(CPUTLGState *env, int intno)
{
    int pl = target_pl(env, intno);

    if (intno == INT_SWINT_2 && pl == 3) {
        hv_fast_call(env);
        return;
    }
    if (pl > 2 && intno != INT_DOUBLE_FAULT) {
        cpu_abort(env_cpu(env), "tilegx: interrupt %d at PL%d is the "
                  "hypervisor's, pc %016" PRIx64 "\n", intno, pl, env->pc);
    }

    if (intno == INT_ITLB_MISS || intno == INT_DTLB_MISS ||
        intno == INT_DTLB_ACCESS) {
        env->system_save[pl][3] = env->fault_addr;
        if (env->ics && pl == env->cpl) {
            /*
             * The hypervisor leaves EX_CONTEXT alone under ICS and
             * passes the PC with the access info instead.
             */
            env->system_save[pl][2] = env->pc | env->fault_info;
            env->pc = env->vector_base[pl] + intno * 256;
            return;
        }
        env->system_save[pl][2] = env->fault_info;
    }

    env->ex_context[pl][0] = env->pc;
    env->ex_context[pl][1] = env->cpl | (env->ics << 2);
    env->cpl = pl;
    env->ics = 1;
    env->pc = env->vector_base[pl] + intno * 256;
}

void tilegx_cpu_do_interrupt(CPUState *cs)
{
    CPUTLGState *env = cpu_env(cs);
    int excp = cs->exception_index;

    cs->exception_index = -1;
    if (qemu_loglevel_mask(CPU_LOG_INT)) {
        qemu_log("tilegx: exception %d at pc %016" PRIx64 " pl %d r0 %"
                 PRIx64 " r10 %" PRIx64 "\n", excp, env->pc, env->cpl,
                 env->regs[0], env->regs[10]);
    }

    switch (excp) {
    case TILEGX_EXCP_HV_CALL:
        tilegx_hv_call(env, (env->pc - env->hv_glue_va) / 32);
        break;
    case TILEGX_EXCP_HALT:
        cs->halted = 1;
        break;
    default:
        if (excp >= 0 && excp < TILEGX_NUM_INTERRUPTS) {
            take_interrupt(env, excp);
        } else {
            cpu_abort(cs, "tilegx: unknown exception %d\n", excp);
        }
        break;
    }
    tilegx_update_irq(env);
}

bool tilegx_cpu_exec_interrupt(CPUState *cs, int interrupt_request)
{
    CPUTLGState *env = cpu_env(cs);
    int irq;

    if (!(interrupt_request & CPU_INTERRUPT_HARD)) {
        return false;
    }
    irq = deliverable_irq(env);
    if (irq < 0) {
        cpu_reset_interrupt(cs, CPU_INTERRUPT_HARD);
        return false;
    }
    take_interrupt(env, irq);
    tilegx_update_irq(env);
    return true;
}

G_NORETURN void tilegx_raise_exception(CPUTLGState *env, int excp,
                                       uintptr_t retaddr)
{
    CPUState *cs = env_cpu(env);

    cs->exception_index = excp;
    cpu_loop_exit_restore(cs, retaddr);
}

void helper_exception(CPUTLGState *env, uint32_t excp)
{
    CPUState *cs = env_cpu(env);

    cs->exception_index = excp;
    cpu_loop_exit(cs);
}

/*
 * iret is allowed where the PL's own EX_CONTEXT SPRs are: Linux lets PL0
 * use it, and uClibc's longjmp does.  It never raises the PL.
 */
void helper_iret(CPUTLGState *env)
{
    int pl = env->cpl;
    uint64_t ex1 = env->ex_context[pl][1];

    if (pl < env->mpl[INT_INTCTRL_0 - pl]) {
        tilegx_raise_exception(env, INT_GPV, GETPC());
    }
    env->pc = env->ex_context[pl][0] & ~7ULL;
    env->cpl = MIN(ex1 & 3, pl);
    env->ics = (ex1 >> 2) & 1;
    tilegx_update_irq(env);
}

void helper_nap(CPUTLGState *env)
{
    CPUState *cs = env_cpu(env);

    /* Resume after the nap; any interrupt, even a masked one, ends it. */
    env->pc += 8;
    if (env->pending & ~env->int_mask[env->cpl]) {
        return;
    }
    cs->halted = 1;
    cs->exception_index = EXCP_HLT;
    cpu_loop_exit(cs);
}

/* ------------------------------------------------------------------ */
/* Tile timer and cycle counter                                        */

uint64_t tilegx_cycle(CPUTLGState *env)
{
    int64_t ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - env->cycle_base_ns;

    return muldiv64(ns, env->cpu_speed, NANOSECONDS_PER_SECOND);
}

static int64_t cycles_to_ns(CPUTLGState *env, uint64_t cycles)
{
    return muldiv64(cycles, NANOSECONDS_PER_SECOND, env->cpu_speed);
}

/*
 * TILE_TIMER_CONTROL: a 31-bit down counter; bit 31 says it has wrapped
 * from zero, and INT_TILE_TIMER is raised as long as it is set.  The
 * register holds the cycle at which the count reaches zero.
 */
static uint64_t timer_read(CPUTLGState *env)
{
    uint64_t now = tilegx_cycle(env);

    if (env->tile_timer_control & (1ULL << 31)) {
        return (1ULL << 31) |
               ((0x7fffffffULL - (now - env->timer_deadline)) & 0x7fffffff);
    }
    if (now >= env->timer_deadline) {
        return 0;
    }
    return env->timer_deadline - now;
}

static void timer_write(CPUTLGState *env, uint64_t val)
{
    uint64_t now = tilegx_cycle(env);

    env->tile_timer_control = val & (1ULL << 31);
    env->timer_deadline = now + (val & 0x7fffffff);
    if (env->tile_timer_control) {
        env->pending |= 1ULL << INT_TILE_TIMER;
        timer_del(env->timer);
    } else {
        env->pending &= ~(1ULL << INT_TILE_TIMER);
        timer_mod(env->timer, env->cycle_base_ns +
                  cycles_to_ns(env, env->timer_deadline) + 1);
    }
}

void tilegx_timer_cb(void *opaque)
{
    TileGXCPU *cpu = opaque;
    CPUTLGState *env = &cpu->env;

    env->tile_timer_control = 1ULL << 31;
    env->pending |= 1ULL << INT_TILE_TIMER;
    tilegx_update_irq(env);
}

/* ------------------------------------------------------------------ */
/* Special purpose registers                                           */

static bool spr_access(CPUTLGState *env, unsigned spr, uintptr_t ra)
{
    unsigned dom = spr >> 8;

    if (dom < TILEGX_SPR_DOMAINS && env->cpl >= env->mpl[dom]) {
        return true;
    }
    env->fault_addr = spr;
    tilegx_raise_exception(env, INT_GPV, ra);
    return false;
}

uint64_t helper_mfspr(CPUTLGState *env, uint32_t spr)
{
    unsigned hi = spr >> 8, lo = spr & 0xff;
    int pl;

    spr_access(env, spr, GETPC());

    if (hi >= 0x22 && hi <= 0x25) {
        pl = 0x25 - hi;
        switch (lo) {
        case 0x80: case 0x81:
            return env->ex_context[pl][lo - 0x80];
        case 0x82: case 0x83: case 0x84: case 0x85:
            return env->system_save[pl][lo - 0x82];
        case 0x05:              /* INTCTRL_n_STATUS */
            return 0;
        case 0x06:
            return env->int_mask[pl];
        case 0x09:
            return env->vector_base[pl];
        }
    } else if (hi >= 0x1c && hi <= 0x1f) {
        pl = 0x1f - hi;
        switch (lo) {
        case 0x05:
            return env->ipi_event[pl];
        case 0x08:
            return env->ipi_mask[pl];
        }
    }

    switch (spr) {
    case 0x2780:                /* CMPEXCH_VALUE */
        return env->cmpexch_value;
    case 0x2781:                /* CYCLE */
        return tilegx_cycle(env);
    case 0x2708:                /* INTERRUPT_CRITICAL_SECTION */
        return env->ics;
    case 0x1605:                /* TILE_TIMER_CONTROL */
        return timer_read(env);
    case 0x0405: case 0x0305: case 0x0205: case 0x0105:
        return env->single_step_control[0x4 - hi];
    case 0x270b:                /* TILE_COORD: tile (0,0) */
        return 0;
    }
    if (hi == 0x27) {
        return env->world_spr[lo];
    }

    qemu_log_mask(LOG_UNIMP, "tilegx: mfspr %#x at pc %016" PRIx64 "\n",
                  spr, env->pc);
    return 0;
}

void helper_mtspr(CPUTLGState *env, uint32_t spr, uint64_t val)
{
    unsigned hi = spr >> 8, lo = spr & 0xff;
    int pl;

    spr_access(env, spr, GETPC());

    if (hi >= 0x22 && hi <= 0x25) {
        pl = 0x25 - hi;
        switch (lo) {
        case 0x80:
            env->ex_context[pl][0] = val;
            return;
        case 0x81:
            env->ex_context[pl][1] = val & 7;
            return;
        case 0x82: case 0x83: case 0x84: case 0x85:
            env->system_save[pl][lo - 0x82] = val;
            return;
        case 0x06:
            env->int_mask[pl] = val;
            goto update;
        case 0x07:
            env->int_mask[pl] &= ~val;
            goto update;
        case 0x08:
            env->int_mask[pl] |= val;
            goto update;
        case 0x09:
            env->vector_base[pl] = val;
            return;
        }
    } else if (hi >= 0x1c && hi <= 0x1f) {
        pl = 0x1f - hi;
        switch (lo) {
        case 0x05:
            env->ipi_event[pl] = val;
            goto update;
        case 0x06:
            env->ipi_event[pl] &= ~val;
            goto update;
        case 0x07:
            env->ipi_event[pl] |= val;
            goto update;
        case 0x08:
            env->ipi_mask[pl] = val;
            goto update;
        case 0x09:
            env->ipi_mask[pl] &= ~val;
            goto update;
        case 0x0a:
            env->ipi_mask[pl] |= val;
            goto update;
        }
    }

    /* MPL_<domain>_SET_<pl> */
    if (hi < TILEGX_SPR_DOMAINS && lo <= 3) {
        env->mpl[hi] = lo;
        goto update;
    }

    switch (spr) {
    case 0x2780:                /* CMPEXCH_VALUE */
        env->cmpexch_value = val;
        return;
    case 0x2708:                /* INTERRUPT_CRITICAL_SECTION */
        env->ics = val & 1;
        goto update;
    case 0x1605:                /* TILE_TIMER_CONTROL */
        timer_write(env, val);
        goto update;
    case 0x0405: case 0x0305: case 0x0205: case 0x0105:
        env->single_step_control[0x4 - hi] = val;
        return;
    case 0x2709: case 0x2707: case 0x2705: /* PASS, FAIL, DONE */
    case 0x2785:                /* SIM_CONTROL */
        return;
    }
    if (hi == 0x27) {
        /* DSTREAM_PF, TILE_RTF_HWM, PROC_STATUS: tuning, no effect here */
        env->world_spr[lo] = val;
        return;
    }

    qemu_log_mask(LOG_UNIMP, "tilegx: mtspr %#x, %#" PRIx64 " at pc %016"
                  PRIx64 "\n", spr, val, env->pc);
    return;

update:
    tilegx_update_irq(env);
}

/* ------------------------------------------------------------------ */
/* MMU                                                                 */

static bool boot_translate(CPUTLGState *env, vaddr va, hwaddr *pa)
{
    int i;

    for (i = 0; i < env->boot_map_count; i++) {
        uint64_t *m = env->boot_map[i];

        if (va >= m[0] && va - m[0] < m[2]) {
            *pa = m[1] + (va - m[0]);
            return true;
        }
    }
    return false;
}

/*
 * Walk the client's page table.  L0 is indexed by VA[41:32], L1 by the
 * large-page bits below that, L2 by the small-page bits; a PTE with
 * HV_PTE_PAGE set at L1 maps a large page.  Returns the leaf PTE (0 when
 * there is none) and its page size.
 */
static uint64_t pt_walk(CPUTLGState *env, vaddr va, int *log2_size,
                        hwaddr *pte_pa)
{
    CPUState *cs = env_cpu(env);
    AddressSpace *as = cs->as;
    int large = env->log2_page_large, small = env->log2_page_small;
    uint64_t base = env->ctx_page_table, pte;
    unsigned idx;

    idx = (va >> L1_SPAN_SHIFT) & ((1 << (VA_WIDTH - L1_SPAN_SHIFT)) - 1);
    *pte_pa = base + idx * 8;
    pte = address_space_ldq_le(as, *pte_pa, MEMTXATTRS_UNSPECIFIED, NULL);
    if (!(pte & PTE_PRESENT)) {
        return 0;
    }
    if (pte & PTE_PAGE) {
        *log2_size = L1_SPAN_SHIFT;
        return pte;
    }

    base = (pte >> PTE_PTFN_SHIFT) << PT_ALIGN_SHIFT;
    idx = (va >> large) & ((1ULL << (L1_SPAN_SHIFT - large)) - 1);
    *pte_pa = base + idx * 8;
    pte = address_space_ldq_le(as, *pte_pa, MEMTXATTRS_UNSPECIFIED, NULL);
    if (!(pte & PTE_PRESENT)) {
        return 0;
    }
    if (pte & PTE_PAGE) {
        *log2_size = large;
        return pte;
    }

    base = (pte >> PTE_PTFN_SHIFT) << PT_ALIGN_SHIFT;
    idx = (va >> small) & ((1ULL << (large - small)) - 1);
    *pte_pa = base + idx * 8;
    pte = address_space_ldq_le(as, *pte_pa, MEMTXATTRS_UNSPECIFIED, NULL);
    if (!(pte & PTE_PRESENT)) {
        return 0;
    }
    *log2_size = small;
    return pte;
}

static bool va_valid(vaddr va)
{
    int64_t s = (int64_t)va >> (VA_WIDTH - 1);

    return s == 0 || s == -1;
}

/* Translate; on failure return the interrupt the client gets. */
static int tilegx_translate(CPUTLGState *env, vaddr va, MMUAccessType access,
                            int mmu_idx, hwaddr *pa, int *prot,
                            int *log2_size)
{
    uint64_t pte, upd;
    hwaddr pte_pa;
    int fault = access == MMU_INST_FETCH ? INT_ITLB_MISS : INT_DTLB_MISS;

    if (!va_valid(va)) {
        return fault;
    }

    /* The glue page is executable whatever the page table says. */
    if (access == MMU_INST_FETCH && va >= env->hv_glue_va &&
        va - env->hv_glue_va < 0x10000) {
        *pa = va - env->hv_glue_va + 0x10000;
        *prot = PAGE_EXEC;
        *log2_size = TARGET_PAGE_BITS;
        return -1;
    }

    if (!env->ctx_valid) {
        if (!boot_translate(env, va, pa)) {
            return fault;
        }
        *prot = PAGE_READ | PAGE_WRITE | PAGE_EXEC;
        *log2_size = TARGET_PAGE_BITS;
        return -1;
    }

    pte = pt_walk(env, va, log2_size, &pte_pa);
    if (!pte) {
        return fault;
    }
    if (mmu_idx == MMU_USER_IDX && !(pte & PTE_USER)) {
        return access == MMU_INST_FETCH ? INT_ITLB_MISS : INT_DTLB_ACCESS;
    }

    *prot = 0;
    if (pte & PTE_READABLE) {
        *prot |= PAGE_READ;
    }
    if (pte & PTE_WRITABLE) {
        *prot |= PAGE_WRITE;
    }
    if (pte & PTE_EXECUTABLE) {
        *prot |= PAGE_EXEC;
    }
    if (access == MMU_INST_FETCH && !(*prot & PAGE_EXEC)) {
        return INT_ITLB_MISS;
    }
    if (access == MMU_DATA_LOAD && !(*prot & PAGE_READ)) {
        return INT_DTLB_ACCESS;
    }
    if (access == MMU_DATA_STORE && !(*prot & PAGE_WRITE)) {
        return INT_DTLB_ACCESS;
    }

    /*
     * The hypervisor keeps ACCESSED and DIRTY.  A clean page is mapped
     * read-only, so the first store comes back here to dirty it.
     */
    upd = pte | PTE_ACCESSED;
    if (access == MMU_DATA_STORE) {
        upd |= PTE_DIRTY;
    } else if (!(pte & PTE_DIRTY)) {
        *prot &= ~PAGE_WRITE;
    }
    if (upd != pte) {
        address_space_stq_le(env_cpu(env)->as, pte_pa, upd,
                             MEMTXATTRS_UNSPECIFIED, NULL);
    }

    *pa = ((pte >> PTE_PTFN_SHIFT) << PT_ALIGN_SHIFT) &
          ~((1ULL << *log2_size) - 1);
    *pa |= va & ((1ULL << *log2_size) - 1);
    return -1;
}

bool tilegx_cpu_tlb_fill(CPUState *cs, vaddr address, int size,
                         MMUAccessType access_type, int mmu_idx,
                         bool probe, uintptr_t retaddr)
{
    CPUTLGState *env = cpu_env(cs);
    hwaddr pa = 0;
    int prot = 0, log2_size = 0, fault;

    fault = tilegx_translate(env, address, access_type, mmu_idx,
                             &pa, &prot, &log2_size);
    qemu_log_mask(CPU_LOG_MMU, "tilegx: tlb_fill %016" PRIx64 " %d idx %d -> "
                  "%d pa %016" PRIx64 " prot %x size %d\n", (uint64_t)address,
                  access_type, mmu_idx, fault, (uint64_t)pa, prot, log2_size);
    if (fault < 0) {
        /* The entry is for the page at ADDRESS; SIZE only tracks flushes. */
        tlb_set_page(cs, address & TARGET_PAGE_MASK, pa & TARGET_PAGE_MASK,
                     prot, mmu_idx, 1ULL << log2_size);
        return true;
    }
    if (probe) {
        return false;
    }

    env->fault_addr = address;
    env->fault_info = access_type == MMU_DATA_STORE;
    tilegx_raise_exception(env, fault, retaddr);
}

/* Only the atomics are translated with MO_ALIGN; the hardware traps. */
G_NORETURN void tilegx_cpu_do_unaligned_access(CPUState *cs, vaddr addr,
                                               MMUAccessType access_type,
                                               int mmu_idx, uintptr_t retaddr)
{
    CPUTLGState *env = cpu_env(cs);

    env->fault_addr = addr;
    tilegx_raise_exception(env, INT_UNALIGN_DATA, retaddr);
}

hwaddr tilegx_cpu_get_phys_addr_debug(CPUState *cs, vaddr addr)
{
    CPUTLGState *env = cpu_env(cs);
    hwaddr pa;
    int prot, log2_size;

    if (tilegx_translate(env, addr, MMU_DATA_LOAD, MMU_KERNEL_IDX,
                         &pa, &prot, &log2_size) >= 0) {
        return -1;
    }
    return pa;
}

void tilegx_flush_tlb(CPUTLGState *env)
{
    tlb_flush(env_cpu(env));
}
