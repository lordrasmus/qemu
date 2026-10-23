/*
 * QEMU Kalray kvx CPU
 *
 * Copyright (c) 2020 GreenSocs SAS
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "accel/tcg/cpu-loop.h"
#include "exec/helper-proto.h"
#include "cpu.h"
#include "exec/cputlb.h"
#include "exec/tlb-flags.h"
#include "internal.h"

/* Virtual addresses are 41 bits sign extended */
static inline uint64_t sign_extend_vaddr(uint64_t vaddr)
{
    return ((int64_t) vaddr)
        << (64 - TARGET_VIRT_ADDR_SPACE_BITS)
        >> (64 - TARGET_VIRT_ADDR_SPACE_BITS);
}

int kvx_cpu_mmu_index(CPUKVXState *env, bool ifetch)
{
    int vs, mmu_idx = 0;
    bool mmup, mme;
    uint64_t ps = get_mmu_ps(env, ifetch);

    mme = KVX_FIELD_EX64(ps, kv3_PS, MME);
    mmu_idx = FIELD_DP64(mmu_idx, TB_STATE, MMU_IDX_MME, mme);

    if (!mme) {
        /* MMU disabled. Do not take into account MMU related flags. */
        return mmu_idx;
    }

    mmup = KVX_FIELD_EX64(ps, kv3_PS, MMUP);
    vs = KVX_FIELD_EX64(ps, kv3_PS, VS);

    mmu_idx = FIELD_DP64(mmu_idx, TB_STATE, MMU_IDX_MMUP, mmup);
    mmu_idx = FIELD_DP64(mmu_idx, TB_STATE, MMU_IDX_VS, vs);

    return mmu_idx;
}

static inline bool tlb_entry_is_valid(const KVXTLBEntry *entry)
{
    return KVX_FIELD_EX64(entry->low, kv3_TEL, ES) != TLB_ES_INVALID;
}

static bool tlb_entry_match(CPUKVXState *env, const KVXTLBEntry *entry,
                            target_ulong vaddr, uint64_t ps)
{
    uint8_t page_size;
    target_ulong page_mask;
    uint64_t cur_asn;
    uint64_t vs;
    target_ulong entry_vaddr;

    if (!tlb_entry_is_valid(entry)) {
        return false;
    }

    entry_vaddr = sign_extend_vaddr(entry->high & KVX_FIELD_MASK(kv3_TEH, PN));

    page_size = KVX_FIELD_EX64(entry->low, kv3_TEL, PS);
    page_mask = MMU_PAGE_SIZE_MASK[page_size];

    if (entry_vaddr != (vaddr & page_mask)) {
        return false;
    }

    vs = KVX_FIELD_EX64(ps, kv3_PS, VS);

    if (KVX_FIELD_EX64(entry->high, kv3_TEH, VS) != vs) {
        return false;
    }

    cur_asn = kvx_register_read_field(env, MMC, ASN);

    if ((!KVX_FIELD_EX64(entry->high, kv3_TEH, G))
        && KVX_FIELD_EX64(entry->high, kv3_TEH, ASN) != cur_asn) {
        return false;
    }

    return true;
}

static inline KVXTLBEntry *find_tlb_entry_in_set(CPUKVXState *env, KVXTLBEntry *set,
                                                 size_t num_ways, target_ulong vaddr,
                                                 uint64_t ps, size_t *way)
{
    size_t i;

    for (i = 0; i < num_ways; i++) {
        KVXTLBEntry *entry = &set[i];

        if (tlb_entry_match(env, entry, vaddr, ps)) {
            if (way) {
                *way = i;
            }

            return entry;
        }
    }

    return NULL;
}

static inline KVXTLBEntry *find_ltlb_entry(CPUKVXState *env, target_ulong vaddr,
                                           uint64_t ps, size_t *way)
{
    return find_tlb_entry_in_set(env, env->ltlb, LTLB_WAYS, vaddr, ps, way);
}

static inline size_t vaddr_to_jtlb_set(target_ulong vaddr, TLBPageSize page_size)
{
    return (vaddr >> MMU_PAGE_SHIFT[page_size]) & 0x3f;
}

static inline KVXTLBEntry *find_jtlb_entry(CPUKVXState *env, target_ulong vaddr,
                                           uint64_t ps, bool ignore_pmj,
                                           size_t *rset, size_t *way)
{
    KVXTLBEntry *entry;
    uint64_t pmj;
    size_t set;
    int i;

    if (ignore_pmj) {
        /* All page sizes enabled during lookup */
        pmj = (1 << KVX_FIELD_LENGTH(kv3_PS, PMJ)) - 1;
    } else {
        pmj = KVX_FIELD_EX64(ps, kv3_PS, PMJ);
    }

    for (i = 0; i < (1 << KVX_FIELD_LENGTH(kv3_TEL, PS)); i++) {
        if (!(pmj & (1 << i))) {
            /* This page size is disabled in PS.PMJ */
            continue;
        }

        set = vaddr_to_jtlb_set(vaddr, i);
        entry = find_tlb_entry_in_set(env, env->jtlb[set], JTLB_WAYS,
                                      vaddr, ps, way);

        if (entry) {
            if (rset) {
                *rset = set;
            }
            return entry;
        }
    }

    return NULL;
}

static KVXTLBEntry *find_tlb_entry(CPUKVXState *env, target_ulong vaddr,
                                   uint64_t ps, bool ignore_pmj, bool *buffer,
                                   size_t *set, size_t *way)
{
    KVXTLBEntry *entry = find_ltlb_entry(env, vaddr, ps, way);

    if (entry) {
        if (buffer) {
            *buffer = true;
        }
        if (set) {
            *set = 0;
        }
        return entry;
    }

    if (buffer) {
        *buffer = false;
    }

    return find_jtlb_entry(env, vaddr, ps, ignore_pmj, set, way);
}

TLBLookupStatus get_physical_addr(CPUKVXState *env, target_ulong vaddr,
                                  MMUAccessType access_type,
                                  int mmu_idx, TLBLookupInfo *info)
{
    KVXTLBEntry *entry;
    uint64_t r_ps;
    uint64_t ps, pa, es, cp;
    bool mmup, ifetch, v64;

    memset(&info->attrs, 0, sizeof(info->attrs));
    info->not_a_modified = false;
    info->device = false;

    ifetch = (access_type == MMU_INST_FETCH);
    r_ps = get_mmu_ps(env, ifetch);
    v64 = KVX_FIELD_EX64(r_ps, kv3_PS, V64);

    if (!v64) {
        /* 32 bits mode, mask out virtual address MSBs */
        vaddr &= 0xffffffffull;
    }

    if (!KVX_FIELD_EX64(r_ps, kv3_PS, MME)) {
        /* MMU disabled */

        if (vaddr >= 1 * TiB) {
            info->fault = access_type == MMU_INST_FETCH ?
                TRAP_PSYSERROR : TRAP_DSYSERROR;
            return TLB_LOOKUP_FAIL;
        }

        info->paddr = vaddr;
        info->page_size = TARGET_PAGE_SIZE;
        info->prot = PAGE_READ | PAGE_WRITE | PAGE_EXEC;
        info->cache_policy = 0;

        return TLB_LOOKUP_SUCCESS;
    }

    if (v64) {
        /* 41 bits sign extension */
        vaddr = sign_extend_vaddr(vaddr);
    }

    entry = find_tlb_entry(env, vaddr, r_ps, false, NULL, NULL, NULL);

    if (entry == NULL) {
        info->fault = TRAP_NOMAPPING;
        return TLB_LOOKUP_FAIL;
    }

    ps = KVX_FIELD_EX64(entry->low, kv3_TEL, PS);
    pa = KVX_FIELD_EX64(entry->low, kv3_TEL, PA);
    cp = KVX_FIELD_EX64(entry->low, kv3_TEL, CP);
    mmup = FIELD_EX64(mmu_idx, TB_STATE, MMU_IDX_MMUP);

    info->paddr = (entry->low & KVX_FIELD_MASK(kv3_TEL, FN))
        | (vaddr & ~MMU_PAGE_SIZE_MASK[ps]);

    info->page_size = MMU_PAGE_SIZE[ps];
    info->prot = MMU_PROTECTION_ATTRS[mmup][pa];
    info->cache_policy = cp;

    if (!(info->prot & (1 << access_type))) {
        info->fault = TRAP_PROTECTION;
        return TLB_LOOKUP_FAIL;
    }

    es = KVX_FIELD_EX64(entry->low, kv3_TEL, ES);

    if (es < TLB_ES_MODIFIED) {
        info->prot &= ~PAGE_WRITE;
    }

    if (!(info->prot & (1 << access_type))) {
        info->fault = TRAP_WRITETOCLEAN;
        return TLB_LOOKUP_FAIL;
    }

    /*
     * We can't detect A_MODIFIED traps here since we don't know if the access
     * is atomic. The state is kept in the QEMU TLB entry so that the
     * check_atomic_access helper can raise the trap if needed.
     */
    info->not_a_modified = (es != TLB_ES_A_MODIFIED);

    /* Device pages don't allow unaligned accesses */
    info->device = (cp == CP_D_DEVICE_I_UNCACHED);

    return TLB_LOOKUP_SUCCESS;
}

static void fill_tlb_excp_syndrome_fetch_side(CPUKVXState *env)
{
    env->excp_syndrome = KVX_FIELD_DP64(env->excp_syndrome, kv3_ES_PL0, RWX, ES_RWX_FETCH_SIDE);

    /* BS is unconditionally set to 1 on fetch side fault */
    env->excp_syndrome = KVX_FIELD_DP64(env->excp_syndrome, kv3_ES_PL0, BS, 1);
}

void fill_tlb_excp_syndrome(CPUKVXState *env, target_ulong fault_addr,
                            MMUAccessType access_type, int fault)
{
    static const int ACCESS_TYPE_TO_PTC_MAPPING[] = {
        [MMU_DATA_LOAD]  = 1,
        [MMU_DATA_STORE] = 2,
        [MMU_INST_FETCH] = 3
    };

    uint64_t ps = get_mmu_ps(env, access_type == MMU_INST_FETCH);

    if (!KVX_FIELD_EX64(ps, kv3_PS, V64)) {
        fault_addr &= 0xffffffffull;
    }

    env->excp_address = fault_addr;
    env->excp_syndrome = KVX_FIELD_DP64(env->excp_syndrome, kv3_ES_PL0,
                                    HTC, fault);

    if (access_type == MMU_INST_FETCH) {
        fill_tlb_excp_syndrome_fetch_side(env);
    }
    /*
     * When the fault is on the data side, all fields are already filled in
     * env->excp_syndrome. See insn_syndrome_pack/unpack
     */

    if (fault == TRAP_PROTECTION) {
        kvx_register_write_field(env, MMC, PTC,
                                 ACCESS_TYPE_TO_PTC_MAPPING[access_type]);
    }

    if (fault == TRAP_PROTECTION || fault == TRAP_NOMAPPING) {
        kvx_register_write_field(env, MMC, S,
                                 KVX_FIELD_EX64(env->excp_syndrome, kv3_ES, NTA));
    }
}

bool kvx_cpu_tlb_fill(CPUState *cs, target_ulong vaddr, int size,
                     MMUAccessType access_type, int mmu_idx,
                     bool probe, uintptr_t retaddr)
{
    KVXCPU *cpu = KVX_CPU(cs);
    CPUKVXState *env = &cpu->env;
    TLBLookupStatus ret;
    TLBLookupInfo info;

    ret = get_physical_addr(env, vaddr, access_type, mmu_idx, &info);

    if (ret == TLB_LOOKUP_SUCCESS) {
        CPUTLBEntryFull full = {
            .phys_addr = info.paddr & TARGET_PAGE_MASK,
            .attrs = info.attrs,
            .prot = info.prot,
            .lg_page_size = ctz64(info.page_size),
            /* memory accesses use MO_ALIGN_TLB_ONLY */
            .tlb_fill_flags = info.device ? TLB_CHECK_ALIGNED : 0,
            .extra.kvx.not_a_modified = info.not_a_modified,
        };

        tlb_set_page_full(cs, mmu_idx, vaddr & TARGET_PAGE_MASK, &full);
        return true;
    }

    if (probe) {
        return false;
    }

    if (!cpu_restore_state(cs, retaddr)) {
        /* Start with a fresh syndrome */
        env->excp_syndrome = 0;
    }

    fill_tlb_excp_syndrome(env, vaddr, access_type, info.fault);
    kvx_raise_exception(env, KVX_EXCP_HW_TRAP, 0);
}

static inline void G_NORETURN raise_misaligned_excp(CPUState *cs, vaddr addr,
                                                       MMUAccessType access_type,
                                                       uintptr_t retaddr)
{
    KVXCPU *cpu = KVX_CPU(cs);
    CPUKVXState *env = &cpu->env;

    cpu_restore_state(cs, retaddr);
    fill_tlb_excp_syndrome(env, addr, access_type, TRAP_DMISALIGN);
    kvx_raise_exception(env, KVX_EXCP_HW_TRAP, 0);
}

/*
 * Only DEVICE pages trap on unaligned accesses: their QEMU TLB entries carry
 * TLB_CHECK_ALIGNED and memory accesses use MO_ALIGN_TLB_ONLY, so we only get
 * here for such a page.  If the access spans two pages, faults on the second
 * page have priority over DMISALIGN.
 */
void kvx_cpu_do_unaligned_access(CPUState *cs, vaddr addr,
                                 MMUAccessType access_type, int mmu_idx,
                                 uintptr_t retaddr)
{
    KVXCPU *cpu = KVX_CPU(cs);
    CPUKVXState *env = &cpu->env;
    TLBLookupInfo info;
    int access_size;

    /*
     * Note: unaligned atomic accesses are caught by the check_atomic_access
     * helper and do not reach this function.
     */
    if (get_physical_addr(env, addr, access_type, mmu_idx, &info)
        == TLB_LOOKUP_SUCCESS) {
        uint64_t page_mask = ~((uint64_t)info.page_size - 1);

        cpu_restore_state(cs, retaddr);
        access_size = KVX_FIELD_EX64(env->excp_syndrome, kv3_ES, AS);
        if (access_size == 0x3f) {
            /* dzerol special case */
            access_size = 64;
        }
        if (-(addr | page_mask) < access_size) {
            vaddr next_page_addr = (addr & page_mask) + info.page_size;

            if (get_physical_addr(env, next_page_addr, access_type, mmu_idx,
                                  &info) == TLB_LOOKUP_FAIL) {
                kvx_cpu_tlb_fill(cs, next_page_addr, 1, access_type, mmu_idx,
                                 false, retaddr);
            }
        }
    }
    raise_misaligned_excp(cs, addr, access_type, retaddr);
}

static inline void invalidate_entry(CPUState *cpu, const KVXTLBEntry *entry)
{
    target_ulong vaddr = entry->high & KVX_FIELD_MASK(kv3_TEH, PN);

    tlb_flush_page(cpu, vaddr);
}

/* Check that TEH.PN and TEL.FN are aligned on the page size */
static inline bool check_tlb_entry(uint64_t tel, uint64_t teh)
{
    uint8_t page_size;
    uint64_t pn, fn, n_page_mask;

    page_size = KVX_FIELD_EX64(tel, kv3_TEL, PS);
    n_page_mask = ~MMU_PAGE_SIZE_MASK[page_size];
    pn = teh & KVX_FIELD_MASK(kv3_TEH, PN);
    fn = tel & KVX_FIELD_MASK(kv3_TEL, FN);

    return (!(pn & n_page_mask)) && (!(fn & n_page_mask));
}

void HELPER(tlb_write)(CPUKVXState *env)
{
    KVXTLBEntry *entry;
    size_t way = kvx_register_read_field(env, MMC, SW);
    uint64_t tel = kvx_register_read_u64(env, REG_kv3_TEL);
    uint64_t teh = kvx_register_read_u64(env, REG_kv3_TEH);
    size_t set;

    if (!check_tlb_entry(tel, teh)) {
        /* error condition */
        kvx_register_write_field(env, MMC, E, 1);
        return;
    }

    if (kvx_register_read_field(env, MMC, SB)) {
        /* LTLB write */
        entry = &env->ltlb[way];
        set = 0;
    } else {
        /* JTLB write */
        if (way > 3) {
            /* error condition */
            kvx_register_write_field(env, MMC, E, 1);
            return;
        }

        TLBPageSize page_size = KVX_FIELD_EX64(tel, kv3_TEL, PS);
        target_ulong vaddr = teh & KVX_FIELD_MASK(kv3_TEH, PN);
        set = vaddr_to_jtlb_set(vaddr, page_size);

        entry = &env->jtlb[set][way];
    }

    if (tlb_entry_is_valid(entry)) {
        invalidate_entry(env_cpu(env), entry);
    }

    entry->low = tel;
    entry->high = teh;

    kvx_register_write_field(env, MMC, E, 0);
    kvx_register_write_field(env, MMC, SS, set);
}

void HELPER(tlb_read)(CPUKVXState *env)
{
    KVXTLBEntry *entry;
    size_t way = kvx_register_read_field(env, MMC, SW);
    size_t set = kvx_register_read_field(env, MMC, SS);;

    kvx_register_write_field(env, MMC, PAR, 0);

    if (kvx_register_read_field(env, MMC, SB)) {
        /* LTLB read */
        if (set) {
            /* error condition */
            kvx_register_write_field(env, MMC, E, 1);
            return;
        }

        entry = &env->ltlb[way];
    } else {
        /* JTLB read */
        if (way > 3) {
            /* error condition */
            kvx_register_write_field(env, MMC, E, 1);
            return;
        }

        entry = &env->jtlb[set][way];
    }

    if (tlb_entry_is_valid(entry)) {
        invalidate_entry(env_cpu(env), entry);
    }

    kvx_register_write_field(env, MMC, E, 0);
    kvx_register_write_u64(env, REG_kv3_TEL, entry->low);
    kvx_register_write_u64(env, REG_kv3_TEH, entry->high);
}

void HELPER(tlb_probe)(CPUKVXState *env)
{
    KVXTLBEntry *entry;
    uint64_t ps, pn;
    bool buffer;
    size_t set, way;

    kvx_register_write_field(env, MMC, PAR, 0);

    ps = get_mmu_ps(env, false);
    pn = kvx_register_read_u64(env, REG_kv3_TEH) & KVX_FIELD_MASK(kv3_TEH, PN);
    pn = sign_extend_vaddr(pn);
    entry = find_tlb_entry(env, pn, ps, true, &buffer, &set, &way);

    if (!entry) {
        /* Probe failed */
        kvx_register_write_field(env, MMC, E, 1);
        return;
    }

    kvx_register_write_field(env, MMC, E, 0);
    kvx_register_write_field(env, MMC, SB, buffer);
    kvx_register_write_field(env, MMC, SS, set);
    kvx_register_write_field(env, MMC, SW, way);
    kvx_register_write_u64(env, REG_kv3_TEL, entry->low);
    kvx_register_write_u64(env, REG_kv3_TEH, entry->high);
}

hwaddr kvx_cpu_get_phys_page_debug(CPUState *cs, vaddr addr)
{
    KVXCPU *cpu = KVX_CPU(cs);
    CPUKVXState *env = &cpu->env;
    int mmu_idx;
    TLBLookupStatus sta;
    TLBLookupInfo info;

    mmu_idx = kvx_cpu_mmu_index(env, false);
    sta = get_physical_addr(env, addr, MMU_DATA_LOAD, mmu_idx, &info);

    if (sta != TLB_LOOKUP_SUCCESS) {
        return -1;
    }

    return info.paddr;
}
