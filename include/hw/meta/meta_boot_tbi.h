#ifndef _META_BOOT_TBI_H
#define _META_BOOT_TBI_H

/* Segment types */
#define SEG_TYPE_TEXT       2
#define SEG_TYPE_DATA       4
#define SEG_TYPE_STACK      6
#define SEG_TYPE_HEAP       10
#define SEG_TYPE_ROOT       12
#define SEG_TYPE_STRING     14

/* Segment scopes */
#define SEG_SCOPE_INIT      0
#define SEG_SCOPE_LOCAL     1
#define SEG_SCOPE_GLOBAL    2
#define SEG_SCOPE_SHARED    2

static inline int meta_tbi_seg_id(unsigned int thread, unsigned int scope,
                                  unsigned int type)
{
    return thread << 26 | scope << 4 | type;
}

struct meta_mmu_seg;

target_ulong meta_tbi_setup(CPUArchState *env, target_ulong *brk, hwaddr v2p);
target_ulong meta_tbi_seg_setup(CPUArchState *env, target_ulong *brk,
                                hwaddr v2p, struct meta_mmu_seg *l_segs,
                                struct meta_mmu_seg *g_segs);

#endif /* _META_TBI_H */
