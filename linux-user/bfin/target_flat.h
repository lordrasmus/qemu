/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef BFIN_TARGET_FLAT_H
#define BFIN_TARGET_FLAT_H

/*
 * arch/blackfin/include/asm/flat.h and kernel/flat.c: the top bits of
 * a relocation give its type, 16-bit halves take their other half from
 * a preceding "persistent" entry.  The value is passed in as read by
 * get_user_ual().
 */
#define FLAT_BFIN_RELOC_TYPE_16_BIT     0
#define FLAT_BFIN_RELOC_TYPE_16H_BIT    1
#define FLAT_BFIN_RELOC_TYPE_32_BIT     2

#define flat_argvp_envp_on_stack()              0
#define flat_reloc_valid(reloc, size)           ((reloc) <= (size))
#define flat_old_ram_flag(flag)                 (flag)
#define flat_get_relocate_addr(relval)          ((relval) & 0x03ffffff)

static inline int flat_set_persistent(abi_ulong relval, abi_ulong *persistent)
{
    if (((relval >> 26) & 7) == 3) {
        *persistent = relval << 16;
        return 1;
    }
    return 0;
}

static inline abi_ulong flat_get_addr_from_rp(abi_ulong val, abi_ulong relval,
                                              abi_ulong flags,
                                              abi_ulong *persistent)
{
    switch ((relval >> 26) & 7) {
    case FLAT_BFIN_RELOC_TYPE_16_BIT:
    case FLAT_BFIN_RELOC_TYPE_16H_BIT:
        val = (val & 0xffff) + *persistent;
        break;
    case FLAT_BFIN_RELOC_TYPE_32_BIT:
        break;
    default:
        return 0;
    }
    /* flatload.c swaps it back */
    if ((flags & FLAT_FLAG_GOTPIC) == 0) {
        val = htonl(val);
    }
    return val;
}

static inline int flat_put_addr_at_rp(abi_ulong rp, abi_ulong addr,
                                      abi_ulong relval)
{
    switch ((relval >> 26) & 7) {
    case FLAT_BFIN_RELOC_TYPE_16_BIT:
        return put_user_u16(addr, rp);
    case FLAT_BFIN_RELOC_TYPE_16H_BIT:
        return put_user_u16(addr >> 16, rp);
    case FLAT_BFIN_RELOC_TYPE_32_BIT:
        return put_user_ual(addr, rp);
    }
    return 0;
}

#endif
