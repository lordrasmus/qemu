/*
 * bFLT relocations of Linux/v850 (include/asm-v850/flat.h): the top four
 * bits of a relocation select a 32-bit word or a hi16s/lo16 (lo15)
 * pair, i.e. a movhi with the low half in the following instruction.
 * The pair is stored in target order, so it is handed back byte-swapped
 * to the generic code, which swaps relocated words from network order.
 */

#ifndef LINUX_USER_V850_TARGET_FLAT_H
#define LINUX_USER_V850_TARGET_FLAT_H

#define FLAT_V850_R_32          0 /* Normal 32-bit reloc */
#define FLAT_V850_R_HI16S_LO15  1 /* High 16-bits + signed 15-bit low field */
#define FLAT_V850_R_HI16S_LO16  2 /* High 16-bits + signed 16-bit low field */

#define flat_v850_type(relval)                  ((relval) >> 28)

#define flat_argvp_envp_on_stack()              0
/* base pointers may point up to 0x8000 past the image */
#define flat_reloc_valid(reloc, size)           ((reloc) <= (size) + 0x8000)
#define flat_old_ram_flag(flag)                 (flag)
#define flat_get_relocate_addr(relval)          ((relval) & 0x0fffffff)
#define flat_set_persistent(relval, persistent) ((void)(persistent), 0)

static inline abi_ulong v850_flat_get_addr(abi_ulong rp, abi_ulong val,
                                           abi_ulong relval)
{
    uint16_t hi, lo;

    switch (flat_v850_type(relval)) {
    case FLAT_V850_R_32:
        return val;
    case FLAT_V850_R_HI16S_LO16:
    case FLAT_V850_R_HI16S_LO15:
        if (get_user_u16(hi, rp) || get_user_u16(lo, rp + 4)) {
            return ~0;
        }
        if (flat_v850_type(relval) == FLAT_V850_R_HI16S_LO15) {
            lo &= ~1;
        }
        return bswap32(((uint32_t)hi << 16) + (int16_t)lo);
    default:
        return ~0;
    }
}

/* The second half lives in the instruction at rp, the caller's variable. */
#define flat_get_addr_from_rp(val, relval, flags, persistent) \
    v850_flat_get_addr(rp, val, relval)

static inline int flat_put_addr_at_rp(abi_ulong rp, abi_ulong addr,
                                      abi_ulong relval)
{
    uint16_t lo;

    switch (flat_v850_type(relval)) {
    case FLAT_V850_R_HI16S_LO16:
        lo = addr;
        break;
    case FLAT_V850_R_HI16S_LO15:
        if (get_user_u16(lo, rp + 4)) {
            return -1;
        }
        lo = (addr & 0xfffe) | (lo & 1);
        break;
    default:
        return put_user_ual(addr, rp);
    }
    if (put_user_u16((addr >> 16) + ((addr >> 15) & 1), rp)) {
        return -1;
    }
    return put_user_u16(lo, rp + 4);
}

#endif
