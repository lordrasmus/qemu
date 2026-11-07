/*
 * bFLT relocations of the lm32 kernel (arch/lm32/include/asm/flat.h and
 * arch/lm32/kernel/flat.c): the top three bits of a relocation select a
 * 32-bit pointer, or the high or low half of an address in the immediate
 * of an mvhi/ori pair; type 3 entries carry the upper 16 bits of the next
 * target address.
 */

#ifndef LINUX_USER_LM32_TARGET_FLAT_H
#define LINUX_USER_LM32_TARGET_FLAT_H

#define FLAT_LM32_RELOC_TYPE_32_BIT     0
#define FLAT_LM32_RELOC_TYPE_HI16_BIT   1
#define FLAT_LM32_RELOC_TYPE_LO16_BIT   2
#define FLAT_LM32_RELOC_TYPE_PERSISTENT 3

#define flat_lm32_type(relval)          (((relval) >> 29) & 7)

#define flat_argvp_envp_on_stack()      0
#define flat_reloc_valid(reloc, size)   ((reloc) <= (size))
#define flat_old_ram_flag(flag)         (flag)
#define flat_get_relocate_addr(relval)  ((relval) & 0x1fffffff)

static inline int flat_set_persistent(abi_ulong relval, abi_ulong *persistent)
{
    if (flat_lm32_type(relval) == FLAT_LM32_RELOC_TYPE_PERSISTENT) {
        *persistent = relval << 16;
        return 1;
    }
    return 0;
}

static inline abi_ulong lm32_flat_get_addr(abi_ulong rp, abi_ulong val,
                                           abi_ulong relval,
                                           abi_ulong *persistent)
{
    uint16_t imm;

    switch (flat_lm32_type(relval)) {
    case FLAT_LM32_RELOC_TYPE_32_BIT:
        return val;
    case FLAT_LM32_RELOC_TYPE_HI16_BIT:
    case FLAT_LM32_RELOC_TYPE_LO16_BIT:
        if (get_user_u16(imm, rp + 2)) {
            return 0;
        }
        return imm + *persistent;
    default:
        return 0;
    }
}

/* The immediate lives in the instruction at rp, the caller's variable. */
#define flat_get_addr_from_rp(val, relval, flags, persistent) \
    lm32_flat_get_addr(rp, val, relval, persistent)

static inline int flat_put_addr_at_rp(abi_ulong rp, abi_ulong addr,
                                      abi_ulong relval)
{
    switch (flat_lm32_type(relval)) {
    case FLAT_LM32_RELOC_TYPE_HI16_BIT:
        return put_user_u16(addr >> 16, rp + 2);
    case FLAT_LM32_RELOC_TYPE_LO16_BIT:
        return put_user_u16(addr, rp + 2);
    default:
        return put_user_ual(addr, rp);
    }
}

#endif
