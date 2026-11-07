/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef H8300_TARGET_FLAT_H
#define H8300_TARGET_FLAT_H

/*
 * arch/h8300/include/asm/flat.h: addresses are 24 bits; the top byte of a
 * relocated word may be an opcode (jsr/jmp @aa:24) and is kept.
 */
#define flat_argvp_envp_on_stack()                           1
#define flat_reloc_valid(reloc, size)                        ((reloc) <= (size))
#define flat_old_ram_flag(flags)                             1
#define flat_get_relocate_addr(relval)                       ((relval) & ~1)
#define flat_get_addr_from_rp(rp, relval, flags, persistent) \
    ((rp) & (((flags) & FLAT_FLAG_GOTPIC) ? 0xffffffff : 0x00ffffff))
#define flat_set_persistent(relval, persistent)              ((void)(persistent), 0)

static inline int flat_put_addr_at_rp(abi_ulong rp, abi_ulong addr,
                                      abi_ulong relval)
{
    uint8_t top;

    if (get_user_u8(top, rp)) {
        return -TARGET_EFAULT;
    }
    return put_user_ual(((abi_ulong)top << 24) | (addr & 0x00ffffff), rp);
}

#endif
