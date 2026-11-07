/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation, or (at your option) any
 * later version. See the COPYING file in the top-level directory.
 */

#ifndef META_TARGET_ELF_H
#define META_TARGET_ELF_H

#include "target_ptrace.h"

#define ELF_CLASS               ELFCLASS32
#define ELF_MACHINE             EM_METAG

/* arch/metag/kernel/user_gateway.S, mapped at USER_GATEWAY_PAGE */
#define COMMPAGE                0x6ffff000u
#define META_GATEWAY_GET_TLS    (COMMPAGE + 0x00)
#define META_GATEWAY_CMPXCHG    (COMMPAGE + 0x40)
#define META_GATEWAY_SWITCH     0xaf000000u     /* SWITCH #0 */

/* The stack grows upwards (CONFIG_STACK_GROWSUP). */
#define STACK_GROWS_DOWN        0

#endif
