/*
 * TILE-Gx helpers for user mode emulation: the PL0 view of the
 * privileged instructions and SPRs.
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "cpu.h"
#include "exec/helper-proto.h"
#include "accel/tcg/cpu-loop.h"

#define SPR_EX_CONTEXT_0_0      0x2580
#define SPR_EX_CONTEXT_0_1      0x2581
#define SPR_ICS                 0x2708
#define SPR_CMPEXCH_VALUE       0x2780
#define SPR_CYCLE               0x2781

static G_NORETURN void raise_gpv(CPUTLGState *env, uintptr_t ra)
{
    CPUState *cs = env_cpu(env);

    cs->exception_index = INT_GPV;
    cpu_loop_exit_restore(cs, ra);
}

void helper_exception(CPUTLGState *env, uint32_t excp)
{
    CPUState *cs = env_cpu(env);

    cs->exception_index = excp;
    cpu_loop_exit(cs);
}

/* Linux lets PL0 use iret through EX_CONTEXT_0_*; uClibc's longjmp does. */
void helper_iret(CPUTLGState *env)
{
    env->pc = env->ex_context[0][0] & ~7ULL;
    env->ics = (env->ex_context[0][1] >> 2) & 1;
}

void helper_nap(CPUTLGState *env)
{
    CPUState *cs = env_cpu(env);

    env->pc += 8;
    cs->exception_index = EXCP_INTERRUPT;
    cpu_loop_exit(cs);
}

uint64_t helper_mfspr(CPUTLGState *env, uint32_t spr)
{
    switch (spr) {
    case SPR_EX_CONTEXT_0_0:
    case SPR_EX_CONTEXT_0_1:
        return env->ex_context[0][spr - SPR_EX_CONTEXT_0_0];
    case SPR_ICS:
        return env->ics;
    case SPR_CMPEXCH_VALUE:
        return env->cmpexch_value;
    case SPR_CYCLE:
        /* Nanoseconds: a cycle counter at 1 GHz */
        return get_clock();
    }
    raise_gpv(env, GETPC());
}

void helper_mtspr(CPUTLGState *env, uint32_t spr, uint64_t val)
{
    switch (spr) {
    case SPR_EX_CONTEXT_0_0:
        env->ex_context[0][0] = val;
        return;
    case SPR_EX_CONTEXT_0_1:
        env->ex_context[0][1] = val & 7;
        return;
    case SPR_ICS:
        env->ics = val & 1;
        return;
    case SPR_CMPEXCH_VALUE:
        env->cmpexch_value = val;
        return;
    }
    raise_gpv(env, GETPC());
}
