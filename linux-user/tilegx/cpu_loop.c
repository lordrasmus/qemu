/*
 *  qemu user cpu loop
 *
 *  Copyright (c) 2003-2008 Fabrice Bellard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qemu.h"
#include "user-internals.h"
#include "user/cpu_loop.h"
#include "signal-common.h"
#include "opcode_tilegx.h"

#define TILEGX_BPT_BUNDLE 0x286a44ae51485000ULL

/*
 * arch/tile/kernel/traps.c special_ill(): "bpt", or a "raise" bundle
 * that names the signal and si_code in an addli of X0.
 */
static bool special_ill(uint64_t bundle, int *sig, int *code)
{
    int maxcode;

    if (bundle == TILEGX_BPT_BUNDLE) {
        *sig = TARGET_SIGTRAP;
        *code = TARGET_TRAP_BRKPT;
        return true;
    }
    if ((bundle & TILEGX_BUNDLE_MODE_MASK) != 0 ||
        get_Opcode_X1(bundle) != RRR_0_OPCODE_X1 ||
        get_RRROpcodeExtension_X1(bundle) != UNARY_RRR_0_OPCODE_X1 ||
        get_UnaryOpcodeExtension_X1(bundle) != ILL_UNARY_OPCODE_X1 ||
        get_Dest_X1(bundle) != 29 || get_SrcA_X1(bundle) != 37 ||
        get_Opcode_X0(bundle) != ADDLI_OPCODE_X0 ||
        get_Dest_X0(bundle) != TILEGX_R_ZERO ||
        get_SrcA_X0(bundle) != TILEGX_R_ZERO) {
        return false;
    }

    switch (get_Imm16_X0(bundle) & 0x3f) {
    case TARGET_SIGILL:
        *sig = TARGET_SIGILL;
        maxcode = 10;
        break;
    case TARGET_SIGFPE:
        *sig = TARGET_SIGFPE;
        maxcode = 8;
        break;
    case TARGET_SIGSEGV:
        *sig = TARGET_SIGSEGV;
        maxcode = 2;
        break;
    case TARGET_SIGBUS:
        *sig = TARGET_SIGBUS;
        maxcode = 3;
        break;
    case TARGET_SIGTRAP:
        *sig = TARGET_SIGTRAP;
        maxcode = 4;
        break;
    default:
        return false;
    }
    *code = (get_Imm16_X0(bundle) >> 6) & 0xf;
    return *code > 0 && *code <= maxcode;
}

void cpu_loop(CPUTLGState *env)
{
    CPUState *cs = env_cpu(env);
    int trapnr, sig, code;
    uint64_t bundle;
    abi_long ret;

    while (1) {
        cpu_exec_start(cs);
        trapnr = cpu_exec(cs);
        cpu_exec_end(cs);
        qemu_process_cpu_events(cs);

        switch (trapnr) {
        case INT_SWINT_1:
            /* The PC is that of the swint1 bundle. */
            env->pc += 8;
            ret = do_syscall(env, env->regs[TILEGX_R_NR],
                             env->regs[0], env->regs[1],
                             env->regs[2], env->regs[3],
                             env->regs[4], env->regs[5],
                             0, 0);
            if (ret == -QEMU_ERESTARTSYS) {
                env->pc -= 8;
            } else if (ret != -QEMU_ESIGRETURN && ret != -QEMU_ESETPC) {
                /* -errno in r0, the positive errno in r1 */
                env->regs[TILEGX_R_RE] = ret;
                env->regs[TILEGX_R_ERR] =
                    (abi_ulong)ret >= (abi_ulong)-4095 ? -ret : 0;
            }
            break;
        case INT_SWINT_0:
        case INT_SWINT_2:
        case INT_SWINT_3:
            force_sig_fault(TARGET_SIGILL, TARGET_ILL_ILLTRP, env->pc);
            break;
        case INT_ILL:
            if (get_user_u64(bundle, env->pc) ||
                !special_ill(bundle, &sig, &code)) {
                sig = TARGET_SIGILL;
                code = TARGET_ILL_ILLOPC;
            }
            force_sig_fault(sig, code, env->pc);
            break;
        case INT_GPV:
        case INT_IDN_ACCESS:
        case INT_UDN_ACCESS:
            force_sig_fault(TARGET_SIGILL, TARGET_ILL_PRVREG, env->pc);
            break;
        case EXCP_INTERRUPT:
            /* just indicate that signals should be handled asap */
            break;
        case EXCP_DEBUG:
            force_sig_fault(TARGET_SIGTRAP, TARGET_TRAP_BRKPT, env->pc);
            break;
        case EXCP_ATOMIC:
            cpu_exec_step_atomic(cs);
            break;
        default:
            fprintf(stderr, "Unhandled trap: 0x%x\n", trapnr);
            cpu_dump_state(cs, stderr, 0);
            exit(EXIT_FAILURE);
        }
        process_pending_signals(env);
    }
}

void init_main_thread(CPUState *cs, struct image_info *info)
{
    CPUArchState *env = cpu_env(cs);

    env->regs[TILEGX_R_SP] = info->start_stack;
    env->pc = info->entry;
}
