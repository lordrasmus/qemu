/*
 * qemu user cpu loop for Kalray kvx
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu.h"
#include "user-internals.h"
#include "user/cpu_loop.h"
#include "signal-common.h"

static void kvx_hw_trap(CPUKVXState *env)
{
    uint64_t pc = kvx_register_read_u64(env, REG_kv3_PC);
    int htc = KVX_FIELD_EX64(env->excp_syndrome, kv3_ES, HTC);

    switch (htc) {
    case TRAP_DMISALIGN:
        force_sig_fault(TARGET_SIGBUS, TARGET_BUS_ADRALN, env->excp_address);
        break;
    case TRAP_NOMAPPING:
    case TRAP_PROTECTION:
    case TRAP_WRITETOCLEAN:
    case TRAP_ATOMICTOCLEAN:
        force_sig_fault(TARGET_SIGSEGV, TARGET_SEGV_MAPERR, env->excp_address);
        break;
    case TRAP_PRIVILEGE:
        force_sig_fault(TARGET_SIGILL, TARGET_ILL_PRVOPC, pc);
        break;
    default:
        force_sig_fault(TARGET_SIGILL, TARGET_ILL_ILLOPC, pc);
        break;
    }
}

void cpu_loop(CPUKVXState *env)
{
    CPUState *cs = env_cpu(env);
    int trapnr;
    abi_long ret;
    uint64_t pc;

    for (;;) {
        cpu_exec_start(cs);
        trapnr = cpu_exec(cs);
        cpu_exec_end(cs);
        qemu_process_cpu_events(cs);

        switch (trapnr) {
        case KVX_EXCP_SYSCALL:
            /* scall is always alone in its bundle (arch/kvx/kernel/signal.c) */
            pc = kvx_register_read_u64(env, REG_kv3_PC) + 4;
            kvx_register_write_u64(env, REG_kv3_PC, pc);
            ret = do_syscall(env,
                             KVX_FIELD_EX64(env->excp_syndrome, kv3_ES, SN),
                             *kvx_gpr(env, 0), *kvx_gpr(env, 1),
                             *kvx_gpr(env, 2), *kvx_gpr(env, 3),
                             *kvx_gpr(env, 4), *kvx_gpr(env, 5), 0, 0);
            if (ret == -QEMU_ERESTARTSYS) {
                kvx_register_write_u64(env, REG_kv3_PC, pc - 4);
            } else if (ret != -QEMU_ESIGRETURN && ret != -QEMU_ESETPC) {
                *kvx_gpr(env, 0) = ret;
            }
            break;
        case KVX_EXCP_HW_TRAP:
            kvx_hw_trap(env);
            break;
        case KVX_EXCP_DEBUG:
        case EXCP_DEBUG:
            force_sig_fault(TARGET_SIGTRAP, TARGET_TRAP_BRKPT,
                            kvx_register_read_u64(env, REG_kv3_PC));
            break;
        case EXCP_INTERRUPT:
            /* We processed the pending cpu work above.  */
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

    /*
     * User mode runs with 64-bit addressing and hardware loops enabled
     * (start_thread() inherits the kernel's PS, entry.S sets HLE).
     */
    kvx_register_write_field(env, PS, V64, 1);
    kvx_register_write_field(env, PS, HLE, 1);
    kvx_register_write_u64(env, REG_kv3_PC, info->entry);
    *kvx_gpr(env, 12) = info->start_stack;
}
