/*
 *  qemu user cpu loop
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

/* asm/siginfo.h of Blackfin */
#define BFIN_ILL_ILLPARAOP      2
#define BFIN_TRAP_ILLTRAP       4

/*
 * The atomic sequences of the fixed code page, one per 16 bytes from
 * ATOMIC_XCHG32 on.  P0 is the address; see fixed_code.S for the
 * registers.  Returns false on a fault.
 */
static bool bfin_atomic_seq(CPUBfinState *env, int seq)
{
    CPUState *cs = env_cpu(env);
    abi_ulong addr = env->reg[BFIN_P0];
    uint32_t *p, old, val, new;

    if (addr & 3) {
        force_sig_fault(TARGET_SIGBUS, TARGET_BUS_ADRALN, addr);
        return false;
    }
    if (!access_ok(cs, VERIFY_WRITE, addr, 4)) {
        force_sig_fault(TARGET_SIGSEGV, TARGET_SEGV_MAPERR, addr);
        return false;
    }
    p = g2h(cs, addr);

    old = qatomic_read(p);
    do {
        val = le32_to_cpu(old);
        switch (seq) {
        case 0:                                 /* xchg32 */
            new = env->reg[BFIN_R0 + 1];
            break;
        case 1:                                 /* cas32 */
            if (val != env->reg[BFIN_R0 + 1]) {
                env->reg[BFIN_R0] = val;
                return true;
            }
            new = env->reg[BFIN_R0 + 2];
            break;
        case 2:                                 /* add32 */
            new = val + env->reg[BFIN_R0];
            break;
        case 3:                                 /* sub32 */
            new = val - env->reg[BFIN_R0];
            break;
        case 4:                                 /* ior32 */
            new = val | env->reg[BFIN_R0];
            break;
        case 5:                                 /* and32 */
            new = val & env->reg[BFIN_R0];
            break;
        default:                                /* xor32 */
            new = val ^ env->reg[BFIN_R0];
            break;
        }
    } while ((old = qatomic_cmpxchg(p, old, cpu_to_le32(new))) !=
             cpu_to_le32(val));

    if (seq <= 1) {
        env->reg[BFIN_R0] = val;
    } else {
        env->reg[BFIN_R0] = new;
        env->reg[BFIN_R0 + 1] = val;
    }
    return true;
}

/* EXCPT 15 in page zero: an atomic sequence or a stray jump */
static void bfin_commpage(CPUBfinState *env)
{
    abi_ulong insn = env->pc - 2;

    if (insn >= BFIN_ATOMIC_SEQS_START && insn < BFIN_ATOMIC_SEQS_END &&
        (insn & 15) == 0) {
        env->pc = insn;
        if (bfin_atomic_seq(env, (insn - BFIN_ATOMIC_SEQS_START) / 16)) {
            env->pc = env->reg[BFIN_RETS];
        }
        return;
    }
    env->pc = insn;
    force_sig_fault(TARGET_SIGSEGV, TARGET_SEGV_MAPERR, insn);
}

/* arch/blackfin/kernel/traps.c */
static void bfin_exception(CPUBfinState *env)
{
    int excause = env->reg[BFIN_SEQSTAT] & 0x3f;
    abi_long ret;

    switch (excause) {
    case 0:                                     /* system call */
        ret = do_syscall(env, env->reg[BFIN_P0],
                         env->reg[BFIN_R0], env->reg[BFIN_R0 + 1],
                         env->reg[BFIN_R0 + 2], env->reg[BFIN_R0 + 3],
                         env->reg[BFIN_R0 + 4], env->reg[BFIN_R0 + 5],
                         0, 0);
        if (ret == -QEMU_ERESTARTSYS) {
            env->pc -= 2;
        } else if (ret != -QEMU_ESIGRETURN && ret != -QEMU_ESETPC) {
            env->reg[BFIN_R0] = ret;
        }
        break;
    case 1:
    case 2:
        force_sig_fault(TARGET_SIGTRAP, BFIN_TRAP_ILLTRAP, env->pc);
        break;
    case 3:
        force_sig_fault(TARGET_SIGSEGV, TARGET_SEGV_MAPERR, env->pc);
        break;
    case 15:
        if (env->pc - 2 < TARGET_PAGE_SIZE) {
            bfin_commpage(env);
            break;
        }
        /* fall through */
    case 4 ... 14:
    case 0x22:                                  /* illegal combination */
        force_sig_fault(TARGET_SIGILL, BFIN_ILL_ILLPARAOP, env->pc);
        break;
    case 0x21:                                  /* undefined instruction */
        force_sig_fault(TARGET_SIGILL, TARGET_ILL_ILLOPC, env->pc);
        break;
    case 0x24:                                  /* misaligned data */
        force_sig_fault(TARGET_SIGBUS, TARGET_BUS_ADRALN,
                        env->dcplb_fault_addr);
        break;
    case 0x2a:                                  /* misaligned fetch */
        force_sig_fault(TARGET_SIGBUS, TARGET_BUS_ADRALN,
                        env->icplb_fault_addr);
        break;
    case 0x2e:                                  /* supervisor resource */
        force_sig_fault(TARGET_SIGILL, TARGET_ILL_PRVOPC, env->pc);
        break;
    default:
        force_sig_fault(TARGET_SIGILL, TARGET_ILL_ILLOPC, env->pc);
        break;
    }
}

void cpu_loop(CPUBfinState *env)
{
    CPUState *cs = env_cpu(env);
    int trapnr;

    while (1) {
        cpu_exec_start(cs);
        trapnr = cpu_exec(cs);
        cpu_exec_end(cs);
        qemu_process_cpu_events(cs);

        switch (trapnr) {
        case EXCP_INTERRUPT:
            /* just indicate that signals should be handled asap */
            break;
        case EXCP_BFIN_EXCEPTION:
            bfin_exception(env);
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

    /* user mode: no event pending */
    env->ipend = 0;
    env->reg[BFIN_SP] = info->start_stack;
    /* start_thread() hands the data segment to FLAT programs in P5 */
    env->reg[BFIN_P0 + 5] = info->start_data;
    env->pc = info->entry;
}
