/*
 * NEC V850E1 helpers
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/helper-proto.h"
#include "accel/tcg/cpu-ldst.h"
#include "accel/tcg/cpu-loop.h"

void HELPER(raise_exception)(CPUV850State *env, uint32_t index)
{
    CPUState *cs = env_cpu(env);

    cs->exception_index = index;
    cpu_loop_exit(cs);
}

/* pc already points past the TRAP */
void HELPER(trap)(CPUV850State *env, uint32_t vector)
{
    CPUState *cs = env_cpu(env);

    if (vector == 31 && env->sim_syscall) {
        env->sim_syscall(env);
        return;
    }
    env->excp_code = 0x40 + vector;
    cs->exception_index = EXCP_TRAP;
    cpu_loop_exit(cs);
}

void HELPER(halt)(CPUV850State *env)
{
    CPUState *cs = env_cpu(env);

    cs->halted = 1;
    cs->exception_index = EXCP_HLT;
    cpu_loop_exit(cs);
}

void HELPER(ldsr)(CPUV850State *env, uint32_t id, uint32_t val)
{
    switch (id) {
    case SR_PSW:
        v850_psw_write(env, val);
        break;
    case SR_ECR:
        break;
    case SR_EIPC:
    case SR_FEPC:
    case SR_CTPC:
    case SR_DBPC:
        env->sr[id] = val & ~1;
        break;
    case SR_EIPSW:
    case SR_FEPSW:
    case SR_CTPSW:
    case SR_DBPSW:
        env->sr[id] = val & PSW_MASK;
        break;
    case SR_CTBP:
        env->sr[id] = val & ~1;
        break;
    default:
        env->sr[id & 31] = val;
        break;
    }
}

uint32_t HELPER(stsr)(CPUV850State *env, uint32_t id)
{
    if (id == SR_PSW) {
        return v850_psw_read(env);
    }
    return env->sr[id & 31];
}

/* RETI (0), CTRET (2), DBRET (3) */
void HELPER(ret_from)(CPUV850State *env, uint32_t which)
{
    int pc, psw;

    switch (which) {
    case 0:
        if (env->psw & PSW_EP) {
            pc = SR_EIPC, psw = SR_EIPSW;
        } else if (env->psw & PSW_NP) {
            pc = SR_FEPC, psw = SR_FEPSW;
        } else {
            pc = SR_EIPC, psw = SR_EIPSW;
        }
        break;
    case 2:
        pc = SR_CTPC, psw = SR_CTPSW;
        break;
    default:
        pc = SR_DBPC, psw = SR_DBPSW;
        break;
    }
    env->pc = env->sr[pc] & ~1;
    v850_psw_write(env, env->sr[psw]);
}

static uint32_t set_logic_flags(CPUV850State *env, uint32_t res, uint32_t cy)
{
    env->zf = env->sf = res;
    env->ovf = 0;
    env->cyf = cy;
    return res;
}

uint32_t HELPER(shl)(CPUV850State *env, uint32_t v, uint32_t n)
{
    n &= 31;
    return set_logic_flags(env, v << n, n ? (v >> (32 - n)) & 1 : 0);
}

uint32_t HELPER(shr)(CPUV850State *env, uint32_t v, uint32_t n)
{
    n &= 31;
    return set_logic_flags(env, v >> n, n ? (v >> (n - 1)) & 1 : 0);
}

uint32_t HELPER(sar)(CPUV850State *env, uint32_t v, uint32_t n)
{
    n &= 31;
    return set_logic_flags(env, (int32_t)v >> n,
                           n ? ((int32_t)v >> (n - 1)) & 1 : 0);
}

/* CY and OV come from the plain operation, S and Z from the result */
static uint32_t saturate(CPUV850State *env, uint32_t res, bool cy, bool ov)
{
    if (ov) {
        res = (int32_t)res < 0 ? 0x7fffffff : 0x80000000;
        env->sat = 1;
    }
    env->zf = env->sf = res;
    env->ovf = ov ? 0x80000000u : 0;
    env->cyf = cy;
    return res;
}

uint32_t HELPER(satadd)(CPUV850State *env, uint32_t a, uint32_t b)
{
    uint32_t res = a + b;

    return saturate(env, res, res < a, ((res ^ a) & ~(a ^ b)) >> 31);
}

uint32_t HELPER(satsub)(CPUV850State *env, uint32_t a, uint32_t b)
{
    uint32_t res = a - b;

    return saturate(env, res, a < b, ((a ^ b) & (a ^ res)) >> 31);
}

/*
 * kind: 0 DIV, 1 DIVU, 2 DIVH (3 operands), 3 DIVHU, 4 DIVH (2 operands).
 * A division by zero sets OV and leaves the registers alone.
 */
void HELPER(div)(CPUV850State *env, uint32_t kind, uint32_t r1,
                 uint32_t r2, uint32_t r3)
{
    uint32_t a = env->gpr[r2], b = env->gpr[r1], q, r;
    bool ov = false;

    switch (kind) {
    case 2:
    case 4:
        b = (int16_t)b;
        /* fall through */
    case 0:
        if (b == 0) {
            env->ovf = 0x80000000u;
            return;
        }
        if (a == 0x80000000u && b == 0xffffffffu) {
            q = a;
            r = 0;
            ov = true;
        } else {
            q = (int32_t)a / (int32_t)b;
            r = (int32_t)a % (int32_t)b;
        }
        break;
    case 3:
        b &= 0xffff;
        /* fall through */
    default:
        if (b == 0) {
            env->ovf = 0x80000000u;
            return;
        }
        q = a / b;
        r = a % b;
        break;
    }
    if (r2) {
        env->gpr[r2] = q;
    }
    if (kind != 4 && r3) {
        env->gpr[r3] = r;
    }
    env->zf = env->sf = q;
    env->ovf = ov ? 0x80000000u : 0;
}

/* BSW (0), BSH (1), HSW (2) */
uint32_t HELPER(swap)(CPUV850State *env, uint32_t v, uint32_t which)
{
    uint32_t res;

    switch (which) {
    case 0:
        res = bswap32(v);
        env->cyf = !(res & 0xff) || !(res & 0xff00) ||
                   !(res & 0xff0000) || !(res & 0xff000000);
        env->zf = res;
        break;
    case 1:
        res = ((v & 0x00ff00ff) << 8) | ((v >> 8) & 0x00ff00ff);
        env->cyf = !(res & 0xff) || !(res & 0xff00);
        env->zf = res & 0xffff;
        break;
    default:
        res = (v << 16) | (v >> 16);
        env->cyf = !(res & 0xffff) || !(res >> 16);
        env->zf = res;
        break;
    }
    env->sf = res;
    env->ovf = 0;
    return res;
}

uint32_t HELPER(callt)(CPUV850State *env, uint32_t imm6, uint32_t next)
{
    uint32_t ctbp = env->sr[SR_CTBP];

    env->sr[SR_CTPC] = next;
    env->sr[SR_CTPSW] = v850_psw_read(env);
    return (ctbp + cpu_lduw_le_data_ra(env, ctbp + (imm6 << 1), GETPC()))
           & ~1;
}

/* list12 bit for r20 + i */
static const uint8_t list12_bit[12] = {
    27, 26, 25, 24, 31, 30, 29, 28, 23, 22, 0, 21
};

void HELPER(prepare)(CPUV850State *env, uint32_t list, uint32_t imm5)
{
    uint32_t sp = env->gpr[R_SP];

    for (int i = 0; i < 12; i++) {
        if (list & (1u << list12_bit[i])) {
            sp -= 4;
            cpu_stl_le_data_ra(env, sp & ~3, env->gpr[20 + i], GETPC());
        }
    }
    env->gpr[R_SP] = sp - (imm5 << 2);
}

void HELPER(dispose)(CPUV850State *env, uint32_t list, uint32_t imm5)
{
    uint32_t sp = env->gpr[R_SP] + (imm5 << 2);

    for (int i = 11; i >= 0; i--) {
        if (list & (1u << list12_bit[i])) {
            env->gpr[20 + i] = cpu_ldl_le_data_ra(env, sp & ~3, GETPC());
            sp += 4;
        }
    }
    env->gpr[R_SP] = sp;
}
