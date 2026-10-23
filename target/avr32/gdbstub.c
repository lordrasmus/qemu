/*
 * QEMU AVR32 gdbstub: r0-r12, sp, lr, pc, sr
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */
#include "qemu/osdep.h"
#include "cpu.h"
#include "gdbstub/helpers.h"

int avr32_cpu_gdb_read_register(CPUState *cs, GByteArray *mem_buf, int n)
{
    CPUAVR32State *env = cpu_env(cs);

    if (n < AVR32_NREGS) {
        return gdb_get_reg32(mem_buf, env->r[n]);
    }
    if (n == AVR32_NREGS) {
        return gdb_get_reg32(mem_buf, avr32_get_sr(env));
    }
    return 0;
}

int avr32_cpu_gdb_write_register(CPUState *cs, uint8_t *mem_buf, int n)
{
    CPUAVR32State *env = cpu_env(cs);
    uint32_t val = ldl_p(mem_buf);

    if (n < AVR32_NREGS) {
        env->r[n] = val;
        return 4;
    }
    if (n == AVR32_NREGS) {
        avr32_set_sr(env, val);
        return 4;
    }
    return 0;
}
