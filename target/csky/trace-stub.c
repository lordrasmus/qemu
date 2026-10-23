/*
 * Storage for the variables the C-SKY trace stub declares.  Tracing is not
 * supported, they stay zero.
 */
#include "qemu/osdep.h"
#include "cpu.h"

struct csky_trace_server_state traceserver;
struct csky_trace_filter tfilter;
long long csky_trace_icount;
int waddr_num;
int raddr_num;

/*
 * Helpers the trace code generator calls.  They lived in the fork's
 * csky-trace.c; the generated calls sit behind gen_tb_trace() and
 * gen_mem_trace(), which are always false, so these never run.
 */
#include "exec/helper-proto.h"

void helper_csky_trace_icount(CPUCSKYState *env, uint32_t a, uint32_t b)
{
}

void helper_trace_tb_start(CPUCSKYState *env, uint32_t a)
{
}

void helper_trace_tb_exit(uint32_t a, uint32_t b)
{
}

#define TRACE_LDST_STUB(name)                                          \
void helper_trace_##name(CPUCSKYState *env, uint32_t pc, uint32_t val, \
                         uint32_t addr)                                \
{                                                                      \
}
TRACE_LDST_STUB(ld8u)
TRACE_LDST_STUB(ld16u)
TRACE_LDST_STUB(ld32u)
TRACE_LDST_STUB(ld8s)
TRACE_LDST_STUB(ld16s)
TRACE_LDST_STUB(ld32s)
TRACE_LDST_STUB(st8)
TRACE_LDST_STUB(st16)
TRACE_LDST_STUB(st32)

void helper_trace_m_addr(CPUCSKYState *env, uint32_t a, uint32_t b,
                         uint32_t c, uint32_t d)
{
}

void helper_trace_m_value(CPUCSKYState *env, uint32_t a, uint32_t b,
                          uint32_t c)
{
}

void helper_trace_update_tcr(CPUCSKYState *env)
{
}
