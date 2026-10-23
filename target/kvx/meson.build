kvx_ss = ss.source_set()
kvx_ss.add(files(
  'cpu.c',
  'cpu_helper.c',
  'debug_helper.c',
  'gdbstub.c',
  'fpu_helper.c',
  'mds-helpers.c',
  'op_helper.c',
  'timers.c',
  'tlb_helper.c',
  'translate.c',
  'translate-insn.override.c',
  'gen/decode.c',
  'gen/translate-insn.c',
  'gen/mds-helpers.c',
))

kvx_system_ss = ss.source_set()
kvx_system_ss.add(files(
  'semihosting.c',
))

target_arch += {'kvx': kvx_ss}
target_system_arch += {'kvx': kvx_system_ss}
