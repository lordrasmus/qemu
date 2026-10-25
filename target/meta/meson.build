meta_ss = ss.source_set()
meta_ss.add(files(
  'cpu.c',
  'decode.c',
  'helper.c',
  'log.c',
  'op_helper.c',
  'translate.c',
))

meta_system_ss = ss.source_set()
meta_system_ss.add(files(
  'core_registers.c',
  'triggers.c',
))

target_arch += {'meta': meta_ss}
target_system_arch += {'meta': meta_system_ss}
