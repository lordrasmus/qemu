ia64_ss = ss.source_set()
ia64_ss.add(files(
  'cpu.c',
  'translate.c',
  'ia64-decode.c',
  'ia64-extract.c',
  'ia64-format.c',
  'fpu_helper.c',
  'op_helper.c',
))

ia64_system_ss = ss.source_set()
ia64_system_ss.add(files(
  'helper.c',
))

target_arch += {'ia64': ia64_ss}
target_system_arch += {'ia64': ia64_system_ss}
