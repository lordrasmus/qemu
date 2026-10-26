v850_ss = ss.source_set()
v850_ss.add(files(
  'cpu.c',
  'translate.c',
  'op_helper.c',
))

v850_system_ss = ss.source_set()
v850_system_ss.add(files(
  'helper.c',
))

target_arch += {'v850': v850_ss}
target_system_arch += {'v850': v850_system_ss}
