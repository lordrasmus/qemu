c6x_ss = ss.source_set()
c6x_ss.add(files(
  'cpu.c',
  'translate.c',
  'op_helper.c',
))

c6x_system_ss = ss.source_set()
c6x_system_ss.add(files(
  'helper.c',
))

target_arch += {'c6x': c6x_ss}
target_system_arch += {'c6x': c6x_system_ss}
