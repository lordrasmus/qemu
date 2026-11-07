c6x_ss = ss.source_set()
c6x_ss.add(files(
  'cpu.c',
  'translate.c',
  'op_helper.c',
  'helper.c',
))

c6x_system_ss = ss.source_set()

target_arch += {'c6x': c6x_ss}
target_system_arch += {'c6x': c6x_system_ss}
