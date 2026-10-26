frv_ss = ss.source_set()
frv_ss.add(files(
  'cpu.c',
  'translate.c',
  'op_helper.c',
))

frv_system_ss = ss.source_set()
frv_system_ss.add(files(
  'helper.c',
))

target_arch += {'frv': frv_ss}
target_system_arch += {'frv': frv_system_ss}
