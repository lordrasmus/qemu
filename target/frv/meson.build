frv_ss = ss.source_set()
frv_ss.add(files(
  'cpu.c',
  'translate.c',
  'op_helper.c',
  'helper.c',
))

frv_system_ss = ss.source_set()

target_arch += {'frv': frv_ss}
target_system_arch += {'frv': frv_system_ss}
