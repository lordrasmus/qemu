tilegx_ss = ss.source_set()
tilegx_ss.add(files(
  'cpu.c',
  'helper.c',
  'simd_helper.c',
  'translate.c',
))
tilegx_ss.add(zlib)

tilegx_system_ss = ss.source_set()
tilegx_system_ss.add(files(
  'sys_helper.c',
))

target_arch += {'tilegx': tilegx_ss}
target_system_arch += {'tilegx': tilegx_system_ss}
