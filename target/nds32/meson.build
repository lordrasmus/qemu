nds32_ss = ss.source_set()
nds32_ss.add(files(
  'cpu.c',
  'helper.c',
  'translate.c',
))

nds32_system_ss = ss.source_set()

target_arch += {'nds32': nds32_ss}
target_system_arch += {'nds32': nds32_system_ss}
