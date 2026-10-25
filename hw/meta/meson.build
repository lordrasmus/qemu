meta_ss = ss.source_set()
meta_ss.add(files(
  'meta_switch.c',
  'meta_da.c',
  'meta_damisc.c',
  'meta_daunexp.c',
  'meta_boot.c',
  'meta_boot_mmu.c',
  'meta_boot_tbi.c',
))
meta_ss.add(when: 'CONFIG_META_01SP', if_true: files('comet.c'))

hw_arch += {'meta': meta_ss}
