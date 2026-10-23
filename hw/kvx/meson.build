kvx_ss = ss.source_set()

kvx_ss.add(when: 'CONFIG_MPPA_CLUSTER', if_true: files(
  'boot.c',
  'apic-gic.c',
  'apic-mailbox.c',
  'dsu-clock.c',
  'ipi-ctrl.c',
  'itgen.c',
  'mppa-coolidge.c',
  'pwr-ctrl.c',
  'ftu.c',
  'machine-descr.c',
  'coolidge-cluster.c',
))

hw_arch += {'kvx': kvx_ss}
