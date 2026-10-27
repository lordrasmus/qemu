cris_ss = ss.source_set()
cris_ss.add(when: 'CONFIG_ETRAX100LX', if_true: files('etrax100lx.c'))
cris_ss.add(when: 'CONFIG_AXIS', if_true: files('boot.c', 'axis_dev88.c'))

hw_arch += {'cris': cris_ss}
