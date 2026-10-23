csky_ss = ss.source_set()
csky_ss.add(when: 'CONFIG_CSKY_VIRT', if_true: files('virt.c', 'boot.c'))

hw_arch += {'csky': csky_ss}
