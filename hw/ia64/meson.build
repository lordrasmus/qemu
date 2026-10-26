ia64_ss = ss.source_set()
ia64_ss.add(when: 'CONFIG_IA64_VIRT', if_true: files('virt.c', 'iosapic.c'))

hw_arch += {'ia64': ia64_ss}
