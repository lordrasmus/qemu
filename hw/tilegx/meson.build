tilegx_ss = ss.source_set()
tilegx_ss.add(when: 'CONFIG_TILEGX_HV', if_true: files('tilegx-hv.c'))

hw_arch += {'tilegx': tilegx_ss}
