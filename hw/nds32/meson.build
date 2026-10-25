nds32_ss = ss.source_set()
nds32_ss.add(when: 'CONFIG_NDS32_AE3XX', if_true: [files('ae3xx.c', 'atcpit100.c'), fdt])

hw_arch += {'nds32': nds32_ss}
