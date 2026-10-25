c6x_ss = ss.source_set()
c6x_ss.add(when: 'CONFIG_C6X_DSK6455', if_true: [files('dsk6455.c', 'c64x_megamod.c', 'c64x_timer64.c'), fdt])

hw_arch += {'c6x': c6x_ss}
