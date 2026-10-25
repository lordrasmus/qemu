lm32_ss = ss.source_set()
lm32_ss.add(when: 'CONFIG_MILKYMIST', if_true: files('milkymist.c'))

hw_arch += {'lm32': lm32_ss}
