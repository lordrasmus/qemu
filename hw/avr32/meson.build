avr32_ss = ss.source_set()
avr32_ss.add(when: 'CONFIG_AP7000', if_true: files(
  'ap7000_intc.c',
  'ap7000_pm.c',
  'atmel_usart.c',
  'atngw100.c',
))

hw_arch += {'avr32': avr32_ss}
