gen = [
  decodetree.process('insn.decode', extra_args: [ '--decode', 'decode_insn',
                                                  '--varinsnwidth', '32'  ])
]

avr32_ss = ss.source_set()
avr32_system_ss = ss.source_set()

avr32_ss.add(gen)
avr32_ss.add(files(
  'cpu.c',
  'helper.c',
  'helper_conditions.c',
  'gdbstub.c',
  'translate.c',
))


target_arch += {'avr32': avr32_ss}
target_system_arch += {'avr32': avr32_system_ss}
