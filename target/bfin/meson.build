gen = [
  decodetree.process('insn16.decode', extra_args: [ '--static-decode', 'decode16',
                                                    '--insnwidth', '16' ]),
  decodetree.process('insn32.decode', extra_args: [ '--static-decode', 'decode32',
                                                    '--insnwidth', '32' ]),
]

bfin_ss = ss.source_set()
bfin_system_ss = ss.source_set()

bfin_ss.add(gen)
bfin_ss.add(files(
  'cpu.c',
  'dsp.c',
  'helper.c',
  'translate.c',
))

target_arch += {'bfin': bfin_ss}
target_system_arch += {'bfin': bfin_system_ss}
