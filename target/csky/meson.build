csky_ss = ss.source_set()
csky_ss.add(when: 'TARGET_CSKYV2', if_true: files(
  'translate_v2.c',
  'op_helper_v2.c',
  'op_fpuv3.c',
  'csky-semi.c',
))
csky_ss.add(files(
  'cpu.c',
  'helper.c',
  'mmu.c',
  'gdbstub.c',
  'trace-stub.c',
))

target_arch += {'csky': csky_ss}
