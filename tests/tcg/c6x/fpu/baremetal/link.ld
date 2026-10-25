/*
 * RAM_BASE and RAM_SIZE can be set with -Wl,--defsym; the defaults are
 * the DDR of qemu's dsk6455 board.
 */
ENTRY(_start)
SECTIONS {
  . = DEFINED(RAM_BASE) ? RAM_BASE : 0xe0000000;
  .text : { *(.text*) }
  .rodata : { *(.rodata*) *(.const*) }
  . = ALIGN(8);
  .data : { *(.data*) *(.fardata*) }
  . = ALIGN(8);
  __c6xabi_DSBT_BASE = .;
  .neardata : { *(.dsbt) *(.got*) *(.neardata*) *(.rodata.cst*) }
  . = ALIGN(8);
  __bss_start = .;
  .bss : { *(.bss*) *(.far*) *(COMMON) }
  __bss_end = .;
  __stack_top = (DEFINED(RAM_BASE) ? RAM_BASE : 0xe0000000) +
                (DEFINED(RAM_SIZE) ? RAM_SIZE : 0x04000000);
}
