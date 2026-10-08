ENTRY(_start)
MEMORY { ram (rwx) : ORIGIN = 0x00010000, LENGTH = 0x01000000 }
SECTIONS {
  .text : { *(.text.start) start.o(.text) *(.text*) *(.rodata*) } > ram
  .init_array : { __init_array_start = .; KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array)) KEEP(*(.ctors)) __init_array_end = .; } > ram
  .data : { *(.data*) *(.sdata*) } > ram
  .bss (NOLOAD) : { __bss_start = .; *(.bss*) *(.sbss*) *(COMMON) . = ALIGN(4); __bss_end = .; } > ram
  . = ALIGN(16);
  . = . + 0x10000;
  __stack_top = .;
  /DISCARD/ : { *(.eh_frame*) *(.comment) }
}
