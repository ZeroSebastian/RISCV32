OUTPUT_FORMAT("elf32-littleriscv")
OUTPUT_ARCH(riscv)
ENTRY(main)

SECTIONS
{
	. = 0x0;
	.text : { *(.text) }
	. = 0x1000;
	.data : { *(.data) }
	.bss : { *(.bss) }
}

