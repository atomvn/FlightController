MEMORY
{
	FLASH(rx) : ORIGIN = 0x08000000, LENGTH = 128K
	SRAM(rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}


/* Enforce emmition of the vector table. */
EXTERN (vector_table)

/* Define the entry point of the output file. */
ENTRY(Reset_Handler)

/* Define sections. */
SECTIONS
{
	.text : {
		KEEP(*(.isr_vector)) /* Vector table */
		*(.text*)	/* Program code */
		. = ALIGN(4);
		*(.rodata*)	/* Read-only data */
		. = ALIGN(4);
		_etext = .;
	} >FLASH

	.data : {
		_sdata = .;
		*(.data*)	/* Read-write initialized data */
		. = ALIGN(4);
		_edata = .;
	} >SRAM AT >FLASH
	_data_loadaddr = LOADADDR(.data);

	.bss : {
		_sbss = .;
		*(.bss*)	/* Read-write zero initialized data */
		*(COMMON)
		. = ALIGN(4);
		_ebss = .;
	} >SRAM
}

PROVIDE(_stack = ORIGIN(SRAM) + LENGTH(SRAM));