MEMORY
{
	/* DRAM up to the NCNB framebuffer at 0x81D00000 (stacks stay in start.s). */
	ram  : org = 0x7FFFFFE0, len = 0x1D00020
	framebuffer : org = 0x81D00000, len = 1M
}

SECTIONS
{
  . = ORIGIN(ram);
  PRGM_START = .;
  .text : 
  {
    build/Debug/GNU_ARM-Linux/start.o (.text)
    *(.text);
  } >ram

  .data :
  {
    *(.data);
    *(.rodata);
  } >ram
  PRGM_END = .;

  BSS_START = .;
  .bss : 
  {
    *(.bss);
  } >ram
  BSS_END = .;

  .framebuffer (NOLOAD) :
  {
    *(.framebuffer);
  } >framebuffer
}

__program_size = PRGM_END - PRGM_START;
