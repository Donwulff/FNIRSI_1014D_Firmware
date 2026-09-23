MEMORY
{
	sram  : org = 0x00000000, len = 32K
  dram  : org = 0x81C00000, len =  4M
}

SECTIONS
{
  . = ORIGIN(sram);
  .text :
  {
    build/start.o (.text)
    *(.text);
    *(.data);
    *(.rodata);
  } >sram

  .dmem :
  {
    *(.dmem);
  } >dram

	__bootloader_size = SIZEOF(.text);
  /* Only SVC runs; IRQ/FIQ stay masked. Reserve at least 3 KB below its 0x7C00 SP. */
  ASSERT(ALIGN(SIZEOF(.text), 512) <= 0x7000, "1014D loader reaches SRAM stack reserve")
}
