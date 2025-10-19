_stack_start = ORIGIN(RAM) + LENGTH(RAM);
_stack_end   = ORIGIN(RAM);

SECTIONS
{
      .axisram (NOLOAD) : ALIGN(8) { *(.axisram .axisram.*); } > AXISRAM
      .heap (NOLOAD) : ALIGN(8)
      {
        __heap_start = .;
        *(.heap .heap.*);
        __heap_end = .;
      } > AXISRAM

      .sram3 (NOLOAD) : ALIGN(4) { *(.sram3 .sram3.*); } > SRAM3
      .sram4 (NOLOAD) : ALIGN(4) { *(.sram4 .sram4.*); } > SRAM4
}