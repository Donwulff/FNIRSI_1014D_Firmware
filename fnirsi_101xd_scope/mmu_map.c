//----------------------------------------------------------------------------------------------------------------------------------

#include "mmu_map.h"

//----------------------------------------------------------------------------------------------------------------------------------

uint32 mmu_flags_for_va(uint32 va)
{
  if(va < 0x00100000u)
  {
    return(MMU_FLAGS_NCNB);
  }

  if((va >= MMU_PERIPH_BASE) && (va < MMU_PERIPH_END))
  {
    return(MMU_FLAGS_NCNB);
  }

  if((va >= MMU_FB_BASE) && (va < MMU_FB_END))
  {
    return(MMU_FLAGS_NCNB);
  }

  if((va >= MMU_DRAM_BASE) && (va < MMU_DRAM_END))
  {
    return(MMU_FLAGS_WB);
  }

  return(0);
}

uint32 mmu_section_desc(uint32 va, uint32 flags)
{
  return((va & 0xFFF00000u) | flags);
}

//----------------------------------------------------------------------------------------------------------------------------------
