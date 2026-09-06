//----------------------------------------------------------------------------------------------------------------------------------

#include "types.h"
#include "mmu.h"
#include "mmu_map.h"
#include "arm32.h"

//----------------------------------------------------------------------------------------------------------------------------------

static uint32 mmu_ttb[4096] __attribute__((aligned(16384)));

//----------------------------------------------------------------------------------------------------------------------------------

void mmu_setup(void)
{
  uint32 i;
  uint32 va;
  uint32 flags;

  for(i = 0; i < 4096; i++)
  {
    va = i << 20;
    flags = mmu_flags_for_va(va);
    mmu_ttb[i] = flags ? mmu_section_desc(va, flags) : 0;
  }

  arm32_dcache_clean_invalidate();
  arm32_icache_invalidate();
  arm32_tlb_invalidate();
  arm32_ttb_set((uint32)mmu_ttb);
  arm32_domain_set(0xFFFFFFFF);
  arm32_mmu_enable();
}

void mmu_off_for_brom(void)
{
  arm32_dcache_clean_invalidate();
  arm32_dcache_disable();
  arm32_mmu_disable();
  arm32_tlb_invalidate();
  arm32_icache_invalidate();
}

//----------------------------------------------------------------------------------------------------------------------------------
