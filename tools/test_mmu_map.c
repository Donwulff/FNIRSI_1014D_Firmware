#include <stdio.h>
#include "../fnirsi_101xd_scope/types.h"
#include "../fnirsi_101xd_scope/mmu_map.h"

static int failures = 0;

static void expect_eq(const char *name, uint32 got, uint32 want)
{
  if(got != want)
  {
    printf("FAIL %s: got 0x%08x want 0x%08x\n", name, got, want);
    failures++;
  }
}

int main(void)
{
  expect_eq("vectors NCNB", mmu_flags_for_va(0x00000000u), MMU_FLAGS_NCNB);
  expect_eq("sram top NCNB", mmu_flags_for_va(0x000FFFFFu), MMU_FLAGS_NCNB);
  expect_eq("Port E NCNB", mmu_flags_for_va(0x01C20800u), MMU_FLAGS_NCNB);
  expect_eq("DEBE NCNB", mmu_flags_for_va(0x01E60800u), MMU_FLAGS_NCNB);
  expect_eq("TCON NCNB", mmu_flags_for_va(0x01C0C000u), MMU_FLAGS_NCNB);
  expect_eq("DRAM WB", mmu_flags_for_va(0x80000000u), MMU_FLAGS_WB);
  expect_eq("DRAM just below FB WB", mmu_flags_for_va(MMU_FB_BASE - 1), MMU_FLAGS_WB);
  expect_eq("FB NCNB", mmu_flags_for_va(MMU_FB_BASE), MMU_FLAGS_NCNB);
  expect_eq("FB last byte NCNB", mmu_flags_for_va(MMU_FB_END - 1), MMU_FLAGS_NCNB);
  expect_eq("stacks WB", mmu_flags_for_va(0x81FB5000u), MMU_FLAGS_WB);
  expect_eq("below periph fault", mmu_flags_for_va(0x01B00000u), 0);
  expect_eq("after periph fault", mmu_flags_for_va(0x02000000u), 0);
  expect_eq("FEL ROM fault", mmu_flags_for_va(0xFFFF0020u), 0);

  expect_eq("desc masks 1MB", mmu_section_desc(0x80012345u, MMU_FLAGS_WB) & 0xFFF00000u, 0x80000000u);
  expect_eq("desc keeps flags", mmu_section_desc(0x80000000u, MMU_FLAGS_WB) & 0x000FFFFFu, MMU_FLAGS_WB);
  expect_eq("desc is section", mmu_section_desc(0x81D00000u, MMU_FLAGS_NCNB) & 3u, 2u);

  if(failures)
  {
    printf("%d failure(s)\n", failures);
    return 1;
  }
  printf("ok\n");
  return 0;
}
