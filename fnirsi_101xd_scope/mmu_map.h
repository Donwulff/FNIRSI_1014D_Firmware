//----------------------------------------------------------------------------------------------------------------------------------

#ifndef MMU_MAP_H
#define MMU_MAP_H

#include "types.h"

//----------------------------------------------------------------------------------------------------------------------------------
//Identity 1 MB sections. FRAMEBUFFER_BASE must match fnirsi_101xd.ld.
//----------------------------------------------------------------------------------------------------------------------------------

#define MMU_FB_BASE       0x81D00000u
#define MMU_FB_END        0x81E00000u
#define MMU_DRAM_BASE     0x80000000u
#define MMU_DRAM_END      0x82000000u
#define MMU_PERIPH_BASE   0x01C00000u
#define MMU_PERIPH_END    0x02000000u

//ARM926 short-descriptor 1 MB section bits (AP=full, domain 0, IMP=1).
#define MMU_DESC_SECTION  0x00000002u
#define MMU_DESC_B        0x00000004u
#define MMU_DESC_C        0x00000008u
#define MMU_DESC_IMP      0x00000010u
#define MMU_DESC_AP_FULL  0x00000C00u

#define MMU_FLAGS_NCNB    (MMU_DESC_SECTION | MMU_DESC_IMP | MMU_DESC_AP_FULL)
#define MMU_FLAGS_WB      (MMU_FLAGS_NCNB | MMU_DESC_C | MMU_DESC_B)

//0 = translation fault (unmapped).
uint32 mmu_flags_for_va(uint32 va);

//va is rounded down to 1 MB. flags must be MMU_FLAGS_*.
uint32 mmu_section_desc(uint32 va, uint32 flags);

//----------------------------------------------------------------------------------------------------------------------------------

#endif /* MMU_MAP_H */
