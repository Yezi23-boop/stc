#ifndef __BUS_COMP_H__
#define __BUS_COMP_H__

#include "nvsns_type_def.h"
#include "gmc_lib.h"


extern void GMC_DcBusCompF16AIWIN(CTD_2SystF16_t *const ptOut, const CTD_2SystF16_t *const ptIn, const GMC_DcBusComp_t *const ptParam);
extern void GMC_DcBusCompF16AIWIN_TEST(CTD_2SystF16_t *const ptOut, const CTD_2SystF16_t *const ptIn, const GMC_DcBusComp_t *const ptParam);
#endif