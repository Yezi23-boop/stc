#ifndef __TRANSFORM_H__
#define __TRANSFORM_H__

#include "nvsns_type_def.h"
#include "mm_lib.h"
#include "gcf_lib.h"

extern int32_t g_i32AngleIncrement;
extern void GMC_InvClarkeF16(CTD_3SystF16_t* ptPhs3Cur, const CTD_2SystF16_t* const ptSttCur);
extern void GMC_InvClarkeModF16(CTD_3SystF16_t* ptPhs3Volt, const CTD_2SystF16_t* const ptSttVolt);
extern int32_t ThetaTo0_2pi(int32_t theta);
extern Frac16_t SatToPosQ15(Frac32_t value);
extern Frac16_t SatToPosQ15div2(Frac32_t value);
extern void  InitFreqToAngleIncrement(float f16Freq, float f16SwitchFreq);
extern Frac16_t  GetAngleIncrement(int32_t i32Freq);
extern Frac16_t WrapAngleSumQ15(Frac16_t base, Frac16_t delta);
extern Frac16_t GCF_IntegratorTR_F16_AIWIN(Frac16_t f16In, GCF_Integrator_TRF16_t *const pInte);
#endif