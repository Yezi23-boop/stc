#ifndef SENSERLESSOBS_H
#define SENSERLESSOBS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nvsns_type_def.h"
#include "amc_lib.h"

#define SENSERLESSOBS_USE_ASM 0


extern CTD_2SystF16_t DqObsEObsrvflt;

void AMC_AIWINPMSMBemfObsvrDQ_F16(Frac16_t *pf16SpeedEst, Frac16_t *pf16PosEst, const CTD_2SystF16_t *const ptIAB,
    const CTD_2SystF16_t *const ptUAB, AMC_BemfcObsvrDQ_t *const ptCtrl);
void AMC_BemfObserver_RealignStates(AMC_BemfcObsvrDQ_t *ptCtrl,
                                           Frac16_t theta_old, Frac16_t theta_new);
#ifdef __cplusplus
}
#endif

#endif // SENSERLESSOBS_H