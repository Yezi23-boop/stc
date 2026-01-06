#ifndef FIELD_WEAKENING_AIWIN_H
#define FIELD_WEAKENING_AIWIN_H

/****************************************************************************
* Includes
****************************************************************************/
#include "amc_lib.h"
#include "gmc_lib.h"
#include "mm_lib.h"
#include "amc_fluxweakening.h"




extern void AMC_FluxWeakening_NSUC_F16_AIWIN(Frac16_t f16IDQReqAmp, Frac16_t f16VelocityFbck, CTD_2SystF16_t * pIDQReq, AMC_FluxWeakening_t *pCtrl,CTD_2SystF16_t *const pUDQReq,Frac16_t f16Ubus);
extern void AMC_FluxWeakening_VsMargin_F16_AIWIN(Frac16_t f16IDQReqAmp, Frac16_t f16VelocityFbck, CTD_2SystF16_t * pIDQReq, AMC_FluxWeakening_t *pCtrl,CTD_2SystF16_t *const pUDQReq,Frac16_t f16Ubus);
extern void AMC_FluxWeakening_IqUqReserveDiff_F16_AIWIN(Frac16_t f16IDQReqAmp, Frac16_t f16VelocityFbck, CTD_2SystF16_t * pIDQReq, AMC_FluxWeakening_t *pCtrl,CTD_2SystF16_t *const pUDQReq,Frac16_t f16Ubus);
#endif

