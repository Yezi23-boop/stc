#ifndef __SVM_LIB_H__
#define __SVM_LIB_H__

#include "nvsns_type_def.h"
#include "mm_lib.h"

void GMC_SvmExDpwm_F16(CTD_LUT_SVM_t *pSvm, const CTD_2SystF16_t *pUab,Frac16_t  f16ThetaRotEl,Frac16_t switchData,Frac16_t zeroSeqComp);
void GMC_Svm_C(CTD_LUT_SVM_t* ptSvm, const CTD_2SystF16_t* const ptUAlBeReq);
void SVPWM_Calculate(CTD_LUT_SVM_t *ptSvm, const CTD_2SystF16_t *const ptUAlBeReqDCB);
#endif