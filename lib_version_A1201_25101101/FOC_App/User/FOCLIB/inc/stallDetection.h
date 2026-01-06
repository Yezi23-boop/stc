#ifndef __STALLDETECTION_H
#define __STALLDETECTION_H

#include "mm_lib.h"
#include "gcf_lib.h"
#include "typedef.h"
#include "nvsns_type_def.h"
#include "nvsns_foc.h"

/******************************************************************************
* Type definition
******************************************************************************/

typedef struct
{
  //bEMFObs of Q from Observer
  Frac16_t                  bEMFObs_Q;
  Frac16_t                  bEMFObsFilter_Q;
  Frac16_t                  wRotElFilt;
  //bEMFKeCal of Q from KE Calculation
  Frac16_t                  bEMFKeCal_Q;
  //bEMFKeCal High Threshold
  Frac16_t                  bEMFKeCalH_Q;
  //bEMFKeCal Low Threshold
  Frac16_t                  bEMFKeCalL_Q;
  //KE coeff
  Frac16_t                  coeffKE;       //coeffKE and coeffKEOFT is from test
  Frac16_t                  coeffKENshift; //coeffKE shift, usually 0
  //KE offset
  Frac16_t                  coeffKEOFT;   
  Frac16_t                  coeffKEOFTNshift; //coeffKEOFT shift, usually 0
  //blank count for stall detection
  unsigned int             blankCnt;
  //black cnt period
  unsigned int             blankCntPeriod; //usually 5ms, but can be changed
  
  //check count for  reliability
  unsigned int             stallDetCnt;    
  //stalldetcnt period
  unsigned int             stallDetCntPeriod; //usually 200ms, but can be changed
  unsigned int             stallDetErrCnt;
  //stalldet err cnt period
  unsigned int             stallDetErrCntPeriod; //usually 5, but can be changed
  Frac16_t                  coeffL;        //usually 0.75 to 0.85    
  Frac16_t                  coeffH;    
  GCF_Filter1_LPF16_t       bEMFObsFilter;
  GCF_Filter1_LPF16_t       wRotElFilter;
  bool                    stallErrFlag;
  bool                  enStallDetection; // Enable stall detection 
  
} stallDetection_T;
extern stallDetection_T tStallDetection;
/* Exported function headers */
extern void stallDetectionInit(stallDetection_T * params);
extern bool stallDetection1ms(stallDetection_T *params,FOC_Driver_t *tDrvFoc);

#endif
