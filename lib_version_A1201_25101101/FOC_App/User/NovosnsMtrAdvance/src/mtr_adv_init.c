#include "mtr_adv_init.h"

void WM_Init(WM_Obsvr_t *tWMObsvr, int16_t i16AdcBemf)
{
    /* windmilling init */
    tWMObsvr->tUabcBemf.f16Arg1 = i16AdcBemf;
    tWMObsvr->tUabcBemf.f16Arg2 = i16AdcBemf;
    tWMObsvr->tUabcBemf.f16Arg3 = i16AdcBemf;
    tWMObsvr->f16AmpSqure = 0;
    tWMObsvr->f16Speed = 0;
    tWMObsvr->f16Theta = 0;
    tWMObsvr->f16AmpDetTh = 15;
    tWMObsvr->f16SpdDetTh = FRAC16(0.025);
    tWMObsvr->AccCnt = 0;
    tWMObsvr->AccumTheta = 0;
    tWMObsvr->tWmDir = NOWIND;
}

void AMC_FWInit_F16(AMC_FluxWeakening_t *const pCtrl, CTD_2SystF16_t* tIDQFbck, 
                    CTD_2SystF16_t* tUDQReq, GCF_CtrlPIAW_RTF16_t* tAxisQCtrl)
{ 
	pCtrl->pFilterFW.f16NSample = 6;
	pCtrl->pFilterFW.f32Acc = (Frac32_t)0;

	pCtrl->pPIpAWFW.f16PropGain = FRAC16(0.3); // 0.3
	pCtrl->pPIpAWFW.s16PropGainShift = (int16_t)0;
	pCtrl->pPIpAWFW.f16IntegGain = FRAC16(0.0002);//FRAC16(0.02);
	pCtrl->pPIpAWFW.s16IntegGainShift = 0;
	pCtrl->pPIpAWFW.f16UpperLimit = 0;
	pCtrl->pPIpAWFW.f16LowerLimit = FRAC16(-20.0/62.5);
	pCtrl->pPIpAWFW.f32IntegPartK_1 = (Frac16_t)0;
	pCtrl->pPIpAWFW.f16InK_1 = (Frac16_t)0;

	pCtrl->pIQFbck = &tIDQFbck->f16Arg2;
	pCtrl->pUQReq = &tUDQReq->f16Arg2;
	pCtrl->pUQLim = &tAxisQCtrl->f16UpperLimit;

}

void StallInit(StallDetection_t *tStall)
{
    tStall->f16EMFObsQ = 0;
    tStall->f16EMFObsFilterQ = 0;
    tStall->f16wRotElFilt = 0;
    tStall->f16EMFKeCalQ = 0;
    tStall->f16EMFKeCalHQ = 0;
    tStall->f16EMFKeCalLQ = 0;
    tStall->f16CoeffKE = FRAC16(0.56459); // tune
    tStall->f16CoeffKEOft = 0; //tune
    tStall->u16BlankCnt = 0;
    tStall->u16StallBlankConst = 5000; // blank pwm period after open loop
    tStall->u16StallDetCnt = 0;
    tStall->u16StallDetConst = 100;
    tStall->u16StallDetErrCnt = 0;
    tStall->u16StallDetErrConst = 90;
    tStall->f16Coeff = FRAC16(0.5);
    tStall->tEMFObsFilter.f16NSample = 2;
    tStall->tEMFObsFilter.f32Acc = 0;
    tStall->tEMFObsFilter.f16State = 0;
    tStall->u16StallErrFlag = 0;
}
