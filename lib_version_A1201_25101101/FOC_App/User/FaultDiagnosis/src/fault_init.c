/************************************************************
 * @file: fault_init.c
 * @author: 
 * @version: V0.0
 * @data: 2023/10/29
 * @brief: nvsns module initilization file
 * @note:
 * @Copyright (C) 2023 Novosense All rights reserved.
 ************************************************************/

/************************************************************
 ************************************************************
 * @par Edition History
 * -V0.0  2023.07.28
 *        - Initilization file
 *
 ************************************************************/
#include "fault_init.h"
#include "fault_diagnose.h"
#include "fault_hw_config.h"
#include "foc_paras.h"

Fault_config_t tFaultCfg;

/************************************************************
 * @brief: Initilize hardware fault detection
 * @return <None>
 ************************************************************/
void FAULT_Init(void)
{
    tFaultCfg.i16SftOVTh = tFocParas.U_DCB_OVERVOLTAGE;
    tFaultCfg.i16SftOVRecTh = tFocParas.U_DCB_OVERVOLTAGE - FRAC16(OV_VOLTAGE_RECOV/V_FULL_SCALE);
    tFaultCfg.i16SftUVTh = tFocParas.U_DCB_UNDERVOLTAGE;
    tFaultCfg.i16SftUVRecTh = tFocParas.U_DCB_UNDERVOLTAGE + FRAC16(UV_VOLTAGE_RECOV/V_FULL_SCALE);
    tFaultCfg.i16SftOTTh = ECU_TEMP_UPLIMT;
    tFaultCfg.i16SftOTRecTh = ECU_TEMP_RECOV;
    tFaultCfg.i16SftOTSTTh = 155;
    tFaultCfg.i16SftOTSTRecTh = 140;
    tFaultCfg.i16SftUTTh = -40;
    tFaultCfg.i16FltHystTimes = 1000;
    
    /* hardware Over Voltage Protection */
    SYSCTRL->PWRCR_b.BOIEN = ECU_BUS_HOV_ENABLE;
    SYSCTRL->PWRCR_b.COVTH = ECU_BUS_HOV_TH;
    SYSCTRL->PWRCR_b.BOF = ECU_BUS_HOV_FT;
    /* hardware Under Voltage Protection */

    SYSCTRL->PWRCR_b.BUIEN = ECU_BUS_HUV_ENABLE;
    SYSCTRL->PWRCR_b.UVVTH = ECU_BUS_HUV_TH;
    SYSCTRL->PWRCR_b.BUF = ECU_BUS_HUV_FT;
#if (ECU_BUS_HUV_ENABLE == 1)
    SYSCTRL->PWRCR_b.BUREN = ECU_BUS_HUV_ENABLE; //欠压复位，防止高速顺风掉电后立即上电由于掉电不完全导致的异常
#endif
    /* hardware Over temperature Protection */
    SYSCTRL->PWRCR_b.TSIEN = ECU_BUS_OT_ENABLE;
    SYSCTRL->PWRCR_b.TSF = ECU_BUS_OT_FT;
    
    /* hardware Charpump OV/UV Protection */
    SYSCTRL->PWRCR_b.CUVTH = ECU_MOT_CUVTH;
    SYSCTRL->PWRCR_b.COVTH = ECU_MOT_COVTH;
    SYSCTRL->ANACR_b.CHGPASTEN = ECU_MOT_CP_CAE;
    SYSCTRL->ANACR_b.CIEN = ECU_MOT_CP_OV_EANABLE;
    SYSCTRL->ANACR_b.CF = ECU_MOT_CP_OV_FT;
    SYSCTRL->ANACR_b.CFEN = 1;

#if (ECU_BUS_HOV_ENABLE || ECU_BUS_HUV_ENABLE || ECU_BUS_OT_ENABLE)
    NVIC_EnableIRQ(PMU_IRQn);
    NVIC_SetPriority(PMU_IRQn, ECU_FAULT_IRQ_PRIO);
#endif

#if (ECU_MOT_CP_OV_EANABLE)
    NVIC_EnableIRQ(CHP_IRQn);
    NVIC_SetPriority(CHP_IRQn, ECU_FAULT_IRQ_PRIO);
#endif

}


