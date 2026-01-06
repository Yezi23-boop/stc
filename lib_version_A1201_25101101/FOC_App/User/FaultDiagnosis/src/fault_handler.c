/************************************************************
 * @file: fault_handler.c
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

#include "fault_handler.h"
#include "fault_diagnose.h"
#include "nsuc1602.h"

/************************************************************
 * @brief: fault handler at fault event
 * @return <None>
 ************************************************************/
void FLT_Handler(void)
{
    
    if (tFaultMode.FM_b.VBUS_SPREOV) {
        /* Customer function */
    }
    
    if (tFaultMode.FM_b.VBUS_SPREUV) {
        /* Customer function */
    }
    
    if (tFaultMode.FM_b.VBUS_SOV) {
        /* Customer function */
    }
    
    if (tFaultMode.FM_b.VBUS_HUV) {
        /* Customer function */
    }
    
    /** etc. **/
}

/************************************************************
 * @brief: clear fault flag
 * @return <None>
 ************************************************************/
void FLT_ClearFlag(void)
{
    // if (tFaultMode.FM_b.VBUS_SOVREC) {
    //     tFaultMode.FM_b.VBUS_SOV = 0;
    //     tFaultMode.FM_b.VBUS_SPREOV = 0;
    //     tFaultMode.FM_b.VBUS_SOVREC = 0;
    // }
    
    // if (tFaultMode.FM_b.VBUS_SUVREC) {
    //     tFaultMode.FM_b.VBUS_SUV = 0;
    //     tFaultMode.FM_b.VBUS_SPREUV = 0;
    //     tFaultMode.FM_b.VBUS_SUVREC = 0;
    // }
    
    // if (tFaultMode.FM_b.ECU_SOTREC) {
    //     tFaultMode.FM_b.ECU_SOT = 0;
    //     tFaultMode.FM_b.ECU_SOTREC = 0;
    // }
    
    // if (tFaultMode.FM_b.ECU_SUTREC) {
    //     tFaultMode.FM_b.ECU_SUT = 0;
    //     tFaultMode.FM_b.ECU_SUTREC = 0;
    // }
}

void CHP_IRQHandler(void)
{
    //电荷泵过压
    if (SYSCTRL->ANACR_b.CHGOV == 1) {
        tFaultMode.FM_b.CP_OV = 1;
        SYSCTRL->ANACR_b.CHGOV = 1;
    }
    // 电荷泵欠压
    if (SYSCTRL->ANACR_b.CHGUV == 1) {
        tFaultMode.FM_b.CP_UV = 1;
        SYSCTRL->ANACR_b.CHGUV = 1;
    }
}

void PMU_IRQHandler(void)
{
    // 硬件欠压
    if (SYSCTRL->PWRCR_b.BUVST) {
        tFaultMode.FM_b.VBUS_HUV = 1;
        SYSCTRL->PWRCR_b.BUVST = 1;
    }
    
    // 硬件过压
    if (SYSCTRL->PWRCR_b.BOVST) {
        tFaultMode.FM_b.VBUS_HOV = 1;
        SYSCTRL->PWRCR_b.BOVST = 1;
    }
    
    //硬件过温
    if (SYSCTRL->PWRCR_b.TSDST) {
        tFaultMode.FM_b.ECU_HOT = 1;
        SYSCTRL->PWRCR_b.TSDST = 1;
    }

}

void GDU_IRQHandler(void)
{
    //硬件过流
    if (GDU->ISR & 0x40) {
        tFaultMode.FM_b.MOT_HOC = 1;
        GDU->ISR = 0x40;
    }
    
    //硬件短路监测
    if (GDU->ISR & 0x3F) {
        tFaultMode.FM_b.ECU_DRV_SHORT = 1;
        GDU->ISR = 0x3F;
    }
}