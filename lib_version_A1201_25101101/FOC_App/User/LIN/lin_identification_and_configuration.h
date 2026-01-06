/****************************************************************************
 * @file    : lin_identification_and_configuration.h
 * @author  : Novosns MCU Team
 * @version : V2.0
 * @Date    : 2023/4/7
 * @brief   : lin stack file
 * @note
 * Copyright (C) 2023 Novosense All rights reserved.
 ****************************************************************************/
#ifndef  LIN_IDENTIFICATION_AND_CONFIGURATION_H
#define  LIN_IDENTIFICATION_AND_CONFIGURATION_H
/*******************************************************************************
  type definitions
*******************************************************************************/
#include <stdint.h>
#include "lin_define.h"
#include "lin_api.h"
#include "lin_api_appl.h"
#include "lin_config.h"


#ifdef L_SNPD_BSM 
    // definition of 'g_bsmNodeState'
    #define  L_NODE_BSM_OFF     0      // initial state: BSM inactive
    #define  L_NODE_INIT        1      // not addressed, not pre-selected
    #define  L_NODE_SELECTED    2      // pre-selected
    #define  L_NODE_ADDRESSED   3      // addressed
    #define  L_NODE_NAD_STORED  4      // NAD accepted and stored to RAM


    /* TODO, should configurate threahold by NSUC1602's PGA and ADC
         - Adjust current thresholds due to impact of current source and/or pull-up
         - Effective R_BSM is 1 Ohm (including the two bond wires)
         

         Calculation of resulting thresholds (in LSB):
            for ISH2: 
              1.0mA through 1 Ohm shunt results in 1.0mV:
              voltage @ ADC = 1.0mV * 45 (LINBSM gain) = 45mV
              ADC value [in LSB] = 56mV * 4096 / 2.5V = 73 LSB
            for ISH3: 
              4.5mA through 1 Ohm shunt results in 4.5mV:
              voltage @ ADC = 4.5mV * 45 (LINBSM gain) = 202mV
              ADC value [in LSB] = 202mV * 4096 / 2.5 V = 331 LSB */

    #define  SNPD_I_DIFF    168        // corresponds to 2.3 mA via 1.0 Ohm
    
  
#endif


typedef struct {
    uint8_t SID;
    void (*ConfigFunc)(void);
} SidToConfigFunc;

#define NAD_INDEX_IN_DIAGNOSIS_FRAME   0x00
#define PCI_INDEX_IN_DIAGNOSIS_FRAME   0x01
#define SID_INDEX_IN_DIAGNOSIS_FRAME   0x02
#define DATA_0_IN_DIAGNOSIS_FRAME      0x03
#define DATA_1_IN_DIAGNOSIS_FRAME      0x04
#define DATA_2_IN_DIAGNOSIS_FRAME      0x05
#define DATA_3_IN_DIAGNOSIS_FRAME      0x06
#define DATA_4_IN_DIAGNOSIS_FRAME      0x07
#define SF_MAX_PCI                     0x06
/* Ncr = 1000ms */
#define TP_LAYER_N_CR_TIMEOUT          100

void LIN_TpLayerNCrTimeoutScheduler(void);
void LIN_NodeConfigAndIndentifByDiagnosticFrame(void);
void BSM_ResetInFrameEnd(void);
l_u8 BSM_GetBSMNodeStatus(void);
l_u8 LIN_GetResponseFlagForDiagnosisFrame(void);
void LIN_ClearResponseFlagForDiagnosisFrame(void);

l_u8 LIN_GetSaveNADRequest(void);
l_u8 LIN_GetSavePIDRequest(void);
void LIN_ClearSaveNADRequest(void);
void LIN_ClearSavePIDRequest(void);

#endif
