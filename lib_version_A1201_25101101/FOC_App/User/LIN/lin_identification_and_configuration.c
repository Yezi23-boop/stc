/****************************************************************************
 * @file    : lin_identification_and_configuration.c
 * @author  : Novosns MCU Team
 * @version : V2.0
 * @Date    : 2023/4/7
 * @brief   : lin stack file
 * @note
 * Copyright (C) 2023 Novosense All rights reserved.
 ****************************************************************************/
#include "include.h"
#include <string.h>
#include "lin_define.h"
#include "lin_api.h"
#include "lin_api_appl.h"
#include "lin_config.h"
#include "lin_identification_and_configuration.h"
#include "lin_nvr.h"

/* if 0, ID_61 frames (slave response) are ignored */
static l_u8 g_sendingResponseForConfig = FALSE;
l_u8 g_nad = INITIAL_NAD;   // slave node address
static l_u8 g_saveNADRequest = FALSE;
static l_u8 g_savePIDRequest = FALSE;
static l_u16 g_tpLayerNCrTimeout = 0x00;
static void LIN_AssignNAD(void);
static void LIN_ReadByIdentifier(void);
static void LIN_ConditionalChangeNAD(void);

#ifdef L_UDS_BOOTLOADER_UPDATE
static void LIN_EnterProgramSession(void);
#endif

#ifdef L_LIN_2_2
static void LIN_AssignFrameIDRange(void);
static void LIN_SaveConfiguration(void);
#else
static void LIN_AssignFrame(void);
#endif

static l_bool MatchSupplierID(l_u8 *pSupplierID);
static l_bool MatchFunctionID(l_u8 *pFunctionID);
static l_bool MatchWildcardID(l_u8 *pSupplierID);

#ifdef L_SNPD_BSM
static void BSM_InitShuntMeasurement(void);
static int16_t BSM_ShuntMeasurement(void);
static void BSM_BitEndCallback(int16_t *pShuntCurrent1, int16_t *pShuntCurrent2, int16_t *pShuntCurrent3);
static void BSM_FeCallback(int16_t *pShuntCurrent1, int16_t *pShuntCurrent2, int16_t *pShuntCurrent3);
static void BSM_Reset(void);
static void LIN_SNPD(void);

static l_u8 g_bsmNodeState = L_NODE_BSM_OFF;  // initial state: BSM inactive
static l_u8 g_bsmBit = 0; // current bit in BSM sequence beginning from '1' (incremented each T_bit)
#define ADC_BUF_LEN 13
static uint32_t g_adcRegBuf[ADC_BUF_LEN] = {0};
static uint32_t g_adcBaseAddr = ADC_BASE;
static uint32_t g_adcIENBuf = 0;
static l_u8 g_adcCfgStoreFlg = 0;


#ifdef DEBUG_TRACE_ENABLE
volatile int16_t g_IShunt1;
volatile int16_t g_IShunt2;
volatile int16_t g_IShunt3;
volatile int16_t g_shuntCurrent_1_AfterComp;
volatile int16_t g_shuntCurrent_1_3_AfterComp;
#endif

#endif

/* search config function by SID */
static const SidToConfigFunc g_SID2Callbacks[] = {
    {.SID = 0xB0, .ConfigFunc = LIN_AssignNAD},
#if !(defined L_LIN_2_2)
    {.SID = 0xB1, .ConfigFunc = LIN_AssignFrame},
#endif
    {.SID = 0xB2, .ConfigFunc = LIN_ReadByIdentifier},
    {.SID = 0xB3, .ConfigFunc = LIN_ConditionalChangeNAD},
#ifdef L_UDS_BOOTLOADER_UPDATE
    {.SID = 0x10, .ConfigFunc = LIN_EnterProgramSession},
#endif
#ifdef L_SNPD_BSM
    {.SID = 0xB5, .ConfigFunc = LIN_SNPD},
#endif
#ifdef L_LIN_2_2
    {.SID = 0xB6, .ConfigFunc = LIN_SaveConfiguration},
    {.SID = 0xB7, .ConfigFunc = LIN_AssignFrameIDRange}
#endif
};

/************************************************************
 * @brief: process a master request frame(ID = 60)
 *         The data from the response field is given in
 *         g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var:
 *         byte[0]      NAD
 *         byte[1]      PCI
 *         byte[2]      SID
 *         byte[3..7]  D1..D5
 * @author: Novosns MCU Team
 * @return <None>
 ************************************************************/
void LIN_NodeConfigAndIndentifByDiagnosticFrame(void)
{

#ifdef L_LIN_2_2
    /* Ignore functional NAD but do not discard the pending request */
    if (g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[NAD_INDEX_IN_DIAGNOSIS_FRAME] != FUNCTIONAL_NAD) {
        /* clear any previous request for slave response
           (except, if a functional request has been received) */
        g_sendingResponseForConfig = FALSE;
    }
#else
    g_sendingResponseForConfig = FALSE;// clear any previous request for slave response
#endif


    for (uint16_t i = 0; i < sizeof(g_SID2Callbacks) / sizeof(g_SID2Callbacks[0]); i++) {
        if (g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[SID_INDEX_IN_DIAGNOSIS_FRAME] == g_SID2Callbacks[i].SID) {
            memset(g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var, 0xFF,
                   g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].length);
            g_SID2Callbacks[i].ConfigFunc();
        }
    }
}

static void LIN_SetSendingResponseForConfig(void)
{
    g_sendingResponseForConfig = TRUE;
    g_tpLayerNCrTimeout = TP_LAYER_N_CR_TIMEOUT;
}

/************************************************************
 * @brief: Tp layer Ncr timeout
 * @author: Novosns MCU Team
 * @return  <None>
 ************************************************************/
void LIN_TpLayerNCrTimeoutScheduler(void)
{
    l_irqmask irq_state = l_sys_irq_disable();  // disable all interrupts
    if (g_tpLayerNCrTimeout != 0x00) {
        g_tpLayerNCrTimeout--;
        if (g_tpLayerNCrTimeout == 0x00) {
            g_sendingResponseForConfig = FALSE;
        }
    }
    l_sys_irq_restore(irq_state);  // restore irqmask
}

/************************************************************
 * @brief: Checks if the given Supplier ID matches the LIN Product Identification
 * @author: Novosns MCU Team
 * @param <l_u8> *pSupplierID, the pointer to Supplier ID low byte
 * @return  0 = no match
*           otherwise =  match or wildcard
 ************************************************************/
static l_bool MatchSupplierID(l_u8 *pSupplierID)
{
    return ((pSupplierID[0] == (WILDCARD_SUPPLIER_ID & 0xFF) &&
             pSupplierID[1] == (WILDCARD_SUPPLIER_ID >> 8)) ||
            (pSupplierID[0] == (g_productIdentifier.supplierID & 0xFF) &&
             pSupplierID[1] == (g_productIdentifier.supplierID >> 8)));
}

/************************************************************
 * @brief: Checks if the given Function ID matches the LIN Product Identification
 * @author: Novosns MCU Team
 * @param <l_u8> *pSupplierID, the pointer to Function ID low byte
 * @return 0 = no match
*        otherwise =  match or wildcard
 ************************************************************/
static l_bool MatchFunctionID(l_u8 *pSupplierID)
{
    return ((pSupplierID[0] == (WILDCARD_FUNCTION_ID & 0xFF) &&
             pSupplierID[1] == (WILDCARD_FUNCTION_ID >> 8)) ||
            (pSupplierID[0] == (g_productIdentifier.functionID & 0xFF) &&
             pSupplierID[1] == (g_productIdentifier.functionID >> 8)));
}

/************************************************************
 * @brief: Checks if the given Supplier ID matches the wildcard value
 * @author: Novosns MCU Team
 * @param <l_u8> *pSupplierID, the pointer to Supplier ID low  byte
 * @return 0 = no match
*          otherwise =  match  to wildcard
 ************************************************************/
static l_bool MatchWildcardID(l_u8 *pSupplierID)
{
    return (pSupplierID[0] == (WILDCARD_SUPPLIER_ID & 0xFF) &&
            pSupplierID[1] == (WILDCARD_SUPPLIER_ID >> 8));
}

/************************************************************
 * @brief: Checks if the given NAD
 * @author: Novosns MCU Team
 * @param <l_u8> nad
 * @return 0 = no match
*          otherwise =  match  to wildcard
 ************************************************************/
static l_bool MatchNAD(l_u8 nad)
{
    return (nad == g_nad || nad == WILDCARD_NAD);
}

/************************************************************
 * @brief: Diagnosis function, to assign NAD
 * @author: Novosns MCU Team
 * @return <None>
 ************************************************************/
static void LIN_AssignNAD(void)
{
    /* check initial NAD, PCI, Supplier ID and Function ID */
    if ((g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[NAD_INDEX_IN_DIAGNOSIS_FRAME] == INITIAL_NAD ||
            g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[NAD_INDEX_IN_DIAGNOSIS_FRAME] == WILDCARD_NAD) &&
            g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[PCI_INDEX_IN_DIAGNOSIS_FRAME] == SF_MAX_PCI &&
            MatchSupplierID(&g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[3]) &&
            MatchFunctionID(&g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[5])) {
        /* assign new NAD */
        g_nad  = g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[DATA_4_IN_DIAGNOSIS_FRAME];

        /*  prepare slave response */
        g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[NAD_INDEX_IN_DIAGNOSIS_FRAME] =  INITIAL_NAD; //  NAD
        g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[PCI_INDEX_IN_DIAGNOSIS_FRAME] =  1;           //  PCI
        g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[SID_INDEX_IN_DIAGNOSIS_FRAME] =  0xF0;        //  RSID

        LIN_SetSendingResponseForConfig();
    }
}

#if !(defined L_LIN_2_2)
/************************************************************
 * @brief: Diagnosis function, to assign frame PID, this function only called In LIN 2.0 stack
 * @author: Novosns MCU Team
 * @return <None>
 ************************************************************/
static void LIN_AssignFrame(void)
{
    l_u16 tmp_messageID;
    /* check NAD, PCI and Supplier ID */
    if (MatchNAD(g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[NAD_INDEX_IN_DIAGNOSIS_FRAME]) &&
            g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[PCI_INDEX_IN_DIAGNOSIS_FRAME] == SF_MAX_PCI &&
            MatchSupplierID(&g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[3])) {
        //assign frame ID
        tmp_messageID = g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[DATA_2_IN_DIAGNOSIS_FRAME] +
                        (g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[DATA_3_IN_DIAGNOSIS_FRAME] << 8);
        /* -2  since  g_configMessageID[]  doesn't contain entries for ID_60 and ID_61 */
        for (l_u8 i = 0; i < L_ID_TABLE_SIZE - 2; i++) {
            if (g_configMessageIDTable[i] == tmp_messageID) {
                g_nodeFramePIDTable[i] = g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[DATA_4_IN_DIAGNOSIS_FRAME];

                //  prepare slave response
                g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[NAD_INDEX_IN_DIAGNOSIS_FRAME] =  g_nad;  //  NAD
                g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[PCI_INDEX_IN_DIAGNOSIS_FRAME] =  1;      //  PCI
                g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[SID_INDEX_IN_DIAGNOSIS_FRAME] =  0xF1;   //  RSID

                LIN_SetSendingResponseForConfig();
                break;
            }
        }
    }
}
#endif

#ifdef L_UDS_BOOTLOADER_UPDATE
/************************************************************
 * @brief: Diagnosis function, Program session and reset jmp to bootloader 
 * according to ISO 14229-1
 * @author: Novosns MCU Team
 * @return <None>
 ************************************************************/
static void LIN_EnterProgramSession(void)
{
    /* check initial NAD, PCI, and subfunction */
    if (MatchNAD(g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[NAD_INDEX_IN_DIAGNOSIS_FRAME]) &&
        g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[PCI_INDEX_IN_DIAGNOSIS_FRAME] == 2 && 
        g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[DATA_0_IN_DIAGNOSIS_FRAME] == 0x02) {
        SetRequestEnterBootloader();
        /* system reset  */
        NVIC_SystemReset();
        while (1) {
        }
    }
}
#endif

/************************************************************
 * @brief: Diagnosis function, to read identifier, such as suppierID, function ID.
 * @author: Novosns MCU Team
 * @return <None>
 ************************************************************/
static void LIN_ReadByIdentifier(void)
{
    // check  NAD, PCI, Supplier ID and  Function ID
    if (MatchNAD(g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[NAD_INDEX_IN_DIAGNOSIS_FRAME]) &&
            g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[PCI_INDEX_IN_DIAGNOSIS_FRAME] == SF_MAX_PCI &&
            MatchSupplierID(&g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[4]) &&
            MatchFunctionID(&g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[6])) {
        /* Check Identifier */
        if (g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[3] == 0) {// read LIN  Product Identification
            /*  prepare slave response  */
            g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[NAD_INDEX_IN_DIAGNOSIS_FRAME] =  g_nad;     //  NAD
            g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[PCI_INDEX_IN_DIAGNOSIS_FRAME] =  0x06;      //  PCI
            g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[SID_INDEX_IN_DIAGNOSIS_FRAME] =  0xF2;      //  RSID = SID+0x40
            g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[DATA_0_IN_DIAGNOSIS_FRAME] =  g_productIdentifier.supplierID & 0xFF;// supplier ID
            g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[DATA_1_IN_DIAGNOSIS_FRAME] =  g_productIdentifier.supplierID >>  8;
            g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[DATA_2_IN_DIAGNOSIS_FRAME] =  g_productIdentifier.functionID & 0xFF;// function ID
            g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[DATA_3_IN_DIAGNOSIS_FRAME] =  g_productIdentifier.functionID >>  8;
            g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[DATA_4_IN_DIAGNOSIS_FRAME] =  g_productIdentifier.variantID; // variant
        } else if ( g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[3] == 1) {//  read serial  number
            /* prepare slave response */
            g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[NAD_INDEX_IN_DIAGNOSIS_FRAME] =  g_nad;     //  NAD
            g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[PCI_INDEX_IN_DIAGNOSIS_FRAME] =  0x05;      //  PCI
            g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[SID_INDEX_IN_DIAGNOSIS_FRAME] =  0xF2;      //  RSID = SID+0x40
            g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[DATA_0_IN_DIAGNOSIS_FRAME] =  L_SERIAL_NUMBER & 0xFF;
            g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[DATA_1_IN_DIAGNOSIS_FRAME] =  (L_SERIAL_NUMBER >> 8) & 0xFF;
            g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[DATA_2_IN_DIAGNOSIS_FRAME] =  (L_SERIAL_NUMBER >> 16) & 0xFF;
            g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[DATA_3_IN_DIAGNOSIS_FRAME] =  (L_SERIAL_NUMBER >> 24) & 0xFF;
        }
#if !(defined L_LIN_2_2)
        else if ((g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[3] >= 16) &&
                 (g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[3] < (16 + L_ID_TABLE_SIZE - 2))) { //  read message ID
            uint16_t i = g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[3] - 16;      //  index
            l_u16 tmp_messageID = g_configMessageIDTable[i];
            /* prepare slave response */
            g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[NAD_INDEX_IN_DIAGNOSIS_FRAME] =  g_nad;//  NAD
            g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[PCI_INDEX_IN_DIAGNOSIS_FRAME] =  0x04;//  PCI
            g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[SID_INDEX_IN_DIAGNOSIS_FRAME] =  0xF2;//  RSID
            g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[DATA_0_IN_DIAGNOSIS_FRAME] =  tmp_messageID;
            g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[DATA_1_IN_DIAGNOSIS_FRAME] =  tmp_messageID >> 8;
            g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[DATA_2_IN_DIAGNOSIS_FRAME] =  g_nodeFramePIDTable[i];
            g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[DATA_3_IN_DIAGNOSIS_FRAME] =  0xFF;   //  unused
            g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[DATA_4_IN_DIAGNOSIS_FRAME] =  0xFF;   //  unused
        }
#endif
        else {
            /* prepare slave response */
            g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[NAD_INDEX_IN_DIAGNOSIS_FRAME] =  g_nad;//  NAD
            g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[PCI_INDEX_IN_DIAGNOSIS_FRAME] =  0x03; //  PCI
            g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[SID_INDEX_IN_DIAGNOSIS_FRAME] =  0x7F; //  RSID
            g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[DATA_0_IN_DIAGNOSIS_FRAME] =  0xB2;    //  requested SID
            g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[DATA_1_IN_DIAGNOSIS_FRAME] =  0x12;    //  error code
        }

        LIN_SetSendingResponseForConfig();
    }
}

/************************************************************
 * @brief: Diagnosis function, to change NAD
 * @author: Novosns MCU Team
 * @return <None>
 ************************************************************/
static void LIN_ConditionalChangeNAD(void)
{
    l_u8 tmp;
    //  check initial NAD, PCI, Supplier ID and Function ID
    if (MatchNAD(g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[NAD_INDEX_IN_DIAGNOSIS_FRAME]) &&
            g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[PCI_INDEX_IN_DIAGNOSIS_FRAME] == SF_MAX_PCI) {
        g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[4]--;//  set 'byte' to 0..x
        /*  check for id=0 (supplier, function or variant ID) */
        if (g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[3] == 0 &&
                g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[4] <= 4) {
            /*  check if 'byte' is in the valid range */
            switch (g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[4]) {
            case 0:
                tmp =  L_SUPPLIER_ID & 0xFF;
                break;
            case 1:
                tmp =  (L_SUPPLIER_ID >> 8)  & 0xFF;
                break;
            case 2:
                tmp =  L_FUNCTION_ID & 0xFF;
                break;
            case 3:
                tmp =  (L_FUNCTION_ID >> 8)  & 0xFF;
                break;
            case 4:
                tmp =  L_VARIANT_ID;
                break;
            }
            /* check for id=1 (serial number) and check if 'byte' is in the valid range */
        } else if (g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[3] == 1 &&
                   g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[4] <= 3) {
            /* load the byte that has to be checked */
            tmp =  g_serialNumber[g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[4]];
        } else {
            return;
        }

        /* Check  the selected id field:
          see LIN spec. ch.  "2.6.2 Conditional change NAD"
          ...
          3.  Do a bitwise XOR with Invert.
          4.  Do a bitwise AND with Mask.
          5.  If the final result is zero then change the NAD to New NAD. */
        if (((tmp ^ g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[6]) &
                g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[5]) == 0) {
            /* prepare slave response */
            g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[NAD_INDEX_IN_DIAGNOSIS_FRAME] = g_nad;// the slave response must contain the old NAD
            g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[PCI_INDEX_IN_DIAGNOSIS_FRAME] = 1;    //  PCI
            g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[SID_INDEX_IN_DIAGNOSIS_FRAME] = 0xF3; //  RSID = SID+0x40

            g_nad  = g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[DATA_4_IN_DIAGNOSIS_FRAME];   // assign new NAD
            LIN_SetSendingResponseForConfig();
        }
    }
}

#if defined L_LIN_2_2
/************************************************************
 * @brief: Diagnosis function, to assign frame PID, this function only called In LIN 2.2 stack
 * @author: Novosns MCU Team
 * @return <None>
 ************************************************************/
static void LIN_AssignFrameIDRange(void)
{
    l_u8 tmp;
    // check NAD and PCI
    if (MatchNAD(g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[NAD_INDEX_IN_DIAGNOSIS_FRAME]) &&
            g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[PCI_INDEX_IN_DIAGNOSIS_FRAME] == SF_MAX_PCI) {
        /* load the start index (0 to 7) from D1 */
        tmp =  g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[3];
        /* check there are valid PIDs in the data field (D2 to D5) */
        for (l_u8 i = 0; i < 4; i++) {
            uint8_t newPID = g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[4 + i];
            /* 0xFF => don't care */
            if (newPID != 0xFF) {
                /* check on valid range before writing to l_PID_Table[] */
                if ((tmp + i) < (L_ID_TABLE_SIZE - 2)) {
                    g_nodeFramePIDTable[tmp + i] = (newPID != 0x00) ? newPID : 0xFF;
                }
            }
        }
        /* prepare slave response */
        g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[NAD_INDEX_IN_DIAGNOSIS_FRAME] =  g_nad;//  NAD
        g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[PCI_INDEX_IN_DIAGNOSIS_FRAME] =  1;    //  PCI
        g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[SID_INDEX_IN_DIAGNOSIS_FRAME] =  0xF7; //  RSID

        LIN_SetSendingResponseForConfig();
    }
}

/************************************************************
 * @brief: Diagnosis function, to save configuration in NVR, this function only called In LIN 2.2 stack
 * @author: Novosns MCU Team
 * @return <None>
 ************************************************************/
static void LIN_SaveConfiguration(void)
{
    // check NAD and PCI
    if (MatchNAD(g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[NAD_INDEX_IN_DIAGNOSIS_FRAME]) &&
            g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[PCI_INDEX_IN_DIAGNOSIS_FRAME] == 0x01) {

        /* prepare slave response */
        g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[NAD_INDEX_IN_DIAGNOSIS_FRAME] =  g_nad;//  NAD
        g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[PCI_INDEX_IN_DIAGNOSIS_FRAME] =  0x01; //  PCI
        g_nodeConfigFrameTable[DIAGNOSIS_RESPONSE_FRAME].var[SID_INDEX_IN_DIAGNOSIS_FRAME] =  0xF6; //  RSID
                
        LIN_SetSendingResponseForConfig();
        g_saveNADRequest = TRUE;
        g_savePIDRequest = TRUE;
    }

}
#endif

#ifdef L_SNPD_BSM

/************************************************************
 * @brief: Diagnosis function, to slave node position detection by bus shunt method
 * @author: Novosns MCU Team
 * @return <None>
 ************************************************************/
static void LIN_SNPD(void)
{
    // check initial NAD, PCI, Supplier ID and Function ID (MSB), and check on Bus Shunt Method 1
    if (g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[NAD_INDEX_IN_DIAGNOSIS_FRAME] == WILDCARD_NAD &&
            g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[PCI_INDEX_IN_DIAGNOSIS_FRAME] == SF_MAX_PCI &&
            MatchWildcardID(&g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[3]) &&
            g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[6] == 0x02) {

        //  Check Function ID (LSB)
        if (g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[5] == 0x01) { //  BSM Initialization

            // start of BSM sequence
            g_bsmNodeState = L_NODE_INIT;  // BSM=active, node is not yet addressed
            g_bsmBit = 0;                  // enable BSM sequence (started with next falling LIN edge)

#ifdef DEBUG_TRACE_ENABLE // for debugging
            GPIO->PDO_b.DO3 = 1;
#endif
#ifdef L_NAD_NVR
            ReadNADFromNVR();
#else
            g_nad = INITIAL_NAD;      // reset NAD
#endif

            // LIN I/O is already enabled
            LINPORT->ISR = 0xFFFF;             // reset all LIN IRQ pending flags
            LINPORT->IEN = LIN_LINFE;          // enable LIN falling edge interrupt
            NVIC_SetPriority(LINPORT_IRQn, LIN_IRQ_PRIO);
            NVIC->ISER[0] = 1 << LINPORT_IRQn; // enable LIN interrupt line(s)
        } else if (g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[5] == 0x02) { // Assign NAD to slave
            if (g_bsmNodeState == L_NODE_ADDRESSED) {   // node has been addressed within the current frame
                /* take the NAD from this frame and store it to NVR */
                g_nad = g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[7];
                g_bsmNodeState = L_NODE_NAD_STORED;
#ifdef DEBUG_TRACE_ENABLE // for debugging
                GPIO->PDO_b.DO3 = 0;
#endif
            }
        } else if (g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[5] == 0x03) {// optional: Store NAD in slave
            g_saveNADRequest = TRUE;
        } else if (g_nodeConfigFrameTable[DIAGNOSIS_REQUEST_FRAME].var[5] == 0x04) {// Assign NAD finished
            BSM_Reset();
            g_bsmNodeState = L_NODE_BSM_OFF; // last BSM command => BSM sequence finished
        } else {
            // error: unsupported function ID
        }
    }
}

/************************************************************
 * @brief: Get BSM node status
 * @author: Novosns MCU Team
 * @return The node staus of BSM
 ************************************************************/
l_u8 BSM_GetBSMNodeStatus(void)
{
    return g_bsmNodeState;
}

/************************************************************
 * @brief: Reset the BSM sequence, further BSM commands might be received
 * @author: Novosns MCU Team
 * @return <None>
 ************************************************************/
static void BSM_Reset(void)
{
    NVIC->ICER[0]  = 1 << LINPORT_IRQn;// disable LIN interrupt line(s)
    LINPORT->IEN = 0;                  // disable LIN interrupt(s)
    LINPORT->BSMCR = 0x02;             // BSM disable, Pull-Up = On, Current Source = Off
    ADC->CR_b.EN = 0;                  // switch ADC off (to reduce current consumption)
    /* restore ADC config */
    if (g_adcCfgStoreFlg) {
        for (l_u8 i = 0; i < ADC_BUF_LEN; i++) {
            *((uint32_t*)(g_adcBaseAddr + i * 4)) = g_adcRegBuf[i];
        }
        ADC->IEN = g_adcIENBuf;
    }
    g_bsmBit = 0;
}

/************************************************************
 * @brief: Reset the BSM sequence, further BSM commands might be received
 * @author: Novosns MCU Team
 * @return <None>
 ************************************************************/
void BSM_ResetInFrameEnd(void)
{
    if (g_bsmNodeState != L_NODE_BSM_OFF) {// BSM == active ?
        
        g_bsmBit = 0;                      // re-enable BSM sequence (started with next falling LIN edge)
        // LIN I/O is already enabled
        LINPORT->ISR = 0xFFFF;             // reset all LIN IRQ pending flags (before enabling interrupt)
        LINPORT->IEN = LIN_LINFE;          // enable LIN falling edge interrupt
        NVIC->ISER[0]  = 1 << LINPORT_IRQn;// enable LIN interrupt line(s)
    }
}

/************************************************************
 * @brief: Initialize ADC for current measurement via the LIN bus shunt
 * @author: Novosns MCU Team
 * @return <None>
 ************************************************************/
static void BSM_InitShuntMeasurement(void)
{
    /* store ADC reg */
    for (l_u8 i = 0; i < ADC_BUF_LEN; i++) {
        g_adcRegBuf[i] = *((uint32_t*)(g_adcBaseAddr + i * 4));
    }
    g_adcIENBuf = ADC->IEN;
    g_adcCfgStoreFlg = 1;
    /* configurate ADC */
    ADC->CR_b.TSRC = 0; // software trigger
    ADC->CR_b.CKSEL = 0; // 48 Mhz
    ADC->CR_b.INBUFBYPASS = 1;
    ADC->CR_b.REFPSEL = 0; //internal ref
    ADC->CR_b.REFNSEL = 0;  
    ADC->CR_b.SINGLEEN = 0;// queue converted. trigger EOC
    /* config delay stable convert sample time */
    ADC->SCR_b.SAMP   = 25;
    ADC->IEN = 0x00;

    ADC->QCR1 = 0;
    ADC->QCR2 = 0;
    ADC->QCR1_b.Q1EN  = 1;
    ADC->QCR1_b.Q1SEL = 5; // 5 LIN BSM Channel
    ADC->QTR1_b.ABS = 0;
    ADC->QTR1_b.QTTR = 1;
    ADC->CR_b.EN = 1;   // enable ADC
}

/************************************************************
 * @brief: Initialize ADC for current measurement via the LIN bus shunt
 * @author: Novosns MCU Team
 * @param <int16_t> *pShuntCurrent1, the pointer to save current 1 value
 * @param <int16_t> *pShuntCurrent2, the pointer to save current 2 value
 * @param <int16_t> *pShuntCurrent3, the pointer to save current 3 value
 * @return <None>
 ************************************************************/
static void BSM_FeCallback(int16_t *pShuntCurrent1, int16_t *pShuntCurrent2, int16_t *pShuntCurrent3)
{
    if (g_bsmBit == 0) {                // 1st bit has been started (T_bit = 0)
        LINPORT->ISR = LIN_BEND;
        LINPORT->IEN = LIN_BEND;        // disable falling edge and enable bit-end interrupt

        // BSM step 0: switch off Pull-Up and current source
        LINPORT->BSMCR = 0x01;          // BSM enable, Pull-Up=Off, Current Source=Off

        *pShuntCurrent1 = 0;            // reset currents (for cumulative measurements with averaging)
        *pShuntCurrent2 = 0;
        *pShuntCurrent3 = 0;

        // Fix: If a node is addressed (and NAD not yet stored), e.g. within a BREAK of an optional "non BSM command",
        // it has to be reset to INIT state, in order to participate in the next BSM sequence.
        if ((g_bsmNodeState != L_NODE_BSM_OFF) &&   // if BSM is active and
                (g_bsmNodeState < L_NODE_NAD_STORED)) { // NAD not yet accepted
            g_bsmNodeState = L_NODE_INIT;           // reset BSM sequence
            BSM_InitShuntMeasurement();
        }
    }
}

/************************************************************
 * @brief: Initialize ADC for current measurement via the LIN bus shunt
 * @author: Novosns MCU Team
 * @param <int16_t> *pShuntCurrent1, the pointer to save current 1 value
 * @param <int16_t> *pShuntCurrent2, the pointer to save current 2 value
 * @param <int16_t> *pShuntCurrent3, the pointer to save current 3 value
 * @return <None>
 ************************************************************/
static void BSM_BitEndCallback(int16_t *pShuntCurrent1, int16_t *pShuntCurrent2, int16_t *pShuntCurrent3)
{
    if (g_bsmBit < 12) {
        g_bsmBit++;
    }

    switch (g_bsmBit) {
        /* BSM step 1: measure offset current (I_sh1) */
    case 1:
    case 2:
    case 3:
        /* if BSM is active and node is not yet addressed , just to measure bus shunt value*/
        if ((g_bsmNodeState != L_NODE_BSM_OFF) && (g_bsmNodeState < L_NODE_ADDRESSED)) {
            *pShuntCurrent1 += BSM_ShuntMeasurement();
        }
        break;
        /* BSM step 2: swtich on current source 0 */
    case 4:
        /* if BSM is active and node is not yet addressed */
        if ((g_bsmNodeState != L_NODE_BSM_OFF) && (g_bsmNodeState < L_NODE_ADDRESSED)) {
            *pShuntCurrent1 /= 3;            // build average from the 3 measurements

#ifdef DEBUG_TRACE_ENABLE
            g_IShunt1 = *pShuntCurrent1;
#endif
            /* swtich on current source 0 */
            LINPORT->BSMCR_b.SWCS0 = 1;
        }
        break;
        /* BSM step 3: measure current source 0 current (I_sh2) */
    case 5:
    case 6:
    case 7:
        /* if BSM is active and node is not yet addressed , just to measure bus shunt value*/
        if ((g_bsmNodeState != L_NODE_BSM_OFF) && (g_bsmNodeState < L_NODE_ADDRESSED)) {
            *pShuntCurrent2 += BSM_ShuntMeasurement();
        }
        break;
        /* BSM step 4: switch on current source */
    case 8:
        /* if BSM is active and node is not yet addressed */
        if ((g_bsmNodeState != L_NODE_BSM_OFF) && (g_bsmNodeState < L_NODE_ADDRESSED)) {
            *pShuntCurrent2 /= 3;            // build average from the 3 measurements
#ifdef DEBUG_TRACE_ENABLE
            g_IShunt2 = *pShuntCurrent2;
            g_shuntCurrent_1_AfterComp = (*pShuntCurrent2 - *pShuntCurrent1);
#endif
            if ((*pShuntCurrent2 - *pShuntCurrent1) > SNPD_I_DIFF) {
                g_bsmNodeState = L_NODE_INIT;
                LINPORT->BSMCR_b.SWCS0 = 0;      // Current Source0 = Off
            } else {
                g_bsmNodeState = L_NODE_SELECTED;
                LINPORT->BSMCR_b.SWCS2 = 1;      // Current Source2 = On
            }
        }
        break;
        /* BSM step 5: measure current source 0&1 current (I_sh3) */
    case 9:
    case 10:
    case 11:
        /* if node is pre-selected */
        if (g_bsmNodeState == L_NODE_SELECTED) {
            *pShuntCurrent3 += BSM_ShuntMeasurement();
        }
        break;
        /* BSM step 6: switch off current source */
    case 12:
        /* if node is pre-selected */
        if (g_bsmNodeState == L_NODE_SELECTED) {
            *pShuntCurrent3 /= 3; // build average from the 3 measurements

#ifdef DEBUG_TRACE_ENABLE
            g_IShunt3 = *pShuntCurrent3;
            g_shuntCurrent_1_3_AfterComp = (*pShuntCurrent3 - *pShuntCurrent1);
#endif
            if ((*pShuntCurrent3 - *pShuntCurrent1) > SNPD_I_DIFF) {
                g_bsmNodeState = L_NODE_INIT;
            } else {
                /* store NAD from the current frame to g_nad */
                g_bsmNodeState = L_NODE_ADDRESSED;
            }
        }
        /* one BSM command completed (further may follow) */
        BSM_Reset();
        break;
    }

}

/************************************************************
 * @brief: Perform current measurement via the LIN bus shunt
 * @author: Novosns MCU Team
 * @return <None>
 ************************************************************/
static int16_t BSM_ShuntMeasurement(void)
{
#define N_SAMPLES 5
    int16_t adcRamDataSum = 0;
    int adcSampleCnt = 0;
    for (adcSampleCnt = 0; adcSampleCnt < N_SAMPLES; adcSampleCnt++) {
        ADC->ISR = 0xFFFFFFFF;
        /* start conversion */
        ADC->CR_b.SWT = 1;
        /* wait until ready */
        while (!ADC->ISR_b.EOCIF);
        /* read ADC data */
        adcRamDataSum += ADC->DR1_b.DATA;
    }
    while (adcSampleCnt < N_SAMPLES);
    return (adcRamDataSum / N_SAMPLES);
}

/************************************************************
 * @brief: LINPORT interrupt handler
 * @author: Novosns MCU Team
 * @return <None>
 ************************************************************/
void LINPORT_IRQHandler(void)
{
    static int16_t shuntCurrent1 = 0x00;    // SNPD shunt current 1
    static int16_t shuntCurrent2 = 0x00;    // SNPD shunt current 2
    static int16_t shuntCurrent3 = 0x00;    // SNPD shunt current 3

    //---------------------------------------------------------------------------
    // During BSM sequence:
    // check on LINRE (rising edge), LINFE (falling edge) and BEND (bit end) interrupts
    //---------------------------------------------------------------------------

    /* Check for rising edge within BSM measurement. This could occur if a non-BSM frame
       is inserted in the BSM sequence. In this case this measurement will be ignored and
       started again with the next falling edge. */
    if (LINPORT->ISR_b.LINREIF) {         // rising edge within BSM sequence => begin of BREAK
        if (LINPORT->ISR_b.LINFEIF == 0) {// if there was no falling edge detected
            LINPORT->ISR = LIN_LINRE;     // clear pending flag
            BSM_Reset();                  // BSM command aborted (further may follow)
            ResetProtocolState();
        }
    }

    if (LINPORT->ISR_b.LINFEIF) {         // falling edge interrupt = begin of BREAK
        LINPORT->ISR = LIN_LINFE | LIN_LINRE; // clear pending flag (also for rising edge)
        BSM_FeCallback(&shuntCurrent1, &shuntCurrent2, &shuntCurrent3);
    }

    if (LINPORT->ISR_b.BENDIF) {          // bit-end interrupt
        LINPORT->ISR = LIN_BEND;          // clear pending flag
        BSM_BitEndCallback(&shuntCurrent1, &shuntCurrent2, &shuntCurrent3);
    }
}
#endif


/************************************************************
 * @brief: Checks Response for frame diagnosis
 * @author: Novosns MCU Team
 * @return  1, Reponse, 0, No Response
 ************************************************************/
l_u8 LIN_GetResponseFlagForDiagnosisFrame(void)
{
    return g_sendingResponseForConfig;
}

/************************************************************
 * @brief: clear Response for frame diagnosis
 * @author: Novosns MCU Team
 * @return <None>
 ************************************************************/
void LIN_ClearResponseFlagForDiagnosisFrame(void)
{
    g_sendingResponseForConfig = FALSE;
}

/************************************************************
 * @brief: clear save NAD request
 * @author: Novosns MCU Team
 * @return <None>
 ************************************************************/
l_u8 LIN_GetSaveNADRequest(void)
{
    return g_saveNADRequest;
}

/************************************************************
 * @brief: Checks save PID request
 * @author: Novosns MCU Team
 * @return  1, Reponse, 0, No Response
 ************************************************************/
l_u8 LIN_GetSavePIDRequest(void)
{
    return g_savePIDRequest;
}
/************************************************************
 * @brief: clear save NAD request
 * @author: Novosns MCU Team
 * @return <None>
 ************************************************************/
void LIN_ClearSaveNADRequest(void)
{
    g_saveNADRequest = FALSE;
}

/************************************************************
 * @brief: clear save PID request
 * @author: Novosns MCU Team
 * @return <None>
 ************************************************************/
void LIN_ClearSavePIDRequest(void)
{
    g_savePIDRequest = FALSE;
}



