/****************************************************************************
 * @file    : lin_init.c
 * @author  : Nonosns MCU Team
 * @version : V2.0
 * @Date    : 2023/4/11
 * @brief   : lin stack file
 * @note
 * Copyright (C) 2023 Novosense All rights reserved.
 ****************************************************************************/


/*******************************************************************************
* Include files
*******************************************************************************/
#include "include.h"
#include <string.h>
#include "lin_define.h"
#include "lin_api.h"
#include "lin_api_appl.h"
#include "lin_config.h"
#include "lin_identification_and_configuration.h"
#include "lin_nvr.h"
#include "lin_init.h"

extern volatile LINStatus g_myLinStatus;
extern l_u8 g_dataBuffTmp[10];                    // temporary buffer for UART communication.
                                                  // holds data received or to be transmitted.
                                                  // byte_0: PID
                                                  // byte_1..8: data
                                                  // byte 9: checksum
extern l_u8 g_nowFrameExpectedByte;               // data Bytes expected (in RX) or to be sent (in TX)
extern const ConfigFrame *g_currentFramePointer;  // pointer to the  current scheduled  message
extern l_u8 g_frameErr;                           // frame error detected when slave response is expected
l_u16 g_busIdleTimeoutCounter = 0xFFFF;             // bus idle timeout measurement disabled

LIN_ERROR g_allLinError;
uint32_t g_linTableSize = L_ID_TABLE_SIZE;

/************************************************************
 * @brief: To initialize wakeup module for wakeup via LIN
 * @author: Novosns MCU Team
 * @return <None>
 ************************************************************/
void PrepareToWakeup(void)
{
}

/************************************************************
 * @brief: Write the current value of the Response_Error signal to the frame
 *          carrying the response error signal
 * @author: Novosns MCU Team
 * @param <l_bool> data, error state
 * @return <None>
 ************************************************************/
static void WriteResponseError(l_bool data)
{
    if (data) {
        g_nodeConfigFrameTable[RESPONSE_ERROR_FRAME].var[RESPONSE_ERROR_BYTE] |= RESPONSE_ERROR_BIT;
    } else {
        g_nodeConfigFrameTable[RESPONSE_ERROR_FRAME].var[RESPONSE_ERROR_BYTE] &= ~RESPONSE_ERROR_BIT;
    }
}

// define reponse error for user
void SetLINResponseErrorForUser(void)
{
    if (g_allLinError.LinError_b.erCKSUM ||
        g_allLinError.LinError_b.erIDPAR || 
        g_allLinError.LinError_b.erIDSTOP ||
        g_allLinError.LinError_b.erRX ||
        g_allLinError.LinError_b.erSHORT ||
        g_allLinError.LinError_b.erSYNC ||
        g_allLinError.LinError_b.erTORESP ||
        g_allLinError.LinError_b.erTXCOL) {
        WriteResponseError(1);
    }
}

// define reponse error for user
void ResetLINResponseErrorForUser(void)
{
    if (g_myLinStatus.task == L_TASK_SEND_DATA && g_dataBuffTmp[0] == g_nodeFramePIDTable[RESPONSE_ERROR_FRAME]) {
        WriteResponseError(0);  // reset Response_Error signal
        g_allLinError.LinError = 0x00;
    }
}

/************************************************************
 * @brief: see LIN 2.0 specification, chapter "CORE API"
 * @author: Novosns MCU Team
 * @return <None>
 ************************************************************/
l_bool l_sys_init(void)
{
    /* prepare wakeup module for the case that power saving mode is entered unintentionally */
    PrepareToWakeup();

#ifdef L_NAD_NVR
    ReadNADFromNVR();  //  Read NAD from NVR (if valid)
#endif

#ifdef L_PID_NVR
    ReadPIDFromNVR(g_nodeFramePIDTable, L_ID_TABLE_SIZE - 2);  // Read NAD from NVR (if valid)
#endif

    /*  Reset all the Status Flags */
    g_myLinStatus.networkState = L_NT_INIT;
    g_myLinStatus.protocolState = L_PROT_INIT;
    g_myLinStatus.task = L_TASK_IDLE;

    /* reset all mask bits */
    g_allLinError.LinError = 0x00;

    return 0;
}

/************************************************************
 * @brief: LINUART interrupt handler
 * @author: Novosns MCU Team
 * @return <None>
 ************************************************************/
void LINUART_IRQHandler(void)
{
    //---------------------------------------------------------------------------
    // check on FE error interrupt when processing slave response
    // This part has been moved before the processing of BRROK interrupt.
    // Otherwise, after no slave response has been received, a FRM error in
    // SYNCH or PID (due to an acceptable clock drift) will lead to LRE error
    // during the processing of the BR/SY interrupt.
    //---------------------------------------------------------------------------

    if ((LINUART->ISR_b.FEIF) && (g_myLinStatus.task == L_TASK_GET_DATA)) {
        /* Detect a missing or incomplete slave response by checking on FRM errors interrupts (LIN2.1 requirement):
           The FRM_int can occur at the following conditions:
           1. no slave response => FRM_int @ next BREAK => LRE must not be set
           2. response incomplete, form errors in response => FRM_int @ corrupted byte of response => LRE must be set
           3. response incomplete, no form errors in response => FRM_int @ next BREAK, LINUART->SR_b.RXFE=0 => LRE must be set */

        LINUART->ISR = LINUART_FE | LINUART_RXFE;
        // set error immediately, if FIFO is not empty
        if (g_frameErr || (LINUART->ISR_b.RXFEIF == 0)) {
            NotifyRxTxStatus(LIN_BIT_ERROR);
            g_allLinError.LinError_b.erRX = 1;
            // error found:
            // - set error status
            // - reset protocol state and switch UART off
            // - Start Bus Idle Measurement
        } 
        g_frameErr = 1; // set FE error detection
    }

    /* check on BRROK interrupt */
    if (LINUART->ISR_b.BRSYIF) {
        LINUART->ISR = LINUART_BRSY;  // clear BRSY pending flag

        /* Fix for LIN 2.1 Test Case 3.8.2 "Incomplete Frame Reception":
           Frame error is ignored after reception of header.
           Thus the error flag is not set after reception of an incomplete header. */
        LINUART->ISR = LINUART_FE;

        // Check on FE, FIFO OVERFLOW or TX error in previous frame
        if (LINUART->ISR_b.RXFOVIF | LINUART->ISR_b.TXERRIF) {
            // reset the UART error pending flags RXFO and TXERR
            LINUART->ISR = LINUART_TXERR | LINUART_RXFO;
            g_allLinError.LinError_b.erTXCOL = 1;
            // error found, set appropriate error status
            //SetLINResponseError();                     
        }

        // The brrok interrupt sets:
        // LINUART->CR.TXRXEN = 1
        // LINUART->CR.RXFCV = 1 (FF interrupt after next byte)
        g_myLinStatus.networkState = L_NT_ACTIVE;    //  Network is active now
        g_myLinStatus.task = L_TASK_IDLE;            //  reset task status
        g_myLinStatus.protocolState = L_PROT_IDENT;  //  PID received
        /* disable possibly set TX or TXERR interrupt and keep RXFC interrupt enabled */
        LINUART->IEN = LINUART_RXFL | LINUART_BRSY;
        SetLINResponseErrorForUser();
    }

    /* check on Rx/Tx FIFO compare or TXERR or FE interrupt */
    if (LINUART->ISR & (LINUART_RXFL | LINUART_TXFC | LINUART_TXERR | LINUART_FE | LINUART_RXFO)) {
        /* check on end of WAKE command */
        if (g_myLinStatus.task == L_TASK_WAKE) {
            /* disable UART and reset state variables */
            ResetProtocolState();
        } else {
            /* handle the interrupt */
            l_ifc_rx_MyLinIfc();
        }
        /* clear RXFL and/or TXFC pending flag 
           (pending flag for TXERR and FE are checked and reset in l_ifc_rx_MyLinIfc) */
        LINUART->ISR = LINUART_RXFL | LINUART_TXFC | LINUART_TXERR | LINUART_FE | LINUART_RXFO;
    }
}

/************************************************************
 * @brief: init the LINUART
 * @author: Novosns MCU Team
 * @return <None>
 ************************************************************/
void LINUART_Init(void)
{
    /* UART Rx Pin routed to LIN */
    LINPORT->CR_b.LAIS = 0;
    LINPORT->CR_b.TXOS = 0;
    LINPORT->CR_b.RXEN = 1;
    LINPORT->CR_b.SR = 1;
    /* Linport still work at uv mode */
    LINPORT->CR_b.LOM = 1;
    /* Enable Linport */
    LINPORT->CR_b.LINPORTEN = 1;

    /* Disable UART when changing configuration */
    LINUART->CR = 0x00;

    LINUART->CR_b.TXFE = 1;
    LINUART->CR_b.RXFE = 1;

    LINUART->CR_b.TXLOOPE = 1;
    LINUART->CR_b.BRRWINE = 1;
    LINUART->CR_b.LINMODE = 1;
    
    LINUART->IEN = LINUART_RXFL | LINUART_BRSY;
    /* support max baudRate 100k, WINDW_MIN = 48000000 / 100000 / 1.2 = 400*/
    LINUART->WINDW_MIN_b.WINDW_MIN = 400;
    /* support min baudRate 1k, WINDW_MIN = 48000000 / 1000 * 1.2  = 0xE100 */
    LINUART->WINDW_MAX_b.WINDW_MAX = 0xE100;

    NVIC_EnableIRQ(LINUART_IRQn);
    NVIC_SetPriority(LINUART_IRQn, LINUART_IRQ_PRIO);

    /* reset all UART IRQ pending flags */
    LINUART->ISR = 0xFFFFFFFF;
    /* When detec LIN Frame break + sync, Disable TX RX function,
       TXRXEN will be enabled if the break + sync frame be deteced. */
    LINUART->CR_b.TXRXE = 0;
    /* Enable auto baudrate */
    LINUART->CR_b.AUTOBRR = 1;
    /* Enable enhance auto baudrate */
    LINUART->CR_b.EAUTOBRR = 1;
}


/************************************************************
 * @brief: Enable the LINUART
 * @author: Novosns MCU Team
 * @return <None>
 ************************************************************/
void LINUART_Enable(void)
{
    LINUART->CR_b.UE = 1;  // enable LINUART
}

/************************************************************
 * @brief: Disable the LIN_UART
 * @author: Novosns MCU Team
 * @return <None>
 ************************************************************/
void LINUART_Disable(void)
{
    NVIC->ICER[0] = 1 << LINUART_IRQn;  // disable LINUART interrupt line(s)
    LINUART->CR_b.UE = 0;               // disable UART
}

/************************************************************
 * @brief: handling of message data to be sent to the UART
 *         status:  OK if msg has been RX / TX successfully
 *         NOK otherwise called in the UART ISR when a data msg need to be sent.
 * @author: Novosns MCU Team
 * @return <None>
 ************************************************************/
void LINUART_TransmitDataByFIFO(void)
{
    /* Enable TXFC and additionally the TXERR and FRM interrupts in order to detect TX errors
       in data bits (=>TXERR_int) or stop bit (=>FRM_int) as soon as possible */
    LINUART->IEN |= LINUART_TXFC | LINUART_TXERR | LINUART_FE;

    // RXFC interrupt cannot be disabled, in order to detect the next LIN header
    // but it should be avoided during the transmission of data
    LINUART->CR_b.RXFCOMP = 0xF;  // prevent RXCF interrupt

    // clear signal "off", if it is set
    LINPORT->ISR = LIN_OC;

    /* Configure TX message  bytes */
    memcpy((l_u8 *)&g_dataBuffTmp[1], (l_u8 *)g_currentFramePointer->var, g_nowFrameExpectedByte);

    /* Calculate checksum */
    g_dataBuffTmp[g_nowFrameExpectedByte + 1] = CalculateCheckSum((l_u8 *)&g_dataBuffTmp[0]);

    __disable_irq();  // disable "all" interrupts to avoid that the transmission of
                      // the 1st byte is completed before the FIFO is filled

    // set FIFO FILL COMPARE to '0' to get an ffint when the last byte has been sent
    LINUART->CR_b.TXFCOMP = 0;

    // write data to the FIFO
    for (l_u8 i = 1; i <= g_nowFrameExpectedByte + 1; i++) {
        LINUART->TXFIFO = g_dataBuffTmp[i];
    }

    __enable_irq();  // re-enable "all" interrupts
}

/************************************************************
 * @brief: handling of message data received from UART  status:
 *         OK  if  msg has been received successfully,
 *         NOK otherwise called in the UART ISR when a data msg has been received
 *         input:  Data and CHKSUM in UART FIFO
 * @author: Novosns MCU Team
 * @return 0: checksum error
 *         1: checksum OK
 ************************************************************/
l_u8 LINUART_ReceiveDataByFIFO(void)
{
    /* store the received data and checksum in the temporary buffer */
    for (l_u8 i = 0; i < g_nowFrameExpectedByte + 1; i++) {
        g_dataBuffTmp[i + 1] = LINUART->RXFIFO;
    }

    /* verify checksum */
    return g_dataBuffTmp[g_nowFrameExpectedByte + 1] == CalculateCheckSum(g_dataBuffTmp);
}

/************************************************************
 * @brief: Checksum calculation
 * @author: Novosns MCU Team
 * @param <l_u8> *data, data Pointer to data
 * @return checksum value
 ************************************************************/
l_u8 CalculateCheckSum(l_u8 *data)
{
    l_u16 addSum = 0;

    for (l_u8 i = 0; i < g_nowFrameExpectedByte; i++) {
        addSum = addSum + data[i + 1];
        if (addSum > 0xFF) {
            addSum = (addSum & 0xFF) + 1;
        }
    }
    /* The signal carried frame calculate checkSum by enhanced method,
       but diagnosis frame calculate checkSum by classic method */
    if ((data[0] & 0x3F) < 0x3C) {
        addSum += data[0];
        if (addSum > 0xFF) {
            addSum = (addSum & 0xFF) + 1;
        }
    }
    addSum = (~addSum) & 0xFF;  // Store inverted checksum

    return ((l_u8)addSum);
}

/************************************************************
 * @brief: Reinit protocol state for next frame RX/TX
 * @author: Novosns MCU Team
 * @return <None>
 ************************************************************/
void ResetProtocolState(void)
{
    /* disable UART (to avoid form error during next BREAK) */
    LINUART->CR_b.TXRXE = 0;

#ifdef L_SNPD_BSM
    BSM_ResetInFrameEnd();
#endif
    /* keep RXFC interrupt enabled and disable possibly set TX, FRM or TXERR interrupt */
    LINUART->IEN = LINUART_RXFL | LINUART_BRSY;
    /* reset all UART IRQ pending flags */
    LINUART->ISR = 0xFFFFFFFF;
    /* reset task status */
    g_myLinStatus.task = L_TASK_IDLE;
    /* reset protocol status */
    g_myLinStatus.protocolState = L_PROT_IDLE;
}

/************************************************************
 * @brief: To switches the LIN node into sleep mode
 * @author: Novosns MCU Team
 * @return <None>
 ************************************************************/
void GotoSleep(void)
{
    // switch to powersaving mode during LIN sleep mode
    PrepareToWakeup();  // prepare wake-up via LIN
    // switch to LIN SLEEP mode
    SYSCTRL->LKKEYR = SYSCTRL_LOCKKEY;
    SYSCTRL->LPCR_b.SLEEP = 1;  // goto SLEEP
    // wait until power saving mode has been entered
    while (1) {
    }
}

/************************************************************
 * @brief: Scheduler for LIN interface (called  each 100  ms).
 * @author: Novosns MCU Team
 * @return <None>
 ************************************************************/
void l_scheduler(void)
{       
    if (LINUART->ISR_b.TOIF) {
        LINUART->ISR = LINUART_TO;
        g_allLinError.LinError_b.erTORESP = 1;
    }
    if (LINUART->ISR_b.BRRERRIF) {
        LINUART->ISR = LINUART_BRRERRIF;
        g_allLinError.LinError_b.erSYNC = 1;
    }
    
    if (LINPORT->ISR_b.OCIF) {
        LINPORT->ISR = LIN_OC;
        g_allLinError.LinError_b.erSHORT = 1;
    }
    if (g_busIdleTimeoutCounter != 0xFFFF) {
        if (++g_busIdleTimeoutCounter > 410)
            GotoSleep();  // switch to sleep mode after 4.1s bus inactivity
    }
}
