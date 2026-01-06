/****************************************************************************
 * @file    : lin_init.h
 * @author  : Nonosns MCU Team
 * @version : V2.0
 * @Date    : 2023/4/7
 * @brief   : lin stack file
 * @note
 * Copyright (C) 2023 Novosense All rights reserved.
 ****************************************************************************/
#ifndef  LIN_INIT_H
#define  LIN_INIT_H
/*******************************************************************************
  type definitions
*******************************************************************************/
typedef union {
    __IO uint32_t LinError;
    struct {
        __IO uint32_t erIDPAR :  1; // PID parity error
        __IO uint32_t erCKSUM :  1; // check sum error
        __IO uint32_t erTXCOL :  1; // Tx Loop Error
        __IO uint32_t erRX    :  1; // RX frame error
        __IO uint32_t erIDSTOP:  1; // PID frame error
        __IO uint32_t erSYNC  :  1; // detect sync error
        __IO uint32_t erSHORT :  1; // lin bus over current
        __IO uint32_t erTORESP:  1; // lin bus timeout
    } LinError_b;
} LIN_ERROR;


void LINUART_Init(void);
void LINUART_Enable(void);
void LINUART_Disable(void);
void LINUART_TransmitDataByFIFO(void);
l_u8 LINUART_ReceiveDataByFIFO(void);
l_u8 CalculateCheckSum(l_u8 *data);
void ResetProtocolState(void);
void PrepareToWakeup(void);
void ResetLINResponseErrorForUser(void);
void GotoSleep(void);

#endif   /* LIN_INIT_H  */