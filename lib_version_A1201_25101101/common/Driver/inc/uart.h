/****************************************************************************
 * @file    : uart.h
 * @author  : novosense
 * @version : V1.0
 * @Date    : 2022/6/23 16:14
 * @brief   : uart header file
 * @note
 * Copyright (C) 2022 novosense All rights reserved.
 ****************************************************************************/
 
#ifndef __UART_H__
#define __UART_H__
#include "include.h"

void InitPrintf(int baudRate);
void InitUart(void);
void UART_DataTransmit(uint8_t u8Char);
uint8_t UART_DataReceive(void);

#endif // __UART_H__

