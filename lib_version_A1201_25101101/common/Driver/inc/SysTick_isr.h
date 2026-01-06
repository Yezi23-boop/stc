/****************************************************************************
 * @file    : SysTick_isr.h
 * @author  : CJL
 * @version : V1.0
 * @Date    : 2021/9/30 16:14
 * @brief   : TMR test head files
 * @note
 * Copyright (C) 2021 novosense All rights reserved.
 ****************************************************************************/

// -------------------------------------------------------------------------//
// Define to prevent recursive inclusion 
// -------------------------------------------------------------------------//
#ifndef __SYSTICK_ISR_H
#define __SYSTICK_ISR_H
#include "stdint.h"

#define STSTICK_PRIO               2         // SysTick interrupt priority
extern volatile uint32_t g_systick_overflows;
extern volatile uint16_t g_tickCnt;
extern uint32_t g_epochSecond;
#endif

/********************************END OF FILE*********************************/
