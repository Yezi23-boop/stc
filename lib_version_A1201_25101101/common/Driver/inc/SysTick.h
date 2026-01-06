/****************************************************************************
 * @file    : basic_example.h
 * @author  : novosense
 * @version : V1.0
 * @Date    : 2021/9/30 16:14
 * @brief   : TMR test head files
 * @note
 * Copyright (C) 2021 novosense All rights reserved.
 ****************************************************************************/

// -------------------------------------------------------------------------//
// Define to prevent recursive inclusion 
// -------------------------------------------------------------------------//
#ifndef __SYSTICK_H
#define __SYSTICK_H


// -------------------------------------------------------------------------//
// Private variables
// -------------------------------------------------------------------------//

uint32_t  SysTick_Init(uint16_t sysclk);
void Set_SysTick_CTRL(uint32_t ctrl);
uint8_t Timer_Stop(uint32_t *duration_t,uint32_t start_t);
void delay_us(uint32_t nus);
void delay_ms(uint16_t nms);
#endif

/********************************END OF FILE*********************************/
