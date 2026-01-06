/****************************************************************************
 * @file    : include.h
 * @author  : Novosense
 * @version : V1.0
 * @Date    : 2022/6/20 
 * @brief   : A special header file containing some basic header files
 * @note
 * Copyright (C) 2022 Novosense All rights reserved.
 ****************************************************************************/

// -------------------------------------------------------------------------//
// Define to prevent recursive inclusion 
// -------------------------------------------------------------------------//

#ifndef __INCLUDE_H__
#define __INCLUDE_H__
#include "nsuc1602.h"
#include "core_cm3.h"
#include "uart.h"

#define SYSCTRL_LOCKKEY                   0x87E4
#define SYSCTRL_UNLOCK_KEY1      0x8a3d
#define SYSCTRL_UNLOCK_KEY2      0x19ec

// void delay_ms(uint32_t n);
// void delay_us(uint32_t n);

#endif //__INCLUDE_H__
