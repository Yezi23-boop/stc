/************************************************************
 * @file: foc_init.h
 * @author: Novosns MCU Team
 * @version: V0.0
 * @data: 2023/12/13
 * @brief: foc initialization header file
 * @note:
 * @Copyright (C) 2023 Novosense All rights reserved.
 ************************************************************/
 
/************************************************************
 ************************************************************
 * @par Edition History
 * -V0.0  2023.12.13
 *        -Initial version for foc_init.h of NSUC1602.
 *
 ************************************************************/
 
#ifndef __FOC_INIT_H__
#define __FOC_INIT_H__
#include "nsuc1602.h"

void FOC_EpwmInit(void);
void FOC_AdcInit(void);
void FOC_GduInit(void);


#endif //__FOC_INIT_H__
