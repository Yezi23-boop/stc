/************************************************************
 * @file: smart align header file
 * @author: Young Leon
 * @version: V1.0
 * @data: 2025/05/07
 * @brief: To be add
 * @note:
 * @Copyright (C) 2023 Novosense All rights reserved.
 ************************************************************/
#ifndef __SMART_ALIGN_H__
#define __SMART_ALIGN_H__
#include "nvsns_foc.h"

/**
 * @brief 电机智能定位（Smart Align）初始化函数
 * @details 初始化智能定位的核心参数（定位电压/电流指令、计时计数器、状态机等），
 *          重置定位状态，为电机启动前的转子初始位置检测做准备
 * @return 无
 */
void SmartAlgnInit();

/**
 * @brief 电机智能定位（Smart Align）执行函数
 * @details 执行电机转子智能定位算法，通过施加定向电压/电流锁定转子初始位置，
 *          适用于无感FOC电机启动前的转子角度校准
 * @return int16_t：定位状态/结果（16位有符号整型）
 *         > 0：定位成功，返回校准后的转子初始电气角度（单位：电角度，标幺值）；
 *         = 0：定位中，尚未完成；
 *         < 0：定位失败（如过流、超时等，负数代表错误码）。
 */
int16_t SmartAlign();
#endif
