/************************************************************
 * @file: Advance motor control header file
 * @author: Young Leon
 * @version: V1.0
 * @data: 2023/04/22
 * @brief: To be add
 * @note:
 * @Copyright (C) 2023 Novosense All rights reserved.
 ************************************************************/
#ifndef __MTR_ADV_INIT_H__
#define __MTR_ADV_INIT_H__
#include "windmilling.h"
#include "amc_fluxweakening.h"
#include "stall_detection.h"
/**
 * @brief 母线电压/反电动势观测器（WM_Obsvr）初始化函数
 * @details 初始化母线观测器结构体的核心参数（ADC采样基准、观测器增益、状态变量等），
 *          为母线电压/反电动势的实时估算做准备
 * @param[in,out] tWMObsvr: 指向母线观测器结构体的指针，输出初始化后的观测器参数
 * @param[in] i16AdcBemf: 反电动势ADC采样基准值，16位有符号整型，用于校准观测器输入基准
 * @return 无
 */
void WM_Init(WM_Obsvr_t *tWMObsvr, int16_t i16AdcBemf);

/**
 * @brief 弱磁控制初始化函数（16位整型）
 * @details 初始化弱磁控制结构体的所有参数（滤波系数、PI调节器、电流/电压指针关联等），
 *          关联电流反馈/电压指令指针，配置弱磁控制的核心依赖参数
 * @param[in,out] pCtrl: 指向弱磁控制结构体的常量指针，输出初始化后的弱磁参数
 * @param[in] tIDQFbck: 指向DQ轴电流反馈结构体的指针，关联弱磁控制的电流反馈源
 * @param[in] tUDQReq: 指向DQ轴电压指令结构体的指针，关联弱磁控制的电压参考源
 * @param[in] tAxisQCtrl: 指向Q轴PI调节器结构体的指针，关联弱磁控制的电压限幅参数
 * @return 无
 */
void AMC_FWInit_F16(AMC_FluxWeakening_t *const pCtrl, CTD_2SystF16_t* tIDQFbck, 
                    CTD_2SystF16_t* tUDQReq, GCF_CtrlPIAW_RTF16_t* tAxisQCtrl);

/**
 * @brief 电机堵转检测初始化函数
 * @details 初始化堵转检测结构体的核心参数（检测阈值、计时计数器、状态标志等），
 *          重置堵转检测状态，确保电机启动前检测逻辑处于初始值
 * @param[in,out] tStall: 指向堵转检测结构体的指针，输出初始化后的堵转检测参数
 * @return 无
 */
void StallInit(StallDetection_t *tStall);
#endif
