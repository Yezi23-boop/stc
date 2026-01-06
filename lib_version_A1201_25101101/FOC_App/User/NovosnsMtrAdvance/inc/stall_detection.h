/************************************************************
 * @file: stall Detection header file
 * @author: Young Leon
 * @version: V1.0
 * @data: 2025/05/07
 * @brief: To be add
 * @note:
 * @Copyright (C) 2025 Novosense All rights reserved.
 ************************************************************/
#ifndef __STALL_DETECTION_H__
#define __STALL_DETECTION_H__
#include "nvsns_foc.h"

/**
 * @brief 电机堵转检测（Stall Detection）控制结构体
 * @details 基于反电动势（BEMF）观测值/计算值实现无感FOC电机堵转检测，
 *          包含反电动势参数、阈值、计数阈值、滤波参数、标志位等核心检测参数
 */
typedef struct
{
    //bEMFObs of Q from Observer
    Frac16_t                  f16EMFObsQ;          /*!< 观测器输出的Q轴反电动势，16位定点数，标幺值，堵转检测核心输入 */
    Frac16_t                  f16EMFObsFilterQ;    /*!< 滤波后的Q轴反电动势观测值，16位定点数，降低高频噪声干扰 */
    Frac16_t                  f16wRotElFilt;       /*!< 滤波后的转子电气角速度，16位定点数，单位：电角度/秒，辅助判断转速是否为0 */
    //bEMFKeCal of Q from KE Calculation
    Frac16_t                  f16EMFKeCalQ;        /*!< 通过KE系数计算的Q轴反电动势，16位定点数，标幺值，作为堵转检测参考 */
    //bEMFKeCal High Threshold
    Frac16_t                  f16EMFKeCalHQ;       /*!< Q轴反电动势计算值高阈值，16位定点数，标幺值，堵转判定上限 */
    //bEMFKeCal Low Threshold
    Frac16_t                  f16EMFKeCalLQ;       /*!< Q轴反电动势计算值低阈值，16位定点数，标幺值，堵转判定下限 */
    //KE coeff 
    Frac16_t                  f16CoeffKE;          //coeffKE and coeffKEOFT is from test：KE系数（反电动势常数），16位定点数，实测标定值，用于反电动势计算
    //KE offset
    Frac16_t                  f16CoeffKEOft;       /*!< KE系数偏移量，16位定点数，补偿KE系数的零漂误差 */
    //blank count for stall detection
    uint16_t                  u16BlankCnt;         /*!< 堵转检测空白计数器，16位无符号整型，用于过滤启动初期的无效检测 */
    uint16_t                  u16StallBlankConst;  /*!< 堵转检测空白计数阈值，16位无符号整型，空白期结束后才启动检测 */
    //check count for  reliability
    uint16_t                  u16StallDetCnt;      /*!< 堵转检测有效计数器，16位无符号整型，累计满足堵转条件的次数 */
    uint16_t                  u16StallDetConst;    /*!< 堵转检测有效计数阈值，16位无符号整型，达到阈值则判定为堵转 */
    uint16_t                  u16StallDetErrCnt;   /*!< 堵转检测错误计数器，16位无符号整型，累计不满足堵转条件的次数 */
    uint16_t                  u16StallDetErrConst; /*!< 堵转检测错误计数阈值，16位无符号整型，达到阈值则取消堵转判定 */
    Frac16_t                  f16Coeff;            //usually 0.2 to 0.4：堵转检测系数，16位定点数（0.2~0.4），调整检测灵敏度
    GCF_Filter1_LPF16_t       tEMFObsFilter;       /*!< 反电动势观测值低通滤波器结构体，用于平滑反电动势信号 */
    uint16_t                  u16StallErrFlag;     /*!< 堵转错误标志位，16位无符号整型：0-无堵转，1-检测到堵转 */
} StallDetection_t;
// extern StallDetection_t tStallDet;
// uint16_t StallDetection(StallDetection_t *tStall);

#endif
