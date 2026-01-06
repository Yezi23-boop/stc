/************************************************************
 * @file: Advance motor control header file
 * @author: Young Leon
 * @version: V1.0
 * @data: 2023/04/22
 * @brief: To be add
 * @note:
 * @Copyright (C) 2023 Novosense All rights reserved.
 ************************************************************/
#ifndef __WINDMILLING_H__
#define __WINDMILLING_H__
#include "mm_lib.h"
#include "nvsns_type_def.h"
#include "gcf_lib.h"
#include "amc_lib.h"
/**
 * @brief 风向类型枚举
 * @details 定义风机/电机运行时的风向状态，用于风速/风向自适应控制
 */
typedef enum { 
    NOWIND = 0,     /*!< 0：无风状态，无明显迎风/顺风影响 */
    HEADWIND = 1,   /*!< 1：逆风（顶风）状态，电机运行方向与风向相反 */
    TAILWIND = 2    /*!< 2：顺风（尾风）状态，电机运行方向与风向相同 */
} WDir_t;

/**
 * @brief 风速/风向观测器结构体
 * @details 存储风机/电机风向检测、转速/角度/反电动势等核心参数，
 *          用于风速自适应控制、风向判定及运行模式切换
 */
typedef struct {
    Frac16_t f16Theta;             /*!< 转子电气角度，16位定点数，单位：电角度(rad)，风向检测的角度基准 */
    Frac16_t f16Speed;             /*!< 转子电气角速度，16位定点数，单位：电角度/秒，风速/转速核心观测值 */
    Frac16_t f16AmpSqure;          /*!< 反电动势幅值平方值，16位定点数，标幺值，风向检测的核心输入 */
    Frac16_t f16AmpSqureFlt;       /*!< 滤波后的反电动势幅值平方值，16位定点数，降低高频噪声干扰 */
    Frac16_t f16AmpDetTh;          /*!< 幅值检测阈值，16位定点数，标幺值，触发风向判定的幅值门槛 */
    Frac16_t f16SpdDetTh;          /*!< 转速检测阈值，16位定点数，电角度/秒，触发风向判定的转速门槛 */
    int16_t i16WmCnt;              /*!< 风速计数变量，16位有符号整型，累计风速判定的有效次数 */
    int16_t i16DirCnt;             /*!< 风向计数变量，16位有符号整型，累计风向判定的有效次数 */
    int16_t AccCnt;                /*!< 累加计数器，16位有符号整型，用于角度/转速的积分统计 */
    Frac16_t PrevTheta;            /*!< 上一周期转子电气角度，16位定点数，单位：电角度(rad)，计算角度变化量 */
    Frac16_t AccumTheta;           /*!< 累计转子电气角度，16位定点数，单位：电角度(rad)，统计总旋转角度 */
    CTD_3SystF16_t tUabcBemf;      /*!< 三相反电动势，16位整型结构体，A/B/C相反电动势原始观测值 */
    CTD_2SystF16_t tBemfUAlBe;     /*!< αβ轴反电动势，16位整型结构体，坐标变换后的反电动势值 */
    WDir_t tWmDir;                 /*!< 当前风向状态，枚举类型（NOWIND/HEADWIND/TAILWIND），风向判定结果 */
} WM_Obsvr_t;

/**
 * @brief 风速/风向检测函数
 * @details 基于反电动势幅值、转子转速/角度等参数，判定当前风机/电机的风向状态（无风/逆风/顺风），
 *          更新风向观测器的计数和状态参数
 * @param[in,out] ptWmObsvr: 指向风速/风向观测器结构体的指针，输入观测参数，输出风向判定结果
 * @return 无
 */
void WM_WindDetection(WM_Obsvr_t* ptWmObsvr);

/**
 * @brief 风向/转速模式平滑切换函数
 * @details 根据风向检测结果，平滑切换转速斜坡、转速PI、Q轴电流PI、反电动势观测器等参数，
 *          避免模式切换导致的电机冲击，提升风速自适应控制的稳定性
 * @param[in,out] ptWmObsvr: 指向风速/风向观测器结构体的指针，输入风向状态，输出更新后的观测参数
 * @param[in,out] tSpeedRamp: 指向转速斜坡发生器结构体的指针，调整转速指令变化速率
 * @param[in,out] tSpdCtrl: 指向转速PI调节器结构体的指针，适配不同风向的转速调节参数
 * @param[in,out] tIqCtrl: 指向Q轴电流PI调节器结构体的指针，适配不同风向的转矩调节参数
 * @param[in,out] tObsvr: 指向反电动势观测器结构体的指针，适配不同风向的观测器增益
 * @return 无
 */
void WM_SwitchSmooth(WM_Obsvr_t* ptWmObsvr, GCF_Ramp_F16_t* tSpeedRamp, GCF_CtrlPIAW_PTF16_t* tSpdCtrl,
                        GCF_CtrlPIAW_RTF16_t* tIqCtrl, AMC_BemfcObsvrDQ_t* tObsvr);
#endif
