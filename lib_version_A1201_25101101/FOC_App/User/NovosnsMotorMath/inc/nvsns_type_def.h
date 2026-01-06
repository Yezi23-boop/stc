/****************************************************************************
 * @file: To be add
 * @author: Young Leon
 * @version: V1.0
 * @date: 2022/04/22
 * @brief: To be add
 * @note: 
 * @Copyright (C) 2022 Novosense All rights reserved.
 */
#ifndef __NVSNS_TYPE_DEF_H__
#define __NVSNS_TYPE_DEF_H__
#include "stdint.h"
#include "mm_lib.h"

// common type define

/**
 * @brief 三相坐标系参数结构体（16位整型）
 * @details 用于电机三相（A/B/C）电压/电流/占空比等参数存储，适配基础电机数学运算
 */
typedef struct
{
    int16_t f16Arg1;  /*!< 三相坐标系参数1（如A相），16位有符号整型 */
    int16_t f16Arg2;  /*!< 三相坐标系参数2（如B相），16位有符号整型 */
    int16_t f16Arg3;  /*!< 三相坐标系参数3（如C相），16位有符号整型 */
}CTD_3SystF16_t; //3 phase coordinate system for basic motor math

/**
 * @brief 三相坐标系参数结构体（32位整型）
 * @details 用于电机三相（A/B/C）高精度电压/电流/占空比等参数存储，适配基础电机数学运算
 */
typedef struct
{
    int32_t f16Arg1;  /*!< 三相坐标系参数1（如A相），32位有符号整型 */
    int32_t f16Arg2;  /*!< 三相坐标系参数2（如B相），32位有符号整型 */
    int32_t f16Arg3;  /*!< 三相坐标系参数3（如C相），32位有符号整型 */
}CTD_3SystF32_t; //3 phase coordinate system for basic motor math

/**
 * @brief 两相坐标系参数结构体（16位整型）
 * @details 用于电机两相（αβ/dq）静止/旋转坐标系参数存储，适配基础电机数学运算
 */
typedef struct
{
    int16_t f16Arg1;  /*!< 两相坐标系参数1（如α轴/d轴），16位有符号整型 */
    int16_t f16Arg2;  /*!< 两相坐标系参数2（如β轴/q轴），16位有符号整型 */
}CTD_2SystF16_t; //2 phase coordinate system for basic motor math

/**
 * @brief 两相坐标系参数结构体（32位整型）
 * @details 用于电机两相（αβ/dq）静止/旋转坐标系高精度参数存储，适配基础电机数学运算
 */
typedef struct
{
    int32_t f16Arg1;  /*!< 两相坐标系参数1（如α轴/d轴），32位有符号整型 */
    int32_t f16Arg2;  /*!< 两相坐标系参数2（如β轴/q轴），32位有符号整型 */
}CTD_2Syst32_t; //2 phase coordinate system for basic motor math

/**
 * @brief 两相坐标系参数结构体（32位无符号整型）
 * @details 用于电机两相（αβ/dq）静止/旋转坐标系非负参数存储，适配基础电机数学运算
 */
typedef struct
{
    uint32_t f32Arg1; /*!< 两相坐标系参数1（如α轴/d轴），32位无符号整型 */
    uint32_t f32Arg2; /*!< 两相坐标系参数2（如β轴/q轴），32位无符号整型 */
}CTD_2SystF32_t; //2 phase coordinate system for basic motor math


/**
 * @brief SVPWM（空间矢量脉宽调制）模式枚举
 * @details 定义SVPWM的两种调制方式，用于切换不同的PWM生成算法
 */
typedef enum {
    SVM_MODE_SEVEN_SEGMENT = 0,  /*!< 0：7段式SVPWM，谐波小、调制比高 */
    SVM_MODE_FIVE_SEGMENT = 1    /*!< 1：5段式SVPWM，开关损耗低 */
} SVM_Mode_t;

/**
 * @brief SVPWM查表法控制结构体
 * @details 存储SVPWM调制的核心参数，用于查表法生成SVPWM波
 */
typedef struct
{
    uint16_t u16Sec;             /*!< SVPWM扇区编号，范围1~6，无符号16位整型 */
    uint16_t u16T1;              /*!< 有效矢量1作用时间，单位：PWM计数周期，无符号16位整型 */
    uint16_t u16T2;              /*!< 有效矢量2作用时间，单位：PWM计数周期，无符号16位整型 */
    uint16_t u16AdcTrg;          /*!< ADC触发时刻配置，用于电流采样同步，无符号16位整型 */
    uint16_t u16RefAmp;          /*!< 参考电压幅值，单位：标幺值(PU)，无符号16位整型 */
    uint16_t u16Angle;           /*!< SVPWM电压矢量角度，单位：电角度，无符号16位整型 */
    CTD_3SystF16_t tPwmDuty;     /*!< 三相PWM占空比，对应A/B/C相，16位整型（标幺值） */
    uint16_t bEnableFiveSegment; /*!< 5段式SVPWM使能标志：0-禁用，1-启用切换功能 */
    int16_t f16ThresholdToFive;  /*!< 切换到5段式的阈值，16位有符号整型（0=全程5段，32767=全程7段） */
    int16_t f16Hysteresis;       /*!< 5段式切回7段式的回差值，用于防止模式频繁切换 */
    SVM_Mode_t svmMode;          /*!< 当前SVPWM调制模式（7段式/5段式） */
    CTD_3SystF32_t tUref;        /*!< 三相参考电压，高精度32位整型，用于SVPWM计算 */
    MM_PolarTrigono_t tUThetaTransform; /*!< 电压矢量极坐标变换结构体，存储幅值/角度变换参数 */
    Frac16_t f16ModulationIndex; /*!< SVPWM调制系数（调制比），16位定点数，范围0~1 */
}CTD_LUT_SVM_t;

#endif