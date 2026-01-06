#ifndef __GCF_LIB_H__
#define __GCF_LIB_H__
// general control function
#include "stdint.h"
#include "mm_lib.h"

/**
 * @brief 一阶低通滤波器（16位定点数）结构体
 * @details 用于电机控制中电流/转速/角度等信号的低通滤波，降低高频噪声
 */
typedef struct {
    Frac16_t f16C1;     // low-pass filter coefficient 1：低通滤波器系数1，决定滤波响应速度
    Frac16_t f16C2;     // low-pass filter coefficient 2：低通滤波器系数2，与C1配合决定滤波截止频率
    Frac16_t f16State;  // low-pass filter state variable：滤波器状态变量，存储上一时刻滤波输出值
    Frac16_t s16NShift; // scaling factor：缩放移位因子，用于定点数运算精度调整（[-15,15]）
    Frac16_t f16NSample; // sample times：滤波采样次数/有效样本数，用于多样本平均滤波扩展
    Frac32_t f32Acc;    // 滤波器累加器，存储中间运算结果，避免定点数精度丢失
} GCF_Filter1_LPF16_t;

/**
 * @brief 递归型PI调节器（16位定点数，抗积分饱和）结构体
 * @details 用于电机控制电流环/转速环的闭环调节，采用递归形式实现，具备输出限幅功能
 */
typedef struct {
    Frac16_t f16CC1sc;      /*CC1 coefficient：PI调节器递归项系数1，对应比例+积分环节增益*/
    Frac16_t f16CC2sc;      /*CC2 coefficient：PI调节器递归项系数2，对应反馈环节增益（通常为负）*/
    Frac16_t f16InErrK1;    /*State variable. Internal previous Error：状态变量，存储上一时刻输入误差值*/
    Frac16_t f16UpperLimit; /*output upper limit：调节器输出上限，防止输出超量程（如母线电压限制）*/
    Frac16_t f16LowerLimit; /*output lower limit：调节器输出下限，防止输出超量程（如负向电压限制）*/
    uint16_t u16NShift;     /*scaling bitwise shift.[0,15]：缩放移位位数，0~15，用于定点数运算精度调整*/
    Frac32_t f32Acc;        /*State variable. Internal controller accumulator：状态变量，调节器内部累加器，存储积分项中间值*/
} GCF_CtrlPIAW_RTF16_t;

/**
 * @brief 比例积分型PI调节器（16位定点数，抗积分饱和）结构体
 * @details 用于电机控制电流环/转速环的闭环调节，经典比例+积分形式，具备输出限幅和积分限幅功能
 */
typedef struct {
    Frac16_t f16PropGain;      /*proportional gain：比例增益，决定调节器响应速度*/
    Frac16_t f16IntegGain;     /*Integral Gain：积分增益，决定调节器稳态误差消除能力*/
    int16_t s16PropGainShift;  /*Proportional Gain shift：比例增益移位因子，调整比例项运算精度*/
    int16_t s16IntegGainShift; /*Integral Gain shift：积分增益移位因子，调整积分项运算精度*/
    Frac16_t f16LowerLimit;    /*lower limit of the controller：调节器输出下限，限制最小输出值*/
    Frac16_t f16UpperLimit;    /*upper limit of the controller：调节器输出上限，限制最大输出值*/
    Frac16_t f16InK_1;         /*input error of k-1：k-1时刻的输入误差值，状态变量*/
    uint16_t u16LimitFlag;     /*Limitation flag：限幅标志位，1表示输出触发限幅，0表示正常输出*/
    Frac32_t f32IntegPartK_1;  /*integral part of k-1：k-1时刻的积分项值，状态变量，防止积分饱和*/
} GCF_CtrlPIAW_PTF16_t;

/**
 * @brief 积分器（16位定点数，递归型）结构体
 * @details 用于电机控制中角速度积分计算角度、电压积分计算磁通等场景
 */
typedef struct {
    Frac16_t f16InK1;   // input value in step K-1：K-1时刻的输入值（如角速度），状态变量
    Frac16_t f16C1;     // intetrator coefficient：积分器系数，对应积分步长（采样时间）
    uint16_t u16NShift; // scaling factor：缩放移位因子，调整积分运算精度（0~15）
    Frac32_t f32State;  // integrator state value：积分器状态值，存储积分结果（如角度、磁通）
} GCF_Integrator_TRF16_t;

/**
 * @brief 斜坡发生器（16位定点数）结构体
 * @details 用于电机控制中转速/电流指令的斜坡限制，防止指令突变导致电机冲击
 */
typedef struct {
    Frac16_t f16State;  // 斜坡发生器当前输出值，状态变量
    Frac16_t f16RampUp; // 斜坡上升速率，单位：标幺值/采样周期，限制指令最大上升速度
    Frac16_t f16RampDown; // 斜坡下降速率，单位：标幺值/采样周期，限制指令最大下降速度
} GCF_Ramp_F16_t;

Frac16_t GCF_CtrlPIrAW_F16(Frac16_t f16InErr, GCF_CtrlPIAW_RTF16_t *const pPIrAW);
Frac16_t GCF_IntegratorTR_F16(Frac16_t f16In, GCF_Integrator_TRF16_t *const pInte);
Frac16_t GCF_LpFilterMA_F16(Frac16_t f16In, GCF_Filter1_LPF16_t *const pLpFilter);
Frac16_t GCF_Ramp_F16(Frac16_t f16In, GCF_Ramp_F16_t *const pRamp);
Frac16_t GCF_CtrlPIpAW_F16(Frac16_t f16InErr, GCF_CtrlPIAW_PTF16_t *const pParam);
void GCF_InitCtrlPIp_F16(GCF_CtrlPIAW_PTF16_t* pPIpAW);
void GCF_InitCtrlPIr_F16(GCF_CtrlPIAW_RTF16_t* pPIrAW);
void GCF_InitIntegrator_F16(GCF_Integrator_TRF16_t* pInte);
void GCF_InitMAFilter_F16(GCF_Filter1_LPF16_t* pFilter);
void GCF_InitRamp_F16(GCF_Ramp_F16_t* pRamp);



#endif
