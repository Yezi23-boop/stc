#ifndef __VARIABLE_PI_CTRL_H__
#define __VARIABLE_PI_CTRL_H__

#include "mm_lib.h"
#include "typedef.h"

// 变PI参数控制器结构体
typedef struct {
    // 基础PI参数 (ap, ai)
    Frac16_t f16Kp_base;            // 基础比例增益 ap
    int16_t i16Kp_base_shift;       // 基础比例增益位移
    Frac16_t f16Ki_base;            // 基础积分增益 ai  
    int16_t i16Ki_base_shift;       // 基础积分增益位移
    
    // 可变比例参数 (bp, cp)
    Frac16_t f16Kp_var;             // 可变比例增益 bp
    int16_t i16Kp_var_shift;        // 可变比例增益位移
    Frac16_t f16Cp;                 // 比例衰减系数 cp
    int16_t i16Cp_shift;            // 比例衰减系数位移
    
    // 积分衰减参数 (ci)
    Frac16_t f16Ci;                 // 积分衰减系数 ci
    int16_t i16Ci_shift;            // 积分衰减系数位移
    
    // 状态变量
    Frac32_t f32Integrator;         // 积分累加器
    
    // 输出限幅
    Frac16_t f16UpperLimit;         // 输出上限
    Frac16_t f16LowerLimit;         // 输出下限
    
    // 控制标志
    bool bAntiWindup;               // 是否启用积分抗饱和
    
} VariablePI_Ctrl_t;

// 变PI控制器初始化
void VariablePI_Init(VariablePI_Ctrl_t* pCtrl,
                     Frac16_t f16Kp_base, int16_t i16Kp_base_shift,    // ap
                     Frac16_t f16Ki_base, int16_t i16Ki_base_shift,    // ai
                     Frac16_t f16Kp_var, int16_t i16Kp_var_shift,     // bp
                     Frac16_t f16Cp, int16_t i16Cp_shift,              // cp
                     Frac16_t f16Ci, int16_t i16Ci_shift);             // ci

// 设置输出限幅
void VariablePI_SetLimits(VariablePI_Ctrl_t* pCtrl, 
                         Frac16_t f16Upper, Frac16_t f16Lower);

// 变PI控制器主函数
Frac16_t VariablePI_Control(VariablePI_Ctrl_t* pCtrl, Frac16_t f16Error);

void SetVariablePIControllerBumpless(VariablePI_Ctrl_t* pCtrl, 
                                    Frac16_t f16TargetOutput, 
                                    Frac16_t f16CurrentError);

// 重置控制器状态
void VariablePI_Reset(VariablePI_Ctrl_t* pCtrl);
Frac16_t ExpApprox_F16_LUT(Frac16_t f16x);
#endif // __VARIABLE_PI_CTRL_H__