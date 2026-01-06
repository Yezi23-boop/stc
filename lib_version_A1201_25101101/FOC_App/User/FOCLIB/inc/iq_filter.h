#ifndef IQ_FILTER_H
#define IQ_FILTER_H

#include "stdint.h"
#include "typedef.h"

// 低通滤波器结构体定义 (IQ 格式)
typedef struct
{
    int32_t  in;          // 输入信号 (IQ 格式)
    int32_t  prein;       // 前一时刻的输入信号 (IQ 格式)
    int32_t  coeff;       // 滤波器系数 (IQ 格式)
    int32_t  out;         // 当前的输出信号 (IQ 格式)
    float  wT;          // 权重时间常数 (IQ 格式)
    // 余数补偿（误差反馈）：保存上次右移丢弃的低位，避免系统性偏置
    int32_t  rem_avg;     // 对 avg = ((in+prein)*0.5)>>15 的余数（Q15低位）
    int32_t  rem_mul;     // 对 delta = (diff*coeff)>>15 的余数（Q15低位）
} IQ_LP_FILTER;

// 高通滤波器结构，内部复用低通滤波器状态
typedef struct {
    int32_t in;        // 当前输入（Q15）
    int32_t out;       // 高通输出（Q15）
    IQ_LP_FILTER lp;    // 内部低通滤波器，用于计算 yLP
    float    wT;        // 时间常数或其它参数，供 ParaCalc 使用
} IQ_HP_FILTER;


// 函数声明
void IQ_LPFilterParaCalc(IQ_LP_FILTER *v);
void IQ_LPFilterInit(IQ_LP_FILTER *v);
void IQ_LPFilterCalc(IQ_LP_FILTER *v);
void IQ_HPFilterParaCalc(IQ_HP_FILTER *v);
void IQ_HPFilterInit(IQ_HP_FILTER *v);
void IQ_HPFilterCalc(IQ_HP_FILTER *v);
void IQ_LPFilterSetCutoffHz(IQ_LP_FILTER *v, float fs_hz, float fc_hz);
void IQ_HPFilterSetCutoffHz(IQ_HP_FILTER *v, float fs_hz, float fc_hz);
#endif // IQ_FILTER_H