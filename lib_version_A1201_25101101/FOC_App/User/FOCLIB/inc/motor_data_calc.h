#ifndef MOTOR_DATA_CALC_H
#define MOTOR_DATA_CALC_H

#include <stdint.h>
#include "mm_lib.h"
#include "gcf_lib.h"

// 电机计算相关结构体
typedef struct {
    // 输入数据 (所有数据均为IQ15格式)
    Frac16_t ia;         // A相电流
    Frac16_t ib;         // B相电流
    Frac16_t ic;         // C相电流
    Frac16_t va;         // A相电压
    Frac16_t vb;         // B相电压
    Frac16_t vc;         // C相电压
    Frac16_t vdc;        // 母线电压

    // id iq ud uq
    Frac16_t f16Id;         // d轴电流
    Frac16_t f16Iq;         // q轴电流
    Frac16_t f16UdRate;
    Frac16_t f16UqRate;
    Frac16_t f16Ud;         // d轴电压
    Frac16_t f16Uq;         // q轴电压

    int32_t i32VDError;
    int32_t i32VQError;
    int32_t i32VDCorrected;
    int32_t i32VQCorrected;

    Frac16_t i_d_32_filtered; // 滤波后的d轴电流
    Frac16_t i_q_32_filtered; // 滤波后的q轴电流

    //速度
    Frac16_t wo;          // 电气角速度 (rad/s)
    //速度考虑因素
    Frac16_t f16SpeedFactor; // 速度计算系数
    //死区考虑因素
    Frac16_t f16DeadTimeFactor; // 死区时间系数

    int32_t i32DeadTimeNs; //死区时间
    int32_t i32TonNs; //功率开关器件开通延时
    int32_t i32ToffNs; //功率开关器件关断延时
    int32_t i32pwmPeriodNs;//PWM周期

    GCF_Filter1_LPF16_t id_32_filter; // d轴电流滤波器
    GCF_Filter1_LPF16_t iq_32_filter; // q轴电流滤波器

    Frac16_t i_alpha;    // α轴电流 (Clarke变换结果)
    Frac16_t i_beta;     // β轴电流 (Clarke变换结果)
    
    // 计算结果 (所有结果均为IQ15格式)
    // Frac16_t i_alpha;    // α轴电流
    // Frac16_t i_beta;     // β轴电流
    //平方和
    Frac16_t i_squared; // 电流平方和
    Frac16_t i_magnitude; // 相电流幅值
    //有效值
    Frac16_t i_rms;      // 相电流有效值
    GCF_Filter1_LPF16_t i_rms_filter; // 相电流有效值滤波器
    //滤波值
    Frac16_t i_rms_filtered; // 滤波后的相电流有效值
    Frac16_t f16IDc;       // 母线电流
    GCF_Filter1_LPF16_t idc_32_filter; // 母线电流滤波器
    Frac16_t f16IDCFiltered; // 滤波后的母线电流
    //直流侧到交流侧的效率
    Frac16_t f16Idc2acEfficiency; // 母线电流到交流侧的效率
    Frac16_t active_power; // 有功功率
    Frac16_t reactive_power; // 无功功率
    Frac16_t apparent_power; // 视在功率
    Frac16_t power_factor; // 功率因数
} MotorCalculations;

// 函数声明
void motorInitCalculations(MotorCalculations* motor);
void motor_calculate_clarke_transform(MotorCalculations* motor);
void motor_calculate_current_magnitude(MotorCalculations* motor);
void motor_calculate_dc_current(MotorCalculations* motor);
void motor_calculate_power(MotorCalculations* motor);
void motorCalculateAllData(MotorCalculations* motor);
void motor_calculate_dc_current_by_sample(MotorCalculations* motor,int16_t adc_sample);

#endif // MOTOR_CALCULATIONS_H
