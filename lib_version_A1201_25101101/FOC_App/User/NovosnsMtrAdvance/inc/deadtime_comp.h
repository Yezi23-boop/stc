/*
 * deadtime_comp.h
 * raymond
 */

#ifndef DEADTIME_COMP_H_
#define DEADTIME_COMP_H_

#include "nvsns_foc.h"
#include "gmc_lib.h"
#include "mm_lib.h"

/**
 * @brief 逆变器死区补偿控制结构体
 * @details 存储死区补偿的核心参数（死区时间、开关时间、电压误差、电流角度等），
 *          用于补偿PWM死区导致的电压畸变，提升电机控制精度
 */
typedef struct
{
    int8_t                      Switch;             /*!< 死区补偿使能开关：0-禁用，1-启用，8位有符号整型 */
    uint8_t                     sector;             /*!< SVPWM扇区编号，0~5，8位无符号整型，用于匹配对应扇区的补偿策略 */
    Frac16_t                    tdead;              //deadtime：死区时间，16位定点数，单位：秒(s)，硬件配置的PWM死区时长
    Frac16_t                    ton;                /*!< 功率管开通时间，16位定点数，单位：秒(s)，补偿开通延时 */
    Frac16_t                    toff;               /*!< 功率管关断时间，16位定点数，单位：秒(s)，补偿关断延时 */
    Frac16_t                    tequal;             /*!< 死区等效补偿时间，16位定点数，单位：秒(s)，ton/toff与tdead的等效合成值 */
    Frac16_t                    udc;                /*!< 直流母线电压，16位定点数，单位：标幺值(PU)，死区补偿的电压基准 */
    Frac16_t                    uerr;               /*!< 死区导致的电压误差，16位定点数，单位：标幺值(PU)，补偿计算的核心误差值 */
    Frac16_t                    CurrentAngle;       /*!< 电流矢量角度，16位定点数，单位：电角度(rad)，用于判断电流方向以确定补偿极性 */
    CTD_2SystF16_t              Idqfiltered;        /*!< 滤波后的DQ轴电流，16位整型结构体，用于补偿计算的电流基准 */
    CTD_2SystF16_t              IalbeCurrent;       /*!< αβ轴电流，16位整型结构体，用于判断电流方向 */
    CTD_2SystF16_t              CompValue;          /*!< 死区补偿值，16位整型结构体，αβ轴电压补偿量，输出到SVPWM前叠加 */
}DeadTimeComp_t;

extern DeadTimeComp_t tDeadTimeComp;               /*!< 死区补偿全局控制结构体实例 */

/**
 * @brief 死区补偿参数更新函数
 * @details 实时计算死区补偿值，根据电流方向、扇区、母线电压等参数更新补偿量，
 *          并叠加到电压指令中
 * @param[in,out] P: 指向死区补偿控制结构体的指针，输入实时参数，输出计算后的补偿值
 * @return 无
 */
extern void DeadTimeComUpdate(DeadTimeComp_t *P);

/**
 * @brief 死区补偿初始化函数
 * @details 初始化死区补偿结构体的所有参数（使能开关、死区时间、开关时间等），
 *          重置补偿状态，确保启动前参数处于初始值
 * @param[in,out] P: 指向死区补偿控制结构体的指针，输出初始化后的参数
 * @return 无
 */
extern void DeadTimeCompensationInit(DeadTimeComp_t *P);

#endif /* DEADTIMECOMP_H_ */