#ifndef __GMC_LIB_H__
#define __GMC_LIB_H__

#include "stdint.h"
#include "nvsns_type_def.h"
#include "mm_lib.h"

typedef struct {
    Frac16_t f16ArgDcBusMsr;
    Frac16_t f16ModIndex;
} GMC_DcBusComp_t;

/**
 * @brief SVPWM（空间矢量脉宽调制）计算函数
 * @details 根据αβ轴参考电压计算SVPWM的扇区、矢量时间、占空比等参数，填充到SVM控制结构体中
 * @param[in,out] ptSvm: 指向SVPWM查表法控制结构体的指针，输出SVM计算结果（扇区、T1/T2、占空比等）
 * @param[in] ptUAlBeReq: 指向αβ轴参考电压结构体的常量指针，输入需要调制的αβ轴电压指令
 * @return 无
 */
void GMC_Svm(CTD_LUT_SVM_t* ptSvm, CTD_2SystF16_t* const ptUAlBeReq);

/**
 * @brief Clarke变换（16位整型）函数
 * @details 将电机三相（A/B/C）静止坐标系的物理量（电流/电压）转换为两相αβ静止坐标系
 * @param[out] ptSttCur: 指向αβ轴静止坐标系结构体的指针，输出Clarke变换结果
 * @param[in] ptPhs3Cur: 指向三相坐标系结构体的常量指针，输入需要变换的三相物理量（如相电流）
 * @return 无
 */
void GMC_ClarkeF16(CTD_2SystF16_t* ptSttCur, const CTD_3SystF16_t* const ptPhs3Cur);

/**
 * @brief Park变换（16位整型）函数
 * @details 将电机两相αβ静止坐标系的物理量转换为两相dq旋转坐标系（同步于转子磁场）
 * @param[out] ptRttCoordinate: 指向dq轴旋转坐标系结构体的指针，输出Park变换结果
 * @param[in] ptSttCoordinate: 指向αβ轴静止坐标系结构体的常量指针，输入需要变换的αβ轴物理量
 * @param[in] ptPloarSinCos: 指向极坐标正余弦结构体的常量指针，输入转子电气角度的sin/cos值（用于坐标旋转）
 * @return 无
 */
void GMC_ParkeF16(CTD_2SystF16_t* ptRttCoordinate, const CTD_2SystF16_t* const ptSttCoordinate, const MM_PolarTrigono_t* const ptPloarSinCos);

/**
 * @brief 反Park变换（16位整型）函数
 * @details 将电机两相dq旋转坐标系的物理量转换为两相αβ静止坐标系，为SVPWM调制做准备
 * @param[out] ptSttCoordinate: 指向αβ轴静止坐标系结构体的指针，输出反Park变换结果
 * @param[in] ptRttCoordinate: 指向dq轴旋转坐标系结构体的常量指针，输入需要变换的dq轴物理量（如电压指令）
 * @param[in] ptPloarSinCos: 指向极坐标正余弦结构体的常量指针，输入转子电气角度的sin/cos值（用于坐标旋转）
 * @return 无
 */
void GMC_InvParkF16(CTD_2SystF16_t* ptSttCoordinate, const CTD_2SystF16_t* const ptRttCoordinate, const MM_PolarTrigono_t* const ptPloarSinCos);

/**
 * @brief 直流母线电压补偿函数（16位整型）
 * @details 补偿母线电压波动对SVPWM调制的影响，保证输出电压指令的精度
 * @param[out] ptOut: 指向补偿后输出结构体的常量指针，输出补偿后的αβ轴电压
 * @param[in] ptIn: 指向补偿前输入结构体的常量指针，输入待补偿的αβ轴电压指令
 * @param[in] ptParam: 指向母线电压补偿参数结构体的常量指针，输入补偿系数、母线电压采样值等参数
 * @return 无
 */
void GMC_DcBusCompF16(CTD_2SystF16_t *const ptOut, const CTD_2SystF16_t *const ptIn, const GMC_DcBusComp_t *const ptParam);
#endif
