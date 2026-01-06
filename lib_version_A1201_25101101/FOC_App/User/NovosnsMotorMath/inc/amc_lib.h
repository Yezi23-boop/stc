/************************************************************
 * @file: Advance motor control header file
 * @author: Young Leon
 * @version: V1.0
 * @data: 2023/04/22
 * @brief: To be add
 * @note:
 * @Copyright (C) 2023 Novosense All rights reserved.
 ************************************************************/
#ifndef __AMC_LIB_H__
#define __AMC_LIB_H__
#include "gcf_lib.h"
#include "gmc_lib.h"
#include "mm_lib.h"
#include "nvsns_type_def.h"

/* BEMF Tracking observer type definition */
typedef struct {
    GCF_CtrlPIAW_RTF16_t tPllCtrl;   /* PLL PI controller */
    GCF_Integrator_TRF16_t tPllInte; /* angle integrator */
    GCF_Filter1_LPF16_t tFilter;     /* speed MA filter */
} AMC_TrackObser_t;

/* BEMF observer type definition */
/**
 * @brief 反电动势（BEMF）观测器结构体（DQ轴坐标系）
 * @details 用于无感FOC控制系统中，估算电机转子反电动势、定子电流，修正转子角度偏差，
 *          是无感算法估算转子位置/转速的核心结构体
 */
typedef struct {
    CTD_2SystF16_t ptEObsrv;              /* Estimated BEMF - D/Q.：估算的D/Q轴反电动势，16位整型，核心观测输出 */
    CTD_2SystF32_t ptIObsrv;              /* Estimated current - D/Q.：估算的D/Q轴定子电流，32位高精度整型，用于电流误差补偿 */
    GCF_CtrlPIAW_RTF16_t tDeltaPIEmfCtrl; /* Observer parameters for D-axis controller.：D轴PI调节器参数，用于反电动势观测器的D轴误差调节 */
    GCF_CtrlPIAW_RTF16_t tGammaPIEmfCtrl; /* Observer parameters for Q-axis controller.：Q轴PI调节器参数，用于反电动势观测器的Q轴误差调节 */
    CTD_2SystF32_t tIObsrvIn_1L;          /* Inputs of RL circuit at step k-1 - low word：k-1时刻RL电路输入值（低字），32位整型，存储定子阻抗模型输入状态 */
    CTD_2SystF16_t tIObsrvIn_1H;          /* Inputs of RL circuit at step k-1 - high word：k-1时刻RL电路输入值（高字），16位整型，配合低字提升数据精度 */
    Frac16_t f16ErrOfDQnGD;               /* misalignment error between rotor frame and quasi-synch frame：转子坐标系与准同步坐标系的角度偏差，16位定点数，用于角度修正 */
    Frac16_t f16GainU;                    /* Gain U：电压增益系数，16位定点数，调整观测器中电压项权重 */
    Frac16_t f16GainE;                    /* Gain E：反电动势增益系数，16位定点数，调整观测器中反电动势项权重 */
    Frac16_t f16GainWI;                   /* Gain W：角速度-电流耦合增益系数，16位定点数，补偿转速对电流的影响 */
    Frac16_t f16GainI;                    /* Gain I：电流增益系数，16位定点数，调整观测器中电流项权重 */
    int16_t f16ScaleK;                    /* Gain Scale：整体缩放增益，16位有符号整型，调整观测器输出幅值范围 */
    AMC_TrackObser_t tTrackObsvr;         // tracking observer：跟踪观测器结构体，用于进一步优化转子角度/转速的跟踪精度
} AMC_BemfcObsvrDQ_t;

/* Misalignment error observer */
/**
 * @brief 永磁同步电机（PMSM）DQ轴反电动势（BEMF）观测器函数（16位整型）
 * @details 基于αβ轴电压/电流，通过反电动势观测器估算电机转子的转速和位置，
 *          同时修正转子坐标系与准同步坐标系的角度偏差，是无感FOC的核心算法
 * @param[out] pf16SpeedEst: 指向估算转速的指针，输出16位定点数格式的转子电气角速度
 * @param[out] pf16PosEst: 指向估算位置的指针，输出16位定点数格式的转子电气角度
 * @param[in] pIAB: 指向αβ轴实际电流的常量指针，输入采样得到的定子两相静止坐标系电流
 * @param[in] pUAB: 指向αβ轴指令电压的常量指针，输入控制算法输出的两相静止坐标系电压指令
 * @param[in,out] ptCtrl: 指向反电动势观测器控制结构体的常量指针，输入观测器参数，输出更新后的观测器状态
 * @return 无
 */
void AMC_PMSMBemfObsvrDQ_F16(Frac16_t *pf16SpeedEst, Frac16_t *pf16PosEst, const CTD_2SystF16_t *const pIAB,
                             const CTD_2SystF16_t *const pUAB, AMC_BemfcObsvrDQ_t *const ptCtrl);

/* Tracking observer with PLL and integrator */
/**
 * @brief 带PLL（锁相环）和积分器的跟踪观测器函数（16位整型）
 * @details 基于转子坐标系与准同步坐标系的角度偏差，通过PLL和积分器优化转速/角度估算结果，
 *          提升无感FOC的角度跟踪精度和稳定性
 * @param[out] pf16SpeedEst: 指向估算转速的指针，输出优化后的转子电气角速度（16位定点数）
 * @param[out] pf16PosEst: 指向估算位置的指针，输出优化后的转子电气角度（16位定点数）
 * @param[in] f16DQtoGDErr: DQ轴与γδ轴的角度偏差值（16位定点数），输入观测器的角度误差源
 * @param[in,out] ptTrackObsvr: 指向跟踪观测器结构体的指针，输入观测器参数，输出更新后的跟踪状态
 * @return 无
 */
void AMC_TrackObsrv_F16(Frac16_t *pf16SpeedEst, Frac16_t *pf16PosEst, Frac16_t f16DQtoGDErr,
                        AMC_TrackObser_t *ptTrackObsvr);

/* Initilize Observer */
/**
 * @brief 反电动势观测器初始化函数（16位整型）
 * @details 初始化反电动势观测器结构体的所有参数（增益、PI调节器、状态变量等），
 *          重置观测器状态，确保无感算法启动前参数处于初始状态
 * @param[in,out] ptCtrl: 指向反电动势观测器控制结构体的指针，输出初始化后的参数/状态
 * @return 无
 */
void AMC_InitObsrv_F16(AMC_BemfcObsvrDQ_t *ptCtrl);
#endif
