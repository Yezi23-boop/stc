/************************************************************
 * @file: nvsns_foc.h
 * @author: Young Leon
 * @version: V0.0
 * @data: 2023/12/13
 * @brief: foc processing header file
 * @note:
 * @Copyright (C) 2023 Novosense All rights reserved.
 ************************************************************/
 
/************************************************************
 ************************************************************
 * @par Edition History
 * -V0.0  2023.12.13
 *        -Initial version for nvsns_foc.h of NSUC1602.
 *
 ************************************************************/
 
#ifndef __NVSNS_FOC_H__
#define __NVSNS_FOC_H__
#include "amc_lib.h"
#include "gcf_lib.h"
#include "state_machine.h"
#include "nvsns_type_def.h"
#include "typedef.h"
#include "windmilling.h"
#include "amc_fluxweakening.h"

// EPWM interrupt flag
#define EPWM_PTUD0IF 0x00000001
#define EPWM_PTDD0IF 0x00000002
#define EPWM_PTUD1IF 0x00000004
#define EPWM_PTDD1IF 0x00000008
#define EPWM_PTUD2IF 0x00000010
#define EPWM_PTDD2IF 0x00000020
#define EPWM_PWMZIF  0x00000040
#define EPWM_PWMPIF  0x00000080

#define EPWM_PTUD0IFC 0x00010000
#define EPWM_PTDD0IFC 0x00020000
#define EPWM_PTUD1IFC 0x00040000
#define EPWM_PTDD1IFC 0x00080000
#define EPWM_PTUD2IFC 0x00100000
#define EPWM_PTDD2IFC 0x00200000
#define EPWM_PWMZIFC  0x00400000
#define EPWM_PWMPIFC  0x00800000

// ADC intterupt flag
#define ADC_EOC     0x00001
#define ADC_TCOLL   0x00002
#define ADC_QCOLL1  0x00004
#define ADC_QCOLL2  0x00008
#define ADC_QCOLL3  0x00010
#define ADC_QCOLL4  0x00020
#define ADC_QCOLL5  0x00040
#define ADC_QCOLL6  0x00080
#define ADC_QCOLL7  0x00100
#define ADC_QCOLL8  0x00200
#define ADC_CALERR1 0x00400
#define ADC_CALERR2 0x00800
#define ADC_CALERR3 0x01000
#define ADC_CALERR4 0x02000
#define ADC_CALERR5 0x04000
#define ADC_CALERR6 0x08000
#define ADC_CALERR7 0x10000
#define ADC_CALERR8 0x20000

typedef enum {
    FORCE = 0,                ///< 强制定位模式（转子预定位/对齐）：启动初期将转子拉到指定电角度，为后续启动做准备
    TRACKING = 1,             ///< 跟踪模式（开环跟踪/斜坡升速）：预定位完成后，开环给定电角度/转速，逐步提升电机转速
    PRESENSORLESS = 2,        ///< 无感前过渡模式：开环升速到无感控制阈值转速前的过渡阶段，准备切换到无感控制
    HFI_MODE = 3,             ///< HFI（高频注入）模式：低速/零速阶段，通过高频电压注入提取转子位置，适配无感控制低速场景
    SENSORLESS = 4            ///< 纯无感控制模式：中高速阶段，通过反电动势观测器（如滑模/龙伯格）提取转子位置，无位置传感器运行
} PosMode_t;

typedef struct {
    Frac16_t f16ThetaRotEl;          /*!< 转子电气角度 (16位定点数)
                                          单位：电角度(rad)，范围0~2π，由转速积分得到 */
    Frac16_t f16wRotE1;              /*!< 转子电气角速度 (16位定点数)
                                          单位：电角度/秒(rad/s)，对应开环目标转速 */
    Frac16_t f16IQUpperLimit;        /*!< Q轴电流上限 (16位定点数)
                                          单位：标幺值(PU)，限制开环阶段最大输出转矩 */
    Frac16_t f16IQLowerLimit;        /*!< Q轴电流下限 (16位定点数)
                                          单位：标幺值(PU)，限制开环阶段最小输出转矩 */
    Frac16_t f16ThetaDifOplpEstm;    /*!< 开环估算角度偏差 (16位定点数)
                                          单位：电角度(rad)，用于开环切闭环前的角度同步校验 */
    GCF_Integrator_TRF16_t tOplpInteg; /*!< 开环角度积分器结构体
                                          用于对电气角速度积分，计算转子电气角度（核心组件） */
} FOC_Openloop_t;

typedef struct {
    Frac16_t f16ThetaRotEl;          // El. position entering to the control loop
    Frac16_t f16wRotE1;              // El. speed entering to the control loop
    Frac16_t f16wRotE1Filt;          // Filtered El. speed entering to the control loop
    Frac16_t f16wRotElReq;           // Required el. speed
    Frac16_t f16wRotElReqRamp;       // Required el. speed converted to the ramp shape
    Frac16_t f16wRotElErr;           // Error of the el. speed entering to speed controller
    int16_t i16SpeedLoopCntr;        // rate between speed and current loop
    GCF_Filter1_LPF16_t twRotFilter; // Speed filtering filter settings
    //阻尼控制后的速度斜坡给出值
    Frac16_t f16wRotElReqIFDeltaW;
} FOC_PospeCtrl_t;

typedef struct {
    Frac16_t f16ThetaRotEl;          /*!< 转子电气角度 (16位定点数)
                                          单位：电角度(rad)，范围0~2π，由无感算法（如反电动势观测器）估算 */
    Frac16_t f16wRotE1;              /*!< 转子电气角速度 (16位定点数)
                                          单位：电角度/秒(rad/s)，无感模式下的实际转速估算值 */
    Frac16_t f16wRotElMatch_1;       /*!< 电气角速度匹配阈值1 (16位定点数)
                                          单位：电角度/秒(rad/s)，用于开环切闭环的转速判定（低阈值） */
    Frac16_t f16wRotElMatch_2;       /*!< 电气角速度匹配阈值2 (16位定点数)
                                          单位：电角度/秒(rad/s)，用于无感算法收敛判定/高速模式切换阈值 */
    Frac16_t f16DQtoGaDeError;       /*!< DQ轴转γδ轴角度误差 (16位定点数)
                                          单位：电角度(rad)，无感观测器的角度跟踪误差，用于算法稳定性校验 */
    Frac16_t f16IQUpperLimit;        /*!< Q轴电流上限 (16位定点数)
                                          单位：标幺值(PU)，限制无感模式下最大输出转矩，防止过流 */
    Frac16_t f16IQLowerLimit;        /*!< Q轴电流下限 (16位定点数)
                                          单位：标幺值(PU)，限制无感模式下最小输出转矩，防止反向抖动 */
    uint16_t u16sensorlessCnt;       /*!< 无感控制状态计数器 (16位无符号整数)
                                          单位：ms/计数周期，用于统计无感算法稳定运行时间、切换延时等 */
    AMC_BemfcObsvrDQ_t tBemfObsvr;   /*!< 反电动势（BEMF）观测器结构体
                                          存储反电动势观测器的核心参数（如观测器增益、积分器、滤波系数等），
                                          是无感FOC估算转子角度/转速的核心组件 */
} FOC_SensorlessCtrl_t;


typedef struct {
    uint16_t u16ProConDetectCnt;
    int16_t u16ContsCnt;
    WDir_t tWDIR;
} FOC_Wdir_t;


typedef struct {
    uint16_t u16Ph1Trg1;
    uint16_t u16Ph2Trg1;
    uint16_t u16DcOffsetTrg;
    uint16_t u16Ph2Trg2;
    uint16_t u16Ph1Trg2;
} FOC_ReTrigT_t;

typedef struct {
    uint16_t u16PhAEdge;
    uint16_t u16PhADbswEdge;
    uint16_t u16PhBEdge;
    uint16_t u16PhBDbswEdge;
    uint16_t u16PhCEdge;
    uint16_t u16PhCDbswEdge;
    FOC_ReTrigT_t tAdcTrg;
} FOC_Mod_t;

/**
 * @brief FOC（磁场定向控制）驱动核心结构体
 * @details 包含FOC控制全流程的状态、参数、反馈值、控制器实例等，适配无感/有感、开环/闭环控制场景
 * 注：Frac16_t为Q15定点数（取值范围[-1, 1-1/32768]），所有10倍放大的成员需在使用时除10还原实际值
 */
typedef struct {
    APP_State_Transfer_t tAppState;              ///< 应用层状态机（如初始化、对齐、运行、故障等）
    uint16_t u16AlignCntr;                       ///< 电机转子对齐计数器（对齐阶段计时/计数）
    uint16_t u16AlignVoltage;                    ///< 转子对齐电压（Q15/原始值，根据母线电压归一化）
    uint16_t u16CalibCntr;                       ///< ADC校准计数器（用于电流/电压采样零点校准）
    int32_t i32CalibSum;                         ///< ADC校准累加和（零点校准用）
    int32_t i32CalibSum2;                        ///< ADC校准累加和2（备用/双通道校准）
    int16_t i16AdcOffset;                        ///< ADC采样零点偏移量（校准后补偿值）
    int16_t i16AdcRaw[8];                        ///< ADC原始采样值数组（包含相电流、母线电压、温度等8路采样）
    Frac16_t f16DcBusFilt;                       ///< 母线电压滤波后值（Q15，归一化到母线额定电压）
    int16_t i16DcBusRaw;                         ///< 母线电压原始采样值（单位：mV/原始码值，需校准转换）
    int16_t i16IntTemp;                          ///< 内部温度采样值（控制器/MCU温度，单位：℃/原始码值）
    uint16_t u16NormalRunTime;                   ///< 电机正常运行时长计数器（单位：ms/s，按需定义）
    Frac16_t f16ExtSpeed;                        ///< 外部给定转速（Q15，归一化到额定转速）
    Frac16_t f16Powerbck;                        ///< 功率反馈值（Q15，归一化到额定功率）
    Frac16_t f16PowerReq;                        ///< 功率给定值（Q15，归一化到额定功率）
    Frac16_t f16Idcbus;                          ///< 母线电流（Q15，归一化到额定母线电流）
    CTD_3SystF16_t tIabcFbck;                    ///< 三相电流反馈值（Ia/Ib/Ic，Q15格式）
    CTD_2SystF16_t tIAlBeFbck;                   ///< αβ坐标系电流反馈值（Clark变换后，Q15格式）
    CTD_2SystF16_t tIDQFbck;                     ///< dq坐标系电流反馈值（Park变换后，Q15格式）
    CTD_2SystF16_t tIDQErr;                      ///< dq坐标系电流误差（给定-反馈，Q15格式）
    CTD_2SystF16_t tIDQReq;                      ///< dq坐标系电流给定值（电流环输出，Q15格式）
    CTD_2SystF16_t tUDQReq;                      ///< dq坐标系电压给定值（电流环输出，Q15格式）
    CTD_2SystF16_t tUAlBeReq;                    ///< αβ坐标系电压给定值（逆Park变换后，Q15格式）
    CTD_2SystF16_t tUAlBeReqDCB;                 ///< 母线电压补偿后的αβ电压给定（Q15格式）
    MM_PolarTrigono_t tThetaTransform;           ///< 角度变换结构体（包含电角度、正余弦值等）
    GCF_CtrlPIAW_RTF16_t tAxisDCtrl;             ///< d轴电流PI控制器（抗积分饱和，Q15格式）
    GCF_CtrlPIAW_RTF16_t tAxisQCtrl;             ///< q轴电流PI控制器（抗积分饱和，Q15格式）
    GCF_CtrlPIAW_PTF16_t tSpeedCtrl;             ///< 转速PI控制器（位置跟踪型，Q15格式）
    GCF_Ramp_F16_t tSpeedRamp;                   ///< 转速斜坡发生器（限制转速变化率，防冲击）
    FOC_PospeCtrl_t tPospeControl;               ///< 位置/速度总控制结构体（统筹开环/无感等模式）
    FOC_Openloop_t tPospeOpenloop;               ///< 开环控制结构体（启动/低速开环运行参数）
    FOC_SensorlessCtrl_t tPospeSensorless;       ///< 无感控制结构体（滑模观测器/HFI相关参数）
    PosMode_t tPosMode;                          ///< 位置控制模式（有感/无感/开环）
    CTD_LUT_SVM_t tSvm;                          ///< SVPWM调制结构体（扇区、占空比、矢量时间等）
    GMC_DcBusComp_t tDcBusComp;                  ///< 母线电压补偿结构体（SVPWM母线电压自适应）
    FOC_Wdir_t tProConWind;                      ///< 电机绕组极性/转向控制（正转/反转/极性修正）

    GCF_Filter1_LPF16_t tIRaw1Filter;            ///< 电流原始采样1低通滤波器（滤除高频噪声）
    GCF_Filter1_LPF16_t tIRaw2Filter;            ///< 电流原始采样2低通滤波器（滤除高频噪声）
    GCF_Filter1_LPF16_t tIaFilter;               ///< A相电流低通滤波器（最终反馈用）
    GCF_Filter1_LPF16_t tIbFilter;               ///< B相电流低通滤波器（最终反馈用）
    GCF_Filter1_LPF16_t tIcFilter;               ///< C相电流低通滤波器（最终反馈用）
    GCF_Filter1_LPF16_t tDcBusFilter;            ///< 母线电压低通滤波器（滤除纹波）
    GCF_Filter1_LPF16_t tExtSpeedFilter;         ///< 外部给定转速低通滤波器（平滑转速给定）
    FOC_Mod_t tPwmMod;                           ///< PWM调制结构体（载波频率、死区、占空比限制等）
    /*windmilling*/
    WM_Obsvr_t tWMObsvr;                         ///< 风车效应观测器（电机拖曳运行时的状态监测）
    /*flux weakening*/
    AMC_FluxWeakening_t tFluxWeakening;          ///< 弱磁控制结构体（高速段弱磁扩速参数）
    uint16_t u16PWMCnt;                          ///< PWM周期计数器（用于同步采样/控制周期）
    int16_t i16IntMOSTemp;                       ///< MOS管/功率模块温度（单位：℃/原始码值）
    uint32_t u32FaultRestartDelayTimer;          ///< 故障重启延迟计时器 (单位：1ms任务周期)
    bool bInFaultDelay;                          ///< 标记是否处于故障停机延迟状态（true=延迟中）
    int16_t i1610Idcbus;                         ///< 母线电流（放大10倍存储，提高精度，使用时÷10）
    int16_t i1610Irms;                           ///< 相电流有效值（放大10倍存储，使用时÷10）
    Frac16_t i1610Power;                         ///< 功率值（放大10倍的Q15格式，使用时÷10）
    uint16_t u16HfiCntr;                         ///< HFI（高频注入）计数器（低速无感控制计时/分频）
    //PERSENSORLESS计数器
    uint32_t u32PreSensorlessCnt;                ///< 无感控制前置计数器（启动阶段切换无感前的计时）
    uint32_t u32PreSensorlessTime;               ///< 无感控制前置时间（启动阶段切换无感的阈值时间）
    //极性修正后的tPospeSensorless角度
    Frac16_t f16ThetaElSensorlessCorrected;      ///< 极性修正后的无感电角度（Q15，-32768~32767对应-π~π）
    //阻尼控制后的速度斜坡给出值
    Frac16_t f16wRotElReqIFDeltaW;               ///< 阻尼控制后的电角速度给定（抑制转速振荡，Q15格式）
    //降低IQ的控制分频计数器
    uint16_t u16ReduceIQControllCnt;             ///< q轴电流限幅控制分频计数器（降低高频调节损耗）
} FOC_Driver_t;

// IQ限制控制结构体
typedef struct {
    // 母线电流限制参数
    Frac16_t f16IdcLimit;           // 母线电流限制值 (Q15格式)
    Frac16_t f16IdcCurrent;         // 当前母线电流值 (Q15格式)
    
    // 功率限制参数  
    Frac16_t f16PowerLimit;         // 功率限制值 (Q15格式)
    Frac16_t f16PowerCurrent;       // 当前功率值 (Q15格式)
    
    // 限制控制参数
    Frac16_t f16IqLimitRatio;       // IQ限制比例 (0.0~1.0, Q15格式)
    Frac16_t f16IqLimit;
    Frac16_t f16IqLimitStep;        // IQ限制步长 (Q15格式)
    Frac16_t f16IqRecoveryStep;     // IQ恢复步长 (Q15格式)
    
    // 内部状态
    bool bIdcOverLimit;             // 母线电流超限标志
    bool bPowerOverLimit;           // 功率超限标志
    bool bLimitActive;              // 限制激活标志
    bool bLastLimitActive;          // 上一次限制激活标志
    
    // 滤波器 - 用于平滑限制比例变化
    GCF_Filter1_LPF16_t tLimitFilter;

    //分频计数器
    uint16_t u16LimitCtrlDivCnt;
    
} IQ_Limit_Ctrl_t;

extern IQ_Limit_Ctrl_t tIqLimitCtrl; // IQ限制控制实例
extern FOC_Driver_t tDrvFoc;
extern int16_t i16StallDetCntr;

#endif
