/************************************************************
 * @file: Advance motor control header file
 * @author: AIWIN MCU teams
 * @version: V1.0
 * @data: 2023/04/22
 * @brief: To be add
 * @note:
 * @Copyright (C) 2023 Novosense All rights reserved.
 ************************************************************/
#ifndef __WINDMILLING_PLL_H__
#define __WINDMILLING_PLL_H__
#include "mm_lib.h"
#include "nvsns_type_def.h"
#include "gcf_lib.h"
#include "amc_lib.h"
#include "foc_paras.h" // Include for FOC_Paras_t definition
#include "pll_pi.h"
#include "typedef.h"
#include "foc_paras.h"
#include "gcf_lib.h"
#include "windmilling.h"


// 风检测子状态枚举
// 风检测子状态枚举
typedef enum {
    WIND_STATE_INIT,    // 初始PWM生成状态
    WIND_STATE_DETECT,  // 风检测状态
    WIND_STATE_BRAKE,   // 刹车状态
    WIND_STATE_DONE     // 检测完成状态
} WindDetectSubState_t;





typedef struct {
    /* Original members */
    CTD_3SystF16_t tUabcBemf;
    CTD_3SystF16_t tUabc;
    uint32_t u32Amp;
    uint32_t u32AmpFlt;
    Frac16_t f16AmpSqure;
    Frac16_t f16AmpSqureFlt;
    //反电动势幅值确认计数器
    int16_t i16AmpDetCnt;      // Counter for amplitude detection
    Frac16_t f16Theta;          // Original angle estimation (can be kept for comparison or removed)
    Frac16_t f16Speed;          // Original speed estimation (can be kept for comparison or removed)    标幺化的电机机械转速
    Frac16_t f16AmpDetTh;
    Frac16_t f16SpdDetTh;       // Ensure this threshold is in RPM if comparing with f16WmSpeedRpm
    int16_t i16WmCnt;
    int16_t i16DirCnt;
    int16_t AccCnt;
    Frac16_t PrevTheta;
    Frac16_t AccumTheta;
    WDir_t tWmDir;

    /* PLL related members */
    CTD_2SystF16_t tBemfUAlBe;      // Estimated BEMF Alpha/Beta from terminal voltage (Input to PLL)   //alpha beta坐标系下的电压
    Frac16_t f16WmThetaEst;         // Estimated electrical angle from PLL (Q15, 0-32767 -> 0-2pi)      //电角度
    Frac16_t f16WmOmegaEst;         // Estimated electrical speed from PLL (Q15, normalized to OMEGA_BASE) //电角速度
    Frac32_t f32WmThetaEst32;       // 32-bit accumulator for angle integration
    Frac16_t f16WmSpeedRpm;         // Estimated mechanical speed in RPM    机械转速
    AMC_TrackObser_t tWmPllObsvr;   // PLL observer structure for wind detection    //顺逆风检测的PLL观测器结构体
    MM_PolarTrigono_t tWmSinCos;      // Sin/Cos values for estimated angle
    Frac16_t f16PllError;      // PLL error input (Q15) // PLL误差输入 可以用于判定是否稳定

    //顺逆风检测时间
    int16_t i16WMDetTimeCnt;

    //刹车计数器
    int16_t i16BrakeCnt; // Count for brake detection (if needed)
    //刹车总时长
    int16_t i16BrakeTotalTime; // Total time for brake detection (if needed)
    //刹车模式 0 刹车 1观测
    int16_t i16BrakeMode; // Brake mode (0 for brake, 1 for observation)
    int16_t i16BrakeCheckTimer; // Timer for brake check (if needed)
    int16_t i16BrakeFreeTimer; // Free time for brake detection (if needed)
    int16_t i16MaxGduVolt; // Maximum voltage for brake detection (if needed)
    int16_t i16MinGduVolt; // Minimum voltage for brake detection (if needed)
    int16_t i16GduDiffVolt; // Voltage difference for brake detection (if needed)
    int16_t i16GduDiffVoltThreshold; // Threshold for voltage difference (if needed)
    int16_t i16GduVoltBrake; //
    int16_t i16EnableGetGduMaxMin;
    //速度稳定计数器
    int16_t i16WmSpeedStablityCnt;
    //顺逆风测总时长
    int16_t i16WmDetectTotaltime;

    GCF_Filter1_LPF16_t tAmpFilter;      // 反电动势幅值平方的滤波器
    GCF_Filter1_LPF16_t tAmpSqureFilter; // 反电动势幅值的滤波器
    


} WM_Obsvr_PLL_t;

extern bool is_ccw_wind;
extern uint32_t wind_detect_time_cnt;
extern uint32_t wind_detect_init_counter; // 风检测初始化计数器
extern uint16_t headwind_det_stability_counter;
extern int16_t headwind_machine_speed; // 机械转速
extern int16_t headwind_pll_error_input; // 机械转速1
// 风检测子状态和计数器变量
extern WindDetectSubState_t wind_detect_state; // 风检测子状态
extern uint16_t brake_counter;  // 刹车计数器

//Q轴PI
extern PLL_PI headwind_axis_q_ctrl;          // Q轴PI控制器
//D轴PI
extern PLL_PI headwind_axis_d_ctrl;          // D轴PI控制器
extern int16_t headwind_bemf_theta;         // 反电动势估算的角度
extern int16_t headwind_machine_speed_flt;  

extern int32_t headwind_bemf_theta_32bit;   // 反电动势估算的角度32位
extern int16_t headwind_bemf_omega;         // 反电动势估算的速度 
extern int16_t flux_theta;                  // 转子角度
extern PLL_PI headwind_pll;              // PLL PI结构体

extern void head_wind_pll_init(WM_Obsvr_PLL_t* ptWmObsvr);
extern void WM_WindDetectionPll(CTD_2SystF16_t *tUDQReq ,WM_Obsvr_PLL_t* ptWmObsvr,const FOC_Paras_t* ptFocParas);
void WM_SwitchSmoothpll(GCF_CtrlPIAW_RTF16_t *tAxisDCtrl,GCF_CtrlPIAW_RTF16_t *tAxisQCtrl,GCF_CtrlPIAW_PTF16_t *tSpeedCtrl,
                        GCF_Ramp_F16_t *tSpeedRamp, WM_Obsvr_PLL_t* ptWmObsvr,AMC_BemfcObsvrDQ_t *tBemfObsvr,CTD_2SystF16_t *tIAlBeFbck,
                        const FOC_Paras_t* ptFocParas);
// extern void run_parallel_pi(PLL_PI *pParm);
#endif // __WINDMILLING_H__
