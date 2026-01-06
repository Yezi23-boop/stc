/*
 * 文件名: foclib_init.c
 * 版本日期: 2025-10-16
 * 描述:
 *   foclib参数初始化相关函数，不封库，方便根据宏定义进行配置
 *
 * 变更记录:
 *   2025-09-08  创建文件，支持7段式与5段式SVPWM，支持切换功能。
 */

#include "foclib_init.h"
#include "foc_paras.h"
#include "foc_config.h"
#include "nsuc1602.h"


void head_wind_pll_init(WM_Obsvr_PLL_t* ptWmObsvr)
{ 
    //PLL参数
    int16_t pll_bandwidth_Hz;
    int16_t pll_bandwidth_flux_Q15;
    int16_t pll_kp,pll_ki;

    is_ccw_wind = false; // 初始化为顺风工况

    // 初始化反电动势幅值滤波器 (8点移动平均)
    ptWmObsvr->tAmpFilter.f16NSample = 8;
    ptWmObsvr->tAmpFilter.f32Acc = 0;
    ptWmObsvr->i16AmpDetCnt = 0; // 初始化反电动势幅值确认计数器
    
    // 初始化反电动势幅值平方根滤波器 (8点移动平均)
    ptWmObsvr->tAmpSqureFilter.f16NSample = 8;
    ptWmObsvr->tAmpSqureFilter.f32Acc = 0;

    ptWmObsvr->u32AmpFlt = 0;
    headwind_machine_speed =0;
    headwind_machine_speed_flt = 0; // 初始化滤波后的机械转速

    wind_detect_state = WIND_STATE_DETECT;
    brake_counter = 0;  // 刹车计数器


    headwind_bemf_theta = 0;
    headwind_bemf_theta_32bit = 0;
    headwind_bemf_omega = 0;


    flux_theta =0;

    headwind_pll_error_input = 0XFFFF;
    wind_detect_init_counter = 0;
    wind_detect_time_cnt = 0;
    headwind_det_stability_counter = 0;
    
    
    pll_bandwidth_Hz = WIND_DETECT_PLL_BANDWIDTH_HZ;       // 15Hz
    pll_bandwidth_flux_Q15 = FRAC16(pll_bandwidth_Hz/FREQUENCY_BASE);
    pll_kp = pll_bandwidth_flux_Q15 *2;

    pll_ki = MulF16(pll_bandwidth_flux_Q15,pll_bandwidth_flux_Q15);
    pll_ki = MulF16(pll_ki,FRAC16((FOC_TS/T_BASE)));  // Ki* Tspu

    init_pll_pi(&headwind_pll,pll_kp,pll_ki,FRAC16(0.998),FRAC16(1.0),-FRAC16(1.0));

    // D轴PI控制器的比例增益计算
    // 计算平均电感值
    Frac16_t avg_inductance = (PHASE_Q_INDUCTANCE + PHASE_D_INDUCTANCE) / 2;
    // 控制带宽因子 (2π*Fsw*1000/11)
    // Frac16_t bandwidth_factor = 6.28 * FOC_PWM_FREQ * 1000 / 11;
    Frac16_t bandwidth_factor = 571 * ((uint16_t)FOC_EPWM_FREQUENCY); //571 = 6.28 * 1000 / 11
    // 计算比例增益
    Frac16_t d_axis_kp = FRAC16(avg_inductance * bandwidth_factor);

    // D轴PI控制器的积分增益计算
    // 分母 = 平均电感 * 开关频率 * 1000
    Frac16_t denominator = avg_inductance * FOC_EPWM_FREQUENCY * 1000;
    // 计算积分增益 = 电阻/分母
    Frac16_t d_axis_ki = FRAC16(PHASE_RESISTANCE / denominator);

    init_pll_pi(&headwind_axis_d_ctrl,d_axis_kp,d_axis_ki,FRAC16(0.998),FRAC16(1.0),-FRAC16(1.0));
    init_pll_pi(&headwind_axis_q_ctrl,d_axis_kp,d_axis_ki,FRAC16(0.998),FRAC16(1.0),-FRAC16(1.0));

}

 // 初始化HFI控制器
void HFI_Init(HFI_Controller_t *pHfi, HFI_Config_t *pConfig)
{
    int16_t pll_bandwidth_Hz;
    int16_t pll_bandwidth_flux_Q15;
    int16_t pll_kp,pll_ki;

    // 初始化电感物理值
    pHfi->f32IdentifyLd = 0.0f;
    pHfi->f32IdentifyLq = 0.0f;
    pHfi->bCalculatePhysicalValues = false;
    // 初始化电感辨识相关变量
    pHfi->f32IdentifyAlpha1Sum = 0;
    pHfi->f32IdentifyAlpha2Sum = 0;
    pHfi->u16IdentifyFilterCount = 0;
    pHfi->f16IdentifyAlpha1Avg = 0;
    pHfi->f16IdentifyAlpha2Avg = 0;

    // 初始化随机注入相关参数
    pHfi->u16RandomCount = 0;
    pHfi->u16NextRandomInterval = HFI_RANDOM_INTERVAL_MIN;
    pHfi->u16RandomDuration = 0;
    pHfi->u16BasePwmFreq = HFI_PWM_FREQUENCY;
    pHfi->u16CurrentPwmFreq = HFI_PWM_FREQUENCY;
    pHfi->f16BaseVoltage = HFI_VOLTAGE;
    pHfi->f16RandomVoltage = HFI_VOLTAGE;
    pHfi->bIsRandomActive = false;

    pHfi->u8InjectionMode = HFI_INJECTION_MODE;  // 默认使用双值模式
    pHfi->u8InjectionState = 0;  // 开始于+1状态
    pHfi->bShouldSample = true;  // 初始允许采样
    pHfi->u8DividerRatio = HFI_DIVIDER_RATIO;    // 默认分频系数为2
    pHfi->u8DividerCount = 0;    // 分频计数器初始化为0

    // 初始化分频模式下电流采样相关变量
    pHfi->tIAlBeAccum1.f16Arg1 = 0;
    pHfi->tIAlBeAccum1.f16Arg2 = 0;
    pHfi->tIAlBeAccum2.f16Arg1 = 0;
    pHfi->tIAlBeAccum2.f16Arg2 = 0;
    pHfi->u8AccumCount1 = 0;
    pHfi->u8AccumCount2 = 0;
    pHfi->bAccumulating1 = true;  // 从第一组开始累加
    pHfi->bAccumCompleted = false;

    // 复位HFI状态
    pHfi->eState = HFI_STATE_IDLE;
    pHfi->f16Theta = 0;
    pHfi->f16Omega = 0;
    pHfi->f16HfiVoltage = HFI_VOLTAGE;
    pHfi->f16DeltaTheta = 0;
    pHfi->f32DeltaThetaSum = 0;
    
    // PLL参数设置
    pHfi->f16PllKp = pConfig->f16PllKp;
    pHfi->f16PllKi = pConfig->f16PllKi;

    pll_bandwidth_Hz = HFI_PLL_BANDWIDTH;       // 80Hz
    pll_bandwidth_flux_Q15 = FRAC16(pll_bandwidth_Hz/FREQUENCY_BASE);
    pll_kp = pll_bandwidth_flux_Q15 *2;

    pll_ki = MulF16(pll_bandwidth_flux_Q15,pll_bandwidth_flux_Q15);
    pll_ki = MulF16(pll_ki,FRAC16((HFI_TS/T_BASE)));  // Ki* Tspu

    init_pll_pi(&pHfi->tPll,pll_kp,pll_ki,FRAC16(0.998),FRAC16(1.0),-FRAC16(1.0));

    // 角度积分系数计算
    // 这里是一个简化的近似，实际应根据PWM频率调整
    pHfi->f16AngleIntegCoef = FRAC16(0.1);  // 这个系数需要根据PWM频率适配
    pHfi->u16AngleIntegShift = 3;           // 这个移位值需要根据PWM频率适配
    
    // 初始化电流相关变量
    pHfi->tIAlBeHfi.f16Arg1 = 0;
    pHfi->tIAlBeHfi.f16Arg2 = 0;

    pHfi->tIAlBeLf.f16Arg1 = 0;
    pHfi->tIAlBeLf.f16Arg2 = 0;

    pHfi->tIAlBePrev.f16Arg1 = 0;
    pHfi->tIAlBePrev.f16Arg2 = 0;

    pHfi->tHFIUdqReq.f16Arg1 = 0;
    pHfi->tHFIUdqReq.f16Arg2 = 0;

    pHfi->tHFIIdqReq.f16Arg1 = 0;
    pHfi->tHFIIdqReq.f16Arg2 = 0;

    pHfi->tHFIIAlBeFbck.f16Arg1 = 0;
    pHfi->tHFIIAlBeFbck.f16Arg2 = 0;

    pHfi->tHFIIdqFbck.f16Arg1 = 0;
    pHfi->tHFIIdqFbck.f16Arg2 = 0;

    pHfi->tHFIIdqErr.f16Arg1 = 0;
    pHfi->tHFIIdqErr.f16Arg2 = 0;

    pHfi->tHFIUAlBeReqDCB.f16Arg1 = 0;
    pHfi->tHFIUAlBeReqDCB.f16Arg2 = 0;


    pHfi->u16StateCnt = 0;


    // 初始化极性判断相关变量
    pHfi->f32IdSum1 = 0;
    pHfi->f32IdSum2 = 0;
    pHfi->bPolarityValid = false;
    pHfi->bPolarityInverted = false;
    
    pHfi->u16StateCnt = 0;
    pHfi->bEnabled = false;

    pConfig->bCheckFluxEnv = CHECK_FLUX_ENV;
    
    pConfig->bIpdDebugEnable = HFI_IPD_DEBUG;
    // HFI_POLARITY_DEBUG
    pConfig->bPolarityDebugEnable = HFI_POLARITY_DEBUG;
    pConfig->bTrackingDebugEnable = HFI_TRACKING_DEBUG;
    pConfig->bEnableIdOffset = ENABLE_ID_OFFEST;
    pConfig->f16IdOffset = ID_OFFEST;

    pConfig->f16PolarityVoltage = HFI_POLARITY_VOLTAGE;
    pConfig->f16SpeedRampUp = HFI_SPEED_RAMP_UP;
    pConfig->f16SpeedRampDown = HFI_SPEED_RAMP_DOWN;
    pConfig->f16TransitionSpeed = HFI_SPEED_TRAN2CLOSE;
    pConfig->u16TransitionConfirmTime = HFI_TRAN2CLOSE_TIME;
    pConfig->u16IpdTime = HFI_IPD_TIME;
    pConfig->u32PolarityIdThreshold = HFI_POLARITY_ID_THRESHOLD;



    hfi_theta_32bit = 0;
    
    
}

// 修改HFI_Enable函数，在启用时保存基础频率
void HFI_Enable(HFI_Controller_t *pHfi, bool bEnable)
{
    if (bEnable && !pHfi->bEnabled) {
        // 启用HFI
        pHfi->bEnabled = true;
        pHfi->eState = HFI_STATE_IPD;
        pHfi->u16StateCnt = 0;
        
        // 保存基础PWM频率和注入电压
        pHfi->u16BasePwmFreq = HFI_PWM_FREQUENCY;
        pHfi->u16CurrentPwmFreq = HFI_PWM_FREQUENCY;
        pHfi->f16BaseVoltage = HFI_VOLTAGE;
        
        // 重置随机注入相关参数，使用改进的随机数生成器
        pHfi->u16RandomCount = 0;
        uint16_t range = HFI_RANDOM_INTERVAL_MAX - HFI_RANDOM_INTERVAL_MIN;
        pHfi->u16NextRandomInterval = HFI_RANDOM_INTERVAL_MIN + (HFI_GenerateRandom() % (range + 1));
        pHfi->bIsRandomActive = false;
        
        // 调整PWM频率为HFI所需频率
        HFI_ChangePwmFreq(HFI_PWM_FREQUENCY);
    }
    else if (!bEnable && pHfi->bEnabled) {
        // 禁用HFI
        pHfi->bEnabled = false;
        pHfi->eState = HFI_STATE_IDLE;
        
        // 恢复默认PWM配置
        HFI_ResetPwmConfig();
    }
}

// PWM频率调整函数
void HFI_ChangePwmFreq(uint16_t u16Freq)
{
    // 保存当前PWM状态
    // uint8_t u8EnableOut = EPWM->PMANUALCON1 ;
    
    // // 禁用PWM输出
    // EPWM->PWMRWEN_b.PWMRLDEN = 0x55; // 使能寄存器写入
    // EPWM->PMANUALCON1 = 0x3F;        // 设为手动控制
    // EPWM->PWMRWEN_b.PWMRLDEN = 0xAA; // 使能寄存器加载
    
    // 调整PWM频率
    EPWM->PWMRWEN_b.PWMRLDEN = 0x55;
    EPWM->PWMPERIOD_b.PWMP =  FOC_EPWM_PERIOD_CALC(u16Freq); // 设置为HFI所需频率
    EPWM->PWMRWEN_b.PWMRLDEN = 0xAA;
    
    // // 如果之前使能了输出，则恢复
    // // if (u8EnableOut) {
    //     EPWM->PWMRWEN_b.PWMRLDEN = 0x55;
    //     EPWM->PMANUALCON1 = u8EnableOut;    // 输出由PWM生成器控制
    //     EPWM->PWMRWEN_b.PWMRLDEN = 0xAA;
    // // }
}

// 重置PWM配置为默认FOC配置
void HFI_ResetPwmConfig(void)
{
    // 恢复原始PWM频率和配置
    EPWM->PWMRWEN_b.PWMRLDEN = 0x55;
    EPWM->PWMPERIOD_b.PWMP = FOC_EPWM_PERIOD;  // 使用项目中定义的PWM周期
    EPWM->PWMRWEN_b.PWMRLDEN = 0xAA;
}



// 启动电感参数辨识
void HFI_StartIdentify(HFI_Controller_t *pHfi, Frac16_t f16StepAngle, uint16_t u16StepTimeMs)
{
    // 设置参数
    pHfi->f16IdentifyStepAngle = f16StepAngle;
    // 将毫秒转换为PWM周期数
    pHfi->u16IdentifyStepTime = HFI_IDENTIFY_STEP_TIME;
    
    // 初始化辨识状态
    pHfi->f32IdentifyAngle = FRAC16(-1.0);  // 从-π开始
    pHfi->u16StateCnt = 0;
    pHfi->u16IdentifyStepCount = 0;
    pHfi->bIdentifyComplete = false;
    pHfi->f16IdentifyMaxValue = 0;
    pHfi->f16IdentifyMinValue = 0;
    pHfi->f16IdentifyDiffValue = 0;

    // 初始化强制定位参数
    pHfi->bForceAlignActive = true;                // 启用强制定位
    pHfi->bCurrentRecoveryActive = false;          // 初始化电流恢复标志
    pHfi->f16AlignTargetVoltage = HFI_IDENTIFY_VOLTAGE;    // 强制定位目标电压
    pHfi->f16AlignVoltageStep = FRAC16(0.0001);    // 电压爬坡步长
    pHfi->u16AlignHoldTime = HFI_IDENTIFY_ALIGN_TIME;    // 保持时间3秒
    pHfi->u16RecoveryTime = HFI_IDENTIFY_ALIGN_DELAY_TIME;      // 电流恢复时间500ms
    pHfi->f16AlignVoltage = 0;                     // 当前电压从0开始
    pHfi->u16AlignCnt = 0;                         // 计数器清零
    pHfi->u16RecoveryCnt = 0;                      // 恢复计数器清零
    pHfi->bAlignVoltageReached = false;            // 尚未达到目标电压
    // 启用HFI并设置为辨识状态
    pHfi->bEnabled = true;
    pHfi->eState = HFI_STATE_IDENTIFY;

    

    pHfi->f32IdentifyAlpha1Sum = 0;
    pHfi->f32IdentifyAlpha2Sum = 0;
    pHfi->u16IdentifyFilterCount = 0;
    pHfi->f16IdentifyAlpha1Avg = 0;
    pHfi->f16IdentifyAlpha2Avg = 0;
    // 调整PWM频率为HFI所需频率
    //HFI_ChangePwmFreq(HFI_PWM_FREQUENCY);
}

// 初始化电机参数辨识
void PARAM_ID_Init(MotorParamId_t *pParamId)
{

    
    
    // 初始化状态变量
    pParamId->state = PARAM_ID_IDLE;
    // pParamId->bIdentifyActive = false;
    pParamId->bIdentifyComplete = false;
    
    // 初始化强制定位相关变量
    pParamId->bForceAlignActive = false;
    pParamId->u16AlignCnt = 0;
    pParamId->f16AlignVoltage = 0;
    pParamId->bAlignVoltageReached = false;
    
    // 初始化电流恢复相关变量
    pParamId->bCurrentRecoveryActive = false;
    pParamId->u16RecoveryCnt = 0;
    
    // 初始化电阻辨识相关变量
    pParamId->f16VoltRcd[0] = 0;
    pParamId->f16VoltRcd[1] = 0;
    pParamId->f16CurrRcd[0] = 0;
    pParamId->f16CurrRcd[1] = 0;
    pParamId->i32VoltAccum = 0;
    pParamId->u8VoltAvgCnt = 0;
    pParamId->u16ParaidCnt = 0;
    
    // 初始化滤波器
    // GCF_Filter1LpInit_F16(&pParamId->tVoltFilter, FRAC16(0.05));
    // GCF_Filter1LpInit_F16(&pParamId->tCurrFilter, FRAC16(0.05));

    pParamId->tVoltFilter.f16NSample = 4; // 设置为4个采样点
    pParamId->tVoltFilter.f32Acc = 0;
    pParamId->tCurrFilter.f16NSample = 4; // 设置为4个采样点
    pParamId->tCurrFilter.f32Acc = 0;
    
    // 初始化电感辨识相关变量
    pParamId->f32ScanAngle = FRAC16(-1.0);  // 从-π开始
    pParamId->f16MaxValue = 0;
    pParamId->f16MinValue = 0;
    pParamId->u16ScanCnt = 0;
    pParamId->u16StepCount = 0;
    
    // 初始化辨识结果
    pParamId->f32Rs = 0.0f;
    pParamId->f32Ld = 0.0f;
    pParamId->f32Lq = 0.0f;
    pParamId->f32Flux = 0.0f;

    pParamId->u16StepTime = HFI_IDENTIFY_STEP_TIME;
}

// 启动电机参数辨识
void PARAM_ID_Start(MotorParamId_t *pParamId)
{
    // 初始化状态
    pParamId->state = PARAM_ID_ADC_CALIBRATION;
    pParamId->bIdentifyActive = true;
    pParamId->bIdentifyComplete = false;

    pParamId->f16AlignVoltageStep = FRAC16(0.0001);  // 设置电压爬坡步长

        // 初始化配置参数
    pParamId->f16AlignTargetVoltage = HFI_IDENTIFY_VOLTAGE;
    pParamId->f16StepAngle = HFI_IDENTIFY_STEP_ANGLE;

    pParamId->f16TestVoltage = PARAM_ID_R_START_VOLTAGE;  // 初始测试电压设为较小值
    pParamId->f16VoltageStep = PARAM_ID_R_STEP_VOLTAGE; // 电压步进值设为较小值以实现平滑调整

    // 初始化强制定位参数
    pParamId->bForceAlignActive = true;
    pParamId->u16AlignTime = HFI_IDENTIFY_ALIGN_TIME; // 强制定位时间
    pParamId->f16AlignVoltage = 0;
    pParamId->u16AlignCnt = 0;
    pParamId->bAlignVoltageReached = false;
    
    // 初始化电流恢复相关变量
    pParamId->bCurrentRecoveryActive = false;
    pParamId->u16RecoveryTime = HFI_IDENTIFY_ALIGN_DELAY_TIME; // 电流恢复时间
    pParamId->u16RecoveryCnt = 0;
    
    // 初始化电阻辨识相关变量
    pParamId->f16VoltRcd[0] = 0;
    pParamId->f16VoltRcd[1] = 0;
    pParamId->f16CurrRcd[0] = 0;
    pParamId->f16CurrRcd[1] = 0;
    pParamId->i32VoltAccum = 0;
    pParamId->u8VoltAvgCnt = 0;
    pParamId->u16ParaidCnt = 0;
    
    // 初始化电感辨识相关变量
    pParamId->f32ScanAngle = FRAC16(-1.0);
    pParamId->f16MaxValue = 0;
    pParamId->f16MinValue = 0;
    pParamId->u16ScanCnt = 0;
    pParamId->u16StepCount = 0;

    //磁链辨识相关
    pParamId->u16InnerSlowCntr = 0;
    pParamId->u8FluxDragUseIq = FLUX_IDENTIFY_USE_IQ;
    pParamId->u16FluxDragStage1Duration = FLUX_IDENTIFY_STAGE1_TIME;
    pParamId->u16FluxDragStage2Duration = FLUX_IDENTIFY_STAGE2_TIME;
    pParamId->u16FluxDragStage3Duration = FLUX_IDENTIFY_STAGE3_TIME;    

    pParamId->f16IdForceTargetStage1 = FLUX_IDENTIFY_STAGE1_CURRENT;
    pParamId->f16IdForceTargetStage2 = FLUX_IDENTIFY_STAGE2_CURRENT;

    pParamId->f16IdForceTarget = pParamId->f16IdForceTargetStage1;

    pParamId->u32FluxDragStageTime = 0;
    pParamId->f16FluxIdUq1 = 0;
    pParamId->f16FluxIdUq2 = 0;
    pParamId->f16FluxIdId1 = 0;
    pParamId->f16FluxIdId2 = 0;

    pParamId->f16wForceReq = FLUX_IDENTIFY_TARGET_SPEED;
    pParamId->f16wForceStep = FLUX_IDENTIFY_SPEED_STEP;
    pParamId-> u16FluxDragSpeedStableTime = FLUX_IDENTIFY_SPEED_STABLE_TIME;
    pParamId->bFluxDragSpeedStable = 0;


}


void stallDetectionInit(stallDetection_T *params)
{
    params->bEMFObs_Q = 0.0F;
    params->bEMFObsFilter.f16NSample = 2;
    params->bEMFObsFilter.f32Acc = 0.0F;
    params->wRotElFilt = 0.0F;
    params->wRotElFilter.f16NSample = 2;
    params->wRotElFilter.f32Acc = 0.0F;
    params->bEMFObsFilter_Q = 0.0F;
    params->bEMFKeCal_Q = 0.0F;
    params->bEMFKeCalL_Q = 0.0F;
    params->bEMFKeCalH_Q = 0.0F;
    params->coeffKE = FRAC16(STALLDETECTION_COEFFKE);             // offline test coeff, depend on the motor parameters
    params->coeffKENshift = STALLDETECTION_COEFFKENSHIFT; // usually 0
    params->coeffKEOFT = FRAC16(STALLDETECTION_COEFFKEOFT);
    params->coeffKEOFTNshift = STALLDETECTION_COEFFKEOFTNSHIFT; // usually 0
    params->blankCnt = 0;
    params->stallDetCnt = 0;
    params->coeffL = STALLDETECTION_COEFFL;
    params->coeffH = STALLDETECTION_COEFFH;
    params->stallErrFlag = false;
    params->enStallDetection = true; // Enable stall detection by default
    params->blankCntPeriod = STALLDETECTION_BLANKCNT;
    params->stallDetCntPeriod = STALLDETECTION_CHKCNT; // usually 200ms, but can be changed
    params->stallDetErrCntPeriod = STALLDETECTION_CHKERRCNT; // usually 5, but can be changed
    }