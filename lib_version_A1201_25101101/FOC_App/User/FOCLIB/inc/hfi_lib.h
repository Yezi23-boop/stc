#ifndef HFI_LIB_H
#define HFI_LIB_H

#include "nvsns_type_def.h"
#include "gcf_lib.h"
#include "gmc_lib.h"
#include "amc_lib.h"
#include "pll_pi.h"
#include "typedef.h"



// HFI状态枚举
typedef enum {
    HFI_STATE_IDLE,         // 空闲状态
    HFI_STATE_IPD,          // 初始位置检测
    HFI_STATE_POLARITY,     // 磁极极性判断
    HFI_STATE_TRACKING,     // 高频注入运行状态
    HFI_STATE_TRANSITION,    // 转换到无传感器控制
    HFI_STATE_IDENTIFY      // 新增：电感参数辨识状态
} HFI_State_t;

// HFI参数配置结构体
typedef struct {
    Frac16_t f16HfiVoltage;         // 高频注入电压幅值
    Frac16_t f16PolarityTestVoltage; // 磁极判断测试电压
    Frac16_t f16IdRefValue;         // 高频注入运行时d轴电流参考值
    Frac16_t f16PllKp;              // HFI PLL的比例增益
    Frac16_t f16PllKi;              // HFI PLL的积分增益   
    uint16_t u16IpdDuration;        // 初始位置检测持续时间(周期数)

    uint16_t u16PwmFreq;            // HFI工作时的PWM频率
    uint16_t u16FilterConstant;     // 高频电流滤波常数
    //是否启用IPD_DEBUG
    int16_t bIpdDebugEnable;
    int16_t bPolarityDebugEnable;
    //HFI_TRACKING_DEBUG
    int16_t bTrackingDebugEnable;
    //CHECK_FLUX_ENV
    int16_t bCheckFluxEnv;
    //ENABLE_ID_OFFSET
    int16_t bEnableIdOffset;
    //ID_OFFSET
    Frac16_t f16IdOffset;
    //HFI_POLARITY_VOLTAGE
    Frac16_t f16PolarityVoltage;
    //HFI_SPEED_RAMP_UP
    Frac16_t f16SpeedRampUp;
    Frac16_t f16SpeedRampDown;
    //HFI_SPEED_TRAN2CLOSE
    Frac16_t f16TransitionSpeed;// 切换到无传感器控制的速度阈值
    //HFI_TRAN2CLOSE_TIME
    uint16_t u16TransitionConfirmTime; // 切换确认时间(周期数)
    //HFI_IPD_TIME
    uint16_t u16IpdTime; // IPD时间(周期数)
    uint32_t u32PolarityIdThreshold; // 磁极判断电流阈值

} HFI_Config_t;

// HFI控制器结构体
typedef struct {
    HFI_State_t eState;             // HFI当前状态
    Frac16_t f16Theta;              // 估计的电气角度
    GCF_CtrlPIAW_RTF16_t tPllCtrl;   /* PLL PI controller */
    GCF_Integrator_TRF16_t tPllInte; /* angle integrator */
    GCF_Filter1_LPF16_t tSpeedFilter;
    Frac16_t f16Omega;              // 估计的角速度
    Frac16_t f16MachineSpeed;       // 机械速度
    Frac16_t f16HfiVoltage;         // 当前高频注入电压(可能会切换正负)
    Frac16_t f16DeltaTheta;         // 位置误差信号
    Frac32_t f32DeltaThetaSum;      // 位置误差积分
    Frac16_t f16PllKp;              // PLL比例增益
    Frac16_t f16PllKi;              // PLL积分增益
    PLL_PI tPll;    // PLL结构体
    Frac16_t f16AngleIntegCoef;     // 角度积分系数
    uint16_t u16AngleIntegShift;    // 角度积分移位
    Frac32_t f32IntegLimit;         // 积分限幅
    uint32_t u16StateCnt;           // 状态计数器
    CTD_2SystF16_t tIAlBeHfi;       // 高频电流分量
    CTD_2SystF16_t tIAlBeHfiLast;       // 高频电流分量
    CTD_2SystF16_t tIAlBeLf;        // 低频电流分量
    CTD_2SystF16_t tIAlBeLfLast;        // 低频电流分量
    CTD_2SystF16_t tIAlBePrev;      // 上一次电流值
    Frac32_t f32IdSum1;             // 极性判断第一方向电流累加
    Frac32_t f32IdSum2;             // 极性判断第二方向电流累加
    Frac32_t f32IdDiff;       // 极性判断电流差值   
    bool bPolarityValid;            // 极性判断是否有效
    bool bPolarityInverted;         // 极性是否需要反转
    bool bEnabled;                  // HFI功能使能标志

    MM_PolarTrigono_t tTrigono; // 三角函数查表结构体
    CTD_2SystF16_t tHFIUdqReq; // HFI出来的Udq
    // CTD_2SystF16_t tHFIUAlBe; // HFI出来的Ualbe
    CTD_2SystF16_t tHFIIdqReq; // HFI出来的Idq参考
    CTD_2SystF16_t tHFIIAlBeFbck; // HFI出来的IAlBe反馈
    CTD_2SystF16_t tHFIIdqFbck; // HFI出来的Idq反馈
    CTD_2SystF16_t tHFIIdqErr; // HFI出来的Idq误差
    CTD_2SystF16_t tHFIUAlBeReq;
    CTD_2SystF16_t tHFIUAlBeReqDCB; // HFI出来的Ualbe参考，直流母线补偿后的
    GCF_CtrlPIAW_RTF16_t tHFIAxisDCtrl;
    GCF_CtrlPIAW_RTF16_t tHFIAxisQCtrl;
    //运行阶段的Q轴电流参考
    Frac16_t f16IdRefRun;
    Frac16_t f16IqRefRun;
    Frac16_t f16DcBusFilt;
    GMC_DcBusComp_t tDcBusComp;
    CTD_3SystF16_t tIabcFbck;

        
    // 高频注入模式控制
    uint8_t u8InjectionMode;     // 0:双值模式(-1/+1)  1:三值模式(-1/0/+1)
    uint8_t u8InjectionState;    // 三值模式下当前的注入状态 0:-1, 1:0, 2:+1
    //uint8_t u8InjectionState;    // 三值模式下当前的注入状态 0:-1, 1:0, 2:+1
                                // 分频模式下为当前注入的计数状态
    bool bShouldSample;          // 是否应该在本周期采样
    uint8_t u8DividerRatio;      // 分频模式下的分频系数
    uint8_t u8DividerCount;      // 分频模式下的当前计数

        // 随机注入噪声降低相关
    uint16_t u16RandomCount;        // 随机计数器
    uint16_t u16NextRandomInterval; // 下一次随机变化的间隔
    uint16_t u16RandomDuration;     // 随机变化持续时间
    uint16_t u16BasePwmFreq;        // 基础PWM频率
    uint16_t u16CurrentPwmFreq;     // 当前PWM频率
    Frac16_t f16BaseVoltage;        // 基础注入电压
    Frac16_t f16RandomVoltage;      // 随机注入电压
    bool bIsRandomActive;           // 是否处于随机模式

    // 分频模式下电流采样相关
    CTD_2Syst32_t tIAlBeAccum1;      // 第一组电流累加值(正极性)
    CTD_2Syst32_t tIAlBeAccum2;      // 第二组电流累加值(负极性)
    uint8_t u8AccumCount1;            // 第一组累加计数
    uint8_t u8AccumCount2;            // 第二组累加计数
    bool bAccumulating1;              // 当前是否在累加第一组
    bool bAccumCompleted;             // 一个完整采样周期是否完成
    
    // 电感辨识相关变量
    Frac16_t f16IdentifyStepAngle;    // 角度步进值，默认FRAC16(0.05)
    uint16_t u16IdentifyStepTime;     // 每个角度点停留时间，默认200ms对应的周期数
    Frac32_t f32IdentifyAngle;        // 当前辨识角度
    Frac16_t f16IdentifyMaxValue;     // 记录的最大值
    Frac16_t f16IdentifyMinValue;     // 记录的最小值
    Frac16_t f16IdentifyDiffValue;    // 最大值和最小值的差值
    uint16_t u16IdentifyStepCount;    // 当前步进计数
    bool bIdentifyComplete;           // 辨识完成标志
    Frac32_t f32IdentifyAlpha1Sum;    // Alpha1电流累加值
    Frac32_t f32IdentifyAlpha2Sum;    // Alpha2电流累加值
    uint16_t u16IdentifyFilterCount;  // 滤波累加计数
    Frac16_t f16IdentifyAlpha1Avg;    // Alpha1平均值
    Frac16_t f16IdentifyAlpha2Avg;    // Alpha2平均值

    // 强制定位和电流恢复相关变量
    bool bForceAlignActive;          // 强制定位标志
    bool bCurrentRecoveryActive;     // 电流恢复标志
    Frac16_t f16AlignTargetVoltage;  // 强制定位目标电压
    Frac16_t f16AlignVoltageStep;    // 电压爬坡步长
    uint16_t u16AlignHoldTime;       // 强制定位保持时间(PWM周期数)
    uint16_t u16RecoveryTime;        // 电流恢复时间(PWM周期数)
    Frac16_t f16AlignVoltage;        // 当前强制定位电压
    uint16_t u16AlignCnt;            // 强制定位计数器
    uint16_t u16RecoveryCnt;         // 电流恢复计数器
    bool bAlignVoltageReached;       // 电压是否已达到目标值
    // 电感物理值
    float f32IdentifyLd;           // D轴电感物理值(mH)
    float f32IdentifyLq;           // Q轴电感物理值(mH)
    //计算物理值标志
    bool bCalculatePhysicalValues; // 是否计算物理值标志
    GCF_CtrlPIAW_PTF16_t tSpeedCtrl;
    //是否是HFI到BEMF的第一拍
    bool bIsFirstHfiToBemf;

} HFI_Controller_t;

extern HFI_Controller_t tHfiCtrl;     // HFI控制器
extern HFI_Config_t tHfiConfig;       // HFI配置
extern int32_t hfi_theta_32bit; 

// 函数声明
extern void HFI_Init(HFI_Controller_t *pHfi, HFI_Config_t *pConfig);
void HFI_ResetState(HFI_Controller_t *pHfi);
void HFI_Process(HFI_Controller_t *pHfi,HFI_Config_t *pHfiCfg);
bool HFI_IsTransitionComplete(HFI_Controller_t *pHfi);
void HFI_ChangePwmFreq(uint16_t u16Freq);
void HFI_GetEstimatedAngle(HFI_Controller_t *pHfi, Frac16_t *pf16Theta, Frac16_t *pf16Omega);
void HFI_Enable(HFI_Controller_t *pHfi, bool bEnable);
extern void HFI_StartIdentify(HFI_Controller_t *pHfi, Frac16_t f16StepAngle, uint16_t u16StepTimeMs);
extern bool HFI_IsIdentifyComplete(HFI_Controller_t *pHfi);
extern void HFI_ResetPwmConfig(void);
extern void HFI_SetInjectionMode(HFI_Controller_t *pHfi, uint8_t u8Mode, uint8_t u8DividerRatio);
extern void HFI_CalculateInductanceValues(HFI_Controller_t *pHfi);
extern void HFI_ProcessPolarity(HFI_Controller_t *pHfi, HFI_Config_t *pHfiCfg);
extern void HFI_ExtractCurrents(HFI_Controller_t *pHfi);
extern void HFI_SwitchInjectionVoltage(HFI_Controller_t *pHfi);
extern uint16_t HFI_GenerateRandom(void);
extern int16_t HFI_GetLimitedRandom(int16_t maxRange);
extern void WIND_SwitchToHFI(HFI_Controller_t *pHfi);


#endif // HFI_LIB_H