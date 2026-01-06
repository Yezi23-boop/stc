#ifndef MOTOR_PARAM_ID_H
#define MOTOR_PARAM_ID_H

#include "nvsns_type_def.h"
#include "gmc_lib.h"
#include "foc_paras.h"
#include "hfi_lib.h"
#include "gcf_lib.h"

// 电机参数辨识状态枚举
typedef enum {
    PARAM_ID_IDLE = 0,               // 空闲状态
    //ADC校准
    PARAM_ID_ADC_CALIBRATION,
    //切换PWM频率
    PARAM_ID_PWM_FREQ_SWITCH,

    PARAM_ID_ALIGN,              // 强制定位阶段
    PARAM_ID_RECOVERY,           // 电流恢复阶段
    PARAM_ID_INDUCTANCE_SCAN,    // 电感辨识-角度扫描
    PARAM_ID_RECOVERY_FOR_INDUCTANCE_SCAN, // 电流恢复-电感辨识
    PARAM_ID_LOW_CURRENT,        // 电阻辨识-低电流
    PARAM_ID_MID_CURRENT,        // 电阻辨识-中等电流

    // 新增：磁链辨识前的强拖阶段（在该阶段内自行执行 StateRun/FocSlowLoop/FocFastLoop 的精简版）
    PARAM_ID_FLUX_DRAG,

    

    PARAM_ID_COMPLETE            // 完成状态
} ParamId_State_t;

// 电机参数辨识结构体
typedef struct {
    // 配置参数
    Frac16_t f16AlignVoltage;        // 强制定位电压
    Frac16_t f16StepAngle;           // 电感辨识角度步进值

    
    // 状态变量
    ParamId_State_t state;           // 当前状态
    Frac16_t bIdentifyActive;            // 辨识是否激活
    bool bIdentifyComplete;          // 辨识是否完成

    CTD_2SystF16_t tUdqReq;          // Udq电压请求
    CTD_2SystF16_t tUAlBeReq;        // UAlBe电压请求
    
    // 强制定位和电流恢复相关变量
    bool bForceAlignActive;          // 强制定位标志
    bool bCurrentRecoveryActive;     // 电流恢复标志
    Frac16_t f16AlignTargetVoltage;  // 强制定位目标电压
    Frac16_t f16AlignVoltageStep;    // 电压爬坡步长


    //Frac16_t f16AlignVoltage;        // 当前强制定位电压
    uint16_t u16AlignCnt;            // 强制定位计数器
    //HFI_IDENTIFY_ALIGN_TIME
    uint16_t u16AlignTime;           // 强制定位时间
    uint16_t u16RecoveryCnt;         // 电流恢复计数器
    //HFI_IDENTIFY_ALIGN_DELAY_TIME
    uint16_t u16RecoveryTime;        // 电流恢复时间
    bool bAlignVoltageReached;       // 电压是否已达到目标值
    
    // 电阻辨识相关变量
    // 电阻辨识相关电压控制变量
    bool bRsDetVoltageReached;           // RS的检测电压是否达到目标值
    Frac16_t f16TestVoltage;        // 当前测试电压
    uint16_t u16VoltageStableCnt;    // 电压稳定计数器
    uint16_t u16DataRecordCnt;      // 数据记录计数器
    Frac16_t f16VoltageStep;        // 电压调整步长
    bool bCurrentStable;             // 电流是否稳定
    uint16_t u16CurrentStableCnt;   // 电流稳定计数器
    Frac16_t f16VoltRcd[2];          // 记录的电压值
    Frac16_t f16CurrRcd[2];          // 记录的电流值
    int32_t i32VoltAccum;            // 电压累加
    uint8_t u8VoltAvgCnt;            // 电压平均计数
    uint16_t u16ParaidCnt;           // 参数辨识计数
    GCF_Filter1_LPF16_t tVoltFilter; // 电压滤波器
    GCF_Filter1_LPF16_t tCurrFilter; // 电流滤波器
    
    // 电感辨识相关变量
    Frac32_t f32ScanAngle;           // 当前扫描角度
    Frac16_t f16MaxValue;            // 记录的最大值
    Frac16_t f16MinValue;            // 记录的最小值
    uint16_t u16ScanCnt;             // 扫描计数
    //HFI_IDENTIFY_STEP_TIME
    uint16_t u16StepTime;            // 每一个扫描点停留步进时间
    uint16_t u16StepCount;           // 当前步进计数
    // 电感辨识所需的额外字段
    MM_PolarTrigono_t tTrigono;      // 三角函数查表结构体
//    CTD_2SystF16_t tUdqReq;          // Udq电压请求
//    CTD_2SystF16_t tUAlBeReq;        // UAlBe电压请求
    CTD_2SystF16_t tUAlBeReqDCB;     // 直流母线补偿后的UAlBe
    GMC_DcBusComp_t tDcBusComp;      // 直流母线补偿

    // 新增：磁链强拖控制参数与状态
    int32_t  i32FluxStage1UampAcc;   // 累加 Udq 幅值
    int32_t  i32FluxStage1IampAcc;   // 累加 Idq 幅值
    uint16_t u16FluxStage1Cnt;       // 采样计数 0..1024
    Frac16_t f16IdForceTarget;     // 目标强拖电流
    //阶段1目标强拖电流
    Frac16_t f16IdForceTargetStage1;
    //阶段2目标强拖电流
    Frac16_t f16IdForceTargetStage2; //阶段2目标强拖电流
    Frac16_t f16IdForceStep;       // Id 爬坡步进
    Frac16_t f16wForceReq;         // 开环强拖电角速度(电频)
    //转速步进
    Frac16_t f16wForceStep;
    uint16_t u16FluxDragTime;      // 强拖保持时间(按PWM周期计数)
    uint16_t u16FluxDragCnt;       // 强拖计数
    uint16_t u16InnerSlowCntr;     // 内部慢环分频计数（复用 tFocParas.SPEED_LOOP_CNTR）
    bool     bFluxDragDone;        // 强拖完成标志
    //转速稳定时间
    uint16_t u16FluxDragSpeedStableTime;
    //转速稳定标志位
    bool     bFluxDragSpeedStable;
    //使用ID还是使用IQ强拖 0 使用ID 1使用IQ
    uint8_t  u8FluxDragUseIq;
    //阶段一所需时间
    uint16_t u16FluxDragStage1Duration;
    //阶段2所需时间
    uint16_t u16FluxDragStage2Duration;
    //阶段3所需时间
    uint16_t u16FluxDragStage3Duration;
    //阶段时间计数器
    uint32_t u32FluxDragStageTime;
    //磁链辨识中的数据记录
    //uq1
    Frac16_t f16FluxIdUq1;
    //ID2
    Frac16_t f16FluxIdId2;
    //UQ2
    Frac16_t f16FluxIdUq2;
    //ID1
    Frac16_t f16FluxIdId1;
    
    // 辨识结果
    float f32Rs;                     // 电阻值(Ω)
    float f32Ld;                     // D轴电感(mH)
    float f32Lq;                     // Q轴电感(mH)
    float f32Flux;                   // 磁链值(Wb)
} MotorParamId_t;



// 外部声明
extern MotorParamId_t tMotorParamId;

// 函数声明
void PARAM_ID_Init(MotorParamId_t *pParamId);
void PARAM_ID_Start(MotorParamId_t *pParamId);
bool PARAM_ID_IsComplete(MotorParamId_t *pParamId);
void PARAM_ID_SetAlignParameters(MotorParamId_t *pParamId, Frac16_t f16AlignVoltage, uint16_t u16AlignTimeMs);
void PARAM_ID_SetRecoveryTime(MotorParamId_t *pParamId, uint16_t u16RecoveryTimeMs);
void PARAM_ID_CalculateResistance(MotorParamId_t *pParamId);
void PARAM_ID_CalculateInductances(MotorParamId_t *pParamId);
void PARAM_ID_CalculateFluxLinkage(MotorParamId_t *pParamId);
void PARAM_ID_Reset(MotorParamId_t *pParamId);
void PARAM_ID_Process(MotorParamId_t *pParamId);
void PARAM_ID_CalculateInductanceValues(MotorParamId_t *pParamId);
#endif // MOTOR_PARAM_ID_H