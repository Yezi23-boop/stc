#include "nvsns_foc.h"
#include "nsuc1602.h"
#include "foc_config.h"
#include "foc_paras.h"
#include "fault_diagnose.h"
// #include "curr_sample_1rps.h"
#include "mtr_adv_init.h"
#include "deadtime_comp.h"
#include "smart_align.h"
#include "current_limit.h"
#include "fault_hw_config.h"
#include "motor_data_calc.h"
#include "SysTick.h"
#include "transform.h"
#include "svm_lib.h"
#include "bus_comp.h"
#include "curr_sample_1rps_aiwin.h"
#include "hfi_lib.h" 
#include "motor_param_id.h"
#include "dq_dual_space_switch.h"
#include "sensorlessObs.h"
#include "amc_fluxweakening_aiwin.h"
#include "iq_filter.h"
#include "qpr_fixed.h"
#include "stallDetection.h"
#include "variable_pi.h"
#include "pi.h"
#include "windmilling_pll.h"


#if  (FOC_EPWM_SHUNT_MODE == -1)
static bool g_bPredictCurrentsPending = false;
#endif



CTD_2SystF16_t tUAlBeReqDCBTest;
static PosMode_t ePrevPosModeSlowLoop = FORCE;

//宏定义 是否使能VF调试
#define ENABLE_VF_DEBUG 0
//是否启用强托时的阻尼补偿，该方案来自瑞萨，没有深度验证
#define ENABLE_DAMPING_COMPENSATION 0
//是否直接闭环启动
#define ENABLE_DIRECT_CLOSED_LOOP_START 0
//==================== PR控制配置 ====================
#define ENABLE_PR_CTRL        1      // 总开关：0关闭/1开启
#define PR_USE_MECH_FREQ      1      // 1: 用机械频率(需极对数)，0: 用电角频率
#define PR_MIN_FREQ_HZ        5      // 低于该频率不注入

// 速度环PR参数（对f16IqReq附加调制）
#define PR_SPEED_KR           FRAC16(0.7) // 增益Kr，建议0.1~0.5
#define PR_SPEED_ALPHA        FRAC16(0.01) // 带宽比例alpha（wc = alpha * w0）
#define PR_SPEED_YLIM         FRAC16(0.30) // 输出限幅（对Iq的附加量幅度）

// 电流环PR参数（对Uq附加ΔUq）
#define PR_IQ_KR              FRAC16(0.10)
#define PR_IQ_ALPHA           FRAC16(0.01)
#define PR_IQ_YLIM            FRAC16(0.2)

#define PR_ID_KR              FRAC16(0.50)
#define PR_ID_ALPHA           FRAC16(0.01)
#define PR_ID_YLIM            FRAC16(0.2)

// ==================== 转矩补偿配置 (Torque Compensation) ====================
#define ENABLE_TQ_COMPENSATION 0           // 1: 开启转矩补偿, 0: 关闭
#define TQ_COMP_GAIN           FRAC16(1.0) // iq_Amp 与 iq_ref 的比例系数 (需调试)
#define TQ_COMP_LUT_SIZE       5           // 补偿角度查表大小

// 转矩补偿查表结构
typedef struct {
    Frac16_t f16Speed;      // 转速 (绝对值, 归一化)
    Frac16_t f16AngleComp;  // 补偿角度 theta_q_comp
} TqCompLut_t;

// 补偿角度表 (示例值，需根据实际电机特性调试)
// f16Speed: 0~1.0 对应 0~最大转速
// f16AngleComp: 对应补偿的相位角
static const TqCompLut_t g_tTqCompLut[TQ_COMP_LUT_SIZE] = {
    {FRAC16(0.05), FRAC16(0.0)},   // 低速
    {FRAC16(0.10), FRAC16(0.1)},
    {FRAC16(0.20), FRAC16(0.2)},
    {FRAC16(0.50), FRAC16(0.15)},
    {FRAC16(1.00), FRAC16(0.0)}    // 高速
};

Frac16_t g_f16IqCompVal = 0; // 用于Debug观测计算出的补偿值

#if (ENABLE_DAMPING_COMPENSATION == 1)
    static Frac16_t f16EdFiltered = 0;
    static Frac16_t f16EdVibration = 0;
#endif

// WM_Obsvr_PLL_t tWmObsvr_PLL;
// VariablePI_Ctrl_t tVariableSpeedCtrl;
CTD_3SystF16_t tIabcFbckMVM;
CTD_LUT_SVM_t spwmVM;
Frac16_t f16DcCurLimit = FRAC16(32.0f/62.5);
FOC_Driver_t tDrvFoc;
#if (ENABLE_CURRENT_POWER_LIMIT == 1)
IQ_Limit_Ctrl_t tIqLimitCtrl;
#endif
MotorCalculations tMotorCalculations;
CTD_3SystF16_t tPwmDutyBak;



Frac32_t delta_uq = 0;
Frac32_t delta_ud = 0;
uint8_t u8EnableOut = 0;
uint16_t PWMPeriod = FOC_EPWM_PERIOD;
uint16_t g_u16SampleSlot = 196;
volatile Frac32_t OL_SpeedRampInc = 0, CL_SpeedRampInc = 0, CL_SpeedRampDec = 0;
Frac16_t f16Uqreq = 2000;
// extern DeadTimeComp_t tDeadTimeComp;
struct tCurrSample1RPS tCurrSample1rpsBak;
void SvmActuate(FOC_Mod_t* ptPwmMod, const CTD_LUT_SVM_t *const ptSvm);
void SetDuty(void);
void SetAdcTrg(void);
void InitFocParas(void);
Frac16_t IQ_LimitCtrl_Process(IQ_Limit_Ctrl_t* pCtrl,Frac16_t f16IdcCurrent,Frac16_t f16PowerCurrent, Frac16_t f16IqInput);
static bool FocFastLoop();
static bool FocSlowLoop();
static void ReconstructPhaseCurrent(void);
static __inline void GetAdcRaw(void);
void EnableOutput();
void DisableOutput(void);
Frac16_t f16IQFiltered = 0;
Frac16_t f16LastIQ = 0;
IQ_LP_FILTER g_tIqFbackLpFilter;
IQ_LP_FILTER g_tIdFbackLpFilter;
// ============ 高通滤波系数配置（用于阻尼补偿）===========
IQ_HP_FILTER g_tEdampHpFilter;
// IQ_HP_FILTER g_tEdampHpFilter2;
// IQ_LP_FILTER g_tIFSpeedFilter;
// //速度微分的低通滤波
// IQ_LP_FILTER g_tOmegaDiffLpf;
// ============= 高通滤波系数配置(用于PR控制的电流误差滤波)
static tqprControllerFxp g_qpr_speed;
static tqprControllerFxp g_qpr3_speed;
static tqprControllerFxp g_qpr_iq;
static tqprControllerFxp g_qpr_id;
// IQ_HP_FILTER g_tIqErrHpFilter;
Frac16_t DeadTimeCompensation(uint16_t Tdead,uint16_t Tpwm,Frac16_t Udc,Frac16_t I,Frac16_t Vce,Frac16_t Vd,Frac16_t Idq)
{

    int8_t signI = (I > 0) ? 1: ((I<0) ? -1 : 0);
    // Frac16_t deadZoneComp = ((int32_t)Tdead * Udc / Tpwm) * signI;
    Frac16_t deadZoneComp = MulF16(FRAC16(0.1) , Udc) * signI;

    Frac16_t switchComp = (Vce - Vd) * signI;
    if(f16IQFiltered == 0) {
        // 首次运行时直接赋值
        f16IQFiltered= Idq;
    } else {
        // 应用IIR滤波公式 (3/4 * 旧值 + 1/4 * 新值)
        f16IQFiltered = (int32_t)((int32_t)f16IQFiltered * 3 + (int32_t)Idq) >> 2;
    }
    Frac16_t currRatio = MM_SatDivF16(f16IQFiltered,FRAC16(0.5));

    // Frac16_t angle_ratio = AbsF16(MM_SatDivF16(tDrvFoc.tPospeControl.f16ThetaRotEl,FRAC16(0.25)));

    return MulF16(MulF16((deadZoneComp + switchComp),currRatio),FRAC16(1.0));
    // return (deadZoneComp + switchComp);
}

CTD_3SystF16_t tPwmDuty;
Frac16_t A_COMP;
void ApllyDeadTimeompenssationForDuty()
{
    return;
    uint16_t Tead = 1;
    uint16_t Tpwm = 100;
    Frac16_t Udc = tDrvFoc.f16DcBusFilt;

    Frac16_t Vce = FRAC16(0.6/50);
    Frac16_t vd = FRAC16(0.02/50);
    //保留补偿前的，便于查看
    // tPwmDuty = tDrvFoc.tSvm.tPwmDuty;
    // A_COMP =;
    tDrvFoc.tSvm.tPwmDuty.f16Arg1 = SatAddF16(tDrvFoc.tSvm.tPwmDuty.f16Arg1,DeadTimeCompensation(Tead,Tpwm,Udc,tDrvFoc.tIabcFbck.f16Arg1,Vce,vd,tDrvFoc.tIDQFbck.f16Arg2)); 
    tDrvFoc.tSvm.tPwmDuty.f16Arg2 = SatAddF16(tDrvFoc.tSvm.tPwmDuty.f16Arg2,DeadTimeCompensation(Tead,Tpwm,Udc,tDrvFoc.tIabcFbck.f16Arg2,Vce,vd,tDrvFoc.tIDQFbck.f16Arg2)); 
    tDrvFoc.tSvm.tPwmDuty.f16Arg3 = SatAddF16(tDrvFoc.tSvm.tPwmDuty.f16Arg3,DeadTimeCompensation(Tead,Tpwm,Udc,tDrvFoc.tIabcFbck.f16Arg3,Vce,vd,tDrvFoc.tIDQFbck.f16Arg2));  
 
}


uint32_t fasttime = 0;
uint8_t errorFlag = 0;

void SetPwmDutyAIWIN(struct tCurrSample1RPS* p)
{

    if(u8EnableOut == 0) return;
    int16_t i16DutyH,i16DutyL;
    // p->UVWCurrRecon.IuOffset = 0;
    // p->UVWCurrRecon.IvOffset = 0;
    // p->UVWCurrRecon.IwOffset = 0;
    /* set pwm compare value */
    EPWM->PWMRWEN_b.PWMRLDEN = 0x55; // enable write register
    //EPWM->PWM0DH             = PMSM_FOC_Y.tAout;
    //EPWM->PWM0DL             = FOC_EPWM_PERIOD+1;
    //EPWM->PWM1DH             = PMSM_FOC_Y.tBout;
    //EPWM->PWM1DL             = FOC_EPWM_PERIOD+1;
    //EPWM->PWM2DH             = PMSM_FOC_Y.tCout;
    //EPWM->PWM2DL             = FOC_EPWM_PERIOD+1;

    //Set PWM0 Duty
    // p->Signal.DtyPhaseU = PWMPeriod;
    // p->Signal.DtyPhaseU = PWMPeriod;
    p->Signal.DtyPhaseU = PWMPeriod - p->Signal.DtyPhaseU;
    
    i16DutyH = p->Signal.DtyPhaseU - p->UVWCurrRecon.IuOffset;
    i16DutyL = p->Signal.DtyPhaseU + p->UVWCurrRecon.IuOffset;
    // p->Signal.DtyPhaseU = 0;
    // p->Signal.DtyPhaseV = 
    if(p->Signal.DtyPhaseU <= 1){
        EPWM->PMANUALCON1_b.PMANUAL0H = 1;
        EPWM->PMANUALCON1_b.PMANUAL0L = 1;
        EPWM->PMANUALCON2_b.POUT0H = 1;
        EPWM->PMANUALCON2_b.POUT0L = 0;
    } 
    else if (p->Signal.DtyPhaseU >= (PWMPeriod - 1)){
        EPWM->PMANUALCON1_b.PMANUAL0H = 1;
        EPWM->PMANUALCON1_b.PMANUAL0L = 1;
        EPWM->PMANUALCON2_b.POUT0H = 0;
        EPWM->PMANUALCON2_b.POUT0L = 1;
    } 
    else{
        EPWM->PMANUALCON1_b.PMANUAL0H = 0;
        EPWM->PMANUALCON1_b.PMANUAL0L = 0;
        if(i16DutyH < 1)
        {
            EPWM->PWM0DH = 1;
        }
        else if(i16DutyH > (PWMPeriod - 1))
        {
            EPWM->PWM0DH = (PWMPeriod - 1);
        } else {
            EPWM->PWM0DH = i16DutyH;
        }

        if(i16DutyL < 1)
        {
            EPWM->PWM0DL = 1;
        }
        else if(i16DutyL > (PWMPeriod - 1))
        {
            EPWM->PWM0DL = (PWMPeriod - 1);
        } else {
            EPWM->PWM0DL = i16DutyL;
        }
    }
   


    //Set PWM1 Duty
    p->Signal.DtyPhaseV = PWMPeriod - p->Signal.DtyPhaseV;
    i16DutyH = p->Signal.DtyPhaseV - p->UVWCurrRecon.IvOffset;
    i16DutyL = p->Signal.DtyPhaseV + p->UVWCurrRecon.IvOffset;

    if(p->Signal.DtyPhaseV <= 1){
        EPWM->PMANUALCON1_b.PMANUAL1H = 1;
        EPWM->PMANUALCON1_b.PMANUAL1L = 1;
        EPWM->PMANUALCON2_b.POUT1H = 1;
        EPWM->PMANUALCON2_b.POUT1L = 0;
    } 
    else if (p->Signal.DtyPhaseV >= (PWMPeriod - 1)){
        EPWM->PMANUALCON1_b.PMANUAL1H = 1;
        EPWM->PMANUALCON1_b.PMANUAL1L = 1;
        EPWM->PMANUALCON2_b.POUT1H = 0;
        EPWM->PMANUALCON2_b.POUT1L = 1;
    } 
    else{
        EPWM->PMANUALCON1_b.PMANUAL1H = 0;
        EPWM->PMANUALCON1_b.PMANUAL1L = 0;
        if(i16DutyH < 1)
        {
            EPWM->PWM1DH = 1;
        }
        else if(i16DutyH > (PWMPeriod - 1))
        {
            EPWM->PWM1DH = (PWMPeriod - 1);
        } else {
            EPWM->PWM1DH = i16DutyH;
        }

        if(i16DutyL < 1)
        {
            EPWM->PWM1DL = 1;
        }
        else if(i16DutyL > (PWMPeriod - 1))
        {
            EPWM->PWM1DL = (PWMPeriod - 1);
        } else {
            EPWM->PWM1DL = i16DutyL;
        }
    }
  

    //Set PWM2 Duty
    p->Signal.DtyPhaseW = PWMPeriod - p->Signal.DtyPhaseW;
    i16DutyH = p->Signal.DtyPhaseW - p->UVWCurrRecon.IwOffset;
    i16DutyL = p->Signal.DtyPhaseW + p->UVWCurrRecon.IwOffset;

    if(p->Signal.DtyPhaseW <= 1){
        EPWM->PMANUALCON1_b.PMANUAL2H = 1;
        EPWM->PMANUALCON1_b.PMANUAL2L = 1;
        EPWM->PMANUALCON2_b.POUT2H = 1;
        EPWM->PMANUALCON2_b.POUT2L = 0;
    } 
    else if (p->Signal.DtyPhaseW >= (PWMPeriod - 1)){
        EPWM->PMANUALCON1_b.PMANUAL2H = 1;
        EPWM->PMANUALCON1_b.PMANUAL2L = 1;
        EPWM->PMANUALCON2_b.POUT2H = 0;
        EPWM->PMANUALCON2_b.POUT2L = 1;
    } 
    else{
        EPWM->PMANUALCON1_b.PMANUAL2H = 0;
        EPWM->PMANUALCON1_b.PMANUAL2L = 0;
        if(i16DutyH < 1)
        {
            EPWM->PWM2DH = 1;
        }
        else if(i16DutyH > (PWMPeriod - 1))
        {
            EPWM->PWM2DH = (PWMPeriod - 1);
        } else {
            EPWM->PWM2DH = i16DutyH;
        }
        
        if(i16DutyL < 1)
        {
            EPWM->PWM2DL = 1;
        }
        else if(i16DutyL > (PWMPeriod - 1))
        {
            EPWM->PWM2DL = (PWMPeriod - 1);
        } else {
            EPWM->PWM2DL = i16DutyL;
        }
    }


    EPWM->PWMRWEN_b.PWMRLDEN = 0xAA; // enable reload register
}

// // --------------------- 测试：三角波与正反切换数据 ---------------------
// volatile Frac16_t g_f16TriWave = 0;          // Q15 三角波：-32768 .. 32767
// volatile uint8_t  g_u8TriIncreasing = 10;     // 1 上升，0 下降
// volatile Frac16_t g_f16TriStep = FRAC16(0.01); // 步进（约0.01），中断频率20kHz时约20ms周期
// volatile int8_t   g_i8TestDir = +1;          // 方向标志：+1 正，-1 反
// volatile int16_t g_u16TriDuty = 32767;          // 映射为 0..PWMPeriod 的占空比值
// // ---
static inline int16_t saturate_16(int32_t val) {
      if (val > 32767) return 32767;
      if (val < -32768) return -32768;
      return (int16_t)val;
}
void EPWM_IRQHandler(void)
{
    uint32_t isrStatus = EPWM->ISR;

    GetAdcRaw();  // get ADC值
    // tDrvFoc.f16DcBusFilt = MulF16( tDrvFoc.i16AdcRaw[3]*8, FRAC16(0.05)) + MulF16(tDrvFoc.f16DcBusFilt, FRAC16(0.95));//tDrvFoc.f16DcBusFilt = GCF_LpFilterMA_F16(tDrvFoc.i16AdcRaw[3]*8, &tDrvFoc.tDcBusFilter);
    tDrvFoc.f16DcBusFilt = GCF_LpFilterMA_F16(tDrvFoc.i16AdcRaw[3]*8, &tDrvFoc.tDcBusFilter);
    //NOTE:偏差校准
    tDrvFoc.f16DcBusFilt += FRAC16(BUS_VOLTAGE_OFFSET/V_FULL_SCALE);

    tHfiCtrl.f16DcBusFilt = tDrvFoc.f16DcBusFilt;

    
    if (isrStatus & EPWM_PWMZIF) {    
        // EnableOutput();
        uint32_t time_start = Timer_Start();
        QPR_FXP_TestLite_Step();
#if (USE_MEASURE_VECTOR == 1)
    // if (tDrvFoc.tAppState.tStatus >= RUN){
    //     g_u8MeasureVectorDebugEnable = 1;
        if (g_u8MeasureVectorFlag == MEASURE_VECTOR_PREPARE) {
            // GPIO->PDO_b.DO0 = ~GPIO->PDO_b.DO0;
            // MeasureVectorPeriod();  //在上一次计算的有效发波的实际发波的起始点，直接设置寄存器在下一个周期发出0矢量测量周期
            // return;
        } 
        else if (g_u8MeasureVectorFlag == MEASURE_VECTOR_RUNNING){
            // delay_us(5);

            g_u8MeasureVectorFlag = MEASURE_VECTOR_DONE; //这个时候处于测量周期的0起始点,不进行FOC计算
            RestoreFromMeasureVectorConfig();           //测量结束，恢复配置，等着下一个周期，也就是桂林的时候生效
            
            // FOC_EpwmInit();
            // NVIC_EnableIRQ(EPWM_IRQn);
            // EPWM->PWMCON2_b.DILDEN = 1;
            // tCurrSample1rps.Signal.DtyPhaseU = tPwmDutyBak.f16Arg1 * FOC_EPWM_PERIOD >> 15;
            // tCurrSample1rps.Signal.DtyPhaseV = tPwmDutyBak.f16Arg2 * FOC_EPWM_PERIOD >> 15;
            // tCurrSample1rps.Signal.DtyPhaseW = tPwmDutyBak.f16Arg3 * FOC_EPWM_PERIOD >> 15;
            // Svm1rpsTrig_AIWIN(&tCurrSample1rpsBak);
            SetPwmDutyAIWIN(&tCurrSample1rpsBak);
            EPWM->PWMRWEN_b.PWMRLDEN = 0xAA;
            tDrvFoc.u16PWMCnt ++;
            EPWM->ISR |= EPWM_PWMZIF;
            if (isrStatus & EPWM_PWMPIF) EPWM->ISR |= EPWM_PWMPIF;
     
            GPIO->PDO_b.DO0 = ~GPIO->PDO_b.DO0;
            return;
        } 
        else if (g_u8MeasureVectorFlag == MEASURE_VECTOR_DONE){
            // RestoreFromMeasureVectorConfig();           //测量周期结束了
            // RestoreFromMeasureVectorConfig();     
            GPIO->PDO_b.DO0 = ~GPIO->PDO_b.DO0;
            g_u8MeasureVectorFlag = MEASURE_VECTOR_CALCULATING;
        }
    // }
#endif //endif for USE_MEASURE_VECTOR
    // if (g_u8MeasureVectorFlag != MEASURE_VECTOR_RUNNING){

#if (FOC_EPWM_SHUNT_MODE == 0)
        GetPhaseCurr_1rps_AIWIN(&tCurrSample1rps,&tCurrAuxInfo);
        tDrvFoc.tIabcFbck.f16Arg1 = tCurrSample1rps.Signal.Iu;   //scaler to +-32767
        tDrvFoc.tIabcFbck.f16Arg2 = tCurrSample1rps.Signal.Iv;
        tDrvFoc.tIabcFbck.f16Arg3 = tCurrSample1rps.Signal.Iw;
#elif (FOC_EPWM_SHUNT_MODE == -1)
    
        // if (g_bPredictCurrentsPending) {
        //     PredictPhaseCurrents(&tCurrSample1rps);
        //     g_bPredictCurrentsPending = false;
        //     tDrvFoc.tIabcFbck.f16Arg1 = tCurrSample1rps.Signal.Iu;   //scaler to +-32767
        //     tDrvFoc.tIabcFbck.f16Arg2 = tCurrSample1rps.Signal.Iv;
        //     tDrvFoc.tIabcFbck.f16Arg3 = tCurrSample1rps.Signal.Iw;
        // } else 
        {
            tCurrSample1rps.Signal.Iu = -(tDrvFoc.i16AdcRaw[0] - tCurrSample1rps.Signal.IuOffset);
            tCurrSample1rps.Signal.Iv = -(tDrvFoc.i16AdcRaw[1] - tCurrSample1rps.Signal.IvOffset);
            tCurrSample1rps.Signal.Iw = -tCurrSample1rps.Signal.Iu - tCurrSample1rps.Signal.Iv;

            tDrvFoc.tIabcFbck.f16Arg1 =  saturate_16(((int32_t)tCurrSample1rps.Signal.Iu) << 4);
            tDrvFoc.tIabcFbck.f16Arg2 =  saturate_16(((int32_t)tCurrSample1rps.Signal.Iv) << 4);
            tDrvFoc.tIabcFbck.f16Arg3 =  saturate_16(((int32_t)tCurrSample1rps.Signal.Iw) << 4);
        }

     
       

#endif

        // tHfiCtrl.tIabcFbck = tDrvFoc.tIabcFbck;
        if (tHfiCtrl.bShouldSample)
        {
            tHfiCtrl.tIabcFbck.f16Arg1 = tDrvFoc.tIabcFbck.f16Arg1;
            tHfiCtrl.tIabcFbck.f16Arg2 = tDrvFoc.tIabcFbck.f16Arg2;
            tHfiCtrl.tIabcFbck.f16Arg3 = tDrvFoc.tIabcFbck.f16Arg3;
        }
               
        if (g_u8MeasureVectorFlag == MEASURE_VECTOR_CALCULATING)
        {
            tIabcFbckMVM = tDrvFoc.tIabcFbck;
            g_u8MeasureVectorFlag = MEASURE_VECTOR_IDLE;
        }
        
        PWMPeriod = FOC_EPWM_PERIOD;  //fix pwm frequency

        tCurrAuxInfo.Id  = tDrvFoc.tIDQFbck.f16Arg1;
        tCurrAuxInfo.Iq  = tDrvFoc.tIDQFbck.f16Arg2;
        tCurrAuxInfo.AngleElectrical = tDrvFoc.tPospeControl.f16ThetaRotEl;
        tCurrAuxInfo.SpeedRPM = tDrvFoc.tPospeControl.f16wRotE1;

        tMotorCalculations.ia = tDrvFoc.tIabcFbck.f16Arg1;
        tMotorCalculations.ib = tDrvFoc.tIabcFbck.f16Arg2;
        tMotorCalculations.ic = tDrvFoc.tIabcFbck.f16Arg3;

        tMotorCalculations.vdc = tDrvFoc.f16DcBusFilt; //tDrvFoc.tDcBusComp.f16ArgDcBusMsr;

        tMotorCalculations.i_alpha = tDrvFoc.tIAlBeFbck.f16Arg1;
        tMotorCalculations.i_beta = tDrvFoc.tIAlBeFbck.f16Arg2;

        // tMotorCalculations.f16Id = tDrvFoc.tIDQFbck.f16Arg1;
        // tMotorCalculations.f16Iq = tDrvFoc.tIDQFbck.f16Arg2;

        // tMotorCalculations.f16Ud = tDrvFoc.tUDQReq.f16Arg1;
        // tMotorCalculations.f16Uq = tDrvFoc.tUDQReq.f16Arg2;

        
        // tMotorCalculations.i_alpha = tDrvFoc.tIAlBeFbck.f16Arg1;
        // tMotorCalculations.i_beta = tDrvFoc.tIAlBeFbck.f16Arg2;


        // MM_PolarTrigono_t invTran_;
        // CTD_2SystF16_t tidq_;
        // MM_ThetaTransferTrigonoLut(tDrvFoc.tPospeControl.f16ThetaRotEl, &invTran_);
        tMotorCalculations.f16Id = tDrvFoc.tIDQFbck.f16Arg1;
        tMotorCalculations.f16Iq = tDrvFoc.tIDQFbck.f16Arg2;

        // CTD_2SystF16_t tUdq_;
        // GMC_ParkeF16(&tUdq_, &tDrvFoc.tUAlBeReqDCB, &invTran_);
        // tMotorCalculations.f16UdRate = MM_DivF16(tDrvFoc.tUDQReq.f16Arg1,tUdq_.f16Arg1);
        // tMotorCalculations.f16UqRate = MM_DivF16(tDrvFoc.tUDQReq.f16Arg2,tUdq_.f16Arg2);
        tMotorCalculations.f16Ud = tDrvFoc.tUDQReq.f16Arg1;//tDrvFoc.tUDQReq.f16Arg1;// tUdq_.f16Arg1;
        tMotorCalculations.f16Uq = tDrvFoc.tUDQReq.f16Arg2;//tDrvFoc.tUDQReq.f16Arg2;

        tMotorCalculations.wo = tDrvFoc.tPospeControl.f16wRotE1;  // 电气角速度
        
#if (ENABLE_PARAMETER_IDENTIFY == 1)
        if (tMotorParamId.bIdentifyActive) {
            if (tMotorParamId.state == PARAM_ID_IDLE || tMotorParamId.state == PARAM_ID_COMPLETE){
                InitFocParas();
                PARAM_ID_Start(&tMotorParamId);
                tDrvFoc.tDcBusComp.f16ModIndex    = FRAC16(0.866025403784439);
                DisableOutput();
                PARAM_ID_Process(&tMotorParamId);
            } else if(tMotorParamId.state == PARAM_ID_PWM_FREQ_SWITCH){
                    EnableOutput();
                    HFI_Enable(&tHfiCtrl, true);
                    HFI_ChangePwmFreq(HFI_PWM_FREQUENCY);
                    tMotorParamId.state = PARAM_ID_ALIGN;
            } else{
                PARAM_ID_Process(&tMotorParamId);
            }
     
        }
// else
        
#else
        {
            // 正常状态处理
            getCurrentMaxMin();  
            //GetCurrentAbsSum025();
            pStateFuncTable[tDrvFoc.tAppState.tEvent][tDrvFoc.tAppState.tStatus]();
        }
#endif

#if FOC_EPWM_SHUNT_MODE == 0
        // ApllyDeadTimeompensssationForDuty();
        ApllyDeadTimeompenssationForDuty();
        //---------------------------测试用-----------------
        // tDrvFoc.tSvm.tPwmDuty.f16Arg1 = FRAC16(0.55);
        // tDrvFoc.tSvm.tPwmDuty.f16Arg2 = FRAC16(0.3);
        // tDrvFoc.tSvm.tPwmDuty.f16Arg3 = FRAC16(0.7);        
        // tDrvFoc.tSvm.tPwmDuty.f16Arg1 = 32767;
        tCurrSample1rps.Signal.DtyPhaseU = (int32_t)tDrvFoc.tSvm.tPwmDuty.f16Arg1 * FOC_EPWM_PERIOD>>15;
        tCurrSample1rps.Signal.DtyPhaseV = (int32_t)tDrvFoc.tSvm.tPwmDuty.f16Arg2 * FOC_EPWM_PERIOD>>15;
        tCurrSample1rps.Signal.DtyPhaseW = (int32_t)tDrvFoc.tSvm.tPwmDuty.f16Arg3 * FOC_EPWM_PERIOD>>15;

#if (USE_MEASURE_VECTOR == 1)
        if (g_u8MeasureVectorFlag == MEASURE_VECTOR_PREPARE) {
            // tPwmDutyBak  = tDrvFoc.tSvm.tPwmDuty;
            Svm1rpsTrig_AIWIN(&tCurrSample1rps);
            tCurrSample1rpsBak = tCurrSample1rps;
            GPIO->PDO_b.DO0 = ~GPIO->PDO_b.DO0;
            MeasureVectorPeriod();  //在上一次计算的有效发波的实际发波的起始点，直接设置寄存器在下一个周期发出0矢量测量周期
        } else{
            Svm1rpsTrig_AIWIN(&tCurrSample1rps);
             if (g_u8MeasureVectorFlag == MEASURE_VECTOR_PREPARE){
            //     // GMC_DcBusCompF16(&tDrvFoc.tUAlBeReqDCB, &tDrvFoc.tUAlBeReq, &tDrvFoc.tDcBusComp);
            //     /* SVPWM */
            //     // tDrvFoc.tUAlBeReqDCB.f16Arg1 = SatAddF16(tDrvFoc.tUAlBeReqDCB.f16Arg1 , MulF16(tDrvFoc.tUAlBeReqDCB.f16Arg1,FRAC16(0.2)));
            //     // tDrvFoc.tUAlBeReqDCB.f16Arg2 = SatAddF16(tDrvFoc.tUAlBeReqDCB.f16Arg2 , MulF16(tDrvFoc.tUAlBeReqDCB.f16Arg2,FRAC16(0.2)));
            //     //     /* Q-axis current control*/
            //     // //补偿丢
                tDrvFoc.tUDQReq.f16Arg1 = SatAddF16(tDrvFoc.tUDQReq.f16Arg1 , MulF16(tDrvFoc.tUDQReq.f16Arg1,FRAC16(0.1)));
                tDrvFoc.tUDQReq.f16Arg2 = SatAddF16(tDrvFoc.tUDQReq.f16Arg2 , MulF16(tDrvFoc.tUDQReq.f16Arg2,FRAC16(0.1)));
  
                /* Inv-Park transformation*/
                GMC_InvParkF16(&tDrvFoc.tUAlBeReq, &tDrvFoc.tUDQReq, &tDrvFoc.tThetaTransform);
                
                tDrvFoc.tDcBusComp.f16ArgDcBusMsr = tDrvFoc.f16DcBusFilt;
                /* BUS voltage complement*/
                GMC_DcBusCompF16AIWIN(&tDrvFoc.tUAlBeReqDCB, &tDrvFoc.tUAlBeReq, &tDrvFoc.tDcBusComp);
                /* SVPWM */
                GMC_Svm(&tDrvFoc.tSvm, &tDrvFoc.tUAlBeReqDCB);
                tCurrSample1rps.Signal.DtyPhaseU = tDrvFoc.tSvm.tPwmDuty.f16Arg1 * FOC_EPWM_PERIOD>>15;
                tCurrSample1rps.Signal.DtyPhaseV = tDrvFoc.tSvm.tPwmDuty.f16Arg2 * FOC_EPWM_PERIOD>>15;
                tCurrSample1rps.Signal.DtyPhaseW = tDrvFoc.tSvm.tPwmDuty.f16Arg3 * FOC_EPWM_PERIOD>>15;
                tCurrSample1rps.UVWCurrRecon.IuOffset = 0;
                tCurrSample1rps.UVWCurrRecon.IvOffset = 0;
                tCurrSample1rps.UVWCurrRecon.IwOffset = 0;

                // GMC_Svm(&tDrvFoc.tSvm, &tDrvFoc.tUAlBeReqDCB);
                // tDrvFoc.tSvm.tPwmDuty.f16Arg1 = SatAddF16(tDrvFoc.tSvm.tPwmDuty.f16Arg1,FRAC16(0.1)); 
                // tDrvFoc.tSvm.tPwmDuty.f16Arg2 = SatAddF16(tDrvFoc.tSvm.tPwmDuty.f16Arg2,FRAC16(0.1)); 
                // tDrvFoc.tSvm.tPwmDuty.f16Arg3 = SatAddF16(tDrvFoc.tSvm.tPwmDuty.f16Arg3,FRAC16(0.1));  
 
             }
            SetPwmDutyAIWIN(&tCurrSample1rps);
        }
#else
        Svm1rpsTrig_AIWIN(&tCurrSample1rps,&tCurrAuxInfo);
        // Svm1rpsTrig(&tCurrSample1rps);
        ADC->QTR3_b.QTTR = tCurrSample1rps.Signal.TP2 + 48;
        SetPwmDutyAIWIN(&tCurrSample1rps);
        // SetPwmDuty(&tCurrSample1rps);
#endif

        // DUTYA =  (FOC_EPWM_PERIOD - EPWM->PWM0DH) + (FOC_EPWM_PERIOD - EPWM->PWM0DL);
        // DUTYB =  (FOC_EPWM_PERIOD - EPWM->PWM1DH) + (FOC_EPWM_PERIOD - EPWM->PWM1DL);
        // DUTYC =  (FOC_EPWM_PERIOD - EPWM->PWM2DH) + (FOC_EPWM_PERIOD - EPWM->PWM2DL);
#elif (FOC_EPWM_SHUNT_MODE == -1)
        // // 定义限幅上下限（Q15格式），比如0.05~0.95
        // #define PWM_DUTY_MIN_Q15  (int16_t)(0.05 * 32767)
        // #define PWM_DUTY_MAX_Q15   (int16_t)(0.95 * 32767)


        // int16_t duty1 = tDrvFoc.tSvm.tPwmDuty.f16Arg1;
        // int16_t duty2 = tDrvFoc.tSvm.tPwmDuty.f16Arg2;
        // int16_t duty3 = tDrvFoc.tSvm.tPwmDuty.f16Arg3;

        // bool bDutyOutOfRange =
        //     ((duty1 > PWM_DUTY_MAX_Q15) ||
        //     (duty2 >PWM_DUTY_MAX_Q15 ));

        // if (bDutyOutOfRange) {
        //     g_bPredictCurrentsPending = true;
        // }

        // tCurrSample1rps.Signal.DtyPhaseU = ((int32_t)duty1 * FOC_EPWM_PERIOD) >> 15;
        // tCurrSample1rps.Signal.DtyPhaseV = ((int32_t)duty2 * FOC_EPWM_PERIOD) >> 15;
        // tCurrSample1rps.Signal.DtyPhaseW = ((int32_t)duty3 * FOC_EPWM_PERIOD) >> 15;
        // // tCurrSample1rps.Signal.DtyPhaseU = (int32_t)tDrvFoc.tSvm.tPwmDuty.f16Arg1 * FOC_EPWM_PERIOD>>15;
        // // tCurrSample1rps.Signal.DtyPhaseV = (int32_t)tDrvFoc.tSvm.tPwmDuty.f16Arg2 * FOC_EPWM_PERIOD>>15;
        // // tCurrSample1rps.Signal.DtyPhaseW = (int32_t)tDrvFoc.tSvm.tPwmDuty.f16Arg3 * FOC_EPWM_PERIOD>>15;
        // tCurrSample1rps.UVWCurrRecon.IuOffset = 0;
        // tCurrSample1rps.UVWCurrRecon.IvOffset = 0;
        // tCurrSample1rps.UVWCurrRecon.IwOffset = 0;
        // SetPwmDutyAIWIN(&tCurrSample1rps);
        // 定义限幅上下限（Q15格式），比如0.05~0.95
        #define PWM_DUTY_MIN_Q15  (int16_t)(0.02 * 32767)
        #define PWM_DUTY_MAX_Q15  (int16_t)(0.98 * 32767)

        // 对Duty进行限幅，防止过大或过小
        int16_t duty1 = tDrvFoc.tSvm.tPwmDuty.f16Arg1;
        int16_t duty2 = tDrvFoc.tSvm.tPwmDuty.f16Arg2;
        int16_t duty3 = tDrvFoc.tSvm.tPwmDuty.f16Arg3;

        // if (duty1 < PWM_DUTY_MIN_Q15) duty1 = PWM_DUTY_MIN_Q15;
        if (duty1 > PWM_DUTY_MAX_Q15) duty1 = PWM_DUTY_MAX_Q15;
        // if (duty2 < PWM_DUTY_MIN_Q15) duty2 = PWM_DUTY_MIN_Q15;
        if (duty2 > PWM_DUTY_MAX_Q15) duty2 = PWM_DUTY_MAX_Q15;
        // if (duty3 < PWM_DUTY_MIN_Q15) duty3 = PWM_DUTY_MIN_Q15;
        // if (duty3 > PWM_DUTY_MAX_Q15) duty3 = PWM_DUTY_MAX_Q15;

        tCurrSample1rps.Signal.DtyPhaseU = ((int32_t)duty1 * FOC_EPWM_PERIOD) >> 15;
        tCurrSample1rps.Signal.DtyPhaseV = ((int32_t)duty2 * FOC_EPWM_PERIOD) >> 15;
        tCurrSample1rps.Signal.DtyPhaseW = ((int32_t)duty3 * FOC_EPWM_PERIOD) >> 15;
        // tCurrSample1rps.Signal.DtyPhaseU = (int32_t)tDrvFoc.tSvm.tPwmDuty.f16Arg1 * FOC_EPWM_PERIOD>>15;
        // tCurrSample1rps.Signal.DtyPhaseV = (int32_t)tDrvFoc.tSvm.tPwmDuty.f16Arg2 * FOC_EPWM_PERIOD>>15;
        // tCurrSample1rps.Signal.DtyPhaseW = (int32_t)tDrvFoc.tSvm.tPwmDuty.f16Arg3 * FOC_EPWM_PERIOD>>15;
        tCurrSample1rps.UVWCurrRecon.IuOffset = 0;
        tCurrSample1rps.UVWCurrRecon.IvOffset = 0;
        tCurrSample1rps.UVWCurrRecon.IwOffset = 0;
        SetPwmDutyAIWIN(&tCurrSample1rps);

#endif
    // }
        //GPIO->PDO_b.DO0 = 0;
        tDrvFoc.u16PWMCnt ++;
        EPWM->ISR |= EPWM_PWMZIF;
        fasttime = Timer_GetFASTTime(time_start);
    }

    if (isrStatus & EPWM_PWMPIF) {
        EPWM->ISR |= EPWM_PWMPIF;
    }


    
}

/************************************************************
 * @brief: ADC interrupt handler
 * @return <None>
 ************************************************************/
void ADC_IRQHandler(void)
{
    uint32_t isrStatus = ADC->ISR;

    if (isrStatus & ADC_EOC) {
        ADC->ISR = ADC_EOC;
    }
}

/************************************************************
 * @brief: Disable EPWM output
 * @return <None>
 ************************************************************/
void BrakeOutput(void)
{
    /* @todo: disable GDU output*/
    u8EnableOut = 0;
    EPWM->PWMRWEN_b.PWMRLDEN = 0x55; // enable write register
    EPWM->PMANUALCON2        = 0x07; // manual output xH:0 xL:1
    EPWM->PMANUALCON1        = 0x3F; // output controled by manual
    EPWM->PWMRWEN_b.PWMRLDEN = 0xAA; // enable write register
}

void DisableOutput(void)
{

    u8EnableOut = 0;
    // return;
    /* @todo: disable GDU output*/
    EPWM->PWMRWEN_b.PWMRLDEN = 0x55; // enable write register
    EPWM->PMANUALCON2        = 0x00; // manual output xH:0 xL:1
    EPWM->PMANUALCON1        = 0x3F; // output controled by manual
    EPWM->PWMRWEN_b.PWMRLDEN = 0xAA; // enable write register
}

/************************************************************
 * @brief: Enable EPWM output
 * @return <None>
 ************************************************************/
void EnableOutput()
{
    // return;
    u8EnableOut = 1;
    /* @todo: enable GDU output*/
    EPWM->PWMRWEN_b.PWMRLDEN = 0x55; // enable write register
    EPWM->PMANUALCON2        = 0x07; // manual output xH:0 xL:1
    EPWM->PMANUALCON1        = 0x00; // output controled by PWM generator
    EPWM->PWMRWEN_b.PWMRLDEN = 0xAA; // enable write register
}

void StateFault()
{
    bool bFnStatus            = false;
    tDrvFoc.tAppState.tStatus = FAULT;
    // BrakeOutput();
    DisableOutput();
    tDrvFoc.tSvm.tPwmDuty.f16Arg1 = FRAC16(0.60);
    tDrvFoc.tSvm.tPwmDuty.f16Arg2 = FRAC16(0.50);
    tDrvFoc.tSvm.tPwmDuty.f16Arg3 = FRAC16(0.50);
    spwmVM.tPwmDuty = tDrvFoc.tSvm.tPwmDuty;
    tDrvFoc.tPospeControl.f16wRotE1 = 0;
    tDrvFoc.f16Idcbus = 0;
    tDrvFoc.i1610Idcbus = 0;
    if (!bFnStatus) {
    } else {
        tDrvFoc.tAppState.tEvent = E_FAULT_CLEAR;
    }
}

void StateInit()
{
    bool initFcnStatus       = true;
    tDrvFoc.tAppState.tStatus = INIT;
    tDrvFoc.tAppState.tEvent = E_INIT;

    /* turn off Output*/
    DisableOutput();
    InitFocParas();
    tDrvFoc.tPospeControl.f16ThetaRotEl = 0;
    tDrvFoc.tPospeControl.f16wRotE1 = 0;
    tDrvFoc.tSvm.tPwmDuty.f16Arg1 = FRAC16(0.60);
    tDrvFoc.tSvm.tPwmDuty.f16Arg2 = FRAC16(0.50);
    tDrvFoc.tSvm.tPwmDuty.f16Arg3 = FRAC16(0.50);
    /* Initialize FOC parameters*/


    spwmVM.tPwmDuty = tDrvFoc.tSvm.tPwmDuty;

    /* initilize calibration parameters*/

#if (FOC_EPWM_SHUNT_MODE == 0)
    ADC->QTR1_b.QTTR             = 0;
    ADC->QTR2_b.QTTR             = 96;
#endif
//    ADC->QTR3_b.QTTR             = 96;
//    ADC->QTR4_b.QTTR             = 400;
    /* initilize adc raw data*/
    for (uint8_t i = 0; i < 8; i++) {
    }

    /* initilize Foc parameter*/

    /* default operation mode*/
    if (!initFcnStatus) {
        //@fixme: add fault
    } else {
        tDrvFoc.tAppState.tEvent = E_INIT_DONE;
    }
}

void StateReady()
{
    tDrvFoc.tAppState.tStatus = READY;
    tDrvFoc.tAppState.tEvent = E_READY;
    
    tDrvFoc.tAppState.tEvent = E_APP_ON;
}

//修改Q5采样为母线电流
void ChangeAdcQ5ToIbus(void)
{
    SYSCTRL->LKKEYR          = SYSCTRL_UNLOCK_KEY; // unlock sysctrl
    ADC->QCR1_b.Q5SEL = 9; // BEMF Voltage 后面需要复用与母线电流采样即A1即9公用

}

void ChangeAdcQ5ToBEMF(void)
{
    SYSCTRL->LKKEYR          = SYSCTRL_UNLOCK_KEY; // unlock sysctrl
    ADC->QCR1_b.Q5SEL = 4; // BEMF Voltage 后面需要复用与母线电流采样即A1即9公用

}

int16_t WMSw = 0;
void StateCalib()
{
    bool calibFcnStatus       = false;
    tDrvFoc.tAppState.tStatus = CALIB;
    tDrvFoc.tAppState.tEvent  = E_CALIB;
    ChangeAdcQ5ToBEMF();
    /* turn off output */
    DisableOutput();
    tDrvFoc.tSvm.tPwmDuty.f16Arg1 = FRAC16(0.60);
    tDrvFoc.tSvm.tPwmDuty.f16Arg2 = FRAC16(0.50);
    tDrvFoc.tSvm.tPwmDuty.f16Arg3 = FRAC16(0.50);
    spwmVM.tPwmDuty = tDrvFoc.tSvm.tPwmDuty;
    if (tDrvFoc.u16CalibCntr++ < 16) {
#if FOC_EPWM_SHUNT_MODE == 0
        tDrvFoc.i32CalibSum += tDrvFoc.i16AdcRaw[0];
        tDrvFoc.i32CalibSum += tDrvFoc.i16AdcRaw[1];
#elif (FOC_EPWM_SHUNT_MODE == -1)
        tDrvFoc.i32CalibSum += tDrvFoc.i16AdcRaw[0];
        tDrvFoc.i32CalibSum2 += tDrvFoc.i16AdcRaw[1];
#endif
    } else {
        tDrvFoc.u16CalibCntr = 16;
#if FOC_EPWM_SHUNT_MODE == 0
        tCurrSample1rps.Signal.IuOffset = tDrvFoc.i32CalibSum / 32;
#elif (FOC_EPWM_SHUNT_MODE == -1)
        tCurrSample1rps.Signal.IuOffset = tDrvFoc.i32CalibSum / 16;
        tCurrSample1rps.Signal.IvOffset = tDrvFoc.i32CalibSum2 / 16;
#endif 
        calibFcnStatus            = true;
    }
//    WMSw = 0;
    if (calibFcnStatus) {
        tDrvFoc.tAppState.tEvent = E_CALIB_DONE;
    }
}

// void StateAlign()
// {
//     bool bTransitionToNextState = false;

//     /* entering state */
//     tDrvFoc.tAppState.tStatus = ALIGN;
//     tDrvFoc.tAppState.tEvent = E_ALIGN;
// #if (ENABLE_VF_DEBUG == 1)
//     EnableOutput();
//     tDrvFoc.tAppState.tEvent = E_ALIGN_DONE;
//     tDrvFoc.tPosMode = FORCE;
//     return;
// #endif

//     tDrvFoc.u16AlignCntr++;

//     // --- 1. 风车检测阶段 (前5000个周期) ---
//     if (tDrvFoc.u16AlignCntr < tFocParas.ALIGN_WIND_DETECT_TIME) {
//         SmartAlgnInit();
//         DisableOutput();

//         // BEMF采样以检测风车
//         if(GDU->GDUMUX_b.GPMUX == 0) {
//             tWmObsvr_PLL.tUabcBemf.f16Arg1 = tDrvFoc.i16AdcRaw[4];
//             GDU->GDUMUX_b.GPMUX = 1;
//         } else if(GDU->GDUMUX_b.GPMUX == 1) {
//             tWmObsvr_PLL.tUabcBemf.f16Arg2 = tDrvFoc.i16AdcRaw[4];
//             GDU->GDUMUX_b.GPMUX = 2;
//         } else if(GDU->GDUMUX_b.GPMUX == 2) {
//             tWmObsvr_PLL.tUabcBemf.f16Arg3 = tDrvFoc.i16AdcRaw[4];
//             GDU->GDUMUX_b.GPMUX = 0;
//         }
//         // WM_WindDetection(&tDrvFoc.tWMObsvr);
//         WM_WindDetectionPll(&tDrvFoc.tUDQReq,&tWmObsvr_PLL,&tFocParas);
//         if (headwind_machine_speed_flt >=FRAC16(100.0/MAX_RPM)) {
//             tWmObsvr_PLL.tWmDir = HEADWIND;
//         } else if (headwind_machine_speed_flt <=FRAC16(-100.0/MAX_RPM)) {
//             tWmObsvr_PLL.tWmDir = TAILWIND;
//         } else {
//             tWmObsvr_PLL.tWmDir = NOWIND;
//         }
//         //如果转速指令小于0，对TAILWIND和HEADWIND进行交换处理
//         if (tDrvFoc.tPospeControl.f16wRotElReq < FRAC16(0)) {
//             tWmObsvr_PLL.tWmDir = (tWmObsvr_PLL.tWmDir == TAILWIND) ? HEADWIND : 
//                                      (tWmObsvr_PLL.tWmDir == HEADWIND) ? TAILWIND : NOWIND;
//         }
//         return; // 在检测阶段，不执行后续逻辑，等待检测结果稳定
//     }

//     // --- 2. 根据风车检测结果进行处理 ---
//     switch (tWmObsvr_PLL.tWmDir)
//     {



//         case TAILWIND: // 顺风处理
//         // void WM_SwitchSmoothpll(GCF_CtrlPIAW_RTF16_t *tAxisDCtrl,GCF_CtrlPIAW_RTF16_t *tAxisQCtrl,GCF_CtrlPIAW_PTF16_t *tSpeedCtrl,
//         //                 GCF_Ramp_F16_t *tSpeedRamp, WM_Obsvr_PLL_t* ptWmObsvr,AMC_BemfcObsvrDQ_t *tBemfObsvr,CTD_2SystF16_t *tIAlBeFbck,
//         //                 const FOC_Paras_t* ptFocParas);
//             WM_SwitchSmoothpll(&tDrvFoc.tAxisDCtrl,
//                               &tDrvFoc.tAxisQCtrl,
//                               &tDrvFoc.tSpeedCtrl,
//                               &tDrvFoc.tSpeedRamp,
//                               &tWmObsvr_PLL,
//                               &tDrvFoc.tPospeSensorless.tBemfObsvr,
//                               &tDrvFoc.tIAlBeFbck,
//                               &tFocParas);
//             tDrvFoc.tPospeSensorless.f16wRotE1 = tDrvFoc.tWMObsvr.f16Speed;
//             tDrvFoc.tPospeSensorless.f16ThetaRotEl = tDrvFoc.tWMObsvr.f16Theta;
//             tDrvFoc.tPospeSensorless.tBemfObsvr.tGammaPIEmfCtrl.f32Acc = -((int32_t)tWmObsvr_PLL.f16AmpSqure << 15);
//             tDrvFoc.tPospeSensorless.tBemfObsvr.ptEObsrv.f16Arg2 = -tDrvFoc.tWMObsvr.f16AmpSqure;
   

//             if (tDrvFoc.tWMObsvr.f16Speed >= tDrvFoc.tPospeSensorless.f16wRotElMatch_2) {
//                 tDrvFoc.tPosMode = SENSORLESS;
//                 // CL_SpeedRampInc = CL_SpeedRampDec;
//                 FocSlowLoop();
//                 EnableOutput();
//             } else {
// #if (ENABLE_HFI == 1)
//                 HFI_Init(&tHfiCtrl, &tHfiConfig);
//                 WIND_SwitchToHFI(&tHfiCtrl);
//                 tDrvFoc.tPosMode = HFI_MODE;
//                 EnableOutput();
// #else
//                 tDrvFoc.tPospeOpenloop.f16ThetaRotEl = tDrvFoc.tWMObsvr.f16Theta;
//                 tDrvFoc.tPospeOpenloop.tOplpInteg.f32State = tDrvFoc.tPospeSensorless.tBemfObsvr.tTrackObsvr.tPllInte.f32State;
//                 tDrvFoc.tPospeOpenloop.f16wRotE1 = tDrvFoc.tSpeedRamp.f16State;
//                 tDrvFoc.u32PreSensorlessTime = PERSENSORLESS_WIND_TIME_MS;
//                 g_tEdampHpFilter.lp.prein = tDrvFoc.tPospeSensorless.f16wRotE1;
//                 EnableOutput();
// #endif
//             }
//             bTransitionToNextState = true;
//             break;

//         case HEADWIND: // 逆风处理
// #if (ENABLE_HFI == 1)
//             tDrvFoc.tPospeSensorless.f16wRotE1 = tDrvFoc.tWMObsvr.f16Speed;
//             tDrvFoc.tPospeSensorless.f16ThetaRotEl = tDrvFoc.tWMObsvr.f16Theta;
//             HFI_Init(&tHfiCtrl, &tHfiConfig);
//             WIND_SwitchToHFI(&tHfiCtrl);
//             tDrvFoc.tPosMode = HFI_MODE;
//             EnableOutput();
//             bTransitionToNextState = true;
// #else
//             // 刹车直到电机静止
//             if (tDrvFoc.u16AlignCntr < tFocParas.ALIGN_BRAKE_TIME) { // 给予一定的刹车时间
//                 BrakeOutput();
//                 WM_Init(&tDrvFoc.tWMObsvr, tDrvFoc.i16AdcRaw[4]); // 重置风车检测
//             } else {
//                 tDrvFoc.u16AlignCntr = 0; // 重新开始对齐流程
//             }
// #endif
//             break;

//         case NOWIND: // 无风处理
// #if (ENABLE_HFI == 1)
//             // 如果启用HFI，直接初始化并切换到HFI模式
//             HFI_Init(&tHfiCtrl, &tHfiConfig);
//             tDrvFoc.tPosMode = HFI_MODE;
//             bTransitionToNextState = true;
// #else
//             // 标准FOC：先刹车，再对齐
//             if (tDrvFoc.u16AlignCntr < (tFocParas.ALIGN_WIND_DETECT_TIME + tFocParas.ALIGN_BRAKE_TIME)) {
//                 // 阶段1: 刹车 (确保电机完全静止)
//                 BrakeOutput();
//             } else if (tDrvFoc.u16AlignCntr < (tFocParas.ALIGN_WIND_DETECT_TIME + tFocParas.ALIGN_BRAKE_TIME + tFocParas.ALIGN_DURATION)) {
//                 // 阶段2: 执行转子定位对齐
//                 EnableOutput();
//                 tDrvFoc.tPospeControl.f16ThetaRotEl = 0;
//                 tDrvFoc.tIDQReq.f16Arg1 = tFocParas.ALIGN_ID_REF; // 使用参数化的对齐电流/电压
//                 tDrvFoc.tIDQReq.f16Arg2 = FRAC16(0.0);
//                 FocFastLoop();
//             } else {
//                 // 阶段3: 对齐完成
//                 tDrvFoc.tPosMode = FORCE;
//                 bTransitionToNextState = true;
//             }
// #endif
//             break;
//     }

//     /* align stage finished, transition to next state */
//     if (bTransitionToNextState) {
// #if (ENABLE_DIRECT_CLOSED_LOOP_START == 1)
//         tDrvFoc.tPosMode = SENSORLESS;
// #endif
//         ChangeAdcQ5ToIbus();
//         ePrevPosModeSlowLoop = tDrvFoc.tPosMode;
//         tDrvFoc.tAppState.tEvent = E_ALIGN_DONE;
//     }
// }

void StateAlign()
{
    bool bTransitionToNextState = false;

    /* entering state */
    tDrvFoc.tAppState.tStatus = ALIGN;
    tDrvFoc.tAppState.tEvent = E_ALIGN;
#if (ENABLE_VF_DEBUG == 1)
    EnableOutput();
    tDrvFoc.tAppState.tEvent = E_ALIGN_DONE;
    tDrvFoc.tPosMode = FORCE;
    return;
#endif

    tDrvFoc.u16AlignCntr++;

    // --- 1. 风车检测阶段 (前5000个周期) ---
    if (tDrvFoc.u16AlignCntr < tFocParas.ALIGN_WIND_DETECT_TIME) {
        SmartAlgnInit();
        DisableOutput();

        // BEMF采样以检测风车
        if(GDU->GDUMUX_b.GPMUX == 0) {
            tDrvFoc.tWMObsvr.tUabcBemf.f16Arg1 = tDrvFoc.i16AdcRaw[4];
            GDU->GDUMUX_b.GPMUX = 1;
        } else if(GDU->GDUMUX_b.GPMUX == 1) {
            tDrvFoc.tWMObsvr.tUabcBemf.f16Arg2 = tDrvFoc.i16AdcRaw[4];
            GDU->GDUMUX_b.GPMUX = 2;
        } else if(GDU->GDUMUX_b.GPMUX == 2) {
            tDrvFoc.tWMObsvr.tUabcBemf.f16Arg3 = tDrvFoc.i16AdcRaw[4];
            GDU->GDUMUX_b.GPMUX = 0;
        }
        WM_WindDetection(&tDrvFoc.tWMObsvr);
          //如果转速指令小于0，对TAILWIND和HEADWIND进行交换处理

        return; // 在检测阶段，不执行后续逻辑，等待检测结果稳定
    } 
    if (tDrvFoc.u16AlignCntr == tFocParas.ALIGN_WIND_DETECT_TIME) {
        if (tDrvFoc.tPospeControl.f16wRotElReq < FRAC16(0)) {
                tDrvFoc.tWMObsvr.tWmDir = (tDrvFoc.tWMObsvr.tWmDir == TAILWIND) ? HEADWIND : 
                                        (tDrvFoc.tWMObsvr.tWmDir == HEADWIND) ? TAILWIND : NOWIND;
            }
    }

    // --- 2. 根据风车检测结果进行处理 ---
    switch (tDrvFoc.tWMObsvr.tWmDir)
    {
        case TAILWIND: // 顺风处理
            // 刹车直到电机静止
            if (tDrvFoc.u16AlignCntr < tFocParas.ALIGN_BRAKE_TIME) { // 给予一定的刹车时间
                BrakeOutput();
                WM_Init(&tDrvFoc.tWMObsvr, tDrvFoc.i16AdcRaw[4]); // 重置风车检测
            } else {
                tDrvFoc.u16AlignCntr = 0; // 重新开始对齐流程
            }
            break;

        case HEADWIND: // 逆风处理
#if (ENABLE_HFI == 1)
            tDrvFoc.tPospeSensorless.f16wRotE1 = tDrvFoc.tWMObsvr.f16Speed;
            tDrvFoc.tPospeSensorless.f16ThetaRotEl = tDrvFoc.tWMObsvr.f16Theta;
            HFI_Init(&tHfiCtrl, &tHfiConfig);
            WIND_SwitchToHFI(&tHfiCtrl);
            tDrvFoc.tPosMode = HFI_MODE;
            EnableOutput();
            bTransitionToNextState = true;
#else
            // 刹车直到电机静止
            if (tDrvFoc.u16AlignCntr < tFocParas.ALIGN_BRAKE_TIME) { // 给予一定的刹车时间
                BrakeOutput();
                WM_Init(&tDrvFoc.tWMObsvr, tDrvFoc.i16AdcRaw[4]); // 重置风车检测
            } else {
                tDrvFoc.u16AlignCntr = 0; // 重新开始对齐流程
            }
#endif
            break;

        case NOWIND: // 无风处理
#if (ENABLE_HFI == 1)
            // 如果启用HFI，直接初始化并切换到HFI模式
            HFI_Init(&tHfiCtrl, &tHfiConfig);
            tDrvFoc.tPosMode = HFI_MODE;
            bTransitionToNextState = true;
#else
            // 标准FOC：先刹车，再对齐
            if (tDrvFoc.u16AlignCntr < (tFocParas.ALIGN_WIND_DETECT_TIME + tFocParas.ALIGN_BRAKE_TIME)) {
                // 阶段1: 刹车 (确保电机完全静止)
                BrakeOutput();
            } else if (tDrvFoc.u16AlignCntr < (tFocParas.ALIGN_WIND_DETECT_TIME + tFocParas.ALIGN_BRAKE_TIME + tFocParas.ALIGN_DURATION)) {
                // 阶段2: 执行转子定位对齐
                EnableOutput();
                tDrvFoc.tPospeControl.f16ThetaRotEl = 0;
                tDrvFoc.tIDQReq.f16Arg1 = tFocParas.ALIGN_ID_REF; // 使用参数化的对齐电流/电压
                tDrvFoc.tIDQReq.f16Arg2 = FRAC16(0.0);
                FocFastLoop();
            } else {
                // 阶段3: 对齐完成
                tDrvFoc.tPosMode = FORCE;
                bTransitionToNextState = true;
            }
#endif
            break;
    }

    /* align stage finished, transition to next state */
    if (bTransitionToNextState) {
#if (ENABLE_DIRECT_CLOSED_LOOP_START == 1)
        tDrvFoc.tPosMode = SENSORLESS;
#endif
        ChangeAdcQ5ToIbus();
        ePrevPosModeSlowLoop = tDrvFoc.tPosMode;
        tDrvFoc.tAppState.tEvent = E_ALIGN_DONE;
    }
}

/************************************************************
 * @brief: Automatic mode for position mode.
 * @return <bool>
 ************************************************************/
int16_t i16ObsvrErr = 0;
Frac16_t f16ThetaErr = 0;
extern Frac16_t DELTA_ANGLE;
bool AutomaticMode()
{
#if (ENABLE_HFI == 1 && ENABLE_HFI_AUTO_SWITCH == 1)
    static uint16_t u16SwitchCounter = 0;
    
    // 获取当前转速的绝对值
    Frac16_t f16AbsSpeed = AbsF16(tDrvFoc.tPospeControl.f16wRotE1);
    
    if (tDrvFoc.tPosMode == HFI_MODE) {
        // 如果当前在HFI模式，检查是否需要切换到反电动势观测器
        if (f16AbsSpeed >= HFI_TO_BEMF_THRESHOLD) {
            u16SwitchCounter++;
            if (u16SwitchCounter >= HFI_SWITCH_STABLE_TIME) {
                // 转速足够高且持续稳定，切换到反电动势观测器
                HFI_SwitchToBemf(&tHfiCtrl);
                tDrvFoc.tPospeControl.i16SpeedLoopCntr = tFocParas.SPEED_LOOP_CNTR;
                u16SwitchCounter = 0;
                
                return true;
            }
        } else {
            u16SwitchCounter = 0; // 不满足切换条件，重置计数器
        }
        return true;
    } 
    else if (tDrvFoc.tPosMode == SENSORLESS) {
        // 如果当前在反电动势模式，检查是否需要切换回HFI
        if (f16AbsSpeed <= BEMF_TO_HFI_THRESHOLD) {
            u16SwitchCounter++;
            if (u16SwitchCounter >= HFI_SWITCH_STABLE_TIME) {
                // 转速足够低且持续稳定，切换到HFI观测器
                HFI_SwitchFromBemf(&tHfiCtrl);
                // HFI_Init(&tHfiCtrl, &tHfiConfig);
                u16SwitchCounter = 0;
                return true;
            }
        } else {
            u16SwitchCounter = 0; // 不满足切换条件，重置计数器
            return true;
        }
    }
    
    // 处理低速顺风切过来的时候时枪托的情况
    if (tDrvFoc.tPosMode == FORCE || tDrvFoc.tPosMode == TRACKING) {
        // 原有的模式切换逻辑
        if (AbsF16(tDrvFoc.tPospeOpenloop.f16wRotE1) >= tDrvFoc.tPospeSensorless.f16wRotElMatch_1 &&
                AbsF16(tDrvFoc.tPospeOpenloop.f16wRotE1) < tDrvFoc.tPospeSensorless.f16wRotElMatch_2) {
            tDrvFoc.tPosMode = TRACKING;
        } else {
            tDrvFoc.tPosMode = SENSORLESS;
        }
    }
#elif (ENABLE_DAMPING_COMPENSATION == 1)
   
    // 原有的模式切换逻辑
    if (AbsF16(tDrvFoc.tPospeOpenloop.f16wRotE1) < tDrvFoc.tPospeSensorless.f16wRotElMatch_1) {
        if(tDrvFoc.tPosMode != SENSORLESS && tDrvFoc.tPosMode != PRESENSORLESS) tDrvFoc.tPosMode = FORCE;
    } else if (AbsF16(tDrvFoc.tPospeOpenloop.f16wRotE1) >= tDrvFoc.tPospeSensorless.f16wRotElMatch_1 &&
              AbsF16(tDrvFoc.tPospeOpenloop.f16wRotE1) < tDrvFoc.tPospeSensorless.f16wRotElMatch_2) {
        if(tDrvFoc.tPosMode != SENSORLESS  && tDrvFoc.tPosMode != PRESENSORLESS) tDrvFoc.tPosMode = TRACKING;
    } else {
        // if (AbsF16(DELTA_ANGLE) >= 1000 && AbsF16(DELTA_ANGLE) < 30000)  tDrvFoc.tPosMode = SENSORLESS;
        // else tDrvFoc.tSpeedRamp.f16RampUp = 0;
         if (tDrvFoc.u32PreSensorlessCnt > tDrvFoc.u32PreSensorlessTime && tDrvFoc.tPosMode != SENSORLESS) {
            tDrvFoc.tPosMode = SENSORLESS;
         }  else {
            if (tDrvFoc.tPosMode != SENSORLESS) tDrvFoc.tPosMode = PRESENSORLESS;
         }
    }
#elif (ENABLE_DIRECT_CLOSED_LOOP_START == 1)
    tDrvFoc.tPosMode = SENSORLESS;
#else
    if (AbsF16(tDrvFoc.tPospeOpenloop.f16wRotE1) < tDrvFoc.tPospeSensorless.f16wRotElMatch_1) {
        if(tDrvFoc.tPosMode != SENSORLESS && tDrvFoc.tPosMode != PRESENSORLESS) tDrvFoc.tPosMode = FORCE;
    } else if (AbsF16(tDrvFoc.tPospeOpenloop.f16wRotE1) >= tDrvFoc.tPospeSensorless.f16wRotElMatch_1 &&
              AbsF16(tDrvFoc.tPospeOpenloop.f16wRotE1) < tDrvFoc.tPospeSensorless.f16wRotElMatch_2) {
        if(tDrvFoc.tPosMode != SENSORLESS  && tDrvFoc.tPosMode != PRESENSORLESS) tDrvFoc.tPosMode = TRACKING;
    } else {
        if (tDrvFoc.tPosMode != SENSORLESS) tDrvFoc.tPosMode = SENSORLESS;
    }
#endif

    return true;
}

/************************************************************
 * @brief: calculate elec. angle and velocity in open-loop controlling
 * @return <none>
 ************************************************************/
void CalcOpenLoop(FOC_Openloop_t *ptOpenLoop, Frac16_t f16SpeedReqRamp)
{
    ptOpenLoop->f16wRotE1     = f16SpeedReqRamp;
    ptOpenLoop->f16ThetaRotEl = GCF_IntegratorTR_F16_AIWIN(f16SpeedReqRamp, &ptOpenLoop->tOplpInteg);
}
int16_t f_mech;
static inline int16_t pr_get_center_freq_hz(void)
{
    int16_t f_elec;
    f_elec = (tDrvFoc.tPospeControl.f16wRotE1) * FREQUENCY_BASE_100_U16 / 32767 /100;
#if PR_USE_MECH_FREQ
    f_mech = (POLEPAIRS > 0) ? (f_elec / POLEPAIRS) : f_elec;
    return f_mech;
#else
    return f_elec;
#endif
}

Frac16_t f16ThetaMech;
// 简单的线性插值查表函数，用于获取补偿角度
static Frac16_t GetThetaCompFromSpeed(Frac16_t f16SpeedAbs)
{
    uint8_t i;
    // 边界处理
    if (f16SpeedAbs <= g_tTqCompLut[0].f16Speed) return g_tTqCompLut[0].f16AngleComp;
    if (f16SpeedAbs >= g_tTqCompLut[TQ_COMP_LUT_SIZE - 1].f16Speed) return g_tTqCompLut[TQ_COMP_LUT_SIZE - 1].f16AngleComp;

    for (i = 0; i < TQ_COMP_LUT_SIZE - 1; i++)
    {
        if (f16SpeedAbs >= g_tTqCompLut[i].f16Speed && f16SpeedAbs < g_tTqCompLut[i+1].f16Speed)
        {
            // 线性插值: y = y0 + (x - x0) * (y1 - y0) / (x1 - x0)
            Frac16_t x0 = g_tTqCompLut[i].f16Speed;
            Frac16_t x1 = g_tTqCompLut[i+1].f16Speed;
            Frac16_t y0 = g_tTqCompLut[i].f16AngleComp;
            Frac16_t y1 = g_tTqCompLut[i+1].f16AngleComp;
            
            // 避免除零
            if (x1 == x0) return y0;

            Frac16_t slope = MM_DivF16(SatSubF16(y1, y0), SatSubF16(x1, x0));
            Frac16_t deltaX = SatSubF16(f16SpeedAbs, x0);
            return SatAddF16(y0, MulF16(deltaX, slope));
        }
    }
    return 0;
}

// current loop
static bool FocFastLoop()
{

#if (ENABLE_VF_DEBUG == 1)

    Frac16_t f16VoltageAmp  = 0;
    MM_ThetaTransferTrigonoLut(tDrvFoc.tPospeControl.f16ThetaRotEl, &tDrvFoc.tThetaTransform);

    /* 计算VF曲线的电压幅值,与速度成比例 */
    f16VoltageAmp = FRAC16(0.12);
            // f16VoltageAmp -=MulF16(AbsF16(tDrvFoc.tPospeControl.f16wRotE1), 
            //                      1000); // VF比例系数可调
    // MulF16(AbsF16(tDrvFoc.tPospeControl.f16wRotE1), 
    //                                FRAC16(1.0)); // VF比例系数可调
    // if (tDrvFoc.tPospeControl.f16wRotE1 < FRAC16(0.15)) {
    //     f16VoltageAmp -=MulF16(AbsF16(tDrvFoc.tPospeControl.f16wRotE1), 
    //                              1000); // VF比例系数可调
    // } 
    // else {
    //     f16VoltageAmp +=MulF16(AbsF16(tDrvFoc.tPospeControl.f16wRotE1), 
    //                              65536); // VF比例系数可调
    // }
    
    // f16VoltageAmp -=MulF16(AbsF16(tDrvFoc.tPospeControl.f16wRotE1), 
    //                              1000); // VF比例系数可调
        


    /* 设定D轴和Q轴电压 */
    tDrvFoc.tUDQReq.f16Arg1 = 0; // D轴电压
    tDrvFoc.tUDQReq.f16Arg2 = f16VoltageAmp;             // Q轴电压为0

    /* Inv-Park变换 */
    GMC_InvParkF16(&tDrvFoc.tUAlBeReq, &tDrvFoc.tUDQReq, &tDrvFoc.tThetaTransform);
    //VF模式下不做补偿
    // tDrvFoc.tUAlBeReqDCB.f16Arg1 = tDrvFoc.tUAlBeReq.f16Arg1;
    // tDrvFoc.tUAlBeReqDCB.f16Arg2 = tDrvFoc.tUAlBeReq.f16Arg2;
    tDrvFoc.tDcBusComp.f16ArgDcBusMsr = tDrvFoc.f16DcBusFilt;
    // tDrvFoc.tDcBusComp.f16ArgDcBusMsr = tDrvFoc.f16DcBusFilt;
    /* BUS voltage complement*/
    GMC_DcBusCompF16AIWIN(&tDrvFoc.tUAlBeReqDCB, &tDrvFoc.tUAlBeReq, &tDrvFoc.tDcBusComp);
    // tDrvFoc.tUAlBeReqDCB.f16Arg1 =  tDrvFoc.tUAlBeReqDCB.f16Arg1 >> 2;
    // tDrvFoc.tUAlBeReqDCB.f16Arg2 =  tDrvFoc.tUAlBeReqDCB.f16Arg2 >> 2;
    // GMC_SvmExDpwm_F16(&spwmVM, &tDrvFoc.tUAlBeReqDCB,angle,tDrvFoc.tPospeControl.f16wRotE1);
    GMC_Svm(&tDrvFoc.tSvm, &tDrvFoc.tUAlBeReqDCB);
#else
#if ((ENABLE_HFI == 0) && (PRESENSORLESS_USE_DUAL_DQ == 1))
    
    DELTA_ANGLE = WrapAngleSumQ15(tDrvFoc.f16ThetaElSensorlessCorrected, -tDrvFoc.tPospeOpenloop.f16ThetaRotEl);
    if (ePrevPosModeSlowLoop != SENSORLESS && tDrvFoc.tPosMode == SENSORLESS && tDrvFoc.u32PreSensorlessTime == PERSENSORLESS_TIME_MS){
        DQ_DualSpaceSwitchInitUseAlphaBeta(tDrvFoc.tPospeOpenloop.f16ThetaRotEl,
                               tDrvFoc.tPospeControl.f16ThetaRotEl);
                               
    //    angle1_t = tDrvFoc.f16ThetaElSensorlessCorrected;
    //    angle2_t = tDrvFoc.tPospeControl.f16ThetaRotEl;
        // tDrvFoc.tSpeedRamp.f16State = tDrvFoc.tPospeControl.f16wRotElReqIFDeltaW;
        // tDrvFoc.tPospeControl.i16SpeedLoopCntr = tFocParas.SPEED_LOOP_CNTR;
    } else
#endif
    {   
#if (ENABLE_HFI == 1)
        if (tHfiCtrl.bIsFirstHfiToBemf != true){
            /* Clarke transformation: transfer three phase current into static coordinate */
            GMC_ClarkeF16(&tDrvFoc.tIAlBeFbck, &tDrvFoc.tIabcFbck);
        }

        tHfiCtrl.bIsFirstHfiToBemf = false;
#else
        GMC_ClarkeF16(&tDrvFoc.tIAlBeFbck, &tDrvFoc.tIabcFbck);
#endif
        
        /* Polar transformation for obtaining sin/cos value */
        MM_ThetaTransferTrigonoLut(tDrvFoc.tPospeControl.f16ThetaRotEl, &tDrvFoc.tThetaTransform);
        /* Parke transformation: transfer static coordinate into rotated coordinate */
        GMC_ParkeF16(&tDrvFoc.tIDQFbck, &tDrvFoc.tIAlBeFbck, &tDrvFoc.tThetaTransform);
        if (AbsF16(tDrvFoc.tPospeControl.f16wRotE1) >  FRAC16(0.05)) { // 超过1500rpm后，保持0d轴电�?
            g_tIqFbackLpFilter.in = tDrvFoc.tIDQFbck.f16Arg2;
            g_tIdFbackLpFilter.in = tDrvFoc.tIDQFbck.f16Arg1;
            IQ_LPFilterCalc(&g_tIqFbackLpFilter);
            IQ_LPFilterCalc(&g_tIdFbackLpFilter);
            tDrvFoc.tIDQFbck.f16Arg2 = g_tIqFbackLpFilter.out;
            tDrvFoc.tIDQFbck.f16Arg1 = g_tIdFbackLpFilter.out;
        } else{
            g_tIqFbackLpFilter.in = tDrvFoc.tIDQFbck.f16Arg2;
            g_tIdFbackLpFilter.in = tDrvFoc.tIDQFbck.f16Arg1;
            IQ_LPFilterCalc(&g_tIqFbackLpFilter);
            IQ_LPFilterCalc(&g_tIdFbackLpFilter);
        }
        // ==================== 转矩补偿逻辑开始 ====================
        Frac16_t f16IqReqFinal = tDrvFoc.tIDQReq.f16Arg2;

#if (ENABLE_TQ_COMPENSATION == 1)
        static Frac16_t s_f16LastThetaElec = 0;
        static Frac16_t s_f16ThetaMech = 0;
        static bool s_bIsFirstRun = true;
        // 仅在闭环模式下生效
        if (tDrvFoc.tPosMode == SENSORLESS) 
        {
            // 初始化上一时刻电角度
            if (s_bIsFirstRun) {
                s_f16LastThetaElec = tDrvFoc.tPospeControl.f16ThetaRotEl;
                s_bIsFirstRun = false;
            }

            // 1. 计算电角度增量并转换为机械角度增量
            // 这种方法保证了机械频率严格为电频率的 1/POLEPAIRS
            Frac16_t f16ThetaElec = tDrvFoc.tPospeControl.f16ThetaRotEl;
            int16_t i16DeltaThetaElec = (int16_t)(f16ThetaElec - s_f16LastThetaElec); // 利用int16溢出特性处理回绕
            s_f16LastThetaElec = f16ThetaElec;

            // 机械角度增量 = 电角度增量 / 极对数
            int16_t i16DeltaThetaMech = i16DeltaThetaElec / POLEPAIRS;
            
            // 累加机械角度
            f16ThetaMech += i16DeltaThetaMech;

            // 2. 查表获取补偿角度 theta_q_comp
            Frac16_t f16SpeedAbs = AbsF16(tDrvFoc.tPospeControl.f16wRotE1);
            Frac16_t f16ThetaComp = FRAC16(f16SpeedAbs);

            // 3. 计算总角度 (theta_mech + theta_comp)
            Frac16_t f16CompAngle = f16ThetaMech + f16ThetaComp;

            // 4. 计算正弦值 sin(theta_mech + theta_comp)
            MM_PolarTrigono_t tCompTrig;
            MM_ThetaTransferTrigonoLut(f16CompAngle, &tCompTrig);
            Frac16_t f16SinVal = tCompTrig.f16Sin;

            // 5. 计算补偿幅度 iq_Amp = Gain * |iq_ref|
            Frac16_t f16IqAmp = MulF16(AbsF16(tDrvFoc.tIDQReq.f16Arg2), TQ_COMP_GAIN);

            // 6. 计算最终补偿值 iq_comp = iq_Amp * sin(...)
            Frac16_t f16IqComp = MulF16(f16IqAmp, f16SinVal);
            g_f16IqCompVal = f16IqComp; // Debug用

            // 7. 叠加到 Iq Ref (前馈)
            f16IqReqFinal = SatAddF16(f16IqReqFinal, f16IqComp);
        }
        else
        {
            // 非闭环模式下，同步状态
            s_f16LastThetaElec = tDrvFoc.tPospeControl.f16ThetaRotEl;
            g_f16IqCompVal = 0;
            s_bIsFirstRun = true;
        }
#endif

        /* Calculate iDQ error */
        tDrvFoc.tIDQErr.f16Arg1 = SatSubF16(tDrvFoc.tIDQReq.f16Arg1, tDrvFoc.tIDQFbck.f16Arg1);
        tDrvFoc.tIDQErr.f16Arg2 = SatSubF16(tDrvFoc.tIDQReq.f16Arg2, tDrvFoc.tIDQFbck.f16Arg2);

        // 90% of available DCbus recalculated to phase voltage = 0.90*uDCB/sqrt(3)
        tDrvFoc.tAxisDCtrl.f16UpperLimit = MulF16(FRAC16(0.57), tDrvFoc.f16DcBusFilt);
        tDrvFoc.tAxisDCtrl.f16LowerLimit = -tDrvFoc.tAxisDCtrl.f16UpperLimit;

        /* D-axis current control*/
        tDrvFoc.tUDQReq.f16Arg1 = GCF_CtrlPIrAW_F16(tDrvFoc.tIDQErr.f16Arg1, &tDrvFoc.tAxisDCtrl);

        // Voltage limit
        
        tDrvFoc.tAxisQCtrl.f16UpperLimit =
            MM_SqrtLut_F16(SatSubF16(MulF16(tDrvFoc.tAxisDCtrl.f16UpperLimit, tDrvFoc.tAxisDCtrl.f16UpperLimit),
                                MulF16(tDrvFoc.tUDQReq.f16Arg1, tDrvFoc.tUDQReq.f16Arg1)));
        tDrvFoc.tAxisQCtrl.f16LowerLimit = -tDrvFoc.tAxisQCtrl.f16UpperLimit;

        /* Q-axis current control*/
        tDrvFoc.tUDQReq.f16Arg2 = GCF_CtrlPIrAW_F16(tDrvFoc.tIDQErr.f16Arg2, &tDrvFoc.tAxisQCtrl);
    
    }    
    ePrevPosModeSlowLoop = tDrvFoc.tPosMode;
   
    /* Inv-Park transformation*/
    GMC_InvParkF16(&tDrvFoc.tUAlBeReq, &tDrvFoc.tUDQReq, &tDrvFoc.tThetaTransform);
  
    
    tDrvFoc.tDcBusComp.f16ArgDcBusMsr = tDrvFoc.f16DcBusFilt;
    /* BUS voltage complement*/
#if (USE_5_SECTION_SVPWM == 0)

    GMC_DcBusCompF16(&tDrvFoc.tUAlBeReqDCB, &tDrvFoc.tUAlBeReq, &tDrvFoc.tDcBusComp);
#else
    GMC_DcBusCompF16AIWIN(&tDrvFoc.tUAlBeReqDCB, &tDrvFoc.tUAlBeReq, &tDrvFoc.tDcBusComp);
#endif

    // Frac16_t f16ModulationIndex =  MM_SqrtLut_F16(SatAddF16(MulF16(tDrvFoc.tUDQReq.f16Arg1, tDrvFoc.tUDQReq.f16Arg1),
    //                           MulF16(tDrvFoc.tUDQReq.f16Arg2, tDrvFoc.tUDQReq.f16Arg2)));
    // tDrvFoc.tSvm.f16ModulationIndex = MM_SatDivF16(f16ModulationIndex, tDrvFoc.tAxisDCtrl.f16UpperLimit);                        
    // /* SVPWM */
    Frac16_t angle;
    angle = tDrvFoc.tPospeControl.f16ThetaRotEl;
    angle += MM_AtanXYF16(tDrvFoc.tUDQReq.f16Arg1,tDrvFoc.tUDQReq.f16Arg2);

    // GMC_SvmExDpwm_F16(&tDrvFoc.tSvm, &tDrvFoc.tUAlBeReqDCB,angle,tDrvFoc.tPospeControl.f16wRotE1);
    // SVPWM_Calculate(&spwmVM, &tDrvFoc.tUAlBeReqDCB);
    // GMC_Svm_C(&tDrvFoc.tSvm, &tDrvFoc.tUAlBeReqDCB);
#if (USE_5_SECTION_SVPWM == 0)
    GMC_Svm(&tDrvFoc.tSvm, &tDrvFoc.tUAlBeReqDCB);
#else
    GMC_SvmExDpwm_F16(&tDrvFoc.tSvm, &tDrvFoc.tUAlBeReqDCB,angle,tDrvFoc.tPospeControl.f16wRotE1,SVPWM_ZERO_SEQ_COMP);
#endif

   

#endif    // GMC_SvmExDpwm_F16(&spwmVM, &tDrvFoc.tUAlBeReqDCB);
    // GMC_Svm(&tDrvFoc.tSvm, &tDrvFoc.tUAlBeReqDCB);
//    SvmActuate(&tDrvFoc.tPwmMod, &tDrvFoc.tSvm); //2.5us
    return (true);
}


/************************************************************
 * @brief: speed loop for FOC
 * @return <bool>
 ************************************************************/
Frac16_t f16IqReq = FRAC16(0.1);
Frac16_t f16IqReq_ = FRAC16(0.1);
Frac16_t f16SpeedERRORSum15 = 0;
Frac16_t f16SpeedPRout = 0;
Frac16_t f16AccelFeedforward = 0;
Frac16_t speedProut1 = 0;
Frac16_t speedProut = 0;
static bool FocSlowLoop()
{
    static Frac16_t f16Current;
    static Frac16_t f16Weight;
    static Frac16_t f16PwrIqReq;
    static Frac16_t f16OpenStateCnt = 0;
    static uint16_t u16SpeedRampCnt = 0;
    //static CTD_2SystF16_t tMtpaIdqReq;
    // tDrvFoc.f16Idcbus = DcCurrentLimit(f16DcCurLimit);
    // motorCalculateAllData(&tMotorCalculations);
    int32_t idcRaw;
    // idcRaw = (Frac16_t)(tDrvFoc.i16AdcRaw[4] - tCurrSample1rps.Signal.IuOffset); // 母线电流采样
    // idcRaw = idcRaw * 8; // 8倍放大
    // idcRaw = SatAddF16(idcRaw, MulF16(idcRaw, I_CALC_COEF));
    // if (idcRaw > 32767) {
    //     idcRaw = 32767;
    // } else if (idcRaw < -32768) {
    //     idcRaw = -32768;
    // }
    // motor_calculate_dc_current_by_sample(&tMotorCalculations,(int16_t)idcRaw);
    // tDrvFoc.f16Idcbus = tMotorCalculations.f16IDCFiltered;
    // tDrvFoc.i1610Idcbus = tDrvFoc.f16Idcbus  * I_FULL_SCALE_UINT * 10 / 32767; //母线电流
    // // tDrvFoc.i1610Irms = tMotorCalculations.i_rms_filtered  * I_FULL_SCALE_UINT * 10 / 32767; //母线电流
    // tMotorCalculations.active_power = MulF16(tMotorCalculations.vdc, tMotorCalculations.f16IDCFiltered);
    // tDrvFoc.i1610Power = tMotorCalculations.active_power * ((I_FULL_SCALE_UINT * V_FULL_SCALE_UINT)) * 10 / 32767; //母线电流;
    /* speed ramping up or down*/
    // tDrvFoc.tIDQReq.f16Arg1 = 0;
    u16SpeedRampCnt++;
#if (PRESENSORLESS_REDUCE_IQ_ENABLE == 1)
    if (((u16SpeedRampCnt >= SPEED_RAMP_SLOWLOOP_DIV)) || (tDrvFoc.tPosMode != SENSORLESS) ) {
        tDrvFoc.tPospeControl.f16wRotElReqRamp = GCF_Ramp_F16(tDrvFoc.tPospeControl.f16wRotElReq, &tDrvFoc.tSpeedRamp);
        u16SpeedRampCnt = 0;
    }
    tDrvFoc.tPospeControl.f16wRotElErr =
        SatSubF16(tDrvFoc.tPospeControl.f16wRotElReqRamp, tDrvFoc.tPospeControl.f16wRotE1);
        f16IqReq = GCF_PositionalPI_F16(tDrvFoc.tPospeControl.f16wRotElErr, &tDrvFoc.tSpeedCtrl);
#else
    if (((u16SpeedRampCnt >= SPEED_RAMP_SLOWLOOP_DIV) ) || (tDrvFoc.tPosMode != SENSORLESS) ) {
        tDrvFoc.tPospeControl.f16wRotElReqRamp = GCF_Ramp_F16(tDrvFoc.tPospeControl.f16wRotElReq, &tDrvFoc.tSpeedRamp);
        u16SpeedRampCnt = 0;
    }
    // if (DQ_DualSpaceGetBlendAlpha() == 0){
        tDrvFoc.tPospeControl.f16wRotElErr =
        SatSubF16(tDrvFoc.tPospeControl.f16wRotElReqRamp, tDrvFoc.tPospeControl.f16wRotE1);
        f16IqReq = GCF_PositionalPI_F16(tDrvFoc.tPospeControl.f16wRotElErr, &tDrvFoc.tSpeedCtrl);
    // } else{
        // f16IqReq = DQ_DualSpaceGetIqStart();
    // }
#endif

    // //运行速度环的PR控制器
    int16_t f0 = AbsF16(pr_get_center_freq_hz());

    // if ((f0) >= 8 ) {
    //     // speedProut = -FRAC16(0.1);
    //     speedProut1 = qprFxpUpdate(&g_qpr_speed, ( tDrvFoc.tPospeControl.f16wRotElErr * 1000), (f0 * 1000), (FOC_EPWM_FREQUENCY / 10)) * 3;
    //     // speedProut = -speedProut;
    //     // speedProut -=FRAC16(0.1);
    //     // qprFxpSetWcByAlphamhz(&g_qpr_speed, PR_SPEED_ALPHA,(f0 * 1000), (FOC_EPWM_FREQUENCY / 10));
    //     // f16IqReq = SatAddF16(f16IqReq, speedProut);

    //     speedProut = qprFxpUpdate(&g_qpr3_speed, ( tDrvFoc.tPospeControl.f16wRotElErr * 1000), (f0 * 3000), (FOC_EPWM_FREQUENCY / 10))* 3;
    //     // speedProut = -speedProut;
    //     // speedProut -=FRAC16(0.1);
    //     // qprFxpSetWcByAlphamhz(&g_qpr3_speed, PR_SPEED_ALPHA,(f0 * 3000), (FOC_EPWM_FREQUENCY / 10));
    //     speedProut = SatAddF16(speedProut1, speedProut);

    //     speedProut = -speedProut;
    //     f16IqReq = SatAddF16(f16IqReq, speedProut);

    // }


#if (SPEED_RAMP_REDUCE_ENABLE == 1)
    // 动态调整加减速斜率,防止超调
    if (tDrvFoc.tPosMode == SENSORLESS) {
        Frac16_t f16AbsSpeedErr = AbsF16(SatSubF16(tDrvFoc.tPospeControl.f16wRotElReq, tDrvFoc.tPospeControl.f16wRotElReqRamp));
        Frac16_t f16AbsSpeedTarget = AbsF16(tDrvFoc.tPospeControl.f16wRotElReq);
        
        // 计算速度误差占目标速度的百分比
        Frac16_t f16ErrorPercent = 0;
        if (f16AbsSpeedTarget > FRAC16(0.01)) {  // 避免除以接近0的数
            f16ErrorPercent = MM_SatDivF16(f16AbsSpeedErr, f16AbsSpeedTarget);
        }
        
        // 当速度误差百分比小于阈值时,开始降低斜率
        if (f16ErrorPercent < SPEED_RAMP_REDUCE_THRESHOLD && f16ErrorPercent > 0) {
            // 计算斜率衰减系数: errorPercent / threshold
            // 当误差从15%降到0时,系数从1.0降到0
            Frac16_t f16RampRatio = MM_SatDivF16(f16ErrorPercent, SPEED_RAMP_REDUCE_THRESHOLD);
            
            // 限制最小斜率比例,避免斜率过小导致响应过慢
            if (f16RampRatio < SPEED_RAMP_MIN_RATIO) {
                f16RampRatio = SPEED_RAMP_MIN_RATIO;
            }
            
            // 根据加速还是减速,分别调整斜率
            if (tDrvFoc.tPospeControl.f16wRotElReqRamp < tDrvFoc.tPospeControl.f16wRotElReq) {
                // 正在加速
                tDrvFoc.tSpeedRamp.f16RampUp = MulF16(CL_SpeedRampInc, f16RampRatio);
                // 确保斜率不为0
                if (tDrvFoc.tSpeedRamp.f16RampUp < 1) {
                    tDrvFoc.tSpeedRamp.f16RampUp = 1;
                }
            } else if (tDrvFoc.tPospeControl.f16wRotElReqRamp > tDrvFoc.tPospeControl.f16wRotElReq) {
                // 正在减速
                tDrvFoc.tSpeedRamp.f16RampDown = MulF16(CL_SpeedRampDec, f16RampRatio);
                // 确保斜率不为0
                if (tDrvFoc.tSpeedRamp.f16RampDown < 1) {
                    tDrvFoc.tSpeedRamp.f16RampDown = 1;
                }
            }
        } else {
            // 误差较大时,恢复正常斜率
            tDrvFoc.tSpeedRamp.f16RampUp = CL_SpeedRampInc;
            tDrvFoc.tSpeedRamp.f16RampDown = CL_SpeedRampDec;
        }
    }
#endif
#if (SPEED_FF_ENABLE == 1)
    // 加速度前馈（仅在闭环模式下生效）
    if (tDrvFoc.tPosMode == SENSORLESS) {
  
        
        // 计算速度误差的绝对值和目标速度的绝对值
        Frac16_t f16AbsSpeedErr = AbsF16(tDrvFoc.tPospeControl.f16wRotElErr);
        Frac16_t f16AbsSpeedTarget = AbsF16(tDrvFoc.tPospeControl.f16wRotElReq);
        
        // 计算速度误差占目标速度的百分比
       
        if (f16AbsSpeedTarget > FRAC16(0.01)) {  // 避免除以接近0的数
            FeedBackf16ErrorPercent = MM_SatDivF16(f16AbsSpeedErr, f16AbsSpeedTarget);
        }
        
        // 当速度误差百分比大于阈值时才使能前馈
        if (FeedBackf16ErrorPercent > SPEED_FF_ERROR_PERCENT) {
            // 判断加速还是减速
            if (tDrvFoc.tPospeControl.f16wRotElReqRamp < tDrvFoc.tPospeControl.f16wRotElReq) {
                // 正在加速，使用加速斜率作为前馈
                f16AccelFeedforward = tDrvFoc.tSpeedRamp.f16RampUp * 20;
            } else if (tDrvFoc.tPospeControl.f16wRotElReqRamp > tDrvFoc.tPospeControl.f16wRotElReq) {
                // 正在减速，使用减速斜率作为前馈（注意符号）
                f16AccelFeedforward = -tDrvFoc.tSpeedRamp.f16RampDown * 20;
            }
            
                // 简单限幅，避免前馈量过大
            if (f16AccelFeedforward > FRAC16(0.3)) {
                f16AccelFeedforward = FRAC16(0.3);
            } else if (f16AccelFeedforward < FRAC16(-0.3)) {
                f16AccelFeedforward = FRAC16(-0.3);
            }
        }else {
            // 误差小于阈值，不使用前馈
            f16AccelFeedforward = 0;
        }
        
        // 添加前馈到PI输出
        f16IqReq = SatAddF16(f16IqReq, f16AccelFeedforward);
    }
#endif    
    switch (tDrvFoc.tPosMode) {
    case FORCE:
        f16Current = tDrvFoc.tPospeOpenloop.f16IQUpperLimit;
        f16Weight = 0;
        tDrvFoc.tIDQReq.f16Arg1 = 0;
        tDrvFoc.tIDQReq.f16Arg2 = (tDrvFoc.tPospeControl.f16wRotElReq >= FRAC16(0)) ? f16Current : -f16Current;
        break;
    case TRACKING:
//        f16Current = tDrvFoc.tPospeOpenloop.f16IQUpperLimit;
//        f16Weight = 0;
//        tDrvFoc.tIDQReq.f16Arg1 = f16Current;
//        tDrvFoc.tIDQReq.f16Arg2 = 0;
        f16Current = tDrvFoc.tPospeOpenloop.f16IQUpperLimit;
        f16Weight = SatSubF16(AbsF16(tDrvFoc.tPospeOpenloop.f16wRotE1), tDrvFoc.tPospeSensorless.f16wRotElMatch_1);
        f16Weight = MM_SatDivF16(f16Weight, SatSubF16(tDrvFoc.tPospeSensorless.f16wRotElMatch_2, tDrvFoc.tPospeOpenloop.f16wRotE1));        
        tDrvFoc.tIDQReq.f16Arg1 = MulF16(MM_SqrtLut_F16(SatSubF16(FRAC16(1.0), f16Weight)), AbsF16(f16Current));
        tDrvFoc.tIDQReq.f16Arg2 = MulF16(MM_SqrtLut_F16(f16Weight), f16Current);
        tDrvFoc.tIDQReq.f16Arg1 = 0;
        tDrvFoc.tIDQReq.f16Arg2 = (tDrvFoc.tPospeControl.f16wRotElReq >= FRAC16(0)) ? f16Current : -f16Current;
        break;
    case PRESENSORLESS:
        // f16Current = tDrvFoc.tPospeOpenloop.f16IQUpperLimit;
#if (PRESENSORLESS_REDUCE_IQ_ENABLE == 1)
        // 计算开环角度与观测器角度的差值
        Frac16_t f16AngleError = WrapAngleSumQ15(
            tDrvFoc.f16ThetaElSensorlessCorrected,  // 观测器角度
            -tDrvFoc.tPospeOpenloop.f16ThetaRotEl    // 开环角度(取负)
        );
        
        // 存储角度误差用于外部观测
        f16ThetaErr = f16AngleError;
        
        // 限制角度误差范围 (防止异常值)
        if (f16AngleError > FRAC16(0.5)) {
            f16AngleError = FRAC16(0.5);
        } else if (f16AngleError < FRAC16(-0.5)) {
            f16AngleError = FRAC16(-0.5);
        }
        tDrvFoc.u16ReduceIQControllCnt++;
        if (tDrvFoc.u16ReduceIQControllCnt >= PRESENSORLESS_DIV){
            // 根据角度误差调整Iq电流: i_q(k+1) = i_q(k) - K_gain * ω_err * T_s
            Frac16_t f16IqAdjustment = MulF16(PRESENSORLESS_ANGLE_ERROR_GAIN, f16AngleError);
            
            // 更新Iq参考值
            f16Current = SatSubF16(f16Current, f16IqAdjustment);
            
            // 限制Iq最小值
            if (AbsF16(f16Current) < PRESENSORLESS_IQ_MIN_LIMIT) {
                f16Current = (f16Current >= 0) ? PRESENSORLESS_IQ_MIN_LIMIT : -PRESENSORLESS_IQ_MIN_LIMIT;
            }
            tDrvFoc.u16ReduceIQControllCnt = 0;
        }
        
        // 设置DQ轴电流参考
        tDrvFoc.tIDQReq.f16Arg1 = 0;
        tDrvFoc.tIDQReq.f16Arg2 = f16Current;
        
        // 检查角度是否收敛 - 直接修改计数器提前切换
        static uint32_t u32AngleConvergedCnt = 0;
        if (AbsF16(f16AngleError) < PRESENSORLESS_ANGLE_CONVERGED) {
            u32AngleConvergedCnt++;
            if (u32AngleConvergedCnt >= PRESENSORLESS_CONVERGED_COUNT) {
                // 角度已收敛，直接触发切换
                tDrvFoc.u32PreSensorlessCnt = tDrvFoc.u32PreSensorlessTime;
                u32AngleConvergedCnt = 0;
            }
        } else {
            u32AngleConvergedCnt = 0;
        }
#else
        f16Current = tDrvFoc.tPospeOpenloop.f16IQUpperLimit;
        tDrvFoc.tIDQReq.f16Arg1 = 0;
        tDrvFoc.tIDQReq.f16Arg2 = (tDrvFoc.tPospeControl.f16wRotElReq >= FRAC16(0)) ? f16Current : -f16Current;
#endif
        break;
    case SENSORLESS:
           /* MTPA */
        //tDrvFoc.tIDQReq.f16Arg1 = 0;//
        // 应用IQ限制控制
#if (ENABLE_CURRENT_POWER_LIMIT == 1)
        f16IqReq_ = IQ_LimitCtrl_Process(&tIqLimitCtrl,
                                                        tMotorCalculations.f16IDCFiltered,  // 滤波后的母线电流
                                                        tMotorCalculations.active_power,    // 有功功率
                                                        f16IqReq);                // 速度环输出的IQ
#else
        f16IqReq_ = f16IqReq;
        
#endif
    
        // AMC_FluxWeakening_F16(f16IqReq_, tDrvFoc.tPospeControl.f16wRotE1, &tDrvFoc.tIDQReq, &tDrvFoc.tFluxWeakening);
        //AMC_FluxWeakening_IqUqReserveDiff_F16_AIWIN(f16IqReq_, tDrvFoc.tPospeControl.f16wRotE1, &tDrvFoc.tIDQReq, &tDrvFoc.tFluxWeakening,&tDrvFoc.tUDQReq,tDrvFoc.f16DcBusFilt);
        ////FRAC16(-0.1); // 保持一个较小的d轴电流，防止磁链下降
        if (AbsF16(tDrvFoc.tPospeControl.f16wRotE1) > FRAC16(200.0/4000.0)){ // 超过1500rpm后，保持0d轴电流
            tDrvFoc.tIDQReq.f16Arg1 = FRAC16(0.05);
        }
        tDrvFoc.tIDQReq.f16Arg1 = 0;
		tDrvFoc.tIDQReq.f16Arg2 = f16IqReq_;
        // //随转速增加绝对值增加，符号与转速方向同号
        // Frac16_t f16IQ = 0;
        // f16IQ = MulF16(FRAC16(0.3), AbsF16(tDrvFoc.tPospeControl.f16wRotE1));
        // if (tDrvFoc.tPospeControl.f16wRotE1 < FRAC16(0)){
        //     f16IQ = -f16IQ;
        // }
        // tDrvFoc.tIDQReq.f16Arg1 = f16IQ;

          // 新：按alpha从切换Id过渡到运行Id（示例保留id_fw=0）
#if((ENABLE_HFI == 0) && (PRESENSORLESS_USE_DUAL_DQ == 1))
          if (DQ_DualSpaceGetBlendAlpha() > 0) {
            Frac16_t f16CurrentSpeedError = 0;
            Frac16_t alpha      = DQ_DualSpaceGetBlendAlpha();
            Frac16_t one_minus  = SatSubF16(FRAC16(1.0), alpha);

            Frac16_t id_start   = DQ_DualSpaceGetIdStart();
            Frac16_t id_fw      = tDrvFoc.tIDQReq.f16Arg1;
            Frac16_t id_blend   = SatAddF16(MulF16(id_start, alpha),
                                            MulF16(id_fw, one_minus));
            tDrvFoc.tIDQReq.f16Arg1 = id_blend;

        //     Frac16_t iq_start   = DQ_DualSpaceGetIqStart();
        //     Frac16_t iq_target  = s_f16IdIQMaintain;
        //     Frac16_t iq_delta   = SatSubF16(iq_target, iq_start);
        //     Frac16_t iq_blend   = SatAddF16(iq_start, MulF16(iq_delta, one_minus));

        //     if (tDrvFoc.tPospeSensorless.f16IQUpperLimit < iq_blend) {
        //         iq_blend = tDrvFoc.tPospeSensorless.f16IQUpperLimit;
        //     } else if (tDrvFoc.tPospeSensorless.f16IQLowerLimit > iq_blend) {
        //         iq_blend = tDrvFoc.tPospeSensorless.f16IQLowerLimit;
        //     }
        //     tDrvFoc.tIDQReq.f16Arg2 = iq_blend;

        //     tDrvFoc.tSpeedCtrl.f32IntegPartK_1 = tDrvFoc.tIDQReq.f16Arg2  << 15;
        //     Frac16_t f16SpeedInte =   tDrvFoc.tIDQReq.f16Arg2;
        //     f16CurrentSpeedError = SatSubF16(
        //             tDrvFoc.tPospeControl.f16wRotElReqRamp,
        //             tDrvFoc.tPospeSensorless.f16wRotE1
        //         );
    
        // // 无扰切换：使速度环下次输出当前的Iq参考值
        //     SetPIControllerBumpless(
        //         &tDrvFoc.tSpeedCtrl, 
        //         f16SpeedInte,      // 目标输出
        //         f16CurrentSpeedError          // 当前误差
        //     ); 
        //      SetVariablePIControllerBumpless(&tVariableSpeedCtrl, 
        //                             f16SpeedInte,
        //                             f16CurrentSpeedError);      

            DQ_DualSpaceDecayBlendAlpha();
        } 

        // tDrvFoc.tIDQReq.f16Arg2 = s_f16IdIQend;
        // tDrvFoc.tIDQReq.f16Arg1 = DQ_DualSpaceGetIdStart();
        // tDrvFoc.tIDQReq.f16Arg1 = DQ_DualSpaceGetIdStart();
        //if(AbsF16(tDrvFoc.tIDQReq.f16Arg1)  < FRAC16(-0.02)) tDrvFoc.tIDQReq.f16Arg1 = FRAC16(-0.02); // 运行时Id设为0
#endif
        break;
    default:
        tDrvFoc.tPosMode = SENSORLESS;
    }

    return true;
}

#if (ENABLE_HFI == 1)
void StateHFI()
{
    bool bFnStatus = true;
    
    /* entering state */
    tDrvFoc.tAppState.tStatus = HFI;
    tDrvFoc.tAppState.tEvent = E_HFI;

    if ((tHfiConfig.bIpdDebugEnable == 0) && (tHfiConfig.bPolarityDebugEnable == 0) && (tHfiConfig.bTrackingDebugEnable == 0)){
        tDrvFoc.tAppState.tEvent = E_HFI_DONE;
        bFnStatus = false;
        return;
    }

    if (tDrvFoc.tPosMode != HFI_MODE){
        tDrvFoc.tAppState.tEvent = E_HFI_DONE;
        bFnStatus = false;
        return;
    }
    
#if (ENABLE_HFI == 1)
    // 只有在启用HFI的情况下执行
    
    // 首次进入HFI状态
    if (tDrvFoc.u16HfiCntr == 0) {
        DisableOutput(); // 关闭输出,因为下面要切换PWM频率
        tDrvFoc.tSvm.tPwmDuty.f16Arg1 = FRAC16(0.0);
        tDrvFoc.tSvm.tPwmDuty.f16Arg2 = FRAC16(0.0);
        tDrvFoc.tSvm.tPwmDuty.f16Arg3 = FRAC16(0.0);
        // 启用HFI
        HFI_Enable(&tHfiCtrl, true);
        // EnableOutput(); // 使能输出
        // 计数器增加
        tDrvFoc.u16HfiCntr++;
        return;
    }
    if (tDrvFoc.u16HfiCntr ==1) EnableOutput();

    // 执行HFI处理
    HFI_Process(&tHfiCtrl,&tHfiConfig);
    
    // 计数器增加
    tDrvFoc.u16HfiCntr++;
    
    // 检查HFI是否完成或超时
    // TODO:这里没有考虑失败的情况，注意，后面要实现失败了直接定位强拖的功能
    if ((tHfiConfig.bIpdDebugEnable == 0) && (tHfiConfig.bPolarityDebugEnable == 0) && (tHfiConfig.bTrackingDebugEnable == 0)){
        if (HFI_IsTransitionComplete(&tHfiCtrl) || tDrvFoc.u16HfiCntr > HFI_TIMEOUT_TIME) {
            
            // 切换到运行状态
            tDrvFoc.tAppState.tEvent = E_HFI_DONE;
            tDrvFoc.tPosMode = SENSORLESS; // 开始运行时以开环方式运行
            
            bFnStatus = false;
        }
    }

#else
    // 如果HFI未启用，直接切换到运行状态
    tDrvFoc.tAppState.tEvent = E_HFI_DONE;
    bFnStatus = false;
#endif

    if (!bFnStatus) {
        // 切换到下一状态
        tDrvFoc.tAppState.tEvent = E_HFI_DONE;
    }
}
#endif

Frac16_t f16OmegaDiff;
Frac16_t f16OmegaDiffFilt;
void StateRun()
{
    static bool stateRunStatus;

#if (ENABLE_DAMPING_COMPENSATION == 1)
   static Frac16_t f16EdFiltered = 0;
    static Frac16_t f16EdVibration = 0;
#endif

    tDrvFoc.tAppState.tStatus = RUN;
    tDrvFoc.tAppState.tEvent  = E_RUN;

    /* reconstruct phase current */
    stateRunStatus = false;
#if (ENABLE_HFI == 1)
    if (tDrvFoc.tPosMode == HFI_MODE) {
        // HFI模式下，不需要其他观测器计算
        // 计算开环速度是必要的，因为可能会从HFI切换回开环
        CalcOpenLoop(&tDrvFoc.tPospeOpenloop, tDrvFoc.tPospeControl.f16wRotElReqRamp);
    } else {

        /* calculate open loop speed */
        CalcOpenLoop(&tDrvFoc.tPospeOpenloop, tDrvFoc.tPospeControl.f16wRotElReqRamp);

        /* Start calculation of the Bemf Observer in tracking mode */
        AMC_AIWINPMSMBemfObsvrDQ_F16(&tDrvFoc.tPospeSensorless.f16wRotE1, &tDrvFoc.tPospeSensorless.f16ThetaRotEl,
                                &tDrvFoc.tIAlBeFbck, &tDrvFoc.tUAlBeReq, &tDrvFoc.tPospeSensorless.tBemfObsvr);
    }
    if (tDrvFoc.u16HfiCntr != 0) {
        tDrvFoc.u16HfiCntr = 0;
        HFI_ResetPwmConfig();
        HFI_Enable(&tHfiCtrl, false);
    }
#else

    

    /* calculate open loop speed */
#if (ENABLE_DAMPING_COMPENSATION == 0)
    //CalcOpenLoop(&tDrvFoc.tPospeOpenloop, tDrvFoc.tPospeControl.f16wRotElReqRamp);
#endif

    /* Start calculation of the Bemf Observer in tracking mode */
    // AMC_AIWINPMSMBemfObsvrDQ_F16(&tDrvFoc.tPospeSensorless.f16wRotE1, &tDrvFoc.tPospeSensorless.f16ThetaRotEl,
    //                         &tDrvFoc.tIAlBeFbck, &tDrvFoc.tUAlBeReq, &tDrvFoc.tPospeSensorless.tBemfObsvr);
    AMC_PMSMBemfObsvrDQ_F16(&tDrvFoc.tPospeSensorless.f16wRotE1, &tDrvFoc.tPospeSensorless.f16ThetaRotEl,
                            &tDrvFoc.tIAlBeFbck, &tDrvFoc.tUAlBeReq, &tDrvFoc.tPospeSensorless.tBemfObsvr);
    // if (tDrvFoc.tPospeSensorless.f16wRotE1 < FRAC16(50.0/4000.0)){
    //     if (f16EdEst < 0){
    //         f16EdEst = 0;
    //     }
    // }
                            
    if (tDrvFoc.tPospeSensorless.tBemfObsvr.ptEObsrv.f16Arg2 > 0 && tDrvFoc.tPospeControl.f16wRotE1 > 0) {
        tDrvFoc.f16ThetaElSensorlessCorrected = tDrvFoc.tPospeSensorless.f16ThetaRotEl+32767;
    } else {
        tDrvFoc.f16ThetaElSensorlessCorrected = tDrvFoc.tPospeSensorless.f16ThetaRotEl; 
    }         
    // tDrvFoc.f16ThetaElSensorlessCorrected = tDrvFoc.tPospeSensorless.f16ThetaRotEl;                
#endif
    // if (tDrvFoc.tWMObsvr.tWmDir != TAILWIND) {
    //     AutomaticMode();
    // }
    AutomaticMode();
    /* different processing in various position mode*/
    switch (tDrvFoc.tPosMode) {
    case FORCE:
#if (ENABLE_DAMPING_COMPENSATION == 1)
        CalcOpenLoop(&tDrvFoc.tPospeOpenloop, tDrvFoc.tPospeControl.f16wRotElReqIFDeltaW);
        tDrvFoc.tPospeOpenloop.f16wRotE1 = tDrvFoc.tPospeControl.f16wRotElReqRamp; 
#else
        CalcOpenLoop(&tDrvFoc.tPospeOpenloop, tDrvFoc.tPospeControl.f16wRotElReqRamp);
#endif
        
        tDrvFoc.tSpeedCtrl.f16UpperLimit = tDrvFoc.tPospeOpenloop.f16IQUpperLimit;
        tDrvFoc.tSpeedCtrl.f16LowerLimit = -(tDrvFoc.tPospeOpenloop.f16IQUpperLimit);

        tDrvFoc.tPospeControl.f16ThetaRotEl = tDrvFoc.tPospeOpenloop.f16ThetaRotEl;
        tDrvFoc.tPospeControl.f16wRotE1     = 0;

        tDrvFoc.tSpeedRamp.f16RampUp   = (OL_SpeedRampInc);
        tDrvFoc.tSpeedRamp.f16RampDown = (OL_SpeedRampInc);

        tDrvFoc.tSpeedCtrl.f32IntegPartK_1 =  tDrvFoc.tIDQReq.f16Arg2 << 14;

        break;
    case TRACKING:
#if (ENABLE_DAMPING_COMPENSATION == 1)
        CalcOpenLoop(&tDrvFoc.tPospeOpenloop, tDrvFoc.tPospeControl.f16wRotElReqIFDeltaW);
        tDrvFoc.tPospeOpenloop.f16wRotE1 = tDrvFoc.tPospeControl.f16wRotElReqRamp; //即使有ENABLE_DAMPING_COMPENSATION，不使用f16wRotElReqIFDeltaW作为切换速度，因为可能提前跳过去到切换速度，此时观测器还没收敛
#else
        CalcOpenLoop(&tDrvFoc.tPospeOpenloop, tDrvFoc.tPospeControl.f16wRotElReqRamp);
#endif
        // 速度环
        f16ThetaErr = tDrvFoc.tPospeSensorless.f16ThetaRotEl - tDrvFoc.tPospeOpenloop.f16ThetaRotEl;
        tDrvFoc.tSpeedCtrl.f16UpperLimit = tDrvFoc.tPospeOpenloop.f16IQUpperLimit;
        tDrvFoc.tSpeedCtrl.f16LowerLimit = -(tDrvFoc.tPospeOpenloop.f16IQUpperLimit);

        tDrvFoc.tPospeControl.f16ThetaRotEl = tDrvFoc.tPospeOpenloop.f16ThetaRotEl;
//        tDrvFoc.tPospeControl.f16wRotE1     = 0;//tDrvFoc.tPospeSensorless.f16wRotE1;
        tDrvFoc.tPospeControl.f16wRotE1     = tDrvFoc.tPospeControl.f16wRotElReqRamp;//tDrvFoc.tPospeSensorless.f16wRotE1; 即使有ENABLE_DAMPING_COMPENSATION，不使用f16wRotElReqIFDeltaW作为切换速度，因为可能提前跳过去到切换速度，此时观测器还没收敛
        tDrvFoc.tSpeedCtrl.f32IntegPartK_1 =  tDrvFoc.tIDQReq.f16Arg2 << 15;

        tDrvFoc.tSpeedRamp.f16RampUp   = (OL_SpeedRampInc);
        tDrvFoc.tSpeedRamp.f16RampDown = (OL_SpeedRampInc);
 
        break;
#if (ENABLE_DAMPING_COMPENSATION == 1)
    case PRESENSORLESS:
        //加速度保持
        tDrvFoc.u32PreSensorlessCnt++;
        CalcOpenLoop(&tDrvFoc.tPospeOpenloop, tDrvFoc.tPospeControl.f16wRotElReqIFDeltaW);
        tDrvFoc.tPospeOpenloop.f16wRotE1 = tDrvFoc.tPospeControl.f16wRotElReqRamp; //即使有ENABLE_DAMPING_COMPENSATION，不使用f16wRotElReqIFDeltaW作为切换速度，因为可能提前跳过去到切换速度，此时观测器还没收敛

        
        f16ThetaErr = tDrvFoc.tPospeSensorless.f16ThetaRotEl - tDrvFoc.tPospeOpenloop.f16ThetaRotEl;
        tDrvFoc.tSpeedCtrl.f16UpperLimit = tDrvFoc.tPospeOpenloop.f16IQUpperLimit;
        tDrvFoc.tSpeedCtrl.f16LowerLimit = -(tDrvFoc.tPospeOpenloop.f16IQUpperLimit);

        tDrvFoc.tPospeControl.f16ThetaRotEl = tDrvFoc.tPospeOpenloop.f16ThetaRotEl;
//        tDrvFoc.tPospeControl.f16wRotE1     = 0;//tDrvFoc.tPospeSensorless.f16wRotE1;
        tDrvFoc.tPospeControl.f16wRotE1     = tDrvFoc.tPospeControl.f16wRotElReqRamp;//tDrvFoc.tPospeSensorless.f16wRotE1; 即使有ENABLE_DAMPING_COMPENSATION，不使用f16wRotElReqIFDeltaW作为切换速度，因为可能提前跳过去到切换速度，此时观测器还没收敛
        tDrvFoc.tSpeedCtrl.f32IntegPartK_1 =  tDrvFoc.tIDQReq.f16Arg2 << 15;

        tDrvFoc.tSpeedRamp.f16RampUp   = (0);
        tDrvFoc.tSpeedRamp.f16RampDown = (0);
        break;
#endif
#if (ENABLE_HFI == 1)
    case HFI_MODE://要嘛静止状态到这里，要嘛从告诉状态下来
        // HFI模式下使用HFI控制器的角度和速度
        tDrvFoc.tSpeedRamp.f16RampUp   = (CL_SpeedRampInc);
        tDrvFoc.tSpeedRamp.f16RampDown = (CL_SpeedRampDec);

        // tDrvFoc.tSpeedCtrl.f16UpperLimit = tDrvFoc.tPospeSensorless.f16IQUpperLimit;
        // tDrvFoc.tSpeedCtrl.f16LowerLimit = -tDrvFoc.tPospeSensorless.f16IQUpperLimit;
        
        // 使用HFI估计的角度和速度
        tDrvFoc.tPospeControl.f16ThetaRotEl = tHfiCtrl.f16Theta;
        tDrvFoc.tPospeControl.f16wRotE1 = tHfiCtrl.f16MachineSpeed;

        tDrvFoc.tPospeSensorless.f16ThetaRotEl = tHfiCtrl.f16Theta;
        tDrvFoc.tPospeSensorless.f16wRotE1 = tHfiCtrl.f16MachineSpeed;
        
        // 保持开环数据更新，以便切换回FORCE/TRACKING模式时平滑过渡
        // tDrvFoc.tPospeOpenloop.f16wRotE1 = tDrvFoc.tPospeControl.f16wRotE1;
        // tDrvFoc.tPospeOpenloop.tOplpInteg.f16InK1 = tDrvFoc.tPospeControl.f16ThetaRotEl;
        // tDrvFoc.tPospeOpenloop.tOplpInteg.f32State = tDrvFoc.tPospeControl.f16ThetaRotEl << 15;
        // HFI_ProcessTracking(&tHfiCtrl);
        if (tHfiCtrl.eState != HFI_STATE_TRACKING){
            if (tDrvFoc.u16HfiCntr == 0) { //一次开机过程复位一次为0
                DisableOutput(); // 关闭输出,因为下面要切换PWM频率
                tDrvFoc.tSvm.tPwmDuty.f16Arg1 = FRAC16(0.0);
                tDrvFoc.tSvm.tPwmDuty.f16Arg2 = FRAC16(0.0);
                tDrvFoc.tSvm.tPwmDuty.f16Arg3 = FRAC16(0.0);
                // 启用HFI
                HFI_Enable(&tHfiCtrl, true);
                // EnableOutput(); // 使能输出
                // 计数器增加
                tDrvFoc.u16HfiCntr++;
                return;
            }

            if (tDrvFoc.u16HfiCntr == 1) EnableOutput();
        }

        // if (tHfiCtrl.bPolarityValid == true || (tDrvFoc.tPospeControl.f16wRotE1 >= BEMF_TO_HFI_THRESHOLD) ) tHfiCtrl.eState = HFI_STATE_TRACKING; //在已经运行过程中，运行HFI的话应该直接运行
        // 执行HFI处理
        HFI_Process(&tHfiCtrl,&tHfiConfig);
        
        // 计数器增加
        // tDrvFoc.u16HfiCntr++;
        break;
#endif
        
    case SENSORLESS:
#if (ENABLE_DAMPING_COMPENSATION == 1)
        if (ePrevPosModeSlowLoop != SENSORLESS && tDrvFoc.tPosMode == SENSORLESS) {
            CalcOpenLoop(&tDrvFoc.tPospeOpenloop, tDrvFoc.tPospeControl.f16wRotElReqIFDeltaW);
            if (tDrvFoc.tPospeControl.i16SpeedLoopCntr >= (tFocParas.SPEED_LOOP_CNTR - 1)) {
               tDrvFoc.tPospeControl.i16SpeedLoopCntr--; // 防止直接运行到速度环，先运行了一次速度环，拿到的已经是速度环控制后的IQ去进行DQ变换
            }
        } else{
            CalcOpenLoop(&tDrvFoc.tPospeOpenloop, tDrvFoc.tPospeControl.f16wRotElReqRamp);
        }
        
#endif

#if (SPEED_RAMP_REDUCE_ENABLE == 0)
        tDrvFoc.tSpeedRamp.f16RampUp   = (CL_SpeedRampInc);
        tDrvFoc.tSpeedRamp.f16RampDown = (CL_SpeedRampDec);
#endif
        tDrvFoc.tSpeedCtrl.f16UpperLimit = tDrvFoc.tPospeSensorless.f16IQUpperLimit;
        tDrvFoc.tSpeedCtrl.f16LowerLimit = tDrvFoc.tPospeSensorless.f16IQLowerLimit;

#if(ENABLE_HFI == 0)
        // if (tDrvFoc.tPospeSensorless.tBemfObsvr.ptEObsrv.f16Arg2 > 0 && tDrvFoc.tPospeControl.f16wRotE1 > 0) {
        //     tDrvFoc.tPospeControl.f16ThetaRotEl = tDrvFoc.tPospeSensorless.f16ThetaRotEl+32767;
        // } else {
        //     tDrvFoc.tPospeControl.f16ThetaRotEl = tDrvFoc.tPospeSensorless.f16ThetaRotEl; 
        // }
#else 
        // if ((tDrvFoc.tPospeSensorless.tBemfObsvr.ptEObsrv.f16Arg2 < 0 && tDrvFoc.tPospeControl.f16wRotE1 > 0) ||
        //     (tDrvFoc.tPospeSensorless.tBemfObsvr.ptEObsrv.f16Arg2 > 0 && tDrvFoc.tPospeControl.f16wRotE1 < 0)) {
        //     tDrvFoc.tPospeControl.f16ThetaRotEl = tDrvFoc.tPospeSensorless.f16ThetaRotEl+32767;
        // } else {
        //     tDrvFoc.tPospeControl.f16ThetaRotEl = tDrvFoc.tPospeSensorless.f16ThetaRotEl; 
        // }
        tDrvFoc.tPospeControl.f16ThetaRotEl = tDrvFoc.tPospeSensorless.f16ThetaRotEl; 
#endif
        // tDrvFoc.tPospeControl.f16ThetaRotEl += GetAngleIncrement(tDrvFoc.tPospeSensorless.f16wRotE1) * 2;
        tDrvFoc.tPospeControl.f16ThetaRotEl = tDrvFoc.f16ThetaElSensorlessCorrected;
        tDrvFoc.tPospeControl.f16wRotE1 = tDrvFoc.tPospeSensorless.f16wRotE1;
        tDrvFoc.tPospeOpenloop.f16wRotE1 = tDrvFoc.tPospeControl.f16wRotE1;
        tDrvFoc.tPospeOpenloop.tOplpInteg.f16InK1 = tDrvFoc.tPospeControl.f16ThetaRotEl;
        tDrvFoc.tPospeOpenloop.tOplpInteg.f32State = tDrvFoc.tPospeControl.f16ThetaRotEl << 15;
        break;
    default:
        tDrvFoc.tPosMode = SENSORLESS;
    }
    

    
    /*-----------------------------------------------------
    Calculate Field Oriented Control FOC
    ----------------------------------------------------- */
    if (HFI_MODE == tDrvFoc.tPosMode)
    {
        return; // HFI模式下不执行正常的电流环与速度环以及堵转检测
    }
    // if (StallDetection(&tStallDet)) {
    //     // BrakeOutput();
    //     if (tFaultMode.FM_b.MOT_STALL == 0) {
    //         tFaultMode.FM_b.MOT_STALL = 1;
    //     }
        
    //     // tDrvFoc.tAppState.tEvent = E_FAULT;
    // }
    
    /*-----------------------------------------------------
    Calculate Field Oriented Control FOC
    ----------------------------------------------------- */
    if (++tDrvFoc.tPospeControl.i16SpeedLoopCntr >= tFocParas.SPEED_LOOP_CNTR) {
        tDrvFoc.tPospeControl.i16SpeedLoopCntr = 0;
        stateRunStatus                         = FocSlowLoop();
    }

    stateRunStatus = FocFastLoop();
#if (ENABLE_DAMPING_COMPENSATION == 1)
    // 实现开环阻尼控制 - 提取d轴反电动势振荡分量
    if (tDrvFoc.tPosMode == FORCE ||tDrvFoc.tPosMode == TRACKING || tDrvFoc.tPosMode == PRESENSORLESS) {
    // if (tDrvFoc.tPosMode == FORCE ||tDrvFoc.tPosMode == TRACKING ) {
    
        //速度高通提供阻尼
        // g_tEdampHpFilter.in = tDrvFoc.tPospeSensorless.f16wRotE1;
        Frac16_t f16InstantPower_Ualpha_Ialpha = MulF16(tDrvFoc.tUAlBeReqDCB.f16Arg1, tDrvFoc.tIAlBeFbck.f16Arg1);
        Frac16_t f16InstantPower_Ubeta_Ibeta = MulF16(tDrvFoc.tUAlBeReqDCB.f16Arg2, tDrvFoc.tIAlBeFbck.f16Arg2);
        int32_t f16InstantPowerSum_AlphaBeta = ((int32_t)f16InstantPower_Ualpha_Ialpha + (int32_t)f16InstantPower_Ubeta_Ibeta);

        // 乘以3/2系数 (3/2 = 1.5 = FRAC16(0.75) * 2)
        int32_t i32TempPower = (int32_t)f16InstantPowerSum_AlphaBeta * 3;
        i32TempPower = i32TempPower >> 1; // 除以2
        
        Frac16_t f16InstantPower;
        if (i32TempPower > 32767) {
            f16InstantPower = FRAC16(0.9999);
        } else if (i32TempPower < -32768) {
            f16InstantPower = FRAC16(-1.0);
        } else {
            f16InstantPower = (Frac16_t)i32TempPower;
        }

        // 将瞬时功率作为低通滤波器输入
        // g_tIFSpeedFilter.in = f16InstantPower;
        // IQ_LPFilterCalc(&g_tIFSpeedFilter);

        g_tEdampHpFilter.in = f16InstantPower;//g_tIFSpeedFilter.out;;
        IQ_HPFilterCalc(&g_tEdampHpFilter);

        f16EdVibration = g_tEdampHpFilter.out;

        // g_tEdampHpFilter2.in = g_tEdampHpFilter.out;

        // IQ_HPFilterCalc(&g_tEdampHpFilter2);
        // f16EdVibration = g_tEdampHpFilter2.out;

        //f16EdVibration = SatAddF16(f16EdVibration,MulF16(FRAC16(0.9),f16EdVibration)); //减去速度微分部分
      //  Frac16_t f16OmegaComp = MulF16(FRAC16(0.9), f16EdVibration); // 阻尼增益为0.01，可调
        int32_t temp = (int32_t)f16EdVibration * 2048 >> 10; // 阻尼增益为0.01，可调
        // if (tDrvFoc.u32PreSensorlessTime == PERSENSORLESS_WIND_TIME_MS){
        //        temp = 0;
        // }
        // if(tDrvFoc.tPosMode == TRACKING || tDrvFoc.tPosMode == PRESENSORLESS){
        //     temp = 0;
        // }
        Frac16_t f16OmegaComp = 0;
         if (temp > 32767) {
             f16OmegaComp = FRAC16(0.9999);
         } else if (temp < -32768) {
             f16OmegaComp = FRAC16(-1.0);
         } else {
             f16OmegaComp = (Frac16_t)temp;
         }
        // 将补偿速度反馈到速度参考值
        tDrvFoc.tPospeControl.f16wRotElReqIFDeltaW = SatSubF16(tDrvFoc.tPospeControl.f16wRotElReqRamp, f16OmegaComp);
        if(tDrvFoc.f16wRotElReqIFDeltaW < 0){
            tDrvFoc.f16wRotElReqIFDeltaW = tDrvFoc.tPospeControl.f16wRotElReqRamp;
        } else if (tDrvFoc.f16wRotElReqIFDeltaW > FRAC16(600/4000.0)){
            tDrvFoc.f16wRotElReqIFDeltaW = FRAC16(600/4000.0);
        } 
        //---速度高通提供阻尼

   
    } else{
        tDrvFoc.f16wRotElReqIFDeltaW = tDrvFoc.tPospeControl.f16wRotElReqRamp;
    }
#endif
}

void StateReset()
{
    bool rstFcnStatus = true;
    /* entering state */
    tDrvFoc.tAppState.tStatus = RESET;
    tDrvFoc.tAppState.tEvent = E_RESET;

    /* turn off Output*/
    DisableOutput();

    /* reset variable */
    /* initialize alignment parameters*/
    /* initilize calibration parameters*/

#if (FOC_EPWM_SHUNT_MODE == 0)
    ADC->QTR1_b.QTTR             = 100;
    ADC->QTR2_b.QTTR             = 200;
    ADC->QTR3_b.QTTR             = 300;
    ADC->QTR4_b.QTTR             = 400;
#endif

    if (!rstFcnStatus) {
        //@fixme: add fault
    } else {
        /* reset successfully*/
        tDrvFoc.tAppState.tEvent = E_RESET_DONE;
    }
}



/************************************************************
 * @brief: config epwm compare time and ADC trig time.
 * @param <svm_t> *pSvm
 * @return <None>
 ************************************************************/
void SetDuty(void)
{
    /* set pwm compare value */
    EPWM->PWMRWEN_b.PWMRLDEN = 0x55; // enable write register
    EPWM->PWM0DH             = tDrvFoc.tPwmMod.u16PhAEdge;
    EPWM->PWM0DL             = tDrvFoc.tPwmMod.u16PhADbswEdge;
    EPWM->PWM1DH             = tDrvFoc.tPwmMod.u16PhBEdge;
    EPWM->PWM1DL             = tDrvFoc.tPwmMod.u16PhBDbswEdge;
    EPWM->PWM2DH             = tDrvFoc.tPwmMod.u16PhCEdge;
    EPWM->PWM2DL             = tDrvFoc.tPwmMod.u16PhCDbswEdge;
    EPWM->PWMRWEN_b.PWMRLDEN = 0xAA; // enable reload register
    SetAdcTrg();
}

/************************************************************
 * @brief: set adc trigger timing.
 * @param <AdcTrg_t> *pAdcTrg
 * @return <None>
 ************************************************************/
void SetAdcTrg()
{
    ADC->QTR1_b.QTTR = tDrvFoc.tPwmMod.tAdcTrg.u16Ph1Trg1;
    ADC->QTR2_b.QTTR = tDrvFoc.tPwmMod.tAdcTrg.u16Ph2Trg1;
//    ADC->QTR3_b.QTTR = tDrvFoc.tPwmMod.tAdcTrg.u16Ph2Trg2;
//    ADC->QTR4_b.QTTR = tDrvFoc.tPwmMod.tAdcTrg.u16Ph1Trg2;
}

/************************************************************
 * @brief: Fetch adc raw data
 * @return <None>
 ************************************************************/
static __inline void GetAdcRaw(void)
{
    tDrvFoc.i16AdcRaw[0] = ADC->DR1_b.DATA;
    tDrvFoc.i16AdcRaw[1] = ADC->DR2_b.DATA;
    tDrvFoc.i16AdcRaw[2] = ADC->DR3_b.DATA;
    tDrvFoc.i16AdcRaw[3] = ADC->DR4_b.DATA;
    tDrvFoc.i16AdcRaw[4] = ADC->DR5_b.DATA;
    tDrvFoc.i16AdcRaw[5] = ADC->DR6_b.DATA; //MOS管温度
    tDrvFoc.i16AdcRaw[6] = ADC->DR7_b.DATA;
    tDrvFoc.i16AdcRaw[7] = ADC->DR8_b.DATA;
}


/**
 * @brief 初始化IQ限制控制器
 * @param pCtrl 指向IQ限制控制器的指针
 * @param f16IdcLimit 母线电流限制值 (Q15格式)
 * @param f16PowerLimit 功率限制值 (Q15格式)
 * @param f16LimitStep IQ限制步长 (Q15格式)
 * @param f16RecoveryStep IQ恢复步长 (Q15格式)
 */
void IQ_LimitCtrl_Init(IQ_Limit_Ctrl_t* pCtrl, 
                       Frac16_t f16IdcLimit, 
                       Frac16_t f16PowerLimit,
                       Frac16_t f16LimitStep,
                       Frac16_t f16RecoveryStep)
{
    // 设置限制值
    pCtrl->f16IdcLimit = f16IdcLimit;
    pCtrl->f16PowerLimit = f16PowerLimit;
    
    // 设置控制参数
    pCtrl->f16IqLimitStep = f16LimitStep;
    pCtrl->f16IqRecoveryStep = f16RecoveryStep;
    
    // 初始化状态
    pCtrl->f16IqLimitRatio = FRAC16(1.0);  // 初始不限制
    pCtrl->f16IqLimit = FRAC16(1.0);  
    pCtrl->bIdcOverLimit = false;
    pCtrl->bPowerOverLimit = false;
    pCtrl->bLimitActive = false;
    
    // 初始化滤波器 - 用于平滑限制比例变化
    pCtrl->tLimitFilter.f16NSample = 4;    // 滤波器系数，可调整
    pCtrl->tLimitFilter.f32Acc = FRAC16(1.0) << 15;  // 初始值为1.0

    pCtrl->u16LimitCtrlDivCnt = 0;
}


/**
 * @brief IQ限制控制主函数
 * @param pCtrl 指向IQ限制控制器的指针
 * @param f16IdcCurrent 当前母线电流值 (Q15格式)
 * @param f16PowerCurrent 当前功率值 (Q15格式) 
 * @param f16IqInput 输入的IQ值 (速度环输出)
 * @return 限制后的IQ值
 */
Frac16_t IQ_LimitCtrl_Process(IQ_Limit_Ctrl_t* pCtrl,
                                                                  Frac16_t f16IdcCurrent,
                                                                  Frac16_t f16PowerCurrent,
                                                                  Frac16_t f16IqInput)
{
    Frac16_t f16IqOutput;
    Frac16_t f16TargetRatio = FRAC16(1.0);  // 目标限制比例
    
    // 更新当前值
    pCtrl->f16IdcCurrent = f16IdcCurrent;
    pCtrl->f16PowerCurrent = f16PowerCurrent;
    
    // 检查母线电流是否超限
    if (AbsF16(f16IdcCurrent) > pCtrl->f16IdcLimit) {
        pCtrl->bIdcOverLimit = true;
    } else {
        // 增加回差，防止震荡
        if ( pCtrl->bIdcOverLimit == true){
            if (AbsF16(f16IdcCurrent) < MulF16(pCtrl->f16IdcLimit, (FRAC16(1.0) - CURRENT_POWER_LIMIT_HYSTERESIS))) {
                pCtrl->bIdcOverLimit = false;
            }
        }
    }
    
    // 检查功率是否超限 (只考虑正功率)
    if (f16PowerCurrent > pCtrl->f16PowerLimit) {
        pCtrl->bPowerOverLimit = true;
    } else {
        // 增加回差，防止震荡
        if ( pCtrl->bPowerOverLimit == true){
            if (f16PowerCurrent < MulF16(pCtrl->f16PowerLimit, (FRAC16(1.0) - CURRENT_POWER_LIMIT_HYSTERESIS))) {
                pCtrl->bPowerOverLimit = false;
            }
        }
    }
    if (pCtrl->u16LimitCtrlDivCnt++ >= POWER_CIRCUIT_IQ_LOOP_DIV){

        
        // 确定是否需要激活限制
        if (pCtrl->bIdcOverLimit || pCtrl->bPowerOverLimit) {
            pCtrl->bLimitActive = true;
            if(pCtrl->bLastLimitActive == false){
                // 第一次进入限制，快速降低到当前IQ水平
                pCtrl->f16IqLimit = f16IqInput;
            }
            pCtrl->f16IqLimit = SatSubF16(pCtrl->f16IqLimit,pCtrl->f16IqLimitStep);
            if (pCtrl->f16IqLimit < 0) pCtrl->f16IqLimit = 0;
        } else {
            // 没有超限，逐步恢复
            pCtrl->bLimitActive = false;
            pCtrl->f16IqLimit = SatAddF16(pCtrl->f16IqLimit,pCtrl->f16IqRecoveryStep);
        }
        pCtrl->u16LimitCtrlDivCnt = 0;
        pCtrl->bLastLimitActive = pCtrl->bLimitActive;
    }
    
    // 对限制比例进行滤波，使变化更平滑
    // Frac16_t f16FilteredRatio = GCF_LpFilterMA_F16(pCtrl->f16IqLimitRatio, &pCtrl->tLimitFilter);
    
    // 应用限制到IQ
    // f16IqOutput = MulF16(f16IqInput, pCtrl->f16IqLimitRatio);
    if (f16IqInput > pCtrl->f16IqLimit){
        f16IqOutput = pCtrl->f16IqLimit;
    } else {
        f16IqOutput = f16IqInput;
    }
    
    return f16IqOutput;
}

/**
 * @brief 获取IQ限制状态信息
 * @param pCtrl 指向IQ限制控制器的指针
 * @return 限制状态位域: bit0-母线电流超限, bit1-功率超限, bit2-限制激活
 */
uint8_t IQ_LimitCtrl_GetStatus(IQ_Limit_Ctrl_t* pCtrl)
{
    uint8_t u8Status = 0;
    
    if (pCtrl->bIdcOverLimit) {
        u8Status |= 0x01;
    }
    
    if (pCtrl->bPowerOverLimit) {
        u8Status |= 0x02;
    }
    
    if (pCtrl->bLimitActive) {
        u8Status |= 0x04;
    }
    
    return u8Status;
}

void InitFocParas(void)
{
    u8EnableOut = 0;
    /* Initialize alignment parameters*/
    tDrvFoc.u16AlignCntr    = 0;
    tDrvFoc.u16AlignVoltage = tFocParas.ALIGN_VOLTAGE;;

    /* Initialize calibration parameters*/
    tDrvFoc.u16CalibCntr = 0;
    tDrvFoc.i32CalibSum  = 0;
    tDrvFoc.i32CalibSum2  = 0;

//    for (uint8_t i = 0; i < 8; i++) {
//        tDrvFoc.i16AdcRaw[i] = 0;
//    }

    /* Initialize position mode */
    tDrvFoc.tPosMode = FORCE;

    /* Initialize current */
    tDrvFoc.tIabcFbck.f16Arg1 = 0;
    tDrvFoc.tIabcFbck.f16Arg2 = 0;
    tDrvFoc.tIabcFbck.f16Arg3 = 0;

    tDrvFoc.tIAlBeFbck.f16Arg1 = 0;
    tDrvFoc.tIAlBeFbck.f16Arg2 = 0;

    tDrvFoc.tIDQFbck.f16Arg1 = 0;
    tDrvFoc.tIDQFbck.f16Arg2 = 0;

    tDrvFoc.tIDQErr.f16Arg1 = 0;
    tDrvFoc.tIDQErr.f16Arg2 = 0;

    tDrvFoc.tIDQReq.f16Arg1 = 0;
    tDrvFoc.tIDQReq.f16Arg2 = 0;

    /* Initialize voltage */
    tDrvFoc.tUDQReq.f16Arg1 = 0;
    tDrvFoc.tUDQReq.f16Arg2 = 0;

    tDrvFoc.tUAlBeReq.f16Arg1 = 0;
    tDrvFoc.tUAlBeReq.f16Arg2 = 0;

    /* Initialize polar transformation */
    tDrvFoc.tThetaTransform.f16Sin = 0;
    tDrvFoc.tThetaTransform.f16Cos = 0;

    /* Initialize SVM */
    tDrvFoc.tSvm.u16Sec           = 0;
    tDrvFoc.tSvm.tPwmDuty.f16Arg1 = 0;
    tDrvFoc.tSvm.tPwmDuty.f16Arg2 = 0;
    tDrvFoc.tSvm.tPwmDuty.f16Arg3 = 0;
    spwmVM.tPwmDuty = tDrvFoc.tSvm.tPwmDuty;

    
    /* Initialize Modulator*/
    tDrvFoc.tPwmMod.u16PhAEdge = 0;
    tDrvFoc.tPwmMod.u16PhADbswEdge = 0;
    tDrvFoc.tPwmMod.u16PhBEdge = 0;
    tDrvFoc.tPwmMod.u16PhBDbswEdge = 0;
    tDrvFoc.tPwmMod.u16PhCEdge = 0;
    tDrvFoc.tPwmMod.u16PhCDbswEdge = 0;
    
    tDrvFoc.tPwmMod.tAdcTrg.u16Ph1Trg1 = 0;
    tDrvFoc.tPwmMod.tAdcTrg.u16Ph2Trg1 = 50;
    tDrvFoc.tPwmMod.tAdcTrg.u16Ph2Trg2 = 100;
    tDrvFoc.tPwmMod.tAdcTrg.u16Ph1Trg2 = 150;
    
    /* Initialize FOC D-axis controller */
    tDrvFoc.tAxisDCtrl.f32Acc        = 0;
    tDrvFoc.tAxisDCtrl.f16CC1sc      = tFocParas.D_CC1SC;
    tDrvFoc.tAxisDCtrl.f16CC2sc      = tFocParas.D_CC2SC;
    tDrvFoc.tAxisDCtrl.f16InErrK1    = 0;
    tDrvFoc.tAxisDCtrl.f16UpperLimit = tFocParas.CLOOP_LIMIT;
    tDrvFoc.tAxisDCtrl.f16LowerLimit = -tFocParas.CLOOP_LIMIT;
    tDrvFoc.tAxisDCtrl.u16NShift     = tFocParas.D_NSHIFT;

    /* Initialize FOC Q-axis controller */
    tDrvFoc.tAxisQCtrl.f32Acc        = 0;
    tDrvFoc.tAxisQCtrl.f16CC1sc      = tFocParas.Q_CC1SC;
    tDrvFoc.tAxisQCtrl.f16CC2sc      = tFocParas.Q_CC2SC;
    tDrvFoc.tAxisQCtrl.f16InErrK1    = 0;
    tDrvFoc.tAxisQCtrl.f16UpperLimit = tFocParas.CLOOP_LIMIT;
    tDrvFoc.tAxisQCtrl.f16LowerLimit = -tFocParas.CLOOP_LIMIT;
    tDrvFoc.tAxisQCtrl.u16NShift     = tFocParas.Q_NSHIFT;

    /* Initialize speed ramp up/down */
    tDrvFoc.tSpeedRamp.f16State    = 0;
    tDrvFoc.tSpeedRamp.f16RampUp   = tFocParas.SPEED_RAMP_UP;
    tDrvFoc.tSpeedRamp.f16RampDown = tFocParas.SPEED_RAMP_DOWN;

    tDrvFoc.tPospeSensorless.f16wRotElMatch_1 = tFocParas.MERG_SPEED_1_TRH;
    tDrvFoc.tPospeSensorless.f16wRotElMatch_2 = tFocParas.MERG_SPEED_2_TRH;

    /* Initialize FOC Speed controller */
    tDrvFoc.tSpeedCtrl.f16PropGain       = tFocParas.SPEED_PI_PROP_GAIN;
    tDrvFoc.tSpeedCtrl.f16IntegGain      = tFocParas.SPEED_PI_INTEG_GAIN;
    tDrvFoc.tSpeedCtrl.s16PropGainShift  = tFocParas.SPEED_PI_PROP_SHIFT;
    tDrvFoc.tSpeedCtrl.s16IntegGainShift = tFocParas.SPEED_PI_INTEG_SHIFT;
    tDrvFoc.tSpeedCtrl.f16UpperLimit     = tFocParas.SPEED_LOOP_HIGH_LIMIT;
    tDrvFoc.tSpeedCtrl.f16LowerLimit     = tFocParas.SPEED_LOOP_LOW_LIMIT;
    tDrvFoc.tSpeedCtrl.f16InK_1          = 0;
    tDrvFoc.tSpeedCtrl.f32IntegPartK_1   = 0;

    /* Initialize open loop */
    tDrvFoc.tPospeOpenloop.tOplpInteg.f16InK1   = 0;
    tDrvFoc.tPospeOpenloop.tOplpInteg.f32State  = 0;
    tDrvFoc.tPospeOpenloop.tOplpInteg.f16C1     = tFocParas.TO_THETA_GAIN;
    tDrvFoc.tPospeOpenloop.tOplpInteg.u16NShift = tFocParas.TO_THETA_SHIFT;

    tDrvFoc.tPospeOpenloop.f16ThetaRotEl   = 0;
    tDrvFoc.tPospeOpenloop.f16wRotE1       = 0;
    tDrvFoc.tPospeOpenloop.f16IQUpperLimit = tFocParas.OL_START_I;
    tDrvFoc.tPospeOpenloop.f16IQLowerLimit = -tFocParas.OL_START_I;

    /* speed filter*/
    tDrvFoc.tPospeControl.twRotFilter.f16NSample = tFocParas.POSPE_SPEED_FILTER_MA_NPOINT;
    tDrvFoc.tPospeControl.twRotFilter.f32Acc     = 0;

    tDrvFoc.tPospeControl.f16wRotElReqRamp = 0;

    /* Initialize observer */
    /* bemf d-axis*/
    tDrvFoc.tPospeSensorless.tBemfObsvr.tDeltaPIEmfCtrl.f16CC1sc      = tFocParas.BEMF_DQ_CC1_GAIN;
    tDrvFoc.tPospeSensorless.tBemfObsvr.tDeltaPIEmfCtrl.f16CC2sc      = tFocParas.BEMF_DQ_CC2_GAIN;
    tDrvFoc.tPospeSensorless.tBemfObsvr.tDeltaPIEmfCtrl.u16NShift     = tFocParas.BEMF_DQ_NSHIFT;
    tDrvFoc.tPospeSensorless.tBemfObsvr.tDeltaPIEmfCtrl.f16UpperLimit = FRAC16(0.9888);
    tDrvFoc.tPospeSensorless.tBemfObsvr.tDeltaPIEmfCtrl.f16LowerLimit = FRAC16(-0.9888);
    tDrvFoc.tPospeSensorless.tBemfObsvr.tDeltaPIEmfCtrl.f32Acc        = 0;
    tDrvFoc.tPospeSensorless.tBemfObsvr.tDeltaPIEmfCtrl.f16InErrK1    = 0;

    /* bemf q-axis*/
    tDrvFoc.tPospeSensorless.tBemfObsvr.tGammaPIEmfCtrl.f16CC1sc      = tFocParas.BEMF_DQ_CC1_GAIN;
    tDrvFoc.tPospeSensorless.tBemfObsvr.tGammaPIEmfCtrl.f16CC2sc      = tFocParas.BEMF_DQ_CC2_GAIN;
    tDrvFoc.tPospeSensorless.tBemfObsvr.tGammaPIEmfCtrl.u16NShift     = tFocParas.BEMF_DQ_NSHIFT;
    tDrvFoc.tPospeSensorless.tBemfObsvr.tGammaPIEmfCtrl.f16UpperLimit = FRAC16(0.9888);
    tDrvFoc.tPospeSensorless.tBemfObsvr.tGammaPIEmfCtrl.f16LowerLimit = FRAC16(-0.9888);
    tDrvFoc.tPospeSensorless.tBemfObsvr.tGammaPIEmfCtrl.f32Acc        = 0;
    tDrvFoc.tPospeSensorless.tBemfObsvr.tGammaPIEmfCtrl.f16InErrK1    = 0;

    /* bemf observer parameters */
    tDrvFoc.tPospeSensorless.tBemfObsvr.f16GainU  = tFocParas.U_GAIN;
    tDrvFoc.tPospeSensorless.tBemfObsvr.f16GainE  = tFocParas.E_GAIN;
    tDrvFoc.tPospeSensorless.tBemfObsvr.f16GainWI = tFocParas.WI_GAIN;
    tDrvFoc.tPospeSensorless.tBemfObsvr.f16GainI  = tFocParas.I_GAIN;
    tDrvFoc.tPospeSensorless.tBemfObsvr.f16ScaleK = tFocParas.BEMF_SHIFT;

    /* tracking observer controller */
    tDrvFoc.tPospeSensorless.tBemfObsvr.tTrackObsvr.tPllCtrl.f16CC1sc      = tFocParas.TO_CC1SC;
    tDrvFoc.tPospeSensorless.tBemfObsvr.tTrackObsvr.tPllCtrl.f16CC2sc      = tFocParas.TO_CC2SC;
    tDrvFoc.tPospeSensorless.tBemfObsvr.tTrackObsvr.tPllCtrl.u16NShift     = tFocParas.TO_NSHIFT;
    tDrvFoc.tPospeSensorless.tBemfObsvr.tTrackObsvr.tPllCtrl.f16UpperLimit = FRAC16(0.988);
    tDrvFoc.tPospeSensorless.tBemfObsvr.tTrackObsvr.tPllCtrl.f16LowerLimit = FRAC16(-0.988);
    tDrvFoc.tPospeSensorless.tBemfObsvr.tTrackObsvr.tPllCtrl.f32Acc        = 0;
    tDrvFoc.tPospeSensorless.tBemfObsvr.tTrackObsvr.tPllCtrl.f16InErrK1    = 0;

    /* tracking observer Integrator */
    tDrvFoc.tPospeSensorless.tBemfObsvr.tTrackObsvr.tPllInte.f16C1     = tFocParas.TO_THETA_GAIN;
    tDrvFoc.tPospeSensorless.tBemfObsvr.tTrackObsvr.tPllInte.u16NShift = tFocParas.TO_THETA_SHIFT;

    tDrvFoc.tPospeSensorless.tBemfObsvr.tTrackObsvr.tFilter.f16NSample = tFocParas.POSPE_SPEED_FILTER_MA_NPOINT;
    tDrvFoc.tPospeSensorless.tBemfObsvr.tTrackObsvr.tFilter.f32Acc     = 0;

    AMC_InitObsrv_F16(&tDrvFoc.tPospeSensorless.tBemfObsvr);

    /* sensorless limittaition*/
    tDrvFoc.tPospeSensorless.f16IQUpperLimit = tFocParas.SPEED_LOOP_HIGH_LIMIT;
    tDrvFoc.tPospeSensorless.f16IQLowerLimit = tFocParas.SPEED_LOOP_LOW_LIMIT;

    /*DC bus compsenste*/
    tDrvFoc.tDcBusComp.f16ArgDcBusMsr = 0;
    tDrvFoc.tDcBusComp.f16ModIndex    = FRAC16(0.866025403784439);

    /* Pro & Con wild detection*/
    tDrvFoc.tProConWind.u16ProConDetectCnt = 0;
    tDrvFoc.tProConWind.tWDIR              = NOWIND;
    tDrvFoc.tProConWind.u16ContsCnt        = 0;

    tDrvFoc.tIRaw1Filter.f16NSample    = 3; // cut frequency:
    tDrvFoc.tIRaw1Filter.f32Acc        = 0;
    tDrvFoc.tIRaw2Filter.f16NSample    = 2; // cut frequency:
    tDrvFoc.tIRaw2Filter.f32Acc        = 0;
    tDrvFoc.tDcBusFilter.f16NSample    = tFocParas.DC_BUS_FILTER_MA_NPOINT; // cut frequency:
    tDrvFoc.tDcBusFilter.f32Acc        = 0;
    tDrvFoc.tExtSpeedFilter.f16NSample = 6; // cut frequency:
    tDrvFoc.tExtSpeedFilter.f32Acc     = 0;
    
    tDrvFoc.tIaFilter.f16NSample    = 2; // cut frequency:
    tDrvFoc.tIaFilter.f32Acc        = 0;
    tDrvFoc.tIbFilter.f16NSample    = 2; // cut frequency:
    tDrvFoc.tIbFilter.f32Acc        = 0;
    tDrvFoc.tIcFilter.f16NSample    = 1; // cut frequency:
    tDrvFoc.tIcFilter.f32Acc        = 0;
    
    tDrvFoc.u16NormalRunTime = 1;
    
    /* Init Current Reconstruction */
#if FOC_EPWM_SHUNT_MODE == 0
    g_u16SampleSlot = 96; //250
    InitMotorParameterAIWIN(&tCurrSample1rps,&tCurrAuxInfo);
#endif
    
    /*windmilling init*/
    WM_Init(&tDrvFoc.tWMObsvr, tDrvFoc.i16AdcRaw[4]);
    DeadTimeCompensationInit(&tDeadTimeComp);
    /*flux weakening init */
    AMC_FWInit_F16(&tDrvFoc.tFluxWeakening, &tDrvFoc.tIDQFbck, 
                    &tDrvFoc.tUDQReq, &tDrvFoc.tAxisQCtrl);
    /*stall Init*/
    stallDetectionInit(&tStallDetection);
    OL_SpeedRampInc = tFocParas.OL_START_RAMP_INC;
    CL_SpeedRampInc = tFocParas.SPEED_RAMP_UP;
    CL_SpeedRampDec = tFocParas.SPEED_RAMP_DOWN;

    motorInitCalculations(&tMotorCalculations);



    tDrvFoc.tPospeControl.i16SpeedLoopCntr = 10;


    
    
    f16IQFiltered = 0;




    spwmVM = tDrvFoc.tSvm;

    tDrvFoc.u16HfiCntr = 0;

    
    tHfiCtrl.tHFIAxisDCtrl.f16CC1sc = tFocParas.HFI_D_CC1SC; // D轴电流控制器带宽
    tHfiCtrl.tHFIAxisDCtrl.f16CC2sc = tFocParas.HFI_D_CC2SC; //
    tHfiCtrl.tHFIAxisDCtrl.u16NShift = tFocParas.HFI_D_NSHIFT;
    tHfiCtrl.tHFIAxisDCtrl.f16UpperLimit = tFocParas.HFI_CLOOP_LIMIT;
    tHfiCtrl.tHFIAxisDCtrl.f16LowerLimit = -tFocParas.HFI_CLOOP_LIMIT;

    tHfiCtrl.tHFIAxisDCtrl.f32Acc = 0 ;
    tHfiCtrl.tHFIAxisDCtrl.f16InErrK1 = 0;

    tHfiCtrl.tHFIAxisQCtrl.f16CC1sc = tFocParas.HFI_Q_CC1SC; // Q轴电流控制器带宽
    tHfiCtrl.tHFIAxisQCtrl.f16CC2sc = tFocParas.HFI_Q_CC2SC; //
    tHfiCtrl.tHFIAxisQCtrl.u16NShift = tFocParas.HFI_Q_NSHIFT;
    tHfiCtrl.tHFIAxisQCtrl.f16UpperLimit = tFocParas.HFI_CLOOP_LIMIT;
    tHfiCtrl.tHFIAxisQCtrl.f16LowerLimit = -tFocParas.HFI_CLOOP_LIMIT;
    tHfiCtrl.tHFIAxisQCtrl.f32Acc = 0 ;
    tHfiCtrl.tHFIAxisQCtrl.f16InErrK1 = 0;

    tHfiCtrl.tDcBusComp = tDrvFoc.tDcBusComp;
    tHfiCtrl.tSpeedFilter =  tDrvFoc.tPospeSensorless.tBemfObsvr.tTrackObsvr.tFilter;

    PARAM_ID_Init(&tMotorParamId);

    DQ_DualSpaceBlendReset();

    ePrevPosModeSlowLoop = FORCE;
    tHfiCtrl.tSpeedCtrl.f16PropGain       = tFocParas.HFI_SPEED_PI_PROP_GAINL;
    tHfiCtrl.tSpeedCtrl.f16IntegGain      = tFocParas.HFI_SPEED_PI_INTEG_GAIN;
    tHfiCtrl.tSpeedCtrl.s16PropGainShift  = tFocParas.HFI_SPEED_PI_PROP_SHIFT;
    tHfiCtrl.tSpeedCtrl.s16IntegGainShift = tFocParas.HFI_SPEED_PI_INTEG_SHIFT;
    tHfiCtrl.tSpeedCtrl.f16UpperLimit     = tFocParas.HFI_SPEED_LOOP_HIGH_LIMIT;
    tHfiCtrl.tSpeedCtrl.f16LowerLimit     = tFocParas.HFI_SPEED_LOOP_LOW_LIMIT;
    tHfiCtrl.tSpeedCtrl.f16InK_1          = 0;
    tHfiCtrl.tSpeedCtrl.f32IntegPartK_1   = 0;

    tHfiCtrl.bIsFirstHfiToBemf = false;

    IQ_HPFilterInit(&g_tEdampHpFilter);
    IQ_LPFilterInit(&g_tIqFbackLpFilter);
    IQ_LPFilterInit(&g_tIdFbackLpFilter);

//    IQ_HPFilterInit(&g_tEdampHpFilter2);

//    IQ_LPFilterInit(&g_tIFSpeedFilter);

//    IQ_LPFilterInit(&g_tOmegaDiffLpf);
    // 滤波iq,id
    IQ_LPFilterSetCutoffHz(&g_tIqFbackLpFilter, FOC_EPWM_FREQUENCY, 20); 
    IQ_LPFilterParaCalc(&g_tIqFbackLpFilter);
    IQ_LPFilterSetCutoffHz(&g_tIdFbackLpFilter, FOC_EPWM_FREQUENCY, 20); 
    IQ_LPFilterParaCalc(&g_tIdFbackLpFilter);

    tDrvFoc.u32PreSensorlessCnt = 0;
    
    
    tDrvFoc.u32PreSensorlessTime = PERSENSORLESS_TIME_MS;

    f16AccelFeedforward = 0;

    tDrvFoc.f16wRotElReqIFDeltaW = 0;


    tCurrSample1rps.UVWCurrRecon.IuOffset = 0;
    tCurrSample1rps.UVWCurrRecon.IvOffset = 0;

    tDrvFoc.tSvm.svmMode = SVM_MODE_SEVEN_SEGMENT;//复位，否则高转速停机后一上电就是五段式，低转速时使用五段式会启动失败

    tDrvFoc.u16ReduceIQControllCnt = 0;
#if (ENABLE_CURRENT_POWER_LIMIT == 1)
    IQ_LimitCtrl_Init(&tIqLimitCtrl,
                    BUS_CURRENT_LIMIT,    // 母线电流限制值 (相对于满量程的80%)
                    POWER_LIMIT,    // 功率限制值 (相对于满量程的90%)
                    DERATING_IQ_STEP,   // IQ限制步长 (每次降低1%)
                    RECOVER_IQ_STEP); // IQ恢复步长 (每次恢复0.2%)
#endif

    // head_wind_pll_init(&tWmObsvr_PLL);

#if (ENABLE_PR_CTRL)
    {
        // IQ_HPFilterSetCutoffHz(&g_tIqErrHpFilter, FOC_EPWM_FREQUENCY, 5); // 5Hz截止频率

        // 以当前速度估算初始中心频率，低速时用一个安全的最小值
        uint16_t f0 = 10;
        if (f0 < PR_MIN_FREQ_HZ) f0 = PR_MIN_FREQ_HZ;

        // 速度环PR初始化
        Frac16_t wc_speed = MulQ15_By_IntmHz(PR_SPEED_ALPHA, (f0 * 1000));
        qprFxpInit(&g_qpr_speed, PR_SPEED_KR, wc_speed, (f0 * 1000), (FOC_EPWM_FREQUENCY / 10), PR_SPEED_YLIM);
        qprFxpSetWcByAlphamhz(&g_qpr_speed, PR_SPEED_ALPHA,(f0 * 1000), (FOC_EPWM_FREQUENCY / 10));
        
        wc_speed = MulQ15_By_IntmHz(PR_SPEED_ALPHA, (f0 * 1000));
        qprFxpInit(&g_qpr3_speed, PR_SPEED_KR, wc_speed, (f0 * 1000), (FOC_EPWM_FREQUENCY / 10), PR_SPEED_YLIM);
        qprFxpSetWcByAlphamhz(&g_qpr3_speed, PR_SPEED_ALPHA,(f0 * 1000), (FOC_EPWM_FREQUENCY / 10));

        // // 电流环PR初始化
        Frac16_t wc_iq = MulQ15_By_IntmHz(PR_IQ_ALPHA,(f0 * 1000));
        Frac16_t kr = 0;
        kr = FRAC16(0.3);//-tFocParas.Q_CC2SC * 1666;
        qprFxpInit(&g_qpr_iq, kr, wc_iq, (f0 * 1000), (FOC_EPWM_FREQUENCY / 5), PR_IQ_YLIM);

        Frac16_t wc_id = MulQ15_By_IntmHz(PR_ID_ALPHA,(f0 * 1000));
        kr = FRAC16(0.3);//-tFocParas.D_CC2SC * 1666;
        qprFxpInit(&g_qpr_id, kr, wc_id, (f0 * 1000), (FOC_EPWM_FREQUENCY / 5), PR_ID_YLIM);

    }
#endif
}


//初始化只需要一次配置的参数
void InitMotorConfig(void)
{
    
    tMotorCalculations.f16Idc2acEfficiency = I_DC2AC_EFFICIENCY;
    tMotorCalculations.f16SpeedFactor = SPEED_FACTOR;
    tMotorCalculations.f16DeadTimeFactor = DEAD_TIME_FACTOR;
    tMotorCalculations.i32DeadTimeNs = DEAD_TIME_NS;
    tMotorCalculations.i32TonNs = TON_NS;
    tMotorCalculations.i32ToffNs = TOFF_NS;
    tMotorCalculations.i32pwmPeriodNs = PWM_PERIOD_NS;



    tDrvFoc.tSvm.bEnableFiveSegment = true;
    tDrvFoc.tSvm.f16ThresholdToFive = FRAC16(0.5); //在13.5V大概2000转左右
    tDrvFoc.tSvm.f16Hysteresis = FRAC16(0.05);

    DQ_DualSpaceSetBlendStep(FRAC16(0.1));

    tDrvFoc.tFluxWeakening.pVsMargin = FRAC16(0.9);
    tDrvFoc.tFluxWeakening.pIQLim = tFocParas.SPEED_LOOP_HIGH_LIMIT;

    InitFreqToAngleIncrement(FREQUENCY_BASE,FOC_EPWM_FREQUENCY);


    tCurrAuxInfo.useMeasureVector = USE_MEASURE_VECTOR;
    tCurrAuxInfo.SpecialMode = ONE_RESISTOR_SAMPLING_USE_HISTORY;
    tCurrAuxInfo.f16switchSpeed = ONE_RESISTOR_SAMPLING_USE_DQ_SPEED;
    tCurrAuxInfo.f16switchSpeedHysteresis = ONE_RESISTOR_SAMPLING_DQ_SPEED_HYSTERESIS;
    tCurrAuxInfo.dqPredictActive = 0;
    tCurrAuxInfo.CompTime = ONE_RESISTOR_SAMPLING_DQ_PREDICT_TIMES;

#if (ENABLE_DAMPING_COMPENSATION == 1)
    // 默认将高通截止频率设为约700Hz（可按需修改/运行时调用接口调整）
    IQ_HPFilterSetCutoffHz(&g_tEdampHpFilter, FOC_EPWM_FREQUENCY, 1); 
    IQ_HPFilterParaCalc(&g_tEdampHpFilter);

    // IQ_HPFilterSetCutoffHz(&g_tEdampHpFilter2, FOC_EPWM_FREQUENCY, 1);
    // IQ_HPFilterParaCalc(&g_tEdampHpFilter2);


    // IQ_LPFilterSetCutoffHz(&g_tIFSpeedFilter, FOC_EPWM_FREQUENCY,100);
    // IQ_LPFilterParaCalc(&g_tIFSpeedFilter);

    // IQ_LPFilterSetCutoffHz(&g_tOmegaDiffLpf, FOC_EPWM_FREQUENCY, 100);
    // IQ_LPFilterParaCalc(&g_tOmegaDiffLpf);
#endif

    // VariablePI_Init(&tVariableSpeedCtrl,
    //                 FRAC16(0.5), 8,          // ap = 1.6 - 减小基础比例增益
    //                 FRAC16(0.85), -8,        // ai = 0.027 - 保持积分增益
    //                 FRAC16(0.6), 5,          // bp = 2.4 - 大误差时额外增益
    //                 FRAC16(0.8), 5,          // cp = 0.8 - 比例项衰减系数
    //                 FRAC16(0.6), 1);         // ci = 1.2 - 积分项衰减系数
    
    // // 设置输出限幅
    // VariablePI_SetLimits(&tVariableSpeedCtrl, 
    //                     tFocParas.SPEED_LOOP_HIGH_LIMIT,
    //                     tFocParas.SPEED_LOOP_LOW_LIMIT);

}




void InitFocStatus(void)
{
    /* Initialize alignment parameters*/
    tDrvFoc.u16AlignCntr    = 0;
    tDrvFoc.u16AlignVoltage = tFocParas.ALIGN_VOLTAGE;

    /* Initialize calibration parameters*/
    tDrvFoc.u16CalibCntr = 0;
    tDrvFoc.i32CalibSum  = 0;
}

void FOC_1MSTASK(void)
{
    if (tDrvFoc.bInFaultDelay) {
        if (tDrvFoc.u32FaultRestartDelayTimer > 0) {
            tDrvFoc.u32FaultRestartDelayTimer--;
            // 此时电机应保持停机状态，不处理其他逻辑或只处理必要的安全逻辑
            // 确保 StateFault() 或类似函数被调用以保持电机停机
            if (tDrvFoc.tAppState.tStatus != FAULT) { // 确保在延迟期间状态是FAULT
                tDrvFoc.tAppState.tStatus = FAULT; // 强制进入故障状态以执行停机
                // 可能需要再次调用 StateFault() 来确保输出被禁用/刹车
                StateFault();
            }
            return; // 在延迟期间不执行其他1ms任务
        } else {
            // 延迟结束，尝试清除故障并重启
            tDrvFoc.bInFaultDelay = false;
            //tDrvFoc.tAppState.tEvent = E_FAULT_CLEAR; // 尝试清除故障
            // 清除故障后，状态机应该会流转到 RESET 或 CALIB 状态
            // 确保 motor_state_ctrl.c 中的 HandleLinCmd/HandlePwmCmd 在 E_FAULT_CLEAR 后能正确响应
            // 并重新获取速度指令。
            // 如果需要，可以强制转换到 RESET 状态：
            // tDrvFoc.tAppState.tStatus = RESET;
            // tDrvFoc.tAppState.tEvent = E_RESET; 
            // pStateFuncTable[tDrvFoc.tAppState.tEvent][tDrvFoc.tAppState.tStatus](); // 手动调用状态函数
        }
    }
}


