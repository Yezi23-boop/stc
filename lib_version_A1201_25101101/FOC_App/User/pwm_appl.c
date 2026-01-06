/****************************************************************************
 * @file    : drv_pwmio.c
 * @author  : CJL
 * @version : V1.0
 * @Date    : 2025/2/25
 * @brief   : pwmio driver，支持1HZ~10kHz的PWM输入信号
 * @note
 * Copyright (C) 2025 aiwin  All rights reserved.
 ****************************************************************************/
#include "Module_Define.h"

#define PWMIO_INT_PRIO            (6)         // PWMIO interrupt priority
#define PWMIO_CLEAR_IRE_INT       (1 << 0)    // Clear PWMIO IRE interrupt flag
#define PWMIO_CLEAR_IFE_INT       (1 << 1)    // Clear PWMIO IFE interrupt flag
#define PWMIO_CLEAR_ICOF_INT      (1 << 2)    // Clear PWMIO ICOF interrupt flag
//PWM频率上限 400HZ 留下余量 到500HZ
#define PWMIO_MAX_FREQ            (400)       // 400Hz
//频率转化成us
#define PWMIO_MAX_FREQ_US        (1000000 / PWMIO_MAX_FREQ) // 2.5ms
//频率下限 8HZ 留下余量 到5HZ
#define PWMIO_MIN_FREQ            (8)         // 8Hz
//频率转化成us
#define PWMIO_MIN_FREQ_US        (1000000 / PWMIO_MIN_FREQ) // 200ms


/*
 * PWM控制状态枚举定义
 */
typedef enum {
    CFM_STATE_ZERO = 0,      // 停止状态 (0 rpm)
    CFM_STATE_MIN,           // 最低转速 (600 rpm)
    CFM_STATE_LINEAR,        // 线性控制区 (600-2250 rpm)
    CFM_STATE_NOMINAL,       // 额定转速 (2250 rpm)
    CFM_STATE_MAX,           // 最高转速 (2700 rpm)
    CFM_STATE_ERROR,         // 异常状态 (10s后全速运行)
} CFM_State_t;  

/*
 * PWM控制状态结构体
 */
typedef struct {
    CFM_State_t current_state;   // 当前状态 (枚举值)
    bool kl15_status;            // KL15点火信号状态  预留KL15状态配置
} CFM_Control_t;
CFM_Control_t g_PWM_CFM_State_t;   ////PWM转速状态




/*
 * @brief: PWMIO over flow times
 */
uint32_t g_u32OverflowCnt = 0;

/*
 * @brief: PWMIO high level cnt
 */
uint32_t g_u32HighLevelCnt = 0;

/*
 * @brief: PWMIO high level time (us)
 */
uint32_t g_u32HighLevelTime = 0;

/*
 * @brief: PWMIO low level cnt
 */
uint32_t g_u32LowLevelCnt = 0;

/*
 * @brief: PWMIO low level time (us)
 */
uint32_t g_u32LowLevelTime = 0;

/*
 * @brief: PWMIO duty cycle (0.1%), range: 0-1000
 */
uint32_t g_u32DutyCycle = 0;

/*
 * @brief: PWMIO period time (us)
 */
uint32_t g_u32PeriodTime = 0;

// 离线标志位
uint8_t g_u8pwmOffineFlag = 0;
//占空比确认计数器
uint32_t g_u32DutyCycleConfirmCnt = 0;
//上一次的占空比
uint32_t g_u32LastDutyCycle = 0;
//确认后的占空比
uint32_t g_u32DutyCycleConfirm = 0;

// 上次边沿中断的时间戳
uint32_t g_u32LastEdgeTime = 0;


static uint32_t g_u32LowDutyOffCounter = 0;    // 低占空比关机计数器
static uint32_t g_u32HighDutyOffCounter = 0;   // 高占空比关机计数器
static uint8_t g_u8PwmMotorState = 0;          // 电机状态：0-关机，1-运行



/************************************************************
 * @brief: Parse DC PWM into normalization speed
 * @return <None>
 /*
/*
 * PWM Duty Cycle vs. CFM Speed Control State Machine
 * 
 * Duty Cycle Range: 0% - 100%
 * Voltage Range: 5V - 12V
 * PWM Frequency: 8-400Hz
 * 
 * Speed States:
 *   N_zero:      0 rpm
 *   N_min:     600 rpm  
 *   N_Linear: 600-2250 rpm (linear region)
 *   N_nominal:2250 rpm
 *   N_max:    2700 rpm
 * 
 * State Transition Diagram:
 * 
 * Duty Cycle(%)    CFM State          Transition Conditions
 * ┌──────────────┬───────────────┬────────────────────────────────────────────┐
 * │     0 - 5.3  │ Error Zone    │    Abnormal duty, low power operation      │
 * │     0 - 10.7 │ N_zero        │                                            │
 * │              │               │                                            │
 * │    10.7-11.9 │ Transition    │ N_zero -> N_min: duty > 11.9%              │
 * │              │ Zone          │ N_min -> N_zero: duty < 10.7%              │
 * │              │               │                                            │
 * │    11.9-15.8 │ N_min         │ Maintains 600 rpm                          │
 * │              │               │                                            │
 * │    15.8-84.3 │ N_Linear      │ Linear control: 600-2250 rpm               │
 * │              │               │                                            │
 * │    84.3-85.1 │ Transition    │ N_nominal -> N_max: duty > 86.2%           │
 * │              │ Zone          │ N_max -> N_nominal: duty < 85.1%           │
 * │              │               │                                            │
 * │    85.1-86.2 │ N_nominal     │ Maintains 2250 rpm                         │
 * │              │               │                                            │
 * │    86.2-95.5 │ N_max         │ Maintains 2700 rpm                         │
 * │              │               │                                            │
 * │    95.5-100  │ Error Zone    │ Abnormal duty, low power operation         │
 * └──────────────┴───────────────┴────────────────────────────────────────────┘
 * Special Functions:
 * - Emergency run after 10s
 * - KL15:ON ignition signal
 * - Hysteresis prevents frequent state switching
 * 特殊功能:
 * - Emergency run: 10秒后进入应急运行模式全速运行
 *
 *
 *
 ************************************************************/

Frac16_t ParsePWMSpeed(uint16_t u16Pwm)
{

    uint16_t u16Rpm = 0;
    static uint16_t u16LasRpm = 0;
    static int16_t abnormalCounter = 0;   //异常占空比计数器

    if(abnormalCounter >= PWM_ABNORMAL_OFF_TIME){    //如果占空比异常超过10s则全速运行
        g_PWM_CFM_State_t.current_state = CFM_STATE_ERROR;   //异常状态
    }
    


    if((u16Pwm >= PWM_IMMEDIATE_OFF_MIN) && (u16Pwm < PWM_MIN_ZERO)){    //5.3-10.7%
        g_PWM_CFM_State_t.current_state = CFM_STATE_ZERO; //关闭状态
        
    }else if((u16Pwm >= PWM_ZERO_MIN) && (u16Pwm < PWM_LINEAR_START_DUTY)){    //11.9-15.8%
        g_PWM_CFM_State_t.current_state = CFM_STATE_MIN; //最小转速状态
        
    }else if((u16Pwm >= PWM_LINEAR_START_DUTY) && (u16Pwm < PWM_LINEAR_END_DUTY)) {  //15.8-84.3%
        g_PWM_CFM_State_t.current_state = CFM_STATE_LINEAR; // 线性区域状态
        
    }else if((u16Pwm >= PWM_LINEAR_END_DUTY) && (u16Pwm < PWM_MAX_NOM_DUTY)){   //84.3-85.1%
        g_PWM_CFM_State_t.current_state = CFM_STATE_NOMINAL; // 额定转速状态
        
    }else if((u16Pwm >= PWM_NOM_MAX_DUTY) && (u16Pwm < PWM_HIGH_OFF_TH)){
        g_PWM_CFM_State_t.current_state = CFM_STATE_MAX; // 最高转速状态  //86.2-95.5%

    }else{   //位于两个回差区间

        if((u16Pwm >= PWM_MIN_ZERO) && (u16Pwm < PWM_ZERO_MIN)){   //第一个回差区间
            if(g_PWM_CFM_State_t.current_state > CFM_STATE_ZERO ){     //上一次状态是否大于关闭状态
                g_PWM_CFM_State_t.current_state = CFM_STATE_MIN; //最小转速状态
            }else{
                g_PWM_CFM_State_t.current_state = CFM_STATE_ZERO;    //关闭状态
            }

        }else if ((u16Pwm >= PWM_MAX_NOM_DUTY) && (u16Pwm < PWM_NOM_MAX_DUTY))//第二个回差区间
        {
            if(g_PWM_CFM_State_t.current_state > CFM_STATE_NOMINAL ){    //上一次状态是否大于额定状态
                g_PWM_CFM_State_t.current_state = CFM_STATE_MAX; // 最高转速状态 
            }else
                g_PWM_CFM_State_t.current_state = CFM_STATE_NOMINAL; // 额定转速状态
        }

    }
    
    //根据状态赋值
    switch (g_PWM_CFM_State_t.current_state)  
    {
    case CFM_STATE_ZERO:
        u16Rpm = 0;                  //停机
        break;
    case CFM_STATE_MIN:
        u16Rpm = PWM_MIN_RPM ;       //最小转速
        break;
    case CFM_STATE_LINEAR:
        u16Rpm = PWM_MIN_RPM + ((u16Pwm - PWM_LINEAR_START_DUTY) * PWM_RAMP_UP);  //线性区域
        break;
    case CFM_STATE_NOMINAL:
        u16Rpm = PWM_NOMINAL_RPM ;   //额定转速
        break;
    case CFM_STATE_MAX:
        u16Rpm = PWM_MAX_RPM;        //全速运行
        break;
    case CFM_STATE_ERROR:
        u16Rpm = PWM_MAX_RPM;        //全速运行
        break;

    default:
        break;
    }
    //异常处理,如果位于0-5.3%和95.5-100%则判断异常。10s后全速运行
    if (((u16Pwm < PWM_IMMEDIATE_OFF_MIN) || (u16Pwm > PWM_HIGH_OFF_TH)) && (g_PWM_CFM_State_t.current_state != CFM_STATE_ERROR)) {    

        // u16Rpm = 0;
        // 无效使用上一次的PWM值
        u16Rpm = u16LasRpm;
        abnormalCounter++;
        return u16Rpm * 32767 / (int16_t)tFocParas.N_MAX;
    }else{    //则占空比正常，清除计数器
        abnormalCounter = 0;  
    }

    // 更新上一次的PWM值
    u16LasRpm = u16Rpm;
    return u16Rpm * 32767 / (int16_t)tFocParas.N_MAX;
}



/****************************************************************
 * @brief: PWMIO initialization
 * @author: Novosns MCU Team
 * @param <None>
 * @return <None>
 ****************************************************************/
void PWMIO_Init(void)
{
    /* ADC clock enable */
    SYSCTRL->LKKEYR = SYSCTRL_LOCKKEY; // Unlock SYSCTRL
    SYSCTRL->SCCR_b.AHBCEN = 1;        // Enable AHB clock
    SYSCTRL->SCCR_b.APBCEN = 1;        // Enable APB clock

    SYSCTRL->AHBCGR_b.GPIOCEN = 1;  // Enable GPIO clock
    SYSCTRL->APBCGR_b.PWMIOCEN = 1; // Enable PWMIO clock
    SYSCTRL->APBCGR_b.LINPCEN = 1;  // Enable LIN clock

    // GPIO->MXR1_b.PM5 = 5;    // GPIO5 set PWMIO_IN mode 配置GPIO5为PWMIO输入
    // GPIO->PDIEN_b.DIEN5 = 1; // Enable GPIO5 input
    LINUART->CR = 0x00;
    //LINPORT->CR_b.RXEN = 1;             // 使能LINPORT接收
	LINPORT->CR_b.LINPORTEN = 1;  // 使能LINPORT
    LINPORT->CR_b.PAIS = 0;  // PWMIO MUX: 0 = LIN RX, 1 = GPIO //配置PWMIO使用GPIO引脚
    LINPORT->CR_b.TXOS = 1;

    PWMIO->CR_b.ICLK = 5;  // PWMIO clk = 48M / (2^5) = 1.5MHz
    PWMIO->CR_b.FDUPL = 0; // Duplex: 0 = half deplex, 1 = full duplex
    PWMIO->CR_b.IDGL = 1;  // Deglitch filter: 0 = disable, 1 = enable,
                           // deglitch filter time is 3.0us~3.2us
    PWMIO->CR_b.EDFEN = 1; // First edge interrupt: 0 = enable, 1 = disable

    PWMIO->IEN_b.IREIE = 1;  // Enbale rasing edge interrupt
    PWMIO->IEN_b.IFEIE = 1;  // Enbale falling edge interrupt
    PWMIO->IEN_b.ICOFIE = 1; // Enbale cnt overflow interrupt

    NVIC_SetPriority(PWMIO_IRQn, PWMIO_INT_PRIO); // Set PWMIO interrupt priority
    NVIC_EnableIRQ(PWMIO_IRQn);                   // Enable PWMIO interrupt

    PWMIO->CR_b.IENA = 1; // Enable PWMIO input
    PWMIO->CR_b.OM =0; //周期PWM输出
}


/****************************************************************
 * @brief: PWMIO interrupt handler
 * @author: Novosns MCU Team
 * @param <None>
 * @return <None>
 ****************************************************************/
void PWMIO_IRQHandler(void)
{
    // GPIO->PDO_b.DO0 = 1;
    if (PWMIO->ISR_b.IREIF) {                       // Check rasing edge interrupt
        
        g_u32LowLevelCnt = PWMIO->ILT_b.ILT;        // Read rasing level cnt
        if (g_u32OverflowCnt) {                     // Caculate low level cnt
            g_u32LowLevelCnt += g_u32OverflowCnt * 0x3FFFU; // 如果发生溢出，加上溢出次数
        }
        g_u32LowLevelTime = (g_u32LowLevelCnt << 1) / 3;    // Caculate low level time (us) 分频系数为5 48M/2^5=1.5M T = 1/1.5M = 0.666us = 2/3us
        g_u32OverflowCnt = 0;                       // Clear over flow times
        
        // Calculate period and duty cycle with integer math
        g_u32PeriodTime = g_u32HighLevelTime + g_u32LowLevelTime;
        
        // 检查频率是否在有效范围内
        // 有效频率范围: 5Hz ~ 500Hz (周期: 200ms ~ 2ms)
        if ((g_u32PeriodTime >= PWMIO_MAX_FREQ_US) && (g_u32PeriodTime <= PWMIO_MIN_FREQ_US) && (g_u32PeriodTime > 0)) {
            // 频率在有效范围内，计算占空比
            g_u32DutyCycle = (g_u32HighLevelTime * 1000) / g_u32PeriodTime;
            g_u32LastEdgeTime = g_epochSecond;
        } else {
            // 频率超出范围，设置占空比为0
            g_u32DutyCycle = 0;
        }
        
        PWMIO->ISR = PWMIO_CLEAR_IRE_INT;           // Clear rising edge interrupt flag
    }

    if (PWMIO->ISR_b.IFEIF) {                       // Check falling edge interrupt
        g_u32HighLevelCnt = PWMIO->IHT_b.IHT;       // Read falling level cnt
        if (g_u32OverflowCnt) {                     // Caculate low level cnt
            g_u32HighLevelCnt += g_u32OverflowCnt * 0x3FFFU;  
        }
        g_u32HighLevelTime = (g_u32HighLevelCnt << 1) / 3;  // Caculate low level time (us)
        g_u32OverflowCnt = 0;                       // Clear over flow times
        PWMIO->ISR= PWMIO_CLEAR_IFE_INT;            // Clear falling edge interrupt flag
    }

    if (PWMIO->ISR_b.ICOFIF) {                      // Check overflow interrupt
        g_u32OverflowCnt++;                         // Increase over flow times 
        PWMIO->ISR = PWMIO_CLEAR_ICOF_INT;          // Clear overflow interrupt flag
    }
    // GPIO->PDO_b.DO0 = 0;
}


/****************************************************************
 * @brief: Set PWMIO duty cycle and period time
 * @param freq: PWM frequency in Hz
 * @param duty: PWM duty cycle in 0.1% (0-1000
 * @return <None>
 ****************************************************************/
void PWMOUT_Set(uint32_t freq, uint32_t duty)
{
    uint32_t opt = 0;
    opt = (1500000 / freq); // Calculate OPT value for 1.5MHz clock
    opt = opt - 1;
    PWMIO->OPT_b.OPT = opt;          // PWM frequence = 1.5MHz / 100 = 1.5kHz
    // PWMIO->OLT_b.OLT = 70;          // PWM duty = 1 - OLT / OPT
    if (duty > 1000) {
        duty = 1000; // Limit duty cycle to 1000 (100%)
    } 
    duty = 1000 - duty;
    
    PWMIO->OLT_b.OLT = (duty * opt) / 1000; // Set duty cycle, OLT = OPT * duty / 1000
}


//设置是否PWM输出
void PWMOUT_SetEnable(uint8_t enable)
{
    if (enable) {
        PWMIO->CR_b.OENA = 1; // Enable PWM output
        PWMIO->CR_b.IENA = 0; // Disable PWMIO input
    } else {
        PWMIO->CR_b.OENA = 0; // Disable PWM output
        PWMIO->CR_b.IENA = 1; // Enable PWMIO input
    }
}

/****************************************************************
 * @brief: Get current PWMIO duty cycle
 * @author: AIWIN MCU Team
 * @param <None>
 * @return uint32_t: duty cycle in 0.1% (0-1000)
 ****************************************************************/
uint32_t PWMIO_GetDutyCycle(void)
{
    return g_u32DutyCycle;
}


/****************************************************************
 * @brief: Get current PWMIO duty cycle in float type (0-100.0%) 
 * @param <None>
 * @return float: duty cycle in 0.1% (0-100.0%)
 ****************************************************************/
float PWMIO_GetDutyCycleFloat(void)
{
    return (float)g_u32DutyCycle / 10.0;
}

/****************************************************************
 * @brief: Get current PWMIO period time
 * @author: AIWIN MCU Team
 * @param <None>
 * @return uint32_t: period time in microseconds
 ****************************************************************/
uint32_t PWMIO_GetPeriodTime(void)
{
    return g_u32PeriodTime;
}


void PWMIO_CheckTimeout(void)
{
    if (((g_epochSecond > g_u32LastEdgeTime) && (g_epochSecond - g_u32LastEdgeTime > PWM_TIMOUT)) 
    || (g_epochSecond < g_u32LastEdgeTime)){
            g_u32DutyCycle = 0;         // 清零占空比
            g_u32HighLevelTime = 0;     // 清零高电平时间
            g_u32LowLevelTime = 0;      // 清零低电平时间
            g_u32PeriodTime = 0;        // 清零周期时间
            g_u32OverflowCnt = 0;       // 清零溢出次数
            g_u32HighLevelCnt = 0;      // 清零高电平计数器
            g_u32LowLevelCnt = 0;       // 清零低电平计数器
            g_u32LastEdgeTime = 0;      // 清零上次边沿中断时间戳

            g_u8pwmOffineFlag = 1;
    } else {
            g_u8pwmOffineFlag = 0;
    }

    
}


// 确认占空比
void PWMIO_ConfirmDutyCycle(void)
{
    if (SAFE_SUBABS(g_u32DutyCycle,g_u32LastDutyCycle) > PWM_CONFIRM_ERR) { // Check if the difference is larger than the error threshold
        g_u32DutyCycleConfirmCnt = 0; // Reset confirm counter
        g_u32LastDutyCycle = g_u32DutyCycle; // Update last duty cycle
    } else {
        g_u32DutyCycleConfirmCnt++;
        if (g_u32DutyCycleConfirmCnt >= PWM_CONFIRM_TIME) { // Confirm time reached
            g_u32DutyCycleConfirm = g_u32LastDutyCycle; // Update confirmed duty cycle
            g_u32DutyCycleConfirmCnt = 0; // Reset confirm counter
        }
    }
}

//PWM控制10ms任务
void PWMIO_Task10ms(void)
{
#if (IS_RELEASE_VERSION != 2) //EMC测试为了避免通讯盒的干扰需要拔掉通讯盒也能运行
    PWMIO_CheckTimeout(); // Check for timeout 先屏蔽，防止与最低占空比延迟关机和最高占空比延迟关机冲突，其实那个就是用于类似通讯超时的情况
#endif
    PWMIO_ConfirmDutyCycle(); // Confirm duty cycle
    // ParsePWMSpeed(g_u32DutyCycleConfirm); // Parse the confirmed duty cycle into speed
}


/**
 * @brief: 获取PWM电机状态
 * @return: 1-运行，0-停止
 */
uint8_t PWMIO_GetMotorState(void)
{
    return g_u8PwmMotorState;
}

/**
 * @brief: 重置PWM控制器状态
 */
void PWMIO_ResetState(void)
{
    g_u32LowDutyOffCounter = 0;
    g_u32HighDutyOffCounter = 0;
    g_u8PwmMotorState = 0;
}

