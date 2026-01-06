#ifndef CONFIG_H_
#define CONFIG_H_

#ifdef __cplusplus
extern "C"
{
#endif


//通讯方式 
#define COM_TYPE_PWM 1
#define COM_TYPE_LIN 2
#define COM_TYPE_ALL 3 //该模式下适用于PWM，该模式下前500ms为LIN通讯模式，在前500ms接收指定LIN帧的方式将其确认为LIN模式，超出500ms没有LIN帧接收，认为为PWM模式


//通讯方式选择
#define COM_TYPE COM_TYPE_LIN // 1:LIN 2:PWM



//是否需要KL15休眠唤醒
#define KL15_WAKEUP_ENABLE (0)

//PWM通讯超时间 单位 S2S
#define PWM_TIMOUT (2)

//PWM控制下的指令
//额定转速
#define PWM_NOMINAL_RPM 2250
//最低转速
#define  PWM_MIN_RPM 600
//最高转速
#define PWM_MAX_RPM 3000

//运行过程拔掉PWM通讯器是否全速运行
#define PWM_RUN_FULL_SPEED_ON_DISCONNECT (0) // 1:全速运行 0:不全速运行

//PWM确认时间  单位 0.01S
#define PWM_CONFIRM_TIME (100) //2S
//PWM容量误差 单位 0.1%
#define PWM_CONFIRM_ERR (8)  //0.8%

/*
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
*/
#define PWM_IMMEDIATE_OFF_MIN   (53)    // 5.3% - 立即关机阈值下限
#define PWM_HIGH_OFF_TH         (955)   // 95.5% - 高占空比关机阈值
//回差区域开始
#define PWM_MIN_ZERO (107) //10.7%   N_min -> N_zero
//回差区域结束
#define PWM_ZERO_MIN (119) //11.9% N_zero -> N_min

//线性上升开始占空比
#define PWM_LINEAR_START_DUTY (158) //15.8%
//线性上升结束占空比
#define PWM_LINEAR_END_DUTY (843) //84.3%
//紧急转速占空比

//回差区域开始
#define PWM_MAX_NOM_DUTY (851) //85.1% N_max -> N_nominal
//回差区域结束
#define PWM_NOM_MAX_DUTY (862) //86.2%  N_nominal -> N_max

//升速斜率 (额定转速-最小转速) / （PWM_LINEAR_END_DUTY-PWM_LINEAR_START_DUTY）
#define PWM_RAMP_UP ((((float)NOMINAL_RPM - PWM_MIN_RPM) / (PWM_LINEAR_END_DUTY - PWM_LINEAR_START_DUTY))*100/100) // 0.1%/s

#define SAFE_SUBABS(a, b) ((a) > (b) ? (a) - (b) : (b) - (a))

//#define PWM_DELAYED_OFF_TIME    (10000)  // 10s - 延迟关机时间(单位:1ms)
//#define PWM_HIGH_OFF_TIME       (10000)  // 10s - 高占空比关机时间(单位:1ms)
#define PWM_ABNORMAL_OFF_TIME       (10000)  // 10s - 占空比异常关机时间(单位:1ms)



//-----------------


#ifdef __cplusplus
}
#endif



#endif


