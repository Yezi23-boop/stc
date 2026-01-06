/************************************************************
 * @file: foc_config.h
 * @author: Novosns MCU Team
 * @version: V0.0
 * @data: 2023/12/13
 * @brief: foc configuration header file
 * @note:
 * @Copyright (C) 2023 Novosense All rights reserved.
 ************************************************************/
 
/************************************************************
 ************************************************************
 * @par Edition History
 * -V0.0  2023.12.13
 *        -Initial version for foc_config.h of NSUC1602.
 *
 ************************************************************/
 
#ifndef __FOC_CONFIG_H__
#define __FOC_CONFIG_H__

#include "foc_paras.h"

//-------------------------- 电机基础参数配置 --------------------------
//额定转速
#define NOMINAL_RPM 2250           /*!< 电机额定转速，单位：RPM（转/分钟），FOC转速控制的基准值 */

//-------------------------- EPWM（增强型PWM）配置 --------------------------
/** EPWM deadtime for FOC. deadtime = FOC_EPWM_DEADTIME / 48M (s)*/
#define FOC_EPWM_DEADTIME  24      /*!< EPWM死区时间配置值，计算方式：死区时间=48/48M=1μs，单位：48M时钟周期数 */

/** EPWM Alignment mode: 0:edge 1:center*/
#define FOC_EPWM_PTMOD 1 // center  /*!< EPWM对齐模式：0-边沿对齐，1-中心对齐（推荐FOC使用中心对齐） */

/** EPWM周期计算宏，X为目标PWM频率(Hz)，中心对齐模式下周期=48M/(频率*2) */
#define FOC_EPWM_PERIOD_CALC(X) (48000000 / (X) / 2)

/** EPWM period: edge aligh: 48M / FOC_EPWM_PERIOD  center align: 48M / (FOC_EPWM_PERIOD * 2)*/
#define FOC_EPWM_PERIOD FOC_EPWM_PERIOD_CALC(FOC_EPWM_FREQUENCY)  /*!< EPWM周期值，由目标频率计算得出，单位：48M时钟周期数 */

/** EPWM out mode: 0:complement 1:independent*/
#define FOC_EPWM_OUT_MODE 0        /*!< EPWM输出模式：0-互补输出（上下桥臂互补），1-独立输出 */

/** EPWM single shunt mode */
#define FOC_EPWM_SHUNT_MODE -1     /*!< EPWM单电阻采样模式：-1-双电阻采样，0-方法0(r1)，1-方法1(相移)，2-方法3(双开关)，4-无电流采样 */

/** EPWM IRQ priority */
#define EPWM_IRQ_PRIO 1            /*!< EPWM中断优先级，取值0~3（0最高），配置为1级优先级 */

//-------------------------- ADC（模数转换）配置 --------------------------
/** ADC buffer */
#define ADC_INBUFEN 0              /*!< ADC输入缓冲使能：0-禁用，1-启用，电流采样通常禁用以提升速度 */

/** ADC buffer bypass. If ADC_INBUFEN is set to 0, ADC_INBUFBYPASS must be set to 1*/
#define ADC_INBUFBYPASS 1          /*!< ADC输入缓冲旁路：0-禁用，1-启用（INBUFEN=0时必须设为1） */

/** ADC IRQ Priority */
#define ADC_IRQ_PRIO 2             /*!< ADC中断优先级，取值0~3，配置为2级（低于EPWM中断） */

//-------------------------- GDU（栅极驱动单元）配置 --------------------------
/**GDU auto close when BVDD over voltage.0:disable 1:enable*/
#define GDU_AUTO_CLOSE_BOV 0       /*!< BVDD过压时GDU自动关闭：0-禁用，1-启用，防止过压损坏功率管 */

/**GDU auto close when BVDD under voltage.0:disable 1:enable*/
#define GDU_AUTO_CLOSE_BUV 0       /*!< BVDD欠压时GDU自动关闭：0-禁用，1-启用，防止欠压导致驱动异常 */

/**GDU auto close mode: 0:turn off all high bridge, turn on all low bridge 1:turn off all bridge*/
#define GDU_ASTMODE 1              /*!< GDU自动关闭模式：0-关断上桥臂、打开下桥臂，1-关断所有桥臂（推荐故障保护用1） */

/**GDU OC close mode.0: close current MOS, 1:close all MOS */
#define GDU_OCSDM 1                /*!< GDU过流关闭模式：0-仅关闭当前过流桥臂，1-关闭所有桥臂（故障保护更安全） */

/**GDU OC hysteresis voltage. 0:0mV 1:10mV 2:20mV 3:40mV*/
#define GDU_OCHYST 1               /*!< GDU过流滞回电压：1对应10mV，防止过流判定频繁跳变 */

/**GDU OC Hysteresis mode: 0:Bilateral 1:Unilateral*/
#define GDU_OCHYSTMODE 0           /*!< GDU过流滞回模式：0-双边滞回，1-单边滞回 */

/**GDU auto close when chargepump OV*/
#define GDU_AUTO_CLOSE_CPOV 1      /*!< 电荷泵过压时GDU自动关闭：0-禁用，1-启用，保护电荷泵和驱动电路 */

/**GDU auto close when chargepump UV*/
#define GDU_AUTO_CLOSE_CPUV 1      /*!< 电荷泵欠压时GDU自动关闭：0-禁用，1-启用，防止驱动能力不足 */

/**GDU ON slew rate control sequence */
#define GDU_ONSEQEN 1              /*!< GDU开启时斜率控制使能：0-禁用，1-启用，缓启动降低冲击 */

/**GDU OFF slew rate control sequence */
#define GDU_OFFSEQEN 1             /*!< GDU关闭时斜率控制使能：0-禁用，1-启用，缓关断降低电压尖峰 */

/**GDU HS fast charge off*/
#define GDU_HSFDISCHG 1            /*!< GDU上桥臂快速放电使能：0-禁用，1-启用，加快上桥臂关断速度 */

/**GDU LS fast charge off*/
#define GDU_LSFDISCHG 1            /*!< GDU下桥臂快速放电使能：0-禁用，1-启用，加快下桥臂关断速度 */

/**GDU deadtime config. 
0:0.2 us  1:0.4 us  2:0.8 us  3:1.6us
4: 3.2 us  5:6.4 us  6:12.8 us 7:disable */
#define GDU_CCPT 0                 /*!< GDU死区时间配置：0对应0.2μs，匹配功率管开关速度，防止上下桥臂直通 */

/**GDU wait chargepump. 0:release pwm regardless of chargepump 1:release pwm untill chargepump ready */
#define GDU_CPWAITEN 0             /*!< GDU等待电荷泵就绪：0-忽略电荷泵状态直接输出PWM，1-等待电荷泵就绪后输出 */

#define GDU_IRQ_PRIO 1             /*!< GDU中断优先级，取值0~3，配置为1级（与EPWM同级） */

#endif //__FOC_CONFIG_H__