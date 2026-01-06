/************************************************************
 * @file: fault_diagnose.h
 * @author: Novosns MCU Team
 * @version: V0.0
 * @data: 2023/6/20
 * @brief: 电机控制系统故障诊断模块头文件
 * @note: 定义故障类型、故障检测阈值及相关函数
 * @Copyright (C) 2023 Novosense All rights reserved.
 ************************************************************/

/************************************************************
 ************************************************************
 * @par 版本历史
 * -V0.0  2023.6.20
 *        -初始版本
 *
 ************************************************************/
#ifndef __FAULT_DIAGNOSE_H__
#define __FAULT_DIAGNOSE_H__

#include "stdint.h"

/* 定义 */
#define V_DROP_DIODE            51  // 二极管压降值 0.5v/40*4095
#define BVDD_TO_ADC_VAL(x)      ((int16_t)(x/40.0*4095) - V_DROP_DIODE)  // 将实际电压转换为ADC值
#define CHECK_COUNT_NO          255 // 故障检测计数阈值
#define ERROR_COUNT_NO          240 // 故障确认计数阈值

#pragma anon_unions
typedef struct {
    union {
        uint32_t u32FaultMode; // 故障模式标志位(32位)
        struct {
            uint32_t VBUS_HOV : 1;      // 位0  母线电压硬件过压标志
            uint32_t VBUS_HUV : 1;      // 位1  母线电压硬件欠压标志
            uint32_t VBUS_SOV : 1;      // 位2  母线电压软件过压标志
            uint32_t VBUS_SPREOV : 1;   // 位3  母线电压软件预过压标志
            uint32_t VBUS_SUV : 1;      // 位4  母线电压软件欠压标志
            uint32_t VBUS_SPREUV : 1;   // 位5  母线电压软件预欠压标志
            uint32_t MOT_STALL_LOCK : 1;// 位6  电机堵转锁定标志
            uint32_t MOT_STALL : 1;     // 位7  电机堵转标志
            uint32_t MOT_HOC : 1;       // 位8  桥臂硬件过流标志
            uint32_t MOT_SOC : 1;       // 位9  桥臂软件过流标志
            uint32_t ECU_HOT : 1;       // 位10 芯片硬件过温标志
            uint32_t ECU_SOT : 1;       // 位11 芯片软件过温标志
            uint32_t ECU_SOTST : 1;     // 位12 芯片软件过温关断标志
            uint32_t ECU_OT_SENS_OPEN : 1; // 位13 温度传感器开路标志
            uint32_t ECU_OT_SENS_SHRT : 1; // 位14 温度传感器短路标志
            uint32_t ECU_SUT : 1;       // 位15 芯片软件低温标志
            uint32_t CP_OV : 1;         // 位16 电荷泵过压标志
            uint32_t CP_UV : 1;         // 位17 电荷泵欠压标志
            uint32_t BSW_COIL_SHRT_GND: 1;    // 位18 低端开关或线圈短接到地标志
            uint32_t TSW_COIL_SHRT_BAT : 1;   // 位19 高端开关或线圈短接到电源标志
            uint32_t BSW_COIL_OPEN : 1;       // 位20 低端开关开路或线圈缺失标志
            uint32_t TSW_OPEN_COIL_SHRT : 1;  // 位21 高端开关开路或线圈互相短接标志
            //缺相
            uint32_t ECU_LACK_PHASE : 1;      // 位22 相缺相标志
            uint32_t ECU_DRV_SHORT : 1;          // 位23 驱动器短路标志
            uint32_t _reserved : 8; // 位24-31 保留位
        } FM_b;
    };
} Fault_Mode_t;

typedef struct {
    /** 电气故障配置 */
    int16_t i16SftOVTh;      // 软件过压阈值
    int16_t i16SftPreOVTh;   // 软件预过压阈值
    int16_t i16SftOVRecTh;   // 软件过压恢复阈值
    int16_t i16SftUVTh;      // 软件欠压阈值
    int16_t i16SftPreUVTh;   // 软件预欠压阈值
    int16_t i16SftUVRecTh;   // 软件欠压恢复阈值
    /** 温度故障配置 */
    int16_t i16SftOTTh;      // 软件过温阈值
    int16_t i16SftOTRecTh;   // 软件过温恢复阈值
    int16_t i16SftOTSTTh;    // 软件过温关断阈值
    int16_t i16SftOTSTRecTh; // 软件过温关断恢复阈值
    int16_t i16SftUTTh;      // 软件低温阈值
    int16_t i16FltHystTimes; // 故障诊断滞后次数
} Fault_config_t;


extern Fault_Mode_t tFaultMode;    // 故障模式全局变量
extern Fault_config_t tFaultCfg;   // 故障配置全局变量

/** 公共函数 */
int8_t FLT_PwrCircuitDetect(void); // 电源电路检测函数
void FLT_ElecDetection(int16_t i16Vdc); // 电气参数检测函数
void FLT_ClimDetection(int16_t i16Temp); // 温度参数检测函数
extern void AlarmCheck10msTask(void);
extern void AlarmCheck1msTask(void);
extern void getCurrentMaxMin(void);
extern int16_t GetCurrentAbsSum025(void);
#endif //__FAULT_DIAGNOSE_H__