/************************************************************
 * @file: fault_hw_config.h
 * @author: Novosns MCU Team
 * @version: V1.0
 * @data: 2023/10/10
 * @brief: 故障检测硬件配置头文件
 * @note:
 * @Copyright (C) 2023 Novosense All rights reserved.
 ************************************************************/
#ifndef __FAULT_HW_CONFIG_H__
#define __FAULT_HW_CONFIG_H__

 /******************** 故障检测配置定义 ********************/
 /* 硬件过压保护 */
#define ECU_BUS_HOV_ENABLE     (0) // 硬件过压检测中断。0:禁用 1:启用
#define ECU_BUS_HOV_FT         (4) // BVDD硬件过压去抖动时间。N*0.5us
#define ECU_BUS_HOV_TH         (0) // BVDD硬件过压阈值: 0:18v 1:28v
 /* 硬件欠压保护 */
#define ECU_BUS_HUV_ENABLE     (0) // 硬件欠压检测中断。0:禁用 1:启用
#define ECU_BUS_HUV_TH         (0) // 硬件欠压触发电平: 0:6.5V 1:7.5V 2:8.5V 3:9.5V
#define ECU_BUS_HUV_FT         (4) // BVDD硬件欠压去抖动时间。N*0.5us
 /* 硬件过温保护 */
#define ECU_BUS_OT_ENABLE      (0) // 过温检测中断。0:禁用 1:启用
#define ECU_BUS_OT_FT          (4) // 过温去抖动时间。N*0.5us
 /* 硬件电荷泵过/欠压保护 */
#define ECU_MOT_CP_OV_EANABLE  (0) // 电荷泵过压中断。0:禁用 1:启用
#define ECU_MOT_CP_OV_FT       (3) // 电荷泵过压滤波: -1:禁用 0:1us 1:2us 2:4us 3:8us
#define ECU_MOT_CP_CAE         (0) // 电荷泵过压时自动关闭。0:禁用 1:启用
#define ECU_MOT_CUVTH          (0) // 电荷泵欠压阈值: 0:5V 1:6V 2:7V 3:8V
#define ECU_MOT_COVTH          (0) // 电荷泵过压阈值: 0:MVDD+16V 1:MVDD+20V

#define ECU_FAULT_PREEMPT_PRIO (0) // 故障检测抢占优先级
#define ECU_FAULT_IRQ_PRIO     (0) // 故障检测优先级

/* power circuits protection */
#define MOS_DESAT_EN (1) // enable driver desaturation
/**GDU HS/LS desaturation threshold.
0:0.25V 1:0.5V 2:0.75V 3:1V 
4:1.25V 5:1.5V 6:1.75V 7:2V*/
#define GDU_DESAT_TH (7)
/** desaturation debounce time (SCDEN+1)/48M s*/
#define GDU_DESAT_SCDEN (96) 
/** desaturation mask time (SCMSK+1)/48M s*/
#define GDU_DESAT_MSK (48)

#define GDU_OCSD_EN (0) // enable over current shutdown
/** shunt OC debounce time  (OCDEB+1)/48M s*/
#define GDU_OCDEB (240)  
/** shunt OC current: 2.5V/255 * MOS_OCVOL V*/
// OC cur/MAX CUR*(2.5-OFFSET Vol)*/2.5*255 + OFFSET Vol/2.5*255
// 60 / 62.5 * (2.5-0.25)/2.5*255 + 0.25/2.5*255 = 0.96 * 2.25/2.5 *255 + 25.5 = 0.96 * 0.9 * 255 + 25.5 = 245.82
//62.5/62.5 * (2.5-0.25)/2.5*255 + 0.25/2.5*255 = 1 * 0.9 * 255 + 25.5 = 229.5 + 25.5 = 255
/**
 * @brief  根据给定的电流参数计算GDU硬件过流保护的电压阈值 (GDU_OCVOL)
 * @param  OC_CUR       期望设置的硬件过流保护电流值 (A)
 * @param  MAX_CUR      电流采样的最大量程 (A)正向的
 * @param  OFFSET_VOL   采样运放的偏置电压 (V), 通常为 Vref/2
 * @note   该宏基于以下公式进行计算：
 *         GDU_OCVOL = (OC_CUR / MAX_CUR) * (2.5 - OFFSET_VOL) / 2.5 * 255 + (OFFSET_VOL / 2.5 * 255)
 *         为了在整数运算中保持精度，所有浮点数都已乘以1000进行放大。
 */
#define CALCULATE_GDU_OCVOL(OC_CUR, MAX_CUR, OFFSET_VOL) \
    (uint8_t)(( \
        (long long)(OC_CUR) * 1000 / (MAX_CUR) * \
        (2500 - (long long)((OFFSET_VOL) * 1000)) / 2500 * 255 / 1000 \
    ) + ( \
        (long long)((OFFSET_VOL) * 1000) * 255 / 2500 \
    ))
#define GDU_OCVOL CALCULATE_GDU_OCVOL(60.0f, 62.5f, 0.25f) 






#endif //__FAULT_HW_CONFIG_H__