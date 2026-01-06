/************************************************************
 * @file: nvsns_init.c
 * @author:
 * @version: V0.0
 * @data: 2023/10/29
 * @brief: nvsns模块初始化文件
 * @note:
 * @Copyright (C) 2023 Novosense All rights reserved.
 ************************************************************/

/************************************************************
 ************************************************************
 * @par 版本历史
 * -V0.0  2023.07.28
 *        - 初始化文件
 *
 ************************************************************/

#include "fault_diagnose.h"
#include "fault_hw_config.h"
#include "nsuc1602.h"
#include "foc_config.h"

//---------------时间参数-------------------
#define VOLT_CHECK_CNT 120 // 1.2s  10ms 50次
#define VOLT_CLEAR_CNT 100 // 1s  10ms 100次

#define OTP_CHECK_CNT      100             // 1s  10ms 100次
#define OTP_CLEAR_CNT      100             // 1s  10ms 100次

/** 机器诊断定义 */
#define PH_A_NO 0
#define PH_B_NO 1
#define PH_C_NO 2

static int32_t g_i32ISum = 0;         // 电流总和
static uint16_t g_u16BridgeOpen = 0;  // 桥臂开路标志
static uint16_t g_u16BridgeShort = 0; // 桥臂短路标志

static int16_t g_i16PhsVol[3] = {0}; // 相电压数组
static int16_t g_i16DcCur[4] = {0};  // 直流电流数组
static int16_t g_i16Dcoft = 2048;    // 直流偏移量

Fault_Mode_t tFaultMode; // 故障模式结构体

//  /************************************************************
//   * @brief: 总线电压诊断函数
//   * @param: i16Vdc, 处理后的总线电压
//   * @return <None>
//   ************************************************************/
//  void FLT_ElecDetection(int16_t i16Vdc)
//  {
//      static int16_t i16VdcAft;             // 总线电压处理后值
//      static int16_t i16OvCntr     = 0;     // 过压计数器
//      static int16_t i16PreOvCntr     = 0;  // 预过压计数器
//      static int16_t i16RecOvCnt = 0;       // 过压恢复计数器
//      static int16_t i16UvCntr   = 0;       // 欠压计数器
//      static int16_t i16PreUvCntr   = 0;    // 预欠压计数器
//      static int16_t i16RecUvCnt = 0;       // 欠压恢复计数器

//      i16VdcAft = i16Vdc;

//      /** 过压(OV)处理 */
//      if (i16VdcAft > tFaultCfg.i16SftPreOVTh) {
//          /** 当Vdc > i16SftOVTh时，增加OV和预OV计数器 */
//          if (i16VdcAft > tFaultCfg.i16SftOVTh) {
//              i16OvCntr ++;
//              i16PreOvCntr ++;
//          } else {
//              /** 当Vdc < i16SftOVTh且Vdc > i16SftPreOVTh时，仅增加预OV计数器 */
//              i16PreOvCntr ++;
//          }

//          /* 仅设置预过压或过压标志 */
//          if (i16PreOvCntr > tFaultCfg.i16FltHystTimes && i16OvCntr < tFaultCfg.i16FltHystTimes) {
//              tFaultMode.FM_b.VBUS_SPREOV = 1;  // 设置预过压标志
//              tFaultMode.FM_b.VBUS_SOV = 0;
//          } else if (i16OvCntr >= tFaultCfg.i16FltHystTimes) {
//              tFaultMode.FM_b.VBUS_SPREOV = 0;
//              tFaultMode.FM_b.VBUS_SOV = 1;     // 设置过压标志
//              tFaultMode.FM_b.VBUS_SOVREC = 0;  // 清除过压恢复标志
//          }
//      } else {
//          i16OvCntr = 0;       // 清除过压计数器
//          i16PreOvCntr = 0;    // 清除预过压计数器
//      }

//      /** 过压恢复处理：当电压在正常范围内且存在过压或预过压标志时 */
//      if (i16VdcAft >=  tFaultCfg.i16SftPreUVTh && i16VdcAft <= tFaultCfg.i16SftPreOVTh &&
//          (tFaultMode.FM_b.VBUS_SOV || tFaultMode.FM_b.VBUS_SPREOV)) {
//          i16RecOvCnt++;
//          if (i16RecOvCnt > tFaultCfg.i16FltHystTimes) {
//              tFaultMode.FM_b.VBUS_SOV = 0;      // 清除过压标志
//              tFaultMode.FM_b.VBUS_SPREOV = 0;   // 清除预过压标志
//              tFaultMode.FM_b.VBUS_SOVREC = 1;   // 设置过压恢复标志
//          }
//      }

//      /** 欠压(UV)处理 */
//      if (i16VdcAft < tFaultCfg.i16SftPreUVTh) {
//          /** 当Vdc < i16SftUVTh时，增加UV和预UV计数器 */
//          if (i16VdcAft < tFaultCfg.i16SftUVTh) {
//              i16UvCntr ++;
//              i16PreUvCntr ++;
//          } else {
//              /** 当Vdc > i16SftUVTh且Vdc < i16SftPreUVTh时，仅增加预UV计数器 */
//              i16PreUvCntr ++;
//          }

//          /* 仅设置预欠压或欠压标志 */
//          if (i16PreUvCntr > tFaultCfg.i16FltHystTimes && i16UvCntr < tFaultCfg.i16FltHystTimes) {
//              tFaultMode.FM_b.VBUS_SPREUV = 1;  // 设置预欠压标志
//              tFaultMode.FM_b.VBUS_SUV = 0;
//          } else if (i16UvCntr >= tFaultCfg.i16FltHystTimes) {
//              tFaultMode.FM_b.VBUS_SPREUV = 0;
//              tFaultMode.FM_b.VBUS_SUV = 1;     // 设置欠压标志
//              tFaultMode.FM_b.VBUS_SUVREC = 0;  // 清除欠压恢复标志
//          }
//      } else {
//          i16UvCntr = 0;      // 清除欠压计数器
//          i16PreUvCntr = 0;   // 清除预欠压计数器
//      }

//      /** 欠压恢复处理：当电压在正常范围内且存在欠压或预欠压标志时 */
//      if (i16VdcAft <= tFaultCfg.i16SftPreOVTh && i16VdcAft >= tFaultCfg.i16SftPreUVTh &&
//          (tFaultMode.FM_b.VBUS_SPREUV || tFaultMode.FM_b.VBUS_SUV)) {
//          i16RecUvCnt ++;
//          if (i16RecUvCnt > tFaultCfg.i16FltHystTimes) {
//              tFaultMode.FM_b.VBUS_SUV = 0;      // 清除欠压标志
//              tFaultMode.FM_b.VBUS_SPREUV = 0;   // 清除预欠压标志
//              tFaultMode.FM_b.VBUS_SUVREC = 1;   // 设置欠压恢复标志
//          }
//      }
//  }

/************************************************************
 * @brief: 总线电压过压诊断函数
 * @param: i16Vdc, 处理后的总线电压
 * @return <None>
 ************************************************************/
void CheckBusOvp(int16_t i16Vdc)
{
    static int16_t i16VdcAft; // 总线电压处理后值
    static int16_t setCnt = 0;
    static int16_t clearCnt = 0;
    i16VdcAft = i16Vdc;

    if (i16VdcAft > tFaultCfg.i16SftOVTh)
    {
        setCnt++;
        clearCnt = 0;
    }
    else if (i16VdcAft < tFaultCfg.i16SftOVRecTh)
    {
        setCnt = 0;
        clearCnt++;
    }

    if (setCnt >= VOLT_CHECK_CNT)
    {
        setCnt = VOLT_CHECK_CNT;
        if (tFaultMode.FM_b.VBUS_SOV == 0)
        {
            tFaultMode.FM_b.VBUS_SOV = 1;
        }
    }

    if (clearCnt >= VOLT_CLEAR_CNT)
    {
        clearCnt = VOLT_CLEAR_CNT;
        tFaultMode.FM_b.VBUS_SOV = 0;
    }
}

/************************************************************
 * @brief: 总线电压欠压诊断函数
 * @param: i16Vdc, 处理后的总线电压
 * @return <None>
 ************************************************************/

void CheckBusUvp(int16_t i16Vdc)
{

    static int16_t i16VdcAft; // 总线电压处理后值
    static int16_t setCnt = 0;
    static int16_t clearCnt = 0;
    i16VdcAft = i16Vdc;

    if (i16VdcAft < tFaultCfg.i16SftUVTh)
    {
        setCnt++;
        clearCnt = 0;
    }
    else if (i16VdcAft > tFaultCfg.i16SftUVRecTh)
    {
        setCnt = 0;
        clearCnt++;
    }

    if (setCnt >= VOLT_CHECK_CNT)
    {
        setCnt = VOLT_CHECK_CNT;
        if (tFaultMode.FM_b.VBUS_SUV == 0)
        {
            tFaultMode.FM_b.VBUS_SUV = 1;
        }
    }

    if (clearCnt >= VOLT_CLEAR_CNT)
    {
        clearCnt = VOLT_CLEAR_CNT;
        tFaultMode.FM_b.VBUS_SUV = 0;
    }
}

/************************************************************
 * @brief: 温度诊断函数
 * @param: i16Temp, 温度值
 * @return <None>
 ************************************************************/
void FLT_ClimDetection(int16_t i16Temp)
{
    static int16_t i16OtCnt = 0;      // 过温计数器
    static int16_t i16OtRecCnt = 0;   // 过温恢复计数器

    if (i16Temp > tFaultCfg.i16SftOTTh)
    {
        i16OtCnt++;
        i16OtRecCnt = 0;
    }
    else if (i16Temp < tFaultCfg.i16SftOTRecTh)
    {
        i16OtCnt = 0;
        i16OtRecCnt++;
    }

    if (i16OtCnt >= OTP_CHECK_CNT)
    {
        i16OtCnt = OTP_CHECK_CNT;
        if (tFaultMode.FM_b.ECU_SOT == 0)
        {
            tFaultMode.FM_b.ECU_SOT = 1;
        }
    }

    if (i16OtRecCnt >= OTP_CLEAR_CNT)
    {
        i16OtRecCnt = OTP_CLEAR_CNT;
        tFaultMode.FM_b.ECU_SOT = 0;
    }

}

/************************************************************
 * @brief: 初始化用于机器检测的ADC模块
 * @return <None>
 ************************************************************/
void FLT_PwrCircuitInit(void)
{
    /* 配置ADC通道 */
    ADC->QCR1_b.Q1EN = 1;
    ADC->QCR1_b.Q1SEL = 3; // 电流传感放大器
    ADC->QCR1_b.Q2EN = 1;
    ADC->QCR1_b.Q2SEL = 3; // 电流传感放大器
    ADC->QCR1_b.Q3EN = 1;
    ADC->QCR1_b.Q3SEL = 3; // 电流传感放大器
    ADC->QCR1_b.Q4EN = 1;
    ADC->QCR1_b.Q4SEL = 3; // 电流传感放大器
    ADC->QCR1_b.Q5EN = 1;
    ADC->QCR1_b.Q5SEL = 4; // GDU相电压
    ADC->QCR1_b.Q6EN = 1;
    ADC->QCR1_b.Q6SEL = 1; // MVDD电源
    ADC->QCR2_b.Q7EN = 0;
    ADC->QCR2_b.Q7SEL = 2; // 温度传感器

    /* 配置ADC转换时序 */
    ADC->QTR1_b.ABS = 0;   // 0:绝对时间 1:相对时间
    ADC->QTR1_b.QTTR = 1;  // 设置默认绝对时间
    ADC->QTR2_b.ABS = 1;   // 0:绝对时间 1:相对时间
    ADC->QTR2_b.QTTR = 26; // 设置默认绝对时间
    ADC->QTR3_b.ABS = 1;   // 0:绝对时间 1:相对时间
    ADC->QTR3_b.QTTR = 26; // 设置默认绝对时间
    ADC->QTR4_b.ABS = 1;   // 0:绝对时间 1:相对时间
    ADC->QTR4_b.QTTR = 26; // 设置默认绝对时间
    ADC->QTR5_b.ABS = 1;   // 0:绝对时间 1:相对时间
    ADC->QTR5_b.QTTR = 25; // 设置默认相对时间
    ADC->QTR6_b.ABS = 1;   // 0:绝对时间 1:相对时间
    ADC->QTR6_b.QTTR = 25; // 设置默认相对时间
    ADC->QTR6_b.ABS = 1;   // 0:绝对时间 1:相对时间
    ADC->QTR6_b.QTTR = 25; // 设置默认相对时间

    /* 配置PWM控制参数 */
    EPWM->PWMRWEN_b.PWMRLDEN = 0x55; // 使能寄存器写入
    EPWM->PWMDT0H = 60;              // 设置PWM死区时间
    EPWM->PWMDT0L = 60;
    EPWM->PWMDT1H = 60;
    EPWM->PWMDT1L = 60;
    EPWM->PWMDT2H = 60;
    EPWM->PWMDT2L = 60;
    EPWM->PMANUALCON2 = 0x38;        // 手动输出控制 xH:0 xL:1
    EPWM->PMANUALCON1 = 0x3F;        // 输出由手动控制
    EPWM->PWMRWEN_b.PWMRLDEN = 0xAA; // 使能寄存器写入
}

/************************************************************
 * @brief: 机器检测后的反初始化
 * @return <None>
 ************************************************************/
void FLT_PwrCircuitDeinit(void)
{
    /* 配置ADC通道 */
    ADC->QCR1_b.Q1EN = 1;
    ADC->QCR1_b.Q1SEL = 3; // 电流传感放大器
    ADC->QCR1_b.Q2EN = 1;
    ADC->QCR1_b.Q2SEL = 3; // 电流传感放大器
    ADC->QCR1_b.Q3EN = 1;
    ADC->QCR1_b.Q3SEL = 3; // 电流传感放大器
    ADC->QCR1_b.Q4EN = 0;
    ADC->QCR1_b.Q4SEL = 3; // 电流传感放大器
    ADC->QCR1_b.Q5EN = 0;
    ADC->QCR1_b.Q5SEL = 4; // GDU相电压
    ADC->QCR1_b.Q6EN = 1;
    ADC->QCR1_b.Q6SEL = 1; // MVDD电源
    ADC->QCR2_b.Q7EN = 1;
    ADC->QCR2_b.Q7SEL = 2; // 温度传感器

    /* 恢复ADC转换时序配置 */
    ADC->QTR1_b.ABS = 0;   // 0:绝对时间 1:相对时间
    ADC->QTR1_b.QTTR = 1;  // 设置默认绝对时间
    ADC->QTR2_b.ABS = 0;   // 0:绝对时间 1:相对时间
    ADC->QTR2_b.QTTR = 26; // 设置默认绝对时间
    ADC->QTR3_b.ABS = 0;   // 0:绝对时间 1:相对时间
    ADC->QTR3_b.QTTR = 51; // 设置默认绝对时间
    ADC->QTR4_b.ABS = 0;   // 0:绝对时间 1:相对时间
    ADC->QTR4_b.QTTR = 76; // 设置默认绝对时间
    ADC->QTR5_b.ABS = 1;   // 0:绝对时间 1:相对时间
    ADC->QTR5_b.QTTR = 25; // 设置默认相对时间
    ADC->QTR6_b.ABS = 1;   // 0:绝对时间 1:相对时间
    ADC->QTR6_b.QTTR = 25; // 设置默认相对时间
    ADC->QTR6_b.ABS = 1;   // 0:绝对时间 1:相对时间
    ADC->QTR6_b.QTTR = 25; // 设置默认相对时间

    /* 恢复PWM死区时间设置 */
    EPWM->PWMRWEN_b.PWMRLDEN = 0x55; // 使能寄存器写入
    EPWM->PWMDT0H = FOC_EPWM_DEADTIME;
    EPWM->PWMDT0L = FOC_EPWM_DEADTIME;
    EPWM->PWMDT1H = FOC_EPWM_DEADTIME;
    EPWM->PWMDT1L = FOC_EPWM_DEADTIME;
    EPWM->PWMDT2H = FOC_EPWM_DEADTIME;
    EPWM->PWMDT2L = FOC_EPWM_DEADTIME;
    EPWM->PWMRWEN_b.PWMRLDEN = 0xAA; // 使能寄存器写入
}

#define BTM_SHRT_OPEN_THRESHOLD 300 // 底部开关短路/开路判断阈值
/************************************************************
 * @brief: 检测是否存在底部开关短路或线圈接地短路
 * @return i8Ret -1:短路
 ************************************************************/
int8_t FLT_BtmSworCoilShrtGnd(void)
{

    int8_t i8Ret = 0;
    int8_t i = 0;
    EPWM->PMANUALCON2 = 0; // 关闭所有开关
    GDU->GDUMUX = PH_A_NO;
    /* 等待100个周期以稳定相电压 */
    for (i = 0; i < 100; i++)
    {
        while (!EPWM->ISR_b.PWMZIF)
            ;
        EPWM->ISR_b.PWMZIF = 1;
    }

    /* 读取各相电压值 */
    for (i = 0; i < 3; i++)
    {
        while (!ADC->ISR_b.EOCIF)
            ;
        ADC->ISR_b.EOCIF = 1;
        g_i16PhsVol[i] = ADC->DR5_b.DATA;
        GDU->GDUMUX = i + 1;
    }
    g_i16Dcoft = ADC->DR1_b.DATA; // 获取直流偏移量

    /* 检查是否有相电压低于阈值，表示有短路 */
    for (int8_t i = 0; i < 3; i++)
    {
        if (g_i16PhsVol[i] < BTM_SHRT_OPEN_THRESHOLD)
        {
            /* 底部开关短路或线圈接地短路 */
            i8Ret = -1;
            break;
        }
    }

    return i8Ret;
}

/************************************************************
 * @brief: 检测是否存在顶部开关短路或线圈接电源短路
 * @return i8Ret -1:短路
 ************************************************************/
int8_t FLT_TopSworCoilShrtBat(void)
{
#define TOP_SHRT_THRESHOLD 500 // 顶部开关短路判断阈值
    int8_t i8Ret = 0;
    int8_t i = 0;
    int32_t i32SumCur = 0;
    EPWM->PMANUALCON2 = 0x7; // 打开所有底部开关
    GDU->GDUMUX = PH_A_NO;
    /* 等待1个周期以稳定相电压 */
    EPWM->ISR_b.PWMZIF = 1;
    while (!EPWM->ISR_b.PWMZIF)
        ;
    EPWM->ISR_b.PWMZIF = 1;

    /* 读取电流值并计算电流总和 */
    while (!ADC->ISR_b.EOCIF)
        ;
    g_i16DcCur[0] = ADC->DR1_b.DATA - g_i16Dcoft;
    g_i16DcCur[1] = ADC->DR2_b.DATA - g_i16Dcoft;
    g_i16DcCur[2] = ADC->DR3_b.DATA - g_i16Dcoft;
    g_i16DcCur[3] = ADC->DR4_b.DATA - g_i16Dcoft;
    i32SumCur = g_i16DcCur[0] + g_i16DcCur[1] + g_i16DcCur[2] + g_i16DcCur[3];

    /* 判断是否有短路情况 */
    if (i32SumCur > TOP_SHRT_THRESHOLD)
    {
        /* MOSFET存在缺陷，不一定是MOSFET短路 */
        i8Ret = -1;
    }
    else if (GDU->ISR_b.OCSHUNTIF || GDU->ISR_b.OCREG0LIF ||
             GDU->ISR_b.OCREG1LIF || GDU->ISR_b.OCREG2LIF)
    {
        /* 顶部MOSFET短路或线圈接电源短路 */
        i8Ret = -1;
    }

    return i8Ret;
}

/************************************************************
 * @brief: 检测是否存在底部开关或线圈开路
 * @return i8Ret -1: 开路
 ************************************************************/
int8_t FLT_BtmSworCoilOpen(void)
{
    int8_t i8Ret = 0;
    int8_t i = 0;
    /* MOUT0底部开关开路测试 */
    EPWM->PMANUALCON2 = 0x1; // 仅打开MOUT0的底部开关
    GDU->GDUMUX = PH_A_NO;
    /* 等待100个周期以稳定相电压 */
    for (i = 0; i < 100; i++)
    {
        while (!EPWM->ISR_b.PWMZIF)
            ;
        EPWM->ISR_b.PWMZIF = 1;
    }

    /* 读取各相电压值 */
    for (i = 0; i < 3; i++)
    {
        while (!ADC->ISR_b.EOCIF)
            ;
        ADC->ISR_b.EOCIF = 1;
        g_i16PhsVol[i] = ADC->DR5_b.DATA;
        GDU->GDUMUX = i + 1;
    }

    /* 判断是否存在开路情况 */
    if (g_i16PhsVol[PH_A_NO] > BTM_SHRT_OPEN_THRESHOLD ||
        g_i16PhsVol[PH_B_NO] > BTM_SHRT_OPEN_THRESHOLD ||
        g_i16PhsVol[PH_C_NO] > BTM_SHRT_OPEN_THRESHOLD)
    {
        /* 如果g_i16PhsVol[PH_A_NO] > BTM_SHRT_OPEN_THRESHOLD，表示MOUT0的底部开关开路
        如果g_i16PhsVol[PH_B_NO] > BTM_SHRT_OPEN_THRESHOLD，表示MOUT1的线圈开路
        如果g_i16PhsVol[PH_C_NO] > BTM_SHRT_OPEN_THRESHOLD，表示MOUT2的线圈开路 */
        i8Ret = -1;
    }

    /* MOUT1开路测试 */
    EPWM->PMANUALCON2 = 0x2; // 仅打开MOUT1的底部开关
    GDU->GDUMUX = PH_A_NO;
    /* 等待100个周期以稳定相电压 */
    for (i = 0; i < 100; i++)
    {
        while (!EPWM->ISR_b.PWMZIF)
            ;
        EPWM->ISR_b.PWMZIF = 1;
    }

    /* 读取各相电压值 */
    for (i = 0; i < 3; i++)
    {
        while (!ADC->ISR_b.EOCIF)
            ;
        ADC->ISR_b.EOCIF = 1;
        g_i16PhsVol[i] = ADC->DR5_b.DATA;
        GDU->GDUMUX = i + 1;
    }

    /* 判断是否存在开路情况 */
    if (g_i16PhsVol[PH_A_NO] > BTM_SHRT_OPEN_THRESHOLD ||
        g_i16PhsVol[PH_B_NO] > BTM_SHRT_OPEN_THRESHOLD ||
        g_i16PhsVol[PH_C_NO] > BTM_SHRT_OPEN_THRESHOLD)
    {
        /* 如果g_i16PhsVol[PH_B_NO] > BTM_SHRT_OPEN_THRESHOLD，表示MOUT1的底部开关开路
        如果g_i16PhsVol[PH_A_NO] > BTM_SHRT_OPEN_THRESHOLD，表示MOUT0的线圈开路
        如果g_i16PhsVol[PH_C_NO] > BTM_SHRT_OPEN_THRESHOLD，表示MOUT2的线圈开路 */
        i8Ret = -1;
    }

    /* MOUT2开路测试 */
    EPWM->PMANUALCON2 = 0x4; // 仅打开MOUT2的底部开关
    GDU->GDUMUX = PH_A_NO;
    /* 等待100个周期以稳定相电压 */
    for (i = 0; i < 100; i++)
    {
        while (!EPWM->ISR_b.PWMZIF)
            ;
        EPWM->ISR_b.PWMZIF = 1;
    }

    /* 读取各相电压值 */
    for (i = 0; i < 3; i++)
    {
        while (!ADC->ISR_b.EOCIF)
            ;
        ADC->ISR_b.EOCIF = 1;
        g_i16PhsVol[i] = ADC->DR5_b.DATA;
        GDU->GDUMUX = i + 1;
    }

    /* 判断是否存在开路情况 */
    if (g_i16PhsVol[PH_A_NO] > BTM_SHRT_OPEN_THRESHOLD ||
        g_i16PhsVol[PH_B_NO] > BTM_SHRT_OPEN_THRESHOLD ||
        g_i16PhsVol[PH_C_NO] > BTM_SHRT_OPEN_THRESHOLD)
    {
        /* 如果g_i16PhsVol[PH_C_NO] > BTM_SHRT_OPEN_THRESHOLD，表示MOUT2的底部开关开路
        如果g_i16PhsVol[PH_A_NO] > BTM_SHRT_OPEN_THRESHOLD，表示MOUT0的线圈开路
        如果g_i16PhsVol[PH_B_NO] > BTM_SHRT_OPEN_THRESHOLD，表示MOUT1的线圈开路 */
        i8Ret = -1;
    }
    return i8Ret;
}

/************************************************************
 * @brief: 主动注入脉冲检测单相中的顶部开关开路或线圈互相短路
 * @return i8Ret -1: 故障
 ************************************************************/
int8_t FLT_InjectPulseDetect(int8_t i8TopSw)
{
    int8_t i = 0;
    int8_t i8Ret = 0;
    static int32_t i32Sum = 0;
    /* 使用PWM生成器而非手动控制 */
    EPWM->PWMRWEN_b.PWMRLDEN = 0x55; // 使能寄存器写入
    EPWM->PMANUALCON2 = 0x38;        // 手动输出控制 xH:0 xL:1
    EPWM->PMANUALCON1 = 0x00;        // 输出由PWM生成器控制
    EPWM->PWMRWEN_b.PWMRLDEN = 0xAA; // 使能寄存器写入

    /* 设置各相PWM输出模式 */
    if (i8TopSw == PH_A_NO)
    {
        EPWM->PWMRWEN_b.PWMRLDEN = 0x55;      // 使能寄存器写入
        EPWM->PWM0DH = FOC_EPWM_PERIOD - 200; // A相高电平时间
        EPWM->PWM0DL = FOC_EPWM_PERIOD + 1;
        EPWM->PWM1DH = FOC_EPWM_PERIOD + 1; // B相高电平时间(关闭)
        EPWM->PWM1DL = FOC_EPWM_PERIOD + 1;
        EPWM->PWM2DH = FOC_EPWM_PERIOD + 1; // C相高电平时间(关闭)
        EPWM->PWM2DL = FOC_EPWM_PERIOD + 1;
        EPWM->PWMRWEN_b.PWMRLDEN = 0xAA; // 使能寄存器写入
    }
    else if (i8TopSw == PH_B_NO)
    {
        EPWM->PWMRWEN_b.PWMRLDEN = 0x55;    // 使能寄存器写入
        EPWM->PWM0DH = FOC_EPWM_PERIOD + 1; // A相高电平时间(关闭)
        EPWM->PWM0DL = FOC_EPWM_PERIOD + 1;
        EPWM->PWM1DH = FOC_EPWM_PERIOD - 200; // B相高电平时间
        EPWM->PWM1DL = FOC_EPWM_PERIOD + 1;
        EPWM->PWM2DH = FOC_EPWM_PERIOD + 1; // C相高电平时间(关闭)
        EPWM->PWM2DL = FOC_EPWM_PERIOD + 1;
        EPWM->PWMRWEN_b.PWMRLDEN = 0xAA; // 使能寄存器写入
    }
    else if (i8TopSw == PH_C_NO)
    {
        EPWM->PWMRWEN_b.PWMRLDEN = 0x55;    // 使能寄存器写入
        EPWM->PWM0DH = FOC_EPWM_PERIOD + 1; // A相高电平时间(关闭)
        EPWM->PWM0DL = FOC_EPWM_PERIOD + 1;
        EPWM->PWM1DH = FOC_EPWM_PERIOD + 1; // B相高电平时间(关闭)
        EPWM->PWM1DL = FOC_EPWM_PERIOD + 1;
        EPWM->PWM2DH = FOC_EPWM_PERIOD - 200; // C相高电平时间
        EPWM->PWM2DL = FOC_EPWM_PERIOD + 1;
        EPWM->PWMRWEN_b.PWMRLDEN = 0xAA; // 使能寄存器写入
    }
    ADC->QTR1_b.QTTR = FOC_EPWM_PERIOD - 100; // 设置ADC转换时间点

    /* 进行10次电流采样并累加 */
    for (i = 0; i < 10; i++)
    {
        while (!ADC->ISR_b.EOCIF)
            ;
        g_i16DcCur[0] = ADC->DR1_b.DATA - g_i16Dcoft;
        g_i16DcCur[1] = ADC->DR2_b.DATA - g_i16Dcoft;
        g_i16DcCur[2] = ADC->DR3_b.DATA - g_i16Dcoft;
        g_i16DcCur[3] = ADC->DR4_b.DATA - g_i16Dcoft;
        i32Sum += g_i16DcCur[0] + g_i16DcCur[1] + g_i16DcCur[2] + g_i16DcCur[3];
        ADC->ISR_b.EOCIF = 1;
    }

    /* 恢复手动控制模式 */
    EPWM->PWMRWEN_b.PWMRLDEN = 0x55; // 使能寄存器写入
    EPWM->PMANUALCON2 = 0x7;         // 手动输出控制 xH:0 xL:1
    EPWM->PMANUALCON1 = 0x3F;        // 输出由手动控制
    EPWM->PWMRWEN_b.PWMRLDEN = 0xAA; // 使能寄存器写入

    /* 判断是否有故障 */
    if (i32Sum < BTM_SHRT_OPEN_THRESHOLD)
    {
        /* 当前顶部开关有缺陷 */
        i8Ret = -1;
    }
    else if (GDU->ISR & 0x7F)
    {
        /* 线圈互相短路 */
        i8Ret = -1;
    }

    return i8Ret;
}

/************************************************************
 * @brief: 主动检测顶部开关开路或线圈互相短路
 * @return i8Ret -1: 故障
 ************************************************************/
int8_t FLT_TopSwOpenorCoilShrt(void)
{
    int8_t i8Ret = 0;

    /* 依次检测三相顶部开关 */
    i8Ret = FLT_InjectPulseDetect(PH_A_NO);
    i8Ret = FLT_InjectPulseDetect(PH_B_NO);
    i8Ret = FLT_InjectPulseDetect(PH_C_NO);
    return i8Ret;
}
/************************************************************
 * @brief: 电路或线圈故障检测
 * @return i8Ret -1: 故障
 ************************************************************/
int8_t FLT_PwrCircuitDetect(void)
{
    static int8_t i8Ret = 0;

    /* 初始化用于检测的ADC配置 */
    FLT_PwrCircuitInit();

    /* 检测底部开关短路或线圈接地短路 */
    i8Ret = FLT_BtmSworCoilShrtGnd();
    if (i8Ret)
    {
        tFaultMode.FM_b.BSW_COIL_SHRT_GND = 1; // 设置底部开关或线圈接地短路标志
        FLT_PwrCircuitDeinit();
        return i8Ret;
    }

    /* 检测顶部开关短路或线圈接电源短路 */
    i8Ret = FLT_TopSworCoilShrtBat();
    if (i8Ret)
    {
        tFaultMode.FM_b.TSW_COIL_SHRT_BAT = 1; // 设置顶部开关或线圈接电源短路标志
        FLT_PwrCircuitDeinit();
        return i8Ret;
    }

    /* 检测底部开关开路或线圈开路 */
    i8Ret = FLT_BtmSworCoilOpen();
    if (i8Ret)
    {
        tFaultMode.FM_b.BSW_COIL_OPEN = 1; // 设置底部开关或线圈开路标志
        FLT_PwrCircuitDeinit();
        return i8Ret;
    }

    /* 检测顶部开关开路或线圈互相短路 */
    i8Ret = FLT_TopSwOpenorCoilShrt();
    if (i8Ret)
    {
        tFaultMode.FM_b.TSW_OPEN_COIL_SHRT = 1; // 设置顶部开关开路或线圈互相短路标志
        FLT_PwrCircuitDeinit();
        return i8Ret;
    }

    /* 恢复ADC配置 */
    FLT_PwrCircuitDeinit();
    return i8Ret;
}