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
#include "nvsns_foc.h"
#include "foc_paras.h"

//---------------时间参数-------------------
#define VOLT_CHECK_CNT 200 // 1.2s  10ms 50次
#define VOLT_CLEAR_CNT 100 // 1s  10ms 100次

#define OTP_CHECK_CNT      100             // 1s  10ms 100次
#define OTP_CLEAR_CNT      100             // 1s  10ms 100次
#define TIME1S_B10ms     100


//---------------------堵转保护参数定义
#define STALL_RECOVER_CNT     5000     // 5秒 = 5000 * 1ms
#define STALL_SHORT_TIMEOUT   6000    // 1分钟 = 6000 * 10ms
#define STALL_LOCK_TIMEOUT    90000 //90000   // 15分钟 = 90000 * 10ms
#define MAX_STALL_COUNT_FOR_LOCK 3      // 短期内触发锁死的堵转次数
#define MAX_STALL_LOCK_CYCLES    3      // 触发永久锁死的堵转锁死次数 (达到此次数后，g_stallLockTimes不再重置，除非重启)
#define STALL_LOCK_FLAG_CLEAR_TIMEOUT 12000 // 堵转锁死标志清除时间 2分钟 = 12000 * 10ms (用于 multiStallClrCnt)

// #define LC_RECOVER_CNT   	6000             // 60s  10ms

#define LP_RECOVER_CNT      1000        // 10s  10ms 
#define PHASE_UNBALANCE_DEBOUNCE_CNT 10 // 不平衡检测消抖计数 10*10ms = 100ms
#define PHASE_LACK_CONFIRM_CNT       50 // 缺相确认计数 50*10ms = 500ms
/** 机器诊断定义 */
#define PH_A_NO 0
#define PH_B_NO 1
#define PH_C_NO 2

//static int32_t g_i32ISum = 0;         // 电流总和
//static uint16_t g_u16BridgeOpen = 0;  // 桥臂开路标志
//static uint16_t g_u16BridgeShort = 0; // 桥臂短路标志

//static int16_t g_i16PhsVol[3] = {0}; // 相电压数组
//static int16_t g_i16DcCur[4] = {0};  // 直流电流数组
//static int16_t g_i16Dcoft = 2048;    // 直流偏移量

// 堵转保护相关全局变量
uint16_t g_stallTimes = 0;        // 短期内堵转次数
uint16_t g_stallLockTimes = 0;    // 堵转锁定次数
uint32_t stallTimeOut = 0;        // 堵转次数计数超时
uint32_t stallLockTimeOut = 0;    // 堵转锁定超时
uint32_t multiStallClrCnt = 0;    // 多次堵转清除计数

int16_t currentU;
int16_t currentV;
int16_t currentW;

int16_t currentU_Max;
int16_t currentV_Max;
int16_t currentW_Max;

int16_t currentU_Min;
int16_t currentV_Min;
int16_t currentW_Min;

Fault_Mode_t tFaultMode; // 故障模式结构体



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
    else if (i16VdcAft <= tFaultCfg.i16SftOVRecTh)
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
//static int16_t i16Ramp = 0;
void CheckBusUvp(int16_t i16Vdc)
{

    static int16_t i16VdcAft; // 总线电压处理后值
    static int16_t setCnt = 0;
    static int16_t clearCnt = 0;
 
    i16VdcAft = i16Vdc;
    // //升速阶段不做欠压
    // if ((tDrvFoc.tPospeControl.f16wRotE1 - 12000 > -400) || tDrvFoc.tPospeControl.f16wRotElReq <= 12000){
    //     i16Ramp++;
    //     setCnt++;
    // } else {
    //     i16Ramp = 0;
    // }
    // // 5S
    // if (i16Ramp > 500){
    //     tFaultCfg.i16SftUVTh = tFocParas.U_DCB_UNDERVOLTAGE;
    //     tFaultCfg.i16SftUVRecTh = tFaultCfg.i16SftUVTh+FRAC16(UV_VOLTAGE_RECOV/BUS_VOLTAGE_UPLIMT);
    // } else {
    //     tFaultCfg.i16SftUVTh = tFocParas.U_DCB_UNDERVOLTAGE- FRAC16((0.5f)/BUS_VOLTAGE_UPLIMT);
    //     tFaultCfg.i16SftUVRecTh = tFaultCfg.i16SftUVTh+FRAC16(UV_VOLTAGE_RECOV/BUS_VOLTAGE_UPLIMT);
    // }

    if (i16VdcAft < tFaultCfg.i16SftUVTh)
    {
        setCnt++;
        clearCnt = 0;
    }
    else if (i16VdcAft >= tFaultCfg.i16SftUVRecTh)
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
 * @brief: 芯片过温
 * @param: i16Temp, 温度值
 * @return <None>
 ************************************************************/
void CheckChipOTp(int16_t i16Temp)
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

void CheckHardwareOcp(void)
{
#define CHECK_DELAY_CNT 1 // 不做延迟

    static int16_t delayCnt = 0;
    static int16_t startupCnt = 0;
    static int16_t hwOcpTimes = 0;
    static int16_t ocpLimitCnt = 0;
    static int16_t hwOcp = 0;

    static int16_t checkCnt = 0;


    if (checkCnt < CHECK_DELAY_CNT) {
        checkCnt++;
    }
    ocpLimitCnt = 1;


    if (tFaultMode.FM_b.MOT_HOC == 1) {
        if ((hwOcp == 0)) {
            delayCnt = 500; //5s 恢复
        }
        hwOcp = 1;
    }

    if (delayCnt > 0) {
        delayCnt--;
    } else {
        if (hwOcp) {
        
            hwOcp = 0;
			
            hwOcpTimes++;
            if (hwOcpTimes > ocpLimitCnt) {
                hwOcpTimes = 0;
            } else {
                // g_delayOnTime = EAS_FAST_ON_DELAY;
            }
        }
    }

    if (checkCnt >= CHECK_DELAY_CNT) {
        tFaultMode.FM_b.MOT_HOC = hwOcp;
    }
}

//硬件驱动器短路 与过流一样
// void CheckHardwareShortCircuit(void)
// {
//     static int16_t delayCnt = 0;
//     static int16_t startupCnt = 0;
//     static int16_t hwShortCircuitTimes = 0;
//     static int16_t shortCircuitLimitCnt = 0;
//     static int16_t hwShortCircuit = 0;

//     static int16_t checkCnt = 0;

//     if (checkCnt < CHECK_DELAY_CNT) {
//         checkCnt++;
//     }
//     shortCircuitLimitCnt = 1;

//     if (tFaultMode.FM_b.ECU_DRV_SHORT == 1) {
//         if ((hwShortCircuit == 0)) {
//             delayCnt = 500; //5s 恢复
//         }
//         hwShortCircuit = 1;
//     }

//     if (delayCnt > 0) {
//         delayCnt--;
//     } else {
//         if (hwShortCircuit) {

//             hwShortCircuit = 0;

//             hwShortCircuitTimes++;
//             if (hwShortCircuitTimes > shortCircuitLimitCnt) {
//                 hwShortCircuitTimes = 0;
//             } else {
//                 // g_delayOnTime = EAS_FAST_ON_DELAY;
//             }
//         }
//     }

//     if (checkCnt >= CHECK_DELAY_CNT) {
//         tFaultMode.FM_b.ECU_DRV_SHORT = hwShortCircuit;
//     }
// }


void CheckHardwareShortCircuit(void)
{
    static int16_t delayCnt = 0;            // 恢复延时计数器
    static int16_t shortCircuitTimes = 0;   // 短路次数计数
    static int16_t resetTimeCnt = 0;        // 重置时间计数器(10ms递增)
    static int16_t hwShortCircuit = 0;      // 硬件短路状态
    static int16_t checkCnt = 0;            // 检测延迟计数器

    if (checkCnt < CHECK_DELAY_CNT) {
        checkCnt++;
    }

    // 检测到硬件短路故障
    if (tFaultMode.FM_b.ECU_DRV_SHORT == 1) {
        if (hwShortCircuit == 0) {
            // 根据当前短路次数确定恢复时间
            if (shortCircuitTimes == 0) {
                delayCnt = 100;           // 第一次触发：1000ms恢复 (100 * 10ms)
            } else if (shortCircuitTimes == 1) {
                delayCnt = 150;           // 第二次触发：2000ms恢复 (200 * 10ms)
            } else {
                delayCnt = 500;           // 第三次及以上触发：5000ms恢复 (500 * 10ms)
            }
            
            // 记录短路发生，重置重置时间计数器
            shortCircuitTimes++;
            resetTimeCnt = 0;
            hwShortCircuit = 1;
        }
    }

    // 恢复延时倒计时
    if (delayCnt > 0) {
        delayCnt--;
    } else {
        // 恢复延时结束，清除短路标志
        if (hwShortCircuit) {
            hwShortCircuit = 0;
        }
    }
    
    // 重置时间计数 (30s = 1000 * 10ms)
    if (shortCircuitTimes > 0 && hwShortCircuit == 0) {
        resetTimeCnt++;
        if (resetTimeCnt >= 3000) {  // 30秒内没有新的短路触发
            // 重置短路次数和重置时间计数器
            shortCircuitTimes = 0;
            resetTimeCnt = 0;
        }
    }

    // 更新硬件短路状态
    if (checkCnt >= CHECK_DELAY_CNT) {
        tFaultMode.FM_b.ECU_DRV_SHORT = hwShortCircuit;
    }
}

//堵转检测
// 修改后的堵转检测函数
void CheckStall(void)
{
    static int16_t stallRecoverTimer = 0;  // 堵转恢复计时器（单位：10ms）
    
    // 如果当前处于堵转锁定状态，直接返回，不执行普通堵转检测
//    if (tFaultMode.FM_b.MOT_STALL_LOCK) {
//        return;
//    }
    
    // 检测堵转
    // if (tFaultMode.FM_b.MOT_STALL == 0) {
    //     // if (stallDetection1ms(&tStallDetection, &tDrvFoc)) {
    //     //     tFaultMode.FM_b.MOT_STALL = 1; // 设置堵转故障标志
    //     //     // 此处可以添加事件记录，类似于 RunTrailInqueue(EVENT_STALL);
    //     // }
    // }
    
    // 如果当前处于堵转状态
    if (tFaultMode.FM_b.MOT_STALL) {
        // 计时5秒后恢复
        if (stallRecoverTimer < STALL_RECOVER_CNT) {
            stallRecoverTimer++;
        } else {
            // 恢复计时器达到5秒，清除堵转故障
            stallRecoverTimer = 0;
            tFaultMode.FM_b.MOT_STALL = 0;
        }
    } else {
        // 如果不处于堵转故障状态，重置计时器
        stallRecoverTimer = 0;
    }

}


/************************************************************
 * @brief: 堵转锁死状态检测与处理
 * @param: None
 * @return <None>
 * @note: 1分钟出现3次则锁死2分钟，记为g_stallLockTimes，15分钟内出现3次g_stallLockTimes，永久锁死
 * @note:该函数实现电机堵转故障的锁死与解锁逻辑。
 *        1. 永久锁死: 当堵转锁死次数 (g_stallLockTimes) 达到预设上限 (MAX_STALL_LOCK_CYCLES) 时，
 *           电机将进入永久锁死状态 (MOT_STALL_LOCK = 1)，除非系统重启或外部干预，否则不会自动解除。
 *        2. 堵转事件检测: 检测 MOT_STALL 标志位从0变为1的上升沿。
 *           - 若短期堵转次数 (g_stallTimes) 为0，则启动短期堵转计数超时计时器 (stallTimeOut)。
 *           - 短期堵转次数 (g_stallTimes) 累加。
 *        3. 短期堵转超时: stallTimeOut 递减，超时后清零 g_stallTimes。
 *        4. 触发堵转锁死: 当短期堵转次数 (g_stallTimes) 达到预设值 (MAX_STALL_COUNT_FOR_LOCK) 时：
 *           - 若堵转锁死总次数 (g_stallLockTimes) 为0，则启动堵转锁死总次数超时计时器 (stallLockTimeOut)。
 *           - 堵转锁死总次数 (g_stallLockTimes) 累加。
 *           - 设置堵转锁死标志的持续时间计时器 (multiStallClrCnt)。
 *           - 设置堵转锁死标志 (tFaultMode.FM_b.MOT_STALL_LOCK = 1)。
 *           - 清零短期堵转次数 (g_stallTimes) 和短期堵转计数超时计时器 (stallTimeOut)。
 *        5. 堵转锁死标志持续时间: multiStallClrCnt 递减。
 *           - 计时结束后，若未达到永久锁死条件，则清除堵转锁死标志 (tFaultMode.FM_b.MOT_STALL_LOCK = 0)。
 *        6. 堵转锁死总次数超时: stallLockTimeOut 递减。
 *           - 计时结束后，若未达到永久锁死条件，则清零堵转锁死总次数 (g_stallLockTimes)。
 ************************************************************/
void CheckStallLockState(void)
{
    static uint16_t stallPre = 0; // 用于检测MOT_STALL标志的上升沿

    // 如果堵转锁死次数达到上限，则永久锁死（除非重启或外部干预）
    // MOT_STALL_LOCK 将保持为1，相关计时器不再递减，g_stallLockTimes 也不再重置。
    if (g_stallLockTimes >= MAX_STALL_LOCK_CYCLES) {
        tFaultMode.FM_b.MOT_STALL_LOCK = 1;
        return; // 退出，不再执行后续的计时器递减和状态解除逻辑
    }

    // 检测堵转事件的发生（MOT_STALL 从 0 变为 1）
    if (tFaultMode.FM_b.MOT_STALL && (stallPre == 0)) {
        if (g_stallTimes == 0) {
            // 如果是短期内的第一次堵转，启动短期堵转计数超时计时器
            stallTimeOut = STALL_SHORT_TIMEOUT; // 例如 1 分钟
        }
        g_stallTimes++; // 堵转次数累加
    }
    stallPre = tFaultMode.FM_b.MOT_STALL; // 更新上一次的堵转状态

    // 短期堵转计数超时处理
    if (stallTimeOut > 0) {
        stallTimeOut--;
    } else {
        // 超时，清零短期堵转次数
        g_stallTimes = 0;
    }

    // 如果短期内堵转次数达到设定值
    if (g_stallTimes >= MAX_STALL_COUNT_FOR_LOCK) {
        if (g_stallLockTimes == 0) {
            // 如果是第一次触发堵转锁死，启动堵转锁死总次数的超时计时器
            stallLockTimeOut = STALL_LOCK_TIMEOUT; // 例如 15 分钟
        }
        
        g_stallLockTimes++; // 堵转锁死总次数累加
        multiStallClrCnt = STALL_LOCK_FLAG_CLEAR_TIMEOUT; // 设置堵转锁死标志的持续时间，例如 2 分钟
        tFaultMode.FM_b.MOT_STALL_LOCK = 1; // 设置堵转锁死标志
        g_stallTimes = 0; // 清零短期堵转次数
        stallTimeOut = 0; // 清零短期堵转计数超时计时器
        // RunTrailInqueue(EVENT_STALL_LOCK); // 如果有事件记录，可以在此记录堵转锁死事件
    }

    // 堵转锁死标志持续时间处理
    if (multiStallClrCnt > 0) {
        multiStallClrCnt--;
    } else {
        // 持续时间到，清除堵转锁死标志 (前提是未达到永久锁死条件)
        if (g_stallLockTimes < MAX_STALL_LOCK_CYCLES) {
             tFaultMode.FM_b.MOT_STALL_LOCK = 0;
        }
    }

    // 堵转锁死总次数超时处理
    if (stallLockTimeOut > 0) {
        stallLockTimeOut--;
    } else {
        // 超时，清零堵转锁死总次数 (前提是未达到永久锁死条件)
        // 如果已达到MAX_STALL_LOCK_CYCLES，则g_stallLockTimes不会在此处被清除，因为函数会在开头返回。
        // 此处逻辑主要用于在未达到MAX_STALL_LOCK_CYCLES时，经过足够长时间后重置g_stallLockTimes。
        if (g_stallLockTimes < MAX_STALL_LOCK_CYCLES) { // 确保不会意外清除已达上限的计数
             g_stallLockTimes = 0;
        }
    }
}

#define ADRESULT_UPLIMIT   4085
#define ADRESULT_DNLIMIT    10
//检查温度传感器开短路 根据ADC阈值
void CheckMOSOTpSensor(int16_t i16Temp)
{
    static uint8_t u8ShortCnt = 0;
    static uint8_t u8OpenCnt = 0;
    
    // 检测温度传感器短路或开路
    if (i16Temp <= ADRESULT_DNLIMIT) {
        u8ShortCnt++;
        if (u8ShortCnt >= TIME1S_B10ms) {
            u8ShortCnt = TIME1S_B10ms;
            tFaultMode.FM_b.ECU_OT_SENS_SHRT = 1;  // 设置传感器短路标志位
        }
    } else {
        if (u8ShortCnt > 0) {
            u8ShortCnt--;
            if (u8ShortCnt == 0) {
                tFaultMode.FM_b.ECU_OT_SENS_SHRT = 0;  // 清除传感器短路标志位
            }
        }
    }
    
    if (i16Temp >= ADRESULT_UPLIMIT) {
        u8OpenCnt++;
        if (u8OpenCnt >= TIME1S_B10ms) {
            u8OpenCnt = TIME1S_B10ms;
            tFaultMode.FM_b.ECU_OT_SENS_OPEN = 1;   // 设置传感器开路标志位
        }
    } else {
        if (u8OpenCnt > 0) {
            u8OpenCnt--;
            if (u8OpenCnt == 0) {
                tFaultMode.FM_b.ECU_OT_SENS_OPEN = 0;   // 清除传感器开路标志位
            }
        }
    }
    
}

/**
 * @brief: 使用电角度周期重置的方式获取三相电流最大最小值
 */
void getCurrentMaxMin(void)
{
    static int16_t prevTheta = 0;   // 上一次的电角度
    static int16_t thetaDiff = 0;   // 电角度变化累计值
    static bool firstRun = true;    // 首次运行标志
    
    // 获取当前三相电流
    currentU = tDrvFoc.tIabcFbck.f16Arg1;
    currentV = tDrvFoc.tIabcFbck.f16Arg2;
    currentW = tDrvFoc.tIabcFbck.f16Arg3;
    
    // 获取电角度（归一化到-32768~32767，对应-π到π）
    int16_t currentTheta = tDrvFoc.tPospeControl.f16ThetaRotEl;
    
    // 首次运行或电机停止时初始化
    if (firstRun || (AbsF16(tDrvFoc.tPospeControl.f16wRotE1) < 100)) {
        currentU_Max = currentU;
        currentU_Min = currentU;
        currentV_Max = currentV;
        currentV_Min = currentV;
        currentW_Max = currentW;
        currentW_Min = currentW;
        prevTheta = currentTheta;
        thetaDiff = 0;
        firstRun = false;
        return;
    }
    
    // 计算角度变化（处理角度跳变的情况）
    int16_t deltaTheta = currentTheta - prevTheta;
    
    // 处理角度跳变（从-π到π或从π到-π）
    if (deltaTheta > 16384) {  // 32768/2 = 16384，超过半圈认为是反向跳变
        deltaTheta -= 32768;
    } else if (deltaTheta < -16384) {
        deltaTheta += 32768;
    }
    
    // 累计角度变化
    thetaDiff += AbsF16(deltaTheta);  // 使用绝对值以支持正反转
    
    // 更新当前电流的最大最小值
    if (currentU > currentU_Max) {
        currentU_Max = currentU;
    } else if (currentU < currentU_Min) {
        currentU_Min = currentU;
    }
    
    if (currentV > currentV_Max) {
        currentV_Max = currentV;
    } else if (currentV < currentV_Min) {
        currentV_Min = currentV;
    }
    
    if (currentW > currentW_Max) {
        currentW_Max = currentW;
    } else if (currentW < currentW_Min) {
        currentW_Min = currentW;
    }
    
    // 当累计角度变化超过一个电周期（32768表示2π）时重置
    if (thetaDiff >= 32768) {
        // 在这里可以执行基于当前最大最小值的计算或存储操作
        // 例如: SaveCurrentPeakValues(); 或 CalculateRmsValues();
        
        // 重置角度累计值，但保留超出部分以维持准确性
        thetaDiff -= 32768;
        
        // 将当前值作为新周期的初始最大最小值
        currentU_Max = currentU;
        currentU_Min = currentU;
        currentV_Max = currentV;
        currentV_Min = currentV;
        currentW_Max = currentW;
        currentW_Min = currentW;
    }
    
    // 保存当前角度作为下一次计算的基准
    prevTheta = currentTheta;
}

int32_t currAbsSum = 0; // 三相电流绝对值之和
/**
 * @brief: 计算三相电流绝对值之和乘以0.25
 * @return int16_t: 三相电流绝对值之和乘以0.25的结果
 */
int16_t GetCurrentAbsSum025(void)
{
    // 获取三相电流绝对值
    int16_t absU = AbsF16(currentU);
    int16_t absV = AbsF16(currentV);
    int16_t absW = AbsF16(currentW);

    currAbsSum = (int32_t)absU + (int32_t)absV + (int32_t)absW;

    // 乘以0.25 (相当于右移2位，即除以4)其实只是为了定点数处理方便
    int16_t currAbsSum = (int16_t)(currAbsSum >> 2);

    return currAbsSum;
}

int16_t ChangePeak2Rms(int16_t *positive, int16_t *negtive)
{
    int16_t I_RMS_SCALE;
    I_RMS_SCALE = FRAC16(0.70710678118654752440084436210485f); // sqrt(2)/2
	int16_t tmp = -(*negtive);
	if (tmp < *positive) {tmp = *positive;}

	*positive = 0;
	*negtive = 0;

	// tmp = (int32_t)tmp * I_RMS_SCALE >> 15;
    tmp = MulF16(tmp, I_RMS_SCALE); // 使用乘法函数进行缩放
	return tmp;
}

void BubbleSort (int16_t* data, int len) 
{
    int temp;
    int i, j;
    for (i = 0; i <len - 1; i++) { 
        for (j=0; j<len-1-i; j++) { /* 内循环为每趟比较的次数，第i趟比较len-i次 */
            if (data[j] > data[j+1]) { /* 相邻元素比较，若逆序则交换（升序为左大于右，降序反之） */
                temp = data[j];
                data[j] = data[j+1];
                data[j+1] = temp;
            }
        }
	}
}

int32_t lackCnt = 0;       // 缺相计数器
int16_t unbalanceU = 0;
int16_t unbalanceV = 0;
int16_t unbalanceW = 0;
/**
 * @brief      检测电机缺相故障
 * @note       该函数通过分析三相电流的有效值(RMS)来判断电机是否发生缺相。
 *             为了提高检测的准确性并避免误判，采用了多重判断标准和时间滤波机制。
 *             该函数应在定时任务中（如10ms任务）周期性调用。
 * 
 * @par        检测策略详细说明:
 *             1. **前提条件**:
 *                - 仅在电机进入无感或有感闭环控制模式 (`tDrvFoc.tPosMode >= TRACKING`) 且系统无其他故障时，才执行缺相检测。
 *                - 这是为了避免在电机启动、低速或已有其他故障时产生误判。
 * 
 *             2. **数据采集与处理**:
 *                - 使用 `ChangePeak2Rms` 函数将一个电气周期内采集到的三相电流最大/最小值转换为有效值(RMS)。
 *                - 对三相电流有效值进行排序，以方便找出最大值和最小值。
 * 
 *             3. **不平衡判断标准 (核心逻辑)**:
 *                - 同时满足以下任一条件，则认为当前采样周期存在不平衡：
 *                  a) **相对不平衡**: 最小相的电流有效值小于最大相电流有效值的30%。 (适用于一相完全断开的场景)
 *                  b) **绝对不平衡**: 任意一相的电流有效值小于三相电流平均值的50%。 (适用于电流严重不平衡的场景)
 * 
 *             4. **故障确认 (时间滤波)**:
 *                - 当检测到电流不平衡时，启动一个不平衡计数器 `unbalanceCnt`。
 *                - 如果不平衡状态持续超过10个检测周期（约100ms），则启动缺相计数器 `lackCnt`。
 *                - 如果缺相状态持续，当 `lackCnt` 达到50（约500ms）时，正式确认发生缺相故障。
 *                - 确认故障后，设置全局缺相故障标志位 `tFaultMode.FM_b.ECU_LACK_PHASE = 1`。
 *                - 同时，根据哪一相电流最小来判断并标记具体的缺相相别 (`unbalanceU/V/W`)。
 * 
 *             5. **故障恢复**:
 *                - 如果电机恢复正常运行（电流平衡），或电机停止/低速运行，则启动一个恢复计数器 `clrCnt`。
 *                - 当正常状态持续足够长的时间（`clrCnt` 达到1000，约10秒）后，系统将清除缺相故障标志及所有相关状态，实现自动恢复。
 */
void CheckLackPhase(void)
{
    // 使用电流最大最小值计算各相电流有效值
    int16_t currRms[3];
    int16_t currRmsSorted[3];
    static int32_t unbalanceCnt = 0;  // 不平衡计数器

    static int32_t clrCnt = 0;        // 恢复计数器
    
    // 计算三相电流有效值
    currRms[0] = ChangePeak2Rms(&currentU_Max, &currentU_Min);
    currRms[1] = ChangePeak2Rms(&currentV_Max, &currentV_Min);
    currRms[2] = ChangePeak2Rms(&currentW_Max, &currentW_Min);
    
    // 复制到排序数组并排序
    currRmsSorted[0] = currRms[0];
    currRmsSorted[1] = currRms[1];
    currRmsSorted[2] = currRms[2];
    BubbleSort(currRmsSorted, 3);
    
    // 计算电机转速的绝对值，用于避免低速误判
    int16_t absSpeed = AbsF16(tDrvFoc.tPospeControl.f16wRotE1);
    
    // 只在电机转速达到一定值时进行缺相检测
    if ((tDrvFoc.tPosMode >= TRACKING) && (tDrvFoc.tAppState.tEvent != E_FAULT)) {  // 约为额定转速的20%
        // 判断电流不平衡情况
        // 1. 最小电流过低（小于最大电流的30%）
        // 2. 三相中有一相电流低于平均值的50%
        int16_t avgCurrent = (currRms[0] + currRms[1] + currRms[2]) / 3;
        
        // 判断是否有缺相情况
        bool isPhaseUnbalance = false;
        
        // 缺相时，最小相电流通常显著低于最大相电流
        if (currRmsSorted[0] < (currRmsSorted[2] * 3 / 10)) {
            isPhaseUnbalance = true;
        }
        
        // 查找异常低的相电流
        for (int i = 0; i < 3; i++) {
            if (currRms[i] < (avgCurrent / 2)) {
                isPhaseUnbalance = true;
                break;
            }
        }
        
        if (isPhaseUnbalance) {
            unbalanceCnt++;
            clrCnt = 0;
					
					if (unbalanceCnt > PHASE_UNBALANCE_DEBOUNCE_CNT){
							lackCnt++;
							// 持续不平衡达到阈值，判定为缺相
							if (lackCnt >= PHASE_LACK_CONFIRM_CNT) {  // 1秒 (10ms x 100)
									lackCnt = PHASE_LACK_CONFIRM_CNT;
									// 确认缺相故障
											// 设置缺相标志位 (假设在Fault_Mode_t中有此位)
											tFaultMode.FM_b.ECU_LACK_PHASE = 1;
											
											// 可以进一步识别缺相的具体相位
											if (currRms[0] < (avgCurrent / 2)) {
													// U相缺相
													// 记录U相缺相
													unbalanceU = 1;
											} else if (currRms[1] < (avgCurrent / 2)) {
													// V相缺相
													// 记录V相缺相
													unbalanceV = 1;
											} else if (currRms[2] < (avgCurrent / 2)) {
													// W相缺相
													// 记录W相缺相
													unbalanceW = 1;
											}
							}
						}
        } else {
            unbalanceCnt = 0;
            
        }
    } else {
        // 电机停止或低速时，重置检测计数器
        unbalanceCnt = 0;
        //clrCnt = 0;
			  // 电流平衡，清除缺相状态
				if (clrCnt < LP_RECOVER_CNT) {
						clrCnt++;
				} else {
						lackCnt = 0;
						// 清除缺相标志位
						tFaultMode.FM_b.ECU_LACK_PHASE = 0;
						unbalanceU = 0;
						unbalanceV = 0;
						unbalanceW = 0;
				}
    }
}


void AlarmCheck10msTask(void)
{
	// CheckBusOvp(tDrvFoc.f16DcBusFilt);
	// CheckBusUvp(tDrvFoc.f16DcBusFilt);

    // CheckChipOTp(tDrvFoc.i16IntMOSTemp);

    CheckHardwareOcp();
    CheckHardwareShortCircuit();
    // CheckStallLockState();

    // CheckMOSOTpSensor(tDrvFoc.i16AdcRaw[5]);
    // CheckLackPhase();
}

void AlarmCheck1msTask(void)
{
    //CheckStall();
}



