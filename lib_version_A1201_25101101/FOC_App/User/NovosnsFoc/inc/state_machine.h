/****************************************************************************
 * @file: state_machine.h
 * @author: Young Leon
 * @version: V1.0
 * @data: 2022/04/22
 * @brief: foc state machine header file.
 * @note:
 * @Copyright (C) 2022 Novosense All rights reserved.
 */
#ifndef __STATE_MACHINE_H__
#define __STATE_MACHINE_H__
#include "foc_paras.h"

/**
 * @brief 状态机函数指针类型
 * @details 定义状态机各状态对应的执行函数类型，所有状态处理函数需符合该函数签名（无参数、无返回值）
 */
typedef void (*pFunc_t)();

/**
 * @brief FOC应用层状态枚举（根据HFI使能动态调整）
 * @details 定义FOC系统的核心运行状态，状态机根据当前状态+触发事件切换至目标状态
 * @note ENABLE_HFI=1时包含HFI状态，=0时无HFI状态
 */
#if (ENABLE_HFI == 1)
typedef enum {
    INIT  = 0,          /*!< 0: 初始化状态 - 系统上电后初始化硬件/参数/变量 */
    FAULT = 1,          /*!< 1: 故障状态 - 检测到过压/过流/过温等故障时进入 */
    READY = 2,          /*!< 2: 就绪状态 - 初始化完成，等待启动指令 */
    CALIB = 3,          /*!< 3: 校准状态 - 电流/电压采样校准、电机参数辨识 */
    ALIGN = 4,          /*!< 4: 对齐状态 - 电机转子初始位置定位（开环对齐） */
    HFI   = 5,          /*!< 5: HFI状态 - 高频注入模式（低速无感启动） */
    RUN   = 6,          /*!< 6: 运行状态 - 正常闭环运行（反电动势无感控制） */
    RESET = 7           /*!< 7: 复位状态 - 故障清除后复位系统参数/状态 */
} AppStates; /* Application state identification user type*/

/**
 * @brief FOC应用层事件枚举（ENABLE_HFI=1时）
 * @details 定义触发状态机状态切换的事件类型，每个事件对应特定的状态转移逻辑
 */
typedef enum {
    E_FAULT       = 0,  /*!< 0: 故障事件 - 检测到故障，触发进入FAULT状态 */
    E_FAULT_CLEAR = 1,  /*!< 1: 故障清除事件 - 故障已排除，可退出FAULT状态 */
    E_INIT        = 2,  /*!< 2: 初始化事件 - 触发系统进入INIT状态 */
    E_INIT_DONE   = 3,  /*!< 3: 初始化完成事件 - INIT状态执行完毕，触发切换至READY */
    E_READY       = 4,  /*!< 4: 就绪事件 - 系统进入READY状态等待启动 */
    E_APP_ON      = 5,  /*!< 5: 启动事件 - 接收到运行指令，触发进入CALIB/ALIGN */
    E_CALIB       = 6,  /*!< 6: 校准事件 - 触发进入CALIB状态 */
    E_CALIB_DONE  = 7,  /*!< 7: 校准完成事件 - CALIB状态执行完毕，触发切换至ALIGN */
    E_ALIGN       = 8,  /*!< 8: 对齐事件 - 触发进入ALIGN状态 */
    E_ALIGN_DONE  = 9,  /*!< 9: 对齐完成事件 - ALIGN状态执行完毕，触发切换至HFI */
    E_HFI         = 10, /*!< 10: HFI事件 - 触发进入HFI状态 */
    E_HFI_DONE    = 11, /*!< 11: HFI完成事件 - HFI状态执行完毕，触发切换至RUN */
    E_RUN         = 12, /*!< 12: 运行事件 - 触发进入RUN状态 */
    E_APP_OFF     = 13, /*!< 13: 停止事件 - 接收到停止指令，触发退出RUN状态 */
    E_RESET       = 14, /*!< 14: 复位事件 - 触发进入RESET状态 */
    E_RESET_DONE  = 15  /*!< 15: 复位完成事件 - RESET状态执行完毕，触发切换至INIT/READY */
} AppEvents; /* Application event identification user type*/

/**
 * @brief 状态机函数表（ENABLE_HFI=1时）
 * @details 二维数组，维度[事件数][状态数]，每个元素为对应"事件-状态"组合的处理函数指针
 * @note 数组大小16(事件)×8(状态)，覆盖所有事件和状态的组合
 */
extern const pFunc_t pStateFuncTable[16][8]; // 修改数组大小

#else
/**
 * @brief FOC应用层状态枚举（ENABLE_HFI=0时，无HFI状态）
 */
typedef enum {
    INIT  = 0,          /*!< 0: 初始化状态 - 系统上电后初始化硬件/参数/变量 */
    FAULT = 1,          /*!< 1: 故障状态 - 检测到过压/过流/过温等故障时进入 */
    READY = 2,          /*!< 2: 就绪状态 - 初始化完成，等待启动指令 */
    CALIB = 3,          /*!< 3: 校准状态 - 电流/电压采样校准、电机参数辨识 */
    ALIGN = 4,          /*!< 4: 对齐状态 - 电机转子初始位置定位（开环对齐） */
    RUN   = 5,          /*!< 5: 运行状态 - 正常闭环运行（反电动势无感控制） */
    RESET = 6           /*!< 6: 复位状态 - 故障清除后复位系统参数/状态 */
} AppStates; /* Application state identification user type*/

/**
 * @brief FOC应用层事件枚举（ENABLE_HFI=0时，无HFI相关事件）
 */
typedef enum {
    E_FAULT       = 0,  /*!< 0: 故障事件 - 检测到故障，触发进入FAULT状态 */
    E_FAULT_CLEAR = 1,  /*!< 1: 故障清除事件 - 故障已排除，可退出FAULT状态 */
    E_INIT        = 2,  /*!< 2: 初始化事件 - 触发系统进入INIT状态 */
    E_INIT_DONE   = 3,  /*!< 3: 初始化完成事件 - INIT状态执行完毕，触发切换至READY */
    E_READY       = 4,  /*!< 4: 就绪事件 - 系统进入READY状态等待启动 */
    E_APP_ON      = 5,  /*!< 5: 启动事件 - 接收到运行指令，触发进入CALIB/ALIGN */
    E_CALIB       = 6,  /*!< 6: 校准事件 - 触发进入CALIB状态 */
    E_CALIB_DONE  = 7,  /*!< 7: 校准完成事件 - CALIB状态执行完毕，触发切换至ALIGN */
    E_ALIGN       = 8,  /*!< 8: 对齐事件 - 触发进入ALIGN状态 */
    E_ALIGN_DONE  = 9,  /*!< 9: 对齐完成事件 - ALIGN状态执行完毕，触发切换至RUN */
    E_RUN         = 10, /*!< 10: 运行事件 - 触发进入RUN状态 */
    E_APP_OFF     = 11, /*!< 11: 停止事件 - 接收到停止指令，触发退出RUN状态 */
    E_RESET       = 12, /*!< 12: 复位事件 - 触发进入RESET状态 */
    E_RESET_DONE  = 13  /*!< 13: 复位完成事件 - RESET状态执行完毕，触发切换至INIT/READY */
} AppEvents; /* Application event identification user type*/

/**
 * @brief 状态机函数表（ENABLE_HFI=0时）
 * @details 二维数组，维度[事件数][状态数]，每个元素为对应"事件-状态"组合的处理函数指针
 * @note 数组大小14(事件)×7(状态)，覆盖所有事件和状态的组合
 */
extern const pFunc_t pStateFuncTable[14][7];
#endif

/**
 * @brief 状态机状态转移结构体
 * @details 存储当前系统的状态和触发的事件，作为状态机处理的核心输入参数
 */
typedef struct {
    AppStates tStatus;  /*!< 当前系统状态 */
    AppEvents tEvent;   /*!< 触发的事件 */
} APP_State_Transfer_t;

/**
 * @brief 故障状态处理函数
 * @details 处理故障状态的核心逻辑：关闭PWM输出、记录故障码、停止电机、等待故障清除
 */
extern void StateFault(void);

/**
 * @brief 初始化状态处理函数
 * @details 执行系统初始化：硬件初始化（PWM/ADC/GDU）、参数初始化、变量清零、状态初始化
 */
extern void StateInit(void);

/**
 * @brief 就绪状态处理函数
 * @details 就绪状态逻辑：检测系统状态、等待启动指令、实时监控故障（未启动时的保护）
 */
extern void StateReady(void);

/**
 * @brief 校准状态处理函数
 * @details 执行校准逻辑：电流采样零点校准、电压采样校准、电机参数辨识（电阻/电感/磁链）
 */
extern void StateCalib(void);

/**
 * @brief 对齐状态处理函数
 * @details 执行转子对齐逻辑：施加定向电压/电流，锁定转子初始位置，为无感启动做准备
 */
extern void StateAlign(void);

/**
 * @brief HFI状态处理函数（仅ENABLE_HFI=1时生效）
 * @details 执行高频注入逻辑：低速无感启动，通过高频电压注入检测转子位置，拖到闭环转速
 */
extern void StateHFI(void);  // 新增HFI状态函数声明

/**
 * @brief 运行状态处理函数
 * @details 执行正常闭环运行逻辑：FOC核心算法（Clark/Park变换、PI调节、SVPWM）、转速/电流闭环控制
 */
extern void StateRun(void);

/**
 * @brief 复位状态处理函数
 * @details 执行复位逻辑：清除故障码、复位系统参数、复位状态机、准备重新初始化
 */
extern void StateReset(void);

#endif