#ifndef TESTFUN_H
#define TESTFUN_H

#include "nvsns_foc.h"
// 测试状态全局变量
typedef struct {
    uint8_t  testMode;          // 测试模式 (0:禁用, 1:运行, 2:停止)
    uint16_t timer;             // 当前阶段计时器
    uint16_t totalCycles;       // 总测试循环次数
    uint16_t successCount;      // 成功启动次数
    uint16_t failCount;         // 启动失败次数
    uint8_t  currentCycleValid; // 当前循环是否有效(用于判断本次启动是否成功)
    int16_t f16testSpeed;      // 测试速度 (定点数)
    int16_t testCnt;          // 测试计数器
    uint16_t runTime;           // 当前循环的运行时间（随机变化）
    uint16_t stopTime;          // 当前循环的停止时间（随机变化）
} MotorTestStatus_t;

extern MotorTestStatus_t* GetMotorTestStatus(void);
extern void StartMotorStartupTest(uint16_t cycles, Frac16_t testSpeed);
extern void StopMotorStartupTest(void);
extern void PrintMotorTestResults(void);

#endif // TESTFUN_H