// 在文件顶部添加测试相关的定义

#include "testFun.h"
#include "foc_paras.h"
#include "nvsns_foc.h"
#include "include.h"
#include "global.h"
#include "fault_diagnose.h"
#include "config.h"

#if (TEST_MODE == 1)
//是否需要随机停机时间
#define RANDOM_STOP_TIME_ENABLED 1 // 启用随机停机时间


// 测试模式相关定义
#define MOTOR_TEST_DISABLED     0   // 测试禁用
#define MOTOR_TEST_RUNNING      1   // 电机运行阶段
#define MOTOR_TEST_STOPPING     2   // 电机停止阶段
#define MOTOR_TEST_WAITING_RECOVERY 3 // 等待堵转恢复阶段
#define MOTOR_TEST_COOLING      4   // 堵转后冷却阶段

// 测试参数定义
#define MOTOR_TEST_RUN_TIME     8000  // 运行时间10秒 (10s = 1000 * 10ms)
#define MOTOR_TEST_STOP_TIME    6000  // 停止时间10秒 (10s = 1000 * 10ms)
#define MOTOR_TEST_MAX_CYCLES   100   // 最大测试循环次数，可根据需要调整

// 随机数生成与限幅相关定义
#define RANDOM_TIME_VARIATION     5000   // 运行/停止时间随机变化范围 ±300 (±3s)
#define RANDOM_SPEED_VARIATION    FRAC16(300/MAX_RPM) // 速度随机变化范围 ±300 RPM

//测试使能指令
uint8_t g_u8MotorTestEnable = 0;

static MotorTestStatus_t motorTest = {
    .testMode = MOTOR_TEST_DISABLED,
    .timer = 0,
    .totalCycles = 0,
    .successCount = 0,
    .failCount = 0,
    .currentCycleValid = 0,
    .f16testSpeed = 0,
};

// 在文件中添加以下函数

/************************************************************
 * @brief: 启动电机启动成功率测试
 * @param: cycles - 要测试的循环次数，0表示无限循环
 * @return <None>
 ************************************************************/
void StartMotorStartupTest(uint16_t cycles,Frac16_t testSpeed)
{
    // 初始化测试参数
    motorTest.testMode = MOTOR_TEST_STOPPING; // 从停止状态开始
    motorTest.timer = 0;
    motorTest.totalCycles = cycles;
    motorTest.successCount = 0;
    motorTest.failCount = 0;
    motorTest.currentCycleValid = 0;
    motorTest.f16testSpeed = testSpeed; // 初始化测试速度为0
    motorTest.testCnt = 0; // 初始化测试计数器
    // 输出测试开始信息
    // 可以添加串口打印或其他方式记录测试开始
}

/************************************************************
 * @brief: 停止电机启动成功率测试
 * @param: None
 * @return <None>
 ************************************************************/
void StopMotorStartupTest(void)
{
    motorTest.testMode = MOTOR_TEST_DISABLED;
    
    // 输出测试结果
    // 可以添加串口打印或其他方式显示测试结果
}


// 获取基于SYSTICK的有限范围随机数
static int16_t GetLimitedRandom(int16_t maxRange)
{

   static uint32_t seed=0;

   seed ^=SysTick->VAL ^ EPWM->PWM0DH;

   seed=(1664525 *seed+ 1013904223);

   int16_t random_val=(int16_t)((seed>>16)%(2*maxRange+1))-maxRange;


    // 从SYSTICK获取当前计数值作为随机种子
    //uint32_t systick_val = SysTick->VAL;
    // 从PWM计数器获取当前值
   // uint32_t pwm_counter = EPWM->PWM0DH;

    //组合SYSTICK和PWM计数器的值作为随机种子
   // systick_val ^= pwm_counter; // 使用异或操作组合两个值


    // 生成[-maxRange, maxRange]范围内的随机数
   // int16_t random_val = (int16_t)((systick_val & 0xFFFF) % (2 * maxRange + 1)) - maxRange;

    return random_val;
}


/************************************************************
 * @brief: 开始新的测试循环
 * @param: None
 * @return <None>
 ************************************************************/
static void StartNewTestCycle(void)
{
    motorTest.testMode = MOTOR_TEST_RUNNING;
    motorTest.timer = 0;
    motorTest.currentCycleValid = 1; // 标记新循环为有效

#if (RANDOM_STOP_TIME_ENABLED == 1)
    // 计算随机运行时间
    motorTest.stopTime = -1000 + GetLimitedRandom(RANDOM_TIME_VARIATION);
    // 确保最小运行时间
    if (motorTest.stopTime < 0) {
        motorTest.stopTime = 0;
    } else if (motorTest.stopTime > MOTOR_TEST_STOP_TIME * 2) {
        motorTest.stopTime = MOTOR_TEST_STOP_TIME * 2;
    } 
#else
    // 如果禁用随机停机时间，则使用固定值
    motorTest.stopTime = MOTOR_TEST_STOP_TIME;
#endif
    
    // 启动电机
    // 计算随机速度
    int16_t speedVariation = GetLimitedRandom(RANDOM_SPEED_VARIATION);
    int16_t targetSpeed = motorTest.f16testSpeed + speedVariation;
    
    // 确保速度在合理范围内
    if (targetSpeed < FRAC16(PWM_MIN_RPM/MAX_RPM)) {
        targetSpeed = FRAC16(PWM_MIN_RPM/MAX_RPM);
    }
    if (targetSpeed > FRAC16(PWM_MAX_RPM/MAX_RPM)) {
        targetSpeed = FRAC16(PWM_MAX_RPM/MAX_RPM);
    }
    
    // 启动电机，使用随机调整后的速度
    tDrvFoc.tPospeControl.f16wRotElReq = targetSpeed;
    //测试速度要对SYSTICK计数器进行取余操作得到一个随机附加速度，并将速度值限制在100转内
    if (tDrvFoc.tAppState.tStatus == FAULT) {
        tDrvFoc.tAppState.tEvent = E_FAULT_CLEAR;
    }

    // 可以添加输出信息记录新循环开始
    // printf("Starting new motor test cycle\r\n");
}

/************************************************************
 * @brief: 电机启动测试任务，应在10ms周期中调用
 * @param: None
 * @return <None>
 ************************************************************/
void MotorStartupTestTask(void)
{
    // 如果测试模式禁用，直接返回
    if (motorTest.testMode == MOTOR_TEST_DISABLED) {
        return;
    }
    if (g_u8MotorTestEnable == 0){
        tDrvFoc.tAppState.tEvent = E_FAULT; // 设置事件为故障，表示当前运行结束
        tDrvFoc.tPospeControl.f16wRotElReq = 0;
        return;
    }
    
    // 检测堵转并立即结束当前运行状态
    if ((tFaultMode.FM_b.MOT_STALL == 1 || tFaultMode.FM_b.MOT_HOC == 1) && motorTest.currentCycleValid == 1) {
        // 记录失败并重置当前循环有效标志
        motorTest.currentCycleValid = 0;
        motorTest.failCount++;
        
        // 立即结束当前运行状态，切换到等待堵转恢复状态
        motorTest.testMode = MOTOR_TEST_WAITING_RECOVERY;
        motorTest.timer = 0;
        tDrvFoc.tAppState.tEvent = E_FAULT;
        
        // 停止电机
        tDrvFoc.tPospeControl.f16wRotElReq = 0;
        
        // 可以添加输出信息记录失败
        // printf("Motor startup test failed: Stall detected\r\n");
        return;
    }

    motorTest.testCnt = motorTest.successCount + motorTest.failCount;

    // 测试状态机处理
    switch (motorTest.testMode) {
        case MOTOR_TEST_RUNNING:
            // 运行阶段
            motorTest.timer++;
            
            if (motorTest.timer >= MOTOR_TEST_RUN_TIME) {
                // 运行时间结束，切换到停止阶段
                motorTest.testMode = MOTOR_TEST_STOPPING;
                motorTest.timer = 0;
                tDrvFoc.tAppState.tEvent = E_FAULT; // 设置事件为故障，表示当前运行结束
                
                // 如果当前循环有效，记录为成功
                if (motorTest.currentCycleValid) {
                    motorTest.successCount++;
                    // 可以添加输出信息记录成功
                    // printf("Motor startup test cycle succeeded\r\n");
                }
                
                // 停止电机
                tDrvFoc.tPospeControl.f16wRotElReq = 0;
            }
            break;
            
        case MOTOR_TEST_STOPPING:
            // 停止阶段
            motorTest.timer++;
            
            if (motorTest.timer >= motorTest.stopTime) {
                // 停止时间结束，开始新的测试循环
                
                // 检查是否达到最大测试循环次数
                if (motorTest.totalCycles > 0 && 
                    (motorTest.successCount + motorTest.failCount) >= motorTest.totalCycles) {
                    // 测试完成
                    StopMotorStartupTest();
                    // 输出最终测试结果
                    PrintMotorTestResults();
                    return;
                }
                
                // 开始新的运行阶段
                StartNewTestCycle();
            }
            break;
            
        case MOTOR_TEST_WAITING_RECOVERY:
            // 等待堵转恢复阶段
            // 在这个状态下，我们不增加timer，仅检查堵转是否已恢复
            if (tFaultMode.u32FaultMode == 0) {
                // 堵转已恢复，等待一个短暂的冷却周期
                motorTest.testMode = MOTOR_TEST_COOLING;
                motorTest.timer = 0;
            }
            break;
            
        case MOTOR_TEST_COOLING:
            // 堵转后冷却阶段
            motorTest.timer++;
            
            if (motorTest.timer >= MOTOR_TEST_STOP_TIME / 2) { // 使用一半的停止时间作为冷却时间
                // 冷却完成，开始新的测试循环
                StartNewTestCycle();
            }
            break;
    }
}


// 添加以下函数，用于外部接口控制测试的启动和停止
/************************************************************
 * @brief: 获取测试状态和结果
 * @param: None
 * @return 测试状态结构体的指针
 ************************************************************/
MotorTestStatus_t* GetMotorTestStatus(void)
{
    return &motorTest;
}

/************************************************************
 * @brief: 输出测试结果
 * @param: None
 * @return <None>
 ************************************************************/
void PrintMotorTestResults(void)
{
    // 这里可以根据实际情况实现，例如通过串口打印
    // printf("Motor Startup Test Results:\r\n");
    // printf("Total Cycles: %u\r\n", motorTest.successCount + motorTest.failCount);
    // printf("Success Count: %u\r\n", motorTest.successCount);
    // printf("Fail Count: %u\r\n", motorTest.failCount);
    // printf("Success Rate: %.2f%%\r\n", 
    //     (motorTest.successCount + motorTest.failCount) > 0 ?f16wRotElReq 
    //     (float)motorTest.successCount * 100.0f / (motorTest.successCount + motorTest.failCount) : 0);
}


#endif