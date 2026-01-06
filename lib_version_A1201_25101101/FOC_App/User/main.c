/****************************************************************************
 * @file    : main.c
 * @author  : Novosense
 * @version : V1.0
 * @Date    : 2022/6/20
 * @brief   : main file
 * @note
 * Copyright (C) 2021 Novosense All rights reserved.
 ****************************************************************************/
#include "include.h"
#include "stdio.h"
#include "nvsns_foc.h"
#include "Module_Define.h"
#include "sleep_control.h"
#include "testFun.h"
#include "hfi_lib.h"

// --------------------- 测试：三角波与正反切换数据 ---------------------
// volatile Frac16_t g_f16TriWave10ms = 0;          // Q15 三角波：-32768 .. 32767
// volatile uint8_t  g_u8TriIncreasing10ms = 10;     // 1 上升，0 下降
// volatile Frac16_t g_f16TriStep10ms = FRAC16(0.01); // 步进（约0.01），中断频率20kHz时约20ms周期
// volatile int8_t   g_i8TestDir10ms = +1;          // 方向标志：+1 正，-1 反
// volatile int16_t g_u16TriDuty10ms = 32767;          // 映射为 0..PWMPeriod 的占空比值
// ---
struct TaskConfig {
    uint16_t taskID;
    uint16_t period;
    uint16_t tick;
    uint16_t event;
    void (*taskFun)(void);
};

#define TASK_ELEMENT_CONFIG(id, period, tick, fn)       {id, period, tick, 0, fn}

extern void LinApplMain(void);
// extern void HandleLinCmd(void);
extern void UpdateLinATCPStatus(void);
extern void UpdateLinATCPFault(void);
extern void FeedDWDG(void);
extern void PWMIO_Init(void);
extern void PWMIO_Task10ms(void);
extern void FOC_1MSTASK(void);
extern void PWMIO_ResetState(void);
extern void InitMotorConfig(void);

void confirmComType(void)
{
#if (COM_TYPE == COM_TYPE_ALL)
    //通讯方式已经确认
    if (g_u8ComType != COM_TYPE_ALL){
        return;
    }
    //500MS没收到确认帧，那么就是PWM模式
    if (g_u16ComTypeConfirmCnt > COM_TYPE_CONFIRM_CNT) {
        g_u8ComType = COM_TYPE_PWM;
        PWMIO_Init();
        g_u16ComTypeConfirmCnt = 0;
    } else {
        g_u16ComTypeConfirmCnt++;
    }

    if (l_get_confirm_com_type_cmd()) {
        l_data_buffer_trans_g_tATCPDebug_struct(g_nodeConfigFrameTable[NO_CONFIRM_COM_TYPE_FRAME].var, &g_tATCPDebug);
        if ((g_tATCPDebug.AS_b.Debugflag == 0xFAFA) && (g_tATCPDebug.AS_b.DeviceType == 0x02) && (g_tATCPDebug.AS_b.Debugflag2 == 0xFA) && (g_tATCPDebug.AS_b.Debugflag3 == 0xFAFAFAFA)) {
            g_u8ComType = COM_TYPE_LIN;
            g_u16ComTypeConfirmCnt = 0;
            l_clr_confirm_com_type_cmd();
        }
    }
#else
    g_u8ComType = COM_TYPE;

#endif
}


void Task1msProcess(void)
{   
    // GPIO->PDO_b.DO0 = 1;
    AlarmCheck1msTask();
    confirmComType();
#if ((IS_RELEASE_VERSION == 1) || (IS_RELEASE_VERSION == 2))

    getMotorCmdSpeed();
#endif
    MotorStateCtrl();
    FOC_1MSTASK();
    CheckSleepStatus();
#if ((COM_TYPE & 0x2) == COM_TYPE_LIN)
    if (g_u8ComType == COM_TYPE_LIN) {
        UpdateLinATCPStatus();
        UpdateLinATCPFault();
    }
#endif
    // GPIO->PDO_b.DO0 = 0;
}

void Task10msCanE2promProcess(void)
{
           // --------------------- 更新测试三角波与方向数据 ---------------------
    // {
    //     // 计算下一步三角波
    //     int32_t next = (int32_t)g_f16TriWave10ms + (g_u8TriIncreasing10ms ? (int32_t)g_f16TriStep10ms : -(int32_t)g_f16TriStep10ms);

    //     if (next >= 32767) {
    //         next = 32767;
    //         g_u8TriIncreasing10ms = 0;   // 触顶转为下降
    //         g_i8TestDir10ms = -1;        // 方向切换为反向
    //     } else if (next <= -32768) {
    //         next = -32768;
    //         g_u8TriIncreasing10ms = 1;   // 触底转为上升
    //         g_i8TestDir10ms = +1;        // 方向切换为正向
    //     }

    //     g_f16TriWave10ms = (Frac16_t)next;

    //     // 将 Q15 三角波映射为 0..PWMPeriod 的占空比
    //     // 先将 [-32768,32767] 映射为 [0,65535]，再按 PWMPeriod 缩放

    //     g_u16TriDuty10ms = -g_u16TriDuty10ms;
    // }
    // ------------------------------------------------------------------
}

void Task10msProcess(void)
{
    // GPIO->PDO_b.DO0 = 1;
    FeedDWDG();
    AlarmCheck10msTask();
#if ((COM_TYPE & 0x2) == COM_TYPE_LIN)
#ifdef L_CONFIG_IDENTIFIER
    /* call LIN tp layer Ncr timeout */
    LIN_TpLayerNCrTimeoutScheduler();
#endif
#endif

    PWMIO_Task10ms();
    // GPIO->PDO_b.DO0 = 0;
    
}

// 结果结构体（便于在调试器里查看）
typedef struct {
    Frac16_t x_q15;   // 输入X（Q15）
    Frac16_t y_q15;   // 输入Y（Q15）
    Frac16_t ang_q15; // 输出角度（Q15，一周=65536）
} AtanXY_TestPoint_t;

// 全局结果缓存（volatile 以免被优化掉）
volatile AtanXY_TestPoint_t g_AtanXY_Test[8];
// 运行测试：生成8个角度的XY并计算角度输出
void AtanXYF16_RunPresetTests(void)
{
    // 预计算用到的常数（Q15），避免运行时三角函数
    const Frac16_t C30 = FRAC16(0.8660254037844386); // cos(30°) = √3/2
    const Frac16_t S30 = FRAC16(0.5);                // sin(30°)
    const Frac16_t C60 = FRAC16(0.5);                // cos(60°)
    const Frac16_t S60 = FRAC16(0.8660254037844386); // sin(60°)

    // 按顺序：30°, 60°, 120°, 150°, -30°, -60°, -120°, -150°
    const Frac16_t x_list[8] = {
        C30,  C60,  (Frac16_t)-C60, (Frac16_t)-C30,
        C30,  C60,  (Frac16_t)-C60, (Frac16_t)-C30
    };
    const Frac16_t y_list[8] = {
        S30,  S60,  S60,            S30,
        (Frac16_t)-S30, (Frac16_t)-S60, (Frac16_t)-S60, (Frac16_t)-S30
    };

    for (int i = 0; i < 8; ++i) {
        Frac16_t x = x_list[i];
        Frac16_t y = y_list[i];
        Frac16_t ang = MM_AtanXYF16(x, y); // 角度（Q15）

        g_AtanXY_Test[i].x_q15   = x;
        g_AtanXY_Test[i].y_q15   = y;
        g_AtanXY_Test[i].ang_q15 = ang;
    }
}
void Task100msProcess(void)
{
    //tDrvFoc.i16IntMOSTemp = CalcIgbtTemp((uint16_t)tDrvFoc.i16AdcRaw[5]);
    //GPIO->PDO_b.DO0 = ~GPIO->PDO_b.DO0;
    // AtanXYF16_RunPresetTests();
}



void Task1000msProcess(void)
{
    // GPIO->PDO_b.DO0 = 1;
	tDrvFoc.i16IntMOSTemp = CalcIgbtTemp((uint16_t)tDrvFoc.i16AdcRaw[5]);
    // GPIO->PDO_b.DO0 = ~GPIO->PDO_b.DO0;
    g_epochSecond++;
    // GPIO->PDO_b.DO0 = 0;
    if (tHfiCtrl.bCalculatePhysicalValues)
    {
        HFI_CalculateInductanceValues(&tHfiCtrl);
    }

}


struct TaskConfig g_sysTask[] = {
    TASK_ELEMENT_CONFIG(1, 1, 0, Task1msProcess),
    TASK_ELEMENT_CONFIG(2, 10, 7,  Task10msCanE2promProcess),
    TASK_ELEMENT_CONFIG(3, 10, 3,  Task10msProcess),
    TASK_ELEMENT_CONFIG(4, 100, 11,  Task100msProcess),
    TASK_ELEMENT_CONFIG(5, 1000, 13,  Task1000msProcess),
};
#define TASK_NUM (sizeof(g_sysTask) / sizeof(struct TaskConfig))

void MainTask(void)
{
    struct TaskConfig *task = &g_sysTask[0];
    struct TaskConfig *taskEnd = &g_sysTask[TASK_NUM - 1];

    if (g_tickCnt < 2) {
        return;
    }

    g_tickCnt = 0;

    for (; task <= taskEnd; task++) {
        if (++task->tick >= task->period) {
            task->event = 1;
        }
        if (task->event) {
            task->event = 0;
            task->tick = 0;
            task->taskFun();
        }
    }
}

int main()
{
    InitMotorConfig();
    g_u8ComType = COM_TYPE;  // 通讯方式选择
#if (COM_TYPE  == COM_TYPE_PWM)   // 只有是确切的PWM模式才初始化为PWM，如果是ALL模式，由代码重新初始化
    PWMIO_Init();
    PWMIO_ResetState();
#endif

#if (KL15_WAKEUP_ENABLE == 1)
    Init_HVIO();
    SLEEP_EnableHVIOWake();
#endif
    LinApplMain();
    Derating_Init(&g_derating_controller, &g_motor_derating_config);
#if (TEST_MODE == 1)
    StartMotorStartupTest(100,FRAC16(600/MAX_RPM)); // 启动电机启动成功率测试，测试100次，测试速度200RPM
#endif
//GPIO->PDO_b.DO0 = 0;
    for(;;)
    {   
        MainTask();
//		GPIO->PDO_b.DO0 = ~GPIO->PDO_b.DO0;
//		delay_ms(1000);
    }

    return 0;
}
