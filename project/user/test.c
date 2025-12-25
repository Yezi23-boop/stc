#include "zf_common_headfile.h"
/************************* 全局变量 *************************/
// 计时结果（单位：微秒）
u32 gStart_us = 0;
u32 gEnd_us = 0;
VOL_U8 SpillCnt = 0;
extern u16 cpu_usage;

/*
 * 测试速度阶跃序列
 * 0–500: 20，500–1000: 15，1000–2000: 18，2000–3000: -40，>=3000: 60
 * 定期调用以验证速度环与电机输出。
 */
float test_speed = 0;
// 状态机实例
StateMachine_t g_state_machine;

void test(void)
{
    static int test_time = 0;
    test_time++;
    Encoder_get(&PID.left_speed, &PID.right_speed);
    if (test_time >= 4000)
    {
        test_speed = 60;
        test_time = 0;
    }
    else if (test_time >= 3000)
    {
        test_speed = -40;
    }
    else if (test_time >= 2000)
    {
        test_speed = 18;
    }
    else if (test_time >= 1000)
    {
        test_speed = 15;
    }
    else if (test_time >= 500)
    {
        test_speed = 20;
    }
    // 更新速度环 PID 并将输出用于电机控制
    pid_speed_update(&PID.left_speed, test_speed, speed_l);  // 左轮速度环
    pid_speed_update(&PID.right_speed, test_speed, speed_r); // 右轮速度环
    g_motor_driver.set_pwm((int)PID.left_speed.output, (int)PID.right_speed.output);
    //  motor_output(-1000, -5000);
}

// 获取开始时间
u32 GetTimeStart(void)
{
    u32 TL0_Time8Bit = TL0;
    u32 TH0_Time8Bit = (u32)(TH0 << 8);
    return TH0_Time8Bit + TL0_Time8Bit;
}

// 获取结束时间，0~65535，为一个周期，
u32 GetTimeEnd(void)
{

    if (SpillCnt)
    {
        //  gStart_us = GetTimeStart（）获取开始的时间
        u32 middle_time = SpillCnt * 65536 - gStart_us;
        // 当前 0~65535 的时间
        u32 TH0_Time8Bit = (u32)(TH0 << 8);
        u32 TL0_Time8Bit = TL0;
        SpillCnt = 0;
        return TH0_Time8Bit + TL0_Time8Bit + middle_time;
    }
    else
    {
        u32 TL0_Time8Bit = TL0;
        u32 TH0_Time8Bit = (u32)(TH0 << 8);
        u32 TimeEnd = TH0_Time8Bit + TL0_Time8Bit;
        return TimeEnd - gStart_us;
    }
}
// 总时间为 = MACHINE_CYCLE*（GetTimeEnd()-GetTimeStart()）
f32 GetTotalTime(void)
{
    return MACHINE_CYCLE * gEnd_us;
}

void TimingStart(void)
{
    SpillCnt = 0;
    gStart_us = GetTimeStart();
}

u32 TimingStopTicks(void)
{
    gEnd_us = GetTimeEnd();
    return gEnd_us;
}

f32 TimingStopSeconds(void)
{
    gEnd_us = GetTimeEnd();
    return MACHINE_CYCLE * (f32)gEnd_us;
}

// 处理事件
void state_machine_process_event(CarEvent_t event)
{
    u8 i;
    // 查找匹配的状态转移
    for (i = 0; i < sizeof(state_transitions) / sizeof(state_transitions[0]); i++)
    {
        if (state_transitions[i].from_state == g_state_machine.current_state &&
            state_transitions[i].event == event)
        {

            // 记录状态转移
            g_state_machine.last_state = g_state_machine.current_state;
            g_state_machine.current_state = state_transitions[i].to_state;
            g_state_machine.state_start_time = 0; // 会在update中设置
            g_state_machine.timeout = (u16)state_transitions[i].timeout;

            // printf("State change: %s -> %s\n",
            //        state_names[g_state_machine.last_state],
            //        state_names[g_state_machine.current_state]);
            return;
        }
    }

    // printf("No transition for current state\n");  //这个printf是没有实现的，所以注释掉
}

// 关闭外事的输出，清零标志位
void state_idle(void)
{
    // 停止电机;
    g_motor_driver.set_pwm(0, 0);
    // 清理编码器
    g_encoder_driver.clear(TIM3_ENCOEDER);
    g_encoder_driver.clear(TIM4_ENCOEDER);
    // 蜂鸣器

    // 其他外设的输出

    // PID积分清零
    if (KEY_START_MOTOR)
    {
        state_machine_process_event(EV_START);
    }
}

// 循迹状态处理
void state_line_follow(void)
{

    if (cpu_usage > 8000)
    {
        state_machine_process_event(EV_ERROR);
        return;
    }
    pid_steer_update(&PID.steer, Err, -imu660ra_gyro_z * 0.01);
    pid_speed_update(&PID.left_speed, speed_run - PID.steer.output, PID.left_speed.speed);
    pid_speed_update(&PID.right_speed, speed_run + PID.steer.output, PID.right_speed.speed);
    if (flat_statr >= 2 && lost_spto == 0)
    {
        g_motor_driver.set_pwm((int)PID.left_speed.output, (int)PID.right_speed.output);
    }
    // 其他跳转的判断
    // 转弯：短时间内转过90°

    // 跷跷板：短时间俯仰角上升

    // 圆筒面：

    // 垂直面：

    // 环岛：

    // 十字

    // 丢线
}

// 跷跷板状态处理
void state_cross(void)
{
    // // 在交叉口停止
    // g_motor_driver.set_pwm(0,0);

    // // 等待1秒
    // static uint32_t stop_time = 0;
    // if (stop_time == 0) {
    //     stop_time = get_time_ms();
    //     printf("Stopping at intersection\n");
    // } else if (get_time_ms() - stop_time > 1000) {
    //     // 1秒后开始转弯
    //     state_machine_process_event(EV_START);  // 触发转弯
    //     stop_time = 0;
    // }
}

// 偏航状态处理（丢线处理）
void state_yaw(void)
{
#define YAWTIMEOUT 5000        // 5S
    static u16 StopCnt = 1000; // 停止1S
    static u8 search_step = 0;
    static u32 search_time = 0;
    // 速度环输出0，希望小车1S内停在原地
    if (StopCnt > 0)
    {
        PID.steer.output = 0;
        pid_speed_update(&PID.left_speed, speed_run - PID.steer.output, PID.left_speed.speed);
        pid_speed_update(&PID.right_speed, speed_run + PID.steer.output, PID.right_speed.speed);
        StopCnt--;
        return;
    }

    // 尝试小范围搜索线路
    search_time++;
    g_state_machine.timeout++;
    // 每500ms切换搜索方向
    if (search_time > 500)
    {
        search_step = (search_step + 1) % 4;
        search_time = 0;
    }
    switch (search_step)
    {
    case 0:
        g_motor_driver.set_pwm(200, -200);
        break; // 左转
    case 1:
        g_motor_driver.set_pwm(-200, 200);
        break; // 右转
    case 2:
        g_motor_driver.set_pwm(150, 150);
        break; // 前进
    case 3:
        g_motor_driver.set_pwm(-150, -150);
        break; // 后退
    }
    // 检查是否找到线路
    if (0 == lost_spto)
    {
        state_machine_process_event(EV_LINE_FOUND);
        StopCnt = 1000;
    }
    // 偏航超时
    if (g_state_machine.timeout > YAWTIMEOUT)
    {
        state_machine_process_event(EV_ERROR);
        g_state_machine.timeout = 0;
    }
}

// 错误状态处理
void state_error(void)
{

    // 等待恢复信号（例如按下复位按钮）
    // if (button_pressed()) {
    //     state_machine_process_event(EV_RECOVER);
    // g_state_machine.timeout = 0;
    // }

    // 超时，跳到空闲
    g_state_machine.timeout++;
    if (g_state_machine.timeout > 5000)
    {
        g_state_machine.timeout = 5000;
        state_machine_process_event(EV_RECOVER);
        return;
    }
    // 速度环输出0，希望小车停在原地
    PID.steer.output = 0;
    pid_speed_update(&PID.left_speed, speed_run - PID.steer.output, PID.left_speed.speed);
    pid_speed_update(&PID.right_speed, speed_run + PID.steer.output, PID.right_speed.speed);

    // 错误处理：闪烁LED、蜂鸣器报警等
    return;
}

// 初始化状态机
void state_machine_init(void)
{
    g_state_machine.current_state = STATE_IDLE;
    g_state_machine.last_state = STATE_IDLE;
    g_state_machine.state_start_time = 0;
    g_state_machine.timeout = 0;

    // printf("State machine initialized to IDLE\n");
}
CarState_t state_machine_get_state(void)
{
    return g_state_machine.current_state;
}
