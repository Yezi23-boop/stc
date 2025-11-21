#include "pid.h"
PID_struct motors_pid;
LowPassFilter_t encoder_l;
LowPassFilter_t encoder_r;
float delta_output = 0, speed_l = 0, speed_r = 0, max_integral = 0;
void PID_Init(PID *pid, float Kp, float Ki, float Kd, float max_output,float min_output)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->error = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev2_error = 0.0f;
    pid->output = 0.0f;
    pid->integral = 0.0f;
    pid->max_output = max_output;
	pid->min_output = min_output;
}
// 编码器读取
void Encoder_get(void)
{
    // 读取编码器计数值并转换为速度（乘以系数0.2调整单位）
    speed_l = encoder_get_count(TIM4_ENCOEDER) * 0.2f;  // 左轮速度
    speed_r = -encoder_get_count(TIM3_ENCOEDER) * 0.2f; // 右轮速度（负号用于方向调整）
    low_pass_filter_mt(&encoder_l, &speed_l, 0.8);
    low_pass_filter_mt(&encoder_r, &speed_r, 0.8);
    // 清除编码器计数，准备下一次计数
    encoder_clear_count(TIM3_ENCOEDER);
    encoder_clear_count(TIM4_ENCOEDER);
}
// 增量式PID
float PID_Calculate(PID *pid, float setpoint, float actual)
{
    // 计算当前误差
    pid->error = setpoint - actual;

    // 计算增量输出
    delta_output = pid->Kp * (pid->error - pid->prev_error) + pid->Ki * pid->error + pid->Kd * (pid->error - 2.0f * pid->prev_error + pid->prev2_error);

    // 更新输出值
    pid->output += delta_output;

    // 输出限幅
    if (pid->output > pid->max_output)
    {
        pid->output = pid->max_output;
    }
    else if (pid->output < -pid->max_output)
    {
        pid->output = -pid->max_output;
    }

    // 更新误差历史
    pid->prev2_error = pid->prev_error;
    pid->prev_error = pid->error;

    return pid->output;
}

// 位置式PID计算
float PID_Positional_Calculate(PID *pid, float actual, float imu)
{
    // 计算当前误差
    pid->error = actual;

    // 更新积分项（抗积分饱和处理）
//    pid->integral += pid->error;

//    //    // 限制积分项，防止积分饱和
//    if (pid->integral = pid->max_output)
//    {
//        pid->integral = 0;
//    }

    // 计算位置式PID输出
    pid->output = pid->Kp * pid->error + pid->Ki * imu + pid->Kd * (pid->error - pid->prev_error);

    // 输出限幅
    if (pid->output > pid->max_output)
    {
        pid->output = pid->max_output;
    }
    else if (pid->output < -pid->min_output)
    {
        pid->output = -pid->min_output;
    }

    // 更新误差历史
    pid->prev_error = pid->error;

    return pid->output;
}