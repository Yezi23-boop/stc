#include "pid.h"
PID_Controllers PID; // 聚合控制器：左右速度环 + 转向环
LowPassFilter_t encoder_l;
LowPassFilter_t encoder_r;
float delta_output = 0, speed_l = 0, speed_r = 0, max_integral = 0;
// 速度环初始化（增量式），命名简洁
void pid_speed_init(PID_Speed *pid, float kp, float ki, float kd, float max_out, float min_out)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->error = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev2_error = 0.0f;
    pid->output = 0.0f;
    pid->max_output = max_out;
    pid->min_output = min_out;
}
// 转向环初始化（位置式），命名简洁
void pid_steer_init(PID_Steer *pid, float kp, float kd, float kd_gyro, float max_out, float min_out)
{
    pid->Kp = kp;
    pid->Kd = kd;
    pid->kd_gyro = kd_gyro;
    pid->error = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
    pid->max_output = max_out;
    pid->min_output = min_out;
}
// 编码器读取
void Encoder_get(PID_Speed *pid_l, PID_Speed *pid_r)
{
    // 读取编码器计数值并转换为速度（乘以系数0.2调整单位）
    pid_l->speed = g_encoder_driver.read(TIM4_ENCOEDER) * 0.2f;  // 左轮速度
    pid_r->speed = -g_encoder_driver.read(TIM3_ENCOEDER) * 0.2f; // 右轮速度（负号用于方向调整）
    low_pass_filter_mt(&encoder_l, &pid_l->speed, 0.8);
    low_pass_filter_mt(&encoder_r, &pid_r->speed, 0.8);
    // 清除编码器计数，准备下一次计数
    g_encoder_driver.clear(TIM3_ENCOEDER);
    g_encoder_driver.clear(TIM4_ENCOEDER);
}
// 速度环更新（增量式）
void pid_speed_update(PID_Speed *pid, float target, float actual)
{
    // 计算当前误差（直接使用结构体成员）
    pid->error = target - actual;

    // 增量式 PID_Direction：使用结构体成员计算差分与输出增量
    delta_output = pid->Kp * (pid->error - pid->prev_error) + pid->Ki * pid->error + pid->Kd * (pid->error - 2.0f * pid->prev_error + pid->prev2_error);

    // 更新输出并限幅
    pid->output += delta_output;
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
}

// 转向环更新（位置式）
void pid_steer_update(PID_Steer *pid, float error, float gyro)
{
    // 计算当前误差（直接写入结构体成员）
    pid->error = error;

    // 位置式 PID_Direction：kd_gyro 以陀螺输入项参与
    pid->output = pid->Kp * pid->error + pid->kd_gyro * gyro + pid->Kd * (pid->error - pid->prev_error);

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
}
