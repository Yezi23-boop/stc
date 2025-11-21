#ifndef __PID_H__
#define __PID_H__
#include "zf_common_headfile.h"
typedef struct
{
  float Kp;          // 比例系数
  float Ki;          // 积分系数
  float Kd;          // 微分系数
  float error;       // 当前误差
  float prev_error;  // 上一次误差
  float prev2_error; // 上上次误差
  float output;      // 当前输出值
  float max_output;  // 输出限幅值
  float min_output;  // 输出限幅值
  float integral;    // 积分项累积值（用于位置式PID）
} PID;

typedef struct
{
  PID left_PID;
  PID right_PID;
  PID Positional_PID;
} PID_struct;

void PID_Init(PID *pid, float Kp, float Ki, float Kd, float max_output, float min_output);
void Encoder_get(void);
float PID_Calculate(PID *pid, float setpoint, float actual);
float PID_Positional_Calculate(PID *pid, float actual, float imu);
extern float speed_l, speed_r;
extern PID_struct motors_pid;
#endif
