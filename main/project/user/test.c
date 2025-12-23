#include "zf_common_headfile.h"

float test_speed_value = 0;
float test_angle_value = 0;

void test_angle_func(void)
{
    Prepare_Data();
    Encoder_get(&PID.left_speed, &PID.right_speed);
    // pid_steer_update(&PID.steer, Err, -gyro_z * 0.01);                // 更新转向环
    pid_angle_update(&PID.angle, test_angle_value, gyro_z * 0.082);               // 更新角度环
    pid_speed_update(&PID.left_speed, -PID.angle.output, PID.left_speed.speed);   // 更新左轮速度环
    pid_speed_update(&PID.right_speed, +PID.angle.output, PID.right_speed.speed); // 更新右轮速度环
    motor_output((int)PID.left_speed.output, (int)PID.right_speed.output);
}

void test_speed_func(void)
{
    Encoder_get(&PID.left_speed, &PID.right_speed);
    pid_speed_update(&PID.left_speed, test_speed_value, PID.left_speed.speed);
    pid_speed_update(&PID.right_speed, test_speed_value, PID.right_speed.speed);
    motor_output((int32)PID.left_speed.output, (int32)PID.right_speed.output);
}
