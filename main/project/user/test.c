#include "zf_common_headfile.h"
float test_speed = 20;
float test_angle = 0;
void test_angle(void)
{
    pid_steer_update(&PID.steer, test_angle, -imu660ra_gyro_z * 0.01); // 更新转向环
    pid_steer_update(&PID.angle, PID.steer.output, 0);                 // 更新角度环
    motor_output((int32)PID.angle.output, (int32)PID.angle.output);
}
void test_speed(void)
{
    //    int32 ff_l;
    //    int32 ff_r;
    int32 lpwm;
    int32 rpwm;
    //	static int test_time = 0;
    //	test_time++;
    Encoder_get(&PID.left_speed, &PID.right_speed);
    //	if (test_time >= 4000)
    //	{
    //		test_speed = 60;
    //		test_time = 0;
    //	}
    //	else if (test_time >= 3000)
    //	{
    //		test_speed = -40;
    //	}
    //	else if (test_time >= 2000)
    //	{
    //		test_speed = 18;
    //	}
    //	else if (test_time >= 1000)
    //	{
    //		test_speed = 15;
    //	}
    //	else if (test_time >= 500)
    //	{
    //		test_speed = 20;
    //	}
    pid_speed_update(&PID.left_speed, test_speed, PID.left_speed.speed);
    pid_speed_update(&PID.right_speed, test_speed, PID.right_speed.speed);

    //    ff_l = motor_speed_to_duty(test_speed);
    //    ff_r = motor_speed_to_duty(test_speed);

    //    lpwm = ff_l + (int32)PID.left_speed.output;
    //    rpwm = ff_r + (int32)PID.right_speed.output;
    if (lpwm > PWM_DUTY_MAX)
        lpwm = PWM_DUTY_MAX;
    if (lpwm < -PWM_DUTY_MAX)
        lpwm = -PWM_DUTY_MAX;
    if (rpwm > PWM_DUTY_MAX)
        rpwm = PWM_DUTY_MAX;
    if (rpwm < -PWM_DUTY_MAX)
        rpwm = -PWM_DUTY_MAX;

    //    motor_output((int32)PID.left_speed.output, (int32)PID.right_speed.output);
    motor_output(2000, 2000);
}
