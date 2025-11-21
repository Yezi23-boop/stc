#include "zf_common_headfile.h"
float test_speed = 0;
void test(void)
{
	static int test_time = 0;
	test_time++;
	Encoder_get();
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
	left_motor = (int)PID_Calculate(&motors_pid.left_PID, test_speed, speed_l); // 速度最大值为110
	right_motor = (int)PID_Calculate(&motors_pid.right_PID, test_speed, speed_r);
	motor_output(left_motor, right_motor);
//	motor_output(-1000, -5000);
}
