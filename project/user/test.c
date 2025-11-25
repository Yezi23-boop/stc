#include "zf_common_headfile.h"
/*
 * 测试速度阶跃序列
 * 0–500: 20，500–1000: 15，1000–2000: 18，2000–3000: -40，>=3000: 60
 * 定期调用以验证速度环与电机输出。
 */
float test_speed = 0;
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
	pid_speed_update(&PID.left_speed, test_speed, speed_l);	 // 左轮速度环
	pid_speed_update(&PID.right_speed, test_speed, speed_r); // 右轮速度环
	motor_output((int)PID.left_speed.output, (int)PID.right_speed.output);
	//  motor_output(-1000, -5000);
}
