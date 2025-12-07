#include "zf_common_headfile.h"
/************************* 全局变量 *************************/
// 计时结果（单位：微秒）
u32 gStart_us = 0;
u32 gEnd_us = 0;
VOL_U8 SpillCnt = 0;

ErrorUnion_U CodeError;
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
