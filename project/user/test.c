#include "zf_common_headfile.h"
/************************* 全局变量 *************************/
// 计时结果（单位：微秒）
u32 gStart_us = 0;
u32 gEnd_us = 0;
VOL_U8 SpillCnt = 0;

ErrorUnion_U CodeError;
/**
 * @brief 测试速度阶跃序列说明
 * @details 0–500: 20，500–1000: 15，1000–2000: 18，2000–3000: -40，>=3000: 60。用于验证速度环与电机输出。
 */
float test_speed = 0;
/**
 * @brief 生成速度阶跃并驱动左右轮速度环
 * @details 内部维护计数以阶段切换目标速度；调用后依次更新左右轮速度环 PID 并输出到电机。
 * @return 无
 */
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

/**
 * @brief 读取 Timer0 当前16位计数值
 * @return 当前计数（ticks，0~65535）
 * @note Timer0 需工作在16位模式；单位为机器周期（12T）
 */
u32 GetTimeStart(void)
{
	u32 TL0_Time8Bit = TL0;
	u32 TH0_Time8Bit = (u32)(TH0 << 8);
	return TH0_Time8Bit + TL0_Time8Bit;
}

/**
 * @brief 计算自基准 gStart_us 的增量 ticks，含溢出累计
 * @return 增量 ticks
 * @note 需先调用 TimingStart() 设置基准；SpillCnt 在 TM0 中断递增用于累计跨越的 65536 周期
 */
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
/**
 * @brief 返回最近一次计时的耗时（秒）
 * @return 耗时（秒）
 * @note 依赖 gEnd_us 已更新为增量 ticks；通过 MACHINE_CYCLE 将 ticks 转换为秒
 */
f32 GetTotalTime(void)
{
	return MACHINE_CYCLE * gEnd_us;
}

/**
 * @brief 启动一次计时窗口并记录起点
 * @return 无
 * @post gStart_us 被更新且 SpillCnt 置零
 */
void TimingStart(void)
{
	SpillCnt = 0;
	gStart_us = GetTimeStart();
}

/**
 * @brief 结束计时并返回增量 ticks
 * @return 增量 ticks（机器周期数）
 * @note 与 TimingStart() 配对使用；同时更新 gEnd_us
 */
u32 TimingStopTicks(void)
{
	gEnd_us = GetTimeEnd();
	return gEnd_us;
}

/**
 * @brief 结束计时并以秒为单位返回耗时
 * @return 耗时（秒）
 * @note 内部将增量 ticks 乘以 MACHINE_CYCLE 实现单位换算；同时更新 gEnd_us
 */
f32 TimingStopSeconds(void)
{
	gEnd_us = GetTimeEnd();
	return MACHINE_CYCLE * (f32)gEnd_us;
}
/**
 * @brief 启动局部计时窗口
 * @details 保存当前 SpillCnt 与 Timer0 计数为起点快照
 * @param w 局部计时窗口指针（必须非空）
 * @note 不重置 SpillCnt，避免与全局计时冲突
 */
void TimingStartLocal(LocalTimingWindow *w)
{
	w->spill_start = SpillCnt;
	w->start_ticks = ((u32)TH0 << 8) + TL0;
}
/**
 * @brief 停止局部计时并返回增量 ticks
 * @details 计算溢出差与计数差的组合增量，不修改全局基准
 * @param w 局部计时窗口指针（必须非空）
 * @return 增量 ticks（机器周期数）
 * @note 适合任务内部耗时与嵌套计时
 */
u32 TimingStopTicksLocal(LocalTimingWindow *w)
{
	u8 spill_stop = SpillCnt;
	u32 stop_ticks = ((u32)TH0 << 8) + TL0;
	return (u32)(spill_stop - w->spill_start) * 65536 + (stop_ticks - w->start_ticks);
}
/**
 * @brief 停止局部计时并返回耗时（秒）
 * @details 将局部增量 ticks 乘以 MACHINE_CYCLE 转换为秒
 * @param w 局部计时窗口指针（必须非空）
 * @return 耗时（秒）
 * @note 不影响 gStart_us/gEnd_us 与 SpillCnt
 */
f32 TimingStopSecondsLocal(LocalTimingWindow *w)
{
	u32 dt = TimingStopTicksLocal(w);
	return MACHINE_CYCLE * (f32)dt;
}
