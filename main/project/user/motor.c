#include "motor.h"

void motor_Init()
{
	pwm_init(PWMB_CH2_P13, 17000, 0);		  // 左电机PWM输出初始化，频率17kHz
	gpio_init(IO_P14, GPO, 1, GPO_PUSH_PULL); // 左电机方向控制初始化
	pwm_init(PWMB_CH3_P52, 17000, 0);		  // 右电机PWM输出初始化，频率17kHz
	gpio_init(IO_P53, GPO, 1, GPO_PUSH_PULL); // 右电机方向控制初始化
}
uint8 stop=0;
uint16 dianya=0;
void motor_output(int32 lpwm, int32 rpwm)
{
   //mV = ADC * 9.2 ≈ (ADC * 92) / 10
    static int32 dianya_count = 0;
    uint16 adc_raw = adc_convert(ADC_CH13_P05);
    dianya = (uint16)(((uint32)adc_raw * 92u) / 10u); // mV 整数表示，避免浮点乘法

    pwm_set_duty(PWMA_CH4N_P07, 0);
    if (dianya < 7600u)
    {
        dianya_count++;
    }
    if (dianya_count > 10)
    {
		stop=1;
        dianya_count = 0;
    }
	if(stop==0)
	if (lpwm > 0)
	{
		P14 = 1;
		pwm_set_duty(PWMB_CH2_P13, lpwm);
	}
	else if (lpwm < 0)
	{
		P14 = 0;
		pwm_set_duty(PWMB_CH2_P13, -lpwm);
	}
	if (rpwm > 0)
	{
		P53 = 0;
		pwm_set_duty(PWMB_CH3_P52, rpwm);
	}
	else if (rpwm < 0)
	{
		P53 = 1;
		pwm_set_duty(PWMB_CH3_P52, -rpwm);
	}
	else
	{
		pwm_set_duty(PWMB_CH2_P13, 100);
		pwm_set_duty(PWMB_CH3_P52, 100);
	}
}
void lost_lines()
{
	static int window_ticks = 0;		  // 时间窗口计数（周期数）
	static int loss_events = 0;			  // 时间窗口内的丢线事件计数
	const int LOST_WINDOW_TICKS = 100;	  // 时间窗口大小（周期数）
	const int LOST_THRESHOLD_EVENTS = 10; // 丢线事件阈值（窗口内超过该次数则触发）

	// 丢线事件判定：四路电感均低于阈值（一次满足记为一件事件）
	if (ad1 < 3 && ad2 < 3 && ad3 < 3 && ad4 < 3)
	{
		if (loss_events < 30000)
			loss_events++; // 防溢出保护
	}

	// 累计窗口时间
	if (window_ticks < 30000)
		window_ticks++; // 防溢出保护

	// 窗口结束时统一判定与复位（满足需求“否则就清零”）
	if (window_ticks >= LOST_WINDOW_TICKS)
	{
		if (loss_events > LOST_THRESHOLD_EVENTS)
		{
			// 触发丢线：进入安全模式（在 TM0 中 stop==1 会屏蔽 PID 输出）
			stop = 1;
		}
		else
		{
			// 未达到阈值：清零触发
			stop = 0;
		}

		// 复位窗口计数与事件计数，开始下一窗口统计
		window_ticks = 0;
		loss_events = 0;
	}
}
