#include "motor.h"

void motor_Init()
{
	pwm_init(PWMB_CH2_P13, 17000, 0);		  // 左电机PWM输出初始化，频率17kHz
	gpio_init(IO_P14, GPO, 1, GPO_PUSH_PULL); // 左电机方向控制初始化
	pwm_init(PWMB_CH3_P52, 17000, 0);		  // 右电机PWM输出初始化，频率17kHz
	gpio_init(IO_P53, GPO, 1, GPO_PUSH_PULL); // 右电机方向控制初始化
}
uint8 stop=0;
float dianya=0;
void motor_output(int32 lpwm, int32 rpwm)
{
	if(stop==0)
    {
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
   }
	else
	{
		pwm_set_duty(PWMB_CH2_P13, 100);
		pwm_set_duty(PWMB_CH3_P52, 100);
	}
}
void lost_lines()
{
    static int8 count=0;
	// 丢线事件判定：四路电感均低于阈值（一次满足记为一件事件）
	if (ad1 < 3 && ad2 < 3 && ad3 < 3 && ad4 < 3)
	{
		count++;
	}
	if(count>5)
	{
		count=0;
		stop = 1;
	}
	
}

void dianya_jiance(void)
{
	static int32 dianya_count = 0;
    uint16 adc_raw = adc_convert(ADC_CH13_P05);
    dianya =adc_raw * 0.0092; 
    if (dianya < 7.4)
    {
        dianya_count++;
    }
    if (dianya_count > 1000)
    {
	  stop=1;
    }

}
