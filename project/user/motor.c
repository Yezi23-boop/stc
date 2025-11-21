#include "motor.h"

void motor_Init()
{
	pwm_init(PWMB_CH2_P13, 17000, 0);		  // 左电机PWM输出初始化，频率17kHz
	gpio_init(IO_P14, GPO, 1, GPO_PUSH_PULL); // 左电机方向控制初始化
	pwm_init(PWMB_CH3_P52, 17000, 0);		  // 右电机PWM输出初始化，频率17kHz
	gpio_init(IO_P53, GPO, 1, GPO_PUSH_PULL); // 右电机方向控制初始化
}

void motor_output(int32 lpwm, int32 rpwm)
{
	if(lpwm>0)
	{
		P14=1;
		pwm_set_duty(PWMB_CH2_P13, lpwm);
	}
  else if(lpwm<0)
	{
		P14=0;
		pwm_set_duty(PWMB_CH2_P13, -lpwm);
	}
	if(rpwm>0)
	{
		P53=0;
		pwm_set_duty(PWMB_CH3_P52, rpwm);
	}
  else if(rpwm<0)
	{
		P53=1;
		pwm_set_duty(PWMB_CH3_P52, -rpwm);
	}
}
int8 lost_spto = 0;
void lost_lines()
{
static int count=0;
static int count_stop=0;
static int count_flat=0;
	if (ad1<3&&ad2<3&&ad3<3&&ad4<3)
	{
	count++;
	count_flat=1;
	}
	if(count_flat==1)
	{
	 count_stop++;
	}
	if(count>100&&count_stop<200)
	{
	lost_spto=1;
	motor_output(100, 100);
	}
	else if(count>400&&count_stop>200)
	{
	count_stop=0;
	count=0;
	count_flat=0;
	}
}