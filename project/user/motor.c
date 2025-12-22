#include "motor.h"

int8 lost_spto = 0;
MCI_Handle_t Mc =
{
    IDLE,            // 对应 MCI_State_t State;
    MC_NO_FAULTS,    // 对应 uint16_t CurrentFaults;
    MC_NO_FAULTS     // 对应 uint16_t PastFaults;
};

MotorConfig_t const MotorInit = {
	.pwm_channel_left = PWMB_CH2_P13,
	.dir_pin_left     = IO_P14,
	.pwm_channel_right = PWMB_CH3_P52,
	.dir_pin_right    = IO_P53,
	.deadband_left    = 40,
	.deadband_right   = 40,
	.frequency_left   = 17000,
	.frequency_right  = 17000
};
MotorDriver g_motor_driver = {
	.init = hal_motor_Init,
	.set_pwm = motor_output
};
EncoderDriver g_encoder_driver = {
	.init = encoder_dir_init,
	.read = encoder_get_count,
	.clear = encoder_clear_count
};
void hal_motor_Init(MotorConfig_t* MotorConfig)
{
	pwm_init(MotorConfig->pwm_channel_left, MotorConfig->frequency_left, 0);		  // 左电机PWM输出初始化，频率17kHz
	gpio_init(MotorConfig->dir_pin_left, GPO, 1, GPO_PUSH_PULL); 					  // 左电机方向控制初始化
	pwm_init(MotorConfig->pwm_channel_right, MotorConfig->frequency_right, 0);		  // 右电机PWM输出初始化，频率17kHz
	gpio_init(MotorConfig->dir_pin_right, GPO, 1, GPO_PUSH_PULL); 					  // 右电机方向控制初始化
}

void motor_output(int32 lpwm, int32 rpwm)
{
	MotorConfig_t const* private = &MotorInit;
	// 应用死区
    if (lpwm > 0 && lpwm < private->deadband_left) lpwm = private->deadband_left;
    if (lpwm < 0 && lpwm > -private->deadband_left) lpwm = -private->deadband_left; 
    if (rpwm > 0 && rpwm < private->deadband_right) rpwm = private->deadband_right;
    if (rpwm < 0 && rpwm > -private->deadband_right) rpwm = -private->deadband_right;
    
	// 输出PWM
	if (lpwm > 0)
	{
		P14 = 1;
		pwm_set_duty(private->pwm_channel_left, lpwm);
	}
	else if (lpwm < 0)
	{
		P14 = 0;
		pwm_set_duty(private->pwm_channel_left, -lpwm);
	}
	if (rpwm > 0)
	{
		P53 = 0;
		pwm_set_duty(private->pwm_channel_right, rpwm);
	}
	else if (rpwm < 0)
	{
		P53 = 1;
		pwm_set_duty(private->pwm_channel_right, -rpwm);
	}
}


void lost_lines()
{
	// 中文说明：时间窗口内统计“丢线事件”次数，超过阈值则触发，否则清零
	// 设计目的：满足“一定时间内丢线次数>100即触发，否则清零”的需求
	// 约定：TM0 周期为 5ms，则 200 个周期≈1s，可按需调整窗口大小

	static int window_ticks = 0;		  // 时间窗口计数（周期数）
	static int loss_events = 0;			  // 时间窗口内的丢线事件计数
	const int LOST_WINDOW_TICKS = 200;	  // 时间窗口大小（周期数），默认约 1s
	const int LOST_THRESHOLD_EVENTS = 20; // 丢线事件阈值（窗口内超过该次数则触发）

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
			// 触发丢线：进入安全模式（在 TM0 中 lost_spto==1 会屏蔽 PID 输出）
			lost_spto = 1;
			g_motor_driver.set_pwm(100, 100); // 低速直行，可按需调整占空比
		}
		else
		{
			// 未达到阈值：清零触发
			lost_spto = 0;
		}

		// 复位窗口计数与事件计数，开始下一窗口统计
		window_ticks = 0;
		loss_events = 0;
	}
}

/**
  * @brief Returns the list of faults that are currently active on the target motor
  *
  * This function returns a bitfield that indicates faults that occured on the Motor
  * Control subsystem for the target motor and that are still active (the conditions
  * that triggered the faults returned are still true).
  *
  * Possible error codes are listed in the @ref fault_codes "Fault codes" section.
  *
  * @param  pHandle Pointer on the target motor drive structure.
  */
uint16_t MC_GetCurrentFaults(MCI_Handle_t *pHandle) //cstat !MISRAC2012-Rule-8.13
{
  return ((uint16_t)pHandle->CurrentFaults);
}


/**
  * @brief Executes periodic Motor Control tasks
  */
void MC_StateMachine(void)
{
	if (MC_GetCurrentFaults(&Mc) == MC_NO_FAULTS)
	{
		switch (Mc.State)
		{
			case IDLE:
			{
				Mc.State = START;
				break;
			}
			case START:
			{
				Mc.State = RUN;
				break;
			}
			case RUN:
			{
				Mc.State = STOP;
				break;				
			}
			case STOP:
			{
				Mc.State = FAULT_OVER;
				break;				
			}
			case FAULT_OVER:
			{
				Mc.State = IDLE;
				break;				
			}
			case FAULT_NOW:
			{
				Mc.State = FAULT_OVER;
				break;				
			}
			default:
				break;
		}
	}
}