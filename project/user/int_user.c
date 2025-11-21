#include "zf_common_headfile.h"
#define TIME_0 5 // 定时器0中断时间
#define TIME_1 20 // 定时器1中断时间
void int_user(void)
{
	// 此处编写用户代码 例如外设初始化代码等
	imu660ra_init();// 初始化 IMU660RA
	eeprom_init();
	pit_ms_init(TIM0_PIT, TIME_0);
	pit_ms_init(TIM1_PIT, TIME_1);
	encoder_dir_init(TIM3_ENCOEDER, IO_P46, TIM3_ENCOEDER_P04); // 左编码器初始化
	encoder_dir_init(TIM4_ENCOEDER, IO_P42, TIM4_ENCOEDER_P06); // 右编码器初始化
	adc_init(ADC_CH13_P05 , ADC_8BIT); 
	adc_init(ADC_CH0_P10 , ADC_12BIT); 
	adc_init(ADC_CH1_P11 , ADC_12BIT); 
	adc_init(ADC_CH8_P00 , ADC_12BIT); 
	adc_init(ADC_CH9_P01, ADC_12BIT); 
	motor_Init(); // 电机初始化
	fuya_Init();//负压电机初始化
	wireless_uart_init();				// 无线串口初始化
//	pwm_init(PWMA_CH4N_P07, 3800, 1);	// 蜂鸣器初始化
	PID_Init(&motors_pid.left_PID, 100, 40, 0, 8000,8000);	 // 左右电机速度环赋值
	PID_Init(&motors_pid.right_PID,100, 40, 0, 8000,8000);	 // 300 80
//	PID_Init(&motors_pid.Positional_PID, 0.3, 0.1, 4, 45,45); // 位置环赋值
	PID_Init(&motors_pid.Positional_PID,kp_Err, kd_gyro, kd_Err, 45,45); // 位置环赋值
	ips114_init();
}


/*********************************************
 *函数：初始零飘处理
 *函数名：offset_init()
 *备注：陀螺仪和加速度的
 *日期：2024/11/26
 *修改日期：
 *********************************************/
float Gyrox=0;
float Gyroy=0;
float Gyroz=0;
float acc_x=0;
float acc_y=0;
float acc_z=0;
float Gyro_offset_x=0;
float Gyro_offset_y=0;
float Gyro_offset_z=0;
float acc_offset_x=0;
float acc_offset_y=0;
float acc_offset_z=0;
void offset_init(void)      
{
    int rt=50;
    int i;
	imu660ra_init();//陀螺仪
    system_delay_init();//延迟
    for (i = 0; i < rt; i++)
    {   
        imu660ra_get_gyro();
    	imu660ra_get_acc();	
		Gyro_offset_x += imu660ra_gyro_transition(imu660ra_gyro_x);
        Gyro_offset_y += imu660ra_gyro_transition(imu660ra_gyro_y);		
        Gyro_offset_z += imu660ra_gyro_transition(imu660ra_gyro_z);
	    acc_offset_x+=(imu660ra_acc_transition(imu660ra_acc_x));
	    acc_offset_y+=(imu660ra_acc_transition(imu660ra_acc_y));
	    acc_offset_z+=(imu660ra_acc_transition(imu660ra_acc_z));
	    system_delay_ms(5);		
    }
  Gyro_offset_x = Gyro_offset_x/rt;
  Gyro_offset_y = Gyro_offset_y/rt;
  Gyro_offset_z = Gyro_offset_z/rt;
  acc_offset_x  = acc_offset_x/rt;
  acc_offset_y  = acc_offset_y/rt;
  acc_offset_z  = acc_offset_z/rt;
}