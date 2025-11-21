#ifndef __EEPROM_H
#define __EEPROM_H

/*********************************************
 * EEPROM系统基础变量声明
 *********************************************/
extern uint8 date_buff[400]; // eeprom数据数组

/*********************************************
 * 启动配置参数变量声明
 *********************************************/
extern int16 start_flag;   // 启动标志
extern int16 circle_flags; // 出环方向标志

/*********************************************
 * PID速度控制参数变量声明
 *********************************************/
extern float kd_gyro;      // 陀螺仪
extern float kp_Err;       // 误差比例系数
extern float fuya_xili;     // 误差差分系数
extern float kd_Err;       // 误差微分系数
extern float pwm_filter;   // PWM滤波系数
extern float speed_run;    // 运行速度

/*********************************************
 * PID角度控制参数变量声明
 *********************************************/
extern float A_1;            // 控制参数A_1
extern float B_1;            // 控制参数B_1
extern float C_l;        // 二次误差微分系数
extern float kp_Angle;       // 角度比例系数
extern float kd_Angle;       // 角度微分系数
extern float limiting_Angle; // 角度限幅值

/*********************************************
 * 圆环控制参数变量声明
 *********************************************/
extern float ring_encoder;
extern float pre_ring_Gyro_set;
extern float in_ring_Gyroz;
extern float pre_out_ring_Gyro_set;
extern float pre_out_ring_Gyroz;
extern float pre_out_ring_encoder;

/*********************************************
 * EEPROM系统函数声明
 *********************************************/
void eeprom_init();  // EEPROM初始化函数
void eeprom_flash(); // 参数保存到EEPROM函数

/*********************************************
 * EEPROM底层读写函数声明
 *********************************************/
void save_int(int32 input, uint8 value_bit); // 整型数据保存函数
int32 read_int(uint8 value_bit);             // 整型数据读取函数

void save_float(float input, uint8 value_bit); // 浮点型数据保存函数
float read_float(uint8 value_bit);             // 浮点型数据读取函数

#endif