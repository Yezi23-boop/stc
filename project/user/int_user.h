#ifndef _INT_USER_H_
#define _INT_USER_H_
#define TIME_500US (500)           // 定时器2中断周期(500us)
#define MODULATE1MS (1000/TIME_500US)  // 在主循环调成1ms的时间刻度
extern float Gyro_offset_x;
extern float Gyro_offset_y;
extern float Gyro_offset_z;
extern float acc_offset_x;
extern float acc_offset_y;
extern float acc_offset_z;

void int_user(void);

void offset_init(void);

#endif