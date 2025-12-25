#ifndef _INIT_USER_H_
#define _INIT_USER_H_
#define MODULATE1MS (1000/500)  // 在主循环调成1ms的时间刻度
extern float Gyro_offset_x;
extern float Gyro_offset_y;
extern float Gyro_offset_z;
extern float acc_offset_x;
extern float acc_offset_y;
extern float acc_offset_z;

void init_user(void);

void offset_init(void);

#endif
