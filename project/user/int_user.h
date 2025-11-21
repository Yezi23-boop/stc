#ifndef _INT_USER_H_
#define _INT_USER_H_
//extern float Gyrox;
//extern float Gyroy;
//extern float Gyroz;
//extern float acc_x;
//extern float acc_y;
//extern float acc_z;
extern float Gyro_offset_x;
extern float Gyro_offset_y;
extern float Gyro_offset_z;
extern float acc_offset_x;
extern float acc_offset_y;
extern float acc_offset_z;




void int_user(void);

void offset_init(void);



#endif