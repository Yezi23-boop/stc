#ifndef __ADC_H__
#define __ADC_H__

// 定义电感数量
#define NUM 4
// 五路电感的归一化数值（0-100范围）
extern float ad1; // 最左侧电感
extern float ad2; // 左侧电感
extern float ad3; // 右侧电感
extern float ad4; // 最右侧电感

extern int Err;     // 电感偏差值
extern float Err_tow; // 双电感偏差值

// 全局变量声明
extern float MA[NUM];  // 电感最大值数组（用于标定）
extern float RAW[NUM]; // 电感原始值数组
extern float Err_tow;  // 双电感偏差值
extern float dianya;
// 函数声明
void scan_track_max_value(void); // 扫描赛道获取电感最大值
void read_AD(void);              // 读取并处理电感数据
void dianya_adc(void);
#endif
