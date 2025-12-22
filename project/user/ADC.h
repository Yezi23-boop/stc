#ifndef __ADC_H__
#define __ADC_H__

// ADC 采样相关常量
#define NUM 4
// 电感传感器ID（4个电感）
typedef enum {
    INDUCTOR_LEFT_OUTER = 0,    // 最左侧
    INDUCTOR_LEFT_INNER = 1,    // 左内侧
    INDUCTOR_RIGHT_INNER = 2,   // 右内侧
    INDUCTOR_RIGHT_OUTER = 3,   // 最右侧
    INDUCTOR_ALL = 4
} InductorID;
// 电感驱动接口
typedef struct {
    // 初始化
    void (*init)(adc_channel_enum ch, adc_resolution_enum resolution);

    // 数据读取
    uint16 (*read)(adc_channel_enum ch);
} InductorDriver;
// 赛道电感归一化值（0-100，使用整数显示更高效）
extern uint16 ad1; // 左前电感（0-100）
extern uint16 ad2; // 左后电感（0-100）
extern uint16 ad3; // 右前电感（0-100）
extern uint16 ad4; // 右后电感（0-100）

extern float Err; // 误差值（整数，用于位置式 PID_Direction）

// 全局采样与标定数据（均为整数，降低 8051 浮点开销）
extern uint16 MA[NUM];  // 标定最大值
extern uint16 RAW[NUM]; // 原始 ADC 值
extern uint16 dianya;   // 电源电压（mV）

// 采样与处理函数声明
void scan_track_max_value(void); // 扫描赛道，采集并更新最大值
void read_AD(void);              // 读取 ADC，更新 RAW/MA/ad1..ad4/Err
void voltage_adc(void);           // 采样电源电压，更新 dianya（mV）

#endif
