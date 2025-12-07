#ifndef _TEST_H_
#define _TEST_H_

// 机器周期 = 12/系统时钟 (8051 标准内核)，单位：秒
#define MACHINE_CYCLE (12.0 / SYSTEM_CLOCK_40M)  

typedef enum {
    GetTimeError = 0,
    GetTimeEndError = 1
}CodeState_t;
extern float test_speed;
void test(void);


u32 GetTimeStart(void);
u32 GetTimeEnd(void);
f32 GetTotalTime(void);


void TimingStart(void);
u32 TimingStopTicks(void);
f32 TimingStopSeconds(void);
#endif
