#ifndef _TEST_H_
#define _TEST_H_

// 机器周期 = 12/系统时钟 (8051 标准内核)，单位：秒
#define MACHINE_CYCLE (12.0 / SYSTEM_CLOCK_40M)  

/*********************调试时使能，正式时关闭减少CPU运行*********************/
#define ENABLECOMM (1)      // 是否使能通信

typedef union {
    u32 eCodeError;
    struct {
        u8 NO_ERROR :1;
        u8 GetTimeError :1;
        u8 GetTimeEndError :1;
        // ......后续补充其他的错误
    }ErrorDetail;
}ErrorUnion_U;


extern float test_speed;
void test(void);


u32 GetTimeStart(void);
u32 GetTimeEnd(void);
f32 GetTotalTime(void);


void TimingStart(void);
u32 TimingStopTicks(void);
f32 TimingStopSeconds(void);
#endif
