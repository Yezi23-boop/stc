
#ifndef SLEEP_CONTROL_H_
#define SLEEP_CONTROL_H_

// 休眠确认计数器
#define SLEEP_CONFIRM_CNT (10) // 休眠确认计数器，单位为1ms任务周期的计数值


extern void Init_HVIO(void);
extern void SLEEP_EnableHVIOWake(void);
extern void SLEEP_DisableHVIOWake(void);
extern void SLEEP_EnableLINWake(void);
extern void SLEEP_DisableLINWake(void);
extern void CheckSleepStatus(void);


#endif