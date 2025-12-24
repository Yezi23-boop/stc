#include "zf_common_headfile.h"
#include "zf_driver_gpio.h"
#include <type.h>
#include "vofa.h"
#include <stdlib.h>
static float task1ms_time_s = 0;
int flat_statr = 0;
u16 cpu_usage = 0;

void task1ms(void)
{
//    gpio_low(IO_P33);
//    TimingStart();
    if (!P32)  IAP_CONTR = 0x60;
      scheduler_private.system_tick++;
	// 1. 更新传感器数据
    read_AD();
    Prepare_Data();
    lost_lines();
	Encoder_get(&PID.left_speed, &PID.right_speed);
	CarState_t current_state = state_machine_get_state();
	switch (current_state) {
		case STATE_IDLE:
			state_idle();
			break;
		case STATE_LINE_FOLLOW:
			state_line_follow();
			break;
		case STATE_TURN:
			state_turn();
			break;
		case STATE_CROSS:
			state_cross();
			break;
		case STATE_yaw:                        :
			state_yaw();
			break;
		case STATE_ERROR:
			state_error();
			break;
		default:
			break:
	}
}
void task10ms(void)
{
	static int flat_statr_date = 0;
	gpio_low(IO_P34);
	TimingStart();
	scan_track_max_value();
	IMUupdate(&Gyr_filt, &Acc_filt, &Att_Angle);
	dianya_adc();
	flat_statr_date++;
	if (P35 == 0 && flat_statr_date > 50)
	{
		flat_statr++;
		flat_statr_date = 0;
	}
	if (flat_statr >= 1 && lost_spto == 0 && start_flag == 1)
	{
		fuya_update_simple();
	}
	task1ms_time_s = TimingStopSeconds();
	gpio_high(IO_P34);
}
void task100ms(void)
{
#if (ENABLECOMM)
    char vofa_cmd[32]; // VOFA 命令缓存
	// ========== 处理 VOFA 命令 ==========
	// 从 FIFO 读取串口数据，使用系统提供的 wireless_uart_read_buffer
	vofa_parse_from_fifo();

	//       检查是否解析到完整命令
	if (vofa_get_command(vofa_cmd, 32))
	{
		handle_vofa_command(vofa_cmd);
	}
	// ips114_show_float(3 * 24, 18 * 0, task1ms_time_s*1000, 4, 2);
	// ========== 数据发送到 VOFA+ ==========
	// 使用 FireWater 协议发送数据
	// printf("%f,%f,%f,%f\n", speed_l, speed_r, test_speed, PID.steer.output);

	// ========== 显示调试功能（按需选择） ==========
	// printf_date();
	// printf_adc();
	// printf_imu();
	// printf_speed_test();
    // Keystroke_Menu();
#endif
}
void task1000ms(void)
{
	ips114_show_float(3 * 24, 18 * 0, task1ms_time_s*1000, 4, 2);
}
typedef void (*pFunc)(void);

struct taskconfig
{
	u8 ID;
	u16 Period;
	u16 Cnt;
	bool_t Event;
	pFunc pTaskFunc;
};
#define TASKFUNCTION(ID, Period, Cnt, Event, fn) {ID, Period, Cnt, Event, fn}
struct taskconfig gTaskList[] = {
	TASKFUNCTION(1, 1, 0, 0, task1ms),
	TASKFUNCTION(2, 10, 1, 0, task10ms),
	TASKFUNCTION(3, 100, 5, 0, task100ms),
	TASKFUNCTION(4, 1000, 7, 0, task1000ms)};
#define TASKSIZEOF (sizeof(gTaskList) / sizeof(struct taskconfig))
struct taskconfig *pTaskEnd = &gTaskList[TASKSIZEOF - 1];
VOL_U8 gTaskStick = 0;
void main_task(void)
{
	struct taskconfig *pTask = &gTaskList[0];
	if (gTaskStick < MODULATE1MS)
	{
		return;
	}
	gTaskStick = 0;

	for (; pTask <= pTaskEnd; pTask++)
	{
		if (++pTask->Cnt >= pTask->Period)
		{
			pTask->Event = 1;
		}
		if (pTask->Event)
		{
			pTask->pTaskFunc();
			pTask->Cnt = 0;
			pTask->Event = 0;
		}
	}
}

void main()
{

	clock_init(SYSTEM_CLOCK_40M); // 时钟初始化
	debug_init();				  // 调试接口初始化
	P32 = 1;					  // 上电安全记录（示例）
	// 初始化（wireless_uart_init 已在 int_user 中完成）
	int_user();
#if (ENABLECOMM)
	vofa_init(); // 初始化 VOFA 通信（可选）
#endif
	while (1)
	{
		u32 StartTime = GetTimeStart();
		 main_task();
		g_task_scheduler.run();
		u32 EndTime   = GetTimeEnd();
		cpu_usage = (EndTime - StartTime)*10000 / (SYSTEM_CLOCK_40M/12);
	}
}
