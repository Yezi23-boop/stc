#include "zf_common_headfile.h"
#include "vofa.h"
#include <stdlib.h>

// 打印与调试函数声明
void printf_adc();
void printf_imu();
void printf_date();
void printf_speed_test();
void task1ms(void)
{

}
void task10ms(void)
{
	
}
void task100ms(void)
{
	// ========== 处理 VOFA 命令 ==========
	// 从 FIFO 读取串口数据，使用系统提供的 wireless_uart_read_buffer
	vofa_parse_from_fifo();

	//      // 检查是否解析到完整命令
	if (vofa_get_command(vofa_cmd, 32))
	{
		handle_vofa_command(vofa_cmd);
	}

	// ========== 数据发送到 VOFA+ ==========
	// 使用 FireWater 协议发送数据
	// printf("%f,%f,%f,%f\n", speed_l, speed_r, test_speed, PID.steer.output);

	// ========== 显示调试功能（按需选择） ==========
	// printf_date();
	// printf_adc();
	// printf_imu();
	// printf_speed_test();
//		 Keystroke_Menu();
}
void task1000ms(void)
{
	
}
typedef void(*pFunc)(void);
struct taskconfig {
	u8 ID;
	u16 Period;
	u16 Cnt;
	bool Event;
	pFunc pTaskFunc;
};
#define TASKFUNCTION(ID, Period, Cnt, Event, fn) {ID, Period, Cnt, Event, fn}
struct taskconfig gTaskList[] = {
	TASKFUNCTION(1, 1,    0, 0, task1ms),
	TASKFUNCTION(2, 10,   1, 0, task10ms),
	TASKFUNCTION(3, 100,  5, 0, task100ms),
	TASKFUNCTION(4, 1000, 7, 0, task1000ms)
};
#define TASKSIZEOF  (sizeof(gTaskList)/sizeof(struct taskconfig))
struct taskconfig* pTaskEnd = &gTaskList[TASKSIZEOF-1];
u16 gTaskStick = 0;
void main_task(void)
{
	if (gTaskStick < MODULATE1MS) {
		return;
	}
	gTaskStick = 0;
	struct taskconfig* pTask = &gTaskList[0];
	for (;pTask <= pTaskEnd; pTask++) {
		if (++pTask->Cnt >= pTask->Period) {
			pTask->Event = 1;
		}
		if (pTask->Event) {
			pTask->pTaskFunc();
			pTask->Cnt = 0;
			pTask->Event = 0;
		}
	}
}


void main()
{
	char vofa_cmd[32]; // VOFA 命令缓存

	clock_init(SYSTEM_CLOCK_40M); // 时钟初始化
	debug_init();				  // 调试接口初始化
	P32 = 1;					  // 上电安全记录（示例）

	// 初始化（wireless_uart_init 已在 int_user 中完成）
	int_user();
	vofa_init(); // 初始化 VOFA 通信（可选）

	while (1)
	{
		main_task();
	}
}

// 原有的数据显示函数（已部分调整）
void printf_date()
{
	// 优化：Err 与电感归一化值使用整数显示，避免浮点格式化开销
	ips114_show_int32(1 * 24, 18 * 0, Err, 3);
	ips114_show_int32(1 * 24, 18 * 1, ad1, 3);
	ips114_show_int32(1 * 24, 18 * 2, ad2, 3);
	ips114_show_int32(1 * 24, 18 * 3, ad3, 3);
	ips114_show_int32(1 * 24, 18 * 4, ad4, 3);

	ips114_show_float(3 * 24, 18 * 0, PID.steer.output, 3, 1);
	ips114_show_float(3 * 24, 18 * 1, speed_run + PID.steer.output, 3, 1);
	ips114_show_float(3 * 24, 18 * 2, speed_run - PID.steer.output, 3, 1);

	// 电压使用 mV 整数显示（dianya 已在 ADC.c 转为 mV 整数）
	ips114_show_int32(6 * 24, 18 * 1, dianya, 5);
}

void printf_adc()
{
	// 电感归一化值（0-100）整数显示
	ips114_show_int32(1 * 24, 18 * 0, ad1, 3);
	ips114_show_int32(1 * 24, 18 * 1, ad2, 3);
	ips114_show_int32(1 * 24, 18 * 2, ad3, 3);
	ips114_show_int32(1 * 24, 18 * 3, ad4, 3);

	ips114_show_int32(1 * 24, 18 * 6, Err, 4);

	// 原始 ADC 计数显示（整数）
	ips114_show_int32(3 * 24, 18 * 0, RAW[0], 4);
	ips114_show_int32(3 * 24, 18 * 1, RAW[1], 4);
	ips114_show_int32(3 * 24, 18 * 2, RAW[2], 4);
	ips114_show_int32(3 * 24, 18 * 3, RAW[3], 4);

	// 标定最大值显示（整数）
	ips114_show_int32(7 * 24, 18 * 0, MA[0], 4);
	ips114_show_int32(7 * 24, 18 * 1, MA[1], 4);
	ips114_show_int32(7 * 24, 18 * 2, MA[2], 4);
	ips114_show_int32(7 * 24, 18 * 3, MA[3], 4);
}

void printf_imu()
{
	ips114_show_float(4 * 24, 18 * 0, Gyr_filt.X, 4, 2);
	ips114_show_float(4 * 24, 18 * 1, Gyr_filt.Y, 4, 2);
	ips114_show_float(4 * 24, 18 * 2, Gyr_filt.Z, 4, 2);
	ips114_show_float(4 * 24, 18 * 4, Acc_filt.X, 4, 2);
	ips114_show_float(4 * 24, 18 * 5, Acc_filt.Y, 4, 2);
	ips114_show_float(4 * 24, 18 * 6, Acc_filt.Z, 4, 2);
	ips114_show_float(7 * 24, 18 * 0, vx, 4, 2);
	ips114_show_float(7 * 24, 18 * 1, vy, 4, 2);
	ips114_show_float(7 * 24, 18 * 2, vz, 4, 2);
	ips114_show_int32(7 * 24, 18 * 4, fuya_date, 4);
	ips114_show_int32(7 * 24, 18 * 5, phase, 4);
}

void printf_speed_test()
{
	ips114_show_float(0, 0, test_speed, 6, 1);
	ips114_show_float(0, 15, PID.steer.output, 6, 1);
	ips114_show_float(0, 35, speed_l, 6, 1);
	ips114_show_float(0, 55, speed_r, 6, 1);
	ips114_show_int32(0, 75, imu660ra_gyro_z, 6);
	ips114_show_int32(0, 95, imu660ra_gyro_z * 0.01, 6);
	printf("%f,%f,%f,%f\n", test_speed, speed_l, speed_r, 0.0);
}

void printf_butten_test()
{
	ips114_show_int32(4 * 24, 18 * 0, P33, 1);
	ips114_show_int32(4 * 24, 18 * 1, P34, 1);
	ips114_show_int32(4 * 24, 18 * 2, P35, 1);
	ips114_show_int32(4 * 24, 18 * 4, P36, 1);
	ips114_show_int32(4 * 24, 18 * 5, P37, 1);
	ips114_show_int32(4 * 24, 18 * 6, flat_statr, 2);
}

/*******************************************************************************
 * VOFA+ 使用示例
 *
 * 左轮 PID_Direction:
 *   L_KP=100!
 *   L_KI=40!
 *   L_KD=0!
 *
 * 右轮 PID_Direction:
 *   R_KP=100!
 *   R_KI=40!
 *
 * 位置式 PID_Direction:
 *   P_KP=0.3!
 *   P_KI=0.1!
 *   P_KD=4!
 *   KD_GYRO=2.5!
 *
 * 运行控制:
 *   SPEED=50!      - 设置目标速度
 *   START!         - 启动电机
 *   STOP!          - 停止电机
 *   SAVE!          - 保存参数
 *   INFO!          - 查看当前参数
 *   RESET!         - 软复位系统
 ******************************************************************************/
