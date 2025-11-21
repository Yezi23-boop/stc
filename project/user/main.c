/*********************************************************************************************************************
 * VOFA+ 集成示例 - 基于您的 main.c 修改
 *
 * 功能：
 * 1. 接收 VOFA+ FireWater 协议命令
 * 2. 实时调整 PID 参数
 * 3. 控制电机运行
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "vofa.h"
#include <stdlib.h>

// 函数声明
void printf_adc();
void printf_imu();
void printf_date();
void printf_speed_test();
void handle_vofa_command(char *cmd);

void main()
{
	char vofa_cmd[32]; // VOFA 命令缓冲区（C89要求变量声明在开头）

	clock_init(SYSTEM_CLOCK_40M); // 务必保留
	debug_init();				  // 务必保留
	P32 = 1;					  // 开启快速烧录

	// 初始化（wireless_uart_init 已在 int_user 中完成）
	int_user();
	vofa_init(); // 初始化 VOFA 解析器

	while (1)
	{
		// ========== VOFA 命令处理 ==========
		// 从 FIFO 读取并解析数据（使用系统自带的 wireless_uart_read_buffer）
		vofa_parse_from_fifo();

		// 检查是否有完整命令
		if (vofa_get_command(vofa_cmd, 32))
		{
			handle_vofa_command(vofa_cmd);
		}

		// ========== 数据发送到 VOFA+ ==========
		// 使用 FireWater 协议发送数据
		// printf("%f,%f,%f,%f\n", speed_l, speed_r, test_speed, motor);

		// ========== 其他显示功能（可选） ==========
		// printf_date();
		// printf_adc();
		// printf_imu();
		// printf_speed_test();
		// Keystroke_Menu();
	}
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     处理 VOFA+ 接收到的命令
// 参数说明     cmd         接收到的命令字符串
// 返回参数     void
// 使用示例     handle_vofa_command("KP=1.5");
//-------------------------------------------------------------------------------------------------------------------
void handle_vofa_command(char *cmd)
{
	char *eq_pos;
	char param_name[16];
	uint8 name_len;
	float value;
	uint8 i;

	// 初始化数组
	for (i = 0; i < 16; i++)
	{
		param_name[i] = 0;
	}

	eq_pos = strchr(cmd, '=');

	if (eq_pos != NULL)
	{
		// ========== 处理带参数的命令 ==========
		name_len = (uint8)(eq_pos - cmd);

		if (name_len < 16)
		{
			// 提取参数名
			memcpy(param_name, cmd, name_len);
			param_name[name_len] = '\0';

			// 提取参数值
			value = atof(eq_pos + 1);

			// ========== 左电机 PID 参数 ==========
			if (strcmp(param_name, "L_KP") == 0)
			{
				motors_pid.left_PID.Kp = value;
				printf("Left Kp = %.2f\n", value);
				ips114_show_float(1 * 24, 18 * 0, value, 3, 2);
			}
			else if (strcmp(param_name, "L_KI") == 0)
			{
				motors_pid.left_PID.Ki = value;
				printf("Left Ki = %.2f\n", value);
			}
			else if (strcmp(param_name, "L_KD") == 0)
			{
				motors_pid.left_PID.Kd = value;
				printf("Left Kd = %.2f\n", value);
			}

			// ========== 右电机 PID 参数 ==========
			else if (strcmp(param_name, "R_KP") == 0)
			{
				motors_pid.right_PID.Kp = value;
				printf("Right Kp = %.2f\n", value);
			}
			else if (strcmp(param_name, "R_KI") == 0)
			{
				motors_pid.right_PID.Ki = value;
				printf("Right Ki = %.2f\n", value);
			}
			else if (strcmp(param_name, "R_KD") == 0)
			{
				motors_pid.right_PID.Kd = value;
				printf("Right Kd = %.2f\n", value);
			}

			// ========== 位置环 PID 参数 ==========
			else if (strcmp(param_name, "P_KP") == 0)
			{
				motors_pid.Positional_PID.Kp = value;
				kp_Err = value; // 同步更新
				printf("Position Kp = %.2f\n", value);
			}
			else if (strcmp(param_name, "P_KI") == 0)
			{
				motors_pid.Positional_PID.Ki = value;
				printf("Position Ki = %.2f\n", value);
			}
			else if (strcmp(param_name, "P_KD") == 0)
			{
				motors_pid.Positional_PID.Kd = value;
				kd_Err = value; // 同步更新
				printf("Position Kd = %.2f\n", value);
			}
			else if (strcmp(param_name, "KD_GYRO") == 0)
			{
				kd_gyro = value;
				motors_pid.Positional_PID.Ki = value; // 陀螺仪微分系数
				printf("Kd Gyro = %.2f\n", value);
			}

			// ========== 速度控制 ==========
			else if (strcmp(param_name, "SPEED") == 0)
			{
				test_speed = value;
				printf("Target Speed = %.2f\n", value);
			}
			else if (strcmp(param_name, "RP") == 0)
			{
				// 处理 RP 参数（根据您的实际需求修改）
				test_speed = value; // 或者改成其他变量
				printf("RP = %.2f\n", value);
			}
			else if (strcmp(param_name, "MOTOR") == 0)
			{
				motor = value;
				printf("Motor = %.2f\n", value);
			}
			else if (strcmp(param_name, "SPEED_RUN") == 0)
			{
				speed_run = value;
				printf("Speed Run = %.2f\n", value);
			}

			// ========== 其他参数 ==========
			else if (strcmp(param_name, "ERR") == 0)
			{
				Err = value;
				printf("Error = %.2f\n", value);
			}
			else
			{
				printf("Unknown parameter: %s = %.2f\n", param_name, value);
			}
		}
	}
	else
	{
		// ========== 处理无参数的命令 ==========
		if (strcmp(cmd, "START") == 0)
		{
			// 启动电机
			printf("Motor START\n");
			// 可以添加启动标志位
		}
		else if (strcmp(cmd, "STOP") == 0)
		{
			// 停止电机
			motor = 0;
			speed_run = 0;
			test_speed = 0;
			printf("Motor STOP\n");
		}
		else if (strcmp(cmd, "RESET") == 0)
		{
			// 复位系统
			IAP_CONTR = 0x60; // 软件复位
		}
		else if (strcmp(cmd, "SAVE") == 0)
		{
			// 保存参数到 EEPROM
			eeprom_flash();
			printf("Parameters saved\n");
		}
		else if (strcmp(cmd, "LOAD") == 0)
		{
			// 从 EEPROM 加载参数
			eeprom_init();
			printf("Parameters loaded\n");
		}
		else if (strcmp(cmd, "INFO") == 0)
		{
			// 打印当前参数信息
			printf("=== Current Parameters ===\n");
			printf("Left PID: %.2f, %.2f, %.2f\n",
				   motors_pid.left_PID.Kp,
				   motors_pid.left_PID.Ki,
				   motors_pid.left_PID.Kd);
			printf("Right PID: %.2f, %.2f, %.2f\n",
				   motors_pid.right_PID.Kp,
				   motors_pid.right_PID.Ki,
				   motors_pid.right_PID.Kd);
			printf("Speed: %.2f\n", test_speed);
		}
		else
		{
			printf("Unknown command: %s\n", cmd);
		}
	}
}

// 原有的显示函数保持不变
void printf_date()
{
	ips114_show_float(1 * 24, 18 * 0, Err, 3, 1);
	ips114_show_float(1 * 24, 18 * 1, ad1, 4, 1);
	ips114_show_float(1 * 24, 18 * 2, ad2, 4, 1);
	ips114_show_float(1 * 24, 18 * 3, ad3, 4, 1);
	ips114_show_float(1 * 24, 18 * 4, ad4, 4, 1);

	ips114_show_float(3 * 24, 18 * 0, motor, 3, 1);
	ips114_show_float(3 * 24, 18 * 1, speed_run + motor, 3, 1);
	ips114_show_float(3 * 24, 18 * 2, speed_run - motor, 3, 1);

	ips114_show_float(6 * 24, 18 * 1, adc_convert(ADC_CH13_P05) * 0.0092, 3, 2);
}

void printf_adc()
{
	ips114_show_float(1 * 24, 18 * 0, ad1, 3, 1);
	ips114_show_float(1 * 24, 18 * 1, ad2, 3, 1);
	ips114_show_float(1 * 24, 18 * 2, ad3, 3, 1);
	ips114_show_float(1 * 24, 18 * 3, ad4, 3, 1);

	ips114_show_float(1 * 24, 18 * 6, Err, 4, 1);

	ips114_show_float(3 * 24, 18 * 0, RAW[0], 4, 1);
	ips114_show_float(3 * 24, 18 * 1, RAW[1], 4, 1);
	ips114_show_float(3 * 24, 18 * 2, RAW[2], 4, 1);
	ips114_show_float(3 * 24, 18 * 3, RAW[3], 4, 1);

	ips114_show_float(7 * 24, 18 * 0, MA[0], 4, 1);
	ips114_show_float(7 * 24, 18 * 1, MA[1], 4, 1);
	ips114_show_float(7 * 24, 18 * 2, MA[2], 4, 1);
	ips114_show_float(7 * 24, 18 * 3, MA[3], 4, 1);
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
	ips114_show_float(0, 15, motor, 6, 1);
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
 * VOFA+ 命令示例：
 *
 * 调整左电机 PID:
 *   L_KP=100!
 *   L_KI=40!
 *   L_KD=0!
 *
 * 调整右电机 PID:
 *   R_KP=100!
 *   R_KI=40!
 *
 * 调整位置环 PID:
 *   P_KP=0.3!
 *   P_KI=0.1!
 *   P_KD=4!
 *   KD_GYRO=2.5!
 *
 * 控制命令:
 *   SPEED=50!      - 设置目标速度
 *   START!         - 启动电机
 *   STOP!          - 停止电机
 *   SAVE!          - 保存参数
 *   INFO!          - 查看当前参数
 *   RESET!         - 复位系统
 ******************************************************************************/
