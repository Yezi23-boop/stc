/*********************************************************************************************************************
 * VOFA+ FireWater 协议使用示例
 *
 * 功能说明：
 * 1. 接收 VOFA+ 上位机发送的 FireWater 协议数据
 * 2. 解析命令格式如："PR=2.32!"
 * 3. 提取参数名和数值进行处理
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "vofa.h"

//-------------------------------------------------------------------------------------------------------------------
// 示例1: 在串口中断中调用 VOFA 解析函数
//-------------------------------------------------------------------------------------------------------------------
// 这个函数应该在 isr.c 中的 DMA_UART3_IRQHandler 中调用
// 因为你的 wireless_uart 使用的是 UART_3

void uart3_vofa_callback(uint8 dat)
{
    // 将接收到的每个字节传递给 VOFA 解析器
    vofa_parse_byte(dat);
}

//-------------------------------------------------------------------------------------------------------------------
// 示例2: 在主循环中处理接收到的命令
//-------------------------------------------------------------------------------------------------------------------
void vofa_process_in_main(void)
{
    char cmd[32];

    // 检查是否有新命令
    if (vofa_get_command(cmd, 32))
    {
        // 成功接收到完整命令
        printf("Received command: %s\n", cmd);

        // 解析并处理命令
        vofa_parse_command(cmd);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 示例3: 完整的初始化和使用流程（在 int_user.c 中）
//-------------------------------------------------------------------------------------------------------------------
void vofa_user_init(void)
{
    // 1. 初始化无线串口（已经在 int_user.c 中完成）
    // wireless_uart_init();

    // 2. 初始化 VOFA 数据结构
    vofa_init();

    // 3. 设置串口3的回调函数为 VOFA 解析函数
    // 在 isr.c 的 DMA_UART3_IRQHandler 中：
    // if (uart3_irq_handler != NULL)
    // {
    //     uart3_irq_handler(uart_rx_buff[UART_3][0]);  // 这里会调用 vofa_parse_byte
    // }
}

//-------------------------------------------------------------------------------------------------------------------
// 示例4: 实际应用 - PID 参数调节
//-------------------------------------------------------------------------------------------------------------------
void vofa_pid_tuning_example(void)
{
    char cmd[32];

    if (vofa_get_command(cmd, 32))
    {
        char *eq_pos = strchr(cmd, '=');

        if (eq_pos != NULL)
        {
            char param_name[16] = {0};
            uint8 name_len = (uint8)(eq_pos - cmd);
            float value;

            if (name_len < 16)
            {
                memcpy(param_name, cmd, name_len);
                param_name[name_len] = '\0';
                value = atof(eq_pos + 1);

                // 根据参数名设置不同的 PID 参数
                if (strcmp(param_name, "L_KP") == 0)
                {
                    // 设置左电机 P 参数
                    motors_pid.left_PID.Kp = value;
                    printf("Left Motor Kp = %.2f\n", value);
                }
                else if (strcmp(param_name, "L_KI") == 0)
                {
                    motors_pid.left_PID.Ki = value;
                    printf("Left Motor Ki = %.2f\n", value);
                }
                else if (strcmp(param_name, "L_KD") == 0)
                {
                    motors_pid.left_PID.Kd = value;
                    printf("Left Motor Kd = %.2f\n", value);
                }
                else if (strcmp(param_name, "R_KP") == 0)
                {
                    motors_pid.right_PID.Kp = value;
                    printf("Right Motor Kp = %.2f\n", value);
                }
                else if (strcmp(param_name, "R_KI") == 0)
                {
                    motors_pid.right_PID.Ki = value;
                    printf("Right Motor Ki = %.2f\n", value);
                }
                else if (strcmp(param_name, "R_KD") == 0)
                {
                    motors_pid.right_PID.Kd = value;
                    printf("Right Motor Kd = %.2f\n", value);
                }
                else if (strcmp(param_name, "SPEED") == 0)
                {
                    // 设置目标速度
                    test_speed = value;
                    printf("Target Speed = %.2f\n", value);
                }
            }
        }
        else
        {
            // 处理无参数的命令
            if (strcmp(cmd, "START") == 0)
            {
                // 启动电机
                printf("Motor START\n");
            }
            else if (strcmp(cmd, "STOP") == 0)
            {
                // 停止电机
                motor = 0;
                speed_run = 0;
                printf("Motor STOP\n");
            }
            else if (strcmp(cmd, "SAVE") == 0)
            {
                // 保存参数到 EEPROM
                eeprom_flash();
                printf("Parameters saved to EEPROM\n");
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 示例5: 数据格式详解
//-------------------------------------------------------------------------------------------------------------------
/*
 * VOFA+ FireWater 协议数据格式：
 *
 * 发送端（VOFA+ 上位机）:
 *   输入: "PR=2.32!"
 *
 * 转换过程:
 *   'P' -> ASCII: 80  (0x50)
 *   'R' -> ASCII: 82  (0x52)
 *   '=' -> ASCII: 61  (0x3D)
 *   '2' -> ASCII: 50  (0x32)
 *   '.' -> ASCII: 46  (0x2E)
 *   '3' -> ASCII: 51  (0x33)
 *   '2' -> ASCII: 50  (0x32)
 *   '!' -> ASCII: 33  (0x21) <- 帧尾标识
 *
 * 接收端（单片机）:
 *   1. 串口中断逐字节接收: 0x50, 0x52, 0x3D, 0x32, 0x2E, 0x33, 0x32, 0x21
 *   2. vofa_parse_byte() 将每个字节存入缓冲区
 *   3. 检测到 '!' (0x21) 时，标记接收完成
 *   4. 缓冲区内容: "PR=2.32"（不包括'!'）
 *   5. 调用 vofa_parse_command() 解析命令
 *
 * 解析结果:
 *   param_name = "PR"
 *   param_value = 2.32
 */
