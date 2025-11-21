#include "zf_common_headfile.h"
#include "vofa.h"
#include <stdlib.h>

// VOFA 数据对象
static vofa_data_struct vofa_data;

// 内部函数声明
static void vofa_parse_byte(uint8 dat);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     VOFA+ 初始化
// 参数说明     void
// 返回参数     void
// 使用示例     vofa_init();
// 备注信息     初始化 VOFA 数据结构（无需额外操作，使用系统自带 FIFO）
//-------------------------------------------------------------------------------------------------------------------
void vofa_init(void)
{
    uint8 i;

    // 清空缓冲区
    for (i = 0; i < VOFA_BUFFER_SIZE; i++)
    {
        vofa_data.buffer[i] = 0;
    }

    for (i = 0; i < VOFA_MAX_CMD_LEN; i++)
    {
        vofa_data.cmd_buffer[i] = 0;
    }

    vofa_data.index = 0;
    vofa_data.cmd_len = 0;
    vofa_data.state = VOFA_PARSE_IDLE;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     VOFA+ FireWater 协议从 FIFO 读取并解析数据
// 参数说明     void
// 返回参数     void
// 使用示例     vofa_parse_from_fifo();  // 在主循环中调用
// 备注信息     FireWater 协议: 以 '!' (0x21) 作为帧尾标识
//              直接使用 wireless_uart_read_buffer() 从 FIFO 读取数据
//-------------------------------------------------------------------------------------------------------------------
void vofa_parse_from_fifo(void)
{
    uint8 dat;

    // 从 FIFO 读取数据（使用系统自带函数）
    while (wireless_uart_read_buffer(&dat, 1) > 0)
    {
        vofa_parse_byte(dat);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     VOFA+ FireWater 协议解析单个字节（内部函数）
// 参数说明     dat             接收到的字节数据
// 返回参数     void
// 使用示例     vofa_parse_byte(0x50);
// 备注信息     FireWater 协议: 以 '!' (0x21) 作为帧尾标识
//              例如: "PR=2.32!" 会被解析为完整命令
//-------------------------------------------------------------------------------------------------------------------
static void vofa_parse_byte(uint8 dat)
{
    // 检测到帧尾标识 '!' (ASCII: 0x21 即十进制 33)
    if (dat == '!' || dat == 0x21)
    {
        if (vofa_data.index > 0)
        {
            // 添加字符串结束符
            vofa_data.buffer[vofa_data.index] = '\0';

            // 复制到命令缓冲区
            memcpy(vofa_data.cmd_buffer, vofa_data.buffer, vofa_data.index + 1);
            vofa_data.cmd_len = vofa_data.index;

            // 标记解析完成
            vofa_data.state = VOFA_PARSE_COMPLETE;

            // 重置索引
            vofa_data.index = 0;
        }
    }
    else
    {
        // 接收数据
        if (vofa_data.index < VOFA_BUFFER_SIZE - 1)
        {
            vofa_data.buffer[vofa_data.index++] = dat;
            vofa_data.state = VOFA_PARSE_RECEIVING;
        }
        else
        {
            // 缓冲区溢出，重置
            vofa_data.index = 0;
            vofa_data.state = VOFA_PARSE_IDLE;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取 VOFA+ 接收到的完整命令
// 参数说明     cmd_out         输出命令字符串的缓冲区
// 参数说明     max_len         缓冲区最大长度
// 返回参数     uint8           1-有新命令 0-无新命令
// 使用示例     char cmd[32];
//              if(vofa_get_command(cmd, 32)) {
//                  // 处理命令
//              }
// 备注信息     获取命令后会自动清除完成标志
//-------------------------------------------------------------------------------------------------------------------
uint8 vofa_get_command(char *cmd_out, uint8 max_len)
{
    uint8 result = 0;

    if (vofa_data.state == VOFA_PARSE_COMPLETE)
    {
        // 复制命令
        if (vofa_data.cmd_len < max_len)
        {
            memcpy(cmd_out, vofa_data.cmd_buffer, vofa_data.cmd_len + 1);
            result = 1;
        }

        // 清除完成标志
        vofa_data.state = VOFA_PARSE_IDLE;
        vofa_data.cmd_len = 0;
    }

    return result;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清空 VOFA 接收缓冲区
// 参数说明     void
// 返回参数     void
// 使用示例     vofa_clear_buffer();
// 备注信息     在出现接收错误时可以调用此函数清空缓冲区
//-------------------------------------------------------------------------------------------------------------------
void vofa_clear_buffer(void)
{
    vofa_data.index = 0;
    vofa_data.cmd_len = 0;
    vofa_data.state = VOFA_PARSE_IDLE;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     VOFA+ FireWater 协议命令解析示例
// 参数说明     cmd             接收到的命令字符串
// 返回参数     void
// 使用示例     vofa_parse_command("PR=2.32");
// 备注信息     示例函数，展示如何解析不同格式的命令
//              格式1: "PR=2.32"  - 解析参数名和浮点数值
//              格式2: "SPEED=100" - 解析参数名和整数值
//              格式3: "START" - 单独的命令
//-------------------------------------------------------------------------------------------------------------------
void vofa_parse_command(char *cmd)
{
    char *eq_pos;
    char param_name[16];
    float param_value;
    uint8 name_len;
    uint8 i;

    // 初始化变量
    for (i = 0; i < 16; i++)
    {
        param_name[i] = 0;
    }
    param_value = 0.0;

    // 查找等号位置
    eq_pos = strchr(cmd, '=');

    if (eq_pos != NULL)
    {
        // 有等号，说明是参数设置命令
        name_len = (uint8)(eq_pos - cmd);

        if (name_len < 16)
        {
            // 提取参数名
            memcpy(param_name, cmd, name_len);
            param_name[name_len] = '\0';

            // 提取参数值
            param_value = atof(eq_pos + 1);

            // 根据参数名执行不同操作
            if (strcmp(param_name, "PR") == 0)
            {
                // 处理 PR 参数
                printf("Received PR = %.2f\n", param_value);
                // 在这里添加你的处理代码
            }
            else if (strcmp(param_name, "SPEED") == 0)
            {
                // 处理 SPEED 参数
                printf("Received SPEED = %.2f\n", param_value);
            }
            else if (strcmp(param_name, "KP") == 0)
            {
                // 处理 KP 参数（PID参数）
                printf("Received KP = %.2f\n", param_value);
            }
            else if (strcmp(param_name, "KI") == 0)
            {
                // 处理 KI 参数
                printf("Received KI = %.2f\n", param_value);
            }
            else if (strcmp(param_name, "KD") == 0)
            {
                // 处理 KD 参数
                printf("Received KD = %.2f\n", param_value);
            }
        }
    }
    else
    {
        // 无等号，说明是单独的命令
        if (strcmp(cmd, "START") == 0)
        {
            printf("Received START command\n");
            // 执行启动操作
        }
        else if (strcmp(cmd, "STOP") == 0)
        {
            printf("Received STOP command\n");
            // 执行停止操作
        }
        else if (strcmp(cmd, "RESET") == 0)
        {
            printf("Received RESET command\n");
            // 执行复位操作
        }
    }
}