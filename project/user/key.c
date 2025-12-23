#include "zf_common_headfile.h"

// 定义按键引脚
#define KEY1_PIN P33 // 上键
#define KEY2_PIN P34 // 下键
#define KEY3_PIN P36 // 确定键
#define KEY4_PIN P37 // 返回键
// 长按时间阈值(单位：次数，取决于调用扫描函数的频率)
#define LONG_PRESS_THRESHOLD 200

uint8 keystroke_label = 0;      // 按下的是哪个键（0=无按键，1-4=短按，5-8=长按）
uint8 key_last_status[4] = {0}; // 上一次按键状态
uint8 key_status[4] = {0};      // 当前按键状态
uint8 key_flag[4] = {0};        // 按键标志位
uint16 key_press_time[4] = {0}; // 按键按下持续时间计数

/**
 * @brief 按键扫描函数
 * @details 读取4个按键的状态，判断短按和长按
 * @note 需要定期调用以实现按键检测
 */
void Keystroke_Scan(void)
{
    uint8 i = 0;
    keystroke_label = 0; // 重置按键标识

    // 保存上一次按键状态
    for (i = 0; i < 4; i++)
    {
        key_last_status[i] = key_status[i];
    }

    // 读取当前按键状态（按键按下为低电平(0)，取反使按下状态在程序中用1表示）
    key_status[0] = !KEY1_PIN; // 上键状态，按下则值为1
    key_status[1] = !KEY2_PIN; // 下键状态，按下则值为1
    key_status[2] = !KEY3_PIN; // 确定键状态，按下则值为1
    key_status[3] = !KEY4_PIN; // 返回键状态，按下则值为1

    // 逐个检测按键状态变化
    for (i = 0; i < 4; i++)
    {
        if (key_status[i]) // 按键被按下
        {
            if (!key_last_status[i]) // 按键刚刚按下（上升沿）
            {
                key_press_time[i] = 0; // 重置计数器
            }
            else // 按键持续按下
            {
                key_press_time[i]++;

                // 检测长按阈值
                if (key_press_time[i] == LONG_PRESS_THRESHOLD)
                {
                    keystroke_label = i + 5; // 长按对应值为5-8
                    break;                   // 检测到长按后立即退出循环
                }
            }
        }
        else // 按键未按下
        {
            // 如果按键释放且之前状态为按下，且未达到长按阈值，则为短按
            if (key_last_status[i] && key_press_time[i] < LONG_PRESS_THRESHOLD)
            {
                keystroke_label = i + 1; // 短按对应值为1-4
                break;                   // 检测到短按后立即退出循环
            }
            key_press_time[i] = 0; // 重置计数器
        }
    }
}