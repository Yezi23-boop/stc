#ifndef _KEY_H_
#define _KEY_H_

// 按键标识变量，用于标识当前按下的按键
// 0=无按键，1=上键短按，2=下键短按，3=确定键短按，4=返回键短按
// 5=上键长按，6=下键长按，7=确定键长按，8=返回键长按
extern uint8 keystroke_label;

/**
 * @brief 按键扫描函数
 * @details 读取4个按键的状态，判断短按和长按
 * @note 需要定期调用以实现按键检测
 */
void Keystroke_Scan(void);

#endif