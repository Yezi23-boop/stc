#include "zf_common_headfile.h"
/*********************************************
 * 按键和菜单系统基础定义
 *********************************************/
// 按键代号定义
#define KEYSTROKE_ONE 1   // 上
#define KEYSTROKE_TWO 2   // 下
#define KEYSTROKE_THREE 3 // 确定
#define KEYSTROKE_FOUR 4  // 返回

// 屏幕显示参数定义
#define ROWS_MAX 7 * 18      // 光标在屏幕上可移动至的最大行数
#define ROWS_MIN 1 * 18      // 光标在屏幕上可移动至的最小行数
#define CENTER_COLUMN 12 * 8 // 中央列
#define EEPROM_MODE 1        // eeporm读写开启则为1

/*********************************************
 * 全局变量定义
 *********************************************/
int display_codename = 0;       // 显示页面代号
int cursor_row = 2 * 18;        // 光标所在行号
int previous_cursor_row = -1;   // 上一次光标所在列号
int menu_next_flag = 0;         // 光标所指菜单进入标志位
float change_unit = 0;          // 单次修改的单位值
int change_unit_multiplier = 1; // 修改单位倍数
int keystroke_three_count = 0;  // 定义一个全局变量记录KEYSTROKE_THREE的触发次数
int Have_Sub_Menu(int menu_id);
void Keystroke_Menu_HOME(void);
void Menu_Start_Process(void);
void Menu_Speed_Process(void);
void Menu_Angle_Process(void);
void Menu_Sensor_Process(void);
void Menu_Circle_Process(void);
/*********************************************
 * 特殊字符数据定义
 *********************************************/
// 中文字符"猥琐车"的点阵数据
const uint8 chinese_data[] = {
    // "猥"
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x00, 0x20,
    0x10, 0x3B, 0xFF, 0xF0, 0x0C, 0x33, 0x0C, 0x30, 0x06, 0x63, 0x0C, 0x30, 0x03, 0xC3, 0x0C, 0x30,
    0x01, 0x83, 0x0C, 0x30, 0x01, 0x83, 0xFF, 0xF0, 0x02, 0x83, 0x0C, 0x30, 0x04, 0xC3, 0x0C, 0x30,
    0x18, 0xC3, 0x0C, 0x30, 0x20, 0x43, 0x0C, 0x30, 0x00, 0xC3, 0xFF, 0xF0, 0x00, 0xE3, 0x00, 0x30,
    0x01, 0xE2, 0x00, 0x00, 0x01, 0x60, 0x00, 0x18, 0x03, 0x7F, 0xFF, 0xFC, 0x06, 0x61, 0x90, 0x00,
    0x04, 0x61, 0x88, 0x10, 0x08, 0x61, 0x88, 0x38, 0x10, 0x61, 0x8C, 0x70, 0x20, 0x61, 0x85, 0x80,
    0x40, 0x61, 0x86, 0x00, 0x00, 0x61, 0x83, 0x00, 0x00, 0x61, 0x89, 0xC0, 0x00, 0xC1, 0xB0, 0xE0,
    0x0F, 0xC1, 0xC0, 0x7E, 0x03, 0x83, 0x80, 0x18, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // "琐"
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x0C, 0x00,
    0x00, 0x03, 0x0C, 0x30, 0x00, 0x31, 0x8C, 0x30, 0x3F, 0xF8, 0xCC, 0x60, 0x01, 0x00, 0xCC, 0x80,
    0x01, 0x00, 0xCD, 0x00, 0x01, 0x02, 0x0D, 0x00, 0x01, 0x03, 0xFF, 0xF8, 0x01, 0x03, 0x00, 0x30,
    0x01, 0x03, 0x00, 0x30, 0x01, 0x13, 0x08, 0x30, 0x3F, 0xFB, 0x0E, 0x30, 0x01, 0x03, 0x0C, 0x30,
    0x01, 0x03, 0x0C, 0x30, 0x01, 0x03, 0x0C, 0x30, 0x01, 0x03, 0x0C, 0x30, 0x01, 0x03, 0x0C, 0x30,
    0x01, 0x03, 0x08, 0x30, 0x01, 0x03, 0x08, 0x30, 0x01, 0x03, 0x18, 0x30, 0x01, 0x1F, 0x18, 0x30,
    0x01, 0xE2, 0x18, 0x00, 0x0F, 0x00, 0x36, 0x00, 0x3C, 0x00, 0x31, 0xC0, 0x30, 0x00, 0x60, 0xF0,
    0x00, 0x00, 0xC0, 0x38, 0x00, 0x03, 0x00, 0x1C, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x30, 0x00, 0x00,
    // "车"
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00,
    0x00, 0x0E, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x1C, 0x00, 0x60, 0x0F, 0xFF, 0xFF, 0xF0,
    0x00, 0x18, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x31, 0xC0, 0x00, 0x00, 0x61, 0x80, 0x00,
    0x00, 0xC1, 0x80, 0x00, 0x00, 0xC1, 0x80, 0x00, 0x01, 0x81, 0x80, 0x00, 0x03, 0x81, 0x80, 0xC0,
    0x07, 0xFF, 0xFF, 0xE0, 0x03, 0x01, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00,
    0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0x80, 0x18, 0x3F, 0xFF, 0xFF, 0xFC, 0x00, 0x01, 0x80, 0x00,
    0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00,
    0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00};

/*********************************************
 * 菜单结构配置数组
 *********************************************/
// 将有菜单页面的代号填入该数组中，防止由箭头所在行号所决定进入不存在的菜单
int menu_have_sub[] = {
    0, // 主菜单
    1,
    11,
    12,
    13,
    14,
    15,
    16,
    17, // 启动配置菜单组
    2,
    21,
    22,
    23,
    24,
    25,
    26, // PID速度参数菜单组
    3,
    31,
    32,
    33,
    34,
    35,
    36, // PID角度参数菜单组
    4,  // 电感数据显示菜单
    5,
    51,
    52,
    53,
    54,
    55,
    56, // 圆环控制参数菜单组
};

/*********************************************
 * 核心功能函数实现
 *********************************************/
/**
 * @brief 菜单箭头标识处理
 * @details 处理光标移动和菜单进入/退出操作
 */
void Cursor(void)
{
    menu_next_flag = 0;
    switch (keystroke_label)
    {
    case KEYSTROKE_ONE:
        cursor_row = (cursor_row > ROWS_MIN) ? cursor_row - 1 * 18 : ROWS_MAX; // 光标行上移
        break;
    case KEYSTROKE_TWO:
        cursor_row = (cursor_row < ROWS_MAX) ? cursor_row + 1 * 18 : ROWS_MIN; // 光标行下移
        break;
    case KEYSTROKE_THREE:
        menu_next_flag = 1; // 进入下级菜单
        break;
    case KEYSTROKE_FOUR:
        menu_next_flag = -1; // 返回上级菜单
        break;
    }

    ips114_show_string(0, cursor_row, ">"); // 在当前位置显示箭头

    // 清除之前箭头位置的显示，避免残留
    if (previous_cursor_row != cursor_row)
    {
        ips114_show_string(0, previous_cursor_row, " ");
        previous_cursor_row = cursor_row;
    }
}

/**
 * @brief 菜单上下级跳转处理
 * @details 处理菜单层级的切换逻辑
 */
void Menu_Next_Back()
{
    int menu_id = 0;
    switch (menu_next_flag)
    {
    case 0:
        break;
    case -1: // 返回上一级
        display_codename = display_codename / 10;
        cursor_row = ROWS_MIN;
        ips114_clear(RGB565_WHITE);
        break;
    case 1: // 进入下一级
        menu_id = display_codename * 10 + (cursor_row / 18);
        if (Have_Sub_Menu(menu_id))
        {
            display_codename = menu_id;
            ips114_clear(RGB565_WHITE);
        }
        break;
    }
    menu_next_flag = 0;
}

/**
 * @brief 检查本行是否存在子菜单
 * @param menu_id 菜单ID
 * @return 1=存在子菜单，0=不存在子菜单
 */
int Have_Sub_Menu(int menu_id)
{
    int i = 0;
    for (i = 0; i < sizeof(menu_have_sub) / sizeof(menu_have_sub[0]); i++)
    {
        if (menu_have_sub[i] == menu_id)
            return 1;
    }
    return 0;
}

/*********************************************
 * 按键处理和参数修改函数
 *********************************************/
/**
 * @brief 处理按键扫描返回页与参数修改倍数逻辑
 * @param keystroke_label 按键标识
 */
void HandleKeystroke(int keystroke_label)
{
    switch (keystroke_label)
    {
    case KEYSTROKE_FOUR:
        display_codename /= 10; // 返回上一页
        ips114_clear(RGB565_WHITE);
        break;
    case KEYSTROKE_THREE:
        keystroke_three_count++;
        change_unit_multiplier = (keystroke_three_count % 3 == 0) ? 1 : (keystroke_three_count % 3 == 1) ? 10
                                                                                                         : 100;
        if (keystroke_three_count >= 3)
            keystroke_three_count = 0;
        break;
    }
}

/**
 * @brief 整型参数修改
 * @param parameter 需要修改的参数指针
 * @param change_unit_MIN 最小修改单位
 */
void Keystroke_int(int *parameter, int change_unit_MIN)
{
    int change_unit = change_unit_MIN * change_unit_multiplier;
    ips114_show_int32(15 * 8, 0, change_unit, 3);

    Keystroke_Scan();
    HandleKeystroke(keystroke_label);

    switch (keystroke_label)
    {
    case KEYSTROKE_ONE:
        *parameter += change_unit;
        break;
    case KEYSTROKE_TWO:
        *parameter -= change_unit;
        break;
    }
}

/**
 * @brief 浮点型参数修改
 * @param parameter 需要修改的参数指针
 * @param change_unit_MIN 最小修改单位
 */
void Keystroke_float(float *parameter, float change_unit_MIN)
{
    float change_unit = change_unit_MIN * change_unit_multiplier;
    ips114_show_float(14 * 8, 0, change_unit, 3, 3);
    Keystroke_Scan();
    HandleKeystroke(keystroke_label);

    switch (keystroke_label)
    {
    case KEYSTROKE_ONE:
        *parameter += change_unit;
        break;
    case KEYSTROKE_TWO:
        *parameter -= change_unit;
        break;
    }
}

/**
 * @brief 整型特值修改，-1或1
 * @param parameter 需要修改的参数指针
 */
void Keystroke_Special_Value(int *parameter)
{
    Keystroke_Scan();
    HandleKeystroke(keystroke_label);

    switch (keystroke_label)
    {
    case KEYSTROKE_ONE:
        *parameter = 1;
        break;
    case KEYSTROKE_TWO:
        *parameter = -1;
        break;
    }
}

/*********************************************
 * 菜单系统主控制函数
 *********************************************/
/**
 * @brief 菜单目录主函数
 * @details 根据display_codename显示对应菜单页面
 * @note 增删页面时请同步修改menu_have_sub[]数组
 */
void Keystroke_Menu(void)
{
    switch (display_codename)
    {
    case 0: // 主菜单
        Keystroke_Menu_HOME();
        break;

    case 1: // 启动配置菜单组
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
    case 16:
    case 17:
        Menu_Start_Process();
        break;

    case 2: // PID速度参数菜单组
    case 21:
    case 22:
    case 23:
    case 24:
    case 25:
    case 26:
        Menu_Speed_Process();
        break;

    case 3: // PID角度参数菜单组
    case 31:
    case 32:
    case 33:
    case 34:
    case 35:
    case 36:
        Menu_Angle_Process();
        break;

    case 4: // 电感数据显示菜单
        Menu_Sensor_Process();
        break;

    case 5: // 圆环控制参数菜单组
    case 51:
    case 52:
    case 53:
    case 54:
    case 55:
    case 56:
        Menu_Circle_Process();
        break;

    default:
        break;
    }
}

/*********************************************
 * 主菜单显示和处理
 *********************************************/
void Keystroke_Menu_HOME(void)
{
    while (menu_next_flag == 0)
    {
        // 显示菜单标题
        ips114_show_string(CENTER_COLUMN, 0, "MENU");

        // 显示菜单项
        ips114_show_string(8 * 2, 1 * 18, "STRAT");  // 启动配置
        ips114_show_string(8 * 2, 2 * 18, "PID_1");  // PID速度参数
        ips114_show_string(8 * 2, 3 * 18, "PID_2");  // PID角度参数
        ips114_show_string(8 * 2, 4 * 18, "PRINTF"); // 电感数据显示
        ips114_show_string(8 * 2, 5 * 18, "RING");   // 圆环控制参数

        // 显示实时数据标签
        ips114_show_string(7 * 15, 1 * 18, "Err");
        ips114_show_string(7 * 15, 2 * 18, "motor");
        ips114_show_string(7 * 15, 3 * 18, "phase");
        ips114_show_string(7 * 15, 4 * 18, "dianya");
        ips114_show_string(7 * 15, 5 * 18, "fuya_date");
        //        // 显示实时数据值
        ips114_show_float(8 * 23, 1 * 18, Err, 3, 1);
        ips114_show_float(8 * 23, 2 * 18, motor, 3, 1);
        ips114_show_float(8 * 23, 3 * 18, phase, 3, 1);
        ips114_show_float(8 * 23, 4 * 18, dianya, 1, 2);
        ips114_show_int32(8 * 23, 5 * 18, fuya_date, 4);

        ips114_show_float(7 * 15, 6 * 18, speed_run - motor, 4, 2);
		ips114_show_float(8 * 23, 6 * 18, speed_run + motor, 4, 2);
        Keystroke_Scan();
        Cursor();
    }

    // 菜单跳转处理
    if (menu_next_flag == 1)
    {
        int menu_id = display_codename * 10 + (cursor_row / 18);
        if (Have_Sub_Menu(menu_id))
        {
            display_codename = menu_id;
            cursor_row = ROWS_MIN;
            ips114_clear(RGB565_WHITE);
        }
    }
    // EEPROM保存处理
    else if (menu_next_flag == -1 && EEPROM_MODE == 1)
    {
        eeprom_flash();
        // 刷写完成提示
        ips114_clear(RGB565_WHITE);
        ips114_show_chinese(11 * 6, 8 * 6, 32, chinese_data, 3, RGB565_BLACK);
        system_delay_ms(200);
        ips114_clear(RGB565_WHITE);
    }

    menu_next_flag = 0;
}

/*********************************************
 * 启动配置菜单（菜单组1）
 *********************************************/
void Menu_Start_Show(uint8 control_line)
{
    ips114_show_string(4 * 8, 0 * 18, "<<STRAT");

    ips114_show_string(1 * 8, 1 * 18, "Start_Flag");   // 启动标志
    ips114_show_string(1 * 8, 2 * 18, "circle_flags"); // 出环方向

    ips114_show_int32(14 * 8, 1 * 18, start_flag, 3);
    ips114_show_int32(14 * 8, 2 * 18, circle_flags, 3);

    // 显示当前编辑标识
    if (control_line == 1)
    {
        ips114_show_string(0, control_line, " ");
    }
    else
    {
        ips114_show_string(0, control_line, "&");
    }
}

void Menu_Start_Process(void)
{
    switch (display_codename)
    {
    case 1: // 启动配置主菜单
        while (menu_next_flag == 0)
        {
            Menu_Start_Show(1);
            Keystroke_Scan();
            Cursor();
        }
        Menu_Next_Back();
        break;

    case 11: // 启动标志修改
        Menu_Start_Show(1 * 18);
        Keystroke_Special_Value(&start_flag);
        break;
    case 12: // 出环方向修改
        Menu_Start_Show(2 * 18);
        Keystroke_Special_Value(&circle_flags);
        break;
    }
}

/*********************************************
 * PID速度参数菜单（菜单组2）
 *********************************************/
void Menu_Speed_Show(uint8 control_line)
{
    ips114_show_string(1 * 8, 0 * 18, "<<PID_SPEED");

    ips114_show_string(1 * 8, 1 * 18, "kp_Err");     // 误差比例系数
    ips114_show_string(1 * 8, 2 * 18, "kd_Err");     // 误差微分系数
    ips114_show_string(1 * 8, 3 * 18, "speed_run");  // 运行速度
    ips114_show_string(1 * 8, 4 * 18, "kd_gyro");    // 误差限幅
    ips114_show_string(1 * 8, 5 * 18, "fuya_xili");  // 误差差分
    ips114_show_string(1 * 8, 6 * 18, "pwm_filter"); // PWM滤波

    ips114_show_float(14 * 8, 1 * 18, kp_Err, 3, 3);
    ips114_show_float(14 * 8, 2 * 18, kd_Err, 3, 3);
    ips114_show_float(14 * 8, 3 * 18, speed_run, 3, 3);
    ips114_show_float(14 * 8, 4 * 18, kd_gyro, 3, 3);
    ips114_show_float(14 * 8, 5 * 18, fuya_xili, 4, 3);
    ips114_show_float(14 * 8, 6 * 18, pwm_filter, 3, 3);

    // 显示当前编辑标识
    if (control_line == 1)
    {
        ips114_show_string(0, control_line, " ");
    }
    else
    {
        ips114_show_string(0, control_line, "&");
    }
}

void Menu_Speed_Process(void)
{
    switch (display_codename)
    {
    case 2: // PID速度参数主菜单
        while (menu_next_flag == 0)
        {
            Menu_Speed_Show(0);
            Keystroke_Scan();
            Cursor();
        }
        Menu_Next_Back();
        break;

    case 21: // 误差比例系数
        Menu_Speed_Show(1 * 18);
        Keystroke_float(&kp_Err, 0.01);
        break;
    case 22: // 误差微分系数
        Menu_Speed_Show(2 * 18);
        Keystroke_float(&kd_Err, 0.1);
        break;
    case 23: // 运行速度
        Menu_Speed_Show(3 * 18);
        Keystroke_float(&speed_run, 1);
        break;
    case 24: // 误差限幅
        Menu_Speed_Show(4 * 18);
        Keystroke_float(&kd_gyro, 0.01);
        break;
    case 25: // 误差差分
        Menu_Speed_Show(5 * 18);
        Keystroke_float(&fuya_xili, 100);
        break;
    case 26: // PWM滤波
        Menu_Speed_Show(6 * 18);
        Keystroke_float(&pwm_filter, 0.1);
        break;
    }
}

/*********************************************
 * PID角度参数菜单（菜单组3）
 *********************************************/
void Menu_Angle_Show(uint8 control_line)
{
    ips114_show_string(1 * 8, 0 * 18, "<<PID_ANGLE");

    ips114_show_string(1 * 8, 1 * 18, "kp_Angle");       // 角度比例系数
    ips114_show_string(1 * 8, 2 * 18, "kd_Angle");       // 角度微分系数
    ips114_show_string(1 * 8, 3 * 18, "limiting_Angle"); // 角度限幅
    ips114_show_string(1 * 8, 4 * 18, "A_1");            // 参数A_1
    ips114_show_string(1 * 8, 5 * 18, "B_1");            // 参数B_1
    ips114_show_string(1 * 8, 6 * 18, "C_l");        // 二次误差微分

    ips114_show_float(15 * 8, 1 * 18, kp_Angle, 3, 2);
    ips114_show_float(15 * 8, 2 * 18, kd_Angle, 3, 2);
    ips114_show_float(15 * 8, 3 * 18, limiting_Angle, 3, 2);
    ips114_show_float(14 * 8, 4 * 18, A_1, 3, 2);
    ips114_show_float(14 * 8, 5 * 18, B_1, 3, 2);
    ips114_show_float(14 * 8, 6 * 18, C_l, 3, 3);

    // 显示当前编辑标识
    if (control_line == 1)
    {
        ips114_show_string(0, control_line, " ");
    }
    else
    {
        ips114_show_string(0, control_line, "&");
    }
}

void Menu_Angle_Process(void)
{
    switch (display_codename)
    {
    case 3: // PID角度参数主菜单
        while (menu_next_flag == 0)
        {
            Menu_Angle_Show(1);
            Keystroke_Scan();
            Cursor();
        }
        Menu_Next_Back();
        break;

    case 31: // 角度比例系数
        Menu_Angle_Show(1 * 18);
        Keystroke_float(&kp_Angle, 0.01);
        break;
    case 32: // 角度微分系数
        Menu_Angle_Show(2 * 18);
        Keystroke_float(&kd_Angle, 0.01);
        break;
    case 33: // 角度限幅
        Menu_Angle_Show(3 * 18);
        Keystroke_float(&limiting_Angle, 10);
        break;
    case 34: // 参数A_1
        Menu_Angle_Show(4 * 18);
        Keystroke_float(&A_1, 0.01);
        break;
    case 35: // 参数B_1
        Menu_Angle_Show(5 * 18);
        Keystroke_float(&B_1, 0.01);
        break;
    case 36: // 二次误差微分
        Menu_Angle_Show(6 * 18);
        Keystroke_float(&C_l, 0.01);
        break;
    }
}

/*********************************************
 * 电感数据显示菜单（菜单4）
 *********************************************/
void Menu_Sensor_Show(void)
{
    // 显示五个电感的标签（已移除ad3和ad5）
    ips114_show_string(1 * 14, 18 * 0, "ad1"); // 最左侧电感
    ips114_show_string(1 * 14, 18 * 1, "ad2"); // 左侧电感
    ips114_show_string(1 * 14, 18 * 3, "ad3"); // 中间电感
    ips114_show_string(1 * 14, 18 * 5, "ad4"); // 右侧电感

    // 显示电感当前值
    ips114_show_float(14 * 4, 18 * 0, ad1, 3, 1);
    ips114_show_float(14 * 4, 18 * 1, ad2, 3, 1);
    ips114_show_float(14 * 4, 18 * 3, ad3, 3, 1);
    ips114_show_float(14 * 4, 18 * 5, ad4, 3, 1);

    // 显示电感最大值（用于标定参考）
    ips114_show_float(14 * 8, 18 * 0, MA[0], 5, 1); // ad1最大值
    ips114_show_float(14 * 8, 18 * 1, MA[1], 5, 1); // ad2最大值
    ips114_show_float(14 * 8, 18 * 2, MA[2], 5, 1); // ad4最大值
    ips114_show_float(14 * 8, 18 * 3, MA[3], 5, 1); // ad6最大值
}

void Menu_Sensor_Process(void)
{
    switch (display_codename)
    {
    case 4:
        while (menu_next_flag == 0)
        {
            Menu_Sensor_Show();
            Keystroke_Scan();
            Cursor();
        }
        Menu_Next_Back();
        break;
    }
}
/*********************************************
 * 圆环控制参数菜单（菜单组5）
 *********************************************/
void Menu_Circle_Show(uint8 control_line)
{
    ips114_show_string(1 * 8, 0 * 18, "<<RING_CTRL");

    ips114_show_string(1 * 8, 1 * 18, "ring_en"); // 圆环编码器阈值
    ips114_show_string(1 * 8, 2 * 18, "p_r_G");   // 入环前陀螺仪设定值
    ips114_show_string(1 * 8, 3 * 18, "i_r_G");   // 入环陀螺仪Z轴值
    ips114_show_string(1 * 8, 4 * 18, "p_o_G");   // 出环前陀螺仪设定值
    ips114_show_string(1 * 8, 5 * 18, "p_o__G");  // 出环前陀螺仪Z轴值
    ips114_show_string(1 * 8, 6 * 18, "p_o_en");  // 出环前编码器阈值

    ips114_show_float(14 * 8, 1 * 18, ring_encoder, 3, 2);
    ips114_show_float(14 * 8, 2 * 18, pre_ring_Gyro_set, 3, 2);
    ips114_show_float(14 * 8, 3 * 18, in_ring_Gyroz, 3, 2);
    ips114_show_float(14 * 8, 4 * 18, pre_out_ring_Gyro_set, 3, 2);
    ips114_show_float(14 * 8, 5 * 18, pre_out_ring_Gyroz, 3, 2);
    ips114_show_float(14 * 8, 6 * 18, pre_out_ring_encoder, 3, 2);

    // 显示当前编辑标识
    if (control_line == 1)
    {
        ips114_show_string(0, control_line, " ");
    }
    else
    {
        ips114_show_string(0, control_line, "&");
    }
}

void Menu_Circle_Process(void)
{
    switch (display_codename)
    {
    case 5: // 圆环控制参数主菜单
        while (menu_next_flag == 0)
        {
            Menu_Circle_Show(1);
            Keystroke_Scan();
            Cursor();
        }
        Menu_Next_Back();
        break;

    case 51: // 圆环编码器阈值
        Menu_Circle_Show(1 * 18);
        Keystroke_float(&ring_encoder, 1);
        break;
    case 52: // 入环前陀螺仪设定值
        Menu_Circle_Show(2 * 18);
        Keystroke_float(&pre_ring_Gyro_set, 10);
        break;
    case 53: // 入环陀螺仪Z轴值
        Menu_Circle_Show(3 * 18);
        Keystroke_float(&in_ring_Gyroz, 10);
        break;
    case 54: // 出环前陀螺仪设定值
        Menu_Circle_Show(4 * 18);
        Keystroke_float(&pre_out_ring_Gyro_set, 10);
        break;
    case 55: // 出环前陀螺仪Z轴值
        Menu_Circle_Show(5 * 18);
        Keystroke_float(&pre_out_ring_Gyroz, 10);
        break;
    case 56: // 出环前编码器阈值
        Menu_Circle_Show(6 * 18);
        Keystroke_float(&pre_out_ring_encoder, 1);
        break;
    }
}