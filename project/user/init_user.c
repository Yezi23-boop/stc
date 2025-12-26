#include "zf_common_headfile.h"
#define TIME_0 500 // 定时器0中断周期(ms)
#define TIME_1 500 // 定时器1中断周期(ms)
// 定时器T0初始化函数
static void timer0_init(void)
{
    // 不可屏蔽中断16位自动重装载
    TMOD &= 0xF0; // 清T0,retain T1
    TMOD |= 0x01;
    T0x12 = 0;
    TL0 = 0xFB; // 配置成9999us≈1MS
    TH0 = 0xF2;
    // 开 Timer0 中断
    ET0 = 1;
    TCON |= 0x10;
}
// 用户的初始化
void init_user(void)
{
    TaskDescriptor_t *task_sensor;
    TaskDescriptor_t *task_control;
    TaskDescriptor_t *task_state;
    TaskDescriptor_t *task_monitor;
    TaskDescriptor_t *task_comm;
    TaskDescriptor_t *task_log;
    // 系统初始化：传感器/存储/定时器/编码器/ADC/电机/无线
    imu660ra_init(); // 初始化 IMU660RA
    eeprom_init();
    pit_us_init(TIM1_PIT, TIME_1);
    g_encoder_driver.init(TIM3_ENCOEDER, IO_P46, TIM3_ENCOEDER_P04);
    g_encoder_driver.init(TIM4_ENCOEDER, IO_P42, TIM4_ENCOEDER_P06);
    g_ADC_Driver.init(ADC_CH13_P05, ADC_8BIT);
    g_ADC_Driver.init(ADC_CH0_P10, ADC_12BIT);
    g_ADC_Driver.init(ADC_CH1_P11, ADC_12BIT);
    g_ADC_Driver.init(ADC_CH8_P00, ADC_12BIT);
    g_ADC_Driver.init(ADC_CH9_P01, ADC_12BIT);
    g_motor_driver.init(&MotorInit);
    SuctionPressure_Init(); // 负压系统初始化
    wireless_uart_init();   // 无线串口初始化
    // 速度环 PID 初始化（误差限幅与输出限幅）
    pid_speed_init(&PID.left_speed, 100, 40, 0, 8000, 8000);
    pid_speed_init(&PID.right_speed, 100, 40, 0, 8000, 8000);
    // 方向环 PID 初始化（误差KP/KD与陀螺KD分离）
    pid_steer_init(&PID.steer, kp_Err, kd_Err, kd_gyro, 45, 45); // 方向环
    ips114_init();
    gpio_init(IO_P33, GPO, 1, GPO_PUSH_PULL);
    gpio_init(IO_P34, GPO, 1, GPO_PUSH_PULL);
    state_machine_init();
    // 5. 初始化任务调度器
    g_task_scheduler.init();
    g_task_scheduler.set_idle_callback(system_idle_callback);
    // 6. 创建并添加所有任务
    // 传感器更新：1000Hz（最高优先级）
    task_sensor = CREATE_TASK(
        task_sensor_update, NULL, PRIORITY_NORMAL, 1, "Sensor Update");
    g_task_scheduler.add_task(task_sensor);

    // 控制算法：200Hz
    task_control = CREATE_TASK(
        task_control_algorithm, NULL, PRIORITY_HIGH, 5, "Control Algorithm");
    g_task_scheduler.add_task(task_control);

    // 状态机处理：200Hz
    task_state = CREATE_TASK(
        task_state_machine, NULL, PRIORITY_HIGH, 5, "State Machine");
    g_task_scheduler.add_task(task_state);

    // 系统监控：2Hz
    task_monitor = CREATE_TASK(
        task_system_monitor, NULL, PRIORITY_LOW, 500, "System Monitor");
    g_task_scheduler.add_task(task_monitor);

    // 通信处理：10Hz
    task_comm = CREATE_TASK(
        task_communication, NULL, PRIORITY_NORMAL, 100, "Communication");
    g_task_scheduler.add_task(task_comm);

    // 数据记录：1Hz
    task_log = CREATE_TASK(
        task_data_logging, NULL, PRIORITY_IDLE, 1000, "Data Logging");
    g_task_scheduler.add_task(task_log);
    
    // 启动调度器
    g_task_scheduler.start();
    timer0_init();
}

/*********************************************
 * 传感器零偏标定
 * 函数: offset_init()
 * 作用: 采样多次取平均，得到陀螺仪与加速度零偏
 * 日期: 2024/11/26
 * 备注: 调用前确保 IMU 已上电稳定
 *********************************************/
float Gyrox = 0;
float Gyroy = 0;
float Gyroz = 0;
float acc_x = 0;
float acc_y = 0;
float acc_z = 0;
float Gyro_offset_x = 0;
float Gyro_offset_y = 0;
float Gyro_offset_z = 0;
float acc_offset_x = 0;
float acc_offset_y = 0;
float acc_offset_z = 0;
void offset_init(void)
{
    int rt = 50;
    int i;
    imu660ra_init();     // 初始化 IMU
    system_delay_init(); // 初始化延时模块
    for (i = 0; i < rt; i++)
    {
        imu660ra_get_gyro();
        imu660ra_get_acc();
        Gyro_offset_x += imu660ra_gyro_transition(imu660ra_gyro_x);
        Gyro_offset_y += imu660ra_gyro_transition(imu660ra_gyro_y);
        Gyro_offset_z += imu660ra_gyro_transition(imu660ra_gyro_z);
        acc_offset_x += (imu660ra_acc_transition(imu660ra_acc_x));
        acc_offset_y += (imu660ra_acc_transition(imu660ra_acc_y));
        acc_offset_z += (imu660ra_acc_transition(imu660ra_acc_z));
        system_delay_ms(5);
    }
    Gyro_offset_x = Gyro_offset_x / rt;
    Gyro_offset_y = Gyro_offset_y / rt;
    Gyro_offset_z = Gyro_offset_z / rt;
    acc_offset_x = acc_offset_x / rt;
    acc_offset_y = acc_offset_y / rt;
    acc_offset_z = acc_offset_z / rt;
}
