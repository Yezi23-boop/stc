#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "zf_common_headfile.h"
// 常用任务创建宏
#define CREATE_TASK(func, ctx, prio, period, name) \
    g_task_scheduler.create_task(func, ctx, prio, period, name)
// 任务优先级（数值越小优先级越高）
typedef enum
{
    PRIORITY_CRITICAL = 0, // 关键任务（故障处理等）
    PRIORITY_HIGH = 1,     // 高优先级（控制算法等）
    PRIORITY_NORMAL = 2,   // 普通优先级（传感器读取等）
    PRIORITY_LOW = 3,      // 低优先级（日志记录等）
    PRIORITY_IDLE = 4      // 空闲任务
} TaskPriority_t;

// 任务状态
typedef enum
{
    TASK_READY,     // 就绪
    TASK_RUNNING,   // 运行中
    TASK_WAITING,   // 等待中
    TASK_SUSPENDED, // 挂起
    TASK_COMPLETED  // 完成
} TaskState_t;

// 任务函数指针类型
typedef void (*TaskFunction_t)(void *context);

// 任务描述符
typedef struct
{
    TaskFunction_t function; // 任务函数
    void *context;           // 任务上下文
    TaskPriority_t priority; // 任务优先级
    TaskState_t state;       // 任务状态

    u32 period_ms;     // 执行周期（0表示单次任务）
    u32 last_run_time; // 上次运行时间
    u32 deadline_ms;   // 截止时间（相对period）

    const char *name; // 任务名称（调试用）
    u32 run_count;    // 运行次数统计
    u32 max_run_time; // 最长运行时间（用于性能监控）
} TaskDescriptor_t;

// 调度器统计
typedef struct
{
    u32 total_tasks;      // 总任务数
    u32 executed_tasks;   // 已执行任务数
    u32 missed_deadlines; // 错过截止时间的任务数
    u32 cpu_usage;        // CPU使用率（百分比）
    u32 idle_time;        // 空闲时间
} SchedulerStats_t;

// 调度器接口
typedef struct
{
    // 初始化
    void (*init)(void);

    // 任务管理
    TaskDescriptor_t *(*create_task)(TaskFunction_t func, void *ctx,
                                     TaskPriority_t prio, u32 period_ms,
                                     const char *name)reentrant;
    u8 (*add_task)(TaskDescriptor_t *task);
    u8 (*remove_task)(TaskDescriptor_t *task);
    u8 (*suspend_task)(TaskDescriptor_t *task);
    u8 (*resume_task)(TaskDescriptor_t *task);

    // 调度器控制
    void (*start)(void);
    void (*stop)(void);
    void (*run)(void); // 在主循环中调用

    // 状态获取
    SchedulerStats_t (*get_stats)(void);
    u32 (*get_tick_count)(void);

    // 系统空闲回调（可选）
    void (*set_idle_callback)(void (*callback)(void));
} TaskScheduler_t;

// 电机配置结构
typedef struct
{
    pwm_channel_enum pwm_channel_left;  // 左电机PWM通道
    gpio_pin_enum dir_pin_left;         // 左电机方向引脚
    pwm_channel_enum pwm_channel_right; // 右电机PWM通道
    gpio_pin_enum dir_pin_right;        // 右电机方向引脚
    uint16_t pwm_frequency;             // PWM频率(Hz)

    uint16_t deadband_left;   // 死区PWM值
    uint16_t deadband_right;  // 死区PWM值
    uint16_t frequency_left;  // 左PWM频率
    uint16_t frequency_right; // 右PWM频率

} MotorConfig_t;

// 电机ID定义
typedef enum
{
    MOTOR_LEFT = 0,
    MOTOR_RIGHT = 1,
    MOTOR_ALL = 2
} MotorID_e;

// 电机方向
typedef enum
{
    MOTOR_DIR_FORWARD = 0,
    MOTOR_DIR_BACKWARD = 1,
    MOTOR_DIR_BRAKE = 2,
    MOTOR_DIR_COAST = 3
} MotorDirection;

// 编码器驱动接口
typedef struct
{
    // 初始化
    void (*init)(encoder_index_enum encoder_n, gpio_pin_enum dir_pin, encoder_channel_enum lsb_pin);

    // 数据读取
    int16 (*read)(encoder_index_enum encoder_n);
    void (*clear)(encoder_index_enum encoder_n);
    // float (*read_velocity)(EncoderID encoder);  // RPM

    // 校准
    // void (*calibrate)(EncoderID encoder, uint32_t calibration_time_ms);

    void *private_data;
} EncoderDriver;

// 电机驱动接口结构
typedef struct
{
    // 初始化
    void (*init)(MotorConfig_t *MotorConfig);
    // 控制函数
    void (*set_pwm)(int32 lpwm_value, int32 rpwm_value);
    // 状态获取
    // 高级功能
    // void (*enable_brake)(bool enable);
    // void (*set_deadband)(uint16_t deadband);  // 设置死区
    // // 私有数据
    // void* private_data;
} MotorDriver;

// 全局电机驱动实例
extern MotorDriver g_motor_driver;
extern EncoderDriver g_encoder_driver;
extern const MotorConfig_t MotorInit;
extern int flat_statr;

// 全局调度器实例
extern TaskScheduler_t g_task_scheduler;
void scheduler_on_tick(void);

void task_sensor_update(void *context);
void task_control_algorithm(void *context);
void task_state_machine(void *context);
void task_system_monitor(void *context);
void task_communication(void *context);
void task_data_logging(void *context);
void hal_motor_Init(MotorConfig_t *MotorConfig);
void system_idle_callback(void);
void motor_output(int32 lpwm, int32 rpwm);
void lost_lines(void);
void state_turn(void);
extern int8 lost_spto;
#endif
