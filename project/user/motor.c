#include "motor.h"

int8 lost_spto = 0;
int flat_statr = 0;
float task1ms_time_s = 0;
// ==================== 私有数据 ====================

#define MAX_TASKS 16          // 最大任务数（可根据需要调整）
#define TICK_FREQUENCY 1000   // 调度器滴答频率（Hz）
MotorConfig_t const MotorInit = {
	.pwm_channel_left = PWMB_CH2_P13,
	.dir_pin_left     = IO_P14,
	.pwm_channel_right = PWMB_CH3_P52,
	.dir_pin_right    = IO_P53,
	.deadband_left    = 40,
	.deadband_right   = 40,
	.frequency_left   = 17000,
	.frequency_right  = 17000
};
MotorDriver g_motor_driver = {
	.init = hal_motor_Init,
	.set_pwm = motor_output
};
EncoderDriver g_encoder_driver = {
	.init = encoder_dir_init,
	.read = encoder_get_count,
	.clear = encoder_clear_count
};
static struct {
    TaskDescriptor_t* tasks[MAX_TASKS];  // 任务指针数组
    uint8_t task_count;                  // 当前任务数
    
    SchedulerStats_t stats;              // 统计信息
    uint32_t system_tick;                // 系统滴答计数
    uint32_t last_stats_time;            // 上次统计时间
    
    bool is_running;                     // 调度器运行状态
    void (*idle_callback)(void);         // 空闲回调函数
    
    // 性能监控
    uint32_t last_idle_check;
    uint32_t idle_counter;
} scheduler_private = {0};


// ==================== 私有函数 ====================

// 获取当前时间（毫秒）,最大的时间49.71 天
static uint32_t get_current_time(void)
{
    return scheduler_private.system_tick;
}
// 查找就绪的最高优先级任务
static TaskDescriptor_t* find_highest_priority_ready_task(void)
{
    TaskDescriptor_t* highest_prio_task = NULL;
    
    for (uint8_t i = 0; i < scheduler_private.task_count; i++) {
        TaskDescriptor_t* task = scheduler_private.tasks[i];
        
        if (task->state == TASK_READY) {
            // 检查是否到达执行时间
            uint32_t current_time = get_current_time();
            uint32_t time_since_last_run = current_time - task->last_run_time;
            
            if (time_since_last_run >= task->period_ms) {
                // 找到优先级更高的任务
                if (highest_prio_task == NULL || 
                    task->priority < highest_prio_task->priority) {
                    highest_prio_task = task;
                }
            }
        }
    }
    
    return highest_prio_task;
}

// 检查任务截止时间
static void check_task_deadlines(void)
{
    uint32_t current_time = get_current_time();
    
    for (uint8_t i = 0; i < scheduler_private.task_count; i++) {
        TaskDescriptor_t* task = scheduler_private.tasks[i];
        
        if (task->state == TASK_RUNNING) {
            uint32_t run_time = current_time - task->last_run_time;
            
            if (run_time > task->deadline_ms) {
                scheduler_private.stats.missed_deadlines++;
                
                // 可以在这里添加超时处理逻辑
                // // printf("Task %s missed deadline! (%lu > %lu ms)\n",
                //        task->name, run_time, task->deadline_ms);
            }
        }
    }
}

// 更新调度器统计信息
static void update_scheduler_stats(void)
{
    uint32_t current_time = get_current_time();
    
    // 每5秒更新一次统计
    if (current_time - scheduler_private.last_stats_time >= 5000) {
        // 计算CPU使用率（简单估算）
        if (scheduler_private.idle_counter > 0) {
            // 这里需要根据实际情况计算
            scheduler_private.stats.cpu_usage = 80;  // 示例值
        }
        
        scheduler_private.last_stats_time = current_time;
        scheduler_private.idle_counter = 0;
    }
}

// ==================== 公开API实现 ====================

// 初始化调度器
static void scheduler_init(void)
{
    // printf("Initializing task scheduler...\n");
    
    memset(&scheduler_private, 0, sizeof(scheduler_private));
    scheduler_private.system_tick = 0;
    scheduler_private.last_stats_time = get_current_time();
    scheduler_private.is_running = false;
    scheduler_private.idle_callback = scheduler_set_idle_callback;
    
    // // printf("Task scheduler initialized (max tasks: %d)\n", MAX_TASKS);
}

// 创建新任务
static TaskDescriptor_t* scheduler_create_task(TaskFunction_t func, void* ctx,
                                              TaskPriority_t prio, uint32_t period_ms,
                                              const char* name)
{
    if (scheduler_private.task_count >= MAX_TASKS) {
        // printf("Error: Cannot create task, maximum tasks reached\n");
        return NULL;
    }
    
    if (func == NULL) {
        // printf("Error: Task function cannot be NULL\n");
        return NULL;
    }
    
    // 分配任务描述符（实际项目中可能需要内存池）
    static TaskDescriptor_t task_pool[MAX_TASKS];
    static uint8_t pool_index = 0;
    
    if (pool_index >= MAX_TASKS) {
        // printf("Error: Task pool exhausted\n");
        return NULL;
    }
    
    TaskDescriptor_t* task = &task_pool[pool_index++];
    
    // 初始化任务
    task->function = func;
    task->context = ctx;
    task->priority = prio;
    task->state = TASK_READY;
    task->period_ms = period_ms;
    task->last_run_time = 0;
    task->deadline_ms = period_ms;  // 默认截止时间等于周期
    task->name = name;
    task->run_count = 0;
    task->max_run_time = 0;
    
    // printf("Task created: %s (priority: %d, period: %lu ms)\n",
    //        name, prio, period_ms);
    
    return task;
}
// 添加任务到调度器
static bool scheduler_add_task(TaskDescriptor_t* task)
{
    if (task == NULL) {
        // printf("Error: Cannot add NULL task\n");
        return false;
    }
    
    if (scheduler_private.task_count >= MAX_TASKS) {
        // printf("Error: Cannot add task, scheduler is full\n");
        return false;
    }
    
    // 添加到任务列表
    scheduler_private.tasks[scheduler_private.task_count++] = task;
    scheduler_private.stats.total_tasks++;
    
    // printf("Task added to scheduler: %s\n", task->name);
    return true;
}

// 启动调度器
static void scheduler_start(void)
{
    if (scheduler_private.is_running) {
        // printf("Scheduler is already running\n");
        return;
    }
    
    scheduler_private.is_running = true;
    scheduler_private.system_tick = 0;
    
    // printf("Task scheduler started\n");
}

// 停止调度器
static void scheduler_stop(void)
{
    scheduler_private.is_running = false;
    // printf("Task scheduler stopped\n");
}

// 获取调度器统计信息
static SchedulerStats_t scheduler_get_stats(void)
{
    return scheduler_private.stats;
}

// 获取系统滴答计数
static uint32_t scheduler_get_tick_count(void)
{
    return scheduler_private.system_tick;
}

// 设置空闲回调
static void scheduler_set_idle_callback(void (*callback)(void))
{
    scheduler_private.idle_callback = callback;
}

// 运行调度器（在主循环中调用）
static void scheduler_run(void)
{
    if (!scheduler_private.is_running) {
        return;
    }
    
    // 更新时间滴答（应该由硬件定时器中断更新）
    
    // 1. 查找并执行就绪的最高优先级任务
    TaskDescriptor_t* task_to_run = find_highest_priority_ready_task();
    
    if (task_to_run != NULL) {
        // 记录开始时间（用于性能监控）
        uint32_t start_time = get_current_time();
        
        // 更新任务状态
        task_to_run->state = TASK_RUNNING;
        task_to_run->last_run_time = start_time;
        
        // 执行任务
        task_to_run->function(task_to_run->context);
        
        // 更新任务统计
        task_to_run->run_count++;
        
        // 计算执行时间
        uint32_t end_time = get_current_time();
        uint32_t execution_time = end_time - start_time;
        
        if (execution_time > task_to_run->max_run_time) {
            task_to_run->max_run_time = execution_time;
        }
        
        // 恢复任务状态
        if (task_to_run->period_ms > 0) {
            task_to_run->state = TASK_READY;  // 周期性任务，准备下次执行
        } else {
            task_to_run->state = TASK_COMPLETED;  // 单次任务，完成
        }
        
        scheduler_private.stats.executed_tasks++;
        
        // 调试输出（可选）
        // if (execution_time > task_to_run->deadline_ms) {
        //     printf("Warning: Task %s took %lu ms (deadline: %lu ms)\n",
        //            task_to_run->name, execution_time, task_to_run->deadline_ms);
        // }
    } else {
        // 没有任务需要执行，进入空闲状态
        scheduler_private.idle_counter++;
        
        // 调用空闲回调（可用于低功耗模式）
        if (scheduler_private.idle_callback != NULL) {
            scheduler_private.idle_callback();
        }
    }
    
    // 2. 检查任务截止时间
    check_task_deadlines();
    
    // 3. 更新统计信息
    update_scheduler_stats();
}

TaskScheduler_t g_task_scheduler = {
    .init = scheduler_init,
    .create_task = scheduler_create_task,
    .add_task = scheduler_add_task,
    .remove_task = NULL,  // 可选实现
    .suspend_task = NULL, // 可选实现
    .resume_task = NULL,  // 可选实现
    .start = scheduler_start,
    .stop = scheduler_stop,
    .run = scheduler_run,
    .get_stats = scheduler_get_stats,
    .get_tick_count = scheduler_get_tick_count,
    .set_idle_callback = scheduler_set_idle_callback
};
// ==================== 任务1：传感器更新任务 ====================
void task_sensor_update(void* context)
{
    (void)context;  // 未使用参数
    
    // 更新所有传感器数据
    // g_sensor_service.update();
    // 1. 更新传感器数据
    read_AD();
    Prepare_Data();
    lost_lines();
	Encoder_get(&PID.left_speed, &PID.right_speed);
    // 生成传感器事件（触发状态机）
    // const SensorData_t* data = g_sensor_service.get_data();
    
    // if (data->line_tracking.line_lost) {
    //     // 触发丢线事件
    //     state_machine_process_event(EV_LINE_LOST);
    // }
    
    // if (data->line_tracking.intersection) {
    //     // 触发交叉口事件
    //     state_machine_process_event(EV_CROSS_DETECT);
    // }
    
}

// ==================== 任务2：控制算法任务 ====================
void task_control_algorithm(void* context)
{
    static int flat_statr_date = 0;
	gpio_low(IO_P34);
	TimingStart();
	scan_track_max_value();
	IMUupdate(&Gyr_filt, &Acc_filt, &Att_Angle);
	voltage_adc();
	flat_statr_date++;
	if (P35 == 0 && flat_statr_date > 50)
	{
		flat_statr++;
		flat_statr_date = 0;
	}
	if (flat_statr >= 1 && lost_spto == 0 && start_flag == 1)
	{
		SuctionPressure_update_simple();
	}
	task1ms_time_s = TimingStopSeconds();
	gpio_high(IO_P34);
    (void)context;
}

// ==================== 任务3：状态机处理任务 ====================
void task_state_machine(void* context)
{
    (void)context;
    
    // 根据当前状态执行不同的控制策略
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
    
    // 调试输出当前状态
    static uint32_t last_print = 0;
    uint32_t current_time = scheduler_get_tick_count();
    
    if (current_time - last_print > 100) {  // 每100ms输出一次
        // printf("Current state: %s\n", state_machine_get_state_name());
        last_print = current_time;
    }
}

// ==================== 任务4：系统监控任务 ====================
void task_system_monitor(void* context)
{
    (void)context;
    if (!P32)  IAP_CONTR = 0x60;
    
    // 监控电池电压
    const SensorData_t* data = g_sensor_service.get_data();
    
    if (data->system.battery_voltage < 11.0f) {
        // printf("Warning: Battery low! %.1fV\n", data->system.battery_voltage);
    }
    
    // 监控CPU使用率
    SchedulerStats_t stats = g_task_scheduler.get_stats();
    
    static uint32_t last_report = 0;
    if (scheduler_get_tick_count() - last_report > 1000) {
        // printf("CPU Usage: %lu%%, Tasks: %lu/%lu\n",
            //    stats.cpu_usage,
            //    stats.executed_tasks,
            //    stats.total_tasks);
        last_report = scheduler_get_tick_count();
    }
}

// ==================== 任务5：通信任务 ====================
void task_communication(void* context)
{
    (void)context;
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
    ips114_show_float(3 * 24, 18 * 0, task1ms_time_s*1000, 4, 2);
#endif
    // 检查是否收到遥控命令
    // if (receive_remote_command()) {
    //     handle_remote_command();
    // }
    
    // 发送状态数据到上位机
    // send_status_to_pc();
}

// ==================== 任务6：数据记录任务 ====================
void task_data_logging(void* context)
{
    (void)context;
    
    // 记录运行数据（用于调试和分析）
    static FILE* log_file = NULL;
    
    if (log_file == NULL) {
        // 打开日志文件
        // log_file = fopen("car_log.csv", "a");
        // f// printf(log_file, "Time,State,Speed,Position\n");
    }
    
    const SensorData_t* data = g_sensor_service.get_data();
    
    // 记录数据
    // f// printf(log_file, "%lu,%s,%.1f,%.2f\n",
    //         scheduler_get_tick_count(),
    //         state_machine_get_state_name(),
    //         data->odometry.linear_speed_mmps,
    //         data->line_tracking.line_position);
}
// 系统空闲回调（用于低功耗）
void system_idle_callback(void)
{
    // 可以在这里进入低功耗模式
    // __WFI();  // 等待中断（ARM Cortex-M）
    
    // 或者简单的延时
    // delay_us(10);
}
void hal_motor_Init(MotorConfig_t* MotorConfig)
{
	pwm_init(MotorConfig->pwm_channel_left, MotorConfig->frequency_left, 0);		  // 左电机PWM输出初始化，频率17kHz
	gpio_init(MotorConfig->dir_pin_left, GPO, 1, GPO_PUSH_PULL); 					  // 左电机方向控制初始化
	pwm_init(MotorConfig->pwm_channel_right, MotorConfig->frequency_right, 0);		  // 右电机PWM输出初始化，频率17kHz
	gpio_init(MotorConfig->dir_pin_right, GPO, 1, GPO_PUSH_PULL); 					  // 右电机方向控制初始化
}

void motor_output(int32 lpwm, int32 rpwm)
{
	MotorConfig_t const* private = &MotorInit;
	// 应用死区
    if (lpwm > 0 && lpwm < private->deadband_left) lpwm = private->deadband_left;
    if (lpwm < 0 && lpwm > -private->deadband_left) lpwm = -private->deadband_left; 
    if (rpwm > 0 && rpwm < private->deadband_right) rpwm = private->deadband_right;
    if (rpwm < 0 && rpwm > -private->deadband_right) rpwm = -private->deadband_right;
    
	// 输出PWM
	if (lpwm > 0)
	{
		P14 = 1;
		pwm_set_duty(private->pwm_channel_left, lpwm);
	}
	else if (lpwm < 0)
	{
		P14 = 0;
		pwm_set_duty(private->pwm_channel_left, -lpwm);
	}
	if (rpwm > 0)
	{
		P53 = 0;
		pwm_set_duty(private->pwm_channel_right, rpwm);
	}
	else if (rpwm < 0)
	{
		P53 = 1;
		pwm_set_duty(private->pwm_channel_right, -rpwm);
	}
}

void lost_lines()
{
	// 中文说明：时间窗口内统计“丢线事件”次数，超过阈值则触发，否则清零
	// 设计目的：满足“一定时间内丢线次数>100即触发，否则清零”的需求
	// 约定：TM0 周期为 5ms，则 200 个周期≈1s，可按需调整窗口大小

	static int window_ticks = 0;		  // 时间窗口计数（周期数）
	static int loss_events = 0;			  // 时间窗口内的丢线事件计数
	const int LOST_WINDOW_TICKS = 200;	  // 时间窗口大小（周期数），默认约 1s
	const int LOST_THRESHOLD_EVENTS = 20; // 丢线事件阈值（窗口内超过该次数则触发）

	// 丢线事件判定：四路电感均低于阈值（一次满足记为一件事件）
	if (ad1 < 3 && ad2 < 3 && ad3 < 3 && ad4 < 3)
	{
		if (loss_events < 30000)
			loss_events++; // 防溢出保护
	}

	// 累计窗口时间
	if (window_ticks < 30000)
		window_ticks++; // 防溢出保护

	// 窗口结束时统一判定与复位（满足需求“否则就清零”）
	if (window_ticks >= LOST_WINDOW_TICKS)
	{
		if (loss_events > LOST_THRESHOLD_EVENTS)
		{
			// 触发丢线：进入安全模式（在 TM0 中 lost_spto==1 会屏蔽 PID 输出）
			lost_spto = 1;
			g_motor_driver.set_pwm(100, 100); // 低速直行，可按需调整占空比
		}
		else
		{
			// 未达到阈值：清零触发
			lost_spto = 0;
		}

		// 复位窗口计数与事件计数，开始下一窗口统计
		window_ticks = 0;
		loss_events = 0;
	}
}


void state_turn(void) {

}

