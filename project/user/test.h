#ifndef _TEST_H_
#define _TEST_H_

// 机器周期 = 12/系统时钟 (8051 标准内核)，单位：秒
#define MACHINE_CYCLE (12.0 / SYSTEM_CLOCK_40M)  
#define P40  KEY_START_MOTOR
/*********************调试时使能，正式时关闭减少CPU运行*********************/
#define ENABLECOMM (1)      // 是否使能通信

// ==================== 状态定义 ====================
// 状态定义 - 只保留核心状态
typedef enum {
    STATE_IDLE = 0,        // 空闲
    STATE_LINE_FOLLOW,     // 循迹
    STATE_TURN,            // 转弯
    STATE_CROSS,           // 交叉口
    STATE_yaw,             // 偏航
    STATE_ERROR            // 错误
} CarState_t;
    
// 事件定义 - 只保留关键事件
typedef enum {
    EV_START = 0,          // 开始
    EV_STOP,               // 停止
    EV_LINE_LOST,          // 丢线
    EV_LINE_FOUND,         // 找到线
    EV_CROSS_DETECT,       // 检测到交叉口
    EV_TURN_DONE,          // 转弯完成
    EV_ERROR,              // 错误
    EV_RECOVER             // 恢复
} CarEvent_t;


// 状态机结构
typedef struct {
    CarState_t current_state;   // 当前状态
    CarState_t last_state;      // 上次状态
    uint32_t state_start_time;  // 状态开始时间
    u16 timeout;     // 状态超时时间(ms)
} StateMachine_t;

// 状态转移表
static const struct {
    CarState_t from_state;
    CarEvent_t event;
    CarState_t to_state;
    uint32_t timeout;  // 新状态的超时时间(ms)，0表示无超时
} state_transitions[] = {
    // 从空闲状态
    {STATE_IDLE, EV_START, STATE_LINE_FOLLOW, 0},
    {STATE_IDLE, EV_ERROR, STATE_ERROR, 0},
    
    // 从循迹状态
    {STATE_LINE_FOLLOW, EV_STOP, STATE_IDLE, 0},
    {STATE_LINE_FOLLOW, EV_LINE_LOST, STATE_yaw, 2000},  // 丢线2秒后超时
    {STATE_LINE_FOLLOW, EV_CROSS_DETECT, STATE_CROSS, 5000}, // 交叉口5秒超时
    {STATE_LINE_FOLLOW, EV_ERROR, STATE_ERROR, 0},
    
    // 从转弯状态
    {STATE_TURN, EV_TURN_DONE, STATE_LINE_FOLLOW, 0},
    {STATE_TURN, EV_ERROR, STATE_ERROR, 0},
    
    // 从交叉口状态
    {STATE_CROSS, EV_START, STATE_TURN, 3000},  // 转弯3秒超时
    {STATE_CROSS, EV_ERROR, STATE_ERROR, 0},
    
    // 从停止状态
    {STATE_yaw, EV_LINE_FOUND, STATE_LINE_FOLLOW, 0},
    {STATE_yaw, EV_ERROR, STATE_ERROR, 0},
    
    // 从错误状态
    {STATE_ERROR, EV_RECOVER, STATE_IDLE, 0},
};


extern float test_speed;
void test(void);
// 状态机函数
void state_machine_init(void);
void state_machine_process_event(CarEvent_t event);
void state_machine_update(void);
CarState_t state_machine_get_state(void);


u32 GetTimeStart(void);
u32 GetTimeEnd(void);
f32 GetTotalTime(void);


void TimingStart(void);
u32 TimingStopTicks(void);
f32 TimingStopSeconds(void);
#endif
