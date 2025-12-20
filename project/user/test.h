#ifndef _TEST_H_
#define _TEST_H_

/**
 * @brief 机器周期时长（秒）
 * @details 8051 标准内核 12T 模式，周期 = 12 / 系统时钟
 */
#define MACHINE_CYCLE (12.0 / SYSTEM_CLOCK_40M)

/** @brief 是否使能通信（调试时使能，正式时关闭以减少CPU开销） */
#define ENABLECOMM (1)

/**
 * @brief 错误码联合体
 * @details 提供位域细分的错误标志
 */
typedef union
{
    u32 eCodeError;
    struct
    {
        u8 NO_ERROR : 1;
        u8 GetTimeError : 1;
        u8 GetTimeEndError : 1;
        // ......后续补充其他的错误
    } ErrorDetail;
} ErrorUnion_U;

/**
 * @brief 生成速度阶跃并驱动左右轮速度环
 * @details 验证控制链路与电机输出
 * @return 无
 */
extern float test_speed;
void test(void);

/**
 * @brief 读取 Timer0 当前16位计数值
 * @return 当前计数（ticks）
 */
u32 GetTimeStart(void);
/**
 * @brief 计算相对 gStart_us 的增量 ticks，含溢出累计
 * @return 增量 ticks
 */
u32 GetTimeEnd(void);
/**
 * @brief 将最近一次增量 ticks 转换为秒返回
 * @return 秒
 */
f32 GetTotalTime(void);

/**
 * @brief 清溢出计数并记录当前起点 ticks 到 gStart_us
 * @return 无
 */
void TimingStart(void);
/**
 * @brief 结束计时并返回增量 ticks，同时更新 gEnd_us
 * @return 增量 ticks
 */
u32 TimingStopTicks(void);
/**
 * @brief 结束计时并以秒为单位返回耗时，同时更新 gEnd_us
 * @return 秒
 */
f32 TimingStopSeconds(void);
/**
 * @brief 局部计时窗口结构体
 * @details 保存起点 Timer0 计数与溢出计数快照，用于任务内部耗时测量
 */
typedef struct
{
    u32 start_ticks;
    u8 spill_start;
} LocalTimingWindow;
/**
 * @brief 启动局部计时
 * @details 记录当前溢出计数与 Timer0 计数为局部起点，不修改全局基准
 * @param w 局部计时窗口指针（必须非空）
 */
void TimingStartLocal(LocalTimingWindow *w);
/**
 * @brief 停止局部计时并返回增量 ticks
 * @details 通过溢出计数差与计数差计算跨越 65536 的增量
 * @param w 局部计时窗口指针（必须非空）
 * @return 增量 ticks（机器周期数）
 */
u32 TimingStopTicksLocal(LocalTimingWindow *w);
/**
 * @brief 停止局部计时并返回耗时（秒）
 * @details 将增量 ticks 乘以 MACHINE_CYCLE 转换为秒
 * @param w 局部计时窗口指针（必须非空）
 * @return 耗时（秒）
 */
f32 TimingStopSecondsLocal(LocalTimingWindow *w);
#endif
