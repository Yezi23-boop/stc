#include "pid.h"
PID_Controllers PID; // 聚合控制器：左右速度环 + 转向环
LowPassFilter_t encoder_l;
LowPassFilter_t encoder_r;
float delta_output = 0, speed_l = 0, speed_r = 0, max_integral = 0;
// 速度环初始化（增量式），命名简洁
void pid_speed_init(PID_Speed *pid, float kp, float ki, float kd, float max_out, float min_out)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->error = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev2_error = 0.0f;
    pid->output = 0.0f;
    pid->max_output = max_out;
    pid->min_output = min_out;
}
// 转向环初始化（位置式），命名简洁
void pid_steer_init(PID_Steer *pid, float kp, float kd, float kp2, float max_out, float min_out)
{
    pid->Kp = kp;
    pid->Kd = kd;
    pid->kp2 = kp2;
    pid->error = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
    pid->max_output = max_out;
    pid->min_output = min_out;
}
// 编码器读取
// 读取编码器并更新到 PID 结构的 speed 字段（带低通滤波）
void Encoder_get(PID_Speed *left, PID_Speed *right)
{

    // 将编码器计数转换为速度（系数需按采样周期与脉冲当量校准）
    left->speed = encoder_get_count(TIM4_ENCOEDER) * 0.2f;   /* 左轮 */
    right->speed = -encoder_get_count(TIM3_ENCOEDER) * 0.2f; /* 右轮，方向取负 */

    // 一阶低通滤波（就地写回）
    low_pass_filter_mt(&encoder_l, &left->speed, 0.9f);
    low_pass_filter_mt(&encoder_r, &right->speed, 0.7f);

    // 清零计数，准备下一周期
    encoder_clear_count(TIM3_ENCOEDER);
    encoder_clear_count(TIM4_ENCOEDER);
}
// 速度环更新（增量式）
void pid_speed_update(PID_Speed *pid, float target, float actual)
{
    // 计算当前误差（直接使用结构体成员）
    pid->error = target - actual;

    // 增量式 PID_Direction：使用结构体成员计算差分与输出增量
    delta_output = pid->Kp * (pid->error - pid->prev_error) + pid->Ki * pid->error + pid->Kd * (pid->error - 2.0f * pid->prev_error + pid->prev2_error);

    // 更新输出并限幅
    pid->output += delta_output;
    if (pid->output > pid->max_output)
    {
        pid->output = pid->max_output;
    }
    else if (pid->output < -pid->max_output)
    {
        pid->output = -pid->max_output;
    }

    // 更新误差历史
    pid->prev2_error = pid->prev_error;
    pid->prev_error = pid->error;
}

// 转向环更新（位置式）
void pid_steer_update(PID_Steer *pid, float error)
{
    // 计算当前误差（直接写入结构体成员）
    pid->error = error;

    // 位置式 PID_Direction：limiting_Err 以陀螺输入项参与
    pid->output = pid->Kp * pid->error + pid->kp2 * error * func_abs(error) + pid->Kd * (pid->error - pid->prev_error);

    // 输出限幅
    if (pid->output > pid->max_output)
    {
        pid->output = pid->max_output;
    }
    else if (pid->output < -pid->min_output)
    {
        pid->output = -pid->min_output;
    }

    // 更新误差历史
    pid->prev_error = pid->error;
}

// 角度环更新（位置式）
void pid_angle_update(PID_Steer *pid, float error, float gyro)
{
    // 计算当前误差（直接写入结构体成员）
    pid->error = error - gyro;

    // 位置式 PID_Direction：limiting_Err 以陀螺输入项参与
    pid->output = pid->Kp * pid->error + pid->Kd * (pid->error - pid->prev_error);

    // 输出限幅
    if (pid->output > pid->max_output)
    {
        pid->output = pid->max_output;
    }
    else if (pid->output < -pid->min_output)
    {
        pid->output = -pid->min_output;
    }

    // 更新误差历史
    pid->prev_error = pid->error;
}
void Pid_Differential(float speed_ref, float delta, float *left_target, float *right_target, float Scope)
{
    float k;

    // 防止除零错误
    if (Scope < 0.001f)
        Scope = 100.0f;

    if (delta >= 0.0f) // 需要向左修正（原逻辑：左轮减速，右轮加速）
    {
        // 根据 Scope 进行归一化：将 delta 映射到 0.0~1.0
        // 如果 Scope 代表最大期望 delta，则 k = delta / Scope
        k = delta / Scope;

        if (k > 1.0f)
            k = 1.0f; // 限幅防止反转

        *left_target = speed_ref * (1.0f - k);

        *right_target = speed_ref * (1.0f + k * 0.5f);
    }
    else
    {
        // k = -ele_Out_1 / Scope; 取相反数并缩放
        k = -delta / Scope;

        if (k > 1.0f)
            k = 1.0f;

        *left_target = speed_ref * (1.0f + k * 0.5f);
        *right_target = speed_ref * (1.0f - k);
    }
}

#define TRACK_WIDTH 80.0f // 轮距 80cm
#define BASE_L 150.0f     // 基础预瞄距离 100cm (可调)
#define K_SPEED 0.5f      // 速度预瞄增益 (可调)

/*****************************************************************************
 * 纯追踪 + 陀螺仪闭环混合控制 (Pure Pursuit + Gyro Loop)
 * 核心思想：
 *   1. Pure Pursuit 规划出“理论目标角速度”。
 *   2. PID 内环根据“陀螺仪实测角速度”去追踪这个目标。
 *   3. 最终输出电机差速。
 * 输入参数：
 *   speed_ref: 基础速度
 *   norm_error: 归一化误差
 *   gyro_z: 当前 Z 轴角速度 (单位需换算统一，建议 rad/s 或度/s)
 *   left_target, right_target: 输出指针
 *****************************************************************************/
void Pure_Pursuit_Gyro_Control(float speed_ref, float norm_error, float gyro_z, float *left_target, float *right_target)
{
    float look_ahead_L;
    float curvature;
    float target_omega; // 目标角速度

    // =========================================================
    // 第一层：路径规划 (Pure Pursuit Path Planning)
    // =========================================================
    // 1. 动态预瞄：速度越快，预瞄距离越长 (BASE_L + K * Speed)
    //    作用：低速灵敏切弯，高速平滑轨迹防止震荡
    look_ahead_L = BASE_L + K_SPEED * speed_ref;

    // 2. 几何解算：根据电感误差算出切回中线所需的圆弧曲率
    //    公式：kappa = 2 * sin(alpha) / L
    curvature = (2.0f * norm_error) / look_ahead_L;

    // 3. 曲率限幅：防止分母过小导致数值爆炸
    if (curvature > 0.1f)
        curvature = 0.1f;
    if (curvature < -0.1f)
        curvature = -0.1f;

    // =========================================================
    // 第二层：姿态控制 (Attitude Control / Gyro Loop)
    // =========================================================
    // 4. 将几何曲率转化为物理目标角速度 (前馈项)
    //    公式：Omega = V * kappa
    //    单位换算：乘以 57.3 将 rad/s 转换为 degree/s (假设 gyro_z 为度/秒)
    target_omega = speed_ref * curvature * 57.3f;

    //  5. 并联 Err PID 补偿 (反馈项)
    // 作用：Pure Pursuit 负责大局路径，PID 负责处理突变和稳态误差
    // 特别是 D 项 (Kd_COMP)，对直角弯的瞬时响应至关重要
    // 复用 PID.steer (原方向环) 作为补偿控制器
    // 注意：需要在初始化时设置 PID.steer 的 Kp 和 Kd 参数
    pid_steer_update(&PID.steer, norm_error);

    // 叠加到目标角速度
    target_omega += PID.steer.output;

    // 6. 陀螺仪闭环控制 (PID)
    //    误差 = 理论目标角速度 - 陀螺仪实测角速度
    //    作用：如果车轮打滑导致车头没转过来，PID 会输出修正量强制车身转动
    pid_angle_update(&PID.angle, target_omega, gyro_z);

    // =========================================================
    // 第三层：电机执行 (Actuation / Differential Drive)
    // =========================================================
    // 6. 使用 Pid_Differential 进行非线性差速分配
    //    输入：基础速度 + 角度环 PID 输出的修正量
    //    策略：不对称加减速 (内轮减速多，外轮加速少)，稳住重心
    Pid_Differential(speed_ref, PID.angle.output, left_target, right_target, 500.0f);
}