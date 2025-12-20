#include "zf_common_headfile.h"

int flat_statr = 0;
static int flat_statr_date = 0;
static int time_1;
void run_time_1(void)
{
    float left_target = 0.0f;
    float right_target = 0.0f;
    if (!P32)
    {
        IAP_CONTR = 0x60; // 判断快速烧录
    }
    read_AD(); // 读取并处理电感数据
    time_1++;
    Prepare_Data();
    Encoder_get(&PID.left_speed, &PID.right_speed);
    if (time_1 == 2)
    {
        pid_steer_update(&PID.steer, Err); // 更新转向环
        time_1 = 0;
    }
    pid_angle_update(&PID.angle, PID.steer.output, gyro_z * 0.082); // 更新角度环

    // 计算差速目标
    // Pid_Differential(speed_run, PID.angle.output, &left_target, &right_target, 500.0f);

    // pid_speed_update(&PID.left_speed, left_target, PID.left_speed.speed);    // 更新左轮速度环
    // pid_speed_update(&PID.right_speed, right_target, PID.right_speed.speed); // 更新右轮速度环
    pid_speed_update(&PID.left_speed, speed_run - PID.angle.output, PID.left_speed.speed);   // 更新左轮速度环
    pid_speed_update(&PID.right_speed, speed_run + PID.angle.output, PID.right_speed.speed); // 更新右轮速度环
    if (flat_statr >= 2)
    {
        motor_output((int)PID.left_speed.output, (int)PID.right_speed.output);
    }
    //      test_angle_func();
//test_speed_func();
//  	  motor_output(5000, 5000);
}

void run_time_2(void)
{
    scan_track_max_value(); // 获取电感最大值
    lost_lines();
    dianya_jiance();
    IMUupdate(&Gyr_filt, &Acc_filt, &Att_Angle);
    flat_statr_date++;
    if (P35 == 0 && flat_statr_date > 50)
    {
        flat_statr++;
        flat_statr_date = 0;
    }
    if (flat_statr >= 1 && start_flag == 1)
    {
        fuya_update_simple();
    }
}

void run_time_3(void)
{
    float delta = 0.0f;
    float left_target = 0.0f;
    float right_target = 0.0f;
    if (!P32)
    {
        IAP_CONTR = 0x60; // 判断快速烧录
    }
    read_AD(); // 读取并处理电感数据
    Prepare_Data();
    Encoder_get(&PID.left_speed, &PID.right_speed);

    // -------------------------------------------------------------------------
    // 控制算法：纯追踪 (Pure Pursuit) + 陀螺仪内环 (Gyro Loop) + 非线性差速
    // -------------------------------------------------------------------------

    // 3. 混合控制核心流程：
    //    Step 1: Pure Pursuit 根据 norm_err 和 speed_run 规划出“理论目标角速度”
    //    Step 2: 角度环 PID 计算“理论角速度”与“实测角速度”的偏差，输出控制量
    //    Step 3: Pid_Differential 将控制量分配给左右轮 (不对称加减速策略)
    Pure_Pursuit_Gyro_Control(speed_run, Err / 100.0f, gyro_z, &left_target, &right_target);

    // 4. 执行速度闭环
    pid_speed_update(&PID.left_speed, left_target, PID.left_speed.speed);
    pid_speed_update(&PID.right_speed, right_target, PID.right_speed.speed);
    if (flat_statr >= 2)
    {
        motor_output((int)PID.left_speed.output, (int)PID.right_speed.output);
    }
}
