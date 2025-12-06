#include "zf_common_headfile.h"

int flat_statr = 0;
static int flat_statr_date = 0;
void run_time_1(void)
{
    if (!P32)
    {
        IAP_CONTR = 0x60; // 判断快速烧录
    }
    read_AD(); // 读取并处理电感数据
    Prepare_Data();
    Encoder_get(&PID.left_speed, &PID.right_speed);
    pid_steer_update(&PID.steer, Err, -imu660ra_gyro_z * 0.01);                              // 更新转向环
    pid_steer_update(&PID.angle, PID.steer.output, 0);                                       // 更新角度环
    pid_speed_update(&PID.left_speed, speed_run - PID.angle.output, PID.left_speed.speed);   // 更新左轮速度环
    pid_speed_update(&PID.right_speed, speed_run + PID.angle.output, PID.right_speed.speed); // 更新右轮速度环
    if (flat_statr >= 2)
    {
        motor_output((int)PID.left_speed.output, (int)PID.right_speed.output);
        //	 motor_output(1000, 1000);
    }
    //    test_speed();
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
    float dec_gain = 0.0f;
    float acc_gain = 0.0f;
    float abs_delta = 0.0f;
    if (!P32)
    {
        IAP_CONTR = 0x60; // 判断快速烧录
    }
    read_AD(); // 读取并处理电感数据
    Prepare_Data();
    Encoder_get(&PID.left_speed, &PID.right_speed);
    pid_steer_update(&PID.steer, Err, 0);                                   // 更新转向环，增加陀螺仪抑制甩尾
    pid_steer_update(&PID.angle, PID.steer.output, imu660ra_gyro_z * 0.01); // 更新角度环
    delta = PID.steer.output;
    abs_delta = delta >= 0.0f ? delta : -delta;
    dec_gain = 1.2f;
    acc_gain = 0.8f;
    if (abs_delta > 40.0f)
    {
        dec_gain = 1.5f;
        acc_gain = 0.5f;
    }
    if (delta >= 0.0f)
    {
        left_target = speed_run - dec_gain * delta;
        right_target = speed_run + acc_gain * delta;
    }
    else
    {
        left_target = speed_run - acc_gain * delta;
        right_target = speed_run + dec_gain * delta;
    }
    pid_speed_update(&PID.left_speed, left_target, PID.left_speed.speed);
    pid_speed_update(&PID.right_speed, right_target, PID.right_speed.speed);
    if (flat_statr >= 2)
    {
        motor_output((int)PID.left_speed.output, (int)PID.right_speed.output);
        //	 motor_output(1000, 1000);
    }
    //    test_speed();
}
