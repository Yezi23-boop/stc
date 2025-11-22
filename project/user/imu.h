#ifndef _IMU_H_
#define _IMU_H_

/*
 * 模块说明
 * - 提供 IMU 姿态解算相关的类型、常量与接口声明�?
 * - 与实现文�? imu.c 配�?使用：先调用 Prepare_Data() 读取/预�?理传感器�?
 *   再调�? IMUupdate() 进�?一次姿态更新，输出欧拉�? Att_Angle�?
 *
 * 坐标/单位约定
 * - 角度：�?拉�?单位为“度”�?
 * - 角速度：Gyr_* 使用“弧�?/秒”（rad/s）；若为“度/秒”，请先乘以 DegtoRad�?
 * - 加速度：单位由底层驱动�?��函数 imu660ra_acc_transition 决定（g �? m/s^2），
 *   姿态解算仅使用其方向，内部会归一化�?
 */

/* 角度/弧度换算常量
 * RadtoDeg：弧�? -> �? 的比例系数（通常�? 57.29578f；�?处按项目定义保持不改�?
 * DegtoRad：度   -> 弧度 的比例系数（�? 0.0174533f�?
 */
#define RadtoDeg 57.324841f
#define DegtoRad 0.0174533f

/* 三轴向量（通用 XYZ�? */
typedef struct
{
    float X; // X 轴分�?
    float Y; // Y 轴分�?
    float Z; // Z 轴分�?
} FLOAT_XYZ;

/* 欧拉角（单位：度�?
 * rol：横滚（Roll，绕 X 轴）
 * pit：俯仰（Pitch，绕 Y 轴）
 * yaw：航向（Yaw，绕 Z 轴）
 */
typedef struct
{
    float rol;
    float pit;
    float yaw;
} FLOAT_ANGLE;

/* 四元数（全局，imu.c 内部维护�?
 * - q0 为标量部，q1~q3 为向量部
 * - �? IMUupdate() 更新，�?部可�??�?
 */
extern float q0, q1, q2, q3;

/* 全局姿态�?输出（度�? */
extern FLOAT_ANGLE Att_Angle;

/* �?��量（方向余弦估�?与�?�?��等），主要供调试查看
 * - vx,vy,vz：当前四元数估�?的重力方向（机体系下�?
 * - ex,ey,ez：重力方向的叉积�?��
 * - norm    ：归一化中间结�?
 */
extern float vx, vy, vz, ex, ey, ez, norm;

/* 预�?理后的传感器�?
 * - Acc_filt：滤波后的加速度（仅方向用于姿态校正）
 * - Gyr_filt：滤波后的�?速度（单位：rad/s�?
 */
extern FLOAT_XYZ Acc_filt, Gyr_filt;

/* �?�? 1/sqrt(x)
 * - 近似算法，速度�?��精度对姿态解算已足�?�?
 * - 若在其他模块调用请注意其近似特性；项目�?��常�?�� imu.c 内部使用�?
 */
float invSqrt(float x);

/* �?��sqrt(x)
 * - 近似算法，速度�?��精度对姿态解算已足�?�?
 * - 若在其他模块调用请注意其近似特性；项目�?��常�?�� imu.c 内部使用�?
 */
float SquareRootFloat(float number);

/* 传感器数�??处理
 * 功能�?
 * - 调用底层驱动读取一�? IMU 原�?数据�?
 * - 完成单位�?��与零偏校正；
 * - 更新全局 Acc_filt、Gyr_filt�?
 * 先决条件�?
 * - 已�?�?��始化 IMU �?��与驱�?��
 * - 提供 Gyro_offset_x/y/z（零偏）�? DegtoRad 常量�?
 */
void Prepare_Data(void);

/* 姿态更新（单�?�?
 * 参数�?
 * - Gyr_rad  ：指向�?速度（三轴），单�? rad/s�?
 * - Acc_filt ：指向加速度（三轴），仅使用方向，内部先归一化�?
 * - Att_Angle：输出�?拉�?（度）�?
 * 行为�?
 * - 基于四元数的 PI 反�?融合（加速度作为重力方向观测）；
 * - 内部维护四元�? q0~q3，并将其�?��为�?拉�?输出�?
 * 调用频率与�?长：
 * - 应与实际采样周期匹配（imu.c 内的 halfT/�?��步长需对应实际 Ts）�?
 */
void IMUupdate(FLOAT_XYZ *Gyr_rad, FLOAT_XYZ *Acc_filt, FLOAT_ANGLE *Att_Angle);

#endif