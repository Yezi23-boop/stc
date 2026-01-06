/************************************************************
 * @file: foc_paras.h
 * @author: Novosns MCU Team
 * @version: V0.0
 * @data: 2023/12/13
 * @brief: foc parameters header file
 * @note:
 * @Copyright (C) 2023 Novosense All rights reserved.
 ************************************************************/
 
/************************************************************
 ************************************************************
 * @par Edition History
 * -V0.0  2023.12.13
 *        -Initial version for foc_paras.h of NSUC1602.
 *
 ************************************************************/
 
#ifndef __FOC_PARAS_H__
#define __FOC_PARAS_H__
#include "mm_lib.h"

//--------------------------------------- 电机基本参数定义 --------------------------------------
// UOFFEST
#define U_OFFSET (0.25f)                  /*!< 电压采样偏置值，单位：V，用于ADC采样校准 */
#define MAX_CUR (62.5f)                   /*!< 最大电流采样量程，单位：A，电流采样硬件最大可测值 */
#define I_FULL_SCALE    MAX_CUR           /*!< 有效电流量程，单位：A，与MAX_CUR一致，简化代码复用 */
#define V_FULL_SCALE (50.0f)              /*!< 电压量程，单位：V，母线电压/相电压最大可测值 */
// 负的偏置电流
#define I_OFFSET (6.9f)                   /*!< 负向电流偏置值，单位：A，补偿电流采样零漂 */
// 电流计算系数
#define I_CALC_COEF (((I_FULL_SCALE + I_OFFSET) / I_FULL_SCALE) - 1.0f)  /*!< 电流计算补偿系数，修正偏置带来的误差 */
#define FOC_EPWM_FREQUENCY              10000               /*!< FOC载波频率，单位：Hz，10kHz为常用FOC载波频率 */

#define FOC_EPWM_PRESCALER               (1)                /*!< PWM分频系数，1表示不分频，直接使用48M时钟 */
#define FOC_TS                              (1.0f / (FOC_EPWM_FREQUENCY))  /*!< FOC控制周期，单位：s，10kHz对应0.0001s */
#define FOC_TIME_MS (1.0f / FOC_EPWM_FREQUENCY * 1000)       /*!< FOC控制周期，单位：ms，10kHz对应0.1ms */
// FOC_TIME_MS_CALC
#define FOC_TIME_MS_CALC(X) ((X) * (FOC_EPWM_FREQUENCY / 1000L))  /*!< 毫秒转FOC控制周期数的宏，用于计时换算 */

// 电流电压量程(整型)
#define I_FULL_SCALE_UINT               ((uint16_t)I_FULL_SCALE)  /*!< 电流量程整型值，用于无浮点运算的场景 */
#define V_FULL_SCALE_UINT               (50U)                     /*!< 电压量程整型值，50对应50V */

// 电机电气参数
#define PHASE_RESISTANCE                (0.0224f)                 /*!< 定子相电阻，单位：Ω，实测标定值 */
#define PHASE_Q_INDUCTANCE              (0.00005f)                /*!< Q轴电感，单位：H（亨），50μH */
#define PHASE_D_INDUCTANCE              (0.000038f)               /*!< D轴电感，单位：H（亨），38μH */
#define POLEPAIRS                       (7)                       /*!< 电机极对数，7对极（14极） */
// 磁链 0.0055wb
#define MOTOR_FLUX_LINKAGE              (0.0055f)                 /*!< 电机永磁体磁链，单位：Wb（韦伯） */

//======================================= 电机基准值定义 =========================================
#define FREQUENCY_BASE                  (466.66)                  /*!< 频率基准值，单位：Hz，略高于额定电频率 */
#define FREQUENCY_BASE_100_U16  	    (uint16_t)(FREQUENCY_BASE*100)  /*!< 频率基准值×100，整型存储提升精度 */
#define MAX_RPM                         (FREQUENCY_BASE * 60 / POLEPAIRS)  /*!< 最大转速，单位：RPM，由电频率和极对数计算 */
#define MIN_RPM                         500                       /*!< 最小稳定转速，单位：RPM，低于此转速易失步 */
#define OMEGA_BASE                      (6.28 * FREQUENCY_BASE)    /*!< 角频率基准值，单位：rad/s，2π×频率基准值 */
#define T_BASE                          (float)(1.0 / OMEGA_BASE)  /*!< 时间基准值，单位：s，角频率的倒数 */
#define L_BASE                          (V_FULL_SCALE / (OMEGA_BASE * I_FULL_SCALE))  /*!< 电感基准值，标幺化用 */
#define PSI_BASE                        (L_BASE * I_FULL_SCALE)    /*!< 磁链基准值，单位：Wb，标幺化用 */

//======================================= 定点化后的电机参数 =========================================
#define LD_Q15                     (FRAC16(PHASE_D_INDUCTANCE / L_BASE))  /*!< D轴电感Q15定点数，标幺化后存储 */
#define LQ_Q15                     (FRAC16(PHASE_Q_INDUCTANCE / L_BASE))  /*!< Q轴电感Q15定点数，标幺化后存储 */
#define FLUX_LINKAGE_Q15          (FRAC16(MOTOR_FLUX_LINKAGE / PSI_BASE)) /*!< 磁链Q15定点数，标幺化后存储 */

//--------------------------------------- 速度状态控制 --------------------------------------
#define PWM_NOMINAL_RPM 2250                                        /*!< 应用层额定转速，单位：RPM */
#define PWM_MIN_RPM 600                                            /*!< 应用层最低转速，单位：RPM */
#define PWM_MAX_RPM 2700                                           /*!< 应用层最高转速，单位：RPM */
#define FOC_STOP_SPEED                  (FRAC16(600/MAX_RPM))      /*!< 停机转速阈值（标幺值），低于此值直接刹车 */
#define FOC_MIN_SPEED                   (FRAC16(MIN_RPM/MAX_RPM))  /*!< 最小运行转速（标幺值），防止失步 */

//--------------------------------------- 速度环相关 --------------------------------------
//速度环执行分频系数
#define SPEED_LOOP_DIV                      10                    /*!< 速度环分频系数，每10个电流环周期执行1次速度环 */
#define SPEED_RAMP_SLOWLOOP_DIV             2                     /*!< 速度斜坡慢环分频系数，进一步降低斜坡更新频率 */
#define SPEED_ACCEL_RPM_S                   1500                  /*!< 速度加速度，单位：RPM/S，限制最大加速速率 */
#define SPEED_DECEL_RPM_S                   500                  /*!< 速度减速度，单位：RPM/S，防止发电回馈损坏电池 */

#define SPEED_LOOP_TIME_MS(X)              ((FOC_TIME_MS_CALC(X) / (SPEED_LOOP_DIV)))  /*!< 速度环时间换算宏 */
#define SPEED_ACCEL_PU_F16                  FRAC16((float)SPEED_ACCEL_RPM_S / (float)((MAX_RPM * FOC_EPWM_FREQUENCY) / (SPEED_LOOP_DIV * SPEED_RAMP_SLOWLOOP_DIV)))  /*!< 加速度标幺值（Q15） */
#define SPEED_DECEL_PU_F16                  FRAC16((float)SPEED_DECEL_RPM_S / (float)((MAX_RPM * FOC_EPWM_FREQUENCY) / (SPEED_LOOP_DIV * SPEED_RAMP_SLOWLOOP_DIV)))  /*!< 减速度标幺值（Q15） */
#define SPEED_FF_ENABLE                     0                    /*!< 速度前馈使能：0-禁用，1-启用（20251104未完成） */
#define SPEED_FF_ACCEL_GAIN                 FRAC16(0.5)          /*!< 加速前馈增益（Q15），0.5表示50%前馈 */
#define SPEED_FF_DECEL_GAIN                 FRAC16(0.4)          /*!< 减速前馈增益（Q15），0.4表示40%前馈 */
#define SPEED_FF_ERROR_PERCENT              FRAC16(0.1)          /*!< 速度误差百分比阈值（10%），低于此值关闭前馈 */

#define SPEED_RAMP_REDUCE_ENABLE             0                   /*!< 接近目标转速时斜率降低使能：0-禁用，1-启用 */
#define SPEED_RAMP_REDUCE_THRESHOLD          FRAC16(0.85)        /*!< 斜率降低阈值（85%），转速误差<15%时降斜率 */
#define SPEED_RAMP_MIN_RATIO                 FRAC16(0.0005)      /*!< 最小斜率比例，防止斜率过小导致响应过慢 */

//======================================= 堵转保护相关 ===========================================
#define TIME_MS(x)                      ((unsigned int)((x) * 1))  /*!< 毫秒转计数器值宏，简化计时配置 */

// 堵转检测时间参数
#define STALLDETECTION_BLANKCNT         TIME_MS(5)                 /*!< 堵转检测空白期，5ms，过滤启动初期无效检测 */
#define STALLDETECTION_CHKCNT           TIME_MS(200)               /*!< 堵转检查周期，200ms，累计200ms判定堵转 */
#define STALLDETECTION_CHKERRCNT        (STALLDETECTION_CHKCNT - 5)/*!< 堵转错误计数阈值，195ms，用于容错判定 */

// 反电动势与转速关系系数 (bEMF = coeffKE * wRotEl + coeffKEOFT)
#define STALLDETECTION_COEFFKE          0.69F                      /*!< 反电动势系数，实测标定，bEMF与转速的比例 */
#define STALLDETECTION_COEFFKENSHIFT    0                          /*!< 反电动势系数移位，0表示不移位 */
#define STALLDETECTION_COEFFKEOFT       0.025F                     /*!< 反电动势偏移量，补偿零漂 */
#define STALLDETECTION_COEFFKEOFTNSHIFT 0.0F                       /*!< 反电动势偏移移位，0表示不移位 */

// 堵转判定范围系数
#define STALLDETECTION_COEFFL           FRAC16(0.5)                /*!< 堵转判定下限系数（Q15），0.5表示50%基准值 */
#define STALLDETECTION_COEFFH           FRAC16(0.5)                /*!< 堵转判定上限系数（Q15），0.5表示50%基准值 */

//======================================= 开环强拖(IF)相关 ========================================
#define PERSENSORLESS_TIME_MS           FOC_TIME_MS_CALC(1500)     /*!< 预无感运行时间，1500ms，开环拖到足够转速 */
#define PERSENSORLESS_WIND_TIME_MS      FOC_TIME_MS_CALC(1500)     /*!< 顺风启动预无感时间，1500ms，适配顺风场景 */
#define PRESENSORLESS_REDUCE_IQ_ENABLE   0                          /*!< 预无感阶段降IQ使能：0-禁用，1-启用 */
#define PRESENSORLESS_USE_DUAL_DQ        0                          /*!< 双DQ空间使能：0-禁用，1-启用 */

// PRESENSORLESS 阶段角度差调节参数
//分频系数
#define PRESENSORLESS_DIV                5                          /*!< 角度误差调节分频系数，每5个周期调节1次 */
#define PRESENSORLESS_ANGLE_ERROR_GAIN  FRAC16(0.005)              /*!< 角度误差增益（Q15），0.005表示小步调节 */
#define PRESENSORLESS_IQ_MIN_LIMIT      FRAC16(0.05)              /*!< IQ最小限制值（Q15），0.05表示5%额定电流 */
#define PRESENSORLESS_ANGLE_CONVERGED   FRAC16(20.0/180.0)        /*!< 角度收敛阈值（Q15），20°/180°≈11.1% */
#define PRESENSORLESS_CONVERGED_COUNT   SPEED_LOOP_TIME_MS(500)   /*!< 角度收敛持续时间，500ms，确保稳定收敛 */

//======================================= 发波相关 ===============================================
// 零序电压注入补偿系数 (0.154时可完全利用母线电压, 0.154 = 1 / (sqrt(3) / 2))
#define SVPWM_ZERO_SEQ_COMP             (FRAC16(0.154f))          /*!< SVPWM零序补偿系数（Q15），提升母线电压利用率 */

//======================================= 单电阻采样配置 =========================================
#define USE_MEASURE_VECTOR              0                          /*!< 测量矢量使能：0-禁用，1-启用（单电阻采样优化） */

// 极限工况电流重构策略
#define ONE_RESISTOR_SAMPLING_USE_HISTORY       (2)                 /*!< 电流重构策略：0-当前值，1-历史值，2-DQ反推 */
#define ONE_RESISTOR_SAMPLING_USE_DQ_SPEED      FRAC16(0.5)         /*!< DQ反推速度阈值（Q15），0.5表示50%最大转速 */
#define ONE_RESISTOR_SAMPLING_DQ_SPEED_HYSTERESIS   FRAC16(0.05)    /*!< DQ反推速度回差（Q15），0.05表示5%最大转速 */
#define ONE_RESISTOR_SAMPLING_DQ_PREDICT_TIMES  (1)                 /*!< DQ反推角度预测拍数，1表示预测1个周期 */

//--------------------------------------- 顺逆风检测相关 --------------------------------------
#define USE_WIND_DETECT_HARDWARE                1                   /*!< 硬件顺风检测使能：0-禁用，1-启用 */
#define WIND_DETECT_PLL_BANDWIDTH_HZ            1000                /*!< 顺风检测PLL带宽，单位：Hz，风机场景需增大 */

//======================================= 高频注入(HFI)相关 ======================================
// 注入模式定义
#define HFI_INJECTION_BIPOLAR           0                          /*!< HFI双值注入模式：0-禁用，1-启用（-1/+1） */
#define HFI_INJECTION_TRIPOLAR          1                          /*!< HFI三值注入模式：0-禁用，1-启用（-1/0/+1） */
#define HFI_INJECTION_DIVIDER           2                          /*!< HFI分频模式：持续N个周期注入 */

// HFI使能与模式配置
#define ENABLE_HFI                      (0)                        /*!< HFI整体使能：0-禁用，1-启用（低速无感） */
#define ENABLE_HFI_AUTO_SWITCH          (0)                        /*!< HFI自动切换使能：0-禁用，1-启用 */
#define HFI_INJECTION_MODE              HFI_INJECTION_TRIPOLAR     /*!< HFI注入模式选择，三值模式 */

// HFI时间与频率参数
#define HFI_PWM_FREQUENCY               8000.0                    /*!< HFI开关频率，单位：Hz，8kHz */
#define HFI_TS                          (1.0f / (HFI_PWM_FREQUENCY)) /*!< HFI控制周期，单位：s */
#define HFI_TIME_MS(X)                  ((X) * (HFI_PWM_FREQUENCY / 1000L)) /*!< HFI时间换算宏 */
#define HFI_PLL_BANDWIDTH               100                       /*!< HFI的PLL带宽，单位：Hz，100Hz */

// HFI注入参数
#define CHECK_FLUX_ENV                  (0)                        /*!< 磁链环境检测使能：0-禁用，1-启用 */
#define HFI_DIVIDER_RATIO               2                          /*!< HFI分频系数，每2个周期注入1次 */
#define ENABLE_ID_OFFEST                (1)                        /*!< D轴偏置电流使能：0-禁用，1-启用 */
#define ID_OFFEST                       FRAC16(0.02)               /*!< D轴偏置电流（Q15），0.02表示2%额定电流 */

// HFI电压参数
#define HFI_VOLTAGE                     FRAC16(0.01)               /*!< HFI测试电压（Q15），0.01表示1%母线电压 */
#define HFI_POLARITY_VOLTAGE            FRAC16(0.05)               /*!< 磁极判定测试电压（Q15），0.05表示5%母线电压 */

// HFI速度控制参数
#define HFI_SPEED_RAMP_UP               5                          /*!< HFI速度上升斜坡，单位：RPM/周期 */
#define HFI_SPEED_RAMP_DOWN             5                          /*!< HFI速度下降斜坡，单位：RPM/周期 */
#define HFI_SPEED_TRAN2CLOSE            FRAC16(0.055)              /*!< HFI切闭环速度阈值（Q15），5.5%最大转速 */
#define HFI_TRAN2CLOSE_TIME             HFI_TIME_MS(10)            /*!< 切换确认时间，10ms，防止误切换 */

// HFI调试开关
#define HFI_IPD_DEBUG                   (0)                        /*!< 初始位置检测调试使能：0-禁用，1-启用 */
#define HFI_POLARITY_DEBUG              (0)                        /*!< 磁极判定调试使能：0-禁用，1-启用 */
#define HFI_TRACKING_DEBUG              (0)                        /*!< HFI跟踪调试使能：0-禁用，1-启用 */

//======================================= HFI自动切换相关 ========================================
#define HFI_TO_BEMF_THRESHOLD           FRAC16(0.2)                /*!< HFI切反电动势阈值（Q15），20%最大转速 */
#define BEMF_TO_HFI_THRESHOLD           FRAC16(0.15)               /*!< 反电动势切HFI阈值（Q15），15%最大转速（回差） */
#define HFI_SWITCH_STABLE_TIME          HFI_TIME_MS(20)            /*!< 转速稳定时间，20ms，确保切换稳定 */

//--------------------------------------高频注入随机化降噪相关（目前不支持）------------

/**                                                     
 * HFI随机注入降噪策略
 * 
 * 总体思路:
 * 通过随机化PWM频率和注入电压，降低电机高频噪声。固定频率的注入会产生单一频率的
 * 听得见的噪声，而随机化后噪声能量被分散到多个频率上，使噪声更接近白噪声，更不易被人耳感知。
 * 
 * 实现机制:
 * 1. 控制参数:
 *    - HFI_RANDOM_INJECTION_ENABLE: 控制是否启用随机注入功能
 *    - HFI_PWM_FREQUENCY_VARIATION: PWM频率变化范围(±值)
 *    - HFI_VOLTAGE_VARIATION: 电压幅值变化范围(百分比)
 *    - HFI_RANDOM_INTERVAL_MIN/MAX: 随机模式间隔范围(周期数) 
 *    - HFI_RANDOM_DURATION_MIN/MAX: 随机模式持续时间范围(周期数)
 *    - HFI_RANDOM_SAMPLE_ENABLE: 在随机模式下是否采样
 *    - HFI_HIGH_FREQ_BIAS_PERCENT: 高频偏向概率(75%时间在基础频率附近变化，25%时间生成更高频率)
 *    - HFI_PWM_FREQUENCY_MAX: 最大允许PWM频率
 * 
 * 2. 工作模式:
 *    系统在两种模式之间交替:
 *    - 固定模式: 使用基础PWM频率和基础注入电压
 *    - 随机模式: 使用随机生成的PWM频率和注入电压
 * 
 * 3. 随机化策略:
 *    a) 时间随机化:
 *       - 随机持续时间: 每次随机模式持续2-8个周期(可配置)
 *       - 随机间隔: 两次随机模式之间间隔20-100个周期(可配置)
 * 
 *    b) 频率随机化 - 双策略设计:
 *       - 小幅变化(75%概率): 在基础频率附近±HFI_PWM_FREQUENCY_VARIATION范围内变化
 *       - 高频突变(25%概率): 生成显著更高的频率，最高可达HFI_PWM_FREQUENCY_MAX
 *         这一策略将部分噪声能量移至人耳较不敏感的高频区域
 * 
 *    c) 电压随机化:
 *       - 在基础电压附近±HFI_VOLTAGE_VARIATION%范围内随机变化
 * 
 * 4. 随机源:
 *    - 使用硬件计时器(SysTick->VAL)和PWM计数器(EPWM->PWM0DH)值作为随机种子
 *    - 使用线性同余法生成随机序列
 * 
 * 工作效果:
 * - 常规变化模式使基本频率不再固定，消除单一频率噪声
 * - 高频突变模式将部分噪声能量转移到人耳不敏感区域
 * - 非对称频率分布创造"长尾"特性，使噪声更接近白噪声
 * - 电压幅值的随机变化进一步分散噪声能
 */

//======================================= HFI随机注入降噪相关(目前不支持) ============================
#define HFI_RANDOM_INJECTION_ENABLE         (0)                     /*!< HFI随机注入降噪使能：0-禁用，1-启用 */

// 随机化参数配置
#define HFI_PWM_FREQUENCY_VARIATION         (200)                   /*!< PWM频率变化范围，±200Hz */
#define HFI_VOLTAGE_VARIATION               (50)                    /*!< 电压变化范围，±50% */
#define HFI_RANDOM_INTERVAL_MIN             (20)                    /*!< 最小随机间隔，20个周期 */
#define HFI_RANDOM_INTERVAL_MAX             (30)                    /*!< 最大随机间隔，30个周期 */
#define HFI_RANDOM_DURATION_MIN             (10)                    /*!< 最小随机持续时间，10个周期 */
#define HFI_RANDOM_DURATION_MAX             (20)                    /*!< 最大随机持续时间，20个周期 */
#define HFI_RANDOM_SAMPLE_ENABLE            (0)                     /*!< 随机模式采样使能：0-禁用，1-启用 */

// 高频偏好参数
#define HFI_HIGH_FREQ_BIAS_PERCENT          (50)                    /*!< 高频偏向概率，50% */
#define HFI_PWM_FREQUENCY_MAX               (80000)                 /*!< 最大PWM频率，80kHz */

//======================================= HFI时间相关 ===========================================
#define HFI_TIMEOUT_TIME                    HFI_TIME_MS(8000)       /*!< HFI超时时间，8s，超时则退出HFI模式 */
#define HFI_IPD_TIME                        HFI_TIME_MS(1500)       /*!< 初始位置检测时间，1500ms */

//======================================= HFI磁极判定相关 ========================================
#define HFI_POLARITY_ID_THRESHOLD           10000                   /*!< 磁极判定电流阈值，整型值，用于判定磁极方向 */

// 磁极判定第一阶段 - 控制电流为0
#define HFI_POLARITY_STAGE1_IDQ0_TIME       HFI_TIME_MS(10)         /*!< 阶段1步骤1时间，10ms，控电流为0 */
#define HFI_POLARITY_STAGE1_VDQ0_TIME       HFI_TIME_MS(10)         /*!< 阶段1步骤2时间，10ms，控电压为0 */
#define HFI_POLARITY_STAGE1_TIME            (HFI_POLARITY_STAGE1_IDQ0_TIME + HFI_POLARITY_STAGE1_VDQ0_TIME) /*!< 阶段1总时间 */

// 磁极判定第二阶段 - 在方向1上加测试电压
#define HFI_POLARITY_DIRECTION1_TIME        (HFI_POLARITY_STAGE1_TIME + HFI_TIME_MS(40)) /*!< 阶段2测试时间，40ms */
#define HFI_POLARITY_DIRECTION1_IDSUM_TIME  (HFI_POLARITY_STAGE1_TIME + HFI_TIME_MS(30)) /*!< 阶段2采样时间，30ms */

// 磁极判定第三阶段 - 关闭测试电压，控制电流为0
#define HFI_POLARITY_STAGE3_IDQ0_TIME       (HFI_POLARITY_DIRECTION1_TIME + HFI_TIME_MS(20)) /*!< 阶段3步骤1时间，20ms */
#define HFI_POLARITY_STAGE3_VDQ0_TIME       (HFI_POLARITY_DIRECTION1_TIME + HFI_TIME_MS(10)) /*!< 阶段3步骤2时间，10ms */

// 磁极判定第四阶段 - 在方向2上加测试电压
#define HFI_POLARITY_DIRECTION2_TIME        (HFI_POLARITY_STAGE3_IDQ0_TIME + HFI_TIME_MS(40)) /*!< 阶段4测试时间，40ms */
#define HFI_POLARITY_DIRECTION2_IDSUM_TIME  (HFI_POLARITY_STAGE3_IDQ0_TIME + HFI_TIME_MS(30)) /*!< 阶段4采样时间，30ms */

// 磁极判定第五阶段 - 关闭测试电压，控制电流为0
#define HFI_POLARITY_STAGE5_IDQ0_TIME       (HFI_POLARITY_DIRECTION2_TIME + HFI_TIME_MS(20)) /*!< 阶段5步骤1时间，20ms */
#define HFI_POLARITY_STAGE5_VDQ0_TIME       (HFI_POLARITY_DIRECTION2_TIME + HFI_TIME_MS(10)) /*!< 阶段5步骤2时间，10ms */

//======================================= 参数辨识相关 ===========================================
//是否开启参数辨识
#define ENABLE_PARAMETER_IDENTIFY          (0)                     /*!< 参数辨识使能：0-禁用，1-启用 */
// 电阻辨识参数
#define PARAM_ID_R_START_VOLTAGE            FRAC16(0.001)           /*!< 电阻辨识起始电压（Q15），0.001倍母线电压 */
#define PARAM_ID_R_STEP_VOLTAGE             FRAC16(0.001)           /*!< 电阻辨识电压步进（Q15），小步提升 */
#define PARAM_ID_R_LOW_VOLTAGE              FRAC16(0.01)            /*!< 电阻辨识低电压（Q15），1%母线电压 */
#define PARAM_ID_R_MID_VOLTAGE              FRAC16(0.07)            /*!< 电阻辨识中等电压（Q15），7%母线电压 */
#define PARAM_ID_R_VOLT_DELAY_TIME          HFI_TIME_MS(1000)       /*!< 电压稳定时间，1000ms */
#define PARAM_ID_R_DELAY_TIME               HFI_TIME_MS(1000)       /*!< 电流稳定时间，1000ms */

// 电感辨识参数
#define ENABLE_HFI_IDENTIFY                 (0)                     /*!< HFI电感辨识使能：0-禁用，1-启用 */
#define HFI_IDENTIFY_VOLTAGE                FRAC16(0.01)            /*!< 电感辨识定位电压（Q15），1%母线电压 */
#define HFI_IDENTIFY_ALIGN_TIME             HFI_TIME_MS(3000)       /*!< 电感辨识定位时间，3000ms */
#define HFI_IDENTIFY_ALIGN_DELAY_TIME       HFI_TIME_MS(500)        /*!< 定位后延时，500ms */
#define HFI_IDENTIFY_STEP_ANGLE             FRAC16(0.01)            /*!< 角度步进值（Q15），1%电角度 */
#define HFI_IDENTIFY_STEP_TIME              HFI_TIME_MS(50)         /*!< 角度点停留时间，50ms */

//磁链辨识
//使用ID还是使用IQ强拖 0 使用ID 1使用IQ
#define FLUX_IDENTIFY_USE_IQ (0)
//阶段1强拖电流
#define FLUX_IDENTIFY_STAGE1_CURRENT FRAC16(0.6) /*!< 磁链辨识阶段1强拖电流（Q15），60%额定电流 */
//阶段2强拖电流
#define FLUX_IDENTIFY_STAGE2_CURRENT FRAC16(0.3) /*!< 磁链辨识阶段2强拖电流（Q15），30%额定电流 */
//转速步进
#define FLUX_IDENTIFY_SPEED_STEP 1               /*!< 转速步进值，1RPM/周期 */
//目标转速
#define FLUX_IDENTIFY_TARGET_SPEED FRAC16(0.15)   /*!< 磁链辨识目标转速（Q15），15%最大转速 */
//转速稳定时间
#define FLUX_IDENTIFY_SPEED_STABLE_TIME FOC_TIME_MS_CALC(2000) /*!< 转速稳定时间，2000ms */

//磁链辨识辨识阶段1时间
#define FLUX_IDENTIFY_STAGE1_TIME FOC_TIME_MS_CALC(1000) /*!< 磁链辨识阶段1时间，1000ms */
//磁链辨识辨识阶段2时间
#define FLUX_IDENTIFY_STAGE2_TIME FOC_TIME_MS_CALC(500)  /*!< 磁链辨识阶段2时间，500ms */
//磁链辨识阶段3时间
#define FLUX_IDENTIFY_STAGE3_TIME FOC_TIME_MS_CALC(1000) /*!< 磁链辨识阶段3时间，1000ms */

//---------------------------------------母线电流估算----------------
#define I_DC2AC_EFFICIENCY FRAC16(1.0)    /*!< DC-AC效率系数（Q15），1.0表示100%效率 */
#define SPEED_FACTOR FRAC16(0.2)          /*!< 速度修正系数（Q15），0.2表示20%修正 */
#define DEAD_TIME_FACTOR FRAC16(0.9)      /*!< 死区时间修正系数（Q15），0.9表示90%补偿 */
#define DEAD_TIME_NS 5000                 /*!< 死区时间，单位：ns，5000ns=5μs */
#define TON_NS 500                       /*!< 功率管开通延时，单位：ns，500ns */
#define TOFF_NS 300                      /*!< 功率管关断延时，单位：ns，300ns */
#define PWM_PERIOD_NS 50000              /*!< PWM周期，单位：ns，50000ns=50μs（20kHz） */

//======================================= 应用层策略相关 ==========================================
#define FAULT_RESTART_DELAY_MS              (50)                    /*!< 故障重启延时，50ms，防止频繁重启 */

// 功能使能开关
#define ENABLE_TEMP_DERATING                (0)                     /*!< 温度降额使能：0-禁用，1-启用 */
#define ENABLE_VOLT_DERATING                (0)                     /*!< 电压降额使能：0-禁用，1-启用 */
#define ENABLE_CURRENT_POWER_LIMIT          (0)                     /*!< 电流/功率限制使能：0-禁用，1-启用 */

// IQ电流功率限制参数
#define BUS_CURRENT_LIMIT                   FRAC16(20.0f / I_FULL_SCALE) /*!< 母线电流限制（Q15），20A/62.5A=32% */
#define POWER_LIMIT                         FRAC16(200.0f / (V_FULL_SCALE * I_FULL_SCALE)) /*!< 功率限制（Q15），200W/(50V×62.5A)=6.4% */
#define POWER_CIRCUIT_IQ_LOOP_DIV              10                     /*!< 电流/功率环分频系数，每10个速度环周期执行1次 */
#define DERATING_IQ_STEP                    1                      /*!< IQ降额步长，每次降1个单位 */
#define RECOVER_IQ_STEP                     1                      /*!< IQ恢复步长，每次升1个单位 */
#define CURRENT_POWER_LIMIT_HYSTERESIS      FRAC16(0.1)            /*!< 电流/功率限制回差（Q15），10% */

//温度功率电流限制策略是否启用-该策略由温度获取对应的功率与电流限值，通过限制值对转速进行上限调控
#define ENABLE_TEMP_POWER_CURRENT_LIMIT     (0)                     /*!< 温度-功率-电流限制使能：0-禁用，1-启用 */
#define POWER_CIRCUIT_LOOP_DIV              100                     /*!< 功率环分频系数，100ms执行1次 */
#define POWER_OVER_LIMIT_CNT_THRESHOLD      500                    /*!< 功率超限确认次数，500次（5ms） */
#define POWER_CLEAR_LIMIT_CNT_THRESHOLD     500                    /*!< 功率恢复确认次数，500次（50ms） */
#define CURRENT_OVER_LIMIT_CNT_THRESHOLD    500                    /*!< 电流超限确认次数，500次（5ms） */
#define CURRENT_CLEAR_LIMIT_CNT_THRESHOLD   500                    /*!< 电流恢复确认次数，500次（50ms） */

#define TEMP_POWER_LIMIT_DEFAULT            FRAC16(452.0f / (V_FULL_SCALE * I_FULL_SCALE)) /*!< 默认功率限制（Q15），452W/(50V×62.5A) */
#define TEMP_CURRENT_LIMIT_DEFAULT          FRAC16(35.0f / I_FULL_SCALE) /*!< 默认电流限制（Q15），35A/62.5A=56% */
#define POWER_CIRCUIT_LOOP_DIV              100                     /*!< 功率环控制周期分频系数，100ms */
#define TEMP_SPEED_DERATE_HYSTERESIS        FRAC16(50.0f / MAX_RPM) /*!< 温度降速回差（Q15），50RPM/最大转速 */
#define TEMP_SPEED_DERATE_STEP              FRAC16(5.0f / MAX_RPM)  /*!< 温度降速步长（Q15），5RPM/最大转速 */
#define TEMP_SPEED_RECOVER_STEP             FRAC16(5.0f / MAX_RPM)  /*!< 温度恢复转速步长（Q15），5RPM/最大转速 */

//======================================= 保护策略相关 ============================================
// 母线电压保护
#define BUS_VOLTAGE_OFFSET                  (0.0f)                  /*!< 母线电压偏置，校准用 */
#define OV_VOLTAGE_UPLIMT                   (18.4f)                 /*!< 过压阈值，18.4V，超过则触发保护 */
#define OV_VOLTAGE_RECOV                    (0.3f)                  /*!< 过压恢复回差，0.3V，低于18.1V恢复 */
#define UV_VOLTAGE_UPLIMT                   (7.8f)                  /*!< 欠压阈值，7.8V，低于则触发保护 */
#define UV_VOLTAGE_RECOV                    (0.6f)                  /*!< 欠压恢复回差，0.6V，高于8.4V恢复 */

//过温阈值
#define ECU_TEMP_UPLIMT (1400U)             /*!< ECU过温阈值，1400对应140℃（10倍放大） */
//过温恢复阈值
#define ECU_TEMP_RECOV (1300U)              /*!< ECU过温恢复阈值，1300对应130℃ */

//缺相电流最小阈值
#define UNBALANCE_MIN_CURR FRAC16(4.0f/I_FULL_SCALE) /*!< 缺相最小电流阈值（Q15），4A/62.5A=6.4% */
//缺相电流最大阈值
#define UNBALANCE_MAX_CURR FRAC16(10.0f/I_FULL_SCALE) /*!< 缺相最大电流阈值（Q15），10A/62.5A=16% */

/**
 * @brief FOC核心参数结构体
 * @details 集中存储FOC控制的所有核心可调参数，包括保护阈值、PI参数、限位值等，便于统一配置和调试
 */
typedef struct {
    float I_MAX;                            /*!< 最大电流限值，单位：A */
    float U_DCB_MAX;                        /*!< 最大母线电压，单位：V */
    float N_MAX;                            /*!< 最大转速，单位：RPM */
    float WEL_MAX;                          /*!< 最大能量限值，单位：J（焦耳） */
    float E_MAX;                            /*!< 最大反电动势，单位：V */
    Frac16_t MOTOR_PP_GAIN;                 /*!< 电机极对数增益（Q15），用于转速/角度换算 */
    uint16_t MOTOR_PP_SHIFT;                /*!< 电机极对数移位，提升运算精度 */
    Frac16_t U_DCB_TRIP;                    /*!< 母线电压跳闸阈值（Q15） */
    Frac16_t U_DCB_UNDERVOLTAGE;            /*!< 母线欠压阈值（Q15） */
    Frac16_t U_DCB_OVERVOLTAGE;             /*!< 母线过压阈值（Q15） */
    Frac16_t I_PH_OVER;                     /*!< 相电流过流阈值（Q15） */
    Frac16_t TEMP_OVER;                     /*!< 过温阈值（Q15） */
    Frac16_t DC_BUS_FILTER_MA_NPOINT;       /*!< 母线电压滤波点数（Q15），滑动平均滤波用 */
    Frac16_t ALIGN_VOLTAGE;                 /*!< 电机对齐电压（Q15），启动前定位用 */
    uint16_t ALIGN_DURATION;                /*!< 对齐持续时间，单位：ms */
    Frac16_t CLOOP_LIMIT;                   /*!< 闭环输出限制（Q15），防止过调制 */
    uint16_t D_NSHIFT;                      /*!< D轴PI调节器移位位数 */
    Frac16_t D_CC1SC;                       /*!< D轴PI调节器CC1系数（Q15） */
    Frac16_t D_CC2SC;                       /*!< D轴PI调节器CC2系数（Q15） */
    uint16_t Q_NSHIFT;                      /*!< Q轴PI调节器移位位数 */
    Frac16_t Q_CC1SC;                       /*!< Q轴PI调节器CC1系数（Q15） */
    Frac16_t Q_CC2SC;                       /*!< Q轴PI调节器CC2系数（Q15） */
    Frac16_t SPEED_PI_PROP_GAIN;            /*!< 速度环PI比例增益（Q15） */
    int16_t SPEED_PI_PROP_SHIFT;            /*!< 速度环PI比例增益移位 */
    Frac16_t SPEED_PI_INTEG_GAIN;           /*!< 速度环PI积分增益（Q15） */
    int16_t SPEED_PI_INTEG_SHIFT;           /*!< 速度环PI积分增益移位 */
    Frac16_t SPEED_LOOP_HIGH_LIMIT;         /*!< 速度环上限（Q15） */
    Frac16_t SPEED_LOOP_LOW_LIMIT;          /*!< 速度环下限（Q15） */
    Frac16_t SPEED_RAMP_UP;                 /*!< 速度上升斜坡（Q15） */
    Frac16_t SPEED_RAMP_DOWN;               /*!< 速度下降斜坡（Q15） */
    uint16_t SPEED_LOOP_CNTR;               /*!< 速度环计数器，分频执行用 */
    Frac16_t POSPE_SPEED_FILTER_MA_NPOINT;  /*!< 转速滤波点数（Q15），滑动平均滤波用 */
    Frac16_t I_GAIN;                        /*!< 电流增益（Q15），反电动势观测器用 */
    Frac16_t U_GAIN;                        /*!< 电压增益（Q15），反电动势观测器用 */
    Frac16_t E_GAIN;                        /*!< 反电动势增益（Q15），反电动势观测器用 */
    Frac16_t WI_GAIN;                       /*!< 角速度-电流耦合增益（Q15） */
    int16_t BEMF_SHIFT;                     /*!< 反电动势移位，提升运算精度 */
    Frac16_t BEMF_DQ_CC1_GAIN;              /*!< 反电动势DQ轴CC1增益（Q15） */
    Frac16_t BEMF_DQ_CC2_GAIN;              /*!< 反电动势DQ轴CC2增益（Q15） */
    uint16_t BEMF_DQ_NSHIFT;                /*!< 反电动势DQ轴移位位数 */
    Frac16_t TO_CC1SC;                      /*!< 跟踪观测器CC1系数（Q15） */
    Frac16_t TO_CC2SC;                      /*!< 跟踪观测器CC2系数（Q15） */
    uint16_t TO_NSHIFT;                     /*!< 跟踪观测器移位位数 */
    Frac16_t TO_THETA_GAIN;                 /*!< 跟踪观测器角度增益（Q15） */
    uint16_t TO_THETA_SHIFT;                /*!< 跟踪观测器角度移位 */
    Frac16_t OL_START_RAMP_INC;             /*!< 开环启动斜坡增量（Q15） */
    Frac16_t OL_START_I;                    /*!< 开环启动电流（Q15） */
    Frac16_t MERG_SPEED_1_TRH;              /*!< 模式切换速度阈值1（Q15） */
    Frac16_t MERG_SPEED_2_TRH;              /*!< 模式切换速度阈值2（Q15） */
    Frac16_t HFI_ID_REF_RUN ;               /*!< HFI运行时D轴电流参考（Q15） */
    Frac16_t HFI_IQ_REF_RUN ;               /*!< HFI运行时Q轴电流参考（Q15） */
    Frac16_t HFI_CLOOP_LIMIT ;              /*!< HFI闭环输出限制（Q15） */
    uint16_t HFI_D_NSHIFT ;                 /*!< HFI的D轴PI移位位数 */
    Frac16_t HFI_D_CC1SC ;                  /*!< HFI的D轴PI CC1系数（Q15） */
    Frac16_t HFI_D_CC2SC ;                  /*!< HFI的D轴PI CC2系数（Q15） */
    uint16_t HFI_Q_NSHIFT ;                 /*!< HFI的Q轴PI移位位数 */
    Frac16_t HFI_Q_CC1SC ;                  /*!< HFI的Q轴PI CC1系数（Q15） */
    Frac16_t HFI_Q_CC2SC ;                  /*!< HFI的Q轴PI CC2系数（Q15） */
    Frac16_t HFI_SPEED_PI_PROP_GAINL;       /*!< HFI速度环PI比例增益（低转速）（Q15） */
    Frac16_t HFI_SPEED_PI_PROP_SHIFT;       /*!< HFI速度环PI比例移位 */
    Frac16_t HFI_SPEED_PI_INTEG_GAIN;       /*!< HFI速度环PI积分增益（Q15） */
    Frac16_t HFI_SPEED_PI_INTEG_SHIFT;      /*!< HFI速度环PI积分移位 */
    Frac16_t HFI_SPEED_LOOP_HIGH_LIMIT;     /*!< HFI速度环上限（Q15） */
    Frac16_t HFI_SPEED_LOOP_LOW_LIMIT;      /*!< HFI速度环下限（Q15） */
    Frac16_t HFI_SPEED_LOOP_CNTR;           /*!< HFI速度环计数器 */
    //对齐阶段定位电流
    Frac16_t ALIGN_ID_REF;                  /*!< 对齐阶段D轴电流参考（Q15） */
    //对齐阶段顺逆风检测时间
    Frac16_t ALIGN_WIND_DETECT_TIME;        /*!< 对齐阶段顺逆风检测时间（Q15） */
    //对齐阶段的刹车时间
    Frac16_t ALIGN_BRAKE_TIME;              /*!< 对齐阶段刹车时间（Q15） */
} FOC_Paras_t;

extern FOC_Paras_t tFocParas;               /*!< FOC核心参数全局实例，所有FOC控制逻辑共用 */
#endif // __FOC_PARAS_H__