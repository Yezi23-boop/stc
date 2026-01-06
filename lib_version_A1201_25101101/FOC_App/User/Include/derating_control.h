#ifndef DERATING_CONTROL_H 
#define DERATING_CONTROL_H 

#include "stdint.h"       
#include "foc_paras.h"   

#define MAX_DERATING_POINTS 5
//电压限额配置表定义区域--  署名：LR
//降额线性区域简易结构
/*
1.低速区域：0-10V
2.线性区域：8.5-11.5V
3.高速区域：11.5V-过压电压阈值
0V            8.5V             11.5V       (过压电压)
|              |                  |             |
|-> 1800rpm <- |-> 1800-2700rpm <-|-> 2700rpm <-|
|              |                  |             |
                |                |        
              9.0V            11.0V       

设置上下限额中状态留有一个缓冲区，这个缓冲区大小可以根据实际情况调整。
1.低速区域转换到线性区域，电压需要超过9.0V，转速则进入线性区域。
2.高速区域转换到线性区域，电压需要低于11.0V，转速则进入线性区域。

3.线性区域转换到高速区域，电压需要高于11.5V，转速则等于2700rpm。
4.线性区域转换到低速区域，电压需要低于8.5V，转速则等于1800rpm。
*/

//线性区域开始阈值
#define LINE_VOLT_START 8.5f   
//线性区域结束阈值
#define LINE_VOLT_END 11.5f

//线性区域下边界进入阈值
#define LINE_VOLT_LOW_ENTER 9.0f

//线性区域上边界进入阈值
#define LINE_VOLT_TOP_ENTER 11.0f

//线性区域最大转速
#define LINE_RPM_MAX 2700
//线性区域最小转速
#define LINE_RPM_MIN 1800
//速度跳变缓冲阈值（当变化转速超过此值时，才会触发跳变处理）
#define SPEED_JUMP_VALUE 40
// 进入线性区域延时1s
#define ENTER_TIME_DELAY 1000 
// 退出线性区域延时1s
#define EXIT_TIME_DELAY 1000

//线性限额区域电压与转速比例系数 
#define LINE_FACTOR_RPM_VOLT  (float)(LINE_RPM_MAX-LINE_RPM_MIN)/(FRAC16(LINE_VOLT_END/V_FULL_SCALE)-FRAC16(LINE_VOLT_START/V_FULL_SCALE))
//电流限额配置表定义区域--  署名：LR


// 单个降额点定义 (可用于温度或电压降额)
typedef struct {
    int16_t activate_threshold;     // 激活此降额等级的阈值
                                    // 对于温度: 单位是实际摄氏度。当温度升高达到或超过此值时激活。
                                    // 对于电压: 单位是 Frac16_t 格式的滤波后母线电压 (VdcFilt)。当电压降低到或低于此值时激活。
    int16_t deactivate_threshold;   // 解除此降额等级的阈值 (用于实现回差控制，防止在临界点抖动)
                                    // 对于温度: 当温度降低到此值以下时解除当前降额等级。
                                    // 对于电压: 当电压升高到此值以上时解除当前降额等级。
    uint16_t max_rpm;               // 在此降额等级下的最大允许RPM (转/分钟)
} DeratingPoint_t;

// 降额配置表定义
typedef struct {
    const DeratingPoint_t *temp_points_table; // 指向温度降额点数组的指针。此数组应按 'activate_threshold' 升序排列。
    uint8_t num_temp_points;                  // 温度降额点数组中的元素个数。
    const DeratingPoint_t *volt_points_table; // 指向电压降额点数组的指针。此数组应按 'activate_threshold' 降序排列 (即更低的电压阈值在前)。
    uint8_t num_volt_points;                  // 电压降额点数组中的元素个数。
    uint16_t default_max_rpm;                 // 系统默认的最大RPM，当没有任何降额条件激活时使用。
    // 可以考虑添加一个指向 tFocParas 的指针，如果 N_MAX 不容易全局访问的话
    // FOC_Paras_t* foc_params_ptr;
} Derating_Config_t;

// 结构体添加计数器字段
typedef struct {
    const Derating_Config_t *config;
    uint8_t current_temp_derating_idx;
    uint8_t current_volt_derating_idx;
    uint16_t current_max_allowed_rpm;
    
    // 添加滤波计数器
    uint16_t temp_entry_counter[MAX_DERATING_POINTS]; // 温度降额进入计数器
    uint16_t temp_exit_counter[MAX_DERATING_POINTS];  // 温度降额退出计数器
    uint16_t volt_entry_counter[MAX_DERATING_POINTS]; // 电压降额进入计数器
    uint16_t volt_exit_counter[MAX_DERATING_POINTS];  // 电压降额退出计数器
} Derating_Controller_t;

#define NO_DERATING_IDX (0xFF) // 定义一个特殊值，用于指示当前没有激活的降额等级索引。

/**
 * @brief 初始化降额控制器。
 * @param controller 指向 Derating_Controller_t 结构体的指针，将被初始化。
 * @param config 指向 Derating_Config_t 配置表的指针，包含降额规则。
 */
extern void Derating_Init(Derating_Controller_t *controller, const Derating_Config_t *config);

/**
 * @brief 根据当前温度和电压，应用降额逻辑到用户请求的目标转速。
 * @param controller 指向 Derating_Controller_t 结构体的指针。
 * @param target_speed_req_frac16 用户请求的目标转速，使用 Frac16_t (Q1.15) 格式表示的归一化值。
 * @param current_temp_celsius 当前电机或环境温度，单位为摄氏度。
 * @param current_vdc_filt_frac16 当前滤波后的母线电压，使用 Frac16_t 格式表示的归一化值。
 * @param n_max_rpm 电机的额定最大物理转速 (RPM)，用于 Frac16_t 和 RPM 值之间的转换。
 * @return Frac16_t 经过降额逻辑限制后的目标转速，仍然是 Frac16_t 格式。
 */
extern Frac16_t Derating_Apply(Derating_Controller_t *controller, Frac16_t target_speed_req_frac16, int16_t current_temp_celsius, Frac16_t current_vdc_filt_frac16, uint16_t n_max_rpm);

/**
 * @brief 计算当前条件下允许的最大RPM（综合考虑温度和电压降额）。
 * @param controller 指向 Derating_Controller_t 结构体的指针。
 * @param current_temp_celsius 当前温度，单位为摄氏度。
 * @param current_vdc_filt_frac16 当前母线电压，Frac16_t 格式。
 * @return uint16_t 当前条件下允许的最大RPM值。
 */
extern uint16_t Derating_GetMaxAllowedRpm(Derating_Controller_t *controller, int16_t current_temp_celsius, Frac16_t current_vdc_filt_frac16);

/**
 * @brief 将RPM值转换为Frac16_t格式的归一化速度值。
 * @param rpm 需要转换的RPM值。
 * @param n_max_rpm 电机额定最大RPM，此值对应于Frac16_t的1.0 (即32767)。
 * @return Frac16_t 转换后的归一化速度值。
 */
extern Frac16_t Derating_ConvertRpmToFrac16(uint16_t rpm, uint16_t n_max_rpm);

/**
 * @brief 将Frac16_t格式的归一化速度值转换为RPM值。
 * @param speed_frac16 需要转换的Frac16_t归一化速度值。
 * @param n_max_rpm 电机额定最大RPM，Frac16_t的1.0对应此RPM值。
 * @return uint16_t 转换后的RPM值。
 */
extern uint16_t Derating_ConvertFrac16ToRpm(Frac16_t speed_frac16, uint16_t n_max_rpm);

extern const Derating_Config_t g_motor_derating_config; // 全局降额配置表，通常在其他源文件中定义和初始化
extern Derating_Controller_t g_derating_controller; // 全局降额控制器实例，用于在应用中跟踪当前状态

#endif // DERATING_CONTROL_H // 防止头文件被重复包含的宏定义结束