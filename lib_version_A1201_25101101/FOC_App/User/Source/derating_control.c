#include "derating_control.h" // 包含降额控制的头文件
#include "nvsns_foc.h"           
#include "nvsns_type_def.h"            
#include "stdint.h"
#include "stdio.h"
#include "config.h"


// 确保MIN宏可用，如果未定义则定义一个
#ifndef MIN
#define MIN(a,b) (((a)<(b))?(a):(b)) // 定义MIN宏，返回两个值中的较小者
#endif


#define DERATING_ENTRY_COUNT  100  // 进入降额状态需要达到的计数值 (约1秒，假设10ms调用一次)
#define DERATING_EXIT_COUNT   200  // 退出降额状态需要达到的计数值 (约2秒)

// 温度降额点定义
// 温度降额点数组应按 activate_threshold 升序排列，以便更高的温度阈值在后面
//温度单位0.1度
//尽量这一级的activate_C要小于下一级的 deactivate_C
const DeratingPoint_t g_temp_derating_profile[] = {
    // {activate_C, deactivate_C, max_rpm}
    {1000, 950, 2200}, 
    {1100, 1050, 2000},
    {1200, 1150, 1500} 
    // //测试用
    // {600, 550, 2200}, 
    // {700, 650, 1800}


};


//电压降额点定义(排序为 activate_threshold 降序，以便更低的电压阈值在前)
// 注意：假设 Vdc_filt 是 Q15 格式，其中 FRAC16(1.0) 是最大 Vdc。
// 尽量这一级的activate_Vdc_filt_Q15要大于下一级的 deactivate_Vdc_filt_Q15
const DeratingPoint_t g_volt_derating_profile[] = {
    // {activate_Vdc_filt_Q15, deactivate_Vdc_filt_Q15, max_rpm}
    {FRAC16(10.0/V_FULL_SCALE), FRAC16(10.5/V_FULL_SCALE), 1500}, 
    // {FRAC16(9/V_FULL_SCALE), FRAC16(9.5/V_FULL_SCALE), 1800}  
};


// 编译时断言检查：确保温度降额配置点数量不超过预设的最大值
#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
    // C11或更高版本支持_Static_assert
    _Static_assert(sizeof(g_temp_derating_profile) / sizeof(DeratingPoint_t) <= MAX_DERATING_POINTS,
                  "Temperature derating profile exceeds maximum allowed points!");
#else
    // 对于不支持_Static_assert的编译器，使用传统的编译时断言方法
    typedef char assert_temp_profile_size_check[(sizeof(g_temp_derating_profile) / sizeof(DeratingPoint_t) <= MAX_DERATING_POINTS) ? 1 : -1];
#endif

// 编译时断言检查：确保电压降额配置点数量不超过预设的最大值
#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
    _Static_assert(sizeof(g_volt_derating_profile) / sizeof(DeratingPoint_t) <= MAX_DERATING_POINTS,
                  "Voltage derating profile exceeds maximum allowed points!");
#else
    typedef char assert_volt_profile_size_check[(sizeof(g_volt_derating_profile) / sizeof(DeratingPoint_t) <= MAX_DERATING_POINTS) ? 1 : -1];
#endif

const Derating_Config_t g_motor_derating_config = {
    .temp_points_table = g_temp_derating_profile,
    .num_temp_points = sizeof(g_temp_derating_profile) / sizeof(DeratingPoint_t),
    .volt_points_table = g_volt_derating_profile,
    .num_volt_points = sizeof(g_volt_derating_profile) / sizeof(DeratingPoint_t),
    .default_max_rpm = PWM_MAX_RPM // System's absolute max RPM without any derating
};

Derating_Controller_t g_derating_controller;

/**
 * @brief 将RPM值转换为Frac16_t归一化速度值。
 * @param rpm 需要转换的RPM值。
 * @param n_max_rpm 电机额定最大RPM，此值对应于Frac16_t的1.0 (即32767)。
 * @return Frac16_t 转换后的归一化速度值。
 */
Frac16_t Derating_ConvertRpmToFrac16(uint16_t rpm, uint16_t n_max_rpm) {
    if (n_max_rpm == 0) return 0; // 如果最大RPM为0，则返回0，防止除零错误
    if (rpm >= n_max_rpm) return FRAC16(1.0); // 如果输入RPM大于或等于最大RPM，则返回Frac16的最大值 (代表1.0)
    
    // 计算归一化速度: (rpm / n_max_rpm) * 32767
    // 使用32位中间变量进行计算以防止溢出
    int32_t temp_speed = ((int32_t)rpm * 32767L) / n_max_rpm;
    
    // 确保结果在Frac16_t (int16_t) 的范围内
    if (temp_speed > INT16_MAX) temp_speed = INT16_MAX; // 上限饱和
    // RPM和n_max_rpm都是uint16_t，结果应该是正数。
    // 如果rpm为0，temp_speed也为0。
    if (temp_speed < 0 && rpm > 0) temp_speed = 0; // 理论上不应发生，但作为保护
    else if (temp_speed < 0) temp_speed = 0;       // 确保非负

    return (Frac16_t)temp_speed; // 转换为Frac16_t并返回
}

/**
 * @brief 将Frac16_t归一化速度值转换为RPM值。
 * @param speed_frac16 Frac16_t 归一化速度。
 * @param n_max_rpm 电机额定最大RPM (对应Frac16_t的1.0)。
 * @return uint16_t RPM值。
 */
uint16_t Derating_ConvertFrac16ToRpm(Frac16_t speed_frac16, uint16_t n_max_rpm) {
    if (n_max_rpm == 0) return 0; // 如果最大RPM为0，则返回0
    if (speed_frac16 < 0) speed_frac16 = 0; // RPM不能为负，将负的Frac16值视为0

    // 计算RPM: (speed_frac16 * n_max_rpm) / 32767
    // 使用32位中间变量进行计算
    int32_t rpm_temp = ((int32_t)speed_frac16 * n_max_rpm) / 32767L;

    if (rpm_temp < 0) rpm_temp = 0; // 确保RPM结果非负
    if (rpm_temp > UINT16_MAX) rpm_temp = UINT16_MAX; // 确保RPM结果在uint16_t范围内，上限饱和
    return (uint16_t)rpm_temp; // 转换为uint16_t并返回
}

/**
 * @brief 初始化降额控制器。
 * @param controller 指向 Derating_Controller_t 结构体的指针。
 * @param config 指向 Derating_Config_t 配置表的指针。
 */
void Derating_Init(Derating_Controller_t *controller, const Derating_Config_t *config) {
    controller->config = config; // 保存配置表的指针
    controller->current_temp_derating_idx = NO_DERATING_IDX; // 初始化当前温度降额索引为无降额
    controller->current_volt_derating_idx = NO_DERATING_IDX; // 初始化当前电压降额索引为无降额
    
    
    // 初始化所有计数器为0
    for (uint8_t i = 0; i < MAX_DERATING_POINTS; ++i) {
        controller->temp_entry_counter[i] = 0;
        controller->temp_exit_counter[i] = 0;
        controller->volt_entry_counter[i] = 0;
        controller->volt_exit_counter[i] = 0;
    }
    
    if (config != NULL) {
        controller->current_max_allowed_rpm = config->default_max_rpm;
    } else {
        controller->current_max_allowed_rpm = 3000; // 备用值
    }
}

/**
 * @brief 计算当前条件下允许的最大RPM（考虑降额）。
 * @param controller 指向 Derating_Controller_t 结构体的指针。
 * @param current_temp_celsius 当前温度。
 * @param current_vdc_filt_frac16 当前母线电压 (Frac16_t)。
 * @return uint16_t 允许的最大RPM。
 */
uint16_t Derating_GetMaxAllowedRpm(Derating_Controller_t *controller, int16_t current_temp_celsius, Frac16_t current_vdc_filt_frac16) {
    if (!controller || !controller->config) {
        return 3000; // 备用值
    }
    static uint16_t max_volt_rpm_last = PWM_MAX_RPM; //上一次计算电压降额后的最大允许RPM，第一次使用默认最大转速

    uint16_t max_rpm_limit = controller->config->default_max_rpm;
    uint16_t rpm_limit_temp = controller->config->default_max_rpm;
    uint16_t rpm_limit_volt =  max_volt_rpm_last;  // 初始值设置为上一次计算电压降额后的最大允许RPM

#if ENABLE_TEMP_DERATING == 1
    // --- 温度降额逻辑 ---
    uint8_t new_temp_idx = NO_DERATING_IDX;
    if (controller->config->num_temp_points > 0) {
        // 重置未使用的计数器
        for (uint8_t i = 0; i < controller->config->num_temp_points; ++i) {
            // 如果温度低于激活阈值，重置进入计数器
            if (current_temp_celsius < controller->config->temp_points_table[i].activate_threshold) {
                controller->temp_entry_counter[i] = 0;
            }
            // 如果温度高于失活阈值，重置退出计数器
            if (current_temp_celsius >= controller->config->temp_points_table[i].deactivate_threshold) {
                controller->temp_exit_counter[i] = 0;
            }
        }
        
        // 更新计数器并检查是否需要进入降额
        for (uint8_t i = 0; i < controller->config->num_temp_points; ++i) {
            if (current_temp_celsius >= controller->config->temp_points_table[i].activate_threshold) {
                // 增加进入计数器
                if (controller->temp_entry_counter[i] < DERATING_ENTRY_COUNT) {
                    controller->temp_entry_counter[i]++;
                }
                
                // 如果进入计数器达到阈值，激活这个降额等级
                if (controller->temp_entry_counter[i] >= DERATING_ENTRY_COUNT) {
                    new_temp_idx = i;
                }
            }
        }
        
        // 检查是否需要退出当前降额
        if (controller->current_temp_derating_idx != NO_DERATING_IDX) {
            if (current_temp_celsius < controller->config->temp_points_table[controller->current_temp_derating_idx].deactivate_threshold) {
                // 增加退出计数器
                if (controller->temp_exit_counter[controller->current_temp_derating_idx] < DERATING_EXIT_COUNT) {
                    controller->temp_exit_counter[controller->current_temp_derating_idx]++;
                }
                
                // 如果退出计数器达到阈值，切换到新的索引或无降额
                if (controller->temp_exit_counter[controller->current_temp_derating_idx] >= DERATING_EXIT_COUNT) {
                    controller->current_temp_derating_idx = new_temp_idx;
                }
            } else {
                // 如果温度高于失活阈值，重置退出计数器
                controller->temp_exit_counter[controller->current_temp_derating_idx] = 0;
                
                // 如果有更高级别的降额需要激活
                if (new_temp_idx > controller->current_temp_derating_idx) {
                    controller->current_temp_derating_idx = new_temp_idx;
                }
            }
        } else if (new_temp_idx != NO_DERATING_IDX) {
            // 如果当前无降额但需要激活新的降额
            controller->current_temp_derating_idx = new_temp_idx;
        }

        if (controller->current_temp_derating_idx != NO_DERATING_IDX) {
            rpm_limit_temp = controller->config->temp_points_table[controller->current_temp_derating_idx].max_rpm;
        }
    }
    max_rpm_limit = MIN(max_rpm_limit, rpm_limit_temp);
#endif // ENABLE_TEMP_DERATING == 1

#if ENABLE_VOLT_DERATING == 1
    // --- 电压降额逻辑（未使用全局降额逻辑） ---

    static uint16_t enter_time_Cnt=0; //进入降额状态计数
    static uint16_t exit_timemin_Cnt=0; //退出降额<start电压状态计数
    static uint16_t exit_timemax_Cnt=0; //退出降额>end电压状态计数
    if(current_vdc_filt_frac16<FRAC16(LINE_VOLT_TOP_ENTER/V_FULL_SCALE)&&current_vdc_filt_frac16>FRAC16(LINE_VOLT_LOW_ENTER/V_FULL_SCALE)){
        if(enter_time_Cnt <= ENTER_TIME_DELAY){
		    enter_time_Cnt++;//电压在降额区间，进入降额状态计数加一
	        exit_timemin_Cnt=0;    
		    exit_timemax_Cnt=0;
        }

	} 

    if(enter_time_Cnt >= ENTER_TIME_DELAY){ //进入降额状态
        rpm_limit_volt = LINE_RPM_MIN + ((current_vdc_filt_frac16-FRAC16(LINE_VOLT_START/V_FULL_SCALE))*LINE_FACTOR_RPM_VOLT); //电压降额计算公式
        rpm_limit_volt = MIN(rpm_limit_volt, LINE_RPM_MAX); //限制最大允许RPM是否需要更新
        rpm_limit_volt = MAX(rpm_limit_volt, LINE_RPM_MIN); //限制最小允许RPM是否需要更新	
    }

    if((current_vdc_filt_frac16 <= FRAC16(LINE_VOLT_START/V_FULL_SCALE))){
        if(exit_timemin_Cnt<=EXIT_TIME_DELAY){   
            exit_timemin_Cnt++;//电压小于降额区间最低值，<min退出降额状态计数加一	
            enter_time_Cnt=0; //退出降额状态计数清零
        }

    }
    if((current_vdc_filt_frac16 >= FRAC16(LINE_VOLT_END/V_FULL_SCALE))){
        if(exit_timemax_Cnt<=EXIT_TIME_DELAY){
            exit_timemax_Cnt++;//电压大于降额区间最高值，>max退出降额状态计数加一
            enter_time_Cnt=0; //退出降额状态计数清零
        }

    }
		
	//<start电压退出线性降额状态
    if(exit_timemin_Cnt>=EXIT_TIME_DELAY){   
		rpm_limit_volt = LINE_RPM_MIN;  
		enter_time_Cnt=0;
		exit_timemax_Cnt=0;
	} 
    
	//>end电压退出线性降额状态
    if(exit_timemax_Cnt>=EXIT_TIME_DELAY){
		rpm_limit_volt = LINE_RPM_MAX;
		enter_time_Cnt=0;
		exit_timemin_Cnt=0;
	}

    // 限制最大允许RPM是否需要更新
    if(SAFE_SUBABS(rpm_limit_volt,max_volt_rpm_last)> SPEED_JUMP_VALUE){
        max_volt_rpm_last = rpm_limit_volt; //更新最大允许RPM
    }else{
        rpm_limit_volt = max_volt_rpm_last; //不更新最大允许RPM,等于上一次计算的最大允许RPM
        if(SAFE_SUBABS(rpm_limit_volt,LINE_RPM_MIN)<SPEED_JUMP_VALUE){
            rpm_limit_volt = LINE_RPM_MIN; //靠近最小允许RPM,限制为最小允许RPM
        }else if(SAFE_SUBABS(rpm_limit_volt,LINE_RPM_MAX)<SPEED_JUMP_VALUE){
            rpm_limit_volt = LINE_RPM_MAX; //靠近最大允许RPM,限制为最大允许RPM
        }

    }
    max_rpm_limit = MIN(max_rpm_limit, rpm_limit_volt);
#endif // ENABLE_VOLT_DERATING == 1
    
    controller->current_max_allowed_rpm = max_rpm_limit;
    return controller->current_max_allowed_rpm;
}

/**
 * @brief 根据当前温度和电压，应用降额逻辑到目标转速。
 * @param controller 指向 Derating_Controller_t 结构体的指针。
 * @param target_speed_req_frac16 用户请求的目标转速 (Frac16_t格式)。
 * @param current_temp_celsius 当前电机或环境温度 (摄氏度)。
 * @param current_vdc_filt_frac16 当前滤波后的母线电压 (Frac16_t格式, 归一化)。
 * @param n_max_rpm 电机的最大物理转速 (用于 Frac16_t 和 RPM 之间的转换)。
 * @return Frac16_t 经过降额限制后的目标转速。
 */
Frac16_t Derating_Apply(Derating_Controller_t *controller, Frac16_t target_speed_req_frac16, int16_t current_temp_celsius, Frac16_t current_vdc_filt_frac16, uint16_t n_max_rpm) {
    if (!controller || !controller->config) { // 检查控制器和配置是否有效
        return target_speed_req_frac16; // 如果无效，则不进行降额，直接返回原始请求速度
    }

    // 获取当前条件下，由降额逻辑决定的最大允许RPM
    uint16_t max_allowed_rpm_by_derating = Derating_GetMaxAllowedRpm(controller, current_temp_celsius, current_vdc_filt_frac16);
    
    // 将降额后的最大允许RPM转换为Frac16_t格式
    Frac16_t max_allowed_speed_frac16 = Derating_ConvertRpmToFrac16(max_allowed_rpm_by_derating, n_max_rpm);

    // 如果用户请求的速度超过了降额限制，则使用降额限制值
    if (target_speed_req_frac16 > max_allowed_speed_frac16) {
        return max_allowed_speed_frac16;
    } else {
        // 否则，使用用户请求的速度
        return target_speed_req_frac16;
    }
}