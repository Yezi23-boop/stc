#include "zf_common_headfile.h"
#include "math.h"
#define AD_VAL_MAX 4000 // 电感最大值上限
#define NUM 4           // 电感数量4
#define SORT_LENGTH 4   // 排序数组长度

/*********************************************
             电感排布
              五电感

    —     l       l     —

   ad1   ad2     ad3   ad4

 *********************************************/
float AD_value[NUM][SORT_LENGTH] = {{0}};                                      // 存储电感原始值的二维数组
float adtemp;                                                                  // 排序临时变量
float ad_sum[NUM] = {0};                                                       // 电感值求和
float ad_ave[NUM] = {0};                                                       // 电感值平均值
float AD_V[NUM], AD_last[NUM] = {0};                                           // 电感值处理后的结果
float RAW[NUM];                                                                // 电感原始值
float ad1 = 0;                                                                 // 第1个电感值（归一化后）
float ad2 = 0;                                                                 // 第2个电感值（归一化后）
float ad3 = 0;                                                                 // 第6个电感值（归一化后）
float ad4 = 0;                                                                 // 第7个电感值（归一化后）
float MAX[NUM] = {AD_VAL_MAX, AD_VAL_MAX, AD_VAL_MAX, AD_VAL_MAX}; // 各电感最大值
float MA[NUM] = {0};                                                           // 扫描记录的最大值
int Err = 0;                                                                 // 当前偏差值
float Err_tow = 0;                                                             // 双电感偏差值
int limit = 100;                                                               // 偏差限幅
float left = 0;                                                                // 左侧电感和
float right = 0;                                                               // 右侧电感和
float diff = 1;                                                                // 左右电感差异系数
void dispose(void)
{
    // 方式一
//    diff = func_abs((ad2 - ad3) / Err_diff) + 1;
    Err = (int)(diff * limit * (sqrt(ad1 * ad1 + ad2 * ad2) - sqrt(ad4 * ad4 + ad3 * ad3)) / (sqrt(ad1 * ad1 + ad2 * ad2) + sqrt(ad4 * ad4 + ad3 * ad3)));

    // 方式二
    // left = Err_diff * ad1 + (1 - Err_diff) * ad2;
    // right = Err_diff * ad4 + (1 - Err_diff) * ad3;
    // Err_tow = limit * (left - right) / (left + right);
    Err_tow = limit * (A_1 * (ad1 - ad4) + B_1 * (ad2 - ad3)) / (A_1 * (ad1 + ad4) + C_l * func_abs(ad2 - ad3));
}

/**
 * @brief 扫描赛道获取电感最大值
 * @details 获取各个电感的当前值，并更新记录的最大值
 * @note 用于电感标定和归一化计算的参考值
 */
void scan_track_max_value(void)
{
    int i = 0;
    RAW[0] = adc_convert(ADC_CH1_P11);  // 读取第1个电感AD值
    RAW[1] = adc_convert(ADC_CH0_P10);  // 读取第2个电感AD值
    RAW[2] = adc_convert(ADC_CH8_P00);  // 读取第6个电感AD值
    RAW[3] = adc_convert(ADC_CH9_P01); // 读取第7个电感AD值

    for (i = 0; i < NUM; i++)
    {
        // 如果当前值大于记录的最大值，则更新最大值
        if (RAW[i] > MA[i])
        {
            MA[i] = RAW[i];
        }
    }
}

/**
 * @brief 读取并处理电感数据
 * @details 多次采样并排序滤波，对电感值进行归一化处理并计算偏差
 * @note 核心电感处理函数，用于赛道识别和循迹控制
 */
void read_AD(void)
{
    int i, j, k;
    float AD_ONE[NUM] = {0.0}; // 归一化后的电感值（0-100）
    int min_idx = 0;

    // 1. 多次采样电感值
    for (i = 0; i < SORT_LENGTH; i++)
    {
        AD_value[0][i] = adc_convert(ADC_CH1_P11);  // 采集第1个电感
        AD_value[1][i] = adc_convert(ADC_CH0_P10);  // 采集第2个电感
        AD_value[2][i] = adc_convert(ADC_CH8_P00);  // 采集第6个电感
        AD_value[3][i] = adc_convert(ADC_CH9_P01); // 采集第7个电感
    }

    // 2. 对每个电感的采样值进行排序（选择排序算法）
    for (i = 0; i < NUM; i++)
    {
        for (j = 0; j < SORT_LENGTH - 1; j++)
        {
            min_idx = j;
            for (k = j + 1; k < SORT_LENGTH; k++)
            {
                if (AD_value[i][k] < AD_value[i][min_idx])
                {
                    min_idx = k;
                }
            }
            if (min_idx != j)
            {
                adtemp = AD_value[i][j];
                AD_value[i][j] = AD_value[i][min_idx];
                AD_value[i][min_idx] = adtemp;
            }
        }

        // 3. 中值计算 - 由于数组长度为4，取中间两个值(下标1和2)的平均值
        ad_sum[i] = AD_value[i][1] + AD_value[i][2];
        ad_ave[i] = ad_sum[i] / 2;
        AD_value[i][SORT_LENGTH - 1] = ad_ave[i];
    }

    memset(ad_sum, 0, sizeof(ad_sum));

    // 4. 计算每个电感的最终值并限幅
    for (i = 0; i < NUM; i++)
    {
        for (j = 0; j < SORT_LENGTH; j++)
        {
            ad_sum[i] += AD_value[i][j];
        }
        AD_V[i] = ad_sum[i] / SORT_LENGTH; // 求平均值
        RAW[i] = AD_V[i];                  // 保存原始AD值
        if (AD_V[i] > MAX[i])              // 限幅处理
        {
            AD_V[i] = MAX[i];
        }
    }

    // 5. 归一化处理（转换为0-100范围）
    for (i = 0; i < NUM; i++)
    {
        AD_ONE[i] = (100 * AD_V[i] / MAX[i]);
    }

    // 6. 将处理后的值赋给各个电感变量
    ad1 = AD_ONE[0];
    ad2 = AD_ONE[1];
    ad3 = AD_ONE[2];
    ad4 = AD_ONE[3];
    dispose(); // 处理电感数据，计算偏差值
}
float dianya=0;
void dianya_adc(void)
{
static int32 dianya_count=0;
dianya=adc_convert(ADC_CH13_P05)* 0.0092;
pwm_set_duty(PWMA_CH4N_P07, 0);
if(dianya<7.6)
{
dianya_count++;
}
if(dianya_count> 2000)
{
	dianya_count=0;
	pwm_set_duty(PWMA_CH4N_P07, 3000);
}
}