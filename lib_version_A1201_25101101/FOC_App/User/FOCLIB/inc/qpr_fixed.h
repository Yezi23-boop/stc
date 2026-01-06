#ifndef QPR_FIXED_H
#define QPR_FIXED_H

#include "nvsns_type_def.h"
#include "mm_lib.h"   // MulF16, SatAddF16, SatSubF16, MM_SatDivF16, AbsF16, MM_ThetaTransferTrigonoLut

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // 参数/状态（Q15）
    Frac16_t kr;          // 增益 Kr
    Frac16_t wc_inc;      // wc 的每采样离散角增量（Q15）
    Frac16_t w0_inc;      // w0 的每采样离散角增量（Q15）

    // 系数（已除以 a0）：b0,b1,b2 为 Q15；a1_half = (a1/a0)/2，a2=(a2/a0)
    Frac16_t b0, b1, b2;
    Frac16_t a1_half;
    Frac16_t a2;

    // 输出限幅（Q15）
    Frac16_t yMax;
    Frac16_t yMin;

    // 历史状态（Q15）
    Frac16_t eKm1, eKm2;
    Frac16_t yKm1, yKm2;
} tqprControllerFxp;

// 初始化：kr, wc_inc, w0_inc, y_limit 为 Q15（y 限幅为 ±y_limit）
extern void qprFxpInit(tqprControllerFxp* qpr,
                Frac16_t kr, uint32_t wc_mhz, uint32_t w0_mhz, uint16_t sample_freq,
                Frac16_t y_limit);

// 运行：errorK 为 Q15，newW0_inc 为新的 w0 离散角增量（Q15）；若与当前不同将实时更新系数
// 返回 yK（Q15）
/******************************************************************************
 * QPR（正交相位调节器）控制器 定点数(Q15)版本接口声明
 * 适用场景：FOC电机控制、逆变器/并网变流器的相位/频率跟踪、电网同步等
 * 注：Frac16_t为Q15格式定点数（16位有符号数，最高位符号位，低15位小数位，取值范围[-1, 1-1/32768]）
 ******************************************************************************/

/**
 * @brief QPR控制器核心更新函数
 * @details 输入相位/频率误差值和基准角频率，执行QPR控制器一次迭代计算，返回控制器输出
 * @param[in,out] qpr 指向QPR控制器句柄的指针（需先完成初始化，非NULL）
 * @param[in] errorK 相位/频率误差值（Q15定点数，归一化处理后）
 * @param[in] newW0_mhz 基准角频率（单位：mHz，毫赫兹，扩大1000倍避免小数运算）
 * @param[in] sample_freq 控制器采样频率（单位：Hz，如FOC常用10000Hz）
 * @return Frac16_t QPR控制器输出值（Q15定点数，需转换为实际控制量后使用）
 */
extern Frac16_t  qprFxpUpdate(tqprControllerFxp* qpr, Frac16_t errorK, uint32_t newW0_mhz, uint16_t sample_freq);

/**
 * @brief QPR控制器轻量级测试函数
 * @details 无参数/无返回值的测试接口，用于快速验证QPR算法逻辑正确性
 * @note 调试阶段使用：初始化QPR后调用该函数，可验证控制器基本功能（如相位跟踪、频率响应），无需外接输入
 */
extern void QPR_FXP_TestLite_Step(void);

/**
 * @brief Q15定点数与16位整数(Hz)乘法运算
 * @details 处理Q15定点数(α)与整数频率值(Hz)的乘法，内部做溢出保护和定点数移位，返回Q15格式结果
 * @param[in] alpha_q15 比例系数（Q15定点数，通常为0~1之间的系数，如0.1对应Q15值3277）
 * @param[in] hz_u16 频率值（单位：Hz，如电网50Hz、FOC载波1000Hz）
 * @return Frac16_t 运算结果（Q15定点数，公式：α × hz_u16，结果超出范围时会饱和）
 */
extern Frac16_t MulQ15_By_IntHz(Frac16_t alpha_q15, uint16_t hz_u16);

// 新增：按 α·w0（α 为 Q15）更新带宽，内部会把 α·w0(Hz) 转为 wc 并立刻重算
/**
 * @brief 按α·w0更新QPR控制器带宽（Hz版本）
 * @details 通过比例系数α和基准频率w0计算带宽wc（wc=α·w0），并立即重算QPR控制器内部参数
 * @param[in,out] qpr 指向QPR控制器句柄的指针（非NULL）
 * @param[in] alpha_q15 比例系数（Q15定点数，用于调整带宽倍数，如0.2对应Q15值6554）
 * @param[in] w0_hz 基准频率（单位：Hz，如电网50Hz、电机基波频率）
 * @param[in] sample_freq 控制器采样频率（单位：Hz，需与qprFxpUpdate的采样频率一致）
 * @note 内部自动将α·w0(Hz)转换为控制器带宽wc，并更新QPR的核心参数（如谐振系数）
 */
extern void qprFxpSetWcByAlpha(tqprControllerFxp* qpr, Frac16_t alpha_q15, uint16_t w0_hz, uint16_t sample_freq);

/**
 * @brief Q15定点数与32位整数(mHz)乘法运算
 * @details 处理Q15定点数(α)与毫赫兹级频率值的乘法，返回32位整数结果（mHz），避免低频段精度损失
 * @param[in] alpha_q15 比例系数（Q15定点数，0~1之间）
 * @param[in] mhz_u32 频率值（单位：mHz，毫赫兹，如50Hz=50000mHz）
 * @return uint32_t 运算结果（单位：mHz，公式：α × mhz_u32，保留毫赫兹级精度）
 */
extern uint32_t MulQ15_By_IntmHz(Frac16_t alpha_q15, uint32_t mhz_u32);

/**
 * @brief 按α·w0更新QPR控制器带宽（mHz版本）
 * @details 与qprFxpSetWcByAlpha功能一致，区别是基准频率单位为mHz，适配低频段高精度场景
 * @param[in,out] qpr 指向QPR控制器句柄的指针（非NULL）
 * @param[in] alpha_q15 比例系数（Q15定点数）
 * @param[in] w0_mhz 基准频率（单位：mHz，毫赫兹，如50Hz=50000mHz）
 * @param[in] sample_freq 控制器采样频率（单位：Hz）
 * @note 适配低频场景：当w0较小时（如几Hz），用mHz可避免整数运算精度丢失
 */
extern void qprFxpSetWcByAlphamhz(tqprControllerFxp* qpr, Frac16_t alpha_q15, uint32_t w0_mhz, uint16_t sample_freq);
#ifdef __cplusplus
}
#endif

#endif // QPR_FIXED_H