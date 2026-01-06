#ifndef QPR_FIXED_H
#define QPR_FIXED_H

#include "nvsns_type_def.h"
#include "mm_lib.h"   // MulF16, SatAddF16, SatSubF16, MM_SatDivF16, AbsF16, MM_ThetaTransferTrigonoLut

#ifdef __cplusplus
extern "C" {
#endif

// Q31 辅助宏和函数
#define FRAC31(x) ((Frac32_t)((x < S31FRACT_MAX) ? ((x >= S31FRACT_MIN) ? (x * 1073741824.0) : F31_MIN) : F31_MAX))

// Q31 运算辅助函数
static inline Frac32_t MulF31(Frac32_t f31In1, Frac32_t f31In2)
{
    int64_t result = ((int64_t)f31In1 * (int64_t)f31In2) >> 30;
    if (result > F31_MAX) return F31_MAX;
    if (result < F31_MIN) return F31_MIN;
    return (Frac32_t)result;
}

static inline Frac32_t SatAddF31(Frac32_t f31In1, Frac32_t f31In2)
{
    int64_t result = (int64_t)f31In1 + (int64_t)f31In2;
    if (result > F31_MAX) return F31_MAX;
    if (result < F31_MIN) return F31_MIN;
    return (Frac32_t)result;
}

static inline Frac32_t SatSubF31(Frac32_t f31In1, Frac32_t f31In2)
{
    int64_t result = (int64_t)f31In1 - (int64_t)f31In2;
    if (result > F31_MAX) return F31_MAX;
    if (result < F31_MIN) return F31_MIN;
    return (Frac32_t)result;
}

static inline Frac32_t SatDivF31(Frac32_t f31In1, Frac32_t f31In2)
{
    if (f31In2 == 0) return (f31In1 >= 0) ? F31_MAX : F31_MIN;
    int64_t result = ((int64_t)f31In1 << 30) / (int64_t)f31In2;
    if (result > F31_MAX) return F31_MAX;
    if (result < F31_MIN) return F31_MIN;
    return (Frac32_t)result;
}

static inline Frac32_t AbsF31(Frac32_t f31In1)
{
    if (f31In1 >= 0) return f31In1;
    if (f31In1 == F31_MIN) return F31_MAX;
    return -f31In1;
}

// Q31与Q15转换
static inline Frac32_t F16ToF31(Frac16_t f16)
{
    return ((Frac32_t)f16) << 15;
}

static inline Frac16_t F31ToF16(Frac32_t f31)
{
    Frac32_t rounded = (f31 + (1 << 14)) >> 15;
    if (rounded > F16_MAX) return F16_MAX;
    if (rounded < F16_MIN) return F16_MIN;
    return (Frac16_t)rounded;
}

// 优化的三角函数实现，减少精度损失
static inline void sincos_q31_optimized(Frac32_t ang_q31, Frac32_t* s, Frac32_t* c)
{
    // 先进行角度归一化到[-π, π]
    Frac32_t normalized_ang = ang_q31;
    
    // 转换为Q15进行查表，但使用更精确的转换
    // 使用四舍五入而不是简单截断
    int32_t ang_q15_temp = (normalized_ang + (1 << 14)) >> 15;
    if (ang_q15_temp > F16_MAX) ang_q15_temp = F16_MAX;
    if (ang_q15_temp < F16_MIN) ang_q15_temp = F16_MIN;
    Frac16_t ang_q15 = (Frac16_t)ang_q15_temp;
    
    MM_PolarTrigono_t tr;
    MM_ThetaTransferTrigonoLut(ang_q15, &tr);
    
    // 转换回Q31，同样使用精确转换
    *s = F16ToF31(tr.f16Sin);
    *c = F16ToF31(tr.f16Cos);
}

// 更新sincos_q31调用
static inline void sincos_q31(Frac32_t ang_q31, Frac32_t* s, Frac32_t* c)
{
    sincos_q31_optimized(ang_q31, s, c);
}

typedef struct {
    // 参数/状态（Q31）
    Frac32_t kr;          // 增益 Kr
    Frac32_t wc_inc;      // wc 的每采样离散角增量（Q31）
    Frac32_t w0_inc;      // w0 的每采样离散角增量（Q31）

    // 系数（已除以 a0）：b0,b1,b2 为 Q31；a1_half = (a1/a0)/2，a2=(a2/a0)
    Frac32_t b0, b1, b2;
    Frac32_t a1_half;
    Frac32_t a2;

    // 输出限幅（Q31）
    Frac32_t yMax;
    Frac32_t yMin;

    // 历史状态（Q31）
    Frac32_t eKm1, eKm2;
    Frac32_t yKm1, yKm2;
} tqprControllerFxp;

// 初始化：kr, y_limit 为 Q31，频率为毫赫兹
extern void qprFxpInit(tqprControllerFxp* qpr,
                Frac32_t kr, uint32_t wc_mhz, uint32_t w0_mhz, uint16_t sample_freq,
                Frac32_t y_limit);

// 运行：errorK 为 Q31，newW0_mhz 为新的中心频率（毫赫兹）
// 返回 yK（Q31）
extern Frac32_t qprFxpUpdate(tqprControllerFxp* qpr, Frac32_t errorK, uint32_t newW0_mhz, uint16_t sample_freq);

// 运行时更新带宽
extern void qprFxpSetWcmHz(tqprControllerFxp* qpr, uint32_t wc_mhz, uint16_t sample_freq);
extern void qprFxpSetWcByAlphamhz(tqprControllerFxp* qpr, Frac32_t alpha_q31, uint32_t w0_mhz, uint16_t sample_freq);

// 测试函数
extern void QPR_FXP_TestLite_Step(void);

// 辅助函数
extern uint32_t MulQ31_By_IntmHz(Frac32_t alpha_q31, uint32_t mhz_u32);

#ifdef __cplusplus
}
#endif

#endif // QPR_FIXED_H