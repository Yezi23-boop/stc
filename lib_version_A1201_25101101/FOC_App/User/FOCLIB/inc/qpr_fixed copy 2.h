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
void qprFxpInit(tqprControllerFxp* qpr,
                Frac16_t kr, Frac16_t wc_inc, Frac16_t w0_inc,
                Frac16_t y_limit);

// 运行：errorK 为 Q15，newW0_inc 为新的 w0 离散角增量（Q15）；若与当前不同将实时更新系数
// 返回 yK（Q15）
Frac16_t qprFxpUpdate(tqprControllerFxp* qpr, Frac16_t errorK, Frac16_t newW0_inc);
void QPR_FXP_TestLite_Step(void);
#ifdef __cplusplus
}
#endif

#endif // QPR_FIXED_H