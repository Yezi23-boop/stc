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
extern Frac16_t  qprFxpUpdate(tqprControllerFxp* qpr, Frac16_t errorK, uint32_t newW0_mhz, uint16_t sample_freq);
extern void QPR_FXP_TestLite_Step(void);
extern Frac16_t MulQ15_By_IntHz(Frac16_t alpha_q15, uint16_t hz_u16);
// 新增：按 α·w0（α 为 Q15）更新带宽，内部会把 α·w0(Hz) 转为 wc 并立刻重算
extern void qprFxpSetWcByAlpha(tqprControllerFxp* qpr, Frac16_t alpha_q15, uint16_t w0_hz, uint16_t sample_freq);
extern uint32_t MulQ15_By_IntmHz(Frac16_t alpha_q15, uint32_t mhz_u32);
extern void qprFxpSetWcByAlphamhz(tqprControllerFxp* qpr, Frac16_t alpha_q15, uint32_t w0_mhz, uint16_t sample_freq);
#ifdef __cplusplus
}
#endif

#endif // QPR_FIXED_H