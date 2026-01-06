#ifndef DQ_DUAL_SPACE_SWITCH_H
#define DQ_DUAL_SPACE_SWITCH_H

#include "nvsns_foc.h"
#include "transform.h"

extern void DQ_DualSpaceSwitchInitUseAlphaBeta(Frac16_t theta_old, Frac16_t theta_new);
// 初始化双DQ坐标空间切换（从 theta_old -> theta_new）
// 会对当前给定和PI积分状态做等效变换，并启动Id过渡
extern void DQ_DualSpaceSwitchInit(Frac16_t theta_old, Frac16_t theta_new);

// 设置Id过渡步长（每次衰减量），默认值为 FRAC16(0.00005)
extern void DQ_DualSpaceSetBlendStep(Frac16_t step);

extern void calcIDendIQend(Frac16_t theta_old, Frac16_t theta_new);

// 获取初始Id（切换时计算得到的Id起点）
extern Frac16_t DQ_DualSpaceGetIdStart(void);
extern Frac16_t DQ_DualSpaceGetIqStart(void);



extern void DQ_DualSpaceSwitchMaintainTorque(CTD_2SystF16_t *pIdq_in, CTD_2SystF16_t *pIdq_out);

// 获取当前过渡系数alpha（[0,1]，>0表示仍在过渡中）
extern Frac16_t DQ_DualSpaceGetBlendAlpha(void);

// 衰减一次alpha，内部饱和到[0,1]
extern void DQ_DualSpaceDecayBlendAlpha(void);

// 复位过渡状态（alpha=0, IdStart=0）
extern void DQ_DualSpaceBlendReset(void);

extern void DQ_DualSpaceSwitchTransformVirDQ(CTD_2SystF16_t* const tDQ_in, CTD_2SystF16_t*  tDQ_out, Frac16_t theta_v ,Frac16_t theta_r);
extern void DQ_DualSpaceSwitchTransformRealDQ(CTD_2SystF16_t* const tDQ_in, CTD_2SystF16_t*  tDQ_out, Frac16_t theta_v ,Frac16_t theta_r);

extern Frac16_t s_f16IdIQend;
extern Frac16_t s_f16IdIQMaintain; 
    
#endif // DQ_DUAL_SPACE_SWITCH_H