/*
 * pi.h
 * 简单 PI 控制器 宏定义框架
 *
 * 说明：
 * - 提供 PI 控制器的数据结构及常用初始化、更新、复位、限幅等宏
 * - 以 float 为默认数据类型，适配嵌入式/普通应用
 * - 使用方法示例：
 *     PI_DECL(myPi);
 *     PI_INIT(myPi, 1.0f, 0.1f, -100.0f, 100.0f);
 *     PI_UPDATE(myPi, error, dt, output);
 */

#ifndef PI_H_INCLUDED
#define PI_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

/* 版本 */
#define PI_H_VERSION "1.0"

#include "gcf_lib.h"
#include "stdint.h"

extern Frac16_t GCF_PositionalPI_F16(Frac16_t f16InErr, GCF_CtrlPIAW_PTF16_t *const pParam);


extern void SetPIControllerBumpless(GCF_CtrlPIAW_PTF16_t *pPICtrl, Frac16_t f16TargetOutput, Frac16_t f16CurrentError);

#ifdef __cplusplus
}
#endif

#endif /* PI_H_INCLUDED */