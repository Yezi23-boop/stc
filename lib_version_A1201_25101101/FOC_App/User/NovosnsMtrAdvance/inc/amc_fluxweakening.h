#ifndef FIELD_WEAKENING_H
#define FIELD_WEAKENING_H

/****************************************************************************
* Includes
****************************************************************************/
#include "amc_lib.h"
#include "gmc_lib.h"
#include "mm_lib.h"

/**
 * @brief 永磁同步电机（PMSM）弱磁控制结构体
 * @details 用于电机基速以上的弱磁扩速控制，通过调节d轴电流实现母线电压利用率最大化，
 *          提升电机最高转速，包含弱磁角度滤波、PI调节器、电流/电压限幅等核心参数
 */
typedef struct
{
	GCF_Filter1_LPF16_t pFilterFW;  /**< Field weakening angle FilterMA paremeters structure.：弱磁角度低通滤波器结构体，用于平滑弱磁角度指令，降低高频波动 */
	GCF_CtrlPIAW_PTF16_t pPIpAWFW;  /**< Field weakening angle ControllerPIpAW paremeters structure.：弱磁角度PI调节器结构体，用于闭环调节弱磁角度，实现精准弱磁 */
	Frac16_t *pIQFbck;  /**< Pointer to the q-axis component of the feedback current in the two-phase rotational orthogonal system (d-q).：指向d-q旋转坐标系Q轴反馈电流的指针，16位定点数，弱磁控制的电流反馈源 */
	Frac16_t *pUQReq;  /**< Pointer to the q-axis component of the required voltage in the two-phase rotational orthogonal system (d-q).：指向d-q旋转坐标系Q轴指令电压的指针，16位定点数，弱磁控制的电压参考源 */
	Frac16_t *pUQLim;  /**< Pointer to the q-axis voltage controller upper limit. Must be a positive value.：指向Q轴电压控制器上限的指针，16位定点数，需为正值，限制弱磁时Q轴最大输出电压 */
	Frac16_t pIQLim;   /**< Q轴电流限幅值，16位定点数，限制弱磁扩速时Q轴最大电流，防止电机过流 */
	//_VsMargin弱磁方式的电压门槛值 母线电压百分比
	Frac16_t pVsMargin;/**< VsMargin弱磁策略的电压门槛值，16位定点数，单位：母线电压百分比（0~1），触发弱磁的电压阈值 */
}AMC_FluxWeakening_t;


void AMC_FluxWeakening_F16(Frac16_t f16IDQReqAmp, Frac16_t f16VelocityFbck, CTD_2SystF16_t *const pIDQReq, AMC_FluxWeakening_t *pCtrl);
#endif

