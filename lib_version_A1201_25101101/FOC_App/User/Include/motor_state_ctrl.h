#ifndef MOTRO_STATE_CTRL_H_
#define MOTRO_STATE_CTRL_H_

#ifdef __cplusplus
extern "C"
{

#endif
//滤波深度
#define PHASE_CURRENT_MAX_FILTER_NPOINT 10
#define COM_TYPE_CONFIRM_CNT 500 //通讯方式确认计数

extern void MotorStateCtrl(void);
extern	void getMotorCmdSpeed(void);
extern void GetUMaxCurrent(int16_t i16Current);
extern void GetVMaxCurrent(int16_t i16Current);
extern void GetWMaxCurrent(int16_t i16Current);

extern uint8_t g_u8ComType;
extern uint16_t g_u16ComTypeConfirmCnt; //通讯方式确认计数

extern Frac16_t f16PhaseCurrentMax[3];
extern GCF_Filter1_LPF16_t tPhaseCurrentMaxFilter[3]; 
extern Frac16_t f16PhaseCurrentMaxFilter[3]; //相电流幅值滤波
extern Frac16_t f16PhaseCurrentRms[3];
//-----------------


#ifdef __cplusplus
}
#endif



#endif


