

#include <stdint.h>
#include "curr_sample_1rps.h"


//测量矢量状态枚举
typedef enum {
    MEASURE_VECTOR_IDLE = 0,

    MEASURE_VECTOR_PREPARE = 1,     //准备
    MEASURE_VECTOR_RUNNING = 2,
    MEASURE_VECTOR_RESTORE = 3,
    MEASURE_VECTOR_DONE = 4,
    //计算
    MEASURE_VECTOR_CALCULATING = 5,
    //结束
    MEASURE_VECTOR_END  = 6
} MeasureVectorState;

// 新的辅助电流/角度/转速结构
struct CurrAuxInfo {
    int16_t                     AngleElectrical;    // 电气角
    int16_t                     NextAngleElectrical;// 下一个电气角
    int16_t                     SpeedRPM;           // 现在的转速
    int16_t                     Id;                 // D轴电流
    int16_t                     Iq;                 // Q轴电流
    bool                        useMeasureVector;
    // 极限工况下，只能采集一项时另一相电流的处理方式：0:当前值 1:历史维持 2:DQ反推
    int16_t                     SpecialMode;
    int16_t                     CompTime;        // DQ电流反推时的补偿时间
    Frac16_t                    f16switchSpeed;    // 切换到预测模式的速度阈值
    Frac16_t                    f16switchSpeedHysteresis;
    Frac16_t                    dqPredictActive;
};

extern uint16_t g_u16SampleSlot;
extern struct tCurrSample1RPS tCurrSample1rps;
extern uint8_t g_u8MeasureVectorFlag;
extern struct CurrAuxInfo tCurrAuxInfo;
extern uint8_t g_u8MeasureVectorDebugEnable;
extern void InitMotorParameterAIWIN(struct tCurrSample1RPS* p,struct CurrAuxInfo* pAux);
extern void Svm1rpsTrig_AIWIN(struct tCurrSample1RPS* p,struct CurrAuxInfo* pAux);
extern void GetPhaseCurr_1rps_AIWIN(struct tCurrSample1RPS* p, struct CurrAuxInfo* pAux);
extern void RestoreFromMeasureVectorConfig(void);
extern void MeasureVectorPeriod(void);


