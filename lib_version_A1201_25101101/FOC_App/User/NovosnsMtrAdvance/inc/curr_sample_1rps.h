#include <stdint.h>
#include "mm_lib.h"
#include "typedef.h"

/**
 * @brief 相电流/占空比信息模板结构体
 * @details 存储电机三相电流采样值、历史值、偏移量、PWM时间参数、相占空比等核心采样/控制参数
 */
struct IUInfoTemplate {
    int16_t     Iu;             /*!< U相电流采样值，16位有符号整型，原始ADC采样值（未校准） */
    int16_t     Iv;             /*!< V相电流采样值，16位有符号整型，原始ADC采样值（未校准） */
    int16_t     Iw;             /*!< W相电流采样值，16位有符号整型，原始ADC采样值（未校准） */

    int16_t     IuPrev;         /*!< U相电流上一采样周期值，16位有符号整型，历史状态变量 */
    int16_t     IvPrev;         /*!< V相电流上一采样周期值，16位有符号整型，历史状态变量 */
    int16_t     IwPrev;         /*!< W相电流上一采样周期值，16位有符号整型，历史状态变量 */

    int16_t     IuOffset;       /*!< U相电流采样偏移量，16位有符号整型，用于ADC零漂校准 */
    int16_t     IvOffset;       /*!< V相电流采样偏移量，16位有符号整型，用于ADC零漂校准 */

    int16_t     TP1;            /*!< PWM时间参数1，16位有符号整型，用于电流采样时刻标定 */
    int16_t     TP2;            /*!< PWM时间参数2，16位有符号整型，用于电流采样时刻标定 */

    int16_t     DtyPhaseU;      /*!< U相PWM占空比，16位有符号整型，标幺值（0~32767对应0~100%） */
    int16_t     DtyPhaseV;      /*!< V相PWM占空比，16位有符号整型，标幺值（0~32767对应0~100%） */
    int16_t     DtyPhaseW;      /*!< W相PWM占空比，16位有符号整型，标幺值（0~32767对应0~100%） */
};

/**
 * @brief 电流重构序列数据结构体
 * @details 存储电流重构所需的序列数据（参考值/索引），用于单电阻/双电阻电流采样重构三相电流
 */
struct SeqData{
    int32_t     DataArr[3][2];     /*!< 当前周期电流重构数据数组，3行2列：3对应三相，2对应索引/参考值 */
    int32_t     DataArrPrev[3][2]; /*!< 上一周期电流重构数据数组，历史状态变量，用于数据平滑 */
    #define     VALUE_REF 1        /*!< 数据数组中参考值的列索引（第2列） */
    #define     INDEX_REF 0        /*!< 数据数组中索引值的列索引（第1列） */
};

/**
 * @brief 电流重构控制结构体
 * @details 单/双电阻电流采样重构的核心参数结构体，包含重构序列数据、电流偏移量、状态变量等
 */
struct CurReconStruct
{
    struct       SeqData UVWData;   /*!< 三相电流重构序列数据结构体，存储重构所需的索引/参考值 */
    int16_t      IuOffset;          /*!< U相电流重构偏移量，16位有符号整型，补充校准用 */
    int16_t      IvOffset;          /*!< V相电流重构偏移量，16位有符号整型，补充校准用 */
    int16_t      IwOffset;          /*!< W相电流重构偏移量，16位有符号整型，补充校准用 */
    int16_t      I0Temp;            /*!< 电流重构临时变量0，16位有符号整型，存储中间运算值 */
    int16_t      I1Temp;            /*!< 电流重构临时变量1，16位有符号整型，存储中间运算值 */
    int16_t      I0Temp0;           /*!< 电流重构临时变量0（备份），16位有符号整型 */
    int16_t      I1Temp0;           /*!< 电流重构临时变量1（备份），16位有符号整型 */
    int16_t      SSMode;            /*!< 电流采样模式，16位有符号整型：0-单电阻，1-双电阻，2-三电阻 */
    int16_t      SSModePre;         /*!< 上一周期电流采样模式，16位有符号整型，防止模式突变 */
    int16_t      t1, t2, t3, t4, t5, t6, t7; /*!< 电流重构时间参数，16位有符号整型，用于采样时刻精准匹配 */
};

/**
 * @brief 电机控制参考值结构体
 * @details 存储电机FOC控制的核心参考指令（电流/电压/占空比/转速）
 */
struct Reference {            
    int16_t CurrentRef;     /*!< 电流参考指令，16位有符号整型，标幺值（对应dq轴电流指令） */
    int16_t VqRef;          /*!< Q轴电压参考指令，16位有符号整型，标幺值（0~32767对应母线电压） */
    int16_t DutyRef;        /*!< PWM占空比参考指令，16位有符号整型，标幺值（0~32767对应0~100%） */
    int16_t SpeedRPMRef;    /*!< 转速参考指令，16位有符号整型，单位：RPM（转/分钟） */
};

/**
 * @brief 每转1次采样的电流控制结构体
 * @details 整合电机控制参考值、电流重构、相电流/占空比信息的顶层结构体，用于1RPS（每转1次）采样控制
 */
struct tCurrSample1RPS{ 
    struct Reference 						RefPU ;         /*!< 电机控制参考值结构体，存储电流/电压/占空比/转速指令 */
    struct CurReconStruct       UVWCurrRecon;  /*!< Struct form current sensing：三相电流重构结构体，单/双电阻采样核心 */
    struct IUInfoTemplate       Signal;        /*!< Internal signals: IU/IV/IW, ID/IQ, VD/VQ：内部信号结构体，存储相电流、占空比等 */
} ;

// 全局变量声明
extern uint16_t g_u16SampleSlot;               /*!< 采样时隙标记，16位无符号整型，用于标记电流采样的PWM时隙 */
extern struct tCurrSample1RPS tCurrSample1rps; /*!< 每转1次采样的电流控制全局结构体，存储所有采样/控制参数 */

// 函数声明注释
/**
 * @brief 1RPS模式下SVPWM触发函数
 * @details 每转1次采样模式下，触发SVPWM生成并配置电流采样时刻
 * @param[in,out] p: 指向每转1次采样控制结构体的指针，输入参考值，输出PWM配置参数
 * @return 无
 */
extern void Svm1rpsTrig(struct tCurrSample1RPS* p);

/**
 * @brief 1RPS模式下相电流获取函数
 * @details 每转1次采样模式下，读取ADC采样值并通过电流重构得到三相电流
 * @param[in,out] p: 指向每转1次采样控制结构体的指针，输入重构参数，输出三相电流采样值
 * @return 无
 */
extern void GetPhaseCurr_1rps(struct tCurrSample1RPS* p);

/**
 * @brief PWM占空比设置函数
 * @details 根据控制指令设置三相PWM占空比，更新到硬件寄存器
 * @param[in,out] p: 指向每转1次采样控制结构体的指针，输入占空比参考值，输出实际配置的占空比
 * @return 无
 */
extern void SetPwmDuty(struct tCurrSample1RPS* p);

/**
 * @brief 电机参数初始化函数
 * @details 初始化电流采样偏移量、重构参数、参考值等电机控制核心参数
 * @param[in,out] p: 指向每转1次采样控制结构体的指针，输出初始化后的参数
 * @return 无
 */
extern void InitMotorParameter(struct tCurrSample1RPS* p);

/**
 * @brief ADC校准函数
 * @details 校准电流采样ADC的零漂和增益，更新电流偏移量参数
 * @return 无
 */
extern void ADC_Calibration(void);