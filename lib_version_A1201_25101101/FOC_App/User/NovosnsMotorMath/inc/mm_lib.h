#ifndef __MM_LIB_H__
#define __MM_LIB_H__
// motor math library
#include "stdint.h"

#define F16_SQRT_1_3 18919

typedef int16_t Frac16_t;
typedef int32_t Frac32_t;

extern const int16_t pTableSin60[];

#define STATIC_FORCEINLINE static __forceinline //__inline//
    
#define F31_MAX ((Frac32_t)0x3FFFFFFF)
#define F31_MIN ((Frac32_t)0xC0000000)
#define F16_MAX ((Frac16_t)32767)
#define F16_MIN ((Frac16_t)-32768)

/*! Constant representing the maximal negative value of a signed 16-bit fixed point fractional number, floating point
 * representation. */
#define S16FRACT_MIN  (-1.0)

/*! Constant representing the maximal positive value of a signed 16-bit fixed point fractional
 *  number,  floating point representation. */
#define S16FRACT_MAX  (0.999969482421875)

/*! Constant representing the maximal negative value of a signed 31-bit fixed point fractional number, floating point
 * representation. */
#define S31FRACT_MIN   (-1.0)
/*! Constant representing the maximal positive value of a signed 31-bit fixed point fractional
 *  number,  floating point representation. */
#define S31FRACT_MAX   (0.9999999990686774245171597455788)

/*! Macro converting a signed fractional [-1,1) number into a 16-bit fixed point number in format Q1.15.*/
#define FRAC16(x) ((Frac16_t)((x < S16FRACT_MAX) ? ((x >= S16FRACT_MIN) ? (x * 32768.0) : F16_MIN) : F16_MAX))

/*! Macro converting a signed fractional [-1,1) number into a 32-bit fixed point number in format Q2.30.*/
#define FRAC32(x) ((Frac32_t)((x < S31FRACT_MAX) ? ((x >= S31FRACT_MIN) ? (x * 1073741824.0) : F31_MIN) : F31_MAX))

typedef struct
{
    Frac16_t f16Sin;
    Frac16_t f16Cos;
}MM_PolarTrigono_t; // polar trigonometric

typedef struct {
    const Frac16_t f16A[3];
} MM_ATAN_TAYLOR_COEF_T_F16; 

void MM_ThetaTransferTrigonoLut(Frac16_t f16Theta, MM_PolarTrigono_t* const tTransfer);
Frac16_t MM_CalAngleAmp(Frac16_t f16inX, Frac16_t f16inY, uint16_t* pAmp);
Frac16_t MM_AtanXYF16(Frac16_t f16inX, Frac16_t f16inY);
Frac16_t MM_DivF16(Frac16_t f16In1, Frac16_t f16In2);
Frac16_t MM_SatDivF16(register Frac16_t f16In1, register Frac16_t f16In2);
Frac16_t MM_SqrtLut_F16(register Frac16_t f16In);
Frac16_t MM_QuartAtanF16(Frac16_t f16inX, Frac16_t f16inY);

STATIC_FORCEINLINE Frac16_t SignF16(Frac16_t f16In1)
{
    return ((f16In1 > 0) ? 1 : ((f16In1 < 0) ? -1 : 0));
}

/************************************************************
 * @brief: Abs function for F16 (symmetric saturation)
 * @param <Frac16_t> f16In1
 * @return <Frac16_t> |f16In1| with -32768 handled safely
 ************************************************************/
STATIC_FORCEINLINE Frac16_t AbsF16(Frac16_t f16In1)
{
    if (f16In1 >= 0) return f16In1;
    // 防止 -32768 取反溢出；按对称饱和返回 32767
    if (f16In1 == F16_MIN) return F16_MAX;
    return (Frac16_t)(-f16In1);
}

/************************************************************
 * @brief: Convert F31 into F16
 * @param <Frac32_t> f32In1
 * @return <None>
 ************************************************************/
STATIC_FORCEINLINE Frac16_t F32ConvertF16(Frac32_t f32In1) {return (Frac16_t)( (Frac32_t)f32In1 >> 15u);}

/************************************************************
 * @brief: Multiply without saturation and return F16
 * @param <Frac16_t> f16In1
 * @param <Frac16_t> f16In2
 * @return <None>
 ************************************************************/
STATIC_FORCEINLINE Frac16_t MulF16(Frac16_t f16In1, Frac16_t f16In2)
{
    return ((((int32_t)(f16In1)) * ((int32_t)(f16In2))) >> (15));
}

/************************************************************
 * @brief: Sub with Saturation in F16
 * @param <Frac16_t> f16In1
 * @param <Frac16_t> f16In2
 * @return <None>
 ************************************************************/
STATIC_FORCEINLINE Frac16_t SatSubF16(Frac16_t f16In1, Frac16_t f16In2)
{
    return __ssat((((f16In1) - (f16In2))), 16);
}

/************************************************************
 * @brief: Add with saturation in F16
 * @param <Frac16_t> f16In1
 * @param <Frac16_t> f16In2
 * @return sum in saturation
 ************************************************************/
STATIC_FORCEINLINE Frac16_t SatAddF16(Frac16_t f16In1, Frac16_t f16In2)
{
    return __ssat(((f16In1) + (f16In2)), 16);
}



#endif
