/****************************************************************************
 * @file    : typedef.h
 * @author  : novosense
 * @version : V1.1.0
 * @date    : 03-05-2018
 * @brief   : A special header file containing self-define or sub-function 
              which are independent of any project                         
              2'nd header                                                        
 * @note
 * Copyright (C) 2019 novosense All rights reserved.
 ***************************************************************************/

// -------------------------------------------------------------------------//
// Define to prevent recursive inclusion
// -------------------------------------------------------------------------//
#ifndef __TYPEDEF_H__
#define __TYPEDEF_H__

#ifdef __cplusplus
extern "C" {
#endif
    
// -------------------------------------------------------------------------//
// Includes
// -------------------------------------------------------------------------//
#include <stdint.h>
#include <stdarg.h>
// math
#include <math.h>
// lib
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
    
// -------------------------------------------------------------------------//
// Typedef
// -------------------------------------------------------------------------//
#define BIT(B)                      (1UL << (B))
#define CLR_BIT(REG, BIT)           ((REG) &= ~(BIT))
#define SET_BIT(REG, BIT)           ((REG) |= (BIT))
#define READ_BIT(REG, BIT)          ((REG) & (BIT))
#define CLR_REG(REG)                ((REG) =  0x0)
#define WRITE_REG(REG, VAL)         ((REG) =  VAL)
#define READ_REG(REG)               ((REG)       )
#define MODIFY_REG(REG, CLEARMASK, SETMASK)     WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define LL_UNUSED(X)        (void)X      /* To avoid gcc/g++ warnings */
  
#define POSITION_VAL(VAL)           (__CLZ(__RBIT(VAL))) 

#define IC_READ_REG(ADDR)           (*((__IO uint32_t *)(ADDR)))   
#define IC_WRITE_REG(ADDR, VAL)     (*((__IO uint32_t *)(ADDR)) = (VAL))
#define IC_MODIFY_REG(ADDR, CLEARMASK, SETMASK)   (*((__IO uint32_t *)(ADDR)) = IC_READ_REG(ADDR) & (~(CLEARMASK)) | (SETMASK))

#define MIN(A, B)                   (((A) < (B)) ? (A) : (B))
#define MAX(A, B)                   (((A) > (B)) ? (A) : (B))

#define ALIGN_UP(x, align)         (((uint32_t)(x) + (align - 1))  & (~(align - 1)))
#define ALIGN_DOWN(x, align)       (((uint32_t)(x)              )  & (~(align - 1)))
#define ALIGN_CHK(x, align)        (((uint32_t)(x) & (align - 1)) == (0x00000000UL))

// -------------------------------------------------------------------------//
// ASM Keyword
// -------------------------------------------------------------------------//
#define __ASM           __asm           // ASM keyword for ARM Compiler
#define __INLINE        __inline        // Inline keyword for ARM Compiler
#define __STATIC_INLINE static __inline

// -------------------------------------------------------------------------//
// Exported_types
// -------------------------------------------------------------------------//
#ifdef __cplusplus
  #define   __I     volatile            // Defines 'read only' permissions
#else
  #define   __I     volatile const      // Defines 'read only' permissions
#endif
#define     __O     volatile            // Defines 'write only' permissions
#define     __IO    volatile            // Defines 'read/write' permissions

// -------------------------------------------------------------------------//
// Standard Peripheral Library old types
// -------------------------------------------------------------------------//
typedef int32_t         s32;
typedef int16_t         s16;
typedef int8_t          s8;

typedef const int32_t   sc32;          
typedef const int16_t   sc16;           
typedef const int8_t    sc8;           

typedef __IO int32_t    vs32;
typedef __IO int16_t    vs16;
typedef __IO int8_t     vs8;

typedef __I int32_t     vsc32;         
typedef __I int16_t     vsc16;         
typedef __I int8_t      vsc8;          

typedef uint32_t        u32;
typedef uint16_t        u16;
typedef uint8_t         u8;

typedef const uint32_t  uc32;        
typedef const uint16_t  uc16;        
typedef const uint8_t   uc8;          

typedef __IO uint32_t   vu32;
typedef __IO uint16_t   vu16;
typedef __IO uint8_t    vu8;

typedef __I uint32_t    vuc32;         
typedef __I uint16_t    vuc16;         
typedef __I uint8_t     vuc8;           

typedef enum {false = 0, true = 1} bool, bool;
//typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState, FnState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

/**
  * @brief  LL Status structures definition
  */
typedef enum
{
    LL_OK       = 0x00U,
    LL_ERROR    = 0x01U,
    LL_BUSY     = 0x02U,
    LL_TIMEOUT  = 0x03U
} LL_StatusTypeDef;


#endif /* __TYPEDEF_H__ */


/********************************END OF FILE*********************************/


