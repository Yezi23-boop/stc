/****************************************************************************
 * @file    : NSUC1602.h
 * @author  : novosense
 * @version : V2.0
 * @Date    : 2023/9/30 16:14
 * @brief   : register define header file
 * @note
 * Copyright (C) 2023 novosense All rights reserved.
 ****************************************************************************/
#ifndef __NSUC1602_H
#define __NSUC1602_H


#include <stdint.h>

#define SYSCTRL_UNLOCK_KEY            0x87e4
#define __XTAL            (48000000UL)    /* Oscillator frequency             */
/*****************************************************************************/
/** @addtogroup Bit-Definitions
 *  @{ */
/*****************************************************************************/
#define BIT_31    (0x80000000UL)
#define BIT_30    (0x40000000UL)
#define BIT_29    (0x20000000UL)
#define BIT_28    (0x10000000UL)
#define BIT_27    (0x08000000UL)
#define BIT_26    (0x04000000UL)
#define BIT_25    (0x02000000UL)
#define BIT_24    (0x01000000UL)

#define BIT_23    (0x00800000UL)
#define BIT_22    (0x00400000UL)
#define BIT_21    (0x00200000UL)
#define BIT_20    (0x00100000UL)
#define BIT_19    (0x00080000UL)
#define BIT_18    (0x00040000UL)
#define BIT_17    (0x00020000UL)
#define BIT_16    (0x00010000UL)

#define BIT_15    (0x00008000UL)
#define BIT_14    (0x00004000UL)
#define BIT_13    (0x00002000UL)
#define BIT_12    (0x00001000UL)
#define BIT_11    (0x00000800UL)
#define BIT_10    (0x00000400UL)
#define BIT_9     (0x00000200UL)
#define BIT_8     (0x00000100UL)

#define BIT_7     (0x00000080UL)
#define BIT_6     (0x00000040UL)
#define BIT_5     (0x00000020UL)
#define BIT_4     (0x00000010UL)
#define BIT_3     (0x00000008UL)
#define BIT_2     (0x00000004UL)
#define BIT_1     (0x00000002UL)
#define BIT_0     (0x00000001UL)

/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
 */

typedef enum IRQn {
    /******  Cortex-M3 Processor Exceptions Numbers ***************************************************/
    NonMaskableInt_IRQn           = -14,    /*!<  2 Cortex-M3 Non Maskable Interrupt                 */
    HardFault_IRQn                = -13,    /*!<  3 Cortex-M3 Hard Fault Interrupt                   */
    MemoryManagement_IRQn         = -12,    /*!<  4 Cortex-M3 Memory Management Interrupt            */
    BusFault_IRQn                 = -11,    /*!<  5 Cortex-M3 Bus Fault Interrupt                    */
    UsageFault_IRQn               = -10,    /*!<  6 Cortex-M3 Usage Fault Interrupt                  */
    SVCall_IRQn                   = -5,     /*!< 11 Cortex-M3 SV Call Interrupt                      */
    DebugMonitor_IRQn             = -4,     /*!< 12 Cortex-M3 Debug Monitor Interrupt                */
    PendSV_IRQn                   = -2,     /*!< 14 Cortex-M3 Pend SV Interrupt                      */
    SysTick_IRQn                  = -1,     /*!< 15 Cortex-M3 System Tick Interrupt                  */

    /******  CM3DS_MPS2 Specific Interrupt Numbers *******************************************************/
    GPIO_IRQn                   = 0,   /*!< GPIO全局中断 */
    TIM0_IRQn                   = 1,   /*!< TIMER0全局中断 */
    TIM1_IRQn                   = 2,   /*!< TIMER1全局中断 */
    LINUART_IRQn                = 3,   /*!< LINUART全局中断 */
    EPWM_IRQn                   = 4,   /*!< EPWM全局中断 */
    UART1_IRQn                  = 5,   /*!< UART1全局中断 */
    ADC_IRQn                    = 6,   /*!< ADC全局中断 */
    GDU_IRQn                    = 7,   /*!< GDU全局中断 */
    SPI_IRQn                    = 8,   /*!< SPI全局中断 */
    FLASH_IRQn                  = 9,   /*!< FLASH全局中断 */
    PWMIO_IRQn                  = 10,   /*!< PWMIO全局中断 */
    LINPORT_IRQn                = 11,   /*!< LINPORT全局中断 */
    RFI_IRQn                    = 12,   /*!< RFI全局中断 */
    DWDG_IRQn                   = 13,   /*!< DWDG全局中断 */
    WWDG_IRQn                   = 14,   /*!< DWDG全局中断 */
    PMU_IRQn                    = 15,   /*!< PMU中断（UV/OV/TSD） */
    CHP_IRQn                    = 16,   /*!< CHARGEPUMP中断 */
    SSI_IRQn                    = 17,   /*!< SSI中断 */
} IRQn_Type;


/* ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/* Configuration of the Cortex-M3 Processor and Core Peripherals */
#define __CM3_REV                 0x0201    /*!< Core Revision r2p1                             */
#define __NVIC_PRIO_BITS          3         /*!< Number of Bits used for Priority Levels        */
#define __Vendor_SysTickConfig    0         /*!< Set to 1 if different SysTick Config is used   */
#define __MPU_PRESENT             1         /*!< MPU present or not                             */

#include "core_cm3.h"
#pragma anon_unions
/**
  * @brief FLASHCTRL (novosense) 
 */
typedef struct {
    union {
        __IO uint32_t RDCR;                 /*!< Address Offset: 0x000  FLASH读取控制寄存器  */
        struct {
            __IO uint32_t WAIT               :  2;
            __IO uint32_t                    : 30;
        } RDCR_b;
    };
    union {
        __IO uint32_t PFCR;                 /*!< Address Offset: 0x004  预取指控制寄存器  */
        struct {
            __IO uint32_t PFTPHS_EN          :  1;
            __IO uint32_t                    : 31;
        } PFCR_b;
    };
    union {
        __IO uint32_t EPCR;                 /*!< Address Offset: 0x008  Flash 擦写控制寄存器  */
        struct {
            __IO uint32_t EREQ               :  1;
            __IO uint32_t PREQ               :  1;
            __IO uint32_t CONREQ             :  1;
            __IO uint32_t                    :  5;
            __IO uint32_t ERTYPE             :  2;
            __IO uint32_t                    : 22;
        } EPCR_b;
    };
    union {
        __IO uint32_t KEY;                 /*!< Address Offset: 0x00C  Flash Key 输入寄存器  */
        struct {
            __IO uint32_t FLS_KEY            : 32;
        } KEY_b;
    };
    union {
        __IO uint32_t NVRKEY;                 /*!< Address Offset: 0x010  Flash NVR Key 输入寄存器  */
        struct {
            __IO uint32_t FLS_NVRKEY         : 32;
        } NVRKEY_b;
    };
    union {
        __IO uint32_t TMCTRL;                 /*!< Address Offset: 0x014  Flash Timing 控制寄存器  */
        struct {
            __IO uint32_t UNIT               :  6;
            __IO uint32_t DPD                :  1;
            __IO uint32_t                    : 25;
        } TMCTRL_b;
    };
    union {
        __IO uint32_t IEN;                 /*!< Address Offset: 0x018  Flash 中断使能寄存器  */
        struct {
            __IO uint32_t ERDIE              :  1;
            __IO uint32_t PRDIE              :  1;
            __IO uint32_t KEYIE              :  1;
            __IO uint32_t ECCIE              :  1;
            __IO uint32_t WOPIE              :  1;
            __IO uint32_t                    : 27;
        } IEN_b;
    };
    union {
        __IO uint32_t ISR;                 /*!< Address Offset: 0x01C  Flash 标志寄存器  */
        struct {
            __IO uint32_t ERDIF              :  1;
            __IO uint32_t PRDIF              :  1;
            __IO uint32_t KEYERRIF           :  1;
            __IO uint32_t ECCERRIF           :  1;
            __IO uint32_t                    : 12;
            __IO uint32_t TRIMRRIF           :  1;
            __IO uint32_t KEYST              :  3;
            __IO uint32_t NVRKEYST           :  3;
            __IO uint32_t WOPERRIF           :  1;
            __IO uint32_t                    :  8;
        } ISR_b;
    };
} FLASHCTRL_TypeDef;
/**
  * @brief SYSCTRL (novosense) 
 */
typedef struct {
    union {
        __IO uint32_t SCCR;                 /*!< Address Offset: 0x000  系统时钟控制寄存器  */
        struct {
            __IO uint32_t SYSCDIV            :  3;
            __IO uint32_t SCKSRC             :  1;
            __IO uint32_t AHBAGE             :  1;
            __IO uint32_t                    : 11;
            __IO uint32_t APBCEN             :  1;
            __IO uint32_t AHBCEN             :  1;
            __IO uint32_t APBRST             :  1;
            __IO uint32_t AHBRST             :  1;
            __IO uint32_t FHRRST             :  1;
            __IO uint32_t HSIMUX             :  1;
            __IO uint32_t LSIMUX             :  1;
            __IO uint32_t CPMUX              :  1;
            __IO uint32_t                    :  8;
        } SCCR_b;
    };
    union {
        __IO uint32_t APBCGR;                 /*!< Address Offset: 0x004  APB时钟门控寄存器  */
        struct {
            __IO uint32_t GDUCEN             :  1;
            __IO uint32_t SPICEN             :  1;
            __IO uint32_t LINPCEN            :  1;
            __IO uint32_t LINUARTCEN         :  1;
            __IO uint32_t PWMIOCEN           :  1;
            __IO uint32_t TIM0CEN            :  1;
            __IO uint32_t TIM1CEN            :  1;
            __IO uint32_t WWDGCEN            :  1;
            __IO uint32_t DWDGCEN            :  1;
            __IO uint32_t UART1CEN           :  1;
            __IO uint32_t SSICEN             :  1;
            __IO uint32_t                    : 21;
        } APBCGR_b;
    };
    union {
        __IO uint32_t AHBCGR;                 /*!< Address Offset: 0x008  AHB时钟门控寄存器  */
        struct {
            __IO uint32_t                    :  1;
            __IO uint32_t GPIOCEN            :  1;
            __IO uint32_t APMUCEN            :  1;
            __IO uint32_t                    :  1;
            __IO uint32_t ADCCEN             :  1;
            __IO uint32_t EPWMCEN            :  1;
            __IO uint32_t                    : 26;
        } AHBCGR_b;
    };
    __IO uint32_t reserved0[1];
    union {
        __IO uint32_t APBSRR;                 /*!< Address Offset: 0x010  APB软复位寄存器  */
        struct {
            __IO uint32_t GDUREN             :  1;
            __IO uint32_t SPIREN             :  1;
            __IO uint32_t LINPREN            :  1;
            __IO uint32_t LINUARTREN         :  1;
            __IO uint32_t PWMIOREN           :  1;
            __IO uint32_t TIM0REN            :  1;
            __IO uint32_t TIM1REN            :  1;
            __IO uint32_t WWDGREN            :  1;
            __IO uint32_t DWDGREN            :  1;
            __IO uint32_t UART1REN           :  1;
            __IO uint32_t SSIREN             :  1;
            __IO uint32_t                    : 21;
        } APBSRR_b;
    };
    union {
        __IO uint32_t AHBSRR;                 /*!< Address Offset: 0x014  AHB软复位寄存器  */
        struct {
            __IO uint32_t FLASHREN           :  1;
            __IO uint32_t GPIOREN            :  1;
            __IO uint32_t APMUREN            :  1;
            __IO uint32_t                    :  1;
            __IO uint32_t ADCREN             :  1;
            __IO uint32_t EPWMREN            :  1;
            __IO uint32_t                    : 26;
        } AHBSRR_b;
    };
    __IO uint32_t reserved1[2];
    union {
        __IO uint32_t SYSWKR;                 /*!< Address Offset: 0x020  系统唤醒寄存器  */
        struct {
            __IO uint32_t BVDDUVE            :  1;
            __IO uint32_t BVDDOVE            :  1;
            __IO uint32_t LINWKE             :  1;
            __IO uint32_t WTWKE              :  1;
            __IO uint32_t                    : 28;
        } SYSWKR_b;
    };
    union {
        __IO uint32_t SYSSTR;                 /*!< Address Offset: 0x024  系统状态寄存器  */
        struct {
            __IO uint32_t BVDDUV             :  1;
            __IO uint32_t BVDDOV             :  1;
            __IO uint32_t LINWK              :  1;
            __IO uint32_t WTWK               :  1;
            __IO uint32_t FHRST              :  1;
            __IO uint32_t OTDST              :  1;
            __IO uint32_t SQRST              :  1;
            __IO uint32_t DWRST              :  1;
            __IO uint32_t WWRST              :  1;
            __IO uint32_t BOVRST             :  1;
            __IO uint32_t BUVRST             :  1;
            __IO uint32_t                    : 21;
        } SYSSTR_b;
    };
    union {
        __IO uint32_t PWRCR;                 /*!< Address Offset: 0x028  电源控制寄存器  */
        struct {
            __IO uint32_t BUF                :  4;
            __IO uint32_t BOF                :  4;
            __IO uint32_t TSF                :  4;
            __IO uint32_t UVVTH              :  2;
            __IO uint32_t OVVTH              :  1;
            __IO uint32_t                    :  1;
            __IO uint32_t COVTH              :  1;
            __IO uint32_t CUVTH              :  2;
            __IO uint32_t                    :  1;
            __IO uint32_t BUIEN              :  1;
            __IO uint32_t BOIEN              :  1;
            __IO uint32_t TSIEN              :  1;
            __IO uint32_t TSDREN             :  1;
            __IO uint32_t BUREN              :  1;
            __IO uint32_t BOREN              :  1;
            __IO uint32_t                    :  1;
            __IO uint32_t                    :  1;
            __IO uint32_t BUVST              :  1;
            __IO uint32_t BOVST              :  1;
            __IO uint32_t TSDST              :  1;
            __IO uint32_t RFIST              :  1;
        } PWRCR_b;
    };
    union {
        __IO uint32_t APMUSTR;                 /*!< Address Offset: 0x02C  APMU Status状态寄存器  */
        struct {
            __IO uint32_t STAR               :  6;
            __IO uint32_t                    : 26;
        } APMUSTR_b;
    };
    union {
        __IO uint32_t LPCR;                 /*!< Address Offset: 0x030  低功耗控制寄存器  */
        struct {
            __IO uint32_t IDLE               :  1;
            __IO uint32_t SLEEP              :  1;
            __IO uint32_t SHAE               :  1;
            __IO uint32_t                    : 29;
        } LPCR_b;
    };
    __IO uint32_t reserved2[1];
    union {
        __IO uint32_t ANACR;                 /*!< Address Offset: 0x038  模拟配置寄存器  */
        struct {
            __IO uint32_t MCEN               :  1;
            __IO uint32_t MCMD               :  1;
            __IO uint32_t                    :  1;
            __IO uint32_t EMREN              :  1;
            __IO uint32_t REFEN              :  1;
            __IO uint32_t REFOUT             :  1;
            __IO uint32_t REFBUFEN           :  1;
            __IO uint32_t REFEXTEN           :  1;
            __IO uint32_t CHGPEN             :  1;
            __IO uint32_t CF                 :  2;
            __IO uint32_t CFEN               :  1;
            __IO uint32_t CIEN               :  1;
            __IO uint32_t CHGOV              :  1;
            __IO uint32_t CHGUV              :  1;
            __IO uint32_t CHGPASTEN          :  1;
            __IO uint32_t CHGOK              :  1;
            __IO uint32_t                    :  7;
            __IO uint32_t CHGPCDIV           :  2;
            __IO uint32_t                    :  6;
        } ANACR_b;
    };
    union {
        __IO uint32_t SYSDBGR;                 /*!< Address Offset: 0x03C  系统调试寄存器  */
        struct {
            __IO uint32_t DWDGDE             :  1;
            __IO uint32_t WWDGDE             :  1;
            __IO uint32_t TIM0DE             :  1;
            __IO uint32_t TIM1DE             :  1;
            __IO uint32_t GDUDE              :  1;
            __IO uint32_t ADCDE              :  1;
            __IO uint32_t PWMDE              :  1;
            __IO uint32_t PIODE              :  1;
            __IO uint32_t SPIDE              :  1;
            __IO uint32_t LINDEN             :  1;
            __IO uint32_t                    : 22;
        } SYSDBGR_b;
    };
    union {
        __IO uint32_t LKKEYR;                 /*!< Address Offset: 0x040  锁键寄存器  */
        struct {
            __IO uint32_t LOCKKEY            : 16;
            __IO uint32_t KEYST0             :  1;
            __IO uint32_t KEYST1             :  1;
            __IO uint32_t                    : 14;
        } LKKEYR_b;
    };
    __IO uint32_t reserved3[12];
    union {
        __IO uint32_t SYSCFR;                 /*!< Address Offset: 0x074  系统控制配置寄存器  */
        struct {
            __IO uint32_t DWDGEN             :  8;
            __IO uint32_t WWDGEN             :  8;
            __IO uint32_t SWEN               :  8;
            __IO uint32_t FREQ               :  1;
            __IO uint32_t                    :  7;
        } SYSCFR_b;
    };
    __IO uint32_t reserved4[12];
    union {
        __IO uint32_t GDUTRIM;                 /*!< Address Offset: 0xA8  gdutrim 寄存器  */
        struct {
            __IO uint32_t GDUTRIMSHAE        :  4;
            __IO uint32_t                    : 28;
        } GDUTRIM_b;
    };
    union {
        __IO uint32_t ADCTRIM;                 /*!< Address Offset: 0xAC  adctrim 寄存器  */
        struct {
            __IO uint32_t ADCTRIMSHAE        : 31;
            __IO uint32_t                    :  1;
        } ADCTRIM_b;
    };
    union {
        __IO uint32_t TEMPTRIM;                 /*!< Address Offset: 0xB0  temptrim 寄存器  */
        struct {
            __IO uint32_t TEMPTRIMSHAE       : 24;
            __IO uint32_t                    :  8;
        } TEMPTRIM_b;
    };
} SYSCTRL_TypeDef;
/**
  * @brief PWM (novosense) 
 */
typedef struct {
    union {
        __IO uint32_t PWMENABLE;                 /*!< Address Offset: 0x000  PWM输出使能寄存器  */
        struct {
            __IO uint32_t PWMOE              :  1;
            __IO uint32_t                    : 31;
        } PWMENABLE_b;
    };
    union {
        __IO uint32_t PWMCON1;                 /*!< Address Offset: 0x004  PWM模块控制寄存器1  */
        struct {
            __IO uint32_t PWM0H              :  1;
            __IO uint32_t PWM1H              :  1;
            __IO uint32_t PWM2H              :  1;
            __IO uint32_t PWM0L              :  1;
            __IO uint32_t PWM1L              :  1;
            __IO uint32_t PWM2L              :  1;
            __IO uint32_t PDCON0             :  1;
            __IO uint32_t PDCON1             :  1;
            __IO uint32_t PDCON2             :  1;
            __IO uint32_t PWMSYM             :  1;
            __IO uint32_t PTMOD              :  2;
            __IO uint32_t POSTPS             :  3;
            __IO uint32_t PWMOUTMODE         :  1;
            __IO uint32_t                    : 16;
        } PWMCON1_b;
    };
    union {
        __IO uint32_t PWMCON2;                 /*!< Address Offset: 0x008  PWM模块控制寄存器2  */
        struct {
            __IO uint32_t CMP1               :  2;
            __IO uint32_t CMP2               :  2;
            __IO uint32_t CMP3               :  2;
            __IO uint32_t CMP4               :  2;
            __IO uint32_t OSYNC              :  1;
            __IO uint32_t DILDEN             :  1;
            __IO uint32_t CILDEN             :  1;
            __IO uint32_t ZDLDEN             :  1;
            __IO uint32_t PDLDEN             :  1;
            __IO uint32_t ZCMLDEN            :  1;
            __IO uint32_t PCMLDEN            :  1;
            __IO uint32_t                    : 17;
        } PWMCON2_b;
    };
    union {
        __IO uint32_t PWMPERIOD;                 /*!< Address Offset: 0x00C  PWM周期寄存器  */
        struct {
            __IO uint32_t PWMP               : 16;
            __IO uint32_t                    : 16;
        } PWMPERIOD_b;
    };
    __IO uint32_t reserved0[1];
    union {
        __IO uint32_t PWMPSQ;                 /*!< Address Offset: 0x014  PWM时钟预分频寄存器  */
        struct {
            __IO uint32_t PWMPSQ             : 16;
            __IO uint32_t                    : 16;
        } PWMPSQ_b;
    };
    union {
        __IO uint32_t PWM0DH;                 /*!< Address Offset: 0x018  PWM0H占空比寄存器1  */
        struct {
            __IO uint32_t PWM0DH             : 16;
            __IO uint32_t                    : 16;
        } PWM0DH_b;
    };
    union {
        __IO uint32_t PWM0DL;                 /*!< Address Offset: 0x01C  PWM0L占空比寄存器2  */
        struct {
            __IO uint32_t PWM0DL             : 16;
            __IO uint32_t                    : 16;
        } PWM0DL_b;
    };
    union {
        __IO uint32_t PWM1DH;                 /*!< Address Offset: 0x020  PWM1H占空比寄存器1  */
        struct {
            __IO uint32_t PWM1DH             : 16;
            __IO uint32_t                    : 16;
        } PWM1DH_b;
    };
    union {
        __IO uint32_t PWM1DL;                 /*!< Address Offset: 0x024  PWM1L占空比寄存器2  */
        struct {
            __IO uint32_t PWM1DL             : 16;
            __IO uint32_t                    : 16;
        } PWM1DL_b;
    };
    union {
        __IO uint32_t PWM2DH;                 /*!< Address Offset: 0x028  PWM2H占空比寄存器2  */
        struct {
            __IO uint32_t PWM2DH             : 16;
            __IO uint32_t                    : 16;
        } PWM2DH_b;
    };
    union {
        __IO uint32_t PWM2DL;                 /*!< Address Offset: 0x02C  PWM2L占空比寄存器2  */
        struct {
            __IO uint32_t PWM2DL             : 16;
            __IO uint32_t                    : 16;
        } PWM2DL_b;
    };
    union {
        __IO uint32_t PWMCMP1;                 /*!< Address Offset: 0x030  事件触发比较寄存器1  */
        struct {
            __IO uint32_t PWMCMP1            : 16;
            __IO uint32_t                    : 16;
        } PWMCMP1_b;
    };
    union {
        __IO uint32_t PWMCMP2;                 /*!< Address Offset: 0x034  事件触发比较寄存器2  */
        struct {
            __IO uint32_t PWMCMP2            : 16;
            __IO uint32_t                    : 16;
        } PWMCMP2_b;
    };
    union {
        __IO uint32_t PWMCMP3;                 /*!< Address Offset: 0x038  事件触发比较寄存器3  */
        struct {
            __IO uint32_t PWMCMP3            : 16;
            __IO uint32_t                    : 16;
        } PWMCMP3_b;
    };
    union {
        __IO uint32_t PWMCMP4;                 /*!< Address Offset: 0x03C  事件触发比较寄存器4  */
        struct {
            __IO uint32_t PWMCMP4            : 16;
            __IO uint32_t                    : 16;
        } PWMCMP4_b;
    };
    union {
        __IO uint32_t PWMDT0H;                 /*!< Address Offset: 0x040  PWM通道0H上升沿死区控制寄存器  */
        struct {
            __IO uint32_t PWMDT0H            : 16;
            __IO uint32_t                    : 16;
        } PWMDT0H_b;
    };
    union {
        __IO uint32_t PWMDT0L;                 /*!< Address Offset: 0x044  PWM通道0L上升沿死区控制寄存器  */
        struct {
            __IO uint32_t PWMDT0L            : 16;
            __IO uint32_t                    : 16;
        } PWMDT0L_b;
    };
    union {
        __IO uint32_t PWMDT1H;                 /*!< Address Offset: 0x048  PWM通道1H上升沿死区控制寄存器  */
        struct {
            __IO uint32_t PWMDT1H            : 16;
            __IO uint32_t                    : 16;
        } PWMDT1H_b;
    };
    union {
        __IO uint32_t PWMDT1L;                 /*!< Address Offset: 0x04C  PWM通道1L上升沿死区控制寄存器  */
        struct {
            __IO uint32_t PWMDT1L            : 16;
            __IO uint32_t                    : 16;
        } PWMDT1L_b;
    };
    union {
        __IO uint32_t PWMDT2H;                 /*!< Address Offset: 0x050  PWM通道2H上升沿死区控制寄存器  */
        struct {
            __IO uint32_t PWMDT2H            : 16;
            __IO uint32_t                    : 16;
        } PWMDT2H_b;
    };
    union {
        __IO uint32_t PWMDT2L;                 /*!< Address Offset: 0x054  PWM通道2L上升沿死区控制寄存器  */
        struct {
            __IO uint32_t PWMDT2L            : 16;
            __IO uint32_t                    : 16;
        } PWMDT2L_b;
    };
    union {
        __IO uint32_t PMANUALCON1;                 /*!< Address Offset: 0x058  PWM0/1/2手动输出设置寄存器1  */
        struct {
            __IO uint32_t PMANUAL0H          :  1;
            __IO uint32_t PMANUAL1H          :  1;
            __IO uint32_t PMANUAL2H          :  1;
            __IO uint32_t PMANUAL0L          :  1;
            __IO uint32_t PMANUAL1L          :  1;
            __IO uint32_t PMANUAL2L          :  1;
            __IO uint32_t                    : 26;
        } PMANUALCON1_b;
    };
    union {
        __IO uint32_t PMANUALCON2;                 /*!< Address Offset: 0x05C  PWM0/1/2手动输出设置寄存器2  */
        struct {
            __IO uint32_t POUT0L             :  1;
            __IO uint32_t POUT1L             :  1;
            __IO uint32_t POUT2L             :  1;
            __IO uint32_t POUT0H             :  1;
            __IO uint32_t POUT1H             :  1;
            __IO uint32_t POUT2H             :  1;
            __IO uint32_t                    : 26;
        } PMANUALCON2_b;
    };
    union {
        __IO uint32_t PWMDSWMOD;                 /*!< Address Offset: 0x060  PWM Double Switch 模式  */
        struct {
            __IO uint32_t CINV0H             :  1;
            __IO uint32_t CINV0L             :  1;
            __IO uint32_t CINV1H             :  1;
            __IO uint32_t CINV1L             :  1;
            __IO uint32_t CINV2H             :  1;
            __IO uint32_t CINV2L             :  1;
            __IO uint32_t PINVA              :  1;
            __IO uint32_t PINVB              :  1;
            __IO uint32_t PINVC              :  1;
            __IO uint32_t PECA               :  1;
            __IO uint32_t PECB               :  1;
            __IO uint32_t PECC               :  1;
            __IO uint32_t PSELA              :  1;
            __IO uint32_t PSELB              :  1;
            __IO uint32_t PSELC              :  1;
            __IO uint32_t                    : 17;
        } PWMDSWMOD_b;
    };
    __IO uint32_t reserved1[3];
    union {
        __IO uint32_t IEN;                 /*!< Address Offset: 0x070  PWM中断使能控制寄存器  */
        struct {
            __IO uint32_t PTUD0IE            :  1;
            __IO uint32_t PTDD0IE            :  1;
            __IO uint32_t PTUD1IE            :  1;
            __IO uint32_t PTDD1IE            :  1;
            __IO uint32_t PTUD2IE            :  1;
            __IO uint32_t PTDD2IE            :  1;
            __IO uint32_t PWMZIE             :  1;
            __IO uint32_t PWMPIE             :  1;
            __IO uint32_t                    :  1;
            __IO uint32_t                    :  1;
            __IO uint32_t                    :  1;
            __IO uint32_t                    : 21;
        } IEN_b;
    };
    union {
        __IO uint32_t ISR;                 /*!< Address Offset: 0x074  PWM中断标志和清除寄存器  */
        struct {
            __IO uint32_t PTUD0IF            :  1;
            __IO uint32_t PTDD0IF            :  1;
            __IO uint32_t PTUD1IF            :  1;
            __IO uint32_t PTDD1IF            :  1;
            __IO uint32_t PTUD2IF            :  1;
            __IO uint32_t PTDD2IF            :  1;
            __IO uint32_t PWMZIF             :  1;
            __IO uint32_t PWMPIF             :  1;
            __IO uint32_t PWMNUPDIF          :  1;
            __IO uint32_t                    : 23;
        } ISR_b;
    };
    union {
        __IO uint32_t PWMRWEN;                 /*!< Address Offset: 0x078  寄存器修改和重载控制寄存器  */
        struct {
            __IO uint32_t PWMRLDEN           :  8;
            __IO uint32_t                    : 24;
        } PWMRWEN_b;
    };
} PWM_TypeDef;
/**
  * @brief ADC (novosense) 
 */
typedef struct {
    union {
        __IO uint32_t CR;                 /*!< Address Offset: 0x000  ADC控制寄存器  */
        struct {
            __IO uint32_t EN                 :  1;
            __IO uint32_t SWT                :  1;
            __IO uint32_t TSRC               :  3;
            __IO uint32_t BUSY               :  1;
            __IO uint32_t CKSEL              :  3;
            __IO uint32_t REFPSEL            :  1;
            __IO uint32_t REFNSEL            :  1;
            __IO uint32_t INBUFEN            :  1;
            __IO uint32_t INBUFBYPASS        :  1;
            __IO uint32_t SINGLEEN           :  1;
            __IO uint32_t RDSTATUS           :  1;
            __IO uint32_t                    : 17;
        } CR_b;
    };
    union {
        __IO uint32_t SCR;                 /*!< Address Offset: 0x004  ADC采样控制寄存器  */
        struct {
            __IO uint32_t SAMP               :  6;
            __IO uint32_t                    : 26;
        } SCR_b;
    };
    union {
        __IO uint32_t TCR;                 /*!< Address Offset: 0x008  ADC校准配置寄存器  */
        struct {
            __IO uint32_t OFFSET             :  8;
            __IO uint32_t GAININTER          : 11;
            __IO uint32_t GAINEXT            : 11;
            __IO uint32_t TRIMEN             :  1;
            __IO uint32_t                    :  1;
        } TCR_b;
    };
    union {
        __IO uint32_t QCR1;                 /*!< Address Offset: 0x00C  ADC队列控制寄存器1  */
        struct {
            __IO uint32_t Q1SEL              :  4;
            __IO uint32_t Q1EN               :  1;
            __IO uint32_t Q2SEL              :  4;
            __IO uint32_t Q2EN               :  1;
            __IO uint32_t Q3SEL              :  4;
            __IO uint32_t Q3EN               :  1;
            __IO uint32_t Q4SEL              :  4;
            __IO uint32_t Q4EN               :  1;
            __IO uint32_t Q5SEL              :  4;
            __IO uint32_t Q5EN               :  1;
            __IO uint32_t Q6SEL              :  4;
            __IO uint32_t Q6EN               :  1;
            __IO uint32_t                    :  2;
        } QCR1_b;
    };
    union {
        __IO uint32_t QCR2;                 /*!< Address Offset: 0x010  ADC队列控制寄存器2  */
        struct {
            __IO uint32_t Q7SEL              :  4;
            __IO uint32_t Q7EN               :  1;
            __IO uint32_t Q8SEL              :  4;
            __IO uint32_t Q8EN               :  1;
            __IO uint32_t                    : 22;
        } QCR2_b;
    };
    union {
        __IO uint32_t QTR1;                 /*!< Address Offset: 0x014  队列启动切换时间  */
        struct {
            __IO uint32_t QTTR               : 15;
            __IO uint32_t ABS                :  1;
            __IO uint32_t                    : 16;
        } QTR1_b;
    };
    union {
        __IO uint32_t QTR2;                 /*!< Address Offset: 0x018  队列启动切换时间  */
        struct {
            __IO uint32_t QTTR               : 15;
            __IO uint32_t ABS                :  1;
            __IO uint32_t                    : 16;
        } QTR2_b;
    };
    union {
        __IO uint32_t QTR3;                 /*!< Address Offset: 0x01c  队列启动切换时间  */
        struct {
            __IO uint32_t QTTR               : 15;
            __IO uint32_t ABS                :  1;
            __IO uint32_t                    : 16;
        } QTR3_b;
    };
    union {
        __IO uint32_t QTR4;                 /*!< Address Offset: 0x020  队列启动切换时间  */
        struct {
            __IO uint32_t QTTR               : 15;
            __IO uint32_t ABS                :  1;
            __IO uint32_t                    : 16;
        } QTR4_b;
    };
    union {
        __IO uint32_t QTR5;                 /*!< Address Offset: 0x024  队列启动切换时间  */
        struct {
            __IO uint32_t QTTR               : 15;
            __IO uint32_t ABS                :  1;
            __IO uint32_t                    : 16;
        } QTR5_b;
    };
    union {
        __IO uint32_t QTR6;                 /*!< Address Offset: 0x028  队列启动切换时间  */
        struct {
            __IO uint32_t QTTR               : 15;
            __IO uint32_t ABS                :  1;
            __IO uint32_t                    : 16;
        } QTR6_b;
    };
    union {
        __IO uint32_t QTR7;                 /*!< Address Offset: 0x02c  队列启动切换时间  */
        struct {
            __IO uint32_t QTTR               : 15;
            __IO uint32_t ABS                :  1;
            __IO uint32_t                    : 16;
        } QTR7_b;
    };
    union {
        __IO uint32_t QTR8;                 /*!< Address Offset: 0x030  队列启动切换时间  */
        struct {
            __IO uint32_t QTTR               : 15;
            __IO uint32_t ABS                :  1;
            __IO uint32_t                    : 16;
        } QTR8_b;
    };
    union {
        __IO uint32_t DR1;                 /*!< Address Offset: 0x034  ADC 数据寄存器  */
        struct {
            __IO uint32_t DATA               : 12;
            __IO uint32_t CHINDEX            :  4;
            __IO uint32_t                    : 16;
        } DR1_b;
    };
    union {
        __IO uint32_t DR2;                 /*!< Address Offset: 0x038  ADC 数据寄存器  */
        struct {
            __IO uint32_t DATA               : 12;
            __IO uint32_t CHINDEX            :  4;
            __IO uint32_t                    : 16;
        } DR2_b;
    };
    union {
        __IO uint32_t DR3;                 /*!< Address Offset: 0x03c  ADC 数据寄存器  */
        struct {
            __IO uint32_t DATA               : 12;
            __IO uint32_t CHINDEX            :  4;
            __IO uint32_t                    : 16;
        } DR3_b;
    };
    union {
        __IO uint32_t DR4;                 /*!< Address Offset: 0x040  ADC 数据寄存器  */
        struct {
            __IO uint32_t DATA               : 12;
            __IO uint32_t CHINDEX            :  4;
            __IO uint32_t                    : 16;
        } DR4_b;
    };
    union {
        __IO uint32_t DR5;                 /*!< Address Offset: 0x044  ADC 数据寄存器  */
        struct {
            __IO uint32_t DATA               : 12;
            __IO uint32_t CHINDEX            :  4;
            __IO uint32_t                    : 16;
        } DR5_b;
    };
    union {
        __IO uint32_t DR6;                 /*!< Address Offset: 0x048  ADC 数据寄存器  */
        struct {
            __IO uint32_t DATA               : 12;
            __IO uint32_t CHINDEX            :  4;
            __IO uint32_t                    : 16;
        } DR6_b;
    };
    union {
        __IO uint32_t DR7;                 /*!< Address Offset: 0x04c  ADC 数据寄存器  */
        struct {
            __IO uint32_t DATA               : 12;
            __IO uint32_t CHINDEX            :  4;
            __IO uint32_t                    : 16;
        } DR7_b;
    };
    union {
        __IO uint32_t DR8;                 /*!< Address Offset: 0x050  ADC 数据寄存器  */
        struct {
            __IO uint32_t DATA               : 12;
            __IO uint32_t CHINDEX            :  4;
            __IO uint32_t                    : 16;
        } DR8_b;
    };
    union {
        __IO uint32_t INTRD;                 /*!< Address Offset: 0x054  已读标志位寄存器  */
        struct {
            __IO uint32_t DR1RD              :  1;
            __IO uint32_t DR2RD              :  1;
            __IO uint32_t DR3RD              :  1;
            __IO uint32_t DR4RD              :  1;
            __IO uint32_t DR5RD              :  1;
            __IO uint32_t DR6RD              :  1;
            __IO uint32_t DR7RD              :  1;
            __IO uint32_t DR8RD              :  1;
            __IO uint32_t                    : 24;
        } INTRD_b;
    };
    union {
        __IO uint32_t IEN;                 /*!< Address Offset: 0x058  中断使能寄存器  */
        struct {
            __IO uint32_t EOCIE              :  1;
            __IO uint32_t TCOLLIE            :  1;
            __IO uint32_t QCOLLIE            :  1;
            __IO uint32_t CALERRIE           :  1;
            __IO uint32_t SMPLCOLLIE         :  1;
            __IO uint32_t                    : 27;
        } IEN_b;
    };
    union {
        __IO uint32_t ISR;                 /*!< Address Offset: 0x05c  中断置起状态寄存器  */
        struct {
            __IO uint32_t EOCIF              :  1;
            __IO uint32_t TCOLLIF            :  1;
            __IO uint32_t QCOLL1IF           :  1;
            __IO uint32_t QCOLL2IF           :  1;
            __IO uint32_t QCOLL3IF           :  1;
            __IO uint32_t QCOLL4IF           :  1;
            __IO uint32_t QCOLL5IF           :  1;
            __IO uint32_t QCOLL6IF           :  1;
            __IO uint32_t QCOLL7IF           :  1;
            __IO uint32_t QCOLL8IF           :  1;
            __IO uint32_t CALERR1IF          :  1;
            __IO uint32_t CALERR2IF          :  1;
            __IO uint32_t CALERR3IF          :  1;
            __IO uint32_t CALERR4IF          :  1;
            __IO uint32_t CALERR5IF          :  1;
            __IO uint32_t CALERR6IF          :  1;
            __IO uint32_t CALERR7IF          :  1;
            __IO uint32_t CALERR8IF          :  1;
            __IO uint32_t SMPLCOLLIF         :  1;
            __IO uint32_t                    : 13;
        } ISR_b;
    };
    union {
        __IO uint32_t IPND;                 /*!< Address Offset: 0x060  中断使能置起状态寄存器  */
        struct {
            __IO uint32_t EOC                :  1;
            __IO uint32_t TCOLL              :  1;
            __IO uint32_t QCOLL1             :  1;
            __IO uint32_t QCOLL2             :  1;
            __IO uint32_t QCOLL3             :  1;
            __IO uint32_t QCOLL4             :  1;
            __IO uint32_t QCOLL5             :  1;
            __IO uint32_t QCOLL6             :  1;
            __IO uint32_t QCOLL7             :  1;
            __IO uint32_t QCOLL8             :  1;
            __IO uint32_t CALERR1            :  1;
            __IO uint32_t CALERR2            :  1;
            __IO uint32_t CALERR3            :  1;
            __IO uint32_t CALERR4            :  1;
            __IO uint32_t CALERR5            :  1;
            __IO uint32_t CALERR6            :  1;
            __IO uint32_t CALERR7            :  1;
            __IO uint32_t CALERR8            :  1;
            __IO uint32_t SAMPLECOLL         :  1;
            __IO uint32_t                    : 13;
        } IPND_b;
    };
    union {
        __IO uint32_t TS;                 /*!< Address Offset: 0x064  温度传感器校准寄存器  */
        struct {
            __IO uint32_t TST                : 12;
            __IO uint32_t TSA                : 12;
            __IO uint32_t                    :  8;
        } TS_b;
    };
} ADC_TypeDef;

/**
  * @brief GPIO (novosense) 
 */
typedef struct {
    union {
        __IO uint32_t PDI;                 /*!< Address Offset: 0x000  GPIO 端口输入数据寄存器  */
        struct {
            __IO uint32_t DI0                :  1;
            __IO uint32_t DI1                :  1;
            __IO uint32_t DI2                :  1;
            __IO uint32_t DI3                :  1;
            __IO uint32_t DI4                :  1;
            __IO uint32_t DI5                :  1;
            __IO uint32_t DI6                :  1;
            __IO uint32_t DI7                :  1;
            __IO uint32_t DI8                :  1;
            __IO uint32_t DI9                :  1;
            __IO uint32_t DI10               :  1;
            __IO uint32_t DI11               :  1;
            __IO uint32_t DI12               :  1;
            __IO uint32_t DI13               :  1;
            __IO uint32_t                    : 18;
        } PDI_b;
    };
    union {
        __IO uint32_t PDO;                 /*!< Address Offset: 0x004  GPIO 输出数据寄存器  */
        struct {
            __IO uint32_t DO0                :  1;
            __IO uint32_t DO1                :  1;
            __IO uint32_t DO2                :  1;
            __IO uint32_t DO3                :  1;
            __IO uint32_t DO4                :  1;
            __IO uint32_t DO5                :  1;
            __IO uint32_t DO6                :  1;
            __IO uint32_t DO7                :  1;
            __IO uint32_t DO8                :  1;
            __IO uint32_t DO9                :  1;
            __IO uint32_t DO10               :  1;
            __IO uint32_t DO11               :  1;
            __IO uint32_t DO12               :  1;
            __IO uint32_t DO13               :  1;
            __IO uint32_t                    : 18;
        } PDO_b;
    };
    union {
        __IO uint32_t PDIEN;                 /*!< Address Offset: 0x008  GPIO 输入使能寄存器  */
        struct {
            __IO uint32_t DIEN0              :  1;
            __IO uint32_t DIEN1              :  1;
            __IO uint32_t DIEN2              :  1;
            __IO uint32_t DIEN3              :  1;
            __IO uint32_t DIEN4              :  1;
            __IO uint32_t DIEN5              :  1;
            __IO uint32_t DIEN6              :  1;
            __IO uint32_t DIEN7              :  1;
            __IO uint32_t DIEN8              :  1;
            __IO uint32_t DIEN9              :  1;
            __IO uint32_t DIEN10             :  1;
            __IO uint32_t DIEN11             :  1;
            __IO uint32_t DIEN12             :  1;
            __IO uint32_t DIEN13             :  1;
            __IO uint32_t                    : 18;
        } PDIEN_b;
    };
    union {
        __IO uint32_t PDOEN;                 /*!< Address Offset: 0x00C  GPIO 输出使能寄存器  */
        struct {
            __IO uint32_t DOEN0              :  1;
            __IO uint32_t DOEN1              :  1;
            __IO uint32_t DOEN2              :  1;
            __IO uint32_t DOEN3              :  1;
            __IO uint32_t DOEN4              :  1;
            __IO uint32_t DOEN5              :  1;
            __IO uint32_t DOEN6              :  1;
            __IO uint32_t DOEN7              :  1;
            __IO uint32_t DOEN8              :  1;
            __IO uint32_t DOEN9              :  1;
            __IO uint32_t DOEN10             :  1;
            __IO uint32_t DOEN11             :  1;
            __IO uint32_t DOEN12             :  1;
            __IO uint32_t DOEN13             :  1;
            __IO uint32_t                    : 18;
        } PDOEN_b;
    };
    union {
        __IO uint32_t IEN;                 /*!< Address Offset: 0x010  GPIO 中断使能寄存器  */
        struct {
            __IO uint32_t PM0IE              :  1;
            __IO uint32_t PM1IE              :  1;
            __IO uint32_t PM2IE              :  1;
            __IO uint32_t PM3IE              :  1;
            __IO uint32_t PM4IE              :  1;
            __IO uint32_t PM5IE              :  1;
            __IO uint32_t PM6IE              :  1;
            __IO uint32_t PM7IE              :  1;
            __IO uint32_t PM8IE              :  1;
            __IO uint32_t PM9IE              :  1;
            __IO uint32_t PM10IE             :  1;
            __IO uint32_t PM11IE             :  1;
            __IO uint32_t PM12IE             :  1;
            __IO uint32_t PM13IE             :  1;
            __IO uint32_t                    : 18;
        } IEN_b;
    };
    union {
        __IO uint32_t INTTYPE;                 /*!< Address Offset: 0x014  GPIO触发模式寄存器  */
        struct {
            __IO uint32_t UD0                :  2;
            __IO uint32_t UD1                :  2;
            __IO uint32_t UD2                :  2;
            __IO uint32_t UD3                :  2;
            __IO uint32_t UD4                :  2;
            __IO uint32_t UD5                :  2;
            __IO uint32_t UD6                :  2;
            __IO uint32_t UD7                :  2;
            __IO uint32_t UD8                :  2;
            __IO uint32_t UD9                :  2;
            __IO uint32_t UD10               :  2;
            __IO uint32_t UD11               :  2;
            __IO uint32_t UD12               :  2;
            __IO uint32_t UD13               :  2;
            __IO uint32_t                    :  4;
        } INTTYPE_b;
    };
    union {
        __IO uint32_t ISR;                 /*!< Address Offset: 0x018  GPIO中断状态寄存器  */
        struct {
            __IO uint32_t PD0IF              :  1;
            __IO uint32_t PD1IF              :  1;
            __IO uint32_t PD2IF              :  1;
            __IO uint32_t PD3IF              :  1;
            __IO uint32_t PD4IF              :  1;
            __IO uint32_t PD5IF              :  1;
            __IO uint32_t PD6IF              :  1;
            __IO uint32_t PD7IF              :  1;
            __IO uint32_t PD8IF              :  1;
            __IO uint32_t PD9IF              :  1;
            __IO uint32_t PD10IF             :  1;
            __IO uint32_t PD11IF             :  1;
            __IO uint32_t PD12IF             :  1;
            __IO uint32_t PD13IF             :  1;
            __IO uint32_t                    : 18;
        } ISR_b;
    };
    union {
        __IO uint32_t IPND;                 /*!< Address Offset: 0x01C  GPIO中断状态使能寄存器  */
        struct {
            __IO uint32_t PD0                :  1;
            __IO uint32_t PD1                :  1;
            __IO uint32_t PD2                :  1;
            __IO uint32_t PD3                :  1;
            __IO uint32_t PD4                :  1;
            __IO uint32_t PD5                :  1;
            __IO uint32_t PD6                :  1;
            __IO uint32_t PD7                :  1;
            __IO uint32_t PD8                :  1;
            __IO uint32_t PD9                :  1;
            __IO uint32_t PD10               :  1;
            __IO uint32_t PD11               :  1;
            __IO uint32_t PD12               :  1;
            __IO uint32_t PD13               :  1;
            __IO uint32_t                    : 18;
        } IPND_b;
    };
    union {
        __IO uint32_t PUR;                 /*!< Address Offset: 0x020  GPIO上拉使能寄存器  */
        struct {
            __IO uint32_t PU0                :  1;
            __IO uint32_t PU1                :  1;
            __IO uint32_t PU2                :  1;
            __IO uint32_t PU3                :  1;
            __IO uint32_t PU4                :  1;
            __IO uint32_t PU5                :  1;
            __IO uint32_t PU6                :  1;
            __IO uint32_t PU7                :  1;
            __IO uint32_t PU8                :  1;
            __IO uint32_t PU9                :  1;
            __IO uint32_t PU10               :  1;
            __IO uint32_t PU11               :  1;
            __IO uint32_t PU12               :  1;
            __IO uint32_t PU13               :  1;
            __IO uint32_t                    : 18;
        } PUR_b;
    };
    union {
        __IO uint32_t PDR;                 /*!< Address Offset: 0x024  GPIO下拉使能寄存器  */
        struct {
            __IO uint32_t PD0                :  1;
            __IO uint32_t PD1                :  1;
            __IO uint32_t PD2                :  1;
            __IO uint32_t PD3                :  1;
            __IO uint32_t PD4                :  1;
            __IO uint32_t PD5                :  1;
            __IO uint32_t PD6                :  1;
            __IO uint32_t PD7                :  1;
            __IO uint32_t PD8                :  1;
            __IO uint32_t PD9                :  1;
            __IO uint32_t PD10               :  1;
            __IO uint32_t PD11               :  1;
            __IO uint32_t PD12               :  1;
            __IO uint32_t PD13               :  1;
            __IO uint32_t                    : 18;
        } PDR_b;
    };
    union {
        __IO uint32_t ODR;                 /*!< Address Offset: 0x028  GPIO输出类型寄存器  */
        struct {
            __IO uint32_t OD0                :  1;
            __IO uint32_t OD1                :  1;
            __IO uint32_t OD2                :  1;
            __IO uint32_t OD3                :  1;
            __IO uint32_t OD4                :  1;
            __IO uint32_t OD5                :  1;
            __IO uint32_t OD6                :  1;
            __IO uint32_t OD7                :  1;
            __IO uint32_t OD8                :  1;
            __IO uint32_t OD9                :  1;
            __IO uint32_t OD10               :  1;
            __IO uint32_t OD11               :  1;
            __IO uint32_t OD12               :  1;
            __IO uint32_t OD13               :  1;
            __IO uint32_t                    : 18;
        } ODR_b;
    };
    union {
        __IO uint32_t DER;                 /*!< Address Offset: 0x02C  GPIO同步使能寄存器  */
        struct {
            __IO uint32_t DE0                :  1;
            __IO uint32_t DE1                :  1;
            __IO uint32_t DE2                :  1;
            __IO uint32_t DE3                :  1;
            __IO uint32_t DE4                :  1;
            __IO uint32_t DE5                :  1;
            __IO uint32_t DE6                :  1;
            __IO uint32_t DE7                :  1;
            __IO uint32_t DE8                :  1;
            __IO uint32_t DE9                :  1;
            __IO uint32_t DE10               :  1;
            __IO uint32_t DE11               :  1;
            __IO uint32_t DE12               :  1;
            __IO uint32_t DE13               :  1;
            __IO uint32_t                    : 18;
        } DER_b;
    };
    union {
        __IO uint32_t DTR;                 /*!< Address Offset: 0x030  GPIO消抖设置寄存器  */
        struct {
            __IO uint32_t DT                 :  3;
            __IO uint32_t                    : 29;
        } DTR_b;
    };
    union {
        __IO uint32_t MXR1;                 /*!< Address Offset: 0x034  GPIO功能复用寄存器1  */
        struct {
            __IO uint32_t PM0                :  4;
            __IO uint32_t PM1                :  4;
            __IO uint32_t PM2                :  4;
            __IO uint32_t PM3                :  4;
            __IO uint32_t PM4                :  4;
            __IO uint32_t PM5                :  4;
            __IO uint32_t PM6                :  4;
            __IO uint32_t PM7                :  4;
        } MXR1_b;
    };
    union {
        __IO uint32_t MXR2;                 /*!< Address Offset: 0x038  GPIO功能复用寄存器2  */
        struct {
            __IO uint32_t PM8                :  4;
            __IO uint32_t PM9                :  4;
            __IO uint32_t PM10               :  4;
            __IO uint32_t PM11               :  4;
            __IO uint32_t PM12               :  4;
            __IO uint32_t PM13               :  4;
            __IO uint32_t                    :  8;
        } MXR2_b;
    };
} GPIO_TypeDef;
/**
  * @brief GDU (novosense) 
 */
typedef struct {
    union {
        __IO uint32_t CR;                 /*!< Address Offset: 0x000  GDU控制寄存器  */
        struct {
            __IO uint32_t GDUB0EN            :  1;
            __IO uint32_t GDUB1EN            :  1;
            __IO uint32_t GDUB2EN            :  1;
            __IO uint32_t BOVASTEN           :  1;
            __IO uint32_t BUVASTEN           :  1;
            __IO uint32_t OCSHUNTEN          :  1;
            __IO uint32_t ASTMOD             :  1;
            __IO uint32_t OCSDM              :  1;
            __IO uint32_t SC0EN              :  1;
            __IO uint32_t SC1EN              :  1;
            __IO uint32_t SC2EN              :  1;
            __IO uint32_t SC3EN              :  1;
            __IO uint32_t SC4EN              :  1;
            __IO uint32_t SC5EN              :  1;
            __IO uint32_t CSAEN              :  1;
            __IO uint32_t COVASTEN           :  1;
            __IO uint32_t CUVASTEN           :  1;
            __IO uint32_t ONSEQEN            :  1;
            __IO uint32_t OFFSEQEN           :  1;
            __IO uint32_t BEMFCBUFFEN        :  1;
            __IO uint32_t BEMFCEN            :  1;
            __IO uint32_t GDUGLOBLEN         :  1;
            __IO uint32_t WAITEN             :  1;
            __IO uint32_t OCHYST             :  2;
            __IO uint32_t OCHYSTMOD          :  1;
            __IO uint32_t HS_FDISCHGEN       :  1;
            __IO uint32_t LS_FDISCHGEN       :  1;
            __IO uint32_t CCPT               :  3;
            __IO uint32_t                    :  1;
        } CR_b;
    };
    union {
        __IO uint32_t SCTHCR;                 /*!< Address Offset: 0x004  VDS短路检测阈值以及滤波配置寄存器  */
        struct {
            __IO uint32_t SCTHHS             :  3;
            __IO uint32_t SCTHLS             :  3;
            __IO uint32_t SCMSK              :  8;
            __IO uint32_t SCDEB              :  9;
            __IO uint32_t OCDEB              :  9;
        } SCTHCR_b;
    };
    union {
        __IO uint32_t GDUMUX;                 /*!< Address Offset: 0x008  GDU mux寄存器  */
        struct {
            __IO uint32_t GPMUX              :  2;
            __IO uint32_t                    : 30;
        } GDUMUX_b;
    };
    union {
        __IO uint32_t CSACR;                 /*!< Address Offset: 0x00C  Current Sense AMP寄存器  */
        struct {
            __IO uint32_t CALIBMOD           :  1;
            __IO uint32_t OFFSET             :  4;
            __IO uint32_t                    : 27;
        } CSACR_b;
    };
    union {
        __IO uint32_t OFFSEQCR;                 /*!< Address Offset: 0x010  off slew rate 控制寄存器  */
        struct {
            __IO uint32_t OFFT1              :  3;
            __IO uint32_t OFFI1              :  4;
            __IO uint32_t                    :  1;
            __IO uint32_t OFFT2              :  3;
            __IO uint32_t OFFI2              :  4;
            __IO uint32_t                    :  1;
            __IO uint32_t OFFT3              :  3;
            __IO uint32_t OFFI3              :  4;
            __IO uint32_t                    :  1;
            __IO uint32_t OFFT4              :  3;
            __IO uint32_t OFFI4              :  4;
            __IO uint32_t                    :  1;
        } OFFSEQCR_b;
    };
    union {
        __IO uint32_t ONSEQCR;                 /*!< Address Offset: 0x014  ON slew rate 控制寄存器  */
        struct {
            __IO uint32_t ONT1               :  3;
            __IO uint32_t ONI1               :  4;
            __IO uint32_t                    :  1;
            __IO uint32_t ONT2               :  3;
            __IO uint32_t ONI2               :  4;
            __IO uint32_t                    :  1;
            __IO uint32_t ONT3               :  3;
            __IO uint32_t ONI3               :  4;
            __IO uint32_t                    :  1;
            __IO uint32_t ONT4               :  3;
            __IO uint32_t ONI4               :  4;
            __IO uint32_t                    :  1;
        } ONSEQCR_b;
    };
    union {
        __IO uint32_t BEMFCR0;                 /*!< Address Offset: 0x018  bemfc控制寄存器0  */
        struct {
            __IO uint32_t INV0               :  1;
            __IO uint32_t INV1               :  1;
            __IO uint32_t INV2               :  1;
            __IO uint32_t BEMFCHYST          :  2;
            __IO uint32_t BHYSTMOD           :  1;
            __IO uint32_t                    :  3;
            __IO uint32_t BUEOP              :  1;
            __IO uint32_t BUTRG              :  1;
            __IO uint32_t BLKT0              :  4;
            __IO uint32_t BLKT1              :  4;
            __IO uint32_t BLKT2              :  4;
            __IO uint32_t                    :  1;
            __IO uint32_t DGL0               :  2;
            __IO uint32_t DGL1               :  2;
            __IO uint32_t DGL2               :  2;
            __IO uint32_t                    :  2;
        } BEMFCR0_b;
    };
    union {
        __IO uint32_t BEMFCR1;                 /*!< Address Offset: 0x01C  bemfc控制寄存器1  */
        struct {
            __IO uint32_t ITS0               :  2;
            __IO uint32_t DTP0               :  2;
            __IO uint32_t DTS0               :  4;
            __IO uint32_t ITS1               :  2;
            __IO uint32_t DTP1               :  2;
            __IO uint32_t DTS1               :  4;
            __IO uint32_t ITS2               :  2;
            __IO uint32_t DTP2               :  2;
            __IO uint32_t DTS2               :  4;
            __IO uint32_t                    :  8;
        } BEMFCR1_b;
    };
    union {
        __IO uint32_t CLDAC;                 /*!< Address Offset: 0x020  CLDAC寄存器  */
        struct {
            __IO uint32_t DATA               :  8;
            __IO uint32_t                    : 24;
        } CLDAC_b;
    };
    union {
        __IO uint32_t IEN;                 /*!< Address Offset: 0x024  中断使能寄存器  */
        struct {
            __IO uint32_t OCREG0HIE          :  1;
            __IO uint32_t OCREG1HIE          :  1;
            __IO uint32_t OCREG2HIE          :  1;
            __IO uint32_t OCREG0LIE          :  1;
            __IO uint32_t OCREG1LIE          :  1;
            __IO uint32_t OCREG2LIE          :  1;
            __IO uint32_t OCSHUNTIE          :  1;
            __IO uint32_t BEMFCPI0IE         :  1;
            __IO uint32_t BEMFCPI1IE         :  1;
            __IO uint32_t BEMFCPI2IE         :  1;
            __IO uint32_t                    : 22;
        } IEN_b;
    };
    union {
        __IO uint32_t ISR;                 /*!< Address Offset: 0x028  中断状态寄存器  */
        struct {
            __IO uint32_t OCREG0HIF          :  1;
            __IO uint32_t OCREG1HIF          :  1;
            __IO uint32_t OCREG2HIF          :  1;
            __IO uint32_t OCREG0LIF          :  1;
            __IO uint32_t OCREG1LIF          :  1;
            __IO uint32_t OCREG2LIF          :  1;
            __IO uint32_t OCSHUNTIF          :  1;
            __IO uint32_t BEMFCPI0IF         :  1;
            __IO uint32_t BEMFCPI1IF         :  1;
            __IO uint32_t BEMFCPI2IF         :  1;
            __IO uint32_t BEMFCCD0IF         :  1;
            __IO uint32_t BEMFCCD1IF         :  1;
            __IO uint32_t BEMFCCD2IF         :  1;
            __IO uint32_t                    : 19;
        } ISR_b;
    };
} GDU_TypeDef;
/**
  * @brief SPI (novosense) 
 */
typedef struct {
    union {
        __IO uint32_t CR;                 /*!< Address Offset: 0x000  SPI控制寄存器  */
        struct {
            __IO uint32_t EN                 :  1;
            __IO uint32_t CPHA               :  1;
            __IO uint32_t CPOL               :  1;
            __IO uint32_t LSBF               :  1;
            __IO uint32_t DWID               :  2;
            __IO uint32_t WMODE              :  1;
            __IO uint32_t SMODE              :  1;
            __IO uint32_t SPIC               :  4;
            __IO uint32_t TXFCV              :  4;
            __IO uint32_t RXFCV              :  4;
            __IO uint32_t TXFCLR             :  1;
            __IO uint32_t RXFCLR             :  1;
            __IO uint32_t                    : 10;
        } CR_b;
    };
    union {
        __IO uint32_t SR;                 /*!< Address Offset: 0x004  SPI状态寄存器  */
        struct {
            __IO uint32_t BUSY               :  1;
            __IO uint32_t                    :  3;
            __IO uint32_t TXFE               :  1;
            __IO uint32_t TXFF               :  1;
            __IO uint32_t TXFC               :  1;
            __IO uint32_t TXFO               :  1;
            __IO uint32_t TXFLVL             :  4;
            __IO uint32_t RXFE               :  1;
            __IO uint32_t RXFF               :  1;
            __IO uint32_t RXFC               :  1;
            __IO uint32_t RXFO               :  1;
            __IO uint32_t RXFLVL             :  4;
            __IO uint32_t                    : 12;
        } SR_b;
    };
    union {
        __IO uint32_t TXF;                 /*!< Address Offset: 0x008  SPI TX FIFO数据寄存器  */
        struct {
            __IO uint32_t DATA               : 32;
        } TXF_b;
    };
    union {
        __IO uint32_t RXF;                 /*!< Address Offset: 0x00C  SPI RX FIFO数据寄存器  */
        struct {
            __IO uint32_t DATA               : 32;
        } RXF_b;
    };
    union {
        __IO uint32_t IEN;                 /*!< Address Offset: 0x010  中断使能寄存器  */
        struct {
            __IO uint32_t TXFEIE             :  1;
            __IO uint32_t TXFNFIE            :  1;
            __IO uint32_t TXFCIE             :  1;
            __IO uint32_t TXFOIE             :  1;
            __IO uint32_t RXFNEIE            :  1;
            __IO uint32_t RXFFIE             :  1;
            __IO uint32_t RXFCIE             :  1;
            __IO uint32_t RXFOIE             :  1;
            __IO uint32_t                    : 24;
        } IEN_b;
    };
    union {
        __IO uint32_t ISR;                 /*!< Address Offset: 0x014  中断使能置起寄存器  */
        struct {
            __IO uint32_t TXFEIF             :  1;
            __IO uint32_t TXFNFIF            :  1;
            __IO uint32_t TXFCIF             :  1;
            __IO uint32_t TXFOIF             :  1;
            __IO uint32_t RXFNEIF            :  1;
            __IO uint32_t RXFFIF             :  1;
            __IO uint32_t RXFCIF             :  1;
            __IO uint32_t RXFOIF             :  1;
            __IO uint32_t                    : 24;
        } ISR_b;
    };
} SPI_TypeDef;
/**
  * @brief LINPORT (novosense) 
 */
typedef struct {
    union {
        __IO uint32_t BSMCR;                 /*!< Address Offset: 0x00  BSM控制寄存器  */
        struct {
            __IO uint32_t AUTOEN             :  1;
            __IO uint32_t SWPU               :  1;
            __IO uint32_t SWCS0              :  1;
            __IO uint32_t SWCS1              :  1;
            __IO uint32_t SWCS2              :  1;
            __IO uint32_t                    : 27;
        } BSMCR_b;
    };
    union {
        __IO uint32_t IEN;                 /*!< Address Offset: 0x04  中断使能寄存器  */
        struct {
            __IO uint32_t LINFEIE            :  1;
            __IO uint32_t LINREIE            :  1;
            __IO uint32_t OCIE               :  1;
            __IO uint32_t DOMTOIE            :  1;
            __IO uint32_t BENDIE             :  1;
            __IO uint32_t                    : 27;
        } IEN_b;
    };
    union {
        __IO uint32_t ISR;                 /*!< Address Offset: 0x08  软件触发中断寄存器  */
        struct {
            __IO uint32_t LINFEIF            :  1;
            __IO uint32_t LINREIF            :  1;
            __IO uint32_t OCIF               :  1;
            __IO uint32_t DOMTOIF            :  1;
            __IO uint32_t BENDIF             :  1;
            __IO uint32_t                    : 27;
        } ISR_b;
    };
    __IO uint32_t reserved0[1];
    union {
        __IO uint32_t DIO;                 /*!< Address Offset: 0x10  数据寄存器  */
        struct {
            __IO uint32_t LINDI              :  1;
            __IO uint32_t LINDO              :  1;
            __IO uint32_t                    : 30;
        } DIO_b;
    };
    union {
        __IO uint32_t CR;                 /*!< Address Offset: 0x14  控制寄存器  */
        struct {
            __IO uint32_t RXFT               :  4;
            __IO uint32_t OCFT               :  2;
            __IO uint32_t LAIS               :  1;
            __IO uint32_t PAIS               :  1;
            __IO uint32_t TXOS               :  2;
            __IO uint32_t SR                 :  2;
            __IO uint32_t DOMTOEN            :  1;
            __IO uint32_t LOM                :  1;
            __IO uint32_t                    :  1;
            __IO uint32_t LINPORTEN          :  1;
            __IO uint32_t RXEN               :  1;
            __IO uint32_t                    : 15;
        } CR_b;
    };
    union {
        __IO uint32_t TIMEOUT;                 /*!< Address Offset: 0x18  LIN TX 超时寄存器  */
        struct {
            __IO uint32_t TIMEOUT            : 22;
            __IO uint32_t                    : 10;
        } TIMEOUT_b;
    };
} LINPORT_TypeDef;
/**
  * @brief LINUART (novosense) 
 */
typedef struct {
    union {
        __IO uint32_t CR;                 /*!< Address Offset: 0x00  控制寄存器  */
        struct {
            __IO uint32_t TXRXE              :  1;
            __IO uint32_t                    :  1;
            __IO uint32_t TXFE               :  1;
            __IO uint32_t RXFE               :  1;
            __IO uint32_t UE                 :  1;
            __IO uint32_t                    :  2;
            __IO uint32_t STOP               :  1;
            __IO uint32_t                    :  6;
            __IO uint32_t PS                 :  1;
            __IO uint32_t PCE                :  1;
            __IO uint32_t AUTOBRR            :  1;
            __IO uint32_t LINMODE            :  1;
            __IO uint32_t TXFCOMP            :  4;
            __IO uint32_t RXFCOMP            :  4;
            __IO uint32_t TEST               :  2;
            __IO uint32_t BRRWINE            :  1;
            __IO uint32_t EAUTOBRR           :  1;
            __IO uint32_t TXLOOPE            :  1;
            __IO uint32_t                    :  1;
        } CR_b;
    };
    union {
        __IO uint32_t BRR;                 /*!< Address Offset: 0x04  波特率寄存器  */
        struct {
            __IO uint32_t BRR                : 20;
            __IO uint32_t                    : 12;
        } BRR_b;
    };
    union {
        __IO uint32_t RDATA;                 /*!< Address Offset: 0x08  接收数据寄存器  */
        struct {
            __IO uint32_t RDATA              :  9;
            __IO uint32_t                    : 23;
        } RDATA_b;
    };
    union {
        __IO uint32_t TDATA;                 /*!< Address Offset: 0x0c  发送数据寄存器  */
        struct {
            __IO uint32_t TDATA              :  9;
            __IO uint32_t                    : 23;
        } TDATA_b;
    };
    union {
        __IO uint32_t IEN;                 /*!< Address Offset: 0x10  中断使能寄存器  */
        struct {
            __IO uint32_t TXENDIE            :  1;
            __IO uint32_t RXCIE              :  1;
            __IO uint32_t RXBUSYIE           :  1;
            __IO uint32_t TXBUSYIE           :  1;
            __IO uint32_t RXOVIE             :  1;
            __IO uint32_t PEIE               :  1;
            __IO uint32_t BRSYIE             :  1;
            __IO uint32_t TXIE               :  1;
            __IO uint32_t FEIE               :  1;
            __IO uint32_t TIMEOUTIE          :  1;
            __IO uint32_t TXFOVIE            :  1;
            __IO uint32_t RXFOVIE            :  1;
            __IO uint32_t TXFFIE             :  1;
            __IO uint32_t RXFFIE             :  1;
            __IO uint32_t TXFEIE             :  1;
            __IO uint32_t RXFEIE             :  1;
            __IO uint32_t TXFLCIE            :  1;
            __IO uint32_t RXFLCIE            :  1;
            __IO uint32_t TXFCIE             :  1;
            __IO uint32_t TXERRIE            :  1;
            __IO uint32_t BRKIE              :  1;
            __IO uint32_t BRRERRIE           :  1;
            __IO uint32_t                    : 10;
        } IEN_b;
    };
    union {
        __IO uint32_t ISR;                 /*!< Address Offset: 0x14  中断状态寄存器  */
        struct {
            __IO uint32_t TXENDIF            :  1;
            __IO uint32_t RXCIF              :  1;
            __IO uint32_t RBSYIF             :  1;
            __IO uint32_t TBSYIF             :  1;
            __IO uint32_t RXOVIF             :  1;
            __IO uint32_t PEIF               :  1;
            __IO uint32_t BRSYIF             :  1;
            __IO uint32_t TXIF               :  1;
            __IO uint32_t FEIF               :  1;
            __IO uint32_t TOIF               :  1;
            __IO uint32_t TXFOVIF            :  1;
            __IO uint32_t RXFOVIF            :  1;
            __IO uint32_t TXFFIF             :  1;
            __IO uint32_t RXFFIF             :  1;
            __IO uint32_t TXFEIF             :  1;
            __IO uint32_t RXFEIF             :  1;
            __IO uint32_t TXFLIF             :  1;
            __IO uint32_t RXFLIF             :  1;
            __IO uint32_t TXFCIF             :  1;
            __IO uint32_t TXERRIF            :  1;
            __IO uint32_t BRKIF              :  1;
            __IO uint32_t TXFFILEVELF        :  5;
            __IO uint32_t RXFFILEVELF        :  5;
            __IO uint32_t BRRERRIF           :  1;
        } ISR_b;
    };
    union {
        __IO uint32_t RXFIFO;                 /*!< Address Offset: 0x18  接收FIFO数据寄存器  */
        struct {
            __IO uint32_t RXFIFO             :  9;
            __IO uint32_t                    : 23;
        } RXFIFO_b;
    };
    union {
        __IO uint32_t TXFIFO;                 /*!< Address Offset: 0x1C  发送FIFO数据寄存器  */
        struct {
            __IO uint32_t TXFIFO             :  9;
            __IO uint32_t                    : 23;
        } TXFIFO_b;
    };
    union {
        __IO uint32_t TIMEOUT;                 /*!< Address Offset: 0x20  LIN RX 超时寄存器  */
        struct {
            __IO uint32_t TIMEOUT            : 22;
            __IO uint32_t                    : 10;
        } TIMEOUT_b;
    };
    __IO uint32_t reserved0[1];
    union {
        __IO uint32_t WINDW_MIN;                 /*!< Address Offset: 0x28    */
        struct {
            __IO uint32_t WINDW_MIN          : 20;
            __IO uint32_t                    : 12;
        } WINDW_MIN_b;
    };
    union {
        __IO uint32_t WINDW_MAX;                 /*!< Address Offset: 0x2c    */
        struct {
            __IO uint32_t WINDW_MAX          : 20;
            __IO uint32_t                    : 12;
        } WINDW_MAX_b;
    };
    union {
        __IO uint32_t TX_DATACHK_DLY;                 /*!< Address Offset: 0x30    */
        struct {
            __IO uint32_t SR_DLY             :  2;
            __IO uint32_t DEGLITCH_DLY       :  3;
            __IO uint32_t                    : 27;
        } TX_DATACHK_DLY_b;
    };
} LINUART_TypeDef;
/**
  * @brief PWMIO (novosense) 
 */
typedef struct {
    union {
        __IO uint32_t CR;                 /*!< Address Offset: 0x000  PWMIO控制寄存器  */
        struct {
            __IO uint32_t IENA               :  1;
            __IO uint32_t OENA               :  1;
            __IO uint32_t OM                 :  2;
            __IO uint32_t FDUPL              :  1;
            __IO uint32_t IDGL               :  1;
            __IO uint32_t ORBE               :  1;
            __IO uint32_t EDFEN              :  1;
            __IO uint32_t ICLK               :  4;
            __IO uint32_t OCLK               :  4;
            __IO uint32_t                    : 16;
        } CR_b;
    };
    union {
        __IO uint32_t IHT;                 /*!< Address Offset: 0x004  输入PWM高电平时间寄存器  */
        struct {
            __IO uint32_t IHT                : 14;
            __IO uint32_t                    : 18;
        } IHT_b;
    };
    union {
        __IO uint32_t ILT;                 /*!< Address Offset: 0x008  输入PWM低电平时间寄存器  */
        struct {
            __IO uint32_t ILT                : 14;
            __IO uint32_t                    : 18;
        } ILT_b;
    };
    union {
        __IO uint32_t ICNT;                 /*!< Address Offset: 0x00C  输入PWM低电平计数器寄存器  */
        struct {
            __IO uint32_t ICNT               : 14;
            __IO uint32_t                    : 18;
        } ICNT_b;
    };
    union {
        __IO uint32_t OPT;                 /*!< Address Offset: 0x010  输出PWM周期时间寄存器  */
        struct {
            __IO uint32_t OPT                : 14;
            __IO uint32_t                    : 18;
        } OPT_b;
    };
    union {
        __IO uint32_t OLT;                 /*!< Address Offset: 0x014  输出PWM低电平时间寄存器  */
        struct {
            __IO uint32_t OLT                : 14;
            __IO uint32_t                    : 18;
        } OLT_b;
    };
    union {
        __IO uint32_t OCNT;                 /*!< Address Offset: 0x018  输出PWM定时器数值寄存器  */
        struct {
            __IO uint32_t OCNT               : 14;
            __IO uint32_t                    : 18;
        } OCNT_b;
    };
    union {
        __IO uint32_t IEN;                 /*!< Address Offset: 0x01C  中断使能寄存器  */
        struct {
            __IO uint32_t IREIE              :  1;
            __IO uint32_t IFEIE              :  1;
            __IO uint32_t ICOFIE             :  1;
            __IO uint32_t                    :  1;
            __IO uint32_t OEOPIE             :  1;
            __IO uint32_t                    : 27;
        } IEN_b;
    };
    union {
        __IO uint32_t IPND;                 /*!< Address Offset: 0x020  中断置起寄存器  */
        struct {
            __IO uint32_t IRE                :  1;
            __IO uint32_t IFE                :  1;
            __IO uint32_t ICOF               :  1;
            __IO uint32_t                    :  1;
            __IO uint32_t OEOP               :  1;
            __IO uint32_t                    : 27;
        } IPND_b;
    };
    union {
        __IO uint32_t ISR;                 /*!< Address Offset: 0x024  中断使能置起寄存器  */
        struct {
            __IO uint32_t IREIF              :  1;
            __IO uint32_t IFEIF              :  1;
            __IO uint32_t ICOFIF             :  1;
            __IO uint32_t                    :  1;
            __IO uint32_t OEOPIF             :  1;
            __IO uint32_t                    : 27;
        } ISR_b;
    };
} PWMIO_TypeDef;
/**
  * @brief TIMER (novosense) 
 */
typedef struct {
    union {
        __IO uint32_t CR;                 /*!< Address Offset: 0x000  定时器控制寄存器  */
        struct {
            __IO uint32_t EN                 :  1;
            __IO uint32_t CM                 :  1;
            __IO uint32_t MOD                :  2;
            __IO uint32_t CIF                :  3;
            __IO uint32_t                    :  1;
            __IO uint32_t CIP                :  2;
            __IO uint32_t ARBE               :  1;
            __IO uint32_t CCBE               :  1;
            __IO uint32_t UEM                :  1;
            __IO uint32_t UIM                :  1;
            __IO uint32_t TU                 :  1;
            __IO uint32_t CCU                :  1;
            __IO uint32_t OCM                :  3;
            __IO uint32_t INMUX              :  2;
            __IO uint32_t                    : 11;
        } CR_b;
    };
    union {
        __IO uint32_t CNT;                 /*!< Address Offset: 0x004  定时器计数器数值寄存器  */
        struct {
            __IO uint32_t CNT                : 16;
            __IO uint32_t                    : 16;
        } CNT_b;
    };
    union {
        __IO uint32_t PS;                 /*!< Address Offset: 0x008  定时器预分频寄存器  */
        struct {
            __IO uint32_t PS                 : 16;
            __IO uint32_t                    : 16;
        } PS_b;
    };
    union {
        __IO uint32_t AR;                 /*!< Address Offset: 0x00C  定时器自动加载寄存器  */
        struct {
            __IO uint32_t AR                 : 16;
            __IO uint32_t                    : 16;
        } AR_b;
    };
    union {
        __IO uint32_t CC;                 /*!< Address Offset: 0x010  捕获比较寄存器  */
        struct {
            __IO uint32_t CC                 : 16;
            __IO uint32_t                    : 16;
        } CC_b;
    };
    union {
        __IO uint32_t IEN;                 /*!< Address Offset: 0x014  中断使能寄存器  */
        struct {
            __IO uint32_t UPDIE              :  1;
            __IO uint32_t CMPIE              :  1;
            __IO uint32_t CAPIE              :  1;
            __IO uint32_t COFIE              :  1;
            __IO uint32_t                    : 28;
        } IEN_b;
    };
    union {
        __IO uint32_t IPND;                 /*!< Address Offset: 0x018  中断置起寄存器  */
        struct {
            __IO uint32_t UPD                :  1;
            __IO uint32_t CMP                :  1;
            __IO uint32_t CAP                :  1;
            __IO uint32_t COF                :  1;
            __IO uint32_t                    : 28;
        } IPND_b;
    };
    union {
        __IO uint32_t ISR;                 /*!< Address Offset: 0x01C  中断使能置起寄存器  */
        struct {
            __IO uint32_t UPDIF              :  1;
            __IO uint32_t CMPIF              :  1;
            __IO uint32_t CAPIF              :  1;
            __IO uint32_t COFIF              :  1;
            __IO uint32_t                    : 28;
        } ISR_b;
    };
} TIMER_TypeDef;
/**
  * @brief WWDG (novosense) 
 */
typedef struct {
    union {
        __IO uint32_t WCKSEL;                 /*!< Address Offset: 0x000  WWDG唤醒时钟选择寄存器  */
        struct {
            __IO uint32_t WCKSEL             :  3;
            __IO uint32_t                    : 29;
        } WCKSEL_b;
    };
    union {
        __IO uint32_t CKSEL;                 /*!< Address Offset: 0x004  WWDG计数器时钟选择寄存器  */
        struct {
            __IO uint32_t CKSEL              :  3;
            __IO uint32_t                    : 29;
        } CKSEL_b;
    };
    union {
        __IO uint32_t TWIN;                 /*!< Address Offset: 0x008  触发窗口寄存器  */
        struct {
            __IO uint32_t TWIN               :  8;
            __IO uint32_t                    : 24;
        } TWIN_b;
    };
    union {
        __IO uint32_t TRG;                 /*!< Address Offset: 0x00C  触发寄存器寄存器  */
        struct {
            __IO uint32_t TRG                :  8;
            __IO uint32_t                    : 24;
        } TRG_b;
    };
    union {
        __IO uint32_t CNT;                 /*!< Address Offset: 0x010  WWDG 计数器数值寄存器  */
        struct {
            __IO uint32_t CNT                :  8;
            __IO uint32_t                    : 24;
        } CNT_b;
    };
    union {
        __IO uint32_t INTCNT;                 /*!< Address Offset: 0x014  提前中断窗口寄存器  */
        struct {
            __IO uint32_t  INTR              :  4;
            __IO uint32_t                    : 28;
        } INTCNT_b;
    };
    union {
        __IO uint32_t IEN;                 /*!< Address Offset: 0x018  WWDG看门狗中断使能  */
        struct {
            __IO uint32_t AHIE               :  1;
            __IO uint32_t                    : 31;
        } IEN_b;
    };
    union {
        __IO uint32_t ISR;                 /*!< Address Offset: 0x01C  中断状态寄存器  */
        struct {
            __IO uint32_t AHIF               :  1;
            __IO uint32_t                    : 31;
        } ISR_b;
    };
} WWDG_TypeDef;
/**
  * @brief DWDG (novosense) 
 */
typedef struct {
    union {
        __IO uint32_t CKSEL;                 /*!< Address Offset: 0x000  DWDG时钟选择寄存器  */
        struct {
            __IO uint32_t CKSEL              :  3;
            __IO uint32_t                    :  5;
            __IO uint32_t INTRWIND           :  8;
            __IO uint32_t                    : 16;
        } CKSEL_b;
    };
    union {
        __IO uint32_t RR;                 /*!< Address Offset: 0x004  DWDG加载寄存器  */
        struct {
            __IO uint32_t RR                 : 16;
            __IO uint32_t                    : 16;
        } RR_b;
    };
    union {
        __IO uint32_t TRG;                 /*!< Address Offset: 0x008  DWDG触发寄存器  */
        struct {
            __IO uint32_t TRG                :  8;
            __IO uint32_t                    : 24;
        } TRG_b;
    };
    union {
        __IO uint32_t CNT;                 /*!< Address Offset: 0x00C  DWDG计数器数值寄存器  */
        struct {
            __IO uint32_t CNT                : 16;
            __IO uint32_t                    : 16;
        } CNT_b;
    };
    union {
        __IO uint32_t IEN;                 /*!< Address Offset: 0x010  中断使能寄存器  */
        struct {
            __IO uint32_t AHIE               :  1;
            __IO uint32_t                    : 31;
        } IEN_b;
    };
    union {
        __IO uint32_t ISR;                 /*!< Address Offset: 0x014  中断状态寄存器  */
        struct {
            __IO uint32_t AHIF               :  1;
            __IO uint32_t                    : 31;
        } ISR_b;
    };
} DWDG_TypeDef;
/**
  * @brief SSI (novosense) 
 */
typedef struct {
    union {
        __IO uint32_t CTRLR0;                 /*!< Address Offset: 0x0  Control Register 0 ,Width:16 bits  */
        struct {
            __IO uint32_t                    :  4;
            __IO uint32_t FRF                :  2;
            __IO uint32_t SCPH               :  1;
            __IO uint32_t SCPOL              :  1;
            __IO uint32_t TMOD               :  2;
            __IO uint32_t SLV_OE             :  1;
            __IO uint32_t SRL                :  1;
            __IO uint32_t CFS                :  4;
            __IO uint32_t DFS                :  4;
            __IO uint32_t                    : 12;
        } CTRLR0_b;
    };
    __IO uint32_t reserved0[1];
    union {
        __IO uint32_t SSIENR;                 /*!< Address Offset: 0x08  SSI Enable Register  */
        struct {
            __IO uint32_t SSI_EN             :  1;
            __IO uint32_t                    : 31;
        } SSIENR_b;
    };
    union {
        __IO uint32_t MWCR;                 /*!< Address Offset: 0x0C  Microwire Control Register  */
        struct {
            __IO uint32_t MWMOD              :  1;
            __IO uint32_t MDD                :  1;
            __IO uint32_t                    : 30;
        } MWCR_b;
    };
    __IO uint32_t reserved1[2];
    union {
        __IO uint32_t TXFTLR;                 /*!< Address Offset: 0x18  Transmit FIFO Threshold Level  */
        struct {
            __IO uint32_t TFT                :  2;
            __IO uint32_t                    : 30;
        } TXFTLR_b;
    };
    union {
        __IO uint32_t RXFTLR;                 /*!< Address Offset: 0x1C  Receive FIFO Threshold Level  */
        struct {
            __IO uint32_t RFT                :  2;
            __IO uint32_t                    : 30;
        } RXFTLR_b;
    };
    union {
        __IO uint32_t TXFLR;                 /*!< Address Offset: 0x20  Transmit FIFO Level Register  */
        struct {
            __IO uint32_t TXFLR              :  3;
            __IO uint32_t                    : 29;
        } TXFLR_b;
    };
    union {
        __IO uint32_t RXFLR;                 /*!< Address Offset: 0x24  Receive FIFO Level Register  */
        struct {
            __IO uint32_t RXFLR              :  3;
            __IO uint32_t                    : 29;
        } RXFLR_b;
    };
    union {
        __IO uint32_t SR;                 /*!< Address Offset: 0x28  Status Register  */
        struct {
            __IO uint32_t BUSY               :  1;
            __IO uint32_t TFNF               :  1;
            __IO uint32_t TFE                :  1;
            __IO uint32_t RFNE               :  1;
            __IO uint32_t RFF                :  1;
            __IO uint32_t TXE                :  1;
            __IO uint32_t                    : 26;
        } SR_b;
    };
    union {
        __IO uint32_t IMR;                 /*!< Address Offset: 0x2C  Interrupt Mask Register  */
        struct {
            __IO uint32_t TXEIM              :  1;
            __IO uint32_t TXOIM              :  1;
            __IO uint32_t RXUIM              :  1;
            __IO uint32_t RXOIM              :  1;
            __IO uint32_t RXFIM              :  1;
            __IO uint32_t                    : 27;
        } IMR_b;
    };
    union {
        __IO uint32_t ISR;                 /*!< Address Offset: 0x30  Interrupt Status Register  */
        struct {
            __IO uint32_t TXEIS              :  1;
            __IO uint32_t TXOIS              :  1;
            __IO uint32_t RXUIS              :  1;
            __IO uint32_t RXOIS              :  1;
            __IO uint32_t RXFIS              :  1;
            __IO uint32_t                    : 27;
        } ISR_b;
    };
    union {
        __IO uint32_t RISR;                 /*!< Address Offset: 0x34  Raw Interrupt Status Register  */
        struct {
            __IO uint32_t TXEIR              :  1;
            __IO uint32_t TXOIR              :  1;
            __IO uint32_t RXUIR              :  1;
            __IO uint32_t RXOIR              :  1;
            __IO uint32_t RXFIR              :  1;
            __IO uint32_t                    : 27;
        } RISR_b;
    };
    union {
        __IO uint32_t TXOICR;                 /*!< Address Offset: 0x38  Transmit FIFO Overflow Interrupt Clear Register  */
        struct {
            __IO uint32_t TXOICR             :  1;
            __IO uint32_t                    : 31;
        } TXOICR_b;
    };
    union {
        __IO uint32_t RXOICR;                 /*!< Address Offset: 0x3C  Receive FIFO Overflow Interrupt Clear Register  */
        struct {
            __IO uint32_t RXOICR             :  1;
            __IO uint32_t                    : 31;
        } RXOICR_b;
    };
    union {
        __IO uint32_t RXUICR;                 /*!< Address Offset: 0x40  Receive FIFO Underflow Interrupt Clear Register  */
        struct {
            __IO uint32_t RXUICR             :  1;
            __IO uint32_t                    : 31;
        } RXUICR_b;
    };
    __IO uint32_t reserved2[1];
    union {
        __IO uint32_t ICR;                 /*!< Address Offset: 0x48  Interrupt Clear Register  */
        struct {
            __IO uint32_t ICR                :  1;
            __IO uint32_t                    : 31;
        } ICR_b;
    };
    __IO uint32_t reserved3[3];
    union {
        __IO uint32_t IDR;                 /*!< Address Offset: 0x58  Identification Register  */
        struct {
            __IO uint32_t IDCODE             : 32;
        } IDR_b;
    };
    union {
        __IO uint32_t SSI_COMP_VERSION;                 /*!< Address Offset: 0x5C  coreKit version ID register  */
        struct {
            __IO uint32_t SSI_COMP_VERSION   : 32;
        } SSI_COMP_VERSION_b;
    };
    union {
        __IO uint32_t DR;                 /*!< Address Offset: 0x60  Data Register.Width:32-bits, When the register is read, data in the receive FIFO buffer is accessed. When it is written to, data are moved into the transmit FIFO buffer; a write can occur only when SSI_EN = 1. FIFOs are reset when SSI_EN = 0.地址范围0x60~0x64，对应FIFO深度为2  */
        struct {
            __IO uint32_t DR                 : 32;
        } DR_b;
    };
} SSI_TypeDef;
 /******************************************************************************/
/*                         Peripheral memory map                              */
/******************************************************************************/
/* Peripheral and SRAM base address */ 
#define ROM_BASE                 (0x01000000)
#define RAM0_BASE                (0x10000000)
#define RAM1_BASE                (0x20000000)
#define FLASH_BASE               (0x08000000)
#define NVR_BASE                 (0x08010000)
#define FLASHCTRL_BASE           (0x40030000)
#define CORE_BASE                (0xE0000000)
#define SYSCTRL_BASE             (0x40020000)
#define EPWM_BASE                (0x40021000)
#define ADC_BASE                 (0x40022000)
#define GPIO_BASE                (0x40025000)
#define GDU_BASE                 (0x40000000)
#define SPI_BASE                 (0x40001000)
#define LINPORT_BASE             (0x40002000)
#define LINUART_BASE             (0x40003000)
#define PWMIO_BASE               (0x40004000)
#define TIM0_BASE                (0x40005000)
#define TIM1_BASE                (0x40006000)
#define WWDG_BASE                (0x40007000)
#define DWDG_BASE                (0x40008000)
#define UART1_BASE               (0x40009000)
#define SSI_BASE                 (0x4000A000)
/******************************************************************************/
/*                         Peripheral declaration                             */
/******************************************************************************/
#define FLASHCTRL             ((FLASHCTRL_TypeDef *)   FLASHCTRL_BASE)
#define SYSCTRL               ((SYSCTRL_TypeDef *)     SYSCTRL_BASE)
#define EPWM                  ((PWM_TypeDef *)         EPWM_BASE)
#define ADC                   ((ADC_TypeDef *)         ADC_BASE)
#define GPIO                  ((GPIO_TypeDef *)        GPIO_BASE)
#define GDU                   ((GDU_TypeDef *)         GDU_BASE)
#define SPI                   ((SPI_TypeDef *)         SPI_BASE)
#define LINPORT               ((LINPORT_TypeDef *)     LINPORT_BASE)
#define LINUART               ((LINUART_TypeDef *)     LINUART_BASE)
#define PWMIO                 ((PWMIO_TypeDef *)       PWMIO_BASE)
#define TIM0                  ((TIMER_TypeDef *)       TIM0_BASE)
#define TIM1                  ((TIMER_TypeDef *)       TIM1_BASE)
#define WWDG                  ((WWDG_TypeDef *)        WWDG_BASE)
#define DWDG                  ((DWDG_TypeDef *)        DWDG_BASE)
#define UART1                 ((LINUART_TypeDef *)     UART1_BASE)
#define SSI                   ((SSI_TypeDef *)         SSI_BASE)

#endif

