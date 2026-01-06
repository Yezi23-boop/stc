#include "foc_init.h"
#include "foc_config.h"
#include "fault_hw_config.h"

#if ((FOC_EPWM_SHUNT_MODE == 0) || (FOC_EPWM_SHUNT_MODE == -1))
/************************************************************
 * @brief: Initialize epwm module for FOC control
 * @return <None>
 ************************************************************/
void FOC_EpwmInit()
{
    /**************************config EPWM******************************/
    SYSCTRL->LKKEYR            = SYSCTRL_UNLOCK_KEY; // unlock sysctrl
    SYSCTRL->AHBCGR_b.EPWMCEN  = 1;                  // enable pwm clock
    EPWM->PWMRWEN_b.PWMRLDEN   = 0x55;               // enable write register

    //PWM Clock and Period
    EPWM->PWMPSQ_b.PWMPSQ      = 0;                  // prescale clk: 48Mhz/(0+1) = 48Mhz
    EPWM->PWMPERIOD_b.PWMP     = FOC_EPWM_PERIOD;    // epwm period  
    EPWM->PWMCON1_b.POSTPS = 0;                      // 2PWM period generate 1 reload interrupt and 1 adc sampling

    /*epwm mode*/
    EPWM->PWMCON1_b.PTMOD      = 1;                  // 0:edge 1:center 2:single
    EPWM->PWMCON1_b.PWMOUTMODE = 0;                  // 0:complement 1:independent 互补

    EPWM->PWMCON2_b.DILDEN     = 0;                  // duty register 0:when period sync or cntr->0 1:effect immediately
    EPWM->PWMCON2_b.ZDLDEN     = 1;                  // cntr -> 0 update register
    EPWM->PWMCON1_b.PWMSYM     = 1;                  // 0:symmetric 1:asymmetric 互补非对称

    /*duty area*/
    EPWM->PWMCON1_b.PDCON0     = 1;                  // PWM0 0:cntr<duty 1:cntr>duty
    EPWM->PWMCON1_b.PDCON1     = 1;                  // PWM1 0:cntr<duty 1:cntr>duty
    EPWM->PWMCON1_b.PDCON2     = 1;                  // PWM2 0:cntr<duty 1:cntr>duty

    EPWM->PWMCON1_b.PWM0H      = 0;                  // PWM0H output in duty area: 0:high level 1:low level
    EPWM->PWMCON1_b.PWM1H      = 0;                  // PWM1H output in duty area: 0:high level 1:low level
    EPWM->PWMCON1_b.PWM2H      = 0;                  // PWM2H output in duty area: 0:high level 1:low level
    EPWM->PWMCON1_b.PWM0L      = 1;                  // PWM0L output in duty area: 0:high level 1:low level
    EPWM->PWMCON1_b.PWM1L      = 1;                  // PWM1L output in duty area: 0:high level 1:low level
    EPWM->PWMCON1_b.PWM2L      = 1;                  // PWM2L output in duty area: 0:high level 1:low level

    /* config dead time */
    EPWM->PWMDT0H              = FOC_EPWM_DEADTIME; 
    EPWM->PWMDT0L              = FOC_EPWM_DEADTIME;
    EPWM->PWMDT1H              = FOC_EPWM_DEADTIME;
    EPWM->PWMDT1L              = FOC_EPWM_DEADTIME;
    EPWM->PWMDT2H              = FOC_EPWM_DEADTIME;
    EPWM->PWMDT2L              = FOC_EPWM_DEADTIME;

    /* default duty*/
    EPWM->PWM0DH               = FOC_EPWM_PERIOD / 2;
    EPWM->PWM1DH               = FOC_EPWM_PERIOD / 2;
    EPWM->PWM2DH               = FOC_EPWM_PERIOD / 2;

    /* config interrupt priority */
    NVIC_DisableIRQ(EPWM_IRQn);
    NVIC_SetPriority(EPWM_IRQn, EPWM_IRQ_PRIO);

    // 默认DISABLE输出
    EPWM->PMANUALCON2        = 0x00; // manual output xH:0 xL:1
    EPWM->PMANUALCON1        = 0x3F; // output controled by manual

    EPWM->IEN_b.PWMZIE  = 1; // enable period zero interrupt for temprary
    //EPWM->IEN_b.PWMPIE = 1; //用第二个通道的PWM1L匹配中断
    EPWM->PWMENABLE          = 1;
    EPWM->PWMRWEN_b.PWMRLDEN = 0xAA;
}

#endif

/************************************************************
 * @brief: Initialize ADC module for FOC control
 * @return <None>
 ************************************************************/
void FOC_AdcInit(void)
{
    /**************************config ADC******************************/
    SYSCTRL->LKKEYR          = SYSCTRL_UNLOCK_KEY; // unlock sysctrl
    SYSCTRL->AHBSRR_b.ADCREN = 1;               // release adc
    SYSCTRL->AHBCGR_b.ADCCEN = 1;               // enable adc clock
    ADC->CR_b.TSRC     = 4; // trig source 4:EPWM reload
    ADC->CR_b.CKSEL    = 0; // 48Mhz
    ADC->CR_b.REFNSEL  = 0; // internal ref
    ADC->CR_b.SINGLEEN = 0; // eoc trigger: 0:queue conv done 1:single conv done

    ADC->CR_b.INBUFEN = 1;
    ADC->CR_b.INBUFBYPASS = 0;
    
    /* sample time*/
    ADC->SCR_b.SAMP  = 20; // sample time: 19/48M temperary 3

    GPIO->MXR1_b.PM3 =  7 ; //复用为AF7，即ADC
    GPIO->MXR1_b.PM4 =  7 ; //复用为AF7，即ADC

#if (FOC_EPWM_SHUNT_MODE == -1)
    /* config ADC channel */
    ADC->QCR1_b.Q1EN  = 1;
    ADC->QCR1_b.Q1SEL = 11; // current sense amplifier
    ADC->QCR1_b.Q2EN  = 1;
    ADC->QCR1_b.Q2SEL = 12; // current sense amplifier
    ADC->QCR1_b.Q3EN  = 1; //双电阻不使用
    ADC->QCR1_b.Q3SEL = 3; // 
    ADC->QCR1_b.Q4EN  = 1;
    ADC->QCR1_b.Q4SEL = 1; // MVDD
    ADC->QCR1_b.Q5EN  = 1;
    ADC->QCR1_b.Q5SEL = 4; // BEMF Voltage
    ADC->QCR1_b.Q6EN  = 1;
    ADC->QCR1_b.Q6SEL = 10; // external tj MOS管温度 A2
#endif
    

    ADC->QTR1_b.ABS  = 0;    // 0:abs time 1:relative time
    ADC->QTR1_b.QTTR = 1;   // set default abs time
    ADC->QTR2_b.ABS  = 0;   // 0:abs time 1:relative time
    ADC->QTR2_b.QTTR = 45;  // set default abs time
    ADC->QTR3_b.ABS  = 0;   // 0:abs time 1:relative time
    ADC->QTR3_b.QTTR = 90;  // set default relative time
    ADC->QTR4_b.ABS  = 1;   // 0:abs time 1:relative time
    ADC->QTR4_b.QTTR = 48;  // set default relative time
    ADC->QTR5_b.ABS  = 1;   // 0:abs time 1:relative time
    ADC->QTR5_b.QTTR = 48;  // set default relative time
    ADC->QTR6_b.ABS  = 1;   // 0:abs time 1:relative time
    ADC->QTR6_b.QTTR = 48;  // set default relative time
    // config interrupt priority
    NVIC_DisableIRQ(ADC_IRQn);
    NVIC_SetPriority(ADC_IRQn, ADC_IRQ_PRIO);

    ADC->IEN_b.EOCIE = 0; // enable EOC interrupt
    ADC->CR_b.EN     = 1;
}

/************************************************************
 * @brief: Initialize GDU module for FOC control
 * @return <None>
 ************************************************************/
void FOC_GduInit(void)
{
    SYSCTRL->LKKEYR          = SYSCTRL_UNLOCK_KEY; // unlock sysctrl
    SYSCTRL->APBCGR_b.GDUCEN = 1;               // enable adc clock
    GDU->CR_b.BOVASTEN = GDU_AUTO_CLOSE_BOV; // BVDD OV auto close
    GDU->CR_b.BUVASTEN = GDU_AUTO_CLOSE_BOV; // BVDD UV auto close
    GDU->CR_b.ASTMOD = GDU_ASTMODE; // AUTO-CLOSE close all bridge
    GDU->CR_b.OCSHUNTEN = GDU_OCSD_EN; //  OCSHUNT enable
    GDU->CR_b.COVASTEN = GDU_AUTO_CLOSE_CPOV; // CP OV auto close
    GDU->CR_b.CUVASTEN = GDU_AUTO_CLOSE_CPUV; // CP UV auto close
    GDU->CR_b.WAITEN = GDU_CPWAITEN; // EPWM transfer to GDU immediately
    GDU->CR_b.OCHYST = GDU_OCHYST; // OC hyst voltage 10mV
    GDU->CR_b.OCHYSTMOD = GDU_OCHYSTMODE; // Hysteresis mode
    GDU->CR_b.CCPT = GDU_CCPT; // dead time
    
    // VDS detection
    GDU->CR_b.SC0EN = MOS_DESAT_EN; // enable VDS detection
    GDU->CR_b.SC1EN = MOS_DESAT_EN;
    GDU->CR_b.SC2EN = MOS_DESAT_EN;
    GDU->CR_b.SC3EN = MOS_DESAT_EN;
    GDU->CR_b.SC4EN = MOS_DESAT_EN;
    GDU->CR_b.SC5EN = MOS_DESAT_EN;
    GDU->SCTHCR_b.SCTHHS = GDU_DESAT_TH; 
    GDU->SCTHCR_b.SCTHLS = GDU_DESAT_TH; 
    GDU->SCTHCR_b.SCMSK = GDU_DESAT_MSK;
    GDU->SCTHCR_b.SCDEB = GDU_DESAT_SCDEN;
    GDU->SCTHCR_b.OCDEB = GDU_OCDEB;
    
    // enable current sense amp
    GDU->CR_b.CSAEN = 1;
    GDU->CSACR_b.CALIBMOD = 0;
    GDU->CSACR_b.OFFSET = 0;
    GDU->CLDAC_b.DATA = GDU_OCVOL;

    // enable ON/OFF sequence
    GDU->CR_b.ONSEQEN = GDU_ONSEQEN;
    GDU->CR_b.OFFSEQEN = GDU_OFFSEQEN;
    GDU->CR_b.HS_FDISCHGEN = GDU_HSFDISCHG;
    GDU->CR_b.LS_FDISCHGEN = GDU_LSFDISCHG;

    GDU->OFFSEQCR_b.OFFI1 = 0xFF; 
    GDU->OFFSEQCR_b.OFFT1 = 3; 
    GDU->OFFSEQCR_b.OFFI2 = 0xFF; 
    GDU->OFFSEQCR_b.OFFT2 = 2;
    GDU->OFFSEQCR_b.OFFI3 = 0xFF;
    GDU->OFFSEQCR_b.OFFT3 = 2;
    GDU->OFFSEQCR_b.OFFI4 = 0xFF;
    GDU->OFFSEQCR_b.OFFT4 = 5;

    GDU->ONSEQCR_b.ONI1 = 0xFF;
    GDU->ONSEQCR_b.ONT1 = 3;
    GDU->ONSEQCR_b.ONI2 = 0xFF;
    GDU->ONSEQCR_b.ONT2 = 2;
    GDU->ONSEQCR_b.ONI3 = 0xFF;
    GDU->ONSEQCR_b.ONT3 = 2;
    GDU->ONSEQCR_b.ONI4 = 0xFF;
    GDU->ONSEQCR_b.ONT4 = 5;

    // enable interrupt
    GDU->IEN_b.OCREG0HIE = MOS_DESAT_EN;
    GDU->IEN_b.OCREG0LIE = MOS_DESAT_EN;
    GDU->IEN_b.OCREG1HIE = MOS_DESAT_EN;
    GDU->IEN_b.OCREG1LIE = MOS_DESAT_EN;
    GDU->IEN_b.OCREG2HIE = MOS_DESAT_EN;
    GDU->IEN_b.OCREG2LIE = MOS_DESAT_EN;
    GDU->IEN_b.OCSHUNTIE = GDU_OCSD_EN;

    #if (MOS_DESAT_EN || GDU_OCSD_EN)
    NVIC_EnableIRQ(GDU_IRQn);
    NVIC_SetPriority(GDU_IRQn, GDU_IRQ_PRIO);
    #endif

    GDU->CR_b.GDUB0EN = 1; // enable GDU bridge
    GDU->CR_b.GDUB1EN = 1;
    GDU->CR_b.GDUB2EN = 1;
    GDU->CR_b.GDUGLOBLEN = 1; // enable GDU global

}
