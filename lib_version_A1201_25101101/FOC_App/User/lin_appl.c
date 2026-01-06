/****************************************************************************
 * @file    : lin_appl.c
 * @author  : Novosns MCU Team
 * @version : V1.0
 * @Date    : 2024/02/01
 * @brief   : ATCP application file for LIN communication
 * @note
 * Copyright (C) 2024 Novosense All rights reserved.
 ****************************************************************************/
#include "include.h"
#include "stdio.h"
#include "nvsns_foc.h"
#include "foc_init.h"
#include "foc_paras.h"
#include "fault_diagnose.h"
#include "fault_init.h"
#include "fault_handler.h"
#include "freemaster.h"
#include "lin_define.h"
#include "lin_api.h"
#include "lin_api_appl.h"
#include "lin_config.h"
#include "lin_nvr.h"
#include "lin_identification_and_configuration.h"
#include "curr_sample_1rps.h"
#include "foc_config.h"
#include "SysTick.h"
#include "config.h"
#include "global.h"


/*******************************************************************************
* prototypes of static functions
*******************************************************************************/
static void ErrorHandler(void);

// static Frac16_t ParsePWMSpeed(uint8_t u8Pwm);
//static void UpdateLinATCPStatus(void);
static void wait_ms(int n);
static void InitWatchDog(void);
static void DebugGPIO_Init(void);

/******************************************************************************
 static variables
******************************************************************************/
static volatile int g_systickDelayCounter = 0; // is decremented in SysTick_Handler

static volatile int systick_delay = 0;

static uint8_t  g_u16DwdgTrigVal = 0xAA;



//初始化GPIO11 12即MOS电源是能否使能叫
void PowerEnableGPIO_Init(void)
{
    SYSCTRL->LKKEYR = SYSCTRL_LOCKKEY; // Unlock SYSCTRL

    SYSCTRL->SCCR_b.AHBCEN = 1;     // Enable AHB clock
    SYSCTRL->AHBCGR_b.GPIOCEN = 1;  // Enable GPIO clock
    
    GPIO->MXR2_b.PM11 = 1;           // GPIO11 set output mode to control MOS电源使能
    GPIO->PDOEN_b.DOEN11 = 1;        // Enable GPIO11 output

    GPIO->MXR2_b.PM12 = 1;           // GPIO12 set output mode to control MOS电源使能
    GPIO->PDOEN_b.DOEN12 = 1;        // Enable GPIO12 output

    GPIO->MXR1_b.PM5 = 0;            //GPIO5 set input mode to 
    GPIO->PDIEN_b.DIEN5 = 1;
    GPIO->PUR_b.PU5 = 1;

    GPIO->MXR1_b.PM6 = 0;
    GPIO->PDIEN_b.DIEN6 = 1;
    GPIO->PUR_b.PU6 = 1;

}

//正转工况下电源使能 即GPIO 11S输出搞
void PowerEnableForward(void)
{
    GPIO->PDO_b.DO11 = 1;
    GPIO->PDO_b.DO12 = 0;
}

//反转工况下电源使能 即GPIO 12S输出搞
void PowerEnableReverse(void)
{
    GPIO->PDO_b.DO11 = 0;
    GPIO->PDO_b.DO12 = 1;
}

extern Frac16_t f16IqReq;
/************************************************************
 * @brief: main function for LIN communication
 * @return <None>
 ************************************************************/
void LinApplMain(void)
{
    __disable_irq();
    /*unlock sysctrl register*/
    SYSCTRL->LKKEYR = SYSCTRL_LOCKKEY;
    SYSCTRL->SYSCFR_b.WWDGEN = 0;       // Disable WWDG
	SYSCTRL->SYSCFR_b.DWDGEN = 0;       // Disable DWDG
    SYSCTRL->LKKEYR = SYSCTRL_UNLOCK_KEY1;
    SYSCTRL->LKKEYR = SYSCTRL_UNLOCK_KEY2;
    FLASHCTRL->RDCR_b.WAIT = 2;
    FLASHCTRL->PFCR_b.PFTPHS_EN = 1;
    /* enable apb ahb clk*/
    SYSCTRL->SCCR_b.APBCEN = 1;
    SYSCTRL->SCCR_b.AHBCEN = 1;
    SYSCTRL->APBCGR_b.GDUCEN = 1;
    SYSCTRL->APBCGR_b.TIM0CEN = 1;
    SYSCTRL->AHBCGR_b.GPIOCEN = 1;
    SYSCTRL->AHBCGR_b.EPWMCEN = 1;
    SYSCTRL->APBCGR_b.LINUARTCEN = 1;     // enable LINUART APB clock
    SYSCTRL->APBCGR_b.LINPCEN = 1;        // enable LINPORT APB clock
    SYSCTRL->ANACR_b.CHGPCDIV = 2;
    SYSCTRL->ANACR_b.REFBUFEN = 1; //enable ref buffer
    SYSCTRL->ANACR_b.REFOUT = 0; // ref out 1.25V
    SYSCTRL->ANACR_b.REFEN = 1; // enable ref
    //抖频 EMC测试用
    // SYSCTRL->ANACR_b.EMREN = 1;
    // SYSCTRL->ANACR_b.MCMD = 0;

    PowerEnableGPIO_Init();
    if ((GPIO->PDI_b.DI6 == 1) && (GPIO->PDI_b.DI5 == 0)) {
        PowerEnableForward();  // 默认正转工况
    } else if (GPIO->PDI_b.DI5 == 1 && (GPIO->PDI_b.DI6 == 0)) {
        PowerEnableReverse();  // 反转工况
    }        


//    InitUart();
//    int8_t i8Ret = 0;
    SYSCTRL->SYSDBGR_b.PWMDE = 0;

    //------------------------------
    // Initialize SYSTICK with 250us
    //------------------------------
    if (SysTick_Init(48)) {       
        ErrorHandler();                 // SYSTICK timer error
    }
    //delay_ms(3000);
    DebugGPIO_Init();
    //FMSTR_Init();
    FAULT_Init();
    FOC_EpwmInit();
    FOC_AdcInit();
    FOC_GduInit();
    
//    if (FLT_PwrCircuitDetect()) {
//        /* Custom Function */
//    }

    tDrvFoc.tAppState.tStatus = RESET;
    tDrvFoc.tAppState.tEvent  = E_RESET;
    tDrvFoc.u16PWMCnt = 0;
    NVIC_EnableIRQ(EPWM_IRQn);
    /* set hold after reset done*/
    tDrvFoc.tAppState.tEvent = E_FAULT;

    
    //----------------------------------------
    // Start LIN System - LIN Init Interface
    //----------------------------------------
#if ((COM_TYPE & 0x2) == COM_TYPE_LIN)
    if (l_sys_init()) {
        ErrorHandler();                 // LIN Initialisation failed
    } else {
        l_ifc_init_MyLinIfc();          // LIN Initialisation successful
    }

    if (l_ifc_connect_MyLinIfc()) {     // LIN connections unsuccessful
        ErrorHandler();
    } else {                            // LIN connections successful
    }

#endif
        /* write SW_VERSION to frame "f_Read_2", byte[1] */
    // g_nodeConfigFrameTable[3].var[1] = SW_VERSION;
    /* enable the interrupts that have been configured before */
    __enable_irq();
    //加载版本信息到缓存区
    l_ver_trans_data_buffer(&g_tATCPVersion, g_nodeConfigFrameTable[NO_VERSION_INFO_FRAME].var);
    //看门狗务必放最后初始化
    InitWatchDog();

}




void FeedDWDG(void)
{
    /* Define the first DWDG trig value */

    /* Feed dog by feed inverse trig value to TRG reg */
    DWDG->TRG = g_u16DwdgTrigVal;
    g_u16DwdgTrigVal = ~g_u16DwdgTrigVal;
}

static void InitWatchDog(void)
{
    SYSCTRL->LKKEYR = SYSCTRL_LOCKKEY;  // Unlock SYSCTRL
    SYSCTRL->SCCR_b.APBCEN = 1;         // Enable APB clock
    SYSCTRL->APBCGR_b.DWDGCEN = 1;      // Enable DWDG clock
    
    DWDG->CKSEL_b.CKSEL = 7;            // DWDG clock = 48M / (2^7) = 375kHz
    DWDG->RR = 37500;                   // DWDG reload value, timeout = 375k / 37.5k = 10Hz = 100ms
    
    SYSCTRL->SYSCFR_b.DWDGEN = 0x55;    // Enable DWDG
	//SYSCTRL->LKKEYR_b.KEYST0 = 1;
}


void DebugGPIO_Init(void)
{
    SYSCTRL->LKKEYR = SYSCTRL_LOCKKEY; // Unlock SYSCTRL

    SYSCTRL->SCCR_b.AHBCEN = 1;     // Enable AHB clock
    SYSCTRL->AHBCGR_b.GPIOCEN = 1;  // Enable GPIO clock
    
    GPIO->MXR1_b.PM0 = 1;           // GPIO6 set output mode to control LED
    GPIO->PDOEN_b.DOEN0 = 1;        // Enable GPIO6 output
}

/************************************************************
 * @brief: Error handler, entered when LIN init was unsuccessful
 * @return <None>
 ************************************************************/
static void ErrorHandler(void)
{
    while(1) {

    }
}




/************************************************************
 * @brief: Update Status Frame
 * @return <None>
 ************************************************************/
void UpdateLinATCPStatus(void)
{
    uint16_t tmpVar = 0;
    /* Measured rotation pump speed */
    tmpVar = tDrvFoc.tPospeControl.f16wRotE1 * (int16_t)tFocParas.N_MAX / 32767;
    g_tATCPState.AS_b.NMeaRotCFM = tmpVar;
    
    /* Measured the chamber of ATCP Controller temperature */
    /* Measured the chamber of ATCP Controller temperature */
    if (tDrvFoc.i16IntMOSTemp < -400) { // 确保温度为正值
        tDrvFoc.i16IntMOSTemp = -400; // 限制最小值
    } else if (tDrvFoc.i16IntMOSTemp > 1500 ) { // 限制最大值
        tDrvFoc.i16IntMOSTemp = 1500;
    }
    tmpVar = tDrvFoc.i16IntMOSTemp + 400;
 
    g_tATCPState.AS_b.TempCFM = tDrvFoc.i16IntMOSTemp + 400; //MOS管温度
    
    
    /* MOFET Current PWM Value */
    g_tATCPState.AS_b.SWVer = EAS_SW_VER; // Software Version
    
    /* The actual supply voltage */

    tmpVar = tDrvFoc.f16DcBusFilt * (int16_t)tFocParas.U_DCB_MAX * 5 / 32767;

    if (tmpVar > 250) {
        tmpVar = 250; // 限制最大值为250 / 5 = 50V
    } else if (tmpVar < 0) {
        tmpVar = 0; // 确保电压为正值
    }
	g_tATCPState.AS_b.DCVolCFM = tmpVar;

    // tDrvFoc.i1610Idcbus = tDrvFoc.f16Idcbus  * (int16_t)tFocParas.I_MAX * 10 / 32767; //母线电流
    /* Measured average pump current */
    if (tDrvFoc.i1610Idcbus < 0)  tDrvFoc.i1610Idcbus = 0; // 确保电流为正值
    // tDrvFoc.i1610Idcbus = (int64_t)tDrvFoc.f16Idcbus  * (int16_t)tFocParas.I_MAX * 10 / 32767; //母线电流
    g_tATCPState.AS_b.DCCurCFM  = tDrvFoc.i1610Idcbus; //母线电流10倍
    
    //把结构体填到缓存区
    l_atcp_struct_trans_data_buffer(&g_tATCPState, g_nodeConfigFrameTable[NO_ATCP_STATUE_FRAME].var);

}

void UpdateLinATCPFault(void)
{
    g_tATCPFault.AS_b.u32FaultMode = tFaultMode.u32FaultMode;
    //将故障信息填充到缓存区
    l_fault_trans_data_buffer(&g_tATCPFault, g_nodeConfigFrameTable[NO_ATCP_FAULT_FRAME].var);
}

/************************************************************
 * @brief: wait for n milliseconds
 * @author: Novosns MCU Team
 * @param <int> n
 * @return <None>
 ************************************************************/
static void wait_ms(int n)
{
    systick_delay = n;
    while (systick_delay) {
    }
}
