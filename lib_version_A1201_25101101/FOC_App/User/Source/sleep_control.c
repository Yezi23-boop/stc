#include "Module_Define.h"
#include "global.h"
#include "sleep_control.h"



#define MON_SLEEP 0x00 //休眠状态标志

//休眠确认计数器
uint16_t g_u16SleepConfirmCnt = 0; //休眠确认计数

#define SYSCTRL_UNLOCK_KEY1      0x8a3d
#define SYSCTRL_UNLOCK_KEY2      0x19ec
#define AMPU_BASE                (0x40024000)

extern void DisableOutput(void);


void Init_HVIO(void)
{
    SYSCTRL->LKKEYR = SYSCTRL_LOCKKEY; // Unlock SYSCTRL

    SYSCTRL->SCCR_b.AHBCEN = 1;     // Enable AHB clock
    GPIO->MXR2_b.PM13 = 0;           // GPIO9 set input mode to read key state
    GPIO->IPND_b.PD13 = 1;            // GPIO9 pull up
    GPIO->PDIEN_b.DIEN13 = 1;        // Enable GPIO9 input
}


void SLEEP_EnableHVIOWake(void)
{
    /* Unlock APMU Register */
    SYSCTRL->LKKEYR = SYSCTRL_UNLOCK_KEY1;
    SYSCTRL->LKKEYR = SYSCTRL_UNLOCK_KEY2;
    /* Enable HVIO wake sleep  */
    *(__IO uint32_t *)(AMPU_BASE + 0x118) |= 1 << 1;
    /* Lock APMU Register */
    SYSCTRL->LKKEYR = 1 << 17;
}

void SLEEP_DisableHVIOWake(void)
{
    /* Unlock APMU Register */
    SYSCTRL->LKKEYR = SYSCTRL_UNLOCK_KEY1;
    SYSCTRL->LKKEYR = SYSCTRL_UNLOCK_KEY2;
    /* Enable HVIO wake sleep  */
    *(__IO uint32_t *)(AMPU_BASE + 0x118) &= ~(1 << 1);
    /* Lock APMU Register */
    SYSCTRL->LKKEYR = 1 << 17;
}

void SLEEP_EnableLINWake(void)
{
    /* Unlock APMU Register */
    SYSCTRL->LKKEYR = SYSCTRL_UNLOCK_KEY1;
    SYSCTRL->LKKEYR = SYSCTRL_UNLOCK_KEY2;
    /* Enable HVIO wake sleep  */
    *(__IO uint32_t *)(AMPU_BASE + 0x118) &= ~(1 << 0);
    /* Lock APMU Register */
    SYSCTRL->LKKEYR = 1 << 17;
}

void SLEEP_DisableLINWake(void)
{
    /* Unlock APMU Register */
    SYSCTRL->LKKEYR = SYSCTRL_UNLOCK_KEY1;
    SYSCTRL->LKKEYR = SYSCTRL_UNLOCK_KEY2;
    /* Enable HVIO wake sleep  */
    *(__IO uint32_t *)(AMPU_BASE + 0x118) |= 1 << 0;
    /* Lock APMU Register */
    SYSCTRL->LKKEYR = 1 << 17;
}

//1ms任务检查休眠状态MON_Type.CNF.STS
void CheckSleepStatus(void)
{
#if (KL15_WAKEUP_ENABLE == 1)
    if (GPIO->PDI_b.DI13 == MON_SLEEP) {
        // 休眠状态，关闭电机输出
        g_u16SleepConfirmCnt++;
        if (g_u16SleepConfirmCnt >= SLEEP_CONFIRM_CNT) {
            g_u16SleepConfirmCnt = 0; // 重置休眠确认计数器
            // tDrvFoc.tAppState.tEvent = E_APP_OFF; // 进入休眠状态
            DisableOutput(); // 禁止输出
            __disable_irq();
            GotoSleep();
        }
    }
#endif
}


