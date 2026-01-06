#include "nsuc1602.h"
#include <string.h>
#include "SysTick.h"

volatile uint32_t g_systick_overflows;  // 溢出计数器
volatile uint16_t g_tickCnt;
uint32_t g_epochSecond = 0;
uint16_t u16ScaleCnt;


/************************************************************
 * @brief: Systick interrupt handler
 * @return <None>
 ************************************************************/
void SysTickHandler(void)
{
    // GPIO->PDO_b.DO0 = 1;
    // u16ScaleCnt++;
    // if (u16ScaleCnt < 2)
    //     return;
    // u16ScaleCnt = 0;
    g_systick_overflows++;
    if (g_systick_overflows >= 0xFFFFFFFF)
    {
        g_systick_overflows = 0;
    }
    g_tickCnt++;
    // GPIO->PDO_b.DO0 = 0;
}