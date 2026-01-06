/****************************************************************************
 * @file    : systick.c
 * @author  : CJL
 * @version : V1.1
 * @Date    : 2025/2/24
 * @brief   :
 * @note
 * Copyright (C) 2025 Aiwin All rights reserved.
 ****************************************************************************/

// -------------------------------------------------------------------------//
// Includes
// -------------------------------------------------------------------------//
#include "nsuc1602.h"
#include <string.h>
#include "SysTick.h"
#include "SysTick_isr.h"



static uint32_t g_fac_us = 0;       /* us延时倍乘数 */
#define delay_ostickspersec 2000    /* OS时钟节拍,即每秒调度次数 定为2000，即500us定时一次 */

void Set_SysTick_CTRL(uint32_t ctrl)
{
	SysTick->CTRL = ctrl;
}

void Set_SysTick_LOAD(uint32_t load)
{
	SysTick->LOAD = load;
}

uint32_t Read_SysTick_VALUE(void)
{
	return(SysTick->VAL);
}


void Set_SysTick_VALUE(uint32_t value)
{
	SysTick->VAL = value;
}


/**
 * @brief     初始化SysTick
 * @param     sysclk: 系统时钟频率, 即CPU频率(HCLK), 48Mhz输入为48
 * @retval    无
 */
uint32_t SysTick_Init(uint16_t sysclk)
{
    uint32_t reload;
    SysTick->CTRL |= (1 << 2);              /* SYSTICK使用内部时钟源，如果未分频，频率就是主频 */ //NOTE:不知道为什么使能了看门狗，必须要用外部时钟源，用内部时钟源的时候会导致时间异常，变得很快，与芯片时钟的内核部分时钟设置有关，手册貌似未看到
    //这里配置成1,delay_ms的动作正常，但是中断频率异常，配置成0，delay_ms动作异常（时间长会延迟时间很长异常），但是中断频率正常,
    //怀疑是本来就要使用1，内部时钟比较好，然后使用1时，delay_ostickspersec要比较小，从4000改成2000后，就正常了，这可能还是延迟函数的问题
    g_fac_us = sysclk;                      /* 不论是否使用OS,g_fac_us都需要使用 */
    SysTick->CTRL |= 1 << 0;                /* 使能Systick */
    SysTick->LOAD = 0X0FFFFFFF;             /* 注意systick计数器24位，所以这里设置最大重装载值 */                     /* 如果需要支持OS. */
    reload = sysclk;                        /* 每秒钟的计数次数 单位为M */
    reload *= 1000000 / delay_ostickspersec;/* 根据delay_ostickspersec设定溢出时间
                                             * reload为24位寄存器,最大值:16777216,在72M下,约合0.233s左右，在48M下,约合0.35s左右
                                             */
    SysTick->CTRL |= 1 << 1;                /* 开启SYSTICK中断 */
    SysTick->LOAD = reload;                 /* 每1/delay_ostickspersec秒中断一次 */
    // 把当前值设置为reload值
    SysTick->VAL = reload;                  //NOTE:必须，否则复位后这里的值异常会导致一开始的任务轮询异常导致看门狗喂狗失败，导致复位
    NVIC_SetPriority(SysTick_IRQn, STSTICK_PRIO);  /* 设置中断优先级 */
    return 0;
}
/**
 * @brief     开始计时，同时清除计数溢出标志
 * @retval    返回开始计时时的计数值
 */
uint32_t Timer_Start(void)
{
    // 清除COUNTFLAG标志位（通过读取CTRL寄存器来清除）
    SysTick->CTRL & 0x10000;
    // 重置溢出计数器
    g_systick_overflows = 0;
    // 返回当前计数值作为起始值
    return SysTick->VAL;
}



/**
 * @brief     停止计时并计算经过的时间
 * @param     duration_t: 指向用于存储运行时间的变量的指针(单位:us)
 * @param     start_t: 开始时的计数值
 * @retval    1: 成功计算出运行时间
 */
uint8_t Timer_Stop(uint32_t *duration_t, uint32_t start_t)
{
    uint32_t tnow = SysTick->VAL;
    uint32_t reload = SysTick->LOAD;
    uint64_t total_ticks;  // 使用64位避免大数溢出
    
    // 计算总的时钟周期数 = 溢出次数 * LOAD值 + 开始到结束的差值
    // total_ticks = ((uint64_t)g_systick_overflows * reload) + (start_t - tnow);
    if (start_t > tnow)
    {
        total_ticks = ((uint64_t)g_systick_overflows * reload) + (start_t - tnow);
    }
    else
    {
        total_ticks = ((uint64_t)g_systick_overflows * reload) + (reload - tnow + start_t);
    }
    
    // 转换为微秒
    *duration_t = (uint32_t)(total_ticks / g_fac_us);
    
    return 1;
}


//快速获取时间，不使用64位计算，只能用于运行时间小于一个reload的情况
//
uint32_t Timer_GetFASTTime(uint32_t start_t)
{
    uint32_t tnow = SysTick->VAL;
    uint32_t elapsed_us;
    
    // 计算当前周期内的微秒数
    if (start_t > tnow) {
        elapsed_us = (start_t - tnow) ;
    } else {
        elapsed_us = (SysTick->LOAD + start_t - tnow);
    }
    
    
    return (elapsed_us  / g_fac_us); // 转换为微秒
}





/**
 * @brief     延时nus
 * @note      用时钟摘取法来做us延时
 * @param     nus: 要延时的us数
 * @note      nus取值范围: 0 ~ (2^32 / fac_us) (fac_us一般等于系统主频, 自行套入计算，
 * 48M为例延迟时间 = (2^32 - 1) / 48 ≈ 89,478,485 us ≈ 89.478 秒
 * 使用该函数注意,如果某一个中断执行时间大于SysTick->LOAD的值,那么延迟时间将会出错,大于实际的延迟时间,因为无法知道SysTick->VAL已经是经过了多少次溢出
 * 这样做的好处是SysTickHandler中断能够按照设定的周期执行,不会因为延时函数而影响
 * @retval    无
 */
void delay_us(uint32_t nus)
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;
    uint32_t reload = SysTick->LOAD;        /* LOAD的值 */
    ticks = nus * g_fac_us;                 /* 需要的节拍数 */
    
    told = SysTick->VAL;                    /* 刚进入时的计数器值 */
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;        /* 这里注意一下SYSTICK是一个递减的计数器就可以了 */
            }
            else
            {
                tcnt += ((uint64_t)(g_systick_overflows * reload))  - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks) 
            {
                break;                      /* 时间超过/等于要延迟的时间,则退出 */
            }
        }
        g_systick_overflows = 0;                /* 清除溢出标志 */
    }
}

/**
 * @brief     延时nms
 * @note      延时时间为nms
 * @param     nms: 要延时的ms数
 * @note      nms取值范围: 理论上可以无限延时, 但实际上受限于uint16_t的最大值
 * @retval    无
 */ 
void delay_ms(uint16_t nms)
{

    while (nms--)
    {
        delay_us(1000);                     /* 每次延时1毫秒 */
    }
}