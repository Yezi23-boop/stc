/****************************************************************************
 * @file    : sysport.c
 * @author  : novosense
 * @version : V1.0
 * @Date    : 2022/3/7
 * @brief   : 
 * @note
 * Copyright (C) 2022 novosense All rights reserved.
 ****************************************************************************/
 
#include "include.h"

// void delay_ms(uint32_t n)
// {
//     for (uint32_t i = 0; i < n; i++) {
//         delay_us(1000);
//     }
// }

// void delay_us(uint32_t n)
// {
//     /* set reload register, set period = 1us */
//     SysTick->LOAD = (uint32_t)(48 - 1UL);
//     SysTick->VAL  = 0UL; /* Load the SysTick Counter Value */
//     SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
//     for (uint32_t i = 0; i < n; i++) {
//        while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
//     }
//     SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
// }


// /**
//  * @brief     延时nus
//  * @note      用时钟摘取法来做us延时
//  * @param     nus: 要延时的us数
//  * @note      nus取值范围: 0 ~ (2^32 / fac_us) (fac_us一般等于系统主频, 自行套入计算，
//  * 48M为例延迟时间 = (2^32 - 1) / 48 ≈ 89,478,485 us ≈ 89.478 秒
//  * 使用该函数注意,如果某一个中断执行时间大于SysTick->LOAD的值,那么延迟时间将会出错,大于实际的延迟时间,因为无法知道SysTick->VAL已经是经过了多少次溢出
//  * 这样做的好处是SysTickHandler中断能够按照设定的周期执行,不会因为延时函数而影响
//  * @retval    无
//  */
// void delay_us(uint32_t nus)
// {
//     uint32_t ticks;
//     uint32_t told, tnow, tcnt = 0;
//     uint32_t reload = SysTick->LOAD;        /* LOAD的值 */
//     ticks = nus * g_fac_us;                 /* 需要的节拍数 */
    
//     told = SysTick->VAL;                    /* 刚进入时的计数器值 */
//     while (1)
//     {
//         tnow = SysTick->VAL;
//         if (tnow != told)
//         {
//             if (tnow < told)
//             {
//                 tcnt += told - tnow;        /* 这里注意一下SYSTICK是一个递减的计数器就可以了 */
//             }
//             else
//             {
//                 tcnt += reload - tnow + told;
//             }
//             told = tnow;
//             if (tcnt >= ticks) 
//             {
//                 break;                      /* 时间超过/等于要延迟的时间,则退出 */
//             }
//         }
//     }
// }

// /**
//  * @brief     延时nms
//  * @note      延时时间为nms
//  * @param     nms: 要延时的ms数
//  * @note      nms取值范围: 理论上可以无限延时, 但实际上受限于uint16_t的最大值
//  * @retval    无
//  */ 
// void delay_ms(uint16_t nms)
// {

//     while (nms--)
//     {
//         delay_us(1000);                     /* 每次延时1毫秒 */
//     }
// }

