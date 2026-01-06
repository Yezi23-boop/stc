/************************************************************
 * @file: uart.c
 * @author: Young Leon
 * @version: V1.0
 * @data: 2022/04/22
 * @brief: uart c file
 * @note:
 * @Copyright (C) 2021 Novosense All rights reserved.
 ************************************************************/

#include "uart.h"

#define __XTAL (48000000UL) /* Oscillator frequency             */

#define UART1_CLEAR_RXC_INT      (1 << 1)    // Clear UART1 RXC inturrupt flag

void UART_DataTransmit(uint8_t u8Char);
uint8_t UART_DataReceive(void);
/************************************************************
 * @brief: init uart1 for stdio.h printf
 * @param <int> baudRate
 * @return <None>
 ************************************************************/
void InitPrintf(int baudRate)
{
    uint16_t brr = 0;
    brr          = (uint16_t)(__XTAL / baudRate);

    /* unlock sysctrl register */
    SYSCTRL->LKKEYR = SYSCTRL_LOCKKEY;

    /* enable apb ahb bus clk */
    SYSCTRL->SCCR_b.AHBCEN = 1;
    SYSCTRL->SCCR_b.APBCEN = 1;

    /* enable uart1 and gpio clk */
    SYSCTRL->APBCGR_b.UART1CEN = 1;
    SYSCTRL->AHBCGR_b.GPIOCEN  = 1;

    /* mux UART1 port */
    GPIO->MXR1_b.PM0 = 2; // UART1 RX
    GPIO->MXR1_b.PM1 = 2; // UART1 TX
    GPIO->PDIEN_b.DIEN0 = 1;
    GPIO->PDOEN_b.DOEN1 = 1;
    LINPORT->CR_b.LAIS = 1; // if LIN_UART

    NVIC_EnableIRQ(UART1_IRQn);
    NVIC_SetPriority(UART1_IRQn, 3);
    /* config uart1 */
    UART1->CR_b.TXFE     = 0;
    UART1->CR_b.RXFE     = 0;   // disable fifo
    UART1->CR_b.STOP     = 0;   // 1 stop bit
    UART1->CR_b.PCE      = 0;   // enable parity
    //UART1->CR_b.PS       = 1;   // odd parity
    UART1->BRR           = brr; // baud rate 115200
    UART1->IEN_b.RXCIE = 0;   // rx 1 byte irq disable
    /* enable UART1 */
    UART1->CR_b.TXRXE = 1;
    UART1->CR_b.UE  = 1;
}

void UART_DataTransmit(uint8_t u8Char)
{
    while (UART1->ISR_b.TBSYIF);  // Wait for sending idle
    UART1->TDATA = u8Char;        // send data
}

uint8_t UART_DataReceive(void)
{
		uint8_t u8Char = 0;
		if(UART1->ISR_b.RXCIF) // receive complete?
		{
			u8Char = UART1->RDATA;
			UART1->ISR = UART1_CLEAR_RXC_INT;
		}
				
		return u8Char;
}
