#ifndef UART_TEST_H
#define UART_TEST_H

#include "stm32f4xx.h"
#include "uart.h"

// Initialize all UARTs (USART1–6, UART4–5)
void UART_All_Init(void);

// Run interactive UART test via USART2 (USB)
void UART_Test_Run(void);

#endif
