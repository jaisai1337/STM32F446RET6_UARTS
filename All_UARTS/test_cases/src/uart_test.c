#include "uart_test.h"
#include "system_clock.h"
#include "systick.h"
#include <stdio.h>

#define LED_PIN 5


//-----------Quick Test Procedure-----------------
// UART	    TX Pin	    RX Pin	    Loopback Test
// USART1	PA9	        PA10	    Short PA9-PA10
// USART3	PB10	    PC5	        Short PB10-PC5
// UART4	PA0	        PA1	        Short PA0-PA1
// UART5	PC12	    PD2	        Short PC12-PD2
// USART6	PC6	        PC7	        Short PC6-PC7

// ---------- Initialize all UARTs ----------
void UART_All_Init(void)
{
    // USART2: USB virtual COM (APB1 = 45 MHz)
    UART2_Init(45000000u, 115200u);
    UART_Write(USART2, "\r\n[BOOT] USART2: USB virtual COM ready\r\n");

    // USART1: PA9-TX, PA10-RX (APB2 = 90 MHz)
    UART1_Init(90000000u, 115200u);
    UART_Write(USART2, "[BOOT] USART1: Ready (PA9-TX, PA10-RX)\r\n");

    // USART3: PB10-TX, PC5-RX (APB1 = 45 MHz)
    UART3_Init(45000000u, 115200u);
    UART_Write(USART2, "[BOOT] USART3: Ready (PB10-TX, PC5-RX)\r\n");

    // UART4: PA0-TX, PA1-RX (APB1 = 45 MHz)
    UART4_Init(45000000u, 115200u);
    UART_Write(USART2, "[BOOT] UART4: Ready (PA0-TX, PA1-RX)\r\n");

    // UART5: PC12-TX, PD2-RX (APB1 = 45 MHz)
    UART5_Init(45000000u, 115200u);
    UART_Write(USART2, "[BOOT] UART5: Ready (PC12-TX, PD2-RX)\r\n");

    // USART6: PC6-TX, PC7-RX (APB2 = 90 MHz)
    UART6_Init(90000000u, 115200u);
    UART_Write(USART2, "[BOOT] USART6: Ready (PC6-TX, PC7-RX)\r\n");

    UART_Write(USART2, "\r\n[INFO] UART test ready. Type 1–6 to ping UARTs.\r\n");
}

// ---------- UART Test Loop ----------
void UART_Test_Run(void)
{
    // LED for activity
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~(3u << (LED_PIN * 2));
    GPIOA->MODER |=  (1u << (LED_PIN * 2));

    while (1)
    {
        // Handle commands from PC via USART2
        if (UART_Available(USART2))
        {
            char c = UART_GetChar(USART2);
            GPIOA->ODR ^= (1u << LED_PIN); // blink activity

            switch (c)
            {
                case '1':
                    UART_Write(USART1, "Ping from USART2 -> USART1\r\n");
                    UART_Write(USART2, "[TX] Sent to USART1\r\n");
                    break;
                case '3':
                    UART_Write(USART3, "Ping from USART2 -> USART3\r\n");
                    UART_Write(USART2, "[TX] Sent to USART3\r\n");
                    break;
                case '4':
                    UART_Write(UART4, "Ping from USART2 -> UART4\r\n");
                    UART_Write(USART2, "[TX] Sent to UART4\r\n");
                    break;
                case '5':
                    UART_Write(UART5, "Ping from USART2 -> UART5\r\n");
                    UART_Write(USART2, "[TX] Sent to UART5\r\n");
                    break;
                case '6':
                    UART_Write(USART6, "Ping from USART2 -> USART6\r\n");
                    UART_Write(USART2, "[TX] Sent to USART6\r\n");
                    break;
                default:
                    UART_Write(USART2, "[HELP] Press 1–6 to test UARTs\r\n");
                    break;
            }
        }

        // Echo back data from external UART5 to PC (USB)
        // if (UART_Available(UART5))
        // {
        //     char c = UART_GetChar(UART5);
        //     UART_PutChar(USART2, c);
        //     //GPIOA->ODR ^= (1u << 5); // Toggle LED on PA5
        // }

        // Handle echoes/responses from all UARTs
        if (UART_Available(USART1))
            UART_PutChar(USART2, UART_GetChar(USART1));
        if (UART_Available(USART3))
            UART_PutChar(USART2, UART_GetChar(USART3));
        if (UART_Available(UART4))
            UART_PutChar(USART2, UART_GetChar(UART4));
        if (UART_Available(UART5))
            UART_PutChar(USART2, UART_GetChar(UART5));
        if (UART_Available(USART6))
            UART_PutChar(USART2, UART_GetChar(USART6));
        

    }
}

