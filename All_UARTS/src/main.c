//Blink led
#include "stm32f4xx.h"
#include "systick.h"
#include "system_clock.h"
#include "uart.h"

#define printf iprintf

#include <stdio.h>
int main(void)
{
    SystemClock_Config(); // Configure system clock to 180 MHz

    // Enable GPIOA clock (for LED)
    RCC->AHB1ENR |= (0x1UL << 0); //RCC_AHB1ENR_GPIOAEN;
    // PA5 as output
    GPIOA->MODER |= (0x1UL << (5 * 2)); // GPIO_MODER_MODE5_0;
    GPIOA->MODER &= ~(0x1UL << (5 * 2 + 1)); // GPIO_MODER_MODE5_1;

    //USART2 PA2-TX, PA3-RX
    UART2_Init(45000000u, 115200u); // APB1 = 45 MHz  // Initialize UART2 TX (PCLK1 = 45 MHz)
    UART_Write(USART2,  "USART2: USB virtual COM ready\r\n");
    //USART1 PA9-TX, PA10-RX
    UART1_Init(90000000u, 115200u); // APB2 = 90 MHz // Initialize UART1 TX (PCLK2 = 90 MHz)
    UART_Write(USART1, "USART1: External device ready\r\n");
    //USART3 PB10-TX, PC5-RX
    UART3_Init(45000000u, 115200u); // APB1 = 45 MHz // Initialize UART3 TX (PCLK1 = 45 MHz)
    UART_Write(USART3, "USART3: External device ready\r\n");
    //USART4 PA0-TX, PA1-RX
    UART4_Init(45000000u, 115200u); // APB1 = 45 MHz // Initialize UART4 TX (PCLK1 = 45 MHz)
    UART_Write(UART4,  "USART4: External device ready\r\n");
    //USART5 PC12-TX, PD2-RX
    UART5_Init(45000000u, 115200u); // APB1 = 45 MHz // Initialize UART5 TX (PCLK1 = 45 MHz)
    UART_Write(UART5,  "USART5: External device ready\r\n");
    //USART6 PC6-TX, PC7-RX
    UART6_Init(90000000u, 115200u); // APB2 = 90 MHz // Initialize USART6 TX (PCLK2 = 90 MHz)
    UART_Write(USART6, "USART6: External device ready\r\n");

    while (1)
    {
        // Echo data from USB (USART2) to external device (USART1)
        if (UART_Available(USART2))
        {
            char c = UART_GetChar(USART2);
            UART_PutChar(USART6, c);
            GPIOA->ODR ^= (1u << 5); // Toggle LED on PA5

        }
        // Echo back data from external UART1 to PC (USB)
        if (UART_Available(USART1))
        {
            char c = UART_GetChar(USART1);
            UART_PutChar(USART2, c);
            GPIOA->ODR ^= (1u << 5); // Toggle LED on PA5
        }
        // Echo back data from external UART3 to PC (USB)
        if (UART_Available(USART3))
        {
            char c = UART_GetChar(USART3);
            UART_PutChar(USART2, c);
            GPIOA->ODR ^= (1u << 5); // Toggle LED on PA5
        }
        // Echo back data from external UART4 to PC (USB)
        if (UART_Available(UART4))
        {
            char c = UART_GetChar(UART4);
            UART_PutChar(USART2, c);
            GPIOA->ODR ^= (1u << 5); // Toggle LED on PA5
        }
        // Echo back data from external UART5 to PC (USB)
        if (UART_Available(UART5))
        {
            char c = UART_GetChar(UART5);
            UART_PutChar(USART2, c);
            GPIOA->ODR ^= (1u << 5); // Toggle LED on PA5
        }
        // Echo back data from external USART6 to PC (USB)
        if (UART_Available(USART6))
        {
            char c = UART_GetChar(USART6);
            UART_PutChar(USART2, c);
            GPIOA->ODR ^= (1u << 5); // Toggle LED on PA5
        }
    }
}
