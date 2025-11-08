//Blink led
#include "stm32f4xx.h"
#include "systick.h"
#include "system_clock.h"
#include "gptUart.h"

#define printf iprintf

#include <stdio.h>
int main(void)
{
    // Configure system clock to 180 MHz
    SystemClock_Config();

    // Enable GPIOA clock (for LED)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    // PA5 as output
    GPIOA->MODER |= GPIO_MODER_MODE5_0;
    GPIOA->MODER &= ~GPIO_MODER_MODE5_1;

    // Initialize UART2 TX (PCLK1 = 45 MHz by default on NUCLEO)
    UART2_Init(45000000u, 115200u);

    // Print float value example
    float val = 232.99192f;
    int int_part = (int)val;
    int frac_part = (int)((val - int_part) * 1000);
    printf("Size: %d.%03d\r\n", int_part, frac_part);


    UART2_Write("Enter an integer: ");
    int32_t num = UART2_ReadInteger();
    UART2_FlushRx(); // Clear any remaining data in RX buffer
    printf("You entered: %ld\r\n", num);

    UART2_Write("UART2 TX/RX Echo Test Ready\r\n");

    char name[32];
    UART2_Write("Enter your name : ");
    UART2_ReadString(name, sizeof(name));
    printf("Hello, %s!\r\n", name);
    printf("Type '1' to toggle the LED\r\n");

    while (1)
    {
        char c = UART2_GetChar();
        static int count = 0;
        // If data is received, echo it back
        if (c == '1'){
            GPIOA->ODR ^= GPIO_ODR_OD5;  // Toggle LED
            printf("LED Toggled and count: %d\r\n", count++);
            
            sysTickDelay(1000);
        }
    }
}
