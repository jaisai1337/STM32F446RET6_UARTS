//Blink led
#include "stm32f4xx.h"
#include "systick.h"
void sysTickDelay(int count);
int main(void)
{
    // Enable GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Set PA5 as output
    GPIOA->MODER |= GPIO_MODER_MODE5_0;
    GPIOA->MODER &= ~GPIO_MODER_MODE5_1;

    while (1)
    {
        // Toggle PA5
        GPIOA->ODR ^= GPIO_ODR_OD5;
        sysTickDelay(1000);
    }
}
