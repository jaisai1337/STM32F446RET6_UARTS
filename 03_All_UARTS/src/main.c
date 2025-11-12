//Blink led
#include "stm32f4xx.h"
#include "systick.h"
#include "system_clock.h"
#include "uart.h"

#define printf iprintf

#include <stdio.h>
int main(void)
{
    SystemClock_Config();
    UART_All_Init();
    UART_Test_Run();

    while(1){


    }
}
