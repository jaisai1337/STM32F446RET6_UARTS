#include "systick.h"

void sysTickDelay(int count)
{
    // Configure SysTick
    SysTick->LOAD = (180000 - 1); // Assuming 180 MHz clock, 1 ms tick
    SysTick->VAL = 0;            // Clear current value
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk;

    for (int i = 0; i < count; i++)
    {
        // Wait until the COUNTFLAG is set
        while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);
    }
    // Disable SysTick
    SysTick->CTRL = 0;
}