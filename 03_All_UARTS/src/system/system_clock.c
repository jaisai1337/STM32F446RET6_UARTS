#include "system_clock.h"

void SystemClock_Config(void)
{
    // 1. Enable HSE (8 MHz external oscillator)
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));  // wait until HSE ready

    // 2. Configure power regulator (needed for >168MHz)
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS; // Scale 1 mode (max frequency)

    // 3. Configure Flash latency and enable prefetch
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;

    // 4. Configure PLL: 
    // Input = 8 MHz, PLLM=8, PLLN=360, PLLP=2 → SYSCLK = 180 MHz
    // PLLQ=7 (for USB 48MHz)
    RCC->PLLCFGR = (8 << RCC_PLLCFGR_PLLM_Pos)   |
                   (360 << RCC_PLLCFGR_PLLN_Pos) |
                   (0 << RCC_PLLCFGR_PLLP_Pos)   | // PLLP = 2
                   (7 << RCC_PLLCFGR_PLLQ_Pos)   |
                   RCC_PLLCFGR_PLLSRC_HSE;

    // 5. Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));  // wait until PLL ready

    // 6. Configure prescalers
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;    // AHB = SYSCLK / 1 = 180MHz
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;   // APB1 = HCLK / 4 = 45MHz
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;   // APB2 = HCLK / 2 = 90MHz

    // 7. Select PLL as system clock
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    // 8. Update SystemCoreClock variable if used
    SystemCoreClockUpdate();
}
