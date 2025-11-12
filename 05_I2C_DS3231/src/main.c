#include "stm32f4xx.h"
#include "system_clock.h"
#include "uart.h"
#include "i2c.h"
#include "DS3231_test.h"


#define USE_DS3231     1   // 1 = enable DS3231 RTC test
#define USE_AT24C32    1   // 1 = enable AT24C32 EEPROM test
#define APB1_CLK_HZ  45000000u  // APB1 clock frequency in Hz
#define APB2_CLK_HZ  90000000u  // APB2 clock frequency in Hz


I2C_TypeDef *I2Cx_Master = I2C1;

int main(void)
{
    SystemClock_Config();
    UART_Init(USART2,APB1_CLK_HZ, 115200u);

    UART_Write(USART2, "\r\n=== DS3231 RTC & AT24C32 Combined Test ===\r\n");

    I2C_Master_Init(I2Cx_Master, 45000000u, 100000u);
    

    // Display config info
    UART_Write(USART2, "\r\nConfig: ");
    if (I2Cx_Master == I2C1) UART_Write(USART2, "I2C1");
    else if (I2Cx_Master == I2C2) UART_Write(USART2, "I2C2");
    else if (I2Cx_Master == I2C3) UART_Write(USART2, "I2C3");
    else UART_Write(USART2, "Unknown");
    UART_Write(USART2, " [Base=");
    UART_WriteHex32(USART2, (uint32_t)I2Cx_Master);
    UART_Write(USART2, "] -> Master\r\n\n");

    // Call unified DS3231 + EEPROM test
    DS3231_Test_Run(I2Cx_Master, USE_DS3231, USE_AT24C32);

    while (1);
}
