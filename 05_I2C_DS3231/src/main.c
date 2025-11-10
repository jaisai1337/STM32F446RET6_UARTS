#include "stm32f4xx.h"
#include "system_clock.h"
#include "uart.h"
#include "i2c.h"
#include "DS3231.h"


I2C_TypeDef *I2Cx_Master = I2C1;


int main(void)
{
    ds3231_data_struct rtc;

    SystemClock_Config();
    UART2_Init(45000000u, 115200u);
    UART_Write(USART2, "\r\n=== DS3231 RTC Test ===\r\n");


    I2C_Master_Init(I2Cx_Master, 45000000u, 100000u);

    UART_Write(USART2, "\r\nConfig: ");
    if (I2Cx_Master == I2C1) UART_Write(USART2, "I2C1");
    else if (I2Cx_Master == I2C2) UART_Write(USART2, "I2C2");
    else if (I2Cx_Master == I2C3) UART_Write(USART2, "I2C3");
    else UART_Write(USART2, "Unknown");
    UART_Write(USART2, " [Base=");
    UART_WriteHex32(USART2, (uint32_t)I2Cx_Master);
    UART_Write(USART2, "] -> Master\r\n");

    DS3231_Init(I2Cx_Master);

    // Optional: Uncomment once to set new time/date
    // DS3231_SetDateTime(25, 11, 10, 2, 23, 14, 10);   // yyyy=2025, mm=11, dd=10, DOW=2 (Mon), hh=23, mm=14, ss=10

    while (1)
    {
        if (DS3231_ReadDateTime(I2Cx_Master,&rtc)) {
            DS3231_PrintDateTime(&rtc);
        } else {
            // Optional: log once if needed
            // UART_Write(USART2, "❌ DS3231 read failed\r\n");
        }
        sysTickDelay(1000);
    }
}