#include "stm32f4xx.h"
#include "uart.h"
#include "i2c.h"
#include "system_clock.h"
#include "DS3231.h"
#include <string.h>

void DS3231_Test_Run(I2C_TypeDef *I2Cx, uint8_t use_rtc, uint8_t use_eeprom)
{
    ds3231_data_struct rtc;
    const char msg[] = "Hello World!";
    uint8_t readBuf[32] = {0};

    // --- RTC Section ---
    if (use_rtc)
    {
        UART_Write(USART2, "=== Initializing DS3231 RTC ===\r\n");
        DS3231_Init(I2Cx);

        // Uncomment once to set date/time
        // DS3231_SetDateTime(25, 11, 10, 2, 23, 15, 0); // yyyy=2025, mm=11, dd=10, DOW=2 (Mon)

        UART_Write(USART2, "DS3231 Ready.\r\n\n");
    }

    // --- EEPROM Section ---
    if (use_eeprom)
    {
        UART_Write(USART2, "=== Initializing AT24C32 EEPROM ===\r\n");
        AT24C32_Init(I2Cx);

        UART_Write(USART2, "Writing data to EEPROM...\r\n");
        AT24C32_WriteData(I2Cx, 0x0000, (uint8_t *)msg, strlen(msg));
        UART_Write(USART2, "Data written.\r\n");

        UART_Write(USART2, "Reading back data...\r\n");
        AT24C32_ReadData(I2Cx, 0x0000, readBuf, strlen(msg));
        readBuf[strlen(msg)] = '\0';
        UART_Write(USART2, "Readback: ");
        UART_Write(USART2, (char *)readBuf);
        UART_Write(USART2, "\r\n\n");
    }

    // --- Continuous RTC Display Loop ---
    if (use_rtc)
    {
        UART_Write(USART2, "=== Reading RTC Time ===\r\n");
        while (1)
        {
            if (DS3231_ReadDateTime(I2Cx, &rtc))
                DS3231_PrintDateTime(&rtc);
            else
                UART_Write(USART2, "⚠️  DS3231 read failed\r\n");

            sysTickDelay(1000);
        }
    }
    else
    {
        UART_Write(USART2, "RTC disabled — test complete.\r\n");
    }
}
