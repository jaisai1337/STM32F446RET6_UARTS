#include "stm32f4xx.h"
#include "DS3231.h"
#include "i2c.h"
#include "uart.h"

#define DS3231_ADDR  0x68

// Convert decimal to BCD
static uint8_t dec_to_bcd(uint8_t val)
{
    return ((val / 10) << 4) | (val % 10);
}

// Convert BCD to decimal
static uint8_t bcd_to_dec(uint8_t val)
{
    return ((val >> 4) * 10) + (val & 0x0F);
}

// ---------------------------------------------------------------------------
// Initialize DS3231 (called after I2C_Master_Init())
// ---------------------------------------------------------------------------
void DS3231_Init(I2C_TypeDef *I2Cx)
{
    UART_Write(USART2, "\r\n=== DS3231 Init ===\r\n\n");
    if (I2C_Master_Address(I2Cx, DS3231_ADDR, I2C_WRITE) == 0) {
        UART_Write(USART2, "DS3231 detected at 0x68\r\n");
        I2C_Master_Stop(I2Cx);
    } else {
        UART_Write(USART2, "DS3231 not responding!\r\n");
    }
}

// ---------------------------------------------------------------------------
// Write time/date to DS3231
// ---------------------------------------------------------------------------
void DS3231_SetDateTime(I2C_TypeDef *I2Cx,uint8_t year, uint8_t month, uint8_t date,
                        uint8_t dow, uint8_t hour, uint8_t minute, uint8_t second)
{
    UART_Write(USART2, "\r\n=== Setting DS3231 Date/Time ===\r\n");

    if (I2C_Master_Address(I2Cx, DS3231_ADDR, I2C_WRITE) != 0) {
        UART_Write(USART2, "Failed to communicate with DS3231\r\n");
        return;
    }

    // Start writing from register 0x00 (Seconds)
    I2C_Master_Write(I2Cx, 0x00);
    I2C_Master_Write(I2Cx, dec_to_bcd(second));  // Seconds
    I2C_Master_Write(I2Cx, dec_to_bcd(minute));  // Minutes
    I2C_Master_Write(I2Cx, dec_to_bcd(hour));    // Hours
    I2C_Master_Write(I2Cx, dec_to_bcd(dow));     // Day of week
    I2C_Master_Write(I2Cx, dec_to_bcd(date));    // Date
    I2C_Master_Write(I2Cx, dec_to_bcd(month));   // Month
    I2C_Master_Write(I2Cx, dec_to_bcd(year));    // Year (00–99)
    I2C_Master_Stop(I2Cx);

    UART_Write(USART2, "DS3231 time set successfully\r\n");
}

// ---------------------------------------------------------------------------
// Read time/date from DS3231
// ---------------------------------------------------------------------------
uint8_t DS3231_ReadDateTime(I2C_TypeDef *I2Cx,ds3231_data_struct *dt)
{
    uint8_t raw[7];

    // ---- Step 1: Set register pointer to 0x00 ----
    if (I2C_Master_Address(I2Cx, DS3231_ADDR, I2C_WRITE) != 0) {
        UART_Write(USART2, "DS3231 read: write phase failed\r\n");
        return 0;
    }

    I2C_Master_Write(I2Cx, 0x00);  // seconds register
    I2C_Master_Stop(I2Cx);

    // ---- Step 2: Start a read transaction ----
    if (I2C_Master_Address(I2Cx, DS3231_ADDR, I2C_READ) != 0) {
        UART_Write(USART2, "DS3231 read: read phase failed\r\n");
        return 0;
    }

    // ---- Step 3: Multi-byte read sequence ----
    I2Cx->CR1 |= I2C_CR1_ACK;    // Enable ACK for all but last byte
    (void)I2Cx->SR2;             // Clear ADDR flag

    for (int i = 0; i < 6; i++)  // read first 6 bytes
    {
        while (!(I2Cx->SR1 & I2C_SR1_RXNE));
        raw[i] = I2Cx->DR;
    }

    // ---- Step 4: Read the 7th byte safely ----
    I2Cx->CR1 &= ~I2C_CR1_ACK;   // NACK for last byte
    I2Cx->CR1 |= I2C_CR1_STOP;   // STOP before reading final byte
    while (!(I2Cx->SR1 & I2C_SR1_RXNE));
    raw[6] = I2Cx->DR;

    // Re-enable ACK for next use
    I2Cx->CR1 |= I2C_CR1_ACK;

    // ---- Step 5: Convert & store ----
    dt->seconds = bcd_to_dec(raw[0] & 0x7F);
    dt->minutes = bcd_to_dec(raw[1]);
    dt->hours   = bcd_to_dec(raw[2] & 0x3F);
    dt->dow     = bcd_to_dec(raw[3]);
    dt->date    = bcd_to_dec(raw[4]);
    dt->month   = bcd_to_dec(raw[5] & 0x1F);
    dt->year    = bcd_to_dec(raw[6]);

    return 1;
    
}


// ---------------------------------------------------------------------------
// Print formatted time/date over UART
// ---------------------------------------------------------------------------
void DS3231_PrintDateTime(const ds3231_data_struct *dt)
{
    const char *dow_str[] = {"", "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

    UART_Write(USART2, "Time: ");
    UART_PutChar(USART2, (dt->hours / 10) + '0');
    UART_PutChar(USART2, (dt->hours % 10) + '0');
    UART_PutChar(USART2, ':');
    UART_PutChar(USART2, (dt->minutes / 10) + '0');
    UART_PutChar(USART2, (dt->minutes % 10) + '0');
    UART_PutChar(USART2, ':');
    UART_PutChar(USART2, (dt->seconds / 10) + '0');
    UART_PutChar(USART2, (dt->seconds % 10) + '0');

    UART_Write(USART2, "  Date: ");
    UART_PutChar(USART2, (dt->date / 10) + '0');
    UART_PutChar(USART2, (dt->date % 10) + '0');
    UART_PutChar(USART2, '/');
    UART_PutChar(USART2, (dt->month / 10) + '0');
    UART_PutChar(USART2, (dt->month % 10) + '0');
    UART_Write(USART2, "/20");
    UART_PutChar(USART2, (dt->year / 10) + '0');
    UART_PutChar(USART2, (dt->year % 10) + '0');

    UART_Write(USART2, "  DOW: ");
    if (dt->dow >= 1 && dt->dow <= 7)
        UART_Write(USART2, dow_str[dt->dow]);
    else
        UART_Write(USART2, "??");

    UART_Write(USART2, "\r\n");
}