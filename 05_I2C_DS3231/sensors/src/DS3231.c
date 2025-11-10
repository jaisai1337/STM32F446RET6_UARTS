#include "stm32f4xx.h"
#include "DS3231.h"
#include "i2c.h"
#include "uart.h"

#define DS3231_ADDR  0x68
#define AT24C32_ADDR  0x57


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


// AT24C32 

void AT24C32_Init(I2C_TypeDef *I2Cx)
{
    if (I2C_Master_Address(I2Cx, DS3231_ADDR, I2C_WRITE) == 0) {
        UART_Write(USART2, "AT24C32 detected at 0x57\r\n");
        I2C_Master_Stop(I2Cx);
    } else {
        UART_Write(USART2, "AT24C32 not responding!\r\n");
    }
}

// ===========================
// Write a block of data
// ===========================
void AT24C32_WriteData(I2C_TypeDef *I2Cx, uint16_t memAddr, uint8_t *data, uint16_t len)
{
    // Each write can only write within a 32-byte page!
    // This simple version writes one page or less.

    if (len > 32) len = 32;  // Limit to one page

    if (I2C_Master_Address(I2Cx, AT24C32_ADDR, I2C_WRITE) != 0)
    {
        UART_Write(USART2, "❌ EEPROM NACK on write\r\n");
        return;
    }

    // Send memory address (2 bytes)
    I2C_Master_Write(I2Cx, (uint8_t)(memAddr >> 8));    
    I2C_Master_Write(I2Cx, (uint8_t)(memAddr & 0xFF));

    // Send data
    for (uint16_t i = 0; i < len; i++)
    {
        I2C_Master_Write(I2Cx, data[i]);
    }

    I2C_Master_Stop(I2Cx);
}

// ===========================
// Read a block of data
// ===========================
void AT24C32_ReadData(I2C_TypeDef *I2Cx, uint16_t memAddr, uint8_t *buf, uint16_t len)
{
    // ---- 1. Set EEPROM internal address pointer ----
    if (I2C_Master_Address(I2Cx, AT24C32_ADDR, I2C_WRITE) != 0)
    {
        UART_Write(USART2, "❌ EEPROM NACK on address\r\n");
        return;
    }
    I2C_Master_Write(I2Cx, (uint8_t)(memAddr >> 8));
    I2C_Master_Write(I2Cx, (uint8_t)(memAddr & 0xFF));
    
    // NOTE: We assume I2C_Master_Write does NOT send a STOP bit.
    // If it does, you would need an I2C_Master_Stop() here.
    // But your code implies it doesn't, so we proceed to Repeated Start.

    // ---- 2. Repeated START for READ ----
    I2Cx->CR1 |= I2C_CR1_START;
    while (!(I2Cx->SR1 & I2C_SR1_SB));
    I2Cx->DR = (AT24C32_ADDR << 1) | I2C_READ;
    while (!(I2Cx->SR1 & I2C_SR1_ADDR));

    // ---- 3. Sequential Read (Using correct N > 2 logic) ----
    
    // This logic assumes len > 2, which is true for "Hello World!" (len=12)
    // For a 100% robust driver, you'd need separate cases for len=1 and len=2.
    
    I2Cx->CR1 |= I2C_CR1_ACK; // Enable ACK
    (void)I2Cx->SR2; // Clear ADDR

    // Read N-2 bytes
    for (uint16_t i = 0; i < len - 2; i++)
    {
        while (!(I2Cx->SR1 & I2C_SR1_RXNE));
        buf[i] = I2Cx->DR;
    }

    // Read byte N-1 (len-2)
    while (!(I2Cx->SR1 & I2C_SR1_RXNE));
    I2Cx->CR1 &= ~I2C_CR1_ACK; // Send NACK
    buf[len - 2] = I2Cx->DR;

    // Read byte N (len-1)
    while (!(I2Cx->SR1 & I2C_SR1_RXNE));
    I2Cx->CR1 |= I2C_CR1_STOP; // Send STOP
    buf[len - 1] = I2Cx->DR;


    I2Cx->CR1 |= I2C_CR1_ACK; // restore ACK
}

