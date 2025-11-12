#include "ADXL345.h"
#include "i2c.h"
#include "uart.h" // For logging
#include <stdio.h>

// === PRIVATE HELPER FUNCTIONS ===

/*
 * @brief Writes a single byte to an ADXL345 register.
 */
static void adxl_write_reg(I2C_TypeDef *I2Cx, uint8_t reg, uint8_t data)
{
    // Assumes I2C_Master_Write waits for BTF
    I2C_Master_Address(I2Cx, ADXL345_ADDR, I2C_WRITE);
    I2C_Master_Write(I2Cx, reg);
    I2C_Master_Write(I2Cx, data);
    I2C_Master_Stop(I2Cx);
}

/*
 * @brief Reads one or more bytes from the ADXL345.
 * This is the robust, multi-byte read logic based on the STM32
 * reference manual, handling N=1, N=2, and N>2 cases.
 */
static void adxl_read_multi(I2C_TypeDef *I2Cx, uint8_t start_reg, uint8_t *buf, uint16_t len)
{
    // 1. Set register pointer (START + ADDR(W) + WRITE(reg))
    I2C_Master_Address(I2Cx, ADXL345_ADDR, I2C_WRITE);
    I2C_Master_Write(I2Cx, start_reg);
    // No STOP here, we need a repeated start

    // 2. Repeated START + ADDR(R)
    I2Cx->CR1 |= I2C_CR1_START;
    while (!(I2Cx->SR1 & I2C_SR1_SB));
    I2Cx->DR = (ADXL345_ADDR << 1) | I2C_READ;
    while (!(I2Cx->SR1 & I2C_SR1_ADDR));

    // 3. Handle the N-byte read sequence
    if (len == 1)
    {
        // N=1 Case (for adxl_read_reg / DEVID)
        I2Cx->CR1 &= ~I2C_CR1_ACK; // Send NACK
        (void)I2Cx->SR2; // Clear ADDR
        I2C_Master_Stop(I2Cx); // Send STOP
        
        while (!(I2Cx->SR1 & I2C_SR1_RXNE));
        buf[0] = I2Cx->DR;
    }
    else if (len == 2)
    {
        // N=2 Case (not used here, but good practice)
        I2Cx->CR1 |= I2C_CR1_POS; // Set POS
        (void)I2Cx->SR2; // Clear ADDR
        I2Cx->CR1 &= ~I2C_CR1_ACK; // Send NACK
        
        while (!(I2Cx->SR1 & I2C_SR1_BTF)); // Wait for 2 bytes
        
        I2C_Master_Stop(I2Cx); // Send STOP
        buf[0] = I2Cx->DR;
        buf[1] = I2Cx->DR;
        
        I2Cx->CR1 &= ~I2C_CR1_POS; // Clear POS
    }
    else // len > 2 (N=6 for accel)
    {
        // N > 2 Case (This is the logic from your EEPROM/DS3231 fix)
        I2Cx->CR1 |= I2C_CR1_ACK; // Enable ACK
        (void)I2Cx->SR2; // Clear ADDR

        // Read N-2 bytes
        for (uint16_t i = 0; i < len - 2; i++) {
            while (!(I2Cx->SR1 & I2C_SR1_RXNE));
            buf[i] = I2Cx->DR;
        }

        // Read byte N-1
        while (!(I2Cx->SR1 & I2C_SR1_RXNE));
        I2Cx->CR1 &= ~I2C_CR1_ACK; // Send NACK
        buf[len - 2] = I2Cx->DR;

        // Read byte N
        while (!(I2Cx->SR1 & I2C_SR1_RXNE));
        I2C_Master_Stop(I2Cx);
        buf[len - 1] = I2Cx->DR;
    }

    I2Cx->CR1 |= I2C_CR1_ACK; // restore ACK
}

/*
 * @brief Reads a single register from the ADXL345.
 */
static uint8_t adxl_read_reg(I2C_TypeDef *I2Cx, uint8_t reg)
{
    uint8_t data;
    adxl_read_multi(I2Cx, reg, &data, 1);
    return data;
}


// === PUBLIC FUNCTIONS ===

uint8_t ADXL345_Test_Connection(I2C_TypeDef *I2Cx)
{
    UART_Write(USART2, "Testing ADXL345 Connection...\r\n");
    uint8_t dev_id = adxl_read_reg(I2Cx, ADXL345_REG_DEVID);

    if (dev_id == 0xE5) {
        UART_Write(USART2, "ADXL345 Connection success! Device ID=0xE5\r\n");
        return 0;
    } else {
        UART_Write(USART2, "ADXL345 Connection failed! Device ID=0x");
        UART_WriteHexByte(USART2, dev_id);
        UART_Write(USART2, "\r\n");
        return 1;
    }
}

void ADXL345_Init(I2C_TypeDef *I2Cx)
{
    UART_Write(USART2, "Initializing ADXL345...\r\n");

    // Set Data Format: 16g range, full resolution
    // 0x0B = 0b00001011
    // Bit 3 (FULL_RES) = 1 (Maintains 4mg/LSB resolution)
    // Bits 1:0 (Range) = 11 (±16g)
    adxl_write_reg(I2Cx, ADXL345_REG_DATA_FORMAT, 0x0B);

    // Wake up: Set Measure bit
    // 0x08 = 0b00001000
    // Bit 3 (Measure) = 1
    adxl_write_reg(I2Cx, ADXL345_REG_POWER_CTL, 0x08);

    UART_Write(USART2, "ADXL345 Init complete.\r\n");
}

void ADXL345_Read_Accel(I2C_TypeDef *I2Cx, int16_t *accel_data)
{
    uint8_t data_buf[6];

    // Read all 6 data bytes (X0, X1, Y0, Y1, Z0, Z1)
    // using the N=6 read sequence.
    adxl_read_multi(I2Cx, ADXL345_REG_DATAX0, data_buf, 6);

    // Combine bytes into 16-bit values
    // (Data is little-endian)
    accel_data[0] = (int16_t)(data_buf[1] << 8 | data_buf[0]); // X
    accel_data[1] = (int16_t)(data_buf[3] << 8 | data_buf[2]); // Y
    accel_data[2] = (int16_t)(data_buf[5] << 8 | data_buf[4]); // Z
}