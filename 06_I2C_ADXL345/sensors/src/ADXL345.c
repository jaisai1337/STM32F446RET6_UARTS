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
    // Uses the generic I2C write functions
    I2C_Master_Address(I2Cx, ADXL345_ADDR, I2C_WRITE);
    I2C_Master_Write(I2Cx, reg);
    I2C_Master_Write(I2Cx, data);
    I2C_Master_Stop(I2Cx);
}

/*
 * @brief Reads one or more bytes from the ADXL345.
 *
 * REFACTORED: This function is now a simple wrapper.
 * All the complex I2C logic (N=1, N=2, N>2) has been
 * moved to the generic I2C_Master_ReadMulti() function.
 */
static void adxl_read_multi(I2C_TypeDef *I2Cx, uint8_t start_reg, uint8_t *buf, uint16_t len)
{
    // Call the generic I2C driver function
    I2C_Master_ReadMulti(I2Cx, ADXL345_ADDR, start_reg, buf, len);
}

/*
 * @brief Reads a single register from the ADXL345.
 * This function is unchanged.
 */
static uint8_t adxl_read_reg(I2C_TypeDef *I2Cx, uint8_t reg)
{
    uint8_t data;
    // This correctly calls our new wrapper function
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
    // This correctly calls our new wrapper function with len=6
    adxl_read_multi(I2Cx, ADXL345_REG_DATAX0, data_buf, 6);

    // Combine bytes into 16-bit values
    // (Data is little-endian)
    accel_data[0] = (int16_t)(data_buf[1] << 8 | data_buf[0]); // X
    accel_data[1] = (int16_t)(data_buf[3] << 8 | data_buf[2]); // Y
    accel_data[2] = (int16_t)(data_buf[5] << 8 | data_buf[4]); // Z
}