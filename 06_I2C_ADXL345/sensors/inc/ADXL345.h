#ifndef ADXL345_TEST_H
#define ADXL345_TEST_H

#include <stdint.h>
#include "stm32f446xx.h" // For I2C_TypeDef
#include "i2c.h"         // For I2C_WRITE/I2C_READ

/*
 * ============================================================================
 * ADXL345 (Default I2C Address: 0x53)
 * * Assumes the ALT_ADDRESS pin (pin 12) is connected to GND.
 * If connected to VCC, the address is 0x1D.
 * ============================================================================
 */

// ADXL345 I2C Address
#define ADXL345_ADDR (0x53) 

// ADXL345 Register Map
#define ADXL345_REG_DEVID       0x00 // Device ID
#define ADXL345_REG_POWER_CTL   0x2D // Power-saving features control
#define ADXL345_REG_DATA_FORMAT 0x31 // Data format control
#define ADXL345_REG_DATAX0      0x32 // X-Axis Data 0
#define ADXL345_REG_DATAX1      0x33 // X-Axis Data 1
#define ADXL345_REG_DATAY0      0x34 // Y-Axis Data 0
#define ADXL345_REG_DATAY1      0x35 // Y-Axis Data 1
#define ADXL345_REG_DATAZ0      0x36 // Z-Axis Data 0
#define ADXL345_REG_DATAZ1      0x37 // Z-Axis Data 1

/*
 * @brief Checks for a valid connection by reading the DEVID register.
 * @param I2Cx: I2C peripheral (e.g., I2C1)
 * @return 0 on success (ID=0xE5), 1 on failure.
 */
uint8_t ADXL345_Test_Connection(I2C_TypeDef *I2Cx);

/*
 * @brief Initializes the ADXL345.
 * Wakes up the device and sets data format.
 * @param I2Cx: I2C peripheral (e.g., I2C1)
 */
void ADXL345_Init(I2C_TypeDef *I2Cx);

/*
 * @brief Reads the X, Y, and Z acceleration data.
 * @param I2Cx: I2C peripheral (e.g., I2C1)
 * @param accel_data: Pointer to an array of 3 int16_t's to store [X, Y, Z]
 */
void ADXL345_Read_Accel(I2C_TypeDef *I2Cx, int16_t *accel_data);

#endif // ADXL345_TEST_H