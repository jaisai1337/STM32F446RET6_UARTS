#ifndef I2C_H
#define I2C_H

#include "stm32f4xx.h"
#include <stdint.h>

#define I2C_WRITE 0
#define I2C_READ  1


void I2C_Master_Init(I2C_TypeDef *I2Cx, uint32_t pclk1, uint32_t speed_hz);
void I2C_Slave_Init(I2C_TypeDef *I2Cx, uint32_t pclk1, uint8_t own_address, uint32_t speed_hz);

int I2C_Master_Address(I2C_TypeDef *I2Cx, uint8_t addr, uint8_t direction);
void I2C_Master_Write(I2C_TypeDef *I2Cx, uint8_t data);
void I2C_Master_Stop(I2C_TypeDef *I2Cx);



#endif // I2C_H