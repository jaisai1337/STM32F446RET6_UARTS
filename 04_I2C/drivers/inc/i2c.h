#ifndef I2C_H
#define I2C_H

#include "stm32f4xx.h"
#include <stdint.h>

#define I2C_WRITE 0
#define I2C_READ  1
#if 0
void I2C1_Init(uint32_t pclk1, uint32_t i2c_speed_hz);
void I2C1_Start(uint8_t address, uint8_t direction);
void I2C1_Write(uint8_t data);
uint8_t I2C1_ReadAck(void);
uint8_t I2C1_ReadNack(void);
void I2C1_Stop(void);
uint8_t I2C1_IsDeviceReady(uint8_t address);
#endif

#if 0
void I2C_Bus_Recovery(void);
void I2C1_Master_Init(uint32_t pclk1, uint32_t speed_hz);
void I2C2_Slave_Init(uint32_t pclk1, uint8_t own_address, uint32_t speed_hz);
void I2C3_Slave_Init(uint32_t pclk1, uint8_t own_address, uint32_t speed_hz);

int  I2C1_Master_Address(uint8_t addr, uint8_t direction);
void I2C1_Master_Write(uint8_t data);
void I2C1_Master_Stop(void);

uint8_t I2C2_Slave_Read(uint8_t *buf, uint8_t len);
#endif


void I2C_Master_Init(I2C_TypeDef *I2Cx, uint32_t pclk1, uint32_t speed_hz);
void I2C_Slave_Init(I2C_TypeDef *I2Cx, uint32_t pclk1, uint8_t own_address, uint32_t speed_hz);

int I2C_Master_Address(I2C_TypeDef *I2Cx, uint8_t addr, uint8_t direction);
void I2C_Master_Write(I2C_TypeDef *I2Cx, uint8_t data);
void I2C_Master_Stop(I2C_TypeDef *I2Cx);



#endif // I2C_H