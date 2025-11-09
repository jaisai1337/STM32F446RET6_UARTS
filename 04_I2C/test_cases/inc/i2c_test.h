#ifndef __I2C_TEST_H__
#define __I2C_TEST_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"



// Common clock/speed settings
#define I2C_TEST_APB1_CLK_HZ   45000000u
#define I2C_TEST_APB2_CLK_HZ   90000000u
#define I2C_TEST_SPEED_HZ      100000u

// Slave addresses
#define I2C1_SLAVE_ADDR        0x3A
#define I2C2_SLAVE_ADDR        0x3A
#define I2C3_SLAVE_ADDR        0x3B


void I2C_Internal_Test_All(uint8_t master_id);


void I2C_Bus_Recovery(void);

#ifdef __cplusplus
}
#endif

#endif // __I2C_TEST_H__
