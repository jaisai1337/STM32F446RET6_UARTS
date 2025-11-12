#ifndef __DS3231_TEST_H__
#define __DS3231_TEST_H__

#include "stm32f4xx.h"

void DS3231_Test_Run(I2C_TypeDef *I2Cx, uint8_t use_rtc, uint8_t use_eeprom);

#endif
