#ifndef DS3231_H
#define DS3231_H

#include <stdint.h>

typedef struct
{
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    uint8_t dow;
    uint8_t date;
    uint8_t month;
    uint8_t year;
} ds3231_data_struct;

void DS3231_Init(I2C_TypeDef *I2Cx);
void DS3231_SetDateTime(I2C_TypeDef *I2Cx,uint8_t year, uint8_t month, uint8_t date,
                        uint8_t dow, uint8_t hour, uint8_t minute, uint8_t second);
uint8_t DS3231_ReadDateTime(I2C_TypeDef *I2Cx,ds3231_data_struct *dt);
void DS3231_PrintDateTime(const ds3231_data_struct *dt);


//AT24C32
void AT24C32_Init(I2C_TypeDef *I2Cx);
void AT24C32_WriteData(I2C_TypeDef *I2Cx, uint16_t memAddr, uint8_t *data, uint16_t len);
void AT24C32_ReadData(I2C_TypeDef *I2Cx, uint16_t memAddr, uint8_t *buf, uint16_t len);


#endif // DS3231_H