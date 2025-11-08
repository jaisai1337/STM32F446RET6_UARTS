#ifndef UART_H
#define UART_H

#include "stm32f4xx.h"
#include <stdint.h>

void UART2_InitTx(uint32_t pclk_hz, uint32_t baud);
void UART2_PutChar(char c);
void UART2_Write(const char *s);
char UART2_GetChar(void);         // blocking receive
void UART2_ReadString(char *buf, uint16_t maxlen); // read string until newline
int32_t UART2_ReadInteger(void); // read integer from UART
uint8_t UART2_Available(void);    // check if RX data ready
void UART2_Flush(void);
void UART2_FlushRx(void);

#endif
