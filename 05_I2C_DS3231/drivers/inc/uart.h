#ifndef UART_H
#define UART_H

#include "stm32f4xx.h"
#include <stdint.h>

void UART_Init(USART_TypeDef *USARTx, uint32_t pclk_hz, uint32_t baud);


// Generic versions that take USART instance pointer
void UART_PutChar(USART_TypeDef *USARTx, char c);
void UART_Write(USART_TypeDef *USARTx, const char *s);
void UART_Flush(USART_TypeDef *USARTx);
void UART_FlushRx(USART_TypeDef *USARTx);
char UART_GetChar(USART_TypeDef *USARTx);
void UART_ReadString(USART_TypeDef *USARTx, char *buf, uint16_t maxlen);
int32_t UART_ReadInteger(USART_TypeDef *USARTx);
uint8_t UART_Available(USART_TypeDef *USARTx);
void UART_WriteHexByte(USART_TypeDef *USARTx, uint8_t value);
void UART_WriteHex8(USART_TypeDef *USARTx, uint8_t value);
void UART_WriteHex32(USART_TypeDef *USARTx, uint32_t value);

#endif // UART_H