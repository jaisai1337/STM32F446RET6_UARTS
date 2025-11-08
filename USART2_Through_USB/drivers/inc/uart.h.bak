#ifndef UART_H_
#define UART_H_
#include "stdint.h"
#include "stm32f4xx.h"
void uart2_tx_init(void);

// UART write functions
void uart2_write_char(char c);
void uart2_write_string(const char *str);
void uart2_write_dec(uint8_t val);
void uart2_write_int(int val);
void uart2_write_hex(uint8_t val);
void uart2_write_ip(uint8_t *ip, const char *label);

#endif /* UART_H_ */