#include "uart.h"

#define GPIOAEN						(1U<<0)
#define UART2EN						(1U<<17)
#define CR1_TE						(1U<<3)
#define CR1_UE						(1U<<13)
#define SR_TXE						(1U<<7)
#define SYS_FREQ					16000000
#define APB1_CLK					SYS_FREQ
#define BAUDRATE					230400

static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate);
static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate);

// void uart2_write_char(char c);
// int _write(int file, char *ptr, int len) {
//     int i;
//     for (i = 0; i < len; i++) {
//         uart2_write_char(*ptr++);
//     }
//     return len;
// }
void uart2_tx_init(void){
	
	// Configure uart gpio pins
	// Enable clock access to gpioa
	RCC->AHB1ENR |=GPIOAEN;
	// Set PA2 mode to alternate function mode
	GPIOA->MODER &=~(1U<<4);
	GPIOA->MODER |= (1U<<5); 
	// Set PA2 alternate function type to UART_TX (AF07)
	GPIOA->AFR[0] |= (1U<<8);
	GPIOA->AFR[0] |= (1U<<9);
	GPIOA->AFR[0] |= (1U<<10);
	GPIOA->AFR[0] &=~(1U<<11);

	// Configure uart module
	// Enable clock access to uart2
	RCC->APB1ENR |=UART2EN;
	// Configure baudrate
	uart_set_baudrate(USART2, APB1_CLK, BAUDRATE);
	// COnfigure the tranfer direction
	USART2->CR1 |= CR1_TE;
	// Enable uart module
	USART2->CR1 |= CR1_UE;
}
void uart2_write_char(char c) {
    while (!(USART2->SR & USART_SR_TXE)); // wait until TX ready
    USART2->DR = c;
}
void uart2_write_string(const char *str) {
    while (*str) {                 // Loop until end of string
        uart2_write_char(*str++);       // Send each character
    }
}
void uart2_write_dec(uint8_t val) {
    if (val >= 100) {
        uart2_write_char('0' + val / 100);
        val %= 100;
        uart2_write_char('0' + val / 10);
        val %= 10;
        uart2_write_char('0' + val);
    } else if (val >= 10) {
        uart2_write_char('0' + val / 10);
        uart2_write_char('0' + val % 10);
    } else {
        uart2_write_char('0' + val);
    }
}
void uart2_write_int(int num){
    char buf[12]; // enough for 32-bit int
    int i = 0;
    if(num == 0) buf[i++] = '0';
    else {
        if(num < 0) {
            uart2_write_string("-"); 
            num = -num;
        }
        int temp = num;
        while(temp > 0) { buf[i++] = (temp % 10) + '0'; temp /= 10; }
        for(int j = i-1; j >= 0; j--) uart2_write_string((char[]){buf[j],0});
        return;
    }
    buf[i] = '\0';
    uart2_write_string(buf);
}
void uart2_write_hex(uint8_t val) {
    const char hex[] = "0123456789ABCDEF";
    uart2_write_char(hex[(val >> 4) & 0x0F]); // high nibble
    uart2_write_char(hex[val & 0x0F]);        // low nibble
}
void uart2_write_ip(uint8_t ip[4], const char* label) {
    uart2_write_string(label);
    uart2_write_string(" : ");
    for (int i = 0; i < 4; i++) {
        uart2_write_dec(ip[i]);
        if (i < 3) uart2_write_char('.');
    }
    uart2_write_string("\r\n");
}
static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate){
	USARTx->BRR = compute_uart_bd(PeriphClk, BaudRate);
}
static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate){
	return	((PeriphClk + (BaudRate/2U))/BaudRate);
}