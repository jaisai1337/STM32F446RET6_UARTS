#include "uart.h"

__attribute__((weak)) int __io_putchar(int ch)
{
    // Send the character to USART2
    UART_PutChar(USART2, (uint8_t)ch);
    return ch;
}

__attribute__((weak)) int __io_getchar(void)
{
    // Wait for and return a character from USART2
    return (int)UART_GetChar(USART2);
}

static void uart_configure_pin(GPIO_TypeDef *Port, uint8_t Pin, uint8_t AF_Num)
{
    // 1. Enable GPIO clock
    RCC->AHB1ENR |= (1u << (((uint32_t)Port - (uint32_t)GPIOA) / 0x400));

    // 2. Configure Pin: AF, High Speed, Push-Pull, No Pull
    Port->MODER   &= ~(3u << (Pin * 2)); // Clear mode
    Port->MODER   |=  (2u << (Pin * 2)); // Set to AF mode
    Port->OSPEEDR |=  (3u << (Pin * 2)); // Very high speed
    Port->OTYPER  &= ~(1u << Pin);      // Push-pull
    Port->PUPDR   &= ~(3u << (Pin * 2)); // No pull-up/pull-down

    // 3. Configure Alternate Function
    uint8_t afr_idx   = Pin / 8; // AFR[0] or AFR[1]
    uint8_t afr_shift = (Pin % 8) * 4;
    Port->AFR[afr_idx] &= ~(0xFu << afr_shift);
    Port->AFR[afr_idx] |= (AF_Num << afr_shift);
}

// ============================================================================
//                          PUBLIC FUNCTIONS
// ============================================================================

void UART_Init(USART_TypeDef *USARTx, uint32_t pclk_hz, uint32_t baud)
{
    // 1. Enable the peripheral clock and configure GPIOs
    if (USARTx == USART1) {
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
        uart_configure_pin(GPIOA, 9,  7); // PA9  (TX)
        uart_configure_pin(GPIOA, 10, 7); // PA10 (RX)
    }
    else if (USARTx == USART2) {
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
        uart_configure_pin(GPIOA, 2, 7); // PA2 (TX)
        uart_configure_pin(GPIOA, 3, 7); // PA3 (RX)
    }
    else if (USARTx == USART3) {
        RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
        uart_configure_pin(GPIOB, 10, 7); // PB10 (TX)
        uart_configure_pin(GPIOC, 5,  7); // PC5  (RX)
    }
    else if (USARTx == UART4) {
        RCC->APB1ENR |= RCC_APB1ENR_UART4EN;
        uart_configure_pin(GPIOA, 0, 8); // PA0 (TX)
        uart_configure_pin(GPIOA, 1, 8); // PA1 (RX)
    }
    else if (USARTx == UART5) {
        RCC->APB1ENR |= RCC_APB1ENR_UART5EN;
        uart_configure_pin(GPIOC, 12, 8); // PC12 (TX)
        uart_configure_pin(GPIOD, 2,  8); // PD2  (RX)
    }
    else if (USARTx == USART6) {
        RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
        uart_configure_pin(GPIOC, 6, 8); // PC6 (TX)
        uart_configure_pin(GPIOC, 7, 8); // PC7 (RX)
    }
    else {
        // Invalid peripheral
        return;
    }

    // 2. Configure USART peripheral (common logic)
    USARTx->CR1 = USARTx->CR2 = USARTx->CR3 = 0;    // Reset registers
    USARTx->BRR = (pclk_hz + (baud / 2u)) / baud;  // Set baud rate
    USARTx->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_UE); // Enable TX, RX, USART
    (void)USARTx->SR; (void)USARTx->DR;            // Clear status registers
}


// ---------------------- Common Generic Functions ----------------------
void UART_PutChar(USART_TypeDef *USARTx, char c)
{
    while ((USARTx->SR & USART_SR_TXE) == 0);
    USARTx->DR = (uint16_t)c;
}

void UART_Write(USART_TypeDef *USARTx, const char *s)
{
    while (*s)
        UART_PutChar(USARTx, *s++);
}

void UART_Flush(USART_TypeDef *USARTx)
{
    while ((USARTx->SR & USART_SR_TC) == 0);
}

void UART_FlushRx(USART_TypeDef *USARTx)
{
    volatile char dummy;
    for (volatile int delay = 0; delay < 50000; delay++);
    while (USARTx->SR & USART_SR_RXNE)
        dummy = (char)USARTx->DR;
}

char UART_GetChar(USART_TypeDef *USARTx)
{
    while ((USARTx->SR & USART_SR_RXNE) == 0);
    return (char)(USARTx->DR & 0xFF);
}

void UART_ReadString(USART_TypeDef *USARTx, char *buf, uint16_t maxlen)
{
    uint16_t i = 0;
    char c;
    while (i < (maxlen - 1))
    {
        c = UART_GetChar(USARTx);
        if (c == '\r' || c == '\n') break;
        buf[i++] = c;
    }
    buf[i] = '\0';
}


int32_t UART_ReadInteger(USART_TypeDef *USARTx)
{
    char buf[16];
    uint8_t i = 0;
    char c;
    int32_t value = 0;
    int8_t sign = 1;

    while (1)
    {
        c = UART_GetChar(USARTx);
        if (c == '\r' || c == '\n') break;
        if (c == '\b' && i > 0) { i--; continue; }
        if (c == '-' && i == 0) { sign = -1; continue; }

        if (c >= '0' && c <= '9')
        {
            if (i < sizeof(buf) - 1)
                buf[i++] = c;
        }
    }

    buf[i] = '\0';
    for (uint8_t j = 0; j < i; j++)
        value = value * 10 + (buf[j] - '0');

    return value * sign;
}

uint8_t UART_Available(USART_TypeDef *USARTx)
{
    return (USARTx->SR & USART_SR_RXNE) ? 1 : 0;
}

void UART_WriteHexByte(USART_TypeDef *USARTx, uint8_t value)
{
    const char hex[] = "0123456789ABCDEF";
    UART_PutChar(USARTx, '0');
    UART_PutChar(USARTx, 'x');
    UART_PutChar(USARTx, hex[(value >> 4) & 0x0F]);
    UART_PutChar(USARTx, hex[value & 0x0F]);
}