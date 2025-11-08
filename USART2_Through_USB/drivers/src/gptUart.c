#include "gptUart.h"

#define UART2_TX_PORT      GPIOA
#define UART2_TX_PIN       2u     // PA2
#define UART2_RX_PIN       3u     // PA3
#define UART2_AF_USART2    7u     // AF7 for USART2




void UART2_Init(uint32_t pclk_hz, uint32_t baud)
{
    // ---- Enable Clocks ----
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;    // GPIOA clock
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;   // USART2 clock

    // ---- Configure PA2 (TX) and PA3 (RX) ----
    // Alternate function mode
    UART2_TX_PORT->MODER &= ~((3u << (UART2_TX_PIN * 2u)) | (3u << (UART2_RX_PIN * 2u)));
    UART2_TX_PORT->MODER |=  ((2u << (UART2_TX_PIN * 2u)) | (2u << (UART2_RX_PIN * 2u)));

    // AF7 for USART2
    UART2_TX_PORT->AFR[0] &= ~((0xFu << (UART2_TX_PIN * 4u)) | (0xFu << (UART2_RX_PIN * 4u)));
    UART2_TX_PORT->AFR[0] |=  ((UART2_AF_USART2 << (UART2_TX_PIN * 4u)) | (UART2_AF_USART2 << (UART2_RX_PIN * 4u)));

    // Very high speed
    UART2_TX_PORT->OSPEEDR |=  ((3u << (UART2_TX_PIN * 2u)) | (3u << (UART2_RX_PIN * 2u)));

    // Push-pull, no pull-up/down
    UART2_TX_PORT->OTYPER &= ~((1u << UART2_TX_PIN) | (1u << UART2_RX_PIN));
    UART2_TX_PORT->PUPDR  &= ~((3u << (UART2_TX_PIN * 2u)) | (3u << (UART2_RX_PIN * 2u)));

    // ---- Configure USART2 ----
    USART2->CR1 = 0;
    USART2->CR2 = 0;
    USART2->CR3 = 0;

    // Baud rate setup
    USART2->BRR = (pclk_hz + (baud / 2u)) / baud;

    // Enable TX, RX, and USART
    USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_UE);

    // Clear status
    (void)USART2->SR;
    (void)USART2->DR;
}

void UART2_PutChar(char c)
{
    while ((USART2->SR & USART_SR_TXE) == 0);
    USART2->DR = (uint16_t)c;
}

void UART2_Write(const char *s)
{
    while (*s)
        UART2_PutChar(*s++);
}

void UART2_Flush(void)
{
    while ((USART2->SR & USART_SR_TC) == 0);
}
void UART2_FlushRx(void)
{
    volatile char dummy;

    // Give UART time to receive any trailing newline
    for (volatile int delay = 0; delay < 50000; delay++);

    // Clear any pending bytes from RX buffer
    while (USART2->SR & USART_SR_RXNE)
        dummy = (char)USART2->DR;
}


char UART2_GetChar(void)
{
    while ((USART2->SR & USART_SR_RXNE) == 0);  // wait until a byte is received
    return (char)(USART2->DR & 0xFF);
}
void UART2_ReadString(char *buf, uint16_t maxlen)
{
    uint16_t i = 0;
    char c;
    while (i < (maxlen - 1))
    {
        c = UART2_GetChar();
        if (c == '\r' || c == '\n') break;
        buf[i++] = c;
        //UART2_PutChar(c); // <-- echo back
    }
    buf[i] = '\0';
}

int32_t UART2_ReadInteger(void)
{
    char buf[16];
    uint8_t i = 0;
    char c;
    int32_t value = 0;
    int8_t sign = 1;

    // Prompt user (optional)
    // UART2_Write("Enter number: ");

    while (1)
    {
        c = UART2_GetChar();   // blocking receive

        // Stop on Enter key
        if (c == '\r' || c == '\n')
            break;

        // Handle backspace (optional)
        if (c == '\b' && i > 0)
        {
            i--;
            continue;
        }

        // Handle minus sign
        if (c == '-' && i == 0)
        {
            sign = -1;
            continue;
        }

        // Accept only digits
        if (c >= '0' && c <= '9')
        {
            if (i < sizeof(buf) - 1)
                buf[i++] = c;
        }
    }

    buf[i] = '\0';

    // Convert manually (no stdlib needed)
    for (uint8_t j = 0; j < i; j++)
    {
        value = value * 10 + (buf[j] - '0');
    }

    return value * sign;
    
}

uint8_t UART2_Available(void)
{
    return (USART2->SR & USART_SR_RXNE) ? 1 : 0;
}
