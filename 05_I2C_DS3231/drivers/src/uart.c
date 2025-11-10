#include "uart.h"

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

void UART_WriteHex32(USART_TypeDef *USARTx, uint32_t value)
{
    const char hex[] = "0123456789ABCDEF";
    UART_PutChar(USARTx, '0');
    UART_PutChar(USARTx, 'x');
    for (int i = 28; i >= 0; i -= 4) {
        UART_PutChar(USARTx, hex[(value >> i) & 0x0F]);
    }
}

//----------------- UART2_Init ----------------
void UART2_Init(uint32_t pclk_hz, uint32_t baud){
    // ---- Enable Clocks ----
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;    // GPIOA clock
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;   // USART2 clock

    // ---- Configure PA2 (TX) and PA3 (RX) ----
    GPIOA->MODER &= ~((3u << (2 * 2u)) | (3u << (3 * 2u))); // Clear mode bits
    GPIOA->MODER |=  ((2u << (2 * 2u)) | (2u << (3 * 2u))); // Set to AF mode

    // Set AF7 (USART2) for PA2 and PA3
    GPIOA->AFR[0] &= ~((0xFu << (2 * 4u)) | (0xFu << (3 * 4u))); // Clear AF bits
    GPIOA->AFR[0] |=  ((7u << (2 * 4u)) | (7u << (3 * 4u))); // Set AF7

    // Configure speed, type, pull-up/pull-down
    GPIOA->OSPEEDR |=  ((3u << (2 * 2u)) | (3u << (3 * 2u))); // Very high speed
    GPIOA->OTYPER &= ~((1u << 2) | (1u << 3)); // Push-pull
    GPIOA->PUPDR  &= ~((3u << (2 * 2u)) | (3u << (3 * 2u))); // No pull-up/pull-down

    // ---- Configure USART2 ----
    USART2->CR1 = USART2->CR2 = USART2->CR3 = 0; // Reset USART registers
    USART2->BRR = (pclk_hz + (baud / 2u)) / baud; // Set baud rate
    USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_UE); // Enable TX, RX, USART2
    (void)USART2->SR; (void)USART2->DR; // Clear status registers
}

// ---------------------- UART1 Init ----------------------
void UART1_Init(uint32_t pclk_hz, uint32_t baud){
    // ---- Enable Clocks ----
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Enable GPIOA clock
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN; // Enable USART1 clock

    // ---- Configure PA9 (TX) and PA10 (RX) ----
    GPIOA->MODER &= ~((3u << (9*2)) | (3u << (10*2))); // Clear mode bits for PA9 and PA10
    GPIOA->MODER |=  ((2u << (9*2)) | (2u << (10*2))); // Set PA9 and PA10 to Alternate Function mode
    
    // Set AF7 (USART1) for PA9 and PA10
    GPIOA->AFR[1] &= ~((0xFu << ((9-8)*4)) | (0xFu << ((10-8)*4))); // Clear AF bits for PA9 and PA10
    GPIOA->AFR[1] |=  ((7u << ((9-8)*4)) | (7u << ((10-8)*4))); // Set AF7 (USART1) for PA9 and PA10

    // Configure speed, type, pull-up/pull-down
    GPIOA->OSPEEDR |= ((3u << (9*2)) | (3u << (10*2))); // Very high speed
    GPIOA->OTYPER  &= ~((1u << 9) | (1u << 10));        // Push-pull
    GPIOA->PUPDR   &= ~((3u << (9*2)) | (3u << (10*2))); // No pull-up/pull-down
    
    // ---- Configure USART1 ----
    USART1->CR1 = USART1->CR2 = USART1->CR3 = 0;        // Reset USART1 registers
    USART1->BRR = (pclk_hz + (baud / 2u)) / baud;       // Set baud rate
    USART1->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_UE); // Enable TX, RX, and USART1
    (void)USART1->SR; (void)USART1->DR;                          // Clear status registers
}
void UART3_Init(uint32_t pclk_hz, uint32_t baud){
    // Enable clocks, configure GPIO pins, set baud rate, and enable UART3
    // ---- Enable Clocks ----
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // Enable GPIOB clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // Enable GPIOC clock
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN; // Enable USART3 clock    
    
    // Configure PB10 (TX) and PC5 (RX)
    GPIOB->MODER &= ~((3u << (10*2))); // Clear mode bits for PB10
    GPIOB->MODER |=  ((2u << (10*2))); // Set PB10 to Alternate Function mode
    GPIOC->MODER &= ~((3u << (5*2)));  // Clear mode bits for PC5
    GPIOC->MODER |=  ((2u << (5*2)));  // Set PC5 to Alternate Function mode


    // Set AF7 (USART3) for PB10 and PC5
    GPIOB->AFR[1] &= ~((0xFu << ((10-8)*4))); // Clear AF bits for PB10
    GPIOB->AFR[1] |=  ((7u << ((10-8)*4))); // Set AF7 (USART3) for PB10
    GPIOC->AFR[0] &= ~((0xFu << (5*4))); // Clear AF bits for PC5
    GPIOC->AFR[0] |=  ((7u << (5*4))); // Set AF7 (USART3) for PC5

    // Configure speed, type, pull-up/pull-down
    GPIOB->OSPEEDR |= ((3u << (10*2))); // Very high speed for PB10
    GPIOC->OSPEEDR |= ((3u << (5*2)));  // Very high speed for PC5
    GPIOB->OTYPER  &= ~((1u << 10));        // Push-pull for PB10
    GPIOC->OTYPER  &= ~((1u << 5));         // Push-pull for PC5
    GPIOB->PUPDR   &= ~((3u << (10*2))); // No pull-up/pull-down for PB10
    GPIOC->PUPDR   &= ~((3u << (5*2)));  // No pull-up/pull-down for PC5
    
    // Configure USART3
    USART3->CR1 = USART3->CR2 = USART3->CR3 = 0;        // Reset USART3 registers
    USART3->BRR = (pclk_hz + (baud / 2u)) / baud;       // Set baud rate
    USART3->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_UE); // Enable TX, RX, and USART3
    (void)USART3->SR; (void)USART3->DR;                          // Clear status registers
}
void UART4_Init(uint32_t pclk_hz, uint32_t baud){
    // Enable clocks, configure GPIO pins, set baud rate, and enable UART4
    // ---- Enable Clocks ----
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Enable GPIOA clock
    RCC->APB1ENR |= RCC_APB1ENR_UART4EN; // Enable UART4 clock

    // ---- Configure PA0 (TX) and PA1 (RX) ----
    GPIOA->MODER &= ~((3u << (0*2)) | (3u << (1*2))); // Clear mode bits for PA0 and PA1
    GPIOA->MODER |=  ((2u << (0*2)) | (2u << (1*2))); // Set PA0 and PA1 to Alternate Function mode

    // Set AF8 (UART4) for PA0 and PA1
    GPIOA->AFR[0] &= ~((0xFu << (0*4)) | (0xFu << (1*4))); // Clear AF bits for PA0 and PA1
    GPIOA->AFR[0] |=  ((8u << (0*4)) | (8u << (1*4))); // Set AF8 (UART4) for PA0 and PA1

    // Configure speed, type, pull-up/pull-down
    GPIOA->OSPEEDR |= ((3u << (0*2)) | (3u << (1*2))); // Very high speed
    GPIOA->OTYPER  &= ~((1u << 0) | (1u << 1));        // Push-pull
    GPIOA->PUPDR   &= ~((3u << (0*2)) | (3u << (1*2))); // No pull-up/pull-down

    // ---- Configure UART4 ----
    UART4->CR1 = UART4->CR2 = UART4->CR3 = 0;        // Reset UART4 registers
    UART4->BRR = (pclk_hz + (baud / 2u)) / baud;       // Set baud rate
    UART4->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_UE); // Enable TX, RX, and UART4
    (void)UART4->SR; (void)UART4->DR;                          // Clear status registers
}
void UART5_Init(uint32_t pclk_hz, uint32_t baud){
    // Enable clocks, configure GPIO pins, set baud rate, and enable UART5
    // ---- Enable Clocks ----
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // Enable GPIOC clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; // Enable GPIOD clock
    RCC->APB1ENR |= RCC_APB1ENR_UART5EN; // Enable UART5 clock    

    // Configure PC12 (TX) and PD2 (RX)
    GPIOC->MODER &= ~((3u << (12*2))); // Clear mode bits for PC12
    GPIOC->MODER |=  ((2u << (12*2))); // Set PC12 to Alternate Function mode
    GPIOD->MODER &= ~((3u << (2*2)));  // Clear mode bits for PD2
    GPIOD->MODER |=  ((2u << (2*2)));  // Set PD2 to Alternate Function mode
    
    // Set AF8 (UART5) for PC12 and PD2
    GPIOC->AFR[1] &= ~((0xFu << ((12-8)*4))); // Clear AF bits for PC12
    GPIOC->AFR[1] |=  ((8u << ((12-8)*4))); // Set AF8 (UART5) for PC12
    GPIOD->AFR[0] &= ~((0xFu << (2*4))); // Clear AF bits for PD2
    GPIOD->AFR[0] |=  ((8u << (2*4))); // Set AF8 (UART5) for PD2
    
    // Configure speed, type, pull-up/pull-down
    GPIOC->OSPEEDR |= ((3u << (12*2))); // Very high speed for PC12
    GPIOD->OSPEEDR |= ((3u << (2*2)));  // Very high speed for PD2
    GPIOC->OTYPER  &= ~((1u << 12));        // Push-pull for PC12
    GPIOD->OTYPER  &= ~((1u << 2));         // Push-pull for PD2
    GPIOC->PUPDR   &= ~((3u << (12*2))); // No pull-up/pull-down for PC12
    GPIOD->PUPDR   &= ~((3u << (2*2)));  // No pull-up/pull-down for PD2
    
    // Configure UART5
    UART5->CR1 = UART5->CR2 = UART5->CR3 = 0;        // Reset UART5 registers
    UART5->BRR = (pclk_hz + (baud / 2u)) / baud;       // Set baud rate
    UART5->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_UE); // Enable TX, RX, and UART5
    (void)UART5->SR; (void)UART5->DR;                          // Clear status registers
}
void UART6_Init(uint32_t pclk_hz, uint32_t baud){
    // Enable clocks, configure GPIO pins, set baud rate, and enable UART6
    // ---- Enable Clocks ----
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // Enable GPIOC clock
    RCC->APB2ENR |= RCC_APB2ENR_USART6EN; // Enable USART6 clock    
    
    // Configure PC6 (TX) and PC7 (RX)
    GPIOC->MODER &= ~((3u << (6*2)) | (3u << (7*2))); // Clear mode bits for PC6 and PC7
    GPIOC->MODER |=  ((2u << (6*2)) | (2u << (7*2))); // Set PC6 and PC7 to Alternate Function mode
    
    // Set AF8 (USART6) for PC6 and PC7
    GPIOC->AFR[0] &= ~((0xFu << (6*4)) | (0xFu << (7*4))); // Clear AF bits for PC6 and PC7
    GPIOC->AFR[0] |=  ((8u << (6*4)) | (8u << (7*4))); // Set AF8 (USART6) for PC6 and PC7
    
    // Configure speed, type, pull-up/pull-down
    GPIOC->OSPEEDR |= ((3u << (6*2)) | (3u << (7*2))); // Very high speed
    GPIOC->OTYPER  &= ~((1u << 6) | (1u << 7));        // Push-pull
    GPIOC->PUPDR   &= ~((3u << (6*2)) | (3u << (7*2))); // No pull-up/pull-down
    
    // ---- Configure USART6 ----
    USART6->CR1 = USART6->CR2 = USART6->CR3 = 0;        // Reset USART6 registers
    USART6->BRR = (pclk_hz + (baud / 2u)) / baud;       // Set baud rate
    USART6->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_UE); // Enable TX, RX, and USART6
    (void)USART6->SR; (void)USART6->DR;                          // Clear status registers
}

