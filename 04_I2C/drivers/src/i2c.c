#include "i2c.h"
#include "stm32f4xx.h"
#if 0
// ----------------------- I2C1 Init -----------------------
void I2C1_Init(uint32_t pclk1, uint32_t i2c_speed_hz)
{
    // Enable clocks for GPIOB and I2C1
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // Configure PB6 (SCL) and PB7 (SDA) as AF4
    GPIOB->MODER   &= ~((3u << (6 * 2)) | (3u << (7 * 2)));
    GPIOB->MODER   |=  ((2u << (6 * 2)) | (2u << (7 * 2)));
    GPIOB->OTYPER  |=  ((1u << 6) | (1u << 7));   // open-drain
    GPIOB->OSPEEDR |=  ((3u << (6 * 2)) | (3u << (7 * 2))); // very high speed
    GPIOB->PUPDR   &= ~((3u << (6 * 2)) | (3u << (7 * 2)));
    GPIOB->PUPDR   |=  ((1u << (6 * 2)) | (1u << (7 * 2))); // pull-up
    GPIOB->AFR[0]  &= ~((0xFu << (6 * 4)) | (0xFu << (7 * 4)));
    GPIOB->AFR[0]  |=  ((4u << (6 * 4)) | (4u << (7 * 4))); // AF4

    // Reset I2C1
    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    // Configure timing: standard-mode 100 kHz (simple formula)
    uint32_t ccr = (pclk1 / (i2c_speed_hz * 2));
    if (ccr < 4) ccr = 4;

    I2C1->CR2 = (pclk1 / 1000000U); // frequency in MHz
    I2C1->CCR = ccr;
    I2C1->TRISE = (pclk1 / 1000000U) + 1;

    // Enable I2C1
    I2C1->CR1 |= I2C_CR1_PE;
}

// ----------------------- Basic I2C Operations -----------------------

void I2C1_Start(uint8_t address, uint8_t direction)
{
    // Generate START condition
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB)); // wait for start bit

    // Send address
    I2C1->DR = (address << 1) | direction;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2; // clear ADDR flag
}

void I2C1_Write(uint8_t data)
{
    while (!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = data;
    while (!(I2C1->SR1 & I2C_SR1_BTF));
}

uint8_t I2C1_ReadAck(void)
{
    I2C1->CR1 |= I2C_CR1_ACK;
    while (!(I2C1->SR1 & I2C_SR1_RXNE));
    return I2C1->DR;
}

uint8_t I2C1_ReadNack(void)
{
    I2C1->CR1 &= ~I2C_CR1_ACK;
    I2C1->CR1 |= I2C_CR1_STOP;
    while (!(I2C1->SR1 & I2C_SR1_RXNE));
    return I2C1->DR;
}

void I2C1_Stop(void)
{
    I2C1->CR1 |= I2C_CR1_STOP;
}

// ----------------------- Device Ready Check -----------------------

uint8_t I2C1_IsDeviceReady(uint8_t address)
{
    uint8_t ready = 0;

    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));

    I2C1->DR = (address << 1) | I2C_WRITE;
    while (!(I2C1->SR1 & (I2C_SR1_ADDR | I2C_SR1_AF)));

    if (I2C1->SR1 & I2C_SR1_ADDR)
    {
        (void)I2C1->SR2;
        ready = 1;
    }

    I2C1->SR1 &= ~I2C_SR1_AF;
    I2C1->CR1 |= I2C_CR1_STOP;

    return ready;
}
#endif


// ============================================================
//  Utility: Recover I2C bus if stuck (pulls SCL/SDA high)
// ============================================================
void I2C_Bus_Recovery(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    GPIOB->MODER &= ~((3<<(6*2))|(3<<(7*2)));
    GPIOB->MODER |=  ((1<<(6*2))|(1<<(7*2)));      // output
    GPIOB->OTYPER |= ((1<<6)|(1<<7));
    GPIOB->ODR    |= ((1<<6)|(1<<7));              // drive high

    for (int i=0; i<9; i++) {
        GPIOB->ODR &= ~(1<<6);
        for (volatile int d=0; d<300; d++);
        GPIOB->ODR |= (1<<6);
        for (volatile int d=0; d<300; d++);
    }

    GPIOB->ODR &= ~(1<<7);
    for (volatile int d=0; d<300; d++);
    GPIOB->ODR |= (1<<6)|(1<<7);
    GPIOB->MODER &= ~((3<<(6*2))|(3<<(7*2)));
}


/**
 * @brief Initialize any I2C peripheral as a Master (I2C1, I2C2, I2C3)
 * @param I2Cx       Pointer to I2C peripheral (I2C1, I2C2, or I2C3)
 * @param pclk1      APB1 clock frequency in Hz
 * @param speed_hz   Desired I2C bus speed (e.g., 100000 for 100kHz)
 */
void I2C_Master_Init(I2C_TypeDef *I2Cx, uint32_t pclk1, uint32_t speed_hz)
{
    volatile uint32_t tmp;

    /* --------------------------
     * 1. Enable GPIO & I2C clocks
     * -------------------------- */
    if (I2Cx == I2C1) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
        RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
        tmp = RCC->AHB1ENR; (void)tmp;

        // PB6 = SCL, PB7 = SDA (AF4)
        GPIOB->MODER   &= ~((3u << (6 * 2)) | (3u << (7 * 2)));
        GPIOB->MODER   |=  ((2u << (6 * 2)) | (2u << (7 * 2)));
        GPIOB->OTYPER  |=  ((1u << 6) | (1u << 7));
        GPIOB->OSPEEDR |=  ((3u << (6 * 2)) | (3u << (7 * 2)));
        GPIOB->PUPDR   &= ~((3u << (6 * 2)) | (3u << (7 * 2)));
        GPIOB->PUPDR   |=  ((1u << (6 * 2)) | (1u << (7 * 2)));
        GPIOB->AFR[0]  &= ~((0xFu << (6 * 4)) | (0xFu << (7 * 4)));
        GPIOB->AFR[0]  |=  ((4u << (6 * 4)) | (4u << (7 * 4)));
    }

    else if (I2Cx == I2C2) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
        RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
        tmp = RCC->AHB1ENR; (void)tmp;

        // PB10 = SCL (AF4)
        GPIOB->MODER   &= ~(3u << (10 * 2));
        GPIOB->MODER   |=  (2u << (10 * 2));
        GPIOB->OTYPER  |=  (1u << 10);
        GPIOB->OSPEEDR |=  (3u << (10 * 2));
        GPIOB->PUPDR   &= ~(3u << (10 * 2));
        GPIOB->PUPDR   |=  (1u << (10 * 2));
        GPIOB->AFR[1]  &= ~(0xFu << ((10 - 8) * 4));
        GPIOB->AFR[1]  |=  (4u  << ((10 - 8) * 4));

        // PC12 = SDA (AF4)
        GPIOC->MODER   &= ~(3u << (12 * 2));
        GPIOC->MODER   |=  (2u << (12 * 2));
        GPIOC->OTYPER  |=  (1u << 12);
        GPIOC->OSPEEDR |=  (3u << (12 * 2));
        GPIOC->PUPDR   &= ~(3u << (12 * 2));
        GPIOC->PUPDR   |=  (1u << (12 * 2));
        GPIOC->AFR[1]  &= ~(0xFu << ((12 - 8) * 4));
        GPIOC->AFR[1]  |=  (4u  << ((12 - 8) * 4));
    }

    else if (I2Cx == I2C3) {
        // === Enable GPIOA, GPIOC, and I2C3 clocks ===
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
        RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;
        volatile uint32_t tmp = RCC->AHB1ENR; (void)tmp;

        // === Configure PA8 = I2C3_SCL (AF4) ===
        GPIOA->MODER   &= ~(3u << (8 * 2));
        GPIOA->MODER   |=  (2u << (8 * 2));        // Alternate function mode
        GPIOA->OTYPER  |=  (1u << 8);              // Open-drain
        GPIOA->OSPEEDR |=  (3u << (8 * 2));        // High speed
        GPIOA->PUPDR   &= ~(3u << (8 * 2));
        GPIOA->PUPDR   |=  (1u << (8 * 2));        // Pull-up
        GPIOA->AFR[1]  &= ~(0xFu << ((8 - 8) * 4));
        GPIOA->AFR[1]  |=  (4u  << ((8 - 8) * 4)); // AF4 for I2C3_SCL

        // === Configure PC9 = I2C3_SDA (AF4) ===
        GPIOC->MODER   &= ~(3u << (9 * 2));
        GPIOC->MODER   |=  (2u << (9 * 2));        // Alternate function
        GPIOC->OTYPER  |=  (1u << 9);              // Open-drain
        GPIOC->OSPEEDR |=  (3u << (9 * 2));        // High speed
        GPIOC->PUPDR   &= ~(3u << (9 * 2));
        GPIOC->PUPDR   |=  (1u << (9 * 2));        // Pull-up
        GPIOC->AFR[1]  &= ~(0xFu << ((9 - 8) * 4));
        GPIOC->AFR[1]  |=  (4u  << ((9 - 8) * 4)); // AF4 for I2C3_SDA

    }

    /* --------------------------
     *  I2C Reset & Timing Setup
     * -------------------------- */
    I2Cx->CR1 |= I2C_CR1_SWRST;
    for (volatile int i = 0; i < 200; i++);
    I2Cx->CR1 &= ~I2C_CR1_SWRST;

    I2Cx->CR1 = 0;
    I2Cx->CR2   = (pclk1 / 1000000U);
    I2Cx->CCR   = (pclk1 / (speed_hz * 2U));
    I2Cx->TRISE = (pclk1 / 1000000U) + 1;

    I2Cx->CR1 |= I2C_CR1_PE;
}

int I2C_Master_Address(I2C_TypeDef *I2Cx, uint8_t addr, uint8_t direction)
{
    // Wait until bus is free
    while (I2Cx->SR2 & I2C_SR2_BUSY);

    // Generate START condition
    I2Cx->CR1 |= I2C_CR1_START;
    while (!(I2Cx->SR1 & I2C_SR1_SB));   // Wait for start bit set

    // Send 7-bit address + R/W bit
    I2Cx->DR = (addr << 1) | direction;

    // Wait for ACK/NACK
    while (1) {
        if (I2Cx->SR1 & I2C_SR1_ADDR) {
            (void)I2Cx->SR2;             // Clear ADDR flag
            return 0;                    // ACK received
        }
        if (I2Cx->SR1 & I2C_SR1_AF) {    // NACK received
            I2Cx->SR1 &= ~I2C_SR1_AF;
            I2Cx->CR1 |= I2C_CR1_STOP;
            return -1;
        }
    }
}

/**
 * @brief  Write a single byte over I²C (blocking).
 * @param  I2Cx   Pointer to I2C instance (I2C1, I2C2, or I2C3)
 * @param  data   Byte to transmit
 */
void I2C_Master_Write(I2C_TypeDef *I2Cx, uint8_t data)
{
    while (!(I2Cx->SR1 & I2C_SR1_TXE));  // Wait until data register empty
    I2Cx->DR = data;                     // Send data
    // Optionally wait for transfer complete
     while (!(I2Cx->SR1 & I2C_SR1_BTF));
}

/**
 * @brief  Generate STOP condition on the I²C bus.
 * @param  I2Cx   Pointer to I2C instance (I2C1, I2C2, or I2C3)
 */
void I2C_Master_Stop(I2C_TypeDef *I2Cx)
{
    I2Cx->CR1 |= I2C_CR1_STOP;           // Generate STOP condition
}

/**
 * @brief  Generic I2C Slave initialization (supports I2C2 and I2C3)
 * @param  I2Cx         I2C2 or I2C3 instance
 * @param  pclk1        APB1 clock frequency (Hz)
 * @param  own_address  7-bit I2C slave address
 * @param  speed_hz     I2C bus speed (e.g. 100000 for 100 kHz)
 */
void I2C_Slave_Init(I2C_TypeDef *I2Cx, uint32_t pclk1, uint8_t own_address, uint32_t speed_hz)
{
    // ------------------------------
    // 1. Enable GPIO & I2C clocks
    // ------------------------------
    if (I2Cx == I2C1) {
        // --- Enable GPIO & I2C1 clocks ---
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
        RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
        volatile uint32_t tmp = RCC->AHB1ENR;
        (void)tmp;

        // PB6 = SCL, PB7 = SDA (AF4)
        GPIOB->MODER   &= ~((3u<<(6*2)) | (3u<<(7*2)));
        GPIOB->MODER   |=  ((2u<<(6*2)) | (2u<<(7*2)));
        GPIOB->OTYPER  |=  ((1u<<6) | (1u<<7));
        GPIOB->OSPEEDR |=  ((3u<<(6*2)) | (3u<<(7*2)));
        GPIOB->PUPDR   &= ~((3u<<(6*2)) | (3u<<(7*2)));
        GPIOB->PUPDR   |=  ((1u<<(6*2)) | (1u<<(7*2)));
        GPIOB->AFR[0]  &= ~((0xFu<<(6*4)) | (0xFu<<(7*4)));
        GPIOB->AFR[0]  |=  ((4u<<(6*4)) | (4u<<(7*4)));

        // --- Reset I2C1 peripheral ---
        I2C1->CR1 |= I2C_CR1_SWRST;
        for (volatile int i = 0; i < 200; i++);
        I2C1->CR1 &= ~I2C_CR1_SWRST;

        // --- Configure timing ---
        I2C1->CR2   = (pclk1 / 1000000U);
        I2C1->CCR   = (pclk1 / (speed_hz * 2U));
        I2C1->TRISE = (pclk1 / 1000000U) + 1;

        // --- Own address ---
        I2C1->OAR1 = (1u << 14) | ((uint32_t)(own_address << 1));   // 7-bit mode

        // --- Enable ACK and peripheral ---
        I2C1->CR1 |= I2C_CR1_ACK | I2C_CR1_PE;
    }else if (I2Cx == I2C2)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
        RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

        volatile uint32_t tmp = RCC->AHB1ENR;
        (void)tmp;

        // ==== PB10 = SCL (AF4) ====
        GPIOB->MODER   &= ~(3u << (10 * 2));
        GPIOB->MODER   |=  (2u << (10 * 2));       // AF
        GPIOB->OTYPER  |=  (1u << 10);             // OD
        GPIOB->OSPEEDR |=  (3u << (10 * 2));       // High speed
        GPIOB->PUPDR   &= ~(3u << (10 * 2));
        GPIOB->PUPDR   |=  (1u << (10 * 2));       // PU
        GPIOB->AFR[1]  &= ~(0xFu << ((10 - 8) * 4));
        GPIOB->AFR[1]  |=  (4u  << ((10 - 8) * 4)); // AF4

        // ==== PC12 = SDA (AF4) ====
        GPIOC->MODER   &= ~(3u << (12 * 2));
        GPIOC->MODER   |=  (2u << (12 * 2));       // AF
        GPIOC->OTYPER  |=  (1u << 12);
        GPIOC->OSPEEDR |=  (3u << (12 * 2));
        GPIOC->PUPDR   &= ~(3u << (12 * 2));
        GPIOC->PUPDR   |=  (1u << (12 * 2));
        GPIOC->AFR[1]  &= ~(0xFu << ((12 - 8) * 4));
        GPIOC->AFR[1]  |=  (4u  << ((12 - 8) * 4)); // AF4
    }
    else if (I2Cx == I2C3)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
        RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;

        volatile uint32_t tmp = RCC->AHB1ENR;
        (void)tmp;

        // ==== PA8 = SCL (AF4) ====
        GPIOA->MODER   &= ~(3u << (8 * 2));
        GPIOA->MODER   |=  (2u << (8 * 2));        // AF
        GPIOA->OTYPER  |=  (1u << 8);
        GPIOA->OSPEEDR |=  (3u << (8 * 2));
        GPIOA->PUPDR   &= ~(3u << (8 * 2));
        GPIOA->PUPDR   |=  (1u << (8 * 2));        // PU
        GPIOA->AFR[1]  &= ~(0xFu << ((8 - 8) * 4));
        GPIOA->AFR[1]  |=  (4u  << ((8 - 8) * 4)); // AF4

        // ==== PC9 = SDA (AF4) ====
        GPIOC->MODER   &= ~(3u << (9 * 2));
        GPIOC->MODER   |=  (2u << (9 * 2));        // AF
        GPIOC->OTYPER  |=  (1u << 9);
        GPIOC->OSPEEDR |=  (3u << (9 * 2));
        GPIOC->PUPDR   &= ~(3u << (9 * 2));
        GPIOC->PUPDR   |=  (1u << (9 * 2));
        GPIOC->AFR[1]  &= ~(0xFu << ((9 - 8) * 4));
        GPIOC->AFR[1]  |=  (4u  << ((9 - 8) * 4)); // AF4
    }
    else
    {
        // Unsupported peripheral
        return;
    }

    // ------------------------------
    // 2. Reset and configure I2C peripheral
    // ------------------------------
    I2Cx->CR1 |= I2C_CR1_SWRST;
    for (volatile int i = 0; i < 200; i++);
    I2Cx->CR1 &= ~I2C_CR1_SWRST;

    I2Cx->CR2   = (pclk1 / 1000000U);          // Peripheral clock in MHz
    I2Cx->CCR   = (pclk1 / (speed_hz * 2U));   // Standard mode (100 kHz)
    I2Cx->TRISE = (pclk1 / 1000000U) + 1;      // Rise time

    // Set own address (7-bit mode)
    I2Cx->OAR1 = (1u << 14) | ((uint32_t)(own_address << 1));

    // Enable peripheral and ACK
    I2Cx->CR1 |= (I2C_CR1_PE | I2C_CR1_ACK);
}

