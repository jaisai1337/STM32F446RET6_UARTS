#include "i2c.h"
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


// ====================  I2C1 MASTER  ====================

void I2C1_Master_Init(uint32_t pclk1, uint32_t speed_hz)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // PB6=SCL, PB7=SDA (AF4, OD, PU)
    GPIOB->MODER   &= ~((3u<<(6*2)) | (3u<<(7*2)));
    GPIOB->MODER   |=  ((2u<<(6*2)) | (2u<<(7*2)));
    GPIOB->AFR[0]  &= ~((0xFu<<(6*4)) | (0xFu<<(7*4)));
    GPIOB->AFR[0]  |=  ((4u<<(6*4)) | (4u<<(7*4)));
    GPIOB->OTYPER  |=  ((1u<<6) | (1u<<7));
    GPIOB->OSPEEDR |=  ((3u<<(6*2)) | (3u<<(7*2)));
    GPIOB->PUPDR   &= ~((3u<<(6*2)) | (3u<<(7*2)));
    GPIOB->PUPDR   |=  ((1u<<(6*2)) | (1u<<(7*2)));   // weak PU

    // Reset
    I2C1->CR1 |= I2C_CR1_SWRST;
    for (volatile int i=0;i<200;i++);
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    // 
    I2C1->CR1 = 0;

    I2C1->CR2   = (pclk1 / 1000000U);
    I2C1->CCR   = (pclk1 / (speed_hz * 2U));
    I2C1->TRISE = (pclk1 / 1000000U) + 1;

    I2C1->CR1 |= I2C_CR1_PE;
    for (volatile int d=0; d<10000; d++); // small delay
}

int I2C1_Master_Address(uint8_t addr, uint8_t direction)
{
    // **** THIS IS THE FIX (Wait for bus to be free) ****
    while (I2C1->SR2 & I2C_SR2_BUSY);

    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));

    I2C1->DR = (addr << 1) | direction;

    while (1) {
        if (I2C1->SR1 & I2C_SR1_ADDR) {
            (void)I2C1->SR2;
            return 0;                       // ACKed
        }
        if (I2C1->SR1 & I2C_SR1_AF) {       // NACK
            I2C1->SR1 &= ~I2C_SR1_AF;
            I2C1->CR1 |= I2C_CR1_STOP;
            return -1;
        }
    }
}

void I2C1_Master_Write(uint8_t data)
{
    while (!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = data;
    // while (!(I2C1->SR1 & I2C_SR1_BTF));
}

void I2C1_Master_Stop(void)
{
    I2C1->CR1 |= I2C_CR1_STOP;
}


// ====================  I2C2 SLAVE  ====================

void I2C2_Slave_Init(uint32_t pclk1, uint8_t own_address, uint32_t speed_hz)
{
    // === Enable all required clocks first ===
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

    // Dummy read to ensure clock domain update before GPIO access
    volatile uint32_t tmp = RCC->AHB1ENR;
    (void)tmp;

    // ==== PB10 = I2C2_SCL (AF4) ====
    GPIOB->MODER   &= ~(3u << (10 * 2));
    GPIOB->MODER   |=  (2u << (10 * 2));         // AF mode
    GPIOB->OTYPER  |=  (1u << 10);               // open-drain
    GPIOB->OSPEEDR |=  (3u << (10 * 2));         // high speed
    GPIOB->PUPDR   &= ~(3u << (10 * 2));
    GPIOB->PUPDR   |=  (1u << (10 * 2));         // pull-up
    GPIOB->AFR[1]  &= ~(0xFu << ((10 - 8) * 4));
    GPIOB->AFR[1]  |=  (4u  << ((10 - 8) * 4));  // AF4 (I2C2_SCL)

    // ==== PC12 = I2C2_SDA (AF4) ====
    GPIOC->MODER   &= ~(3u << (12 * 2));
    GPIOC->MODER   |=  (2u << (12 * 2));         // AF mode
    GPIOC->OTYPER  |=  (1u << 12);               // open-drain
    GPIOC->OSPEEDR |=  (3u << (12 * 2));         // high speed
    GPIOC->PUPDR   &= ~(3u << (12 * 2));
    GPIOC->PUPDR   |=  (1u << (12 * 2));         // pull-up
    
    // **** THIS IS THE FIX (was AFR[3]) ****
    GPIOC->AFR[1]  &= ~(0xFu << ((12 - 8) * 4));
    GPIOC->AFR[1]  |=  (4u  << ((12 - 8) * 4));  // AF4 (I2C2_SDA)

    // Reset peripheral
    I2C2->CR1 |= I2C_CR1_SWRST;
    for (volatile int i = 0; i < 200; i++);
    I2C2->CR1 &= ~I2C_CR1_SWRST;

    // Configure timing
    I2C2->CR2   = (pclk1 / 1000000U);
    I2C2->CCR   = (pclk1 / (speed_hz * 2U));
    I2C2->TRISE = (pclk1 / 1000000U) + 1;

    // Own address
    I2C2->OAR1  = (1u<<14) | ((uint32_t)(own_address << 1));

    // Enable ACK + Peripheral
    I2C2->CR1  |= I2C_CR1_ACK | I2C_CR1_PE;
}

uint8_t I2C2_Slave_Read(uint8_t *buf, uint8_t len)
{
    uint8_t i = 0;

    while (!(I2C2->SR1 & I2C_SR1_ADDR));
    (void)I2C2->SR1; (void)I2C2->SR2;

    while (i < len)
    {
        while (!(I2C2->SR1 & I2C_SR1_RXNE)) {
            if (I2C2->SR1 & I2C_SR1_STOPF) {
                volatile uint32_t tmp = I2C2->SR1; (void)tmp;
                I2C2->CR1 |= I2C_CR1_PE;
                return i;
            }
        }
        buf[i++] = I2C2->DR;

        if (I2C2->SR1 & I2C_SR1_STOPF) {
            volatile uint32_t tmp = I2C2->SR1; (void)tmp;
            I2C2->CR1 |= I2C_CR1_PE;
            break;
        }
    }
    return i;
}
// I2C Slave Initialization

void I2C3_Slave_Init(uint32_t pclk1, uint8_t own_address, uint32_t speed_hz)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;

    volatile uint32_t tmp = RCC->AHB1ENR; (void)tmp;

    // PA8 = SCL (AF4)
    GPIOA->MODER   &= ~(3u << (8 * 2));
    GPIOA->MODER   |=  (2u << (8 * 2));
    GPIOA->OTYPER  |=  (1u << 8);
    GPIOA->OSPEEDR |=  (3u << (8 * 2));
    GPIOA->PUPDR   &= ~(3u << (8 * 2));
    GPIOA->PUPDR   |=  (1u << (8 * 2));
    GPIOA->AFR[1]  &= ~(0xFu << ((8 - 8) * 4));
    GPIOA->AFR[1]  |=  (4u  << ((8 - 8) * 4));

    // PC9 = SDA (AF4)
    GPIOC->MODER   &= ~(3u << (9 * 2));
    GPIOC->MODER   |=  (2u << (9 * 2));
    GPIOC->OTYPER  |=  (1u << 9);
    GPIOC->OSPEEDR |=  (3u << (9 * 2));
    GPIOC->PUPDR   &= ~(3u << (9 * 2));
    GPIOC->PUPDR   |=  (1u << (9 * 2));
    GPIOC->AFR[1]  &= ~(0xFu << ((9 - 8) * 4));
    GPIOC->AFR[1]  |=  (4u  << ((9 - 8) * 4));

    // Reset and enable I2C3
    I2C3->CR1 |= I2C_CR1_SWRST;
    for (volatile int i = 0; i < 200; i++);
    I2C3->CR1 &= ~I2C_CR1_SWRST;

    I2C3->CR2   = (pclk1 / 1000000U);
    I2C3->CCR   = (pclk1 / (speed_hz * 2U));
    I2C3->TRISE = (pclk1 / 1000000U) + 1;
    I2C3->OAR1  = (1u<<14) | ((uint32_t)(own_address << 1));
    I2C3->CR1  |= I2C_CR1_ACK | I2C_CR1_PE;
}
