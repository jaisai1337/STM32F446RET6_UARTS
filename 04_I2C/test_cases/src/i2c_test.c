#include "stm32f4xx.h"
#include "system_clock.h"
#include "i2c.h"
#include "i2c_test.h"
#include "uart.h"


static const uint8_t slaveA_msg[] = {'J','A','I','S','A','I'};
static const uint8_t slaveB_msg[] = {'S','T','M','3','2','F','4'};
static uint8_t rxbuf[16] = {0};



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

// ============================================================
//  Internal Master-Slave I2C Test
// ============================================================
void I2C_Internal_Test_All(void)
{
    UART_Write(USART2, "\r\n=== I2C1 Master -> I2C2/I2C3 Slaves Test ===\r\n");

    I2C_Bus_Recovery();

    I2C2_Slave_Init(I2C_TEST_APB1_CLK_HZ, I2C2_SLAVE_ADDR, I2C_TEST_SPEED_HZ);
    I2C3_Slave_Init(I2C_TEST_APB1_CLK_HZ, I2C3_SLAVE_ADDR, I2C_TEST_SPEED_HZ);
    I2C1_Master_Init(I2C_TEST_APB1_CLK_HZ, I2C_TEST_SPEED_HZ);

    // ---------------------- Test 1: Slave A ----------------------
    UART_Write(USART2, "\r\n[TEST] Master -> Slave A (I2C2 @ 0x3A)\r\n");
    I2C1_Master_Address(I2C2_SLAVE_ADDR, I2C_WRITE);
    (void)I2C2->SR1; (void)I2C2->SR2;

    for (uint8_t i=0; i<sizeof(slaveA_msg); i++) {
        I2C1_Master_Write(slaveA_msg[i]);
        while (!(I2C2->SR1 & I2C_SR1_RXNE));
        rxbuf[i] = I2C2->DR;
    }
    I2C1_Master_Stop();
    UART_Write(USART2, "SlaveA received: ");
    for (uint8_t i=0; i<sizeof(slaveA_msg); i++) UART_PutChar(USART2, rxbuf[i]);
    UART_Write(USART2, "\r\n");

    // ---------------------- Test 2: Slave B ----------------------
    UART_Write(USART2, "\r\n[TEST] Master -> Slave B (I2C3 @ 0x3B)\r\n");
    I2C1_Master_Address(I2C3_SLAVE_ADDR, I2C_WRITE);
    (void)I2C3->SR1; (void)I2C3->SR2;

    for (uint8_t i=0; i<sizeof(slaveB_msg); i++) {
        I2C1_Master_Write(slaveB_msg[i]);
        while (!(I2C3->SR1 & I2C_SR1_RXNE));
        rxbuf[i] = I2C3->DR;
    }
    I2C1_Master_Stop();
    UART_Write(USART2, "SlaveB received: ");
    for (uint8_t i=0; i<sizeof(slaveB_msg); i++) UART_PutChar(USART2, rxbuf[i]);
    UART_Write(USART2, "\r\n");

    UART_Write(USART2, "\r\n=== All I2C Tests Completed Successfully ===\r\n");
}