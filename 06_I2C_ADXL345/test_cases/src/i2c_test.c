#include "stm32f4xx.h"
#include "system_clock.h"
#include "i2c.h"
#include "i2c_test.h"
#include "uart.h"
#include <stddef.h>


static const uint8_t slaveA_msg[] = {'J','A','I','S','A','I'};
static const uint8_t slaveB_msg[] = {'S','T','M','3','2','F','4'};
static uint8_t rxbuf[16] = {0};


// ----------------------------------------------------------
// Unified Internal I2C Test
// ----------------------------------------------------------
void I2C_Internal_Test_All(uint8_t master_id)
{
    I2C_TypeDef *I2Cx_Master = NULL;
    I2C_TypeDef *I2Cx_SlaveA = NULL;
    I2C_TypeDef *I2Cx_SlaveB = NULL;
    uint8_t slaveA_addr = 0, slaveB_addr = 0;
    const uint8_t *msgA = slaveA_msg;
    const uint8_t *msgB = slaveB_msg;
    uint8_t lenA = sizeof(slaveA_msg);
    uint8_t lenB = sizeof(slaveB_msg);

    UART_Write(USART2, "\r\n=== Running I2C Internal Loopback Test ===\r\n");
    I2C_Bus_Recovery();

    // ------------------------------------------------------
    // 1. Configure roles dynamically
    // ------------------------------------------------------
    switch (master_id)
    {
        case 1:
            UART_Write(USART2, "\r\nConfig: I2C1 -> Master, I2C2/I2C3 -> Slaves\r\n");
            I2Cx_Master = I2C1;
            I2Cx_SlaveA = I2C2;  slaveA_addr = I2C2_SLAVE_ADDR;
            I2Cx_SlaveB = I2C3;  slaveB_addr = I2C3_SLAVE_ADDR;
            break;

        case 2:
            UART_Write(USART2, "\r\nConfig: I2C2 -> Master, I2C1/I2C3 -> Slaves\r\n");
            I2Cx_Master = I2C2;
            I2Cx_SlaveA = I2C1;  slaveA_addr = I2C1_SLAVE_ADDR;
            I2Cx_SlaveB = I2C3;  slaveB_addr = I2C3_SLAVE_ADDR;
            break;

        case 3:
            UART_Write(USART2, "\r\nConfig: I2C3 -> Master, I2C1/I2C2 -> Slaves\r\n");
            I2Cx_Master = I2C3;
            I2Cx_SlaveA = I2C1;  slaveA_addr = I2C1_SLAVE_ADDR;
            I2Cx_SlaveB = I2C2;  slaveB_addr = I2C2_SLAVE_ADDR;
            break;

        default:
            UART_Write(USART2, "Invalid master ID! Use 1, 2, or 3.\r\n");
            return;
    }

    // ------------------------------------------------------
    // 2. Initialize slaves first, then master
    // ------------------------------------------------------
    I2C_Slave_Init(I2Cx_SlaveA, I2C_TEST_APB1_CLK_HZ, slaveA_addr, I2C_TEST_SPEED_HZ);
    I2C_Slave_Init(I2Cx_SlaveB, I2C_TEST_APB1_CLK_HZ, slaveB_addr, I2C_TEST_SPEED_HZ);
    I2C_Master_Init(I2Cx_Master, I2C_TEST_APB1_CLK_HZ, I2C_TEST_SPEED_HZ);

    for (volatile int d = 0; d < 20000; d++);

    // ------------------------------------------------------
    // 3. Test 1 – Master -> Slave A
    // ------------------------------------------------------
    UART_Write(USART2, "\r\n[TEST] Master -> Slave A\r\n");

    if (I2C_Master_Address(I2Cx_Master, slaveA_addr, I2C_WRITE) != 0)
    {
        UART_Write(USART2, "No ACK from Slave A!\r\n");
        return;
    }

    (void)I2Cx_SlaveA->SR1; (void)I2Cx_SlaveA->SR2; // Clear slave ADDR

    for (uint8_t i = 0; i < lenA; i++)
    {
        I2C_Master_Write(I2Cx_Master, msgA[i]);
        while (!(I2Cx_SlaveA->SR1 & I2C_SR1_RXNE));
        rxbuf[i] = I2Cx_SlaveA->DR;
    }

    I2C_Master_Stop(I2Cx_Master);
    UART_Write(USART2, "Slave A received: ");
    for (uint8_t i = 0; i < lenA; i++) UART_PutChar(USART2, rxbuf[i]);
    UART_Write(USART2, "\r\n");

    // ------------------------------------------------------
    // 4. Test 2 – Master -> Slave B
    // ------------------------------------------------------
    UART_Write(USART2, "\r\n[TEST] Master -> Slave B\r\n");

    // Clear buffer from last test
    for (uint8_t i = 0; i < 16; i++) rxbuf[i] = 0;

    if (I2C_Master_Address(I2Cx_Master, slaveB_addr, I2C_WRITE) != 0)
    {
        UART_Write(USART2, "No ACK from Slave B!\r\n");
        return;
    }

    (void)I2Cx_SlaveB->SR1; (void)I2Cx_SlaveB->SR2; // Clear slave ADDR

    for (uint8_t i = 0; i < lenB; i++)
    {
        I2C_Master_Write(I2Cx_Master, msgB[i]);
        while (!(I2Cx_SlaveB->SR1 & I2C_SR1_RXNE));
        rxbuf[i] = I2Cx_SlaveB->DR;
    }

    I2C_Master_Stop(I2Cx_Master);
    UART_Write(USART2, "Slave B received: ");
    for (uint8_t i = 0; i < lenB; i++) UART_PutChar(USART2, rxbuf[i]);
    UART_Write(USART2, "\r\n");

    // Add a small delay for bus to settle
    for (volatile int d = 0; d < 20000; d++);

    // ------------------------------------------------------
    // 5. Test 3 – Slave A -> Master
    // ------------------------------------------------------
    UART_Write(USART2, "\r\n[TEST] Slave A -> Master\r\n");
    for (uint8_t i = 0; i < 16; i++) rxbuf[i] = 0; // Clear buffer

    // Master: Send address with READ bit
    if (I2C_Master_Address(I2Cx_Master, slaveA_addr, I2C_READ) != 0)
    {
        UART_Write(USART2, "No ACK from Slave A (on read)!\r\n");
        return;
    }

    // Master: Clear its own ADDR flag and enable ACKing
    (void)I2Cx_Master->SR1; (void)I2Cx_Master->SR2;
    I2Cx_Master->CR1 |= I2C_CR1_ACK;

    // Slave: Wait for and clear its ADDR flag
    while (!(I2Cx_SlaveA->SR1 & I2C_SR1_ADDR));
    (void)I2Cx_SlaveA->SR1; (void)I2Cx_SlaveA->SR2;

    // --- Coordinated Read/Write Loop ---
    for (uint8_t i = 0; i < lenA; i++)
    {
        // Slave: Wait for TXE and write data to DR
        while (!(I2Cx_SlaveA->SR1 & I2C_SR1_TXE));
        I2Cx_SlaveA->DR = msgA[i];

        // Master: Check if this is the last byte
        if (i == (lenA - 1))
        {
            // Master: Clear ACK bit (to NACK)
            I2Cx_Master->CR1 &= ~I2C_CR1_ACK;
            // Master: Send STOP condition
            I2C_Master_Stop(I2Cx_Master);
        }

        // Master: Wait for RXNE and read data from DR
        while (!(I2Cx_Master->SR1 & I2C_SR1_RXNE));
        rxbuf[i] = I2Cx_Master->DR;
    }

    // Slave: Clear the Acknowledge Failure (AF) flag caused by NACK
    I2Cx_SlaveA->SR1 &= ~I2C_SR1_AF;

    UART_Write(USART2, "Master received from A: ");
    for (uint8_t i = 0; i < lenA; i++) UART_PutChar(USART2, rxbuf[i]);
    UART_Write(USART2, "\r\n");

    // Add a small delay for bus to settle
    for (volatile int d = 0; d < 20000; d++);

    // ------------------------------------------------------
    // 6. Test 4 – Slave B -> Master
    // ------------------------------------------------------
    UART_Write(USART2, "\r\n[TEST] Slave B -> Master\r\n");
    for (uint8_t i = 0; i < 16; i++) rxbuf[i] = 0; // Clear buffer

    // Master: Send address with READ bit
    if (I2C_Master_Address(I2Cx_Master, slaveB_addr, I2C_READ) != 0)
    {
        UART_Write(USART2, "No ACK from Slave B (on read)!\r\n");
        return;
    }

    // Master: Clear its own ADDR flag and enable ACKing
    (void)I2Cx_Master->SR1; (void)I2Cx_Master->SR2;
    I2Cx_Master->CR1 |= I2C_CR1_ACK;

    // Slave: Wait for and clear its ADDR flag
    while (!(I2Cx_SlaveB->SR1 & I2C_SR1_ADDR));
    (void)I2Cx_SlaveB->SR1; (void)I2Cx_SlaveB->SR2;

    // --- Coordinated Read/Write Loop ---
    for (uint8_t i = 0; i < lenB; i++)
    {
        // Slave: Wait for TXE and write data to DR
        while (!(I2Cx_SlaveB->SR1 & I2C_SR1_TXE));
        I2Cx_SlaveB->DR = msgB[i];

        // Master: Check if this is the last byte
        if (i == (lenB - 1))
        {
            // Master: Clear ACK bit (to NACK)
            I2Cx_Master->CR1 &= ~I2C_CR1_ACK;
            // Master: Send STOP condition
            I2C_Master_Stop(I2Cx_Master);
        }

        // Master: Wait for RXNE and read data from DR
        while (!(I2Cx_Master->SR1 & I2C_SR1_RXNE));
        rxbuf[i] = I2Cx_Master->DR;
    }

    // Slave: Clear the Acknowledge Failure (AF) flag caused by NACK
    I2Cx_SlaveB->SR1 &= ~I2C_SR1_AF;

    UART_Write(USART2, "Master received from B: ");
    for (uint8_t i = 0; i < lenB; i++) UART_PutChar(USART2, rxbuf[i]);
    UART_Write(USART2, "\r\n");

    // ------------------------------------------------------
    // 7. Done
    // ------------------------------------------------------
    UART_Write(USART2, "\r\n=== All I2C Tests Completed Successfully ===\r\n");
}