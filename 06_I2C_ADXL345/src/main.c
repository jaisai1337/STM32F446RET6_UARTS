#include "uart.h"
#include "system_clock.h"
#include "systick.h"
#include "i2c.h"         // Your I2C driver
#include "ADXL345_test.h" // The new driver


int main(void)
{
    int16_t accel[3]; // Array to hold [X, Y, Z]

    SystemClock_Config();
    UART2_Init(45000000u, 115200u);
    I2C_Master_Init(I2C1, 45000000u, 100000u); // Use I2C1

    UART_Write(USART2, "\r\n=== ADXL345 Test ===\r\n");

    // 1. Check connection
    if (ADXL345_Test_Connection(I2C1) != 0) {
        UART_Write(USART2, "Halting system.\r\n");
        while(1); // Stop here if sensor not found
    }

    // 2. Initialize
    ADXL345_Init(I2C1);
    
    sysTickDelay(100); // Small delay to let sensor stabilize

    while (1) {
        // 3. Read data
        ADXL345_Read_Accel(I2C1, accel);

        // 4. Print data
        UART_Write(USART2, "X: ");
        UART_WriteInt(USART2, accel[0]);
        UART_Write(USART2, "\t Y: ");
        UART_WriteInt(USART2, accel[1]);
        UART_Write(USART2, "\t Z: ");
        UART_WriteInt(USART2, accel[2]);
        UART_Write(USART2, "\r\n");

        sysTickDelay(500); // Read twice per second
    }
}