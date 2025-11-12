#include "uart.h"
#include "system_clock.h"
#include "systick.h"
#include "i2c.h"         // Your I2C driver
#include "ADXL345.h" // The new driver
#define APB2_PCLK1 45000000u
#define APB2_PCLK2 45000000u

//PIN Configurations for ADLX345
//SCL -> PB6
//SDA -> PB7
//VCC -> 3.3V
//GND -> GND
//CS -> 3.3V (I2C Mode)
//SDO -> GND (I2C Address = 0x53)
int main(void)
{
    int16_t accel[3]; // Array to hold [X, Y, Z]

    SystemClock_Config();
    UART_Init(USART2,APB2_PCLK1, 115200u);
    I2C_Master_Init(I2C1, APB2_PCLK1, 100000u); // Use I2C1



    UART_Write(USART2, "\r\n=== ADXL345 Test ===\r\n"); 

    if (ADXL345_Test_Connection(I2C1) != 0) {
        UART_Write(USART2, "Halting system.\r\n");
        while(1); // Stop here if sensor not found
    }

    ADXL345_Init(I2C1); 
    
    sysTickDelay(100); // Wait for sensor to stabilize

    while (1) {
        ADXL345_Read_Accel(I2C1, accel);

        UART_Write(USART2, "X: ");
        UART_WriteInt(USART2, accel[0]);
        UART_Write(USART2, "\t Y: ");
        UART_WriteInt(USART2, accel[1]);
        UART_Write(USART2, "\t Z: ");
        UART_WriteInt(USART2, accel[2]);
        UART_Write(USART2, "\r\n");

        sysTickDelay(500);
    }
}