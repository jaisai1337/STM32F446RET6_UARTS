#include "i2c_test.h"
#include "uart.h"
#include "system_clock.h"
#define PCLK1_FREQ 45000000u
#define PCLK2_FREQ 90000000u
int main(void)
{
    SystemClock_Config();
    UART_Init(USART2, PCLK1_FREQ, 115200u);

    UART_Write(USART2, "Running I2C internal loopback test...\r\n");
    I2C_Internal_Test_All(3); //I2C1 as Master and I2C-2,3 as Slave
    

    while (1);
}