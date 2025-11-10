#include "i2c_test.h"
#include "uart.h"
#include "system_clock.h"

int main(void)
{
    SystemClock_Config();
    UART2_Init(45000000u, 115200u);

    UART_Write(USART2, "Running I2C internal loopback test...\r\n");
    I2C_Internal_Test_All(2); //I2C1 as Master and I2C-2,3 as Slave
    

    while (1);
}