README — STM32F446 Internal I²C Multi-Slave Loopback Test
Overview

This project demonstrates bare-metal I²C communication between three I²C peripherals on a single STM32F446RET6 MCU without using HAL or CubeMX.

It verifies full I²C master–slave operation using register-level code with interrupt-free flag handling.

The test sends data from I2C1 (Master) to two internal slaves:
I2C2 (Slave A) at address 0x3A
I2C3 (Slave B) at address 0x3B
All transfers and acknowledgments happen entirely inside one MCU, with status logged over UART2.

### 🧩 MCU Configuration

| Peripheral | Role     | Address | SCL Pin | SDA Pin | AF  | Notes                    |
|-------------|----------|----------|----------|----------|-----|--------------------------|
| **I2C1**    | Master   | —        | PB6      | PB7      | AF4 | Initiates all transfers  |
| **I2C2**    | Slave A  | 0x3A     | PB10     | PC12     | AF4 | Responds to Master       |
| **I2C3**    | Slave B  | 0x3B     | PA8      | PC9      | AF4 | Responds to Master       |
| **UART2**   | Debug log| —        | PA2 (TX) | PA3 (RX) | AF7 | 115200 baud              |
| **System Clock** | —  | —        | —        | —        | —   | 180 MHz core / 45 MHz APB1 |


Wiring & Hardware Setup:
Connection	        Description
PB6 - PB10 - PA8	Common SCL line
PB7 - PC12 - PC9	Common SDA line
4.7 kΩ resistor	    Between SCL and 3.3 V
4.7 kΩ resistor	    Between SDA and 3.3 V
Common GND	        All pins share MCU ground

Use external pull-ups even if internal pull-ups are configured (for stable logic levels).
Ensure idle bus levels measure ≈3.3 V on SCL and SDA before flashing.

Firmware Flow Summary:

SystemClock_Config() — sets up 180 MHz system clock, APB1 = 45 MHz.
UART2_Init() — initializes debug serial (115200 bps).
I2C_Bus_Recovery() — toggles PB6/PB7 to free any stuck SDA/SCL lines.
I2C2_Slave_Init() — configures Slave A (PB10/PC12).
I2C3_Slave_Init() — configures Slave B (PA8/PC9).
I2C1_Master_Init() — initializes Master on PB6/PB7.
Master → Slave A (0x3A) — sends "JAISAI".
Master → Slave B (0x3B) — sends "STM32F4".
UART prints confirmation for both.