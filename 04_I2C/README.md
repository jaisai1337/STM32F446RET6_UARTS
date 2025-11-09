README — STM32F446 Internal I²C Multi-Slave Loopback Test
Overview

This project demonstrates bare-metal I²C communication between three I²C peripherals on a single STM32F446RET6 MCU without using HAL or CubeMX.

It verifies full I²C master–slave operation using register-level code with interrupt-free flag handling.

The test sends data from I2C1 (Master) to two internal slaves:
I2C2 (Slave A) at address 0x3A
I2C3 (Slave B) at address 0x3B
All transfers and acknowledgments happen entirely inside one MCU, with status logged over UART2.

###  MCU Configuration

| Peripheral | Role     | Address | SCL Pin | SDA Pin | AF  | Notes                    |
|-------------|----------|----------|----------|----------|-----|--------------------------|
| **I2C1**    | Master   | —        | PB6      | PB7      | AF4 | Initiates all transfers  |
| **I2C2**    | Slave A  | 0x3A     | PB10     | PC12     | AF4 | Responds to Master       |
| **I2C3**    | Slave B  | 0x3B     | PA8      | PC9      | AF4 | Responds to Master       |
| **UART2**   | Debug log| —        | PA2 (TX) | PA3 (RX) | AF7 | 115200 baud              |
| **System Clock** | —  | —        | —        | —        | —   | 180 MHz core / 45 MHz APB1 |


###  Wiring & Hardware Setup

| Connection | Description |
|-------------|--------------|
| **PB6 ↔ PB10 ↔ PA8** | Common SCL line (shared between I2C1, I2C2, and I2C3) |
| **PB7 ↔ PC12 ↔ PC9** | Common SDA line (shared between I2C1, I2C2, and I2C3) |
| **4.7 kΩ resistor** | Between SCL and 3.3 V (pull-up) |
| **4.7 kΩ resistor** | Between SDA and 3.3 V (pull-up) |
| **Common GND** | All pins share MCU ground |


Use external pull-ups even if internal pull-ups are configured (for stable logic levels).
Ensure idle bus levels measure ≈3.3 V on SCL and SDA before flashing.

###  Firmware Flow Summary

1. **SystemClock_Config()**  Configures the MCU to run at **180 MHz** core clock with **APB1 = 45 MHz**.
2. **UART2_Init()**  Sets up the debug UART on **PA2/PA3** at **115200 bps** for status output.
3. **I2C_Bus_Recovery()** Toggles **PB6 (SCL)** and **PB7 (SDA)** to release any stuck I²C lines before initialization.
4. **I2C2_Slave_Init()** Initializes **Slave A** (address `0x3A`) on **PB10/PC12**.
5. **I2C3_Slave_Init()** Initializes **Slave B** (address `0x3B`) on **PA8/PC9**.
6. **I2C1_Master_Init()**  Configures **I2C1** as the **master** on **PB6/PB7**.
7. **Data Transfer Phase**  
   - Master → Slave A (`0x3A`): sends **"JAISAI"**  
   - Master → Slave B (`0x3B`): sends **"STM32F4"**
8. **Result Logging** UART2 prints transfer progress, received data, and completion status.

### 🧾 Expected UART Output

After flashing the firmware and opening a serial terminal at **115200 bps (8N1)**, you should see output similar to the following:


===I2C1 Master → I2C2 / I2C3 Slaves Test===


[TEST] Master → Slave A (I2C2 @ 0x3A)

Slave A received: JAISAI

[TEST] Master → Slave B (I2C3 @ 0x3B)

Slave B received: STM32F4

===All I2C transfers completed successfully===


