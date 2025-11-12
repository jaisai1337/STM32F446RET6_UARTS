# ADXL345 Bare-Metal Driver for STM32 (I2C)

A lightweight, fully bare-metal ADXL345 accelerometer driver for STM32 microcontrollers.  
This driver uses direct register access (no HAL/LL) and includes a clean I²C master implementation supporting robust multi-byte reads.

---

## ✨ Features

- Full bare-metal STM32 I2C communication  
- Multi-byte read logic (N = 1, 2, >2) compliant with STM32 RM0090  
- ±16g full-resolution mode (4 mg/LSB)  
- Read raw X/Y/Z acceleration  
- Optional conversion to g-units  
- Clean modular architecture  
- No dependencies on HAL/LL libraries  

---


---

## 🔌 Wiring (I2C)

| ADXL345 | STM32 (Example I2C1) |
|--------|------------------------|
| VCC    | 3.3V |
| GND    | GND  |
| SDA    | PB7 (I2C1_SDA) |
| SCL    | PB6 (I2C1_SCL) |
| CS     | 3.3V (Force I²C mode) |
| SD0    | GND → I2C Address = 0x53 |

---


