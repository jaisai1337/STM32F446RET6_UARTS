# 🕒 DS3231 RTC Interface with STM32F446

This project demonstrates interfacing a **DS3231 Real-Time Clock (RTC)** module with an **STM32F446RE** microcontroller using **bare-metal I²C** (no HAL/LL drivers).  
It includes I²C initialization, DS3231 time/date set and read functions, and formatted UART output.

---

## 🚀 Features
- Bare-metal **I²C Master driver** (works with I2C1 / I2C2 / I2C3)
- DS3231 RTC **read/write** implementation  
- BCD ↔ Decimal conversion  
- Optional **time/date setup** from STM32  
- UART debug output for easy monitoring  
- Compatible with **STM32F4xx** series (tested on F446RE)

---

## 🔌 Hardware Setup

| Connection | Description |
|-------------|-------------|
| **PB6 (I2C1_SCL)** or I2C2/I2C3 SCL pin | Clock line |
| **PB7 (I2C1_SDA)** or I2C2/I2C3 SDA pin | Data line |
| **3.3V / GND** | Power for DS3231 |
| **CR2032 Battery** | Keeps RTC running when MCU is powered off |

Pull-up resistors (4.7 kΩ) between SCL/SDA and 3.3 V are **required**.

---



### Key Files
| File | Description |
|------|-------------|
| `src/main.c` | Main application (I2C config + RTC test loop) |
| `drivers/src/i2c.c` | Custom bare-metal I2C driver |
| `drivers/src/uart.c` | UART2 debug output |
| `sensors/src/DS3231.c` | DS3231 RTC driver implementation |
| `sensors/inc/DS3231.h` | RTC function prototypes and structure definitions |

---

## 🧭 Example Serial Output

```text
=== DS3231 RTC Test ===

Config: I2C1 [Base=0x40005400] -> Master

=== DS3231 Init ===

DS3231 detected at 0x68
Time: 23:45:12  Date: 10/11/2025  DOW: Mon
Time: 23:45:13  Date: 10/11/2025  DOW: Mon
Time: 23:45:14  Date: 10/11/2025  DOW: Mon
