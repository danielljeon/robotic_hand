# robotic_hand

![arm_gcc_build](https://github.com/danielljeon/robotic_hand/actions/workflows/arm_gcc_build.yaml/badge.svg)

Robotic hand project for actuators/power electronics and sensors/instrumentation
university courses (firmware).

robotic_hand_pcb hardware
repo: [robotic_hand_pcb](https://github.com/danielljeon/robotic_hand_pcb).

---

<details markdown="1">
  <summary>Table of Contents</summary>

<!-- TOC -->
* [robotic_hand](#robotic_hand)
  * [1 Overview](#1-overview)
    * [1.1 Bill of Materials (BOM)](#11-bill-of-materials-bom)
    * [1.2 Block Diagram](#12-block-diagram)
    * [1.3 Pin Configurations](#13-pin-configurations)
    * [1.4 Clock Configurations](#14-clock-configurations)
<!-- TOC -->

</details>

---

## 1 Overview

### 1.1 Bill of Materials (BOM)

| Manufacturer Part Number | Manufacturer            | Description                 | Quantity | Notes |
|--------------------------|-------------------------|-----------------------------|---------:|-------|
| STM32F446RE              | STMicroelectronics      | 32-bit MCU                  |        1 |       |
| BNO085                   | CEVA Technologies, Inc. | 9-DOF IMU                   |        1 |       |
| ADS114S08                | Texas Instruments       | ADC IC                      |        1 |       |
| VL53L4CD                 | STMicroelectronics      | Time of Flight (TOF) Sensor |        1 |       |
| WS2812B                  | (Various)               | PWM Addressable RGB LED     |   (Many) |       |

### 1.2 Block Diagram

![robotic_hand.drawio.png](docs/robotic_hand.drawio.png)

> Drawio file here: [robotic_hand.drawio](docs/robotic_hand.drawio)

### 1.3 Pin Configurations

<details markdown="1">
  <summary>CubeMX Pinout</summary>

![CubeMX Pinout.png](docs/CubeMX%20Pinout.png)

</details>

<details markdown="1">
  <summary>Pin & Peripherals Table</summary>

| STM32F446RE | Peripheral              | Config                | Connection                       | Notes                                 |
|-------------|-------------------------|-----------------------|----------------------------------|---------------------------------------|
| PB3         | `SYS_JTDO-SWO`          |                       | TC2050 SWD Pin 6: `SWO`          |                                       |
| PA14        | `SYS_JTCK-SWCLK`        |                       | TC2050 SWD Pin 4: `SWCLK`        |                                       |
| PA13        | `SYS_JTMS-SWDIO`        |                       | TC2050 SWD Pin 2: `SWDIO`        |                                       |
|             | `TIM5_CH1`              | PWM no output         |                                  | BNO085 SH2 driver timer.              |
| PA5         | `SPI1_SCK`              |                       | BNO085 Pin 19: `H_SCL/SCK/RX`    |                                       |
| PB9         | `GPIO_Output` (SPI1 CS) | Pull-up, set high     | BNO085 Pin 18: `H_CSN`           |                                       |
| PA6         | `SPI1_MISO`             |                       | BNO085 Pin 20: `H_SDA/H_MISO/TX` |                                       |
| PA7         | `SPI1_MOSI`             |                       | BNO085 Pin 17: `SA0/H_MOSI`      |                                       |
| PA4         | `GPIO_EXTI4`            | Pull-up, falling edge | BNO085 Pin 14: `H_INTN`          |                                       |
| PC5         | `GPIO_Output`           |                       | BNO085 Pin 6: `PS0/Wake`         | Pull low to trigger wake.             |
|             |                         | Hardware pull-up      | BNO085 Pin 5: `PS1`              |                                       |
| PC4         | `GPIO_Output`           |                       | BNO085 Pin 11: `NRST`            | Pull low to reset.                    |
| PC10        | `SPI3_SCK`              |                       | ADS114S08 Pin 11: `SCLK`         |                                       |
| PA15        | `GPIO_Output` (SPI3 CS) | Pull-up, set high     | ADS114S08 Pin 9: `CS_N`          |                                       |
| PC11        | `SPI3_MISO`             |                       | ADS114S08 Pin 12: `DOUT`         |                                       |
| PC12        | `SPI3_MOSI`             |                       | ADS114S08 Pin 10: `DIN`          |                                       |
| PD2         | `GPIO_EXTI2`            |                       | ADS114S08 Pin 13: `DRDY_N`       |                                       |
| PA10        | `GPIO_Output`           |                       | ADS114S08 Pin 18: `RESET_N`      |                                       |
| PA11        | `GPIO_Output`           |                       | ADS114S08 Pin 8: `START_SYNC`    |                                       |
|             |                         | Hardware pull-down    | ADS114S08 Pin 17: `CLK`          |                                       |
| PB6         | `I2C1_SCL`              |                       | VL53L4CD Pin 10: `SCL`           |                                       |
| PB7         | `I2C1_SDA`              |                       | VL53L4CD Pin 9: `SDA`            |                                       |
| PB4         | `GPIO_Output`           |                       | VL53L4CD Pin 5: `XSHUT`          |                                       |
| PB5         | `GPIO_EXTI5`            |                       | VL53L4CD Pin 7: `GPIO1`          |                                       |
| PA8         | `TIM1_CH1`              | PWM Generation CH1    | WS2812B Pin: `DIN`               | DIN pin number depends on IC variant. |

</details>

### 1.4 Clock Configurations

```
8 MHz High Speed External (HSE)
↓
Phase-Locked Loop Main (PLLM)
↓
180 MHz SYSCLK
↓
180 MHz HCLK
↓
 → 45 MHz APB1 (Maxed) → 90 MHz APB1 Timer
 → 90 MHz APB2 (Maxed) → 180 MHz APB2 Timer
```

---
