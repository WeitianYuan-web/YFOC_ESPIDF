# YFOC_ESP - ESP32 FOC Motor Control System
# YFOC_ESP - ESP32磁场定向控制电机控制系统

[English](#english) | [中文](#中文)

---

## English

### Overview

YFOC_ESP is a comprehensive Field-Oriented Control (FOC) motor control system implemented on the ESP32 platform. It provides both open-loop and closed-loop control modes for brushless DC (BLDC) motors with high precision and performance.

### Features

- **Dual Control Modes**:
  - Open-loop control for simple applications
  - Closed-loop control with multiple feedback modes (Torque, Velocity, Position)
  
- **Advanced Control Algorithms**:
  - Clarke and Park transformations
  - Space Vector Pulse Width Modulation (SVPWM)
  - PID controllers for current, velocity, and position loops
  
- **Sensor Integration**:
  - AS5600 magnetic encoder for position feedback
  - Current sensing with ADC for real-time monitoring
  
- **Communication Interface**:
  - UART command interface for real-time parameter adjustment
  - Data streaming for monitoring and debugging

### Hardware Requirements

- **MCU**: ESP32-S3 (recommended) or compatible ESP32 variants
- **Motor Driver**: 3-phase MOSFET bridge with enable control
- **Position Sensor**: AS5600 magnetic encoder
- **Current Sensors**: Hall effect or shunt resistor based current sensors
- **Motor**: BLDC motor (2208, 2508 gimbal motors tested)

### Pin Configuration

```c
// Motor Driver Control
#define EXAMPLE_FOC_DRV_EN_GPIO          48
#define EXAMPLE_FOC_PWM_U_H_GPIO         17
#define EXAMPLE_FOC_PWM_U_L_GPIO         16
#define EXAMPLE_FOC_PWM_V_H_GPIO         19
#define EXAMPLE_FOC_PWM_V_L_GPIO         18
#define EXAMPLE_FOC_PWM_W_H_GPIO         33
#define EXAMPLE_FOC_PWM_W_L_GPIO         21

// Current Sensing
#define CURRENT_SENSE_U_PIN              3
#define CURRENT_SENSE_V_PIN              4

// AS5600 Encoder
#define AS5600_SDA_PIN                   8
#define AS5600_SCL_PIN                   9

// UART Communication
#define UART_TXD                         43
#define UART_RXD                         44
```

### Getting Started

#### Prerequisites

- ESP-IDF v5.0 or later
- Compatible development board
- Motor and driver hardware setup

#### Building and Flashing

1. Clone the repository:
```bash
git clone <repository_url>
cd YFOC_ESP
```

2. Set up ESP-IDF environment:
```bash
. $IDF_PATH/export.sh
```

3. Configure the project:
```bash
idf.py menuconfig
```

4. Build and flash:
```bash
idf.py build
idf.py flash monitor
```

### Configuration

#### Motor Parameters

Edit `main/motor_config.h` to configure motor parameters:

```c
// Select motor configuration
#define MOTOR_CONFIG_SELECT MOTOR_CONFIG_2

// Motor configurations available:
// MOTOR_CONFIG_1 - 2508 Gimbal Motor
// MOTOR_CONFIG_2 - 2208 Brushless Motor  
// MOTOR_CONFIG_3 - Custom Motor
```

#### Control Mode Selection

In `main/app_main.c`:

```c
// Select FOC control mode
#define FOC_CONTROL_MODE_SELECT FOC_CONTROL_OPENLOOP  // or FOC_CONTROL_CLOSEDLOOP
```

### UART Commands

The system supports real-time parameter adjustment via UART:

- `a<value>`: Set speed (open-loop) or control parameter (closed-loop)
- `b<value>`: Set maximum current limit
- `v<value>`: Set voltage magnitude (open-loop mode)
- `c<value>`: Set velocity parameter (closed-loop mode)
- `d<value>`: Set control mode (0=Torque, 1=Velocity, 2=Position Single, 3=Position Full, 4=Combined)

Example: `a100` sets speed to 100 RPM in open-loop mode.

### Control Modes

#### Open-Loop Control
- Direct voltage/frequency control
- No feedback required
- Suitable for simple applications

#### Closed-Loop Control
1. **Torque Mode**: Direct current control
2. **Velocity Mode**: Speed regulation with PID control
3. **Position Mode**: Precise position control
4. **Combined Mode**: Multi-level control cascade

### Performance Specifications

- **Control Frequency**: Up to 8 kHz (configurable)
- **Position Resolution**: 12-bit (AS5600 encoder)
- **Current Sensing**: Real-time ADC sampling
- **Communication**: 115200 baud UART


### Project Structure

```
YFOC_ESP/
├── main/
│   ├── app_main.c          # Main application
│   ├── esp_foc.c/h         # FOC algorithms implementation
│   ├── foc_control.c/h     # Control loop implementation
│   ├── as5600.c/h          # Encoder driver
│   ├── motor_current_sense.c/h  # Current sensing
│   ├── uart_command.c/h    # UART interface
│   └── motor_config.c/h    # Motor parameters
├── managed_components/
│   └── espressif__iqmath/  # Fixed-point math library
└── img/                    # Documentation images
```

---


### 概述

YFOC_ESP是一个基于ESP32平台实现的综合性磁场定向控制(FOC)电机控制系统。它为无刷直流电机(BLDC)提供开环和闭环控制模式，具有高精度和高性能。

### 功能特点

- **双控制模式**：
  - 开环控制适用于简单应用
  - 闭环控制支持多种反馈模式（转矩、速度、位置）
  
- **先进控制算法**：
  - Clarke和Park变换
  - 空间矢量脉宽调制(SVPWM)
  - 电流、速度和位置环PID控制器
  
- **传感器集成**：
  - AS5600磁编码器位置反馈
  - ADC电流采样实时监测
  
- **通信接口**：
  - UART命令接口实时参数调节
  - 数据流用于监控和调试

### 硬件要求

- **微控制器**: ESP32-S3（推荐）或兼容的ESP32系列
- **电机驱动器**: 带使能控制的三相MOSFET桥
- **位置传感器**: AS5600磁编码器
- **电流传感器**: 基于霍尔效应或分流电阻的电流传感器
- **电机**: 无刷直流电机（已测试2208、2508云台电机）

### 引脚配置

```c
// 电机驱动控制
#define EXAMPLE_FOC_DRV_EN_GPIO          48
#define EXAMPLE_FOC_PWM_U_H_GPIO         17
#define EXAMPLE_FOC_PWM_U_L_GPIO         16
#define EXAMPLE_FOC_PWM_V_H_GPIO         19
#define EXAMPLE_FOC_PWM_V_L_GPIO         18
#define EXAMPLE_FOC_PWM_W_H_GPIO         33
#define EXAMPLE_FOC_PWM_W_L_GPIO         21

// 电流采样
#define CURRENT_SENSE_U_PIN              3
#define CURRENT_SENSE_V_PIN              4

// AS5600编码器
#define AS5600_SDA_PIN                   8
#define AS5600_SCL_PIN                   9

// UART通信
#define UART_TXD                         43
#define UART_RXD                         44
```

### 快速开始

#### 先决条件

- ESP-IDF v5.0或更高版本
- 兼容的开发板
- 电机和驱动器硬件设置

#### 构建和烧录

1. 克隆仓库：
```bash
git clone <repository_url>
cd YFOC_ESP
```

2. 设置ESP-IDF环境：
```bash
. $IDF_PATH/export.sh
```

3. 配置项目：
```bash
idf.py menuconfig
```

4. 构建和烧录：
```bash
idf.py build
idf.py flash monitor
```

### 配置

#### 电机参数

编辑 `main/motor_config.h` 配置电机参数：

```c
// 选择电机配置
#define MOTOR_CONFIG_SELECT MOTOR_CONFIG_2

// 可用的电机配置：
// MOTOR_CONFIG_1 - 2508云台电机
// MOTOR_CONFIG_2 - 2208无刷电机
// MOTOR_CONFIG_3 - 自定义电机
```

#### 控制模式选择

在 `main/app_main.c` 中：

```c
// 选择FOC控制模式
#define FOC_CONTROL_MODE_SELECT FOC_CONTROL_OPENLOOP  // 或 FOC_CONTROL_CLOSEDLOOP
```

### UART命令

系统支持通过UART实时调节参数：

- `a<数值>`: 设置速度（开环）或控制参数（闭环）
- `b<数值>`: 设置最大电流限制
- `v<数值>`: 设置电压幅值（开环模式）
- `c<数值>`: 设置速度参数（闭环模式）
- `d<数值>`: 设置控制模式（0=转矩，1=速度，2=单圈位置，3=多圈位置，4=组合）

例如：`a100` 在开环模式下设置速度为100 RPM。

### 控制模式

#### 开环控制
- 直接电压/频率控制
- 无需反馈
- 适用于简单应用

#### 闭环控制
1. **转矩模式**: 直接电流控制
2. **速度模式**: PID速度调节
3. **位置模式**: 精确位置控制
4. **组合模式**: 多级控制级联

### 性能规格

- **控制频率**: 最高8 kHz（可配置）
- **位置分辨率**: 12位（AS5600编码器）
- **电流采样**: 实时ADC采样
- **通信**: 115200波特率UART

### 测试

运行包含的测试套件：
```bash
pytest pytest_foc_open_loop.py
```

### 项目结构

```
YFOC_ESP/
├── main/
│   ├── app_main.c          # 主应用程序
│   ├── esp_foc.c/h         # FOC算法实现
│   ├── foc_control.c/h     # 控制环路实现
│   ├── as5600.c/h          # 编码器驱动
│   ├── motor_current_sense.c/h  # 电流采样
│   ├── uart_command.c/h    # UART接口
│   └── motor_config.c/h    # 电机参数
├── managed_components/
│   └── espressif__iqmath/  # 定点数学库
└── img/                    # 文档图片
```

