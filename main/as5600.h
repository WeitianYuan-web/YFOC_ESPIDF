/*
 * AS5600 磁编码器驱动
 * 使用标准浮点数计算
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c.h"

// AS5600常量定义
#define AS5600_ADDR         0x36    // AS5600 I2C地址
#define AS5600_ANGLE_REG    0x0C    // 角度寄存器
#define AS5600_STATUS_REG   0x0B    // 状态寄存器
#define AS5600_AGC_REG      0x1A    // 自动增益控制寄存器
#define AS5600_MAGNITUDE_REG 0x1B   // 磁场强度寄存器

#define AS5600_MAX_RAW_ANGLE 4095   // 12位编码器的最大原始值

// AS5600数据结构
typedef struct {
    uint16_t raw_angle;             // 原始角度值 (0-4095)
    uint16_t angle_prev;            // 上一次的角度值
    int32_t full_rotations;         // 完整旋转圈数
    int32_t total_angle_raw;        // 总角度原始值
    float rotor_phy_angle;          // 物理角度(弧度) - 范围0-2π
    float position_rad;             // 位置(弧度) - 多圈可能很大
    float rotor_zero_angle_rad;     // 零点角度(弧度) - 范围0-2π
    float rotor_phy_angle_rad;      // 物理角度(弧度) - 范围0-2π
    float velocity_rad_per_sec;     // 速度(弧度/秒)
    float velocity_rpm;             // 速度(RPM) - 可能达到数千
    float velocity_filtered;        // 滤波后的速度
    float dt;                       // 采样周期
    uint8_t initialized;            // 初始化标志
} as5600_t;

/**
 * @brief 初始化AS5600编码器和I2C
 * 
 * @param sda_pin SDA引脚
 * @param scl_pin SCL引脚
 * @param i2c_freq I2C频率
 * @param i2c_port I2C端口号
 * @return esp_err_t ESP_OK: 成功, 其他: 失败
 */
esp_err_t as5600_init(int sda_pin, int scl_pin, uint32_t i2c_freq, i2c_port_t i2c_port);

/**
 * @brief 设置AS5600采样周期
 * 
 * @param dt_sec 采样周期(秒)
 */
void as5600_set_dt(float dt_sec);

/**
 * @brief 设置零点角度
 * 
 * @param zero_angle_rad 零点角度(弧度)
 */
void as5600_set_zero_angle(float zero_angle_rad);

/**
 * @brief 读取并处理编码器数据
 * 
 * @return esp_err_t ESP_OK: 成功, 其他: 失败
 */
esp_err_t as5600_update(void);

/**
 * @brief 获取原始角度值
 * 
 * @return uint16_t 原始角度值(0-4095)
 */
uint16_t as5600_get_raw_angle(void);

/**
 * @brief 获取物理角度
 * 
 * @return float 物理角度(弧度)
 */
float as5600_get_angle(void);

/**
 * @brief 获取电角度
 * 
 * @param pole_pairs 电机极对数
 * @return float 电角度(弧度)
 */
float as5600_get_electrical_angle(int pole_pairs);

/**
 * @brief 获取转速
 * 
 * @return float 转速(RPM)
 */
float as5600_get_speed(void); 

/**
 * @brief 获取总角度原始值
 * 
 * @return int32_t 总角度原始值
 */
int32_t as5600_get_total_angle_raw(void);

/**
 * @brief 获取完整旋转圈数
 * 
 * @return int32_t 完整旋转圈数
 */
int32_t as5600_get_full_rotations(void);

/**
 * @brief 获取位置
 * 
 * @return float 位置(弧度)
 */
float as5600_get_position(void);

