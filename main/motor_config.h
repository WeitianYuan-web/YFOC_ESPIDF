/*
 * 电机参数配置文件
 * 用于存储不同电机的参数配置
 */
#pragma once

#include <stdint.h>

// 选择要使用的电机配置
#define MOTOR_CONFIG_SELECT MOTOR_CONFIG_2

// 电机配置1 - 2508云台电机
#define MOTOR_CONFIG_1 1
// 电机配置2 - 2208无刷电机
#define MOTOR_CONFIG_2 2
// 电机配置3 - 精灵无刷电机
#define MOTOR_CONFIG_3 3


// 电机参数结构体
typedef struct {
    const char* name;           // 电机名称
    uint8_t pole_pairs;         // 电机极对数
    float resistance;           // 电机相电阻(欧姆)
    float supply_voltage;       // 电机供电电压(伏特)
    float kv;                   // 电机KV值(RPM/V)
    int8_t direction;           // 电机方向
    float current_limit;        // 电流限制(安培)
    float max_voltage;          // 最大电压，对应PWM占空比的倍数 (0.0-1.0)
    uint16_t openloop_rpm;      // 开环控制转速 (RPM)
} motor_config_t;

// 获取当前选择的电机配置
const motor_config_t* get_motor_config(void); 