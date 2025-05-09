/*
 * 电机参数配置文件
 * 用于存储不同电机的参数配置
 */
#include "motor_config.h"

// 电机配置1 - 2508云台电机
static const motor_config_t motor_config_1 = {
    .name = "2508云台电机",
    .pole_pairs = 7,            // 电机极对数
    .resistance = 9.2f,         // 电机相电阻(欧姆)
    .supply_voltage = 12.0f,    // 电机供电电压(伏特)
    .kv = 180.0f,               // 电机KV值(RPM/V)
    .direction = 1,            // 电机方向
    .current_limit = 0.9f,      // 电流限制(安培)
    .max_voltage = 0.9f,        // 最大电压，对应PWM占空比的倍数 (0.0-1.0)
    .openloop_rpm = 400         // 开环控制转速 (RPM)
};

// 电机配置2 - 2208无刷电机
static const motor_config_t motor_config_2 = {
    .name = "2208无刷电机",
    .pole_pairs = 7,           // 电机极对数
    .resistance = 0.1f,         // 电机相电阻(欧姆)
    .supply_voltage = 12.0f,    // 电机供电电压(伏特)
    .kv = 2300.0f,               // 电机KV值(RPM/V)
    .direction = 1,             // 电机方向
    .current_limit = 5.0f,      // 电流限制(安培)
    .max_voltage = 0.3f,        // 最大电压，对应PWM占空比的倍数 (0.0-1.0)
    .openloop_rpm = 500         // 开环控制转速 (RPM)
};

// 电机配置3 - 精灵无刷电机
static const motor_config_t motor_config_3 = {
    .name = "精灵无刷电机",
    .pole_pairs = 7,           // 电机极对数
    .resistance = 0.3f,         // 电机相电阻(欧姆)
    .supply_voltage = 12.0f,    // 电机供电电压(伏特)
    .kv = 1000.0f,               // 电机KV值(RPM/V)
    .direction = -1,             // 电机方向
    .current_limit = 5.0f,      // 电流限制(安培)
    .max_voltage = 0.2f,        // 最大电压，对应PWM占空比的倍数 (0.0-1.0)
    .openloop_rpm = 1000         // 开环控制转速 (RPM)
};

// 获取当前选择的电机配置
const motor_config_t* get_motor_config(void)
{
    switch (MOTOR_CONFIG_SELECT) {
        case MOTOR_CONFIG_1:
            return &motor_config_1;
        case MOTOR_CONFIG_2:
            return &motor_config_2;
        case MOTOR_CONFIG_3:
            return &motor_config_3;
        default:
            return &motor_config_1; // 默认返回配置1
    }
} 