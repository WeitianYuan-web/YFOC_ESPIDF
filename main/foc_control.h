/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <math.h>
#include "esp_err.h"
#include "esp_foc.h"
#include "esp_svpwm.h"  // 为了使用 inverter_handle_t
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief FOC开环控制参数
 */
typedef struct {
    float voltage_magnitude;     // 电压矢量幅值 (0.0-1.0)
    float resistance;            // 电机相电阻(欧姆)
    float supply_voltage;        // 电机供电电压(伏特)
    float kv;                    // 电机KV值(RPM/V)
    float current_limit;         // 电流限制(安培)
    float speed_rpm;             // 设定转速 (RPM)
    int pole_pairs;              // 电机极对数
    int period;                  // 周期
    int direction;               // 电机方向
    float zero_angle_rad;        // 机械零角度(rad)
} foc_openloop_params_t;

/**
 * @brief FOC开环控制状态
 */
typedef struct {
    float angle_elec;            // 当前电角度 (rad)
    float angle_mech;            // 当前机械角度 (rad)
    float speed_elec;            // 当前电角速度 (rad/s)
    float speed_mech;            // 当前机械角速度 (rad/s)
    float target_speed_elec;     // 目标电角速度 (rad/s)
    float timestamp_us;          // 上次更新时间戳 (us)
    float duty_u;
    float duty_v;
    float duty_w;
    float q_current;            // q轴电流/转矩电流
    float d_current;            // d轴电流/磁链电流
} foc_openloop_state_t;

/**
 * @brief PI控制器参数
 */
typedef struct {
    float kp;               // 比例增益
    float ki;               // 积分增益
    float kd;               // 微分增益
    float integral;         // 积分项累积值
    float last_error;       // 上一次误差
    float output_limit;     // 输出限制，防止积分饱和
} foc_pid_controller_t;

/**
 * @brief FOC闭环控制模式
 */
typedef enum {
    FOC_CONTROL_MODE_TORQUE = 0,  // 转矩控制模式
    FOC_CONTROL_MODE_VELOCITY = 1,    // 速度控制模式
    FOC_CONTROL_MODE_POSITION = 2,     // 位置控制模式
    FOC_CONTROL_MODE_TORQUE_VELOCITY_POSITION = 3 // 转矩速度位置控制模式
} foc_control_mode_t;

/**
 * @brief FOC闭环控制参数
 */
typedef struct {
    int pole_pairs;              // 电机极对数
    int direction;               // 电机方向
    int period;                  // PWM周期
    float current_limit;         // 电流限制(安培)
    float supply_voltage;        // 电机供电电压(伏特)
    float kv;                    // 电机KV值(RPM/V)
    float resistance;            // 电机相电阻(欧姆)
    float voltage_limit;         // 电压限制(0.0-1.0)
    float dt;                    // 控制周期(秒)
    float zero_angle_rad;        // 机械零角度(rad)
    
    // 控制模式
    foc_control_mode_t control_mode;
    
    // 电机配置
    bool invert_phase_order;     // 是否反转相序
    
    // 目标值
    float target_iq;             // 目标q轴电流(转矩电流)
    float target_id;             // 目标d轴电流(通常为0)
    float target_maxcurrent;     // 目标最大电流(A)
    float target_velocity;       // 目标速度(rad/s)
    float target_position;       // 目标位置(rad)
    
    // 低通滤波参数
    float current_filter_alpha;  // 电流低通滤波系数 (0.0-1.0)
    
    // PI控制器参数
    foc_pid_controller_t id_pid;   // d轴电流控制器
    foc_pid_controller_t iq_pid;   // q轴电流控制器
    foc_pid_controller_t velocity_pid; // 速度控制器
    foc_pid_controller_t position_pid; // 位置控制器
} foc_closedloop_params_t;

/**
 * @brief FOC闭环控制状态
 */
typedef struct {
    // 反馈状态
    float id;                    // 实际d轴电流
    float iq;                    // 实际q轴电流
    float velocity;              // 实际速度(rad/s)
    float position;              // 实际位置(rad)
    float electrical_angle;      // 电气角度(rad)
    
    // 控制输出
    float vd;                    // d轴电压控制输出
    float vq;                    // q轴电压控制输出
    int duty_u;
    int duty_v;
    int duty_w;
    
    uint64_t timestamp_us;       // 上次更新时间戳(us)
} foc_closedloop_state_t;

/**
 * @brief FOC控制目标值结构体
 */
typedef struct {
    foc_control_mode_t control_mode;  ///< 控制模式
    float target_current;              ///< 目标电流(A)
    float target_maxcurrent;          ///< 目标最大电流(A)
    float target_velocity;            ///< 目标速度(rad/s)
    float target_position;            ///< 目标位置(rad)
} foc_target_t;

typedef struct {
    int duty_u;
    int duty_v;
    int duty_w;
} foc_duty_t;

/**
 * @brief 设置相电压
 * 
 * @param uq q轴电压(比0-1)
 * @param ud d轴电压(比0-1)
 * @param electrical_angle 电气角度(rad)
 * @param inverter SVPWM逆变器句柄
 */
esp_err_t foc_set_PhaseVoltage(float uq, float ud, float electrical_angle, int period, float voltage_magnitude, foc_duty_t *duty, inverter_handle_t inverter);
/**
 * @brief 初始化FOC开环控制
 * 
 * @param params 开环控制参数
 * @param state 开环控制状态，如果为NULL则会在内部创建
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t foc_openloop_init(const foc_openloop_params_t *params, foc_openloop_state_t *state);

/**
 * @brief 设置转速
 * 
 * @param speed_rpm 目标转速(RPM)
 * @return esp_err_t ESP_OK成功，其他失败
 */

esp_err_t foc_openloop_set_targetSpeed(float speed_rpm);

/**
 * @brief 设置电压幅值
 * 
 * @param voltage_magnitude 目标电压幅值(0.0-1.0)
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t foc_openloop_set_targetVoltage(float voltage_magnitude);

/**
 * @brief 执行FOC开环控制并直接输出PWM
 * 
 * @param inverter SVPWM逆变器句柄
 * @param rpm 目标转速(RPM)
 * @param voltage_magnitude 电压幅值(0.0-1.0)
 * @param pole_pairs 电机极对数
 * @return float 当前电角度(rad)
 */

float foc_openloop_output(inverter_handle_t inverter);

/**
 * @brief 停止开环控制，将速度逐渐降为0
 * 
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t foc_openloop_stop(void);

/**
 * @brief 获取FOC开环控制状态
 * 
 * @return 指向开环控制状态的指针
 */
foc_openloop_state_t* foc_openloop_get_state(void);

/**
 * @brief 获取FOC开环控制参数
 * 
 * @return 指向开环控制参数的指针
 */
foc_openloop_params_t* foc_openloop_get_params(void);

/**
 * @brief 初始化FOC闭环控制
 * 
 * @param params 闭环控制参数
 * @param state 闭环控制状态，如果为NULL则会在内部创建
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t foc_closedloop_init(const foc_closedloop_params_t *params, foc_closedloop_state_t *state);

/**
 * @brief 执行FOC闭环控制
 * 
 * @param inverter SVPWM逆变器句柄
 * @param phase_currents 三相电流测量值
 * @param electrical_angle 电气角度(rad)，通常从编码器获取
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t foc_closedloop_output(inverter_handle_t inverter, const foc_uvw_coord_t *phase_currents, _iq electrical_angle);

/**
 * @brief 设置控制模式和目标值
 * 
 * @param mode 控制模式(转矩/速度/位置)
 * @param target 目标值(根据模式不同而不同)
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t foc_closedloop_set_target(const foc_target_t *target);

/**
 * @brief 设置速度和位置
 * 
 * @param velocity 当前速度(rad/s)
 * @param position 当前位置(rad)
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t foc_closedloop_set_motion_state(float velocity, float position);

/**
 * @brief 获取FOC闭环控制状态
 * 
 * @return foc_closedloop_state_t* 指向闭环控制状态的指针
 */
foc_closedloop_state_t* foc_closedloop_get_state(void);

/**
 * @brief 获取FOC闭环控制参数
 * 
 * @return foc_closedloop_params_t* 指向闭环控制参数的指针
 */
foc_closedloop_params_t* foc_closedloop_get_params(void);

/**
 * @brief 停止闭环控制
 * 
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t foc_closedloop_stop(void);

#ifdef __cplusplus
}
#endif 