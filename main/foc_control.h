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
    float speed_rpm;             // 设定转速 (RPM)
    int pole_pairs;              // 电机极对数
    int period;                  // 周期
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
} foc_openloop_state_t;

/**
 * @brief 初始化FOC开环控制
 * 
 * @param params 开环控制参数
 * @param state 开环控制状态，如果为NULL则会在内部创建
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t foc_openloop_init(const foc_openloop_params_t *params, foc_openloop_state_t *state);

/**
 * @brief 运行FOC开环控制，计算下一个电角度和输出电压
 * 
 * @param timestamp_us 当前时间戳(us)
 * @param dq_out 输出的dq坐标值
 * @return 当前电角度值(rad)
 */

esp_err_t foc_openloop_set_voltage(float voltage);

/**
 * @brief 执行FOC开环控制并直接输出PWM
 * 
 * @param inverter SVPWM逆变器句柄
 * @param rpm 目标转速(RPM)
 * @param voltage_magnitude 电压幅值(0.0-1.0)
 * @param pole_pairs 电机极对数
 * @return _iq 当前电角度(IQ格式)
 */

_iq foc_openloop_output(inverter_handle_t inverter, const foc_openloop_params_t *params);

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

#ifdef __cplusplus
}
#endif 