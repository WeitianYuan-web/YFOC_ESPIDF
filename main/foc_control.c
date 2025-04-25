/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_check.h"
#include "foc_control.h"
#include <stdbool.h>
#include "esp_timer.h"

static const char *TAG = "foc_control";

// 全局状态变量，如果用户没有提供状态变量，就使用这个
static foc_openloop_state_t s_openloop_state;
static foc_openloop_params_t s_openloop_params;

// 指向当前使用的状态变量的指针
static foc_openloop_state_t *p_state = NULL;
static bool s_is_initialized = false;

esp_err_t foc_openloop_init(const foc_openloop_params_t *params, foc_openloop_state_t *state)
{
    ESP_RETURN_ON_FALSE(params, ESP_ERR_INVALID_ARG, TAG, "params is NULL");
    ESP_RETURN_ON_FALSE(params->pole_pairs > 0, ESP_ERR_INVALID_ARG, TAG, "pole_pairs must be positive");
    ESP_RETURN_ON_FALSE(params->period > 0, ESP_ERR_INVALID_ARG, TAG, "period must be positive");
    
    if (state) {
        p_state = state;
    } else {
        p_state = &s_openloop_state;
    }
    
    // 复制参数
    memcpy(&s_openloop_params, params, sizeof(foc_openloop_params_t));
    
    // 初始化状态
    memset(p_state, 0, sizeof(foc_openloop_state_t));
    p_state->timestamp_us = esp_timer_get_time();
    
    // 计算目标电角速度
    float speed_rps = params->speed_rpm / 60.0f;  // 转换为每秒转数
    p_state->target_speed_elec = speed_rps * 2.0f * M_PI * params->pole_pairs;  // 转换为电角速度
    
    ESP_LOGI(TAG, "FOC开环控制初始化完成，目标速度: %.2f RPM, 电压幅值: %.2f, 极对数: %d, 周期: %d",
             params->speed_rpm, params->voltage_magnitude, params->pole_pairs, params->period);
    
    s_is_initialized = true;
    return ESP_OK;
}


esp_err_t foc_openloop_set_voltage(float voltage)
{
    if (!s_is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (voltage < 0.0f || voltage > 1.0f) {
        ESP_LOGW(TAG, "电压幅值应在0.0到1.0之间，当前值: %.2f", voltage);
        voltage = voltage < 0.0f ? 0.0f : 1.0f;
    }
    
    s_openloop_params.voltage_magnitude = voltage;
    ESP_LOGI(TAG, "设置电压幅值为 %.2f", voltage);
    return ESP_OK;
}

esp_err_t foc_openloop_stop(void)
{
    if (!s_is_initialized || !p_state) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // 设置目标速度为0，让电机慢慢停下
    p_state->target_speed_elec = 0.0f;
    ESP_LOGI(TAG, "开始减速停止");
    return ESP_OK;
}

/**
 * @brief 执行FOC开环控制并直接输出PWM
 * 
 * @param inverter SVPWM逆变器句柄
 * @param params 开环控制参数
 * @return _iq 当前电角度(IQ格式)
 */
_iq foc_openloop_output(inverter_handle_t inverter, const foc_openloop_params_t *params)
{
    static uint64_t last_timestamp_us = 0;
    static _iq angle_elec = _IQ(0);
    
    if (!params) {
        ESP_LOGW(TAG, "参数为NULL");
        return _IQ(0);
    }
    
    // 获取当前时间戳
    uint64_t current_time = esp_timer_get_time();
    
    // 计算时间增量(秒)
    float dt_f = (current_time - last_timestamp_us) / 1e6f;
    last_timestamp_us = current_time;
    
    // 异常保护，防止时间间隔过大或首次调用
    if (dt_f > 0.5f || dt_f <= 0) {
        dt_f = 0.001f;  // 使用默认值
    }
    _iq dt = _IQ(dt_f);
    
    // 计算电角速度(rad/s) = 机械转速(RPM) * 极对数 * 2π / 60
    p_state->target_speed_elec = _IQmpy(_IQmpy(_IQ(params->speed_rpm * params->pole_pairs), _IQ(2.0f * M_PI)), _IQ(1.0f / 60.0f));
    
    // 计算电角度增量
    _iq angle_increment = _IQmpy(p_state->target_speed_elec, dt);
    
    // 更新电角度
    angle_elec = angle_elec + angle_increment;
    
    // 规范化电角度到 [0, 2π)
    _iq two_pi = _IQ(2.0f * M_PI);
    while (_IQtoF(angle_elec) >= _IQtoF(two_pi)) {
        angle_elec = angle_elec - two_pi;
    }
    while (_IQtoF(angle_elec) < 0) {
        angle_elec = angle_elec + two_pi;
    }
    
    // 设置dq输出(d轴施加电压，q轴设为0)
    foc_dq_coord_t dq_out;
    dq_out.d = _IQ(params->voltage_magnitude);
    dq_out.q = _IQ(0);
    
    // 执行Park逆变换
    foc_ab_coord_t ab_out;
    foc_inverse_park_transform(angle_elec, &dq_out, &ab_out);
    
    // 计算SVPWM占空比
    foc_uvw_coord_t uvw_out;
    foc_svpwm_duty_calculate(&ab_out, &uvw_out);
    
    // 转换为占空比值
    int uvw_duty[3];
    uvw_duty[0] = _IQtoF(params->voltage_magnitude * _IQdiv2(uvw_out.u)) * params->period/4 + (params->period / 4);
    uvw_duty[1] = _IQtoF(params->voltage_magnitude * _IQdiv2(uvw_out.v)) * params->period/4 + (params->period / 4);
    uvw_duty[2] = _IQtoF(params->voltage_magnitude * _IQdiv2(uvw_out.w)) * params->period/4 + (params->period / 4); 
    p_state->duty_u = uvw_duty[0];
    p_state->duty_v = uvw_duty[1];
    p_state->duty_w = uvw_duty[2];
    
    // 输出PWM
    svpwm_inverter_set_duty(inverter, uvw_duty[0], uvw_duty[1], uvw_duty[2]);
    
    // 返回电角度
    return angle_elec;
}

foc_openloop_state_t* foc_openloop_get_state(void)
{
    return p_state;
}

foc_openloop_params_t* foc_openloop_get_params(void)
{
    return &s_openloop_params;
}