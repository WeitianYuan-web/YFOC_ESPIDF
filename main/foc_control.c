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

// 闭环控制全局变量
static foc_closedloop_state_t s_closedloop_state;
static foc_closedloop_params_t s_closedloop_params;
static foc_closedloop_state_t *p_cl_state = NULL;
static bool s_cl_is_initialized = false;

// 指向当前使用的状态变量的指针
static foc_openloop_state_t *p_state = NULL;
static bool s_is_initialized = false;

// 内部函数声明
static float foc_pi_controller_update(foc_pi_controller_t *pi, float error, float dt);

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
    p_state->target_speed_elec = speed_rps * 2.0f * M_PI * params->pole_pairs * params->direction;  // 转换为电角速度
    
    ESP_LOGI(TAG, "FOC开环控制初始化完成，目标速度: %.2f RPM, 电压幅值: %.2f, 极对数: %d, 方向: %d, 周期: %d",
             params->speed_rpm, params->voltage_magnitude, params->pole_pairs, params->direction, params->period);
    
    s_is_initialized = true;
    return ESP_OK;
}


esp_err_t foc_openloop_set_target(float speed_rpm)
{
    if (!s_is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    s_openloop_params.speed_rpm = speed_rpm;
    ESP_LOGI(TAG, "设置转速为 %.2f RPM", speed_rpm);
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
float foc_openloop_output(inverter_handle_t inverter)
{
    static uint64_t last_timestamp_us = 0;
    // 获取当前时间戳   
    uint64_t current_time = esp_timer_get_time();
    
    // 计算时间增量(秒)
    float dt = (current_time - last_timestamp_us) / 1e6f;
    last_timestamp_us = current_time;
    
    // 异常保护，防止时间间隔过大或首次调用
    if (dt > 0.1f || dt <= 0) {
        dt = 0.0005f;  // 使用默认值
    }

    
    // 计算电角速度(rad/s) = 机械转速(RPM) * 极对数 * 2π / 60
    p_state->target_speed_elec = s_openloop_params.speed_rpm * s_openloop_params.pole_pairs * s_openloop_params.direction * (2.0f * M_PI) * (1.0f / 60.0f);
    
    // 计算电角度增量
    float angle_increment = p_state->target_speed_elec * dt;
    
    // 更新电角度
    p_state->angle_elec = p_state->angle_elec + angle_increment;
    
    // 规范化电角度到 [0, 2π)
    float two_pi = 2.0f * M_PI;
    while (p_state->angle_elec >= two_pi) {
        p_state->angle_elec = p_state->angle_elec - two_pi;
    }
    while (p_state->angle_elec < 0) {
        p_state->angle_elec = p_state->angle_elec + two_pi;
    }
    
    // 获取电机参数
    float current_limit = s_openloop_params.current_limit;
    float supply_voltage = s_openloop_params.supply_voltage;
    float kv = s_openloop_params.kv;
    float resistance = s_openloop_params.resistance;
    float voltage_magnitude = s_openloop_params.voltage_magnitude;
    
    // 计算反电动势常数（Ke）
    // Ke = 1/(kv * 2π/60) - 将kv(RPM/V)转换为电动势常数(V/(rad/s))
    float ke = 1.0f / (kv * (2.0f * M_PI / 60.0f));
    
    // 计算反电动势大小
    float bemf = ke * fabsf(p_state->target_speed_elec);
    
    // 计算电压输出
    float vd = 0;
    // q轴电压需要克服反电动势
    float vq = bemf;
    
    // 考虑电阻损耗的额外电压
    float v_ir = resistance * current_limit;
    vq += v_ir;
    
    // 确保不超过电压限制
    float v_magnitude = sqrtf(vd*vd + vq*vq);
    float v_limit = supply_voltage * voltage_magnitude;
    
    if (v_magnitude > v_limit) {
        // 按比例缩小电压
        float scale = v_limit / v_magnitude;
        vd *= scale;
        vq *= scale;
    }
    
    // 根据电压反推电流（I=V/R，简化模型）
    float id = vd / resistance;
    float iq = vq / resistance;
    
    // 确保电流不超过限制
    float i_magnitude = sqrtf(id*id + iq*iq);
    if (i_magnitude > current_limit) {
        float i_scale = current_limit / i_magnitude;
        id *= i_scale;
        iq *= i_scale;
    }

    // 设置dq电流输出
    foc_dq_coord_t dq_out;
    dq_out.d = _IQ(id);
    dq_out.q = _IQ(iq);
    
    // 执行Park逆变换
    foc_ab_coord_t ab_out;
    foc_inverse_park_transform(_IQ(p_state->angle_elec), &dq_out, &ab_out);
    
    // 计算SVPWM占空比
    foc_uvw_coord_t uvw_out;
    foc_svpwm_duty_calculate(&ab_out, &uvw_out);
    
    // 转换为占空比值
    int uvw_duty[3]; //占空比范围为0-1000
    uvw_duty[0] = _IQtoF(_IQmpy(_IQmpy(_IQ(s_openloop_params.voltage_magnitude), _IQdiv2(uvw_out.u)), _IQ(s_openloop_params.period/4))) + (s_openloop_params.period / 4);
    uvw_duty[1] = _IQtoF(_IQmpy(_IQmpy(_IQ(s_openloop_params.voltage_magnitude), _IQdiv2(uvw_out.v)), _IQ(s_openloop_params.period/4))) + (s_openloop_params.period / 4);
    uvw_duty[2] = _IQtoF(_IQmpy(_IQmpy(_IQ(s_openloop_params.voltage_magnitude), _IQdiv2(uvw_out.w)), _IQ(s_openloop_params.period/4))) + (s_openloop_params.period / 4); 
    p_state->duty_u = uvw_duty[0];
    p_state->duty_v = uvw_duty[1];
    p_state->duty_w = uvw_duty[2];
    
    // 输出PWM
    svpwm_inverter_set_duty(inverter, uvw_duty[0], uvw_duty[1], uvw_duty[2]);
    
    // 返回电角度
    return p_state->angle_elec;
}

foc_openloop_state_t* foc_openloop_get_state(void)
{
    return p_state;
}

foc_openloop_params_t* foc_openloop_get_params(void)
{
    return &s_openloop_params;
}

/**
 * @brief 更新PI控制器
 * 
 * @param pi PI控制器
 * @param error 误差值
 * @param dt 时间增量
 * @return float 控制输出
 */
static float foc_pi_controller_update(foc_pi_controller_t *pi, float error, float dt)
{
    // 计算比例项
    float proportional = pi->kp * error;
    
    // 更新积分项
    pi->integral += error * dt;
    
    // 积分限幅，防止积分饱和
    if (pi->integral > pi->output_limit) {
        pi->integral = pi->output_limit;
    } else if (pi->integral < -pi->output_limit) {
        pi->integral = -pi->output_limit;
    }
    
    // 计算积分项的贡献
    float integral = pi->ki * pi->integral;
    
    // 计算最终输出
    float output = proportional + integral;
    
    // 输出限幅
    if (output > pi->output_limit) {
        output = pi->output_limit;
    } else if (output < -pi->output_limit) {
        output = -pi->output_limit;
    }
    
    return output;
}

/**
 * @brief 初始化FOC闭环控制
 * 
 * @param params 闭环控制参数
 * @param state 闭环控制状态，如果为NULL则会在内部创建
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t foc_closedloop_init(const foc_closedloop_params_t *params, foc_closedloop_state_t *state)
{
    ESP_RETURN_ON_FALSE(params, ESP_ERR_INVALID_ARG, TAG, "params is NULL");
    ESP_RETURN_ON_FALSE(params->pole_pairs > 0, ESP_ERR_INVALID_ARG, TAG, "pole_pairs must be positive");
    ESP_RETURN_ON_FALSE(params->period > 0, ESP_ERR_INVALID_ARG, TAG, "period must be positive");
    
    if (state) {
        p_cl_state = state;
    } else {
        p_cl_state = &s_closedloop_state;
    }
    
    // 复制参数
    memcpy(&s_closedloop_params, params, sizeof(foc_closedloop_params_t));
    
    // 初始化状态
    memset(p_cl_state, 0, sizeof(foc_closedloop_state_t));
    p_cl_state->timestamp_us = esp_timer_get_time();
    
    ESP_LOGI(TAG, "FOC闭环控制初始化完成，控制模式: %d", params->control_mode);
    
    s_cl_is_initialized = true;
    return ESP_OK;
}

/**
 * @brief 执行FOC闭环控制
 * 
 * @param inverter SVPWM逆变器句柄
 * @param phase_currents 三相电流测量值
 * @param electrical_angle 电气角度(rad)，通常从编码器获取
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t foc_closedloop_output(inverter_handle_t inverter, const foc_uvw_coord_t *phase_currents, _iq electrical_angle)
{
    if (!s_cl_is_initialized || !p_cl_state) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!phase_currents) {
        ESP_LOGW(TAG, "电流测量值为NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    // 获取当前时间戳
    uint64_t current_time = esp_timer_get_time();
    
    // 计算时间增量(秒)
    float dt = (current_time - p_cl_state->timestamp_us) / 1e6f;
    p_cl_state->timestamp_us = current_time;
    
    // 异常保护，防止时间间隔过大或首次调用
    if (dt > 0.5f || dt <= 0) {
        dt = s_closedloop_params.dt;  // 使用默认值
    }
    
    // Step 1: 计算d-q轴电流
    foc_dq_coord_t dq_currents;
    foc_calculate_dq_current(phase_currents, electrical_angle, &dq_currents);
    
    // 获取d-q轴电流
    float id_raw = _IQtoF(dq_currents.d);
    float iq_raw = _IQtoF(dq_currents.q);
    
    // 应用低通滤波
    float alpha = s_closedloop_params.current_filter_alpha;
    if (alpha <= 0.0f) alpha = 1.0f; // 如果未设置滤波系数，不进行滤波
    
    // 根据滤波系数计算滤波后的电流值
    float id_filtered = alpha * id_raw + (1.0f - alpha) * p_cl_state->id;
    float iq_filtered = alpha * iq_raw + (1.0f - alpha) * p_cl_state->iq;
    
    // 更新当前状态中的d-q轴电流 (使用滤波后的值)
    p_cl_state->id = id_filtered;
    p_cl_state->iq = iq_filtered;
    p_cl_state->electrical_angle = _IQtoF(electrical_angle);
    
    // Step 2: 根据控制模式确定目标值
    float target_id = s_closedloop_params.target_id; // 通常d轴电流目标为0
    float target_iq = s_closedloop_params.target_iq;
    float vd = 0;
    float vq = 0;

    switch (s_closedloop_params.control_mode)
    {
    case FOC_CONTROL_MODE_TORQUE:
        target_iq = s_closedloop_params.target_iq;
        // 电流限幅
        if (target_iq > s_closedloop_params.current_limit)
        {
            target_iq = s_closedloop_params.current_limit;
        }
        else if (target_iq < -s_closedloop_params.current_limit)
        {
            target_iq = -s_closedloop_params.current_limit;
        }

        // Step 3: 电流PI控制器计算电压输出
        float id_error = target_id - p_cl_state->id;
        float iq_error = target_iq - p_cl_state->iq;

        // 电流闭环控制是最内环，直接影响力矩
        // 这里d轴控制磁链，通常设为0；q轴控制转矩
        vd = foc_pi_controller_update(&s_closedloop_params.id_pi, id_error, dt);
        vq = foc_pi_controller_update(&s_closedloop_params.iq_pi, iq_error, dt);

        break;
    case FOC_CONTROL_MODE_VELOCITY:
        float velocity_error = s_closedloop_params.target_velocity - p_cl_state->velocity;
        target_iq = foc_pi_controller_update(&s_closedloop_params.velocity_pi, velocity_error, dt);
        break;
    case FOC_CONTROL_MODE_POSITION:
        float position_error = s_closedloop_params.target_position - p_cl_state->position;
        float target_velocity = foc_pi_controller_update(&s_closedloop_params.position_pi, position_error, dt);
        break;
    case FOC_CONTROL_MODE_TORQUE_VELOCITY:
        target_iq = s_closedloop_params.target_iq;
        break;
    case FOC_CONTROL_MODE_TORQUE_POSITION:
        target_iq = s_closedloop_params.target_iq;
        break;
    case FOC_CONTROL_MODE_TORQUE_VELOCITY_POSITION:
        target_iq = s_closedloop_params.target_iq;
        break;
    }

    // 电压限幅 - 使用矢量限幅确保不超过电源电压
    float v_magnitude = sqrtf(vd * vd + vq * vq);
    if (v_magnitude > s_closedloop_params.voltage_limit && v_magnitude > 0) {
        float scale = s_closedloop_params.voltage_limit / v_magnitude;
        vd *= scale;
        vq *= scale;
    }
    
    // 更新电压输出状态
    p_cl_state->vd = vd;
    p_cl_state->vq = vq;
    
    // Step 4: 将d-q轴电压转换到α-β坐标系，再转换到三相电压
    foc_dq_coord_t vdq;
    vdq.d = _IQ(vd);
    vdq.q = _IQ(vq);
    
    // 如果设置了相序反转，反转q轴电压方向
    if (s_closedloop_params.invert_phase_order) {
        vdq.q = -vdq.q;
    }
    
    foc_ab_coord_t vab;
    foc_inverse_park_transform(electrical_angle, &vdq, &vab);
    
    foc_uvw_coord_t vuvw;
    foc_svpwm_duty_calculate(&vab, &vuvw);
    
    
    // 计算PWM占空比
    int uvw_duty[3];
    uvw_duty[0] = _IQtoF(s_closedloop_params.voltage_limit * _IQdiv2(vuvw.u)) * s_closedloop_params.period/4 + (s_closedloop_params.period / 4);
    uvw_duty[1] = _IQtoF(s_closedloop_params.voltage_limit * _IQdiv2(vuvw.v)) * s_closedloop_params.period/4 + (s_closedloop_params.period / 4);
    uvw_duty[2] = _IQtoF(s_closedloop_params.voltage_limit * _IQdiv2(vuvw.w)) * s_closedloop_params.period/4 + (s_closedloop_params.period / 4);
    
    // 更新占空比状态
    p_cl_state->duty_u = uvw_duty[0];
    p_cl_state->duty_v = uvw_duty[1];
    p_cl_state->duty_w = uvw_duty[2];
    
    // 输出PWM
    svpwm_inverter_set_duty(inverter, uvw_duty[0], uvw_duty[1], uvw_duty[2]);
    
    return ESP_OK;
}

/**
 * @brief 设置控制模式和目标值
 * 
 * @param mode 控制模式(转矩/速度/位置)
 * @param target 目标值(根据模式不同而不同)
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t foc_closedloop_set_target(foc_control_mode_t mode, float target)
{
    if (!s_cl_is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // 设置控制模式
    s_closedloop_params.control_mode = mode;
    
    // 根据控制模式设置目标值
    switch (mode) {
        case FOC_CONTROL_MODE_TORQUE:
            s_closedloop_params.target_iq = target;
            ESP_LOGI(TAG, "设置转矩控制模式，目标q轴电流: %.2f A", target);
            break;
        case FOC_CONTROL_MODE_VELOCITY:
            s_closedloop_params.target_velocity = target;
            ESP_LOGI(TAG, "设置速度控制模式，目标速度: %.2f rad/s", target);
            break;
        case FOC_CONTROL_MODE_POSITION:
            s_closedloop_params.target_position = target;
            ESP_LOGI(TAG, "设置位置控制模式，目标位置: %.2f rad", target);
            break;
        case FOC_CONTROL_MODE_TORQUE_VELOCITY:
            s_closedloop_params.target_iq = target;
            ESP_LOGI(TAG, "设置转矩速度控制模式，目标q轴电流: %.2f A", target);
            break;
        case FOC_CONTROL_MODE_TORQUE_POSITION:
            s_closedloop_params.target_iq = target;
            ESP_LOGI(TAG, "设置转矩位置控制模式，目标q轴电流: %.2f A", target);
            break;
        case FOC_CONTROL_MODE_TORQUE_VELOCITY_POSITION:
            s_closedloop_params.target_iq = target;
            ESP_LOGI(TAG, "设置转矩速度位置控制模式，目标q轴电流: %.2f A", target);
            break;
        default:
            ESP_LOGW(TAG, "未知的控制模式: %d", mode);
            return ESP_ERR_INVALID_ARG;
    }
    
    return ESP_OK;
}

/**
 * @brief 设置速度和位置
 * 
 * @param velocity 当前速度(rad/s)
 * @param position 当前位置(rad)
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t foc_closedloop_set_motion_state(float velocity, float position)
{
    if (!s_cl_is_initialized || !p_cl_state) {
        return ESP_ERR_INVALID_STATE;
    }
    
    p_cl_state->velocity = velocity;
    p_cl_state->position = position;
    
    return ESP_OK;
}

/**
 * @brief 获取FOC闭环控制状态
 * 
 * @return foc_closedloop_state_t* 指向闭环控制状态的指针
 */
foc_closedloop_state_t* foc_closedloop_get_state(void)
{
    return p_cl_state;
}

/**
 * @brief 获取FOC闭环控制参数
 * 
 * @return foc_closedloop_params_t* 指向闭环控制参数的指针
 */
foc_closedloop_params_t* foc_closedloop_get_params(void)
{
    return &s_closedloop_params;
}

/**
 * @brief 停止闭环控制
 * 
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t foc_closedloop_stop(void)
{
    if (!s_cl_is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // 根据当前模式设置合适的停止方式
    switch (s_closedloop_params.control_mode) {
        case FOC_CONTROL_MODE_TORQUE:
            // 对于转矩控制，设置目标转矩为0
            s_closedloop_params.target_iq = 0;
            break;
        case FOC_CONTROL_MODE_VELOCITY:
            // 对于速度控制，设置目标速度为0
            s_closedloop_params.target_velocity = 0;
            break;
        case FOC_CONTROL_MODE_POSITION:
            // 对于位置控制，保持当前位置
            s_closedloop_params.target_position = p_cl_state->position;
            break;
        case FOC_CONTROL_MODE_TORQUE_VELOCITY:
            // 对于转矩速度控制，保持当前状态
            s_closedloop_params.target_iq = p_cl_state->iq;
            s_closedloop_params.target_velocity = p_cl_state->velocity;
            break;
        case FOC_CONTROL_MODE_TORQUE_POSITION:
            // 对于转矩位置控制，保持当前状态
            s_closedloop_params.target_iq = p_cl_state->iq;
            s_closedloop_params.target_position = p_cl_state->position;
            break;
        case FOC_CONTROL_MODE_TORQUE_VELOCITY_POSITION:
            // 对于转矩速度位置控制，保持当前状态
            s_closedloop_params.target_iq = p_cl_state->iq;
            s_closedloop_params.target_velocity = p_cl_state->velocity;
            s_closedloop_params.target_position = p_cl_state->position;
            break;
    }
    
    ESP_LOGI(TAG, "FOC闭环控制已停止");
    return ESP_OK;
}