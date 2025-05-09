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


#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define _2PI        6.28318530718f

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
static float foc_pid_controller_update(foc_pid_controller_t *pid, float error, float dt);

/**
 * @brief 计算两个角度之间的差值，并确保结果在-180度到180度之间
 * 
 * @param diff 两个角度之间的差值
 * @param cycle 角度周期（通常为360度）
 * @return float 调整后的角度差值
 */
float cycle_diff(float diff, float cycle)
{
    // 参数校验
    const float eps = 1e-6f;
    if (cycle <= 0.0f || isnan(cycle)) {
        return NAN;  // 或返回原值/抛出错误
    }

    // 计算半周期并调整
    const float half_cycle = cycle / 2.0f;
    if (diff > half_cycle + eps) {
        diff -= cycle;
    } else if (diff < -half_cycle - eps) {
        diff += cycle;
    }
    return diff;
}

esp_err_t foc_set_PhaseVoltage(float uq, float ud, float electrical_angle, int period, float voltage_magnitude, foc_duty_t *duty, inverter_handle_t inverter)
{
    uq = _constrain(uq, -1.0f, 1.0f);
    ud = _constrain(ud, -1.0f, 1.0f);
    // 设置dq电流输出
    foc_dq_coord_t dq_out;
    dq_out.d = _IQ(ud);
    dq_out.q = _IQ(uq);
    
    // 执行Park逆变换
    foc_ab_coord_t ab_out;
    foc_inverse_park_transform(_IQ(electrical_angle), &dq_out, &ab_out);
    
    // 计算SVPWM占空比
    foc_uvw_coord_t uvw_out;
    foc_svpwm_duty_calculate(&ab_out, &uvw_out);
    
    int _period = period / 2;
    // 转换为占空比值
    int uvw_duty[3]; //占空比范围为0-1000
    // 将IQ数据转换为PWM占空比
    uvw_duty[0] = (_IQtoF(_IQdiv2(uvw_out.u)) + 0.25f) * _period * voltage_magnitude;
    uvw_duty[1] = (_IQtoF(_IQdiv2(uvw_out.v)) + 0.25f) * _period * voltage_magnitude;
    uvw_duty[2] = (_IQtoF(_IQdiv2(uvw_out.w)) + 0.25f) * _period * voltage_magnitude;
    uvw_duty[0] = _constrain(uvw_duty[0], 0, _period);
    uvw_duty[1] = _constrain(uvw_duty[1], 0, _period);
    uvw_duty[2] = _constrain(uvw_duty[2], 0, _period);
    duty->duty_u = uvw_duty[0];
    duty->duty_v = uvw_duty[1];
    duty->duty_w = uvw_duty[2];
    // 输出PWM
    svpwm_inverter_set_duty(inverter, uvw_duty[0], uvw_duty[1], uvw_duty[2]);
    return ESP_OK;
}

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


esp_err_t foc_openloop_set_targetSpeed(float speed_rpm)
{
    if (!s_is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    s_openloop_params.speed_rpm = speed_rpm;
    ESP_LOGI(TAG, "设置转速为 %.2f RPM", speed_rpm);
    return ESP_OK;
}

esp_err_t foc_openloop_set_targetVoltage(float voltage_magnitude)
{
    if (!s_is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    s_openloop_params.voltage_magnitude = voltage_magnitude;
    ESP_LOGI(TAG, "设置电压幅值为 %.2f", voltage_magnitude);
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
    float max_current = supply_voltage/resistance;

    if(current_limit >= max_current){
        current_limit = max_current;
    }
    // 计算反电动势常数（Ke）
    // Ke = 1/(kv * 2π/60) - 将kv(RPM/V)转换为电动势常数(V/(rad/s))
    float ke = 1.0f / (kv * (2.0f * M_PI / 60.0f));
    
    // 计算反电动势大小
    float bemf = ke * fabsf(p_state->target_speed_elec);
    
    // 考虑电阻损耗的额外电压
    float v_ir = resistance * current_limit;
    
    // 计算电压输出
    float vd = 0;

    // q轴电压需要克服反电动势
    float vq = bemf + v_ir;

    // 确保不超过电压限制
    float v_magnitude = sqrtf(vd*vd + vq*vq);
    float v_limit = supply_voltage * voltage_magnitude;
    
    if (v_magnitude > v_limit) {
        // 按比例缩小电压
        float scale = v_limit / v_magnitude;
        vd *= scale;
        vq *= scale;
    }
    
    
    float id = vd / supply_voltage ;
    float iq = vq / supply_voltage;
    
    id = _constrain(id, -current_limit/max_current, current_limit/max_current);
    iq = _constrain(iq, -current_limit/max_current, current_limit/max_current);
    foc_duty_t duty;
    foc_set_PhaseVoltage(iq, id, p_state->angle_elec, s_openloop_params.period, 1.0f, &duty, inverter);
    p_state->duty_u = duty.duty_u;  // 保存占空比
    p_state->duty_v = duty.duty_v;
    p_state->duty_w = duty.duty_w;
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
 * @param error 误差值 (单位取决于具体应用)
 * @param dt 时间增量 (秒)
 * @return float 控制输出 (归一化电压，范围为-1到1)
 */
static float foc_pid_controller_update(foc_pid_controller_t *pid, float error, float dt)
{
    // 参数检查
    if (!pid || dt <= 0) {
        ESP_LOGW(TAG, "PI控制器参数无效");
        return 0.0f;
    }
    
    // 计算比例项
    float proportional = pid->kp * error;
    
    // 计算积分项增量
    float integral_increment = error * dt;
    
    integral_increment = _constrain(integral_increment, -pid->output_limit, pid->output_limit);
    // 更新积分项
    pid->integral += integral_increment;
    
    // 计算积分项的贡献
    float integral = pid->ki * pid->integral;
    
    // 计算最终输出
    float output = proportional + integral + pid->kd * (error-pid->last_error) / dt;
    
    // 更新上一次误差
    pid->last_error = error;
    

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
    ESP_RETURN_ON_FALSE(params->voltage_limit > 0 && params->voltage_limit <= 1.0f, 
                      ESP_ERR_INVALID_ARG, TAG, "voltage_limit must be in range (0, 1]");
    
    
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
    
    ESP_LOGI(TAG, "FOC闭环控制初始化完成，控制模式: %d, 电压限制: %.2f", 
            params->control_mode, params->voltage_limit);
    
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
        ESP_LOGW(TAG, "控制周期异常，使用默认值: %.6f s", dt);
    }

    float max_current = s_closedloop_params.supply_voltage/s_closedloop_params.resistance;

    // 计算反电动势常数（Ke）
    float ke = 9.55f / s_closedloop_params.kv;
        
    // 计算反电动势大小
    float bemf = ke * fabsf(p_cl_state->velocity);

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
    float target_id = 0.0f; // 通常d轴电流目标为0
    float target_iq = 0.0f;
    float target_velocity = 0.0f;
    float target_position = 0.0f;
    float vd = 0;
    float vq = 0;

    // 根据控制模式计算目标值
    switch (s_closedloop_params.control_mode) {
        case FOC_CONTROL_MODE_TORQUE:
            // 转矩控制模式：直接使用设定的q轴电流目标值
            target_iq = s_closedloop_params.target_iq;
            break;
            
        case FOC_CONTROL_MODE_VELOCITY:
            // 速度控制模式：通过速度PI控制器计算q轴电流目标值
            {   
                target_velocity = s_closedloop_params.target_velocity;
                float velocity_error = target_velocity - p_cl_state->velocity;
                float vel_derivative = (velocity_error - s_closedloop_params.velocity_pid.last_error) / dt;
                s_closedloop_params.velocity_pid.last_error = velocity_error;
                
                float p_vel = s_closedloop_params.velocity_pid.kp * velocity_error;
                float d_vel = s_closedloop_params.velocity_pid.kd * vel_derivative;
                float pd_vel = p_vel + d_vel;
                
             if (fabs(pd_vel) < 1.0f) {
                    s_closedloop_params.velocity_pid.integral += velocity_error * dt;
                    s_closedloop_params.velocity_pid.integral = _constrain(s_closedloop_params.velocity_pid.integral, -s_closedloop_params.velocity_pid.output_limit, s_closedloop_params.velocity_pid.output_limit);
                } else {

                    s_closedloop_params.velocity_pid.integral *= 0.98f;
                }
                target_iq = pd_vel + s_closedloop_params.velocity_pid.integral * s_closedloop_params.velocity_pid.ki;
            }
            break;
            
        case FOC_CONTROL_MODE_POSITION_SINGLE:
            // 位置控制模式：通过位置PI控制器计算速度目标，再通过速度PI控制器计算q轴电流目标
            {   
                target_position = s_closedloop_params.target_position;
                float position_error = target_position - p_cl_state->position_single;
                position_error = cycle_diff(position_error, _2PI);
                float pos_derivative = (position_error - s_closedloop_params.position_single_pid.last_error) / dt;
                s_closedloop_params.position_single_pid.last_error = position_error;
                float p_pos = s_closedloop_params.position_single_pid.kp * position_error;
                float d_pos = s_closedloop_params.position_single_pid.kd * pos_derivative;
                float pd_pos = p_pos + d_pos;
                pd_pos = _constrain(pd_pos, -s_closedloop_params.position_single_pid.output_limit, s_closedloop_params.position_single_pid.output_limit);
                if (fabs(position_error) < 1.57f) {
                    s_closedloop_params.position_single_pid.integral += position_error * dt;
                    s_closedloop_params.position_single_pid.integral = _constrain(s_closedloop_params.position_single_pid.integral, -50.0f, 50.0f);
                } else {
                    s_closedloop_params.position_single_pid.integral = 0.0f;
                }
                target_iq = pd_pos + s_closedloop_params.position_single_pid.integral * s_closedloop_params.position_single_pid.ki;
            }
            break;
            
        case FOC_CONTROL_MODE_POSITION_FULL:
            {
                // 位置控制模式：直接使用设定的位置目标值
                target_position = s_closedloop_params.target_position;
                float position_error = target_position - p_cl_state->position_full;
                float pos_derivative = (position_error - s_closedloop_params.position_full_pid.last_error) / dt;
                s_closedloop_params.position_full_pid.last_error = position_error;
                float p_pos = s_closedloop_params.position_full_pid.kp * position_error;
                float d_pos = s_closedloop_params.position_full_pid.kd * pos_derivative;
                float pd_pos = p_pos + d_pos;
                if (fabs(position_error) < 1.57f) {
                    s_closedloop_params.position_full_pid.integral += position_error * dt;
                    s_closedloop_params.position_full_pid.integral = _constrain(s_closedloop_params.position_full_pid.integral, -10.0f, 10.0f);
                } else {
                    s_closedloop_params.position_full_pid.integral *= 0.98f;
                }
                pd_pos = _constrain(pd_pos, -s_closedloop_params.position_full_pid.output_limit, s_closedloop_params.position_full_pid.output_limit);
                target_iq = pd_pos + s_closedloop_params.position_full_pid.integral * s_closedloop_params.position_full_pid.ki;
            }
            break;

        case FOC_CONTROL_MODE_TORQUE_VELOCITY_POSITION:
            // 转矩速度位置控制模式：综合考虑三个控制目标
            {   
                target_position = s_closedloop_params.target_position;
                target_velocity = s_closedloop_params.target_velocity;
                float position_error = target_position - p_cl_state->position_full;
                float position_velocity = foc_pid_controller_update(&s_closedloop_params.position_pid, position_error, dt);
                position_velocity = _constrain(position_velocity, -target_velocity, target_velocity);
                if (fabs(position_error) < 1.57f) {
                    s_closedloop_params.position_pid.integral = _constrain(s_closedloop_params.position_pid.integral, -10.0f, 10.0f);
                } else {
                    s_closedloop_params.position_pid.integral *= 0.98f;
                }
                float velocity_error = position_velocity - p_cl_state->velocity;
                float velocity_iq = foc_pid_controller_update(&s_closedloop_params.velocity_pid, velocity_error, dt);
                target_iq = velocity_iq;
            }
            break;
            
        default:
            ESP_LOGW(TAG, "未知的控制模式: %d", s_closedloop_params.control_mode);
            return ESP_ERR_INVALID_ARG;
    } 

    float min_limit = fminf(s_closedloop_params.current_limit, 
                          s_closedloop_params.target_maxcurrent);
    
    if (target_iq > min_limit) {
        target_iq = min_limit;
    } else if (target_iq < -min_limit) {
        target_iq = -min_limit;
    }

/*     float uq = target_iq * s_closedloop_params.resistance / s_closedloop_params.supply_voltage;
    float ud = target_id * s_closedloop_params.resistance / s_closedloop_params.supply_voltage;
 */
    // Step 3: 电流PI控制器计算电压输出 (结果为归一化电压，范围-1到1)
    float id_error = target_id - p_cl_state->id;
    float iq_error = target_iq - p_cl_state->iq;
    // 电流闭环控制是最内环，直接影响力矩
    // 这里d轴控制磁链，通常设为0；q轴控制转矩
    float id_integral = 0.0f;
    float iq_integral = 0.0f;
    id_integral += id_error * dt;
    iq_integral += iq_error * dt;
    id_integral = _constrain(id_integral, -1.0f, 1.0f);
    iq_integral = _constrain(iq_integral, -1.0f, 1.0f);
    vq = s_closedloop_params.iq_pid.kp * iq_error + s_closedloop_params.iq_pid.ki * iq_integral + target_iq * 0.7f + (bemf / s_closedloop_params.supply_voltage) * 0.05f;
    vd = s_closedloop_params.id_pid.kp * id_error + s_closedloop_params.id_pid.ki * id_integral - (bemf / s_closedloop_params.supply_voltage) * 0.7f;
    vq = _constrain(vq, -s_closedloop_params.iq_pid.output_limit, s_closedloop_params.iq_pid.output_limit);
    vd = _constrain(vd, -s_closedloop_params.id_pid.output_limit, s_closedloop_params.id_pid.output_limit);

    // 更新电压输出状态
    p_cl_state->vd = vd;
    p_cl_state->vq = vq; 
    
    // Step 4: 将d-q轴电压转换到α-β坐标系，再转换到三相电压
    foc_duty_t duty;
    foc_set_PhaseVoltage(vq, vd, p_cl_state->electrical_angle, s_closedloop_params.period, s_closedloop_params.voltage_limit, &duty, inverter);
    p_cl_state->duty_u = duty.duty_u;
    p_cl_state->duty_v = duty.duty_v;
    p_cl_state->duty_w = duty.duty_w;
    
    return ESP_OK;
}

/**
 * @brief 设置控制模式和目标值
 * 
 * @param target 目标值结构体
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t foc_closedloop_set_target(const foc_target_t *target)
{
    if (!s_cl_is_initialized || !p_cl_state) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!target) {
        ESP_LOGW(TAG, "目标值为NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    // 记录当前控制模式
    foc_control_mode_t prev_mode = s_closedloop_params.control_mode;
    
    // 更新控制模式
    s_closedloop_params.control_mode = target->control_mode;
    // 限制目标最大电流
    float max_current = s_closedloop_params.current_limit;
    float target_maxcurrent = target->target_maxcurrent;
    if (target_maxcurrent > max_current) {
        target_maxcurrent = max_current;
        ESP_LOGW(TAG, "目标电流超过电流限制数值，设置为目标电流为限制电流");
    }

    // 根据控制模式设置相应的目标值
    switch (target->control_mode) {
        case FOC_CONTROL_MODE_TORQUE:
            if (target->target_current > target_maxcurrent) {
                ESP_LOGW(TAG, "目标电流超过电流限制");
            }
            // 转矩控制模式：设置q轴电流目标值
            s_closedloop_params.target_iq = target->target_current;
            s_closedloop_params.target_maxcurrent = target_maxcurrent;
            break;
            
        case FOC_CONTROL_MODE_VELOCITY:
            // 速度控制模式：设置速度目标值
            s_closedloop_params.target_velocity = target->target_velocity;
            s_closedloop_params.target_maxcurrent = target_maxcurrent;
            break;
            
        case FOC_CONTROL_MODE_POSITION_SINGLE:
            // 位置控制模式：设置位置目标值
            s_closedloop_params.target_position = target->target_position;
            s_closedloop_params.target_maxcurrent = target_maxcurrent;
            break;
        case FOC_CONTROL_MODE_POSITION_FULL:
            // 位置控制模式：设置位置目标值
            s_closedloop_params.target_position = target->target_position;
            s_closedloop_params.target_maxcurrent = target_maxcurrent;
            break;
        case FOC_CONTROL_MODE_TORQUE_VELOCITY_POSITION:
            // 转矩速度位置控制模式：设置所有目标值
            s_closedloop_params.target_velocity = target->target_velocity;
            s_closedloop_params.target_position = target->target_position;
            s_closedloop_params.target_maxcurrent = target_maxcurrent;
            break;
            
        default:
            ESP_LOGW(TAG, "未知的控制模式: %d", target->control_mode);
            return ESP_ERR_INVALID_ARG;
    }
    
    // 如果控制模式发生变化，重置相关控制器的积分项
    if (prev_mode != target->control_mode) {
        switch (target->control_mode) {
            case FOC_CONTROL_MODE_VELOCITY:
                s_closedloop_params.velocity_pid.integral = 0;
                break;
            case FOC_CONTROL_MODE_TORQUE_VELOCITY_POSITION:
                s_closedloop_params.velocity_pid.integral = 0;
                s_closedloop_params.position_pid.integral = 0;
                s_closedloop_params.target_position = p_cl_state->position_full;
                break;
            case FOC_CONTROL_MODE_POSITION_SINGLE:
                s_closedloop_params.position_single_pid.integral = 0;
                s_closedloop_params.target_position = p_cl_state->position_single;
                break;
            case FOC_CONTROL_MODE_POSITION_FULL:
                s_closedloop_params.position_full_pid.integral = 0;
                s_closedloop_params.target_position = p_cl_state->position_full;
                break;
            default:
                break;
        }
        
        ESP_LOGI(TAG, "控制模式从 %d 切换到 %d", prev_mode, target->control_mode);
    }
    
    // 记录目标值设置
    ESP_LOGD(TAG, "设置控制目标: 模式=%d, 电流=%.3f A, 最大电流=%.3f A, 速度=%.3f rad/s, 位置=%.3f rad",
             target->control_mode, target->target_current, target->target_maxcurrent, target->target_velocity, target->target_position);
    
    return ESP_OK;
}

/**
 * @brief 设置速度和位置
 * 
 * @param velocity 当前速度(rad/s)
 * @param position_single 单圈位置(rad)
 * @param position_full 多圈位置(rad)
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t foc_closedloop_set_motion_state(float velocity, float position_single, float position_full)
{
    // 检查初始化状态
    ESP_RETURN_ON_FALSE(s_cl_is_initialized, ESP_ERR_INVALID_STATE, TAG, "FOC闭环控制未初始化");
    ESP_RETURN_ON_FALSE(p_cl_state, ESP_ERR_INVALID_STATE, TAG, "FOC闭环状态指针为NULL");
    
    // 检查参数是否为NaN
    ESP_RETURN_ON_FALSE(!isnan(velocity), ESP_ERR_INVALID_ARG, TAG, "速度参数为NaN");
    ESP_RETURN_ON_FALSE(!isnan(position_single), ESP_ERR_INVALID_ARG, TAG, "单圈位置参数为NaN");
    ESP_RETURN_ON_FALSE(!isnan(position_full), ESP_ERR_INVALID_ARG, TAG, "多圈位置参数为NaN");
    
    // 更新运动状态
    p_cl_state->velocity = velocity;
    p_cl_state->position_single = position_single;
    p_cl_state->position_full = position_full;
    
    // 记录调试信息
    ESP_LOGD(TAG, "更新运动状态: 速度=%.3f rad/s, 单圈位置=%.3f rad, 多圈位置=%.3f rad",
             velocity, position_single, position_full);
    
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
        case FOC_CONTROL_MODE_POSITION_SINGLE:
            // 对于位置控制，保持当前位置
            s_closedloop_params.target_position = p_cl_state->position_single;
            break;
        case FOC_CONTROL_MODE_POSITION_FULL:
            // 对于位置控制，保持当前位置
            s_closedloop_params.target_position = p_cl_state->position_full;
            break;
        case FOC_CONTROL_MODE_TORQUE_VELOCITY_POSITION:
            // 对于转矩速度位置控制，保持当前位置
            s_closedloop_params.target_position = p_cl_state->position_full;
            break;
    }
    
    ESP_LOGI(TAG, "FOC闭环控制已停止");
    return ESP_OK;
}