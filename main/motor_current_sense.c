/*
 * 电机三相电流采样驱动实现
 * 使用ESP32 ADC单次转换模式
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "motor_current_sense.h"

static const char *TAG = "current_sense";

// 默认ADC配置
#define ADC_BITWIDTH         SOC_ADC_DIGI_MAX_BITWIDTH  // 使用SOC支持的最大位宽
#define ADC_ATTEN            ADC_ATTEN_DB_12            // 12dB衰减，0-3.3V输入范围

// 全局变量
static current_sense_config_t g_config;
static adc_oneshot_unit_handle_t g_adc_handle = NULL;
static bool g_is_initialized = false;
static bool g_is_running = false;

// 添加全局当前读数缓冲区
static current_sense_reading_t g_current_reading = {0};

// 零点校准值
static float g_zero_adc_u = 2048.0f;  // 默认值为12位ADC中点
static float g_zero_adc_v = 2048.0f;  // 默认值为12位ADC中点

// 回调函数相关
static current_sense_callback_t g_user_callback = NULL;
static void *g_user_data = NULL;

// 添加直接ADC采样函数
esp_err_t current_sense_sample(current_sense_reading_t *reading)
{
    if (!g_is_initialized) {
        ESP_LOGE(TAG, "电流采样模块未初始化");
        return ESP_ERR_INVALID_STATE;
    }

    if (!g_is_running) {
        ESP_LOGE(TAG, "电流采样模块未启动");
        return ESP_ERR_INVALID_STATE;
    }

    current_sense_reading_t local_reading = {0};
    esp_err_t ret;

    // 采样U相
    int adc_u_raw;
    ret = adc_oneshot_read(g_adc_handle, g_config.u_phase_channel, &adc_u_raw);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "读取U相ADC数据失败: %d", ret);
        return ret;
    }
    local_reading.adc_u_raw = adc_u_raw;
    
    // 采样V相
    int adc_v_raw;
    ret = adc_oneshot_read(g_adc_handle, g_config.v_phase_channel, &adc_v_raw);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "读取V相ADC数据失败: %d", ret);
        return ret;
    }
    local_reading.adc_v_raw = adc_v_raw;
    
    // 转换为电流值
    float vref = 3.3f;
    float max_adc = (1 << ADC_BITWIDTH) - 1;
    
    float adc_diff_u = (float)local_reading.adc_u_raw - g_zero_adc_u;
    local_reading.current_u = adc_diff_u * vref / max_adc / (CURRENT_SENSE_GAIN * CURRENT_SENSE_SHUNT_RESISTANCE);
    
    float adc_diff_v = (float)local_reading.adc_v_raw - g_zero_adc_v;
    local_reading.current_v = adc_diff_v * vref / max_adc / (CURRENT_SENSE_GAIN * CURRENT_SENSE_SHUNT_RESISTANCE);
    
    if (g_config.use_w_calculation) {
        local_reading.current_w = -(local_reading.current_u + local_reading.current_v);
    }
    
    // 更新全局缓冲区
    memcpy(&g_current_reading, &local_reading, sizeof(current_sense_reading_t));
    
    // 如果有输出参数，复制结果
    if (reading) {
        memcpy(reading, &local_reading, sizeof(current_sense_reading_t));
    }
    
    // 如果注册了回调函数，执行回调
    if (g_user_callback) {
        g_user_callback(&local_reading, g_user_data);
    }
    
    return ESP_OK;
}

esp_err_t current_sense_init(const current_sense_config_t *config)
{
    if (g_is_initialized) {
        ESP_LOGW(TAG, "电流采样模块已初始化");
        return ESP_OK;
    }

    if (!config) {
        ESP_LOGE(TAG, "无效的配置参数");
        return ESP_ERR_INVALID_ARG;
    }

    // 保存配置
    memcpy(&g_config, config, sizeof(current_sense_config_t));

    // 如果未设置采样频率，使用默认值
    if (g_config.sample_freq_hz == 0) {
        g_config.sample_freq_hz = CURRENT_SENSE_SAMPLE_FREQ;
    }

    // 初始化零点值
    g_zero_adc_u = config->zero_current_adc_u;
    g_zero_adc_v = config->zero_current_adc_v;

    // 创建ADC单次转换句柄
    adc_oneshot_unit_init_cfg_t adc_config = {
        .unit_id = g_config.u_phase_unit, // 使用U相的ADC单元
    };
    
    esp_err_t ret = adc_oneshot_new_unit(&adc_config, &g_adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "创建ADC单次转换句柄失败: %d", ret);
        return ret;
    }

    // 配置ADC通道
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH,
    };
    
    // 配置U相通道
    ret = adc_oneshot_config_channel(g_adc_handle, g_config.u_phase_channel, &chan_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "配置U相ADC通道失败: %d", ret);
        adc_oneshot_del_unit(g_adc_handle);
        g_adc_handle = NULL;
        return ret;
    }
    
    // 配置V相通道
    ret = adc_oneshot_config_channel(g_adc_handle, g_config.v_phase_channel, &chan_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "配置V相ADC通道失败: %d", ret);
        adc_oneshot_del_unit(g_adc_handle);
        g_adc_handle = NULL;
        return ret;
    }

    g_is_initialized = true;
    ESP_LOGI(TAG, "电流采样模块初始化成功，采样频率: %lu Hz", (unsigned long)g_config.sample_freq_hz);
    return ESP_OK;
}

esp_err_t current_sense_start(void)
{
    if (!g_is_initialized) {
        ESP_LOGE(TAG, "电流采样模块未初始化");
        return ESP_ERR_INVALID_STATE;
    }

    if (g_is_running) {
        ESP_LOGW(TAG, "电流采样模块已在运行");
        return ESP_OK;
    }

    // 直接设置运行标志，不创建任务
    g_is_running = true;
    ESP_LOGI(TAG, "电流采样模块已启动");
    return ESP_OK;
}

esp_err_t current_sense_stop(void)
{
    if (!g_is_initialized) {
        ESP_LOGE(TAG, "电流采样模块未初始化");
        return ESP_ERR_INVALID_STATE;
    }

    if (!g_is_running) {
        ESP_LOGW(TAG, "电流采样模块未运行");
        return ESP_OK;
    }

    // 清除运行标志
    g_is_running = false;
    ESP_LOGI(TAG, "电流采样模块已停止");
    return ESP_OK;
}

esp_err_t current_sense_calibrate(uint32_t samples)
{
    if (!g_is_initialized) {
        ESP_LOGE(TAG, "电流采样模块未初始化");
        return ESP_ERR_INVALID_STATE;
    }

    if (g_is_running) {
        ESP_LOGE(TAG, "校准前请先停止电流采样");
        return ESP_ERR_INVALID_STATE;
    }

    if (samples == 0) {
        samples = 100; // 默认采集100个样本
    }

    ESP_LOGI(TAG, "开始电流传感器零点校准，采样次数: %lu", (unsigned long)samples);
    
    float sum_u = 0;
    float sum_v = 0;
    uint32_t count_u = 0;
    uint32_t count_v = 0;
    
    // 采集多个样本并求平均值
    for (uint32_t i = 0; i < samples; i++) {
        int adc_u_raw, adc_v_raw;
        esp_err_t ret;
        
        // 读取U相ADC
        ret = adc_oneshot_read(g_adc_handle, g_config.u_phase_channel, &adc_u_raw);
        if (ret == ESP_OK) {
            sum_u += adc_u_raw;
            count_u++;
        } else {
            ESP_LOGE(TAG, "读取U相ADC数据失败: %d", ret);
        }
        
        // 读取V相ADC
        ret = adc_oneshot_read(g_adc_handle, g_config.v_phase_channel, &adc_v_raw);
        if (ret == ESP_OK) {
            sum_v += adc_v_raw;
            count_v++;
        } else {
            ESP_LOGE(TAG, "读取V相ADC数据失败: %d", ret);
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // 短暂延时
    }
    
    // 计算平均值
    if (count_u > 0) {
        g_zero_adc_u = sum_u / count_u;
    }
    
    if (count_v > 0) {
        g_zero_adc_v = sum_v / count_v;
    }
    
    ESP_LOGI(TAG, "零点校准完成，U相: %.2f (%lu samples), V相: %.2f (%lu samples)", 
             g_zero_adc_u, (unsigned long)count_u, g_zero_adc_v, (unsigned long)count_v);
    
    return ESP_OK;
}

esp_err_t current_sense_read(current_sense_reading_t *reading, uint32_t timeout_ms)
{
    if (!g_is_initialized) {
        ESP_LOGE(TAG, "电流采样模块未初始化");
        return ESP_ERR_INVALID_STATE;
    }

    if (!g_is_running) {
        ESP_LOGE(TAG, "电流采样模块未启动");
        return ESP_ERR_INVALID_STATE;
    }

    if (!reading) {
        ESP_LOGE(TAG, "无效的读数指针");
        return ESP_ERR_INVALID_ARG;
    }
    
    // 直接返回ADC任务中最新处理的数据
    memcpy(reading, &g_current_reading, sizeof(current_sense_reading_t));
    
    return ESP_OK;
}

esp_err_t current_sense_register_callback(current_sense_callback_t callback, void *user_data)
{
    if (!g_is_initialized) {
        ESP_LOGE(TAG, "电流采样模块未初始化");
        return ESP_ERR_INVALID_STATE;
    }
    
    g_user_callback = callback;
    g_user_data = user_data;
    
    return ESP_OK;
}

esp_err_t current_sense_deinit(void)
{
    if (!g_is_initialized) {
        ESP_LOGW(TAG, "电流采样模块未初始化");
        return ESP_OK;
    }

    if (g_is_running) {
        ESP_LOGI(TAG, "先停止运行中的电流采样");
        current_sense_stop();
    }

    // 释放ADC单次转换句柄
    if (g_adc_handle) {
        adc_oneshot_del_unit(g_adc_handle);
        g_adc_handle = NULL;
    }

    g_is_initialized = false;
    g_user_callback = NULL;
    g_user_data = NULL;
    
    ESP_LOGI(TAG, "电流采样模块已释放");
    
    return ESP_OK;
} 