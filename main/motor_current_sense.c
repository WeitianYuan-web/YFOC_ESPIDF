/*
 * 电机三相电流采样驱动实现
 * 使用ESP32 ADC连续转换模式
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "esp_adc/adc_continuous.h"
#include "motor_current_sense.h"

static const char *TAG = "current_sense";

// 默认ADC配置
#define ADC_BITWIDTH         SOC_ADC_DIGI_MAX_BITWIDTH  // 使用SOC支持的最大位宽
#define ADC_ATTEN            ADC_ATTEN_DB_12            // 12dB衰减，0-3.3V输入范围
#define ADC_CONV_MODE        ADC_CONV_SINGLE_UNIT_1     // 单ADC模式
#define ADC_OUTPUT_TYPE      ADC_DIGI_OUTPUT_FORMAT_TYPE1 // 输出格式

// 全局变量
static current_sense_config_t g_config;
static adc_continuous_handle_t g_adc_handle = NULL;
static bool g_is_initialized = false;
static bool g_is_running = false;

// 添加全局当前读数缓冲区
static current_sense_reading_t g_current_reading = {0};

// 零点校准值
static float g_zero_adc_u = 2048.0f;  // 默认值为12位ADC中点
static float g_zero_adc_v = 2048.0f;  // 默认值为12位ADC中点

// ADC原始数据读取缓冲区
static uint8_t g_adc_raw_buffer[CURRENT_SENSE_FRAME_SIZE];

// 回调函数相关
static current_sense_callback_t g_user_callback = NULL;
static void *g_user_data = NULL;

// 在motor_current_sense.c中添加必要的任务控制变量
static TaskHandle_t adc_task_handle = NULL;
static bool adc_task_running = false;

// 修改回调函数，使用任务通知机制
static bool adc_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    
    // 检查缓冲区大小但不存储长度
    if (edata->size > sizeof(g_adc_raw_buffer)) {
        ESP_LOGW(TAG, "缓冲区过小，需要更大的缓冲区");
        return false;
    }
    
    // 只复制数据
    memcpy(g_adc_raw_buffer, edata->conv_frame_buffer, edata->size);
    
    // 通知ADC任务处理数据
    vTaskNotifyGiveFromISR(adc_task_handle, &mustYield);
    
    return (mustYield == pdTRUE);
}

// 创建ADC处理任务函数
static void adc_process_task(void *arg)
{
    uint32_t ret_num = 0;
    uint8_t result[CURRENT_SENSE_FRAME_SIZE] = {0};
    esp_err_t ret;
    current_sense_reading_t local_reading = {0};
    
    while (adc_task_running) {
        // 等待ADC转换完成通知
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        // 累计ADC数据
        uint32_t count_u = 0;
        uint32_t count_v = 0;
        uint32_t sum_u = 0;
        uint32_t sum_v = 0;
        
        // 循环读取所有可用数据
        while (1) {
            ret = adc_continuous_read(g_adc_handle, result, CURRENT_SENSE_FRAME_SIZE, &ret_num, 0);
            if (ret == ESP_OK) {
                // 解析数据
                for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                    adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
                    uint32_t chan_num = p->type1.channel;
                    uint32_t data = p->type1.data;
                    
                    if (chan_num == g_config.u_phase_channel) {
                        sum_u += data;
                        count_u++;
                    } else if (chan_num == g_config.v_phase_channel) {
                        sum_v += data;
                        count_v++;
                    }
                }
            } else if (ret == ESP_ERR_TIMEOUT) {
                // 所有数据已读取
                break;
            } else {
                ESP_LOGE(TAG, "读取ADC数据失败: %d", ret);
                break;
            }
        }
        
        // 计算平均值
        if (count_u > 0) {
            local_reading.adc_u_raw = sum_u / count_u;
        }
        
        if (count_v > 0) {
            local_reading.adc_v_raw = sum_v / count_v;
        }
        
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
        
        // 如果注册了回调函数，执行回调
        if (g_user_callback) {
            g_user_callback(&local_reading, g_user_data);
        }
    }
    
    vTaskDelete(NULL);
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

    // 创建ADC连续转换句柄
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = CURRENT_SENSE_MAX_BUFFER_SIZE,
        .conv_frame_size = CURRENT_SENSE_FRAME_SIZE,
        .flags.flush_pool = true,
    };
    
    esp_err_t ret = adc_continuous_new_handle(&adc_config, &g_adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "创建ADC连续转换句柄失败: %d", ret);
        return ret;
    }

    // 配置ADC通道
    adc_digi_pattern_config_t adc_pattern[2];
    adc_pattern[0].atten = ADC_ATTEN; // U相
    adc_pattern[0].channel = g_config.u_phase_channel;
    adc_pattern[0].unit = g_config.u_phase_unit;
    adc_pattern[0].bit_width = ADC_BITWIDTH;
    
    adc_pattern[1].atten = ADC_ATTEN; // V相
    adc_pattern[1].channel = g_config.v_phase_channel;
    adc_pattern[1].unit = g_config.v_phase_unit;
    adc_pattern[1].bit_width = ADC_BITWIDTH;

    adc_continuous_config_t dig_cfg = {
        .pattern_num = 2,   // 两个通道
        .adc_pattern = adc_pattern,
        .sample_freq_hz = g_config.sample_freq_hz,
        .conv_mode = ADC_CONV_MODE,
        .format = ADC_OUTPUT_TYPE,
    };
    
    ret = adc_continuous_config(g_adc_handle, &dig_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "配置ADC连续转换失败: %d", ret);
        adc_continuous_deinit(g_adc_handle);
        g_adc_handle = NULL;
        return ret;
    }

    // 注册ADC转换完成回调
    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = adc_conv_done_cb,
    };
    
    ret = adc_continuous_register_event_callbacks(g_adc_handle, &cbs, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "注册ADC事件回调失败: %d", ret);
        adc_continuous_deinit(g_adc_handle);
        g_adc_handle = NULL;
        return ret;
    }

    // 创建ADC处理任务
    adc_task_running = true;
    xTaskCreate(adc_process_task, "adc_task", 4096, NULL, 5, &adc_task_handle);
    
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

    // 启动ADC连续转换
    esp_err_t ret = adc_continuous_start(g_adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "启动ADC连续转换失败: %d", ret);
        return ret;
    }

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

    // 停止ADC连续转换
    esp_err_t ret = adc_continuous_stop(g_adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "停止ADC连续转换失败: %d", ret);
        return ret;
    }

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
    
    // 暂时启动ADC进行校准
    esp_err_t ret = adc_continuous_start(g_adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "启动ADC连续转换失败: %d", ret);
        return ret;
    }
    
    float sum_u = 0;
    float sum_v = 0;
    uint32_t count_u = 0;
    uint32_t count_v = 0;
    
    // 采集多个样本并求平均值
    for (uint32_t i = 0; i < samples; i++) {
        uint32_t read_len = 0;
        ret = adc_continuous_read(g_adc_handle, g_adc_raw_buffer, CURRENT_SENSE_FRAME_SIZE, &read_len, 100);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "读取ADC数据失败: %d", ret);
            continue;
        }
        
        // 解析ADC数据
        for (int j = 0; j < read_len; j += SOC_ADC_DIGI_RESULT_BYTES) {
            adc_digi_output_data_t *data = (adc_digi_output_data_t*)&g_adc_raw_buffer[j];
            if (data->type1.channel == g_config.u_phase_channel) {
                sum_u += data->type1.data;
                count_u++;
            } else if (data->type1.channel == g_config.v_phase_channel) {
                sum_v += data->type1.data;
                count_v++;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // 短暂延时
    }

    // 停止ADC
    adc_continuous_stop(g_adc_handle);
    
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
    g_user_data = user_data;  // 可以是TaskHandle_t
    
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
    
    // 停止ADC任务
    if (adc_task_handle != NULL) {
        adc_task_running = false;
        vTaskDelay(pdMS_TO_TICKS(100)); // 等待任务结束
    }

    // 释放ADC连续转换句柄
    if (g_adc_handle) {
        adc_continuous_deinit(g_adc_handle);
        g_adc_handle = NULL;
    }

    g_is_initialized = false;
    g_user_callback = NULL;
    g_user_data = NULL;
    
    ESP_LOGI(TAG, "电流采样模块已释放");
    
    return ESP_OK;
} 