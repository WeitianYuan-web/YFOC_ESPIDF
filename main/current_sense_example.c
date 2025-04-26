#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "motor_current_sense.h"
#include "esp_adc/adc_continuous.h"

static const char *TAG = "current_example";

// 定义使用的GPIO引脚
#define CURRENT_SENSE_U_PIN  39  // U相采样引脚 (GPIO39)
#define CURRENT_SENSE_V_PIN  36  // V相采样引脚 (GPIO36)

// 电流采样回调函数
static void current_callback(const current_sense_reading_t *reading, void *user_data)
{
    // 在回调函数中处理电流读数
    ESP_LOGI(TAG, "回调电流读数: U相=%.3fA, V相=%.3fA, W相=%.3fA", 
             reading->current_u, reading->current_v, reading->current_w);
}

// 电流采样任务
void current_sense_task(void *pvParameters)
{
    esp_err_t ret;
    
    // 获取GPIO对应的ADC通道和单元
    adc_unit_t u_unit;
    adc_channel_t u_channel;
    adc_unit_t v_unit;
    adc_channel_t v_channel;
    
    ret = adc_continuous_io_to_channel(CURRENT_SENSE_U_PIN, &u_unit, &u_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO %d不是有效的ADC引脚", CURRENT_SENSE_U_PIN);
        vTaskDelete(NULL);
        return;
    }
    
    ret = adc_continuous_io_to_channel(CURRENT_SENSE_V_PIN, &v_unit, &v_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO %d不是有效的ADC引脚", CURRENT_SENSE_V_PIN);
        vTaskDelete(NULL);
        return;
    }
    
    // 配置电流采样模块
    current_sense_config_t config = {
        .u_phase_gpio = CURRENT_SENSE_U_PIN,
        .v_phase_gpio = CURRENT_SENSE_V_PIN,
        .u_phase_unit = u_unit,
        .u_phase_channel = u_channel,
        .v_phase_unit = v_unit,
        .v_phase_channel = v_channel,
        .sample_freq_hz = 10000,          // 10kHz采样频率
        .zero_current_adc_u = 2048.0f,    // 默认零点值
        .zero_current_adc_v = 2048.0f,    // 默认零点值
        .use_w_calculation = true         // 通过计算得到W相电流
    };
    
    // 初始化电流采样模块
    ESP_LOGI(TAG, "初始化电流采样模块");
    ret = current_sense_init(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "初始化电流采样模块失败: %d", ret);
        vTaskDelete(NULL);
        return;
    }
    
    // 注册回调函数（可选）
    current_sense_register_callback(current_callback, NULL);
    
    // 电机未通电时校准零点
    ESP_LOGI(TAG, "开始电流传感器零点校准, 请确保电机未通电");
    vTaskDelay(pdMS_TO_TICKS(1000));  // 等待1秒钟
    ret = current_sense_calibrate(100);  // 采集100个样本进行校准
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "校准电流传感器失败: %d", ret);
        current_sense_deinit();
        vTaskDelete(NULL);
        return;
    }
    
    // 启动ADC连续转换
    ESP_LOGI(TAG, "启动ADC连续转换");
    ret = current_sense_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "启动电流采样失败: %d", ret);
        current_sense_deinit();
        vTaskDelete(NULL);
        return;
    }
    
    current_sense_reading_t reading;
    
    // 循环读取电流
    while (1) {
        // 从ADC读取电流值（使用1000ms超时）
        ret = current_sense_read(&reading, 1000);
        if (ret == ESP_OK) {
            // 打印电流值
            ESP_LOGI(TAG, "电流读数: U相=%.3fA, V相=%.3fA, W相=%.3fA, U-ADC=%d, V-ADC=%d", 
                     reading.current_u, reading.current_v, reading.current_w,
                     reading.adc_u_raw, reading.adc_v_raw);
        } else if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "读取ADC数据超时");
        } else {
            ESP_LOGE(TAG, "读取电流值失败: %d", ret);
        }
        
        // 延时100ms
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // 以下代码不会执行到，除非在循环中添加退出条件
    ESP_LOGI(TAG, "停止电流采样");
    current_sense_stop();
    current_sense_deinit();
}

// 示例应用入口函数
void app_current_sense_example(void)
{
    ESP_LOGI(TAG, "启动电流采样示例");
    
    // 创建电流采样任务
    xTaskCreate(current_sense_task, "current_sense_task", 4096, NULL, 5, NULL);
} 