/*
 * 电机三相电流采样驱动
 * 使用ESP32 ADC连续转换模式
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_adc/adc_continuous.h"
#include "soc/soc_caps.h"

#ifdef __cplusplus
extern "C" {
#endif

// 电流采样相关常量
#define CURRENT_SENSE_SHUNT_RESISTANCE  0.01f  // 10毫欧采样电阻
#define CURRENT_SENSE_GAIN              50.0f  // 50倍放大倍数

// ADC连续转换配置常量
#define CURRENT_SENSE_SAMPLE_FREQ       40000  // 采样频率 40KHz
#define CURRENT_SENSE_FRAME_SIZE        512    // 单次转换帧大小(字节)
#define CURRENT_SENSE_MAX_BUFFER_SIZE   4096   // 最大缓冲区大小(字节)

// 电流传感器配置结构体
typedef struct {
    int u_phase_gpio;              // U相GPIO引脚
    int v_phase_gpio;              // V相GPIO引脚
    adc_unit_t u_phase_unit;       // U相ADC单元
    adc_channel_t u_phase_channel; // U相ADC通道
    adc_unit_t v_phase_unit;       // V相ADC单元
    adc_channel_t v_phase_channel; // V相ADC通道
    uint32_t sample_freq_hz;       // 采样频率
    float zero_current_adc_u;      // U相零电流ADC值
    float zero_current_adc_v;      // V相零电流ADC值
    bool use_w_calculation;        // 是否通过计算得到W相电流
} current_sense_config_t;

// 电流读数结构体
typedef struct {
    float current_u;    // U相电流(安培)
    float current_v;    // V相电流(安培)
    float current_w;    // W相电流(安培)
    uint32_t adc_u_raw; // U相ADC原始值
    uint32_t adc_v_raw; // V相ADC原始值
} current_sense_reading_t;

/**
 * @brief 初始化电机电流采样
 * 
 * @param config 电流采样配置
 * @return esp_err_t 初始化结果
 */
esp_err_t current_sense_init(const current_sense_config_t *config);

/**
 * @brief 启动电流采样
 * 
 * @return esp_err_t 启动结果
 */
esp_err_t current_sense_start(void);

/**
 * @brief 停止电流采样
 * 
 * @return esp_err_t 停止结果
 */
esp_err_t current_sense_stop(void);

/**
 * @brief 校准电流传感器零点
 * 
 * @param samples 校准采样次数
 * @return esp_err_t 校准结果
 */
esp_err_t current_sense_calibrate(uint32_t samples);

/**
 * @brief 读取三相电流
 * 
 * @param reading 电流读数结果
 * @param timeout_ms 超时时间(毫秒)
 * @return esp_err_t 读取结果
 */
esp_err_t current_sense_read(current_sense_reading_t *reading, uint32_t timeout_ms);

/**
 * @brief 反初始化电流采样模块
 * 
 * @return esp_err_t 结果
 */
esp_err_t current_sense_deinit(void);

/**
 * @brief 注册回调函数，当ADC有新数据时调用
 *
 * @param callback 回调函数指针
 * @param user_data 用户数据
 * @return esp_err_t 结果
 */
typedef void (*current_sense_callback_t)(const current_sense_reading_t *reading, void *user_data);
esp_err_t current_sense_register_callback(current_sense_callback_t callback, void *user_data);

#ifdef __cplusplus
}
#endif 