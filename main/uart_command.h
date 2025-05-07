/*
 * UART命令解析库
 * 用于解析格式为 "a:553" 的串口命令
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 命令类型
 */
typedef enum {
    UART_CMD_TYPE_A = 'a',  /*!< A类型命令 */
    UART_CMD_TYPE_B = 'b',  /*!< B类型命令 */
    UART_CMD_TYPE_V = 'v',  /*!< V类型命令 */
    UART_CMD_TYPE_C = 'c',  /*!< C类型命令 */
    UART_CMD_TYPE_D = 'd',  /*!< D类型命令 */
    UART_CMD_TYPE_E = 'e',  /*!< E类型命令 */
    // 可在此处添加更多命令类型
} uart_cmd_type_t;

/**
 * @brief 命令回调函数类型
 * 
 * @param cmd_type 命令类型
 * @param value 命令值
 * @param user_data 用户数据
 */
typedef void (*uart_cmd_callback_t)(uart_cmd_type_t cmd_type, int value, void* user_data);

/**
 * @brief UART命令处理器配置
 */
typedef struct {
    uart_port_t uart_port;       /*!< UART端口号 */
    uint32_t rx_buffer_size;     /*!< 接收缓冲区大小 */
    uart_cmd_callback_t callback; /*!< 命令回调函数 */
    void* user_data;             /*!< 用户数据，会传递给回调函数 */
} uart_cmd_config_t;

/**
 * @brief 初始化UART命令处理器
 * 
 * @param config 配置参数
 * @return esp_err_t ESP_OK: 成功; 其他: 失败
 */
esp_err_t uart_cmd_init(const uart_cmd_config_t* config);

/**
 * @brief 读取并处理UART命令
 * 
 * 此函数应该周期性地被调用，例如在任务循环中
 * 
 * @param timeout_ms 超时时间(毫秒)
 * @return esp_err_t ESP_OK: 成功读取并处理了命令; 
 *                   ESP_ERR_TIMEOUT: 超时未收到数据; 
 *                   其他: 发生错误
 */
esp_err_t uart_cmd_process(uint32_t timeout_ms);

/**
 * @brief 设置命令回调函数
 * 
 * @param callback 回调函数
 * @param user_data 用户数据
 * @return esp_err_t ESP_OK: 成功; 其他: 失败
 */
esp_err_t uart_cmd_set_callback(uart_cmd_callback_t callback, void* user_data);

/**
 * @brief 反初始化UART命令处理器
 * 
 * @return esp_err_t ESP_OK: 成功; 其他: 失败
 */
esp_err_t uart_cmd_deinit(void);

#ifdef __cplusplus
}
#endif 