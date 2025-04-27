/*
 * UART命令解析库
 * 用于解析格式为 "a:553" 的串口命令
 */
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "uart_command.h"

static const char *TAG = "uart_cmd";

// 内部状态
typedef struct {
    uart_port_t uart_port;
    uint32_t rx_buffer_size;
    uint8_t *rx_buffer;
    char cmd_buffer[32];
    int cmd_pos;
    uart_cmd_callback_t callback;
    void* user_data;
    bool initialized;
} uart_cmd_ctx_t;

// 全局上下文
static uart_cmd_ctx_t g_uart_cmd_ctx = {
    .uart_port = UART_NUM_0,
    .rx_buffer_size = 0,
    .rx_buffer = NULL,
    .cmd_pos = 0,
    .callback = NULL,
    .user_data = NULL,
    .initialized = false
};

esp_err_t uart_cmd_init(const uart_cmd_config_t* config)
{
    // 检查参数
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 检查是否已初始化
    if (g_uart_cmd_ctx.initialized) {
        ESP_LOGW(TAG, "UART命令处理器已初始化");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 分配接收缓冲区
    g_uart_cmd_ctx.rx_buffer = (uint8_t*)malloc(config->rx_buffer_size);
    if (g_uart_cmd_ctx.rx_buffer == NULL) {
        ESP_LOGE(TAG, "无法分配接收缓冲区");
        return ESP_ERR_NO_MEM;
    }
    
    // 初始化上下文
    g_uart_cmd_ctx.uart_port = config->uart_port;
    g_uart_cmd_ctx.rx_buffer_size = config->rx_buffer_size;
    g_uart_cmd_ctx.callback = config->callback;
    g_uart_cmd_ctx.user_data = config->user_data;
    g_uart_cmd_ctx.cmd_pos = 0;
    memset(g_uart_cmd_ctx.cmd_buffer, 0, sizeof(g_uart_cmd_ctx.cmd_buffer));
    
    // 标记为已初始化
    g_uart_cmd_ctx.initialized = true;
    
    ESP_LOGI(TAG, "UART命令处理器初始化完成，端口:%d, 缓冲区大小:%lu", 
             (int)config->uart_port, (unsigned long)config->rx_buffer_size);
    
    return ESP_OK;
}

esp_err_t uart_cmd_process(uint32_t timeout_ms)
{
    // 检查是否已初始化
    if (!g_uart_cmd_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // 从UART读取数据
    int len = uart_read_bytes(g_uart_cmd_ctx.uart_port, 
                             g_uart_cmd_ctx.rx_buffer, 
                             g_uart_cmd_ctx.rx_buffer_size, 
                             pdMS_TO_TICKS(timeout_ms));
    
    // 如果没有数据，返回超时
    if (len <= 0) {
        return ESP_ERR_TIMEOUT;
    }
    
    // 处理接收到的数据
    for (int i = 0; i < len; i++) {
        // 检测命令结束符（回车或换行）
        if (g_uart_cmd_ctx.rx_buffer[i] == '\r' || g_uart_cmd_ctx.rx_buffer[i] == '\n') {
            // 如果有数据，处理命令
            if (g_uart_cmd_ctx.cmd_pos > 0) {
                // 添加字符串结束符
                g_uart_cmd_ctx.cmd_buffer[g_uart_cmd_ctx.cmd_pos] = '\0';
                
                // 解析命令，格式为 "a:553"
                if (g_uart_cmd_ctx.cmd_pos >= 3 && g_uart_cmd_ctx.cmd_buffer[1] == ':') {
                    char cmd_type = g_uart_cmd_ctx.cmd_buffer[0];
                    int value = atoi(&g_uart_cmd_ctx.cmd_buffer[2]);
                    
                    ESP_LOGI(TAG, "接收到命令: %c:%d", cmd_type, value);
                    
                    // 调用回调函数
                    if (g_uart_cmd_ctx.callback != NULL) {
                        g_uart_cmd_ctx.callback((uart_cmd_type_t)cmd_type, value, g_uart_cmd_ctx.user_data);
                    }
                } else {
                    ESP_LOGW(TAG, "无效命令格式: %s", g_uart_cmd_ctx.cmd_buffer);
                }
                
                // 重置命令缓冲区
                g_uart_cmd_ctx.cmd_pos = 0;
            }
        } else if (g_uart_cmd_ctx.cmd_pos < sizeof(g_uart_cmd_ctx.cmd_buffer) - 1) {
            // 将字符添加到命令缓冲区
            g_uart_cmd_ctx.cmd_buffer[g_uart_cmd_ctx.cmd_pos++] = g_uart_cmd_ctx.rx_buffer[i];
        }
    }
    
    return ESP_OK;
}

esp_err_t uart_cmd_set_callback(uart_cmd_callback_t callback, void* user_data)
{
    // 检查是否已初始化
    if (!g_uart_cmd_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // 设置回调函数
    g_uart_cmd_ctx.callback = callback;
    g_uart_cmd_ctx.user_data = user_data;
    
    return ESP_OK;
}

esp_err_t uart_cmd_deinit(void)
{
    // 检查是否已初始化
    if (!g_uart_cmd_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // 释放资源
    if (g_uart_cmd_ctx.rx_buffer != NULL) {
        free(g_uart_cmd_ctx.rx_buffer);
        g_uart_cmd_ctx.rx_buffer = NULL;
    }
    
    // 重置状态
    g_uart_cmd_ctx.initialized = false;
    g_uart_cmd_ctx.cmd_pos = 0;
    g_uart_cmd_ctx.callback = NULL;
    g_uart_cmd_ctx.user_data = NULL;
    
    ESP_LOGI(TAG, "UART命令处理器已释放");
    
    return ESP_OK;
} 