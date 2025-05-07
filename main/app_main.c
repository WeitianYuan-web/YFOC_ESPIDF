/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_foc.h"
#include "foc_control.h"
#include "driver/uart.h"
#include "as5600.h"
#include "motor_current_sense.h"   // 添加电流采样库头文件
#include "uart_command.h"
#include "motor_config.h"  // 添加电机配置头文件
#include "esp_task_wdt.h"
#include "esp_adc/adc_oneshot.h"  // 添加ADC单次转换头文件

#define _2PI        6.28318530718f
#define _PI         3.14159265359f
#define _2PI_7      0.89759790102f
#define _3PI_2      4.71238898038f
// 选择FOC控制模式：FOC_CONTROL_OPENLOOP为开环控制，FOC_CONTROL_CLOSEDLOOP为闭环控制
#define FOC_CONTROL_MODE_SELECT    FOC_CONTROL_CLOSEDLOOP
#define FOC_CONTROL_MODE_DEBUG     1
// 控制模式定义
#define FOC_CONTROL_OPENLOOP       0
#define FOC_CONTROL_CLOSEDLOOP     1
 
static const char *TAG = "example_foc";

//////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////// Please update the following configuration according to your HardWare spec /////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EXAMPLE_FOC_DRV_EN_GPIO          12
#define EXAMPLE_FOC_PWM_U_GPIO           33
#define EXAMPLE_FOC_PWM_V_GPIO           32
#define EXAMPLE_FOC_PWM_W_GPIO           25

// UART配置
#define UART_NUM UART_NUM_0
#define UART_TXD 1
#define UART_RXD 3
#define UART_BAUD_RATE 115200

#define EXAMPLE_FOC_MCPWM_TIMER_RESOLUTION_HZ 20000000 // 20MHz, 1 tick = 0.05us
#define EXAMPLE_FOC_MCPWM_PERIOD              2000     // 2000 * 0.05us = 100us, 10KHz

// 电流采样引脚配置
#define CURRENT_SENSE_U_PIN 36  
#define CURRENT_SENSE_V_PIN 39  

// 获取当前电机配置
static const motor_config_t* motor_config;

// 使用电机配置中的参数替换原有的宏定义
#define EXAMPLE_MOTOR_POLE_PAIRS          (motor_config->pole_pairs)
#define EXAMPLE_MOTOR_RESISTANCE          (motor_config->resistance)
#define EXAMPLE_MOTOR_SUPPLY_VOLTAGE      (motor_config->supply_voltage)
#define EXAMPLE_MOTOR_KV                  (motor_config->kv)
#define EXAMPLE_MOTOR_DIRECTION           (motor_config->direction)
#define EXAMPLE_MOTOR_CURRENT_LIMIT       (motor_config->current_limit)
#define EXAMPLE_MOTOR_MAX_VOLTAGE         (motor_config->max_voltage)
#define EXAMPLE_MOTOR_OPENLOOP_RPM        (motor_config->openloop_rpm)

// AS5600编码器配置
#define AS5600_SDA_PIN      19      // SDA引脚
#define AS5600_SCL_PIN      18      // SCL引脚
#define AS5600_I2C_FREQ     1000000  // 1MHz
#define AS5600_I2C_PORT     I2C_NUM_0  // 使用I2C_0端口

void bsp_bridge_driver_init(void)
{
    gpio_config_t drv_en_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_FOC_DRV_EN_GPIO,
    };
    ESP_ERROR_CHECK(gpio_config(&drv_en_config));
}

void bsp_bridge_driver_enable(bool enable)
{
    ESP_LOGI(TAG, "%s MOSFET gate", enable ? "Enable" : "Disable");
    gpio_set_level(EXAMPLE_FOC_DRV_EN_GPIO, enable);
}

bool inverter_update_cb(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t *edata, void *user_ctx)
{
    BaseType_t task_yield = pdFALSE;
    xSemaphoreGiveFromISR(*((SemaphoreHandle_t *)user_ctx), &task_yield);
    return task_yield;
}

uint32_t last_update_time = 0;
uint32_t dt = 0;

#if FOC_CONTROL_MODE_DEBUG == 1  // 修正拼写错误
// Debug模式配置
#define FOC_TASK_STACK_SIZE     8192
#define FOC_TASK_PRIORITY       5
#define FOC_CONTROL_FREQ_HZ     2000  // 2kHz控制频率
#define FOC_TASK_CORE_ID        1
#else
// 非Debug模式配置
#define FOC_TASK_STACK_SIZE     8192
#define FOC_TASK_PRIORITY       5
#define FOC_CONTROL_FREQ_HZ     8000  // 8kHz控制频率
#define FOC_TASK_CORE_ID        1
#endif
// FOC控制任务全局变量
static TaskHandle_t foc_task_handle = NULL;
static SemaphoreHandle_t foc_timer_semaphore = NULL;
static esp_timer_handle_t foc_timer_handle = NULL;

// 定义电流采样变量
static current_sense_reading_t g_current_reading;
static bool g_current_sense_initialized = false;

// FOC控制定时器回调函数
static void foc_timer_callback(void* arg)
{
    // 通知FOC任务执行控制循环
    xSemaphoreGiveFromISR(foc_timer_semaphore, NULL);
}

// 定义串口任务常量
#define UART_TASK_STACK_SIZE    4096
#define UART_TASK_PRIORITY      3     // 优先级低于FOC任务
#define UART_TASK_CORE_ID       0     // 在核心0上运行
#define UART_QUEUE_SIZE         10    // 队列大小

// 串口数据包结构
typedef struct {
    uint64_t timestamp;
    float duty_u;
    float duty_v;
    float duty_w;
    float speed_rpm;
    float speed_rad;
    float voltage;
    float encoder_angle;     // 编码器角度(弧度)
    float encoder_speed;     // 编码器速度(RPM)
    float encoder_electrical_angle;     // 编码器电角度(弧度)
    int32_t total_angle_raw; // 总角度原始值
    float total_angle_rad;   // 总角度(弧度)
    int32_t full_rotations;  // 完整旋转圈数
    float current_u;         // U相电流(安培)
    float current_v;         // V相电流(安培)
    float current_w;         // W相电流(安培)
    float d_current;         // d轴电流(安培)
    float q_current;         // q轴电流(安培)
    float target_current;    // 目标电流(安培)
    float target_velocity;   // 目标速度(RPM)
    float target_position;   // 目标位置(弧度)
    float target_maxcurrent; // 目标最大电流(安培)
    foc_control_mode_t control_mode;
    float test_data;         // 测试数据
    float test_data2;        // 测试数据2
} uart_data_packet_t;

// 全局队列句柄
static QueueHandle_t uart_queue = NULL;

// 目标值
foc_target_t target = {
        .control_mode = FOC_CONTROL_MODE_POSITION_SINGLE,
        .target_current = 0.2f,
        .target_maxcurrent = 0.5f,
        .target_velocity = 30.0f,
        .target_position = 3.0f
 };

// 定义UART命令回调函数
static void uart_cmd_handler(uart_cmd_type_t cmd_type, int value, void* user_data)
{
    switch (cmd_type) {
        case UART_CMD_TYPE_A:
            // 处理'a'类型命令 - 设置速度
            ESP_LOGI(TAG, "设置参数a为: %d", value);
            
            #if FOC_CONTROL_MODE_SELECT == FOC_CONTROL_OPENLOOP
            // 如果是开环模式，设置转速
            foc_openloop_set_targetSpeed(value);
            ESP_LOGI(TAG, "设置开环转速为: %d RPM", value);
            #else
            if(target.control_mode == FOC_CONTROL_MODE_TORQUE   )
            {
                target.target_current = value / 100.0f;
                foc_closedloop_set_target(&target);
                ESP_LOGI(TAG, "设置闭环电流为: %.2f A", target.target_current);
            }
            else if(target.control_mode == FOC_CONTROL_MODE_VELOCITY)
            {
                target.target_velocity = value / 10.0f;
                foc_closedloop_set_target(&target);
                ESP_LOGI(TAG, "设置闭环速度为: %.2f RPM", target.target_velocity);
            }else if(target.control_mode == FOC_CONTROL_MODE_POSITION_SINGLE)
            {
                target.target_position = value / 100.0f;
                target.target_position = fmod(target.target_position, _2PI);
                foc_closedloop_set_target(&target);
                ESP_LOGI(TAG, "设置闭环位置为: %.2f rad", target.target_position);
            }
            else if(target.control_mode == FOC_CONTROL_MODE_POSITION_FULL)
            {
                target.target_position = value / 10.0f;
                foc_closedloop_set_target(&target);
                ESP_LOGI(TAG, "设置闭环位置为: %.2f rad", target.target_position);
            }else if(target.control_mode == FOC_CONTROL_MODE_TORQUE_VELOCITY_POSITION)
            {
                target.target_position = value / 10.0f;
                foc_closedloop_set_target(&target);
                ESP_LOGI(TAG, "设置闭环位置为: %.2f rad", target.target_position);
            }
#endif
            break;

        case UART_CMD_TYPE_B:
            // 处理'b'类型命令
            target.target_maxcurrent = value / 100.0f;
            foc_closedloop_set_target(&target);
            ESP_LOGI(TAG, "设置闭环最大电流为: %.2f A", target.target_maxcurrent);

            ESP_LOGI(TAG, "设置参数b为: %d", value);
            // 这里添加对b命令的具体处理
            break;

        case UART_CMD_TYPE_V:
            // 处理电压控制命令
            {
                const float VOLTAGE_SCALE = 100.0f;
                const float MIN_VOLTAGE = 0.0f;
                const float MAX_VOLTAGE = 1.0f;
                const float VELOCITY_SCALE = 10.0f;

                float voltage = value / VOLTAGE_SCALE;
#if FOC_CONTROL_MODE_SELECT == FOC_CONTROL_OPENLOOP
                if (voltage < MIN_VOLTAGE || voltage > MAX_VOLTAGE)
                {
                    ESP_LOGW(TAG, "电压值超出范围 (%.1f-%.1f): %.2f",
                             MIN_VOLTAGE, MAX_VOLTAGE, voltage);
                    break;
                }
                voltage = _constrain(voltage, 0.0f, 1.0f);
                foc_openloop_set_targetVoltage(voltage);
                ESP_LOGI(TAG, "设置开环电压为: %.2f V", voltage);
#else

#endif
            }
            break;
        case UART_CMD_TYPE_C:
            // 处理'c'类型命令
            if (target.control_mode == FOC_CONTROL_MODE_TORQUE_VELOCITY_POSITION)
            {
                target.target_velocity = value / 10;
                foc_closedloop_set_target(&target);
                ESP_LOGI(TAG, "设置闭环速度为: %.2f rad/s", target.target_velocity);
            }
            else
            {
                ESP_LOGW(TAG, "当前控制模式不支持速度设置");
            }
            ESP_LOGI(TAG, "设置参数c为: %d", value);
            break;
        case UART_CMD_TYPE_D:
            if (value == 0)
            {
                target.control_mode = FOC_CONTROL_MODE_TORQUE;
            }
            else if (value == 1)
            {
                target.control_mode = FOC_CONTROL_MODE_VELOCITY;
            }
            else if (value == 2)
            {
                target.control_mode = FOC_CONTROL_MODE_POSITION_SINGLE;
            }
            else if (value == 3)
            {
                target.control_mode = FOC_CONTROL_MODE_POSITION_FULL;
            }
            else if (value == 4)
            {
                target.control_mode = FOC_CONTROL_MODE_TORQUE_VELOCITY_POSITION;
            }
            foc_closedloop_set_target(&target);
            // 处理'd'类型命令
            ESP_LOGI(TAG, "设置参数d为: %d", value);
            break;
        default:
            ESP_LOGW(TAG, "未知命令类型: %c", cmd_type);
            break;
        }
}

// 修改串口通信任务
static void uart_communication_task(void* arg)
{
    uart_data_packet_t data;
    uint32_t last_print_time = 0;
    uint32_t current_time;
    
    ESP_LOGI(TAG, "串口通信任务已启动在核心%d上", UART_TASK_CORE_ID);
    
    // 初始化UART命令处理器
    uart_cmd_config_t cmd_config = {
        .uart_port = UART_NUM,
        .rx_buffer_size = 128,
        .callback = uart_cmd_handler,
        .user_data = NULL
    };
    
    esp_err_t ret = uart_cmd_init(&cmd_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART命令处理器初始化失败: %d", ret);
    }
    
    while (true) {
        // 处理串口命令
        uart_cmd_process(10); // 10ms超时
        
        // 从队列接收数据，等待最多50ms
        if (xQueueReceive(uart_queue, &data, pdMS_TO_TICKS(50)) == pdTRUE) {
            // 发送数据到串口
            float uart_buffer[22];
            uart_buffer[0] = data.duty_u;
            uart_buffer[1] = data.duty_v;
            uart_buffer[2] = data.duty_w;
            uart_buffer[3] = data.timestamp;
            uart_buffer[4] = data.speed_rpm;
            uart_buffer[5] = data.speed_rad;
            uart_buffer[6] = data.encoder_angle;
            uart_buffer[7] = data.encoder_electrical_angle;
            uart_buffer[8] = data.total_angle_rad;
            uart_buffer[9] = data.full_rotations;
            uart_buffer[10] = data.current_u;
            uart_buffer[11] = data.current_v;
            uart_buffer[12] = data.current_w;
            uart_buffer[13] = data.d_current;
            uart_buffer[14] = data.q_current;
            uart_buffer[15] = data.test_data;
            uart_buffer[16] = data.test_data2;
            uart_buffer[17] = data.target_current;
            uart_buffer[18] = data.target_velocity;
            uart_buffer[19] = data.target_position;
            uart_buffer[20] = data.target_maxcurrent;
            uart_buffer[21] = data.control_mode;

            // 发送数据
            uart_write_bytes(UART_NUM, (const char*)uart_buffer, sizeof(float) * 22);
            
            // 发送帧尾
            uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
            uart_write_bytes(UART_NUM, (const char*)tail, 4);
            
            // 每两秒打印一次系统状态
            current_time = esp_timer_get_time() / 1000; // 毫秒
            if (current_time - last_print_time > 2000) {
                last_print_time = current_time;
                ESP_LOGI(TAG, "系统状态: 速度=%.1f RPM(设定)/%.1f RPM(实际), 电压=%.2f, 编码器角度=%.2f rad",
                         data.speed_rpm, data.encoder_speed, data.voltage, data.encoder_angle);
            }
        }
    }
}

// FOC控制任务
static void foc_control_task(void* arg)
{
    // 禁用任务看门狗
    esp_task_wdt_delete(NULL);  // 从当前任务中删除任务看门狗
    
    // 创建任务看门狗配置
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 10000,           // 10秒超时时间
        .idle_core_mask = 0,           // 不监视任何空闲任务
        .trigger_panic = false         // 超时不触发panic
    };
    
    // 重新初始化任务看门狗，不监视空闲任务
    esp_task_wdt_deinit();             // 先完全卸载任务看门狗
    esp_task_wdt_init(&twdt_config);   // 使用新配置初始化任务看门狗
    
    inverter_handle_t inverter = (inverter_handle_t)arg;
    
    uint8_t count = 0;
    
    // 状态输出时间控制
    uint32_t last_print_time = 0;
    
    // 初始化电流采样模块（如果尚未初始化）
    if (!g_current_sense_initialized) {
        // 获取GPIO对应的ADC通道和单元
        adc_unit_t u_unit;
        adc_channel_t u_channel;
        adc_unit_t v_unit;
        adc_channel_t v_channel;
        
        esp_err_t ret;
        ret = adc_oneshot_io_to_channel(CURRENT_SENSE_U_PIN, &u_unit, &u_channel);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "GPIO %d不是有效的ADC引脚", CURRENT_SENSE_U_PIN);
        } else {
            ret = adc_oneshot_io_to_channel(CURRENT_SENSE_V_PIN, &v_unit, &v_channel);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "GPIO %d不是有效的ADC引脚", CURRENT_SENSE_V_PIN);
            } else {
                // 配置电流采样模块
                current_sense_config_t config = {
                    .u_phase_gpio = CURRENT_SENSE_U_PIN,
                    .v_phase_gpio = CURRENT_SENSE_V_PIN,
                    .u_phase_unit = u_unit,
                    .u_phase_channel = u_channel,
                    .v_phase_unit = v_unit,
                    .v_phase_channel = v_channel,
                    .sample_freq_hz = 80000,          // 连续转换80kHz采样频率，单次转换使用不到
                    .zero_current_adc_u = 2048.0f,    // 默认零点值
                    .zero_current_adc_v = 2048.0f,    // 默认零点值
                    .use_w_calculation = true         // 通过计算得到W相电流
                };
                
                // 初始化并校准电流采样模块
                ret = current_sense_init(&config);
                if (ret == ESP_OK) {
                    // 电机未通电时校准零点
                    ESP_LOGI(TAG, "开始电流传感器零点校准");
                    ret = current_sense_calibrate(100);  // 采集100个样本进行校准
                    if (ret == ESP_OK) {
                        // 启动ADC单次转换
                        ret = current_sense_start();
                        if (ret == ESP_OK) {
                            g_current_sense_initialized = true;
                            ESP_LOGI(TAG, "电流采样模块已初始化并启动");
                        } else {
                            ESP_LOGE(TAG, "启动电流采样失败: %d", ret);
                        }
                    } else {
                        ESP_LOGE(TAG, "校准电流传感器失败: %d", ret);
                    }
                } else {
                    ESP_LOGE(TAG, "初始化电流采样模块失败: %d", ret);
                }
            }
        }
    }
    
#if FOC_CONTROL_MODE_SELECT == FOC_CONTROL_OPENLOOP
    // 初始化FOC开环控制模式
    foc_openloop_params_t openloop_params = {
        .voltage_magnitude = EXAMPLE_MOTOR_MAX_VOLTAGE,
        .resistance = EXAMPLE_MOTOR_RESISTANCE,
        .supply_voltage = EXAMPLE_MOTOR_SUPPLY_VOLTAGE,
        .kv = EXAMPLE_MOTOR_KV,
        .current_limit = EXAMPLE_MOTOR_CURRENT_LIMIT,
        .speed_rpm = EXAMPLE_MOTOR_OPENLOOP_RPM,
        .pole_pairs = EXAMPLE_MOTOR_POLE_PAIRS,
        .period = EXAMPLE_FOC_MCPWM_PERIOD,
        .direction = EXAMPLE_MOTOR_DIRECTION,
    };
    foc_openloop_init(&openloop_params, NULL);
    foc_openloop_set_targetSpeed(EXAMPLE_MOTOR_OPENLOOP_RPM);
    ESP_LOGI(TAG, "FOC开环控制初始化完成，设置转速: %d RPM", EXAMPLE_MOTOR_OPENLOOP_RPM);
#else
    // 初始化FOC闭环控制模式
    foc_closedloop_params_t closedloop_params = {
        .pole_pairs = EXAMPLE_MOTOR_POLE_PAIRS,
        .direction = EXAMPLE_MOTOR_DIRECTION,
        .period = EXAMPLE_FOC_MCPWM_PERIOD,
        .supply_voltage = EXAMPLE_MOTOR_SUPPLY_VOLTAGE,
        .kv = EXAMPLE_MOTOR_KV,
        .resistance = EXAMPLE_MOTOR_RESISTANCE,
        .current_limit = EXAMPLE_MOTOR_CURRENT_LIMIT,          
        .voltage_limit = EXAMPLE_MOTOR_MAX_VOLTAGE,
        .dt = 1.0f / FOC_CONTROL_FREQ_HZ, // 控制周期
        .zero_angle_rad = 0.598f,
        // 使用速度控制模式 - 与下方set_target保持一致
        .control_mode = FOC_CONTROL_MODE_VELOCITY,
        
        .invert_phase_order = false,
        
        .target_iq = 0.0f,              // 初始q轴电流为0
        .target_id = 0.0f,              // d轴电流目标为0
        
        .current_filter_alpha = 0.005f,   // 较小的值滤波效果更强
        
        // 初始化电流PI控制器
        .id_pid = {
            .kp = 4.5f,     // 为d轴添加控制，保持在0电流
            .ki = 200.0f,     // 适当的积分系数
            .kd = 0.0f,     // 不使用微分控制
            .output_limit = 1.0f,//电压比例限幅0-1
            .integral = 0.0f
        },
        .iq_pid = {
            .kp = 12.0f,
            .ki = 500.0f,    // 增加积分系数提高响应性
            .kd = 0.0f,     // 不使用微分控制
            .output_limit = 1.0f,//电压比例限幅0-1
            .integral = 0.0f
        },

        // 速度和位置控制器参数
        .velocity_pid = {
            .kp = 0.022f,    // 比例系数
            .ki = 0.3f,     // 积分系数
            .kd = 0.00002f,     // 微分控制
            .output_limit = 1.0f,//速度PID积分限幅
            .integral = 0.0f
        },
        .position_single_pid = {
            .kp = 0.35f,
            .ki = 0.1f,
            .kd = 0.00001f,     // 微分控制
            .output_limit = 0.8f,//位置PID积分限幅
            .integral = 0.0f
        },
        .position_full_pid = {
            .kp = 0.22f,
            .ki = 0.12f,
            .kd = 0.001f,     // 微分控制
            .output_limit = 0.5f,//位置PID积分限幅
            .integral = 0.0f
        },
        .position_pid = {
            .kp = 1.5f,
            .ki = 0.23f,
            .kd = 0.001f,     // 微分控制
            .output_limit = 1.0f,//位置PID积分限幅
            .integral = 0.0f
        }
    };
    
    // 初始化闭环控制
    esp_err_t ret = foc_closedloop_init(&closedloop_params, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "初始化FOC闭环控制失败: %d", ret);
    } else {
        ESP_LOGI(TAG, "FOC闭环控制初始化完成");
    }
    
    ESP_LOGI(TAG, "设置零电角度");
    foc_duty_t duty;
    foc_set_PhaseVoltage(0.5f, 0.0f, _3PI_2, EXAMPLE_FOC_MCPWM_PERIOD, EXAMPLE_MOTOR_MAX_VOLTAGE, &duty, inverter);
    vTaskDelay(pdMS_TO_TICKS(100));
    as5600_update();
    closedloop_params.zero_angle_rad = as5600_get_electrical_angle(EXAMPLE_MOTOR_POLE_PAIRS, EXAMPLE_MOTOR_DIRECTION, 0.0f);

    ESP_LOGI(TAG, "设置零电角度: %.2f rad", closedloop_params.zero_angle_rad);
    foc_set_PhaseVoltage(0.0f, 0.0f, 0.0f, EXAMPLE_FOC_MCPWM_PERIOD, EXAMPLE_MOTOR_MAX_VOLTAGE, &duty, inverter); 
    vTaskDelay(pdMS_TO_TICKS(100));

    target.target_position = as5600_get_total_angle_rad();
    // 设置目标
    foc_closedloop_set_target(&target);
    ESP_LOGI(TAG, "设置电流: %.2f A，开始闭环控制", target.target_current);
#endif

    // 主循环变量声明 - 放在这里避免循环中重复声明
    float angle_elec = 0;
    float electrical_angle = 0;
    foc_uvw_coord_t phase_currents = {.u = _IQ(0), .v = _IQ(0), .w = _IQ(0)};
    foc_dq_coord_t dq_currents = {.d = _IQ(0), .q = _IQ(0)};
    float velocity = 0;
    float position_single = 0;
    float position_full = 0;

    while (true) {
        // 等待定时器触发
        if (xSemaphoreTake(foc_timer_semaphore, portMAX_DELAY) == pdTRUE) {
            dt = esp_timer_get_time() - last_update_time;
            last_update_time = esp_timer_get_time();
            
            // ======== 传感器采集部分 ========
            // 在闭环模式或Debug模式下采集传感器数据
#if FOC_CONTROL_MODE_SELECT == FOC_CONTROL_CLOSEDLOOP || FOC_CONTROL_MODE_DEBUG == 1
            // 更新编码器
            as5600_set_dt(dt / 1000000.0f);
            as5600_update();
            electrical_angle = as5600_get_electrical_angle(EXAMPLE_MOTOR_POLE_PAIRS, EXAMPLE_MOTOR_DIRECTION, closedloop_params.zero_angle_rad);
            
            // 读取电流数据
            if (g_current_sense_initialized) {
                // 直接采样获取电流，而不是读取其他任务更新的数据
                esp_err_t ret = current_sense_sample(&g_current_reading);
                if (ret != ESP_OK) {
                    ESP_LOGW(TAG, "读取电流值失败: %d", ret);
                }
            }
            
            // 设置相电流
            phase_currents.u = _IQ(g_current_reading.current_u);
            phase_currents.v = _IQ(g_current_reading.current_v);
            phase_currents.w = _IQ(g_current_reading.current_w);
#endif
            
            // ======== 控制逻辑部分 ========
#if FOC_CONTROL_MODE_SELECT == FOC_CONTROL_OPENLOOP
            // 开环控制
            angle_elec = foc_openloop_output(inverter);
            
    #if FOC_CONTROL_MODE_DEBUG == 1
            foc_dq_coord_t dq_currents_r = {.d = _IQ(0), .q = _IQ(0)};
            // Debug模式下，计算dq电流
            foc_calculate_dq_current(&phase_currents, _IQ(angle_elec), &dq_currents);
            foc_calculate_dq_current(&phase_currents, _IQ(electrical_angle), &dq_currents_r);
    #endif
#else
            // 闭环控制
            velocity = as5600_get_speed_rad();
            position_single = as5600_get_position();
            position_full = as5600_get_total_angle_rad();
            foc_closedloop_set_motion_state(velocity, position_single, position_full);
            foc_closedloop_output(inverter, &phase_currents, _IQ(electrical_angle));


            foc_calculate_dq_current(&phase_currents, _IQ(electrical_angle), &dq_currents);
#endif
            
#if FOC_CONTROL_MODE_SELECT == FOC_CONTROL_OPENLOOP
            // 获取开环控制状态
            foc_openloop_state_t *ol_state = foc_openloop_get_state();
            // 获取FOC参数
            foc_openloop_params_t* params = foc_openloop_get_params();
#else
            // 获取闭环控制状态
            foc_closedloop_state_t *cl_state = foc_closedloop_get_state();
            foc_closedloop_params_t* params = foc_closedloop_get_params();
#endif

            // 每100次循环(约50ms)向串口任务发送一次数据
            if (count >= 100) {
                count = 0;
                
                // 准备数据包
                uart_data_packet_t data = {
                    .timestamp = dt,
#if FOC_CONTROL_MODE_SELECT == FOC_CONTROL_OPENLOOP
                    .duty_u = ol_state->duty_u, 
                    .duty_v = ol_state->duty_v,
                    .duty_w = ol_state->duty_w,
                    .speed_rpm = params->speed_rpm,
                    .voltage = params->voltage_magnitude,
#if FOC_CONTROL_MODE_DEBUG == 1
                    .test_data = _IQtoF(electrical_angle),
                    .test_data2 = ol_state->angle_elec,
                    .d_current = - _IQtoF(dq_currents_r.d), 
                    .q_current = - _IQtoF(dq_currents_r.q),
#endif
#else
                    .duty_u = cl_state->duty_u,
                    .duty_v = cl_state->duty_v,
                    .duty_w = cl_state->duty_w,
                    .speed_rpm = as5600_get_speed_rpm(),
                    .speed_rad = as5600_get_speed_rad(),
                    .d_current = cl_state->id,
                    .q_current = cl_state->iq,
                    .target_current = params->target_iq,
                    .target_velocity = params->target_velocity,
                    .target_position = params->target_position,
                    .target_maxcurrent = params->target_maxcurrent,
                    .control_mode = params->control_mode,
#endif
                    .encoder_angle = as5600_get_position(),
                    .encoder_speed = as5600_get_speed_rpm(),
                    .total_angle_raw = as5600_get_total_angle_raw(),
                    .total_angle_rad = as5600_get_total_angle_rad(),
                    .full_rotations = as5600_get_full_rotations(),
                    .current_u = g_current_reading.current_u,
                    .current_v = g_current_reading.current_v,
                    .current_w = g_current_reading.current_w,
                    .encoder_electrical_angle = electrical_angle,
                    .test_data = cl_state->vd,
                    .test_data2 = cl_state->vq,
                };
                
                // 发送数据到队列，不等待(非阻塞)
                xQueueSendToBack(uart_queue, &data, 0);
                
                // 添加控制状态打印（仅每秒一次）
                uint32_t current_time = esp_timer_get_time() / 1000; // 转换为毫秒
                if (current_time - last_print_time > 1000) {
                    last_print_time = current_time;
                    
#if FOC_CONTROL_MODE_SELECT == FOC_CONTROL_OPENLOOP
                    /* ESP_LOGI(TAG, "开环控制状态: 设定速度=%.1f RPM, 电压=%.2f",
                             (float)EXAMPLE_MOTOR_OPENLOOP_RPM, EXAMPLE_MOTOR_MAX_VOLTAGE); */
#else
                    // 计算力矩和功率估计值
                    float torque_est = cl_state->iq * 0.1f; // 假设转矩常数为0.1 Nm/A
                    float power_est = torque_est * velocity; // 机械功率 = 转矩 * 角速度
                    
/*                     ESP_LOGI(TAG, "闭环控制状态: 速度=%.1f rad/s, Id=%.2fA, Iq=%.2fA, 估计转矩=%.2f Nm, 估计功率=%.2f W",
                             velocity, cl_state->id, cl_state->iq, torque_est, power_est); */
#endif
                }
            }
            count++;
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Hello FOC");
    
    // 初始化电机配置
    motor_config = get_motor_config();
    ESP_LOGI(TAG, "使用电机配置: %s", motor_config->name);
    
    // counting semaphore used to sync update foc calculation when mcpwm timer updated
    SemaphoreHandle_t update_semaphore = xSemaphoreCreateCounting(1, 0);

    
    // 配置UART
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TXD, UART_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // 安装驱动程序，启用收发缓冲区
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, 1024, 1024, 0, NULL, 0));
    ESP_LOGI(TAG, "UART初始化完成，启用收发功能");

    inverter_config_t cfg = {
        .timer_config = {
            .group_id = 0,
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = EXAMPLE_FOC_MCPWM_TIMER_RESOLUTION_HZ,
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN,   //UP_DOWN mode will generate center align pwm wave, which can reduce MOSFET switch times on same effect, extend life
            .period_ticks = EXAMPLE_FOC_MCPWM_PERIOD,
        },
        .operator_config = {
            .group_id = 0,
        },
        .compare_config = {
            .flags.update_cmp_on_tez = true,
        },
        .gen_gpios = {
            {EXAMPLE_FOC_PWM_U_GPIO, -1},  // 只使用高边PWM，低边设为-1表示不使用
            {EXAMPLE_FOC_PWM_V_GPIO, -1},
            {EXAMPLE_FOC_PWM_W_GPIO, -1},
        },
        .dt_config = {
            .posedge_delay_ticks = 1,
        },
        .inv_dt_config = {
            .negedge_delay_ticks = 1,
            .flags.invert_output = true,
        },
    };
    inverter_handle_t inverter1;
    ESP_ERROR_CHECK(svpwm_new_inverter(&cfg, &inverter1));
    ESP_LOGI(TAG, "Inverter init OK");

    mcpwm_timer_event_callbacks_t cbs = {
        .on_full = inverter_update_cb,
    };
    ESP_ERROR_CHECK(svpwm_inverter_register_cbs(inverter1, &cbs, &update_semaphore));
    ESP_ERROR_CHECK(svpwm_inverter_start(inverter1, MCPWM_TIMER_START_NO_STOP));
    ESP_LOGI(TAG, "Inverter start OK");


    // Enable gate driver chip
    bsp_bridge_driver_init();
    bsp_bridge_driver_enable(true);

    ESP_LOGI(TAG, "Start FOC");
#if FOC_CONTROL_MODE_SELECT == FOC_CONTROL_OPENLOOP
    ESP_LOGI(TAG, "使用FOC开环控制模式");
#else
    ESP_LOGI(TAG, "使用FOC闭环控制模式");
#endif
    
    // 创建FOC控制定时器信号量
    foc_timer_semaphore = xSemaphoreCreateBinary();
    
    // 创建高精度定时器，周期为 1/4000 秒
    const esp_timer_create_args_t timer_args = {
        .callback = &foc_timer_callback,
        .name = "foc_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &foc_timer_handle));
    
    // 创建FOC控制任务
    xTaskCreatePinnedToCore(
        foc_control_task,       // 任务函数
        "foc_task",             // 任务名称
        FOC_TASK_STACK_SIZE,    // 栈大小
        inverter1,              // 任务参数
        FOC_TASK_PRIORITY,      // 任务优先级
        &foc_task_handle,       // 任务句柄
        FOC_TASK_CORE_ID        // 运行的核心
    );
    
    // 初始化AS5600编码器
    ESP_ERROR_CHECK(as5600_init(AS5600_SDA_PIN, AS5600_SCL_PIN, AS5600_I2C_FREQ, AS5600_I2C_PORT));
    
    // 设置编码器采样周期 (与FOC控制频率相同)
    as5600_set_dt(1.0f / FOC_CONTROL_FREQ_HZ);
    
    // 创建通信队列
    uart_queue = xQueueCreate(UART_QUEUE_SIZE, sizeof(uart_data_packet_t));
    
    // 创建串口通信任务
    xTaskCreatePinnedToCore(
        uart_communication_task, // 任务函数
        "uart_task",            // 任务名称
        UART_TASK_STACK_SIZE,   // 栈大小
        NULL,                   // 任务参数
        UART_TASK_PRIORITY,     // 任务优先级
        NULL,                   // 任务句柄(不需要存储)
        UART_TASK_CORE_ID       // 运行的核心
    );
    
    // 启动定时器，周期为1/2000秒 (500微秒)
    ESP_ERROR_CHECK(esp_timer_start_periodic(foc_timer_handle, 1000000 / FOC_CONTROL_FREQ_HZ));
    
    ESP_LOGI(TAG, "系统初始化完成，带编码器反馈的FOC控制");
    
    // 主循环
    while (true) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
    // 下面的代码不会执行到，除非在主循环添加退出条件
    ESP_ERROR_CHECK(esp_timer_stop(foc_timer_handle));
    ESP_ERROR_CHECK(esp_timer_delete(foc_timer_handle));
    vTaskDelete(foc_task_handle);
    
    // 清理资源
    foc_openloop_stop();
    bsp_bridge_driver_enable(false);
    ESP_ERROR_CHECK(svpwm_inverter_start(inverter1, MCPWM_TIMER_STOP_EMPTY));
    ESP_ERROR_CHECK(svpwm_del_inverter(inverter1));
    uart_driver_delete(UART_NUM);
    
    // 如果电流采样模块已初始化，则释放资源
    if (g_current_sense_initialized) {
        current_sense_stop();
        current_sense_deinit();
    }
}
