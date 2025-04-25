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

static const char *TAG = "example_foc";

//////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////// Please update the following configuration according to your HardWare spec /////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EXAMPLE_FOC_DRV_EN_GPIO          12
#define EXAMPLE_FOC_PWM_U_GPIO           32
#define EXAMPLE_FOC_PWM_V_GPIO           33
#define EXAMPLE_FOC_PWM_W_GPIO           25

// UART配置
#define UART_NUM UART_NUM_0
#define UART_TXD 1
#define UART_RXD 3
#define UART_BAUD_RATE 115200

#define EXAMPLE_FOC_MCPWM_TIMER_RESOLUTION_HZ 20000000 // 20MHz, 1 tick = 0.05us
#define EXAMPLE_FOC_MCPWM_PERIOD              2000     // 2000 * 0.05us = 100us, 10KHz


// 电机参数
#define EXAMPLE_MOTOR_POLE_PAIRS          7   // 电机极对数
#define EXAMPLE_MOTOR_MAX_VOLTAGE         0.8 // 最大电压，对应PWM占空比的倍数 (0.0-1.0)
#define EXAMPLE_MOTOR_OPENLOOP_RPM        2000 // 开环控制转速 (RPM)

// AS5600编码器配置
#define AS5600_SDA_PIN      19      // SDA引脚
#define AS5600_SCL_PIN      18      // SCL引脚
#define AS5600_I2C_FREQ     400000  // 400kHz - 降低频率以提高稳定性
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

// 定义FOC任务常量
#define FOC_TASK_STACK_SIZE     8192
#define FOC_TASK_PRIORITY       5
#define FOC_CONTROL_FREQ_HZ     4000  // 4kHz控制频率
#define FOC_TASK_CORE_ID        1     // 在核心1上运行

// FOC控制任务全局变量
static TaskHandle_t foc_task_handle = NULL;
static SemaphoreHandle_t foc_timer_semaphore = NULL;
static esp_timer_handle_t foc_timer_handle = NULL;

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
    float voltage;
    float encoder_angle;     // 编码器角度(弧度)
    float encoder_speed;     // 编码器速度(RPM)
    int32_t total_angle_raw; // 总角度原始值
    int32_t full_rotations;  // 完整旋转圈数
} uart_data_packet_t;

// 全局队列句柄
static QueueHandle_t uart_queue = NULL;

// 串口通信任务
static void uart_communication_task(void* arg)
{
    uart_data_packet_t data;
    uint32_t last_print_time = 0;
    uint32_t current_time;
    
    ESP_LOGI(TAG, "串口通信任务已启动在核心%d上", UART_TASK_CORE_ID);
    
    while (true) {
        // 从队列接收数据，等待最多100ms
        if (xQueueReceive(uart_queue, &data, pdMS_TO_TICKS(100)) == pdTRUE) {
            // 发送数据到串口
            float uart_buffer[10]; // 增加两个元素存放编码器数据
            uart_buffer[0] = data.duty_u;
            uart_buffer[1] = data.duty_v;
            uart_buffer[2] = data.duty_w;
            uart_buffer[3] = data.timestamp;
            uart_buffer[4] = data.speed_rpm;
            uart_buffer[5] = data.voltage;
            uart_buffer[6] = data.encoder_angle;
            uart_buffer[7] = data.encoder_speed;
            uart_buffer[8] = data.total_angle_raw;
            uart_buffer[9] = data.full_rotations;
            
            // 发送数据
            uart_write_bytes(UART_NUM, (const char*)uart_buffer, sizeof(float) * 10);
            
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
        
        // 无数据时也可以执行其他低优先级任务
        // vTaskDelay(1); // 可选的短暂延时让出CPU
    }
}

// FOC控制任务
static void foc_control_task(void* arg)
{
    inverter_handle_t inverter = (inverter_handle_t)arg;
    
    // 获取FOC参数
    foc_openloop_params_t* params = foc_openloop_get_params();
    uint8_t count = 0;
    
    while (true) {
        // 等待定时器触发
        if (xSemaphoreTake(foc_timer_semaphore, portMAX_DELAY) == pdTRUE) {
            dt = esp_timer_get_time() - last_update_time;
            last_update_time = esp_timer_get_time();
            as5600_set_dt(dt / 1000000.0f);
            // 更新编码器数据
            as5600_update(); 
            
            // 读取当前电角度 (可用于闭环控制)
            // _iq electrical_angle = as5600_get_electrical_angle_iq(params->pole_pairs);
            
            // 执行FOC控制 (当前仍为开环)
            //foc_openloop_output(inverter, params);
            
            // 每100次循环(约40ms)向串口任务发送一次数据
            if (count >= 100) {
                count = 0;
                
                // 获取当前状态
                foc_openloop_state_t *state = foc_openloop_get_state();
                
                // 准备数据包
                uart_data_packet_t data = {
                    .timestamp = dt,
                    .duty_u = state->duty_u,
                    .duty_v = state->duty_v,
                    .duty_w = state->duty_w,
                    .speed_rpm = params->speed_rpm,
                    .voltage = params->voltage_magnitude,
                    .encoder_angle = as5600_get_position(),
                    .encoder_speed = as5600_get_speed(),
                    .total_angle_raw = as5600_get_total_angle_raw(),
                    .full_rotations = as5600_get_full_rotations()
                };
                
                // 发送数据到队列，不等待(非阻塞)
                xQueueSendToBack(uart_queue, &data, 0);
            }
            count++;
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Hello FOC");
    // counting semaphore used to sync update foc calculation when mcpwm timer updated
    SemaphoreHandle_t update_semaphore = xSemaphoreCreateCounting(1, 0);

    foc_dq_coord_t dq_out = {_IQ(0), _IQ(0)};
    foc_ab_coord_t ab_out;
    foc_uvw_coord_t uvw_out;
    int uvw_duty[3];
    float elec_theta_deg = 0;
    _iq elec_theta_rad;
    
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
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, 1024, 0, 0, NULL, 0));
    ESP_LOGI(TAG, "UART初始化完成");

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

    // 初始化开环控制
    foc_openloop_params_t openloop_params = {
        .voltage_magnitude = EXAMPLE_MOTOR_MAX_VOLTAGE,
        .speed_rpm = EXAMPLE_MOTOR_OPENLOOP_RPM,  // 初始速度设为0
        .pole_pairs = EXAMPLE_MOTOR_POLE_PAIRS,
        .period = EXAMPLE_FOC_MCPWM_PERIOD,
    };
    ESP_ERROR_CHECK(foc_openloop_init(&openloop_params, NULL));

    extern foc_openloop_state_t* foc_openloop_get_state(void);

    // Enable gate driver chip
    bsp_bridge_driver_init();
    bsp_bridge_driver_enable(true);

    ESP_LOGI(TAG, "Start FOC");
    ESP_LOGI(TAG, "使用FOC开环控制模式");
    
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
    
    // 启动定时器，周期为1/4000秒 (250微秒)
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
}
