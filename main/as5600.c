/*
 * AS5600 磁编码器驱动
 * 使用IQMath定点运算优化
 */
#include <string.h>
#include <math.h>
#include "as5600.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "as5600";

// 定义数学常量
#define _2PI        6.28318530718f
#define _60        60.0f
#define _scale      0.0015339808f    
#define _rpm_scale  (60.0f / (2.0f * M_PI))
// I2C通信参数
#define I2C_RETRY_COUNT     3        // 减少重试次数
#define I2C_TIMEOUT_MS      20       // 减少超时时间
#define I2C_RETRY_DELAY_MS  2        // 减少重试延迟

// 全局变量
static as5600_t g_as5600;           // AS5600状态
static i2c_port_t g_i2c_port;       // I2C端口号

// 读取统计数据
static struct {
    uint32_t total_reads;           // 总读取次数
    uint32_t failed_reads;          // 失败的读取次数
    uint32_t retry_success;         // 重试成功次数
} read_stats = {0};

// 低通滤波器 (IQ版本)
static _iq low_pass_filter_iq(_iq input, _iq prev_output, _iq alpha) {
    // output = alpha * input + (1 - alpha) * prev_output
    _iq one_minus_alpha = _IQ(1.0) - alpha;
    return _IQmpy(alpha, input) + _IQmpy(one_minus_alpha, prev_output);
}

// 读取AS5600寄存器
static esp_err_t as5600_read_register(uint8_t reg_addr, uint8_t *data, size_t len) {
    esp_err_t ret = ESP_FAIL;
    int retry = 0;
    
    read_stats.total_reads++;
    
    // 添加重试机制
    while (retry < I2C_RETRY_COUNT && ret != ESP_OK) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        
        // 第一阶段：发送寄存器地址
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (AS5600_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, reg_addr, true);
        
        // 第二阶段：读取数据
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (AS5600_ADDR << 1) | I2C_MASTER_READ, true);
        
        if (len > 1) {
            i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
        }
        i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        
        // 增加超时时间以提高可靠性
        ret = i2c_master_cmd_begin(g_i2c_port, cmd, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        
        if (ret != ESP_OK) {
            if (retry == 0) {
                read_stats.failed_reads++;
            }
            
            ESP_LOGW(TAG, "I2C读取失败(尝试 %d/%d): %s", 
                     retry + 1, I2C_RETRY_COUNT, esp_err_to_name(ret));
            retry++;
            
            // 重置I2C总线
            i2c_reset_tx_fifo(g_i2c_port);
            i2c_reset_rx_fifo(g_i2c_port);
            
            // 重试前等待更长时间
            vTaskDelay(I2C_RETRY_DELAY_MS / portTICK_PERIOD_MS);
        } else if (retry > 0) {
            // 记录重试成功
            read_stats.retry_success++;
            ESP_LOGD(TAG, "I2C读取重试成功(第%d次尝试)", retry + 1);
        }
    }
    
/*     // 周期性输出统计信息(每1000次读取)
    if ((read_stats.total_reads % 1000) == 0) {
        float success_rate = 100.0f * (read_stats.total_reads - read_stats.failed_reads) / read_stats.total_reads;
        ESP_LOGI(TAG, "AS5600读取统计: 总次数=%lu, 成功率=%.1f%%, 重试成功=%lu", 
                 read_stats.total_reads, success_rate, read_stats.retry_success);
    } */
    
    return ret;
}

// 初始化AS5600编码器和I2C
esp_err_t as5600_init(int sda_pin, int scl_pin, uint32_t i2c_freq, i2c_port_t i2c_port) {
    // 保存I2C端口号
    g_i2c_port = i2c_port;
    
    // 配置I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = i2c_freq,
    };
    
    esp_err_t ret = i2c_param_config(i2c_port, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C参数配置失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 使用DMA模式安装I2C驱动
    // 参数: 端口, 模式, RX缓冲区大小, TX缓冲区大小, 中断标志
    ret = i2c_driver_install(i2c_port, conf.mode, 128, 128, ESP_INTR_FLAG_IRAM);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C驱动安装失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 初始化AS5600结构体
    memset(&g_as5600, 0, sizeof(as5600_t));
    g_as5600.dt_iq = _IQ(0.001);  // 默认采样周期1ms (IQ格式)
    
    // 等待设备上电稳定
    vTaskDelay(50 / portTICK_PERIOD_MS);
    
    // 尝试多次读取初始角度
    uint8_t data[2];
    bool init_success = false;
    
    for (int i = 0; i < 5; i++) {
        ret = as5600_read_register(AS5600_ANGLE_REG, data, 2);
        if (ret == ESP_OK) {
            init_success = true;
            break;
        }
        ESP_LOGW(TAG, "初始化读取尝试 %d 失败: %s", i+1, esp_err_to_name(ret));
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    
    if (!init_success) {
        ESP_LOGE(TAG, "AS5600初始化失败: 无法读取初始角度");
        return ESP_FAIL;
    }
    
    // 解析原始角度
    uint16_t raw_angle = ((uint16_t)data[0] << 8) | data[1];
    raw_angle &= 0x0FFF;  // 保留12位
    
    g_as5600.raw_angle = raw_angle;
    g_as5600.angle_prev = raw_angle;
    g_as5600.initialized = 1;
    
    // 读取编码器状态
    uint8_t status;
    if (as5600_read_register(AS5600_STATUS_REG, &status, 1) == ESP_OK) {
        ESP_LOGI(TAG, "AS5600状态: 0x%02X (MH=%d, ML=%d, MD=%d)", 
                status, 
                (status >> 3) & 0x01,  // MH (磁场过强)
                (status >> 4) & 0x01,  // ML (磁场过弱)
                (status >> 5) & 0x01); // MD (检测到磁铁)
    }
    
    ESP_LOGI(TAG, "AS5600初始化成功，初始角度: %d (0x%03X)", raw_angle, raw_angle);
    
    // 重置统计数据
    memset(&read_stats, 0, sizeof(read_stats));
    
    return ESP_OK;
}

// 设置AS5600采样周期
void as5600_set_dt(float dt_sec) {
    g_as5600.dt_iq = _IQ(dt_sec);
}

// 设置零点角度
void as5600_set_zero_angle(float zero_angle_rad) {
    // 未使用零点角度，保留此接口以兼容旧代码
}

// 获取原始角度值
uint16_t as5600_get_raw_angle(void) {
    uint8_t data[2];
    esp_err_t ret = as5600_read_register(AS5600_ANGLE_REG, data, 2);
    
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "读取角度失败: %s", esp_err_to_name(ret));
        return g_as5600.raw_angle; // 读取失败时返回上次的值
    }
    
    // 组合两个字节，高字节在前，低字节在后
    uint16_t raw_angle = ((uint16_t)data[0] << 8) | data[1];
    
    // AS5600是12位分辨率，所以我们只保留低12位
    raw_angle &= 0x0FFF;
    
    return raw_angle;
}

// 读取并处理编码器数据
esp_err_t as5600_update(void) {
    if (!g_as5600.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // 读取当前角度
    uint8_t data[2];
    esp_err_t ret = as5600_read_register(AS5600_ANGLE_REG, data, 2);
    
    uint16_t raw_angle;
    
    if (ret != ESP_OK) {
        // 错误处理：使用上次读取的角度继续计算
        raw_angle = g_as5600.raw_angle;
    } else {
        // 读取成功，正确解析字节顺序
        raw_angle = ((uint16_t)data[0] << 8) | data[1];
        // 保留12位
        raw_angle &= 0x0FFF;
        g_as5600.raw_angle = raw_angle;
    }
    
    // 保存上一次角度用于计算差值
    uint16_t angle_prev = g_as5600.angle_prev;
    g_as5600.angle_prev = raw_angle;
    
    // 检测圈数变化 (使用整数计算)
    int16_t angle_diff = (int16_t)raw_angle - (int16_t)angle_prev;
    
    // 检测跨越零点
    if (angle_diff < -2048) { // 过零点，顺时针旋转
        g_as5600.full_rotations++;
    } else if (angle_diff > 2048) { // 过零点，逆时针旋转
        g_as5600.full_rotations--;
    }
    
    // 计算总角度（原始值）
    g_as5600.total_angle_raw = g_as5600.full_rotations * 4096 + raw_angle;
    
    // 使用IQMath计算物理角度 (弧度) - 优化计算
    _iq raw_angle_iq = _IQ(raw_angle);
    g_as5600.rotor_phy_angle_iq = _IQmpy(raw_angle_iq, _SCALE_IQ);

    // 将IQ物理角度标准化到 [0, 2π) 范围内
    while (_IQtoF(g_as5600.rotor_phy_angle_iq) >= _IQtoF(_2PI_IQ)) {
        g_as5600.rotor_phy_angle_iq = g_as5600.rotor_phy_angle_iq - _2PI_IQ;
    }
    while (_IQtoF(g_as5600.rotor_phy_angle_iq) < 0) {
        g_as5600.rotor_phy_angle_iq = g_as5600.rotor_phy_angle_iq + _2PI_IQ;
    }
    
    // 同时更新浮点物理角度以供外部接口使用
    g_as5600.rotor_phy_angle = _IQtoF(g_as5600.rotor_phy_angle_iq);
    
    // 计算角速度 (使用IQMath)
    _iq angle_diff_iq;
    
    if (angle_diff < -2048) {
        // 过零点，顺时针旋转
        angle_diff_iq = _IQmpy(_IQ(angle_diff + 4096), _SCALE_IQ);
    } else if (angle_diff > 2048) {
        // 过零点，逆时针旋转
        angle_diff_iq = _IQmpy(_IQ(angle_diff - 4096), _SCALE_IQ);
    } else {
        // 正常情况
        angle_diff_iq = _IQmpy(_IQ(angle_diff), _SCALE_IQ);
    }
    
    // 计算角速度 (弧度/秒) (IQ格式)
    g_as5600.velocity_rad_per_sec_iq = _IQdiv(angle_diff_iq, g_as5600.dt_iq);
    
        // 应用低通滤波 - 优化IQ计算
    _iq alpha = _IQ(0.02); // 滤波系数
    
    if (g_as5600.initialized == 1) {
        // 第一次运行，初始化滤波值
        g_as5600.velocity_filtered_iq = g_as5600.velocity_rad_per_sec_iq;
        g_as5600.initialized = 2;
    } else {
        // 应用滤波 (IQ格式)
        g_as5600.velocity_filtered_iq = low_pass_filter_iq(
            g_as5600.velocity_rad_per_sec_iq,
            g_as5600.velocity_filtered_iq,
            alpha
        );
    }
    // 计算RPM (RPM = rad/s * 60 / 2π) (IQ格式)
    g_as5600.velocity_rpm_iq = _IQmpy(g_as5600.velocity_filtered_iq, _RPM_SCALE_IQ);
    
    
    return ret; // 返回读取结果，即使出错也完成了计算
}

// 获取物理角度
float as5600_get_angle(void) {
    return g_as5600.rotor_phy_angle;
}

// 获取电角度 (浮点格式，兼容原接口)
float as5600_get_electrical_angle(int pole_pairs) {
    float elec_angle = g_as5600.rotor_phy_angle * pole_pairs;
    
    // 确保角度在[0, 2π)范围内
    while (elec_angle >= 6.28318530718f) {
        elec_angle -= 6.28318530718f;
    }
    while (elec_angle < 0) {
        elec_angle += 6.28318530718f;
    }
    
    return elec_angle;
}

// 获取电角度 (IQ格式) - 新接口
_iq as5600_get_electrical_angle_iq(int pole_pairs, int direction) {
    _iq pole_pairs_iq = _IQ(pole_pairs);
    _iq direction_iq = _IQ(direction);
    _iq elec_angle_iq = _IQmpy(g_as5600.rotor_phy_angle_iq, pole_pairs_iq);
    elec_angle_iq = _IQmpy(elec_angle_iq, direction_iq);
    
    // 确保角度在[0, 2π)范围内
    while (_IQtoF(elec_angle_iq) >= _IQtoF(_2PI_IQ)) {
        elec_angle_iq = elec_angle_iq - _2PI_IQ;
    }
    while (_IQtoF(elec_angle_iq) < 0) {
        elec_angle_iq = elec_angle_iq + _2PI_IQ;
    }
    
    return elec_angle_iq;
}

// 获取转速 (浮点格式，兼容原接口)
float as5600_get_speed_rad(void) {
    return _IQtoF(g_as5600.velocity_filtered_iq);
} 

// 获取转速(RPM)
float as5600_get_speed_rpm(void) {
    return _IQtoF(g_as5600.velocity_rpm_iq);
}

// 获取总角度原始值
int32_t as5600_get_total_angle_raw(void) {
    return g_as5600.total_angle_raw;
}

// 获取完整旋转圈数
int32_t as5600_get_full_rotations(void) {
    return g_as5600.full_rotations;
}

// 获取位置
float as5600_get_position(void) {
    return g_as5600.rotor_phy_angle;
}

