//
// Created by bendstein on 12/27/2025.
//

#ifndef AUDIO_CONTROLLER_SETUP_H
#define AUDIO_CONTROLLER_SETUP_H
#include <driver/uart.h>

#include "app_state.h"
#include "i2c/gp2y0e02b/distance_sensor.h"

struct setup_cfg_dev
{
    uint8_t address;
    uint8_t bus_num;
};

struct setup_cfg_uart_port
{
    uart_port_t port;
    size_t buffer_size;
    size_t queue_len;
    int8_t pin_tx;
    int8_t pin_rx;
    int8_t pin_rts;
    int8_t pin_cts;
};

static constexpr i2c_master_create_cfg BUS_OPTIONS[BUS_COUNT] = {
    { .port = I2C_NUM_0, .scl = GPIO_NUM_20, .sda = GPIO_NUM_22 },
    { .port = I2C_NUM_1, .scl = GPIO_NUM_27, .sda = GPIO_NUM_33 }
};

static constexpr setup_cfg_dev SENSOR_CFG[SENSORS_COUNT] = {
    { .address = gp2y0e02b::distance_sensor::I2C_ADDR_DFT, .bus_num = 0 },
    { .address = gp2y0e02b::distance_sensor::I2C_ADDR_DFT, .bus_num = 1 }
};

static constexpr uart_config_t UART_OPTIONS = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_EVEN,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 122,
    .rx_glitch_filt_thresh = 0,
    .source_clk = UART_SCLK_DEFAULT,
    .flags = {
        .allow_pd = false,
        .backup_before_sleep = false
    }
};

static constexpr setup_cfg_uart_port UART_CFG_TX = {
    .port = UART_NUM_0,
    .buffer_size = 1 << 11,
    .queue_len = 255,
    .pin_tx = GPIO_NUM_5,
    .pin_rx = UART_PIN_NO_CHANGE,
    .pin_rts = GPIO_NUM_4,
    .pin_cts = GPIO_NUM_4
};

static constexpr setup_cfg_uart_port UART_CFG_RX = {
    .port = UART_NUM_0,
    .buffer_size = 1 << 11,
    .queue_len = 255,
    .pin_tx = UART_PIN_NO_CHANGE,
    .pin_rx = GPIO_NUM_5,
    .pin_rts = GPIO_NUM_4,
    .pin_cts = GPIO_NUM_4
};

[[nodiscard]] app_state_controller start_controller();
[[nodiscard]] app_state_audio start_audio();

void init_i2c_buses(app_state_controller* state);
void init_distance_sensors(app_state_controller* state);
void init_signal_generator(app_state_audio* state);
QueueHandle_t init_uart_controller(const setup_cfg_uart_port& uart_port_config);
bool try_create_distance_sensor_task(const std::string& task_name, BaseType_t* result_code, size_t bus_ndx, app_state_controller* state);
bool try_create_send_audio_task(const std::string& task_name, BaseType_t* result_code, app_state_controller* state);
bool try_create_receive_audio_task(const std::string& task_name, BaseType_t* result_code, app_state_audio* state);

#endif //AUDIO_CONTROLLER_SETUP_H