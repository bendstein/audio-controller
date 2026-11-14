//
// Created by bendstein on 12/27/2025.
//

#ifndef AUDIO_CONTROLLER_SETUP_H
#define AUDIO_CONTROLLER_SETUP_H
#include <driver/uart.h>

#include "fixed_vec.h"
#include "audio/signal_generator.h"
#include "audio/tones.h"
#include "i2c/i2c.h"
#include "i2c/gp2y0e02b/distance_sensor.h"

constexpr size_t BUS_COUNT = 2;
constexpr size_t SENSORS_COUNT = 2;
constexpr size_t MAX_VOICES = signal_generator::TONE_DATA_CAPACITY;
constexpr uint8_t MAX_INDIVIDUAL_VOLUME = 255 / MAX_VOICES;

struct app_state_controller
{
    std::unique_ptr<i2c_master> i2c_buses[BUS_COUNT];
    fixed_vec<std::unique_ptr<gp2y0e02b::distance_sensor>, SENSORS_COUNT> distance_sensors[BUS_COUNT];
    TaskHandle_t sensor_tasks[BUS_COUNT];
    double current_distances[SENSORS_COUNT];
    bool distance_changes[SENSORS_COUNT];
    TaskHandle_t send_audio_task;
    SemaphoreHandle_t current_distances_mutex;
    QueueHandle_t uart_queue;
};

struct app_state_audio
{
    std::unique_ptr<signal_generator> signal_gtor;
    TaskHandle_t receive_audio_task;
    QueueHandle_t uart_queue;
};

struct setup_cfg_sensor
{
    uint8_t bus_num;
    uint8_t address;
    musical_note_tone tone;
};

struct setup_cfg_uart_port
{
    uart_port_t port;
    int32_t buffer_size;
    int32_t queue_len;
    int8_t pin_tx;
    int8_t pin_rx;
    int8_t pin_rts;
    int8_t pin_cts;
};

static constexpr i2c_master_create_cfg BUS_OPTIONS[BUS_COUNT] = {
    { .port = I2C_NUM_0, .scl = GPIO_NUM_20, .sda = GPIO_NUM_22 },
    { .port = I2C_NUM_1, .scl = GPIO_NUM_27, .sda = GPIO_NUM_33 }
};

static constexpr setup_cfg_sensor SENSOR_CFG[SENSORS_COUNT] = {
    { .bus_num = 0, .address = 0x80, .tone = musical_note_tone(musical_note::A, 4) },
    { .bus_num = 1, .address = 0x80, .tone = musical_note_tone(musical_note::F, 4) }
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