//
// Created by bendstein on 12/27/2025.
//

#ifndef AUDIO_CONTROLLER_SETUP_H
#define AUDIO_CONTROLLER_SETUP_H
#include "app_state.h"
#include "i2c/gp2y0e02b/distance_sensor.h"

struct setup_cfg_dev
{
    uint8_t address;
    uint8_t bus_num;
};

static constexpr i2c_master_create_cfg BUS_OPTIONS[BUS_COUNT] = {
    { .port = I2C_NUM_0, .scl = GPIO_NUM_20, .sda = GPIO_NUM_22 },
    { .port = I2C_NUM_1, .scl = GPIO_NUM_5, .sda = GPIO_NUM_19 }
};

static constexpr setup_cfg_dev SENSOR_CFG[SENSORS_COUNT] = {
    { .address = gp2y0e02b::distance_sensor::I2C_ADDR_DFT, .bus_num = 0 },
    { .address = gp2y0e02b::distance_sensor::I2C_ADDR_DFT, .bus_num = 1 }
};

[[nodiscard]] app_state_standard start_standard();
[[nodiscard]] app_state_signal_generator start_signal_generator();

void init_i2c_buses(app_state_standard* app_state);
void init_distance_sensors(app_state_standard* app_state);
void init_signal_generator(app_state_signal_generator* app_state);
bool try_create_distance_sensor_task(const std::string& task_name, BaseType_t* result_code, size_t bus_ndx, app_state_standard* app_state);
bool try_create_dac_write_task(const std::string& task_name, BaseType_t* result_code, app_state_signal_generator* app_state);

#endif //AUDIO_CONTROLLER_SETUP_H