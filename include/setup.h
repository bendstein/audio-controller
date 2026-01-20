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
    { .port = I2C_NUM_1, .scl = GPIO_NUM_19, .sda = GPIO_NUM_5 }
};

static constexpr setup_cfg_dev SENSOR_CFG[SENSORS_COUNT] = {
    { .address = gp2y0e02b::distance_sensor::I2C_ADDR_DFT, .bus_num = 0 },
    { .address = gp2y0e02b::distance_sensor::I2C_ADDR_DFT, .bus_num = 1 }
};

// static constexpr setup_cfg_dev DAC_CFG = { .address = mcp4725::dac::I2C_ADDR_DFT, .bus_num = 0 };

[[nodiscard]] app_state do_setup();

void init_i2c_buses(app_state* app_state);
void init_distance_sensors(app_state* setup);
void init_dac_controller(app_state* setup);
bool try_create_distance_sensor_task(const std::string& task_name, BaseType_t* result_code, size_t sensor_ndx, app_state* setup);
bool try_create_dac_write_task(const std::string& task_name, BaseType_t* result_code, app_state* setup);

#endif //AUDIO_CONTROLLER_SETUP_H