//
// Created by bendstein on 12/27/2025.
//

#ifndef AUDIO_CONTROLLER_SETUP_H
#define AUDIO_CONTROLLER_SETUP_H

#include "app_state.h"
#include "i2c/gp2y0e02b/distance_sensor.h"

static constexpr uint8_t SENSOR_ADDRESSES[SENSORS_COUNT] = {
    gp2y0e02b::distance_sensor::I2C_ADDR_DFT
};
static constexpr uint8_t DAC_ADDRESS = mcp4725::dac::I2C_ADDR_DFT;

static constexpr auto DISTANCE_SENSOR_TASK_PRIORITY = 2;
static constexpr auto DISTANCE_SENSOR_TASK_STACK_SIZE = 0x1000;
static constexpr auto DAC_WRITE_TASK_PRIORITY = 2;
static constexpr auto DAC_WRITE_TASK_STACK_SIZE = 0x1000;

static constexpr float PIECEWISE_FREQUENCY_BREAKPOINTS[PIECEWISE_FREQUENCY_BREAKPOINT_COUNT] = {
    0.4,
    0.8,
    1
};

[[nodiscard]] app_state do_setup();

[[nodiscard]] i2c_master_bus_handle_t init_i2c_bus(i2c_port_num_t port, gpio_num_t sda, gpio_num_t scl);
void init_distance_sensors(app_state* setup);
void init_dac_controller(app_state* setup);
bool try_create_distance_sensor_task(const std::string& task_name, BaseType_t* result_code, size_t sensor_ndx, app_state* setup);
bool try_create_dac_write_task(const std::string& task_name, BaseType_t* result_code, app_state* setup);

#endif //AUDIO_CONTROLLER_SETUP_H