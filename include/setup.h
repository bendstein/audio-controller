//
// Created by bendstein on 12/27/2025.
//

#ifndef AUDIO_CONTROLLER_SETUP_H
#define AUDIO_CONTROLLER_SETUP_H

#include "audio/musical_distance_sensor.h"
#include "i2c/mcp4725.h"
#include "i2c/gp2y0e02b/distance_sensor.h"

constexpr size_t SENSORS_COUNT = 1;
constexpr uint8_t SENSOR_ADDRESSES[SENSORS_COUNT] = {
    gp2y0e02b::distance_sensor::I2C_ADDR_DFT
};
constexpr uint8_t DAC_ADDRESS = mcp4725::dac::I2C_ADDR_DFT;

constexpr auto DISTANCE_SENSOR_TASK_PRIORITY = 4;
constexpr auto DISTANCE_SENSOR_TASK_STACK_SIZE = 0x1800;
constexpr auto DAC_TASK_PRIORITY = 4;
constexpr auto DAC_TASK_STACK_SIZE = 0x1800;

constexpr auto PIECEWISE_FREQUENCY_BREAKPOINT_COUNT = 2;
constexpr double PIECEWISE_FREQUENCY_BREAKPOINTS[PIECEWISE_FREQUENCY_BREAKPOINT_COUNT] = {
    0.5,
    1.
};

struct setup_data
{
    i2c_master_bus_handle_t i2c_bus_0;
    i2c_master_bus_handle_t i2c_bus_1;
    std::optional<gp2y0e02b::distance_sensor*> sensors[SENSORS_COUNT];
    std::optional<mcp4725::dac*> dac;
    std::optional<musical_distance_sensor*> musical_distance_sensors[SENSORS_COUNT];
    std::optional<TaskHandle_t> sensor_tasks[SENSORS_COUNT];
    std::optional<TaskHandle_t> dac_task;
};

[[nodiscard]] setup_data do_setup();

[[nodiscard]] i2c_master_bus_handle_t init_i2c_bus(i2c_port_num_t port, gpio_num_t sda, gpio_num_t scl);
void init_distance_sensors(setup_data* setup);
void init_musical_distance_sensors(setup_data* setup);
void init_dac(setup_data* setup);
bool try_configure_gp2y0e02b(gp2y0e02b::distance_sensor* sensor);
bool try_create_distance_sensor_task(const std::string& task_name, BaseType_t* result_code, size_t sensor_ndx, setup_data* setup);
bool try_create_dac_task(const std::string& task_name, BaseType_t* result_code, setup_data* setup);

#endif //AUDIO_CONTROLLER_SETUP_H