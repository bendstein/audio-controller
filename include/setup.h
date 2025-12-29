//
// Created by bendstein on 12/27/2025.
//

#ifndef AUDIO_CONTROLLER_SETUP_H
#define AUDIO_CONTROLLER_SETUP_H

#include "i2c/mcp4725.h"
#include "i2c/gp2y0e02b/distance_sensor.h"

[[nodiscard]] i2c_master_bus_handle_t init_i2c_bus(i2c_port_num_t port, gpio_num_t sda, gpio_num_t scl);
void init_distance_sensors(i2c_master_bus_handle_t bus, const uint8_t sensor_addresses[], size_t sensor_count, std::optional<std::pair<gp2y0e02b::distance_sensor*, TaskHandle_t>> sensor_tasks[]);
void init_dac(i2c_master_bus_handle_t bus, uint8_t dac_address, std::optional<mcp4725::dac*>* dac, std::optional<TaskHandle_t>* task);
bool try_configure_gp2y0e02b(gp2y0e02b::distance_sensor* sensor);

#endif //AUDIO_CONTROLLER_SETUP_H