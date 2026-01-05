//
// Created by bendstein on 1/4/2026.
//

#ifndef AUDIO_CONTROLLER_APP_STATE_H
#define AUDIO_CONTROLLER_APP_STATE_H
#include <memory>
#include <optional>
#include <driver/i2c_types.h>

#include "audio/frequency_range.h"
#include "audio/wave_provider.h"
#include "i2c/mcp4725.h"
#include "i2c/gp2y0e02b/distance_sensor.h"

constexpr size_t SENSORS_COUNT = 1;
constexpr size_t PIECEWISE_FREQUENCY_BREAKPOINT_COUNT = 2;

struct app_state
{
    i2c_master_bus_handle_t i2c_bus_0{};
    i2c_master_bus_handle_t i2c_bus_1{};
    std::optional<gp2y0e02b::distance_sensor> distance_sensors[SENSORS_COUNT];
    std::optional<mcp4725::dac> dac;
    wave_provider* wave;
    std::optional<TaskHandle_t> sensor_tasks[SENSORS_COUNT];
    std::optional<TaskHandle_t> dac_task;
    piecewise_frequency_range_breakpoint piecewise_frequencies[SENSORS_COUNT][PIECEWISE_FREQUENCY_BREAKPOINT_COUNT];
};

#endif //AUDIO_CONTROLLER_APP_STATE_H