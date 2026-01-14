//
// Created by bendstein on 1/4/2026.
//

#ifndef AUDIO_CONTROLLER_APP_STATE_H
#define AUDIO_CONTROLLER_APP_STATE_H
#include <driver/i2c_types.h>

#include "audio/dac_controller.h"
#include "audio/frequency_range.h"
#include "audio/wave_provider.h"
#include "i2c/mcp4725.h"
#include "i2c/gp2y0e02b/distance_sensor.h"

struct app_state
{
    i2c_master_bus_handle_t i2c_bus_0{};
    i2c_master_bus_handle_t i2c_bus_1{};
    std::unique_ptr<gp2y0e02b::distance_sensor> distance_sensors[SENSORS_COUNT];
    // std::unique_ptr<mcp4725::dac> dac;
    std::unique_ptr<dac_controller> dac_ctrl;
    wave_provider* wave;
    std::unique_ptr<TaskHandle_t> sensor_tasks[SENSORS_COUNT];
    // std::unique_ptr<TaskHandle_t> dac_task;
    std::unique_ptr<TaskHandle_t> dac_write_task;
    piecewise_frequency_range_breakpoint piecewise_frequencies[SENSORS_COUNT][PIECEWISE_FREQUENCY_BREAKPOINT_COUNT];
};

#endif //AUDIO_CONTROLLER_APP_STATE_H