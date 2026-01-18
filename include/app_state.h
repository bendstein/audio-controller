//
// Created by bendstein on 1/4/2026.
//

#ifndef AUDIO_CONTROLLER_APP_STATE_H
#define AUDIO_CONTROLLER_APP_STATE_H
#include <driver/i2c_types.h>

#include "audio/dac_controller.h"
#include "i2c/gp2y0e02b/distance_sensor.h"

constexpr size_t BUS_COUNT = 2;
constexpr size_t SENSORS_COUNT = 2;
constexpr size_t SENSOR_TONES_LEN = 3;

const float SENSOR_TONES[SENSORS_COUNT][SENSOR_TONES_LEN]
{
    { musical_note_freq_hz(musical_note::A, 3), musical_note_freq_hz(musical_note::A, 4), 0 },
    { musical_note_freq_hz(musical_note::C, 3), musical_note_freq_hz(musical_note::C, 4), 0 }
};
constexpr float SENSOR_TONES_BREAKPOINTS[SENSOR_TONES_LEN] = {
    0.4,
    0.8,
    1
};

struct app_state
{
    i2c_master_bus_handle_t i2c_buses[BUS_COUNT];
    std::unique_ptr<dac_controller> dac_ctrl;
    std::unique_ptr<TaskHandle_t> dac_write_task;
    std::unique_ptr<gp2y0e02b::distance_sensor> distance_sensors[SENSORS_COUNT];
    std::unique_ptr<TaskHandle_t> sensor_tasks[SENSORS_COUNT];
    float current_frequencies[SENSORS_COUNT];
};
#endif //AUDIO_CONTROLLER_APP_STATE_H