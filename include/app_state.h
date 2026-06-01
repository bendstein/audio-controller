//
// Created by bendstein on 1/4/2026.
//

#ifndef AUDIO_CONTROLLER_APP_STATE_H
#define AUDIO_CONTROLLER_APP_STATE_H

#include "fixed_vec.h"
#include "audio/dac_controller.h"
#include "i2c/gp2y0e02b/distance_sensor.h"

constexpr size_t BUS_COUNT = 2;
constexpr size_t SENSORS_COUNT = 2;
constexpr size_t SENSOR_TONES_LEN = 4;

constexpr musical_note_tone SENSOR_TONES[SENSORS_COUNT][SENSOR_TONES_LEN]
{
    { musical_note_tone(musical_note::F, 3), musical_note_tone(musical_note::A, 3), musical_note_tone(musical_note::C, 4), musical_note_tone::create_zero() },
    { musical_note_tone(musical_note::D, 3), musical_note_tone(musical_note::G, 3), musical_note_tone(musical_note::B_Flat, 3), musical_note_tone::create_zero() }
};

constexpr float SENSOR_TONES_BREAKPOINTS[SENSOR_TONES_LEN] = {
    0.267,
    0.534,
    0.8,
    1
};

struct app_state
{
    std::unique_ptr<i2c_master> i2c_buses[BUS_COUNT];
    std::unique_ptr<dac_controller> dac_ctrl;
    std::unique_ptr<TaskHandle_t> dac_write_task;
    fixed_vec<std::unique_ptr<gp2y0e02b::distance_sensor>, SENSORS_COUNT> distance_sensors[BUS_COUNT];
    std::unique_ptr<TaskHandle_t> sensor_tasks[BUS_COUNT];
    musical_note_tone current_tones[SENSORS_COUNT];
};
#endif //AUDIO_CONTROLLER_APP_STATE_H