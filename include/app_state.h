//
// Created by bendstein on 1/4/2026.
//

#ifndef AUDIO_CONTROLLER_APP_STATE_H
#define AUDIO_CONTROLLER_APP_STATE_H

#include "fixed_vec.h"
#include "audio/dac_controller.h"
#include "audio/signal_generator.h"
#include "i2c/gp2y0e02b/distance_sensor.h"

constexpr size_t BUS_COUNT = 2;
constexpr size_t SENSORS_COUNT = 2;
constexpr size_t SENSOR_TONES_LEN = 1;
constexpr uint8_t MAX_INDIVIDUAL_VOLUME = 0xCA;
constexpr uint8_t MAX_INDIVIDUAL_VOLUME_THRESHOLD = static_cast<uint8_t>(0.95 * MAX_INDIVIDUAL_VOLUME);

constexpr musical_note_tone SENSOR_TONES[SENSORS_COUNT][SENSOR_TONES_LEN]
{
    { musical_note_tone(musical_note::A, 4) },
    { musical_note_tone(musical_note::F, 4) }
};

constexpr float SENSOR_TONES_BREAKPOINTS[SENSOR_TONES_LEN] = {
    1
};

struct app_state
{
    std::unique_ptr<i2c_master> i2c_buses[BUS_COUNT];
    // std::unique_ptr<dac_controller> dac_ctrl;
    std::unique_ptr<signal_generator> signal_gtor;
    std::unique_ptr<TaskHandle_t> dac_write_task;
    fixed_vec<std::unique_ptr<gp2y0e02b::distance_sensor>, SENSORS_COUNT> distance_sensors[BUS_COUNT];
    std::unique_ptr<TaskHandle_t> sensor_tasks[BUS_COUNT];
    musical_note_tone_volume current_tones[SENSORS_COUNT];
    uint8_t current_distances[SENSORS_COUNT];
};

enum struct main_method_type
{
    standard,
    configure_gp2y0e02b
};

constexpr const char* main_method_type_names[] = {
    NAMEOF(main_method_type::standard),
    NAMEOF(main_method_type::configure_gp2y0e02b)
};

//Which mode the application is running in
#ifdef CFG_GP2Y0E02B_I2C_ADDR
constexpr auto MAIN_METHOD_TYPE = main_method_type::configure_gp2y0e02b;
#else
constexpr auto MAIN_METHOD_TYPE = main_method_type::standard;
#endif

constexpr auto MAIN_METHOD_TYPE_NAME = main_method_type_names[static_cast<uint32_t>(MAIN_METHOD_TYPE)];

#endif //AUDIO_CONTROLLER_APP_STATE_H