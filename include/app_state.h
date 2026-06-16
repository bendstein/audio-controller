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

struct app_state
{
    std::unique_ptr<i2c_master> i2c_buses[BUS_COUNT];
    std::unique_ptr<dac_controller> dac_ctrl;
    std::unique_ptr<TaskHandle_t> dac_write_task;
    fixed_vec<std::unique_ptr<gp2y0e02b::distance_sensor>, SENSORS_COUNT> distance_sensors[BUS_COUNT];
    std::unique_ptr<TaskHandle_t> sensor_tasks[BUS_COUNT];
    musical_note_tone current_tones[SENSORS_COUNT];
};

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


enum struct main_method_type
{
    standard,
    configure_gp2y0e02b,
    gp2y0e02b_interface
};

constexpr const char* main_method_type_names[] = {
    NAMEOF(main_method_type::standard),
    NAMEOF(main_method_type::configure_gp2y0e02b),
    NAMEOF(main_method_type::gp2y0e02b_interface)
};

//Which mode the application is running in
#ifdef CFG_GP2Y0E02B_I2C_ADDR
constexpr auto MAIN_METHOD_TYPE = main_method_type::configure_gp2y0e02b;
#elifdef GP2Y0E02B_INTERFACE
constexpr auto MAIN_METHOD_TYPE = main_method_type::gp2y0e02b_interface;
#else
constexpr auto MAIN_METHOD_TYPE = main_method_type::standard;
#endif

constexpr auto MAIN_METHOD_TYPE_NAME = main_method_type_names[static_cast<uint32_t>(MAIN_METHOD_TYPE)];

#endif //AUDIO_CONTROLLER_APP_STATE_H