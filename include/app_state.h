//
// Created by bendstein on 1/4/2026.
//

#ifndef AUDIO_CONTROLLER_APP_STATE_H
#define AUDIO_CONTROLLER_APP_STATE_H

#include "fixed_vec.h"
#include "audio/signal_generator.h"
#include "i2c/i2c.h"
#include "i2c/gp2y0e02b/distance_sensor.h"

constexpr size_t BUS_COUNT = 2;
constexpr size_t SENSORS_COUNT = 2;
constexpr size_t SENSOR_TONES_LEN = 1;
constexpr size_t MAX_VOICES = signal_generator::TONE_DATA_CAPACITY;
constexpr uint8_t MAX_INDIVIDUAL_VOLUME = 250 / MAX_VOICES;
constexpr uint8_t MAX_INDIVIDUAL_VOLUME_THRESHOLD = static_cast<uint8_t>(0.95 * MAX_INDIVIDUAL_VOLUME);

constexpr musical_note_tone SENSOR_TONES[SENSORS_COUNT][SENSOR_TONES_LEN]
{
    { musical_note_tone(musical_note::A, 4) },
    { musical_note_tone(musical_note::F, 4) }
};

constexpr float SENSOR_TONES_BREAKPOINTS[SENSOR_TONES_LEN] = {
    1
};

struct app_state_controller
{
    std::unique_ptr<i2c_master> i2c_buses[BUS_COUNT];
    fixed_vec<std::unique_ptr<gp2y0e02b::distance_sensor>, SENSORS_COUNT> distance_sensors[BUS_COUNT];
    TaskHandle_t sensor_tasks[BUS_COUNT];
    double current_distances[SENSORS_COUNT];
    bool distance_changes[SENSORS_COUNT];
    TaskHandle_t send_audio_task;
    SemaphoreHandle_t current_distances_mutex;
    QueueHandle_t uart_queue;
};

struct app_state_audio
{
    std::unique_ptr<signal_generator> signal_gtor;
    TaskHandle_t receive_audio_task;
    QueueHandle_t uart_queue;
};

#endif //AUDIO_CONTROLLER_APP_STATE_H