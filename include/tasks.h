//
// Created by bendstein on 12/27/2025.
//

#ifndef AUDIO_CONTROLLER_TASKS_H
#define AUDIO_CONTROLLER_TASKS_H
#include "setup.h"
#include "i2c/mcp4725.h"
#include "audio/musical_distance_sensor.h"
#include "audio/wave_provider.h"

struct dac_task_param {
    const mcp4725::dac* dac;
    const wave_provider* wave;
    std::optional<musical_distance_sensor*> musical_distance_sensors[SENSORS_COUNT];
};

[[noreturn]]
void distance_sensor_task(void* sensor_pointer);

[[noreturn]]
void dac_task(void* dac_pointer);

#endif //AUDIO_CONTROLLER_TASKS_H