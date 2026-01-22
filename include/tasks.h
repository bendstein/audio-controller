//
// Created by bendstein on 12/27/2025.
//

#ifndef AUDIO_CONTROLLER_TASKS_H
#define AUDIO_CONTROLLER_TASKS_H
#include "setup.h"

static constexpr auto DISTANCE_SENSOR_TASK_PRIORITY = 5;
static constexpr auto DISTANCE_SENSOR_TASK_STACK_SIZE = 0x1000;
static constexpr auto DAC_WRITE_TASK_PRIORITY = 2;
static constexpr auto DAC_WRITE_TASK_STACK_SIZE = 0x1000;

struct sensor_task_param
{
    app_state* state;
    size_t index;
};

struct dac_write_task_param
{
    app_state* state;
};

[[noreturn]]
void distance_sensor_task(void* task_param_pointer);

[[noreturn]]
void dac_write_task(void* task_param_pointer);

#endif //AUDIO_CONTROLLER_TASKS_H