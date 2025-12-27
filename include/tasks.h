//
// Created by bendstein on 12/27/2025.
//

#ifndef AUDIO_CONTROLLER_TASKS_H
#define AUDIO_CONTROLLER_TASKS_H
#include "gp2y0e02b.h"
#include "app_common.h"

bool try_create_distance_sensor_task(const std::string& task_name, gp2y0e02b::distance_sensor* sensor, BaseType_t* result_code, TaskHandle_t* task_handle);

[[noreturn]]
void distance_sensor_task(void* sensor_pointer);

#endif //AUDIO_CONTROLLER_TASKS_H