//
// Created by bendstein on 12/27/2025.
//

#ifndef AUDIO_CONTROLLER_TASKS_H
#define AUDIO_CONTROLLER_TASKS_H
#include "i2c/gp2y0e02b/distance_sensor.h"
#include "i2c/mcp4725.h"
#include "app_common.h"

bool try_create_distance_sensor_task(const std::string& task_name, gp2y0e02b::distance_sensor* sensor, BaseType_t* result_code, TaskHandle_t* task_handle);
bool try_create_dac_task(const std::string& task_name, mcp4725::dac* dac, BaseType_t* result_code, TaskHandle_t* task_handle);

[[noreturn]]
void distance_sensor_task(void* sensor_pointer);

[[noreturn]]
void dac_task(void* dac_pointer);

#endif //AUDIO_CONTROLLER_TASKS_H