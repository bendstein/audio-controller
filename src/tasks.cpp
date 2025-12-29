//
// Created by bendstein on 12/27/2025.
//
#include "tasks.h"
#include "i2c/mcp4725.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cmath>

bool try_create_distance_sensor_task(const std::string& task_name,
    gp2y0e02b::distance_sensor* sensor,
    BaseType_t* result_code,
    TaskHandle_t* task_handle)
{
    constexpr uint32_t STACK_SIZE = 0x0800;
    constexpr UBaseType_t PRIORITY = 4;

    *result_code = xTaskCreate(
        distance_sensor_task,
        task_name.c_str(),
        STACK_SIZE,
        sensor,
        PRIORITY,
        task_handle
    );

    return *result_code == pdPASS;
}

[[noreturn]]
void distance_sensor_task(void* sensor_pointer)
{
    const auto sensor = static_cast<gp2y0e02b::distance_sensor*>(sensor_pointer);

    for (uint32_t i = 0; ; i = (i + 1) % std::numeric_limits<uint32_t>::max())
    {
        uint8_t distance;

        if (sensor->try_update_distance(&distance))
        {

        }
    }
}

bool try_create_dac_task(const std::string& task_name,
    mcp4725::dac* dac,
    BaseType_t* result_code,
    TaskHandle_t* task_handle)
{
    constexpr uint32_t STACK_SIZE = 0x0800;
    constexpr UBaseType_t PRIORITY = 4;

    *result_code = xTaskCreate(
        dac_task,
        task_name.c_str(),
        STACK_SIZE,
        dac,
        PRIORITY,
        task_handle
    );

    return *result_code == pdPASS;
}

[[noreturn]]
void dac_task(void* dac_pointer)
{
    const auto dac = static_cast<mcp4725::dac*>(dac_pointer);

    for (uint32_t i = 0; ; i = (i + 1) % std::numeric_limits<uint32_t>::max())
    {
        const auto x = static_cast<ushort>(floor(
            (0x0FFF / 2.)
            * (1 * sin(400 * i))
        ));

        if (dac->try_write_value(x))
        {

        }
        // if (dac->ping())
        // {
        //
        // }
        //
        // vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
