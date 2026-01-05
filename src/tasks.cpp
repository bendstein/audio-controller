//
// Created by bendstein on 12/27/2025.
//
#include "tasks.h"

#include <algorithm>

#include "i2c/mcp4725.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cmath>
#include <memory>

#include "../include/audio/notes.h"

[[noreturn]]
void distance_sensor_task(void* task_param_pointer)
{
    const auto sensor = static_cast<gp2y0e02b::distance_sensor*>(task_param_pointer);

    for (uint32_t i = 0; ; i = (i + 1) % std::numeric_limits<uint32_t>::max())
    {
        uint8_t distance;

        if (sensor->try_update_distance(&distance))
        { }
    }
}

[[noreturn]]
void dac_task(void* task_param_pointer)
{
    const auto [setup_data] = *static_cast<dac_task_param*>(task_param_pointer);

    timeval ts {};

    for (uint32_t i = 0; ; i = (i + 1) % std::numeric_limits<uint32_t>::max())
    {
        //Get tone as frequency in MHz for each sensor, then determine frequency at current time
        float tones[SENSORS_COUNT] = {};
        uint8_t actual_tone_count = 0;

        for (auto j = 0; j < SENSORS_COUNT; j++)
        {
            if (setup_data->distance_sensors[j].has_value())
            {
                const auto sensor = *setup_data->distance_sensors[j];
                const auto sensor_frequency_mappings = setup_data->piecewise_frequencies[j];

                //Get ratio from sensor's current to max distance, use that to determine tone
                const auto current_distance = sensor.get_distance();
                const auto current_max_distance = sensor.get_distance_shift_value();

                const auto ratio = static_cast<float>(current_distance) / static_cast<float>(current_max_distance);
                const auto tone = piecewise_frequency_range_breakpoint::get_tone(sensor_frequency_mappings, PIECEWISE_FREQUENCY_BREAKPOINT_COUNT, ratio);
                tones[j] = tone;

                actual_tone_count++;
            }
            else
            {
                tones[j] = 0;
            }
        }

        //If no valid tones, delay and then try again
        if (actual_tone_count == 0)
        {
            vTaskDelay(1 / portTICK_PERIOD_MS);
            continue;
        }

        gettimeofday(&ts, nullptr);
        const auto current_ts = total_microseconds(&ts);

        if (const auto next = setup_data->wave->wave(current_ts, tones, SENSORS_COUNT);
            setup_data->dac->try_write_value(static_cast<ushort>(std::ranges::max(0., floor(0x0FFF * next)))))
        {

        }
    }
}
