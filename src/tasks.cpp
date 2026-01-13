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
    try
    {
        if (task_param_pointer == nullptr)
            throw std::invalid_argument("No data was provided to distance_sensor_task.");

        const auto [setup_data, sensor_index] = *static_cast<sensor_task_param*>(task_param_pointer);

        const auto maybe_sensor = &setup_data->distance_sensors[sensor_index];

        if (maybe_sensor == nullptr || *maybe_sensor == nullptr)
            throw std::invalid_argument("No sensor was provided to distance_sensor_task.");

        auto& sensor = **maybe_sensor;

        for (uint32_t i = 0; ; i = (i + 1) % std::numeric_limits<uint32_t>::max())
        {
            try
            {
                uint8_t distance;

                if (sensor.try_update_distance(&distance))
                { }

                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
            catch (std::exception& e)
            {
                loge(NAMEOF(distance_sensor_task), std::format("{} An exception occurred during distance sensor task (n = {}): {}",
                    sensor.get_log_key(),
                    i,
                    e.what()));
            }
        }
    }
    catch (std::exception& e)
    {
        loge(NAMEOF(distance_sensor_task), std::format("An exception occurred during distance sensor task: {}", e.what()));
    }

    while (true) { vTaskDelay(portMAX_DELAY); }
}

[[noreturn]]
void dac_task(void* task_param_pointer)
{
    try
    {
        if (task_param_pointer == nullptr)
            throw std::invalid_argument("No data was provided to dac_task.");

        const auto [setup_data] = *static_cast<dac_task_param*>(task_param_pointer);

        const auto maybe_dac = &setup_data->dac;

        if (*maybe_dac == nullptr)
            throw std::invalid_argument("No dac was provided to dac_task.");

        const auto& dac = **maybe_dac;

        timeval ts {};

        for (uint32_t i = 0; ; i = (i + 1) % std::numeric_limits<uint32_t>::max())
        {
            try
            {
                //Get tone as frequency in MHz for each sensor, then determine frequency at current time
                float tones[SENSORS_COUNT] = {};
                uint8_t actual_tone_count = 0;

                for (auto j = 0; j < SENSORS_COUNT; j++)
                {
                    if (setup_data->distance_sensors[j] != nullptr)
                    {
                        const auto sensor_frequency_mappings = setup_data->piecewise_frequencies[j];

                        //Get ratio from sensor's current to max distance, use that to determine tone
                        const auto current_distance = setup_data->distance_sensors[j]->get_distance();
                        const auto current_max_distance = setup_data->distance_sensors[j]->get_distance_shift_value();

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
                    vTaskDelay(50 / portTICK_PERIOD_MS);
                    continue;
                }

                gettimeofday(&ts, nullptr);
                const auto current_ts = total_microseconds(&ts);

                if (const auto next = setup_data->wave->wave(current_ts, tones, SENSORS_COUNT);
                    dac.try_write_value(static_cast<ushort>(std::ranges::max(0., floor(0x0FFF * next)))))
                {

                }

                vTaskDelay(10 / portTICK_PERIOD_US);
            }
            catch (std::exception& e)
            {
                loge(NAMEOF(dac_task), std::format("{} An exception occurred during DAC task (n = {}): {}",
                    dac.get_log_key(),
                    i,
                    e.what()));
            }
        }
    }
    catch (std::exception& e)
    {
        loge(NAMEOF(dac_task), std::format("An exception occurred during DAC task: {}", e.what()));
    }

    while (true) { vTaskDelay(portMAX_DELAY); }
}