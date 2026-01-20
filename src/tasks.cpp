//
// Created by bendstein on 12/27/2025.
//
#include "tasks.h"

#include <algorithm>

#include "i2c/mcp4725.h"
#include "audio/dac_controller.h"

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

        const auto [app_state, sensor_index] = *static_cast<sensor_task_param*>(task_param_pointer);

        if (app_state->distance_sensors[sensor_index] == nullptr)
            throw std::invalid_argument("No sensor was provided to distance_sensor_task.");

        for (uint32_t i = 0; ; i = (i + 1) % std::numeric_limits<uint32_t>::max())
        {
            try
            {
                uint8_t distance;

                const auto current_max_distance = app_state->distance_sensors[sensor_index]->get_distance_shift_value();

                if (app_state->distance_sensors[sensor_index]->try_update_distance(&distance))
                {
                    // FLOGI("{} {}cm", app_state->distance_sensors[sensor_index]->get_log_key(), distance);
                }
                else
                {
                    //Use max distance (i.e. no obstruction) on failure
                    distance = current_max_distance;
                }

                //Get the frequency corresponding to the sensor's given distance
                const auto ratio = static_cast<float>(distance) / static_cast<float>(current_max_distance);

                bool assigned = false;

                for (int j = 0; j < SENSOR_TONES_LEN; j++)
                {
                    if (const auto current_breakpoint = SENSOR_TONES_BREAKPOINTS[j];
                        ratio <= current_breakpoint)
                    {
                        app_state->current_frequencies[sensor_index] = SENSOR_TONES[sensor_index][j];
                        assigned = true;
                        break;
                    }
                }

                if (!assigned)
                    app_state->current_frequencies[sensor_index] = 0;

                vTaskDelay(500 / portTICK_PERIOD_US);
            }
            catch (std::exception& e)
            {
                FLOGE("{} An exception occurred during distance sensor task (n = {}): {}",
                    app_state->distance_sensors[sensor_index]->get_log_key(),
                    i,
                    e.what());
            }
        }
    }
    catch (std::exception& e)
    {
        FLOGE("An exception occurred during distance sensor task: {}", e.what());
    }

    while (true) { vTaskDelay(portMAX_DELAY); }
}

[[noreturn]]
void dac_write_task(void* task_param_pointer)
{
    try
    {
        if (task_param_pointer == nullptr)
            throw std::invalid_argument("No data was provided to dac_write_task.");

        const auto [app_state] = *static_cast<dac_write_task_param*>(task_param_pointer);

        if (app_state->dac_ctrl == nullptr)
            throw std::invalid_argument("No dac controller was provided to dac_write_task.");

        auto& dac_controller = *app_state->dac_ctrl;

        //Start dac_controller inner task
        if (const auto start_result = dac_controller.start();
            start_result == dac_controller_start_result::ERR)
        {
            FLOGE("{} Failed to start inner tasks.", dac_controller::LOG_KEY);
        }
        else
        {
            FLOGI("{} Inner tasks are started.", dac_controller::LOG_KEY);
        }

        //Collect the current frequencies corresponding to each sensor,
        //and send them to the DAC controller
        float frequencies[SENSORS_COUNT] {};
        uint8_t frequency_count = 0;

        do
        {
            try
            {
                //Get frequency in Hz for each sensor, then give to DAC controller
                const uint8_t frequency_count_prev = frequency_count;
                frequency_count = 0;

                bool has_change = false;

                for (const auto t : app_state->current_frequencies)
                {
                    if (!check_frequency_equivalency(t, 0)) //Frequency = 0 -> Invalid
                    {
                        if (!check_frequency_equivalency(t, frequencies[frequency_count])) //Frequency has changed since last iteration
                        {
                            has_change = true;
                            frequencies[frequency_count] = t;
                        }

                        frequency_count++;
                    }
                }

                has_change |= frequency_count != frequency_count_prev;

                if (has_change) //DAC only requires update on change
                {
                    dac_controller.write(frequencies, frequency_count);
                }
            }
            catch (std::exception& e)
            {
                FLOGE("{} An exception occurred during DAC write task: {}",
                    dac_controller::LOG_KEY,
                    e.what());
            }

            vTaskDelay(100 / portTICK_PERIOD_US);
        } while (true);
    }
    catch (std::exception& e)
    {
        FLOGE("An exception occurred during DAC write task: {}", e.what());
    }

    while (true) { vTaskDelay(portMAX_DELAY); }
}