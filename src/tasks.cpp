//
// Created by bendstein on 12/27/2025.
//
#include "tasks.h"

#include <algorithm>

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

        const auto [app_state, bus_index] = *static_cast<bus_sensor_task_param*>(task_param_pointer);
        const auto& sensors = app_state->distance_sensors[bus_index];

        for (uint32_t i = 0; ; i = (i + 1) % std::numeric_limits<uint32_t>::max())
        {
            try
            {
                for (auto s = 0; s < sensors.size(); s++)
                {
                    auto& sensor = sensors[s];
                    const auto sensor_index = sensor->get_index();

                    uint8_t distance;

                    const auto current_max_distance = sensor->get_distance_shift_value();

                    if (sensor->try_update_distance(&distance))
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

                    auto new_tone = musical_note_tone::create_invalid();

                    for (int j = 0; j < SENSOR_TONES_LEN; j++)
                    {
                        if (const auto current_breakpoint = SENSOR_TONES_BREAKPOINTS[j];
                            ratio <= current_breakpoint)
                        {
                            new_tone = SENSOR_TONES[sensor_index][j];
                            break;
                        }
                    }

                    app_state->current_tones[sensor_index] = new_tone;
                }

                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
            catch (std::exception& e)
            {
                FLOGE("[Bus {}] An exception occurred during distance sensor task (n = {}): {}",
                    bus_index,
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

        // const float f[]
        // {
        //     musical_note_freq_hz(musical_note::B, 3),
        //     musical_note_freq_hz(musical_note::D, 4),
        //     musical_note_freq_hz(musical_note::F, 4),
        //     musical_note_freq_hz(musical_note::A, 4)
        // };
        // dac_controller.write(f, sizeof(f) / sizeof(float));
        //
        // do { vTaskDelay(portMAX_DELAY); } while (true);

        //Collect the current tones corresponding to each sensor,
        //and send them to the DAC controller
        fixed_vec<musical_note_tone, dac_controller::TONE_DATA_CAPACITY> tone_data {};

        do
        {
            try
            {
                //Get frequency in Hz for each sensor, then give to DAC controller
                for (const auto t : app_state->current_tones)
                {
                    if (!t.is_invalid() && t != 0) //Frequency = 0 -> Invalid
                    {
                        tone_data.add_to_end(t);
                    }
                }

                dac_controller.write(tone_data);
            }
            catch (std::exception& e)
            {
                FLOGE("{} An exception occurred during DAC write task: {}",
                    dac_controller::LOG_KEY,
                    e.what());
            }

            vTaskDelay(100 / portTICK_PERIOD_US);
        } while (true);

        tone_data.clear();
    }
    catch (std::exception& e)
    {
        FLOGE("An exception occurred during DAC write task: {}", e.what());
    }

    while (true) { vTaskDelay(portMAX_DELAY); }
}