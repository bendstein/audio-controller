//
// Created by bendstein on 12/27/2025.
//

#include <algorithm>

#include <cmath>
#include <memory>

#include <freertos/FreeRTOS.h>
#include <freertos/FreeRTOSConfig.h>
#include <freertos/task.h>

#include "audio/dac_controller.h"
#include "tasks.h"
#include "app_common.h"

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
                FVERBOSE("[Bus {}] Looping over {} sensors. (n = {})", bus_index, sensors.size(), i);

                for (auto s = 0; s < sensors.size(); s++)
                {
                    auto& sensor = sensors[s];
                    const auto sensor_index = sensor->get_index();

                    uint8_t distance;

                    const auto current_max_distance = sensor->get_distance_shift_value();

                    uint8_t prev_distance = current_max_distance;

                    if (sensor->try_update_distance(&distance, &prev_distance))
                    {
                        // FLOGI("{} {}cm", app_state->distance_sensors[sensor_index]->get_log_key(), distance);
                    }
                    else
                    {
                        //Use max distance (i.e. no obstruction) on failure
                        distance = current_max_distance;
                    }

                    if (distance != prev_distance)
                    {
                        //Update state
                        app_state->current_distances[sensor_index] = distance;

                        // const auto volume = static_cast<uint8_t>((static_cast<float>(distance) / static_cast<float>(current_max_distance)) * MAX_INDIVIDUAL_VOLUME);
                        const uint8_t volume = sensor_index == 0? 30 : 0;
                        const auto tone = volume >= MAX_INDIVIDUAL_VOLUME_THRESHOLD
                            ? musical_note_tone::create_invalid()
                            : SENSOR_TONES[sensor_index][0];

                        app_state->current_tones[sensor_index] = {
                            .tone = tone,
                            .volume = volume
                        };
                    }

                    // //Get the frequency corresponding to the sensor's given distance
                    // const auto ratio = static_cast<float>(distance) / static_cast<float>(current_max_distance);
                    //
                    // auto new_tone = musical_note_tone::create_invalid();
                    //
                    // for (int j = 0; j < SENSOR_TONES_LEN; j++)
                    // {
                    //     if (const auto current_breakpoint = SENSOR_TONES_BREAKPOINTS[j];
                    //         ratio <= current_breakpoint)
                    //     {
                    //         new_tone = SENSOR_TONES[sensor_index][j];
                    //         break;
                    //     }
                    // }
                    //
                    // app_state->current_tones[sensor_index] = new_tone;
                }

                vTaskDelay(2 / portTICK_PERIOD_MS);
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

        // constexpr auto CHORDS_LEN = 3;
        // const fixed_vec<musical_note_tone, 3> chords[CHORDS_LEN]
        // {
        //     fixed_vec({
        //         musical_note_tone(musical_note::A, 4),
        //         musical_note_tone(musical_note::C, 5),
        //         musical_note_tone(musical_note::E, 5)
        //     }),
        //     fixed_vec({
        //         musical_note_tone(musical_note::B, 4),
        //         musical_note_tone(musical_note::D, 5),
        //         musical_note_tone(musical_note::G, 5),
        //     }),
        //     fixed_vec({
        //         musical_note_tone(musical_note::C, 5),
        //         musical_note_tone(musical_note::G, 5),
        //         musical_note_tone(musical_note::E, 5),
        //     }),
        // };
        //
        // fixed_vec<musical_note_tone, dac_controller::TONE_DATA_CAPACITY> tone_data {};
        //
        // if constexpr (CHORDS_LEN == 1)
        // {
        //     tone_data.clone_from(chords[0]);
        //
        //     FLOGD("Chord {} | [{}]", 0, tone_data.to_string([](const musical_note_tone& t) -> std::string { return t.name(); }));
        //
        //     dac_controller.write(tone_data);
        // }
        // else
        // {
        //     for (size_t i = 0; ; i = (i + 1) % CHORDS_LEN)
        //     {
        //         tone_data.clone_from(chords[i]);
        //
        //         FLOGD("Chord {} | [{}]", i, tone_data.to_string([](const musical_note_tone& t) -> std::string { return t.name(); }));
        //
        //         dac_controller.write(tone_data);
        //
        //         vTaskDelay(3000 / portTICK_PERIOD_MS);
        //     }
        // }

        //Collect the current tones corresponding to each sensor,
        //and send them to the DAC controller
        fixed_vec<musical_note_tone_volume, dac_controller::TONE_DATA_CAPACITY> tone_data {};
        fixed_vec<musical_note_tone_volume, dac_controller::TONE_DATA_CAPACITY> tone_data_prev {};

        do
        {
            try
            {
                tone_data.clear();

                //Get frequency in Hz for each sensor, then give to DAC controller
                for (const auto t : app_state->current_tones)
                {
                    if (!t.is_zero_or_invalid())
                        tone_data.add_to_end(t);
                }

                //Only write to DAC ctrl if data changed
                if (!tone_data.sequence_equals(tone_data_prev))
                {
                    FLOGD("Tone data changed -> [{}]", tone_data.to_string([](const musical_note_tone_volume& t) -> std::string { return t.tone.name(); }));

                    tone_data_prev.clone_from(tone_data); //Overwrite previous with current data for comparison next iteration
                    dac_controller.write(tone_data);
                }
                else
                {
                    FVERBOSE("{} Tone data is unchanged during DAC write task.", dac_controller::LOG_KEY);
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