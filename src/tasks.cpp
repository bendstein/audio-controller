//
// Created by bendstein on 12/27/2025.
//

#include <algorithm>
#include <cmath>
#include <memory>

#include <freertos/FreeRTOS.h>
#include <freertos/FreeRTOSConfig.h>
#include <freertos/task.h>

#include "tasks.h"
#include "app_common.h"
#include "rtos_utils.h"

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
                        const auto ratio = static_cast<double>(distance) / current_max_distance;

                        //Update state (w/ mutex since current_distances is used in another task).
                        if (const auto handle = rtos_semaphore_handle::try_take(app_state->current_distances_mutex, 10 / portTICK_PERIOD_MS);
                            handle != nullptr)
                        {
                            //Record distance as ratio of max distance
                            FLOGI("{}cm / {}cm ({}%)", distance, current_max_distance, ratio * 100);
                            app_state->current_distances[sensor_index] = ratio;
                        }
                    }
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

void send_audio_task(void* task_param_pointer)
{
    try
    {
        LOGI("Starting send_audio_task.");

        if (task_param_pointer == nullptr)
            throw std::invalid_argument("No data was provided to send audio task.");

        const auto [app_state] = *static_cast<send_audio_task_param*>(task_param_pointer);
        double current_distances[SENSORS_COUNT] {};
        musical_note_tone_volume current_tones[SENSORS_COUNT] {};

        for (auto& current_tone : current_tones)
            current_tone = musical_note_tone_volume::create_invalid();

        //TODO
        do
        {
            try
            {
                //Copy current distances data in mutex since current_distances_mutex is mutated in another task
                if (const auto handle = rtos_semaphore_handle::try_take(app_state->current_distances_mutex);
                    handle != nullptr)
                {
                    memcpy(current_distances, app_state->current_distances, SENSORS_COUNT * sizeof(double));
                }

                //Calculate new tone data
                for (size_t i = 0; i < SENSORS_COUNT; i++)
                {
                    const auto distance_ratio = current_distances[i];

                    // if (i == 0)
                    //     FLOGI("{}%", distance_ratio * 100);

                    const auto volume = static_cast<uint8_t>(distance_ratio * MAX_INDIVIDUAL_VOLUME);

                    const auto tone = volume >= MAX_INDIVIDUAL_VOLUME_THRESHOLD
                        ? musical_note_tone::create_invalid()
                        : SENSOR_TONES[i][0];

                    current_tones[i] = {
                        .tone = tone,
                        .volume = volume
                    };
                }

                //Send tone data
            }
            catch (std::exception& e)
            {
                FLOGE("An exception occurred during send audio task loop: {}", e.what());
            }

            vTaskDelay(5 / portTICK_PERIOD_MS);
        }
        while (true);
    }
    catch (std::exception& e)
    {
        FLOGE("An exception occurred during send audio task: {}", e.what());
    }

    while (true) { vTaskDelay(portMAX_DELAY); }
}

void receive_audio_task(void* task_param_pointer)
{
    try
    {
        LOGI("Starting receive_audio_task.");

        if (task_param_pointer == nullptr)
            throw std::invalid_argument("No data was provided to receive audio task.");

        const auto [app_state] = *static_cast<receive_audio_task_param*>(task_param_pointer);

        if (app_state->signal_gtor == nullptr)
            throw std::invalid_argument("No signal generator was provided to receive audio task.");

        auto& signal_generator = *app_state->signal_gtor;

        //Start signal_generator inner task
        if (const auto start_result = signal_generator.start();
            start_result == signal_generator_start_result::ERR)
        {
            FLOGE("{} Failed to start inner tasks.", signal_generator::LOG_KEY);
        }
        else
        {
            FLOGI("{} Inner tasks are started.", signal_generator::LOG_KEY);
        }

        fixed_vec<musical_note_tone_volume, signal_generator::TONE_DATA_CAPACITY> tone_data{};
        fixed_vec<musical_note_tone_volume, signal_generator::TONE_DATA_CAPACITY> tone_data_prev{};

        for (size_t i = 0, n = 0, o = musical_note_data::MIN_OCTAVE; ; i = (i + 1) % std::numeric_limits<size_t>::max())
        {
            try
            {
                //Receive new tone data
                // tone_data.clear();
                //TODO

                if (i % 1000 == 0)
                {
                    tone_data.clear();

                    const auto note = static_cast<musical_note>(n);
                    n = (n + 1) % static_cast<uint8_t>(musical_note::MAX);

                    for (size_t x = 0; x < signal_generator::TONE_DATA_CAPACITY; x++)
                    {
                        tone_data.add_to_end({
                            musical_note_tone(note, o),
                            MAX_INDIVIDUAL_VOLUME
                        });
                    }

                    if (n == 0)
                        o = std::max(musical_note_data::MIN_OCTAVE, (o + 1) % (musical_note_data::MAX_OCTAVE + 1));
                }

                //Only write to DAC ctrl if data changed
                if (!tone_data.sequence_equals(tone_data_prev))
                {
                    FLOGV("Tone data changed -> [{}]",
                        tone_data.to_string([](const musical_note_tone_volume& t) -> std::string { return t.tone.name(); }));

                    tone_data_prev.clone_from(tone_data);
                    //Overwrite previous with current data for comparison next iteration
                    signal_generator.update_tones(&tone_data_prev);
                }
                else
                {
                    FVERBOSE("{} Tone data is unchanged during receive audio task.", signal_generator::LOG_KEY);
                }
            }
            catch (std::exception& e)
            {
                FLOGE("{} An exception occurred during receive audio task: {}",
                      signal_generator::LOG_KEY,
                      e.what());
            }

            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
    }
    catch (std::exception& e)
    {
        FLOGE("An exception occurred during receive audio task: {}", e.what());
    }

    while (true) { vTaskDelay(portMAX_DELAY); }
}