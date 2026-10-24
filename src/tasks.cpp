//
// Created by bendstein on 12/27/2025.
//

#include <algorithm>
#include <cmath>
#include <memory>

#include "app_common.h"

#include <freertos/FreeRTOS.h>
#include <freertos/FreeRTOSConfig.h>
#include <freertos/task.h>
#include <driver/uart.h>

#include "tasks.h"
#include "rtos_utils.h"
#include "uart_receive_queue.h"

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
                            // FLOGI("{}cm / {}cm ({}%)", distance, current_max_distance, ratio * 100);
                            app_state->current_distances[sensor_index] = ratio;
                            app_state->distance_changes[sensor_index] = true;
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
        uint8_t tone_message_data[2 * sizeof(uint32_t)] {};
        double current_distances[SENSORS_COUNT] {};
        bool distance_changes[SENSORS_COUNT] {};
        musical_note_tone_volume current_tones[SENSORS_COUNT] {};

        //Prefix tone message data with start pattern
        tone_message_data[0] = 0xFF;
        tone_message_data[1] = 0xFF;
        tone_message_data[2] = 0xFF;
        tone_message_data[3] = 0xFF;

        for (auto& current_tone : current_tones)
            current_tone = musical_note_tone_volume::create_invalid();

        for (size_t i = 0, r = 0; ; i = (i + 1) % std::numeric_limits<size_t>::max())
        {
            try
            {
                //Copy current distances data in mutex since current_distances_mutex is mutated in another task
                if (const auto handle = rtos_semaphore_handle::try_take(app_state->current_distances_mutex);
                    handle != nullptr)
                {
                    memcpy(current_distances, app_state->current_distances, SENSORS_COUNT * sizeof(double));

                    //Copy and clear list of changes
                    memcpy(distance_changes, app_state->distance_changes, SENSORS_COUNT * sizeof(bool));
                    memset(app_state->distance_changes, false, SENSORS_COUNT * sizeof(bool));
                }

                //Calculate new tone data
                bool any_change = false;

                for (uint8_t s = 0; s < SENSORS_COUNT; s++)
                {
                    //Skip if unchanged
                    if (!distance_changes[s])
                        continue;

                    any_change = true;

                    const auto distance_ratio = current_distances[s];

                    // if (i == 0)
                    //     FLOGI("{}%", distance_ratio * 100);

                    const auto volume = static_cast<uint8_t>(distance_ratio * MAX_INDIVIDUAL_VOLUME);

                    const auto tone = volume >= MAX_INDIVIDUAL_VOLUME_THRESHOLD
                        ? musical_note_tone::create_invalid()
                        : SENSOR_TONES[s][0];

                    current_tones[s] = {
                        .tone = tone,
                        .volume = volume
                    };

                    //Send changed tone
                    tone_message {
                        .data = current_tones[s],
                        .index = s
                    }.write_bytes(tone_message_data + sizeof(uint32_t));

                    if (const auto bytes_sent = uart_write_bytes(UART_NUM_0, tone_message_data, 2 * sizeof(uint32_t));
                        bytes_sent < 0)
                    {
                        FLOGE("Failed to send tone message {} ({}).", s, bytes_sent);
                    }
                }

                //If no change, occasionally take opportunity to resend everything
                if (!any_change && (i - r) > 100)
                {
                    r = i;

                    for (uint8_t s = 0; s < SENSORS_COUNT; s++)
                    {
                        tone_message {
                            .data = current_tones[s],
                            .index = s
                        }.write_bytes(tone_message_data + sizeof(uint32_t));

                        if (const auto bytes_sent = uart_write_bytes(UART_NUM_0, tone_message_data, 2 * sizeof(uint32_t));
                            bytes_sent < 0)
                        {
                            FLOGE("Failed to send tone message {} ({}).", s, bytes_sent);
                        }
                    }
                }
            }
            catch (std::exception& e)
            {
                FLOGE("An exception occurred during send audio task loop: {}", e.what());
            }

            vTaskDelay(5 / portTICK_PERIOD_MS);
        }
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

        constexpr size_t UART_QUEUE_BUFFER_CAPACITY = 255;
        constexpr size_t UART_QUEUE_BUFFER2_CAPACITY = 255;
        uint8_t uart_queue_buffer[UART_QUEUE_BUFFER_CAPACITY] {};
        uint8_t uart_queue_buffer2[UART_QUEUE_BUFFER2_CAPACITY] {};
        uint8_t uart_read_buffer[sizeof(uint32_t)] {};

        auto uart_data_queue = uart_receive_queue(UART_NUM_0,
            uart_queue_buffer, UART_QUEUE_BUFFER_CAPACITY,
            uart_queue_buffer2, UART_QUEUE_BUFFER2_CAPACITY,
            sizeof(uint32_t), 0xFF);

        tone_message message_data[signal_generator::TONE_DATA_CAPACITY] {};

        for (uint8_t i = 0; i < signal_generator::TONE_DATA_CAPACITY; i++)
        {
            message_data[i] = {
                .data = musical_note_tone_volume::create_invalid(),
                .index = i
            };
        }

        fixed_vec<musical_note_tone_volume, signal_generator::TONE_DATA_CAPACITY> tone_data{};
        fixed_vec<musical_note_tone_volume, signal_generator::TONE_DATA_CAPACITY> tone_data_prev{};

        do
        {
            try
            {
                //Receive new tone data
                bool has_change = false;
                do
                {
                    // VERBOSE("Waiting to read data from uart.");

                    //Read new data from uart rx buffer
                    if (const auto fill_queue_result = uart_data_queue.fill_buffer(portMAX_DELAY);
                        fill_queue_result != ESP_OK)
                    {
                        FLOGE("Failed to fill uart queue. (0x{:02X}", fill_queue_result);
                        vTaskDelay(1 / portTICK_PERIOD_MS);

                        continue;
                    }

                    //Handle new messages
                    size_t copied;

                    do
                    {
                        copied = 0;

                        if (const auto read_item_result = uart_data_queue.next(uart_read_buffer, sizeof(uint32_t), copied);
                            read_item_result != ESP_OK)
                        {
                            FLOGE("Failed to read from uart queue. (0x{:02X}", read_item_result);
                            vTaskDelay(1 / portTICK_PERIOD_MS);
                            copied = 0;
                        }

                        if (copied > 0)
                        {
                            assert(copied == sizeof(uint32_t));

                            // FVERBOSE("Read {} bytes ({}, {}, {}, {}) from uart data queue.", copied,
                                // uart_read_buffer[0], uart_read_buffer[1], uart_read_buffer[2], uart_read_buffer[3]);

                            const auto [new_data, new_index] = tone_message::read_bytes(uart_read_buffer, copied);

                            if (new_index >= signal_generator::TONE_DATA_CAPACITY)
                            {
                                FLOGE("Tone message index {} out of range", new_index);
                                continue;
                            }

                            if (auto& current_message = message_data[new_index];
                                new_data != current_message.data)
                            {
                                current_message.data = new_data;
                                // FVERBOSE("Tone data {} => (T: {}, V: {}).", new_index, current_message.data.tone.name(), current_message.data.volume);
                                has_change = true;
                            }
                            else
                            {
                                // FVERBOSE("Tone data {} unchanged.", current_message.index);
                            }
                        }
                    }
                    while (copied > 0);

                    if (!has_change)
                        vTaskDelay(2 / portTICK_PERIOD_MS);
                }
                while (!has_change);

                tone_data.clear();

                for (const auto& [data, _] : message_data)
                {
                    if (!data.is_zero_or_invalid())
                    {
                        tone_data.add_to_end(data);
                    }
                }

                //Only write to DAC ctrl if data changed
                FVERBOSE("Tone data changed -> [{}]",
                        tone_data.to_string([](const musical_note_tone_volume& t) -> std::string { return t.tone.name(); }));

                tone_data_prev.clone_from(tone_data);
                //Overwrite previous with current data for comparison next iteration
                signal_generator.update_tones(&tone_data_prev);
            }
            catch (std::exception& e)
            {
                FLOGE("{} An exception occurred during receive audio task: {}",
                      signal_generator::LOG_KEY,
                      e.what());
            }

            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        while (true);
    }
    catch (std::exception& e)
    {
        FLOGE("An exception occurred during receive audio task: {}", e.what());
    }

    while (true) { vTaskDelay(portMAX_DELAY); }
}

void tone_message::write_bytes(uint8_t* buf) const
{
    if (buf == nullptr)
        throw std::invalid_argument("Cannot write tone_message to nullptr.");

    size_t n = 0;

    buf[n++] = static_cast<uint8_t>(data.tone.note);
    buf[n++] = data.tone.octave;
    buf[n++] = data.volume;
    buf[n++] = index;

    assert(n == sizeof(uint32_t));
}

tone_message tone_message::read_bytes(uint8_t* buf, const size_t len)
{
    if (buf == nullptr)
        throw std::invalid_argument("Cannot read tone_message from nullptr.");

    assert(len == sizeof(uint32_t));

    size_t n = 0;

    return tone_message {
        .data = {
            .tone = {
                .note = static_cast<musical_note>(buf[n++]),
                .octave = buf[n++]
            },
            .volume = buf[n++]
        },
        .index = buf[n++]
    };

    assert(n == sizeof(uint32_t));
}