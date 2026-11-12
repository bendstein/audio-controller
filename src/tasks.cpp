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
        uint8_t tone_message_data[2 * tone_message::MESSAGE_LENGTH] {};
        double current_distances[SENSORS_COUNT] {};
        bool distance_changes[SENSORS_COUNT] {};
        frequency_volume current_tones[SENSORS_COUNT] {};

        //Prefix tone message data with start pattern
        for (auto i = 0; i < tone_message::MESSAGE_LENGTH; i++)
            tone_message_data[i] = 0xFF;

        for (auto& current_tone : current_tones)
            current_tone = frequency_volume(musical_note_tone_volume::create_invalid());

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

                    // const auto volume = static_cast<uint8_t>(distance_ratio * MAX_INDIVIDUAL_VOLUME);

                    constexpr auto RATIO_MIN = 0.05;
                    constexpr auto RATIO_MAX = 0.95;

                    if (distance_ratio <= RATIO_MIN || distance_ratio >= RATIO_MAX)
                    {
                        current_tones[s] = frequency_volume(musical_note_tone_volume::create_invalid());
                    }
                    else
                    {
                        // //Vary frequency based on distance
                        // const auto vibrato = distance_ratio * 2;
                        constexpr auto vibrato = 1.;
                        current_tones[s] = frequency_volume(SENSOR_TONES[s][0], MAX_INDIVIDUAL_VOLUME, vibrato);
                    }

                    // constexpr auto volume = MAX_INDIVIDUAL_VOLUME;
                    //
                    // const auto tone = volume >= MAX_INDIVIDUAL_VOLUME_THRESHOLD
                    //     ? musical_note_tone::create_invalid()
                    //     : SENSOR_TONES[s][0];
                    //
                    // current_tones[s] = {
                    //     .tone = tone,
                    //     .volume = volume
                    // };

                    //Send changed tone
                    tone_message(current_tones[s], s).serialize(tone_message_data + tone_message::MESSAGE_LENGTH);

                    if (const auto bytes_sent = uart_write_bytes(UART_NUM_0, tone_message_data, 2 * tone_message::MESSAGE_LENGTH);
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
                        tone_message(current_tones[s], s).serialize(tone_message_data + tone_message::MESSAGE_LENGTH);

                        if (const auto bytes_sent = uart_write_bytes(UART_NUM_0, tone_message_data, 2 * tone_message::MESSAGE_LENGTH);
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
            start_result != ESP_OK)
        {
            FLOGE("{} Failed to start inner tasks (0x{:02X}).", signal_generator::LOG_KEY, start_result);
        }
        else
        {
            FLOGI("{} Inner tasks are started.", signal_generator::LOG_KEY);
        }

        //Init queue for incoming uart messages
        constexpr size_t UART_QUEUE_BUFFER_CAPACITY = 255;
        constexpr size_t UART_QUEUE_BUFFER2_CAPACITY = 255;
        uint8_t uart_queue_buffer[UART_QUEUE_BUFFER_CAPACITY] {};
        uint8_t uart_queue_buffer2[UART_QUEUE_BUFFER2_CAPACITY] {};
        uint8_t uart_read_buffer[tone_message::MESSAGE_LENGTH] {};

        auto uart_data_queue = uart_receive_queue(UART_NUM_0,
            uart_queue_buffer, UART_QUEUE_BUFFER_CAPACITY,
            uart_queue_buffer2, UART_QUEUE_BUFFER2_CAPACITY,
            tone_message::MESSAGE_LENGTH, 0xFF);

        tone_message message_data[signal_generator::TONE_DATA_CAPACITY] {};

        //Buffers with tone data to send to signal_generator, are rotated after each send
        //to prevent writing to a buffer that is in used
        signal_generator_tone tone_data_buf0[signal_generator::TONE_DATA_CAPACITY] {};
        signal_generator_tone tone_data_buf1[signal_generator::TONE_DATA_CAPACITY] {};
        signal_generator_tone tone_data_buf2[signal_generator::TONE_DATA_CAPACITY] {};

        signal_generator_tone* tone_data0 = tone_data_buf0;
        signal_generator_tone* tone_data1 = tone_data_buf1;
        signal_generator_tone* tone_data2 = tone_data_buf2;

        for (uint8_t i = 0; i < signal_generator::TONE_DATA_CAPACITY; i++)
        {
            message_data[i] = tone_message(musical_note_tone_volume::create_invalid(), i);
            tone_data_buf0[i] = {
                .frequency_hz = message_data[i].frequency_hz(),
                .volume = message_data[i].volume
            };
            tone_data_buf1[i] = {
                .frequency_hz = message_data[i].frequency_hz(),
                .volume = message_data[i].volume
            };
            tone_data_buf2[i] = {
                .frequency_hz = message_data[i].frequency_hz(),
                .volume = message_data[i].volume
            };
        }

        // tone_data_buf0[0] = {
        //     .frequency_hz = musical_note_freq_hz(musical_note::A, 4),
        //     .volume = MAX_INDIVIDUAL_VOLUME,
        // };

        // fixed_vec<musical_note_tone_volume, signal_generator::TONE_DATA_CAPACITY> tone_data{};
        // fixed_vec<musical_note_tone_volume, signal_generator::TONE_DATA_CAPACITY> tone_data_prev{};

        // tone_data.add_to_end(musical_note_tone_volume {
        //     .tone = {
        //         .note = musical_note::A,
        //         .octave = 4
        //     },
        //     .volume = MAX_INDIVIDUAL_VOLUME
        // });
        // tone_data.add_to_end(musical_note_tone_volume {
        //     .tone = {
        //         .note = musical_note::C_Sharp,
        //         .octave = 5
        //     },
        //     .volume = MAX_INDIVIDUAL_VOLUME
        // });
        // tone_data.add_to_end(musical_note_tone_volume {
        //     .tone = {
        //         .note = musical_note::E,
        //         .octave = 5
        //     },
        //     .volume = MAX_INDIVIDUAL_VOLUME
        // });
        // tone_data.add_to_end(musical_note_tone_volume {
        //     .tone = {
        //         .note = musical_note::B,
        //         .octave = 5
        //     },
        //     .volume = MAX_INDIVIDUAL_VOLUME
        // });
        //
        // signal_generator.update_tones(tone_data0);
        //
        // while (true) { vTaskDelay(portMAX_DELAY); }

        do
        {
            try
            {
                //Receive new tone data
                bool has_change = false;
                do
                {
                    // VERBOSE("Waiting to read data from uart.");

                    //Read new data from uart rx buffer into message queue
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

                        if (const auto read_item_result = uart_data_queue.next(uart_read_buffer, tone_message::MESSAGE_LENGTH, copied);
                            read_item_result != ESP_OK)
                        {
                            FLOGE("Failed to read from uart queue. (0x{:02X}", read_item_result);
                            vTaskDelay(1 / portTICK_PERIOD_MS);
                            copied = 0;
                        }

                        if (copied > 0)
                        {
                            assert(copied == tone_message::MESSAGE_LENGTH);

                            // FVERBOSE("Read {} bytes ({}, {}, {}, {}) from uart data queue.", copied,
                                // uart_read_buffer[0], uart_read_buffer[1], uart_read_buffer[2], uart_read_buffer[3]);

                            const auto new_message = tone_message::deserialize(uart_read_buffer, copied);

                            if (new_message.index >= signal_generator::TONE_DATA_CAPACITY)
                            {
                                FLOGE("Tone message index {} out of range", new_message.index);
                                continue;
                            }

                            if (auto& current_message = message_data[new_message.index];
                                new_message != current_message)
                            {
                                current_message.copy_from(new_message);
                                // FVERBOSE("Tone data {} => (T: {}, V: {}).", new_index, current_message.data.tone.name(), current_message.data.volume);
                                has_change = true;
                            }
                            // else
                            // {
                            //     FVERBOSE("Tone data {} unchanged.", current_message.index);
                            // }
                        }
                    }
                    while (copied > 0);

                    if (!has_change)
                        vTaskDelay(2 / portTICK_PERIOD_MS);
                }
                while (!has_change);

                //Handle new tone data
                for (const auto& message : message_data)
                {
                    tone_data0[message.index] = {
                        .frequency_hz = message_data[message.index].frequency_hz(),
                        .volume = message_data[message.index].volume
                    };
                }

                signal_generator.update_tones(tone_data0);

                //Swap tone data buffers (3 buffers to prevent conflicting
                //with signal generator while tone data is being copied after queue
                //message is received)
                std::swap(tone_data0, tone_data1);
                std::swap(tone_data1, tone_data2);
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

void tone_message::serialize(uint8_t* buf) const
{
    if (buf == nullptr)
        throw std::invalid_argument("Cannot write tone_message to nullptr.");

    size_t n = 0;

    // x: frequency_dhz / z: volume -> (0000 0000)xxxx xxxx | xxxx xxxx | xxxx xxxx | zzzz zzzz
    buf[n++] = static_cast<uint8_t>((frequency_dhz & 0xFF0000) >> 16);
    buf[n++] = static_cast<uint8_t>((frequency_dhz & 0x00FF00) >> 8);
    buf[n++] = static_cast<uint8_t>(frequency_dhz & 0x0000FF);
    buf[n++] = static_cast<uint8_t>(volume); // zzzz zzzz [7, 0]
    buf[n++] = index;

    assert(n == MESSAGE_LENGTH);
}

tone_message tone_message::deserialize(const uint8_t* buf, const size_t len)
{
    if (buf == nullptr)
        throw std::invalid_argument("Cannot read tone_message from nullptr.");

    assert(len == MESSAGE_LENGTH);

    size_t n = 0;

    const auto b0 = buf[n++];
    const auto b1 = buf[n++];
    const auto b2 = buf[n++];
    const auto b3 = buf[n++];
    const auto b4 = buf[n++];

    assert(n == MESSAGE_LENGTH);

    return {
        static_cast<uint32_t>((b0 << 16) | (b1 << 8) | b2),
        b3,
        b4,
    };
}