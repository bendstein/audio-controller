//
// Created by bendstein on 12/27/2025.
//
#include "tasks.h"

#include <algorithm>

#include "i2c/mcp4725.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cmath>

#include "notes.h"

bool try_create_distance_sensor_task(const std::string& task_name,
                                     gp2y0e02b::distance_sensor* sensor,
                                     BaseType_t* result_code,
                                     TaskHandle_t* task_handle)
{
    constexpr uint32_t STACK_SIZE = 0x1800;
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
    constexpr uint32_t STACK_SIZE = 0x1800;
    constexpr UBaseType_t PRIORITY = 4;

    auto task_param = dac_task_param {
        .dac = dac,
        .wave_provider = mcp4725::wave_providers::sawtooth_wave
    };
    
    *result_code = xTaskCreate(
        dac_task,
        task_name.c_str(),
        STACK_SIZE,
        &task_param,
        PRIORITY,
        task_handle
    );

    return *result_code == pdPASS;
}

[[noreturn]]
void dac_task(void* dac_pointer)
{
    const auto [dac, wave_provider] = *static_cast<dac_task_param*>(dac_pointer);

    constexpr auto tempo_bpm = 240;
    const long beat_time_us = static_cast<long>(std::floor(1. / tempo_bpm * 60 * 1000000));

    const std::pair<tone*, uint8_t> melody[] = {
        { new tone(musical_note::C, 5), 1 },
        { new tone(musical_note::C_Sharp, 5), 2 },
        { new tone(musical_note::D, 5), 1 },
        { new tone(musical_note::D_Sharp, 5), 3 },
        { new tone(musical_note::E, 5), 1 },
        { new tone(musical_note::F, 5), 4 },
        { new tone(musical_note::F_Sharp, 5), 1 },
        { new tone(musical_note::G, 5), 5 },
        { new tone(musical_note::G_Sharp, 5), 1 },
        { new tone(musical_note::A, 5), 6 },
        { new tone(musical_note::B_Flat, 5), 1 },
        { new tone(musical_note::B, 5), 8 },
    };

    timeval ts {};
    gettimeofday(&ts, nullptr);
    auto prev_ts = total_microseconds(&ts);

    for (uint32_t i = 0; ; i = (i + 1) % std::numeric_limits<uint32_t>::max())
    {
        constexpr auto melody_len = sizeof(melody) / sizeof(std::pair<tone*, uint8_t>);

        const auto [current_tone, current_counter] = melody[i % melody_len];

        for (auto j = 0; j < current_counter; j++)
        {
            gettimeofday(&ts, nullptr);
            long long current_ts;

            do
            {
                // logi(NAMEOF(dac_task), std::format("{} {} / {}", current_ts, prev_ts, beat_time_us));

                gettimeofday(&ts, nullptr);
                current_ts = total_microseconds(&ts);

                const auto next = wave_provider == nullptr
                    ? 0
                    : wave_provider(current_ts, current_tone, 1);

                // const auto s = (1 + sin(2 * M_PI * (current_frequency / 1000000) * current_ts)) / 2.;

                if (dac->try_write_value(static_cast<ushort>(std::ranges::max(0., floor(0x0FFF * next)))))
                {

                }
            } while (current_ts - prev_ts < beat_time_us);

            prev_ts = current_ts;
        }
    }

    // timeval ts {};
    // time_t last_sec = 0;
    // size_t j = 0;
    //
    // for (uint32_t i = 0; ; i = (i + 1) % std::numeric_limits<uint32_t>::max())
    // {
    //     gettimeofday(&ts, nullptr);
    //     const long us = ts.tv_usec;
    //
    //     if (const time_t sec = ts.tv_sec; sec - last_sec > 1)
    //     {
    //         last_sec = sec;
    //
    //         if (i > 0)
    //             j = (j + 1) % CHORDS_M;
    //     }
    //
    //     const auto chord = chords[j];
    //
    //     double s = 0;
    //     size_t chord_n = 0;
    //
    //     for (auto k = 0; k < CHORDS_N; k++)
    //     {
    //         if (const auto tone = chord[k]; tone > 0)
    //         {
    //             chord_n++;
    //             const auto sin_k = (1 + sin(2 * M_PI * (tone / 1000000) * us)) / 2.;
    //             // const auto square_k = sin_k < 0.5? 0 : 1;
    //             s += sin_k;
    //             // s += square_k;
    //         }
    //     }
    //
    //     if (chord_n > 0)
    //         s /= chord_n;
    //
    //     // //Scale
    //     const auto x = static_cast<ushort>(floor(0x0FFF * s));
    //
    //     if (dac->try_write_value(x))
    //     {
    //
    //     }
    //     // if (dac->ping())
    //     // {
    //     //
    //     // }
    //     //
    //     // vTaskDelay(100 / portTICK_PERIOD_MS);
    // }
}
