//
// Created by bendstein on 12/27/2025.
//
#include "tasks.h"
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

    constexpr size_t CHORDS_M = 5;
    constexpr size_t CHORDS_N = 4;

    const double chords[CHORDS_M][CHORDS_N] = {
        {
            GetMusicalNoteFrequency(MusicalNote::C, 5),
            GetMusicalNoteFrequency(MusicalNote::E, 5),
            GetMusicalNoteFrequency(MusicalNote::G, 5)
        },
        {
            GetMusicalNoteFrequency(MusicalNote::A, 4),
            GetMusicalNoteFrequency(MusicalNote::C, 5),
            GetMusicalNoteFrequency(MusicalNote::E, 5),
            GetMusicalNoteFrequency(MusicalNote::F, 5)
        },
        {
            GetMusicalNoteFrequency(MusicalNote::B, 4),
            GetMusicalNoteFrequency(MusicalNote::D, 5),
            GetMusicalNoteFrequency(MusicalNote::F, 5)
        },
        {
            GetMusicalNoteFrequency(MusicalNote::A, 4),
            GetMusicalNoteFrequency(MusicalNote::D, 5),
            GetMusicalNoteFrequency(MusicalNote::F, 5)
        },
        {
            GetMusicalNoteFrequency(MusicalNote::G, 4),
            GetMusicalNoteFrequency(MusicalNote::B, 4),
            GetMusicalNoteFrequency(MusicalNote::D, 4),
            GetMusicalNoteFrequency(MusicalNote::E, 4)
        }
    };

    timeval ts {};
    time_t last_sec = 0;
    size_t j = 0;

    for (uint32_t i = 0; ; i = (i + 1) % std::numeric_limits<uint32_t>::max())
    {
        gettimeofday(&ts, nullptr);
        const long us = ts.tv_usec;

        if (const time_t sec = ts.tv_sec; sec - last_sec > 1)
        {
            last_sec = sec;

            if (i > 0)
                j = (j + 1) % CHORDS_M;
        }

        const auto chord = chords[j];

        double s = 0;
        size_t chord_n = 0;

        for (auto k = 0; k < CHORDS_N; k++)
        {
            if (const auto tone = chord[k]; tone > 0)
            {
                chord_n++;
                const auto sin_k = (1 + sin(2 * M_PI * (tone / 1000000) * us)) / 2.;
                // const auto square_k = sin_k < 0.5? 0 : 1;
                s += sin_k;
                // s += square_k;
            }
        }

        if (chord_n > 0)
            s /= chord_n;

        // //Scale
        const auto x = static_cast<ushort>(floor(0x0FFF * s));

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
