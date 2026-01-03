//
// Created by bendstein on 12/27/2025.
//
#include "tasks.h"

#include <algorithm>

#include "i2c/mcp4725.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cmath>

#include "../include/audio/notes.h"

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

[[noreturn]]
void dac_task(void* dac_pointer)
{
    const auto [dac, wave, musical_distance_sensors] = *static_cast<dac_task_param*>(dac_pointer);

    timeval ts {};

    for (uint32_t i = 0; ; i = (i + 1) % std::numeric_limits<uint32_t>::max())
    {
        //Get tone for each sensor, then determine frequency at current time
        const tone* tones[SENSORS_COUNT] = {};
        uint8_t actual_tone_count = 0;

        for (auto j = 0; j < SENSORS_COUNT; j++)
        {
            if (const auto maybe_musical_distance_sensor = musical_distance_sensors[j];
                maybe_musical_distance_sensor.has_value())
            {
                const auto musical_distance_sensor = *maybe_musical_distance_sensor;

                if (const auto maybe_tone_j = musical_distance_sensor->get_current_tone();
                    maybe_tone_j.has_value())
                {
                    tones[j] = *maybe_tone_j;
                    actual_tone_count++;
                }
                else
                {
                    tones[j] = new tone(0);
                }
            }
            else
            {
                tones[j] = new tone(0);
            }
        }

        //If no valid sensors, delay and then try again
        if (actual_tone_count == 0)
        {
            vTaskDelay(1 / portTICK_PERIOD_MS);
            continue;
        }

        gettimeofday(&ts, nullptr);
        const auto current_ts = total_microseconds(&ts);

        const auto next = wave == nullptr
            ? 0
            : wave->wave(current_ts, tones, SENSORS_COUNT);

        if (dac->try_write_value(static_cast<ushort>(std::ranges::max(0., floor(0x0FFF * next)))))
        {

        }
    }
}
