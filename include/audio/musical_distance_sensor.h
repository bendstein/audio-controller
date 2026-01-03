//
// Created by bendstein on 1/3/2026.
//

#ifndef AUDIO_CONTROLLER_MUSICAL_DISTANCE_SENSOR_H
#define AUDIO_CONTROLLER_MUSICAL_DISTANCE_SENSOR_H
#include "frequency_range.h"
#include "i2c/gp2y0e02b/distance_sensor.h"

class musical_distance_sensor
{
    const gp2y0e02b::distance_sensor* sensor;
    const frequency_range* sound_data;
public:
    musical_distance_sensor(const gp2y0e02b::distance_sensor* sensor, const frequency_range* sound_data)
        : sensor(sensor), sound_data(sound_data)
    {
        assert(sound_data != nullptr);
    }

    [[nodiscard]]
    std::optional<const tone*> get_current_tone() const
    {
        //Get ratio of current distance to max distance
        const auto current_distance = sensor->get_distance();
        const auto current_max_distance = sensor->get_distance_shift_value();

        const auto ratio = (1. * current_distance) / current_max_distance;

        return sound_data->get_tone(ratio);
    }
};

#endif //AUDIO_CONTROLLER_MUSICAL_DISTANCE_SENSOR_H