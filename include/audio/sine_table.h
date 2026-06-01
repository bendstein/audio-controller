//
// Created by bendstein on 1/21/2026.
//

#ifndef AUDIO_CONTROLLER_SINE_TABLE_H
#define AUDIO_CONTROLLER_SINE_TABLE_H

struct SineTable
{
    size_t length;
    const uint8_t* data;

    [[nodiscard]] uint8_t get_value(const size_t n) const { return (length == 0 || data == nullptr)? 0 : data[n % length]; }
};

#endif //AUDIO_CONTROLLER_SINE_TABLE_H