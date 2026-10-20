//
// Created by bendstein on 1/21/2026.
//

#ifndef AUDIO_CONTROLLER_SINE_TABLE_H
#define AUDIO_CONTROLLER_SINE_TABLE_H
#include <cstdint>
#include <cstddef>
#include <string>

struct SineTable
{
    size_t length;
    const double* data;

    [[nodiscard]] double get_value(const size_t n) const { return (length == 0 || data == nullptr)? 0 : data[n % length]; }
    [[nodiscard]] uint8_t get_value(size_t n, uint8_t scale) const;
    [[nodiscard]] std::string print_table(uint8_t scale) const;
};

#endif //AUDIO_CONTROLLER_SINE_TABLE_H