//
// Created by bendstein on 10/20/2026.
//
#include <cmath>
#include <sstream>
#include <format>
#include <algorithm>

#include "audio/sine_table.h"

uint8_t SineTable::get_value(const size_t n, const uint8_t scale) const
{
    const auto scaled = get_value(n) * scale;

    return static_cast<uint8_t>(std::clamp(
        scaled < 128? std::ceil(scaled) : std::floor(scaled),
        0.,
        255.
    ));
}

std::string SineTable::print_table(const uint8_t scale) const
{
    std::stringstream ss {};

    ss << std::endl;

    for (size_t i = 0; i < length; i++)
    {
        const auto prev_value = get_value(i == 0? length - 1 : i - 1, scale);
        const auto value = get_value(i, scale);
        const auto next_value = get_value(i == length - 1? 0 : i + 1, scale);

        const auto start_at = std::min(std::min(prev_value, value), next_value);

        ss << std::format("[0x{:02X}] | ", value);

        for (uint8_t j = 0; ; j++)
        {
            if (j == scale / 2)
            {
                ss << '|';
            }
            else if (j < start_at)
            {
                ss << '.';
            }
            else if (j <= value)
            {
                ss << '*';
            }
            else
            {
                ss << ' ';
            }

            if (j == scale)
                break;
        }

        ss << std::endl;
    }

    return ss.str();
}