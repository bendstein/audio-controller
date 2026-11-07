//
// Created by bendstein on 10/15/2026.
//
#include <algorithm>

#include "audio/tones.h"

uint32_t musical_note_tone::get_period(const uint32_t sample_rate) const
{
    if (is_invalid())
        return 0;

    return get_period(frequency_hz(), sample_rate);
}

double musical_note_tone::get_periodd(const uint32_t sample_rate) const
{
    if (is_invalid())
        return 0;
    return get_periodd(frequency_hz(), sample_rate);
}

uint32_t musical_note_tone::get_period(const double frequency_hz, const uint32_t sample_rate)
{
    return static_cast<uint32_t>(std::clamp(
        std::round(get_periodd(frequency_hz, sample_rate)),
        0.,
        static_cast<double>(std::numeric_limits<uint32_t>::max())));
}

double musical_note_tone::get_periodd(const double frequency_hz, const uint32_t sample_rate)
{
    if (frequency_hz == 0)
        return std::numeric_limits<double>::max();

    return std::abs(sample_rate / frequency_hz);
}