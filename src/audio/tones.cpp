//
// Created by bendstein on 10/15/2026.
//
#include <algorithm>

#include "audio/tones.h"

size_t musical_note_tone::sine_table_index() const
{
    if (is_invalid() || octave < musical_note_data::MIN_OCTAVE || octave > musical_note_data::MAX_OCTAVE)
        return std::numeric_limits<size_t>::max();

    const auto note_number = static_cast<uint8_t>(note);

    if (note_number >= musical_note_data::ALL_SINE_TABLES_LENGTH_0)
        return std::numeric_limits<size_t>::max();

    return note_number * musical_note_data::ALL_SINE_TABLES_LENGTH_1 + (octave - musical_note_data::MIN_OCTAVE);
}
uint32_t musical_note_tone::get_period(const uint32_t sample_rate) const
{
    if (is_invalid())
        return 0;

    return get_period(frequency_hz(), sample_rate);
}

uint32_t musical_note_tone::get_period(const double frequency_hz, const uint32_t sample_rate)
{
    if (frequency_hz == 0)
        return std::numeric_limits<uint32_t>::max();

    return static_cast<uint32_t>(std::clamp(
        sample_rate / frequency_hz,
        0.,
        static_cast<double>(std::numeric_limits<uint32_t>::max())));
}
