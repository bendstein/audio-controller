//
// Created by bendstein on 1/28/2026.
//

#ifndef AUDIO_CONTROLLER_TONES_H
#define AUDIO_CONTROLLER_TONES_H

#include <cmath>

#include "app_common.h"
#include "musical_note_data.h"
#include "musical_note.h"

constexpr float FREQUENCY_C0 = 16.35f; //Frequency, in hz, for C in octave 0
constexpr float FREQUENCY_HZ_EPS = 0.1; //Consider fr within this amount to be equivalent

[[nodiscard]]
constexpr float musical_note_freq_hz(const musical_note note, const uint8_t octave)
{
    return FREQUENCY_C0 * std::powf(2, static_cast<float>(static_cast<uint8_t>(note) + (octave * static_cast<uint8_t>(musical_note::MAX))) / (static_cast<uint8_t>(musical_note::MAX) * 1.f));
}

[[nodiscard]]
inline const SineTable* get_sine_table_from_note(const musical_note note, const uint8_t octave)
{
    assert(static_cast<size_t>(note) < musical_note_data::ALL_SINE_TABLES_LENGTH_0
        && octave >= musical_note_data::MIN_OCTAVE
        && octave <= musical_note_data::MAX_OCTAVE);
    return &musical_note_data::ALL_SINE_TABLES[static_cast<size_t>(note)][octave - musical_note_data::MIN_OCTAVE];
}

[[nodiscard]]
constexpr float hz_to_megahz(const float hz) { return hz / US_PER_SECOND; }

[[nodiscard]]
constexpr bool check_frequency_equivalency(const float a, const float b) { return std::abs(a - b) <= FREQUENCY_HZ_EPS; }

struct musical_note_tone
{
    musical_note note;
    uint8_t octave;

    [[nodiscard]] bool is_invalid() const { return octave == 0xFF; }
    [[nodiscard]] const SineTable* sine_table() const { return get_sine_table(*this); }
    [[nodiscard]] float frequency_hz() const { return is_invalid()? 0 : musical_note_freq_hz(note, octave); }
    [[nodiscard]] const char* name(const bool prefer_flat = false) const { return get_note_name(note, octave, prefer_flat); }
    [[nodiscard]] const char* name_no_octave(const bool prefer_flat = false) const { return get_note_name(note, prefer_flat); }

    bool operator==(const musical_note_tone& other) const { return check_frequency_equivalency(frequency_hz(), other.frequency_hz()); }
    bool operator==(const float other_hz) const { return check_frequency_equivalency(frequency_hz(), other_hz); }

    [[nodiscard]] static consteval musical_note_tone create_invalid() { return musical_note_tone(musical_note::C, 0xFF); }
    [[nodiscard]] static const SineTable* get_sine_table(const musical_note_tone& tone) { return get_sine_table_from_note(tone.note, tone.octave); }
};

#endif //AUDIO_CONTROLLER_TONES_H