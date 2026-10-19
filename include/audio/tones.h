//
// Created by bendstein on 1/28/2026.
//

#ifndef AUDIO_CONTROLLER_TONES_H
#define AUDIO_CONTROLLER_TONES_H
#include <cmath>

#include "app_common.h"
#include "musical_note_data.h"
#include "musical_note.h"

constexpr double FREQUENCY_C0 = 16.35; //Frequency, in hz, for C in octave 0
constexpr double FREQUENCY_HZ_EPS = 0.1; //Consider fr within this amount to be equivalent

[[nodiscard]] constexpr double musical_note_freq_hz(const musical_note note, const uint8_t octave)
{
    if (octave == 0) //Special case; if octave is 0, return 0
        return 0.;

    return FREQUENCY_C0 * std::pow(2, static_cast<float>(static_cast<uint8_t>(note) + (octave * static_cast<uint8_t>(musical_note::MAX))) / (static_cast<uint8_t>(musical_note::MAX) * 1.));
}

[[nodiscard]] inline const SineTable* get_sine_table_from_note(const musical_note note, const uint8_t octave)
{
    if (octave == 0) //Special case; if octave is 0, return empty table
        return &musical_note_data::SINE_TABLE_EMPTY;

    assert(static_cast<size_t>(note) < musical_note_data::ALL_SINE_TABLES_LENGTH_0
        && octave >= musical_note_data::MIN_OCTAVE
        && octave <= musical_note_data::MAX_OCTAVE);

    return &musical_note_data::ALL_SINE_TABLES[static_cast<size_t>(note)][octave - musical_note_data::MIN_OCTAVE];
}

[[nodiscard]] constexpr double hz_to_megahz(const double hz) { return hz / US_PER_SECOND; }

[[nodiscard]] constexpr bool check_frequency_equivalency(const double a, const double b) { return std::abs(a - b) <= FREQUENCY_HZ_EPS; }

struct musical_note_tone
{
    musical_note note;
    uint8_t octave;

    [[nodiscard]] bool is_zero() const { return octave == 0; }
    [[nodiscard]] bool is_invalid() const { return octave == 0xFF; }
    [[nodiscard]] bool is_zero_or_invalid() const { return octave == 0 || octave == 0xFF; }
    [[nodiscard]] const SineTable* sine_table() const { return get_sine_table(*this); }
    [[nodiscard]] double frequency_hz() const { return is_invalid()? 0 : musical_note_freq_hz(note, octave); }
    [[nodiscard]] const char* name(const bool prefer_flat = false) const { return get_note_name(note, octave, prefer_flat); }
    [[nodiscard]] const char* name_no_octave(const bool prefer_flat = false) const { return get_note_name(note, prefer_flat); }
    [[nodiscard]] size_t sine_table_index() const;
    [[nodiscard]] uint32_t get_period(uint32_t sample_rate) const;

    bool operator==(const musical_note_tone& other) const { return check_frequency_equivalency(frequency_hz(), other.frequency_hz()); }
    bool operator==(const double other_hz) const { return check_frequency_equivalency(frequency_hz(), other_hz); }

    [[nodiscard]] static consteval musical_note_tone create_zero() { return musical_note_tone(musical_note::C, 0); }
    [[nodiscard]] static consteval musical_note_tone create_invalid() { return musical_note_tone(musical_note::C, 0xFF); }
    [[nodiscard]] static const SineTable* get_sine_table(const musical_note_tone& tone) { return get_sine_table_from_note(tone.note, tone.octave); }
    [[nodiscard]] static uint32_t get_period(double frequency_hz, uint32_t sample_rate);
};

struct musical_note_tone_volume
{
    musical_note_tone tone;
    uint8_t volume;

    [[nodiscard]] bool is_zero() const { return volume == 0 || tone.is_zero(); }
    [[nodiscard]] bool is_invalid() const { return tone.is_invalid(); }
    [[nodiscard]] bool is_zero_or_invalid() const { return volume == 0 || tone.is_invalid(); }

    bool operator==(const musical_note_tone_volume& other) const { return tone == other.tone && volume == other.volume; }

    [[nodiscard]] static consteval musical_note_tone_volume create_zero() { return musical_note_tone_volume(musical_note_tone::create_zero(), 0); }
    [[nodiscard]] static consteval musical_note_tone_volume create_invalid() { return musical_note_tone_volume(musical_note_tone::create_invalid(), 0); }
};

#endif //AUDIO_CONTROLLER_TONES_H