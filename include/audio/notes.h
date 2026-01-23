//
// Created by bendstein on 12/2/2025.
//

#ifndef AUDIO_CONTROLLER_NOTES_H
#define AUDIO_CONTROLLER_NOTES_H

#include <cmath>

#include "app_common.h"
#include "musical_note_data.h"

constexpr float FREQUENCY_C0 = 16.35f; //Frequency, in hz, for C in octave 0
constexpr float FREQUENCY_HZ_EPS = 0.1; //Consider fr within this amount to be equivalent

/**
 * Enum of all notes in the chromatic scale
 */
enum struct musical_note : uint8_t
{
    C = 0,
    C_Sharp = 1, D_Flat = C_Sharp,
    D = 2,
    D_Sharp = 3, E_Flat = D_Sharp,
    E = 4,
    F = 5,
    F_Sharp = 6, G_Flat = F_Sharp,
    G = 7,
    G_Sharp = 8, A_Flat = G_Sharp,
    A = 9,
    A_Sharp = 10, B_Flat = A_Sharp,
    B = 11,
    MAX
};


[[nodiscard]]
constexpr float musical_note_freq_hz(const musical_note note, const uint8_t octave)
{
    return FREQUENCY_C0 * std::powf(2, static_cast<float>(static_cast<uint8_t>(note) + (octave * static_cast<uint8_t>(musical_note::MAX))) / (static_cast<uint8_t>(musical_note::MAX) * 1.f));
}

[[nodiscard]]
consteval const SineTable* get_sine_table_from_note_consteval(const musical_note note, const uint8_t octave)
{
    assert(static_cast<size_t>(note) < sine_tables::ALL_SINE_TABLES_LENGTH_0
        && octave >= sine_tables::MIN_OCTAVE
        && octave <= sine_tables::MAX_OCTAVE);
    return &sine_tables::ALL_SINE_TABLES[static_cast<size_t>(note)][octave - sine_tables::MIN_OCTAVE];
}

[[nodiscard]]
inline const SineTable* get_sine_table_from_note(const musical_note note, const uint8_t octave)
{
    assert(static_cast<size_t>(note) < sine_tables::ALL_SINE_TABLES_LENGTH_0
        && octave >= sine_tables::MIN_OCTAVE
        && octave <= sine_tables::MAX_OCTAVE);
    return &sine_tables::ALL_SINE_TABLES[static_cast<size_t>(note)][octave - sine_tables::MIN_OCTAVE];
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
    [[nodiscard]] bool is_equivalent(const musical_note_tone& other) const { return check_frequency_equivalency(frequency_hz(), other.frequency_hz()); }
    [[nodiscard]] bool is_equivalent(const float other_hz) const { return check_frequency_equivalency(frequency_hz(), other_hz); }

    [[nodiscard]] static consteval musical_note_tone create_invalid() { return musical_note_tone(musical_note::C, 0xFF); }

    [[nodiscard]] static consteval const SineTable* get_sine_table_consteval(const musical_note_tone& tone) { return get_sine_table_from_note_consteval(tone.note, tone.octave); }
    [[nodiscard]] static const SineTable* get_sine_table(const musical_note_tone& tone) { return get_sine_table_from_note(tone.note, tone.octave); }
};

#endif //AUDIO_CONTROLLER_NOTES_H