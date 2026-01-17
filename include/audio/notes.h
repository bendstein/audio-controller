//
// Created by bendstein on 12/2/2025.
//

#ifndef AUDIO_CONTROLLER_NOTES_H
#define AUDIO_CONTROLLER_NOTES_H

#include <cmath>

#include "app_common.h"

constexpr float FREQUENCY_C0 = 16.35f; //Frequency, in hz, for C in octave 0
constexpr float TONE_HZ_EPS = 0.1; //Consider tones within this amount to be equivalent

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

/**
 * Get the frequency of the given musical note in the given octave
 * @param note The note to get the frequency of
 * @param octave The octave to get the frequency of the note in
 * @return The frequency of the given note in the given octave
 * @remark Frequencies calculated as equal temperament, starting with C0 at FREQUENCY_C0
 * @todo Add another argument with an enum representing the tuning type,
 *  e.g. Tuning::Equal, Tuning::Just?
 */
[[nodiscard]]
constexpr float musical_note_freq_hz(const musical_note note, const uint8_t octave)
{
    return FREQUENCY_C0 * std::powf(2, static_cast<float>(static_cast<uint8_t>(note) + (octave * static_cast<uint8_t>(musical_note::MAX))) / (static_cast<uint8_t>(musical_note::MAX) * 1.f));
}

enum struct tone_type : uint8_t {
    note,
    frequency
};

struct octave_note
{
    musical_note name;
    uint8_t octave;
};

union tone_value
{
    float frequency_hz;
    octave_note note;

    tone_value() : frequency_hz(0) {}
    explicit tone_value(const float frequency_hz) : frequency_hz(frequency_hz) {}
    explicit tone_value(const octave_note note) : note(note) {}
    tone_value(const musical_note note_value, const uint8_t octave) : note(note_value, octave) {}
};

struct tone {
    tone_type type;
    tone_value value;

    tone() : type(tone_type::frequency) {}
    tone(const tone& other) : type(other.type), value(tone_value(other.value)) {}
    tone& operator=(const tone& other)
    {
        type = other.type;
        value = tone_value(other.value);
        return *this;
    }

    tone(tone&& other) noexcept : type(other.type), value(tone_value(other.value)) {}

    tone& operator=(tone&& other) noexcept
    {
        type = other.type;
        value = tone_value(other.value);
        return *this;
    }

    explicit tone(const float frequency_hz) : type(tone_type::frequency), value(frequency_hz) { }
    explicit tone(const octave_note note) : type(tone_type::note), value(note) { }
    tone(const musical_note note, const uint8_t octave) : type(tone_type::note), value(note, octave) { }

    [[nodiscard]]
    float frequency_hz() const {
        switch(type) {
            case tone_type::note:
                return musical_note_freq_hz(value.note.name, value.note.octave);
            case tone_type::frequency:
                return value.frequency_hz;
            default:
                return 0;
        }
    }

    [[nodiscard]]
    float frequency_megahz() const {
        return frequency_hz() / US_PER_SECOND;
    }

    [[nodiscard]]
    bool is_equivalent_to(const tone& other) const
    {
        return std::abs(frequency_hz() - other.frequency_hz()) <= TONE_HZ_EPS;
    }

    [[nodiscard]]
    bool is_zero() const { return is_equivalent_to(*dft()); }

    static const tone* dft() {
        static auto default_tone = tone();
        return &default_tone;
    }
};

#endif //AUDIO_CONTROLLER_NOTES_H