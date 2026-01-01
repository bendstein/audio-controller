//
// Created by bendstein on 12/2/2025.
//

#ifndef AUDIO_CONTROLLER_NOTES_H
#define AUDIO_CONTROLLER_NOTES_H

#include <cmath>

#define FREQUENCY_C0 16.35 //Frequency, in hz, for C in octave 0

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
constexpr double musical_note_freq_hz(const musical_note note, const uint8_t octave)
{
    return FREQUENCY_C0 * std::pow(2, (static_cast<uint8_t>(note) + (octave * static_cast<uint8_t>(musical_note::MAX))) / (static_cast<uint8_t>(musical_note::MAX) * 1.));
}

enum struct tone_type : uint8_t {
    note,
    frequency
};

struct tone {
    tone_type type;
    union tone_value {
        tone_value() : frequency_hz(0) {}
        explicit tone_value(const double frequency_hz)
            : frequency_hz(frequency_hz) {}
        tone_value(const musical_note note_value, const uint8_t octave)
            : note({.name = note_value, .octave = octave}) {}

        double frequency_hz;
        struct note {
            musical_note name;
            uint8_t octave;
        } note;
    } value;

    tone() : type(tone_type::frequency) {}

    explicit tone(const double frequency_hz) : type(tone_type::frequency), value(frequency_hz) { }

    tone(const musical_note note, const uint8_t octave) : type(tone_type::note), value(note, octave) { }

    [[nodiscard]]
    double frequency_hz() const {
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
    double frequency_megahz() const {
        constexpr auto US_PER_SECOND = 1000000;
        return frequency_hz() / US_PER_SECOND;
    }
};

#endif //AUDIO_CONTROLLER_NOTES_H