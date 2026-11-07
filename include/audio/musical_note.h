//
// Created by bendstein on 12/2/2025.
//

#ifndef AUDIO_CONTROLLER_MUSICAL_NOTE_H
#define AUDIO_CONTROLLER_MUSICAL_NOTE_H
#include <cstdint>

/**
 * Enum of all notes in the chromatic scale
 */
enum struct musical_note : uint8_t
{
    C = 0, FIRST = C,
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
    B = 11, LAST = B,
    MAX
};

inline musical_note next_note(const musical_note note, bool* octave_change = nullptr)
{
    constexpr auto first_note = static_cast<uint8_t>(musical_note::FIRST);
    constexpr auto max_note = static_cast<uint8_t>(musical_note::MAX);
    const auto next_note_value = static_cast<uint8_t>(note) + 1;
    const auto adj = next_note_value % max_note;

    if (octave_change != nullptr)
        *octave_change = adj == first_note;

    return static_cast<musical_note>(adj);
}

inline musical_note prev_note(const musical_note note, bool* octave_change = nullptr)
{
    constexpr auto last_note = static_cast<uint8_t>(musical_note::LAST);
    constexpr auto max_note = static_cast<uint8_t>(musical_note::MAX);
    const auto prev_note_value = static_cast<uint8_t>(note) - 1;
    const auto adj = (prev_note_value + max_note) % max_note;

    if (octave_change != nullptr)
        *octave_change = adj == last_note;

    return static_cast<musical_note>(adj);
}

#endif //AUDIO_CONTROLLER_MUSICAL_NOTE_H