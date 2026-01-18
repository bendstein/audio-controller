//
// Created by bendstein on 12/2/2025.
//

#ifndef AUDIO_CONTROLLER_NOTES_H
#define AUDIO_CONTROLLER_NOTES_H

#include <cmath>

#include "app_common.h"

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
constexpr float hz_to_megahz(const float hz) { return hz / US_PER_SECOND; }

[[nodiscard]]
constexpr bool check_frequency_equivalency(const float a, const float b) { return std::abs(a - b) <= FREQUENCY_HZ_EPS; }

#endif //AUDIO_CONTROLLER_NOTES_H