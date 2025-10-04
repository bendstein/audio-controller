//
// Created by bendstein on 9/29/2025.
//

#ifndef AUDIO_CONTROLLER_NOTES_H
#define AUDIO_CONTROLLER_NOTES_H

#define FREQUENCY_C0 16.35

enum MusicalNote
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

constexpr double GetMusicalNoteFrequency(const MusicalNote note, const uint8_t octave)
{
    return FREQUENCY_C0 * std::pow(2, (note + (octave * MAX)) / (MAX * 1.));
}

#endif //AUDIO_CONTROLLER_NOTES_H