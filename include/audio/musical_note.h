//
// Created by bendstein on 12/2/2025.
//

#ifndef AUDIO_CONTROLLER_MUSICAL_NOTE_H
#define AUDIO_CONTROLLER_MUSICAL_NOTE_H

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

#endif //AUDIO_CONTROLLER_MUSICAL_NOTE_H