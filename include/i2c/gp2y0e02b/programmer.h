//
// Created by bendstein on 11/15/2026.
//

#ifndef AUDIO_CONTROLLER_PROGRAMMER_H
#define AUDIO_CONTROLLER_PROGRAMMER_H
#include <cstdint>

namespace gp2y0e02b
{
    enum struct programmer_event_type : uint8_t
    {
        NONE = 0,
        BUTTON = 1,
        ROTARY = 2
    };

    struct programmer_event
    {
        programmer_event_type type;
        int8_t data;
    };
}

#endif //AUDIO_CONTROLLER_PROGRAMMER_H