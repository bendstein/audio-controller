//
// Created by bendstein on 9/28/2025.
//

#ifndef AUDIO_CONTROLLER_ROTARY_ENCODER_H
#define AUDIO_CONTROLLER_ROTARY_ENCODER_H

#include <Arduino.h>
#include "common.h"

typedef void RotaryEncoderCallback(bool clockwise);

class RotaryEncoder
{
    /**
     * The first output pin of the rotary encoder
     */
    uint8_t pin_a;

    /**
     * The second output pin of the rotary encoder
     */
    uint8_t pin_b;

    /**
     * Function pointer to execute on interrupt
     */
    RotaryEncoderCallback* callback;

    /**
     * Handle a falling-edge interrupt on a rotary encoder's pin a
     * @param rotary_encoder_pointer Address which should point to a rotary encoder. void* so that attachInterruptArg can be called.
     */
    static void HandleInterrupt(void* rotary_encoder_pointer)
    {
        if (rotary_encoder_pointer == nullptr) //No rotary encoder is present
            return;

        const auto rotary_encoder = static_cast<RotaryEncoder*>(rotary_encoder_pointer);

        //No pin b and/or callback given
        if (rotary_encoder->pin_b == PIN_INVALID || rotary_encoder->callback == nullptr)
            return;

        //If pin b matches pin a, then clockwise, otherwise, counterclockwise
        const auto clockwise = digitalRead(rotary_encoder->pin_b) == digitalRead(rotary_encoder->pin_a);

        //Execute callback
        rotary_encoder->callback(clockwise);
    }

public:
    /**
     *
     * @param pin_a The first output pin of the rotary encoder
     * @param pin_b The second output pin of the rotary encoder
     * @param callback Function pointer to execute when rotary encoder is turned
     */
    explicit RotaryEncoder(const uint8_t pin_a = PIN_INVALID, const uint8_t pin_b = PIN_INVALID, RotaryEncoderCallback* callback = nullptr)
    {
        this->pin_a = pin_a;
        this->pin_b = pin_b;
        this->callback = callback;
    }

    /**
     * Attach an interrupt to pin a, executing the callback on its falling edge
     * @return Whether the interrupt was successfully attached
     */
    bool Attach()
    {
        if (this->pin_a == PIN_INVALID || this->pin_b == PIN_INVALID || this->callback == nullptr)
            return false;

        attachInterruptArg(digitalPinToInterrupt(this->pin_a), HandleInterrupt, this, FALLING);
        return true;
    }
};

#endif //AUDIO_CONTROLLER_ROTARY_ENCODER_H