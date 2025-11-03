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
public:
    static constexpr long unsigned INACTIVE_THRESHOLD_MS = 300;

private:
    /**
     * The first pin of the rotary encoder
     */
    const uint8_t pin_a;

    /**
     * The second pin of the rotary encoder
     */
    const uint8_t pin_b;

    uint8_t state: 7;

    uint8_t direction: 1;

    /**
     * Function pointer to execute on interrupt
     */
    RotaryEncoderCallback* callback;

    void Update(const uint8_t new_state)
    {
        const auto prev_state = state;

        //A valid transition must differ by exactly 1 bit
        const auto state_diff = new_state ^ prev_state;

        if (state_diff == 0 || (state_diff & (state_diff - 1)) != 0) //Invalid
        {
            state = 0;
            return;
        }

        //If in start state, determine direction, update, and return
        if (prev_state == 0)
        {
            /*
             * new_state must either be 0b01 or 0b10 due
             * to previous conditions, so can determine
             * direction by checking lsb
             */
            direction = new_state & 0b01;
            state = new_state;

            return;
        }

        //Verify that transition is valid

        //For counterclockwise, flip bits on new state
        //so that state machine is unchanged
        const auto new_state_normalized = (direction == 1)
            ? new_state
            : ((new_state & 0b01) << 1) + ((new_state & 0b10) >> 1);

        switch (prev_state)
        {
            case 0b01: // 01 => 11
            {
                if (new_state_normalized == 0b11)
                {}
                else //Invalid
                {
                    state = 0;
                    return;
                }
            }
            break;
            case 0b10: // 10 => 00
            {
                if (new_state_normalized == 0b00)
                {}
                else //Invalid
                {
                    state = 0;
                    return;
                }
            }
            break;
            case 0b11: // 11 => 10
            {
                if (new_state_normalized == 0b10)
                {}
                else //Invalid
                {
                    state = 0;
                    return;
                }
            }
            break;
            default: //?
                state = 0;
                return;
        }

        //At this point, state must be valid. Update.
        state = new_state;

        //Execute callback if at end state
        if (state == 0)
        {
            callback(direction == 1);
        }
    }

    uint8_t GetState() const
    {
        return (digitalRead(pin_a) == 0? 0 : 1)
            | ((digitalRead(pin_b) == 0? 0 : 1) << 1);
    }

    /**
     * Handle an interrupt on either pin of the encoder
     * @param rotary_encoder_pointer Address which should point to a rotary encoder. void* so that attachInterruptArg can be called.
     */
    static void HandleInterrupt(void* rotary_encoder_pointer)
    {
        if (rotary_encoder_pointer == nullptr) //No rotary encoder is present
            return;

        const auto rotary_encoder = static_cast<RotaryEncoder*>(rotary_encoder_pointer);

        rotary_encoder->Update(rotary_encoder->GetState());
    }

public:
    /**
     *
     * @param pin_a The first output pin of the rotary encoder
     * @param pin_b The second output pin of the rotary encoder
     * @param callback Function pointer to execute when rotary encoder is turned
     */
    explicit RotaryEncoder(const uint8_t pin_a = PIN_INVALID, const uint8_t pin_b = PIN_INVALID, RotaryEncoderCallback* callback = nullptr)
        : pin_a(pin_a), pin_b(pin_b), callback(callback)
    {
        state = 0;
        direction = 0;
    }

    /**
     * Attach interrupts to pins, and set pin modes.
     * @return Whether the interrupts were successfully attached
     */
    bool Attach()
    {
        if (this->pin_a == PIN_INVALID || this->pin_b == PIN_INVALID || this->callback == nullptr)
            return false;

        pinMode(this->pin_a, INPUT | PULLDOWN);
        pinMode(this->pin_b, INPUT | PULLDOWN);

        attachInterruptArg(digitalPinToInterrupt(this->pin_a), HandleInterrupt, this, CHANGE);
        attachInterruptArg(digitalPinToInterrupt(this->pin_b), HandleInterrupt, this, CHANGE);
        return true;
    }
};

#endif //AUDIO_CONTROLLER_ROTARY_ENCODER_H