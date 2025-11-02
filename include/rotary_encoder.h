//
// Created by bendstein on 9/28/2025.
//

#ifndef AUDIO_CONTROLLER_ROTARY_ENCODER_H
#define AUDIO_CONTROLLER_ROTARY_ENCODER_H

#include <Arduino.h>
#include "common.h"

typedef void RotaryEncoderCallback(bool clockwise);

enum RotaryEncoderState : uint8_t
{
    /**
     * Start
     */
    Start,
    /**
     * Start --> Pin A High ; Expecting Pin B High for CW
     */
    AWaitingB,
    /**
     * Start --> Pin B High ; Expecting Pin A High for CCW
     */
    BWaitingA
};

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

    /**
     * Current state of the rotary encoder
     */
    RotaryEncoderState state = RotaryEncoderState::Start;

    /**
     * Function pointer to execute on interrupt
     */
    RotaryEncoderCallback* callback;

    /**
     * Last time the encoder state changed
     */
    long unsigned last_ts = 0;

    /**
     * @param is_input_a Which pin triggered the update
     */
    void ChangeState(const bool is_input_a)
    {
        //Reset state machine if inactive for a while
        const auto current_ts = millis();

        if (current_ts - last_ts > INACTIVE_THRESHOLD_MS)
        {
            state = RotaryEncoderState::Start;
        }

        last_ts = current_ts;

        switch (state)
        {
            case RotaryEncoderState::Start:
            {
                state = is_input_a
                    ? RotaryEncoderState::AWaitingB
                    : RotaryEncoderState::BWaitingA;
            }
            break;
            case RotaryEncoderState::AWaitingB:
            {
                if (is_input_a) //Unchanged
                { }
                else //Triggered A, then B -> Clockwise
                {
                    state = RotaryEncoderState::Start;
                    callback(true);
                }
            }
            break;
            case RotaryEncoderState::BWaitingA:
            {
                if (is_input_a) //Triggered B, then A -> Counter-clockwise
                {
                    state = RotaryEncoderState::Start;
                    callback(false);
                }
                else //Unchanged
                { }
            }
            break;
            default: //Invalid; move to start
            {
                state = RotaryEncoderState::Start;
            }
            break;
        }
    }

    /**
     * Handle a rising-edge interrupt on a rotary encoder's pin a
     * @param rotary_encoder_pointer Address which should point to a rotary encoder. void* so that attachInterruptArg can be called.
     */
    static void HandleInterruptA(void* rotary_encoder_pointer)
    {
        DEBOUNCE_MS(30)

        if (rotary_encoder_pointer == nullptr) //No rotary encoder is present
            return;

        const auto rotary_encoder = static_cast<RotaryEncoder*>(rotary_encoder_pointer);

        rotary_encoder->ChangeState(true);
    }

    /**
     * Handle a rising-edge interrupt on a rotary encoder's pin b
     * @param rotary_encoder_pointer Address which should point to a rotary encoder. void* so that attachInterruptArg can be called.
     */
    static void HandleInterruptB(void* rotary_encoder_pointer)
    {
        DEBOUNCE_MS(30)

        if (rotary_encoder_pointer == nullptr) //No rotary encoder is present
            return;

        const auto rotary_encoder = static_cast<RotaryEncoder*>(rotary_encoder_pointer);

        rotary_encoder->ChangeState(false);
    }

public:
    /**
     *
     * @param pin_a The first output pin of the rotary encoder
     * @param pin_b The second output pin of the rotary encoder
     * @param callback Function pointer to execute when rotary encoder is turned
     */
    explicit RotaryEncoder(const uint8_t pin_a = PIN_INVALID, const uint8_t pin_b = PIN_INVALID, RotaryEncoderCallback* callback = nullptr)
        : pin_a(pin_a), pin_b(pin_b), callback(callback) { }

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

        attachInterruptArg(digitalPinToInterrupt(this->pin_a), HandleInterruptA, this, RISING);
        attachInterruptArg(digitalPinToInterrupt(this->pin_b), HandleInterruptB, this, RISING);
        return true;
    }
};

#endif //AUDIO_CONTROLLER_ROTARY_ENCODER_H