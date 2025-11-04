/*
 * Copyright 2011 Ben Buxton. Licenced under the GNU GPL Version 3. Contact: bb@cactii.net
 * See: https://github.com/buxtronix/arduino/tree/master/libraries/Rotary
 *
 * Changes (Benjamin Goldstein):
 *  - Combined Rotary.cpp and Rotary.h into header-only file rotary.h.
 *  - Omit half-step logic.
 *  - Prefix defines with R_ (unless already included)
 *  - Rotary class move private members to top
 *  - Change uses of unsigned char to byte, make some variables const
 *  - Minor name/case changes
 *  - Make process private, add static version for interrupt
 *  - Make state table static
 *  - Instead of returning state from process, have it execute a callback (type RotaryCallback) on terminal state
 *  - Add callback, register interrupts in ctor. Unregister in destructor.
 */
#ifndef AUDIO_CONTROLLER_ROTARY_H
#define AUDIO_CONTROLLER_ROTARY_H

#include "Arduino.h"

typedef void RotaryCallback(bool clockwise);

// Values returned by 'process'
// No complete step yet.
#define R_DIR_NONE 0x0
// Clockwise step.
#define R_DIR_CW 0x10
// Anti-clockwise step.
#define R_DIR_CCW 0x20

// Defines for state table (emits a code at 00 only)
#define R_START 0x0
#define R_CW_FINAL 0x1
#define R_CW_BEGIN 0x2
#define R_CW_NEXT 0x3
#define R_CCW_BEGIN 0x4
#define R_CCW_FINAL 0x5
#define R_CCW_NEXT 0x6

static const unsigned char ttable[7][4] = {
    // R_START
    {R_START,    R_CW_BEGIN,  R_CCW_BEGIN, R_START},
    // R_CW_FINAL
    {R_CW_NEXT,  R_START,     R_CW_FINAL,  R_START | R_DIR_CW},
    // R_CW_BEGIN
    {R_CW_NEXT,  R_CW_BEGIN,  R_START,     R_START},
    // R_CW_NEXT
    {R_CW_NEXT,  R_CW_BEGIN,  R_CW_FINAL,  R_START},
    // R_CCW_BEGIN
    {R_CCW_NEXT, R_START,     R_CCW_BEGIN, R_START},
    // R_CCW_FINAL
    {R_CCW_NEXT, R_CCW_FINAL, R_START,     R_START | R_DIR_CCW},
    // R_CCW_NEXT
    {R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START},
};

class Rotary
{
    byte state = R_START;
    const byte pin1;
    const byte pin2;
    RotaryCallback* callback;

    void process()
    {
        // Grab state of input pins.
        const byte pin_state = (digitalRead(pin2) << 1) | digitalRead(pin1);
        // Determine new state from the pins and state table.
        state = ttable[state & 0xf][pin_state];

        const auto result = state & 0x30;

        if (result == R_DIR_CW) //Terminal state, rotated clockwise
        {
            callback(true);
        }
        else if (result == R_DIR_CCW) //Terminal state, rotated counter-clockwise
        {
            callback(false);
        }
    }

    static void process__isr(void* rotary_ptr)
    {
        if (rotary_ptr == nullptr) //No rotary encoder is present
            return;

        const auto rotary = static_cast<Rotary*>(rotary_ptr);
        rotary->process();
    }
public:
    Rotary(const byte pin1, const byte pin2, RotaryCallback* callback) : pin1(pin1), pin2(pin2), callback(callback)
    {
        // Set pins to input.
        pinMode(pin1, INPUT);
        pinMode(pin2, INPUT);

        //Attach interrupts
        attachInterruptArg(digitalPinToInterrupt(pin1), process__isr, this, CHANGE);
        attachInterruptArg(digitalPinToInterrupt(pin2), process__isr, this, CHANGE);
    }

    ~Rotary()
    {
        //Detach interrupts
        detachInterrupt(digitalPinToInterrupt(pin1));
        detachInterrupt(digitalPinToInterrupt(pin2));
    }
};

#endif //AUDIO_CONTROLLER_ROTARY_H