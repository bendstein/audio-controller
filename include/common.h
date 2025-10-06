//
// Created by bendstein on 10/5/2025.
//

#ifndef AUDIO_CONTROLLER_UTILS_H
#define AUDIO_CONTROLLER_UTILS_H

#define PIN_INVALID 255      //Value representing an invalid pin
#define INPUT_MAX_VALUE 4096 //Max value that can be received by analogRead()
#define DAC_MAX 255          //Max output value for a DAC channel

/**
 * Add logic to a function for debouncing.
 * @param threshold_ms Threshold, in milliseconds, that must elapse between calls for the remainder of the call to execute
 * @remark Assumes wrapper function returns void
 */
#define DEBOUNCE_MS(threshold_ms)           \
    static long unsigned last_ts = 0;       \
    const auto current_ts = millis();       \
    const auto dt = current_ts - last_ts;   \
    last_ts = current_ts;                   \
    if(dt <= threshold_ms)                  \
        return;                             \

#endif //AUDIO_CONTROLLER_UTILS_H