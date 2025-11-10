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


/**
 * Count the number of set bits in an integer value,
 * using Brian Kernighan's Algorithm
 * @param unsigned_integer_type The type of integer to create the function for
 */
#define COUNT_SET_BITS_TYPED(unsigned_integer_type)     \
    uint8_t CountSetBits(unsigned_integer_type value)   \
    {                                                   \
        uint8_t count = 0;                              \
                                                        \
        while (value != 0)                              \
        {                                               \
            value = value & (value - 1);                \
            count++;                                    \
        }                                               \
                                                        \
        return count;                                   \
    }                                                   \
                                                        \

/**
 * Get the last MSB for which a bit is set
 * @param unsigned_integer_type The type of integer to create the function for
 */
#define GET_MAX_SET_BIT_TYPED(unsigned_integer_type)    \
    uint8_t GetMaxSetBit(unsigned_integer_type value)   \
    {                                                   \
        uint8_t count = 0;                              \
                                                        \
        while (value != 0)                              \
        {                                               \
            value = value >> 1;                         \
            count++;                                    \
        }                                               \
                                                        \
        return count;                                   \
    }                                                   \
                                                        \

/**
 * Clamp the given value to the given bounds
 */
#define CLAMP_TYPED(number_type)                                                                \
    number_type Clamp(const number_type value, const number_type min, const number_type max)    \
    {                                                                                           \
        return std::max(min, std::min(max, value));                                             \
    }                                                                                           \
                                                                                                \

COUNT_SET_BITS_TYPED(uint32_t)
COUNT_SET_BITS_TYPED(uint16_t)
COUNT_SET_BITS_TYPED(uint8_t)

GET_MAX_SET_BIT_TYPED(uint32_t)
GET_MAX_SET_BIT_TYPED(uint16_t)
GET_MAX_SET_BIT_TYPED(uint8_t)

CLAMP_TYPED(uint32_t)
CLAMP_TYPED(uint16_t)
CLAMP_TYPED(uint8_t)
CLAMP_TYPED(double)

#endif //AUDIO_CONTROLLER_UTILS_H