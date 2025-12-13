//
// Created by bendstein on 12/1/2025.
//

#ifndef AUDIO_CONTROLLER_COMMON_H
#define AUDIO_CONTROLLER_COMMON_H

#include "driver/gpio.h"
#include <esp_log.h>
#include <format>
#include <string>

#define PIN_LED_BUILTIN gpio_num_t::GPIO_NUM_13 //Built-in LED

#define LOW 0
#define HIGH 1
#define DAC_MAX 255
#define FREQ_MIN_HZ 1

#define DIGITAL(boolean_value) boolean_value ? HIGH : LOW

/**
 * Count the number of set bits in an integer value,
 * using Brian Kernighan's Algorithm
 */
template<typename TNumber>
[[nodiscard]]
uint8_t count_set_bits(TNumber value)
{
    uint8_t count = 0;

    while (value != 0)
    {
        value &= value - 1;
        count++;
    }

    return count;
}

/**
 * Get the last MSB for which a bit is set
 */
template<typename TNumber>
[[nodiscard]]
uint8_t get_max_set_bit(TNumber value)
{
    uint8_t count = 0;

    while (value != 0)
    {
        value >>= 1;
        count++;
    }

    return count;
}

#endif //AUDIO_CONTROLLER_COMMON_H