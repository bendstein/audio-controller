//
// Created by bendstein on 12/1/2025.
//

#ifndef AUDIO_CONTROLLER_COMMON_H
#define AUDIO_CONTROLLER_COMMON_H

#include "driver/gpio.h"
#include <esp_log.h>
#include <format>
#include <string>
#include <freertos/FreeRTOS.h>

#define PIN_LED_BUILTIN gpio_num_t::GPIO_NUM_13 //Built-in LED
#define portTICK_PERIOD_US ((TickType_t)1000 / portTICK_PERIOD_MS)

#define LOW 0
#define HIGH 1
#define DAC_MAX 255
#define FREQ_MIN_HZ 1

//Change parameter into HIGH or LOW for digital write
#define DIGITAL(boolean_value) boolean_value ? HIGH : LOW

//Change parameter name directly to string
#define NAMEOF(any) #any

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

/**
 * Write message to the log.
 * @param tag Used to specify which component the message is from, and
 *            also controls whether the message is displayed. If null,
 *            the tag is not included in the message.
 * @param message The message to log.
 * @param level The level to log at.
 * @remark Replacement for ESP_LOG* macros, as the only allow logging string literals
 */
inline void log_message(const std::string& tag, const std::string& message, const esp_log_level_t level)
{
    if (!_ESP_LOG_ENABLED(level))
        return;

    std::string log_level_string;

    switch (level)
    {
        case ESP_LOG_VERBOSE:
            log_level_string = "V";
            break;
        case ESP_LOG_DEBUG:
            log_level_string = "D";
            break;
        case ESP_LOG_WARN:
            log_level_string = "W";
            break;
        case ESP_LOG_ERROR:
            log_level_string = "E";
            break;
        case ESP_LOG_INFO:
        default:
            log_level_string = "I";
            break;
    }

    //Is effectively the same as the macro, but puts log level string,
    //and message in format args
    esp_log({
            .data = static_cast<uint32_t>(level)
        }, tag.c_str(), "%s (%lu) %s: %s\n",
        log_level_string.c_str(),
        esp_log_timestamp(),
        tag.c_str(),
        message.c_str()
    );
}

inline void logv(const std::string& tag, const std::string& message)
{
    log_message(tag, message, ESP_LOG_VERBOSE);
}
inline void logi(const std::string& tag, const std::string& message)
{
    log_message(tag, message, ESP_LOG_INFO);
}

inline void logd(const std::string& tag, const std::string& message)
{
    log_message(tag, message, ESP_LOG_DEBUG);
}

inline void logw(const std::string& tag, const std::string& message)
{
    log_message(tag, message, ESP_LOG_WARN);
}

inline void loge(const std::string& tag, const std::string& message)
{
    log_message(tag, message, ESP_LOG_ERROR);
}

inline void log(const std::string& tag, const std::string& message)
{
    logi(tag, message);
}
#endif //AUDIO_CONTROLLER_COMMON_H