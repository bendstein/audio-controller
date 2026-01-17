//
// Created by bendstein on 12/1/2025.
//

#ifndef AUDIO_CONTROLLER_COMMON_H
#define AUDIO_CONTROLLER_COMMON_H

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

#include <esp_log.h>
#include <string>
#include <ctime>
#include "driver/gpio.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sys/_timeval.h>

// #define LOG_VERBOSE //If present, enable VERBOSE log macros, as well as additional debug info
#define LOG_VERBOSE_TAG "AC-VRBS" //Tag to use in the VERBOSE log macro

#ifdef LOG_VERBOSE
//Whether the application should perform additional debugging/logging logic
constexpr auto FLAG_VERBOSE = true;
#else
//Whether the application should perform additional debugging/logging logic
constexpr auto FLAG_VERBOSE = false;
#endif

constexpr auto US_PER_MS = 1000;
constexpr auto MS_PER_SECOND = 1000;
constexpr auto US_PER_SECOND = 1000000;
constexpr auto PIN_LED_BUILTIN = GPIO_NUM_13;
constexpr uint8_t LOW = 0;
constexpr uint8_t HIGH = 0;

constexpr size_t SENSORS_COUNT = 1;
constexpr size_t PIECEWISE_FREQUENCY_BREAKPOINT_COUNT = 3;
constexpr size_t I2C_DEVICE_CAPACITY = SENSORS_COUNT;

//Doesn't need to be a macro, but I'm doing it to match portTICK_PERIOD_MS being a macro
#define portTICK_PERIOD_US ((TickType_t)US_PER_MS / portTICK_PERIOD_MS)

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

inline long long total_microseconds(const timeval* time)
{
    return time->tv_usec
        + time->tv_sec * US_PER_SECOND;
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

#define LOGV(message) logv(__FILE_NAME__, message)
#define LOGI(message) logi(__FILE_NAME__, message)
#define LOGD(message) logd(__FILE_NAME__, message)
#define LOGW(message) logw(__FILE_NAME__, message)
#define LOGE(message) loge(__FILE_NAME__, message)
#define LOGEX(ex) loge(__FILE_NAME__, ex.what())

#define FLOGV(message, ...) LOGV(std::format(message __VA_OPT__(,) __VA_ARGS__))
#define FLOGI(message, ...) LOGI(std::format(message __VA_OPT__(,) __VA_ARGS__))
#define FLOGD(message, ...) LOGD(std::format(message __VA_OPT__(,) __VA_ARGS__))
#define FLOGW(message, ...) LOGW(std::format(message __VA_OPT__(,) __VA_ARGS__))
#define FLOGE(message, ...) LOGE(std::format(message __VA_OPT__(,) __VA_ARGS__))

//Only verbose log if enabled
#ifdef LOG_VERBOSE
#define VERBOSE(message) logv(LOG_VERBOSE_TAG, std::format("({}:{}) {}", __FILE_NAME__, __LINE__, message))
#define VERBOSE_LB(message) logv(LOG_VERBOSE_TAG, std::format("({}:{})\r\n{}", __FILE_NAME__, __LINE__, message))
#define FVERBOSE(message, ...) logv(LOG_VERBOSE_TAG, std::format("({}:{}) {}", __FILE_NAME__, __LINE__, std::format(message __VA_OPT__(,) __VA_ARGS__)))
#define FVERBOSE_LB(message, ...) logv(LOG_VERBOSE_TAG, std::format("({}:{})\r\n{}", __FILE_NAME__, __LINE__, std::format(message __VA_OPT__(,) __VA_ARGS__)))
#else
#define VERBOSE(message) {}
#define VERBOSE_LB(message) {}
#define FVERBOSE(message, ...) {}
#define FVERBOSE_LB(message, ...) {}
#endif

#endif //AUDIO_CONTROLLER_COMMON_H