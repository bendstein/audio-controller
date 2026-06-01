//
// Created by bendstein on 12/1/2025.
//

#ifndef AUDIO_CONTROLLER_COMMON_H
#define AUDIO_CONTROLLER_COMMON_H

//Use instead of enum for #ifs near bottom
#define LOG_LEVEL_ERROR     1
#define LOG_LEVEL_WARN      2
#define LOG_LEVEL_INFO      3
#define LOG_LEVEL_DEBUG     4
#define LOG_LEVEL_VERBOSE   5

#define LOG_LOCAL_LEVEL LOG_LEVEL_INFO

#include <esp_log.h>
#include <string>
#include <ctime>
#include "driver/gpio.h"
#include <freertos/FreeRTOS.h>
#include <freertos/FreeRTOSConfig.h>
#include <freertos/task.h>
#include <sys/_timeval.h>

//Whether the application should perform additional debugging/logging logic
constexpr auto FLAG_VERBOSE = LOG_LOCAL_LEVEL >= LOG_LEVEL_VERBOSE;
// constexpr auto FLAG_VERBOSE = false;

constexpr auto US_PER_MS = 1000;
constexpr auto MS_PER_SECOND = 1000;
constexpr auto US_PER_SECOND = 1000000;
constexpr auto PIN_LED_BUILTIN = GPIO_NUM_13;
constexpr uint8_t LOW = 0;
constexpr uint8_t HIGH = 1;

//Doesn't need to be a macro, but I'm doing it to match portTICK_PERIOD_MS being a macro
#define portTICK_PERIOD_US ((TickType_t)US_PER_MS / portTICK_PERIOD_MS)

//Change parameter into HIGH or LOW for digital write
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
    if constexpr (FLAG_VERBOSE && LOG_LOCAL_LEVEL >= LOG_LEVEL_VERBOSE)
        log_message(tag, message, ESP_LOG_VERBOSE);
}
inline void logd(const std::string& tag, const std::string& message)
{
    if constexpr (LOG_LOCAL_LEVEL >= LOG_LEVEL_DEBUG)
        log_message(tag, message, ESP_LOG_DEBUG);
}
inline void logi(const std::string& tag, const std::string& message)
{
    if constexpr (LOG_LOCAL_LEVEL >= LOG_LEVEL_INFO)
    log_message(tag, message, ESP_LOG_INFO);
}
inline void logw(const std::string& tag, const std::string& message)
{
    if constexpr (LOG_LOCAL_LEVEL >= LOG_LEVEL_WARN)
    log_message(tag, message, ESP_LOG_WARN);
}
inline void loge(const std::string& tag, const std::string& message)
{
    if constexpr (LOG_LOCAL_LEVEL >= LOG_LEVEL_ERROR)
    log_message(tag, message, ESP_LOG_ERROR);
}

#if LOG_LOCAL_LEVEL >= LOG_LEVEL_VERBOSE

#define LOGV(message) do { logv(__FILE_NAME__, message); } while(false)

#else

#define LOGV(message) do { } while(false)

#endif

#if LOG_LOCAL_LEVEL >= LOG_LEVEL_DEBUG

#define LOGD(message) do { logd(__FILE_NAME__, std::format("({}:{}) {}", __func__, __LINE__, message)); } while(false)

#else

#define LOGD(message) do { } while(false)

#endif

#if LOG_LOCAL_LEVEL >= LOG_LEVEL_INFO

#define LOGI(message) do { logi(__FILE_NAME__, message); } while(false)

#else

#define LOGI(message) do { } while(false)

#endif

#if LOG_LOCAL_LEVEL >= LOG_LEVEL_WARN

#define LOGW(message) do { logw(__FILE_NAME__, message); } while(false)

#else

#define LOGW(message) do { } while(false)

#endif

#if LOG_LOCAL_LEVEL >= LOG_LEVEL_ERROR

#define LOGE(message) do { loge(__FILE_NAME__, std::format("({}:{}) {}", __func__, __LINE__, message)); } while(false)
#define LOGEX(ex) do { loge(__FILE_NAME__, std::format("({}:{}) {}", __func__, __LINE__, ex.what())); } while(false)

#else

#define LOGE(message) do { } while(false)
#define LOGEX(ex) do { } while(false)

#endif

#define FLOGV(message, ...) LOGV(std::format(message __VA_OPT__(,) __VA_ARGS__))
#define FLOGD(message, ...) LOGD(std::format(message __VA_OPT__(,) __VA_ARGS__))
#define FLOGI(message, ...) LOGI(std::format(message __VA_OPT__(,) __VA_ARGS__))
#define FLOGW(message, ...) LOGW(std::format(message __VA_OPT__(,) __VA_ARGS__))
#define FLOGE(message, ...) LOGE(std::format(message __VA_OPT__(,) __VA_ARGS__))

#define VERBOSE(message) LOGV(std::format("({}:{}) {}", __func__, __LINE__, message))
#define VERBOSE_LB(message) LOGV(std::format("({}:{})\r\n{}", __func__, __LINE__, message))
#define FVERBOSE(message, ...) LOGV(std::format("({}:{}) {}", __func__, __LINE__, std::format(message __VA_OPT__(,) __VA_ARGS__)))
#define FVERBOSE_LB(message, ...) LOGV(std::format("({}:{})\r\n{}", __func__, __LINE__, std::format(message __VA_OPT__(,) __VA_ARGS__)))

#endif //AUDIO_CONTROLLER_COMMON_H