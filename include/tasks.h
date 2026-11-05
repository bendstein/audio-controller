//
// Created by bendstein on 12/27/2025.
//

#ifndef AUDIO_CONTROLLER_TASKS_H
#define AUDIO_CONTROLLER_TASKS_H
#include "app_state.h"
#include "setup.h"

static constexpr auto DISTANCE_SENSOR_TASK_PRIORITY = 2;
static constexpr auto DISTANCE_SENSOR_TASK_STACK_SIZE = 0x1000;
static constexpr auto SEND_AUDIO_TASK_PRIORITY = 2;
static constexpr auto SEND_AUDIO_TASK_STACK_SIZE = 0x1000;
static constexpr auto RECEIVE_AUDIO_TASK_PRIORITY = 2;
static constexpr auto RECEIVE_AUDIO_TASK_STACK_SIZE = 0x1000;

struct bus_sensor_task_param
{
    app_state_controller* state;
    size_t index;
};

struct send_audio_task_param
{
    app_state_controller* state;
};

struct receive_audio_task_param
{
    app_state_audio* state;
};

struct tone_message
{
    static constexpr size_t MESSAGE_LENGTH = 5;

    uint32_t frequency_dhz: 24;
    uint8_t volume: 8;
    uint8_t index;

    tone_message() = default;
    tone_message(const uint32_t frequency_dhz, const uint8_t volume, const uint8_t index)
        : frequency_dhz(std::min(frequency_dhz, static_cast<uint32_t>((1 << 20) - 1))),
        volume(volume),
        index(index) {}
    tone_message(const musical_note_tone_volume data, const uint8_t index)
        : frequency_dhz(static_cast<uint32_t>(std::min(data.tone.frequency_hz() * 100, static_cast<double>((1 << 20) - 1)))),
        volume(data.volume),
        index(index) {}

    [[nodiscard]] double frequency_hz() const { return frequency_dhz / 100.; }
    void copy_from(const tone_message& other)
    {
        frequency_dhz = other.frequency_dhz;
        volume = other.volume;
        // index = other.index;
    }

    void serialize(uint8_t* buf) const;
    static tone_message deserialize(const uint8_t* buf, size_t len);

    bool operator==(const tone_message& other) const { return
        frequency_dhz == other.frequency_dhz
        && volume == other.volume
        && index == other.index; }
};

[[noreturn]]
void distance_sensor_task(void* task_param_pointer);

[[noreturn]]
void send_audio_task(void* task_param_pointer);

[[noreturn]]
void receive_audio_task(void* task_param_pointer);

#endif //AUDIO_CONTROLLER_TASKS_H