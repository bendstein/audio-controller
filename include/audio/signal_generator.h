//
// Created by bendstein on 11/2/2026.
//

#ifndef AUDIO_CONTROLLER_SIGNAL_GENERATOR2_H
#define AUDIO_CONTROLLER_SIGNAL_GENERATOR2_H
#include <memory>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/dac_oneshot.h>
#include <driver/gptimer.h>

struct signal_generator_param
{
    uint64_t sample_rate_hz;
    size_t timer_queue_size;
};

struct signal_generator_tone
{
    double frequency_hz;
    uint8_t volume;
};

class signal_generator
{
public:
    static constexpr auto LOG_KEY = "[Signal Gen]";
    static constexpr size_t TONE_DATA_CAPACITY = 5;
    static constexpr uint32_t TIMER_RESOLUTION_HZ = 1000000;
private:
    static constexpr auto TASK_NAME = "<SIGNAL-GEN>";
    static constexpr auto TASK_PRIORITY = configMAX_PRIORITIES - 1;
    static constexpr auto TASK_STACK_SIZE = 0x1000;

    dac_oneshot_handle_t dac_handle;
    dac_oneshot_handle_t dac_handle1;
    gptimer_handle_t timer_handle;
    TaskHandle_t task;
    QueueHandle_t timer_queue_handle;
    QueueHandle_t tones_queue_handle;

    uint64_t sample_rate_hz;

    static bool IRAM_ATTR timer_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t* ev, void* isr_param);

    [[noreturn]] static void dac_task(void* task_param);

    explicit signal_generator(const signal_generator_param& param);
public:
    signal_generator() = delete;
    signal_generator(const signal_generator& other) = delete;
    signal_generator(signal_generator&& other) = delete;
    signal_generator& operator=(const signal_generator& other) = delete;
    signal_generator& operator=(signal_generator&& other) = delete;

    esp_err_t start();
    bool update_tones(const signal_generator_tone data[TONE_DATA_CAPACITY], TickType_t timeout = portMAX_DELAY);

    [[nodiscard]] static std::unique_ptr<signal_generator> try_create(const signal_generator_param& param);

    ~signal_generator();
};

#endif //AUDIO_CONTROLLER_SIGNAL_GENERATOR2_H