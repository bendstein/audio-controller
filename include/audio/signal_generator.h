//
// Created by bendstein on 8/15/2026.
//

#ifndef AUDIO_CONTROLLER_SIGNAL_GENERATOR_H
#define AUDIO_CONTROLLER_SIGNAL_GENERATOR_H
#include <memory>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/dac_continuous.h>

#include "fixed_vec.h"
#include "tones.h"

enum struct signal_generator_start_result
{
    OK,
    ERR,
    ALREADY_STARTED
};

struct signal_generator_tone
{
    musical_note_tone_volume tone;
    uint32_t period;

    [[nodiscard]] bool is_zero() const { return period == 0 || tone.is_zero(); }
    [[nodiscard]] bool is_invalid() const { return tone.is_invalid(); }
    [[nodiscard]] bool is_zero_or_invalid() const { return period == 0 || tone.is_invalid(); }

    bool operator==(const signal_generator_tone& other) const { return tone == other.tone && period == other.period; }

    [[nodiscard]] static consteval signal_generator_tone create_zero() { return signal_generator_tone(musical_note_tone_volume::create_zero(), 0); }
    [[nodiscard]] static consteval signal_generator_tone create_invalid() { return signal_generator_tone(musical_note_tone_volume::create_invalid(), 0); }
};

class signal_generator
{
public:
    static constexpr auto LOG_KEY = "[Signal Gen]";
    static constexpr size_t TONE_DATA_CAPACITY = 5;

    typedef fixed_vec<musical_note_tone_volume, TONE_DATA_CAPACITY> tone_vec_t;
    typedef fixed_vec<signal_generator_tone, TONE_DATA_CAPACITY> tone_period_vec_t;
private:
    static constexpr auto TASK_NAME = "<SIGNAL-GEN>";
    static constexpr auto TASK_PRIORITY = 0;
    static constexpr auto TASK_STACK_SIZE = 0x1000;

    /**
     * Count of DMA descriptors for DAC config
     */
    static constexpr int DAC_CFG_DESC_NUM = 2;

    /**
     * DMA buffer size for DAC, should be 32-4092, multiple of 4
     */
    static constexpr size_t DAC_CFG_BUFFER_SIZE = 1 << 8;

    /**
     * Sample rate in Hz for DAC
     */
    static constexpr int DAC_CFG_SAMPLE_RATE = 14000;

    /**
     * Offset of DAC data, -128 to 127
     */
    static constexpr int DAC_CFG_OFFSET = 0;

    /**
     * Max number of events in the event queue that is populated in ISR,
     * oldest item is dropped once exceeded
     */
    static constexpr size_t DAC_WRITE_QUEUE_CAPACITY = 8;

    /**
     * Max number of events in the event queue that contains changes to tone  data,
     * oldest item is dropped once exceeded
     */
    static constexpr size_t TONES_QUEUE_CAPACITY = 1;

    /**
     * Size of the working buffer
     */
    static constexpr size_t WORKING_BUFFER_SIZE = DAC_CFG_BUFFER_SIZE;

    dac_continuous_handle_t handle;
    TaskHandle_t task;

    uint8_t* buffer;

    SemaphoreHandle_t task_started_sem;
    QueueHandle_t tones_queue_handle;
    QueueHandle_t dac_write_queue_handle;

    static bool IRAM_ATTR dac_isr(dac_continuous_handle_t, const dac_event_data_t* ev, void* isr_param);

    [[noreturn]] static void dac_task(void* task_param);

    void update_buffer(const tone_period_vec_t& tone_data, size_t offset) const;
public:
    signal_generator();

    signal_generator(const signal_generator& other) = delete;
    signal_generator(signal_generator&& other) = delete;
    signal_generator& operator=(const signal_generator& other) = delete;
    signal_generator& operator=(signal_generator&& other) = delete;

    void update_tones(const tone_vec_t* data, TickType_t timeout = portMAX_DELAY) const;

    signal_generator_start_result start();

    [[nodiscard]] static std::unique_ptr<signal_generator> try_create(dac_channel_mask_t channels = DAC_CHANNEL_MASK_ALL, dac_continuous_channel_mode_t channel_mode = DAC_CHANNEL_MODE_SIMUL);

    ~signal_generator();
};

#endif //AUDIO_CONTROLLER_SIGNAL_GENERATOR_H